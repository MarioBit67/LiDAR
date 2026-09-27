#include "capSpin.h"
#include "libDiscipline.h"

static const float cCornerPitchDeg = 12.f; // corner fans: ceiling creases of the far walls mid-picture, the floor still in view

//--------------------------------------------------------------------------------
TSpinConfig TSpinConfig::UltraWide(void)
{
   TSpinConfig c = {};

   c.headingBins = 24;
   c.bandCount = 1;
   c.bandPitchDeg[0] = 0.f;
   c.bandHalfDeg = 25.f;
   c.maxRateDps = 45.f;
   c.maxAccuracyDeg = 30.f;
   c.minSepFrac = 0.5f;
   c.maxDriftM = 0.f;
   return c;
}

//--------------------------------------------------------------------------------
TSpinConfig TSpinConfig::Wide(void)
{
   TSpinConfig c = {};

   c.headingBins = 24;
   c.bandCount = 3;
   c.bandPitchDeg[0] = -35.f;
   c.bandPitchDeg[1] = 0.f;
   c.bandPitchDeg[2] = 35.f;
   c.bandHalfDeg = 12.f;
   c.maxRateDps = 30.f;
   c.maxAccuracyDeg = 30.f;
   c.minSepFrac = 0.5f;
   c.maxDriftM = 0.5f;
   return c;
}

//--------------------------------------------------------------------------------
TSpinConfig TSpinConfig::ForFov(float hfovDeg, float vfovDeg)
{
   const float span = 110.f,  // pitch -55..+55: near-wall floor and ceiling creases at eye height
               overlap = 0.8f;
   TSpinConfig c = UltraWide();
   int         bands = 1;

   if (vfovDeg < span)
      bands = 1 + (int)ceilf((span - vfovDeg)/(overlap*vfovDeg));
   if (bands > spinMaxBands)
      bands = spinMaxBands;

   float spacing = bands > 1 ? (span - vfovDeg)/(float)(bands - 1) : 0.f;

   c.bandCount = bands;
   for (int i = 0; i < bands; i++)
      c.bandPitchDeg[i] = -0.5f*spacing*(float)(bands - 1) + spacing*(float)i;
   c.bandHalfDeg = bands > 1 ? 0.5f*spacing : 0.25f*vfovDeg;
   c.headingBins = (int)ceilf(360.f/(0.5f*hfovDeg));
   if (c.headingBins < 12)
      c.headingBins = 12;
   if (c.headingBins > spinMaxBins)
      c.headingBins = spinMaxBins;
   c.maxRateDps = 0.5f*hfovDeg; // half a frame width per second keeps motion blur low
   return c;
}

//--------------------------------------------------------------------------------
TSpinConfig TSpinConfig::ForCorner(float hfovDeg, float vfovDeg)
{
   TSpinConfig c = UltraWide();

   c.bandCount = 1;
   c.bandPitchDeg[0] = cCornerPitchDeg; // aimed a little up: the far creases (and the notch of an L) mid-picture
   c.bandHalfDeg = 0.25f*vfovDeg;
   c.fanDeg = 60.f;
   c.headingBins = (int)ceilf(c.fanDeg/(0.5f*hfovDeg)); // bins no wider than half a frame
   if (c.headingBins < 3)
      c.headingBins = 3;
   c.maxRateDps = 0.5f*hfovDeg;
   return c;
}

//--------------------------------------------------------------------------------
TSpinTracker::TSpinTracker(const TSpinConfig &cfg) : Pcfg(cfg)
{
   if (Pcfg.bandCount < 1)
      Pcfg.bandCount = 1;
   if (Pcfg.bandCount > spinMaxBands)
      Pcfg.bandCount = spinMaxBands;
   if (Pcfg.headingBins < 1)
      Pcfg.headingBins = 1;
   if (Pcfg.headingBins > spinMaxBins)
      Pcfg.headingBins = spinMaxBins;
   Reset();
}

//--------------------------------------------------------------------------------
void TSpinTracker::Reset(void)
{
   for (int band = 0; band < spinMaxBands; band++)
      for (int bin = 0; bin < spinMaxBins; bin++)
      {
         Pbin[band][bin] = false;
         Pkept[band][bin] = 0.f;
      }
   PhasLast = false;
   PhasOrigin = false;
   PfanSet = false;
   PfanCenter = 0.f;
   PlastHeading = 0.f;
   PlastPitch = 0.f;
   PlastStampNs = 0u;
   Porigin = { 0.f, 0.f, 0.f };
   Pfilled = 0;
   PkeptBand = 0;
   PkeptBin = 0;
}

//--------------------------------------------------------------------------------
bool TSpinTracker::BinFilled(int band, int bin) const
{
   if (band < 0 || band >= Pcfg.bandCount || bin < 0 || bin >= Pcfg.headingBins)
      return false;
   return Pbin[band][bin];
}

//--------------------------------------------------------------------------------
int TSpinTracker::BandFilled(int band) const
{
   int n = 0;

   if (band < 0 || band >= Pcfg.bandCount)
      return 0;
   for (int bin = 0; bin < Pcfg.headingBins; bin++)
      if (Pbin[band][bin])
         n++;
   return n;
}

//--------------------------------------------------------------------------------
int TSpinTracker::bandOf(float pitchDeg) const
{
   for (int band = 0; band < Pcfg.bandCount; band++)
      if (fabsf(pitchDeg - Pcfg.bandPitchDeg[band]) <= Pcfg.bandHalfDeg)
         return band;
   return -1;
}

//--------------------------------------------------------------------------------
float TSpinTracker::binWidthDeg(void) const
{
   return (Pcfg.fanDeg > 0.f ? Pcfg.fanDeg : 360.f)/(float)Pcfg.headingBins;
}

/*--------------------------------------------------------------------------------
   Full circle: absolute bins from north. Fan: bins relative to the aim, the aim heading
   sitting in the middle of the fan; -1 when outside it (or before the aim is taken).
  --------------------------------------------------------------------------------*/
int TSpinTracker::binOf(float headingDeg) const
{
   if (Pcfg.fanDeg <= 0.f)
      return (int)(headingDeg*(float)Pcfg.headingBins/360.f)%Pcfg.headingBins;
   if (!PfanSet)
      return -1;

   float rel = headingDeg - PfanCenter;

   while (rel > 180.f)
      rel -= 360.f;
   while (rel < -180.f)
      rel += 360.f;
   rel += 0.5f*Pcfg.fanDeg;
   if (rel < 0.f || rel >= Pcfg.fanDeg)
      return -1;
   return (int)(rel/binWidthDeg());
}

//--------------------------------------------------------------------------------
// True when a kept frame in this or an adjacent bin is closer than minSepFrac bins
bool TSpinTracker::nearKept(int band, int bin, float headingDeg) const
{
   float binDeg = binWidthDeg(),
         minDeg = Pcfg.minSepFrac*binDeg;

   for (int d = -1; d <= 1; d++)
   {
      int b = (bin + d + Pcfg.headingBins)%Pcfg.headingBins;

      if (Pbin[band][b] && geomHeadingDiffDeg(Pkept[band][b], headingDeg) < minDeg)
         return true;
   }
   return false;
}

//--------------------------------------------------------------------------------
TSpinVerdict TSpinTracker::Offer(QWORD stampNs, const TMat4 &cameraToWorld, float accuracyDeg, bool allowKeep)
{
   TVec3 fwd = cameraToWorld.Forward(),
         pos = cameraToWorld.Translation();
   float heading = geomHeadingDeg(fwd),
         pitch = geomPitchDeg(fwd),
         rate = 0.f;
   bool  hadLast = PhasLast;
   QWORD lastStamp = PlastStampNs;
   float lastHeading = PlastHeading;

   PhasLast = true;
   PlastStampNs = stampNs;
   PlastHeading = heading;
   PlastPitch = pitch;

   if (!PhasOrigin)
   {
      Porigin = pos;
      PhasOrigin = true;
   }
   if (hadLast && stampNs > lastStamp)
   {
      float dt = (float)(stampNs - lastStamp)*1e-9f;

      if (dt < 0.5f)
         rate = geomHeadingDiffDeg(heading, lastHeading)/dt;
   }

   if (Pcfg.maxDriftM > 0.f)
   {
      float dx = pos.x - Porigin.x,
            dy = pos.y - Porigin.y,
            dz = pos.z - Porigin.z;

      if (sqrtf(dx*dx + dy*dy + dz*dz) > Pcfg.maxDriftM)
         return svDrifted;
   }
   if (accuracyDeg == accuracyDeg && accuracyDeg > Pcfg.maxAccuracyDeg) // NaN = unknown, accepted
      return svBadCompass;
   if (rate > Pcfg.maxRateDps)
      return svTooFast;

   int band = bandOf(pitch);

   if (band < 0)
      return svOffBand;

   if (Pcfg.fanDeg > 0.f && !PfanSet) // the first steady frame is the aim at the target corner
   {
      PfanCenter = heading;
      PfanSet = true;
   }

   int bin = binOf(heading);

   if (bin < 0)
      return svOutside;
   if (Pbin[band][bin] || nearKept(band, bin, heading) || !allowKeep)
      return svCovered;
   Pbin[band][bin] = true;
   Pkept[band][bin] = heading;
   PkeptBand = band;
   PkeptBin = bin;
   Pfilled++;
   return svKeep;
}

//--------------------------------------------------------------------------------
bool TSpinTracker::AimedEmpty(int &band, int &bin) const
{
   if (!PhasLast)
      return false;
   band = bandOf(PlastPitch);
   bin = band >= 0 ? binOf(PlastHeading) : -1;
   return bin >= 0 && !Pbin[band][bin];
}

//--------------------------------------------------------------------------------
float TSpinTracker::BinCenterDeg(int bin, float aimDeg) const
{
   float w = binWidthDeg();

   if (Pcfg.fanDeg <= 0.f)
      return ((float)bin + 0.5f)*w;

   float center = PfanSet ? PfanCenter : aimDeg;

   return center - 0.5f*Pcfg.fanDeg + ((float)bin + 0.5f)*w;
}

//--------------------------------------------------------------------------------
