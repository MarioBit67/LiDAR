#ifndef CAPSPIN_H
#define CAPSPIN_H
#include "winTypes.h"
#include "capGeom.h"

/* Guides the in-place 360-degree spin of one room and decides which camera frames are kept.
 * Coverage is a grid of heading bins per pitch band: the horizon band sees the walls, the
 * upper and lower bands (narrow lenses only) reach the ceiling and floor creases. A frame
 * is kept when it lands in an empty bin, is sharp enough (slow turn) and trustworthy
 * (compass accuracy, operator still standing on the same spot). */

enum {
   spinMaxBands = 3,
   spinMaxBins  = 72
};

// Why a frame was kept or rejected (drives the operator hints)
enum TSpinVerdict {
   svKeep,        // new viewpoint, stored
   svCovered,     // bin already filled
   svTooFast,     // turning too fast, motion blur likely
   svOffBand,     // camera pitch outside the guided band (outside every band once all are in)
   svBadCompass,  // heading accuracy too poor
   svDrifted,     // operator walked away from the spin center
   svOutside      // heading outside the aimed fan (corner stations)
};

struct TSpinConfig {
   int   headingBins, // bins per band across the full circle, or across the fan when fanDeg > 0
         bandCount;
   float bandPitchDeg[spinMaxBands], // band centers, degrees above the horizon
         bandLoDeg[spinMaxBands],    // a band's accepted pitches when set (lo < hi; they may overlap: the guided band wins)
         bandHiDeg[spinMaxBands],
         bandHalfDeg,                // else the accepted pitch deviation around a band center
         maxRateDps,                 // max turn rate for a sharp frame
         maxAccuracyDeg,             // max compass error (NaN accuracy = unknown, accepted)
         minSepFrac,                 // min heading gap to a kept neighbor, in bins
         maxDriftM,                  // max distance from the first pose (0 disables)
         fanDeg,                     // 0 = full circle; > 0 = only a fan this wide around the first steady aim
         reachDownDeg,               // < 0: the lowest band also takes pitches down to this (a small room's floor)
         reachUpDeg;                 // > 0: the highest band also takes pitches up to this (its ceiling)

   static TSpinConfig UltraWide(void); // 0.5x lens: one horizon band covers creases
   static TSpinConfig Wide(void);      // 1x lens (ARKit + LiDAR): three bands

   /* Derived from the lens as the operator holds it: enough pitch bands to see the floor and
    * ceiling creases (pitch -55..+55) with 20% overlap, and heading bins no wider than half
    * the horizontal field of view. */
   static TSpinConfig ForFov(float hfovDeg, float vfovDeg);

   /* Corner station: standing in one corner, aiming at the opposite one. One horizon band (the far
    * corner's floor and ceiling creases both fit a normal lens) and a 60-degree fan so both walls
    * meeting at the target corner are seen, in five views a quarter frame apart (the middle ones refine it). */
   static TSpinConfig ForCorner(float hfovDeg, float vfovDeg);
   static TSpinConfig ForFloorView(float hfovDeg, float vfovDeg); // one view down to the floor, aimed with AimFan
};

class TSpinTracker
{
 public:
   explicit TSpinTracker(const TSpinConfig &cfg);

   // Every frame should be offered (it tracks the turn rate); allowKeep false = observe only (encoder busy)
   TSpinVerdict Offer(QWORD stampNs, const TMat4 &cameraToWorld, float accuracyDeg, bool allowKeep = true);
   void Reset(void);

   int   Filled(void) const { return Pfilled; }
   int   Total(void) const { return Pcfg.bandCount*Pcfg.headingBins; }
   bool  Complete(void) const { return Pfilled == Total(); }
   bool  BinFilled(int band, int bin) const;
   int   BandCount(void) const { return Pcfg.bandCount; }
   int   HeadingBins(void) const { return Pcfg.headingBins; }
   float BandPitchDeg(int band) const { return Pcfg.bandPitchDeg[band]; }
   int   BandFilled(int band) const;
   float LastHeadingDeg(void) const { return PlastHeading; }
   float LastPitchDeg(void) const { return PlastPitch; }
   int   LastKeptBand(void) const { return PkeptBand; }
   int   LastKeptBin(void) const { return PkeptBin; }
   float BinWidthDeg(void) const { return binWidthDeg(); }
   float BinCenterDeg(int bin, float aimDeg) const; // heading of a bin center; a fan not yet aimed centers on aimDeg
   bool  AimedEmpty(int &band, int &bin) const;       // the bin the camera aims at now is still empty
   bool  At(int band, int bin) const;                  // the last pose still lies in this band and bin (a burst holds on it)
   int   PitchSide(int band, float pitchDeg) const;    // -1 below the band's accepted pitches, +1 above, 0 inside
   bool  Locate(int &band, int &bin) const;           // the band and bin the last pose lies in (false off every allowed band)
   /* the next steady view of the bin replaces its frame (the bin still counts); wrong = the frame is off (orange):
      it may be retaken even outside the guided band */
   void  Reopen(int band, int bin, bool wrong = false);
   void  AimFan(float headingDeg);  // a fan centered on this heading instead of the first steady aim
   int   GuidedBand(void) const;    // the band to hold now (ceiling first); -1 once every bin is in
   bool  PoseAllowed(void) const;   // the last pose lies in the guided band (or on an orange bin): false = red, nothing kept

   TSpinTracker(const TSpinTracker &) = delete;
   TSpinTracker &operator=(const TSpinTracker &) = delete;
   TSpinTracker(TSpinTracker &&) = delete;
   TSpinTracker &operator=(TSpinTracker &&) = delete;

 private:
   bool inBand(int band, float pitchDeg) const;
   int  bandOf(float pitchDeg) const;
   int  allowedBand(float pitchDeg, float headingDeg) const; // bandOf, but -1 outside the guided band (orange bins aside)
   bool nearKept(int band, int bin, float headingDeg) const;
   int  binOf(float headingDeg) const; // -1 outside the fan
   float binWidthDeg(void) const;

   TSpinConfig Pcfg;
   bool        Pbin[spinMaxBands][spinMaxBins],
               Pretake[spinMaxBands][spinMaxBins], // kept, but its frame may be replaced (orange, or a sharper one wanted)
               Pwrong[spinMaxBands][spinMaxBins],  // that frame is off (orange): retakable outside the guided band
               PhasLast,
               PhasOrigin,
               PfanSet;
   float       Pkept[spinMaxBands][spinMaxBins], // heading of the frame kept in each bin
               PlastHeading,
               PlastPitch,
               PfanCenter;
   QWORD       PlastStampNs;
   TVec3       Porigin;
   int         Pfilled,
               PkeptBand,
               PkeptBin;
};

#endif // CAPSPIN_H
