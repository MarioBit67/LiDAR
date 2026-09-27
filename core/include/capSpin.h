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
   svOffBand,     // camera pitch outside every band
   svBadCompass,  // heading accuracy too poor
   svDrifted,     // operator walked away from the spin center
   svOutside      // heading outside the aimed fan (corner stations)
};

struct TSpinConfig {
   int   headingBins, // bins per band across the full circle, or across the fan when fanDeg > 0
         bandCount;
   float bandPitchDeg[spinMaxBands], // band centers, degrees above the horizon
         bandHalfDeg,                // accepted pitch deviation around a band center
         maxRateDps,                 // max turn rate for a sharp frame
         maxAccuracyDeg,             // max compass error (NaN accuracy = unknown, accepted)
         minSepFrac,                 // min heading gap to a kept neighbor, in bins
         maxDriftM,                  // max distance from the first pose (0 disables)
         fanDeg;                     // 0 = full circle; > 0 = only a fan this wide around the first steady aim

   static TSpinConfig UltraWide(void); // 0.5x lens: one horizon band covers creases
   static TSpinConfig Wide(void);      // 1x lens (ARKit + LiDAR): three bands

   /* Derived from the lens as the operator holds it: enough pitch bands to see the floor and
    * ceiling creases (pitch -55..+55) with 20% overlap, and heading bins no wider than half
    * the horizontal field of view. */
   static TSpinConfig ForFov(float hfovDeg, float vfovDeg);

   /* Corner station: standing in one corner, aiming at the opposite one. One horizon band (the far
    * corner's floor and ceiling creases both fit a normal lens) and a 60-degree fan so both walls
    * meeting at the target corner are seen. */
   static TSpinConfig ForCorner(float hfovDeg, float vfovDeg);
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

   TSpinTracker(const TSpinTracker &) = delete;
   TSpinTracker &operator=(const TSpinTracker &) = delete;
   TSpinTracker(TSpinTracker &&) = delete;
   TSpinTracker &operator=(TSpinTracker &&) = delete;

 private:
   int  bandOf(float pitchDeg) const;
   bool nearKept(int band, int bin, float headingDeg) const;
   int  binOf(float headingDeg) const; // -1 outside the fan
   float binWidthDeg(void) const;

   TSpinConfig Pcfg;
   bool        Pbin[spinMaxBands][spinMaxBins],
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
