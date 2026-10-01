#ifndef CAPVANISH_H
#define CAPVANISH_H
#include "winTypes.h"
#include "capGeom.h"

/* Vanishing points of a keyframe in a Manhattan room: floor, ceiling and corner creases follow three
 * orthogonal directions, the vertical one and two horizontal room axes 90 degrees apart. Gravity fixes
 * the vertical, so the search is one-dimensional: the heading of the room axes. Edge pixels of the luma
 * (Scharr + structure tensor) vote through their interpretation planes (the plane through the camera
 * center and the edge line; its normal is orthogonal to the 3D direction of the line). The best heading
 * is then refined per direction by least squares over its edges.
 *
 * What it yields per frame: the room-axis heading measured by the image (the exact yaw of the frame,
 * modulo 90 degrees, independent of the gyroscope drift), the tilt between the measured vertical and
 * gravity (sensor check) and the orthogonality error between measured directions (intrinsics check).
 * Embedded detection and validation; a server refines later from the same JPEG. */

enum {
   vanishDirs = 3 // 0 vertical, 1 room axis A, 2 room axis B (A + 90 degrees)
};

enum TVanishFlag {
   vfVertical = 1,
   vfAxisA    = 2,
   vfAxisB    = 4
};

struct TVanishConfig {
   int   maxSide,      // long side of the analysis image (the luma is box-downsampled to fit)
         minGrad,      // Scharr magnitude threshold, |gx| + |gy| on the analysis image
         minSupport,   // edge pixels needed to trust a direction
         maxEdges;     // edge pixels kept (evenly thinned beyond this)
   float minCoherence, // structure-tensor coherence of an edge pixel (1 = perfect straight edge)
         tolDeg,       // angular tolerance of an edge to a vanishing direction
         stepDeg,      // heading search step
         maxTiltDeg,   // accepted gap between the gravity vertical and a refined direction's prediction
         priorAxisDeg, // room-axis heading (world, [0, 90)) the heading search is held near
         priorWindowDeg, // half window about priorAxisDeg (0: the whole quadrant)
         upBias[3];    // camera-to-sensor tilt: rotation vector (radians, camera axes) turning gravity into the true vertical

   static TVanishConfig Default(void);
};

struct TVanishResult {
   BYTE  flags;             // TVanishFlag bits of the directions with enough support
   float roomAxisDeg,       // world heading of room axis A, [0, 90); NaN without a horizontal direction
         tiltErrDeg,        // measured vertical vs gravity; NaN without the vertical
         orthoErrDeg;       // worst deviation from 90 degrees between measured directions; NaN with fewer than 2
   TVec3 dirCam[vanishDirs], // measured directions in camera axes (the prediction where the flag is off)
         upFix;             // rotation vector (camera axes) from the RAW gravity to the measured vertical (vfVertical)
   DWORD support[vanishDirs],
         edges;             // edge pixels that voted
};

// Optional per-edge output (caller buffers, capacity >= maxEdges or nothing is written)
struct TVanishEdges {
   float *ray;   // capacity*3: unit camera ray through each edge pixel
   LPBYTE label; // capacity: 1 + index of the measured direction the edge's 3D line follows, 0 none
   DWORD  capacity,
          count;
};

/* luma: native sensor pixels (row stride in bytes); intr: pinhole of that image; cameraToWorld: pose of
 * the frame (camera x right, y up, looking down -z; pixel v grows downward). False when the image has too
 * few edges for any verdict. */
bool vanishDetect(LPCBYTE luma, int width, int height, int stride, const TIntrinsics &intr,
                  const TMat4 &cameraToWorld, const TVanishConfig &cfg, TVanishResult &out,
                  TVanishEdges *edges = NULL);

// Rotation vectors (axis * angle in radians): turn v by w; the rotation taking unit a onto unit b
TVec3 vanishRotate(const TVec3 &v, const TVec3 &w);
TVec3 vanishRotationBetween(const TVec3 &a, const TVec3 &b);

/* Running camera-to-sensor tilt. The Moto's gravity disagrees with the image vertical by 1.5-4 degrees,
 * mostly a constant mounting / sensor-fusion offset: the mean of the measured corrections (frames with a
 * well supported vertical) is fed back as TVanishConfig::upBias, so frames without vertical lines still
 * get a true horizontal plane. */
class TTiltBias
{
 public:
   TTiltBias(void);

   void Reset(void);
   void Add(const TVanishResult &r);
   void Apply(TVanishConfig &cfg) const; // zero until enough frames were measured
   TVec3 Bias(void) const;                // the same correction as a rotation vector (zero until ready)
   int  Count(void) const { return Pcount; }

 private:
   TVec3 Pmean;
   int   Pcount;
};

// Signed smallest difference a - b between two room-axis headings (modulo 90), (-45, 45]
float vanishAxisDiffDeg(float a, float b);

enum {
   axisCheckMax    = 128, // measured frames kept per room
   axisCheckQuorum = 3,   // frames that must agree before the room has a reference
   axisCheckWindow = 10   // recent frames the consensus is taken over (follows the slow gyroscope drift)
};

// Verdict of one keyframe against the room axes agreed by the room's frames so far
enum TAxisVerdict {
   avNoLines    = 0, // no horizontal direction measured: frame not verifiable
   avPending    = 1, // measured, but no quorum agrees yet
   avReference  = 2, // this frame completed the quorum that fixed the reference
   avAligned    = 3, // agrees with the reference
   avMisaligned = 4  // disagrees: gyroscope drift, a skewed wall or a spurious detection (diagonal pattern)
};

/* Capture-time coherence of a room: every keyframe of every station (center spin and corners) shares the
 * sensor reference, so all of them must measure the same room axes. The reference is the CONSENSUS of the
 * room's sound frames (the largest group agreeing within the tolerance, averaged), so neither the first
 * frame nor an outlier dictates it; each new frame is judged against the consensus at its time, and the
 * aligned / misaligned counts always reflect the current consensus. */
class TAxisCheck
{
 public:
   explicit TAxisCheck(float tolDeg);

   void         Reset(void);
   TAxisVerdict Offer(const TVanishResult &r, float *deviationDeg); // deviation NaN unless judged

   bool  HasReference(void) const { return PhasRef; }
   float ReferenceDeg(void) const { return PrefDeg; }
   int   Aligned(void) const { return Paligned; }
   int   Misaligned(void) const { return Pmisaligned; }
   int   Unverified(void) const { return Punverified; }
   float LastDeviationDeg(void) const { return PlastDev; }
   float DriftDeg(void) const { return PhasRef ? vanishAxisDiffDeg(PrefDeg, PfirstRefDeg) : 0.f; } // gyroscope yaw drift so far

 private:
   void updateReference(void);

   float Pvalues[axisCheckMax],
         Ptol,
         PrefDeg,
         PfirstRefDeg,
         PlastDev;
   int   Pcount,
         Paligned,
         Pmisaligned,
         Punverified;
   bool  Psound[axisCheckMax], // orthogonality sound enough to vote in the consensus
         PhasRef;
};

#endif // CAPVANISH_H
