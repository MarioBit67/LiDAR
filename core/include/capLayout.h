#ifndef CAPLAYOUT_H
#define CAPLAYOUT_H
#include "winTypes.h"
#include "alloc.h"
#include "capGeom.h"
#include "capVanish.h"

/* Floor plan of a room from its center spin (wall-to-wall "broom" dimensions). Every horizontal edge that
 * follows a room axis is a candidate crease: above the horizon it lies on the ceiling plane, below it on
 * the floor plane. With the ceiling height H assumed (2.80 m by default), a wall whose ceiling crease is
 * seen at elevation a and floor crease at depression b stands at d = H/(tan a + tan b) - the camera
 * height is not needed; it is solved as the height that makes the ceiling and floor creases coincide.
 * Walls become axis-aligned segments; walking them clockwise around the spin point closes a rectilinear
 * polygon (rectangle, L, ...), and corner stations are planned on its convex corners.
 *
 * Plan coordinates: meters, origin at the spin point, u along room axis A (world heading axisDeg), w along
 * axis B (axisDeg + 90). Drawn with u up and w right it is a true top view, and increasing
 * atan2(w, u) runs clockwise - the operator's walking order. */

enum {
   layoutMaxVerts    = 16,
   layoutMaxStations = 8,
   layoutMaxWalls    = 24,
   layoutMaxFrames   = 160,
   layoutDriftWindow = 5     // recent accepted keyframe axes the drift reference is the median of
};

// Why a plan could not be made (shown to the operator, logged)
enum TPlanFailure {
   pfNone       = 0,
   pfNoData     = 1, // no room axes or no horizontal edges from the center spin
   pfFewWalls   = 2, // fewer than four walls found
   pfNoPolygon  = 3,
   pfTooSmall   = 4,
   pfOutside    = 5, // the walls do not enclose the spin point
   pfNoStations = 6
};

struct TPlanPoint {
   float u, w;
};

struct TLayoutPlan {
   bool       valid,
              complete,      // the walls cover (nearly) every heading around the spin point
              heightSolved,  // floor creases paired with ceiling creases; else the default height
              doorFound;     // door / window heads found: the plan is scaled by them (2.10 m)
   float      axisDeg,       // world heading of plan axis u
              ceilingM,      // ceiling height: measured when doorFound, else the assumption
              assumedCeilingM,
              doorScale,     // 2.10 / head height in the assumed scale (1 without doors)
              cameraHeightM,
              extentU,       // bounding box of the polygon
              extentW,
              areaM2;
   TPlanFailure failure;
   int        wallKind[layoutMaxWalls]; // diagnostics: the wall segments the polygon was walked from
   float      wallOffset[layoutMaxWalls],
              wallA0[layoutMaxWalls],
              wallA1[layoutMaxWalls],
              wallWeight[layoutMaxWalls];
   int        heightCorners, // room corners (floor-to-ceiling edges) the camera height was measured on (0: floor lines)
              wallCount,
              vertexCount;
   TPlanPoint verts[layoutMaxVerts]; // clockwise seen from above
   BYTE       convex[layoutMaxVerts];
   int        stationCount;
   BYTE       stationVertex[layoutMaxStations], // corner stations in walking order
              targetVertex[layoutMaxStations];  // corner each station aims at
};

class TRoomLayout
{
 public:
   TRoomLayout(void);

   void  Reset(void);
   void  AddFrame(const TMat4 &cameraToWorld, const TVanishResult &r, const TVanishEdges &edges,
                 const TVec3 &tiltBias, bool centerSpin); // tiltBias: TTiltBias::Bias(); corner stations: height only
   bool  Solve(float axisDeg, float ceilingM, TLayoutPlan &out) const;
   DWORD Count(void) const { return Pcount; }
   float AnchorDeg(void) const; // plan axis: the median of the stored frames' room axes

   TRoomLayout(const TRoomLayout &) = delete;
   TRoomLayout &operator=(const TRoomLayout &) = delete;

 private:
   bool FrameKept(int f) const; // its axis agrees with its neighbors on both sides

   TBlock<float> Pdata; // per edge: world ray x, y, z and the world heading of its 3D line, [0, 180)
   DWORD         Pcount,
                 Pcap;
   DWORD         PframeStart[layoutMaxFrames + 1]; // first stored edge of each accepted keyframe
   float         PframePitch[layoutMaxFrames];      // elevation the keyframe aims at
   bool          PframeCenter[layoutMaxFrames];     // taken from the spin point (walls); else a corner station
   float         PframeAxis[layoutMaxFrames];       // its own room axis, unwrapped (the edges are turned onto the anchor)
   int           Pframes;
   float         PanchorDeg,
                 Precent[layoutDriftWindow]; // recent accepted axes (unwrapped): their median follows the drift
   int           PrecentCount;
};

#endif // CAPLAYOUT_H
