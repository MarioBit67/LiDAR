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
   layoutDriftWindow = 5,    // recent accepted keyframe axes the drift reference is the median of
   layoutMaxCreases  = 256,  // diagnostics: creases the walls were clustered from
   layoutMaxOpenDoors = 4,
   layoutMaxCenters  = 16   // centers of the room: one per rectangle of its decomposition (an L has three)
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

// One crease the walls were clustered from (diagnostics): seen by a keyframe, rewritten from the spin point
struct TPlanCrease {
   int   frame;   // layout keyframe (stored order)
   BYTE  half,    // kind*2 + (offset < 0)
         station; // 0: the spin point saw it; k: corner station k
   float dist,    // distance from the spin point
         a0,      // along range
         a1,
         weight;
};

struct TPlanPoint {
   float u, w;
};

struct TLayoutPlan {
   bool       valid,
              complete,      // the walls cover (nearly) every heading around the spin point
              heightSolved,  // floor creases paired with ceiling creases; else the default height
              doorFound,     // door / window heads found (2.10 m): they measure the ceiling height
              ceilingSnapped; // the heads put the ceiling on a typical height (2.70 / 2.80): the plan is scaled to it
   float      axisDeg,       // world heading of plan axis u
              ceilingM,      // ceiling height the plan uses: the typical one the door heads chose, else the assumption
              assumedCeilingM,
              doorScale,     // 2.10 / head height in the assumed scale (1 without doors)
              impliedCeilingM, // ceiling height the door heads imply (assumption x doorScale)
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
   int        wallViews[layoutMaxWalls];
   int        heightCorners, // room corners (floor-to-ceiling edges) the camera height was measured on (0: floor lines)
              stationLines,  // creases the corner stations added (placed by the walls the spin point saw)
              wallCount,
              vertexCount;
   TPlanPoint verts[layoutMaxVerts]; // clockwise seen from above
   BYTE       convex[layoutMaxVerts];
   int        stationCount;
   BYTE       stationVertex[layoutMaxStations], // corner stations in walking order
              targetVertex[layoutMaxStations];  // corner each station aims at
   int         centerCount;       // middles of the room: a rectangle has one, an L one per arm and one where they meet
   TPlanPoint  centers[layoutMaxCenters];
   int         openDoors,         // open door leaves found (worth closing: a closed door is a better ruler)
               openDoorWall[layoutMaxOpenDoors];
   float       openDoorAt[layoutMaxOpenDoors]; // along that wall
   int         creaseCount;
   TPlanCrease creases[layoutMaxCreases];
};

class TRoomLayout
{
 public:
   TRoomLayout(void);

   void  Reset(void);
   void  AddFrame(const TMat4 &cameraToWorld, const TVanishResult &r, const TVanishEdges &edges,
                 const TVec3 &tiltBias, int station); // tiltBias: TTiltBias::Bias(); station 0: the center spin
   bool  Solve(float axisDeg, float ceilingM, TLayoutPlan &out) const; // ceilingM: the assumption; door heads may snap it
   DWORD Count(void) const { return Pcount; }
   float AnchorDeg(void) const; // plan axis: the median of the stored frames' room axes

   TRoomLayout(const TRoomLayout &) = delete;
   TRoomLayout &operator=(const TRoomLayout &) = delete;

 private:
   bool FrameKept(int f) const; // its axis agrees with its neighbors on both sides
   bool solveAt(float axisDeg, float ceilingM, TLayoutPlan &out) const;

   TBlock<float> Pdata; // per edge: world ray x, y, z and the world heading of its 3D line, [0, 180)
   DWORD         Pcount,
                 Pcap;
   DWORD         PframeStart[layoutMaxFrames + 1]; // first stored edge of each accepted keyframe
   float         PframePitch[layoutMaxFrames];      // elevation the keyframe aims at
   BYTE          PframeStation[layoutMaxFrames];    // 0: the center spin; k: corner station k (placed by the known walls)
   float         PframeAxis[layoutMaxFrames];       // its own room axis, unwrapped (the edges are turned onto the anchor)
   int           Pframes;
   float         PanchorDeg,
                 Precent[layoutDriftWindow]; // recent accepted axes (unwrapped): their median follows the drift
   int           PrecentCount;
};

#endif // CAPLAYOUT_H
