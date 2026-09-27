#ifndef CAPDOOR_H
#define CAPDOOR_H
#include "winTypes.h"
#include "capGeom.h"
#include "capJPEG.h"
#include "capVanish.h"

/* Doors as the ruler of a room (user, 2026-09-27: "a verga tem 2,10"). A keyframe is turned by a pure
 * rotation into a frontal, level view of one wall: verticals vertical, the wall's creases horizontal, and
 * every point of the wall at the same depth - so heights on the wall are proportional to image rows, with
 * no distance or ceiling assumed. There a door is an upright rectangle:
 *  - two jambs: long vertical edges reaching the floor, ending at the same row (the head);
 *  - the head: a horizontal edge between them that does not run on past the casing (a wardrobe's divider
 *    runs across all its panels);
 *  - above the head, plain wall: the color of the wall beside the door and no edges (a wardrobe goes on
 *    with panels up to the ceiling);
 *  - tie-breakers: the knob at about 1 m on one jamb's side, and a room corner close by (doors sit near
 *    corners almost always).
 * On the same columns the highest ceiling line (the crease the floor plan uses, molding included) and the
 * floor give ratios to the head: with the head at 2.10 m they are the crease height and the camera height. */

enum {
   doorViewW   = 900,  // frontal view, pixels
   doorViewH   = 1600, // tall enough for the whole frame: a door's foot must not fall off the view
   doorMaxDoors = 4,
   doorMaxJambs = 96,
   doorMaxTried = 256
};

// A frontal, level view of one wall (caller buffers: doorViewW*doorViewH bytes each)
struct TDoorView {
   LPBYTE y, u, v,  // luma and chroma of the view
          valid;    // 1 where the source frame saw the pixel
   float  focalPx,  // pinhole of the view (square pixels)
          cx,
          horizonRow; // the row of the rays level with the camera
};

struct TDoor {
   float leftCol, rightCol, // the two jambs (view columns)
         headRow,           // the head: bottom of the lintel, top of the opening
         floorRow,          // the jambs' feet
         creaseRow,         // the highest ceiling line over the door (NaN: out of the view)
         creaseOverHead,    // (floor - crease)/(floor - head): crease height in door heads
         cameraOverHead,    // (floor - horizon)/(floor - head): camera height in door heads
         colorGap,          // chroma+luma distance of the wall above the head to the wall beside it
         score;
   bool  knob,              // a compact dark blob at knob height next to a jamb
         footCut,           // its foot fell off the photo: floorRow is the photo's edge, nothing measured on it
         nearCorner;        // a full-height vertical edge (a room corner) close to a jamb
};

/* Frontal view of the wall with horizontal normal nCam (camera axes, from the camera into the wall - the
 * direction the view looks along) from a keyframe; upCam: the true vertical in camera axes. The view keeps
 * the frame's own field centered (principal point shifted, like a shift lens). */
void doorFrontal(const TYUVImage &img, const TIntrinsics &k, const TVec3 &upCam, const TVec3 &nCam, TDoorView &view);

/* The wall a keyframe shows the most, from its vanishing measure (capVanish): upCam = the measured
 * vertical, nCam = that wall's horizontal normal from the camera into it (as doorFrontal takes it). Aimed
 * at a corner, the wall whose lines have more edge support wins. False without a horizontal direction. */
bool doorFrameWall(const TVanishResult &vr, TVec3 &upCam, TVec3 &nCam);

/* Both walls a frame aimed at a corner shows (the dominant first), or the one it faces: a door on the
 * wall seen sideways is only upright in that wall's own frontal view (124744: frames 37, 38, 44). */
int doorFrameWalls(const TVanishResult &vr, TVec3 &upCam, TVec3 *nCam);

// World heading of a column of a frontal view (the view's level ray through it)
float doorColumnHeadingDeg(const TDoorView &view, const TVec3 &upCam, const TVec3 &nCam, const TMat4 &cameraToWorld,
                           float col);

// Where the candidates of a view fell (diagnosis)
struct TDoorStats {
   int jambs,     // long vertical runs kept
       pairs,     // jamb pairs tried
       shape,     // rejected: aspect, unequal tops/feet, head below or floor above the horizon
       head,      // rejected: no head edge across the opening
       overrun,   // rejected: the head edge runs on beside the casing, or both jambs run on above it (a wardrobe)
       above,     // rejected: edges on the wall above the head, or no wall to compare with
       color,     // rejected: the wall above differs from the wall beside
       jambCol[doorMaxJambs], // the jambs themselves: column, top and bottom rows
       jambTop[doorMaxJambs],
       jambBottom[doorMaxJambs],
       tried,                  // pairs listed below (the first doorMaxTried)
       triedA[doorMaxTried],   // their jamb columns and the check that stopped them (0 found, 1 shape,
       triedB[doorMaxTried],   // 2 foot, 3 plausibility, 4 head, 5 overrun, 6 above, 7 color, 8 leaf)
       triedWhy[doorMaxTried];
   float triedValue[doorMaxTried]; // the measure that failed (edges above, color gap...)
};

// Doors found in a frontal view, best first; returns how many (stats optional)
int doorDetect(const TDoorView &view, TDoor *doors, int cap, TDoorStats *stats = NULL);

#endif // CAPDOOR_H
