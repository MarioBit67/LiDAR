#ifndef CAPDOOR_H
#define CAPDOOR_H
#include "winTypes.h"
#include "capGeom.h"
#include "capJPEG.h"

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
   doorViewH   = 1200,
   doorMaxDoors = 4,
   doorMaxJambs = 96
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
         nearCorner;        // a full-height vertical edge (a room corner) close to a jamb
};

/* Frontal view of the wall with horizontal normal nCam (camera axes, from the camera into the wall - the
 * direction the view looks along) from a keyframe; upCam: the true vertical in camera axes. The view keeps
 * the frame's own field centered (principal point shifted, like a shift lens). */
void doorFrontal(const TYUVImage &img, const TIntrinsics &k, const TVec3 &upCam, const TVec3 &nCam, TDoorView &view);

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
       jambBottom[doorMaxJambs];
};

// Doors found in a frontal view, best first; returns how many (stats optional)
int doorDetect(const TDoorView &view, TDoor *doors, int cap, TDoorStats *stats = NULL);

#endif // CAPDOOR_H
