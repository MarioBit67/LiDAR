#ifndef CAPMOSAIC_H
#define CAPMOSAIC_H
#include "winTypes.h"
#include "alloc.h"
#include "capGeom.h"
#include "capLayout.h"

/* Wall mosaics of one room from its center-spin keyframes (desktop tool). Every wall of the plan is
 * a known rectangle; each keyframe is placed by its vanishing-point rotation and its center on the
 * spin circle, then refined: each frame's picture of a wall is matched (edges, subpixel) against the
 * mosaic of the OTHER frames, and the shift on the wall turns into a small rotation of that frame.
 * Windows, jambs, handles and prints carry the signal; the ceiling box stays the anchor. */

// One center-spin keyframe ready for the mosaic
struct TMosaicFrame {
   TVec3        camA,    // camera axes of: the room axis, the true vertical, their cross product
                camUp,
                camC,
                worldA,  // the same axes in the world (plan axis + 90k, up, their cross product)
                worldC,
                delta,   // refinement: small world rotation vector applied before the camera mapping
                gyroX,   // camera axes in the gyroscope's world: frame to frame the attitude is sharper than any line
                gyroY,
                gyroZ;
   float        camX,    // camera center: its station plus the spin circle (meters, spin point at the origin)
                camZ,
                camY,    // camera height off the plan's (the corner fit sets it; 0 until then)
                stationX, // where the operator stood (the center spin at the origin; corner stations adjusted)
                stationZ;
   TIntrinsics  k;       // of the reduced picture
   bool         xyz,     // its own picture measured the vertical and both room axes: only these enter the merge
                picked;  // named by the user as a corner frame (--corner-frames): XYZ whatever the measures say
   int          w,
                h,
                index,   // keyframe number in the session
                station; // 0: the center spin, k: corner station k
   TBlock<BYTE> bgr;     // reduced picture, 24-bit BGR
};

/* Refines the frames (rounds against the others, bundle rounds with the geometry, optional joint rounds), writes
 * parede_K_<length>m.bmp; the refined plan is printed. frames[0, centers) are the center spin, the rest corner
 * stations (used by the bundle adjustment and the final composition only). fixPlan: the bundle holds walls and camera
 * height on the plan (only rotations, stations and spin radius move) */
void mosaicWalls(TMosaicFrame *frames, int count, int centers, const TLayoutPlan &plan, int rounds,
                 int bundleRounds, int jointRounds, float spinRadiusM, bool fixPlan, LPCSTR outDir);

// Names the five target images for the components (--face-names "O,N,L,S,P": walls in plan order, then the floor)
void mosaicSetFaceNames(LPCSTR list);

// A face's name for files and prints: the user's (--face-names), else parede<K> / piso
void mosaicFaceLabel(int face, LPSTR out, size_t cap);

#endif // CAPMOSAIC_H
