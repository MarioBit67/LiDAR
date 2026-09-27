#ifndef CAPGEOM_H
#define CAPGEOM_H
#include "winTypes.h"

/* Geometry shared by every capture platform.
 * World frame: x east, y up, -z north (the ARKit gravityAndHeading frame).
 * Poses are camera-to-world, column-major; camera axes x right, y up, looking down -z
 * (the ARKit / ARCore / OpenGL camera convention). */

// 3-component vector (meters or unit direction)
struct TVec3 {
   float x, y, z;
};

// Pinhole intrinsics in pixels, for the resolution of the image they travel with
struct TIntrinsics {
   float fx, fy, cx, cy;
};

// 4x4 rigid transform, column-major: m[col*4 + row]
struct TMat4 {
   float m[16];

   static TMat4 Identity(void);

   TVec3 TransformPoint(const TVec3 &p) const;
   TVec3 RotateVector(const TVec3 &v) const;
   TVec3 Translation(void) const;
   TVec3 Forward(void) const; // world direction the camera looks at (-z axis)
   TMat4 Multiply(const TMat4 &b) const;
};

// Clockwise angle from north of a world direction, [0, 360)
float geomHeadingDeg(const TVec3 &dir);

// Elevation of a world direction above the horizon, [-90, 90]
float geomPitchDeg(const TVec3 &dir);

// Roll about the camera forward axis, gravity as zero: degrees, clockwise as the camera sees it (NaN straight up/down)
float geomRollDeg(const TMat4 &cameraToWorld);

// Smallest absolute difference between two headings, [0, 180]
float geomHeadingDiffDeg(float a, float b);

#endif // CAPGEOM_H
