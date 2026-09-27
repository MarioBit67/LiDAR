#include "capGeom.h"
#include "libDiscipline.h"

static const float cRadToDeg = 57.29577951f;

//--------------------------------------------------------------------------------
TMat4 TMat4::Identity(void)
{
   TMat4 r = {};

   r.m[0] = 1.f;
   r.m[5] = 1.f;
   r.m[10] = 1.f;
   r.m[15] = 1.f;
   return r;
}

//--------------------------------------------------------------------------------
TVec3 TMat4::TransformPoint(const TVec3 &p) const
{
   TVec3 r = RotateVector(p);

   r.x += m[12];
   r.y += m[13];
   r.z += m[14];
   return r;
}

//--------------------------------------------------------------------------------
TVec3 TMat4::RotateVector(const TVec3 &v) const
{
   TVec3 r;

   r.x = m[0]*v.x + m[4]*v.y + m[8]*v.z;
   r.y = m[1]*v.x + m[5]*v.y + m[9]*v.z;
   r.z = m[2]*v.x + m[6]*v.y + m[10]*v.z;
   return r;
}

//--------------------------------------------------------------------------------
TVec3 TMat4::Translation(void) const
{
   TVec3 r;

   r.x = m[12];
   r.y = m[13];
   r.z = m[14];
   return r;
}

//--------------------------------------------------------------------------------
TVec3 TMat4::Forward(void) const
{
   TVec3 r;

   r.x = -m[8];
   r.y = -m[9];
   r.z = -m[10];
   return r;
}

//--------------------------------------------------------------------------------
TMat4 TMat4::Multiply(const TMat4 &b) const
{
   TMat4 r;

   for (int col = 0; col < 4; col++)
      for (int row = 0; row < 4; row++)
      {
         float sum = 0.f;

         for (int k = 0; k < 4; k++)
            sum += m[k*4 + row]*b.m[col*4 + k];
         r.m[col*4 + row] = sum;
      }
   return r;
}

//--------------------------------------------------------------------------------
float geomHeadingDeg(const TVec3 &dir)
{
   float east = dir.x,
         north = -dir.z,
         deg = atan2f(east, north)*cRadToDeg;

   if (deg < 0.f)
      deg += 360.f;
   return deg;
}

//--------------------------------------------------------------------------------
float geomPitchDeg(const TVec3 &dir)
{
   float horiz = sqrtf(dir.x*dir.x + dir.z*dir.z);

   return atan2f(dir.y, horiz)*cRadToDeg;
}

/*--------------------------------------------------------------------------------
   Roll of the camera about its forward axis, gravity as zero: the angle from world up (projected
   onto the image plane) to the camera's up axis, positive clockwise as the camera sees it. NaN
   when the camera looks almost straight up or down (world up has no image-plane direction).
  --------------------------------------------------------------------------------*/
float geomRollDeg(const TMat4 &cameraToWorld)
{
   TVec3 f = cameraToWorld.Forward(),
         cy = { 0.f, 1.f, 0.f },
         up = cameraToWorld.RotateVector(cy);
   float d = f.y; // world up . forward
   TVec3 ref = { -d*f.x, 1.f - d*f.y, -d*f.z };
   float len = sqrtf(ref.x*ref.x + ref.y*ref.y + ref.z*ref.z);

   if (len < 0.14f) // within ~8 degrees of the zenith or nadir
      return NAN;
   ref.x /= len;
   ref.y /= len;
   ref.z /= len;

   // sin from (ref x up) . forward, cos from ref . up; forward points away from the viewer
   TVec3 c = { ref.y*up.z - ref.z*up.y, ref.z*up.x - ref.x*up.z, ref.x*up.y - ref.y*up.x };

   return atan2f(-(c.x*f.x + c.y*f.y + c.z*f.z), ref.x*up.x + ref.y*up.y + ref.z*up.z)*cRadToDeg;
}

//--------------------------------------------------------------------------------
float geomHeadingDiffDeg(float a, float b)
{
   float d = fabsf(fmodf(a - b, 360.f));

   if (d > 180.f)
      d = 360.f - d;
   return d;
}

//--------------------------------------------------------------------------------
