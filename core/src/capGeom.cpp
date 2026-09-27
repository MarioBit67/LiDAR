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

//--------------------------------------------------------------------------------
float geomHeadingDiffDeg(float a, float b)
{
   float d = fabsf(fmodf(a - b, 360.f));

   if (d > 180.f)
      d = 360.f - d;
   return d;
}

//--------------------------------------------------------------------------------
