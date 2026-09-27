#include "capOrient.h"
#include "libDiscipline.h"

//--------------------------------------------------------------------------------
// Rotation-only 4x4 from three column vectors (column-major)
static TMat4 orientFromColumns(const TVec3 &c0, const TVec3 &c1, const TVec3 &c2)
{
   TMat4 r = TMat4::Identity();

   r.m[0] = c0.x;
   r.m[1] = c0.y;
   r.m[2] = c0.z;
   r.m[4] = c1.x;
   r.m[5] = c1.y;
   r.m[6] = c1.z;
   r.m[8] = c2.x;
   r.m[9] = c2.y;
   r.m[10] = c2.z;
   return r;
}

//--------------------------------------------------------------------------------
static TMat4 orientFromQuat(const float quat[4])
{
   float x = quat[0],
         y = quat[1],
         z = quat[2],
         w = quat[3];
   TVec3 c0 = { 1.f - 2.f*(y*y + z*z), 2.f*(x*y + w*z), 2.f*(x*z - w*y) },
         c1 = { 2.f*(x*y - w*z), 1.f - 2.f*(x*x + z*z), 2.f*(y*z + w*x) },
         c2 = { 2.f*(x*z + w*y), 2.f*(y*z - w*x), 1.f - 2.f*(x*x + y*y) };

   return orientFromColumns(c0, c1, c2);
}

//--------------------------------------------------------------------------------
// Maps coordinates of the platform frame into the capture frame
static TMat4 orientFrameChange(TSensorFrame frame)
{
   TVec3 c0 = { 1.f, 0.f, 0.f },
         c1 = { 0.f, 1.f, 0.f },
         c2 = { 0.f, 0.f, 1.f };

   switch (frame)
   {
      case sfEastNorthUp :
         c1 = { 0.f, 0.f, -1.f }; // north -> -z
         c2 = { 0.f, 1.f, 0.f };  // up -> y
         break;

      case sfNorthWestUp :
         c0 = { 0.f, 0.f, -1.f }; // north -> -z
         c1 = { -1.f, 0.f, 0.f }; // west -> -x
         c2 = { 0.f, 1.f, 0.f };  // up -> y
         break;

      default :
         break;
   }
   return orientFromColumns(c0, c1, c2);
}

//--------------------------------------------------------------------------------
// Camera axes expressed in device coordinates, for the native sensor image rotation
static TMat4 orientCameraToDevice(int sensorRotDeg)
{
   TVec3 c0 = { 1.f, 0.f, 0.f },
         c1 = { 0.f, 1.f, 0.f },
         c2 = { 0.f, 0.f, 1.f };

   switch (sensorRotDeg)
   {
      case 90 :
         c0 = { 0.f, -1.f, 0.f };
         c1 = { 1.f, 0.f, 0.f };
         break;

      case 180 :
         c0 = { -1.f, 0.f, 0.f };
         c1 = { 0.f, -1.f, 0.f };
         break;

      case 270 :
         c0 = { 0.f, 1.f, 0.f };
         c1 = { -1.f, 0.f, 0.f };
         break;

      default :
         break;
   }
   return orientFromColumns(c0, c1, c2);
}

//--------------------------------------------------------------------------------
TMat4 orientCameraToWorld(const float quat[4], TSensorFrame frame, int sensorRotDeg)
{
   TMat4 deviceToFrame = orientFromQuat(quat),
         frameToWorld = orientFrameChange(frame),
         cameraToDevice = orientCameraToDevice(sensorRotDeg);

   return frameToWorld.Multiply(deviceToFrame).Multiply(cameraToDevice);
}

//--------------------------------------------------------------------------------
