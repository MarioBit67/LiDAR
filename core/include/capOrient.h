#ifndef CAPORIENT_H
#define CAPORIENT_H
#include "winTypes.h"
#include "capGeom.h"

/* Converts a platform attitude into the capture world frame (x east, y up, -z north).
 * The quaternion is device-to-frame (x, y, z, w). Device axes: x right, y top, z out of
 * the screen (portrait). The back camera looks down -z of the device. */

// Reference frame the platform attitude is expressed in
enum TSensorFrame {
   sfCapture,     // already the capture frame (ARKit gravityAndHeading)
   sfEastNorthUp, // Android TYPE_ROTATION_VECTOR: x east, y north (magnetic), z up
   sfNorthWestUp  // CoreMotion xTrueNorthZVertical: x north, y west, z up
};

/* Camera-to-world for the back camera. sensorRotDeg is the clockwise rotation (0, 90, 180,
 * 270) that turns the native sensor image upright in portrait (Android
 * SENSOR_ORIENTATION; 90 for iPhone buffers). The result has zero translation. */
TMat4 orientCameraToWorld(const float quat[4], TSensorFrame frame, int sensorRotDeg);

#endif // CAPORIENT_H
