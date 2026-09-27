#ifndef CAPPORT_H
#define CAPPORT_H
#include "winTypes.h"
#include "capGeom.h"
#include "capOrient.h"
#include "capRecord.h"
#include "capSession.h"
#include "capJPEG.h"

#include <stddef.h>

/* ==================================================================================================
   TCapPort - the abstract PLATFORM PORT of the capture app.

   The platform layer READS sensors (camera, attitude, GNSS) and DISPLAYS (a pixel surface, text); it
   hands raw facts up and never decides anything. Every business rule - which frame to keep, attitude
   conversion, spin coverage, JPEG, session and rooms, operator hints, layout - lives in the ONE common
   app (capApp), written once for every platform.

      capApp (common)  <-- methods (down) / events (up, TCapSink) -->  TCapPort  <-- TAndroid / TiOS
   ================================================================================================== */

// Pixel surface to draw on: 32-bit pixels in memory order R, G, B, X
struct TSurface {
   LPDWORD pixels;
   int     width, height, stride; // stride in pixels
};

struct TTextStyle {
   int  sizePx;
   bool bold;
};

// Static facts of the camera the platform opened (all raw, as the hardware reports them)
struct TCamInfo {
   int   width, height,       // output buffer size, native sensor orientation
         arrayWidth,          // pixel array the calibration refers to
         arrayHeight,
         sensorRotDeg;        // clockwise rotation that makes the image upright in portrait
   float calib[5],            // fx fy cx cy s in pixel-array coordinates (cx = cy = 0 when unknown)
         distortion[5],
         focalMm,
         sensorWidthMm,       // physical sensor size, native orientation
         sensorHeightMm;
   char  label[48];           // e.g. "back 0 (4.71 mm)"
};

// One camera frame; the planes are valid only during OnFrame
struct TCamFrame {
   QWORD     stampNs; // sensor clock (same base as attitude stamps)
   TYUVImage yuv;
};

// Device attitude as the platform sensor reports it
struct TAttitude {
   QWORD        stampNs;
   float        quat[4];     // device-to-frame (x, y, z, w)
   TSensorFrame frame;
   bool         magnetic;    // false: yaw reference is arbitrary (no magnetometer)
   float        accuracyDeg; // heading accuracy, NaN unknown
};

// Inbound events (platform -> app); threads as noted
class TCapSink
{
 public:
   virtual ~TCapSink() {}

   virtual void OnResize(int w, int h, int densityQ8) = 0;    // main thread; 256 = 1x
   virtual void OnPaint(TSurface &s) = 0;                     // main thread; draw the whole frame
   virtual void OnTap(int x, int y) = 0;                      // main thread
   virtual bool OnBack(void) = 0;                             // main thread; false = let the platform exit
   virtual void OnPause(void) = 0;                            // main thread; app goes to background
   virtual void OnResume(void) = 0;                           // main thread
   virtual void OnPermissions(bool camera, bool location) = 0; // main thread
   virtual void OnCameraReady(const TCamInfo &info) = 0;      // any thread
   virtual void OnFocus(bool locked, float diopters) = 0;     // camera thread; diopters where the lens settled (NaN unknown)
   virtual void OnFrame(const TCamFrame &frame) = 0;          // camera thread
   virtual void OnAttitude(const TAttitude &a) = 0;           // sensor thread
   virtual void OnLocation(QWORD stampNs, const TLocationRecord &loc) = 0; // any thread
};

// Outbound services (app -> platform)
class TCapPort
{
 public:
   virtual ~TCapPort() {}

   virtual void SetSink(TCapSink *sink) = 0;

   virtual void RequestPermissions(void) = 0;       // answer arrives through OnPermissions
   virtual int  ListCameras(TCamInfo *out, int max) = 0; // back cameras; width/height = largest YUV size
   /* Largest 4:3 YUV <= maxPixels; focusDiopters > 0 = fixed focus (manual lens when supported, else
      continuous AF), <= 0 = continuous AF; exposure capped at 1/maxExposureHz. OnCameraReady follows. */
   virtual bool StartCamera(int index, int maxPixels, float focusDiopters, int maxExposureHz) = 0;
   virtual void StopCamera(void) = 0;
   /* One focus run on what the camera sees now (autofocus), then the lens is held where it settled (manual
      lens), so the intrinsics stay put until the next run. OnFocus follows; without a manual lens the camera
      keeps its continuous AF: false, and no OnFocus follows. */
   /* focus measured in count rectangles of the NATIVE image, 4 floats each (left, top, right, bottom, 0..1 on each
      axis), most telling first: the platform takes as many as the camera allows */
   virtual bool Autofocus(const float *rects, int count) = 0;
   virtual void StartSensors(void) = 0;             // attitude at ~100 Hz
   virtual void StopSensors(void) = 0;
   virtual void StartLocation(void) = 0;            // GNSS fixes as they come
   virtual void StopLocation(void) = 0;
   virtual void KeepScreenOn(bool on) = 0;
   virtual void Vibrate(int ms) = 0;
   virtual void RequestPaint(void) = 0;             // thread-safe: schedule an OnPaint

   virtual void MeasureText(LPCSTR text, const TTextStyle &style, int *outW, int *outH) = 0;
   virtual void DrawText(TSurface &s, int x, int y, LPCSTR text, DWORD rgba, const TTextStyle &style) = 0;

   virtual void  DataDir(LPSTR out, size_t cap) = 0; // writable, user-retrievable folder
   virtual void  Device(TDeviceInfo &out) = 0;       // platform / model / os filled in
   virtual QWORD WallClockNs(void) = 0;              // Unix epoch
   virtual QWORD SensorClockNs(void) = 0;            // same base as frame and attitude stamps
   virtual void  Log(LPCSTR msg) = 0;
};

// Launch the one common app on a platform port; the app registers itself as the sink
void appStart(TCapPort *port);

// Tear the app down (closes the session)
void appStop(void);

#endif // CAPPORT_H
