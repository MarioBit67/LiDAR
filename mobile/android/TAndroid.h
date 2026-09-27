#ifndef TANDROID_H
#define TANDROID_H
#include "winTypes.h"
#include "capPort.h"
#include "thread.h"

#include <android_native_app_glue.h>
#include <android/sensor.h>
#include <camera/NdkCameraManager.h>
#include <camera/NdkCameraDevice.h>
#include <camera/NdkCaptureRequest.h>
#include <media/NdkImageReader.h>

/* ==================================================================================================
   TAndroid - the Android TCapPort (NativeActivity, pure C++). It READS the platform (NDK camera, the
   rotation-vector sensor, GNSS through the Java bridge) and DISPLAYS (the window's pixel buffer, text
   rasterized by android.graphics). No business rule lives here: every event is a raw fact handed to
   the sink, every decision is the common app's. All JVM names and signatures live in TAndroid.cpp and
   reach capJNI as parameters.
   ================================================================================================== */

enum {
   androidMaxCameras = 8,
   androidIdLen      = 32
};

class TAndroid : public TCapPort
{
 public:
   explicit TAndroid(struct android_app *app);

   ~TAndroid(void);

   // -- glue side (android_main loop) --
   void HandleCommand(LONG cmd);
   LONG HandleInput(AInputEvent *e);
   void PumpSensors(void);
   void Tick(void);
   void DeliverImage(AImageReader *reader); // camera thread

   // -- TCapPort --
   void  SetSink(TCapSink *sink) override;
   void  RequestPermissions(void) override;
   int   ListCameras(TCamInfo *out, int max) override;
   bool  StartCamera(int index, int maxPixels, float focusDiopters, int maxExposureHz) override;
   void  StopCamera(void) override;
   void  StartSensors(void) override;
   void  StopSensors(void) override;
   void  StartLocation(void) override;
   void  StopLocation(void) override;
   void  KeepScreenOn(bool on) override;
   void  Vibrate(int ms) override;
   void  RequestPaint(void) override;
   void  MeasureText(LPCSTR text, const TTextStyle &style, int *outW, int *outH) override;
   void  DrawText(TSurface &s, int x, int y, LPCSTR text, DWORD rgba, const TTextStyle &style) override;
   void  DataDir(LPSTR out, size_t cap) override;
   void  Device(TDeviceInfo &out) override;
   QWORD WallClockNs(void) override;
   QWORD SensorClockNs(void) override;
   void  Log(LPCSTR msg) override;

   TAndroid(const TAndroid &) = delete;
   TAndroid &operator=(const TAndroid &) = delete;

 private:
   bool   hasPermission(LPCSTR name);
   LPVOID systemService(LPCSTR name);
   LPVOID paintFor(const TTextStyle &style);
   void   pollLocation(void);
   void   paintNow(void);
   void   applyResize(void);

   struct android_app     *Papp;
   TCapSink               *Psink;
   TMutex                  Pmutex;
   LPVOID                  PlocMgr,      // global refs (Java)
                           PlocIntent,
                           Pvibrator,
                           Pbitmap,
                           Pcanvas,
                           Ppaint,
                           PpaintBold;
   char                    PcamIds[androidMaxCameras][androidIdLen];
   TCamInfo                Pcams[androidMaxCameras];
   int                     PcamCount,
                           PbitmapW,
                           PbitmapH,
                           PdownX,
                           PdownY,
                           Pdensity;
   QWORD                   PlastFixNs,
                           PlastPollNs,
                           PlastPaintNs;
   bool                    Pmagnetic,
                           Pasked,
                           Pdirty,
                           Pvisible;

   ACameraManager         *PcamMgr; // NDK handles
   ACameraDevice          *Pdevice;
   ACameraCaptureSession  *Psession;
   ACaptureSessionOutputContainer *Pcontainer;
   ACaptureSessionOutput  *Poutput;
   ACameraOutputTarget    *Ptarget;
   ACaptureRequest        *Prequest;
   AImageReader           *Preader;
   ASensorManager         *PsensorMgr;
   ASensorEventQueue      *Pqueue;

   const ASensor *Pattitude;
};

#endif // TANDROID_H
