#include "TAndroid.h"
#include "capJNI.h"

#include <android/log.h>
#include <android/window.h>
#include <camera/NdkCameraMetadata.h>
#include <sys/system_properties.h>
#include <time.h>
#include "libDiscipline.h"

static LPCSTR cPackage = "io.aeroblox.lidar",
              cLogTag = "lidar",
              cPermCamera = "android.permission.CAMERA",
              cPermLocation = "android.permission.ACCESS_FINE_LOCATION",
              cStringArg = "(Ljava/lang/String;)Ljava/lang/Object;",
              cCtor = "<init>";

enum {
   androidTapSlopDp  = 24,
   androidPollMs     = 50,
   androidPaintMs    = 33,
   androidLocationMs = 1000,
   androidTextRows   = 160,
   androidSensorUs   = 10000,
   androidMaxImages  = 4,
   androidAntiAlias  = 1,         // Paint.ANTI_ALIAS_FLAG
   androidPendingFlags = 0x0C000000 // PendingIntent.FLAG_IMMUTABLE | FLAG_UPDATE_CURRENT
};

//--------------------------------------------------------------------------------
static void androidOnImage(LPVOID context, AImageReader *reader)
{
   ((TAndroid *)context)->DeliverImage(reader);
}

//--------------------------------------------------------------------------------
static void androidOnDisconnected(LPVOID context, ACameraDevice *device)
{
   (void)context;
   (void)device;
}

//--------------------------------------------------------------------------------
static void androidOnError(LPVOID context, ACameraDevice *device, int error)
{
   (void)device;
   ((TAndroid *)context)->Log(error ? "camera device error" : "camera device event");
}

//--------------------------------------------------------------------------------
static void androidOnSession(LPVOID context, ACameraCaptureSession *session)
{
   (void)context;
   (void)session;
}

//--------------------------------------------------------------------------------
static QWORD androidClock(clockid_t id)
{
   struct timespec ts;

   clock_gettime(id, &ts);
   return (QWORD)ts.tv_sec*1000000000u + (QWORD)ts.tv_nsec;
}

//--------------------------------------------------------------------------------
TAndroid::TAndroid(struct android_app *app) : Papp(app), Psink(NULL), PlocMgr(NULL), PlocIntent(NULL),
   Pvibrator(NULL), Pbitmap(NULL), Pcanvas(NULL), Ppaint(NULL), PpaintBold(NULL), PcamIds(), Pcams(),
   PcamCount(0), PbitmapW(0), PbitmapH(0), PdownX(0), PdownY(0), Pdensity(256), PlastFixNs(0u),
   PlastPollNs(0u), PlastPaintNs(0u), Pmagnetic(false), Pasked(false), Pdirty(true), Pvisible(false),
   PcamMgr(NULL), Pdevice(NULL), Psession(NULL), Pcontainer(NULL), Poutput(NULL), Ptarget(NULL),
   Prequest(NULL), Preader(NULL), PsensorMgr(NULL), Pqueue(NULL), Pattitude(NULL)
{
   jniInit(app->activity->vm, app->activity->clazz);
   PcamMgr = ACameraManager_create();
   Pvibrator = jniGlobal(systemService("vibrator"));

   TJArg flags = jniInt(androidAntiAlias, jatInt),
         white = jniInt((QWORD)(LONG)-1, jatInt);

   jniFrameBegin(16);
   Ppaint = jniGlobal(jniNew("android/graphics/Paint", cCtor, "(I)V", &flags, 1));
   PpaintBold = jniGlobal(jniNew("android/graphics/Paint", cCtor, "(I)V", &flags, 1));
   jniCallVoid(Ppaint, "setColor", "(I)V", &white, 1);
   jniCallVoid(PpaintBold, "setColor", "(I)V", &white, 1);

   TJArg bold = jniObjArg(jniStaticField("android/graphics/Typeface", "DEFAULT_BOLD", "Landroid/graphics/Typeface;"));

   jniCallObj(PpaintBold, "setTypeface", "(Landroid/graphics/Typeface;)Landroid/graphics/Typeface;", &bold, 1);
   jniFrameEnd();
}

//--------------------------------------------------------------------------------
TAndroid::~TAndroid(void)
{
   StopCamera();
   StopSensors();
   StopLocation();
   if (PcamMgr)
      ACameraManager_delete(PcamMgr);
   jniDropGlobal(Pvibrator);
   jniDropGlobal(Pbitmap);
   jniDropGlobal(Pcanvas);
   jniDropGlobal(Ppaint);
   jniDropGlobal(PpaintBold);
}

//--------------------------------------------------------------------------------
void TAndroid::SetSink(TCapSink *sink)
{
   Psink = sink;
   applyResize();
}

//--------------------------------------------------------------------------------
void TAndroid::applyResize(void)
{
   if (!Psink || !Papp->window)
      return;

   int dpi = AConfiguration_getDensity(Papp->config);

   Pdensity = dpi > 0 ? dpi*256/160 : 256;
   Psink->OnResize(ANativeWindow_getWidth(Papp->window), ANativeWindow_getHeight(Papp->window), Pdensity);
   RequestPaint();
}

//--------------------------------------------------------------------------------
void TAndroid::HandleCommand(LONG cmd)
{
   switch (cmd)
   {
      case APP_CMD_INIT_WINDOW :
         ANativeWindow_setBuffersGeometry(Papp->window, 0, 0, WINDOW_FORMAT_RGBX_8888);
         Pvisible = true;
         applyResize();
         break;

      case APP_CMD_TERM_WINDOW :
         Pvisible = false;
         break;

      case APP_CMD_WINDOW_RESIZED :
      case APP_CMD_CONFIG_CHANGED :
         applyResize();
         break;

      case APP_CMD_RESUME :
         if (Psink)
            Psink->OnResume();
         break;

      case APP_CMD_PAUSE :
         if (Psink)
            Psink->OnPause();
         break;

      default :
         break;
   }
}

//--------------------------------------------------------------------------------
LONG TAndroid::HandleInput(AInputEvent *e)
{
   if (AInputEvent_getType(e) == AINPUT_EVENT_TYPE_KEY)
   {
      if (AKeyEvent_getKeyCode(e) != AKEYCODE_BACK)
         return 0;
      if (AKeyEvent_getAction(e) == AKEY_EVENT_ACTION_UP && Psink && !Psink->OnBack())
         ANativeActivity_finish(Papp->activity);
      return 1;
   }
   if (AInputEvent_getType(e) != AINPUT_EVENT_TYPE_MOTION)
      return 0;

   int action = AMotionEvent_getAction(e) & AMOTION_EVENT_ACTION_MASK,
       x = (int)AMotionEvent_getX(e, 0),
       y = (int)AMotionEvent_getY(e, 0),
       slop = androidTapSlopDp*Pdensity/256;

   if (action == AMOTION_EVENT_ACTION_DOWN)
   {
      PdownX = x;
      PdownY = y;
   }
   else if (action == AMOTION_EVENT_ACTION_UP && Psink && abs(x - PdownX) < slop && abs(y - PdownY) < slop)
      Psink->OnTap(x, y);
   return 1;
}

//--------------------------------------------------------------------------------
LPVOID TAndroid::systemService(LPCSTR name)
{
   TJArg arg = jniStr(name);

   return jniCallObj(jniActivity(), "getSystemService", cStringArg, &arg, 1);
}

//--------------------------------------------------------------------------------
bool TAndroid::hasPermission(LPCSTR name)
{
   TJArg arg = jniStr(name);
   int   granted;

   jniFrameBegin(8);
   granted = jniCallInt(jniActivity(), "checkSelfPermission", "(Ljava/lang/String;)I", &arg, 1);
   jniFrameEnd();
   return granted == 0; // PackageManager.PERMISSION_GRANTED
}

//--------------------------------------------------------------------------------
void TAndroid::RequestPermissions(void)
{
   bool camera = hasPermission(cPermCamera),
        location = hasPermission(cPermLocation);

   if ((camera && location) || Pasked)
   {
      if (Psink)
         Psink->OnPermissions(camera, location);
      return;
   }

   LPCSTR perms[2] = { cPermCamera, cPermLocation };
   TJArg  args[2];

   Pasked = true; // the answer comes back with the next APP_CMD_RESUME
   jniFrameBegin(8);
   args[0] = jniObjArg(jniStringArray(perms, 2));
   args[1] = jniInt(1u, jatInt);
   jniCallVoid(jniActivity(), "requestPermissions", "([Ljava/lang/String;I)V", args, 2);
   jniFrameEnd();
}

//--------------------------------------------------------------------------------
int TAndroid::ListCameras(TCamInfo *out, int max)
{
   ACameraIdList *list = NULL;

   PcamCount = 0;
   if (!PcamMgr || ACameraManager_getCameraIdList(PcamMgr, &list) != ACAMERA_OK)
      return 0;
   for (int i = 0; i < list->numCameras && PcamCount < androidMaxCameras && PcamCount < max; i++)
   {
      ACameraMetadata *meta = NULL;

      if (ACameraManager_getCameraCharacteristics(PcamMgr, list->cameraIds[i], &meta) != ACAMERA_OK)
         continue;

      TCamInfo info = {};
      bool     back = false,
               compatible = false;

      ACameraMetadata_const_entry e;

      if (ACameraMetadata_getConstEntry(meta, ACAMERA_LENS_FACING, &e) == ACAMERA_OK)
         back = e.data.u8[0] == ACAMERA_LENS_FACING_BACK;
      if (ACameraMetadata_getConstEntry(meta, ACAMERA_REQUEST_AVAILABLE_CAPABILITIES, &e) == ACAMERA_OK)
         for (DWORD k = 0; k < e.count; k++)
            if (e.data.u8[k] == ACAMERA_REQUEST_AVAILABLE_CAPABILITIES_BACKWARD_COMPATIBLE)
               compatible = true;
      if (back && compatible)
      {
         if (ACameraMetadata_getConstEntry(meta, ACAMERA_SCALER_AVAILABLE_STREAM_CONFIGURATIONS, &e) == ACAMERA_OK)
            for (DWORD k = 0; k + 3 < e.count; k += 4)
               if (e.data.i32[k] == AIMAGE_FORMAT_YUV_420_888 && e.data.i32[k + 3] == 0
                   && (QWORD)e.data.i32[k + 1]*e.data.i32[k + 2] > (QWORD)info.width*info.height)
               {
                  info.width = e.data.i32[k + 1];
                  info.height = e.data.i32[k + 2];
               }
         if (ACameraMetadata_getConstEntry(meta, ACAMERA_SENSOR_INFO_PRE_CORRECTION_ACTIVE_ARRAY_SIZE, &e) == ACAMERA_OK)
         {
            info.arrayWidth = e.data.i32[2];
            info.arrayHeight = e.data.i32[3];
         }
         if (ACameraMetadata_getConstEntry(meta, ACAMERA_LENS_INTRINSIC_CALIBRATION, &e) == ACAMERA_OK)
            for (DWORD k = 0; k < e.count && k < 5; k++)
               info.calib[k] = e.data.f[k];
         if (ACameraMetadata_getConstEntry(meta, ACAMERA_LENS_DISTORTION, &e) == ACAMERA_OK)
            for (DWORD k = 0; k < e.count && k < 5; k++)
               info.distortion[k] = e.data.f[k];
         if (ACameraMetadata_getConstEntry(meta, ACAMERA_LENS_INFO_AVAILABLE_FOCAL_LENGTHS, &e) == ACAMERA_OK)
            info.focalMm = e.data.f[0];
         if (ACameraMetadata_getConstEntry(meta, ACAMERA_SENSOR_INFO_PHYSICAL_SIZE, &e) == ACAMERA_OK)
         {
            info.sensorWidthMm = e.data.f[0];
            info.sensorHeightMm = e.data.f[1];
         }
         if (ACameraMetadata_getConstEntry(meta, ACAMERA_SENSOR_ORIENTATION, &e) == ACAMERA_OK)
            info.sensorRotDeg = e.data.i32[0];
         snprintf(info.label, sizeof(info.label), "back %s (%.2f mm)", list->cameraIds[i], info.focalMm);
         snprintf(PcamIds[PcamCount], androidIdLen, "%s", list->cameraIds[i]);
         Pcams[PcamCount] = info;
         out[PcamCount] = info;
         PcamCount++;
      }
      ACameraMetadata_free(meta);
   }
   ACameraManager_deleteCameraIdList(list);
   return PcamCount;
}

//--------------------------------------------------------------------------------
bool TAndroid::StartCamera(int index, int maxPixels, float focusDiopters, int maxExposureHz)
{
   TCamInfo info;
   int      w = 0,
            h = 0;

   ACameraMetadata *meta = NULL;
   ANativeWindow   *win = NULL;

   if (index < 0 || index >= PcamCount)
      return false;
   StopCamera();
   info = Pcams[index];
   if (ACameraManager_getCameraCharacteristics(PcamMgr, PcamIds[index], &meta) != ACAMERA_OK)
      return false;

   ACameraMetadata_const_entry e;

   if (ACameraMetadata_getConstEntry(meta, ACAMERA_SCALER_AVAILABLE_STREAM_CONFIGURATIONS, &e) == ACAMERA_OK)
      for (DWORD k = 0; k + 3 < e.count; k += 4)
      {
         int cw = e.data.i32[k + 1],
             ch = e.data.i32[k + 2];

         if (e.data.i32[k] == AIMAGE_FORMAT_YUV_420_888 && e.data.i32[k + 3] == 0 && cw*3 == ch*4
             && (QWORD)cw*ch <= (QWORD)maxPixels && (QWORD)cw*ch > (QWORD)w*h)
         {
            w = cw;
            h = ch;
         }
      }

   bool manualLens = false,
        fpsLock = false;

   if (ACameraMetadata_getConstEntry(meta, ACAMERA_REQUEST_AVAILABLE_CAPABILITIES, &e) == ACAMERA_OK)
      for (DWORD k = 0; k < e.count; k++)
         if (e.data.u8[k] == ACAMERA_REQUEST_AVAILABLE_CAPABILITIES_MANUAL_SENSOR)
            manualLens = true;
   if (ACameraMetadata_getConstEntry(meta, ACAMERA_CONTROL_AE_AVAILABLE_TARGET_FPS_RANGES, &e) == ACAMERA_OK)
      for (DWORD k = 0; k + 1 < e.count; k += 2)
         if (e.data.i32[k] == maxExposureHz && e.data.i32[k + 1] == maxExposureHz)
            fpsLock = true;
   ACameraMetadata_free(meta);
   if (!w)
      return false;

   AImageReader_ImageListener listener = { this, androidOnImage };
   ACameraDevice_StateCallbacks deviceCb = { this, androidOnDisconnected, androidOnError };
   ACameraCaptureSession_stateCallbacks sessionCb = { this, androidOnSession, androidOnSession, androidOnSession };
   BYTE af = ACAMERA_CONTROL_AF_MODE_CONTINUOUS_PICTURE;

   if (AImageReader_new(w, h, AIMAGE_FORMAT_YUV_420_888, androidMaxImages, &Preader) != AMEDIA_OK)
      return false;
   AImageReader_setImageListener(Preader, &listener);
   AImageReader_getWindow(Preader, &win);
   if (ACameraManager_openCamera(PcamMgr, PcamIds[index], &deviceCb, &Pdevice) != ACAMERA_OK)
   {
      StopCamera();
      return false;
   }
   ACaptureSessionOutputContainer_create(&Pcontainer);
   ACaptureSessionOutput_create(win, &Poutput);
   ACaptureSessionOutputContainer_add(Pcontainer, Poutput);
   ACameraOutputTarget_create(win, &Ptarget);
   ACameraDevice_createCaptureRequest(Pdevice, TEMPLATE_RECORD, &Prequest);
   ACaptureRequest_addTarget(Prequest, Ptarget);
   if (focusDiopters > 0.f && manualLens)
   {
      BYTE off = ACAMERA_CONTROL_AF_MODE_OFF;

      ACaptureRequest_setEntry_u8(Prequest, ACAMERA_CONTROL_AF_MODE, 1, &off);
      ACaptureRequest_setEntry_float(Prequest, ACAMERA_LENS_FOCUS_DISTANCE, 1, &focusDiopters);
   }
   else
      ACaptureRequest_setEntry_u8(Prequest, ACAMERA_CONTROL_AF_MODE, 1, &af);
   if (fpsLock)
   {
      LONG range[2] = { maxExposureHz, maxExposureHz };

      ACaptureRequest_setEntry_i32(Prequest, ACAMERA_CONTROL_AE_TARGET_FPS_RANGE, 2, range);
   }
   if (ACameraDevice_createCaptureSession(Pdevice, Pcontainer, &sessionCb, &Psession) != ACAMERA_OK
       || ACameraCaptureSession_setRepeatingRequest(Psession, NULL, 1, &Prequest, NULL) != ACAMERA_OK)
   {
      StopCamera();
      return false;
   }
   info.width = w;
   info.height = h;
   if (Psink)
      Psink->OnCameraReady(info);
   return true;
}

//--------------------------------------------------------------------------------
void TAndroid::StopCamera(void)
{
   if (Psession)
   {
      ACameraCaptureSession_stopRepeating(Psession);
      ACameraCaptureSession_close(Psession);
      Psession = NULL;
   }
   if (Prequest)
   {
      ACaptureRequest_free(Prequest);
      Prequest = NULL;
   }
   if (Ptarget)
   {
      ACameraOutputTarget_free(Ptarget);
      Ptarget = NULL;
   }
   if (Pcontainer)
   {
      if (Poutput)
         ACaptureSessionOutputContainer_remove(Pcontainer, Poutput);
      ACaptureSessionOutputContainer_free(Pcontainer);
      Pcontainer = NULL;
   }
   if (Poutput)
   {
      ACaptureSessionOutput_free(Poutput);
      Poutput = NULL;
   }
   if (Pdevice)
   {
      ACameraDevice_close(Pdevice);
      Pdevice = NULL;
   }
   if (Preader)
   {
      AImageReader_delete(Preader);
      Preader = NULL;
   }
}

//--------------------------------------------------------------------------------
void TAndroid::DeliverImage(AImageReader *reader)
{
   TCamFrame f = {};
   LONG      w = 0,
             h = 0,
             yRow = 0,
             uvRow = 0,
             uvPix = 0;
   LPBYTE    y = NULL,
             u = NULL,
             v = NULL;
   int       len = 0;

   AImage *img = NULL;

   if (AImageReader_acquireLatestImage(reader, &img) != AMEDIA_OK || !img)
      return;

   LONGLONG stamp = 0;

   AImage_getWidth(img, &w);
   AImage_getHeight(img, &h);
   AImage_getTimestamp(img, &stamp);
   AImage_getPlaneData(img, 0, &y, &len);
   AImage_getPlaneData(img, 1, &u, &len);
   AImage_getPlaneData(img, 2, &v, &len);
   AImage_getPlaneRowStride(img, 0, &yRow);
   AImage_getPlaneRowStride(img, 1, &uvRow);
   AImage_getPlanePixelStride(img, 1, &uvPix);
   f.stampNs = (QWORD)stamp;
   f.yuv.width = (int)w;
   f.yuv.height = (int)h;
   f.yuv.y = y;
   f.yuv.yStride = (int)yRow;
   f.yuv.u = u;
   f.yuv.v = v;
   f.yuv.uvRowStride = (int)uvRow;
   f.yuv.uvPixelStride = (int)uvPix;
   if (Psink && y && u && v)
      Psink->OnFrame(f);
   AImage_delete(img);
}

//--------------------------------------------------------------------------------
void TAndroid::StartSensors(void)
{
   if (Pqueue)
      return;
   PsensorMgr = ASensorManager_getInstanceForPackage(cPackage);
   if (!PsensorMgr)
      return;
   Pattitude = ASensorManager_getDefaultSensor(PsensorMgr, ASENSOR_TYPE_ROTATION_VECTOR);
   Pmagnetic = Pattitude != NULL;
   if (!Pattitude)
      Pattitude = ASensorManager_getDefaultSensor(PsensorMgr, ASENSOR_TYPE_GAME_ROTATION_VECTOR);
   if (!Pattitude)
      return;
   Pqueue = ASensorManager_createEventQueue(PsensorMgr, Papp->looper, LOOPER_ID_USER, NULL, NULL);
   ASensorEventQueue_enableSensor(Pqueue, Pattitude);
   ASensorEventQueue_setEventRate(Pqueue, Pattitude, androidSensorUs);
}

//--------------------------------------------------------------------------------
void TAndroid::StopSensors(void)
{
   if (!Pqueue)
      return;
   ASensorEventQueue_disableSensor(Pqueue, Pattitude);
   ASensorManager_destroyEventQueue(PsensorMgr, Pqueue);
   Pqueue = NULL;
}

//--------------------------------------------------------------------------------
void TAndroid::PumpSensors(void)
{
   ssize_t n;

   ASensorEvent events[16];

   if (!Pqueue)
      return;
   while ((n = ASensorEventQueue_getEvents(Pqueue, events, 16)) > 0)
      for (ssize_t i = 0; i < n; i++)
      {
         TAttitude a = {};

         a.stampNs = (QWORD)events[i].timestamp;
         a.quat[0] = events[i].data[0];
         a.quat[1] = events[i].data[1];
         a.quat[2] = events[i].data[2];
         a.quat[3] = events[i].data[3];
         a.frame = sfEastNorthUp; // game rotation vector: same axes, arbitrary yaw
         a.magnetic = Pmagnetic;
         a.accuracyDeg = Pmagnetic && events[i].data[4] >= 0.f ? events[i].data[4]*57.2957795f : NAN;
         if (Psink)
            Psink->OnAttitude(a);
      }
}

//--------------------------------------------------------------------------------
void TAndroid::StartLocation(void)
{
   if (PlocMgr)
      return;
   jniFrameBegin(16);
   PlocMgr = jniGlobal(systemService("location"));

   TJArg action = jniStr("io.aeroblox.lidar.GNSS"),
         args[4];

   args[0] = jniObjArg(jniActivity());
   args[1] = jniInt(0u, jatInt);
   args[2] = jniObjArg(jniNew("android/content/Intent", cCtor, "(Ljava/lang/String;)V", &action, 1));
   args[3] = jniInt(androidPendingFlags, jatInt);
   PlocIntent = jniGlobal(jniStaticObj("android/app/PendingIntent", "getBroadcast",
                                       "(Landroid/content/Context;ILandroid/content/Intent;I)Landroid/app/PendingIntent;",
                                       args, 4));

   // an active request keeps the GNSS engine producing fixes; they are read back by polling
   LPCSTR providers[2] = { "gps", "network" };

   for (int i = 0; i < 2 && PlocMgr && PlocIntent; i++)
   {
      TJArg req[4] = { jniStr(providers[i]), jniInt(androidLocationMs, jatLong), jniFloat(0.f), jniObjArg(PlocIntent) };

      jniCallVoid(PlocMgr, "requestLocationUpdates", "(Ljava/lang/String;JFLandroid/app/PendingIntent;)V", req, 4);
      jniFailed(); // a missing provider only means no fixes from it
   }
   jniFrameEnd();
}

//--------------------------------------------------------------------------------
void TAndroid::StopLocation(void)
{
   if (!PlocMgr)
      return;
   if (PlocIntent)
   {
      TJArg arg = jniObjArg(PlocIntent);

      jniFrameBegin(4);
      jniCallVoid(PlocMgr, "removeUpdates", "(Landroid/app/PendingIntent;)V", &arg, 1);
      jniFrameEnd();
      jniDropGlobal(PlocIntent);
      PlocIntent = NULL;
   }
   jniDropGlobal(PlocMgr);
   PlocMgr = NULL;
}

//--------------------------------------------------------------------------------
void TAndroid::pollLocation(void)
{
   if (!PlocMgr || !Psink)
      return;
   jniFrameBegin(16);

   TJArg  gps = jniStr("gps"),
          net = jniStr("network");
   LPVOID loc = jniCallObj(PlocMgr, "getLastKnownLocation", "(Ljava/lang/String;)Landroid/location/Location;", &gps, 1);

   if (!loc)
      loc = jniCallObj(PlocMgr, "getLastKnownLocation", "(Ljava/lang/String;)Landroid/location/Location;", &net, 1);
   if (loc)
   {
      QWORD fixNs = jniCallLong(loc, "getElapsedRealtimeNanos", "()J", NULL, 0);

      if (fixNs && fixNs != PlastFixNs)
      {
         TLocationRecord r = {};

         PlastFixNs = fixNs;
         r.latE7 = jniCallFixed(loc, "getLatitude", "()D", 10000000);
         r.lonE7 = jniCallFixed(loc, "getLongitude", "()D", 10000000);
         r.altMm = jniCallFixed(loc, "getAltitude", "()D", 1000);
         r.horizAccMm = (DWORD)(jniCallFloat(loc, "getAccuracy", "()F", NULL, 0)*1000.f);
         r.vertAccMm = (DWORD)(jniCallFloat(loc, "getVerticalAccuracyMeters", "()F", NULL, 0)*1000.f);
         r.fixNs = fixNs;
         Psink->OnLocation(SensorClockNs(), r);
      }
   }
   jniFailed();
   jniFrameEnd();
}

//--------------------------------------------------------------------------------
void TAndroid::KeepScreenOn(bool on)
{
   if (on)
      ANativeActivity_setWindowFlags(Papp->activity, AWINDOW_FLAG_KEEP_SCREEN_ON, 0);
   else
      ANativeActivity_setWindowFlags(Papp->activity, 0, AWINDOW_FLAG_KEEP_SCREEN_ON);
}

//--------------------------------------------------------------------------------
void TAndroid::Vibrate(int ms)
{
   TJArg arg = jniInt((QWORD)ms, jatLong);

   if (!Pvibrator)
      return;
   jniFrameBegin(4);
   jniCallVoid(Pvibrator, "vibrate", "(J)V", &arg, 1);
   jniFrameEnd();
}

//--------------------------------------------------------------------------------
void TAndroid::RequestPaint(void)
{
   {
      TMutexLock lock(Pmutex, thisInfo);

      Pdirty = true;
   }
   ALooper_wake(Papp->looper);
}

//--------------------------------------------------------------------------------
LPVOID TAndroid::paintFor(const TTextStyle &style)
{
   LPVOID paint = style.bold ? PpaintBold : Ppaint;
   TJArg  size = jniFloat((float)style.sizePx);

   jniCallVoid(paint, "setTextSize", "(F)V", &size, 1);
   return paint;
}

//--------------------------------------------------------------------------------
void TAndroid::MeasureText(LPCSTR text, const TTextStyle &style, int *outW, int *outH)
{
   jniFrameBegin(8);

   LPVOID paint = paintFor(style);
   TJArg  str = jniStr(text);
   float  w = jniCallFloat(paint, "measureText", "(Ljava/lang/String;)F", &str, 1),
          asc = jniCallFloat(paint, "ascent", "()F", NULL, 0),
          desc = jniCallFloat(paint, "descent", "()F", NULL, 0);

   jniFrameEnd();
   *outW = (int)ceilf(w);
   *outH = (int)ceilf(desc - asc);
}

//--------------------------------------------------------------------------------
void TAndroid::DrawText(TSurface &s, int x, int y, LPCSTR text, DWORD rgba, const TTextStyle &style)
{
   jniFrameBegin(16);

   LPVOID paint = paintFor(style);
   TJArg  str = jniStr(text);
   float  wF = jniCallFloat(paint, "measureText", "(Ljava/lang/String;)F", &str, 1),
          asc = -jniCallFloat(paint, "ascent", "()F", NULL, 0),
          desc = jniCallFloat(paint, "descent", "()F", NULL, 0);
   int    w = (int)ceilf(wF) + 2,
          h = (int)ceilf(asc + desc) + 2;

   if (!Pbitmap || PbitmapW < w || PbitmapH < h)
   {
      jniDropGlobal(Pbitmap);
      jniDropGlobal(Pcanvas);
      PbitmapW = w > s.width ? w : s.width;
      PbitmapH = h > androidTextRows ? h : androidTextRows;

      TJArg cfg = jniObjArg(jniStaticField("android/graphics/Bitmap$Config", "ARGB_8888",
                                           "Landroid/graphics/Bitmap$Config;")),
            dims[3] = { jniInt((QWORD)PbitmapW, jatInt), jniInt((QWORD)PbitmapH, jatInt), cfg };

      Pbitmap = jniGlobal(jniStaticObj("android/graphics/Bitmap", "createBitmap",
                                       "(IILandroid/graphics/Bitmap$Config;)Landroid/graphics/Bitmap;", dims, 3));

      TJArg bmp = jniObjArg(Pbitmap);

      Pcanvas = jniGlobal(jniNew("android/graphics/Canvas", cCtor, "(Landroid/graphics/Bitmap;)V", &bmp, 1));
   }

   TJArg clear = jniInt(0u, jatInt),
         draw[4] = { str, jniFloat(0.f), jniFloat(asc), jniObjArg(paint) };

   jniCallVoid(Pbitmap, "eraseColor", "(I)V", &clear, 1);
   jniCallVoid(Pcanvas, "drawText", "(Ljava/lang/String;FFLandroid/graphics/Paint;)V", draw, 4);

   LPBYTE px = NULL;
   int    bw = 0,
          bh = 0,
          stride = 0;
   DWORD  alpha = rgba >> 24;

   if (jniLockBitmap(Pbitmap, &px, &bw, &bh, &stride))
   {
      for (int row = 0; row < h && row < bh; row++)
      {
         int ty = y + row;

         if (ty < 0 || ty >= s.height)
            continue;

         LPCBYTE src = px + (size_t)row*stride;
         LPDWORD dst = s.pixels + (size_t)ty*s.stride;

         for (int col = 0; col < w && col < bw; col++)
         {
            int   tx = x + col;
            DWORD a = (DWORD)src[col*4 + 3]*alpha/255u;

            if (tx < 0 || tx >= s.width || !a)
               continue;

            DWORD d = dst[tx],
                  out = 0xFF000000u;

            for (int sh = 0; sh < 24; sh += 8)
               out |= ((((rgba >> sh) & 0xFFu)*a + ((d >> sh) & 0xFFu)*(255u - a))/255u) << sh;
            dst[tx] = out;
         }
      }
      jniUnlockBitmap(Pbitmap);
   }
   jniFrameEnd();
}

//--------------------------------------------------------------------------------
void TAndroid::paintNow(void)
{
   ANativeWindow_Buffer buf;

   if (!Psink || !Pvisible || !Papp->window)
      return;
   if (ANativeWindow_lock(Papp->window, &buf, NULL) != 0)
      return;

   TSurface s;

   s.pixels = (LPDWORD)buf.bits;
   s.width = buf.width;
   s.height = buf.height;
   s.stride = buf.stride;
   Psink->OnPaint(s);
   ANativeWindow_unlockAndPost(Papp->window);
}

//--------------------------------------------------------------------------------
void TAndroid::Tick(void)
{
   QWORD now = SensorClockNs();
   bool  dirty;

   if (now - PlastPollNs >= (QWORD)androidLocationMs*1000000u)
   {
      PlastPollNs = now;
      pollLocation();
   }
   {
      TMutexLock lock(Pmutex, thisInfo);

      dirty = Pdirty;
   }
   if (dirty && now - PlastPaintNs >= (QWORD)androidPaintMs*1000000u)
   {
      {
         TMutexLock lock(Pmutex, thisInfo);

         Pdirty = false;
      }
      PlastPaintNs = now;
      paintNow();
   }
}

//--------------------------------------------------------------------------------
void TAndroid::DataDir(LPSTR out, size_t cap)
{
   LPCSTR dir = Papp->activity->externalDataPath ? Papp->activity->externalDataPath : Papp->activity->internalDataPath;

   snprintf(out, cap, "%s", dir ? dir : ".");
}

//--------------------------------------------------------------------------------
void TAndroid::Device(TDeviceInfo &out)
{
   char value[PROP_VALUE_MAX];

   snprintf(out.platform, sizeof(out.platform), "android");
   value[0] = '\0';
   __system_property_get("ro.product.model", value);
   snprintf(out.model, sizeof(out.model), "%s", value);
   value[0] = '\0';
   __system_property_get("ro.build.version.release", value);
   snprintf(out.osVersion, sizeof(out.osVersion), "Android %s", value);
}

//--------------------------------------------------------------------------------
QWORD TAndroid::WallClockNs(void)
{
   return androidClock(CLOCK_REALTIME);
}

//--------------------------------------------------------------------------------
QWORD TAndroid::SensorClockNs(void)
{
   return androidClock(CLOCK_BOOTTIME);
}

//--------------------------------------------------------------------------------
void TAndroid::Log(LPCSTR msg)
{
   __android_log_print(ANDROID_LOG_INFO, cLogTag, "%s", msg);
}

//--------------------------------------------------------------------------------
