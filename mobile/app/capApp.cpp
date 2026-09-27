#include "capPort.h"
#include "capCanvas.h"
#include "capSpin.h"
#include "capFrameMeta.h"
#include "capEXIF.h"
#include "capVanish.h"
#include "capLayout.h"
#include "thread.h"
#include "alloc.h"
#include "libDiscipline.h"

/* ==================================================================================================
   capApp - the ONE capture application, common to every platform. It owns every business rule:
   permissions flow, FOV-driven spin configuration, attitude-to-pose conversion, keyframe selection,
   JPEG encoding, session and rooms, operator hints and the whole screen layout.
   ================================================================================================== */

enum {
   appRingSize      = 256,      // attitude history, ~2.5 s at 100 Hz
   appPreviewW      = 360,      // portrait preview width in pixels
   appMaxPixels     = 12600000, // camera request: largest 4:3 YUV up to ~12.6 MP
   appJPEGQuality   = 92,
   appMaxAttitudeMs = 40,       // a frame without an attitude this close is not kept
   appMaxCameras    = 8,
   appBlurPx        = 4,        // accepted motion smear of a keyframe at the assumed exposure
   appExposureHz    = 30        // exposure cap requested from the camera (1/30 s)
};

static const float cFocusDiopters = 0.5f, // fixed focus at 2 m for the whole spin: sharp ~1.3-4 m, constant intrinsics
                   cAxisTolDeg = 2.f,     // a keyframe whose room axes drift more than this from the room reference is flagged
                   cCeilingM = 2.8f;       // assumed ceiling height until door heads (2.10 m) measure it

// Where the operator is in the room protocol: two center spins, then each corner aiming at the opposite one
enum TRoomPhase {
   rpNone,   // no room open
   rpCenter, // in-place spin from the middle (both pitch bands)
   rpWalk,   // walking to the next corner, waiting for "arrived"
   rpCorner  // standing in a corner, sweeping the fan around the opposite one
};

class TCapApp;

// Encodes the pending keyframe off the camera thread
class TKeyframeWorker : public TCormWorker
{
 public:
   explicit TKeyframeWorker(TCapApp &app) : TCormWorker("keyframe"), Papp(app) {}

 protected:
   void DoJob(void) override;

 private:
   TCapApp &Papp;
};

// A tap target laid out by the last paint
struct TButton {
   int  x, y, w, h;
   bool visible;
};

class TCapApp : public TCapSink
{
 public:
   explicit TCapApp(TCapPort *port);

   ~TCapApp(void);

   void Launch(void);
   void EncodePending(void); // worker thread

   void OnResize(int w, int h, int densityQ8) override;
   void OnPaint(TSurface &s) override;
   void OnTap(int x, int y) override;
   bool OnBack(void) override;
   void OnPause(void) override;
   void OnResume(void) override;
   void OnPermissions(bool camera, bool location) override;
   void OnCameraReady(const TCamInfo &info) override;
   void OnFrame(const TCamFrame &frame) override;
   void OnAttitude(const TAttitude &a) override;
   void OnLocation(QWORD stampNs, const TLocationRecord &loc) override;

   TCapApp(const TCapApp &) = delete;
   TCapApp &operator=(const TCapApp &) = delete;

 private:
   int   scaleDp(int v) const { return (v*Pdensity)/256; }
   bool  startBestCamera(void);
   TMat4 poseOf(const TAttitude &a) const;
   bool  attitudeAt(QWORD stampNs, TAttitude &out) const;
   void  updatePreview(const TCamFrame &f);
   void  queueKeyframe(const TCamFrame &f, const TMat4 &pose, const TAttitude &a);
   void  startRoom(void);
   void  finishRoom(void);
   void  beginStation(TStationKind kind);
   void  endStation(void);
   void  stationDone(void);
   int   nextCorner(void) const;
   int   deduceCorner(float headingDeg) const;
   DWORD binColor(int band, int bin) const;
   void  drawPlan(TSurface &s, int x, int y, int size);
   void  solvePlan(void);
   void  finishProperty(void);
   bool  openSession(void);
   void  drawLabel(TSurface &s, int x, int y, LPCSTR text, DWORD rgba, int sizeDp, bool bold, bool centered);
   void  drawButton(TSurface &s, TButton &b, LPCSTR text, DWORD rgba);
   void  drawCoverage(TSurface &s, int cx, int cy, int radius);
   void  hintText(LPSTR out, size_t cap) const;

   TCapPort            *Pport;
   mutable TMutex       Pmutex;
   TSessionWriter       Psession;
   TJPEGEncoder         Pencoder;
   TKeyframeWorker      Pworker;
   TBlock<TSpinTracker> Pspin;
   TBlock<DWORD>        Ppreview;
   TBlock<BYTE>         PjobPlanes;
   TByteBuf             Pjpeg,
                        Pexif,
                        Pfinal,
                        PmetaBuf;
   TFrameMeta           PjobMeta;
   TAttitude            Pring[appRingSize];
   TCamInfo             Pcam;
   TIntrinsics          Pintr;
   TLocationRecord      Ploc;
   TImageRecord         Pjob;
   TSpinVerdict         Pverdict;
   TSpinConfig          PcenterCfg,
                        PcornerCfg;
   TVanishConfig        PvanishCfg;
   TTiltBias            PtiltBias;   // camera tilt calibrated from the measured verticals (worker thread only)
   TAxisCheck           PaxisCheck;
   TAxisVerdict         PaxisVerdict;
   TRoomLayout          Playout;
   TLayoutPlan          Pplan;
   TBlock<float>        PedgeRays;
   TBlock<BYTE>         PedgeLabels;
   TVanishEdges         Pedges;
   int                  PcornerStep,  // corner stations already captured
                        PcornerCount, // planned corner stations (4 without a plan)
                        PcornerSlot;  // the station at hand: suggested while walking, then deduced from the aimed diagonal
   DWORD                PcornerMask;  // stations captured, one bit per plan station
   float                PbinAxis[spinMaxBands][spinMaxBins]; // room axis measured on the keyframe of each bin (NaN: no lines)
   bool                 PbinDone[spinMaxBands][spinMaxBins], // the keyframe of the bin went through the vanishing check
                        PplanWanted,
                        PplanSketch;  // Pplan is only a square on the room axes (no plan could be made): no dimensions
   TRoomPhase           Pphase;
   TStationRecord       Pstation;
   TButton              PbtnMain,
                        PbtnFinish;
   QWORD                PjobStampNs;
   DWORD                PkeySeq,
                        PstationCount,
                        ProomIndex;
   char                 ProomName[roomNameMax],
                        PdevModel[64],
                        Pstatus[96];
   int                  PviewW,
                        PviewH,
                        Pdensity,
                        PringHead,
                        PringCount,
                        PpreviewH,
                        Prooms;
   float                PhfovDeg,
                        PvfovDeg;
   bool                 PcamReady,
                        PlocAllowed,
                        PhasLoc,
                        PhasPreview,
                        ProomOpen,
                        PjobBusy,
                        PcompleteSignaled,
                        Pmagnetic,
                        PsensorFresh;
};

static TCapApp *gApp = NULL;

//--------------------------------------------------------------------------------
void TKeyframeWorker::DoJob(void)
{
   Papp.EncodePending();
}

//--------------------------------------------------------------------------------
TCapApp::TCapApp(TCapPort *port) : Pport(port), Pencoder(appJPEGQuality), Pworker(*this), PjobMeta(), Pring(),
   Pcam(), Pintr(), Ploc(), Pjob(), Pverdict(svCovered), PcenterCfg(), PcornerCfg(),
   PvanishCfg(TVanishConfig::Default()), PaxisCheck(cAxisTolDeg), PaxisVerdict(avNoLines), Pplan(), Pedges(),
   PcornerStep(0), PcornerCount(4), PcornerSlot(0), PcornerMask(0u), PbinAxis(), PbinDone(), PplanWanted(false), PplanSketch(false), Pphase(rpNone), Pstation(), PbtnMain(), PbtnFinish(), PjobStampNs(0u), PkeySeq(0u),
   PstationCount(0u), ProomIndex(0u), PviewW(0), PviewH(0), Pdensity(256), PringHead(0), PringCount(0), PpreviewH(0), Prooms(0), PhfovDeg(60.f),
   PvfovDeg(60.f), PcamReady(false), PlocAllowed(false), PhasLoc(false), PhasPreview(false),
   ProomOpen(false), PjobBusy(false), PcompleteSignaled(false), Pmagnetic(false), PsensorFresh(true)
{
   ProomName[0] = '\0';
   PdevModel[0] = '\0';
   snprintf(Pstatus, sizeof(Pstatus), "Iniciando...");
}

//--------------------------------------------------------------------------------
TCapApp::~TCapApp(void)
{
   Pworker.Terminate();
   Pworker.Join();
   Psession.Close(Pport->WallClockNs());
}

//--------------------------------------------------------------------------------
void TCapApp::Launch(void)
{
   Pworker.Start(thisInfo);
   Pport->SetSink(this);
   Pport->KeepScreenOn(true); // permissions are (re)checked on every OnResume
}

//--------------------------------------------------------------------------------
void TCapApp::OnPermissions(bool camera, bool location)
{
   {
      TMutexLock lock(Pmutex, thisInfo);

      PlocAllowed = location;
      snprintf(Pstatus, sizeof(Pstatus), camera ? "Abrindo a câmera..." : "Permita o uso da câmera");
   }
   if (camera && !startBestCamera())
   {
      TMutexLock lock(Pmutex, thisInfo);

      snprintf(Pstatus, sizeof(Pstatus), "Câmera indisponível");
   }
   Pport->StartSensors();
   if (location)
      Pport->StartLocation();
   Pport->RequestPaint();
}

/*--------------------------------------------------------------------------------
   The widest back lens among the full-resolution ones (at least half the pixels of the best
   sensor): a 2 MP macro or depth helper never wins just for being wide.
  --------------------------------------------------------------------------------*/
bool TCapApp::startBestCamera(void)
{
   TCamInfo cams[appMaxCameras];
   int      n = Pport->ListCameras(cams, appMaxCameras),
            best = -1;
   QWORD    maxPixels = 0u;
   float    bestFov = 0.f;

   for (int i = 0; i < n; i++)
   {
      QWORD px = (QWORD)cams[i].width*(QWORD)cams[i].height;

      if (px > maxPixels)
         maxPixels = px;
   }
   for (int i = 0; i < n; i++)
   {
      QWORD px = (QWORD)cams[i].width*(QWORD)cams[i].height;
      float fov = cams[i].focalMm > 0.f ? 2.f*atan2f(0.5f*cams[i].sensorWidthMm, cams[i].focalMm) : 0.f;

      if (2u*px >= maxPixels && fov > bestFov)
      {
         bestFov = fov;
         best = i;
      }
   }
   return best >= 0 && Pport->StartCamera(best, appMaxPixels, cFocusDiopters, appExposureHz);
}

//--------------------------------------------------------------------------------
void TCapApp::OnCameraReady(const TCamInfo &info)
{
   TMutexLock lock(Pmutex, thisInfo);
   float      scale = info.arrayWidth > 0 ? (float)info.width/(float)info.arrayWidth : 1.f;

   Pcam = info;
   if (info.calib[0] > 0.f)
   {
      Pintr.fx = info.calib[0]*scale;
      Pintr.fy = info.calib[1]*scale;
   }
   else
   {
      Pintr.fx = info.focalMm/info.sensorWidthMm*(float)info.width;
      Pintr.fy = Pintr.fx;
   }
   Pintr.cx = info.calib[2] > 0.f ? info.calib[2]*scale : 0.5f*(float)info.width;
   Pintr.cy = info.calib[3] > 0.f ? info.calib[3]*scale : 0.5f*(float)info.height;

   float nativeH = 2.f*atan2f(0.5f*(float)info.width, Pintr.fx)*57.2957795f,
         nativeV = 2.f*atan2f(0.5f*(float)info.height, Pintr.fy)*57.2957795f;
   bool  portraitSwap = info.sensorRotDeg == 90 || info.sensorRotDeg == 270;

   PhfovDeg = portraitSwap ? nativeV : nativeH;
   PvfovDeg = portraitSwap ? nativeH : nativeV;

   float pxPerDeg = Pintr.fx*0.01745329f,
         maxRate = (float)appBlurPx*(float)appExposureHz/pxPerDeg; // stop-and-shoot: smear <= appBlurPx

   PcenterCfg = TSpinConfig::ForFov(PhfovDeg, PvfovDeg);
   PcenterCfg.maxRateDps = maxRate;
   PcornerCfg = TSpinConfig::ForCorner(PhfovDeg, PvfovDeg);
   PcornerCfg.maxRateDps = maxRate;

   TSpinTracker *spin = new TSpinTracker(PcenterCfg);

   Pspin = spin;

   int portraitW = portraitSwap ? info.height : info.width,
       portraitH = portraitSwap ? info.width : info.height;

   PpreviewH = appPreviewW*portraitH/portraitW;
   {
      TAlloc<DWORD> preview((size_t)appPreviewW*PpreviewH);
      TAlloc<BYTE>  planes((size_t)info.width*info.height*3u/2u + 16u);

      preview.Drop(Ppreview);
      planes.Drop(PjobPlanes);
   }
   if (!PedgeRays())
   {
      // the classified edges of each keyframe: without them the floor plan gets no lines at all
      TAlloc<float> rays((size_t)PvanishCfg.maxEdges*3u);
      TAlloc<BYTE>  labels((size_t)PvanishCfg.maxEdges);

      rays.Drop(PedgeRays);
      labels.Drop(PedgeLabels);
      Pedges.ray = PedgeRays();
      Pedges.label = PedgeLabels();
      Pedges.capacity = (DWORD)PvanishCfg.maxEdges;
      Pedges.count = 0u;
   }
   PhasPreview = false;
   PcamReady = true;
   snprintf(Pstatus, sizeof(Pstatus), "Pronto. Toque em Iniciar cômodo");
}

//--------------------------------------------------------------------------------
TMat4 TCapApp::poseOf(const TAttitude &a) const
{
   return orientCameraToWorld(a.quat, a.frame, PcamReady ? Pcam.sensorRotDeg : 90);
}

//--------------------------------------------------------------------------------
bool TCapApp::attitudeAt(QWORD stampNs, TAttitude &out) const
{
   QWORD bestGap = ~(QWORD)0;
   bool  found = false;

   for (int i = 0; i < PringCount; i++)
   {
      const TAttitude &a = Pring[i];
      QWORD            gap = a.stampNs > stampNs ? a.stampNs - stampNs : stampNs - a.stampNs;

      if (gap < bestGap)
      {
         bestGap = gap;
         out = a;
         found = true;
      }
   }
   return found && bestGap <= (QWORD)appMaxAttitudeMs*1000000u;
}

//--------------------------------------------------------------------------------
void TCapApp::OnAttitude(const TAttitude &a)
{
   TMutexLock lock(Pmutex, thisInfo);

   Pring[PringHead] = a;
   PringHead = (PringHead + 1)%appRingSize;
   if (PringCount < appRingSize)
      PringCount++;
   Pmagnetic = a.magnetic;
   if (!ProomOpen)
      return;

   TPoseRecord pose;

   pose.cameraToWorld = poseOf(a);
   pose.compass.magneticDeg = a.magnetic ? geomHeadingDeg(pose.cameraToWorld.Forward()) : NAN;
   pose.compass.trueDeg = NAN;
   pose.compass.accuracyDeg = a.accuracyDeg;
   pose.tracking = PsensorFresh ? tsNone : tsNormal; // tsNone marks a yaw-reference restart
   PsensorFresh = false;
   Psession.WritePose(a.stampNs, pose);
}

//--------------------------------------------------------------------------------
void TCapApp::OnLocation(QWORD stampNs, const TLocationRecord &loc)
{
   TMutexLock lock(Pmutex, thisInfo);

   Ploc = loc;
   PhasLoc = true;
   if (Psession.IsOpen())
      Psession.WriteLocation(stampNs, loc);
}

//--------------------------------------------------------------------------------
// Downsampled, rotated-upright RGB preview of the frame (BT.601 full range)
void TCapApp::updatePreview(const TCamFrame &f)
{
   const TYUVImage &y = f.yuv;
   int              rot = Pcam.sensorRotDeg,
                    nw = y.width,
                    nh = y.height,
                    pw = (rot == 90 || rot == 270) ? nh : nw;
   LPDWORD          dst = Ppreview();

   if (!dst)
      return;
   for (int pv = 0; pv < PpreviewH; pv++)
      for (int pu = 0; pu < appPreviewW; pu++)
      {
         int u = pu*pw/appPreviewW,
             v = pv*pw/appPreviewW,
             x = u,
             yy = v;

         if (rot == 90)
         {
            x = v;
            yy = nh - 1 - u;
         }
         else if (rot == 270)
         {
            x = nw - 1 - v;
            yy = u;
         }
         else if (rot == 180)
         {
            x = nw - 1 - u;
            yy = nh - 1 - v;
         }
         if (x < 0 || x >= nw || yy < 0 || yy >= nh)
            continue;

         int luma = y.y[(size_t)yy*y.yStride + x],
             ci = (yy/2)*y.uvRowStride + (x/2)*y.uvPixelStride,
             d = (int)y.u[ci] - 128,
             e = (int)y.v[ci] - 128,
             r = luma + ((359*e) >> 8),
             g = luma - ((88*d + 183*e) >> 8),
             b = luma + ((454*d) >> 8);

         r = r < 0 ? 0 : (r > 255 ? 255 : r);
         g = g < 0 ? 0 : (g > 255 ? 255 : g);
         b = b < 0 ? 0 : (b > 255 ? 255 : b);
         dst[(size_t)pv*appPreviewW + pu] = canvasRGBA(r, g, b, 255);
      }
   PhasPreview = true;
}

//--------------------------------------------------------------------------------
// Copies the frame planes for the worker (planar Y, U, V) and posts the encode
void TCapApp::queueKeyframe(const TCamFrame &f, const TMat4 &pose, const TAttitude &a)
{
   const TYUVImage &src = f.yuv;
   int              cw = (src.width + 1)/2,
                    ch = (src.height + 1)/2;
   LPBYTE           yDst = PjobPlanes(),
                    uDst = yDst + (size_t)src.width*src.height,
                    vDst = uDst + (size_t)cw*ch;

   for (int row = 0; row < src.height; row++)
      memcpy(yDst + (size_t)row*src.width, src.y + (size_t)row*src.yStride, (size_t)src.width);
   for (int row = 0; row < ch; row++)
      for (int col = 0; col < cw; col++)
      {
         size_t si = (size_t)row*src.uvRowStride + (size_t)col*src.uvPixelStride;

         uDst[(size_t)row*cw + col] = src.u[si];
         vDst[(size_t)row*cw + col] = src.v[si];
      }

   Pjob = TImageRecord();
   Pjob.width = (DWORD)src.width;
   Pjob.height = (DWORD)src.height;
   Pjob.format = pfJPEG;
   Pjob.intr = Pintr;
   for (int i = 0; i < 5; i++)
      Pjob.distortion[i] = Pcam.distortion[i];
   Pjob.cameraToWorld = pose;
   Pjob.compass.magneticDeg = a.magnetic ? geomHeadingDeg(pose.Forward()) : NAN;
   Pjob.compass.trueDeg = NAN;
   Pjob.compass.accuracyDeg = a.accuracyDeg;
   PjobMeta = TFrameMeta();
   Psession.SessionId(PjobMeta.sessionId);
   PjobMeta.roomIndex = ProomIndex;
   PjobMeta.seq = PkeySeq++;
   PjobMeta.sensorNs = f.stampNs;
   PjobMeta.wallNs = Pport->WallClockNs();
   PjobMeta.cameraToWorld = pose;
   PjobMeta.compass = Pjob.compass;
   PjobMeta.headingRef = a.magnetic ? hrMagnetic : hrArbitrary;
   PjobMeta.intr = Pintr;
   for (int i = 0; i < 5; i++)
      PjobMeta.distortion[i] = Pcam.distortion[i];
   PjobMeta.headingDeg = Pspin->LastHeadingDeg();
   PjobMeta.pitchDeg = Pspin->LastPitchDeg();
   PjobMeta.spinBand = (BYTE)Pspin->LastKeptBand();
   PjobMeta.spinBin = (BYTE)Pspin->LastKeptBin();
   PjobMeta.stationIndex = (BYTE)Pstation.index;
   PjobMeta.stationKind = (BYTE)Pstation.kind;
   PjobMeta.cornerIndex = Pstation.kind == skCorner ? Pstation.corner : 0u;
   PjobMeta.targetCorner = Pstation.kind == skCorner ? Pstation.target : 0u;
   PjobMeta.focalMm = Pcam.focalMm;
   if (PhasLoc)
   {
      PjobMeta.latE7 = Ploc.latE7;
      PjobMeta.lonE7 = Ploc.lonE7;
      PjobMeta.altMm = Ploc.altMm;
      PjobMeta.horizAccMm = Ploc.horizAccMm;
      PjobMeta.fixAgeNs = f.stampNs > Ploc.fixNs ? f.stampNs - Ploc.fixNs : 0u;
   }
   PjobStampNs = f.stampNs;
   PjobBusy = true;
   Pworker.Post();
}

//--------------------------------------------------------------------------------
void TCapApp::EncodePending(void)
{
   TYUVImage     img;
   TImageRecord  rec;
   TFrameMeta    meta;
   TVanishResult vr;
   TVanishRecord vrec = {};
   TVanishConfig tuned = PvanishCfg;
   QWORD         stamp;
   bool          measured;

   {
      TMutexLock lock(Pmutex, thisInfo);

      if (!PjobBusy)
         return;

      int cw = ((int)Pjob.width + 1)/2,
          ch = ((int)Pjob.height + 1)/2;

      img.width = (int)Pjob.width;
      img.height = (int)Pjob.height;
      img.y = PjobPlanes();
      img.yStride = img.width;
      img.u = img.y + (size_t)img.width*img.height;
      img.v = img.u + (size_t)cw*ch;
      img.uvRowStride = cw;
      img.uvPixelStride = 1;
      rec = Pjob;
      meta = PjobMeta;
      stamp = PjobStampNs;

      TEXIFInfo exif = {};

      snprintf(exif.model, sizeof(exif.model), "%s", PdevModel);
      snprintf(exif.software, sizeof(exif.software), "Aeroblox LiDAR 0.1.0");
      exif.wallNs = PjobMeta.wallNs;
      exif.orientation = exifOrientation(PjobMeta.cameraToWorld);
      exif.focalMm = PjobMeta.focalMm;
      exif.width = Pjob.width;
      exif.height = Pjob.height;
      exif.hasGPS = PjobMeta.horizAccMm != 0u;
      exif.latE7 = PjobMeta.latE7;
      exif.lonE7 = PjobMeta.lonE7;
      exif.altMm = PjobMeta.altMm;
      Pexif.Clear();
      exifBuild(exif, Pexif);
   }

   // the planes stay put while the job is busy: detect without holding the camera thread
   PtiltBias.Apply(tuned);
   measured = vanishDetect(img.y, img.width, img.height, img.yStride, rec.intr, rec.cameraToWorld, tuned, vr,
                           &Pedges);
   if (measured)
      PtiltBias.Add(vr);
   {
      TMutexLock lock(Pmutex, thisInfo);
      float      dev = NAN;
      bool       sameRoom = ProomOpen && ProomIndex == meta.roomIndex;

      PaxisVerdict = measured && sameRoom ? PaxisCheck.Offer(vr, &dev) : avNoLines;
      meta.vanishFlags = measured ? vr.flags : 0u;
      meta.axisVerdict = (BYTE)PaxisVerdict;
      meta.roomAxisDeg = vr.roomAxisDeg;
      meta.axisDevDeg = dev;
      meta.tiltErrCdeg = isnan(vr.tiltErrDeg) ? 0xFFFFu : (WORD)(vr.tiltErrDeg*100.f + 0.5f);
      meta.orthoErrCdeg = isnan(vr.orthoErrDeg) ? 0xFFFFu : (WORD)(vr.orthoErrDeg*100.f + 0.5f);
      PmetaBuf.Clear();
      meta.Encode(PmetaBuf);
      vrec.roomIndex = meta.roomIndex;
      vrec.seq = meta.seq;
      vrec.flags = meta.vanishFlags;
      vrec.verdict = meta.axisVerdict;
      vrec.roomAxisDeg = vr.roomAxisDeg;
      vrec.deviationDeg = dev;
      vrec.tiltErrDeg = vr.tiltErrDeg;
      vrec.orthoErrDeg = vr.orthoErrDeg;
      for (int i = 0; i < vanishDirs; i++)
      {
         vrec.dirCam[i] = vr.dirCam[i];
         vrec.support[i] = vr.support[i];
      }
      vrec.edges = vr.edges;
      if (sameRoom && meta.stationIndex == (BYTE)Pstation.index && meta.spinBand < spinMaxBands && meta.spinBin < spinMaxBins)
      {
         PbinAxis[meta.spinBand][meta.spinBin] = measured ? vr.roomAxisDeg : NAN;
         PbinDone[meta.spinBand][meta.spinBin] = true;
      }
      if (measured && sameRoom) // the floor plan: center spin for the walls, every station for the camera height
         Playout.AddFrame(rec.cameraToWorld, vr, Pedges, PtiltBias.Bias(), meta.stationKind == (BYTE)skCenter);
   }
   Psession.WriteVanish(stamp, vrec);

   Pjpeg.Clear();
   Pfinal.Clear();
   if (Pencoder.Encode(img, Pjpeg, PmetaBuf.Data(), PmetaBuf.Size())
       && jpegInsertSegment(Pjpeg.Data(), Pjpeg.Size(), 0xE1, Pexif.Data(), Pexif.Size(), Pfinal))
   {
      rec.pixels = Pfinal.Data();
      rec.pixelBytes = (DWORD)Pfinal.Size();
      Psession.WriteImage(stamp, rec);
   }
   {
      TMutexLock lock(Pmutex, thisInfo);

      PjobBusy = false;
      if (PplanWanted)
         solvePlan(); // the keyframe that closed the center spin is in: the plan can be made
   }
   Pport->RequestPaint();
}

//--------------------------------------------------------------------------------
void TCapApp::OnFrame(const TCamFrame &frame)
{
   TMutexLock lock(Pmutex, thisInfo);
   TAttitude  a;

   if (!PcamReady || frame.yuv.width != Pcam.width || frame.yuv.height != Pcam.height)
      return;
   updatePreview(frame);
   if ((Pphase == rpCenter || Pphase == rpCorner) && Pspin && attitudeAt(frame.stampNs, a))
   {
      TMat4 pose = poseOf(a);

      if (Pphase == rpCorner && !Pspin->Filled()) // until the fan is aimed, the diagonal names the station
      {
         int slot = deduceCorner(geomHeadingDeg(pose.Forward()));

         if (slot != PcornerSlot)
         {
            PcornerSlot = slot;
            Pstation.corner = Pplan.valid ? Pplan.stationVertex[slot] : (BYTE)slot;
            Pstation.target = Pplan.valid ? Pplan.targetVertex[slot] : (BYTE)((slot + 2)%4);
         }
      }
      Pverdict = Pspin->Offer(frame.stampNs, pose, a.accuracyDeg, !PjobBusy); // busy: observe only, never mark a bin we cannot save
      if (Pverdict == svKeep)
      {
         queueKeyframe(frame, pose, a);
         Pport->Vibrate(15);
      }
      if (Pspin->Complete() && !PcompleteSignaled && Pverdict == svKeep)
      {
         PcompleteSignaled = true;
         Pport->Vibrate(250);
         stationDone(); // automatic: center spins -> corner 1 -> ... -> corner 4 closes the room
      }
   }
   Pport->RequestPaint();
}

//--------------------------------------------------------------------------------
bool TCapApp::openSession(void)
{
   TDeviceInfo dev = {};
   char        root[sessionPathMax - 64],
               dir[sessionPathMax];
   time_t      t = (time_t)(Pport->WallClockNs()/1000000000u);
   struct tm  *lt = localtime(&t);

   Pport->Device(dev);
   snprintf(dev.appVersion, sizeof(dev.appVersion), "0.1.0");
   snprintf(PdevModel, sizeof(PdevModel), "%s", dev.model);
   snprintf(dev.camera, sizeof(dev.camera), "%s", Pcam.label);
   snprintf(dev.depthSource, sizeof(dev.depthSource), "none");
   snprintf(dev.headingRef, sizeof(dev.headingRef), Pmagnetic ? "magnetic" : "arbitrary");
   Pport->DataDir(root, sizeof(root));
   snprintf(dir, sizeof(dir), "%s/imovel_%04d%02d%02d_%02d%02d%02d", root, lt->tm_year + 1900, lt->tm_mon + 1,
            lt->tm_mday, lt->tm_hour, lt->tm_min, lt->tm_sec);
   if (!Psession.Open(dir, dev, Pport->WallClockNs()))
      return false;
   Prooms = 0;
   if (PhasLoc)
      Psession.WriteLocation(Pport->SensorClockNs(), Ploc);
   return true;
}

//--------------------------------------------------------------------------------
void TCapApp::startRoom(void)
{
   if (!PcamReady || !Pspin)
      return;
   if (!Psession.IsOpen() && !openSession())
   {
      snprintf(Pstatus, sizeof(Pstatus), "Não foi possível criar a sessão");
      return;
   }
   Prooms++;
   snprintf(ProomName, sizeof(ProomName), "Cômodo %d", Prooms);
   ProomIndex = Psession.BeginRoom(Pport->SensorClockNs(), ProomName);
   PstationCount = 0u;
   ProomOpen = true;
   PaxisCheck.Reset();
   PaxisVerdict = avNoLines;
   Playout.Reset();
   Pplan = TLayoutPlan();
   PplanSketch = false;
   PplanWanted = false;
   PcornerStep = 0;
   PcornerCount = 4;
   PcornerSlot = 0;
   PcornerMask = 0u;
   Pstation.corner = 0u;
   beginStation(skCenter);
   Pport->Vibrate(40);
}

//--------------------------------------------------------------------------------
// Opens a station and arms a fresh tracker for it (center spin or corner fan)
void TCapApp::beginStation(TStationKind kind)
{
   TSpinTracker *spin = new TSpinTracker(kind == skCenter ? PcenterCfg : PcornerCfg);

   Pspin = spin;
   Pstation.event = seBegin;
   Pstation.roomIndex = ProomIndex;
   Pstation.index = PstationCount++;
   Pstation.kind = kind;
   if (kind == skCorner && Pplan.valid)
   {
      Pstation.corner = Pplan.stationVertex[PcornerSlot]; // plan corners: where to stand and what to aim at
      Pstation.target = Pplan.targetVertex[PcornerSlot];
   }
   else
   {
      Pstation.corner = (BYTE)PcornerSlot;
      Pstation.target = (BYTE)((PcornerSlot + 2)%4);
   }
   Psession.WriteStation(Pport->SensorClockNs(), Pstation);
   Pphase = kind == skCenter ? rpCenter : rpCorner;
   memset(PbinDone, 0, sizeof(PbinDone));
   PcompleteSignaled = false;
   Pverdict = svCovered;
}

//--------------------------------------------------------------------------------
void TCapApp::endStation(void)
{
   if (Pphase != rpCenter && Pphase != rpCorner)
      return;
   Pstation.event = seEnd;
   Psession.WriteStation(Pport->SensorClockNs(), Pstation);
}

/*--------------------------------------------------------------------------------
   Station finished (complete or skipped): after the center spin the floor plan is solved and the
   operator walks to its first corner station (4 diagonal corners without a plan); after each
   corner to the next one clockwise; the last planned corner closes the room.
  --------------------------------------------------------------------------------*/
void TCapApp::stationDone(void)
{
   bool wasCenter = Pphase == rpCenter;

   endStation();
   if (wasCenter)
   {
      PplanWanted = true;
      if (!PjobBusy)
         solvePlan(); // otherwise the worker solves once the closing keyframe is in
   }
   if (!wasCenter && PcornerStep + 1 >= PcornerCount)
   {
      Pphase = rpWalk; // the station is already closed: finishRoom must not close it again
      finishRoom();
      return;
   }
   if (!wasCenter)
   {
      PcornerMask |= 1u << PcornerSlot;
      PcornerStep++;
   }
   PcornerSlot = nextCorner();
   Pphase = rpWalk;
}

//--------------------------------------------------------------------------------
// First station still to capture, in walking order: the one suggested on the map
int TCapApp::nextCorner(void) const
{
   for (int i = 0; i < PcornerCount; i++)
      if (!(PcornerMask & (1u << i)))
         return i;
   return 0;
}

/*--------------------------------------------------------------------------------
   Which station the operator actually chose, from where the phone aims: from a corner one looks
   along its diagonal, so the station whose aim (corner -> target on the plan, turned to the gyro
   heading) is closest to the heading wins among those still to capture. In a rectangle any corner
   can be the first; in an L (five 90-degree corners plus the one that juts in) two aims can be
   near each other, so the suggested station keeps a 15-degree edge. Without a plan the four
   diagonals of the room axes (axis + 45 + 90k) name corner k.
  --------------------------------------------------------------------------------*/
int TCapApp::deduceCorner(float headingDeg) const
{
   int   best = PcornerSlot;
   float bestScore = 1e9f;

   for (int i = 0; i < PcornerCount; i++)
   {
      if (PcornerMask & (1u << i))
         continue;

      float aimDeg;

      if (Pplan.valid)
      {
         const TPlanPoint &c = Pplan.verts[Pplan.stationVertex[i]],
                          &t = Pplan.verts[Pplan.targetVertex[i]];

         aimDeg = Pplan.axisDeg + atan2f(t.w - c.w, t.u - c.u)*57.29578f;
      }
      else if (PaxisCheck.HasReference())
         aimDeg = PaxisCheck.ReferenceDeg() + 45.f + 90.f*(float)i;
      else
         return PcornerSlot;

      float score = fabsf(geomHeadingDiffDeg(aimDeg, headingDeg)) - (i == PcornerSlot ? 15.f : 0.f);

      if (score < bestScore)
      {
         bestScore = score;
         best = i;
      }
   }
   return best;
}

//--------------------------------------------------------------------------------
void TCapApp::finishRoom(void)
{
   if (!ProomOpen)
      return;
   endStation();
   Psession.EndRoom(Pport->SensorClockNs(), Pplan.valid ? Pplan.cameraHeightM : NAN);
   ProomOpen = false;
   Pphase = rpNone;
   snprintf(Pstatus, sizeof(Pstatus), "%s salvo (%lu estações)", ProomName, (unsigned long)PstationCount);
   Pport->Vibrate(80);
}

/*--------------------------------------------------------------------------------
   Floor plan of the room once its center spin is in (lock held): the corner stations follow it
   (4 diagonal corners when no plan could be made), and it is recorded in the room's own frame
   for the property plan the operator will arrange later.
  --------------------------------------------------------------------------------*/
void TCapApp::solvePlan(void)
{
   TLayoutRecord rec = {};

   PplanWanted = false;
   if (!Playout.Solve(Playout.AnchorDeg(), cCeilingM, Pplan))
      Pplan.valid = false;
   PcornerCount = Pplan.valid ? Pplan.stationCount : 4;
   PcornerSlot = nextCorner();
   rec.roomIndex = ProomIndex;
   rec.flags = (BYTE)((Pplan.valid ? lfValid : 0) | (Pplan.complete ? lfComplete : 0)
                      | (Pplan.heightSolved ? lfHeightSolved : 0));
   rec.vertexCount = (BYTE)Pplan.vertexCount;
   rec.stationCount = (BYTE)Pplan.stationCount;
   rec.axisDeg = Pplan.axisDeg;
   rec.ceilingM = Pplan.ceilingM;
   rec.assumedCeilingM = Pplan.assumedCeilingM;
   rec.cameraHeightM = Pplan.cameraHeightM;
   rec.doorScale = Pplan.doorScale;
   for (int i = 0; i < Pplan.vertexCount && i < layoutRecMaxVerts; i++)
   {
      rec.u[i] = Pplan.verts[i].u;
      rec.w[i] = Pplan.verts[i].w;
   }
   for (int i = 0; i < Pplan.stationCount && i < layoutRecMaxStations; i++)
   {
      rec.stationVertex[i] = Pplan.stationVertex[i];
      rec.targetVertex[i] = Pplan.targetVertex[i];
   }
   Psession.WriteLayout(Pport->SensorClockNs(), rec);
   Pport->Vibrate(Pplan.valid ? 60 : 20);
   PplanSketch = !Pplan.valid && PaxisCheck.HasReference();
   if (PplanSketch)
   {
      /* no plan: the room axes are still known, so the map shows a square on them - the operator still sees
         where to stand and where to aim (four diagonal corners, clockwise from above) */
      static const float cu[4] = { 1.5f, 1.5f, -1.5f, -1.5f },
                         cw[4] = { -1.5f, 1.5f, 1.5f, -1.5f };

      Pplan.axisDeg = PaxisCheck.ReferenceDeg();
      Pplan.vertexCount = 4;
      Pplan.stationCount = 4;
      for (int i = 0; i < 4; i++)
      {
         Pplan.verts[i].u = cu[i];
         Pplan.verts[i].w = cw[i];
         Pplan.convex[i] = 1u;
         Pplan.stationVertex[i] = (BYTE)i;
         Pplan.targetVertex[i] = (BYTE)((i + 2)%4);
      }
      Pplan.valid = true;
   }
}

//--------------------------------------------------------------------------------
void TCapApp::finishProperty(void)
{
   TSessionCounts c = Psession.Counts();

   finishRoom();
   Psession.Close(Pport->WallClockNs());
   snprintf(Pstatus, sizeof(Pstatus), "Imóvel salvo: %lu cômodos, %llu imagens", (unsigned long)c.rooms,
            (unsigned long long)c.images);
   Prooms = 0;
}

//--------------------------------------------------------------------------------
void TCapApp::OnTap(int x, int y)
{
   TMutexLock lock(Pmutex, thisInfo);

   if (PbtnMain.visible && x >= PbtnMain.x && x < PbtnMain.x + PbtnMain.w && y >= PbtnMain.y
       && y < PbtnMain.y + PbtnMain.h)
   {
      if (Pphase == rpNone)
         startRoom();
      else if (Pphase == rpWalk)
         beginStation(skCorner); // "arrived at corner N"
      else
         stationDone();          // skip the rest of this station
   }
   else if (PbtnFinish.visible && x >= PbtnFinish.x && x < PbtnFinish.x + PbtnFinish.w && y >= PbtnFinish.y
            && y < PbtnFinish.y + PbtnFinish.h)
   {
      if (ProomOpen)
         finishRoom();
      else
         finishProperty();
   }
   Pport->RequestPaint();
}

//--------------------------------------------------------------------------------
bool TCapApp::OnBack(void)
{
   TMutexLock lock(Pmutex, thisInfo);

   if (ProomOpen)
   {
      finishRoom();
      Pport->RequestPaint();
      return true;
   }
   if (Psession.IsOpen())
      finishProperty();
   return false;
}

//--------------------------------------------------------------------------------
void TCapApp::OnPause(void)
{
   {
      TMutexLock lock(Pmutex, thisInfo);

      finishRoom();
      PcamReady = false;
      PringCount = 0;
      PsensorFresh = true; // the yaw reference restarts with the sensor
   }
   Pport->StopCamera();
   Pport->StopSensors();
   Pport->StopLocation();
}

//--------------------------------------------------------------------------------
void TCapApp::OnResume(void)
{
   Pport->RequestPermissions();
}

//--------------------------------------------------------------------------------
void TCapApp::OnResize(int w, int h, int densityQ8)
{
   TMutexLock lock(Pmutex, thisInfo);

   PviewW = w;
   PviewH = h;
   Pdensity = densityQ8 > 0 ? densityQ8 : 256;
}

//--------------------------------------------------------------------------------
void TCapApp::hintText(LPSTR out, size_t cap) const
{
   const TSpinTracker *spin = Pspin();

   out[0] = '\0';
   if (!ProomOpen || !spin)
      return;
   if (Pphase == rpWalk)
   {
      if (PplanWanted)
         snprintf(out, cap, "Calculando a planta...");
      else if (Pplan.valid)
         snprintf(out, cap, "Fique no ponto amarelo, mire o anel (%d de %d)",
                  PcornerStep + 1, PcornerCount);
      else
         snprintf(out, cap, "Vá ao canto %d (sentido horário) e toque em Cheguei", PcornerStep + 1);
      return;
   }
   if (Pphase == rpCorner)
   {
      if (!spin->Filled())
         snprintf(out, cap, Pplan.valid ? "Aponte para onde a seta do mapa indica e pare"
                                        : "Aponte para o canto oposto e pare");
      else if (Pverdict == svOutside)
         snprintf(out, cap, "Volte a mirar o canto oposto");
      else if (Pverdict == svTooFast)
         snprintf(out, cap, "Pare um instante para fotografar");
      else
         snprintf(out, cap, "Varra devagar as duas paredes do canto");
      return;
   }
   switch (Pverdict)
   {
      case svTooFast :
         snprintf(out, cap, "Pare um instante para fotografar");
         break;

      case svBadCompass :
         snprintf(out, cap, "Calibre a bússola: desenhe um 8 no ar");
         break;

      case svDrifted :
         snprintf(out, cap, "Volte ao ponto central do cômodo");
         break;

      default :
         break;
   }
   if (out[0])
      return;

   int aimBand,
       aimBin;

   if (PjobBusy && spin->AimedEmpty(aimBand, aimBin))
   {
      snprintf(out, cap, "Segure: processando a foto anterior");
      return;
   }

   /* ceiling first: its creases are rarely hidden by furniture, so the room axes (vanishing points) are
      agreed early and every later frame is checked against a firm reference; then down to the floor */
   int   best = spin->BandCount() - 1;
   float pitch = spin->LastPitchDeg();

   while (best > 0 && spin->BandFilled(best) >= spin->HeadingBins())
      best--;
   if (spin->BandCount() > 1 && spin->BandFilled(best) < spin->HeadingBins())
   {
      float target = spin->BandPitchDeg(best);

      if (pitch < target - 8.f)
         snprintf(out, cap, "Incline para cima (%s)", target > 0.f ? "linha do teto" : "paredes");
      else if (pitch > target + 8.f)
         snprintf(out, cap, "Incline para baixo (%s)", target < 0.f ? "linha do piso" : "paredes");
      else
         snprintf(out, cap, "Gire devagar no lugar");
   }
   else
      snprintf(out, cap, "Gire devagar no lugar");
}

//--------------------------------------------------------------------------------
void TCapApp::drawLabel(TSurface &s, int x, int y, LPCSTR text, DWORD rgba, int sizeDp, bool bold, bool centered)
{
   TTextStyle style;
   int        w = 0,
              h = 0;

   style.sizePx = scaleDp(sizeDp);
   style.bold = bold;
   if (centered)
   {
      Pport->MeasureText(text, style, &w, &h);
      x -= w/2;
   }
   Pport->DrawText(s, x, y, text, rgba, style);
}

//--------------------------------------------------------------------------------
void TCapApp::drawButton(TSurface &s, TButton &b, LPCSTR text, DWORD rgba)
{
   TTextStyle style;
   int        tw = 0,
              th = 0;

   style.sizePx = scaleDp(18);
   style.bold = true;
   canvasFillRect(s, b.x, b.y, b.w, b.h, rgba);
   Pport->MeasureText(text, style, &tw, &th);
   Pport->DrawText(s, b.x + (b.w - tw)/2, b.y + (b.h - th)/2, text, canvasRGBA(255, 255, 255, 255), style);
   b.visible = true;
}

/*--------------------------------------------------------------------------------
   Floor plan sprite in a size x size box: the polygon from the ceiling creases, turned heading-up
   (what the operator faces is up, so the yellow corner lies where to walk), the spin point, the
   corner stations (green done, yellow ring the current one, white still to go) and, for the
   current one, the dashed aim toward the corner to point at. Dimensions below it.
  --------------------------------------------------------------------------------*/
void TCapApp::drawPlan(TSurface &s, int x, int y, int size)
{
   float minU = 1e9f,
         maxU = -1e9f,
         minW = 1e9f,
         maxW = -1e9f,
         heading = geomHeadingDeg(poseOf(Pring[(PringHead + appRingSize - 1)%appRingSize]).Forward()),
         turn = (Pplan.axisDeg - heading)*0.01745329f,
         cs = cosf(turn),
         sn = sinf(turn);
   int   sx[layoutMaxVerts],
         sy[layoutMaxVerts],
         cx = x + size/2,
         cy = y + size/2;

   for (int i = 0; i < Pplan.vertexCount; i++)
   {
      float u = Pplan.verts[i].u,
            w = Pplan.verts[i].w,
            px = w*cs + u*sn,
            py = u*cs - w*sn;

      minU = py < minU ? py : minU;
      maxU = py > maxU ? py : maxU;
      minW = px < minW ? px : minW;
      maxW = px > maxW ? px : maxW;
   }

   float span = fmaxf(maxU - minU, maxW - minW),
         scale = span > 0.1f ? 0.8f*(float)size/span : 1.f,
         midX = 0.5f*(minW + maxW),
         midY = 0.5f*(minU + maxU);

   for (int i = 0; i < Pplan.vertexCount; i++)
   {
      float u = Pplan.verts[i].u,
            w = Pplan.verts[i].w;

      sx[i] = cx + (int)((w*cs + u*sn - midX)*scale);
      sy[i] = cy - (int)((u*cs - w*sn - midY)*scale);
   }
   canvasFillRect(s, x, y, size, size, canvasRGBA(0, 0, 0, 170));
   for (int i = 0; i < Pplan.vertexCount; i++)
   {
      int j = (i + 1)%Pplan.vertexCount;

      canvasLine(s, sx[i], sy[i], sx[j], sy[j], scaleDp(3), canvasRGBA(255, 255, 255, 255));
   }
   canvasFillCircle(s, cx - (int)(midX*scale), cy + (int)(midY*scale), scaleDp(5), canvasRGBA(255, 210, 0, 255));
   for (int i = 0; i < Pplan.stationCount; i++)
   {
      int v = Pplan.stationVertex[i];

      if (PcornerMask & (1u << i))
         canvasFillCircle(s, sx[v], sy[v], scaleDp(8), canvasRGBA(60, 220, 110, 255));
      else if (i == PcornerSlot)
      {
         int t = Pplan.targetVertex[i],
             steps = 8;

         for (int k = 0; k < steps; k += 2) // dashed aim line
            canvasLine(s, sx[v] + (sx[t] - sx[v])*k/steps, sy[v] + (sy[t] - sy[v])*k/steps,
                       sx[v] + (sx[t] - sx[v])*(k + 1)/steps, sy[v] + (sy[t] - sy[v])*(k + 1)/steps, scaleDp(3),
                       canvasRGBA(255, 210, 0, 255));
         canvasRing(s, sx[t], sy[t], scaleDp(9), scaleDp(3), canvasRGBA(255, 210, 0, 255));
         canvasFillCircle(s, sx[v], sy[v], scaleDp(12), canvasRGBA(255, 210, 0, 255));
      }
      else
         canvasFillCircle(s, sx[v], sy[v], scaleDp(6), canvasRGBA(255, 255, 255, 220));
   }

   char dims[48];

   if (PplanSketch)
      snprintf(dims, sizeof(dims), "esboço: cantos pelos eixos");
   else
      snprintf(dims, sizeof(dims), "%.2f x %.2f m", Pplan.extentU, Pplan.extentW);
   for (LPSTR p = dims; *p; p++)
      if (*p == '.')
         *p = ','; // pt-BR decimals
   drawLabel(s, cx, y + size + scaleDp(4), dims, canvasRGBA(255, 255, 255, 255), 16, true, true);
}

//--------------------------------------------------------------------------------
/* Bin color by the coherence verdict of its keyframe, against the CURRENT room consensus (so the frames
   that waited for the quorum turn green by themselves): green aligned, orange off the axes, blue no lines to
   check, pale green still measuring or waiting for the quorum, faint white empty. */
DWORD TCapApp::binColor(int band, int bin) const
{
   const TSpinTracker *spin = Pspin();

   if (!spin || !spin->BinFilled(band, bin))
      return canvasRGBA(255, 255, 255, 40);
   if (!PbinDone[band][bin])
      return canvasRGBA(160, 230, 170, 255);
   if (isnan(PbinAxis[band][bin]))
      return canvasRGBA(110, 170, 255, 255);
   if (!PaxisCheck.HasReference())
      return canvasRGBA(160, 230, 170, 255);
   if (fabsf(vanishAxisDiffDeg(PbinAxis[band][bin], PaxisCheck.ReferenceDeg())) <= cAxisTolDeg)
      return canvasRGBA(60, 220, 110, 255);
   return canvasRGBA(255, 150, 60, 255);
}

/*--------------------------------------------------------------------------------
   Coverage as the room in front of the operator, in perspective (the current heading at the
   center, scrolling as the operator turns): heading bins become panels, the upper band fans up to
   the ceiling, the lower band down to the floor, each colored by its coherence verdict. With the
   room axes known, every corner (axis + 45 + 90k) is drawn as its vertical edge with the ceiling
   creases "\ /" above and the floor creases "/ \" below. The next region to capture is outlined in
   yellow; off screen, an arrow tells which way to turn.
  --------------------------------------------------------------------------------*/
void TCapApp::drawCoverage(TSurface &s, int cx, int cy, int radius)
{
   int   bands = Pspin->BandCount(),
         bins = Pspin->HeadingBins(),
         halfW = radius*3/2,
         halfH = radius,
         yCeil = cy - halfH/5,
         yFloor = cy + halfH/5,
         yTop = cy - halfH,
         yBottom = cy + halfH,
         target = -1,
         targetBand = bands - 1;
   float spread = 1.45f,
         unit = 0.62f*(float)halfW/90.f, // pixels per degree along the creases
         binDeg = Pspin->BinWidthDeg(),
         heading = geomHeadingDeg(poseOf(Pring[(PringHead + appRingSize - 1)%appRingSize]).Forward()),
         bestGap = 1e9f;
   DWORD edge = canvasRGBA(255, 255, 255, 255),
         aim = canvasRGBA(255, 210, 0, 255);

   while (targetBand > 0 && Pspin->BandFilled(targetBand) >= bins)
      targetBand--;
   for (int bin = 0; bin < bins; bin++)
   {
      float gap = fabsf(geomHeadingDiffDeg(Pspin->BinCenterDeg(bin, heading), heading));

      if (!Pspin->BinFilled(targetBand, bin) && gap < bestGap)
      {
         bestGap = gap;
         target = bin;
      }
   }
   canvasFillRect(s, cx - halfW, yTop, 2*halfW, 2*halfH, canvasRGBA(0, 0, 0, 90));

   int  aimBand = -1,
        aimBin = -1;
   bool busy = PjobBusy && Pspin->AimedEmpty(aimBand, aimBin);

   for (int b = 0; b < bands; b++)
   {
      float pitch = Pspin->BandPitchDeg(b);
      int   inner = pitch > 5.f ? yCeil : yFloor,
            outer = pitch > 5.f ? yTop : yBottom;
      bool  middle = pitch >= -5.f && pitch <= 5.f;

      for (int bin = 0; bin < bins; bin++)
      {
         float d0 = geomHeadingDiffDeg(Pspin->BinCenterDeg(bin, heading), heading),
               signedD = fmodf(Pspin->BinCenterDeg(bin, heading) - heading + 540.f, 360.f) - 180.f,
               a0 = signedD - 0.5f*binDeg + 1.f,
               a1 = signedD + 0.5f*binDeg - 1.f;

         if (d0 > 100.f)
            continue;

         int xs[4],
             ys[4];

         xs[0] = cx + (int)(a0*unit);
         xs[1] = cx + (int)(a1*unit);
         if (middle)
         {
            xs[2] = xs[1];
            xs[3] = xs[0];
            ys[0] = bands == 1 ? yTop : yCeil; // a single band (corner fan) spans the whole wall
            ys[1] = ys[0];
            ys[2] = bands == 1 ? yBottom : yFloor;
            ys[3] = ys[2];
         }
         else
         {
            xs[2] = cx + (int)(a1*unit*spread);
            xs[3] = cx + (int)(a0*unit*spread);
            ys[0] = inner;
            ys[1] = inner;
            ys[2] = outer;
            ys[3] = outer;
         }
         // aimed at while the previous photo is still processing: yellow, the operator holds instead of wondering
         canvasFillQuad(s, xs, ys,
                        busy && b == aimBand && bin == aimBin ? canvasRGBA(255, 210, 0, 120) : binColor(b, bin));
         if (b == targetBand && bin == target)
            for (int i = 0; i < 4; i++)
               canvasLine(s, xs[i], ys[i], xs[(i + 1)%4], ys[(i + 1)%4], scaleDp(3), aim);
      }
   }
   if (PaxisCheck.HasReference())
      for (int k = 0; k < 4; k++)
      {
         float corner = PaxisCheck.ReferenceDeg() + 45.f + 90.f*(float)k,
               d = fmodf(corner - heading + 540.f, 360.f) - 180.f;

         if (fabsf(d) > 95.f)
            continue;

         int x = cx + (int)(d*unit),
             reach = halfW/3;

         canvasLine(s, x, yCeil, x, yFloor, scaleDp(2), edge);
         canvasLine(s, x, yCeil, x - reach, yTop, scaleDp(2), edge);
         canvasLine(s, x, yCeil, x + reach, yTop, scaleDp(2), edge);
         canvasLine(s, x, yFloor, x - reach, yBottom, scaleDp(2), edge);
         canvasLine(s, x, yFloor, x + reach, yBottom, scaleDp(2), edge);
      }

   // what the camera sees now, and the way to the next region when it is off screen
   float pitchNow = geomPitchDeg(poseOf(Pring[(PringHead + appRingSize - 1)%appRingSize]).Forward());
   int   fw = (int)(0.5f*PhfovDeg*unit),
         fy = cy - (int)(pitchNow/60.f*(float)halfH),
         fh = (int)(0.5f*PvfovDeg/60.f*(float)halfH);

   canvasLine(s, cx - fw, fy - fh, cx + fw, fy - fh, scaleDp(2), aim);
   canvasLine(s, cx + fw, fy - fh, cx + fw, fy + fh, scaleDp(2), aim);
   canvasLine(s, cx + fw, fy + fh, cx - fw, fy + fh, scaleDp(2), aim);
   canvasLine(s, cx - fw, fy + fh, cx - fw, fy - fh, scaleDp(2), aim);
   if (target >= 0 && bestGap > 95.f)
   {
      float d = fmodf(Pspin->BinCenterDeg(target, heading) - heading + 540.f, 360.f) - 180.f;
      int   side = d > 0.f ? 1 : -1,
            tip = cx + side*(halfW - scaleDp(8)),
            base = tip - side*scaleDp(28);

      canvasLine(s, base, cy - scaleDp(20), tip, cy, scaleDp(4), aim);
      canvasLine(s, base, cy + scaleDp(20), tip, cy, scaleDp(4), aim);
   }

   // legend: what each color means
   static const LPCSTR cLegend[3] = { "ok", "fora do eixo", "sem linhas" };
   const DWORD         legendColor[3] = { canvasRGBA(60, 220, 110, 255), canvasRGBA(255, 150, 60, 255),
                                          canvasRGBA(110, 170, 255, 255) };
   int                 box = scaleDp(12),
                       ly = yBottom + scaleDp(10);

   canvasFillRect(s, cx - halfW, ly - scaleDp(4), 2*halfW, box + scaleDp(8), canvasRGBA(0, 0, 0, 150));
   for (int i = 0; i < 3; i++)
   {
      int x = cx - halfW + (2*halfW)*i/3 + scaleDp(8);

      canvasFillRect(s, x, ly, box, box, legendColor[i]);
      drawLabel(s, x + box + scaleDp(6), ly - scaleDp(2), cLegend[i], canvasRGBA(255, 255, 255, 255), 13, false, false);
   }
}

//--------------------------------------------------------------------------------
void TCapApp::OnPaint(TSurface &s)
{
   TMutexLock lock(Pmutex, thisInfo);
   char       line[128],
              hint[96];
   DWORD      white = canvasRGBA(255, 255, 255, 255),
              bar = canvasRGBA(0, 0, 0, 150);

   if (!Pdensity)
      Pdensity = 256;
   canvasFillRect(s, 0, 0, s.width, s.height, canvasRGBA(16, 16, 20, 255));
   if (PhasPreview)
      canvasBlit(s, 0, 0, s.width, s.height, Ppreview(), appPreviewW, PpreviewH);

   canvasFillRect(s, 0, 0, s.width, scaleDp(98), bar);
   if (Pphase == rpCenter && Pspin)
      snprintf(line, sizeof(line), "%s  -  centro %d%%", ProomName, Pspin->Filled()*100/Pspin->Total());
   else if (Pphase == rpCorner && Pspin)
      snprintf(line, sizeof(line), "%s  -  canto %d/%d  %d/%d", ProomName, PcornerStep + 1, PcornerCount,
               Pspin->Filled(), Pspin->Total());
   else if (Pphase == rpWalk)
      snprintf(line, sizeof(line), "%s  -  cantos %d/%d", ProomName, PcornerStep, PcornerCount);
   else
      snprintf(line, sizeof(line), "%s", Pstatus);
   drawLabel(s, scaleDp(16), scaleDp(12), line, white, 20, true, false);

   float heading = PringCount ? geomHeadingDeg(poseOf(Pring[(PringHead + appRingSize - 1)%appRingSize]).Forward()) : 0.f;
   char  gps[48];

   if (PhasLoc)
      snprintf(gps, sizeof(gps), "GPS +-%u m", (unsigned)(Ploc.horizAccMm/1000u));
   else
      snprintf(gps, sizeof(gps), PlocAllowed ? "GPS aguardando" : "GPS sem permissão");
   snprintf(line, sizeof(line), "Rumo %03d%s  -  %s", (int)heading, Pmagnetic ? "" : " (relativo)", gps);
   drawLabel(s, scaleDp(16), scaleDp(46), line, canvasRGBA(220, 220, 220, 255), 14, false, false);
   if (ProomOpen)
   {
      DWORD tone = canvasRGBA(90, 230, 130, 255);

      if (PaxisVerdict == avMisaligned)
         tone = canvasRGBA(255, 150, 60, 255);
      else if (PaxisVerdict == avNoLines || PaxisVerdict == avPending)
         tone = canvasRGBA(170, 170, 170, 255);
      if (PaxisCheck.HasReference())
         snprintf(line, sizeof(line), "Linhas: %d alinhados, %d fora, %d sem linhas  (desvio %.1f)",
                  PaxisCheck.Aligned(), PaxisCheck.Misaligned(), PaxisCheck.Unverified(),
                  isnan(PaxisCheck.LastDeviationDeg()) ? 0.f : PaxisCheck.LastDeviationDeg());
      else
         snprintf(line, sizeof(line), "Linhas: aguardando referência (%d sem linhas)", PaxisCheck.Unverified());
      drawLabel(s, scaleDp(16), scaleDp(70), line, tone, 14, false, false);
   }

   if (Pphase == rpCenter && Pspin && PringCount)
      drawCoverage(s, s.width/2, s.height/2, s.width*3/10);
   if (Pphase == rpCorner && Pspin && PringCount)
      drawCoverage(s, s.width/2, s.height/2, s.width*3/10); // the corner fan in the same perspective view
   if (Pplan.valid && Pphase == rpWalk)
      drawPlan(s, s.width/10, s.height/2 - s.width*4/10, s.width*8/10); // walking: the map is the guide
   if (Pplan.valid && Pphase == rpCorner)
   {
      int room = s.height/2 - s.width*3/10 - scaleDp(106 + 30), // between the top bar and the perspective view
          size = room < s.width/2 ? room : s.width/2;

      if (size > scaleDp(80))
         drawPlan(s, (s.width - size)/2, scaleDp(106), size); // where to stand and where to aim
   }

   hintText(hint, sizeof(hint));
   if (hint[0])
   {
      canvasFillRect(s, 0, s.height - scaleDp(200), s.width, scaleDp(44), bar);
      drawLabel(s, s.width/2, s.height - scaleDp(190), hint, canvasRGBA(255, 220, 90, 255), 18, true, true);
   }

   PbtnMain.visible = false;
   PbtnFinish.visible = false;
   if (PcamReady)
   {
      PbtnMain.w = s.width - scaleDp(32);
      PbtnMain.h = scaleDp(60);
      PbtnMain.x = scaleDp(16);
      PbtnMain.y = s.height - scaleDp(84);

      char main[48];

      if (Pphase == rpNone)
         snprintf(main, sizeof(main), "Iniciar cômodo");
      else if (Pphase == rpCenter)
         snprintf(main, sizeof(main), "Ir para os cantos");
      else if (Pphase == rpWalk)
         snprintf(main, sizeof(main), "Cheguei");
      else
         snprintf(main, sizeof(main), PcornerStep + 1 >= PcornerCount ? "Concluir último canto" : "Próximo canto");
      drawButton(s, PbtnMain, main, Pphase == rpNone ? canvasRGBA(30, 110, 220, 235) : canvasRGBA(40, 160, 90, 235));
      if (ProomOpen || Psession.IsOpen())
      {
         PbtnFinish.w = PbtnMain.w;
         PbtnFinish.h = scaleDp(48);
         PbtnFinish.x = PbtnMain.x;
         PbtnFinish.y = PbtnMain.y - scaleDp(60);
         drawButton(s, PbtnFinish, ProomOpen ? "Concluir cômodo" : "Finalizar imóvel", canvasRGBA(90, 90, 100, 235));
      }
   }
}

//--------------------------------------------------------------------------------
void appStart(TCapPort *port)
{
   if (gApp)
      return;
   gApp = new TCapApp(port);
   gApp->Launch();
}

//--------------------------------------------------------------------------------
void appStop(void)
{
   delete gApp;
   gApp = NULL;
}

//--------------------------------------------------------------------------------
