#include "capPort.h"
#include "capCanvas.h"
#include "capSpin.h"
#include "capFrameMeta.h"
#include "capEXIF.h"
#include "capVanish.h"
#include "capLayout.h"
#include "capBlur.h"
#include "capDoor.h"
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
   appSlots         = 8,        // keyframe pipeline slots (~18 MB each at 12 MP)
   appSlotReserve   = 2,        // slots a bin's extra candidates leave free: every bin gets its first frame (user: no gap for a sharper one)
   appHoldMs        = 1000,     // a bin's sharpest goes on after this without a sharper one, even with the pose still on it
   appFocusRects    = 3,        // focus regions of an aim (a triangle as three bands)
   appFocusMeasurePx = 1024,    // a focus region is grown to this for its blur (the tiles are 256 px of a 4x reduction)
   appPreviewW      = 360,      // portrait preview width in pixels
   appMaxPixels     = 50000000, // camera request: the largest 4:3 YUV the sensor gives (every pixel counts later)
   appJPEGQuality   = 92,
   appMaxAttitudeMs = 40,       // a frame without an attitude this close is not kept
   appMaxCameras    = 8,
   appBlurPx        = 4,        // accepted motion smear of a keyframe at the assumed exposure
   appExposureHz    = 30,       // exposure cap requested from the camera (1/30 s)
   appFloorBand     = 1,        // spin band recorded on a corner station's floor view (its fan is band 0)
   appBlurTries     = 2,        // sharper retakes offered to a green bin
   appBackArmMs     = 3000,     // a second Back within this ends the room (the first only warns)
   appFocusMaxMs    = 4000,     // a focus run the camera never answers is given up
   appFocusTries    = 2,        // focus runs per station when the lens settles at an odd distance
   appBlurHistory   = 64,       // blur of the latest keyframes (the camera's own softness), for their median
   appBlurMinSamples = 5,       // no blur verdict before this many
   appMaxGuides     = 16,       // corner support lines on the spin view
   appMaxDoors      = 12,       // door candidates of a room
   appDoorShotMs    = 1500,     // between two door-station shots
   appMaxSuperseded = 4096,     // replaced photos of a property, dropped from the log at its end (production)
   appKeepSuperseded = 0,       // 0: a retake overwrites, the replaced photo leaves the log (user, 2026-09-28); 1 = debug: both stay
   appAlertMs       = 150       // vibration of an orange bin or a gap left in a band: look again (user, 2026-09-28)
};

static const float cFocusDiopters = 0.5f, // fixed focus at 2 m for the whole spin: sharp ~1.3-4 m, constant intrinsics
                   cAxisTolDeg = 2.f,     // a keyframe whose room axes drift more than this from the room reference is flagged
                   cOffAxisShowDeg = 8.f, // shown orange only beyond this (gross errors); the fine flag stays in the records
                   cDiagonalTolDeg = 6.f, // an axis this close to 45 degrees off is a diagonal tile floor, not an error
                   cCeilingM = 2.8f,       // assumed ceiling height until door heads (2.10 m) measure it
                   cBlurMaxPx = 4.f,       // blur floor: beyond it the keyframe is orange (083920: 15 and 44 out of focus, 5.7-6.3)
                   cBlurGoodPx = 2.f,      // a green keyframe above this stays open for a sharper one (32 almost good, 2.6)
                   cBlurOrangeRel = 1.3f,  // ...and both floors follow the camera: orange beyond 1.3x the median of the latest keyframes,
                   cBlurRetakeRel = 1.1f,  // a sharper retake beyond 1.1x (the Moto's own softness reads ~5 px: 102600)
                   cFocusApexV = 0.5f,     // the focus triangle: top corners of the upright view and this far down the middle
                   cFocusLevelHalf = 0.15f, // level poses: a horizontal strip, the middle 30% of the view high (user, 2026-09-28)
                   cFocusLevelSide = 0.05f, // and the whole width but this margin each side
                   cFocusMinDiopters = 0.1f, // a room lies between ~0.6 and 10 m; beyond, the AF locked on the wrong thing
                   cFocusMaxDiopters = 1.6f,
                   cGuideReachM = 0.35f,   // two walls this close to meeting make a corner (the support lines)
                   cDoorMergeDeg = 8.f,    // doors found this close in heading, from one station, are one
                   cDoorDrawHalfDeg = 6.f, // half width of a door outline on the spin view
                   cDoorSameM = 0.6f,      // candidates placed this close on the plan are one door
                   cDoorAimDeg = 8.f,      // the camera fits the door: its heading this close to the door's
                   cDoorLevelDeg = 35.f,   // and pitched within this (up for the molding, down for the foot)
                   cDoorSteadyDps = 4.f,   // turning slower than this
                   cDoorHeadM = 2.10f,     // a door head (the ruler)
                   cDoorHalfWidthM = 0.4f, // half of a door opening, for its outline
                   cDoorStandM = 2.5f,     // shooting a door from this far in front (door and molding fit a level photo)
                   cDoorStandMinM = 1.0f,  // never nearer
                   cDoorStandClearM = 0.4f, // and short of the opposite wall by this
                   cDoorFitDeg = 6.f,      // the door seen fits the one drawn: its heading this close...
                   cDoorFitMinM = 0.55f,   // ...and its width within these
                   cDoorFitMaxM = 1.15f,
                   cDoorFitDepthFrac = 0.25f, // ...and its distance (by head and foot) this close to the wall's from the eye
                   cRulerMinM = 2.35f,     // a ceiling line a door may measure (2.40-3.00 m ceilings, molding included)
                   cRulerMaxM = 3.05f,
                   cWireNearM = 0.05f,     // the wireframe's near plane (camera space)
                   cWireCornerInM = 0.4f,  // standing in a corner: this far inside it
                   cWireCameraM = 1.5f;    // camera height before the plan measures it

// A door seen in the room: candidates for the door station (the operator confirms or drops them)
struct TDoorCandidate {
   float headingDeg,     // world heading from where it was seen (the spin point or a corner station)
         ratioSum;       // sum of the crease-over-head ratios measured on it (a level frame with the crease)
   int   views,          // keyframes that found it
         ratios;         // of them, with the ratio
   DWORD station;        // station of its first sighting
   float obsU,           // where it was seen from, on the plan (NaN: no plan then)
         obsW;
};

// A door of the room on the plan, for the door station
struct TPlanDoor {
   TPlanPoint at,       // where its heading met the plan's walls
              stand,    // where to shoot it from: in front of it, on its wall's normal
              along;    // unit direction of its wall (u, w): the opening spans +-half a door along it
   float      ratioSum; // crease-over-head ratios measured on it
   int        ratios;
   BYTE       state;    // TDoorState: 0 to shoot, 1 confirmed, 2 dropped
   QWORD      imageNs;  // the shot that confirmed it (its rtImage)
   float      tanHead,  // door-station shots from one spot: tangents of the head, foot and ceiling line elevations
              tanFoot,  // in the level frontal view (sums) - the ratio needs no single photo to hold them all
              tanCrease;
   int        nHead,
              nFoot,
              nCrease;
};

// Where the focus is measured: what the pose is after
enum TFocusAim {
   faCeiling, // the triangle anchored on the top edge of the upright view
   faLevel,   // a horizontal strip across the middle
   faFloor    // the triangle anchored on the bottom edge
};

// Where the operator is in the room protocol: two center spins, then each corner aiming at the opposite one
enum TRoomPhase {
   rpNone,   // no room open
   rpCenter, // in-place spin from the middle (both pitch bands)
   rpWalk,   // walking to the next corner, waiting for "arrived"
   rpCorner, // standing in a corner, sweeping the fan around the opposite one
   rpDoor    // the doors: each candidate shot frontally, confirmed when found at the image center, or dropped
};

class TCapApp;

/* The keyframe pipeline (user, 2026-09-28): the camera thread fills slots with candidate frames (planes, pose,
   gyroscope), each tagged with the bin its pose lies in, at the focus worker's own rate - the frames between skipped
   by a Bresenham on the camera's rate; the first frame of an empty bin always enters. The focus worker measures each
   candidate's focus and keeps it only if sharper than what its bin holds (an empty bin holds nothing: any frame wins);
   the bin's sharpest is held while the pose stays on it, then goes to the keyframe worker, which does the final
   measures, the storage and the bins' state (what the screen draws) - at the pace of the bins, not of the camera */
enum TSlotState {
   ssFree,      // ready for the camera thread
   ssCandidate, // a frame waiting for the focus worker
   ssMeasuring, // its focus being measured
   ssHeld,      // its bin's sharpest so far, while the pose stays on the bin
   ssChosen,    // queued for the keyframe worker
   ssFinal      // being measured, encoded and stored
};

// One frame in the pipeline
struct TKeySlot {
   TBlock<BYTE> planes; // planar Y, U, V
   TImageRecord rec;
   TFrameMeta   meta;
   TBlurResult  blur;   // measured by the focus worker, reused by the keyframe worker
   QWORD        stamp;
   int          band,   // the bin it stands for (a door or floor-view shot: direct, no comparison)
                bin;
   bool         direct;
   float        focus[4*appFocusRects]; // the focus regions of its aim, where its sharpness is measured
   int          focusCount;
   BYTE         state;  // TSlotState
};

// Encodes the chosen keyframes off the camera thread
class TKeyframeWorker : public TCormWorker
{
 public:
   explicit TKeyframeWorker(TCapApp &app) : TCormWorker("keyframe"), Papp(app) {}

 protected:
   void DoJob(void) override;

 private:
   TCapApp &Papp;
};

// Picks the sharpest frame of each group of candidates
class TFocusWorker : public TCormWorker
{
 public:
   explicit TFocusWorker(TCapApp &app) : TCormWorker("focus"), Papp(app) {}

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
   void EncodePending(void); // keyframe worker thread: every chosen frame in turn
   void FocusPending(void);  // focus worker thread: every closed group in turn

   void OnResize(int w, int h, int densityQ8) override;
   void OnPaint(TSurface &s) override;
   void OnTap(int x, int y) override;
   bool OnBack(void) override;
   void OnPause(void) override;
   void OnResume(void) override;
   void OnPermissions(bool camera, bool location) override;
   void OnCameraReady(const TCamInfo &info) override;
   void OnFocus(bool locked, float diopters) override;
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
   bool  queueKeyframe(const TCamFrame &f, const TMat4 &pose, const TAttitude &a, bool floorFrame, int band, int bin, bool direct);
   bool  encodeOne(void);
   int   freeSlots(void) const;
   bool  pipelineIdle(void) const;
   bool  binLive(int band, int bin) const;
   void  startRoom(void);
   void  finishRoom(void);
   void  beginStation(TStationKind kind);
   void  endStation(void);
   void  stationDone(void);
   int   nextCorner(void) const;
   int   deduceCorner(float headingDeg) const;
   bool  binOffAxis(int band, int bin) const;
   void  checkComplete(void);
   void  beginFloorView(void);
   void  requestFocus(TFocusAim aim);
   int   focusRects(TFocusAim aim, float *rects) const;
   void  wantFocus(TFocusAim aim);
   TPlanPoint floorViewTarget(void) const;
   float floorViewHeadingDeg(void) const;
   int   orangeBins(void) const;
   float blurFloorPx(float rel, float minPx) const;
   void  addRoomBlur(float px);
   DWORD binColor(int band, int bin) const;
   void  drawPlan(TSurface &s, int x, int y, int size);
   void  drawEye(TSurface &s, int x, int y, float dx, float dy, int size, DWORD rgba);
   void  solvePlan(void);
   void  writeLayout(bool doorScaled);
   void  finishDoors(void);
   void  updateGuides(void);
   void  addDoor(float headingDeg, float creaseOverHead, DWORD station);
   int   buildPlanDoors(void);
   float planRayHit(float u, float w, float du, float dw, int *edge) const;
   void  beginDoors(void);
   void  nextDoor(void);
   float doorAimDeg(void) const;
   void  doorShot(const TDoor *doors, const float *headings, const float *horizons, const float *focals, const TVec3 *walls,
                  int found, const TFrameMeta &meta, const TMat4 &pose);
   void  drawDoorAim(TSurface &s, int cx, int cy, int radius);
   bool  wireToScreen(const TSurface &s, const TVec3 &pc, float &sx, float &sy) const;
   void  wireEdge(TSurface &s, const TMat4 &pose, const TVec3 &a, const TVec3 &b, DWORD rgba);
   void  drawWireframe(TSurface &s);
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
   TFocusWorker         PfocusWorker;
   TKeySlot             Pslot[appSlots];
   TBlock<TSpinTracker> Pspin,
                        Pahead;       // at a corner, the floor view taken early (tilted down during the fan)
   TBlock<DWORD>        Ppreview;
   TBlock<BYTE>         PdoorBuf;     // the frontal view the door finder works on (4 planes, worker thread)
   TByteBuf             Pjpeg,
                        Pexif,
                        Pfinal,
                        PmetaBuf;
   TAttitude            Pring[appRingSize];
   TCamInfo             Pcam;
   TIntrinsics          Pintr;
   TLocationRecord      Ploc;
   TSpinVerdict         Pverdict;
   TSpinConfig          PcenterCfg,
                        PcornerCfg,
                        PfloorCfg;    // from a corner, one view down to the middle of the room (the floor tiles)
   TVanishConfig        PvanishCfg;
   TTiltBias            PtiltBias;   // camera tilt calibrated from the measured verticals (worker thread only)
   TAxisCheck           PaxisCheck;
   TAxisVerdict         PaxisVerdict;
   TRoomLayout          Playout;
   TLayoutPlan          Pplan,
                        PguidePlan;   // the walls so far, for the support lines (worker thread)
   TBlock<float>        PedgeRays;
   TBlock<BYTE>         PedgeLabels;
   TVanishEdges         Pedges;
   TFocusAim            PfocusAim,    // where the last focus run measured
                        PfocusWant;   // and where the pending one will
   int                  PfocusTries,  // focus runs of the current station
                        PfocusBand,   // the guided band the center spin focused for
                        PcornerStep,  // corner stations already captured
                        PcornerCount, // planned corner stations (4 without a plan)
                        PcornerSlot;  // the station at hand: suggested while walking, then deduced from the aimed diagonal
   DWORD                PcornerMask;  // stations captured, one bit per plan station
   float                PbinAxis[spinMaxBands][spinMaxBins]; // room axis measured on the keyframe of each bin (NaN: no lines)
   bool                 PbinDone[spinMaxBands][spinMaxBins], // the keyframe of the bin went through the vanishing check
                        PbinOrange[spinMaxBands][spinMaxBins], // its verdict: far off the axes or blurred (accepted, open to a retake)
                        PbinGap[spinMaxBands][spinMaxBins];    // empty between two done bins, already signaled
   float                PbinBlur[spinMaxBands][spinMaxBins];   // FFT blur of the bin's keyframe (NaN: plain image)
   BYTE                 PbinTries[spinMaxBands][spinMaxBins];  // retakes offered to a green bin for a sharper photo
   QWORD                PbinStamp[spinMaxBands][spinMaxBins],  // stamp of the bin's current photo (its rtImage)
                        Psuperseded[appMaxSuperseded];         // photos replaced so far in the property
   float                ProomBlur[appBlurHistory];             // latest keyframe blurs of the capture (ring)
   int                  ProomBlurCount,
                        PsupersededCount,
                        PguideCount,
                        PdoorCount,
                        PplanDoorCount,
                        PdoorIdx;     // the door being shot
   TDoorCandidate       PdoorCand[appMaxDoors];  // doors seen in the room so far
   TPlanDoor            PplanDoors[appMaxDoors]; // the candidates placed on the plan (the door station goes through them)
   TPlanPoint           PdoorEye;                // where the operator stands during the door station (plan)
   float                PguideDeg[appMaxGuides]; // world headings of the ceiling corners found so far
   TPlanPoint           PguideAt[appMaxGuides];  // and where they are on the plan (the wireframe)
   bool                 PguideConvex[appMaxGuides], // that corner juts into the room (the walls leave it away from the spin point)
                        PplanWanted,
                        PplanSketch;  // Pplan is only a square on the room axes (no plan could be made): no dimensions
   TRoomPhase           Pphase;
   TStationRecord       Pstation;
   TButton              PbtnMain,
                        PbtnFinish;
   QWORD                PlastFrameNs, // the camera's previous frame (its native rate)
                        PfocusNs,     // the focus run began
                        PbackArmedNs, // first Back pressed at (0: not armed)
                        PdoorShotNs,  // the door station's last shot
                        PdoorPrevNs;  // and its previous frame (turn rate)
   DWORD                PkeySeq,
                        PstationCount,
                        ProomIndex;
   char                 ProomName[roomNameMax],
                        PdevModel[64],
                        Pstatus[96];
   int                  Pchosen[appSlots], // the chosen slots, in order, for the keyframe worker (a ring)
                        PchosenHead,
                        PchosenCount,
                        PcurBand,     // the bin the pose is on (-1: none), whose sharpest the focus worker holds
                        PcurBin,
                        PviewW,
                        PviewH,
                        Pdensity,
                        PringHead,
                        PringCount,
                        PpreviewH,
                        PpropRatios,  // how many
                        Prooms;
   float                PcamFps,          // the camera's native frame rate (smoothed)
                        PfocusSec,        // the focus worker's net time per frame (smoothed): the capture cadence
                        PkeepErr,         // the Bresenham error of the frames taken as candidates
                        PdoorPrevHeading, // heading of the door station's previous frame
                        PpropCeilingM,    // the property's ceiling line from its doors (NaN until a door station measures it)
                        PpropRatioSum,    // the confirmed doors' crease-over-head ratios so far
                        PhfovDeg,
                        PvfovDeg;
   bool                 PcamReady,
                        PlocAllowed,
                        PhasLoc,
                        PhasPreview,
                        ProomOpen,
                        Palert,       // an orange bin or a gap: the camera thread vibrates on its next frame
                        PcompleteSignaled,
                        PfloorView,   // the corner station is on its floor view (after its fan)
                        Pmagnetic,
                        PsensorFresh,
                        Pfocusing,    // a focus run is on: no keyframe until the lens settles
                        PfocusPending, // a focus run waits for the pose to reach its band (PfocusWant)
                        PaheadDone,   // the floor view of this corner is already in
                        PposeOk;      // the last pose is one to capture (else the view is red, nothing kept)
};

static TCapApp *gApp = NULL;

//--------------------------------------------------------------------------------
void TKeyframeWorker::DoJob(void)
{
   Papp.EncodePending();
}

//--------------------------------------------------------------------------------
void TFocusWorker::DoJob(void)
{
   Papp.FocusPending();
}

//--------------------------------------------------------------------------------
TCapApp::TCapApp(TCapPort *port) : Pport(port), Pencoder(appJPEGQuality), Pworker(*this), PfocusWorker(*this), Pslot(), Pring(),
   Pcam(), Pintr(), Ploc(), Pverdict(svCovered), PcenterCfg(), PcornerCfg(), PfloorCfg(),
   PvanishCfg(TVanishConfig::Default()), PaxisCheck(cAxisTolDeg), PaxisVerdict(avNoLines), Pplan(), PguidePlan(), Pedges(),
   PfocusAim(faLevel), PfocusWant(faLevel), PfocusTries(0), PfocusBand(0), PcornerStep(0), PcornerCount(4), PcornerSlot(0), PcornerMask(0u), PbinAxis(), PbinDone(), PbinOrange(), PbinGap(), PbinBlur(), PbinTries(), PbinStamp(), Psuperseded(), ProomBlur(), ProomBlurCount(0), PsupersededCount(0), PguideCount(0), PdoorCount(0), PplanDoorCount(0), PdoorIdx(0), PdoorCand(), PplanDoors(), PdoorEye(), PguideDeg(), PguideAt(), PguideConvex(), PplanWanted(false), PplanSketch(false), Pphase(rpNone), Pstation(), PbtnMain(), PbtnFinish(), PlastFrameNs(0u), PfocusNs(0u), PbackArmedNs(0u), PdoorShotNs(0u), PdoorPrevNs(0u), PkeySeq(0u),
   PstationCount(0u), ProomIndex(0u), Pchosen(), PchosenHead(0), PchosenCount(0), PcurBand(-1), PcurBin(-1),
   PviewW(0), PviewH(0), Pdensity(256), PringHead(0), PringCount(0), PpreviewH(0), PpropRatios(0), Prooms(0), PcamFps(30.f), PfocusSec(0.3f), PkeepErr(0.f), PdoorPrevHeading(0.f), PpropCeilingM(NAN), PpropRatioSum(0.f), PhfovDeg(60.f),
   PvfovDeg(60.f), PcamReady(false), PlocAllowed(false), PhasLoc(false), PhasPreview(false),
   ProomOpen(false), Palert(false), PcompleteSignaled(false), PfloorView(false), Pmagnetic(false), PsensorFresh(true),
   Pfocusing(false), PfocusPending(false), PaheadDone(false), PposeOk(true)
{
   ProomName[0] = '\0';
   PdevModel[0] = '\0';
   snprintf(Pstatus, sizeof(Pstatus), "Iniciando...");
}

//--------------------------------------------------------------------------------
TCapApp::~TCapApp(void)
{
   PfocusWorker.Terminate();
   PfocusWorker.Join();
   Pworker.Terminate();
   Pworker.Join();
   Psession.Close(Pport->WallClockNs());
}

//--------------------------------------------------------------------------------
void TCapApp::Launch(void)
{
   Pworker.Start(thisInfo);
   PfocusWorker.Start(thisInfo);
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
   PfloorCfg = TSpinConfig::ForFloorView(PhfovDeg, PvfovDeg);
   PfloorCfg.maxRateDps = maxRate;

   TSpinTracker *spin = new TSpinTracker(PcenterCfg);

   Pspin = spin;

   int portraitW = portraitSwap ? info.height : info.width,
       portraitH = portraitSwap ? info.width : info.height;

   PpreviewH = appPreviewW*portraitH/portraitW;
   {
      TAlloc<DWORD> preview((size_t)appPreviewW*PpreviewH);

      preview.Drop(Ppreview);
      for (int s = 0; s < appSlots; s++) // the pipeline's slots, each a whole frame (kept when the camera comes back)
      {
         if (Pslot[s].planes())
            continue;

         TAlloc<BYTE> planes((size_t)info.width*info.height*3u/2u + 16u);

         planes.Drop(Pslot[s].planes);
         Pslot[s].state = (BYTE)ssFree;
      }
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
   if (!PdoorBuf())
   {
      TAlloc<BYTE> door((size_t)doorViewW*doorViewH*4u); // luma, chroma and the valid mask of a frontal view

      door.Drop(PdoorBuf);
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
/* Copies the frame into a free slot as a candidate of its bin (planar Y, U, V); direct: a floor-view or door shot,
   passed on as it is; false when no slot is free */
bool TCapApp::queueKeyframe(const TCamFrame &f, const TMat4 &pose, const TAttitude &a, bool floorFrame, int band, int bin,
                            bool direct)
{
   int slot = -1;

   for (int s = 0; s < appSlots && slot < 0; s++)
      slot = Pslot[s].state == (BYTE)ssFree && Pslot[s].planes() ? s : slot;
   if (slot < 0)
      return false;

   TKeySlot        &k = Pslot[slot];
   const TYUVImage &src = f.yuv;
   int              cw = (src.width + 1)/2,
                    ch = (src.height + 1)/2;
   LPBYTE           yDst = k.planes(),
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

   k.rec = TImageRecord();
   k.rec.width = (DWORD)src.width;
   k.rec.height = (DWORD)src.height;
   k.rec.format = pfJPEG;
   k.rec.intr = Pintr;
   for (int i = 0; i < 5; i++)
      k.rec.distortion[i] = Pcam.distortion[i];
   k.rec.cameraToWorld = pose;
   k.rec.compass.magneticDeg = a.magnetic ? geomHeadingDeg(pose.Forward()) : NAN;
   k.rec.compass.trueDeg = NAN;
   k.rec.compass.accuracyDeg = a.accuracyDeg;
   k.meta = TFrameMeta();
   Psession.SessionId(k.meta.sessionId);
   k.meta.roomIndex = ProomIndex;
   k.meta.seq = PkeySeq++;
   k.meta.sensorNs = f.stampNs;
   k.meta.wallNs = Pport->WallClockNs();
   k.meta.cameraToWorld = pose;
   k.meta.compass = k.rec.compass;
   k.meta.headingRef = a.magnetic ? hrMagnetic : hrArbitrary;
   k.meta.intr = Pintr;
   for (int i = 0; i < 5; i++)
      k.meta.distortion[i] = Pcam.distortion[i];
   k.meta.headingDeg = geomHeadingDeg(pose.Forward()); // as the tracker saw it (the door station has none)
   k.meta.pitchDeg = geomPitchDeg(pose.Forward());

   TVec3 fwd = pose.Forward();

   k.meta.forward[0] = fwd.x; // the gyroscope attitude, explicit per frame: where it looks and how it is rolled
   k.meta.forward[1] = fwd.y;
   k.meta.forward[2] = fwd.z;
   k.meta.rollDeg = geomRollDeg(pose);
   k.meta.spinBand = (BYTE)(floorFrame ? appFloorBand : (Pphase == rpDoor ? 0 : band)); // floor view: own band
   k.meta.spinBin = (BYTE)(floorFrame ? 0 : (Pphase == rpDoor ? PdoorIdx : bin)); // door station: the door
   k.meta.stationIndex = (BYTE)Pstation.index;
   k.meta.stationKind = (BYTE)Pstation.kind;
   k.meta.cornerIndex = Pstation.kind == skCorner ? Pstation.corner : 0u;
   k.meta.targetCorner = Pstation.kind == skCorner ? Pstation.target : 0u;
   k.meta.focalMm = Pcam.focalMm;
   if (PhasLoc)
   {
      k.meta.latE7 = Ploc.latE7;
      k.meta.lonE7 = Ploc.lonE7;
      k.meta.altMm = Ploc.altMm;
      k.meta.horizAccMm = Ploc.horizAccMm;
      k.meta.fixAgeNs = f.stampNs > Ploc.fixNs ? f.stampNs - Ploc.fixNs : 0u;
   }
   k.stamp = f.stampNs;
   k.band = band;
   k.bin = bin;
   k.direct = direct;

   // measured where its band focuses: the ceiling's triangle, the floor's, the level strip (a direct shot: the lens's aim)
   float     pitch = !direct && Pspin && band >= 0 ? Pspin->BandPitchDeg(band) : 0.f;
   TFocusAim aim = direct ? PfocusAim : (pitch > 5.f ? faCeiling : (pitch < -5.f ? faFloor : faLevel));

   k.focusCount = focusRects(aim, k.focus);
   k.state = (BYTE)ssCandidate;
   return true;
}

//--------------------------------------------------------------------------------
int TCapApp::freeSlots(void) const
{
   int n = 0;

   for (int s = 0; s < appSlots; s++)
      n += Pslot[s].state == (BYTE)ssFree && Pslot[s].planes() ? 1 : 0;
   return n;
}

//--------------------------------------------------------------------------------
// No frame anywhere in the pipeline (the plan waits for the last verdicts)
bool TCapApp::pipelineIdle(void) const
{
   for (int s = 0; s < appSlots; s++)
      if (Pslot[s].state != (BYTE)ssFree)
         return false;
   return true;
}

//--------------------------------------------------------------------------------
// The bin still takes candidates: a frame of it waits in the pipeline, the keyframe worker not yet on it
bool TCapApp::binLive(int band, int bin) const
{
   for (int s = 0; s < appSlots; s++)
   {
      const TKeySlot &k = Pslot[s];

      if (k.state != (BYTE)ssFree && k.state != (BYTE)ssFinal && !k.direct && k.band == band && k.bin == bin)
         return true;
   }
   return false;
}

/*--------------------------------------------------------------------------------
   The sharpness of a slot on its focus regions only (user, 2026-09-28): each region measured on its
   own, grown around its center to what the blur tiles need, the median over all their textured
   tiles; no regions: the whole frame.
  --------------------------------------------------------------------------------*/
static void focusBlur(const TKeySlot &k, TBlurResult &out)
{
   int   w = (int)k.rec.width,
         h = (int)k.rec.height,
         n = 0,
         tiles = 0;
   float v[(int)appFocusRects*(int)blurGridSide*(int)blurGridSide],
         sharp = NAN;

   out = TBlurResult();
   if (k.focusCount <= 0)
   {
      blurMeasure(k.planes(), w, h, w, out);
      return;
   }
   for (int r = 0; r < k.focusCount && r < appFocusRects; r++)
   {
      const float *q = k.focus + 4*r;
      int          cx = (int)(0.5f*(q[0] + q[2])*(float)w),
                   cy = (int)(0.5f*(q[1] + q[3])*(float)h),
                   rw = (int)((q[2] - q[0])*(float)w),
                   rh = (int)((q[3] - q[1])*(float)h);

      rw = rw < appFocusMeasurePx ? appFocusMeasurePx : rw;
      rh = rh < appFocusMeasurePx ? appFocusMeasurePx : rh;
      rw = rw > w ? w : rw;
      rh = rh > h ? h : rh;

      int         x0 = cx - rw/2,
                  y0 = cy - rh/2;
      TBlurResult part;

      x0 = x0 < 0 ? 0 : (x0 + rw > w ? w - rw : x0);
      y0 = y0 < 0 ? 0 : (y0 + rh > h ? h - rh : y0);
      blurMeasure(k.planes() + (size_t)y0*w + x0, rw, rh, w, part);
      tiles += part.tiles;
      for (int t = 0; t < blurGridSide*blurGridSide; t++)
         if (!isnan(part.tilePx[t]))
         {
            v[n++] = part.tilePx[t];
            sharp = isnan(sharp) || part.tilePx[t] < sharp ? part.tilePx[t] : sharp;
         }
   }
   for (int i = 1; i < n; i++) // the median
   {
      float x = v[i];
      int   j = i - 1;

      while (j >= 0 && v[j] > x)
      {
         v[j + 1] = v[j];
         j--;
      }
      v[j + 1] = x;
   }
   for (int t = 0; t < blurGridSide*blurGridSide; t++)
      out.tilePx[t] = t < n ? v[t] : NAN;
   out.tiles = tiles;
   out.textured = n;
   out.sharpPx = sharp;
   out.medianPx = !n ? NAN : (n%2 ? v[n/2] : 0.5f*(v[n/2 - 1] + v[n/2]));
}

//--------------------------------------------------------------------------------
void TCapApp::EncodePending(void)
{
   while (encodeOne())
      ;
}

/*--------------------------------------------------------------------------------
   The focus worker (user, 2026-09-28): each candidate, oldest first, its focus measured (the blur
   of the raw luma) and compared with the frame its bin holds - none held is focus 0: the first
   frame always enters; the sharper one stays, the other's slot is freed. A bin the keyframe worker
   has begun (its slot final, set under the mutex) is no longer updated. What a bin holds goes on
   to the keyframe worker once the pose leaves it; a door or floor-view shot goes on as it is.
  --------------------------------------------------------------------------------*/
void TCapApp::FocusPending(void)
{
   for (;;)
   {
      int   id = -1;
      QWORD started = Pport->SensorClockNs();

      {
         TMutexLock lock(Pmutex, thisInfo);

         for (int s = 0; s < appSlots; s++)
            if (Pslot[s].state == (BYTE)ssCandidate && (id < 0 || Pslot[s].stamp < Pslot[id].stamp))
               id = s;
         if (id >= 0)
            Pslot[id].state = (BYTE)ssMeasuring;
      }
      if (id >= 0) // the slot stays put while measuring: no lock
      {
         TKeySlot &k = Pslot[id];

         focusBlur(k, k.blur);
      }
      {
         TMutexLock lock(Pmutex, thisInfo);

         if (id >= 0)
         {
            TKeySlot &k = Pslot[id];
            int       rival = -1;
            bool      locked = false; // the keyframe worker began the bin: it stays as it is

            for (int s = 0; s < appSlots && !k.direct; s++)
            {
               const TKeySlot &o = Pslot[s];

               if (s == id || o.direct || o.band != k.band || o.bin != k.bin)
                  continue;
               if (o.state == (BYTE)ssFinal)
                  locked = true;
               else if (o.state == (BYTE)ssHeld || o.state == (BYTE)ssChosen)
                  rival = s;
            }

            float px = isnan(k.blur.medianPx) ? 1e8f : k.blur.medianPx,
                  heldPx = rival >= 0 && !isnan(Pslot[rival].blur.medianPx) ? Pslot[rival].blur.medianPx : 1e8f;

            if (k.direct)
               k.state = (BYTE)ssHeld; // passed on below
            else if (locked || (rival >= 0 && px >= heldPx))
               k.state = (BYTE)ssFree;
            else if (rival < 0)
               k.state = (BYTE)ssHeld;
            else
            {
               k.state = Pslot[rival].state; // takes the rival's place, in the keyframe worker's queue too
               for (int i = 0; i < PchosenCount; i++)
                  if (Pchosen[(PchosenHead + i)%appSlots] == rival)
                     Pchosen[(PchosenHead + i)%appSlots] = id;
               Pslot[rival].state = (BYTE)ssFree;
            }
            PfocusSec = 0.8f*PfocusSec + 0.2f*(float)(Pport->SensorClockNs() - started)*1e-9f; // the capture cadence
         }

         bool passed = false;

         for (int s = 0; s < appSlots; s++) // the bins the pose left go on
         {
            TKeySlot &k = Pslot[s];

            if (k.state == (BYTE)ssHeld && PchosenCount < appSlots && (k.direct || k.band != PcurBand || k.bin != PcurBin
                                                                   || PlastFrameNs > k.stamp + (QWORD)appHoldMs*1000000u))
            {
               k.state = (BYTE)ssChosen;
               Pchosen[(PchosenHead + PchosenCount)%appSlots] = s;
               PchosenCount++;
               passed = true;
            }
         }
         if (passed)
            Pworker.Post();
         if (id < 0)
            return;
      }
   }
}

//--------------------------------------------------------------------------------
// One chosen frame through the final measures, the storage and the bins' state; false when none waits
bool TCapApp::encodeOne(void)
{
   TYUVImage     img;
   TImageRecord  rec;
   TFrameMeta    meta;
   TVanishResult vr;
   TVanishRecord vrec = {};
   TVanishConfig tuned = PvanishCfg;
   TBlurResult   blur;
   QWORD         stamp;
   int           slot;
   bool          measured,
                 alert = false; // this frame turned its bin orange or left a gap in its band

   {
      TMutexLock lock(Pmutex, thisInfo);

      if (!PchosenCount)
         return false;
      slot = Pchosen[PchosenHead];
      PchosenHead = (PchosenHead + 1)%appSlots;
      PchosenCount--;

      TKeySlot &k = Pslot[slot];
      int       cw = ((int)k.rec.width + 1)/2,
                ch = ((int)k.rec.height + 1)/2;

      k.state = (BYTE)ssFinal;
      img.width = (int)k.rec.width;
      img.height = (int)k.rec.height;
      img.y = k.planes();
      img.yStride = img.width;
      img.u = img.y + (size_t)img.width*img.height;
      img.v = img.u + (size_t)cw*ch;
      img.uvRowStride = cw;
      img.uvPixelStride = 1;
      rec = k.rec;
      meta = k.meta;
      stamp = k.stamp;
      blur = k.blur; // the focus worker's measure

      TEXIFInfo exif = {};

      snprintf(exif.model, sizeof(exif.model), "%s", PdevModel);
      snprintf(exif.software, sizeof(exif.software), "Sorena LiDAR 0.1.0");
      exif.wallNs = meta.wallNs;
      exif.orientation = exifOrientation(meta.cameraToWorld);
      exif.focalMm = meta.focalMm;
      exif.width = rec.width;
      exif.height = rec.height;
      exif.hasGPS = meta.horizAccMm != 0u;
      exif.latE7 = meta.latE7;
      exif.lonE7 = meta.lonE7;
      exif.altMm = meta.altMm;
      Pexif.Clear();
      exifBuild(exif, Pexif);
   }

   // the slot stays put while it is final: detect without holding the camera thread
   PtiltBias.Apply(tuned);
   measured = vanishDetect(img.y, img.width, img.height, img.yStride, rec.intr, rec.cameraToWorld, tuned, vr,
                           &Pedges);
   if (measured)
      PtiltBias.Add(vr);

   /* doors on the walls the frame shows (both when aimed at a corner: a door on the wall seen sideways is only upright
      in that wall's frontal view): candidates for the door station, which the operator confirms */
   TDoor     doors[doorMaxDoors];
   float     doorHeading[doorMaxDoors],
             doorHorizon[doorMaxDoors],  // the frontal view each door was found in: its level row and focal
             doorFocal[doorMaxDoors];
   TVec3     doorWallN[doorMaxDoors],    // and that wall's normal (camera axes)
             doorUp = {},
             doorN[2] = {};
   int       doorsFound = 0,
             walls = measured && PdoorBuf() ? doorFrameWalls(vr, doorUp, doorN) : 0;
   size_t    plane = (size_t)doorViewW*doorViewH;
   TDoorView view = { PdoorBuf(), PdoorBuf() + plane, PdoorBuf() + 2*plane, PdoorBuf() + 3*plane, 0.f, 0.f, 0.f };

   for (int k = 0; k < walls && doorsFound < doorMaxDoors; k++)
   {
      TDoor found[doorMaxDoors];
      int   n;

      doorFrontal(img, rec.intr, doorUp, doorN[k], view);
      n = doorDetect(view, found, doorMaxDoors);
      for (int i = 0; i < n && doorsFound < doorMaxDoors; i++)
      {
         doors[doorsFound] = found[i];
         doorHeading[doorsFound] = doorColumnHeadingDeg(view, doorUp, doorN[k], rec.cameraToWorld,
                                                        0.5f*(found[i].leftCol + found[i].rightCol));
         doorHorizon[doorsFound] = view.horizonRow;
         doorFocal[doorsFound] = view.focalPx;
         doorWallN[doorsFound] = doorN[k];
         doorsFound++;
      }
   }
   {
      TMutexLock lock(Pmutex, thisInfo);
      float      dev = NAN;
      bool       sameRoom = ProomOpen && ProomIndex == meta.roomIndex,
                 doorFrame = meta.stationKind == (BYTE)skDoor;

      meta.blurPx = blur.medianPx;
      meta.blurMinPx = blur.sharpPx;
      for (int i = 0; i < doorsFound && sameRoom && !doorFrame; i++)
         addDoor(doorHeading[i], doors[i].creaseOverHead, meta.stationIndex);
      if (doorFrame && sameRoom && Pphase == rpDoor && meta.stationIndex == (BYTE)Pstation.index)
         doorShot(doors, doorHeading, doorHorizon, doorFocal, doorWallN, doorsFound, meta, rec.cameraToWorld);
      if (sameRoom && !isnan(blur.medianPx))
         addRoomBlur(blur.medianPx);
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

      // a floor-view frame counts for the floor view only, a fan frame for the fan only
      bool floorFrame = meta.stationKind == (BYTE)skCorner && meta.spinBand == (BYTE)appFloorBand;
      int  band = floorFrame ? 0 : (int)meta.spinBand;

      if (sameRoom && !doorFrame && meta.stationIndex == (BYTE)Pstation.index && floorFrame == PfloorView && band < spinMaxBands
          && meta.spinBin < spinMaxBins)
      {
         int  bin = meta.spinBin;
         bool sharper = isnan(PbinBlur[band][bin]) || (!isnan(blur.medianPx) && blur.medianPx < PbinBlur[band][bin]);

         /* a green bin retaken for sharpness keeps its photo unless the new one is sharper (user, 2026-09-27:
            "uma nova captura, mesmo verde, com borrão menor deve substituir a anterior") */
         TElectRecord elect = {};

         elect.roomIndex = meta.roomIndex;
         elect.stationIndex = meta.stationIndex;
         elect.band = (BYTE)band;
         elect.bin = (BYTE)bin;
         if (!PbinDone[band][bin] || PbinOrange[band][bin] || sharper)
         {
            elect.electedNs = stamp; // this photo is the bin's now; the one before it (if any) is superseded
            elect.supersededNs = PbinDone[band][bin] ? PbinStamp[band][bin] : 0u;
            PbinStamp[band][bin] = stamp;
            PbinAxis[band][bin] = measured ? vr.roomAxisDeg : NAN;
            PbinBlur[band][bin] = blur.medianPx;
            PbinDone[band][bin] = true;

            /* the verdict is kept as it came: green is final; orange is accepted but always open, the bin takes a
               new photo whenever aimed at again. Off the axes (not on the floor view: a diagonal tile floor fills
               it) or blurred beyond the floor: autofocus still converging, a shaken hand */
            PbinOrange[band][bin] = (!floorFrame && binOffAxis(band, bin))
                                    || (!isnan(blur.medianPx) && blur.medianPx > blurFloorPx(cBlurOrangeRel, cBlurMaxPx));
            alert = PbinOrange[band][bin];
         }
         else
         {
            elect.electedNs = PbinStamp[band][bin]; // a retake that was no sharper: it loses to the bin's photo
            elect.supersededNs = stamp;
         }

         /* a gap: an empty bin between two done ones of the band (around the whole circle on the center spin, along
            the fan on a corner) - the turn skipped it; signaled once (user, 2026-09-28: vibrate on orange or a gap) */
         int  bins = Pspin ? Pspin->HeadingBins() : 0;
         bool ring = meta.stationKind == (BYTE)skCenter;

         for (int b = 0; b < bins && !floorFrame; b++)
         {
            int prev = ring ? (b + bins - 1)%bins : b - 1,
                next = ring ? (b + 1)%bins : b + 1;

            if (PbinDone[band][b] || PbinGap[band][b] || prev < 0 || next >= bins)
               continue;
            if (PbinDone[band][prev] && PbinDone[band][next])
            {
               PbinGap[band][b] = true;
               alert = true;
            }
         }
         Psession.WriteElect(stamp, elect);
         if (elect.supersededNs && PsupersededCount < appMaxSuperseded)
            Psuperseded[PsupersededCount++] = elect.supersededNs;
         if (Pspin && PbinOrange[band][bin])
            Pspin->Reopen(band, bin, true);
         else if (Pspin && PbinBlur[band][bin] > blurFloorPx(cBlurRetakeRel, cBlurGoodPx) && PbinTries[band][bin] < appBlurTries)
         {
            PbinTries[band][bin]++;
            Pspin->Reopen(band, bin); // still green: a steadier look at it may bring a sharper photo
         }
      }
      if (measured && sameRoom) // the floor plan: center spin for the walls, every station for the camera height
         Playout.AddFrame(rec.cameraToWorld, vr, Pedges, PtiltBias.Bias(),
                          meta.stationKind == (BYTE)skCenter ? 0 : (int)meta.stationIndex);
      if (measured && sameRoom && meta.stationKind == (BYTE)skCenter)
         updateGuides();
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

      Pslot[slot].state = (BYTE)ssFree;
      if (alert)
         Palert = true;
      checkComplete(); // the verdict of this frame may be the last one the station waited for
      if (PplanWanted && pipelineIdle())
         solvePlan(); // the keyframe that closed the center spin is in: the plan can be made
   }
   Pport->RequestPaint();
   return true;
}

/*--------------------------------------------------------------------------------
   Focus once per station, then held: the Moto's fixed 0.5 D was an APPROXIMATE calibration and
   left whole rooms 4.5-6 px blurred (093519). No keyframe is kept while the lens moves.
   The region follows what the pose is after (user, 2026-09-27): toward the ceiling, the triangle of
   the upright view touching both top corners and the center (the creases sit at the walls' distance,
   almost never hidden); toward the floor, the same triangle upside down; level, a horizontal strip
   across the middle (user, 2026-09-28). A triangle goes as three bands, narrower toward its apex,
   the middle one first (a camera that takes one region keeps that). The focus worker measures the
   sharpness on the same regions, the rest of the image ignored (user, 2026-09-28).
  --------------------------------------------------------------------------------*/
void TCapApp::requestFocus(TFocusAim aim)
{
   float rects[4*appFocusRects];
   int   count = focusRects(aim, rects);

   PfocusAim = aim;
   PfocusTries++;
   Pfocusing = PcamReady && Pport->Autofocus(rects, count);
   PfocusNs = Pport->SensorClockNs();
}

//--------------------------------------------------------------------------------
// The focus regions of an aim, normalized native image rectangles (x0, y0, x1, y1 each); how many
int TCapApp::focusRects(TFocusAim aim, float *rects) const
{
   const float band[3][2] = { { 0.17f, 0.32f }, { 0.02f, 0.17f }, { 0.32f, 0.47f } }; // from the anchoring edge
   int         rot = ((Pcam.sensorRotDeg%360) + 360)%360,
               count = aim == faLevel ? 1 : 3;

   for (int i = 0; i < count; i++)
   {
      float v0 = aim == faFloor ? 1.f - band[i][1] : band[i][0],
            v1 = aim == faFloor ? 1.f - band[i][0] : band[i][1],
            half = 0.5f*(1.f - 0.5f*(band[i][0] + band[i][1])/cFocusApexV), // the triangle's half width there
            u0 = 0.5f - half,
            u1 = 0.5f + half;

      if (aim == faLevel)
      {
         u0 = cFocusLevelSide;
         u1 = 1.f - cFocusLevelSide;
         v0 = 0.5f - cFocusLevelHalf;
         v1 = 0.5f + cFocusLevelHalf;
      }

      int base = 4*i;

      if (rot == 90) // upright (u right, v down) is the native image turned clockwise
      {
         rects[base + 0] = v0;
         rects[base + 1] = 1.f - u1;
         rects[base + 2] = v1;
         rects[base + 3] = 1.f - u0;
      }
      else if (rot == 180)
      {
         rects[base + 0] = 1.f - u1;
         rects[base + 1] = 1.f - v1;
         rects[base + 2] = 1.f - u0;
         rects[base + 3] = 1.f - v0;
      }
      else if (rot == 270)
      {
         rects[base + 0] = 1.f - v1;
         rects[base + 1] = u0;
         rects[base + 2] = 1.f - v0;
         rects[base + 3] = u1;
      }
      else
      {
         rects[base + 0] = u0;
         rects[base + 1] = v0;
         rects[base + 2] = u1;
         rects[base + 3] = v1;
      }
   }
   return count;
}

/*--------------------------------------------------------------------------------
   A focus run for this aim, held until the pose reaches its band (OnFrame starts it): run at once, it
   measured whatever the phone still looked at - the floor spin focused while the camera still faced
   the ceiling, and came out blurred throughout (153013). No keyframe is kept meanwhile.
  --------------------------------------------------------------------------------*/
void TCapApp::wantFocus(TFocusAim aim)
{
   PfocusWant = aim;
   PfocusPending = true;
   PfocusTries = 0;
}

//--------------------------------------------------------------------------------
// A lens settled farther than a room or nearer than arm's length locked on the wrong thing: one more run
void TCapApp::OnFocus(bool locked, float diopters)
{
   TMutexLock lock(Pmutex, thisInfo);
   char       msg[64];
   bool       odd = !(diopters >= cFocusMinDiopters && diopters <= cFocusMaxDiopters); // NaN too

   snprintf(msg, sizeof(msg), "focus %s at %.2f D (try %d)", locked ? "locked" : "held unlocked", diopters, PfocusTries);
   Pport->Log(msg);
   Pfocusing = false;
   if (odd && PfocusTries < appFocusTries)
      requestFocus(PfocusAim);
   Pport->RequestPaint();
}

//--------------------------------------------------------------------------------
void TCapApp::OnFrame(const TCamFrame &frame)
{
   TMutexLock lock(Pmutex, thisInfo);
   TAttitude  a;

   if (Palert) // the encoder's verdict asks for a look back (JNI calls stay on this thread)
   {
      Palert = false;
      Pport->Vibrate(appAlertMs);
   }
   if (!PcamReady || frame.yuv.width != Pcam.width || frame.yuv.height != Pcam.height)
      return;
   updatePreview(frame);
   if (PlastFrameNs && frame.stampNs > PlastFrameNs) // the camera's native rate, smoothed: the capture cadence's base
   {
      float dt = (float)(frame.stampNs - PlastFrameNs)*1e-9f;

      if (dt > 1e-3f && dt < 1.f)
         PcamFps = 0.95f*PcamFps + 0.05f/dt;
   }
   PlastFrameNs = frame.stampNs;

   int curBand = -1, // the bin the pose is on (none off a spin: what the bins hold goes on)
       curBin = -1;

   if ((Pphase == rpCenter || Pphase == rpCorner) && Pspin && attitudeAt(frame.stampNs, a))
   {
      TMat4 pose = poseOf(a);

      /* until the fan is aimed, the diagonal names the station - only without a plan: with one, the map's eye says
         where to stand (150306: the reflex corner's aim was read as another corner's, and it was asked for again) */
      if (Pphase == rpCorner && !Pspin->Filled() && (!Pplan.valid || PplanSketch))
      {
         int slot = deduceCorner(geomHeadingDeg(pose.Forward()));

         if (slot != PcornerSlot)
         {
            PcornerSlot = slot;
            Pstation.corner = Pplan.valid ? Pplan.stationVertex[slot] : (BYTE)slot;
            Pstation.target = Pplan.valid ? Pplan.targetVertex[slot] : (BYTE)((slot + 2)%4);
         }
      }
      if (Pfocusing && frame.stampNs - PfocusNs > (QWORD)appFocusMaxMs*1000000u)
         Pfocusing = false; // the camera never answered: go on with the lens as it is

      // the center spin moves on to the floor band: focus again, for the floor
      int guided = Pspin->GuidedBand();

      if (Pphase == rpCenter && guided >= 0 && guided != PfocusBand && !Pfocusing)
      {
         PfocusBand = guided;
         PfocusTries = 0;
         wantFocus(Pspin->BandPitchDeg(guided) > 5.f ? faCeiling
                                                     : (Pspin->BandPitchDeg(guided) < -5.f ? faFloor : faLevel));
      }

      // no free slot or focusing: observe only, never mark a bin we cannot save sharp
      Pverdict = Pspin->Offer(frame.stampNs, pose, a.accuracyDeg, freeSlots() > 0 && !Pfocusing && !PfocusPending);
      PposeOk = Pspin->PoseAllowed();

      // a pending focus runs once the pose sits in its band and holds still: the aim is then what will be captured
      if (PfocusPending && !Pfocusing && PposeOk && Pverdict != svTooFast)
      {
         PfocusPending = false;
         requestFocus(PfocusWant);
      }

      /* a kept bin's first frame always enters (silent: only an orange bin or a gap vibrates); while the pose holds on
         a bin still open in the pipeline, more candidates follow at the focus worker's own pace - the keyframe worker
         never slows the camera - the frames between skipped by a Bresenham on the camera's rate (user, 2026-09-28: the
         skip adapts, evenly, nothing wasted), only while appSlotReserve slots stay free (the next bin's first frame
         never waits: no gap for a sharper candidate) */
      if (!PfloorView)
         Pspin->Locate(curBand, curBin);
      if (Pverdict == svKeep)
      {
         PkeepErr = 0.f;
         if (queueKeyframe(frame, pose, a, PfloorView, Pspin->LastKeptBand(), Pspin->LastKeptBin(), PfloorView))
            PfocusWorker.Post();
      }
      else if (curBand >= 0 && binLive(curBand, curBin))
      {
         PkeepErr = fminf(1.f, PkeepErr + 1.f/fmaxf(1.f, PfocusSec*PcamFps)); // frames taken per camera frame
         if (PkeepErr >= 1.f && Pverdict != svTooFast && !Pfocusing && freeSlots() > appSlotReserve
             && queueKeyframe(frame, pose, a, false, curBand, curBin, false))
         {
            PkeepErr -= 1.f;
            PfocusWorker.Post();
         }
      }

      /* tilted down during a corner's fan: aimed at the room's middle it is the floor view, taken now instead of in
         the next step; aimed elsewhere it is red and nothing is kept (user, 2026-09-27) */
      if (Pphase == rpCorner && !PfloorView && Pahead && !PaheadDone && Pverdict == svOffBand
          && Pspin->LastPitchDeg() < Pspin->BandPitchDeg(0))
      {
         Pahead->AimFan(floorViewHeadingDeg());

         TSpinVerdict fv = Pahead->Offer(frame.stampNs, pose, a.accuracyDeg, freeSlots() > 0 && !Pfocusing && !PfocusPending);

         PposeOk = fv != svOffBand && fv != svOutside;
         if (fv == svKeep)
         {
            PaheadDone = true;
            if (queueKeyframe(frame, pose, a, true, -1, -1, true)) // one frame, as it is
               PfocusWorker.Post();
         }
      }
      if (Pverdict == svKeep)
         checkComplete();
   }
   else if (Pphase == rpDoor && attitudeAt(frame.stampNs, a))
   {
      /* the door station: a shot whenever the phone is steady and pitched within the window - the detector decides,
         a door at the image center confirms; the aim view only guides (150306: where the operator stood was not where
         the plan put the eye, the predicted aim never matched and two shots were all the station took) */
      TMat4 pose = poseOf(a);
      float heading = geomHeadingDeg(pose.Forward()),
            pitch = geomPitchDeg(pose.Forward()),
            dt = PdoorPrevNs && frame.stampNs > PdoorPrevNs ? (float)(frame.stampNs - PdoorPrevNs)*1e-9f : 0.f,
            rate = dt > 0.f && dt < 0.5f ? fabsf(geomHeadingDiffDeg(heading, PdoorPrevHeading))/dt : 1e9f;

      PdoorPrevNs = frame.stampNs;
      PdoorPrevHeading = heading;
      if (Pfocusing && frame.stampNs - PfocusNs > (QWORD)appFocusMaxMs*1000000u)
         Pfocusing = false;
      PposeOk = fabsf(pitch) <= cDoorLevelDeg;
      if (PfocusPending && !Pfocusing && PposeOk && rate <= cDoorSteadyDps)
      {
         PfocusPending = false;
         requestFocus(PfocusWant);
      }
      if (PposeOk && rate <= cDoorSteadyDps && pipelineIdle() && !Pfocusing && !PfocusPending && PdoorIdx >= 0
          && frame.stampNs - PdoorShotNs > (QWORD)appDoorShotMs*1000000u)
      {
         PdoorShotNs = frame.stampNs;
         if (queueKeyframe(frame, pose, a, false, -1, -1, true)) // one frame: the detector judges each shot before the next
            PfocusWorker.Post();
         Pport->Vibrate(15);
      }
   }
   if (curBand != PcurBand || curBin != PcurBin) // the pose left a bin: its sharpest goes on to the keyframe worker
   {
      PcurBand = curBand;
      PcurBin = curBin;
      PfocusWorker.Post();
   }
   Pport->RequestPaint();
}

/*--------------------------------------------------------------------------------
   Automatic end of a station: every bin captured, every verdict in and none orange. With orange bins
   left the station stays open - each one retakes when aimed at - until the operator moves on with the
   button (the orange ones are accepted as they are).
  --------------------------------------------------------------------------------*/
void TCapApp::checkComplete(void)
{
   if (!Pspin || PcompleteSignaled || !Pspin->Complete() || orangeBins() || (Pphase != rpCenter && Pphase != rpCorner))
      return;
   for (int b = 0; b < Pspin->BandCount(); b++)
      for (int bin = 0; bin < Pspin->HeadingBins(); bin++)
         if (!PbinDone[b][bin])
            return;
   if (Pphase == rpCorner && !PfloorView && !PaheadDone)
   {
      beginFloorView(); // the fan is in: from the same corner, the floor toward the middle of the room
      return;
   }
   PcompleteSignaled = true;
   Pport->Vibrate(250);
   stationDone(); // automatic: center spin -> corners in the order the plan and the aim choose
}

/*--------------------------------------------------------------------------------
   Second part of a corner station (user, 2026-09-27): the fan is level, frontal to the corner; now
   the camera tilts down toward the middle of the room, so every corner sees the floor tiles in its
   own perspective - the grid behind furniture removal, and a clearly different pose to check.
   Same station, a one-bin tracker aimed at the spin point; its frames carry band 1.
  --------------------------------------------------------------------------------*/
void TCapApp::beginFloorView(void)
{
   TSpinTracker *spin = new TSpinTracker(PfloorCfg);

   spin->AimFan(floorViewHeadingDeg());
   Pspin = spin;
   PfloorView = true;
   memset(PbinDone, 0, sizeof(PbinDone));
   memset(PbinOrange, 0, sizeof(PbinOrange));
   memset(PbinGap, 0, sizeof(PbinGap));
   memset(PbinTries, 0, sizeof(PbinTries));
   Pverdict = svCovered;
   PfocusTries = 0;
   wantFocus(faFloor); // the floor lies at another distance than the fan
   Pport->Vibrate(60);
}

/*--------------------------------------------------------------------------------
   Where the floor view from the current corner aims: the middle of the room nearest to it - its own
   arm of an L, the rectangle's only middle - or the spin point when the plan has none.
  --------------------------------------------------------------------------------*/
TPlanPoint TCapApp::floorViewTarget(void) const
{
   TPlanPoint target = { 0.f, 0.f };

   if (!Pplan.valid || PcornerSlot < 0 || PcornerSlot >= Pplan.stationCount)
      return target;

   const TPlanPoint &c = Pplan.verts[Pplan.stationVertex[PcornerSlot]];
   float             best = 1e9f;

   for (int i = 0; i < Pplan.centerCount; i++)
   {
      float du = Pplan.centers[i].u - c.u,
            dw = Pplan.centers[i].w - c.w,
            d = du*du + dw*dw;

      if (d < best)
      {
         best = d;
         target = Pplan.centers[i];
      }
   }
   return target;
}

//--------------------------------------------------------------------------------
// World heading from the station's corner to its floor-view target
float TCapApp::floorViewHeadingDeg(void) const
{
   if (!Pplan.valid || PcornerSlot < 0 || PcornerSlot >= Pplan.stationCount)
      return geomHeadingDeg(poseOf(Pring[(PringHead + appRingSize - 1)%appRingSize]).Forward());

   const TPlanPoint &c = Pplan.verts[Pplan.stationVertex[PcornerSlot]];
   TPlanPoint        t = floorViewTarget();

   return fmodf(Pplan.axisDeg + atan2f(t.w - c.w, t.u - c.u)*57.29578f + 720.f, 360.f);
}

/*--------------------------------------------------------------------------------
   Corners found so far during the center spin: the walls the ceiling creases give (the plan solved on
   what is in, complete or not), and wherever a wall across u and one across w reach each other, a
   corner. Concave when both walls run from it back toward the spin point, convex (jutting into the
   room) when both run away from it. Worker thread, under the app lock.
  --------------------------------------------------------------------------------*/
void TCapApp::updateGuides(void)
{
   Playout.Solve(Playout.AnchorDeg(), cCeilingM, PguidePlan);
   PguideCount = 0;
   for (int i = 0; i < PguidePlan.wallCount && i < layoutMaxWalls; i++)
      for (int j = 0; j < PguidePlan.wallCount && j < layoutMaxWalls && PguideCount < appMaxGuides; j++)
      {
         if (PguidePlan.wallKind[i] != 0 || PguidePlan.wallKind[j] != 1)
            continue;

         float u = PguidePlan.wallOffset[i], // wall i: u constant, runs along w
               w = PguidePlan.wallOffset[j]; // wall j: w constant, runs along u

         if (w < PguidePlan.wallA0[i] - cGuideReachM || w > PguidePlan.wallA1[i] + cGuideReachM
             || u < PguidePlan.wallA0[j] - cGuideReachM || u > PguidePlan.wallA1[j] + cGuideReachM)
            continue;

         float midW = 0.5f*(PguidePlan.wallA0[i] + PguidePlan.wallA1[i]),
               midU = 0.5f*(PguidePlan.wallA0[j] + PguidePlan.wallA1[j]);
         bool  backW = (midW - w)*w < 0.f, // wall i heads from the corner toward the spin point's side
               backU = (midU - u)*u < 0.f;

         PguideDeg[PguideCount] = fmodf(PguidePlan.axisDeg + atan2f(w, u)*57.29578f + 720.f, 360.f);
         PguideAt[PguideCount].u = u;
         PguideAt[PguideCount].w = w;
         PguideConvex[PguideCount] = !backW && !backU;
         PguideCount++;
      }
}

/*--------------------------------------------------------------------------------
   A door found on a keyframe joins the candidate seen from the same station at a heading this close,
   or becomes a new one; the crease-over-head ratio (a level frame with the crease) accumulates.
  --------------------------------------------------------------------------------*/
void TCapApp::addDoor(float headingDeg, float creaseOverHead, DWORD station)
{
   int k = -1;

   for (int i = 0; i < PdoorCount && k < 0; i++)
      if (PdoorCand[i].station == station && fabsf(geomHeadingDiffDeg(PdoorCand[i].headingDeg, headingDeg)) <= cDoorMergeDeg)
         k = i;
   if (k < 0)
   {
      if (PdoorCount >= appMaxDoors)
         return;
      k = PdoorCount++;
      PdoorCand[k] = TDoorCandidate();
      PdoorCand[k].headingDeg = headingDeg;
      PdoorCand[k].station = station;
      PdoorCand[k].obsU = NAN;
      PdoorCand[k].obsW = NAN;
      if (station == 0u) // the spin point is the plan's origin
      {
         PdoorCand[k].obsU = 0.f;
         PdoorCand[k].obsW = 0.f;
      }
      else if (Pplan.valid && !PplanSketch && Pstation.kind == skCorner && Pstation.index == station
               && Pstation.corner < Pplan.vertexCount)
      {
         PdoorCand[k].obsU = Pplan.verts[Pstation.corner].u;
         PdoorCand[k].obsW = Pplan.verts[Pstation.corner].w;
      }
   }
   PdoorCand[k].views++;
   if (!isnan(creaseOverHead))
   {
      PdoorCand[k].ratioSum += creaseOverHead;
      PdoorCand[k].ratios++;
   }
}

/*--------------------------------------------------------------------------------
   A ray on the plan from (u, w) along (du, dw): the distance to the first wall of the polygon it
   meets (1e9 when none), and that wall's first vertex (edge, optional).
  --------------------------------------------------------------------------------*/
float TCapApp::planRayHit(float u, float w, float du, float dw, int *edge) const
{
   float best = 1e9f;

   if (edge)
      *edge = -1;
   for (int i = 0; i < Pplan.vertexCount; i++)
   {
      const TPlanPoint &a = Pplan.verts[i],
                       &b = Pplan.verts[(i + 1)%Pplan.vertexCount];
      float             eu = b.u - a.u,
                        ew = b.w - a.w,
                        den = du*ew - dw*eu;

      if (fabsf(den) < 1e-6f)
         continue;

      float s = ((a.u - u)*ew - (a.w - w)*eu)/den, // along the ray
            r = ((a.u - u)*dw - (a.w - w)*du)/den; // along the wall

      if (s > 0.2f && r >= 0.f && r <= 1.f && s < best)
      {
         best = s;
         if (edge)
            *edge = i;
      }
   }
   return best;
}

/*--------------------------------------------------------------------------------
   The candidates on the plan: from where each was seen, its heading runs to the first wall of the
   polygon; candidates of different stations landing within cDoorSameM are one door.
  --------------------------------------------------------------------------------*/
int TCapApp::buildPlanDoors(void)
{
   PplanDoorCount = 0;
   if (!Pplan.valid || PplanSketch || Pplan.vertexCount < 3)
      return 0;
   for (int k = 0; k < PdoorCount; k++)
   {
      const TDoorCandidate &c = PdoorCand[k];

      if (isnan(c.obsU))
         continue;

      float t = (c.headingDeg - Pplan.axisDeg)*0.01745329f,
            du = cosf(t),
            dw = sinf(t);
      int   edge = -1;
      float best = planRayHit(c.obsU, c.obsW, du, dw, &edge);

      if (best > 1e8f || edge < 0)
         continue;

      TPlanPoint at = { c.obsU + best*du, c.obsW + best*dw };
      int        same = -1;

      for (int i = 0; i < PplanDoorCount && same < 0; i++)
      {
         float eu = PplanDoors[i].at.u - at.u,
               ew = PplanDoors[i].at.w - at.w;

         if (sqrtf(eu*eu + ew*ew) <= cDoorSameM)
            same = i;
      }
      if (same < 0 && PplanDoorCount < appMaxDoors)
      {
         /* where to shoot it from (user, 2026-09-27: from the spin point the door was seen askew): on the wall's
            inward normal, far enough for the door and its molding to fit a level photo, short of the opposite wall */
         const TPlanPoint &a = Pplan.verts[edge],
                          &b = Pplan.verts[(edge + 1)%Pplan.vertexCount];
         float             eu = b.u - a.u,
                           ew = b.w - a.w,
                           len = fmaxf(1e-3f, sqrtf(eu*eu + ew*ew)),
                           nu = -ew/len,
                           nw = eu/len;

         if (nu*(c.obsU - at.u) + nw*(c.obsW - at.w) < 0.f) // toward where it was seen from: into the room
         {
            nu = -nu;
            nw = -nw;
         }

         float room = planRayHit(at.u + 0.05f*nu, at.w + 0.05f*nw, nu, nw, NULL),
               back = fmaxf(cDoorStandMinM, fminf(cDoorStandM, room - cDoorStandClearM));

         same = PplanDoorCount++;
         PplanDoors[same] = TPlanDoor();
         PplanDoors[same].at = at;
         PplanDoors[same].stand.u = at.u + back*nu;
         PplanDoors[same].stand.w = at.w + back*nw;
         PplanDoors[same].along.u = eu/len;
         PplanDoors[same].along.w = ew/len;
      }
      if (same >= 0)
      {
         PplanDoors[same].ratioSum += c.ratioSum;
         PplanDoors[same].ratios += c.ratios;
      }
   }
   return PplanDoorCount;
}

/*--------------------------------------------------------------------------------
   The door station (user, 2026-09-27): the last one of the room. The map's eye goes in front of each
   door in turn, on its wall's normal (from the last corner a door may stand right beside, 123038; from
   the spin point it is seen askew) - "the eye is my guide in the room". The aim view shows the door
   as the camera should frame it from there; each steady shot goes to the detector, and a door found
   at its center confirms it. "Descartar" drops a false one. Each shot also tells where the operator
   stands (doorShot).
  --------------------------------------------------------------------------------*/
void TCapApp::beginDoors(void)
{
   PdoorEye.u = 0.f; // the spin point until nextDoor puts the eye in front of the first door (the operator follows it)
   PdoorEye.w = 0.f;
   Pstation.event = seBegin;
   Pstation.roomIndex = ProomIndex;
   Pstation.index = PstationCount++;
   Pstation.kind = skDoor;
   Psession.WriteStation(Pport->SensorClockNs(), Pstation);
   Pphase = rpDoor;
   PdoorIdx = -1;
   PdoorShotNs = 0u;
   PposeOk = true;
   nextDoor();
   if (Pphase == rpDoor)
   {
      PfocusTries = 0;
      wantFocus(faLevel);
   }
}

//--------------------------------------------------------------------------------
// The next door to shoot; none left: the room is done
void TCapApp::nextDoor(void)
{
   int next = -1;

   for (int i = 0; i < PplanDoorCount && next < 0; i++)
      if (PplanDoors[i].state == 0u)
         next = i;
   PdoorIdx = next;
   if (next >= 0)
      PdoorEye = PplanDoors[next].stand; // the map's eye moves in front of the door to shoot
   if (next >= 0)
      return;
   finishDoors();
   endStation();
   Pphase = rpWalk; // the station is already closed: finishRoom must not close it again
   finishRoom();
}

//--------------------------------------------------------------------------------
// World heading from where the operator stands to the door being shot
float TCapApp::doorAimDeg(void) const
{
   if (PdoorIdx < 0 || PdoorIdx >= PplanDoorCount)
      return 0.f;

   const TPlanPoint &d = PplanDoors[PdoorIdx].at;

   return fmodf(Pplan.axisDeg + atan2f(d.w - PdoorEye.w, d.u - PdoorEye.u)*57.29578f + 720.f, 360.f);
}

/*--------------------------------------------------------------------------------
   A door-station keyframe came back from the worker. A door found at the image center confirms the
   door being shot (its ratio joins the door's). Any door found also places the operator (user: "o
   giroscópio mais o teto dão a posição"): in the level frontal view the crease stands at elevation e
   above the camera, so the wall lies (crease - camera) / tan(e) away - both heights known in door
   heads from the door itself - and the operator is that far back from the door along the ray.
  --------------------------------------------------------------------------------*/
void TCapApp::doorShot(const TDoor *doors, const float *headings, const float *horizons, const float *focals,
                       const TVec3 *walls, int found, const TFrameMeta &meta, const TMat4 &pose)
{
   /* the door that fits the one drawn (user, 2026-09-27: "a escala do wireframe da porta deve encaixar na porta vista
      na câmera"): its heading within cDoorFitDeg of where the plan puts it from the eye, and its width - the frontal
      view's columns at the wall's distance from the eye - that of a door */
   int   centered = -1;
   float bestOff = 1e9f;

   if (PdoorIdx >= 0 && PdoorIdx < PplanDoorCount)
   {
      const TPlanDoor &pd = PplanDoors[PdoorIdx];
      float            wallM = fabsf((PdoorEye.u - pd.at.u)*(-pd.along.w) + (PdoorEye.w - pd.at.w)*pd.along.u),
                       aim = doorAimDeg();

      float camH = Pplan.cameraHeightM > 0.5f ? Pplan.cameraHeightM : cWireCameraM;

      for (int i = 0; i < found; i++)
      {
         float off = fabsf(geomHeadingDiffDeg(headings[i], aim)),
               widthM = (doors[i].rightCol - doors[i].leftCol)*wallM/focals[i],
               tanHead = (horizons[i] - doors[i].headRow)/focals[i],
               tanFoot = (doors[i].floorRow - horizons[i])/focals[i],
               byHead = tanHead > 0.02f ? (cDoorHeadM - camH)/tanHead : NAN,         // the door's distance, by its head
               byFoot = !doors[i].footCut && tanFoot > 0.02f ? camH/tanFoot : NAN,  // and by its foot
               far = isnan(byFoot) ? byHead : (isnan(byHead) ? byFoot : 0.5f*(byHead + byFoot));

         /* and at the wall's distance: a door seen THROUGH the opening, in the hall beyond, is farther (153013 frame
            102: the far door framed, its ratio 1.51 instead of ~1.32) */
         if (isnan(far) || fabsf(far - wallM) > cDoorFitDepthFrac*wallM)
            continue;
         if (off <= cDoorFitDeg && widthM >= cDoorFitMinM && widthM <= cDoorFitMaxM && off < bestOff)
         {
            bestOff = off;
            centered = i;
         }
      }
   }
   for (int i = 0; i < found; i++)
   {
      const TDoor &d = doors[i];

      if (isnan(d.creaseRow) || isnan(d.creaseOverHead))
         continue;

      float e = atan2f(horizons[i] - d.creaseRow, focals[i]),
            wallM = cDoorHeadM*(d.creaseOverHead - d.cameraOverHead)/tanf(e), // perpendicular distance to the wall
            normalDeg = geomHeadingDeg(pose.RotateVector(walls[i])),
            cosPhi = cosf((headings[i] - normalDeg)*0.01745329f),
            t = (headings[i] - Pplan.axisDeg)*0.01745329f;

      if (!(e > 0.05f) || fabsf(cosPhi) < 0.3f || !(wallM > 0.3f && wallM < 12.f))
         continue;

      // only the centered door is surely the one being shot: it places the operator
      int   k = i == centered ? PdoorIdx : -1;
      float range = wallM/fabsf(cosPhi);

      if (k < 0 || k >= PplanDoorCount)
         continue;
      PdoorEye.u = PplanDoors[k].at.u - range*cosf(t);
      PdoorEye.w = PplanDoors[k].at.w - range*sinf(t);
   }
   if (centered < 0 || PdoorIdx < 0 || PdoorIdx >= PplanDoorCount)
      return;

   /* the ratio by angles: in the level frontal view every point of the wall lies at the same distance, so a height
      above the floor is D (tan e - tan e_foot); head, foot and ceiling line may come from different shots of one
      spot (a close door never fits whole: 132058 frame 53 had the foot, 37/38 the molding) */
   TPlanDoor   &pd = PplanDoors[PdoorIdx];
   const TDoor &d = doors[centered];
   bool         first = pd.state != (BYTE)dsConfirmed;

   pd.tanHead += (horizons[centered] - d.headRow)/focals[centered];
   pd.nHead++;
   if (!d.footCut)
   {
      pd.tanFoot += (horizons[centered] - d.floorRow)/focals[centered];
      pd.nFoot++;
   }
   if (!isnan(d.creaseRow))
   {
      pd.tanCrease += (horizons[centered] - d.creaseRow)/focals[centered];
      pd.nCrease++;
   }
   pd.state = (BYTE)dsConfirmed;
   if (first)
   {
      pd.imageNs = meta.sensorNs;
      Pport->Vibrate(120);
   }
   if (!pd.nFoot || !pd.nCrease)
      return; // confirmed; the aim view asks for the part still missing

   float th = pd.tanHead/(float)pd.nHead,
         tf = pd.tanFoot/(float)pd.nFoot,
         tc = pd.tanCrease/(float)pd.nCrease;

   float ratio = th - tf > 0.05f ? (tc - tf)/(th - tf) : NAN;

   // a ceiling line off every building's (a misread) never becomes the ruler: the door stays confirmed, unmeasured
   if (ratio*cDoorHeadM >= cRulerMinM && ratio*cDoorHeadM <= cRulerMaxM)
   {
      pd.ratioSum = ratio;
      pd.ratios = 1;
   }
   Pport->Vibrate(200);
   nextDoor();
}

/*--------------------------------------------------------------------------------
   The door station is over (every door shot or dropped, or the room ended): the confirmed doors'
   ratios measure the ceiling line (the head is 2.10 m), for this room and - one ceiling height in a
   building (user, 2026-09-27) - for the whole property, the next rooms solving at it. The plan is
   scaled to it and recorded again; every door gets its record in that final scale.
  --------------------------------------------------------------------------------*/
void TCapApp::finishDoors(void)
{
   float before = Pplan.ceilingM;

   for (int i = 0; i < PplanDoorCount; i++)
      if (PplanDoors[i].state == (BYTE)dsConfirmed && PplanDoors[i].ratios > 0)
      {
         PpropRatioSum += PplanDoors[i].ratioSum;
         PpropRatios += PplanDoors[i].ratios;
      }
   if (PpropRatios > 0)
      PpropCeilingM = cDoorHeadM*PpropRatioSum/(float)PpropRatios;

   bool scaled = !isnan(PpropCeilingM) && Pplan.valid && !PplanSketch && before > 0.f;

   if (scaled)
   {
      float f = PpropCeilingM/before;

      layoutScalePlan(Pplan, PpropCeilingM);
      for (int i = 0; i < PplanDoorCount; i++)
      {
         PplanDoors[i].at.u *= f;
         PplanDoors[i].at.w *= f;
      }
      PdoorEye.u *= f;
      PdoorEye.w *= f;
      writeLayout(true);
   }
   for (int i = 0; i < PplanDoorCount; i++)
   {
      TDoorRecord rec = {};

      rec.roomIndex = ProomIndex;
      rec.index = (BYTE)i;
      rec.state = (TDoorState)PplanDoors[i].state;
      rec.u = PplanDoors[i].at.u;
      rec.w = PplanDoors[i].at.w;
      rec.ratio = PplanDoors[i].ratios ? PplanDoors[i].ratioSum/(float)PplanDoors[i].ratios : NAN;
      rec.ratios = PplanDoors[i].ratios;
      rec.imageNs = PplanDoors[i].imageNs;
      Psession.WriteDoor(Pport->SensorClockNs(), rec);
   }

   char msg[96];

   snprintf(msg, sizeof(msg), "doors: %d, ceiling line %.2f m (%d ratios), plan %s", PplanDoorCount, PpropCeilingM,
            PpropRatios, scaled ? "scaled" : "as solved");
   Pport->Log(msg);
}

//--------------------------------------------------------------------------------
void TCapApp::addRoomBlur(float px)
{
   ProomBlur[ProomBlurCount%appBlurHistory] = px;
   ProomBlurCount++;
}

/*--------------------------------------------------------------------------------
   A blur floor that follows the camera: rel times the median blur of the latest keyframes,
   never below minPx. A soft lens reads soft everywhere, and only a frame well above its own median
   is a lost focus or a shaken hand. Until appBlurMinSamples keyframes are in, no verdict.
  --------------------------------------------------------------------------------*/
float TCapApp::blurFloorPx(float rel, float minPx) const
{
   float v[appBlurHistory];
   int   n = ProomBlurCount < appBlurHistory ? ProomBlurCount : appBlurHistory;

   if (n < appBlurMinSamples)
      return 1e9f;
   for (int i = 0; i < n; i++)
   {
      float x = ProomBlur[i];
      int   k = i - 1;

      while (k >= 0 && v[k] > x)
      {
         v[k + 1] = v[k];
         k--;
      }
      v[k + 1] = x;
   }

   float median = n%2 ? v[n/2] : 0.5f*(v[n/2 - 1] + v[n/2]);

   return fmaxf(minPx, rel*median);
}

//--------------------------------------------------------------------------------
int TCapApp::orangeBins(void) const
{
   const TSpinTracker *spin = Pspin();
   int                 n = 0;

   for (int b = 0; spin && b < spin->BandCount(); b++)
      for (int bin = 0; bin < spin->HeadingBins(); bin++)
         n += spin->BinFilled(b, bin) && PbinOrange[b][bin] ? 1 : 0;
   return n;
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
   PpropCeilingM = NAN; // a new property: its doors will measure its ceiling
   PpropRatioSum = 0.f;
   PpropRatios = 0;
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
   PdoorCount = 0;
   PguideCount = 0;
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
   memset(PbinOrange, 0, sizeof(PbinOrange));
   memset(PbinGap, 0, sizeof(PbinGap));
   memset(PbinTries, 0, sizeof(PbinTries));
   PcompleteSignaled = false;
   PfloorView = false;
   PaheadDone = false;
   PposeOk = true;
   Pahead = kind == skCorner ? new TSpinTracker(PfloorCfg) : NULL; // the floor view, should it come early
   Pverdict = svCovered;
   PfocusTries = 0;
   PfocusBand = kind == skCenter ? PcenterCfg.bandCount - 1 : 0; // the center spin starts at the ceiling
   wantFocus(kind == skCenter && PcenterCfg.bandCount > 1 ? faCeiling : faLevel);
}

//--------------------------------------------------------------------------------
void TCapApp::endStation(void)
{
   if (Pphase != rpCenter && Pphase != rpCorner && Pphase != rpDoor)
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
      if (pipelineIdle())
         solvePlan(); // otherwise the worker solves once the closing keyframe is in
   }
   if (!wasCenter && PcornerStep + 1 >= PcornerCount)
   {
      PcornerMask |= 1u << PcornerSlot;
      PcornerStep++;
      if (buildPlanDoors() > 0)
      {
         beginDoors(); // the last station of the room: its doors
         return;
      }
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
   if (Pphase == rpDoor) // ended during the door station: the doors so far still measure
      finishDoors();
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
   PplanWanted = false;

   /* the property's ceiling line, once a door station measured it, is this room's too (one ceiling height in a
      building, user 2026-09-27): it replaces the assumption and no door-head guess may snap it away */
   bool ruler = !isnan(PpropCeilingM);

   if (!Playout.Solve(Playout.AnchorDeg(), ruler ? PpropCeilingM : cCeilingM, Pplan))
      Pplan.valid = false;
   if (ruler && Pplan.valid)
      layoutScalePlan(Pplan, PpropCeilingM);
   PcornerCount = Pplan.valid ? Pplan.stationCount : 4;
   PcornerSlot = nextCorner();
   writeLayout(ruler);
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
      Pplan.centerCount = 1; // the square's only middle: the spin point
      Pplan.centers[0].u = 0.f;
      Pplan.centers[0].w = 0.f;
      Pplan.valid = true;
   }
}

//--------------------------------------------------------------------------------
// The room's plan as a record (its own frame); doorScaled: its scale is a door station's measure
void TCapApp::writeLayout(bool doorScaled)
{
   TLayoutRecord rec = {};

   rec.roomIndex = ProomIndex;
   rec.flags = (BYTE)((Pplan.valid ? lfValid : 0) | (Pplan.complete ? lfComplete : 0)
                      | (Pplan.heightSolved ? lfHeightSolved : 0) | (doorScaled ? lfDoorScaled : 0));
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
}

//--------------------------------------------------------------------------------
void TCapApp::finishProperty(void)
{
   TSessionCounts c = Psession.Counts();

   finishRoom();

   // production: a replaced photo is overwritten - the log drops it once, now (user, 2026-09-27); debug keeps both
   if (!appKeepSuperseded && PsupersededCount)
      Psession.Compact(Psuperseded, PsupersededCount);
   PsupersededCount = 0;
   c = Psession.Counts();
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
      else if (Pphase == rpDoor && PdoorIdx >= 0 && PdoorIdx < PplanDoorCount)
      {
         if (PplanDoors[PdoorIdx].state != (BYTE)dsConfirmed)
            PplanDoors[PdoorIdx].state = (BYTE)dsDropped; // "Descartar": a false candidate; confirmed: "Pular medida"
         nextDoor();
      }
      else if (Pphase == rpWalk)
         beginStation(skCorner); // "arrived at corner N"
      else if (Pphase != rpDoor)
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
   QWORD      now = Pport->SensorClockNs();

   /* a capture cannot be resumed: one stray Back would end it (user, 2026-09-27). The first press only arms,
      a second one within appBackArmMs ends the room (or the property) */
   if ((ProomOpen || Psession.IsOpen()) && (!PbackArmedNs || now - PbackArmedNs > (QWORD)appBackArmMs*1000000u))
   {
      PbackArmedNs = now;
      Pport->RequestPaint();
      return true;
   }
   PbackArmedNs = 0u;
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
   if ((Pfocusing || (PfocusPending && PposeOk)) && (Pphase == rpCenter || Pphase == rpCorner || Pphase == rpDoor))
   {
      snprintf(out, cap, "Focando: segure firme");
      return;
   }
   if (Pphase == rpDoor)
   {
      int left = 0;

      for (int i = 0; i < PplanDoorCount; i++)
         left += PplanDoors[i].state == 0u ? 1 : 0;
      if (!PposeOk)
         snprintf(out, cap, "Deixe a câmera na horizontal");
      else if (!pipelineIdle())
         snprintf(out, cap, "Segure: conferindo a porta");
      else
      {
         const TPlanDoor *d = PdoorIdx >= 0 && PdoorIdx < PplanDoorCount ? &PplanDoors[PdoorIdx] : NULL;

         if (d && d->state == (BYTE)dsConfirmed && !d->nFoot)
            snprintf(out, cap, "Porta confirmada: incline para baixo até o pé dela");
         else if (d && d->state == (BYTE)dsConfirmed && !d->nCrease)
            snprintf(out, cap, "Porta confirmada: incline para cima até a moldura");
         else
            snprintf(out, cap, "No olho laranja, de frente para a porta amarela: mire (%d)", left);
      }
      return;
   }
   if (Pphase == rpWalk)
   {
      if (PplanWanted)
         snprintf(out, cap, "Calculando a planta...");
      else if (Pplan.valid)
         snprintf(out, cap, "Vá ao olho laranja e mire o círculo amarelo (%d de %d)",
                  PcornerStep + 1, PcornerCount);
      else
         snprintf(out, cap, "Vá ao canto %d (sentido horário) e toque em Cheguei", PcornerStep + 1);
      return;
   }
   int orange = orangeBins();

   if ((Pphase == rpCenter || Pphase == rpCorner) && spin->Complete() && orange)
   {
      snprintf(out, cap, "%d em laranja: aponte para refazer ou toque em %s", orange,
               Pphase == rpCenter ? "Ir para os cantos"
                                  : (PcornerStep + 1 >= PcornerCount ? "Concluir último canto" : "Próximo canto"));
      return;
   }
   if (Pphase == rpCorner)
   {
      if (PfloorView) // the second part of the corner: tilted down to the middle of the room
      {
         if (Pverdict == svOffBand)
            snprintf(out, cap, spin->LastPitchDeg() > spin->BandPitchDeg(0) ? "Incline mais para o piso"
                                                                             : "Incline um pouco menos");
         else if (Pverdict == svOutside)
            snprintf(out, cap, "Mire o centro do cômodo (círculo amarelo do mapa)");
         else if (Pverdict == svTooFast)
            snprintf(out, cap, "Pare um instante para fotografar");
         else
            snprintf(out, cap, "Agora mire o centro do cômodo, olhando para o piso");
      }
      else if (Pverdict == svOffBand && spin->LastPitchDeg() < spin->BandPitchDeg(0) && !PaheadDone)
         snprintf(out, cap, PposeOk ? "Segure: foto do piso adiantada" : "Para o piso, mire o centro do cômodo");
      else if (Pverdict == svOffBand)
         snprintf(out, cap, "Deixe a câmera na horizontal, de frente para o canto");
      else if (!spin->Filled())
         snprintf(out, cap, Pplan.valid ? "Mire o círculo amarelo do mapa e pare"
                                        : "Aponte para o canto oposto e pare");
      else if (Pverdict == svOutside)
         snprintf(out, cap, "Volte a mirar o canto oposto");
      else if (Pverdict == svTooFast)
         snprintf(out, cap, "Pare um instante para fotografar");
      else
         snprintf(out, cap, "Varra devagar as duas paredes do canto");
      return;
   }
   if (Pverdict == svOffBand && spin->GuidedBand() >= 0 && spin->BandCount() > 1) // the red view of the center spin
   {
      int    guided = spin->GuidedBand(),
             side = spin->PitchSide(guided, spin->LastPitchDeg());
      float  target = spin->BandPitchDeg(guided);
      LPCSTR what = target > 0.f ? "linha do teto" : (target < 0.f ? "linha do piso" : "paredes");

      // red only for a reason the pitch explains: outside the band's range, toward its nearest edge
      if (side != 0)
      {
         snprintf(out, cap, side < 0 ? "Incline para cima (%s)" : "Incline para baixo (%s)", what);
         return;
      }
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

   if (freeSlots() == 0 && spin->AimedEmpty(aimBand, aimBin))
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
      // a nudge only outside the band's accepted pitches (user, 2026-09-28: inside them it misled the operator)
      float target = spin->BandPitchDeg(best);
      int   side = spin->PitchSide(best, pitch);

      if (side < 0)
         snprintf(out, cap, "Incline para cima (%s)", target > 0.f ? "linha do teto" : "paredes");
      else if (side > 0)
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
   Eye in profile looking along (dx, dy) with three sight lines ahead of it (the user's olho.png):
   the lids meet at the back point, the cornea bulges forward, the pupil is a half disc inside.
   Drawn in a local frame (x forward, y across) scaled by size and turned onto the aim.
  --------------------------------------------------------------------------------*/
void TCapApp::drawEye(TSurface &s, int x, int y, float dx, float dy, int size, DWORD rgba)
{
   static const float cLid[5][2] = { { -1.f, 0.f }, { -0.62f, 0.42f }, { -0.25f, 0.68f }, { 0.f, 0.8f },
                                     { 0.08f, 0.86f } },
                      cCornea[5][2] = { { -0.02f, 0.74f }, { 0.1f, 0.4f }, { 0.14f, 0.f }, { 0.1f, -0.4f },
                                        { -0.02f, -0.74f } },
                      cSight[3][4] = { { 0.45f, 0.36f, 1.05f, 0.6f }, { 0.45f, 0.f, 1.1f, 0.f },
                                       { 0.45f, -0.36f, 1.05f, -0.6f } };
   float len = sqrtf(dx*dx + dy*dy);
   int   width = scaleDp(2);

   if (len < 1e-3f)
      return;

   float fx = dx/len*(float)size,
         fy = dy/len*(float)size,
         ax = -fy,
         ay = fx;

   // local (u forward, v across) -> screen
   auto eyeX = [&](float u, float v)
   {
      return x + (int)(u*fx + v*ax);
   };
   auto eyeY = [&](float u, float v)
   {
      return y + (int)(u*fy + v*ay);
   };

   for (int side = -1; side <= 1; side += 2)
      for (int i = 0; i + 1 < 5; i++)
         canvasLine(s, eyeX(cLid[i][0], side*cLid[i][1]), eyeY(cLid[i][0], side*cLid[i][1]),
                    eyeX(cLid[i + 1][0], side*cLid[i + 1][1]), eyeY(cLid[i + 1][0], side*cLid[i + 1][1]), width, rgba);
   for (int i = 0; i + 1 < 5; i++)
      canvasLine(s, eyeX(cCornea[i][0], cCornea[i][1]), eyeY(cCornea[i][0], cCornea[i][1]),
                 eyeX(cCornea[i + 1][0], cCornea[i + 1][1]), eyeY(cCornea[i + 1][0], cCornea[i + 1][1]), width, rgba);
   for (int i = 0; i < 6; i++) // pupil: the back half of a disc against the cornea
   {
      float a0 = 1.5708f + 3.1416f*(float)i/6.f,
            a1 = 1.5708f + 3.1416f*(float)(i + 1)/6.f;

      canvasLine(s, eyeX(0.1f + 0.3f*cosf(a0), 0.3f*sinf(a0)), eyeY(0.1f + 0.3f*cosf(a0), 0.3f*sinf(a0)),
                 eyeX(0.1f + 0.3f*cosf(a1), 0.3f*sinf(a1)), eyeY(0.1f + 0.3f*cosf(a1), 0.3f*sinf(a1)), width, rgba);
   }
   for (int i = 0; i < 3; i++)
      canvasLine(s, eyeX(cSight[i][0], cSight[i][1]), eyeY(cSight[i][0], cSight[i][1]),
                 eyeX(cSight[i][2], cSight[i][3]), eyeY(cSight[i][2], cSight[i][3]), width, rgba);
}

/*--------------------------------------------------------------------------------
   Cohen-Sutherland: the part of a segment inside the box [x, x + size] x [y, y + size]; false when
   nothing of it is.
  --------------------------------------------------------------------------------*/
static bool clipToBox(int &x0, int &y0, int &x1, int &y1, int x, int y, int size)
{
   auto code = [&](int px, int py)
   {
      return (px < x ? 1 : 0) | (px > x + size ? 2 : 0) | (py < y ? 4 : 0) | (py > y + size ? 8 : 0);
   };
   int  c0 = code(x0, y0),
        c1 = code(x1, y1);

   for (int guard = 0; guard < 8; guard++)
   {
      if (!(c0 | c1))
         return true;
      if (c0 & c1)
         return false;

      int   c = c0 ? c0 : c1;
      float fx = 0.f,
            fy = 0.f,
            dx = (float)(x1 - x0),
            dy = (float)(y1 - y0);

      if (c & 8)
      {
         fy = (float)(y + size);
         fx = (float)x0 + dx*(fy - (float)y0)/dy;
      }
      else if (c & 4)
      {
         fy = (float)y;
         fx = (float)x0 + dx*(fy - (float)y0)/dy;
      }
      else if (c & 2)
      {
         fx = (float)(x + size);
         fy = (float)y0 + dy*(fx - (float)x0)/dx;
      }
      else
      {
         fx = (float)x;
         fy = (float)y0 + dy*(fx - (float)x0)/dx;
      }
      if (c == c0)
      {
         x0 = (int)fx;
         y0 = (int)fy;
         c0 = code(x0, y0);
      }
      else
      {
         x1 = (int)fx;
         y1 = (int)fy;
         c1 = code(x1, y1);
      }
   }
   return false;
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

   /* at a station the map is a navigator's: the eye pinned low in the middle, the room turning about it, so
      what lies above the eye is what the camera sees (user, 2026-09-27) */
   if (PcornerSlot >= 0 && PcornerSlot < Pplan.stationCount && !(PcornerMask & (1u << PcornerSlot)))
   {
      const TPlanPoint &e = Pplan.verts[Pplan.stationVertex[PcornerSlot]];
      float             reach = 0.1f;

      for (int i = 0; i < Pplan.vertexCount; i++)
      {
         float du = Pplan.verts[i].u - e.u,
               dw = Pplan.verts[i].w - e.w;

         reach = fmaxf(reach, sqrtf(du*du + dw*dw));
      }
      midX = e.w*cs + e.u*sn;
      midY = e.u*cs - e.w*sn;
      scale = 0.72f*(float)size/reach;
      cy = y + size*85/100;
   }
   else if (Pphase == rpDoor) // the door station: the eye where the operator was placed (last corner, then each shot)
   {
      const TPlanPoint &e = PdoorEye;
      float             reach = 0.1f;

      for (int i = 0; i < Pplan.vertexCount; i++)
      {
         float du = Pplan.verts[i].u - e.u,
               dw = Pplan.verts[i].w - e.w;

         reach = fmaxf(reach, sqrtf(du*du + dw*dw));
      }
      midX = e.w*cs + e.u*sn;
      midY = e.u*cs - e.w*sn;
      scale = 0.72f*(float)size/reach;
      cy = y + size*85/100;
   }

   for (int i = 0; i < Pplan.vertexCount; i++)
   {
      float u = Pplan.verts[i].u,
            w = Pplan.verts[i].w;

      sx[i] = cx + (int)((w*cs + u*sn - midX)*scale);
      sy[i] = cy - (int)((u*cs - w*sn - midY)*scale);
   }
   // at a station, a pose off its band paints the map red like the grid; the eye keeps its color
   bool  tilted = Pphase == rpCorner && !PposeOk;
   DWORD wall = tilted ? canvasRGBA(235, 60, 50, 255) : canvasRGBA(255, 255, 255, 255);

   canvasFillRect(s, x, y, size, size, tilted ? canvasRGBA(220, 30, 30, 120) : canvasRGBA(0, 0, 0, 170));
   for (int i = 0; i < Pplan.vertexCount; i++)
   {
      int j = (i + 1)%Pplan.vertexCount,
          x0 = sx[i],
          y0 = sy[i],
          x1 = sx[j],
          y1 = sy[j];

      if (clipToBox(x0, y0, x1, y1, x, y, size)) // the room turns about the eye: walls may leave the box
         canvasLine(s, x0, y0, x1, y1, scaleDp(3), wall);
   }

   // the spin point: a small white cross (yellow is the target's color)
   int ox = cx - (int)(midX*scale),
       oy = cy + (int)(midY*scale),
       arm = scaleDp(5);

   canvasLine(s, ox - arm, oy, ox + arm, oy, scaleDp(2), canvasRGBA(255, 255, 255, 200));
   canvasLine(s, ox, oy - arm, ox, oy + arm, scaleDp(2), canvasRGBA(255, 255, 255, 200));

   // stations: green done, white still to go; the current one last, on top of the others on its corner
   for (int i = 0; i < Pplan.stationCount; i++)
   {
      int v = Pplan.stationVertex[i];

      if (PcornerMask & (1u << i))
         canvasFillCircle(s, sx[v], sy[v], scaleDp(8), canvasRGBA(60, 220, 110, 255));
      else if (i != PcornerSlot)
         canvasFillCircle(s, sx[v], sy[v], scaleDp(6), canvasRGBA(255, 255, 255, 220));
   }
   if (PcornerSlot >= 0 && PcornerSlot < Pplan.stationCount && !(PcornerMask & (1u << PcornerSlot)))
   {
      /* orange eye where to stand, looking along a thin dashed orange line at the corner to shoot: a large hollow
         yellow circle (user, 2026-09-27: shapes and colors tell stand from aim at a glance) */
      int         v = Pplan.stationVertex[PcornerSlot],
                  t = Pplan.targetVertex[PcornerSlot],
                  tx = sx[t],
                  ty = sy[t],
                  steps = 12;
      const DWORD eye = canvasRGBA(255, 140, 30, 255);

      if (PfloorView) // the floor view aims at the middle of the corner's own part of the room
      {
         TPlanPoint m = floorViewTarget();

         tx = cx + (int)((m.w*cs + m.u*sn - midX)*scale);
         ty = cy - (int)((m.u*cs - m.w*sn - midY)*scale);
      }
      for (int k = 3; k < steps - 1; k += 2)
         canvasLine(s, sx[v] + (tx - sx[v])*k/steps, sy[v] + (ty - sy[v])*k/steps,
                    sx[v] + (tx - sx[v])*(k + 1)/steps, sy[v] + (ty - sy[v])*(k + 1)/steps, scaleDp(2), eye);
      canvasRing(s, tx, ty, scaleDp(15), scaleDp(3), canvasRGBA(255, 220, 0, 255));
      drawEye(s, sx[v], sy[v], (float)(tx - sx[v]), (float)(ty - sy[v]), scaleDp(12), eye);
   }

   // the doors: white to shoot, green confirmed (dropped ones vanish); the current one a yellow ring, the eye aimed at it
   for (int i = 0; Pphase == rpDoor && i < PplanDoorCount; i++)
   {
      if (PplanDoors[i].state == 2u)
         continue;

      int px = cx + (int)((PplanDoors[i].at.w*cs + PplanDoors[i].at.u*sn - midX)*scale),
          py = cy - (int)((PplanDoors[i].at.u*cs - PplanDoors[i].at.w*sn - midY)*scale);

      canvasFillCircle(s, px, py, scaleDp(6), PplanDoors[i].state == 1u ? canvasRGBA(60, 220, 110, 255)
                                                                        : canvasRGBA(255, 255, 255, 220));
      if (i == PdoorIdx)
      {
         int ex = cx + (int)((PdoorEye.w*cs + PdoorEye.u*sn - midX)*scale),
             ey = cy - (int)((PdoorEye.u*cs - PdoorEye.w*sn - midY)*scale);

         canvasRing(s, px, py, scaleDp(15), scaleDp(3), canvasRGBA(255, 220, 0, 255));
         drawEye(s, ex, ey, (float)(px - ex), (float)(py - ey), scaleDp(12), canvasRGBA(255, 140, 30, 255));
      }
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
/* Color of a captured bin (an empty one is drawn as an outline), from the verdict kept when it came: green is
   final; orange only for a keyframe far off the room axes (a 45-degree floor pattern, a stray line). The fine
   verdict (+-2 degrees, no lines to check) stays in the records, not on screen (user, 2026-09-27). */
DWORD TCapApp::binColor(int band, int bin) const
{
   const TSpinTracker *spin = Pspin();

   if (!spin || !spin->BinFilled(band, bin))
      return canvasRGBA(255, 255, 255, 40);
   return PbinOrange[band][bin] ? canvasRGBA(255, 150, 60, 255) : canvasRGBA(60, 220, 110, 255);
}

/*--------------------------------------------------------------------------------
   The bin's latest frame lies far off the room axes (a stray line, a gyroscope jump). An axis at 45
   degrees is a diagonal tile floor - a real pattern of the room, not a wrong frame (081054: the
   floor band retook one bin nine times) - and stays green.
  --------------------------------------------------------------------------------*/
bool TCapApp::binOffAxis(int band, int bin) const
{
   if (!PbinDone[band][bin] || isnan(PbinAxis[band][bin]) || !PaxisCheck.HasReference())
      return false;

   float dev = fabsf(vanishAxisDiffDeg(PbinAxis[band][bin], PaxisCheck.ReferenceDeg()));

   return dev > cOffAxisShowDeg && dev < 45.f - cDiagonalTolDeg;
}

/*--------------------------------------------------------------------------------
   A camera-space point (x right, y up, looking down -z) on the screen: the pinhole of the native
   image, then the preview's rotation to upright, then canvasBlit's cover scaling. False behind.
  --------------------------------------------------------------------------------*/
bool TCapApp::wireToScreen(const TSurface &s, const TVec3 &pc, float &sx, float &sy) const
{
   if (pc.z > -cWireNearM || !PpreviewH || !Pcam.width)
      return false;

   float nu = Pintr.cx + Pintr.fx*pc.x/-pc.z, // native pixels
         nv = Pintr.cy - Pintr.fy*pc.y/-pc.z,
         nw = (float)Pcam.width,
         nh = (float)Pcam.height,
         u = nu,
         v = nv;
   int   rot = ((Pcam.sensorRotDeg%360) + 360)%360;

   if (rot == 90) // the inverse of updatePreview's turn
   {
      u = nh - 1.f - nv;
      v = nu;
   }
   else if (rot == 270)
   {
      u = nv;
      v = nw - 1.f - nu;
   }
   else if (rot == 180)
   {
      u = nw - 1.f - nu;
      v = nh - 1.f - nv;
   }

   float pw = (rot == 90 || rot == 270) ? nh : nw,
         pu = u*(float)appPreviewW/pw, // preview pixels
         pv = v*(float)appPreviewW/pw,
         stepX = (float)appPreviewW/(float)s.width,
         stepY = (float)PpreviewH/(float)s.height,
         step = stepX < stepY ? stepX : stepY,
         offX = 0.5f*((float)appPreviewW - step*(float)s.width),
         offY = 0.5f*((float)PpreviewH - step*(float)s.height);

   sx = (pu - offX)/step;
   sy = (pv - offY)/step;
   return true;
}

//--------------------------------------------------------------------------------
// A plan point at a height above the camera, as a world offset from the eye (du, dw: plan axes in the world)
static TVec3 wireAt(const TPlanPoint &eye, const TVec3 &du, const TVec3 &dw, float u, float w, float y)
{
   TVec3 p = { (u - eye.u)*du.x + (w - eye.w)*dw.x, y, (u - eye.u)*du.z + (w - eye.w)*dw.z };

   return p;
}

//--------------------------------------------------------------------------------
// One 3D edge (world offsets from the eye) cut at the near plane and drawn
void TCapApp::wireEdge(TSurface &s, const TMat4 &pose, const TVec3 &a, const TVec3 &b, DWORD rgba)
{
   TVec3 ca = { pose.m[0]*a.x + pose.m[1]*a.y + pose.m[2]*a.z, pose.m[4]*a.x + pose.m[5]*a.y + pose.m[6]*a.z,
                pose.m[8]*a.x + pose.m[9]*a.y + pose.m[10]*a.z },
         cb = { pose.m[0]*b.x + pose.m[1]*b.y + pose.m[2]*b.z, pose.m[4]*b.x + pose.m[5]*b.y + pose.m[6]*b.z,
                pose.m[8]*b.x + pose.m[9]*b.y + pose.m[10]*b.z };

   if (ca.z > -cWireNearM && cb.z > -cWireNearM)
      return; // wholly behind
   if (ca.z > -cWireNearM || cb.z > -cWireNearM)
   {
      TVec3 &in = ca.z > -cWireNearM ? ca : cb;
      const TVec3 &keep = ca.z > -cWireNearM ? cb : ca;
      float t = (-cWireNearM - keep.z)/(in.z - keep.z);

      in.x = keep.x + (in.x - keep.x)*t;
      in.y = keep.y + (in.y - keep.y)*t;
      in.z = -cWireNearM;
   }

   float x0,
         y0,
         x1,
         y1;

   if (wireToScreen(s, ca, x0, y0) && wireToScreen(s, cb, x1, y1))
      canvasLine(s, (int)x0, (int)y0, (int)x1, (int)y1, scaleDp(2), rgba);
}

/*--------------------------------------------------------------------------------
   The room's walls as a wireframe over the live preview (user, 2026-09-27: "a confirmação em
   wireframe das paredes do ambiente"): each wall's ceiling line and floor line and the vertical edge
   of every corner, placed by the plan (or, during the center spin, by the walls the ceiling gave so
   far - the corners found on the ceiling band show on the floor band too, furniture or not) and
   projected with the gyroscope's pose. Where it sits on the creases, the plan is confirmed.
  --------------------------------------------------------------------------------*/
void TCapApp::drawWireframe(TSurface &s)
{
   if (!PringCount || (Pphase != rpCenter && Pphase != rpCorner && Pphase != rpDoor))
      return;

   const TLayoutPlan &plan = Pplan.valid && !PplanSketch ? Pplan : PguidePlan;
   bool               polygon = Pplan.valid && !PplanSketch;
   TPlanPoint         eye = { 0.f, 0.f };

   if (!polygon && !PguidePlan.wallCount)
      return;
   if (Pphase == rpDoor)
      eye = PdoorEye;
   else if (Pphase == rpCorner && polygon && PcornerSlot >= 0 && PcornerSlot < Pplan.stationCount)
   {
      // standing in the corner, a little inside: toward the corner it aims at
      const TPlanPoint &c = Pplan.verts[Pplan.stationVertex[PcornerSlot]],
                       &t = Pplan.verts[Pplan.targetVertex[PcornerSlot]];
      float             du = t.u - c.u,
                        dw = t.w - c.w,
                        len = fmaxf(0.01f, sqrtf(du*du + dw*dw));

      eye.u = c.u + cWireCornerInM*du/len;
      eye.w = c.w + cWireCornerInM*dw/len;
   }

   TMat4 pose = poseOf(Pring[(PringHead + appRingSize - 1)%appRingSize]);
   float a = plan.axisDeg*0.01745329f,
         camH = plan.cameraHeightM > 0.5f ? plan.cameraHeightM : cWireCameraM,
         top = (plan.ceilingM > 1.f ? plan.ceilingM : cCeilingM) - camH,
         bottom = -camH;
   TVec3 du = { sinf(a), 0.f, -cosf(a) }, // plan u and w in the world (x east, y up, -z north)
         dw = { cosf(a), 0.f, sinf(a) };
   DWORD tone = canvasRGBA(0, 230, 170, 220);

   if (polygon)
      for (int i = 0; i < plan.vertexCount; i++)
      {
         const TPlanPoint &p = plan.verts[i],
                          &q = plan.verts[(i + 1)%plan.vertexCount];

         wireEdge(s, pose, wireAt(eye, du, dw, p.u, p.w, top), wireAt(eye, du, dw, q.u, q.w, top), tone);
         wireEdge(s, pose, wireAt(eye, du, dw, p.u, p.w, bottom), wireAt(eye, du, dw, q.u, q.w, bottom), tone);
         wireEdge(s, pose, wireAt(eye, du, dw, p.u, p.w, bottom), wireAt(eye, du, dw, p.u, p.w, top), tone);
      }
   else
   {
      for (int i = 0; i < plan.wallCount && i < layoutMaxWalls; i++)
      {
         float o = plan.wallOffset[i],
               a0 = plan.wallA0[i],
               a1 = plan.wallA1[i];
         bool  alongW = plan.wallKind[i] == 0; // u constant, runs along w

         for (int k = 0; k < 2; k++)
         {
            float y = k == 0 ? top : bottom;

            wireEdge(s, pose, alongW ? wireAt(eye, du, dw, o, a0, y) : wireAt(eye, du, dw, a0, o, y),
                     alongW ? wireAt(eye, du, dw, o, a1, y) : wireAt(eye, du, dw, a1, o, y), tone);
         }
      }
      for (int k = 0; k < PguideCount; k++)
         wireEdge(s, pose, wireAt(eye, du, dw, PguideAt[k].u, PguideAt[k].w, bottom),
                  wireAt(eye, du, dw, PguideAt[k].u, PguideAt[k].w, top), tone);
   }

   /* the doors of the door station at true scale on their walls - an opening 2 x cDoorHalfWidthM wide up to the head:
      the one to shoot yellow, confirmed green. The door the camera sees must fit it (user, 2026-09-27) */
   for (int i = 0; Pphase == rpDoor && i < PplanDoorCount; i++)
   {
      const TPlanDoor &d = PplanDoors[i];

      if (d.state == (BYTE)dsDropped)
         continue;

      DWORD color = d.state == (BYTE)dsConfirmed ? canvasRGBA(60, 220, 110, 255)
                                                 : (i == PdoorIdx ? canvasRGBA(255, 210, 0, 255) : canvasRGBA(255, 255, 255, 180));
      float lu = d.at.u - cDoorHalfWidthM*d.along.u,
            lw = d.at.w - cDoorHalfWidthM*d.along.w,
            ru = d.at.u + cDoorHalfWidthM*d.along.u,
            rw = d.at.w + cDoorHalfWidthM*d.along.w,
            head = cDoorHeadM - camH;

      wireEdge(s, pose, wireAt(eye, du, dw, lu, lw, bottom), wireAt(eye, du, dw, lu, lw, head), color);
      wireEdge(s, pose, wireAt(eye, du, dw, lu, lw, head), wireAt(eye, du, dw, ru, rw, head), color);
      wireEdge(s, pose, wireAt(eye, du, dw, ru, rw, head), wireAt(eye, du, dw, ru, rw, bottom), color);
   }
}

/*--------------------------------------------------------------------------------
   The door to shoot as the camera should frame it (the same perspective as the spin view): the
   opening from the floor to its head, where the plan puts it from where the operator stands, and
   the camera frame now. The two fitting - aimed and level - turns the door green; the shot follows.
   Red when the phone is not level.
  --------------------------------------------------------------------------------*/
void TCapApp::drawDoorAim(TSurface &s, int cx, int cy, int radius)
{
   if (PdoorIdx < 0 || PdoorIdx >= PplanDoorCount)
      return;

   const TPlanPoint &d = PplanDoors[PdoorIdx].at;
   TMat4             pose = poseOf(Pring[(PringHead + appRingSize - 1)%appRingSize]);
   float             heading = geomHeadingDeg(pose.Forward()),
                     pitch = geomPitchDeg(pose.Forward()),
                     du = d.u - PdoorEye.u,
                     dw = d.w - PdoorEye.w,
                     range = fmaxf(0.5f, sqrtf(du*du + dw*dw)),
                     camH = Pplan.cameraHeightM > 0.5f ? Pplan.cameraHeightM : 1.5f,
                     off = fmodf(doorAimDeg() - heading + 540.f, 360.f) - 180.f,
                     halfDeg = atanf(cDoorHalfWidthM/range)*57.29578f,
                     topDeg = atanf((cDoorHeadM - camH)/range)*57.29578f,
                     bottomDeg = atanf(-camH/range)*57.29578f;
   int               halfW = radius*3/2,
                     halfH = radius;
   float             unit = 0.62f*(float)halfW/90.f; // the spin view's pixels per degree
   bool              fits = PposeOk && fabsf(off) <= cDoorAimDeg;
   DWORD             tone = fits ? canvasRGBA(60, 220, 110, 255) : canvasRGBA(255, 210, 0, 255),
                     frame = canvasRGBA(255, 255, 255, 220);

   canvasFillRect(s, cx - halfW, cy - halfH, 2*halfW, 2*halfH, PposeOk ? canvasRGBA(0, 0, 0, 90) : canvasRGBA(220, 30, 30, 120));

   // the door, fixed in the world: its heading off the camera's, its elevations off the horizon
   int x0 = cx + (int)((off - halfDeg)*unit),
       x1 = cx + (int)((off + halfDeg)*unit),
       yTop = cy - (int)(topDeg/60.f*(float)halfH),
       yBottom = cy - (int)(bottomDeg/60.f*(float)halfH);

   canvasLine(s, x0, yBottom, x0, yTop, scaleDp(4), tone);
   canvasLine(s, x0, yTop, x1, yTop, scaleDp(4), tone);
   canvasLine(s, x1, yTop, x1, yBottom, scaleDp(4), tone);

   // confirmed but not yet measured: what the next shot must include, blinking (the foot, or the ceiling line)
   const TPlanDoor &pd = PplanDoors[PdoorIdx];
   bool             blink = (Pport->SensorClockNs()/400000000u)%2u == 0u;
   DWORD            want = canvasRGBA(255, 210, 0, 255);

   if (pd.state == (BYTE)dsConfirmed && blink)
   {
      float ceilingDeg = atanf(((Pplan.ceilingM > 1.f ? Pplan.ceilingM : cCeilingM) - camH)/range)*57.29578f;
      int   y = !pd.nFoot ? yBottom : cy - (int)(ceilingDeg/60.f*(float)halfH);

      if (!pd.nFoot || !pd.nCrease)
         canvasLine(s, x0 - scaleDp(20), y, x1 + scaleDp(20), y, scaleDp(8), want);
   }

   // the camera frame now (moves with the pitch)
   int fw = (int)(0.5f*PhfovDeg*unit),
       fy = cy - (int)(pitch/60.f*(float)halfH),
       fh = (int)(0.5f*PvfovDeg/60.f*(float)halfH);

   canvasLine(s, cx - fw, fy - fh, cx + fw, fy - fh, scaleDp(2), frame);
   canvasLine(s, cx + fw, fy - fh, cx + fw, fy + fh, scaleDp(2), frame);
   canvasLine(s, cx + fw, fy + fh, cx - fw, fy + fh, scaleDp(2), frame);
   canvasLine(s, cx - fw, fy + fh, cx - fw, fy - fh, scaleDp(2), frame);
   if (fabsf(off) > 95.f) // off screen: which way to turn
   {
      int side = off > 0.f ? 1 : -1,
          tip = cx + side*(halfW - scaleDp(8)),
          base = tip - side*scaleDp(28);

      canvasLine(s, base, cy - scaleDp(20), tip, cy, scaleDp(4), tone);
      canvasLine(s, base, cy + scaleDp(20), tip, cy, scaleDp(4), tone);
   }
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
   /* a pose off the guided band turns the panels red and nothing is kept: the ceiling spin level at most, the floor
      spin the opposite, the corner fan level, the floor view tilted down (user 2026-09-27) */
   bool  tilted = !PposeOk;
   DWORD grid = tilted ? canvasRGBA(235, 60, 50, 230) : canvasRGBA(255, 255, 255, 110),
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
   canvasFillRect(s, cx - halfW, yTop, 2*halfW, 2*halfH, tilted
                                                        ? canvasRGBA(220, 30, 30, 120) : canvasRGBA(0, 0, 0, 90));

   int  aimBand = -1,
        aimBin = -1;
   bool busy = freeSlots() == 0 && Pspin->AimedEmpty(aimBand, aimBin);

   for (int b = 0; b < bands; b++)
   {
      float pitch = Pspin->BandPitchDeg(b);
      int   inner = bands == 2 ? cy : (pitch > 5.f ? yCeil : yFloor), // two bands meet at the horizon: no hollow strip
            outer = pitch > 5.f ? yTop : yBottom;
      bool  middle = bands == 1 || (pitch >= -5.f && pitch <= 5.f); // a single band (corner fan) spans the wall

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
         /* still to shoot: outline only (a translucent fill reads as gray); aimed at while the previous photo is still
            processing: yellow, the operator holds instead of wondering */
         if (busy && b == aimBand && bin == aimBin)
            canvasFillQuad(s, xs, ys, canvasRGBA(255, 210, 0, 120));
         else if (Pspin->BinFilled(b, bin))
            canvasFillQuad(s, xs, ys, tilted ? canvasRGBA(235, 60, 50, 230) : binColor(b, bin));
         if (tilted || !Pspin->BinFilled(b, bin))
            for (int i = 0; i < 4; i++)
               canvasLine(s, xs[i], ys[i], xs[(i + 1)%4], ys[(i + 1)%4], tilted ? scaleDp(2) : scaleDp(1), grid);
         if (b == targetBand && bin == target)
            for (int i = 0; i < 4; i++)
               canvasLine(s, xs[i], ys[i], xs[(i + 1)%4], ys[(i + 1)%4], scaleDp(3), aim);
      }
   }

   // doors found from this station: an opening outline at their heading, floor up to the head (cyan)
   const DWORD doorTone = canvasRGBA(0, 200, 255, 255);

   for (int k = 0; k < PdoorCount; k++)
   {
      if (PdoorCand[k].station != Pstation.index)
         continue;

      float d = fmodf(PdoorCand[k].headingDeg - heading + 540.f, 360.f) - 180.f;

      if (fabsf(d) > 95.f)
         continue;

      int x = cx + (int)(d*unit),
          half = (int)(cDoorDrawHalfDeg*unit),
          top = cy - halfH/3,
          bottom = yBottom - halfH/6;

      canvasLine(s, x - half, bottom, x - half, top, scaleDp(3), doorTone);
      canvasLine(s, x - half, top, x + half, top, scaleDp(3), doorTone);
      canvasLine(s, x + half, top, x + half, bottom, scaleDp(3), doorTone);
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

   /* the camera's own heading as a vertical taller than the box (user, 2026-09-28): the bin it crosses is the one
      the next photo fills or retakes */
   int lineTop = (fy - fh < yTop ? fy - fh : yTop) - scaleDp(14),
       lineBottom = (fy + fh > yBottom ? fy + fh : yBottom) + scaleDp(14);

   canvasLine(s, cx, lineTop, cx, lineBottom, scaleDp(2), aim);
   if (target >= 0 && bestGap > 95.f)
   {
      float d = fmodf(Pspin->BinCenterDeg(target, heading) - heading + 540.f, 360.f) - 180.f;
      int   side = d > 0.f ? 1 : -1,
            tip = cx + side*(halfW - scaleDp(8)),
            base = tip - side*scaleDp(28);

      canvasLine(s, base, cy - scaleDp(20), tip, cy, scaleDp(4), aim);
      canvasLine(s, base, cy + scaleDp(20), tip, cy, scaleDp(4), aim);
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
   if (PhasPreview)
      drawWireframe(s); // the room's walls over the live view

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
   if (ProomOpen && PdoorCount)
      snprintf(line, sizeof(line), "Rumo %03d%s  -  %s  -  portas %d", (int)heading, Pmagnetic ? "" : " (relativo)", gps,
               PdoorCount);
   else
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
   if (Pphase == rpDoor && PringCount)
      drawDoorAim(s, s.width/2, s.height/2, s.width*3/10); // the door as the camera should frame it
   if (Pplan.valid && (Pphase == rpCorner || Pphase == rpDoor))
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

   if (PbackArmedNs && Pport->SensorClockNs() - PbackArmedNs <= (QWORD)appBackArmMs*1000000u)
   {
      canvasFillRect(s, 0, s.height/2 - scaleDp(30), s.width, scaleDp(60), canvasRGBA(200, 40, 40, 230));
      drawLabel(s, s.width/2, s.height/2 - scaleDp(14), ProomOpen ? "Voltar de novo encerra o cômodo"
                                                                    : "Voltar de novo encerra o imóvel",
                white, 18, true, true);
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
      else if (Pphase == rpDoor)
         snprintf(main, sizeof(main), PdoorIdx >= 0 && PdoorIdx < PplanDoorCount && PplanDoors[PdoorIdx].state == (BYTE)dsConfirmed
                                      ? "Pular medida" : "Descartar");
      else
         snprintf(main, sizeof(main), PcornerStep + 1 >= PcornerCount ? "Concluir último canto" : "Próximo canto");
      drawButton(s, PbtnMain, main, Pphase == rpNone ? canvasRGBA(30, 110, 220, 235)
                                                     : (Pphase == rpDoor ? (strcmp(main, "Descartar") ? canvasRGBA(60, 90, 160, 235)
                                                                                                    : canvasRGBA(210, 45, 40, 235))
                                                                         : canvasRGBA(40, 160, 90, 235)));
      if (ProomOpen || Psession.IsOpen())
      {
         PbtnFinish.w = PbtnMain.w;
         PbtnFinish.h = scaleDp(48);
         PbtnFinish.x = PbtnMain.x;
         PbtnFinish.y = PbtnMain.y - scaleDp(60);

         /* right above "Cheguei": a slip would end the room early (user, 2026-09-27). Disabled until every planned
            station is in - the last one closes the room by itself; a double Back still leaves a room on purpose */
         bool locked = ProomOpen && PcornerStep < PcornerCount;

         drawButton(s, PbtnFinish, ProomOpen ? "Concluir cômodo" : "Finalizar imóvel",
                    locked ? canvasRGBA(60, 60, 66, 110) : canvasRGBA(90, 90, 100, 235));
         PbtnFinish.visible = !locked;
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
