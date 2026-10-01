#include "winTypes.h"
#include "arg.h"
#include "idle.h"
#include "testCache.h"
#include "capGeom.h"
#include "capOrient.h"
#include "capSpin.h"
#include "capSession.h"
#include "capJPEG.h"
#include "capFrameMeta.h"
#include "capEXIF.h"
#include "capVanish.h"
#include "capLayout.h"
#include "capBlur.h"
#ifdef _WIN32
#include <direct.h>
#endif
#include "libDiscipline.h"

static int gFailures = 0;

#define checkThat(cond) testCheck((cond), #cond, __LINE__)

//--------------------------------------------------------------------------------
static void testCheck(bool ok, LPCSTR what, int line)
{
   if (ok)
      return;
   fprintf(stderr, "capTest.cpp:%d: FAILED %s\n", line, what);
   gFailures++;
}

//--------------------------------------------------------------------------------
static bool closeTo(float a, float b, float tol)
{
   return fabsf(a - b) <= tol;
}

//--------------------------------------------------------------------------------
// Camera level with the horizon, facing heading hDeg (clockwise from north)
static TMat4 yawPose(float hDeg)
{
   float th = -hDeg*0.01745329252f,
         c = cosf(th),
         s = sinf(th);
   TMat4 m = TMat4::Identity();

   m.m[0] = c;
   m.m[2] = -s;
   m.m[8] = s;
   m.m[10] = c;
   return m;
}

//--------------------------------------------------------------------------------
// Camera at heading hDeg, pitch pDeg, no roll (camera x right, y up, looking down -z)
static TMat4 headPitchPose(float hDeg, float pDeg)
{
   float h = hDeg*0.01745329252f,
         p = pDeg*0.01745329252f;
   TVec3 f = { cosf(p)*sinf(h), sinf(p), -cosf(p)*cosf(h) },
         r = { cosf(h), 0.f, sinf(h) },
         u = { r.y*f.z - r.z*f.y, r.z*f.x - r.x*f.z, r.x*f.y - r.y*f.x };
   TMat4 m = TMat4::Identity();

   m.m[0] = r.x;
   m.m[1] = r.y;
   m.m[2] = r.z;
   m.m[4] = u.x;
   m.m[5] = u.y;
   m.m[6] = u.z;
   m.m[8] = -f.x;
   m.m[9] = -f.y;
   m.m[10] = -f.z;
   return m;
}

//--------------------------------------------------------------------------------
static void quatMul(const float a[4], const float b[4], float out[4])
{
   out[0] = a[3]*b[0] + a[0]*b[3] + a[1]*b[2] - a[2]*b[1];
   out[1] = a[3]*b[1] - a[0]*b[2] + a[1]*b[3] + a[2]*b[0];
   out[2] = a[3]*b[2] + a[0]*b[1] - a[1]*b[0] + a[2]*b[3];
   out[3] = a[3]*b[3] - a[0]*b[0] - a[1]*b[1] - a[2]*b[2];
}

//--------------------------------------------------------------------------------
static void testGeometry(void)
{
   TVec3 north = { 0.f, 0.f, -1.f },
         east = { 1.f, 0.f, 0.f },
         west = { -1.f, 0.f, 0.f },
         up = { 0.f, 1.f, 0.f };

   checkThat(closeTo(geomHeadingDeg(north), 0.f, 0.01f));
   checkThat(closeTo(geomHeadingDeg(east), 90.f, 0.01f));
   checkThat(closeTo(geomHeadingDeg(west), 270.f, 0.01f));
   checkThat(closeTo(geomPitchDeg(up), 90.f, 0.01f));
   checkThat(closeTo(geomHeadingDiffDeg(350.f, 10.f), 20.f, 0.01f));
   checkThat(closeTo(geomHeadingDeg(yawPose(135.f).Forward()), 135.f, 0.01f));
}

//--------------------------------------------------------------------------------
static void testOrientation(void)
{
   float h = 0.70710678f,
         upright[4] = { h, 0.f, 0.f, h },   // ENU: portrait, back camera facing north
         turnEast[4] = { 0.f, 0.f, -h, h }, // -90 degrees about up
         facingEast[4],
         identity[4] = { 0.f, 0.f, 0.f, 1.f };
   TVec3 fwd;

   fwd = orientCameraToWorld(upright, sfEastNorthUp, 90).Forward();
   checkThat(closeTo(geomHeadingDeg(fwd), 0.f, 0.05f));
   checkThat(closeTo(geomPitchDeg(fwd), 0.f, 0.05f));

   quatMul(turnEast, upright, facingEast);
   fwd = orientCameraToWorld(facingEast, sfEastNorthUp, 90).Forward();
   checkThat(closeTo(geomHeadingDeg(fwd), 90.f, 0.05f));

   fwd = orientCameraToWorld(identity, sfEastNorthUp, 90).Forward(); // flat, screen up
   checkThat(closeTo(geomPitchDeg(fwd), -90.f, 0.05f));

   fwd = orientCameraToWorld(identity, sfNorthWestUp, 0).Forward();
   checkThat(closeTo(geomPitchDeg(fwd), -90.f, 0.05f));

   TVec3 right = orientCameraToWorld(identity, sfNorthWestUp, 0).RotateVector({ 1.f, 0.f, 0.f });

   checkThat(closeTo(right.z, -1.f, 0.001f)); // device x is north in NWU -> -z
}

//--------------------------------------------------------------------------------
static void testSpin(void)
{
   TSpinTracker spin(TSpinConfig::UltraWide());
   QWORD        ns = 1000000000u;
   int          kept = 0;

   for (int i = 0; i < 600; i++) // 30 Hz, 20 deg/s, 20 s = 400 degrees
   {
      if (spin.Offer(ns, yawPose((float)i*20.f/30.f), NAN) == svKeep)
         kept++;
      ns += 33333333u;
   }
   checkThat(spin.Complete());
   checkThat(kept == spin.Total());
   checkThat(spin.Filled() == 24);

   TSpinConfig moto = TSpinConfig::ForFov(54.f, 68.5f), // Moto G9 Play main camera, portrait
               ultra = TSpinConfig::ForFov(95.f, 115.f);

   checkThat(moto.bandCount == 3 && moto.headingBins == 36);
   checkThat(closeTo(moto.bandPitchDeg[0], -25.f, 0.01f) && closeTo(moto.bandPitchDeg[1], 0.f, 0.01f)
             && closeTo(moto.bandPitchDeg[2], 25.f, 0.01f) && moto.bandLoDeg[2] == 10.f && moto.bandHiDeg[0] == -10.f);
   checkThat(ultra.bandCount == 2 && ultra.headingBins == 36);

   /* the zigzag (user, 2026-09-30): column by column in a serpentine, ceiling first; every cell stays free (a pose in
      any band fills its own), the serpentine only names the next one */
   TSpinTracker zig(moto);
   QWORD        zns = 1000000000u;
   int          zb = -1,
                zc = -1;
   const float  zh0 = 0.5f*360.f/(float)moto.headingBins,
                zh1 = 1.5f*360.f/(float)moto.headingBins;

   checkThat(moto.zigzag && zig.GuidedCell(zb, zc) && zb == 2);
   checkThat(zig.Offer(zns, headPitchPose(zh0, 25.f), NAN) == svKeep);
   checkThat(zig.GuidedCell(zb, zc) && zb == 1 && zc == 0);
   zns += 2000000000u;
   checkThat(zig.Offer(zns, headPitchPose(zh0, 0.f), NAN) == svKeep);
   checkThat(zig.GuidedCell(zb, zc) && zb == 0 && zc == 0);
   zns += 2000000000u;
   checkThat(zig.Offer(zns, headPitchPose(zh0, -25.f), NAN) == svKeep);
   checkThat(zig.GuidedCell(zb, zc) && zb == 0 && zc == 1); // the next column back up from the floor
   zns += 2000000000u;
   checkThat(zig.Offer(zns, headPitchPose(zh1, -25.f), NAN) == svKeep);
   checkThat(zig.GuidedCell(zb, zc) && zb == 1 && zc == 1);
   zns += 2000000000u;
   checkThat(zig.Offer(zns, headPitchPose(60.f, 0.f), NAN) == svKeep && zig.PoseAllowed()); // a free cell off the serpentine
   checkThat(zig.GuidedCell(zb, zc) && zb == 1 && zc == 1);
   zns += 2000000000u;
   zig.Offer(zns, headPitchPose(zh1, 0.f), NAN);
   checkThat(zig.Offer(zns + 33333333u, headPitchPose(zh1, 3.f), NAN) == svTooFast); // a tilt of 90 degrees/s, heading still

   // one band at a time, ceiling first: down while the ceiling is due is red and never kept, then the opposite
   TSpinConfig layered = moto;

   layered.zigzag = false;

   TSpinTracker guided(layered);
   QWORD        gns = 1000000000u;
   const int    top = moto.bandCount - 1;
   const float  up = moto.bandPitchDeg[top],
                down = moto.bandPitchDeg[top - 1],
                h0 = 0.5f*360.f/(float)moto.headingBins; // bin 0's center: a bin edge is as near the last bin as minSepFrac allows

   checkThat(guided.GuidedBand() == top);
   checkThat(guided.Offer(gns, headPitchPose(h0, down), NAN) == svOffBand && !guided.PoseAllowed());
   gns += 1000000000u;
   checkThat(guided.Offer(gns, headPitchPose(h0, up), NAN) == svKeep && guided.PoseAllowed());
   for (int bin = 1; bin < moto.headingBins; bin++)
   {
      gns += 2000000000u; // 10-degree bins, 2 s apart: slower than the blur limit
      guided.Offer(gns, headPitchPose(((float)bin + 0.5f)*360.f/(float)moto.headingBins, up), NAN);
   }
   checkThat(guided.BandFilled(top) == moto.headingBins && guided.GuidedBand() == top - 1);
   gns += 1000000000u;
   checkThat(guided.Offer(gns, headPitchPose(h0, up), NAN) == svOffBand && !guided.PoseAllowed());

   // a small room: the floor view takes a pose tilted down to -60, not beyond
   TSpinTracker floorView(TSpinConfig::ForFloorView(54.f, 68.5f));

   floorView.AimFan(90.f);
   checkThat(floorView.Offer(5000000000u, headPitchPose(90.f, -58.f), NAN) == svKeep);
   checkThat(floorView.Offer(6000000000u, headPitchPose(90.f, -63.f), NAN) == svOffBand);

   // an orange ceiling bin may be retaken during the floor spin; once retaken (or merely green) it is red again
   guided.Reopen(top, 0, true);
   gns += 1000000000u;
   checkThat(guided.Offer(gns, headPitchPose(h0, up), NAN) == svKeep);
   gns += 1000000000u;
   checkThat(guided.Offer(gns, headPitchPose(h0, up), NAN) == svOffBand);
   guided.Reopen(top, 0);
   gns += 1000000000u;
   checkThat(guided.Offer(gns, headPitchPose(h0, up), NAN) == svOffBand); // a sharper-photo retake stays in order
   gns += 1000000000u;
   checkThat(guided.Offer(gns, headPitchPose(h0, down), NAN) == svKeep);

   // corner station: the first steady frame sets the aim; only a 60-degree fan around it counts
   TSpinTracker corner(TSpinConfig::ForCorner(54.f, 68.5f));
   QWORD        cns = 1000000000u;
   int          cornerKept = 0;

   checkThat(corner.Total() == 5);
   for (int i = 0; i <= 120; i++) // sweep 100..160 degrees (aim at 130 first) at 1 degree/s
   {
      float h = i == 0 ? 130.f : 100.f + (float)(i - 1)*0.5f;

      if (corner.Offer(cns, yawPose(h), NAN) == svKeep)
         cornerKept++;
      cns += 500000000u;
   }
   checkThat(corner.Complete() && cornerKept == 5);
   checkThat(corner.Offer(cns - 250000000u, yawPose(190.f), NAN) == svTooFast); // 30 degrees in 0.25 s
   checkThat(corner.Offer(cns + 1000000000u, yawPose(190.f), NAN) == svOutside);

   TSpinTracker fast(TSpinConfig::UltraWide());

   fast.Offer(0u, yawPose(0.f), NAN);
   checkThat(fast.Offer(33333333u, yawPose(5.f), NAN) == svTooFast); // 150 deg/s
   checkThat(fast.Offer(66666666u, yawPose(5.f), 60.f) == svBadCompass);

   TSpinTracker wide(TSpinConfig::Wide());
   TMat4        tilted = yawPose(0.f),
                moved = yawPose(0.f);

   tilted.m[9] = -0.3007f; // forward 17.5 degrees up, facing north: between the bands
   tilted.m[10] = 0.9537f;
   checkThat(wide.Offer(0u, tilted, NAN) == svOffBand);
   moved.m[12] = 1.f;
   checkThat(wide.Offer(1000000000u, moved, NAN) == svDrifted);
}

//--------------------------------------------------------------------------------
static void testRecords(void)
{
   TByteBuf     buf;
   TImageRecord img = {},
                back = {};
   BYTE         jpeg[5] = { 0xFF, 0xD8, 1, 2, 3 };

   img.width = 4000u;
   img.height = 3000u;
   img.format = pfJPEG;
   img.intr = { 1500.f, 1500.f, 2000.f, 1500.f };
   img.cameraToWorld = yawPose(42.f);
   img.compass = { 40.f, 41.5f, 12.f };
   img.pixels = jpeg;
   img.pixelBytes = sizeof(jpeg);
   img.Encode(buf);
   checkThat(back.Decode(buf.Data(), buf.Size()));
   checkThat(back.width == 4000u && back.format == pfJPEG);
   checkThat(closeTo(back.compass.trueDeg, 41.5f, 0.f));

   // a confirmed door: the ruler of the ceiling line
   TDoorRecord door = {},
               doorBack = {};
   TByteBuf    doorBuf;

   door.roomIndex = 2u;
   door.index = 1u;
   door.state = dsConfirmed;
   door.u = 1.8f;
   door.w = -0.4f;
   door.ratio = 1.322f;
   door.ratios = 3;
   door.imageNs = 123456789u;
   door.Encode(doorBuf);
   checkThat(doorBack.Decode(doorBuf.Data(), doorBuf.Size()) && doorBack.state == dsConfirmed && doorBack.index == 1u
             && closeTo(doorBack.ratio, 1.322f, 0.f) && doorBack.imageNs == 123456789u);

   // a plan solved at 2.80 m, the doors measuring 2.78: every length scales by 2.78/2.80
   TLayoutPlan plan = {};

   plan.ceilingM = 2.8f;
   plan.vertexCount = 1;
   plan.verts[0].u = 2.8f;
   plan.areaM2 = 10.f;
   layoutScalePlan(plan, 2.78f);
   checkThat(closeTo(plan.verts[0].u, 2.78f, 1e-4f) && closeTo(plan.areaM2, 10.f*0.99286f*0.99286f, 1e-3f)
             && closeTo(plan.ceilingM, 2.78f, 0.f));
   checkThat(back.pixelBytes == 5u && back.pixels[4] == 3);
   checkThat(!back.Decode(buf.Data(), buf.Size() - 1u));

   TLocationRecord loc = { -235505200, -466333100, 760000, 25000u, 40000u, 7u },
                   locBack = {};

   buf.Clear();
   loc.Encode(buf);
   checkThat(locBack.Decode(buf.Data(), buf.Size()));
   checkThat(locBack.latE7 == -235505200 && locBack.lonE7 == -466333100 && locBack.fixNs == 7u);

   float       verts[9] = { 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 1.f, 0.f };
   DWORD       tri[3] = { 0u, 1u, 2u },
               badTri[3] = { 0u, 1u, 3u };
   TMeshRecord mesh = {},
               meshBack = {};

   mesh.localToWorld = TMat4::Identity();
   mesh.vertices = (LPCBYTE)verts;
   mesh.vertexCount = 3u;
   mesh.indices = (LPCBYTE)tri;
   mesh.indexCount = 3u;
   buf.Clear();
   mesh.Encode(buf);
   checkThat(meshBack.Decode(buf.Data(), buf.Size()));
   checkThat(closeTo(meshBack.Vertex(2).y, 1.f, 0.f));
   mesh.indices = (LPCBYTE)badTri;
   buf.Clear();
   mesh.Encode(buf);
   checkThat(!meshBack.Decode(buf.Data(), buf.Size()));
}

//--------------------------------------------------------------------------------
static void testSession(void)
{
   static LPCSTR dir = "capTestSession";
   char          path[sessionPathMax];

   snprintf(path, sizeof(path), "%s/capture.lrec", dir);
   remove(path);
   snprintf(path, sizeof(path), "%s/session.json", dir);
   remove(path);

   TDeviceInfo    dev = {};
   TSessionWriter w;
   TPoseRecord    pose = {};
   BYTE           jpeg[3] = { 1, 2, 3 };
   TImageRecord   img = {};

   snprintf(dev.platform, sizeof(dev.platform), "test");
   snprintf(dev.model, sizeof(dev.model), "quote\" and \\ slash");
   checkThat(w.Open(dir, dev, 1000u));
   checkThat(!w.Open(dir, dev, 1000u));
   pose.cameraToWorld = yawPose(10.f);
   pose.tracking = tsNormal;
   img.format = pfJPEG;
   img.pixels = jpeg;
   img.pixelBytes = 3u;
   checkThat(w.BeginRoom(10u, "Sala") == 0u);
   checkThat(w.WritePose(11u, pose));
   checkThat(w.WriteImage(12u, img));

   // a retake replaced the image at 13: production drops it from the log, the rest stays in order
   TElectRecord elect = {};
   QWORD        drop = 13u;

   elect.electedNs = 14u;
   elect.supersededNs = 13u;
   checkThat(w.WriteImage(13u, img) && w.WriteImage(14u, img) && w.WriteElect(14u, elect));
   checkThat(w.Compact(&drop, 1) == 1 && w.Counts().images == 2u);
   checkThat(w.WritePose(15u, pose)); // the log goes on after the rewrite
   checkThat(w.BeginRoom(20u, "Cozinha") == 1u); // ends "Sala"
   w.EndRoom(30u, 1.45f);
   checkThat(!w.RoomOpen());
   w.Close(2000u);
   checkThat(w.Counts().rooms == 2u && w.Counts().images == 2u);

   TSessionWriter again;

   checkThat(!again.Open(dir, dev, 3000u)); // never overwrite a session

   snprintf(path, sizeof(path), "%s/capture.lrec", dir);

   FILE *f = fopen(path, "ab");

   checkThat(f != NULL);
   if (f)
   {
      fwrite("junk", 1u, 4u, f); // simulate a crash mid-record
      fclose(f);
   }

   TSessionReader r;
   TRecordView    v;
   TRoomRecord    room = {};
   int            n = 0,
                  rooms = 0;
   float          lastHeight = 0.f;

   checkThat(r.Open(dir));
   while (r.Next(v))
   {
      n++;
      if (v.type == rtRoom && room.Decode(v.payload, v.length))
      {
         rooms++;
         lastHeight = room.cameraHeightM;
      }
   }
   checkThat(n == 9); // the image at 13 is gone: pose, 2 images, elect, pose, 4 room markers
   checkThat(rooms == 4);
   checkThat(closeTo(lastHeight, 1.45f, 0.f));
   checkThat(r.Truncated());
}

/*--------------------------------------------------------------------------------
   Encode a synthetic NV21 frame with odd dimensions, decode it back through WIC
   (Windows' own JPEG decoder) and require the decoded luma to match the source Y plane.
  --------------------------------------------------------------------------------*/
static void testJPEG(void)
{
   const int    w = 333,
                h = 251;
   TAlloc<BYTE> yPlane((size_t)w*h),
                vu((size_t)((w + 1)/2)*2*((h + 1)/2));

   for (int y = 0; y < h; y++)
      for (int x = 0; x < w; x++)
         yPlane[(size_t)y*w + x] = (BYTE)(64 + (x*96)/w + (y*64)/h + ((x/40 + y/40)%2)*32);
   for (int y = 0; y < (h + 1)/2; y++)
      for (int x = 0; x < (w + 1)/2; x++)
      {
         vu[(size_t)y*((w + 1)/2)*2 + (size_t)x*2] = (BYTE)(128 + x/8);
         vu[(size_t)y*((w + 1)/2)*2 + (size_t)x*2 + 1] = (BYTE)(128 - y/8);
      }

   TYUVImage    img = { w, h, yPlane(), w, vu() + 1, vu(), ((w + 1)/2)*2, 2 };
   TJPEGEncoder enc(90);
   TByteBuf     jpg;

   TFrameMeta meta = {},
              back = {};
   TByteBuf   metaBuf;

   meta.sessionId[0] = 0xAB;
   meta.roomIndex = 3u;
   meta.seq = 17u;
   meta.cameraToWorld = yawPose(77.f);
   meta.headingRef = hrArbitrary;
   meta.headingDeg = 77.f;
   meta.spinBand = 1u;
   meta.spinBin = 5u;
   meta.latE7 = -235505200;
   meta.stationIndex = 3u;
   meta.stationKind = (BYTE)skCorner;
   meta.cornerIndex = 2u;
   meta.targetCorner = 0u;
   meta.Encode(metaBuf);
   checkThat(metaBuf.Size() == (size_t)frameMetaSize);
   checkThat(enc.Encode(img, jpg, metaBuf.Data(), metaBuf.Size()));
   checkThat(jpg.Size() > 1000u && jpg.Data()[0] == 0xFF && jpg.Data()[1] == 0xD8);

   size_t  metaLen = 0u;
   LPCBYTE found = TFrameMeta::Find(jpg.Data(), jpg.Size(), &metaLen);

   checkThat(found != NULL && back.Decode(found, metaLen));
   checkThat(back.sessionId[0] == 0xAB && back.seq == 17u && back.spinBin == 5u && back.latE7 == -235505200);
   checkThat(closeTo(geomHeadingDeg(back.cameraToWorld.Forward()), 77.f, 0.01f) && back.refineFlags == 0u);
   checkThat(back.stationIndex == 3u && back.stationKind == (BYTE)skCorner && back.cornerIndex == 2u);

   TStationRecord st = { seBegin, 1u, 3u, skCorner, 2u, 0u },
                  stBack = {};
   TByteBuf       stBuf;

   st.Encode(stBuf);
   checkThat(stBack.Decode(stBuf.Data(), stBuf.Size()) && stBack.target == 0u && stBack.kind == skCorner);

   // EXIF: upright from the pose; a portrait Android frame (sensor 90) needs "rotate 90 clockwise"
   float     half = 0.70710678f,
             upright[4] = { half, 0.f, 0.f, half };
   TEXIFInfo ex = {};
   TByteBuf  exBuf,
             withEXIF;

   checkThat(exifOrientation(yawPose(33.f)) == 1u);
   checkThat(exifOrientation(orientCameraToWorld(upright, sfEastNorthUp, 90)) == 6u);
   snprintf(ex.model, sizeof(ex.model), "moto g(9) play");
   snprintf(ex.software, sizeof(ex.software), "capTest");
   ex.wallNs = 1790454990000000000u;
   ex.orientation = 6u;
   ex.focalMm = 4.71f;
   ex.width = (DWORD)w;
   ex.height = (DWORD)h;
   ex.hasGPS = true;
   ex.latE7 = -235505200;
   ex.lonE7 = -466333100;
   ex.altMm = 760000;
   exifBuild(ex, exBuf);
   checkThat(jpegInsertSegment(jpg.Data(), jpg.Size(), 0xE1, exBuf.Data(), exBuf.Size(), withEXIF));
   checkThat(withEXIF.Data()[3] == 0xE1 && !memcmp(withEXIF.Data() + 6, "Exif", 4u));

#ifdef _WIN32
   IWICImagingFactory    *factory = NULL;
   IWICStream            *stream = NULL;
   IWICBitmapDecoder     *decoder = NULL;
   IWICBitmapFrameDecode *frame = NULL;
   IWICFormatConverter   *gray = NULL;

   UINT                 dw = 0u,
                         dh = 0u;
   TAlloc<BYTE>          out((size_t)w*h);
   bool                  decoded = false;

   CoInitializeEx(NULL, COINIT_MULTITHREADED);
   if (SUCCEEDED(CoCreateInstance(CLSID_WICImagingFactory, NULL, CLSCTX_INPROC_SERVER, IID_PPV_ARGS(&factory)))
       && SUCCEEDED(factory->CreateStream(&stream))
       && SUCCEEDED(stream->InitializeFromMemory((LPBYTE)withEXIF.Data(), (DWORD)withEXIF.Size()))
       && SUCCEEDED(factory->CreateDecoderFromStream(stream, NULL, WICDecodeMetadataCacheOnDemand, &decoder))
       && SUCCEEDED(decoder->GetFrame(0u, &frame))
       && SUCCEEDED(frame->GetSize(&dw, &dh))
       && SUCCEEDED(factory->CreateFormatConverter(&gray))
       && SUCCEEDED(gray->Initialize(frame, GUID_WICPixelFormat8bppGray, WICBitmapDitherTypeNone, NULL, 0.f,
                                     WICBitmapPaletteTypeCustom))
       && dw == (UINT)w && dh == (UINT)h)
      decoded = SUCCEEDED(gray->CopyPixels(NULL, (UINT)w, (UINT)(w*h), out()));
   checkThat(decoded);
   if (decoded)
   {
      float sse = 0.f;

      for (size_t i = 0; i < (size_t)w*h; i++)
      {
         float d = (float)out[i] - (float)yPlane[i];

         sse += d*d;
      }

      float mse = sse/(float)(w*h),
            psnr = 10.f*log10f(255.f*255.f/(mse > 0.f ? mse : 1e-6f));

      printf("capTest: JPEG %u bytes, luma PSNR %.1f dB\n", (unsigned)jpg.Size(), psnr);
      checkThat(psnr > 32.f);
   }
   if (gray)
      gray->Release();
   if (frame)
      frame->Release();
   if (decoder)
      decoder->Release();
   if (stream)
      stream->Release();
   if (factory)
      factory->Release();
   CoUninitialize();
#endif
}

//--------------------------------------------------------------------------------
// Tile grid (0.5 m) on a surface: dark lines along both in-plane room directions
static int roomShade(float s, float t, int base)
{
   float fs = s*2.f - floorf(s*2.f),
         ft = t*2.f - floorf(t*2.f);

   return fs < 0.04f || ft < 0.04f ? base - 70 : base;
}

/*--------------------------------------------------------------------------------
   One ray (world direction d) inside a box room with axes a and b: floor 1.4 m below and
   ceiling 1.2 m above the camera, each surface with its own tone and a tile grid.
  --------------------------------------------------------------------------------*/
static int roomRay(const TVec3 &d, const TVec3 &a, const TVec3 &b)
{
   float pa = 0.3f,
         pb = -0.2f,
         halfA = 2.2f,
         halfB = 1.6f,
         floorY = -1.4f,
         ceilY = 1.2f,
         da = d.x*a.x + d.z*a.z,
         db = d.x*b.x + d.z*b.z,
         best = 1e9f;
   int   face = -1;
   float ta = da > 0.f ? (halfA - pa)/da : (da < 0.f ? (-halfA - pa)/da : 1e9f),
         tb = db > 0.f ? (halfB - pb)/db : (db < 0.f ? (-halfB - pb)/db : 1e9f),
         ty = d.y > 0.f ? ceilY/d.y : (d.y < 0.f ? floorY/d.y : 1e9f);

   if (ta < best)
   {
      best = ta;
      face = da > 0.f ? 0 : 1;
   }
   if (tb < best)
   {
      best = tb;
      face = db > 0.f ? 2 : 3;
   }
   if (ty < best)
   {
      best = ty;
      face = d.y > 0.f ? 4 : 5;
   }

   float ha = pa + best*da,
         hb = pb + best*db,
         hy = best*d.y;

   if (face <= 1)
      return roomShade(hb, hy, face == 0 ? 190 : 150);
   if (face <= 3)
      return roomShade(ha, hy, face == 2 ? 170 : 130);
   return roomShade(ha, hb, face == 4 ? 210 : 110);
}

//--------------------------------------------------------------------------------
// Renders the box room with axes at heading axisDeg (and +90), 4x4 supersampled like a real lens (pixel centers on integers, as the intrinsics)
static void renderRoom(LPBYTE luma, int w, int h, const TIntrinsics &k, const TMat4 &pose, float axisDeg)
{
   float ang = axisDeg*0.01745329252f;
   TVec3 a = { sinf(ang), 0.f, -cosf(ang) },
         b = { cosf(ang), 0.f, sinf(ang) };

   for (int v = 0; v < h; v++)
      for (int u = 0; u < w; u++)
      {
         int sum = 0;

         for (int sy = 0; sy < 4; sy++)
            for (int sx = 0; sx < 4; sx++)
            {
               TVec3 dc = { ((float)u - 0.375f + 0.25f*(float)sx - k.cx)/k.fx,
                            -((float)v - 0.375f + 0.25f*(float)sy - k.cy)/k.fy, -1.f };

               sum += roomRay(pose.RotateVector(dc), a, b);
            }
         luma[(size_t)v*w + u] = (BYTE)(sum/16);
      }
}

/*--------------------------------------------------------------------------------
   One ray inside a rectilinear room given as a polygon in room axes (a, b), camera at the
   origin, floor 1.4 m below and ceiling 1.2 m above: plain ceiling (as real ones), tiled floor,
   gridded walls - the floor grout lines must not turn into walls.
  --------------------------------------------------------------------------------*/
static int planRoomRay(const TVec3 &d, const TVec3 &a, const TVec3 &b, const float *poly, int n)
{
   float da = d.x*a.x + d.z*a.z,
         db = d.x*b.x + d.z*b.z,
         best = 1e9f,
         along = 0.f,
         floorY = -1.4f,
         ceilY = 1.2f;
   int   wall = -1;

   for (int i = 0; i < n; i++)
   {
      float a0 = poly[2*i],
            b0 = poly[2*i + 1],
            a1 = poly[2*((i + 1)%n)],
            b1 = poly[2*((i + 1)%n) + 1];

      if (a0 == a1 && fabsf(da) > 1e-6f)
      {
         float t = a0/da,
               hb = t*db;

         if (t > 0.f && t < best && hb >= fminf(b0, b1) && hb <= fmaxf(b0, b1))
         {
            best = t;
            along = hb;
            wall = i;
         }
      }
      else if (b0 == b1 && fabsf(db) > 1e-6f)
      {
         float t = b0/db,
               ha = t*da;

         if (t > 0.f && t < best && ha >= fminf(a0, a1) && ha <= fmaxf(a0, a1))
         {
            best = t;
            along = ha;
            wall = i;
         }
      }
   }

   float ty = d.y > 0.f ? ceilY/d.y : (d.y < 0.f ? floorY/d.y : 1e9f);

   if (ty < best)
   {
      if (d.y > 0.f)
         return 215;
      return roomShade(ty*da, ty*db, 110);
   }
   if (wall < 0)
      return 0;

   // wall: vertical joints every 0.5 m, baseboard top 0.10 m above the floor, a door head at 2.10 m on part of it
   float y = best*d.y - floorY,
         fa = along*2.f - floorf(along*2.f);
   int   tone = 130 + 15*(wall%4);

   if (y < 0.10f)
      return tone - 60;
   if (fa < 0.04f || (y > 2.10f && y < 2.14f && along > 0.2f && along < 1.0f))
      return tone - 70;
   return tone;
}

//--------------------------------------------------------------------------------
static void renderPlanRoom(LPBYTE luma, int w, int h, const TIntrinsics &k, const TMat4 &pose, float axisDeg,
                           const float *poly, int n)
{
   float ang = axisDeg*0.01745329252f;
   TVec3 a = { sinf(ang), 0.f, -cosf(ang) },
         b = { cosf(ang), 0.f, sinf(ang) };

   for (int v = 0; v < h; v++)
      for (int u = 0; u < w; u++)
      {
         int sum = 0;

         for (int sy = 0; sy < 4; sy++)
            for (int sx = 0; sx < 4; sx++)
            {
               TVec3 dc = { ((float)u - 0.375f + 0.25f*(float)sx - k.cx)/k.fx,
                            -((float)v - 0.375f + 0.25f*(float)sy - k.cy)/k.fy, -1.f };

               sum += planRoomRay(pose.RotateVector(dc), a, b, poly, n);
            }
         luma[(size_t)v*w + u] = (BYTE)(sum/16);
      }
}

//--------------------------------------------------------------------------------
// Center spin of a synthetic room (12 headings x 2 pitch bands) through the detector into the layout
static bool spinLayout(const float *poly, int n, float axisDeg, float ceilingM, TLayoutPlan &plan)
{
   const int     w = 640,
                 h = 480;
   TIntrinsics   k = { 500.f, 500.f, 320.f, 240.f };
   TVanishConfig cfg = TVanishConfig::Default();
   TAlloc<BYTE>  luma((size_t)w*h);
   TAlloc<float> rays((size_t)cfg.maxEdges*3u);
   TAlloc<BYTE>  labels((size_t)cfg.maxEdges);
   TVanishEdges  edges = { rays(), labels(), (DWORD)cfg.maxEdges, 0u };
   TRoomLayout   layout;
   TVec3         noTilt = { 0.f, 0.f, 0.f }; // synthetic poses are exact

   for (int band = 0; band < 2; band++)
      for (int i = 0; i < 12; i++)
      {
         TMat4         pose = headPitchPose(5.f + 30.f*(float)i, band ? -22.f : 22.f);
         TVanishResult r;

         renderPlanRoom(luma(), w, h, k, pose, axisDeg, poly, n);
         if (vanishDetect(luma(), w, h, w, k, pose, cfg, r, &edges))
            layout.AddFrame(pose, r, edges, noTilt, 0);
      }
   return layout.Solve(axisDeg, ceilingM, plan);
}

//--------------------------------------------------------------------------------
static void printPlan(LPCSTR name, const TLayoutPlan &p)
{
   printf("capTest: layout %s: valid %d height %.2f (solved %d) extent %.2f x %.2f area %.2f, %d corners, %d stations:",
          name, (int)p.valid, p.cameraHeightM, (int)p.heightSolved, p.extentU, p.extentW, p.areaM2, p.vertexCount,
          p.stationCount);
   for (int i = 0; i < p.stationCount; i++)
      printf(" %u->%u", (unsigned)p.stationVertex[i], (unsigned)p.targetVertex[i]);
   printf("\n");
}

//--------------------------------------------------------------------------------
static void testLayout(void)
{
   const float rect[8] = { 2.5f, -1.4f, 2.5f, 1.8f, -1.9f, 1.8f, -1.9f, -1.4f },
               ell[12] = { 2.7f, -1.5f, 2.7f, 0.5f, 0.7f, 0.5f, 0.7f, 2.5f, -2.3f, 2.5f, -2.3f, -1.5f };
   TLayoutPlan plan;

   checkThat(spinLayout(rect, 4, 17.f, 2.6f, plan));
   printPlan("rectangle", plan);
   checkThat(plan.heightSolved && closeTo(plan.cameraHeightM, 1.4f, 0.06f));
   checkThat(closeTo(plan.extentU, 4.4f, 0.15f) && closeTo(plan.extentW, 3.2f, 0.15f));
   checkThat(plan.vertexCount == 4 && plan.stationCount == 4 && plan.centerCount == 1);
   for (int i = 0; i < plan.stationCount; i++)
      checkThat(plan.targetVertex[i] == (BYTE)((plan.stationVertex[i] + 2)%4)); // the opposite corner

   checkThat(spinLayout(ell, 6, 17.f, 2.6f, plan));
   printPlan("L", plan);
   checkThat(closeTo(plan.extentU, 5.f, 0.2f) && closeTo(plan.extentW, 4.f, 0.2f) && closeTo(plan.areaM2, 16.f, 0.8f));
   /* five convex corners plus the reflex one; the corner facing it aims at both arms and back at it (user's map,
      2026-09-27) */
   checkThat(plan.vertexCount == 6 && plan.stationCount == 8);
   checkThat(plan.centerCount == 3); // one middle per arm of the L and one where they meet

   // a wrong ceiling assumption (2.80 for a 2.60 room): the 2.10 m door head measures the scale error
   checkThat(spinLayout(rect, 4, 17.f, 2.8f, plan));
   printPlan("rectangle, ceiling assumed 2.80", plan);
   printf("capTest: layout door: found %d scale %.3f ceiling measured %.2f\n", (int)plan.doorFound, plan.doorScale,
          plan.ceilingM);
   checkThat(plan.doorFound && closeTo(plan.doorScale, 2.6f/2.8f, 0.03f));
   // the heads imply 2.60: nearer the typical 2.70 than the assumed 2.80, the plan is scaled to 2.70 (error 8% -> 4%)
   checkThat(plan.ceilingSnapped && closeTo(plan.ceilingM, 2.7f, 1e-4f));
}

//--------------------------------------------------------------------------------
static void testVanish(void)
{
   const int     w = 640,
                 h = 480;
   const float   axisDeg = 17.f;
   TIntrinsics   k = { 500.f, 500.f, 320.f, 240.f };
   TMat4         pose = headPitchPose(40.f, 15.f);
   TAlloc<BYTE>  luma((size_t)w*h);
   TVanishConfig cfg = TVanishConfig::Default();
   TVanishResult r;
   TAxisCheck    check(2.f);
   float         dev = 0.f;

   checkThat(closeTo(vanishAxisDiffDeg(89.f, 1.f), -2.f, 0.01f));
   checkThat(closeTo(vanishAxisDiffDeg(1.f, 89.f), 2.f, 0.01f));
   checkThat(closeTo(vanishAxisDiffDeg(30.f, 120.f), 0.f, 0.01f));

   memset(luma(), 128, (size_t)w*h);
   checkThat(!vanishDetect(luma(), w, h, w, k, pose, cfg, r)); // flat image: no verdict
   checkThat(check.Offer(r, &dev) == avNoLines && isnan(dev));

   renderRoom(luma(), w, h, k, pose, axisDeg);
   checkThat(vanishDetect(luma(), w, h, w, k, pose, cfg, r));
   printf("capTest: vanish exact pose: axis %.2f tilt %.2f ortho %.2f support %lu/%lu/%lu of %lu\n", r.roomAxisDeg,
          r.tiltErrDeg, r.orthoErrDeg, (unsigned long)r.support[0], (unsigned long)r.support[1],
          (unsigned long)r.support[2], (unsigned long)r.edges);
   checkThat(r.flags == (BYTE)(vfVertical | vfAxisA | vfAxisB));
   checkThat(fabsf(vanishAxisDiffDeg(r.roomAxisDeg, axisDeg)) < 0.3f);
   checkThat(r.tiltErrDeg < 0.6f && r.orthoErrDeg < 0.5f); // the vertical VP lies far off the image: weakest direction
   checkThat(check.Offer(r, &dev) == avPending && !check.HasReference()); // one frame is no quorum

   // the same image with a 1.5-degree gravity error: the vertical measured by the image exposes it
   TMat4 tilted = headPitchPose(40.f, 16.5f);

   checkThat(vanishDetect(luma(), w, h, w, k, tilted, cfg, r));
   printf("capTest: vanish tilted pose: axis %.2f tilt %.2f\n", r.roomAxisDeg, r.tiltErrDeg);
   checkThat(closeTo(r.tiltErrDeg, 1.5f, 0.4f));
   checkThat(fabsf(vanishAxisDiffDeg(r.roomAxisDeg, axisDeg)) < 0.5f);

   // the gyroscope drifted 3 degrees: the image still shows the true room, so the frame will disagree
   TMat4         drifted = headPitchPose(43.f, 15.f);
   TVanishResult rDrift;

   checkThat(vanishDetect(luma(), w, h, w, k, drifted, cfg, rDrift));
   printf("capTest: vanish drifted pose: axis %.2f\n", rDrift.roomAxisDeg);

   // two more views of the room complete the quorum
   TMat4 other = headPitchPose(200.f, -18.f),
         third = headPitchPose(115.f, 8.f);

   renderRoom(luma(), w, h, k, other, axisDeg);
   checkThat(vanishDetect(luma(), w, h, w, k, other, cfg, r));
   printf("capTest: vanish second view: axis %.2f\n", r.roomAxisDeg);
   checkThat(check.Offer(r, &dev) == avPending);
   renderRoom(luma(), w, h, k, third, axisDeg);
   checkThat(vanishDetect(luma(), w, h, w, k, third, cfg, r));
   printf("capTest: vanish third view: axis %.2f\n", r.roomAxisDeg);
   checkThat(check.Offer(r, &dev) == avReference && check.HasReference() && fabsf(dev) < 0.4f);
   checkThat(fabsf(vanishAxisDiffDeg(check.ReferenceDeg(), axisDeg)) < 0.3f);
   checkThat(check.Offer(rDrift, &dev) == avMisaligned && closeTo(dev, 3.f, 0.4f));
   checkThat(check.Aligned() == 3 && check.Misaligned() == 1 && check.Unverified() == 1);

   /* a long spin with the gyroscope drifting slowly (0.1 degree per frame, 25 degrees over 250 frames): the reference
      follows past the store's size, so no frame is judged off (the store once stopped taking new measures at 128) */
   TAxisCheck    slow(2.f);
   TVanishResult rs = r;
   int           off = 0;

   for (int i = 0; i < 250; i++)
   {
      rs.roomAxisDeg = fmodf(axisDeg + 0.1f*(float)i, 90.f);
      off += slow.Offer(rs, &dev) == avMisaligned ? 1 : 0;
   }
   checkThat(off == 0 && closeTo(slow.DriftDeg(), 24.9f, 0.6f));

   // the summary survives the LIDARCAP round trip inside the fixed 300-byte block
   TFrameMeta meta = {},
              back = {};
   TByteBuf   buf;

   meta.roomAxisDeg = r.roomAxisDeg;
   meta.axisDevDeg = dev;
   meta.tiltErrCdeg = 37u;
   meta.orthoErrCdeg = 0xFFFFu;
   meta.vanishFlags = r.flags;
   meta.axisVerdict = (BYTE)avAligned;
   meta.blurPx = 2.37f;
   meta.blurMinPx = NAN;
   meta.Encode(buf);
   checkThat(buf.Size() == (size_t)frameMetaSize && back.Decode(buf.Data(), buf.Size()));
   checkThat(closeTo(back.blurPx, 2.37f, 0.01f) && isnan(back.blurMinPx));
   checkThat(closeTo(back.roomAxisDeg, r.roomAxisDeg, 1e-4f) && back.tiltErrCdeg == 37u && back.orthoErrCdeg == 0xFFFFu);
   checkThat(back.vanishFlags == r.flags && back.axisVerdict == (BYTE)avAligned);

   TVanishRecord rec = {},
                 recBack = {};
   TByteBuf      recBuf;

   rec.seq = 9u;
   rec.roomAxisDeg = r.roomAxisDeg;
   rec.dirCam[1] = r.dirCam[1];
   rec.support[2] = r.support[2];
   rec.Encode(recBuf);
   checkThat(recBack.Decode(recBuf.Data(), recBuf.Size()) && recBack.seq == 9u && recBack.support[2] == r.support[2]);
}

/*--------------------------------------------------------------------------------
   Blur: random gray blocks over several octaves (sharp steps at every scale, like tile joints and
   furniture edges, with the 1/f^2 spectrum of a scene) against the same image through three box passes (close to a Gaussian of sigma ~6 px) and a
   plain gray image (no texture: no verdict).
  --------------------------------------------------------------------------------*/
static void testBlur(void)
{
   const int    side = 1200;
   TAlloc<BYTE> sharp((size_t)side*side),
                soft((size_t)side*side),
                line((size_t)side);
   DWORD        seed = 12345u;
   TBlurResult  r = {};

   memset(sharp(), 128, (size_t)side*side);
   for (int block = 150; block >= 3; block /= 2) // octaves 150..4 px, the same amplitude each: 1/f^2
   {
      int amp = 14;

      for (int by = 0; by*block < side; by++)
         for (int bx = 0; bx*block < side; bx++)
         {
            seed = seed*1664525u + 1013904223u;

            int delta = (int)((seed >> 16)%(DWORD)(2*amp + 1)) - amp;

            for (int y = by*block; y < (by + 1)*block && y < side; y++)
               for (int x = bx*block; x < (bx + 1)*block && x < side; x++)
               {
                  int v = (int)sharp[(size_t)y*side + x] + delta;

                  sharp[(size_t)y*side + x] = (BYTE)(v < 0 ? 0 : (v > 255 ? 255 : v));
               }
         }
   }
   memcpy(soft(), sharp(), (size_t)side*side);
   for (int pass = 0; pass < 6; pass++) // three box passes of radius 6 in each direction
      for (int row = 0; row < side; row++)
      {
         bool   across = pass%2 == 0;
         LPBYTE p = soft();

         for (int i = 0; i < side; i++)
         {
            int sum = 0,
                n = 0;

            for (int d = -6; d <= 6; d++)
               if (i + d >= 0 && i + d < side)
               {
                  sum += across ? p[(size_t)row*side + i + d] : p[(size_t)(i + d)*side + row];
                  n++;
               }
            line[i] = (BYTE)(sum/n);
         }
         for (int i = 0; i < side; i++)
            if (across)
               p[(size_t)row*side + i] = line[i];
            else
               p[(size_t)i*side + row] = line[i];
      }

   checkThat(blurMeasure(sharp(), side, side, side, r) && r.textured == r.tiles);

   float sharpPx = r.medianPx;

   checkThat(blurMeasure(soft(), side, side, side, r));
   printf("capTest: blur sharp %.2f px, box-blurred %.2f px\n", sharpPx, r.medianPx);
   checkThat(sharpPx < 3.f && r.medianPx > 5.f && r.medianPx < 8.f); // three boxes of 13: sigma 6.5
   memset(sharp(), 128, (size_t)side*side);
   checkThat(!blurMeasure(sharp(), side, side, side, r) && isnan(r.medianPx) && !r.textured);
}

//--------------------------------------------------------------------------------
int main(int argc, LPSTR *argv)
{
   abArgInit(argc, argv);
   abSetIdlePriority();

   TAbTestCache tc(argc, argv);

   if (tc.isHit())
      return tc.code();
   testGeometry();
   testOrientation();
   testSpin();
   testRecords();
   testSession();
   testJPEG();
   testVanish();
   testLayout();
   testBlur();
   printf("capTest: %d failure(s)\n", gFailures);
   return tc.commit(gFailures ? 1 : 0);
}

//--------------------------------------------------------------------------------
