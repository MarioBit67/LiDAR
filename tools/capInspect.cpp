#include "winTypes.h"
#include "idle.h"
#include "capSession.h"
#include "capFrameMeta.h"
#include "capEXIF.h"
#include "capVanish.h"
#include "capLayout.h"
#include "capMosaic.h"
#include "alloc.h"
#include "json.h"

#include <stdio.h>
#include <string.h>
#ifdef _WIN32
#include <direct.h>
#include <windows.h>
#include <wincodec.h>
#endif

/* capInspect - desktop inspection of a capture session.
 *
 *   capInspect <sessionDir> <outDir> [--vanish] [--ceiling meters] [--mingrad n] [--edges] [--rectify] [--walls]
 *
 * Extracts every keyframe JPEG (with its embedded LIDARCAP block) to outDir, writes frames.csv with the
 * per-frame correlation metadata and poses.csv with the attitude stream, and prints a summary. Frames
 * captured before the app measured vanishing points (or every frame, with --vanish) are decoded through
 * WIC and measured here with the same core detector, room by room. */

enum {
   inspectMaxFrames = 512,  // keyframes whose measures are kept for the wall composition
   inspectWallPxPerM = 250, // wall orthophoto resolution: 4 mm per pixel
   inspectStations   = 512  // frame info: room*inspectStations + station index
};

//--------------------------------------------------------------------------------
static void inspectMakeDir(LPCSTR dir)
{
#ifdef _WIN32
   _mkdir(dir);
#else
   mkdir(dir, 0755);
#endif
}

//--------------------------------------------------------------------------------
static LPCSTR inspectTypeName(TRecordType t)
{
   switch (t)
   {
      case rtPose :
         return "pose";

      case rtImage :
         return "image";

      case rtDepth :
         return "depth";

      case rtMesh :
         return "mesh";

      case rtLocation :
         return "location";

      case rtRoom :
         return "room";

      case rtStation :
         return "station";

      case rtLayout :
         return "layout";

      case rtVanish :
         return "vanish";

      default :
         return "unknown";
   }
}

//--------------------------------------------------------------------------------
// True when the JPEG already carries an EXIF APP1 before the image data
static bool inspectHasEXIF(LPCBYTE jpg, size_t n)
{
   size_t pos = 2u;

   while (pos + 10u <= n && jpg[pos] == 0xFF && jpg[pos + 1u] != 0xDA)
   {
      size_t len = ((size_t)jpg[pos + 2u] << 8) | jpg[pos + 3u];

      if (jpg[pos + 1u] == 0xE1 && !memcmp(jpg + pos + 4u, "Exif", 4u))
         return true;
      pos += 2u + len;
   }
   return false;
}

/*--------------------------------------------------------------------------------
   Device model and lens focal length from the session manifest (parsed with TJSON; the
   focal length comes from the camera label, e.g. "back 0 (4.71 mm)").
  --------------------------------------------------------------------------------*/
static void inspectReadDevice(LPCSTR dir, LPSTR model, size_t cap, float *focalMm)
{
   char path[sessionPathMax];

   snprintf(model, cap, "unknown");
   *focalMm = 0.f;
   snprintf(path, sizeof(path), "%s/session.json", dir);

   FILE *f = fopen(path, "rb");

   if (!f)
      return;

   char   text[4096];
   size_t n = fread(text, 1u, sizeof(text) - 1u, f);

   fclose(f);
   text[n] = '\0';

   TJSON *root = TJSON::Decode(text, n);

   if (!root)
      return;

   const TJSON *dev = root->Child("device"),
               *mdl = dev ? dev->Child("model") : NULL,
               *cam = dev ? dev->Child("camera") : NULL;

   if (mdl)
      snprintf(model, cap, "%.*s", (int)mdl->len, mdl->text);
   if (cam)
   {
      char label[64];

      snprintf(label, sizeof(label), "%.*s", (int)cam->len, cam->text);

      LPCSTR open = strchr(label, '(');

      if (open)
         sscanf(open + 1, "%f", focalMm);
   }
   delete root;
}

//--------------------------------------------------------------------------------
// Decodes a JPEG into 8-bit luma or 24-bit BGR (stored pixel order, EXIF orientation ignored)
static bool inspectDecode(LPCBYTE jpg, size_t n, DWORD width, DWORD height, bool color, TAlloc<BYTE> &out)
{
#ifdef _WIN32
   IWICImagingFactory    *factory = NULL;
   IWICStream            *stream = NULL;
   IWICBitmapDecoder     *decoder = NULL;
   IWICBitmapFrameDecode *frame = NULL;
   IWICFormatConverter   *gray = NULL;

   UINT dw = 0u,
        dh = 0u;
   bool decoded = false;

   if (SUCCEEDED(CoCreateInstance(CLSID_WICImagingFactory, NULL, CLSCTX_INPROC_SERVER, IID_PPV_ARGS(&factory)))
       && SUCCEEDED(factory->CreateStream(&stream))
       && SUCCEEDED(stream->InitializeFromMemory((LPBYTE)jpg, (DWORD)n))
       && SUCCEEDED(factory->CreateDecoderFromStream(stream, NULL, WICDecodeMetadataCacheOnDemand, &decoder))
       && SUCCEEDED(decoder->GetFrame(0u, &frame))
       && SUCCEEDED(frame->GetSize(&dw, &dh))
       && SUCCEEDED(factory->CreateFormatConverter(&gray))
       && SUCCEEDED(gray->Initialize(frame, color ? GUID_WICPixelFormat24bppBGR : GUID_WICPixelFormat8bppGray,
                                     WICBitmapDitherTypeNone, NULL, 0.f, WICBitmapPaletteTypeCustom))
       && dw == (UINT)width && dh == (UINT)height)
      decoded = SUCCEEDED(gray->CopyPixels(NULL, (UINT)width*(color ? 3u : 1u), (UINT)(width*height*(color ? 3u : 1u)),
                                           out()));
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
   return decoded;
#else
   return false;
#endif
}

//--------------------------------------------------------------------------------
// Room-axis statistics of one room, printed when the room changes and at the end
struct TInspectAxis {
   int   aligned,
         misaligned,
         unverified;
   float sumDev,
         sumDev2,
         maxAbsDev,
         sumTilt,
         sumOrtho;
   int   tiltN,
         orthoN;
};

//--------------------------------------------------------------------------------
static void inspectAxisPrint(LPCSTR label, const TInspectAxis &a)
{
   int   judged = a.aligned + a.misaligned;
   float mean = judged ? a.sumDev/(float)judged : 0.f,
         var = judged ? a.sumDev2/(float)judged - mean*mean : 0.f;

   printf("  vanishing %s: %d aligned, %d misaligned, %d unverified; deviation mean %.2f sd %.2f max %.2f deg;"
          " tilt %.2f deg, ortho %.2f deg\n", label, a.aligned, a.misaligned, a.unverified, mean,
          var > 0.f ? sqrtf(var) : 0.f, a.maxAbsDev, a.tiltN ? a.sumTilt/(float)a.tiltN : 0.f,
          a.orthoN ? a.sumOrtho/(float)a.orthoN : 0.f);
}

//--------------------------------------------------------------------------------
static void inspectAxisAdd(TInspectAxis &a, const TVanishRecord &r)
{
   if (r.verdict == (BYTE)avAligned || r.verdict == (BYTE)avReference)
      a.aligned++;
   else if (r.verdict == (BYTE)avMisaligned)
      a.misaligned++;
   else
      a.unverified++;
   if (!isnan(r.deviationDeg))
   {
      a.sumDev += r.deviationDeg;
      a.sumDev2 += r.deviationDeg*r.deviationDeg;
      if (fabsf(r.deviationDeg) > a.maxAbsDev)
         a.maxAbsDev = fabsf(r.deviationDeg);
   }
   if (!isnan(r.tiltErrDeg))
   {
      a.sumTilt += r.tiltErrDeg;
      a.tiltN++;
   }
   if (!isnan(r.orthoErrDeg))
   {
      a.sumOrtho += r.orthoErrDeg;
      a.orthoN++;
   }
}

/*--------------------------------------------------------------------------------
   Edge picture of one keyframe (--edges): Sobel magnitude of the luma box-reduced by 5, turned
   upright (portrait, 90 degrees clockwise), with the edges the vanishing detector accepted on top:
   red vertical, green room axis A, cyan room axis B. 24-bit BMP.
  --------------------------------------------------------------------------------*/
static void inspectEdgeImage(LPCBYTE luma, int w, int h, const TIntrinsics &k, const TVanishEdges &edges,
                             LPCSTR path)
{
   const int    f = 5;
   int          sw = w/f,
                sh = h/f,
                ow = sh, // upright
                oh = sw,
                rowBytes = (ow*3 + 3) & ~3;
   TAlloc<BYTE> shrunk((size_t)sw*sh),
                rgb((size_t)sw*sh*3u);

   for (int y = 0; y < sh; y++)
      for (int x = 0; x < sw; x++)
      {
         DWORD sum = 0u;

         for (int dy = 0; dy < f; dy++)
            for (int dx = 0; dx < f; dx++)
               sum += luma[(size_t)(y*f + dy)*w + x*f + dx];
         shrunk[(size_t)y*sw + x] = (BYTE)(sum/(DWORD)(f*f));
      }
   memset(rgb(), 0, (size_t)sw*sh*3u);
   for (int y = 1; y + 1 < sh; y++)
      for (int x = 1; x + 1 < sw; x++)
      {
         LPCBYTE p = shrunk() + (size_t)y*sw + x;
         int     gx = (p[-sw + 1] + 2*p[1] + p[sw + 1]) - (p[-sw - 1] + 2*p[-1] + p[sw - 1]),
                 gy = (p[sw - 1] + 2*p[sw] + p[sw + 1]) - (p[-sw - 1] + 2*p[-sw] + p[-sw + 1]),
                 m = (abs(gx) + abs(gy))*2;
         BYTE    g = (BYTE)(m > 255 ? 255 : m);
         LPBYTE  o = rgb() + ((size_t)y*sw + x)*3u;

         o[0] = g;
         o[1] = g;
         o[2] = g;
      }
   for (DWORD e = 0; e < edges.count; e++)
   {
      float rx = edges.ray[3u*e],
            ry = edges.ray[3u*e + 1u],
            rz = edges.ray[3u*e + 2u];

      if (rz >= -1e-6f || !edges.label[e])
         continue;

      int x = (int)((k.cx + k.fx*rx/-rz)/(float)f),
          y = (int)((k.cy - k.fy*ry/-rz)/(float)f);

      if (x < 0 || x >= sw || y < 0 || y >= sh)
         continue;

      LPBYTE o = rgb() + ((size_t)y*sw + x)*3u; // BGR

      o[0] = edges.label[e] == 1u ? 0u : (edges.label[e] == 3u ? 255u : 0u);
      o[1] = edges.label[e] == 1u ? 0u : 255u;
      o[2] = edges.label[e] == 1u ? 255u : 0u;
   }

   FILE *bmp = fopen(path, "wb");

   if (!bmp)
      return;

   DWORD imageBytes = (DWORD)rowBytes*(DWORD)oh,
         header[13] = { 0u, 0u, 54u, 40u, (DWORD)ow, (DWORD)oh, 0x00180001u, 0u, imageBytes, 2835u, 2835u, 0u, 0u };
   BYTE  magic[2] = { 'B', 'M' };
   BYTE  pad[4] = {};

   header[0] = 54u + imageBytes;
   fwrite(magic, 1u, 2u, bmp);
   fwrite(header, 4u, 13u, bmp);
   for (int oy = oh - 1; oy >= 0; oy--) // bottom-up
   {
      for (int ox = 0; ox < ow; ox++)
      {
         int sx = oy,          // clockwise: output (ox, oy) <- source (oy, sh - 1 - ox)
             sy = sh - 1 - ox;

         fwrite(rgb() + ((size_t)sy*sw + sx)*3u, 1u, 3u, bmp);
      }
      fwrite(pad, 1u, (size_t)(rowBytes - ow*3), bmp);
   }
   fclose(bmp);
}

/*--------------------------------------------------------------------------------
   One frontal view: a pure rotation of the camera (homography K R K^-1, no depth needed) onto a
   virtual camera looking straight along the horizontal wall normal n, level and without roll -
   verticals vertical, that wall's creases horizontal. A shift of the principal point keeps the
   original view centered, like an architectural shift lens, so nothing tilts. 24-bit BMP, upright.
  --------------------------------------------------------------------------------*/
static void inspectFrontal(const TImageRecord &img, LPCBYTE bgr, const TVec3 &up, const TVec3 &n, LPCSTR path)
{
   const int    ow = 900,
                oh = 1200,
                rowBytes = (ow*3 + 3) & ~3;
   const float  f = 0.3f*img.intr.fx;
   int          w = (int)img.width,
                h = (int)img.height;
   TAlloc<BYTE> out((size_t)rowBytes*oh);

   // virtual camera: looks along n, y = true up, x = y cross z
   TVec3 zv = { -n.x, -n.y, -n.z },
         xv = { up.y*zv.z - up.z*zv.y, up.z*zv.x - up.x*zv.z, up.x*zv.y - up.y*zv.x };
   float elev = asinf(fmaxf(-0.9f, fminf(0.9f, -up.z))), // the real camera looks down -z
         yaw = atan2f(-xv.z, zv.z),
         cxv = 0.5f*(float)ow - f*tanf(fmaxf(-0.9f, fminf(0.9f, yaw))),
         cyv = 0.5f*(float)oh + f*tanf(elev);

   memset(out(), 0, (size_t)rowBytes*oh);
   for (int vv = 0; vv < oh; vv++)
   {
      LPBYTE row = out() + (size_t)(oh - 1 - vv)*rowBytes; // bottom-up

      for (int uu = 0; uu < ow; uu++)
      {
         float rx = ((float)uu - cxv)/f,
               ry = -((float)vv - cyv)/f,
               cx = xv.x*rx + up.x*ry - zv.x,
               cy = xv.y*rx + up.y*ry - zv.y,
               cz = xv.z*rx + up.z*ry - zv.z;

         if (cz > -1e-6f)
            continue;

         float su = img.intr.cx + img.intr.fx*cx/-cz,
               sv = img.intr.cy - img.intr.fy*cy/-cz;
         int   iu = (int)floorf(su),
               iv = (int)floorf(sv);

         if (iu < 0 || iv < 0 || iu + 1 >= w || iv + 1 >= h)
            continue;

         float du = su - (float)iu,
               dv = sv - (float)iv;

         for (int ch = 0; ch < 3; ch++)
         {
            LPCBYTE p = bgr + ((size_t)iv*w + iu)*3u + ch;
            float   top = (float)p[0]*(1.f - du) + (float)p[3]*du,
                    bot = (float)p[(size_t)w*3u]*(1.f - du) + (float)p[(size_t)w*3u + 3u]*du;

            row[uu*3 + ch] = (BYTE)(top*(1.f - dv) + bot*dv + 0.5f);
         }
      }
   }

   FILE *bmp = fopen(path, "wb");

   if (!bmp)
      return;

   DWORD imageBytes = (DWORD)rowBytes*(DWORD)oh,
         header[13] = { 54u + imageBytes, 0u, 54u, 40u, (DWORD)ow, (DWORD)oh, 0x00180001u, 0u, imageBytes, 2835u, 2835u,
                        0u, 0u };
   BYTE  magic[2] = { 'B', 'M' };

   fwrite(magic, 1u, 2u, bmp);
   fwrite(header, 4u, 13u, bmp);
   fwrite(out(), 1u, imageBytes, bmp);
   fclose(bmp);
}

/*--------------------------------------------------------------------------------
   Frontal views of one keyframe (--rectify). The room axes in camera axes come from the frame's
   own vanishing points (vertical measured or tilt-calibrated; A measured, or B turned about the
   vertical, or the frame's own heading prediction, or the room consensus). The wall the frame faces
   gets a frontal view; a frame aimed at a corner (more than 25 degrees off that wall) gets one per
   wall, since neither is frontal in it. File names tell the wall: rect_NNN_paredeK.bmp, K = 0..3 clockwise
   from the room axis (the same physical wall in every frame).
  --------------------------------------------------------------------------------*/
static void inspectRectify(const TImageRecord &img, const TVanishResult &vr, float refAxisDeg, LPCSTR outDir,
                           int index)
{
   TAlloc<BYTE> bgr((size_t)img.width*img.height*3u);

   if (!inspectDecode(img.pixels, img.pixelBytes, img.width, img.height, true, bgr))
      return;

   const TMat4 &pose = img.cameraToWorld;
   TVec3        up = vr.dirCam[0],
                a = vr.dirCam[1];

   if (!(vr.flags & vfAxisA) && (vr.flags & vfAxisB)) // only B measured: A is B turned about the vertical
   {
      const TVec3 &bm = vr.dirCam[2];

      a.x = bm.y*up.z - bm.z*up.y;
      a.y = bm.z*up.x - bm.x*up.z;
      a.z = bm.x*up.y - bm.y*up.x;
   }
   else if (!(vr.flags & (vfAxisA | vfAxisB)) && !isnan(refAxisDeg)) // no horizontal lines: the room consensus
   {
      float t = refAxisDeg*0.01745329f;
      TVec3 aw = { sinf(t), 0.f, -cosf(t) };

      a.x = pose.m[0]*aw.x + pose.m[1]*aw.y + pose.m[2]*aw.z;
      a.y = pose.m[4]*aw.x + pose.m[5]*aw.y + pose.m[6]*aw.z;
      a.z = pose.m[8]*aw.x + pose.m[9]*aw.y + pose.m[10]*aw.z;
   }

   float d = a.x*up.x + a.y*up.y + a.z*up.z;

   a.x -= d*up.x;
   a.y -= d*up.y;
   a.z -= d*up.z;

   float len = sqrtf(a.x*a.x + a.y*a.y + a.z*a.z);

   if (!(len > 1e-6f)) // also NaN
      return;
   a.x /= len;
   a.y /= len;
   a.z /= len;

   TVec3 b = { up.y*a.z - up.z*a.y, up.z*a.x - up.x*a.z, up.x*a.y - up.y*a.x };
   float fa = -a.z,  // camera forward (0, 0, -1) along each axis
         fb = -b.z;
   bool  facingA = fabsf(fa) >= fabsf(fb);

   for (int pass = 0; pass < 2; pass++)
   {
      bool  useA = pass == 0 ? facingA : !facingA;
      float s = (useA ? fa : fb) >= 0.f ? 1.f : -1.f;
      TVec3 n = useA ? a : b;
      float along = fabsf(useA ? fa : fb),
            across = fabsf(useA ? fb : fa);

      if (pass == 1 && atan2f(across, along) > 60.f*0.01745329f)
         break; // the second wall is barely seen: the frame faces the first one
      n.x *= s;
      n.y *= s;
      n.z *= s;

      // wall number from the WORLD heading of its normal against the room axis: the same wall in every frame
      TVec3 nw = pose.RotateVector(n);
      float axis = !isnan(refAxisDeg) ? refAxisDeg
                                      : (!isnan(vr.roomAxisDeg) ? vr.roomAxisDeg
                                                                : fmodf(geomHeadingDeg(pose.RotateVector(a)), 90.f)),
            rel = fmodf(geomHeadingDeg(nw) - axis + 720.f + 45.f, 360.f);
      int   wall = (int)(rel/90.f)%4;
      char  path[sessionPathMax];

      snprintf(path, sizeof(path), "%s/rect_%03d_parede%d.bmp", outDir, index, wall);
      inspectFrontal(img, bgr(), up, n, path);
      if (pass == 0 && atan2f(across, along) < 25.f*0.01745329f)
         break; // aimed at this wall: one frontal view
   }
}

/*--------------------------------------------------------------------------------
   Vanishing measure of one keyframe: the app's own rtVanish record (written right before the
   image) when its seq matches, otherwise decoded and measured here against the room's check.
   Center-spin frames are always decoded too: their edges feed the room's floor plan.
  --------------------------------------------------------------------------------*/
static void inspectMeasure(const TImageRecord &img, const TFrameMeta &meta, bool hasMeta, const TVanishRecord *app,
                           bool force, const TVanishConfig &cfg, TAxisCheck &axis, TVanishEdges &edges,
                           TRoomLayout &layout, TTiltBias &tilt, LPCSTR edgePath, LPCSTR rectDir, int index,
                           TVanishRecord &out, TVanishResult &kept)
{
   bool useApp = app && !force && hasMeta && app->seq == meta.seq && app->roomIndex == meta.roomIndex,
        center = !hasMeta || meta.stationKind == (BYTE)skCenter;

   out = TVanishRecord();
   out.roomIndex = hasMeta ? meta.roomIndex : 0u;
   out.seq = hasMeta ? meta.seq : 0u;
   out.verdict = (BYTE)avNoLines;
   out.roomAxisDeg = NAN;
   out.deviationDeg = NAN;
   out.tiltErrDeg = NAN;
   out.orthoErrDeg = NAN;
   if (useApp)
      out = *app;

   TAlloc<BYTE>  luma((size_t)img.width*img.height);
   TVanishResult vr = {};
   float         dev = NAN;
   bool          ok;
   TVanishConfig tuned = cfg;

   vr.roomAxisDeg = NAN;
   tilt.Apply(tuned);
   ok = inspectDecode(img.pixels, img.pixelBytes, img.width, img.height, false, luma)
        && vanishDetect(luma(), (int)img.width, (int)img.height, (int)img.width, img.intr, img.cameraToWorld,
                        tuned, vr, &edges);
   kept = vr; // the frame's own measure (or its predictions): the wall composition turns the frame with it
   if (ok)
      tilt.Add(vr);
   if (ok && edgePath)
      inspectEdgeImage(luma(), (int)img.width, (int)img.height, img.intr, edges, edgePath);
   if (ok && rectDir)
      inspectRectify(img, vr, axis.HasReference() ? axis.ReferenceDeg() : vr.roomAxisDeg, rectDir, index);
   if (ok)
      layout.AddFrame(img.cameraToWorld, vr, edges, tilt.Bias(), center ? 0 : (int)meta.stationIndex);
   if (useApp)
   {
      axis.Offer(vr, &dev); // the plan needs this tool's own room reference
      return;
   }
   if (!ok)
   {
      axis.Offer(vr, &dev); // counts the frame as unverified
      return;
   }
   out.verdict = (BYTE)axis.Offer(vr, &dev);
   out.flags = vr.flags;
   out.roomAxisDeg = vr.roomAxisDeg;
   out.deviationDeg = dev;
   out.tiltErrDeg = vr.tiltErrDeg;
   out.orthoErrDeg = vr.orthoErrDeg;
   for (int i = 0; i < vanishDirs; i++)
   {
      out.dirCam[i] = vr.dirCam[i];
      out.support[i] = vr.support[i];
   }
   out.edges = vr.edges;
}

//--------------------------------------------------------------------------------
static void inspectWalls(const TLayoutPlan &plan)
{
   for (int i = 0; i < plan.wallCount && i < layoutMaxWalls; i++)
      printf("    wall %c = %.2f along %.2f..%.2f weight %.0f\n", plan.wallKind[i] ? 'w' : 'u', plan.wallOffset[i],
             plan.wallA0[i], plan.wallA1[i], plan.wallWeight[i]);
}

/*--------------------------------------------------------------------------------
   Floor plan of the finished room: printed and drawn as plan_room<N>.svg (u up, w right, 50 px
   per meter), spin point, corners, planned stations and their aim lines.
  --------------------------------------------------------------------------------*/
static bool inspectPlan(LPCSTR outDir, DWORD room, const TRoomLayout &layout, const TAxisCheck &axis, float ceilingM,
                        TLayoutPlan &plan)
{
   char path[sessionPathMax];

   plan = TLayoutPlan();
   if (!axis.HasReference() || !layout.Solve(layout.AnchorDeg(), ceilingM, plan))
   {
      printf("  plan room %lu: not solved (%lu edges, reference %s, %d walls, failure %d, height %.2f %s)\n",
             (unsigned long)room, (unsigned long)layout.Count(), axis.HasReference() ? "yes" : "no", plan.wallCount,
             (int)plan.failure, plan.cameraHeightM, plan.heightSolved ? "solved" : "default");
      for (int i = 0; i < plan.vertexCount; i++)
         printf("    corner %d: u %.2f w %.2f\n", i, plan.verts[i].u, plan.verts[i].w);
      inspectWalls(plan);
      return false;
   }
   printf("  plan room %lu: %.2f x %.2f m, area %.2f m2, camera height %.2f m (%s), ceiling %.2f m, %d corners%s\n",
          (unsigned long)room, plan.extentU, plan.extentW, plan.areaM2, plan.cameraHeightM,
          plan.heightSolved ? "solved" : "default", plan.ceilingM, plan.vertexCount, plan.complete ? "" : " (incomplete)");
   printf("    door head: %s, scale %.3f (assumed ceiling %.2f m)\n", plan.doorFound ? "found" : "none", plan.doorScale,
          plan.assumedCeilingM);
   printf("    camera height from %d room corners; %d creases from the corner stations\n", plan.heightCorners,
          plan.stationLines);
   for (int i = 0; i < plan.vertexCount; i++)
      printf("    corner %d: u %.2f w %.2f %s\n", i, plan.verts[i].u, plan.verts[i].w, plan.convex[i] ? "" : "(reflex)");
   for (int i = 0; i < plan.stationCount; i++)
      printf("    station %d: corner %u aiming at corner %u\n", i + 1, (unsigned)plan.stationVertex[i],
             (unsigned)plan.targetVertex[i]);
   inspectWalls(plan);
   snprintf(path, sizeof(path), "%s/plan_room%lu.svg", outDir, (unsigned long)room);

   FILE *svg = fopen(path, "wb");

   if (!svg)
      return true; // solved, only the picture could not be written

   const float scale = 50.f,
               cx = 300.f,
               cy = 300.f;

   fprintf(svg, "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"600\" height=\"640\" viewBox=\"0 0 600 640\">\n");
   fprintf(svg, "<rect width=\"600\" height=\"640\" fill=\"#15161a\"/>\n<polygon fill=\"#2a3140\" stroke=\"#fff\" "
                "stroke-width=\"3\" points=\"");
   for (int i = 0; i < plan.vertexCount; i++)
      fprintf(svg, "%.1f,%.1f ", cx + plan.verts[i].w*scale, cy - plan.verts[i].u*scale);
   fprintf(svg, "\"/>\n");
   for (int i = 0; i < plan.stationCount; i++)
   {
      const TPlanPoint &a = plan.verts[plan.stationVertex[i]],
                       &b = plan.verts[plan.targetVertex[i]];

      fprintf(svg, "<line x1=\"%.1f\" y1=\"%.1f\" x2=\"%.1f\" y2=\"%.1f\" stroke=\"#ffd200\" stroke-width=\"1.5\" "
                   "stroke-dasharray=\"6 4\"/>\n", cx + a.w*scale, cy - a.u*scale, cx + b.w*scale, cy - b.u*scale);
      fprintf(svg, "<circle cx=\"%.1f\" cy=\"%.1f\" r=\"8\" fill=\"#3cdc6e\"/><text x=\"%.1f\" y=\"%.1f\" fill=\"#000\" "
                   "font-size=\"11\" text-anchor=\"middle\">%d</text>\n", cx + a.w*scale, cy - a.u*scale,
              cx + a.w*scale, cy - a.u*scale + 4.f, i + 1);
   }
   fprintf(svg, "<circle cx=\"%.1f\" cy=\"%.1f\" r=\"6\" fill=\"#ffd200\"/>\n", cx, cy);
   fprintf(svg, "<text x=\"300\" y=\"625\" fill=\"#ddd\" font-family=\"sans-serif\" font-size=\"16\" "
                "text-anchor=\"middle\">%.2f x %.2f m  -  %.1f m2  -  pe-direito %.2f m</text>\n</svg>\n",
           plan.extentU, plan.extentW, plan.areaM2, plan.ceilingM);
   fclose(svg);
   return true;
}

/*--------------------------------------------------------------------------------
   Camera-to-world rotation of one keyframe corrected by its own vanishing points: its true vertical
   goes to world up and its room axis to the plan axis (anchor + 90k, the k nearest to where the
   gyroscope thinks it points). The spin frames share one center, so between them there is rotation
   only: with these rotations they overlay exactly, no depth needed.
  --------------------------------------------------------------------------------*/
static bool inspectFrameRotation(const TMat4 &pose, const TVanishResult &vr, float axisDeg, TVec3 &camA, TVec3 &camUp,
                                 TVec3 &camC, TVec3 &worldA, TVec3 &worldC)
{
   TVec3 up = vr.dirCam[0],
         a = vr.dirCam[1];

   if (!(vr.flags & vfAxisA) && (vr.flags & vfAxisB))
   {
      const TVec3 &b = vr.dirCam[2];

      a.x = b.y*up.z - b.z*up.y;
      a.y = b.z*up.x - b.x*up.z;
      a.z = b.x*up.y - b.y*up.x;
   }
   if (!(vr.flags & (vfAxisA | vfAxisB)))
      return false; // no room axis measured: the frame cannot be trusted for the overlay

   float d = a.x*up.x + a.y*up.y + a.z*up.z;

   a.x -= d*up.x;
   a.y -= d*up.y;
   a.z -= d*up.z;

   float len = sqrtf(a.x*a.x + a.y*a.y + a.z*a.z);

   if (!(len > 1e-6f))
      return false;
   a.x /= len;
   a.y /= len;
   a.z /= len;

   float gyro = geomHeadingDeg(pose.RotateVector(a)),
         k = floorf((gyro - axisDeg)/90.f + 0.5f),
         t = (axisDeg + 90.f*k)*0.01745329f;

   if (fabsf(gyro - axisDeg - 90.f*k) > 12.f)
      return false; // the frame's axis disagrees with the gyroscope beyond any drift: a 45-degree floor pattern, a stray line

   camA = a;
   camUp = up;
   camC.x = a.y*up.z - a.z*up.y;
   camC.y = a.z*up.x - a.x*up.z;
   camC.z = a.x*up.y - a.y*up.x;
   worldA.x = sinf(t);
   worldA.y = 0.f;
   worldA.z = -cosf(t);
   worldC.x = -worldA.z; // worldA x up
   worldC.y = 0.f;
   worldC.z = worldA.x;
   return true;
}

/*--------------------------------------------------------------------------------
   Wall orthophotos (--walls): the room's keyframes are loaded with their vanishing-point rotations,
   pictures halved (about 2 mm per pixel on the walls) - the center spin first, then the corner
   stations. A corner station starts at the plan corner most opposite to where it aims (it looks at
   the opposite corner), a quarter of the way in; the bundle adjustment then finds where it really
   stood. capMosaic refines everything and composes every wall at 4 mm per pixel; the corner views
   fill what the center spin did not see. parede_K_<length>m.bmp.
  --------------------------------------------------------------------------------*/
static void inspectComposeWalls(LPCSTR sessionDir, LPCSTR outDir, DWORD room, const TLayoutPlan &plan,
                                const TVanishResult *frameVr, const DWORD *frameInfo, int frames, float spinRadiusM,
                                int rounds, int bundleRounds, int jointRounds)
{
   TAlloc<TMosaicFrame> list((size_t)inspectMaxFrames);
   int                  count = 0,
                        centers = 0;

   for (int pass = 0; pass < 2; pass++) // 0: center spin, 1: corner stations
   {
      TSessionReader reader;
      TRecordView    v;
      int            index = 0;

      if (!reader.Open(sessionDir))
         return;
      while (reader.Next(v) && count < inspectMaxFrames)
      {
         TImageRecord img;

         if (v.type != rtImage || !img.Decode(v.payload, v.length))
            continue;

         int f = index++;

         if (f >= frames || f >= inspectMaxFrames || frameInfo[f]/inspectStations != room)
            continue;

         int station = (int)(frameInfo[f]%inspectStations);

         if ((pass == 0) != (station == 0))
            continue;

         TMosaicFrame &m = list[count];

         if (!inspectFrameRotation(img.cameraToWorld, frameVr[f], plan.axisDeg, m.camA, m.camUp, m.camC, m.worldA,
                                   m.worldC))
            continue;

         TVec3 ex = { 1.f, 0.f, 0.f },
               ey = { 0.f, 1.f, 0.f },
               ez = { 0.f, 0.f, 1.f };

         m.gyroX = img.cameraToWorld.RotateVector(ex);
         m.gyroY = img.cameraToWorld.RotateVector(ey);
         m.gyroZ = img.cameraToWorld.RotateVector(ez);

         int          w = (int)img.width,
                      h = (int)img.height;
         TAlloc<BYTE> full((size_t)w*h*3u);

         if (!inspectDecode(img.pixels, img.pixelBytes, img.width, img.height, true, full))
            continue;

         // halved picture (2x2 box) and its pinhole
         m.w = w/2;
         m.h = h/2;
         {
            TAlloc<BYTE> half((size_t)m.w*m.h*3u);

            for (int y = 0; y < m.h; y++)
               for (int x = 0; x < m.w; x++)
                  for (int ch = 0; ch < 3; ch++)
                  {
                     size_t p = ((size_t)(2*y)*w + 2*x)*3u + ch;

                     half[((size_t)y*m.w + x)*3u + ch] = (BYTE)((full[p] + full[p + 3u] + full[p + (size_t)w*3u]
                                                                + full[p + (size_t)w*3u + 3u] + 2u)/4u);
                  }
            half.Drop(m.bgr);
         }
         m.k.fx = 0.5f*img.intr.fx;
         m.k.fy = 0.5f*img.intr.fy;
         m.k.cx = 0.5f*(img.intr.cx + 0.5f) - 0.5f;
         m.k.cy = 0.5f*(img.intr.cy + 0.5f) - 0.5f;
         m.index = f;
         m.station = station;
         m.stationX = 0.f;
         m.stationZ = 0.f;
         m.delta = TVec3();
         count++;
         if (pass == 0)
            centers++;
      }
   }

   // corner stations: start at the plan corner most opposite to their mean aim, a quarter of the way in
   float a = plan.axisDeg*0.01745329f;
   TVec3 ud = { sinf(a), 0.f, -cosf(a) },
         wd = { cosf(a), 0.f, sinf(a) };

   for (int s = 1; s < inspectStations && s < 64; s++)
   {
      float ax = 0.f,
            az = 0.f;
      int   n = 0;

      for (int i = centers; i < count; i++)
         if (list[i].station == s)
         {
            ax += list[i].worldA.x*(-list[i].camA.z) + list[i].worldC.x*(-list[i].camC.z);
            az += list[i].worldA.z*(-list[i].camA.z) + list[i].worldC.z*(-list[i].camC.z);
            n++;
         }
      if (!n)
         continue;

      int   best = 0;
      float bestCos = 2.f;

      for (int k = 0; k < plan.vertexCount; k++)
      {
         float px = plan.verts[k].u*ud.x + plan.verts[k].w*wd.x,
               pz = plan.verts[k].u*ud.z + plan.verts[k].w*wd.z,
               cs = (px*ax + pz*az)/(sqrtf(px*px + pz*pz)*sqrtf(ax*ax + az*az) + 1e-6f);

         if (cs < bestCos)
         {
            bestCos = cs;
            best = k;
         }
      }

      float sx = 0.75f*(plan.verts[best].u*ud.x + plan.verts[best].w*wd.x),
            sz = 0.75f*(plan.verts[best].u*ud.z + plan.verts[best].w*wd.z);

      for (int i = centers; i < count; i++)
         if (list[i].station == s)
         {
            list[i].stationX = sx;
            list[i].stationZ = sz;
         }
      printf("  station %d: %d frames, starts near plan corner %d (%.2f, %.2f)\n", s, n, best, sx, sz);
   }

   // every camera center: its station plus the spin circle out along its horizontal forward
   for (int i = 0; i < count; i++)
   {
      TMosaicFrame &m = list[i];
      float         fwdX = m.worldA.x*(-m.camA.z) + m.worldC.x*(-m.camC.z),
                    fwdZ = m.worldA.z*(-m.camA.z) + m.worldC.z*(-m.camC.z),
                    fwdLen = sqrtf(fwdX*fwdX + fwdZ*fwdZ);

      m.camX = m.stationX + (fwdLen > 1e-6f ? spinRadiusM*fwdX/fwdLen : 0.f);
      m.camZ = m.stationZ + (fwdLen > 1e-6f ? spinRadiusM*fwdZ/fwdLen : 0.f);
   }
   printf("  walls: %d walls from %d center-spin and %d corner-station frames\n", plan.vertexCount, centers,
          count - centers);
   mosaicWalls(list(), count, centers, plan, rounds, bundleRounds, jointRounds, spinRadiusM, outDir);
}

/*--------------------------------------------------------------------------------
   Room box over every center-spin keyframe (--overlay): the four ceiling corners of the plan and,
   H below, the four floor corners give each wall as a full rectangle; its 12 edges are projected
   with the frame's vanishing-point rotation and the spin radius - yellow ceiling, cyan floor (drawn
   even where furniture hides it), magenta corners. Where the drawing sits on the real creases the
   geometry holds; the gap is the correction still to make. BMP, upright, a quarter of the size.
  --------------------------------------------------------------------------------*/
static void inspectOverlay(LPCSTR sessionDir, LPCSTR outDir, DWORD room, const TLayoutPlan &plan,
                           const TVanishResult *frameVr, const DWORD *frameInfo, int frames, float spinRadiusM)
{
   const int   shrink = 4,
               samples = 600;
   float       a = plan.axisDeg*0.01745329f,
               ceilY = plan.ceilingM - plan.cameraHeightM,
               floorY = -plan.cameraHeightM;
   TVec3       ud = { sinf(a), 0.f, -cosf(a) },
               wd = { cosf(a), 0.f, sinf(a) };

   TSessionReader reader;
   TRecordView    v;
   int            index = 0;

   if (!reader.Open(sessionDir))
      return;
   while (reader.Next(v))
   {
      TImageRecord img;

      if (v.type != rtImage || !img.Decode(v.payload, v.length))
         continue;

      int f = index++;

      if (f >= frames || f >= inspectMaxFrames || frameInfo[f] != room*inspectStations)
         continue;

      TVec3 camA,
            camUp,
            camC,
            worldA,
            worldC;

      if (!inspectFrameRotation(img.cameraToWorld, frameVr[f], plan.axisDeg, camA, camUp, camC, worldA, worldC))
         continue;

      int          w = (int)img.width,
                   h = (int)img.height,
                   sw = w/shrink,
                   sh = h/shrink;
      TAlloc<BYTE> bgr((size_t)w*h*3u),
                   reduced((size_t)sw*sh*3u);

      if (!inspectDecode(img.pixels, img.pixelBytes, img.width, img.height, true, bgr))
         continue;
      for (int y = 0; y < sh; y++)
         for (int x = 0; x < sw; x++)
            for (int ch = 0; ch < 3; ch++)
               reduced[((size_t)y*sw + x)*3u + ch] = bgr[((size_t)(y*shrink)*w + x*shrink)*3u + ch];

      float fwdX = worldA.x*(-camA.z) + worldC.x*(-camC.z),
            fwdZ = worldA.z*(-camA.z) + worldC.z*(-camC.z),
            fwdLen = sqrtf(fwdX*fwdX + fwdZ*fwdZ),
            camX = fwdLen > 1e-6f ? spinRadiusM*fwdX/fwdLen : 0.f,
            camZ = fwdLen > 1e-6f ? spinRadiusM*fwdZ/fwdLen : 0.f;

      // 12 edges: per corner i -> i+1 on the ceiling and on the floor, plus the vertical at corner i
      for (int i = 0; i < plan.vertexCount; i++)
         for (int kind = 0; kind < 3; kind++)
         {
            const TPlanPoint &p = plan.verts[i],
                             &q = plan.verts[(i + 1)%plan.vertexCount];
            BYTE              color[3] = { 0u, 0u, 0u };

            color[0] = kind == 1 ? 255u : (kind == 2 ? 255u : 0u); // B
            color[1] = kind == 0 ? 220u : (kind == 1 ? 255u : 0u); // G
            color[2] = kind == 0 ? 255u : (kind == 2 ? 255u : 0u); // R
            for (int sIdx = 0; sIdx <= samples; sIdx++)
            {
               float t = (float)sIdx/(float)samples,
                     pu = kind == 2 ? p.u : p.u + (q.u - p.u)*t,
                     pw = kind == 2 ? p.w : p.w + (q.w - p.w)*t,
                     py = kind == 0 ? ceilY : (kind == 1 ? floorY : floorY + (ceilY - floorY)*t);
               TVec3 wp = { pu*ud.x + pw*wd.x - camX, py, pu*ud.z + pw*wd.z - camZ };
               float ka = wp.x*worldA.x + wp.z*worldA.z,
                     kc = wp.x*worldC.x + wp.z*worldC.z;
               TVec3 rc = { camA.x*ka + camUp.x*wp.y + camC.x*kc, camA.y*ka + camUp.y*wp.y + camC.y*kc,
                            camA.z*ka + camUp.z*wp.y + camC.z*kc };

               if (rc.z > -1e-3f)
                  continue;

               int x = (int)((img.intr.cx + img.intr.fx*rc.x/-rc.z)/(float)shrink),
                   y = (int)((img.intr.cy - img.intr.fy*rc.y/-rc.z)/(float)shrink);

               for (int dy = -1; dy <= 1; dy++)
                  for (int dx = -1; dx <= 1; dx++)
                     if (x + dx >= 0 && x + dx < sw && y + dy >= 0 && y + dy < sh)
                        for (int ch = 0; ch < 3; ch++)
                           reduced[((size_t)(y + dy)*sw + x + dx)*3u + ch] = color[ch];
            }
         }

      // upright (90 degrees clockwise) BMP
      int  ow = sh,
           oh = sw,
           rowBytes = (ow*3 + 3) & ~3;
      char path[sessionPathMax];

      snprintf(path, sizeof(path), "%s/overlay_%03d.bmp", outDir, f);

      FILE *bmp = fopen(path, "wb");

      if (!bmp)
         continue;

      DWORD imageBytes = (DWORD)rowBytes*(DWORD)oh,
            header[13] = { 54u + imageBytes, 0u, 54u, 40u, (DWORD)ow, (DWORD)oh, 0x00180001u, 0u, imageBytes, 2835u,
                           2835u, 0u, 0u };
      BYTE  magic[2] = { 'B', 'M' },
            pad[4] = {};

      fwrite(magic, 1u, 2u, bmp);
      fwrite(header, 4u, 13u, bmp);
      for (int oy = oh - 1; oy >= 0; oy--)
      {
         for (int ox = 0; ox < ow; ox++)
            fwrite(reduced() + ((size_t)(sh - 1 - ox)*sw + oy)*3u, 1u, 3u, bmp);
         fwrite(pad, 1u, (size_t)(rowBytes - ow*3), bmp);
      }
      fclose(bmp);
   }
}

//--------------------------------------------------------------------------------
int main(int argc, LPSTR *argv)
{
   abSetIdlePriority();
   if (argc < 3)
   {
      fprintf(stderr, "usage: capInspect <sessionDir> <outDir> [--vanish] [--ceiling meters] [--mingrad n] [--edges] [--rectify]\n");
      return 2;
   }

   TSessionReader reader;
   TRecordView    v;
   char           path[sessionPathMax];
   QWORD          counts[rtLayout + 1] = {},
                  firstNs = 0u,
                  lastNs = 0u;
   int            images = 0;
   TByteBuf       fixed,
                  exifBuf;
   char           model[64];
   float          modelFocalMm;
   TAxisCheck     axis(2.f);
   TInspectAxis   roomStats = {},
                  allStats = {};
   TVanishRecord  appVanish = {};
   bool           hasAppVanish = false,
                  force = false,
                  edgeImages = false,
                  rectify = false,
                  walls = false,
                  overlay = false;
   DWORD          statRoom = ~0ul;
   TRoomLayout    layout;
   TTiltBias      tilt;
   TLayoutPlan    plan;
   TAlloc<TVanishResult> frameVr((size_t)inspectMaxFrames);
   TAlloc<DWORD>  frameInfo((size_t)inspectMaxFrames); // room*inspectStations + station (0: the center spin)
   TVanishResult  scratchVr;
   TVanishConfig  vcfg = TVanishConfig::Default();
   TAlloc<float>  rays((size_t)vcfg.maxEdges*3u);
   TAlloc<BYTE>   labels((size_t)vcfg.maxEdges);
   TVanishEdges   edges = { rays(), labels(), (DWORD)vcfg.maxEdges, 0u };
   int            refineRounds = 6, // subpixel rounds of the wall mosaic (each frame against the others)
                  bundleRounds = 5, // bundle adjustment rounds: rotations with the geometry
                  jointRounds = 0;  // joint ceiling-anchored rounds (experimental)
   float          spinRadiusM = 0.3f, // phone circle around the operator's body during the spin
                  ceilingM = 2.8f;

   for (int i = 3; i < argc; i++)
      if (!strcmp(argv[i], "--vanish"))
         force = true;
      else if (!strcmp(argv[i], "--ceiling") && i + 1 < argc)
         ceilingM = (float)atof(argv[++i]);
      else if (!strcmp(argv[i], "--mingrad") && i + 1 < argc)
         vcfg.minGrad = atoi(argv[++i]);
      else if (!strcmp(argv[i], "--edges"))
         edgeImages = true;
      else if (!strcmp(argv[i], "--rectify"))
         rectify = true;
      else if (!strcmp(argv[i], "--walls"))
         walls = true;
      else if (!strcmp(argv[i], "--overlay"))
         overlay = true;
      else if (!strcmp(argv[i], "--refine") && i + 1 < argc)
         refineRounds = atoi(argv[++i]);
      else if (!strcmp(argv[i], "--joint") && i + 1 < argc)
         jointRounds = atoi(argv[++i]);
      else if (!strcmp(argv[i], "--bundle") && i + 1 < argc)
         bundleRounds = atoi(argv[++i]);
      else if (!strcmp(argv[i], "--spin-radius") && i + 1 < argc)
         spinRadiusM = (float)atof(argv[++i]);

#ifdef _WIN32
   CoInitializeEx(NULL, COINIT_MULTITHREADED);
#endif
   if (!reader.Open(argv[1]))
   {
      fprintf(stderr, "capInspect: cannot open session %s\n", argv[1]);
      return 1;
   }
   inspectReadDevice(argv[1], model, sizeof(model), &modelFocalMm);
   inspectMakeDir(argv[2]);
   snprintf(path, sizeof(path), "%s/frames.csv", argv[2]);

   FILE *frames = fopen(path, "wb");

   snprintf(path, sizeof(path), "%s/poses.csv", argv[2]);

   FILE *poses = fopen(path, "wb");

   if (!frames || !poses)
   {
      fprintf(stderr, "capInspect: cannot write into %s\n", argv[2]);
      return 1;
   }
   fprintf(frames, "seq,room,station,kind,corner,target,band,bin,headingDeg,pitchDeg,sensorNs,width,height,fx,fy,cx,cy,"
                   "jpegBytes,file,vanishFlags,axisVerdict,roomAxisDeg,axisDevDeg,tiltErrDeg,orthoErrDeg,supV,supA,supB,"
                   "edges\r\n");
   fprintf(poses, "sensorNs,headingDeg,pitchDeg,tracking\r\n");
   while (reader.Next(v))
   {
      if ((int)v.type <= (int)rtLayout)
         counts[v.type]++;
      if (!firstNs)
         firstNs = v.stampNs;
      lastNs = v.stampNs;
      if (v.type == rtPose)
      {
         TPoseRecord p;

         if (p.Decode(v.payload, v.length))
         {
            TVec3 f = p.cameraToWorld.Forward();

            fprintf(poses, "%llu,%.2f,%.2f,%d\r\n", (unsigned long long)v.stampNs, geomHeadingDeg(f), geomPitchDeg(f),
                    (int)p.tracking);
         }
      }
      else if (v.type == rtRoom)
      {
         TRoomRecord r;

         if (r.Decode(v.payload, v.length))
            printf("room %lu %s \"%s\" at %llu ns\n", (unsigned long)r.index, r.event == reBegin ? "begin" : "end",
                   r.name, (unsigned long long)v.stampNs);
      }
      else if (v.type == rtStation)
      {
         TStationRecord st;

         if (st.Decode(v.payload, v.length))
         {
            if (st.kind == skCenter && st.event == seBegin)
               printf("  station %lu: center spin\n", (unsigned long)st.index);
            else if (st.kind != skCenter)
               printf("  station %lu %s: corner %u aiming at corner %u\n", (unsigned long)st.index,
                      st.event == seBegin ? "begin" : "end", (unsigned)st.corner, (unsigned)st.target);
         }
      }
      else if (v.type == rtImage)
      {
         TImageRecord img;
         TFrameMeta   meta = {};
         size_t       metaLen = 0u;

         if (!img.Decode(v.payload, v.length))
            continue;
         if (!images)
            printf("lens: %lux%lu fx %.1f fy %.1f cx %.1f cy %.1f distortion %.4f %.4f %.4f %.4f %.4f\n",
                   (unsigned long)img.width, (unsigned long)img.height, img.intr.fx, img.intr.fy, img.intr.cx,
                   img.intr.cy, img.distortion[0], img.distortion[1], img.distortion[2], img.distortion[3],
                   img.distortion[4]);

         LPCBYTE block = TFrameMeta::Find(img.pixels, img.pixelBytes, &metaLen);
         bool    hasMeta = block && meta.Decode(block, metaLen);

         snprintf(path, sizeof(path), "%s/frame_%03d.jpg", argv[2], images);

         FILE   *jpg = fopen(path, "wb");
         LPCBYTE outBytes = img.pixels;
         size_t  outLen = img.pixelBytes;

         fixed.Clear();
         if (hasMeta && !inspectHasEXIF(img.pixels, img.pixelBytes)) // captures made before the app wrote EXIF
         {
            TEXIFInfo info = {};

            snprintf(info.model, sizeof(info.model), "%s", model);
            snprintf(info.software, sizeof(info.software), "Aeroblox LiDAR capInspect");
            info.wallNs = meta.wallNs;
            info.orientation = exifOrientation(img.cameraToWorld);
            info.focalMm = meta.focalMm > 0.f ? meta.focalMm : modelFocalMm;
            info.width = img.width;
            info.height = img.height;
            info.hasGPS = meta.horizAccMm != 0u;
            info.latE7 = meta.latE7;
            info.lonE7 = meta.lonE7;
            info.altMm = meta.altMm;
            exifBuf.Clear();
            exifBuild(info, exifBuf);
            if (jpegInsertSegment(img.pixels, img.pixelBytes, 0xE1, exifBuf.Data(), exifBuf.Size(), fixed))
            {
               outBytes = fixed.Data();
               outLen = fixed.Size();
            }
         }
         if (jpg)
         {
            fwrite(outBytes, 1u, outLen, jpg);
            fclose(jpg);
         }
         TVanishRecord vm;
         DWORD         room = hasMeta ? meta.roomIndex : 0u;

         if (room != statRoom)
         {
            if (statRoom != ~0ul)
            {
               inspectAxisPrint("room", roomStats);
               inspectPlan(argv[2], statRoom, layout, axis, ceilingM, plan);
            }
            roomStats = TInspectAxis();
            axis.Reset();
            layout.Reset();
            statRoom = room;
         }
         snprintf(path, sizeof(path), "%s/edges_%03d.bmp", argv[2], images);
         inspectMeasure(img, meta, hasMeta, hasAppVanish ? &appVanish : NULL, force, vcfg, axis, edges, layout,
                        tilt, edgeImages ? path : NULL, rectify ? argv[2] : NULL, images, vm,
                        images < inspectMaxFrames ? frameVr[images] : scratchVr);
         if (images < inspectMaxFrames)
            frameInfo[images] = (DWORD)(hasMeta ? meta.roomIndex : 0u)*inspectStations + (hasMeta ? meta.stationIndex : 0u);
         inspectAxisAdd(roomStats, vm);
         inspectAxisAdd(allStats, vm);
         hasAppVanish = false;
         fprintf(frames, "%lu,%lu,%u,%s,%u,%u,%d,%d,%.2f,%.2f,%llu,%lu,%lu,%.1f,%.1f,%.1f,%.1f,%lu,frame_%03d.jpg,",
                 hasMeta ? (unsigned long)meta.seq : 0ul, hasMeta ? (unsigned long)meta.roomIndex : 0ul,
                 (unsigned)meta.stationIndex, meta.stationKind == (BYTE)skCorner ? "corner" : "center",
                 meta.cornerIndex + 1u, meta.targetCorner + 1u,
                 hasMeta ? meta.spinBand : -1, hasMeta ? meta.spinBin : -1,
                 geomHeadingDeg(img.cameraToWorld.Forward()), geomPitchDeg(img.cameraToWorld.Forward()),
                 (unsigned long long)v.stampNs, (unsigned long)img.width, (unsigned long)img.height, img.intr.fx,
                 img.intr.fy, img.intr.cx, img.intr.cy, (unsigned long)img.pixelBytes, images);
         fprintf(frames, "%u,%u,%.2f,%.2f,%.2f,%.2f,%lu,%lu,%lu,%lu\r\n", (unsigned)vm.flags, (unsigned)vm.verdict,
                 vm.roomAxisDeg, vm.deviationDeg, vm.tiltErrDeg, vm.orthoErrDeg, (unsigned long)vm.support[0],
                 (unsigned long)vm.support[1], (unsigned long)vm.support[2], (unsigned long)vm.edges);
         images++;
      }
      else if (v.type == rtVanish)
         hasAppVanish = appVanish.Decode(v.payload, v.length);
      else if (v.type == rtLayout)
      {
         TLayoutRecord lr;

         if (!lr.Decode(v.payload, v.length))
            continue;
         printf("  app plan room %lu (after %lu images): flags %u, %u corners, %u stations, axis %.2f, camera %.2f m\n",
                (unsigned long)lr.roomIndex, (unsigned long)images, (unsigned)lr.flags, (unsigned)lr.vertexCount,
                (unsigned)lr.stationCount, lr.axisDeg, lr.cameraHeightM);
      }
   }
   if (statRoom != ~0ul)
   {
      inspectAxisPrint("room", roomStats);
      if (inspectPlan(argv[2], statRoom, layout, axis, ceilingM, plan))
      {
         if (walls)
            inspectComposeWalls(argv[1], argv[2], statRoom, plan, frameVr(), frameInfo(), images, spinRadiusM,
                                refineRounds, bundleRounds, jointRounds);
         if (overlay)
            inspectOverlay(argv[1], argv[2], statRoom, plan, frameVr(), frameInfo(), images, spinRadiusM);
      }
      inspectAxisPrint("session", allStats);
   }
#ifdef _WIN32
   CoUninitialize();
#endif
   fclose(frames);
   fclose(poses);
   printf("records:");
   for (int t = 1; t <= (int)rtLayout; t++)
      printf(" %s=%llu", inspectTypeName((TRecordType)t), (unsigned long long)counts[t]);
   printf("\nspan: %.1f s, truncated tail: %s\n", (float)(lastNs - firstNs)*1e-9f, reader.Truncated() ? "yes" : "no");
   return 0;
}

//--------------------------------------------------------------------------------
