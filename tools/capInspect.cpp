#include "winTypes.h"
#include "idle.h"
#include "capSession.h"
#include "capFrameMeta.h"
#include "capEXIF.h"
#include "capVanish.h"
#include "capLayout.h"
#include "capBlur.h"
#include "capDoor.h"
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
 *   capInspect <sessionDir> <outDir> [--vanish] [--ceiling meters] [--mingrad n] [--edges] [--rectify] [--walls] [--blur] [--doors] [--floor] [--panorama] [--faces]
 *
 * Extracts every keyframe JPEG (with its embedded LIDARCAP block) to outDir, writes frames.csv with the
 * per-frame correlation metadata and poses.csv with the attitude stream, and prints a summary. Frames
 * captured before the app measured vanishing points (or every frame, with --vanish) are decoded through
 * WIC and measured here with the same core detector, room by room. */

enum {
   inspectMaxFrames = 512,  // keyframes whose measures are kept for the wall composition
   inspectWallPxPerM = 250, // wall orthophoto resolution: 4 mm per pixel
   inspectStations   = 512, // frame info: room*inspectStations + station index
   inspectRectReach  = 3    // --rectify: a frame without a crease borrows the corrections of measured ones this close
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

      case rtDoor :
         return "door";

      case rtElect :
         return "elect";

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

//--------------------------------------------------------------------------------
// TIFF words of an EXIF block in its own byte order
static DWORD inspectTiff16(LPCBYTE t, size_t o, bool le)
{
   return le ? (DWORD)t[o] | ((DWORD)t[o + 1] << 8) : ((DWORD)t[o] << 8) | (DWORD)t[o + 1];
}

//--------------------------------------------------------------------------------
static DWORD inspectTiff32(LPCBYTE t, size_t o, bool le)
{
   return le ? inspectTiff16(t, o, le) | (inspectTiff16(t, o + 2, le) << 16)
             : (inspectTiff16(t, o, le) << 16) | inspectTiff16(t, o + 2, le);
}

/*--------------------------------------------------------------------------------
   The EXIF Software tag (0x0131, first IFD) rewritten in place to the project's name, the rest of
   the old value from " LiDAR" on kept (the version), NUL-padded to the tag's own length. True when
   it changed; false without EXIF, without the tag, or when it already carries the name.
  --------------------------------------------------------------------------------*/
static bool inspectEXIFSoftware(LPBYTE jpg, size_t n)
{
   const char name[] = "Sorena";
   size_t     pos = 2u;

   while (pos + 10u <= n && jpg[pos] == 0xFF && jpg[pos + 1u] != 0xDA)
   {
      size_t len = ((size_t)jpg[pos + 2u] << 8) | jpg[pos + 3u];

      if (jpg[pos + 1u] != 0xE1 || memcmp(jpg + pos + 4u, "Exif", 4u))
      {
         pos += 2u + len;
         continue;
      }

      LPBYTE tiff = jpg + pos + 10u;
      size_t room = pos + 2u + len > n ? 0u : len - 8u; // the TIFF block's own bytes
      bool   le = tiff[0] == 'I';

      if (room < 8u)
         return false;

      size_t ifd = inspectTiff32(tiff, 4u, le);

      if (ifd + 2u > room)
         return false;
      for (DWORD e = 0; e < inspectTiff16(tiff, ifd, le) && ifd + 2u + 12u*(e + 1u) <= room; e++)
      {
         size_t ent = ifd + 2u + 12u*e;
         DWORD  count = inspectTiff32(tiff, ent + 4u, le);

         if (inspectTiff16(tiff, ent, le) != 0x0131u || inspectTiff16(tiff, ent + 2u, le) != 2u || count < sizeof(name))
            continue;

         size_t at = count <= 4u ? ent + 8u : inspectTiff32(tiff, ent + 8u, le);

         if (at + count > room || !memcmp(tiff + at, name, sizeof(name) - 1))
            return false;

         char   old[256] = {};
         size_t keep = count < sizeof(old) ? count : sizeof(old) - 1;

         memcpy(old, tiff + at, keep);

         LPCSTR rest = strstr(old, " LiDAR");

         memset(tiff + at, 0, count);
         snprintf((LPSTR)tiff + at, count, "%s%s", name, rest ? rest : " LiDAR");
         return true;
      }
      return false;
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

// The ceiling crease as the frontal view shows it: slope (rows per column) and elevation (the virtual camera is level)
struct TRectMeasure {
   bool  ok,
         floorLine; // the floor crease (no ceiling crease in the view): its own median per wall
   int   wall,
         points;   // columns on the fitted line
   float slope,
         elevDeg;  // of the line at the view's middle column, above the level horizon
};

static TRectMeasure inspectRect[inspectMaxFrames]; // first pass of --rectify: each frame's crease as rotated by its own vertical
static QWORD        inspectStamp[inspectMaxFrames],  // rtImage stamp of each frame
                    inspectDropped[inspectMaxFrames]; // stamps the app's rtElect records superseded
static int          inspectDroppedCount = 0;
static BYTE         inspectKind[inspectMaxFrames],   // TStationKind of each frame
                    inspectBand[inspectMaxFrames],   // spin band and heading bin of each frame
                    inspectBin[inspectMaxFrames],
                    inspectPick[inspectMaxFrames];   // --corner-frames: the frames the user named
static bool         inspectPicking = false;          // (only these take part in walls and floor)

//--------------------------------------------------------------------------------
static bool inspectIsSuperseded(QWORD stampNs)
{
   for (int i = 0; i < inspectDroppedCount; i++)
      if (inspectDropped[i] == stampNs)
         return true;
   return false;
}

/*--------------------------------------------------------------------------------
   The app's elections, read ahead: every image a retake replaced (or that lost to the bin's photo).
   A debug capture keeps them all in the log; the analysis uses only the elected ones.
  --------------------------------------------------------------------------------*/
static void inspectReadElections(LPCSTR sessionDir)
{
   TSessionReader reader;
   TRecordView    v;

   inspectDroppedCount = 0;
   if (!reader.Open(sessionDir))
      return;
   while (reader.Next(v))
   {
      TElectRecord e;

      if (v.type == rtElect && e.Decode(v.payload, v.length) && e.supersededNs && inspectDroppedCount < inspectMaxFrames)
         inspectDropped[inspectDroppedCount++] = e.supersededNs;
   }
}
static const float  cRectRollSign = -1.f,          // correction senses (view axes: x right, y down)
                    cRectPitchSign = 1.f,
                    cRectMinElevDeg = 3.f,     // a crease line this far off the horizon (else: some other line)
                    cRectMaxRollDeg = 3.f;     // and this near to flat (the vertical errs by 1-2 degrees)

/*--------------------------------------------------------------------------------
   The crease in a frontal view: in each column the highest strong horizontal edge of the upper part,
   then a straight line through them - least squares, points farther than 4 px dropped, three rounds
   (a lamp, a curtain rod or a shelf top leave the line). The crease of a wall is horizontal in the
   world, so in a true frontal view it is flat, and seen from the spin point it stands at one
   elevation in every frame.
  --------------------------------------------------------------------------------*/
static void inspectCreaseLine(LPCBYTE out, int ow, int oh, int rowBytes, float f, float cyv, bool floor, TRectMeasure &m)
{
   TAlloc<float> pc((size_t)ow),
                 pr((size_t)ow);
   TAlloc<BYTE>  keep((size_t)ow);
   int           n = 0;

   m.ok = false;
   m.points = 0;
   for (int c = 2; c < ow - 2; c += 2)
      for (int k = 2; k < oh*7/10; k++)
      {
         int     r = floor ? oh - 1 - k : k; // the ceiling from the top down, the floor from the bottom up
         LPCBYTE p = out + (size_t)(oh - 1 - r)*rowBytes + c*3; // bottom-up rows
         int     gy = 0,
                 gx = 0;

         for (int d = -1; d <= 1; d++)
         {
            LPCBYTE up = out + (size_t)(oh - r)*rowBytes + (c + d)*3,
                    dn = out + (size_t)(oh - 2 - r)*rowBytes + (c + d)*3;

            gy += ((int)dn[1] - (int)up[1])*(d ? 1 : 2);
         }
         gx = ((int)p[4] - (int)p[-2])*2;
         if (!p[0] && !p[1] && !p[2]) // outside the frame
            continue;
         if ((gy < 0 ? -gy : gy) >= 48 && (gy < 0 ? -gy : gy) >= 2*(gx < 0 ? -gx : gx))
         {
            pc[n] = (float)c;
            pr[n] = (float)r;
            keep[n] = 1u;
            n++;
            break;
         }
      }
   if (n < ow/12)
      return;

   float a = 0.f,
         b = 0.f;
   int   used = 0;

   for (int round = 0; round < 3; round++)
   {
      float sx = 0.f,
            sy = 0.f,
            sxx = 0.f,
            sxy = 0.f;

      used = 0;
      for (int i = 0; i < n; i++)
         if (keep[i])
         {
            sx += pc[i];
            sy += pr[i];
            sxx += pc[i]*pc[i];
            sxy += pc[i]*pr[i];
            used++;
         }
      if (used < ow/12)
         return;

      float det = (float)used*sxx - sx*sx;

      if (fabsf(det) < 1e-3f)
         return;
      a = ((float)used*sxy - sx*sy)/det;
      b = (sy - a*sx)/(float)used;
      for (int i = 0; i < n; i++)
         keep[i] = fabsf(pr[i] - (a*pc[i] + b)) <= 4.f ? 1u : 0u;
   }

   float mid = a*0.5f*(float)ow + b;

   m.points = used;
   m.slope = a;
   m.elevDeg = atan2f(cyv - mid, f)*57.29578f;
   m.floorLine = floor;

   // a ceiling crease stands above the camera, a floor crease below; nearly flat either way
   m.ok = (floor ? m.elevDeg < -cRectMinElevDeg : m.elevDeg > cRectMinElevDeg)
          && fabsf(atanf(a))*57.29578f <= cRectMaxRollDeg
          && (!floor || used >= ow/6); // furniture hides floor creases: the line must run twice as long
}

static bool  inspectRectOut = false, // --rectify: write the frontal views
             inspectDoors = false,   // --doors: look for doors in them
             inspectDoorsAll = false, // --doors-all: and write every view, door or not (diagnosis)
             inspectRectOwn = false,  // --rectify-own: frontal views by each frame's own lines, no crease levelling
             inspectThirdAxis = false, // --third: the receding lines join the frontal square (measured either way)
             inspectRollOnly = false;  // --roll-only: the frontal square turns the view about its axis only
static void inspectPanoExp(const float *w, float *e);
static bool inspectPanoSolve(float *a, float *b, int n);
static float inspectQuantile(float *v, int n, float q);

/*--------------------------------------------------------------------------------
   The squareness of a frontal view (user, 2026-09-30: "no rectification is valid if the ceiling/
   floor is not horizontal or cabinets/doors/windows are not vertical"). In the view's own normalized
   coordinates (x right, y up, from its principal point), a small turn w of the virtual camera leans a
   vertical line by dx/dy = -w_z + w_x x and slopes a horizontal line of the wall by dy/dx = w_z - w_y y.
   Every strong edge within cSquareTolDeg of vertical or horizontal votes (weight: its gradient);
   robust line fits of lean against x and slope against y give w. Degrees out: the lean and slope at
   the view's center and their spread across it; false with too few edges of either family.
  --------------------------------------------------------------------------------*/
static const float cSquareTolDeg = 12.f,  // an edge this near vertical or horizontal votes
                   cSquareLeanBinDeg = 0.3f, // a line's lean bins
                   cSquarePosBinPx = 4.f,    // and its crossing bins (view pixels)
                   cSquareEdgeDeg = 4.f,     // leans an edge votes for around its gradient's
                   cSquareRidge = 0.02f,     // the pull of w_x, w_y toward 0 in the joint fit
                   cSquareHorzShare = 0.05f, // the horizontals' weight in the fit beside the verticals
                   cSquareLineSpan = 0.03f,  // the spread (focals, 1 sigma) of a line's edges its own fit needs
                   cSquareConsensusDeg = 3.f, // a line this far off its family's median lean is other structure
                   cSquareMinGrad = 30.f,  // Sobel magnitude of a voting edge
                   cSquareFaintGrad = 12.f, // and the second look of a view short of a family (a white frame on beige: ~15)
                   cCornerWindowDeg = 2.5f, // a vertical line this near the corner's predicted bearing is the corner
                   cCornerEyeM = 0.3f,      // the eye's offset from the spin point along the aim (the arm's sphere)
                   cSquareGain = 0.6f,      // the share of a round's measured turn applied
                   cSquareDoneDeg = 0.05f;  // a turn below this ends the rounds

enum {
   cSquareMinVotes = 45,  // edges of long lines a family's fit needs: one clear line is a needle (user)
   cSquareLineVotes = 45, // edges (sampled every 2 px, about one per column) a line needs: ~100 px of it
   cSquareRounds = 7,     // measure-and-turn rounds of one view (the last only measures)
   cSquareMaxLines = 48   // vertical lines a view reports (the corner's candidates)
};

// The long vertical lines of a view: where each crosses the visible center row (normalized x) and its edges
struct TSquareLines {
   int   count;
   float x[cSquareMaxLines];
   int   votes[cSquareMaxLines];
};

// The room corners of the wall a view faces, as bearings from the eye against the wall's normal (plan, center spin)
struct TCornerHint {
   int   count;
   float bearing[2];
};

static const TCornerHint *inspectSquareCorner = NULL; // the corner hint of the view being squared

//--------------------------------------------------------------------------------
// A view point (normalized x) within cCornerWindowDeg of a corner bearing of the view being squared
static bool inspectNearCorner(float x)
{
   if (!inspectSquareCorner)
      return false;

   float b = atanf(x);

   for (int c = 0; c < inspectSquareCorner->count; c++)
      if (fabsf(b - inspectSquareCorner->bearing[c]) <= cCornerWindowDeg*0.01745329f)
         return true;
   return false;
}
static const TLayoutPlan *inspectCornerPlan = NULL; // the plan the corner anchor reads (the center spin's frames)
static bool               inspectCornerCenter = false; // the frame being rectified is of the center spin
static int                inspectCornerFrame = -1;

/*--------------------------------------------------------------------------------
   The third canonical axis of a frontal view (user, 2026-09-30: "81 has a sharp point where three axes
   meet... mark the three canonical axes and you will see they are not normalized"): the lines running
   straight away from the wall (a cabinet's base, a door's reveal, the neighbor wall's creases) meet at the
   view's principal point when the view is square. Long such lines by a Hough vote about the principal
   point (direction t, offset o = cos t y - sin t x), each refined by the fit of its edges. A small turn w
   of the virtual camera moves their meeting point by (w_y, -w_x), so each line tells o = sin t w_y +
   cos t w_x: the square's least squares takes them with the verticals and the horizontals.
  --------------------------------------------------------------------------------*/
static const float cRecedeDeg = 8.f,      // an edge this near the ray to the principal point may recede
                   cRecedeOffMax = 0.35f; // a receding line passes this near the principal point (focals)

enum {
   cRecedeMaxLines = 64
};

// The receding lines of a view: direction, offset from the principal point (normalized), edges
struct TRecedeLines {
   int   count;
   float t[cRecedeMaxLines],
         o[cRecedeMaxLines], // from the family's meeting point
         w[cRecedeMaxLines];
   BYTE  fam[cRecedeMaxLines]; // 0 straight away from the wall; 1, 2 the floor's diagonals (cRecedeTargetX)
};

/* where each family meets on the horizon of a square view, normalized x from the principal point: straight away
   from the wall, and with --diagonal-floor the tiles laid at 45 degrees to the walls (101732; user: the tiles are
   diagonal) */
static const float cRecedeTargetX[3] = { 0.f, 1.f, -1.f };
static bool        inspectDiagonalFloor = false; // --diagonal-floor

//--------------------------------------------------------------------------------
static void inspectFrontalRecede(const float *px, const float *py, const float *pt, LPCBYTE pk, int n, float f,
                                 TRecedeLines &rl)
{
   const float bin = cSquareLeanBinDeg*0.01745329f;
   const int   bins = (int)(3.14159265f/bin) + 1,
               cuts = (int)(2.f*cRecedeOffMax*f/cSquarePosBinPx) + 1,
               spread = (int)(cSquareEdgeDeg/cSquareLeanBinDeg),
               cells = 3*bins*cuts; // a plane of cells per family
   TAlloc<int> count((size_t)cells),
               lineOf((size_t)n + 1u);

   rl.count = 0;
   if (n <= 0)
      return;
   memset(count(), 0, sizeof(int)*(size_t)cells);
   for (int pass = 0; pass < 2; pass++)
      for (int k = 0; k < n; k++)
      {
         int li0 = (int)(pt[k]/bin),
             plane = (int)pk[k]*bins*cuts,
             best = -1;

         for (int dl = -spread; dl <= spread; dl++)
         {
            int   li = (li0 + dl + bins)%bins;
            float t = ((float)li + 0.5f)*bin,
                  o = cosf(t)*py[k] - sinf(t)*px[k];
            int   pi = (int)((o + cRecedeOffMax)*f/cSquarePosBinPx);

            if (pi < 0 || pi >= cuts)
               continue;
            if (pass == 0)
               count[plane + li*cuts + pi]++;
            else if (best < 0 || count[plane + li*cuts + pi] > count[best])
               best = plane + li*cuts + pi;
         }
         if (pass == 1)
            lineOf[k] = best >= 0 && count[best] >= cSquareLineVotes ? best : -1;
      }

   // each line's edges: their straight fit (the principal direction of the positions) gives its direction and offset
   TAlloc<float> s((size_t)cells*6u);

   memset(s(), 0, sizeof(float)*(size_t)cells*6u);
   for (int k = 0; k < n; k++)
   {
      float *c = s() + (size_t)6*(size_t)(lineOf[k] >= 0 ? lineOf[k] : 0);

      if (lineOf[k] < 0)
         continue;
      c[0] += 1.f;
      c[1] += px[k];
      c[2] += py[k];
      c[3] += px[k]*px[k];
      c[4] += px[k]*py[k];
      c[5] += py[k]*py[k];
   }

   rl.count = 0;
   for (int c = 0; c < cells && rl.count < cRecedeMaxLines; c++)
   {
      const float *q = s() + (size_t)6*(size_t)c;

      if (q[0] < (float)cSquareLineVotes)
         continue;

      float mx = q[1]/q[0],
            my = q[2]/q[0],
            cxx = q[3]/q[0] - mx*mx,
            cxy = q[4]/q[0] - mx*my,
            cyy = q[5]/q[0] - my*my,
            t = 0.5f*atan2f(2.f*cxy, cxx - cyy),
            o = cosf(t)*my - sinf(t)*mx;

      if (fabsf(o) > cRecedeOffMax)
         continue;
      rl.t[rl.count] = t;
      rl.o[rl.count] = o;
      rl.fam[rl.count] = (BYTE)(c/(bins*cuts));
      rl.w[rl.count++] = q[0];
   }
}

//--------------------------------------------------------------------------------
// Where a view's receding lines meet, by themselves (the measure of the third axis): degrees off the principal point
static bool inspectRecedeMeet(const TRecedeLines &rl, float f, float *deg)
{
   float v[2] = {},
         gate = 1e9f;

   if (rl.count == 0)
      return false;
   for (int round = 0; round < 3; round++)
   {
      float a[3] = {},
            b[2] = {},
            sw = 0.f,
            sq = 0.f;

      for (int j = 0; j < rl.count; j++)
      {
         float jx = -sinf(rl.t[j]),
               jy = cosf(rl.t[j]),
               res = rl.o[j] - (jx*v[0] + jy*v[1]);

         if (rl.fam[j] != 0 || fabsf(res) > gate) // the receding family only
            continue;
         a[0] += rl.w[j]*jx*jx;
         a[1] += rl.w[j]*jx*jy;
         a[2] += rl.w[j]*jy*jy;
         b[0] += rl.w[j]*jx*rl.o[j];
         b[1] += rl.w[j]*jy*rl.o[j];
         sw += rl.w[j];
      }
      if (sw <= 0.f)
         return false;
      a[0] += 1e-3f*sw; // one line fixes the point across it only: the ridge holds it along
      a[2] += 1e-3f*sw;

      float det = a[0]*a[2] - a[1]*a[1];

      if (fabsf(det) < 1e-12f)
         return false;
      v[0] = (a[2]*b[0] - a[1]*b[1])/det;
      v[1] = (a[0]*b[1] - a[1]*b[0])/det;
      for (int j = 0; j < rl.count; j++)
      {
         float res = rl.o[j] - (-sinf(rl.t[j])*v[0] + cosf(rl.t[j])*v[1]);

         if (rl.fam[j] == 0 && fabsf(res) <= gate)
            sq += rl.w[j]*res*res;
      }
      gate = 3.f*sqrtf(sq/sw) + 2.f/f;
   }
   deg[0] = atanf(v[0])*57.2957795f;
   deg[1] = atanf(v[1])*57.2957795f;
   return true;
}

//--------------------------------------------------------------------------------
// One equation of the square's least squares: family 0 a vertical edge, 1 a horizontal edge, 2 a receding line
static void inspectSquareRow(int fam, int k, const float *vx, const float *vl, const float *vw, const float *hy,
                             const float *hs, const float *hw, const TRecedeLines &rl, float *j, float &obs, float &wt)
{
   if (fam == 0)
   {
      j[0] = vx[k];
      j[1] = 0.f;
      j[2] = -1.f;
      obs = vl[k];
      wt = vw[k];
   }
   else if (fam == 1)
   {
      j[0] = 0.f;
      j[1] = -hy[k];
      j[2] = 1.f;
      obs = hs[k];
      wt = hw[k];
   }
   else // its family meets at (T, 0): a turn w moves that point by (w_y (1 + T^2), -w_x - w_z T)
   {
      float tx = cRecedeTargetX[rl.fam[k]];

      j[0] = cosf(rl.t[k]);
      j[1] = sinf(rl.t[k])*(1.f + tx*tx);
      j[2] = cosf(rl.t[k])*tx;
      obs = rl.o[k];
      wt = rl.w[k];
   }
}

//--------------------------------------------------------------------------------
static bool inspectFrontalSquare(LPCBYTE out, int ow, int oh, int rowBytes, float f, float cxv, float cyv, float *w,
                                 float *stats, TSquareLines *lines, float *vp)
{
   TAlloc<float> vx((size_t)ow*oh/4u + 1u),
                 vl((size_t)ow*oh/4u + 1u),
                 vw((size_t)ow*oh/4u + 1u),
                 hy((size_t)ow*oh/4u + 1u),
                 hs((size_t)ow*oh/4u + 1u),
                 hw((size_t)ow*oh/4u + 1u),
                 vy((size_t)ow*oh/4u + 1u),
                 hx((size_t)ow*oh/4u + 1u),
                 rx((size_t)ow*oh/4u + 1u),
                 ry((size_t)ow*oh/4u + 1u),
                 rt((size_t)ow*oh/4u + 1u);
   TAlloc<BYTE>  rk((size_t)ow*oh/4u + 1u);
   int           nv = 0,
                 nh = 0,
                 nr = 0,
                 cap = ow*oh/4;
   const float   tol = tanf(cSquareTolDeg*0.01745329f),
                 recede = sinf(cRecedeDeg*0.01745329f);

   if (lines)
      lines->count = 0;
   for (int vv = 2; vv + 2 < oh; vv += 2)
      for (int uu = 2; uu + 2 < ow; uu += 2)
      {
         // gray of a pixel of the bottom-up BMP rows; v grows downward in the view
         LPCBYTE r0 = out + (size_t)(oh - 1 - (vv - 1))*rowBytes + (size_t)uu*3u,
                 r1 = out + (size_t)(oh - 1 - vv)*rowBytes + (size_t)uu*3u,
                 r2 = out + (size_t)(oh - 1 - (vv + 1))*rowBytes + (size_t)uu*3u;
         float   g00 = (float)(r0[-3] + r0[-2] + r0[-1]),
                 g01 = (float)(r0[0] + r0[1] + r0[2]),
                 g02 = (float)(r0[3] + r0[4] + r0[5]),
                 g10 = (float)(r1[-3] + r1[-2] + r1[-1]),
                 g12 = (float)(r1[3] + r1[4] + r1[5]),
                 g20 = (float)(r2[-3] + r2[-2] + r2[-1]),
                 g21 = (float)(r2[0] + r2[1] + r2[2]),
                 g22 = (float)(r2[3] + r2[4] + r2[5]);

         if (g00 == 0.f || g02 == 0.f || g20 == 0.f || g22 == 0.f) // off the photo (black border)
            continue;

         float gx = ((g02 + 2.f*g12 + g22) - (g00 + 2.f*g10 + g20))/3.f,
               gd = ((g20 + 2.f*g21 + g22) - (g00 + 2.f*g01 + g02))/3.f, // toward v growing (down)
               gy = -gd,                                                // toward y up
               mag = sqrtf(gx*gx + gy*gy);

         float x = ((float)uu - cxv)/f,
               y = -((float)vv - cyv)/f;

         /* faint edges count only as a vertical where the plan puts the room's corner (user: "you know which line is
            the corner"): 101732's 75 on wall O, a white cabinet frame on a beige wall */
         if (mag < cSquareMinGrad
             && !(mag >= cSquareFaintGrad && fabsf(gy) < tol*fabsf(gx) && inspectNearCorner(x)))
            continue;

         if (fabsf(gy) < tol*fabsf(gx) && nv < cap) // a near-vertical line: its lean dx/dy = -gy/gx
         {
            vx[nv] = x;
            vl[nv] = -gy/gx;
            vw[nv] = mag;
            vy[nv++] = y;
         }
         else if (fabsf(gx) < tol*fabsf(gy) && nh < cap) // a near-horizontal line: its slope dy/dx = -gx/gy
         {
            hy[nh] = y;
            hs[nh] = -gx/gy;
            hw[nh] = mag;
            hx[nh++] = x;
         }
         else if (vp && nr < cap) // a line running toward a family's meeting point: the third axis
         {
            float dx = -gy/mag,
                  dy = gx/mag,
                  bestMiss = recede;
            int   bestK = -1;

            for (int k = 0; k < (inspectDiagonalFloor ? 3 : 1); k++)
            {
               float xs = x - cRecedeTargetX[k],
                     r = sqrtf(xs*xs + y*y),
                     miss = r > 0.05f ? fabsf(dx*y - dy*xs)/r : 1.f;

               if (miss < bestMiss)
               {
                  bestMiss = miss;
                  bestK = k;
               }
            }
            if (bestK >= 0)
            {
               float t = atan2f(dy, dx);

               rx[nr] = x - cRecedeTargetX[bestK]; // about the family's meeting point
               ry[nr] = y;
               rk[nr] = (BYTE)bestK;
               rt[nr++] = t < 0.f ? t + 3.14159265f : (t >= 3.14159265f ? t - 3.14159265f : t);
            }
         }
      }
   TRecedeLines rl;

   rl.count = 0;
   if (vp) // measured always; it joins the fit unless --no-third
   {
      vp[0] = 0.f;
      vp[1] = NAN;
      vp[2] = NAN;
      vp[3] = NAN;
      inspectFrontalRecede(rx(), ry(), rt(), rk(), nr, f, rl);
      inspectRecedeMeet(rl, f, vp + 2);
   }

   const int nr3 = inspectThirdAxis ? rl.count : 0; // the receding lines in the fit

   /* only long straight lines count (user: the ceiling's lines and the columns). A Hough vote: each edge votes for
      every lean within cSquareEdgeDeg of its gradient's (one pixel's gradient wanders degrees) at the crossing that
      lean gives with the view's middle row (column for the horizontals); a line is a peak of cSquareLineVotes edges,
      and its edges take the peak's lean. A bed sheet's folds and a neighbor wall's slant scatter and drop out */
   for (int fam = 0; fam < 2; fam++)
   {
      float *P = fam == 0 ? vx() : hy(),  // the position the fit runs on
            *O = fam == 0 ? vy() : hx(),  // the other coordinate
            *L = fam == 0 ? vl() : hs(),
            *W = fam == 0 ? vw() : hw();
      /* the crossings span the image itself, not a window about the principal point: a view turned toward a corner
         has its principal point far off the image (101732's frame 81 on wall O: 663 px left of it) */
      float x0 = -cxv/f,
            x1 = ((float)ow - cxv)/f,
            y0 = -((float)oh - cyv)/f,
            y1 = cyv/f,
            reachO = fam == 0 ? fmaxf(fabsf(y0), fabsf(y1)) : fmaxf(fabsf(x0), fabsf(x1)),
            lo = (fam == 0 ? x0 : y0) - tol*reachO,
            hi = (fam == 0 ? x1 : y1) + tol*reachO;
      int   &n = fam == 0 ? nv : nh,
            bins = (int)(2.f*cSquareTolDeg/cSquareLeanBinDeg) + 1,
            cuts = (int)((hi - lo)*f/cSquarePosBinPx) + 1,
            spread = (int)(cSquareEdgeDeg/cSquareLeanBinDeg);
      TAlloc<int> count((size_t)bins*cuts),
                  lineOf((size_t)n + 1u);

      memset(count(), 0, sizeof(int)*(size_t)bins*cuts);
      for (int pass = 0; pass < 2; pass++)
      {
         int kept = 0;

         for (int k = 0; k < n; k++)
         {
            int li0 = (int)((atanf(L[k])*57.2957795f + cSquareTolDeg)/cSquareLeanBinDeg),
                best = -1,
                bestLi = 0;

            for (int li = li0 - spread; li <= li0 + spread; li++)
            {
               if (li < 0 || li >= bins)
                  continue;

               float t = tanf(((float)li + 0.5f)*cSquareLeanBinDeg*0.01745329f - cSquareTolDeg*0.01745329f);
               int   pi = (int)((P[k] - t*O[k] - lo)*f/cSquarePosBinPx);

               if (pi < 0 || pi >= cuts)
                  continue;
               if (pass == 0)
                  count[li*cuts + pi]++;
               else if (best < 0 || count[li*cuts + pi] > count[best])
               {
                  best = li*cuts + pi;
                  bestLi = li;
               }
            }
            if (pass == 1 && best >= 0 && count[best] >= cSquareLineVotes)
            {
               P[kept] = P[k];
               O[kept] = O[k];
               L[kept] = tanf(((float)bestLi + 0.5f)*cSquareLeanBinDeg*0.01745329f - cSquareTolDeg*0.01745329f);
               lineOf[kept] = best;
               W[kept] = W[k];
               kept++;
            }
         }
         if (pass == 1)
            n = kept;
      }
      /* the peak's lean is only as fine as its bin: each line's lean comes from a straight fit of its edges' positions
         (P against O), sub-pixel, the bin's kept when the edges span too short a stretch */
      TAlloc<float> s0((size_t)bins*cuts),
                    s1((size_t)bins*cuts),
                    s2((size_t)bins*cuts),
                    s3((size_t)bins*cuts),
                    s4((size_t)bins*cuts);

      memset(s0(), 0, sizeof(float)*(size_t)bins*cuts);
      memset(s1(), 0, sizeof(float)*(size_t)bins*cuts);
      memset(s2(), 0, sizeof(float)*(size_t)bins*cuts);
      memset(s3(), 0, sizeof(float)*(size_t)bins*cuts);
      memset(s4(), 0, sizeof(float)*(size_t)bins*cuts);
      for (int k = 0; k < n; k++)
      {
         int c = lineOf[k];

         s0[c] += 1.f;
         s1[c] += O[k];
         s2[c] += P[k];
         s3[c] += O[k]*O[k];
         s4[c] += O[k]*P[k];
      }
      for (int k = 0; k < n; k++)
      {
         int   c = lineOf[k];
         float den = s0[c]*s3[c] - s1[c]*s1[c];

         if (den > s0[c]*s0[c]*cSquareLineSpan*cSquareLineSpan)
            L[k] = (s0[c]*s4[c] - s1[c]*s2[c])/den;
      }

      // the vertical lines out (the corner's candidates): each at the visible center row, near ones (4 px) merged
      float cyRow = -(0.5f*(float)oh - cyv)/f;

      for (int k = 0; fam == 0 && lines && k < n; k++)
      {
         int c = lineOf[k];

         if (s0[c] < (float)cSquareLineVotes)
            continue; // too short, or already out (the mark below)

         float x = s2[c]/s0[c] + L[k]*(cyRow - s1[c]/s0[c]);
         int   votes = (int)s0[c],
               j = 0;

         s0[c] = -1.f;
         while (j < lines->count && fabsf(lines->x[j] - x)*f > cSquarePosBinPx)
            j++;
         if (j < lines->count)
         {
            if (votes > lines->votes[j])
               lines->x[j] = x;
            lines->votes[j] += votes;
         }
         else if (lines->count < cSquareMaxLines)
         {
            lines->x[lines->count] = x;
            lines->votes[lines->count++] = votes;
         }
      }
   }
   // a family short of long lines drops out; the other still squares what it can (the ridge holds the rest)
   if (nv < cSquareMinVotes)
      nv = 0;
   if (nh < cSquareMinVotes)
      nh = 0;
   if (nv == 0 && nh == 0 && nr3 == 0)
      return false;

   /* the lines of one family are parallel in the world (user: several verticals, one at the corner, the rest
      parallel to it): the family's consensus is its median lean, and a line off it by more than cSquareConsensusDeg
      is other structure (the floor's tile joints, a neighbor wall's slant) and drops out */
   for (int fam = 0; fam < 2; fam++)
   {
      float *P = fam == 0 ? vx() : hy(),
            *O = fam == 0 ? vy() : hx(),
            *L = fam == 0 ? vl() : hs(),
            *W = fam == 0 ? vw() : hw();
      int   &n = fam == 0 ? nv : nh;

      if (n == 0)
         continue;

      TAlloc<float> tmp((size_t)n);

      memcpy(tmp(), L, sizeof(float)*(size_t)n);

      float med = inspectQuantile(tmp(), n, 0.5f),
            tol = tanf(cSquareConsensusDeg*0.01745329f);
      int   kept = 0;

      for (int k = 0; k < n; k++)
         if (fabsf(L[k] - med) <= tol)
         {
            P[kept] = P[k];
            O[kept] = O[k];
            L[kept] = L[k];
            W[kept] = W[k];
            kept++;
         }
      n = kept >= cSquareMinVotes ? kept : 0;
   }
   if (nv == 0 && nh == 0 && nr3 == 0)
      return false;

   /* one least squares for the turn w of the three canonical families (the view's principal point sits far off
      the image when the frame looked up or down, so a family alone cannot tell its lean at x = 0 from its
      keystone): lean_i = -w_z + w_x x_i, slope_j = w_z - w_y y_j, and each receding line o_k = cos t_k w_x +
      sin t_k w_y; each family weighs the same in total, a mild ridge keeps w_x and w_y from an edge set bunched in
      one row, and 3 rounds drop the far ones beyond 3x the weighted RMS */
   float gate[3] = { 1e9f, 1e9f, 1e9f },
         famW[3] = {},
         cx = (0.5f*(float)ow - cxv)/f, // the visible center, in the view's normalized coordinates
         cy = -(0.5f*(float)oh - cyv)/f,
         /* the verticals are the needle (user: several verticals, one at the corner, the rest parallel to it): with
            them in the view and no receding line the horizontals only steer what the verticals cannot see (w_y), at
            cSquareHorzShare; with the third axis all three weigh alike */
         share[3] = { 1.f, nv > 0 && nr3 == 0 ? cSquareHorzShare : 1.f, 1.f };

   for (int k = 0; k < nv; k++)
      famW[0] += vw[k];
   for (int k = 0; k < nh; k++)
      famW[1] += hw[k];
   for (int k = 0; k < nr3; k++)
      famW[2] += rl.w[k];
   w[0] = w[1] = w[2] = 0.f;
   for (int round = 0; round < 3; round++)
   {
      float A[9] = {},
            B[3] = {},
            sq[3] = {},
            sw[3] = {};

      for (int fam = 0; fam < 3; fam++)
      {
         int   n = fam == 0 ? nv : (fam == 1 ? nh : nr3);
         float scale = share[fam]/fmaxf(famW[fam], 1.f);

         for (int k = 0; k < n; k++)
         {
            float j[3],
                  obs,
                  wt;

            inspectSquareRow(fam, k, vx(), vl(), vw(), hy(), hs(), hw(), rl, j, obs, wt);
            wt *= scale;

            float res = obs - (j[0]*w[0] + j[1]*w[1] + j[2]*w[2]);

            if (fabsf(res) > gate[fam])
               continue;
            for (int r = 0; r < 3; r++)
            {
               for (int c = 0; c < 3; c++)
                  A[r*3 + c] += wt*j[r]*j[c];
               B[r] += wt*j[r]*obs;
            }
         }
      }
      A[0] += cSquareRidge;
      A[4] += cSquareRidge*share[1]; // w_y is the horizontals' (and the receding lines'): its ridge at their share
      if (!inspectPanoSolve(A, B, 3))
         return false;
      memcpy(w, B, sizeof(B));
      for (int fam = 0; fam < 3; fam++)
      {
         int n = fam == 0 ? nv : (fam == 1 ? nh : nr3);

         for (int k = 0; k < n; k++)
         {
            float j[3],
                  obs,
                  wt;

            inspectSquareRow(fam, k, vx(), vl(), vw(), hy(), hs(), hw(), rl, j, obs, wt);

            float res = obs - (j[0]*w[0] + j[1]*w[1] + j[2]*w[2]);

            if (fabsf(res) <= gate[fam])
            {
               sq[fam] += wt*res*res;
               sw[fam] += wt;
            }
         }
         gate[fam] = 3.f*sqrtf(sq[fam]/fmaxf(sw[fam], 1.f)) + (fam == 2 ? 2.f/f : 0.002f);
         if (fam == 2 && vp)
         {
            vp[0] = (float)nr3;
            vp[1] = sqrtf(sq[2]/fmaxf(sw[2], 1.f))*f; // the receding lines' miss of the principal point, view pixels
         }
      }
   }
   // what the lines measure, not the fit: each family's mean residual added back
   float mean[2] = {},
         msw[2] = {};

   for (int k = 0; k < nv; k++)
   {
      float res = vl[k] - (vx[k]*w[0] - w[2]);

      if (fabsf(res) <= gate[0])
      {
         mean[0] += vw[k]*res;
         msw[0] += vw[k];
      }
   }
   for (int k = 0; k < nh; k++)
   {
      float res = hs[k] - (-hy[k]*w[1] + w[2]);

      if (fabsf(res) <= gate[1])
      {
         mean[1] += hw[k]*res;
         msw[1] += hw[k];
      }
   }
   mean[0] /= fmaxf(msw[0], 1.f);
   mean[1] /= fmaxf(msw[1], 1.f);
   stats[0] = atanf(-w[2] + w[0]*cx + mean[0])*57.2957795f; // the verticals' lean at the visible center
   stats[1] = atanf(w[2] - w[1]*cy + mean[1])*57.2957795f;  // the horizontals' slope there
   stats[2] = w[0]*57.2957795f*0.5f;               // the verticals' lean change over half a focal of x (keystone)
   stats[3] = -w[1]*57.2957795f*0.5f;              // the horizontals' slope change over half a focal of y
   if (nv == 0)
      stats[0] = stats[2] = NAN;
   if (nh == 0)
      stats[1] = stats[3] = NAN;
   return true;
}

/*--------------------------------------------------------------------------------
   The square of a floor view (user, 2026-10-01: 75 face O "completely out of square"; the floor's lines
   are a needle too): the floor lies square to the view, so its lines keep their true angles - the
   baseboards at 0 and 90 degrees, and with --diagonal-floor the tiles at +-45. For a line of unit
   direction u at the view point (x, y), a small turn w of the virtual camera changes its angle by
   -w_z + w_y (u_x^2 y - u_x u_y x) + w_x (u_y^2 x - u_x u_y y) (the verticals' and horizontals' rows are
   its cases), so every long line of a known angle is one row of a least squares for w: the floor's
   normal (w_x, w_y) and its turn (w_z). Long lines by the same Hough vote, in coordinates turned to each
   family; each family weighs the same. Degrees out: each family's mean angle off its own (0, 90, +45,
   -45; NaN without the family); false with fewer than two families.
  --------------------------------------------------------------------------------*/
static const float cFloorTolDeg = 8.f,    // an edge this near a family's angle votes for it
                   cFloorMaxTurnDeg = 3.f; // a floor's square moves the frame's vertical at most this much

enum {
   cFloorMinLines = 2 // long lines a family needs
};

//--------------------------------------------------------------------------------
static bool inspectFloorSquare(LPCBYTE out, int ow, int oh, int rowBytes, float f, float cxv, float cyv, float *w,
                               float *stats)
{
   const float   fam0[4] = { 0.f, 90.f, 45.f, -45.f };
   const int     cap = ow*oh/4,
                 famCount = inspectDiagonalFloor ? 4 : 2;
   TAlloc<float> ex((size_t)cap + 1u),
                 ey((size_t)cap + 1u),
                 et((size_t)cap + 1u),
                 ew((size_t)cap + 1u);
   TAlloc<BYTE>  ek((size_t)cap + 1u);
   int           n = 0;

   for (int vv = 2; vv + 2 < oh; vv += 2)
      for (int uu = 2; uu + 2 < ow; uu += 2)
      {
         LPCBYTE r0 = out + (size_t)(oh - 1 - (vv - 1))*rowBytes + (size_t)uu*3u,
                 r1 = out + (size_t)(oh - 1 - vv)*rowBytes + (size_t)uu*3u,
                 r2 = out + (size_t)(oh - 1 - (vv + 1))*rowBytes + (size_t)uu*3u;
         float   g00 = (float)(r0[-3] + r0[-2] + r0[-1]),
                 g01 = (float)(r0[0] + r0[1] + r0[2]),
                 g02 = (float)(r0[3] + r0[4] + r0[5]),
                 g10 = (float)(r1[-3] + r1[-2] + r1[-1]),
                 g12 = (float)(r1[3] + r1[4] + r1[5]),
                 g20 = (float)(r2[-3] + r2[-2] + r2[-1]),
                 g21 = (float)(r2[0] + r2[1] + r2[2]),
                 g22 = (float)(r2[3] + r2[4] + r2[5]);

         if (g00 == 0.f || g02 == 0.f || g20 == 0.f || g22 == 0.f || n >= cap)
            continue;

         float gx = ((g02 + 2.f*g12 + g22) - (g00 + 2.f*g10 + g20))/3.f,
               gy = -((g20 + 2.f*g21 + g22) - (g00 + 2.f*g01 + g02))/3.f, // toward y up
               mag = sqrtf(gx*gx + gy*gy);

         if (mag < cSquareMinGrad)
            continue;

         // the line's angle (counterclockwise, y up), folded into [-90, 90)
         float t = atan2f(gx, -gy)*57.2957795f,
               bestD = cFloorTolDeg;
         int   best = -1;

         t -= 180.f*floorf((t + 90.f)/180.f);
         for (int k = 0; k < famCount; k++)
         {
            float d = t - fam0[k];

            d -= 180.f*floorf((d + 90.f)/180.f);
            if (fabsf(d) < bestD)
            {
               bestD = fabsf(d);
               best = k;
            }
         }
         if (best < 0)
            continue;
         ex[n] = ((float)uu - cxv)/f;
         ey[n] = -((float)vv - cyv)/f;
         et[n] = t;
         ew[n] = mag;
         ek[n++] = (BYTE)best;
      }

   // per family, in coordinates turned to it: the long lines' edges, their lean from the line's own fit
   float         famW[4] = {};
   int           famN[4] = {};
   TAlloc<float> lean((size_t)n + 1u);
   TAlloc<BYTE>  keep((size_t)n + 1u);

   memset(keep(), 0, (size_t)n + 1u);
   for (int k = 0; k < famCount; k++)
   {
      float c0 = cosf(fam0[k]*0.01745329f),
            s0 = sinf(fam0[k]*0.01745329f),
            reach = 0.f;

      for (int i = 0; i < n; i++)
         if (ek[i] == k)
            reach = fmaxf(reach, fabsf(-s0*ex[i] + c0*ey[i]) + cFloorTolDeg*0.01745329f*fabsf(c0*ex[i] + s0*ey[i]));

      const int     bins = (int)(2.f*cFloorTolDeg/cSquareLeanBinDeg) + 1,
                    cuts = (int)(2.f*(reach + 0.01f)*f/cSquarePosBinPx) + 1,
                    spread = (int)(cSquareEdgeDeg/cSquareLeanBinDeg);
      TAlloc<int>   count((size_t)bins*cuts),
                    cellOf((size_t)n + 1u);
      TAlloc<float> s((size_t)bins*cuts*5u);

      memset(count(), 0, sizeof(int)*(size_t)bins*cuts);
      memset(s(), 0, sizeof(float)*(size_t)bins*cuts*5u);
      for (int pass = 0; pass < 2; pass++)
         for (int i = 0; i < n; i++)
         {
            if (ek[i] != k)
               continue;

            float pp = -s0*ex[i] + c0*ey[i], // across the family
                  oo = c0*ex[i] + s0*ey[i],  // along it
                  d = et[i] - fam0[k];
            int   best = -1;

            d -= 180.f*floorf((d + 90.f)/180.f);

            int li0 = (int)((d + cFloorTolDeg)/cSquareLeanBinDeg);

            for (int li = li0 - spread; li <= li0 + spread; li++)
            {
               if (li < 0 || li >= bins)
                  continue;

               float tl = tanf(((float)li + 0.5f)*cSquareLeanBinDeg*0.01745329f - cFloorTolDeg*0.01745329f);
               int   pi = (int)((pp - tl*oo + reach + 0.01f)*f/cSquarePosBinPx);

               if (pi < 0 || pi >= cuts)
                  continue;
               if (pass == 0)
                  count[li*cuts + pi]++;
               else if (best < 0 || count[li*cuts + pi] > count[best])
                  best = li*cuts + pi;
            }
            if (pass == 1)
            {
               cellOf[i] = best >= 0 && count[best] >= cSquareLineVotes ? best : -1;
               if (cellOf[i] >= 0)
               {
                  float *q = s() + (size_t)5*(size_t)cellOf[i];

                  q[0] += 1.f;
                  q[1] += oo;
                  q[2] += pp;
                  q[3] += oo*oo;
                  q[4] += oo*pp;
               }
            }
         }
      for (int i = 0; i < n; i++)
      {
         if (ek[i] != k || cellOf[i] < 0)
            continue;

         const float *q = s() + (size_t)5*(size_t)cellOf[i];
         float        den = q[0]*q[3] - q[1]*q[1];

         if (!(den > q[0]*q[0]*cSquareLineSpan*cSquareLineSpan))
            continue;
         lean[i] = (q[0]*q[4] - q[1]*q[2])/den; // the line's slope in the family's coordinates: its angle off
         keep[i] = 1;
         famW[k] += ew[i];
         famN[k]++;
      }
      int famLines = 0;

      for (int c = 0; c < bins*cuts; c++)
         famLines += s[(size_t)5*(size_t)c] >= (float)cSquareLineVotes ? 1 : 0;
      if (famN[k] < cSquareMinVotes || famLines < cFloorMinLines) // a family short of long lines drops out (a blanket's edge)
      {
         for (int i = 0; i < n; i++)
            if (ek[i] == k)
               keep[i] = 0;
         famW[k] = 0.f;
         famN[k] = 0;
      }
   }

   int families = 0;

   for (int k = 0; k < famCount; k++)
      families += famN[k] > 0 ? 1 : 0;
   if (families < 2)
      return false;

   float allW = famW[0] + famW[1] + famW[2] + famW[3];

   // the least squares: 3 rounds, the far edges out beyond 3x the weighted rms of their family
   float gate[4] = { 1e9f, 1e9f, 1e9f, 1e9f },
         mean[4] = {},
         msw[4] = {};

   w[0] = w[1] = w[2] = 0.f;
   for (int round = 0; round < 3; round++)
   {
      float A[9] = {},
            B[3] = {},
            sq[4] = {},
            sw[4] = {};

      for (int i = 0; i < n; i++)
      {
         if (!keep[i])
            continue;

         int   k = ek[i];
         float ux = cosf(fam0[k]*0.01745329f),
               uy = sinf(fam0[k]*0.01745329f),
               j[3] = { -(uy*uy*ex[i] - ux*uy*ey[i]), -(ux*ux*ey[i] - ux*uy*ex[i]), 1.f },
               wt = ew[i]/allW, // by edges: a family weighs as much as its long lines
               res = lean[i] - (j[0]*w[0] + j[1]*w[1] + j[2]*w[2]);

         if (fabsf(res) > gate[k])
            continue;
         for (int r = 0; r < 3; r++)
         {
            for (int c = 0; c < 3; c++)
               A[r*3 + c] += wt*j[r]*j[c];
            B[r] += wt*j[r]*lean[i];
         }
      }
      A[0] += cSquareRidge;
      A[4] += cSquareRidge;
      if (!inspectPanoSolve(A, B, 3))
         return false;
      memcpy(w, B, sizeof(B));
      memset(mean, 0, sizeof(mean));
      memset(msw, 0, sizeof(msw));
      for (int i = 0; i < n; i++)
      {
         if (!keep[i])
            continue;

         int   k = ek[i];
         float ux = cosf(fam0[k]*0.01745329f),
               uy = sinf(fam0[k]*0.01745329f),
               res = lean[i] - (-(uy*uy*ex[i] - ux*uy*ey[i])*w[0] - (ux*ux*ey[i] - ux*uy*ex[i])*w[1] + w[2]);

         if (fabsf(res) <= gate[k])
         {
            sq[k] += ew[i]*res*res;
            sw[k] += ew[i];
         }
         mean[k] += ew[i]*lean[i];
         msw[k] += ew[i];
      }
      for (int k = 0; k < 4; k++)
         gate[k] = 3.f*sqrtf(sq[k]/fmaxf(sw[k], 1.f)) + 0.002f;
   }
   for (int k = 0; k < 4; k++) // what the lines measure: each family's mean angle off its own
      stats[k] = msw[k] > 0.f ? atanf(mean[k]/msw[k])*57.2957795f : NAN;
   return true;
}

/*--------------------------------------------------------------------------------
   The merge (--merge; user, 2026-10-01: "we are very close to the panorama... shall we merge the files?"):
   a canvas per wall (along the wall by height, meters) and one of the floor (the plan, w right, u up),
   cMergePixelM a pixel. A squared frontal view stands square to the room, so a wall point (s, h) is seen
   at x = (s - s_eye)/D, y = (h - h_eye)/D, D the eye's distance to the wall (the plan's), the eye the spin
   point plus cCornerEyeM along the aim (the arm's sphere); the floor alike with the eye's height. Each view
   is laid onto its canvas as it is written, weighed 1/(1 + x^2 + y^2)^2: the middle of a photo, the least
   stretched (user: the image center has the least distortion), wins.
  --------------------------------------------------------------------------------*/
static const float cMergePixelM = 0.004f,  // a canvas pixel
                   cMergeMarginM = 0.05f;  // beyond the wall's corners (farther shows the neighbor wall)

enum {
   cMergeCanvases = 5 // walls 0..3 (clockwise from the room axis), the floor
};

struct TMergeCanvas {
   int           w,
                 h;
   float         x0,  // the left edge's coordinate (wall: along it; floor: plan w)
                 top; // the top edge's (wall: height; floor: plan u)
   TBlock<float> acc; // rgb sums and the weight, 4 a pixel
};

struct TMergeTarget {
   int   canvas; // -1: the view is not merged
   float nu, nw, // wall: the normal in plan axes (into the wall), its right, the wall's offset along the normal
         ru, rw,
         reach,
         tu, tw, // floor: the plan direction at the view's top
         eu, ew, // the eye in plan axes, and its height
         eh;
};

static bool          inspectMergeOn = false; // --merge
static TMergeCanvas  inspectMergeCanvas[cMergeCanvases];
static TMergeTarget  inspectMergeTarget = { -1 };

//--------------------------------------------------------------------------------
// A canvas's extent from the plan: a wall from corner to corner (and margin) by floor to ceiling; the floor's box
static void inspectMergeCanvasInit(int k, const TLayoutPlan &pl, float nu, float nw, float ru, float rw)
{
   TMergeCanvas &c = inspectMergeCanvas[k];

   if (c.acc)
      return;

   float x0 = 1e9f,
         x1 = -1e9f,
         y0 = 0.f,
         y1 = pl.ceilingM + 0.1f;

   if (k < 4)
   {
      float reach = -1e9f;

      for (int v = 0; v < pl.vertexCount; v++)
         reach = fmaxf(reach, pl.verts[v].u*nu + pl.verts[v].w*nw);
      for (int v = 0; v < pl.vertexCount; v++)
         if (pl.verts[v].u*nu + pl.verts[v].w*nw >= reach - 0.05f)
         {
            float s = pl.verts[v].u*ru + pl.verts[v].w*rw;

            x0 = fminf(x0, s);
            x1 = fmaxf(x1, s);
         }
   }
   else
   {
      y0 = 1e9f;
      y1 = -1e9f;
      for (int v = 0; v < pl.vertexCount; v++)
      {
         x0 = fminf(x0, pl.verts[v].w);
         x1 = fmaxf(x1, pl.verts[v].w);
         y0 = fminf(y0, pl.verts[v].u);
         y1 = fmaxf(y1, pl.verts[v].u);
      }
   }
   c.x0 = x0 - cMergeMarginM;
   c.top = y1 + (k < 4 ? 0.f : cMergeMarginM);
   c.w = (int)((x1 - x0 + 2.f*cMergeMarginM)/cMergePixelM) + 1;
   c.h = (int)((c.top - (y0 - (k < 4 ? 0.f : cMergeMarginM)))/cMergePixelM) + 1;

   TAlloc<float> a((size_t)c.w*c.h*4u);

   memset(a(), 0, sizeof(float)*(size_t)c.w*c.h*4u);
   a.Drop(c.acc);
}

//--------------------------------------------------------------------------------
// One squared view laid onto its canvas (the target set by inspectRectify): the box its footprint covers, sampled
static void inspectMergeSplat(LPCBYTE out, int ow, int oh, int rowBytes, float f, float cxv, float cyv)
{
   const TMergeTarget &t = inspectMergeTarget;

   if (t.canvas < 0 || t.canvas >= cMergeCanvases || !inspectMergeCanvas[t.canvas].acc)
      return;

   TMergeCanvas &c = inspectMergeCanvas[t.canvas];
   bool          wall = t.canvas < 4;
   // the view's axes in plan terms: x along (xu, xw), y along (yu, yw) (the floor) or up (a wall)
   float         xu = wall ? t.ru : -t.tw,
                 xw = wall ? t.rw : t.tu,
                 depth = wall ? t.reach - (t.eu*t.nu + t.ew*t.nw) : t.eh,
                 xe = t.eu*xu + t.ew*xw,                  // the eye along the view's x
                 ye = wall ? t.eh : t.eu*t.tu + t.ew*t.tw; // and along its y

   if (!(depth > 0.2f))
      return;

   /* the footprint: the view's corners on the canvas plane, in the canvas's own coordinates (a wall: along it and up;
      the floor: plan w and u, the view's axes any of the room's) */
   float cmin[2] = { 1e9f, 1e9f },
         cmax[2] = { -1e9f, -1e9f };

   for (int k = 0; k < 4; k++)
   {
      float vx = xe + depth*((k & 1 ? (float)ow : 0.f) - cxv)/f, // along the view's x and y
            vy = ye + depth*(cyv - (k & 2 ? (float)oh : 0.f))/f,
            cx = wall ? vx : vx*xw + vy*t.tw, // plan w = x xw + y tw (the axes are orthonormal)
            cy = wall ? vy : vx*xu + vy*t.tu; // plan u

      cmin[0] = fminf(cmin[0], cx);
      cmax[0] = fmaxf(cmax[0], cx);
      cmin[1] = fminf(cmin[1], cy);
      cmax[1] = fmaxf(cmax[1], cy);
   }

   int i0 = (int)fmaxf(0.f, (cmin[0] - c.x0)/cMergePixelM),
       i1 = (int)fminf((float)(c.w - 1), (cmax[0] - c.x0)/cMergePixelM),
       j0 = (int)fmaxf(0.f, (c.top - cmax[1])/cMergePixelM),
       j1 = (int)fminf((float)(c.h - 1), (c.top - cmin[1])/cMergePixelM);

   for (int j = j0; j <= j1; j++)
      for (int i = i0; i <= i1; i++)
      {
         // the canvas point in the view's axes; floor canvas: x = plan w, y = plan u
         float cxp = c.x0 + (float)i*cMergePixelM,
               cyp = c.top - (float)j*cMergePixelM,
               px,
               py;

         if (wall)
         {
            px = cxp;
            py = cyp;
         }
         else
         {
            px = cyp*xu + cxp*xw; // the plan point (u = cyp, w = cxp) along the view's axes
            py = cyp*t.tu + cxp*t.tw;
         }

         float x = (px - xe)/depth,
               y = (py - ye)/depth,
               su = cxv + f*x,
               sv = cyv - f*y;
         int   iu = (int)floorf(su),
               iv = (int)floorf(sv);

         if (iu < 0 || iv < 0 || iu + 1 >= ow || iv + 1 >= oh)
            continue;

         float   du = su - (float)iu,
                 dv = sv - (float)iv,
                 rgb[3];
         LPCBYTE r0 = out + (size_t)(oh - 1 - iv)*rowBytes + (size_t)iu*3u,
                 r1 = out + (size_t)(oh - 2 - iv)*rowBytes + (size_t)iu*3u;

         if (r0[0] + r0[1] + r0[2] == 0 || r0[3] + r0[4] + r0[5] == 0 || r1[0] + r1[1] + r1[2] == 0
             || r1[3] + r1[4] + r1[5] == 0)
            continue; // off the photo
         for (int ch = 0; ch < 3; ch++)
            rgb[ch] = ((float)r0[ch]*(1.f - du) + (float)r0[3 + ch]*du)*(1.f - dv)
                      + ((float)r1[ch]*(1.f - du) + (float)r1[3 + ch]*du)*dv;

         float  q = 1.f + x*x + y*y,
                wt = 1.f/(q*q),
               *a = c.acc() + ((size_t)j*c.w + i)*4u;

         a[0] += wt*rgb[0];
         a[1] += wt*rgb[1];
         a[2] += wt*rgb[2];
         a[3] += wt;
      }
}

//--------------------------------------------------------------------------------
// The canvases out: mosaico_paredeK.bmp (K as the views' rect_NNN_paredeK) and mosaico_piso.bmp
static void inspectMergeWrite(LPCSTR outDir)
{
   for (int k = 0; k < cMergeCanvases; k++)
   {
      TMergeCanvas &c = inspectMergeCanvas[k];

      if (!c.acc)
         continue;

      int          rowBytes = (c.w*3 + 3) & ~3;
      TAlloc<BYTE> img((size_t)rowBytes*c.h);
      char         path[sessionPathMax];

      memset(img(), 0, (size_t)rowBytes*c.h);
      for (int j = 0; j < c.h; j++)
         for (int i = 0; i < c.w; i++)
         {
            const float *a = c.acc() + ((size_t)j*c.w + i)*4u;
            LPBYTE       p = img() + (size_t)(c.h - 1 - j)*rowBytes + (size_t)i*3u;

            if (a[3] <= 0.f)
               continue;
            for (int ch = 0; ch < 3; ch++)
               p[ch] = (BYTE)fminf(255.f, a[ch]/a[3] + 0.5f);
         }
      if (k < 4)
         snprintf(path, sizeof(path), "%s/mosaico_parede%d.bmp", outDir, k);
      else
         snprintf(path, sizeof(path), "%s/mosaico_piso.bmp", outDir);

      FILE *bmp = fopen(path, "wb");

      if (!bmp)
         continue;

      DWORD imageBytes = (DWORD)rowBytes*(DWORD)c.h,
            header[13] = { 54u + imageBytes, 0u, 54u, 40u, (DWORD)c.w, (DWORD)c.h, 0x00180001u, 0u, imageBytes, 2835u,
                           2835u, 0u, 0u };
      BYTE  magic[2] = { 'B', 'M' };

      fwrite(magic, 1u, 2u, bmp);
      fwrite(header, sizeof(header), 1u, bmp);
      fwrite(img(), 1u, imageBytes, bmp);
      fclose(bmp);
      printf("merge: %s, %d x %d px (%.2f x %.2f m)\n", path, c.w, c.h, (float)c.w*cMergePixelM,
             (float)c.h*cMergePixelM);
   }
}

//--------------------------------------------------------------------------------
// The view rendered for the virtual camera (x, y = up, z = -n in the real camera's axes); its principal point out
static void inspectFrontalRender(const TImageRecord &img, LPCBYTE bgr, const TVec3 &up, const TVec3 &n, LPBYTE out,
                                 int ow, int oh, int rowBytes, float f, float &cxv, float &cyv)
{
   int   w = (int)img.width,
         h = (int)img.height;
   TVec3 zv = { -n.x, -n.y, -n.z },
         xv = { up.y*zv.z - up.z*zv.y, up.z*zv.x - up.x*zv.z, up.x*zv.y - up.y*zv.x };
   float elev = asinf(fmaxf(-0.9f, fminf(0.9f, -up.z))), // the real camera looks down -z
         yaw = atan2f(-xv.z, zv.z);

   cxv = 0.5f*(float)ow - f*tanf(fmaxf(-0.9f, fminf(0.9f, yaw)));
   cyv = 0.5f*(float)oh + f*tanf(elev);
   memset(out, 0, (size_t)rowBytes*oh);
   for (int vv = 0; vv < oh; vv++)
   {
      LPBYTE row = out + (size_t)(oh - 1 - vv)*rowBytes; // bottom-up

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
}

static FILE *inspectSquareCsv = NULL; // --rectify-own: each view's squareness before and after its rounds

/*--------------------------------------------------------------------------------
   One frontal view: a pure rotation of the camera (homography K R K^-1, no depth needed) onto a
   virtual camera looking straight along the horizontal wall normal n, level and without roll -
   verticals vertical, that wall's creases horizontal. A shift of the principal point keeps the
   original view centered, like an architectural shift lens, so nothing tilts. 24-bit BMP, upright
   (path NULL: nothing written); measure (optional): the crease line of the view.
  --------------------------------------------------------------------------------*/
static void inspectFrontal(const TImageRecord &img, LPCBYTE bgr, const TVec3 &up, const TVec3 &n, LPCSTR path,
                           TRectMeasure *measure, const TCornerHint *corner = NULL, bool floor = false,
                           TVec3 *finalU = NULL, TVec3 *finalN = NULL)
{
   const int    ow = 900,
                oh = 1200,
                rowBytes = (ow*3 + 3) & ~3;
   const float  f = 0.3f*img.intr.fx;
   TAlloc<BYTE> out((size_t)rowBytes*oh);
   TVec3        U = up,
                N = n;
   float        cxv = 0.f,
                cyv = 0.f,
                first[6] = { NAN, NAN, NAN, NAN, NAN, NAN }, // lean, slope, keystones, the receding lines' meeting x y
                last[6] = { NAN, NAN, NAN, NAN, NAN, NAN };

   inspectSquareCorner = floor ? NULL : corner;
   /* own lines (--rectify-own): measured in the view itself, and the virtual camera turned until its verticals and
      horizontals stand square (the needle of the balance), a few rounds; else as the directions came */
   for (int round = 0; round < (inspectRectOwn ? cSquareRounds : 1); round++)
   {
      float        wv[3],
                   st[4],
                   vp[4],
                   yaw = 0.f;
      TSquareLines lines;

      inspectFrontalRender(img, bgr, U, N, out(), ow, oh, rowBytes, f, cxv, cyv);
      lines.count = 0;
      vp[0] = 0.f;
      vp[1] = vp[2] = vp[3] = NAN;
      if (!inspectRectOwn
          || !(floor ? inspectFloorSquare(out(), ow, oh, rowBytes, f, cxv, cyv, wv, st)
                     : inspectFrontalSquare(out(), ow, oh, rowBytes, f, cxv, cyv, wv, st, &lines, vp)))
         break;

      bool third = vp[0] > 0.f; // receding lines in the square's least squares: they set the pitch and the yaw

      if (floor && path)
         printf("  floor square %s round %d: off 0/90/+45/-45 %.2f %.2f %.2f %.2f deg, turn %+.2f %+.2f %+.2f\n",
                strrchr(path, '/') ? strrchr(path, '/') + 1 : path, round, st[0], st[1], st[2], st[3],
                wv[0]*57.2957795f, wv[1]*57.2957795f, wv[2]*57.2957795f);

      if (third && path)
         printf("  third axis %s round %d: %d receding lines, %.1f px rms off the principal point after the fit, turn"
                " %+.2f %+.2f %+.2f deg\n", strrchr(path, '/') ? strrchr(path, '/') + 1 : path, round, (int)vp[0], vp[1],
                wv[0]*57.2957795f, wv[1]*57.2957795f, wv[2]*57.2957795f);
      if (round == 0)
      {
         memcpy(first, st, sizeof(st));
         first[4] = vp[2];
         first[5] = vp[3];
      }
      memcpy(last, st, sizeof(st));
      last[4] = vp[2];
      last[5] = vp[3];

      /* the corner anchor (user: "you know which line is the corner, the rotation must be forced to align that
         axis"): the plan's corner is seen from the eye at a known bearing against the wall's normal; the vertical
         line nearest it (within cCornerWindowDeg) is the corner, and the view turns about its vertical until the
         line stands there - only in a view without horizontals: where it has them, they are the needle (user: the
         ceiling and floor lines), and a corner from the plan misses by about a degree */
      if (!third && isnan(st[1]) && corner && corner->count > 0 && round >= 1) // round 0 plumbs the verticals first
      {
         float best = 1e9f,
               bestX = NAN,
               bestBearing = NAN;

         for (int j = 0; j < lines.count; j++)
            for (int c = 0; c < corner->count; c++)
            {
               float d = fabsf(atanf(lines.x[j]) - corner->bearing[c]);

               if (d < best && d <= cCornerWindowDeg*0.01745329f)
               {
                  best = d;
                  bestX = lines.x[j];
                  bestBearing = corner->bearing[c];
               }
            }
         if (isnan(bestX) && path && round == 1)
         {
            float nearest = 1e9f;

            for (int j = 0; j < lines.count; j++)
               for (int c = 0; c < corner->count; c++)
                  if (fabsf(atanf(lines.x[j]) - corner->bearing[c]) < fabsf(nearest))
                     nearest = atanf(lines.x[j]) - corner->bearing[c];
            printf("  corner %s: predicted %+.2f / %+.2f deg, %d lines, the nearest %+.2f off (cx %.0f):",
                   strrchr(path, '/') ? strrchr(path, '/') + 1 : path, corner->bearing[0]*57.2957795f,
                   corner->count > 1 ? corner->bearing[1]*57.2957795f : NAN, lines.count, nearest*57.2957795f, cxv);
            for (int j = 0; j < lines.count; j++)
               printf(" %.0fpx/%.1f(%d)", cxv + lines.x[j]*f, atanf(lines.x[j])*57.2957795f, lines.votes[j]);
            printf("\n");
         }
         if (!isnan(bestX))
         {
            yaw = atanf(bestX) - bestBearing;
            wv[1] = 0.f;
            if (path)
               printf("  corner %s round %d: predicted %+.2f, the line at %+.2f deg: turn %+.2f\n",
                      strrchr(path, '/') ? strrchr(path, '/') + 1 : path, round, bestBearing*57.2957795f,
                      atanf(bestX)*57.2957795f, yaw*57.2957795f);
         }
      }
      if (round + 1 == cSquareRounds
          || (fmaxf(fabsf(wv[0]), fmaxf(fabsf(wv[1]), fabsf(wv[2]))) < cSquareDoneDeg*0.01745329f && yaw == 0.f))
         break; // the last round only measures; a view already square stops early

      // V' = V E(w): the virtual camera's axes (columns x, up, -n, in the real camera's) turned by w in its own axes
      float lim = 5.f*0.01745329f,
            e[9];

      for (int k = 0; k < 3; k++) // damped: the receding lines and the keystone pull a little against each other
         wv[k] = fmaxf(-lim, fminf(lim, cSquareGain*wv[k]));
      inspectPanoExp(wv, e);

      TVec3 zv = { -N.x, -N.y, -N.z },
            xv = { U.y*zv.z - U.z*zv.y, U.z*zv.x - U.x*zv.z, U.x*zv.y - U.y*zv.x },
            col[3] = { xv, U, zv },
            nc[3];

      for (int j = 0; j < 3; j++)
      {
         nc[j].x = col[0].x*e[j] + col[1].x*e[3 + j] + col[2].x*e[6 + j];
         nc[j].y = col[0].y*e[j] + col[1].y*e[3 + j] + col[2].y*e[6 + j];
         nc[j].z = col[0].z*e[j] + col[1].z*e[3 + j] + col[2].z*e[6 + j];
      }
      U = nc[1];
      N.x = -nc[2].x;
      N.y = -nc[2].y;
      N.z = -nc[2].z;
      if (yaw != 0.f) // the corner's turn about the view's vertical: the normal toward the view's right by yaw
      {
         TVec3 zn = { -N.x, -N.y, -N.z },
               xn = { U.y*zn.z - U.z*zn.y, U.z*zn.x - U.x*zn.z, U.x*zn.y - U.y*zn.x };
         float t = fmaxf(-lim, fminf(lim, yaw)),
               cy = cosf(t),
               sy = sinf(t);

         N.x = N.x*cy + xn.x*sy;
         N.y = N.y*cy + xn.y*sy;
         N.z = N.z*cy + xn.z*sy;
      }
   }
   inspectSquareCorner = NULL;
   if (inspectMergeOn && path)
      inspectMergeSplat(out(), ow, oh, rowBytes, f, cxv, cyv);
   inspectMergeTarget.canvas = -1;
   if (finalU)
      *finalU = U;
   if (finalN)
      *finalN = N;
   if (inspectRectOwn && path)
      printf("  view %s: principal point %.1f %.1f, focal %.1f px\n", strrchr(path, '/') ? strrchr(path, '/') + 1 : path,
             cxv, cyv, f);
   if (inspectSquareCsv && path)
   {
      LPCSTR base = strrchr(path, '/');

      fprintf(inspectSquareCsv, "%s,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\r\n", base ? base + 1 : path,
              first[0], first[1], first[2], first[3], last[0], last[1], last[2], last[3], first[4], first[5], last[4],
              last[5]);
   }
   if (measure)
   {
      inspectCreaseLine(out(), ow, oh, rowBytes, f, cyv, false, *measure);
      if (!measure->ok)
         inspectCreaseLine(out(), ow, oh, rowBytes, f, cyv, true, *measure); // the floor spin: the floor crease
   }
   if (!path)
      return;

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

static float inspectDoorTiltDeg = 0.f, // --door-tilt deg
             inspectBlurNoise = 0.f;   // --blur-noise sigma (gray levels)

/*--------------------------------------------------------------------------------
   Doors on one frontal view (--doors): the view is built from the frame's YUV (BGR converted, chroma
   halved like the camera's), searched, printed and written as door_NNN_paredeK.bmp - gray view, jambs
   and head in green, the ceiling line in yellow, the horizon in blue.
  --------------------------------------------------------------------------------*/
static void inspectDoorView(const TImageRecord &img, LPCBYTE bgr, const TVec3 &up, const TVec3 &n, LPCSTR outDir,
                            int index, int wall)
{
   const int    w = (int)img.width,
                h = (int)img.height,
                cw = w/2,
                ch = h/2,
                vw = doorViewW,
                vh = doorViewH,
                rowBytes = (vw*3 + 3) & ~3;
   TAlloc<BYTE> y((size_t)w*h),
                u((size_t)cw*ch),
                v((size_t)cw*ch),
                vy((size_t)vw*vh),
                vu((size_t)vw*vh),
                vv((size_t)vw*vh),
                valid((size_t)vw*vh),
                out((size_t)rowBytes*vh);

   for (int r = 0; r < h; r++)
      for (int c = 0; c < w; c++)
      {
         LPCBYTE p = bgr + ((size_t)r*w + c)*3u;
         float   b = (float)p[0],
                 g = (float)p[1],
                 rr = (float)p[2],
                 yy = 0.299f*rr + 0.587f*g + 0.114f*b;

         y[(size_t)r*w + c] = (BYTE)(yy + 0.5f);
         if (r%2 == 0 && c%2 == 0 && r/2 < ch && c/2 < cw)
         {
            u[(size_t)(r/2)*cw + c/2] = (BYTE)fmaxf(0.f, fminf(255.f, 128.f + 0.564f*(b - yy)));
            v[(size_t)(r/2)*cw + c/2] = (BYTE)fmaxf(0.f, fminf(255.f, 128.f + 0.713f*(rr - yy)));
         }
      }

   TYUVImage yuv = { w, h, y(), w, u(), v(), cw, 1 };
   TDoorView view = { vy(), vu(), vv(), valid(), 0.f, 0.f, 0.f };
   TDoor     doors[doorMaxDoors];
   TVec3     upT = up; // --door-tilt: the vertical turned about the camera x axis (sensitivity to a tilt error)

   if (inspectDoorTiltDeg != 0.f)
   {
      float t = inspectDoorTiltDeg*0.01745329f,
            c = cosf(t),
            s = sinf(t);

      upT.y = up.y*c - up.z*s;
      upT.z = up.y*s + up.z*c;
   }
   doorFrontal(yuv, img.intr, upT, n, view);

   TDoorStats stats = {};
   int        found = doorDetect(view, doors, doorMaxDoors, &stats);

   printf("  doors frame %03d wall %d: %d jambs, %d pairs; rejected shape %d head %d overrun %d above %d color %d; %d found\n",
          index, wall, stats.jambs, stats.pairs, stats.shape, stats.head, stats.overrun, stats.above, stats.color, found);
   for (int i = 0; i < stats.jambs; i++)
      printf("    jamb col %d rows %d-%d\n", stats.jambCol[i], stats.jambTop[i], stats.jambBottom[i]);

   static LPCSTR cWhy[9] = { "door", "shape", "foot", "plausibility", "head", "overrun", "above", "color", "leaf" };

   for (int i = 0; inspectDoorsAll && i < stats.tried; i++)
      printf("    pair %d-%d: %s %.3f\n", stats.triedA[i], stats.triedB[i],
             stats.triedWhy[i] >= 0 && stats.triedWhy[i] < 9 ? cWhy[stats.triedWhy[i]] : "?", stats.triedValue[i]);
   for (int i = 0; i < found; i++)
   {
      const TDoor &d = doors[i];

      printf("  door frame %03d wall %d: cols %.0f-%.0f head %.0f floor %.0f crease %.0f -> crease/head %.3f camera/head %.3f"
             " (crease %.2f m, camera %.2f m at 2.10) color %.1f knob %d corner %d score %.2f\n", index, wall, d.leftCol,
             d.rightCol, d.headRow, d.floorRow, d.creaseRow, d.creaseOverHead, d.cameraOverHead, 2.1f*d.creaseOverHead,
             2.1f*d.cameraOverHead, d.colorGap, d.knob ? 1 : 0, d.nearCorner ? 1 : 0, d.score);
   }
   if (!found && !inspectDoorsAll)
      return; // an image only where a door was found
   for (int r = 0; r < vh; r++)
   {
      LPBYTE row = out() + (size_t)(vh - 1 - r)*rowBytes; // bottom-up

      for (int c = 0; c < vw; c++)
      {
         BYTE g = vy[(size_t)r*vw + c];

         row[c*3] = g;
         row[c*3 + 1] = g;
         row[c*3 + 2] = g;
      }
   }

   // marks: 0 green (door), 1 yellow (ceiling line), 2 blue (horizon)
   for (int i = 0; i < found; i++)
      for (int m = 0; m < 5; m++)
      {
         const TDoor &d = doors[i];
         bool         rowMark = m >= 2;
         float        at = m == 0 ? d.leftCol : (m == 1 ? d.rightCol : (m == 2 ? d.headRow : (m == 3 ? d.creaseRow
                                                                                                  : view.horizonRow)));
         BYTE         color[3] = { 0u, 255u, 0u };

         if (isnan(at))
            continue;
         if (m == 3)
         {
            color[0] = 0u;
            color[1] = 255u;
            color[2] = 255u;
         }
         if (m == 4)
         {
            color[0] = 255u;
            color[1] = 0u;
            color[2] = 0u;
         }
         for (int t = 0; t < (rowMark ? vw : vh); t++)
         {
            int r = rowMark ? (int)at : t,
                c = rowMark ? t : (int)at;

            if (rowMark && m != 4 && (c < (int)d.leftCol - 40 || c > (int)d.rightCol + 40))
               continue;
            if (!rowMark && (r < (int)d.headRow || r > (int)d.floorRow))
               continue;
            if (r < 0 || r >= vh || c < 0 || c >= vw)
               continue;

            LPBYTE p = out() + (size_t)(vh - 1 - r)*rowBytes + c*3;

            p[0] = color[0];
            p[1] = color[1];
            p[2] = color[2];
         }
      }

   char path[sessionPathMax];

   snprintf(path, sizeof(path), "%s/door_%03d_parede%d.bmp", outDir, index, wall);

   FILE *bmp = fopen(path, "wb");

   if (!bmp)
      return;

   DWORD imageBytes = (DWORD)rowBytes*(DWORD)vh,
         header[13] = { 54u + imageBytes, 0u, 54u, 40u, (DWORD)vw, (DWORD)vh, 0x00180001u, 0u, imageBytes, 2835u, 2835u,
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
   gets a frontal view; a frame aimed at a corner (more than 25 degrees off that wall) gets the one of the
   wall it shows more of (edge support). File names tell the wall: rect_NNN_paredeK.bmp, K = 0..3 clockwise
   from the room axis (the same physical wall in every frame).
  --------------------------------------------------------------------------------*/
static void inspectRectify(const TImageRecord &img, const TVanishResult &vr, float refAxisDeg, LPCSTR outDir,
                           int index, float rollDeg, float pitchDeg, TRectMeasure *measure, bool write)
{
   TAlloc<BYTE> bgr((size_t)img.width*img.height*3u);

   if (measure)
      measure->ok = false;
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

   /* the floor first, by the same rule (user, 2026-09-30: N/S/L/O/P): its normal is the measured vertical, the view
      looks straight down, and its top is the room axis nearest to where the camera faced. Its lines keep their true
      angles (user, 2026-10-01: the floor's lines are a needle), so with --diagonal-floor the floor's square turns the
      frame itself: the vertical and the room axis it settles on go into the wall views (101732's frame 75: the tiles
      at 86 degrees in its floor view, its walls out of square) */
   float down = -up.z; // the camera's forward (0, 0, -1) against the measured up

   if (inspectRectOut && inspectRectOwn && write && down < -0.34f) // tilted down beyond ~20 degrees
   {
      TVec3 b0 = { up.y*a.z - up.z*a.y, up.z*a.x - up.x*a.z, up.x*a.y - up.y*a.x },
            fh = { -down*up.x, -down*up.y, -1.f - down*up.z }, // the forward laid flat: its vertical part taken out
            cand[4] = { a, { -a.x, -a.y, -a.z }, b0, { -b0.x, -b0.y, -b0.z } };
      int   best = 0;
      float bestDot = -2.f;

      for (int k = 0; k < 4; k++)
      {
         float dd = cand[k].x*fh.x + cand[k].y*fh.y + cand[k].z*fh.z;

         if (dd > bestDot)
         {
            bestDot = dd;
            best = k;
         }
      }

      char  floorPath[sessionPathMax];
      TVec3 into = { -up.x, -up.y, -up.z }, // the view's normal runs along the aim, into the surface: down
            fu = cand[best],
            fn = into;

      snprintf(floorPath, sizeof(floorPath), "%s/rect_%03d_piso.bmp", outDir, index);
      if (inspectMergeOn && inspectCornerCenter && inspectCornerPlan && inspectCornerPlan->vertexCount >= 3)
      {
         // the merge: the floor's canvas; the view's top as a plan direction (axis A, B = A turned -90), the eye
         const TLayoutPlan &pl = *inspectCornerPlan;
         float axisF = !isnan(refAxisDeg) ? refAxisDeg
                                          : (!isnan(vr.roomAxisDeg) ? vr.roomAxisDeg
                                                                    : fmodf(geomHeadingDeg(pose.RotateVector(a)), 90.f)),
               thA = 90.f*floorf((axisF - pl.axisDeg)/90.f + 0.5f)*0.01745329f,
               au = cosf(thA),
               aw = sinf(thA),
               tu = best == 0 ? au : (best == 1 ? -au : (best == 2 ? aw : -aw)),
               tw = best == 0 ? aw : (best == 1 ? -aw : (best == 2 ? -au : au));
         TVec3 cr = { cand[best].y*up.z - cand[best].z*up.y, cand[best].z*up.x - cand[best].x*up.z,
                      cand[best].x*up.y - cand[best].y*up.x }; // the view's right: its top turned +90
         float phi = atan2f(-cr.z, -cand[best].z),
               el = asinf(fmaxf(-1.f, fminf(1.f, -up.z)));
         TMergeTarget &mt = inspectMergeTarget;

         inspectMergeCanvasInit(4, pl, 0.f, 0.f, 0.f, 0.f);
         mt.canvas = 4;
         mt.tu = tu;
         mt.tw = tw;
         mt.eu = cCornerEyeM*cosf(el)*(cosf(phi)*tu - sinf(phi)*tw);
         mt.ew = cCornerEyeM*cosf(el)*(cosf(phi)*tw + sinf(phi)*tu);
         mt.eh = pl.cameraHeightM + cCornerEyeM*sinf(el);
      }
      inspectFrontal(img, bgr(), cand[best], into, floorPath, NULL, NULL, true, &fu, &fn);
      if (inspectDiagonalFloor)
      {
         // the turn R taking the floor view's axes (x, top, normal) as they came onto the squared ones: R = V1 V0^T
         TVec3 u0 = cand[best],
               n0 = into,
               x0 = { u0.y*-n0.z - u0.z*-n0.y, u0.z*-n0.x - u0.x*-n0.z, u0.x*-n0.y - u0.y*-n0.x },
               x1 = { fu.y*-fn.z - fu.z*-fn.y, fu.z*-fn.x - fu.x*-fn.z, fu.x*-fn.y - fu.y*-fn.x },
               v0[3] = { x0, u0, n0 },
               v1[3] = { x1, fu, fn },
               src[2] = { up, a },
               dst[2];

         for (int s = 0; s < 2; s++)
         {
            float c[3];

            for (int k = 0; k < 3; k++) // the vector in the old axes
               c[k] = v0[k].x*src[s].x + v0[k].y*src[s].y + v0[k].z*src[s].z;
            dst[s].x = v1[0].x*c[0] + v1[1].x*c[1] + v1[2].x*c[2];
            dst[s].y = v1[0].y*c[0] + v1[1].y*c[1] + v1[2].y*c[2];
            dst[s].z = v1[0].z*c[0] + v1[1].z*c[1] + v1[2].z*c[2];
         }

         float turn = acosf(fminf(1.f, fabsf(dst[0].x*up.x + dst[0].y*up.y + dst[0].z*up.z)))*57.2957795f,
               d = dst[1].x*dst[0].x + dst[1].y*dst[0].y + dst[1].z*dst[0].z,
               l;

         if (turn <= cFloorMaxTurnDeg) // farther is a floor read wrong (a bed, a blanket), not the gyroscope's error
         {
            up = dst[0];
            a.x = dst[1].x - d*up.x;
            a.y = dst[1].y - d*up.y;
            a.z = dst[1].z - d*up.z;
            l = sqrtf(a.x*a.x + a.y*a.y + a.z*a.z);
            a.x /= l;
            a.y /= l;
            a.z /= l;
         }
         printf("  floor %03d: its square turns the frame's vertical by %.2f deg%s\n", index, turn,
                turn <= cFloorMaxTurnDeg ? "" : " - not taken");
      }
   }

   TVec3 b = { up.y*a.z - up.z*a.y, up.z*a.x - up.x*a.z, up.x*a.y - up.y*a.x };
   float fa = -a.z,  // camera forward (0, 0, -1) along each axis
         fb = -b.z;
   bool  facingA = fabsf(fa) >= fabsf(fb),
         corner = atan2f(fminf(fabsf(fa), fabsf(fb)), fmaxf(fabsf(fa), fabsf(fb))) >= 25.f*0.01745329f;

   /* aimed at a corner: the one wall that fills more of the frame (user, 2026-09-27: frame 3 was turned to a sliver
      of wall). A wall holds the lines running along it - the wall facing axis A holds the B edges - so the edge
      support of each direction tells how much of each wall the frame shows */
   if (corner && (vr.flags & vfAxisA) && (vr.flags & vfAxisB))
      facingA = vr.support[2] >= vr.support[1];

   for (int pass = 0; pass < (corner && !inspectDoors && !inspectRectOwn ? 1 : 2); pass++) // corner: the second wall for doors (or own lines)
   {
      bool  useA = pass == 0 ? facingA : !facingA;
      float s = (useA ? fa : fb) >= 0.f ? 1.f : -1.f;
      TVec3 n = useA ? a : b;
      float along = fabsf(useA ? fa : fb),
            across = fabsf(useA ? fb : fa);

      if (pass == 1 && atan2f(across, along) > (inspectRectOwn ? 75.f : 60.f)*0.01745329f)
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

      /* leveled by the crease (second pass): the vertical turned about the view axis (roll) and about the view's
         horizontal axis (pitch), the wall normal turned with it so it stays level */
      TVec3 upv = up;

      if (rollDeg != 0.f || pitchDeg != 0.f)
      {
         TVec3 zv = { -n.x, -n.y, -n.z };
         float cr = cosf(rollDeg*0.01745329f),
               sr = sinf(rollDeg*0.01745329f),
               cp = cosf(pitchDeg*0.01745329f),
               sp = sinf(pitchDeg*0.01745329f);
         TVec3 zu = { zv.y*upv.z - zv.z*upv.y, zv.z*upv.x - zv.x*upv.z, zv.x*upv.y - zv.y*upv.x };

         upv.x = upv.x*cr + zu.x*sr;
         upv.y = upv.y*cr + zu.y*sr;
         upv.z = upv.z*cr + zu.z*sr;

         TVec3 xv = { upv.y*zv.z - upv.z*zv.y, upv.z*zv.x - upv.x*zv.z, upv.x*zv.y - upv.y*zv.x },
               xu = { xv.y*upv.z - xv.z*upv.y, xv.z*upv.x - xv.x*upv.z, xv.x*upv.y - xv.y*upv.x },
               xn = { xv.y*n.z - xv.z*n.y, xv.z*n.x - xv.x*n.z, xv.x*n.y - xv.y*n.x };

         upv.x = upv.x*cp + xu.x*sp;
         upv.y = upv.y*cp + xu.y*sp;
         upv.z = upv.z*cp + xu.z*sp;
         n.x = n.x*cp + xn.x*sp;
         n.y = n.y*cp + xn.y*sp;
         n.z = n.z*cp + xn.z*sp;
      }
      /* the corner anchor's hint (own lines, center spin): the wall's normal in plan axes (u along plan.axisDeg, w
         clockwise from it), the eye cCornerEyeM from the spin point along the aim, and the bearing of each corner of
         the wall from there against the normal, positive to the view's right */
      TCornerHint hint = {};

      if (inspectRectOwn && inspectCornerCenter && inspectCornerPlan && inspectCornerPlan->vertexCount >= 3)
      {
         const TLayoutPlan &pl = *inspectCornerPlan;
         float th = 90.f*floorf((axis + 90.f*(float)wall - pl.axisDeg)/90.f + 0.5f)*0.01745329f,
               nu = cosf(th),
               nw = sinf(th),
               ru = -nw,
               rw = nu;
         TVec3 zv = { -n.x, -n.y, -n.z },
               xv = { upv.y*zv.z - upv.z*zv.y, upv.z*zv.x - upv.x*zv.z, upv.x*zv.y - upv.y*zv.x };
         float phi = atan2f(-xv.z, -n.z), // the aim against the normal
               eu = cCornerEyeM*(cosf(phi)*nu + sinf(phi)*ru),
               ew = cCornerEyeM*(cosf(phi)*nw + sinf(phi)*rw),
               reach = -1e9f;

         for (int v = 0; v < pl.vertexCount; v++)
            reach = fmaxf(reach, pl.verts[v].u*nu + pl.verts[v].w*nw);
         for (int v = 0; v < pl.vertexCount && hint.count < 2; v++)
         {
            float du = pl.verts[v].u - eu,
                  dw = pl.verts[v].w - ew;

            if (pl.verts[v].u*nu + pl.verts[v].w*nw < reach - 0.05f)
               continue; // not on the wall
            hint.bearing[hint.count++] = atan2f(du*ru + dw*rw, du*nu + dw*nw);
         }
         if (inspectMergeOn && write) // the merge: this wall's canvas, the eye on the arm's sphere
         {
            float el = asinf(fmaxf(-1.f, fminf(1.f, -upv.z))); // the aim's elevation
            TMergeTarget &mt = inspectMergeTarget;

            inspectMergeCanvasInit(wall, pl, nu, nw, ru, rw);
            mt.canvas = wall;
            mt.nu = nu;
            mt.nw = nw;
            mt.ru = ru;
            mt.rw = rw;
            mt.reach = reach;
            mt.eu = eu*cosf(el);
            mt.ew = ew*cosf(el);
            mt.eh = pl.cameraHeightM + cCornerEyeM*sinf(el);
         }
      }
      if (inspectRectOut && pass == 0) // one frontal view per frame: the dominant wall
      {
         inspectFrontal(img, bgr(), upv, n, write ? path : NULL, measure, &hint);
         if (measure)
            measure->wall = wall;
      }
      else if (inspectRectOut && inspectRectOwn && write) // own lines: the second wall of a corner as well
         inspectFrontal(img, bgr(), upv, n, path, NULL, &hint);
      if (inspectDoors)
         inspectDoorView(img, bgr(), up, n, outDir, index, wall);
      if (pass == 0 && atan2f(across, along) < 25.f*0.01745329f)
         break; // aimed at this wall: one frontal view
   }
}

static BYTE    inspectHidden[inspectMaxFrames]; // --hidden-floor 51-53,42: frames whose hidden floor lines are traced
static LPCSTR inspectOutDir = NULL;
static float  inspectFocalScale = 1.f, // --focal-scale: the lens's focal length against the app's (panorama and faces)
              inspectIntrScale = 1.f;  // --intr-scale: every image record's focal scaled as read (the square-tile lens check)
static bool   inspectFaceFramesOn = false; // --face-frames: each frame rectified alone onto each face, a folder per face
static const TVanishResult *inspectFrameVr = NULL; // each keyframe's vanishing measure (the main loop's), for the registration's check
static bool                 inspectPlumbOn = true,  // --no-plumb: keep the registered tilt (no plumb from the vanishing points)
                            inspectLeverOn = true,  // --no-lever: every camera center at the pivot (pure rotation)
                            inspectLeverScan = false; // --lever-scan: the registration's residual at trial levers (diagnosis)
static TVec3                inspectLever = {};      // the camera center from the spin's pivot, camera axes (m): c = R o
static int                  inspectPanoTop = 3;     // --pano-top L: the registration starts at level L (0 = 1/8; 3 = 1/64: coarser drifts by degrees without the parallax)
static float                inspectVpAlpha = NAN;   // the room axis heading the lines aligned the frames to (radians; NaN: none)
static bool                 inspectPanoFixedOn = true, // --no-fixed-pattern: keep the camera-fixed pattern in the registration pyramid
                            inspectFocalOn = true;     // --no-focal: the registration keeps the focal scale as given
static int                  inspectPairA = -1,      // --pair-scan A,B: one pair's relative turn by brute force, per image band
                            inspectPairB = -1;
static bool                 inspectDtScan = false;  // --dt-scan: the registration's residual at trial pose-to-image delays
static float                inspectPoseDtSec = 0.f; // --pose-dt S: every frame's attitude taken S seconds off its picture's stamp
static bool                 inspectPairsOn = false, // --pairs: the points' pair search before the Gauss-Newton (diagnosis until the bundle)
                            inspectPairDebug = false, // --pair-debug: every failed pair of the first round listed
                            inspectVpSolveOn = false, // --vp-solve: the rotations by the lines + gyroscope links, no photometric fit
                            inspectCircleHeights = false, // --circle-heights: the body circle also solves each band's height
                            inspectCircleFlat = false;    // --circle: the centers on the trunk's circle (flat aim), not the sphere
static float  inspectHiddenCeilingM = 2.7f,  // --hidden-heights H h: the room's ceiling and the camera height
              inspectHiddenCameraM = 1.49f;  // (defaults: the bedroom 064701 plan)
static const float cHiddenGateShare = 0.1f,   // a floor line crosses the vertical this near the expected foot (share of the corner's height)
                   cHiddenBoardM = 0.12f,     // the floor junction is looked for this far below a baseboard's top
                   cHiddenPeakShare = 0.5f,   // share of the pick's own votes the floor pile below needs
                   cHiddenBoardMinM = 0.03f,  // and how far below the pick it must be (not the baseboard top's own edges)
                   cHiddenStepMin = 10.f,     // luma step (3 px across) of a local edge that may vote
                   cHiddenStepShare = 0.4f,   // and share of the point's own strongest edge
                   cHiddenPredictPx = 150.f,  // second pass: a molding passes this near the corner the station predicts
                   cHiddenGateFine = 0.05f,   // and the gate around the foot the station's camera height expects
                   cHiddenPileShare = 0.3f,   // share of the strongest pile the station's floor pile needs
                   cHiddenPileFrameShare = 0.15f, // a frame backs a pile with this share of its votes
                   cFloorReachPx = 400.f,     // a floor line is read this far along its wall from the foot, and beyond it
                   cFloorHold = 2.f,          // mean step it keeps along its wall (low: a short segment is a candidate too)
                   cFloorLineMin = 200.f,     // summed steps a floor candidate needs (a short segment behind a door counts)
                   cInheritTolShare = 0.012f, // a neighbor's line lands on a candidate within this share of the corner's height
                   cFloorBeyondShare = 0.5f,  // and at most this share of it past the corner
                   cHiddenProminence = 2.f,   // a baseboard's lower edge stands this many times above the profile's median
                   cHiddenSnapShare = 0.03f,  // a frame's own line answers the station's within this share of the corner's height
                   cHiddenEdgeShare = 0.1f,   // a predicted corner may fall this share outside the picture
                   cHiddenAgreeShare = 0.03f, // the two walls' feet agree within this share of the corner's height
                   cHiddenReachGap = 150.f,   // a molding pair makes a corner when both segments end this near their crossing
                   cFloorRunMaxPx = 2400.f,   // farthest a floor line's Sobel run is followed from the foot
                   cFloorRunMin = 20.f,       // supported samples (3 px apart) a floor candidate's own run needs
                   cFloorRunGapPx = 12.f,      // a floor line's run breaks at a gap this long between strong samples
                   cFloorRunStep = 16.f,       // a strong sample of a run (floor texture rarely reaches it)
                   cCornerStepMin = 8.f,      // premise zero: luma step (4 px across) of an edge running toward a vanishing point
                   cCornerLineMin = 600.f,    // the summed steps a real line of a family gathers
                   cCornerHold = 5.f,         // and the mean step each molding keeps right out of the corner top
                   cHiddenMoldingPx = 600.f;  // length of a premise-zero molding drawn from the corner top along its wall

enum {
   inspectPencilMax  = 32,     // lines kept per family
   inspectRunSamples = 801,    // samples of a Sobel run (cFloorRunMaxPx every 3 px)
   inspectPencilBins = 200000,
   inspectPencilMin  = 15,     // edge pixels a line of a family needs (a floor line is sparse: darker, cut by furniture)
   inspectPencilLen  = 400,    // and the length it must span (full-picture pixels): sparse but real, not a short blob
   inspectProfileMax = 600,    // offsets of the floor profile (pixels)
   inspectVotesMin   = 12,     // votes (of 201 points, 3 offsets) a floor line pile needs
   inspectHiddenMax  = 64,     // frames of one --hidden-floor run
   inspectPileBin    = 4,      // station votes along the central frame's corner vertical: pixels per bin
   inspectPileBins   = 1500,
   inspectSweepStep  = 3,      // premise zero: pixels sampled every 3 on the upper picture
   inspectSweepBins  = 40000,
   inspectSweepMax   = 96,     // strongest lines kept per family (a window blind alone makes dozens)
   inspectPileWin    = 3       // station piles gathered over +-3 bins (12 px)
};

// What the station tells one of its frames on the later passes (NAN: unknown)
struct THiddenHint {
   float predX,       // upright picture point where the station's corner top should be (the gyroscope from the central frame)
         predY,
         cameraM,     // the station's camera height
         gateShare;   // gate around the expected foot, share of the corner's height on the picture
   int   elect[2];    // per horizontal family: the own candidate the station elected (-1 hidden here, -2 no decision)
};

// What one frame measured: its corner, its vanishing points, and per horizontal family the floor candidates
struct THiddenResult {
   bool  corner;
   float ccx,
         ccy,
         vx[3],       // upright vanishing points (0 vertical, 1 A, 2 B)
         vy[3],
         heightM[2],  // camera height its chosen floor line gives, NAN none
         share[2],    // share of the picture on that family's side of the corner (its wall's view)
         side[2],     // +1 when that family's wall runs from the corner toward its vanishing point, -1 away (upright)
         efx,         // the foot the heights expect (upright)
         efy,
         cand[2][inspectPencilMax][5]; // floor candidates near the expected foot: x0, y0, x1, y1, edge support
   int   cands[2];
};

// One frame of --hidden-floor kept for the station passes
struct THiddenFrame {
   int           index,
                 station;
   TMat4         pose;
   TIntrinsics   intr;
   DWORD         width,
                 height;
   TVanishResult vr;
   TBlock<float> ray;
   TBlock<BYTE>  label;
   DWORD         count;
   THiddenResult res;
};

static THiddenFrame inspectHiddenData[inspectHiddenMax];
static int          inspectHiddenCount = 0;

// One line of a pencil: through the family's vanishing point at angle phi; its edge support and extent
struct TPencilLine {
   float phi,
         meanY,
         x0,
         y0,
         x1,
         y1;
   int   count;
};

/*--------------------------------------------------------------------------------
   Upright full-picture point of a camera direction (a vanishing point) or ray: the pinhole, then the
   picture turned 90 degrees clockwise (x' = height - 1 - v, y' = u), as the frame is seen standing.
  --------------------------------------------------------------------------------*/
static bool inspectUpright(const TImageRecord &img, const TVec3 &d, float &x, float &y)
{
   if (fabsf(d.z) < 1e-6f)
      return false;

   float u = img.intr.cx - img.intr.fx*d.x/d.z,
         v = img.intr.cy + img.intr.fy*d.y/d.z;

   x = (float)img.height - 1.f - v;
   y = u;
   return true;
}

/*--------------------------------------------------------------------------------
   The lines of one family: every edge of the family lies on a line through its vanishing point, so
   one angle around that point tells the line. A histogram of the angles (bins of 2 px at the picture
   center), smoothed over three bins; its peaks with enough edges are the family's real lines,
   strongest first, each with its extent along the line.
  --------------------------------------------------------------------------------*/
static int inspectPencil(const float *ex, const float *ey, LPCBYTE el, int n, BYTE label, float vx, float vy, float cx,
                         float cy, TPencilLine *out)
{
   float dist = sqrtf((cx - vx)*(cx - vx) + (cy - vy)*(cy - vy)),
         bin = 2.f/fmaxf(dist, 1.f);
   int   bins = (int)(3.14159265f/bin) + 1,
         found = 0;

   if (bins > inspectPencilBins)
   {
      bins = inspectPencilBins;
      bin = 3.14159265f/(float)bins;
   }

   TAlloc<int> hist((size_t)bins),
               smooth((size_t)bins);

   memset(hist(), 0, sizeof(int)*(size_t)bins);
   for (int i = 0; i < n; i++)
   {
      if (el[i] != label)
         continue;

      float phi = atan2f(ey[i] - vy, ex[i] - vx);

      if (phi < 0.f)
         phi += 3.14159265f;
      hist[(int)(phi/bin)%bins]++;
   }
   for (int b = 0; b < bins; b++)
      smooth[b] = hist[(b + bins - 1)%bins] + hist[b] + hist[(b + 1)%bins];
   for (int b = 0; b < bins && found < inspectPencilMax; b++)
   {
      bool peak = smooth[b] >= inspectPencilMin;

      for (int d = -3; peak && d <= 3; d++)
         if (d && smooth[(b + d + bins)%bins] > smooth[b])
            peak = false;
      if (!peak || (b > 0 && smooth[b - 1] == smooth[b]))
         continue; // a plateau counts once

      TPencilLine &l = out[found];
      float        dx,
                   dy,
                   tMin = 1e9f,
                   tMax = -1e9f,
                   sumY = 0.f;

      l.phi = ((float)b + 0.5f)*bin;
      l.count = 0;
      dx = cosf(l.phi);
      dy = sinf(l.phi);
      for (int i = 0; i < n; i++)
      {
         if (el[i] != label)
            continue;

         float phi = atan2f(ey[i] - vy, ex[i] - vx);

         if (phi < 0.f)
            phi += 3.14159265f;
         if (fabsf(phi - l.phi) > 1.5f*bin)
            continue;

         float t = (ex[i] - vx)*dx + (ey[i] - vy)*dy;

         tMin = fminf(tMin, t);
         tMax = fmaxf(tMax, t);
         sumY += ey[i];
         l.count++;
      }
      if (l.count < inspectPencilMin || tMax - tMin < (float)inspectPencilLen)
         continue;
      l.meanY = sumY/(float)l.count;
      l.x0 = vx + tMin*dx;
      l.y0 = vy + tMin*dy;
      l.x1 = vx + tMax*dx;
      l.y1 = vy + tMax*dy;
      found++;
   }
   return found;
}

//--------------------------------------------------------------------------------
// Height of a pencil line at column x (a vertical line: its mean height)
static float inspectPencilY(const TPencilLine &l, float vx, float vy, float x)
{
   float c = cosf(l.phi);

   return fabsf(c) < 1e-6f ? l.meanY : vy + (x - vx)*sinf(l.phi)/c;
}

//--------------------------------------------------------------------------------
// Crossing of the lines (p, p + d) and (q, q + e); false when nearly parallel
static bool inspectCross(float px, float py, float dx, float dy, float qx, float qy, float ex, float ey, float &x, float &y)
{
   float det = dx*(-ey) - dy*(-ex);

   if (fabsf(det) < 1e-6f)
      return false;

   float t = ((qx - px)*(-ey) - (qy - py)*(-ex))/det;

   x = px + t*dx;
   y = py + t*dy;
   return true;
}

/*--------------------------------------------------------------------------------
   Sobel magnitude of a keyframe, standing (turned 90 degrees clockwise) and box-reduced by 4, as a
   24-bit gray BMP: the background the pencil lines are drawn on (--hidden-floor).
  --------------------------------------------------------------------------------*/
static void inspectSobelUpright(const TImageRecord &img, LPCSTR path)
{
   const int    f = 4;
   int          w = (int)img.width,
                h = (int)img.height,
                sw = w/f,
                sh = h/f,
                ow = sh,
                oh = sw,
                rowBytes = (ow*3 + 3) & ~3;
   TAlloc<BYTE> luma((size_t)w*h),
                reduced((size_t)sw*sh),
                mag((size_t)sw*sh),
                line((size_t)rowBytes);

   if (!inspectDecode(img.pixels, img.pixelBytes, img.width, img.height, false, luma))
      return;
   for (int y = 0; y < sh; y++)
      for (int x = 0; x < sw; x++)
      {
         DWORD sum = 0u;

         for (int dy = 0; dy < f; dy++)
            for (int dx = 0; dx < f; dx++)
               sum += luma[(size_t)(y*f + dy)*w + x*f + dx];
         reduced[(size_t)y*sw + x] = (BYTE)(sum/(DWORD)(f*f));
      }
   memset(mag(), 0, (size_t)sw*sh);
   for (int y = 1; y + 1 < sh; y++)
      for (int x = 1; x + 1 < sw; x++)
      {
         LPCBYTE p = reduced() + (size_t)y*sw + x;
         int     gx = (p[-sw + 1] + 2*p[1] + p[sw + 1]) - (p[-sw - 1] + 2*p[-1] + p[sw - 1]),
                 gy = (p[sw - 1] + 2*p[sw] + p[sw + 1]) - (p[-sw - 1] + 2*p[-sw] + p[-sw + 1]),
                 m = (int)sqrtf((float)(gx*gx + gy*gy))*2;

         mag[(size_t)y*sw + x] = (BYTE)(m > 255 ? 255 : m);
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
   for (int oy = oh - 1; oy >= 0; oy--) // bottom-up; clockwise: output (ox, oy) <- source (oy, sh - 1 - ox)
   {
      memset(line(), 0, (size_t)rowBytes);
      for (int ox = 0; ox < ow; ox++)
         memset(line() + ox*3, mag[(size_t)(sh - 1 - ox)*sw + oy], 3u);
      fwrite(line(), 1u, (size_t)rowBytes, bmp);
   }
   fclose(bmp);
}

//--------------------------------------------------------------------------------
// Luma at an upright full-picture point (nearest pixel); -1 outside
static float inspectUprightLuma(const TImageRecord &img, LPCBYTE luma, float x, float y)
{
   int u = (int)(y + 0.5f),
       v = (int)img.height - 1 - (int)(x + 0.5f);

   if (u < 0 || v < 0 || u >= (int)img.width || v >= (int)img.height)
      return -1.f;
   return (float)luma[(size_t)v*img.width + u];
}

/*--------------------------------------------------------------------------------
   The real floor line under a baseboard (user, 2026-09-28: the baseboard's top has more contrast than
   the floor junction, the pick sat ~7 cm high; the vanishing point is filter zero - of two lines
   through it, the one a little lower wins even if fainter). At 200 points along the picked line,
   the luma step across it is read on the full picture down to cHiddenBoardM below; every local peak
   of a point's steps votes for its offset. Real lines of the pencil pile votes at one offset while a
   fan, a cable or a tile joint scatter theirs. The lowest vote peak holding cHiddenPeakShare of the
   strongest wins, and the line moves there, still through the vanishing point.
  --------------------------------------------------------------------------------*/
static void inspectFloorProfile(const TImageRecord &img, LPCBYTE luma, float vx, float vy, float pxPerM, TPencilLine &l,
                                int index, LPCSTR name)
{
   float nx = -sinf(l.phi),
         ny = cosf(l.phi),
         mx = 0.5f*(l.x0 + l.x1),
         my = 0.5f*(l.y0 + l.y1),
         t0 = (l.x0 - vx)*cosf(l.phi) + (l.y0 - vy)*sinf(l.phi),
         t1 = (l.x1 - vx)*cosf(l.phi) + (l.y1 - vy)*sinf(l.phi);
   int   reach = (int)(cHiddenBoardM*pxPerM*1.5f) + 4, // the baseboard stands nearer than the corner: more pixels per meter
         votes[inspectProfileMax],
         smooth[inspectProfileMax],
         best = 0;

   if (ny < 0.f)
   {
      nx = -nx;
      ny = -ny; // downward on the picture
   }
   if (reach > inspectProfileMax - 2)
      reach = inspectProfileMax - 2;
   memset(votes, 0, sizeof(votes));
   for (int s = 0; s <= 200; s++)
   {
      float t = t0 + (t1 - t0)*(float)s/200.f,
            bx = vx + t*cosf(l.phi),
            by = vy + t*sinf(l.phi),
            step[inspectProfileMax];

      for (int d = 0; d < reach + 2; d++)
      {
         float x = bx + (float)(d - 2)*nx,
               y = by + (float)(d - 2)*ny,
               above = inspectUprightLuma(img, luma, x - 1.5f*nx, y - 1.5f*ny),
               below = inspectUprightLuma(img, luma, x + 1.5f*nx, y + 1.5f*ny);

         step[d] = above < 0.f || below < 0.f ? 0.f : fabsf(below - above);
      }
      float strongest = 0.f; // this point's own strongest edge: the grain of the picture does not vote

      for (int d = 2; d < reach; d++)
         strongest = fmaxf(strongest, step[d]);
      for (int d = 2; d < reach; d++) // step[d] is offset d - 2
         if (step[d] >= cHiddenStepMin && step[d] >= cHiddenStepShare*strongest && step[d] >= step[d - 1]
             && step[d] >= step[d + 1] && step[d] >= step[d - 2] && step[d] >= step[d + 2])
            votes[d - 2]++;
   }
   for (int d = 0; d < reach; d++)
   {
      smooth[d] = votes[d] + (d > 0 ? votes[d - 1] : 0) + (d + 1 < reach ? votes[d + 1] : 0);
      best = smooth[d] > best ? smooth[d] : best;
   }

   int pick = 0;

   printf("  hidden floor frame %03d: %s floor line vote piles (offset px: votes):", index, name);
   for (int d = 0; d + 1 < reach; d++)
      if (smooth[d] >= inspectVotesMin && smooth[d] >= (d > 0 ? smooth[d - 1] : 0) && smooth[d] > smooth[d + 1])
         printf(" %d:%d", d, smooth[d]);
   printf("\n");

   // the pick's own pile, then the strongest pile a baseboard's height below it that holds a good share of it
   int own = 0,
       first = (int)(cHiddenBoardMinM*pxPerM);

   for (int d = 0; d <= 5 && d < reach; d++)
      own = smooth[d] > own ? smooth[d] : own;

   /* the profile's background: its median pile; a lower edge must stand out of it (cHiddenProminence) - on a flat
      profile (tiles, no baseboard) the line stays where its own Sobel points put it */
   int sorted[inspectProfileMax],
       median;

   memcpy(sorted, smooth, sizeof(int)*(size_t)reach);
   for (int i = 1; i < reach; i++)
   {
      int v = sorted[i],
          j = i - 1;

      while (j >= 0 && sorted[j] > v)
      {
         sorted[j + 1] = sorted[j];
         j--;
      }
      sorted[j + 1] = v;
   }
   median = sorted[reach/2];
   for (int d = first > 3 ? first : 3; d + 1 < reach; d++)
      if ((float)smooth[d] >= cHiddenPeakShare*(float)own && smooth[d] >= inspectVotesMin && smooth[d] >= smooth[d - 1]
          && smooth[d] >= smooth[d + 1] && (float)smooth[d] >= cHiddenProminence*(float)median
          && (!pick || smooth[d] > smooth[pick]))
         pick = d;
   printf("  hidden floor frame %03d: %s floor line votes below the pick: strongest %d, at the pick %d, floor pile %d px"
          " down (%d votes) - %.1f cm at the corner's scale\n", index, name, best, own, pick, smooth[pick],
          100.f*(float)pick/pxPerM);
   if (!pick)
      return;

   float px = mx + (float)pick*nx,
         py = my + (float)pick*ny;

   l.phi = atan2f(py - vy, px - vx);
   l.x0 = vx + t0*cosf(l.phi);
   l.y0 = vy + t0*sinf(l.phi);
   l.x1 = vx + t1*cosf(l.phi);
   l.y1 = vy + t1*sinf(l.phi);
}

//--------------------------------------------------------------------------------
// Camera ray of an upright full-picture point of a frame (the inverse of inspectUpright)
static TVec3 inspectUprightRay(const TIntrinsics &k, DWORD height, float x, float y)
{
   TVec3 d = { (y - k.cx)/k.fx, -((float)height - 1.f - x - k.cy)/k.fy, -1.f };

   return d;
}

//--------------------------------------------------------------------------------
// Upright full-picture point of a camera direction with the given pinhole; false behind the camera
static bool inspectUprightAt(const TIntrinsics &k, DWORD height, const TVec3 &d, float &x, float &y)
{
   if (d.z > -1e-6f)
      return false;

   float u = k.cx - k.fx*d.x/d.z,
         v = k.cy + k.fy*d.y/d.z;

   x = (float)height - 1.f - v;
   y = u;
   return true;
}

/*--------------------------------------------------------------------------------
   A frame's pose against its own corner (user, 2026-09-28: the ceiling corner guides the homography):
   the room axes in camera coordinates from its vanishing points - up turned to the world's up, A
   turned to agree with the station's reference axis refA (world), B = up x A - and the camera's
   position in the room, origin on the floor under the corner, y up: the corner top's ray, with the
   ceiling height above and the camera height h, places the camera. No gyroscope drift enters it.
  --------------------------------------------------------------------------------*/
static bool inspectCornerPose(const THiddenFrame &fr, const TVec3 &refA, float h, TVec3 &ax, TVec3 &au, TVec3 &ab,
                              TVec3 &pos)
{
   au = fr.vr.dirCam[0];
   ax = fr.vr.dirCam[1];
   if (fr.pose.RotateVector(au).y < 0.f)
   {
      au.x = -au.x;
      au.y = -au.y;
      au.z = -au.z;
   }

   float d = ax.x*au.x + ax.y*au.y + ax.z*au.z;

   ax.x -= d*au.x;
   ax.y -= d*au.y;
   ax.z -= d*au.z;

   float len = sqrtf(ax.x*ax.x + ax.y*ax.y + ax.z*ax.z);

   if (!fr.res.corner || !(len > 1e-6f))
      return false;
   ax.x /= len;
   ax.y /= len;
   ax.z /= len;

   TVec3 wa = fr.pose.RotateVector(ax);

   if (wa.x*refA.x + wa.y*refA.y + wa.z*refA.z < 0.f)
   {
      ax.x = -ax.x;
      ax.y = -ax.y;
      ax.z = -ax.z;
   }
   ab.x = au.y*ax.z - au.z*ax.y;
   ab.y = au.z*ax.x - au.x*ax.z;
   ab.z = au.x*ax.y - au.y*ax.x;

   TVec3 r = inspectUprightRay(fr.intr, fr.height, fr.res.ccx, fr.res.ccy);
   float ra = r.x*ax.x + r.y*ax.y + r.z*ax.z,
         ry = r.x*au.x + r.y*au.y + r.z*au.z,
         rb = r.x*ab.x + r.y*ab.y + r.z*ab.z;

   if (ry <= 1e-6f)
      return false;

   float t = (inspectHiddenCeilingM - h)/ry;

   pos.x = -t*ra;
   pos.y = h;
   pos.z = -t*rb;
   return true;
}

/*--------------------------------------------------------------------------------
   A point of frame f seen from frame g. The frames of a station are taken turning the body in place
   (user, 2026-09-28): one camera center, same height, same distances - so the transfer is a pure
   rotation (H = K R K^-1), for the floor and for furniture alike, needing no distance or height. The
   rotation comes from each frame's own corner axes (its vanishing points), finer than the gyroscope.
  --------------------------------------------------------------------------------*/
static bool inspectFloorTransfer(const THiddenFrame &f, const THiddenFrame &g, const TVec3 &refA, float h, float x, float y,
                                 float &ox, float &oy)
{
   TVec3 fa,
         fu,
         fb,
         fp,
         ga,
         gu,
         gb,
         gp;

   if (!inspectCornerPose(f, refA, h, fa, fu, fb, fp) || !inspectCornerPose(g, refA, h, ga, gu, gb, gp))
      return false;

   TVec3 r = inspectUprightRay(f.intr, f.height, x, y);
   float da = r.x*fa.x + r.y*fa.y + r.z*fa.z,
         dy = r.x*fu.x + r.y*fu.y + r.z*fu.z,
         db = r.x*fb.x + r.y*fb.y + r.z*fb.z;
   TVec3 c = { ga.x*da + gu.x*dy + gb.x*db, ga.y*da + gu.y*dy + gb.y*db, ga.z*da + gu.z*dy + gb.z*db };

   return inspectUprightAt(g.intr, g.height, c, ox, oy);
}

//--------------------------------------------------------------------------------
// Subpixel peak of three samples around a local maximum (parabola), in [-0.5, 0.5]
static float inspectPeak(float before, float at, float after)
{
   float den = before - 2.f*at + after;

   if (!(den < -1e-6f))
      return 0.f;

   float off = 0.5f*(before - after)/den;

   return off < -0.5f ? -0.5f : (off > 0.5f ? 0.5f : off);
}

/*--------------------------------------------------------------------------------
   Edge response across the pencil line through (vx, vy) at an upright point: the luma step across the
   line (4 px), kept only where the edge runs along the line (the step along it is less than half); 0
   elsewhere or outside.
  --------------------------------------------------------------------------------*/
static float inspectPencilStep(const TImageRecord &img, LPCBYTE luma, float vx, float vy, float x, float y)
{
   float tx = x - vx,
         ty = y - vy,
         tl = sqrtf(tx*tx + ty*ty);

   if (tl < 1.f)
      return 0.f;
   tx /= tl;
   ty /= tl;

   float nx = -ty,
         ny = tx,
         a = inspectUprightLuma(img, luma, x - 2.f*nx, y - 2.f*ny),
         b = inspectUprightLuma(img, luma, x + 2.f*nx, y + 2.f*ny),
         c = inspectUprightLuma(img, luma, x - 2.f*tx, y - 2.f*ty),
         d = inspectUprightLuma(img, luma, x + 2.f*tx, y + 2.f*ty);

   if (a < 0.f || b < 0.f || c < 0.f || d < 0.f)
      return 0.f;

   float across = fabsf(b - a),
         along = fabsf(d - c);

   return across >= cCornerStepMin && across >= 2.f*along ? across : 0.f;
}

//--------------------------------------------------------------------------------
// Angle a relative to base, wrapped into (-pi, pi]
static float inspectRelAngle(float a, float base)
{
   float d = a - base;

   while (d > 3.14159265f)
      d -= 6.28318531f;
   while (d <= -3.14159265f)
      d += 6.28318531f;
   return d;
}

/*--------------------------------------------------------------------------------
   The strong lines of one family on the full picture between rows yFrom and yTo: every sampled pixel whose edge runs
   toward the family's vanishing point adds its step to the pencil line through it (angle bins 0.5 px
   wide at the picture's middle); the peaks, placed to a subpixel, are the family's real lines - a
   molding's edges each on its own, however close. Returns the count, strongest first up to max.
  --------------------------------------------------------------------------------*/
static int inspectPencilSweep(const TImageRecord &img, LPCBYTE luma, float vx, float vy, float yFrom, float yTo, float minSum,
                              float *phiOut,
                              float *powOut, int max)
{
   float w = (float)img.height,
         h = (float)img.width,
         cx = 0.5f*w,
         cy = 0.5f*(yFrom + yTo),
         dist = sqrtf((cx - vx)*(cx - vx) + (cy - vy)*(cy - vy)),
         bin = 0.5f/fmaxf(dist, 1.f),
         base = atan2f(cy - vy, cx - vx), // angles measured from the picture's middle: no wrap at +-pi
         lo = 1e9f,
         hi = -1e9f;
   float corners[4][2] = { { 0.f, yFrom }, { w, yFrom }, { 0.f, yTo }, { w, yTo } };

   for (int i = 0; i < 4; i++)
   {
      float a = inspectRelAngle(atan2f(corners[i][1] - vy, corners[i][0] - vx), base);

      lo = fminf(lo, a);
      hi = fmaxf(hi, a);
   }
   if (hi - lo > 3.14159265f) // the vanishing point inside the picture: not a horizontal family
      return 0;

   int bins = (int)((hi - lo)/bin) + 3;

   if (bins > inspectSweepBins)
      bins = inspectSweepBins;
   bin = (hi - lo)/(float)(bins - 2);

   TAlloc<float> acc((size_t)bins);

   memset(acc(), 0, sizeof(float)*(size_t)bins);
   for (int y = (int)fmaxf(2.f, yFrom); y < (int)fminf(h - 2.f, yTo); y += inspectSweepStep)
      for (int x = 2; x + 2 < (int)w; x += inspectSweepStep)
      {
         float r = inspectPencilStep(img, luma, vx, vy, (float)x, (float)y);

         if (r <= 0.f)
            continue;

         int b = (int)((inspectRelAngle(atan2f((float)y - vy, (float)x - vx), base) - lo)/bin) + 1;

         if (b > 0 && b + 1 < bins)
            acc[b] += r;
      }

   int n = 0;

   for (int b = 2; b + 2 < bins; b++)
   {
      float s = acc[b - 1] + acc[b] + acc[b + 1];

      if (s < acc[b - 2] + acc[b - 1] + acc[b] || s <= acc[b] + acc[b + 1] + acc[b + 2] || s < minSum)
         continue;

      float phi = base + lo + ((float)b - 0.5f + inspectPeak(acc[b - 1], acc[b], acc[b + 1]))*bin;
      bool  full = n == max;
      int   at = full ? max - 1 : n++; // kept strongest first: when full, a weaker one drops off the end

      if (full && s <= powOut[at])
         continue;
      while (at > 0 && powOut[at - 1] < s)
      {
         phiOut[at] = phiOut[at - 1];
         powOut[at] = powOut[at - 1];
         at--;
      }
      phiOut[at] = phi;
      powOut[at] = s;
   }
   return n;
}

//--------------------------------------------------------------------------------
// Mean edge response along a pencil line from a point toward one side (+1 along (cos phi, sin phi)), between two distances
static float inspectLineSupportAt(const TImageRecord &img, LPCBYTE luma, float vx, float vy, float phi, float x, float y, float d0,
                                  float d1, float side)
{
   float sum = 0.f,
         dx = cosf(phi),
         dy = sinf(phi);
   int   n = 0;

   for (float d = d0; d <= d1; d += 3.f)
   {
      float px = x + side*d*dx,
            py = y + side*d*dy,
            r = 0.f;

      for (int o = -1; o <= 1; o++) // a pixel of slack across the line
         r = fmaxf(r, inspectPencilStep(img, luma, vx, vy, px - (float)o*dy, py + (float)o*dx));
      sum += r;
      n++;
   }
   return n ? sum/(float)n : 0.f;
}

/*--------------------------------------------------------------------------------
   The Sobel run of a pencil line from a point toward one side (user, 2026-09-28: a vector only from
   its own Sobel points). Samples every 3 px up to cFloorRunMaxPx, each the edge step along the line
   (a pixel of slack across it); the strong ones (cFloorRunStep) with gaps of at most cFloorRunGapPx
   make a run, and the longest run is the line's cloud: a real edge is continuous and strong, floor
   texture only strings weak loose steps. The run may start anywhere along the wall (something may
   hide the line near the corner). Its ends, the sum of its steps and its samples; its points go to
   the csv when one is given.
  --------------------------------------------------------------------------------*/
static int inspectLineRun(const TImageRecord &img, LPCBYTE luma, float vx, float vy, float phi, float x, float y, float side,
                          float &sx, float &sy, float &ex, float &ey, float &sum, FILE *csv, LPCSTR tag)
{
   float dx = cosf(phi),
         dy = sinf(phi),
         rs[inspectRunSamples];
   int   n = 0,
         bestFrom = 0,
         bestTo = -1,
         bestHits = 0,
         from = -1,
         last = -1,
         hits = 0;

   for (float d = 0.f; d <= cFloorRunMaxPx && n < inspectRunSamples; d += 3.f)
   {
      float px = x + side*d*dx,
            py = y + side*d*dy,
            r = 0.f;

      for (int o = -1; o <= 1; o++)
         r = fmaxf(r, inspectPencilStep(img, luma, vx, vy, px - (float)o*dy, py + (float)o*dx));
      rs[n++] = r;
   }
   for (int i = 0; i <= n; i++)
   {
      bool strong = i < n && rs[i] >= cFloorRunStep;

      if (strong && from >= 0 && 3.f*(float)(i - last) > cFloorRunGapPx)
      {
         if (hits > bestHits)
         {
            bestHits = hits;
            bestFrom = from;
            bestTo = last;
         }
         from = -1;
      }
      if (strong)
      {
         if (from < 0)
         {
            from = i;
            hits = 0;
         }
         last = i;
         hits++;
      }
      else if (i == n && from >= 0 && hits > bestHits)
      {
         bestHits = hits;
         bestFrom = from;
         bestTo = last;
      }
   }
   sx = x + side*3.f*(float)bestFrom*dx;
   sy = y + side*3.f*(float)bestFrom*dy;
   ex = bestTo >= 0 ? x + side*3.f*(float)bestTo*dx : sx;
   ey = bestTo >= 0 ? y + side*3.f*(float)bestTo*dy : sy;
   sum = 0.f;
   for (int i = bestFrom; i <= bestTo; i++)
   {
      if (rs[i] < cFloorRunStep)
         continue;
      sum += rs[i];
      if (csv)
         fprintf(csv, "sup,%s,%.1f,%.1f,%.1f,\r\n", tag, x + side*3.f*(float)i*dx, y + side*3.f*(float)i*dy, rs[i]);
   }
   return bestHits;
}

//--------------------------------------------------------------------------------
// Mean edge response along a pencil line from a point, on the side where it is stronger, between two distances
static float inspectLineSupport(const TImageRecord &img, LPCBYTE luma, float vx, float vy, float phi, float x, float y, float d0,
                                float d1, float &sideOut)
{
   float best = 0.f,
         dx = cosf(phi),
         dy = sinf(phi);

   for (int side = -1; side <= 1; side += 2)
   {
      float sum = 0.f;
      int   n = 0;

      for (float d = d0; d <= d1; d += 3.f)
      {
         float px = x + (float)side*d*dx,
               py = y + (float)side*d*dy,
               r = 0.f;

         for (int o = -1; o <= 1; o++) // a pixel of slack across the line
            r = fmaxf(r, inspectPencilStep(img, luma, vx, vy, px - (float)o*dy, py + (float)o*dx));
         sum += r;
         n++;
      }
      if ((n ? sum/(float)n : 0.f) > best)
      {
         best = sum/(float)n;
         sideOut = (float)side;
      }
   }
   return best;
}

/*--------------------------------------------------------------------------------
   Premise zero (user, 2026-09-28): the corner top, before anything else. The strong lines of both
   horizontal families on the full picture; every crossing of an A line and a B line where both lines
   stay strong right out of the crossing (cCornerHold of their own strength over the next 250 px on
   their wall's side) closes a corner; the highest one is the top of the moldings. False when no pair
   closes (the corner out of the picture, or no molding).
  --------------------------------------------------------------------------------*/
static bool inspectCornerTop(const TImageRecord &img, LPCBYTE luma, const float *vx, const float *vy, int index, float &ccx,
                             float &ccy, float &phiA, float &phiB, float &sideA, float &sideB, float &holdA, float &holdB)
{
   float pa[inspectSweepMax],
         wa[inspectSweepMax],
         pb[inspectSweepMax],
         wb[inspectSweepMax],
         bestY = 1e9f;
   int   na = inspectPencilSweep(img, luma, vx[1], vy[1], 0.f, 0.6f*(float)img.width, cCornerLineMin, pa, wa,
                                 inspectSweepMax),
         nb = inspectPencilSweep(img, luma, vx[2], vy[2], 0.f, 0.6f*(float)img.width, cCornerLineMin, pb, wb,
                                 inspectSweepMax);
   bool  found = false;

   for (int a = 0; a < na; a++)
      for (int b = 0; b < nb; b++)
      {
         float x,
               y;

         if (!inspectCross(vx[1], vy[1], cosf(pa[a]), sinf(pa[a]), vx[2], vy[2], cosf(pb[b]), sinf(pb[b]), x, y))
            continue;
         if (x < 0.f || y < 0.f || x >= (float)img.height || y >= 0.6f*(float)img.width || y >= bestY)
            continue;

         float ta,
               tb,
               sa = inspectLineSupport(img, luma, vx[1], vy[1], pa[a], x, y, 8.f, 250.f, ta),
               sb = inspectLineSupport(img, luma, vx[2], vy[2], pb[b], x, y, 8.f, 250.f, tb);

         if (sa < cCornerHold || sb < cCornerHold)
            continue;
         bestY = y;
         ccx = x;
         ccy = y;
         phiA = pa[a];
         phiB = pb[b];
         sideA = ta;
         sideB = tb;
         holdA = sa;
         holdB = sb;
         found = true;
      }
   printf("  hidden floor frame %03d: premise zero: %d A and %d B lines on the full picture, corner top %s (%.1f, %.1f)\n", index,
          na, nb, found ? "closed at" : "not closed", found ? ccx : NAN, found ? ccy : NAN);
   return found;
}

/*--------------------------------------------------------------------------------
   The hidden floor line of a corner frame (--hidden-floor N; user, 2026-09-28: furniture edges share
   the vanishing point of their wall's direction; with the corner's foot, that point gives the floor
   line of a wall hidden behind furniture). On the upright picture, from the vanishing detector's
   own edges: the lines of each family (vertical, A, B) as peaks of the angle around its vanishing
   point; the ceiling corner where the highest A and B lines meet (the moldings); the corner's
   vertical from there to the vertical vanishing point; its foot on the lowest A line in the lower
   half (the A wall's floor line); and the B wall's floor line from that foot to the B vanishing
   point. Everything goes to oculto_NNN.csv (edges, vanishing points, lines, points) to be drawn on
   the picture.
  --------------------------------------------------------------------------------*/
static void inspectHiddenFloor(const TImageRecord &img, const TVanishResult &vr, const TVanishEdges &edges, int index,
                               const THiddenHint *hint, THiddenResult &res)
{
   char  path[sessionPathMax];
   float camM = hint && !isnan(hint->cameraM) ? hint->cameraM : inspectHiddenCameraM,
         gateShare = hint ? hint->gateShare : cHiddenGateShare;
   bool  predicted = hint && !isnan(hint->predX);

   res.corner = false;
   res.heightM[0] = NAN;
   res.heightM[1] = NAN;
   res.share[0] = 0.f;
   res.share[1] = 0.f;
   res.cands[0] = 0;
   res.cands[1] = 0;

   snprintf(path, sizeof(path), "%s/sobel_%03d.bmp", inspectOutDir, index);
   inspectSobelUpright(img, path);
   snprintf(path, sizeof(path), "%s/oculto_%03d.csv", inspectOutDir, index);

   FILE *csv = fopen(path, "wb");

   if (!csv)
      return;

   int           n = (int)edges.count;
   TAlloc<float> ex((size_t)(n > 0 ? n : 1)),
                 ey((size_t)(n > 0 ? n : 1));
   TAlloc<BYTE>  el((size_t)(n > 0 ? n : 1));
   float         vx[3],
                 vy[3],
                 cx = 0.5f*(float)img.height,
                 cy = 0.5f*(float)img.width;
   TPencilLine   family[3][inspectPencilMax];
   int           lines[3];

   fprintf(csv, "type,name,x1,y1,x2,y2\r\n");
   for (int i = 0; i < n; i++)
   {
      TVec3 r = { edges.ray[3*i], edges.ray[3*i + 1], edges.ray[3*i + 2] };

      el[i] = inspectUpright(img, r, ex[i], ey[i]) ? edges.label[i] : 0u;
      if (el[i])
         fprintf(csv, "edge,%u,%.1f,%.1f,,\r\n", (unsigned)el[i], ex[i], ey[i]);
   }
   for (int k = 0; k < 3; k++)
   {
      lines[k] = 0;
      if (!inspectUpright(img, vr.dirCam[k], vx[k], vy[k]))
         continue;
      fprintf(csv, "vp,%d,%.1f,%.1f,,\r\n", k + 1, vx[k], vy[k]);
      res.vx[k] = vx[k];
      res.vy[k] = vy[k];
      lines[k] = inspectPencil(ex(), ey(), el(), n, (BYTE)(k + 1), vx[k], vy[k], cx, cy, family[k]);
      for (int l = 0; l < lines[k]; l++)
      {
         fprintf(csv, "line,%d,%.1f,%.1f,%.1f,%.1f\r\n", k + 1, family[k][l].x0, family[k][l].y0, family[k][l].x1,
                 family[k][l].y1);
         printf("  hidden floor frame %03d: family %d line %d, %d edges, mean height %.0f px\n", index, k + 1, l,
                family[k][l].count, family[k][l].meanY);
      }
   }

   /* the moldings: on the ceiling the highest line wins (below it are furniture tops and the molding's lower edges),
      compared on a column where the family's upper lines really are - the median of their segments' middles; far
      from their support, lines a little apart in angle cross and swap. On the second pass only the lines passing
      near the corner the station predicts compete (a frame turned aside may show another corner's moldings higher) */
   int top[3] = { -1, -1, -1 };

   for (int k = 1; k <= 2; k++)
   {
      float mids[inspectPencilMax],
            refX = cx;
      int   n = 0;

      for (int l = 0; l < lines[k]; l++)
         if (family[k][l].meanY < 0.5f*(float)img.width)
            mids[n++] = 0.5f*(family[k][l].x0 + family[k][l].x1);
      for (int i = 1; i < n; i++)
      {
         float v = mids[i];
         int   j = i - 1;

         while (j >= 0 && mids[j] > v)
         {
            mids[j + 1] = mids[j];
            j--;
         }
         mids[j + 1] = v;
      }
      if (n)
         refX = mids[n/2];
      for (int l = 0; l < lines[k]; l++)
      {
         float off = (hint ? hint->predX - vx[k] : 0.f)*sinf(family[k][l].phi)
                     - (hint ? hint->predY - vy[k] : 0.f)*cosf(family[k][l].phi);

         if (predicted && fabsf(off) > cHiddenPredictPx)
            continue;
         if (top[k] < 0
             || inspectPencilY(family[k][l], vx[k], vy[k], refX) < inspectPencilY(family[k][top[k]], vx[k], vy[k], refX))
            top[k] = l;
      }
   }

   TAlloc<BYTE> luma((size_t)img.width*img.height);
   bool         hasLuma = inspectDecode(img.pixels, img.pixelBytes, img.width, img.height, false, luma),
                paired = false;
   float        ccx = NAN,
                ccy = NAN,
                margin = predicted ? cHiddenEdgeShare : 0.f,
                pairY = 1e9f,
                zeroPhi[3] = { 0.f, 0.f, 0.f },
                zeroSide[3] = { 1.f, 1.f, 1.f },
                zeroHold[3] = { 0.f, 0.f, 0.f };

   // premise zero on the full picture first: the moldings' own edges, each on its own, closing on the corner top
   if (hasLuma
       && inspectCornerTop(img, luma(), vx, vy, index, ccx, ccy, zeroPhi[1], zeroPhi[2], zeroSide[1], zeroSide[2], zeroHold[1],
                           zeroHold[2]))
   {
      for (int k = 1; k <= 2; k++)
      {
         if (lines[k] >= inspectPencilMax)
            lines[k] = inspectPencilMax - 1;

         TPencilLine &l = family[k][lines[k]];

         l.phi = zeroPhi[k];
         l.x0 = ccx;
         l.y0 = ccy;
         l.x1 = ccx + zeroSide[k]*cHiddenMoldingPx*cosf(zeroPhi[k]);
         l.y1 = ccy + zeroSide[k]*cHiddenMoldingPx*sinf(zeroPhi[k]);
         l.count = (int)zeroHold[k];
         l.meanY = ccy;
         top[k] = lines[k]++;
      }
      paired = true;
   }

   /* premise zero (user, 2026-09-28): the top lines converge on the corner top - before anything else. Every pair of
      upper lines of the two families where BOTH segments really reach their crossing (end within cHiddenReachGap of
      it) is a corner; the highest such crossing is the top of the moldings. Only when no pair closes (the corner out
      of the picture) do the single highest lines and the station's prediction stand in */
   for (int a = 0; !paired && a < lines[1]; a++)
      for (int b = 0; b < lines[2]; b++)
      {
         const TPencilLine &la = family[1][a],
                           &lb = family[2][b];
         float              x,
                            y;

         if (la.meanY >= 0.5f*(float)img.width || lb.meanY >= 0.5f*(float)img.width
             || !inspectCross(vx[1], vy[1], cosf(la.phi), sinf(la.phi), vx[2], vy[2], cosf(lb.phi), sinf(lb.phi), x, y))
            continue;

         float ga = fminf(sqrtf((la.x0 - x)*(la.x0 - x) + (la.y0 - y)*(la.y0 - y)),
                          sqrtf((la.x1 - x)*(la.x1 - x) + (la.y1 - y)*(la.y1 - y))),
               gb = fminf(sqrtf((lb.x0 - x)*(lb.x0 - x) + (lb.y0 - y)*(lb.y0 - y)),
                          sqrtf((lb.x1 - x)*(lb.x1 - x) + (lb.y1 - y)*(lb.y1 - y)));

         if (ga > cHiddenReachGap || gb > cHiddenReachGap || y >= pairY)
            continue;
         pairY = y;
         ccx = x;
         ccy = y;
         top[1] = a;
         top[2] = b;
         paired = true;
      }

   int topA = top[1],
       topB = top[2];

   /* no pair closes: where the single highest moldings meet; a frame turned aside may show one molding or none -
      then the station's prediction is the corner (on the molding that shows, the point nearest the prediction) */
   if (paired)
      printf("  hidden floor frame %03d: the moldings close on the corner top (%.0f, %.0f)\n", index, ccx, ccy);
   else if (topA >= 0 && topB >= 0)
      inspectCross(vx[1], vy[1], cosf(family[1][topA].phi), sinf(family[1][topA].phi), vx[2], vy[2],
                   cosf(family[2][topB].phi), sinf(family[2][topB].phi), ccx, ccy);
   else if (predicted && (topA >= 0 || topB >= 0))
   {
      int   k = topA >= 0 ? 1 : 2;
      float dx = cosf(family[k][top[k]].phi),
            dy = sinf(family[k][top[k]].phi),
            t = (hint->predX - vx[k])*dx + (hint->predY - vy[k])*dy;

      ccx = vx[k] + t*dx;
      ccy = vy[k] + t*dy;
      printf("  hidden floor frame %03d: one molding in view: the corner is its point nearest the station's (%.0f, %.0f)\n",
             index, ccx, ccy);
   }
   else if (predicted)
   {
      ccx = hint->predX;
      ccy = hint->predY;
      printf("  hidden floor frame %03d: no molding in view: the corner is the station's (%.0f, %.0f)\n", index, ccx, ccy);
   }
   if (isnan(ccx))
   {
      printf("  hidden floor frame %03d: the moldings do not meet\n", index);
      fclose(csv);
      return;
   }
   if (ccx < -margin*(float)img.height || ccy < -margin*(float)img.width || ccx >= (1.f + margin)*(float)img.height
       || ccy >= (1.f + margin)*(float)img.width)
   {
      printf("  hidden floor frame %03d: the moldings meet outside the picture (%.0f, %.0f): not a corner frame\n", index,
             ccx, ccy);
      fclose(csv);
      return;
   }
   fprintf(csv, "point,ceiling corner,%.1f,%.1f,,\r\n", ccx, ccy);
   for (int k = 1; k <= 2; k++) // premise zero: both moldings reach the corner top they make
      if (top[k] >= 0)
      {
         const TPencilLine &m = family[k][top[k]];
         float              d0 = sqrtf((m.x0 - ccx)*(m.x0 - ccx) + (m.y0 - ccy)*(m.y0 - ccy)),
                            d1 = sqrtf((m.x1 - ccx)*(m.x1 - ccx) + (m.y1 - ccy)*(m.y1 - ccy));

         printf("  hidden floor frame %03d: %s molding (%d edges) ends %.0f px from the corner top\n", index, k == 1 ? "A" : "B",
                m.count, fminf(d0, d1));
      }
   res.corner = true;
   res.ccx = ccx;
   res.ccy = ccy;
   for (int k = 1; k <= 2; k++) // each wall's view: the share of the picture on its molding's side of the corner
   {
      if (top[k] < 0)
         continue;

      const TPencilLine &m = family[k][top[k]];
      float              farX = fabsf(m.x0 - ccx) > fabsf(m.x1 - ccx) ? m.x0 : m.x1,
                         left = fminf(1.f, fmaxf(0.f, ccx/(float)img.height));

      res.share[k - 1] = farX < ccx ? left : 1.f - left;
      res.side[k - 1] = (farX - ccx)*(vx[k] - ccx) >= 0.f ? 1.f : -1.f; // does the wall run toward its vanishing point
      fprintf(csv, "pick,%s molding,%.1f,%.1f,%.1f,%.1f\r\n", k == 1 ? "A" : "B", m.x0, m.y0, m.x1, m.y1);
   }
   for (int k = 1; k <= 2; k++) // a molding out of view: its wall is on the other side of the corner
      if (top[k] < 0)
      {
         const TPencilLine *o = top[3 - k] >= 0 ? &family[3 - k][top[3 - k]] : NULL;
         float              otherX = o ? (fabsf(o->x0 - ccx) > fabsf(o->x1 - ccx) ? o->x0 : o->x1) : ccx - 1.f;

         res.share[k - 1] = o ? 1.f - res.share[2 - k] : 0.5f;
         res.side[k - 1] = (ccx - otherX)*(vx[k] - ccx) >= 0.f ? 1.f : -1.f;
      }

   /* the foot the room's heights expect: the ceiling corner's elevation gives the distance to the corner
      (ceiling above the camera / tan), the camera height puts the foot below it on the same vertical */
   TVec3 up = vr.dirCam[0],
         rc = { (ccy - img.intr.cx)/img.intr.fx, -((float)img.height - 1.f - ccx - img.intr.cy)/img.intr.fy, -1.f };
   float s = rc.x*up.x + rc.y*up.y + rc.z*up.z;
   TVec3 hc = { rc.x - s*up.x, rc.y - s*up.y, rc.z - s*up.z };
   float hl = sqrtf(hc.x*hc.x + hc.y*hc.y + hc.z*hc.z),
         dist = hl > 1e-6f && s > 1e-6f ? (inspectHiddenCeilingM - camM)*hl/s : NAN,
         ex0 = NAN,
         ey0 = NAN;

   if (!isnan(dist))
   {
      TVec3 fd = { hc.x/hl*dist - up.x*camM, hc.y/hl*dist - up.y*camM,
                   hc.z/hl*dist - up.z*camM };

      if (!inspectUpright(img, fd, ex0, ey0))
         ex0 = NAN;
   }
   printf("  hidden floor frame %03d: ceiling corner at (%.0f, %.0f), %.2f m away; the heights (%.2f / %.2f m) expect"
          " its foot at (%.0f, %.0f)\n", index, ccx, ccy, dist, inspectHiddenCeilingM, camM, ex0, ey0);
   res.efx = ex0;
   res.efy = ey0;

   /* every line of each horizontal family (user, 2026-09-28): the ones crossing the corner's vertical far from the
      expected foot are furniture - the expected foot is rough, so the gate is a share of the corner's height on the
      picture; of the ones left, the lowest crossing is the floor (above it: baseboard tops, grooves), then refined
      down to the junction by the votes. The ones left are kept as the frame's candidates for the station; on the last
      pass the station elects one of them (hint->elect) */
   bool         picked[2] = { false, false };
   float        pick[2][4],
                fcx = ex0,
                fcy = ey0,
                bestGap = 1e9f,
                pxPerM = isnan(ey0) ? 0.f : fabsf(ey0 - ccy)/inspectHiddenCeilingM, // on the corner's vertical
                gate = isnan(ey0) ? 0.f : gateShare*fabsf(ey0 - ccy);

   for (int k = 1; k <= 2; k++)
   {
      /* the family's lines on the full picture below the corner top, each weighed by its own strength (user: a clean
         strong floor line - the wardrobe's base in frame 44 - must vote as strongly as it is) */
      float sweepPhi[inspectSweepMax],
            sweepPow[inspectSweepMax],
            candY = -1e9f,
            x,
            y;
      int   swept = hasLuma ? inspectPencilSweep(img, luma(), vx[k], vy[k], isnan(ey0) ? 0.5f*(float)img.width
                                                 : 0.5f*(ccy + ey0), (float)img.width, cFloorLineMin, sweepPhi, sweepPow,
                                                 inspectSweepMax)
                            : 0,
            kept = 0,
            dropped = 0,
            ended = 0;
      TPencilLine best = {};

      for (int l = 0; l < swept; l++)
      {
         float dx = cosf(sweepPhi[l]),
               dy = sinf(sweepPhi[l]);

         if (!inspectCross(ccx, ccy, vx[0] - ccx, vy[0] - ccy, vx[k], vy[k], dx, dy, x, y))
            continue;

         float gap = isnan(ey0) ? 1e9f : sqrtf((x - ex0)*(x - ex0) + (y - ey0)*(y - ey0)),
               way = res.side[k - 1] > 0.f ? -1.f : 1.f; // along the wall: toward the vanishing point is -(dx, dy)

         if (gap > gate || y <= ccy)
         {
            dropped++;
            continue;
         }

         /* the floor's own premise: its line ENDS at the corner's foot - strong along its wall up to the vertical,
            not beyond it (there the other wall's base starts). A tile joint parallel to the wall runs on across the
            vertical, a furniture edge ends anywhere */
         float wall = hasLuma ? inspectLineSupportAt(img, luma(), vx[k], vy[k], sweepPhi[l], x, y, 10.f, cFloorReachPx, way)
                              : 0.f,
               beyond = hasLuma ? inspectLineSupportAt(img, luma(), vx[k], vy[k], sweepPhi[l], x, y, 10.f, cFloorReachPx, -way)
                                : 0.f,
               ends = wall > 0.f ? fmaxf(0.f, 1.f - beyond/wall) : 0.f; // 1: nothing past the vertical, 0: as much as along

         // the candidate IS its own Sobel run along its wall: its far end and its strength
         float runSX = x,
               runSY = y,
               runX = x,
               runY = y,
               runSum = 0.f;
         int   run = hasLuma ? inspectLineRun(img, luma(), vx[k], vy[k], sweepPhi[l], x, y, way, runSX, runSY, runX, runY, runSum,
                                              NULL, NULL)
                             : 0;

         if (wall < cFloorHold || (float)run < cFloorRunMin)
         {
            ended++;
            continue;
         }
         kept++;
         if (res.cands[k - 1] < inspectPencilMax)
         {
            float *c = res.cand[k - 1][res.cands[k - 1]++];

            c[0] = x;
            c[1] = y;
            c[2] = runX;
            c[3] = runY;
            c[4] = runSum*ends; // its points' strength, as far as it ends at the foot
         }
         if (ends >= cFloorBeyondShare && y > candY) // the lowest that ends at the foot: above it are baseboard tops
         {
            candY = y;
            best.phi = sweepPhi[l];
            best.x0 = x;
            best.y0 = y;
            best.x1 = runX;
            best.y1 = runY;
            best.count = run;
            best.meanY = y;
         }
      }
      printf("  hidden floor frame %03d: %s family: %d full-picture lines, %d along its wall, %d not there, %d"
             " far from the expected foot (gate %.0f px)\n", index, k == 1 ? "A" : "B", swept, kept, ended, dropped, gate);

      /* the station elects among this frame's OWN candidates (user, 2026-09-28: the homography only moves vote weights,
         never a line's geometry): hint->elect is the index of the one its own plus inherited votes chose, -1 when
         none - the wall is hidden in this frame. Without a station decision the frame chooses alone, the lowest */
      int elect = hint ? hint->elect[k - 1] : -2;

      if (elect == -1 || (elect >= 0 && elect >= res.cands[k - 1]))
      {
         printf("  hidden floor frame %03d: %s: the station's votes elect none of its own lines - hidden here\n", index,
                k == 1 ? "A" : "B");
         continue;
      }
      if (elect >= 0)
      {
         const float *c = res.cand[k - 1][elect];

         best.phi = atan2f(c[1] - vy[k], c[0] - vx[k]);
         best.x0 = c[0];
         best.y0 = c[1];
         best.x1 = c[2];
         best.y1 = c[3];
         best.count = (int)c[4];
         best.meanY = c[1];
      }
      else if (candY < -1e8f)
         continue;

      /* the baseboard refinement may move the line only onto another line with its own Sobel run along its wall;
         otherwise the candidate stands. The pick is drawn exactly over its run, the run's points go to the csv */
      TPencilLine chosen = best;
      float       wayK = (best.x1 - best.x0)*cosf(best.phi) + (best.y1 - best.y0)*sinf(best.phi) >= 0.f ? 1.f : -1.f,
                  runSX = x,
                  runSY = y,
                  runX = x,
                  runY = y,
                  runSum = 0.f;
      int         run = 0;

      if (hasLuma && pxPerM > 0.f)
         inspectFloorProfile(img, luma(), vx[k], vy[k], pxPerM, best, index, k == 1 ? "A" : "B");
      if (inspectCross(ccx, ccy, vx[0] - ccx, vy[0] - ccy, vx[k], vy[k], cosf(best.phi), sinf(best.phi), x, y) && hasLuma)
         run = inspectLineRun(img, luma(), vx[k], vy[k], best.phi, x, y, wayK, runSX, runSY, runX, runY, runSum, NULL, NULL);
      if ((float)run < cFloorRunMin)
      {
         if (best.phi != chosen.phi)
            printf("  hidden floor frame %03d: %s: the refined line has no Sobel run of its own (%d samples) - the"
                   " candidate stands\n", index, k == 1 ? "A" : "B", run);
         best = chosen;
      }
      if (!inspectCross(ccx, ccy, vx[0] - ccx, vy[0] - ccy, vx[k], vy[k], cosf(best.phi), sinf(best.phi), x, y))
         continue;
      run = hasLuma ? inspectLineRun(img, luma(), vx[k], vy[k], best.phi, x, y, wayK, runSX, runSY, runX, runY, runSum, csv,
                                     k == 1 ? "A" : "B")
                    : 0;
      printf("  hidden floor frame %03d: %s floor line: its Sobel run holds %d strong samples from %.0f to %.0f px off the"
             " foot\n", index, k == 1 ? "A" : "B", run, sqrtf((runSX - x)*(runSX - x) + (runSY - y)*(runSY - y)),
             sqrtf((runX - x)*(runX - x) + (runY - y)*(runY - y)));
      fprintf(csv, "ext,%s floor,%.1f,%.1f,%.1f,%.1f\r\n", k == 1 ? "A" : "B", x, y, runSX, runSY); // from the foot to its run
      pick[k - 1][0] = runSX;
      pick[k - 1][1] = runSY;
      pick[k - 1][2] = runX;
      pick[k - 1][3] = runY;

      // the camera height this wall's floor line gives with the ceiling corner: ceiling x tan e_foot/(tan e_top + tan e_foot)
      float gap = isnan(ey0) ? 0.f : sqrtf((x - ex0)*(x - ex0) + (y - ey0)*(y - ey0));
      TVec3 rk = inspectUprightRay(img.intr, img.height, x, y);
      float sk = rk.x*up.x + rk.y*up.y + rk.z*up.z;
      TVec3 hk = { rk.x - sk*up.x, rk.y - sk*up.y, rk.z - sk*up.z };
      float tanK = -sk/sqrtf(hk.x*hk.x + hk.y*hk.y + hk.z*hk.z);

      res.heightM[k - 1] = inspectHiddenCeilingM*tanK/(s/hl + tanK);
      printf("  hidden floor frame %03d: %s floor line%s crosses the vertical at (%.0f, %.0f), %.0f px from the expected"
             " foot; camera %.2f m by it, its wall's view %.0f%%\n", index, k == 1 ? "A" : "B",
             elect >= 0 ? " (its own, elected by the station)" : "", x, y, gap, res.heightM[k - 1], 100.f*res.share[k - 1]);
      picked[k - 1] = true;
      if (gap < bestGap)
      {
         bestGap = gap;
         fcx = x;
         fcy = y;
      }
   }
   if (isnan(fcx))
   {
      printf("  hidden floor frame %03d: no foot for the corner\n", index);
      fclose(csv);
      return;
   }
   /* a foot seen on a real floor line: the ceiling and floor crossings together give the distance and the camera
      height without assuming either - distance = ceiling height / (tan e_top + tan e_foot), height = distance tan e_foot */
   if (bestGap < 1e8f)
   {
      TVec3 rf = inspectUprightRay(img.intr, img.height, fcx, fcy);
      float sf = rf.x*up.x + rf.y*up.y + rf.z*up.z;
      TVec3 hf = { rf.x - sf*up.x, rf.y - sf*up.y, rf.z - sf*up.z };
      float tanTop = s/hl,
            tanFoot = -sf/sqrtf(hf.x*hf.x + hf.y*hf.y + hf.z*hf.z),
            both = inspectHiddenCeilingM/(tanTop + tanFoot);

      printf("  hidden floor frame %03d: ceiling and floor seen together: %.2f m to the corner, camera %.2f m high\n",
             index, both, both*tanFoot);
   }
   fprintf(csv, "pick,corner vertical,%.1f,%.1f,%.1f,%.1f\r\n", ccx, ccy, fcx, fcy);
   fprintf(csv, "point,floor corner,%.1f,%.1f,,\r\n", fcx, fcy);
   for (int k = 1; k <= 2; k++)
      if (picked[k - 1])
         fprintf(csv, "pick,%s floor,%.1f,%.1f,%.1f,%.1f\r\n", k == 1 ? "A" : "B", pick[k - 1][0], pick[k - 1][1],
                 pick[k - 1][2], pick[k - 1][3]);
      else
      {
         // drawn from the foot along its wall's side (the line runs through the vanishing point both ways)
         float hx = vx[k] - fcx,
               hy = vy[k] - fcy,
               hl = sqrtf(hx*hx + hy*hy) + 1e-6f;

         fprintf(csv, "hidden,%s floor,%.1f,%.1f,%.1f,%.1f\r\n", k == 1 ? "A" : "B", fcx, fcy,
                 fcx + res.side[k - 1]*cFloorRunMaxPx*hx/hl, fcy + res.side[k - 1]*cFloorRunMaxPx*hy/hl);
         printf("  hidden floor frame %03d: %s floor line deduced from the foot (%.0f, %.0f) toward its vanishing point"
                " (%.0f, %.0f)\n", index, k == 1 ? "A" : "B", fcx, fcy, vx[k], vy[k]);
      }
   fclose(csv);
}

static const float cAxisPriorGateDeg = 12.f,  // a frame's axis this far off the room's (mod 90) is measured again
                   cAxisPriorWindowDeg = 10.f, // held within this of the room's (the gyroscope's drift and sway)
                   cDiagonalAgreeDeg = 3.f;    // the tiles' axis (less 45) and the walls' agree within this

enum {
   cDiagonalMinSupport = 300 // tile edges (both diagonals) the joint axis needs
};

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

   /* own lines (--rectify-own): a horizontal axis far off the room's (101732's frame 81 took the diagonal tiles,
      44 degrees off) is measured again held near the room's (user: "a horizontal vector in the region of the
      gyroscope's degree cannot be ignored"); the wall's own lines - a cabinet's base, a baseboard - win there */
   if (ok && inspectRectOwn && axis.HasReference() && (vr.flags & (vfAxisA | vfAxisB)) && !isnan(vr.roomAxisDeg)
       && fabsf(vanishAxisDiffDeg(vr.roomAxisDeg, axis.ReferenceDeg())) > cAxisPriorGateDeg)
   {
      TVanishConfig held = tuned;
      TVanishResult again = {};

      held.priorAxisDeg = axis.ReferenceDeg();
      held.priorWindowDeg = cAxisPriorWindowDeg;
      if (vanishDetect(luma(), (int)img.width, (int)img.height, (int)img.width, img.intr, img.cameraToWorld, held,
                       again, &edges)
          && (again.flags & (vfAxisA | vfAxisB)))
      {
         printf("  frame %d: axis %.2f is %.1f deg off the room's %.2f; held near it: %.2f (support A %lu B %lu)\n",
                index, vr.roomAxisDeg, vanishAxisDiffDeg(vr.roomAxisDeg, axis.ReferenceDeg()), axis.ReferenceDeg(),
                again.roomAxisDeg, (unsigned long)again.support[1], (unsigned long)again.support[2]);
         vr = again;
      }
      else // the held search found nothing: the edges go back to the frame's own measure
         vanishDetect(luma(), (int)img.width, (int)img.height, (int)img.width, img.intr, img.cameraToWorld, tuned, vr,
                      &edges);
   }
   /* --diagonal-floor: the tiles laid at 45 degrees to the walls are the room's axis too (user: the floor's lines are
      a needle). The frame measured again held near the tiles' heading; the room axis is the walls' and the tiles'
      (less 45) together, by support, when the two agree within cDiagonalAgreeDeg - in 101732's frame 75 the walls'
      own lines (A 375 edges) sat 1.9 degrees off the tiles' */
   if (ok && inspectDiagonalFloor && axis.HasReference() && (vr.flags & (vfAxisA | vfAxisB)) && !isnan(vr.roomAxisDeg))
   {
      TVanishConfig held = tuned;
      TVanishResult tiles = {};

      held.priorAxisDeg = fmodf(vr.roomAxisDeg + 45.f, 90.f);
      held.priorWindowDeg = cDiagonalAgreeDeg;
      if (vanishDetect(luma(), (int)img.width, (int)img.height, (int)img.width, img.intr, img.cameraToWorld, held,
                       tiles, NULL)
          && (tiles.flags & vfAxisA) && (tiles.flags & vfAxisB) && !isnan(tiles.roomAxisDeg))
      {
         float tilesAxis = fmodf(tiles.roomAxisDeg + 45.f, 90.f),
               gap = vanishAxisDiffDeg(tilesAxis, vr.roomAxisDeg),
               ww = (float)(vr.support[1] + vr.support[2]),
               wt = (float)(tiles.support[1] + tiles.support[2]),
               joint = vr.roomAxisDeg + gap*wt/fmaxf(ww + wt, 1.f);

         if (fabsf(gap) <= cDiagonalAgreeDeg && wt >= (float)cDiagonalMinSupport)
         {
            const TMat4 &pose = img.cameraToWorld;
            TVec3        up = vr.dirCam[0];

            for (int k = 1; k <= 2; k++) // axes A and B at the joint heading, square to the measured vertical
            {
               float h = (joint + (k == 2 ? 90.f : 0.f))*0.01745329f;
               TVec3 aw = { sinf(h), 0.f, -cosf(h) },
                     a = { pose.m[0]*aw.x + pose.m[1]*aw.y + pose.m[2]*aw.z, pose.m[4]*aw.x + pose.m[5]*aw.y + pose.m[6]*aw.z,
                           pose.m[8]*aw.x + pose.m[9]*aw.y + pose.m[10]*aw.z };
               float d = a.x*up.x + a.y*up.y + a.z*up.z,
                     l;

               a.x -= d*up.x;
               a.y -= d*up.y;
               a.z -= d*up.z;
               l = sqrtf(a.x*a.x + a.y*a.y + a.z*a.z);
               if (l > 1e-6f)
               {
                  vr.dirCam[k].x = a.x/l;
                  vr.dirCam[k].y = a.y/l;
                  vr.dirCam[k].z = a.z/l;
               }
            }
            printf("  frame %d: walls %.2f (A %lu B %lu), tiles %.2f less 45 (A %lu B %lu): the room axis %.2f\n", index,
                   vr.roomAxisDeg, (unsigned long)vr.support[1], (unsigned long)vr.support[2], tilesAxis,
                   (unsigned long)tiles.support[1], (unsigned long)tiles.support[2], joint);
            vr.roomAxisDeg = joint;
            vr.flags |= (BYTE)(vfAxisA | vfAxisB);
         }
      }
   }
   kept = vr; // the frame's own measure (or its predictions): the wall composition turns the frame with it
   if (ok)
      tilt.Add(vr);
   if (ok && index < inspectMaxFrames && inspectHidden[index] && inspectOutDir && inspectHiddenCount < inspectHiddenMax)
   {
      THiddenFrame &h = inspectHiddenData[inspectHiddenCount++]; // kept for the station passes

      h.index = index;
      h.station = hasMeta ? (int)meta.stationIndex : 0;
      h.pose = img.cameraToWorld;
      h.vr = vr;
      h.count = edges.count;
      h.intr = img.intr;
      h.width = img.width;
      h.height = img.height;
      {
         TAlloc<float> ray((size_t)edges.count*3u + 1u);
         TAlloc<BYTE>  label((size_t)edges.count + 1u);

         memcpy(ray(), edges.ray, sizeof(float)*(size_t)edges.count*3u);
         memcpy(label(), edges.label, (size_t)edges.count);
         ray.Drop(h.ray);
         label.Drop(h.label);
      }
      inspectHiddenFloor(img, vr, edges, index, NULL, h.res); // blind: each frame on its own
   }
   if (ok && edgePath)
      inspectEdgeImage(luma(), (int)img.width, (int)img.height, img.intr, edges, edgePath);
   if (ok && rectDir)
      inspectRectify(img, vr, axis.HasReference() ? axis.ReferenceDeg() : vr.roomAxisDeg, rectDir, index, 0.f, 0.f,
                     index < inspectMaxFrames ? &inspectRect[index] : NULL, false); // measured; written leveled later
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
      printf("    wall %c = %.2f along %.2f..%.2f weight %.0f views %d\n", plan.wallKind[i] ? 'w' : 'u', plan.wallOffset[i],
             plan.wallA0[i], plan.wallA1[i], plan.wallWeight[i], plan.wallViews[i]);
}

static bool inspectShowCreases = false, // --creases: list every crease the walls were clustered from
            inspectFreePlan = false;    // --free-plan: the bundle may move walls and camera height (else held on the plan)

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
   printf("    door head: %s, scale %.3f (assumed ceiling %.2f m, implied %.2f m)%s\n", plan.doorFound ? "found" : "none",
          plan.doorScale, plan.assumedCeilingM, plan.impliedCeilingM,
          plan.ceilingSnapped ? " - plan scaled to the typical ceiling" : "");
   for (int i = 0; i < plan.centerCount; i++)
      printf("    center %d: u %.2f w %.2f\n", i, plan.centers[i].u, plan.centers[i].w);
   for (int i = 0; i < plan.openDoors; i++)
      printf("    open door: wall %d (%c = %.2f) at %.2f\n", plan.openDoorWall[i], plan.wallKind[plan.openDoorWall[i]] ? 'w' : 'u',
             plan.wallOffset[plan.openDoorWall[i]], plan.openDoorAt[i]);
   printf("    camera height from %d room corners; %d creases from the corner stations\n", plan.heightCorners,
          plan.stationLines);
   for (int i = 0; inspectShowCreases && i < plan.creaseCount; i++)
      printf("    crease: frame %d station %u side %u dist %.2f along %.2f..%.2f weight %.0f\n", plan.creases[i].frame,
             (unsigned)plan.creases[i].station, (unsigned)plan.creases[i].half, plan.creases[i].dist, plan.creases[i].a0,
             plan.creases[i].a1, plan.creases[i].weight);
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

         if ((pass == 0) != (station == 0) || inspectKind[f] == (BYTE)skDoor
             || (inspectIsSuperseded(inspectStamp[f]) && !(inspectPicking && inspectPick[f])) // a named frame beats the election
             || (inspectPicking && !inspectPick[f]))
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
         m.picked = inspectPicking && inspectPick[f];
         m.xyz = m.picked || (frameVr[f].flags & (vfVertical | vfAxisA | vfAxisB)) == (vfVertical | vfAxisA | vfAxisB);
         m.stationX = 0.f;
         m.stationZ = 0.f;
         m.camY = 0.f;
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
   int xyz = 0;

   for (int i = 0; i < count; i++)
      xyz += list[i].xyz ? 1 : 0;
   for (int f = 0; inspectPicking && f < frames && f < inspectMaxFrames; f++)
   {
      bool placed = false;

      for (int i = 0; i < count && !placed; i++)
         placed = list[i].index == f;
      if (inspectPick[f] && !placed && frameInfo[f]/inspectStations == room)
         printf("  named frame %03d left out: no vanishing rotation that agrees with the gyroscope%s\n", f,
                inspectIsSuperseded(inspectStamp[f]) ? " (or a replaced photo)" : "");
   }
   printf("  walls: %d walls from %d center-spin and %d corner-station frames, %d of them XYZ (merged)\n",
          plan.vertexCount, centers, count - centers, xyz);
   mosaicWalls(list(), count, centers, plan, rounds, bundleRounds, jointRounds, spinRadiusM, !inspectFreePlan, outDir);
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
// Point (u, w) inside the plan polygon (even-odd rule)
static bool inspectInsidePlan(const TLayoutPlan &plan, float u, float w)
{
   bool inside = false;

   for (int i = 0; i < plan.vertexCount; i++)
   {
      const TPlanPoint &p = plan.verts[i],
                       &q = plan.verts[(i + plan.vertexCount - 1)%plan.vertexCount];

      if ((p.u > u) != (q.u > u) && w < p.w + (q.w - p.w)*(u - p.u)/(q.u - p.u))
         inside = !inside;
   }
   return inside;
}

static const float cPanoDegPerPx = 0.08f, // --panorama: equirectangular resolution (a frame pixel is ~0.02 degrees)
                   cPanoSharp = 4.f;      // refined panorama: feather power (1 blends evenly, higher leans on the central view)

static const float cFloorMinShare = 0.03f,   // of the frame's sampled rays on the floor, else it adds nothing
                   cFloorRangeM = 4.5f,      // horizontal reach from the camera (at 1.5 m: 18 degrees below the horizon)
                   cFloorMarginM = 0.3f;     // mosaic margin around the plan polygon

static LPCSTR cFloorAxes[4] = { "z", "xz", "yz", "xy" }; // axes measured besides the vertical, no corner in view

// A floor accumulator on the 4 mm lattice (col = floor(w*ppm), row = floor(-u*ppm)): weighted b, g, r and the weight
struct TFloorGrid {
   int   row0,
         col0,
         rows,
         cols;
   float *acc;
};

// The frame's turn (inspectFrameRotation's form) and where its camera stands, world offsets from the grid origin
struct TFloorPose {
   TVec3 camA,
         camUp,
         camC,
         worldA,
         worldC;
   float camX,
         camZ;
};

/*--------------------------------------------------------------------------------
   The frame's rotation: its vanishing points when they agree with the gyroscope, else the gyroscope
   (world x, up, z in camera axes: the rows of the camera-to-world rotation). On the spin the camera
   rides the circle out along its forward; elsewhere it stands on the grid origin.
  --------------------------------------------------------------------------------*/
static bool inspectFloorPose(const TImageRecord &img, const TVanishResult &vr, float axisDeg, bool onSpin,
                             float spinRadiusM, TFloorPose &p)
{
   bool byVanish = inspectFrameRotation(img.cameraToWorld, vr, axisDeg, p.camA, p.camUp, p.camC, p.worldA, p.worldC);

   if (!byVanish)
   {
      const TMat4 &m = img.cameraToWorld;

      p.camA.x = m.m[0];
      p.camA.y = m.m[4];
      p.camA.z = m.m[8];
      p.camUp.x = m.m[1];
      p.camUp.y = m.m[5];
      p.camUp.z = m.m[9];
      p.camC.x = m.m[2];
      p.camC.y = m.m[6];
      p.camC.z = m.m[10];
      p.worldA.x = 1.f;
      p.worldA.y = 0.f;
      p.worldA.z = 0.f;
      p.worldC.x = 0.f;
      p.worldC.y = 0.f;
      p.worldC.z = 1.f;
   }

   float fwdX = p.worldA.x*(-p.camA.z) + p.worldC.x*(-p.camC.z),
         fwdZ = p.worldA.z*(-p.camA.z) + p.worldC.z*(-p.camC.z),
         fwdLen = sqrtf(fwdX*fwdX + fwdZ*fwdZ);

   p.camX = onSpin && fwdLen > 1e-6f ? spinRadiusM*fwdX/fwdLen : 0.f;
   p.camZ = onSpin && fwdLen > 1e-6f ? spinRadiusM*fwdZ/fwdLen : 0.f;
   return byVanish;
}

/*--------------------------------------------------------------------------------
   Where a frame lands on the floor: a grid of its rays, within reach (and inside the room when
   clipped). Returns the share of rays on the floor and their bounding box in plan axes.
  --------------------------------------------------------------------------------*/
static float inspectFloorFootprint(const TImageRecord &img, const TFloorPose &p, const TVec3 &ud, const TVec3 &wd, float h,
                                   const TLayoutPlan *clip, float &uMin, float &uMax, float &wMin, float &wMax)
{
   int rays = 0,
       onFloor = 0;

   uMin = 1e9f;
   uMax = -1e9f;
   wMin = 1e9f;
   wMax = -1e9f;
   for (DWORD y = 8u; y < img.height; y += 32u)
      for (DWORD x = 8u; x < img.width; x += 32u)
      {
         float dx = ((float)x - img.intr.cx)/img.intr.fx,
               dy = -((float)y - img.intr.cy)/img.intr.fy,
               ka = dx*p.camA.x + dy*p.camA.y - p.camA.z,
               ky = dx*p.camUp.x + dy*p.camUp.y - p.camUp.z,
               kc = dx*p.camC.x + dy*p.camC.y - p.camC.z,
               wx = p.worldA.x*ka + p.worldC.x*kc,
               wz = p.worldA.z*ka + p.worldC.z*kc;

         rays++;
         if (ky >= -1e-3f)
            continue;

         float t = h/-ky,
               px = p.camX + t*wx,
               pz = p.camZ + t*wz,
               pu = px*ud.x + pz*ud.z,
               pw = px*wd.x + pz*wd.z;

         if (t*sqrtf(wx*wx + wz*wz) > cFloorRangeM)
            continue;
         if (clip && !inspectInsidePlan(*clip, pu, pw))
            continue;
         onFloor++;
         uMin = fminf(uMin, pu);
         uMax = fmaxf(uMax, pu);
         wMin = fminf(wMin, pw);
         wMax = fmaxf(wMax, pw);
      }
   return rays ? (float)onFloor/(float)rays : 0.f;
}

/*--------------------------------------------------------------------------------
   A room corner in the picture: some plan vertex's floor-to-ceiling edge projects inside it (at
   least 3 of 10 points, 5% off the border) - the frame shows X, Y and Z of the room at once, the
   ones the fit trusts (user, 2026-09-28). A floor tile grid measures both axes too, but no corner.
  --------------------------------------------------------------------------------*/
static bool inspectFloorSeesCorner(const TImageRecord &img, const TFloorPose &p, const TLayoutPlan &plan, const TVec3 &ud,
                                   const TVec3 &wd, float h)
{
   for (int k = 0; k < plan.vertexCount; k++)
   {
      int inView = 0;

      for (int s = 0; s < 10; s++)
      {
         float ox = plan.verts[k].u*ud.x + plan.verts[k].w*wd.x - p.camX,
               oz = plan.verts[k].u*ud.z + plan.verts[k].w*wd.z - p.camZ,
               oy = plan.ceilingM*((float)s + 0.5f)/10.f - h,
               ka = ox*p.worldA.x + oz*p.worldA.z,
               kc = ox*p.worldC.x + oz*p.worldC.z;
         TVec3 rc = { p.camA.x*ka + p.camUp.x*oy + p.camC.x*kc, p.camA.y*ka + p.camUp.y*oy + p.camC.y*kc,
                      p.camA.z*ka + p.camUp.z*oy + p.camC.z*kc };

         if (rc.z > -1e-3f)
            continue;

         float su = img.intr.cx + img.intr.fx*rc.x/-rc.z,
               sv = img.intr.cy - img.intr.fy*rc.y/-rc.z;

         if (su > 0.05f*(float)img.width && su < 0.95f*(float)img.width && sv > 0.05f*(float)img.height
             && sv < 0.95f*(float)img.height)
            inView++;
      }
      if (inView >= 3)
         return true;
   }
   return false;
}

/*--------------------------------------------------------------------------------
   One frame onto a floor grid, 2x2 samples per pixel (near the camera one output pixel covers
   several source pixels). A sample weighs sin^3 of its depression below the horizon: the steep
   views are the sharp ones and the ones furniture leans least in.
  --------------------------------------------------------------------------------*/
static void inspectFloorSplat(const TImageRecord &img, LPCBYTE bgr, const TFloorPose &p, const TVec3 &ud, const TVec3 &wd,
                              float h, float ppm, const TLayoutPlan *clip, TFloorGrid &g)
{
   int w = (int)img.width,
       ih = (int)img.height;

   for (int r = 0; r < g.rows; r++)
      for (int c = 0; c < g.cols; c++)
      {
         float *cell = g.acc + ((size_t)r*g.cols + c)*4u;

         for (int s = 0; s < 4; s++)
         {
            float pu = -((float)(g.row0 + r) + 0.25f + 0.5f*(float)(s/2))/ppm,
                  pw = ((float)(g.col0 + c) + 0.25f + 0.5f*(float)(s%2))/ppm,
                  ox = pu*ud.x + pw*wd.x - p.camX,
                  oz = pu*ud.z + pw*wd.z - p.camZ,
                  d2 = ox*ox + oz*oz;

            if (d2 > cFloorRangeM*cFloorRangeM)
               continue;
            if (clip && !inspectInsidePlan(*clip, pu, pw))
               continue;

            float ka = ox*p.worldA.x + oz*p.worldA.z,
                  kc = ox*p.worldC.x + oz*p.worldC.z;
            TVec3 rc = { p.camA.x*ka - p.camUp.x*h + p.camC.x*kc, p.camA.y*ka - p.camUp.y*h + p.camC.y*kc,
                         p.camA.z*ka - p.camUp.z*h + p.camC.z*kc };

            if (rc.z > -1e-3f)
               continue;

            float su = img.intr.cx + img.intr.fx*rc.x/-rc.z,
                  sv = img.intr.cy - img.intr.fy*rc.y/-rc.z;
            int   iu = (int)floorf(su),
                  iv = (int)floorf(sv);

            if (iu < 0 || iv < 0 || iu + 1 >= w || iv + 1 >= ih)
               continue;

            float du = su - (float)iu,
                  dv = sv - (float)iv,
                  sn = h/sqrtf(d2 + h*h),
                  wgt = sn*sn*sn;

            for (int ch = 0; ch < 3; ch++)
            {
               LPCBYTE q = bgr + ((size_t)iv*w + iu)*3u + ch;
               float   top = (float)q[0]*(1.f - du) + (float)q[3]*du,
                       bot = (float)q[(size_t)w*3u]*(1.f - du) + (float)q[(size_t)w*3u + 3u]*du;

               cell[ch] += wgt*(top*(1.f - dv) + bot*dv);
            }
            cell[3] += wgt;
         }
      }
}

/*--------------------------------------------------------------------------------
   A floor grid as a 24-bit BMP (top row = the highest u; the BMP resolution field carries 250 px/m),
   the plan walls in yellow and the marks (camera nadirs) as white crosses.
  --------------------------------------------------------------------------------*/
static void inspectFloorBMP(const TFloorGrid &g, float ppm, const TLayoutPlan *plan, const TPlanPoint *marks, int markCount,
                            LPCSTR path)
{
   int          rowBytes = (g.cols*3 + 3) & ~3;
   TAlloc<BYTE> out((size_t)rowBytes*g.rows);

   memset(out(), 0, (size_t)rowBytes*g.rows);
   for (int r = 0; r < g.rows; r++)
   {
      LPBYTE row = out() + (size_t)(g.rows - 1 - r)*rowBytes; // bottom-up

      for (int c = 0; c < g.cols; c++)
      {
         const float *cell = g.acc + ((size_t)r*g.cols + c)*4u;

         for (int ch = 0; cell[3] > 0.f && ch < 3; ch++)
            row[c*3 + ch] = (BYTE)fminf(255.f, cell[ch]/cell[3] + 0.5f);
      }
   }
   for (int i = 0; plan && i < plan->vertexCount; i++)
   {
      const TPlanPoint &p = plan->verts[i],
                       &q = plan->verts[(i + 1)%plan->vertexCount];
      float             len = sqrtf((q.u - p.u)*(q.u - p.u) + (q.w - p.w)*(q.w - p.w));
      int               steps = (int)(len*ppm) + 1;

      for (int s = 0; s <= steps; s++)
      {
         float t = (float)s/(float)steps;
         int   r = (int)floorf(-(p.u + (q.u - p.u)*t)*ppm) - g.row0,
               c = (int)floorf((p.w + (q.w - p.w)*t)*ppm) - g.col0;

         for (int d = -1; d <= 1; d++)
            for (int e = -1; e <= 1; e++)
               if (r + d >= 0 && r + d < g.rows && c + e >= 0 && c + e < g.cols)
               {
                  LPBYTE px = out() + (size_t)(g.rows - 1 - r - d)*rowBytes + (c + e)*3;

                  px[0] = 0u;
                  px[1] = 220u;
                  px[2] = 255u;
               }
      }
   }
   for (int i = 0; i < markCount; i++)
      for (int k = -40; k <= 40; k++)
      {
         int r = (int)floorf(-marks[i].u*ppm) - g.row0,
             c = (int)floorf(marks[i].w*ppm) - g.col0;

         for (int arm = 0; arm < 2; arm++)
         {
            int rr = arm ? r : r + k,
                cc = arm ? c + k : c;

            if (rr >= 0 && rr < g.rows && cc >= 0 && cc < g.cols)
               memset(out() + (size_t)(g.rows - 1 - rr)*rowBytes + cc*3, 255, 3u);
         }
      }

   FILE *bmp = fopen(path, "wb");

   if (!bmp)
      return;

   DWORD imageBytes = (DWORD)rowBytes*(DWORD)g.rows,
         header[13] = { 54u + imageBytes, 0u, 54u, 40u, (DWORD)g.cols, (DWORD)g.rows, 0x00180001u, 0u, imageBytes,
                        (DWORD)ppm, (DWORD)ppm, 0u, 0u };
   BYTE  magic[2] = { 'B', 'M' };

   fwrite(magic, 1u, 2u, bmp);
   fwrite(header, 4u, 13u, bmp);
   fwrite(out(), 1u, imageBytes, bmp);
   fclose(bmp);
}

/*--------------------------------------------------------------------------------
   Panorama registration (user, 2026-09-28: a panorama is worth only its coherent seams - no ghosts).
   The gyroscope places each center-spin frame to a few degrees; the overlaps (each view shares
   ~3/4 with the next) fix it. Every frame becomes a pyramid of high-pass images (1/8 down to 1/256
   of the picture, normalized: exposure differences drop out; user, 2026-09-30: the coarse levels
   catch what the gyroscope misses by degrees, before any finer freedom is let loose). At each strong-edge pixel of a frame
   its world direction is projected into every neighbor seeing it, and the two intensities there are
   one residual. All rotations are solved together, coarse to fine, by Gauss-Newton on small world
   rotations (3 per frame, least squares), each residual weighed by Huber (furniture near the camera shifts by
   parallax - the spin turns about the body, not the lens) and a weak damping that holds the
   set's free overall rotation at the gyroscope's. The refined rotations land in inspectPanoRot.
  --------------------------------------------------------------------------------*/
enum {
   inspectPanoMax    = 128,  // center-spin frames registered at most
   inspectPanoLevels = 6,    // 1/8 .. 1/256: the coarsest pixel ~5 degrees, wide enough for the gyroscope's errors
   inspectPanoShrink = 8,    // the finest level's pixel: this many picture pixels
   inspectPanoIters  = 8     // Gauss-Newton rounds per level
};

static const float cPanoNeighborCos = 0.5f,   // frames whose forwards lie within 60 degrees overlap
                   cPanoHuber = 1.5f,         // Huber knee, in RMS residuals of the previous round
                   cPanoPrior = 1e-4f,        // damping, share of the mean diagonal
                   cPanoShiftPrior = 1e-2f,   // each frame's own shift held harder (alone on one wall it trades with the turn)
                   cPanoEdgeShare = 1.f,      // sample pixels: gradient above this share of the frame's mean
                   cDtScanStepSec = 0.05f,    // --dt-scan: between trial delays
                   cPairCos = 0.8f,           // the pairs: forwards within ~37 degrees
                   cPairHuberDeg = 0.5f,      // a pair's misfit beyond this weighs less
                   cPairGyroPull = 1e-3f;     // the weak pull toward the gyroscope (share of the mean diagonal)

enum {
   cPanoFocalLevel = 2, // the focal scale is solved from this level (1/32) down
   cDtScanSteps = 20,   // --dt-scan: trial delays each side
   cPairRounds = 2,     // the pairs: measured and solved this many times
   cPairIters = 5,      // reweighed solves per round
   cPairMinInliers = 12 // a pair with fewer points meeting gives no turn
};

// One center-spin frame being registered: its pinhole, rotation (camera to world, TMat4 layout) and pyramid
struct TPanoFrame {
   int           index;
   QWORD         stamp;       // its picture's stamp (the attitude log is searched by it)
   float         fx,
                 fy,
                 cx,
                 cy,
                 rot[9],      // columns: camera x, y, z in world (m0 m1 m2 / m4 m5 m6 / m8 m9 m10)
                 shift[3],    // its camera center's own offset beyond the lever (world, m): the body sways
                 gyro[9],     // the gyroscope's, kept for the report
                 edge[inspectPanoLevels]; // mean gradient of each level (the sampling threshold)
   int           w[inspectPanoLevels],
                 h[inspectPanoLevels];
   TBlock<float> hp[inspectPanoLevels];
   TBlock<BYTE>  gray[3];     // plain gray at 1/2, 1/4 and 1/16 (the points' pyramid)
   int           gw[3],
                 gh[3];
};

static TPanoFrame inspectPano[inspectPanoMax];
static int        inspectPanoCount = 0;
static float      inspectPanoRot[inspectMaxFrames][9]; // refined rotation of each frame (TMat4 layout, 3x3)
static bool       inspectPanoHas[inspectMaxFrames];

/*--------------------------------------------------------------------------------
   The spin turns about the body, not the lens (user, 2026-09-29: the parallax is one more freedom,
   to be used): every camera center sits at c = R o from the turn's pivot (the world origin), o the
   lever arm in camera axes, one for the whole spin. Once the plan gives the room's box (walls,
   floor, ceiling), a pixel's ray from its own center meets a known plane, and that point seen from
   a neighbor's center moves by the parallax: the registration then solves o with the rotations.
  --------------------------------------------------------------------------------*/
enum {
   inspectBoxMax = layoutMaxVerts + 2
};

static float inspectBoxN[inspectBoxMax][3], // outward normals: n X = D on the plane, D > 0 (the pivot inside)
             inspectBoxD[inspectBoxMax];
static int   inspectBoxCount = 0;
static bool  inspectLeverFit = false,      // the registration solves the lever arm too
             inspectShiftFit = false,      // and each frame's own shift
             inspectBodyFit = false;       // and the body's radius and the phone's height per band (below)
static bool  inspectFocalFit = false;      // and the lens's focal scale (every frame alike: inspectFocalScale follows)

/* The operator turns the body about its own vertical, the phone held out in front (user, 2026-09-30:
   the furniture on L, the window on O, the handles on N, the bed on S - the freedoms not yet orchestrated).
   Per band b the camera center sits r_b ahead of the pivot along the frame's horizontal forward and y_b
   above it: c = r_b f_h + y_b up, plus the frame's own sway. Tilting to the ceiling or to the floor moves
   the phone about the wrist, not about the body: each band has its own pair, not one arm turned with the
   camera (c = R o coupled the tilt into the height and came out ~0). */
enum {
   inspectBodyBands = 3
};

static float inspectBody[inspectBodyBands][2]; // r_b, y_b (m), carried into every frame's shift

//--------------------------------------------------------------------------------
// A frame's camera center in the world: the lever turned by its rotation (TMat4 layout, 3x3), plus its own shift (NULL: none)
static TVec3 inspectCenter(const float *rot, const float *shift)
{
   TVec3 c = { rot[0]*inspectLever.x + rot[3]*inspectLever.y + rot[6]*inspectLever.z,
               rot[1]*inspectLever.x + rot[4]*inspectLever.y + rot[7]*inspectLever.z,
               rot[2]*inspectLever.x + rot[5]*inspectLever.y + rot[8]*inspectLever.z };

   if (shift)
   {
      c.x += shift[0];
      c.y += shift[1];
      c.z += shift[2];
   }
   return c;
}

//--------------------------------------------------------------------------------
// A frame's forward laid flat (unit, world): the body turns about the vertical, the phone held along it
static TVec3 inspectHorizontalForward(const float *rot)
{
   TVec3 f = { -rot[6], 0.f, -rot[8] };
   float l = sqrtf(f.x*f.x + f.z*f.z);

   if (l > 1e-6f)
   {
      f.x /= l;
      f.z /= l;
   }
   return f;
}

//--------------------------------------------------------------------------------
// The body pair a frame's band uses
static int inspectBodyBandOf(const TPanoFrame &p)
{
   int b = (int)inspectBand[p.index];

   return b >= 0 && b < inspectBodyBands ? b : 0;
}

//--------------------------------------------------------------------------------
// The ray c + t d (d unit) to the room's box: the nearest plane ahead; false when none
static bool inspectBoxHit(const TVec3 &c, const TVec3 &d, float &t, int &plane)
{
   t = 1e9f;
   plane = -1;
   for (int k = 0; k < inspectBoxCount; k++)
   {
      const float *n = inspectBoxN[k];
      float        nd = n[0]*d.x + n[1]*d.y + n[2]*d.z,
                   tk;

      if (nd < 1e-4f)
         continue;
      tk = (inspectBoxD[k] - (n[0]*c.x + n[1]*c.y + n[2]*c.z))/nd;
      if (tk > 0.f && tk < t)
      {
         t = tk;
         plane = k;
      }
   }
   return plane >= 0;
}

//--------------------------------------------------------------------------------
// Bilinear sample of a level image; false outside (a pixel of margin for the gradient)
static bool inspectPanoAt(const float *img, int w, int h, float x, float y, float &val, float &gx, float &gy)
{
   if (x < 1.f || y < 1.f || x >= (float)(w - 2) || y >= (float)(h - 2))
      return false;

   int   ix = (int)x,
         iy = (int)y;
   float fx = x - (float)ix,
         fy = y - (float)iy;
   const float *p = img + (size_t)iy*w + ix;
   float a = p[0],
         b = p[1],
         c = p[w],
         d = p[w + 1];

   val = a*(1.f - fx)*(1.f - fy) + b*fx*(1.f - fy) + c*(1.f - fx)*fy + d*fx*fy;
   gx = (b - a)*(1.f - fy) + (d - c)*fy;
   gy = (c - a)*(1.f - fx) + (d - b)*fx;
   return true;
}

//--------------------------------------------------------------------------------
// Rotation matrix of a small world rotation (Rodrigues), row-major 3x3
static void inspectPanoExp(const float *w, float *e)
{
   float t = sqrtf(w[0]*w[0] + w[1]*w[1] + w[2]*w[2]),
         k[3] = { 0.f, 0.f, 0.f },
         s = sinf(t),
         c = 1.f - cosf(t);

   if (t > 1e-9f)
   {
      k[0] = w[0]/t;
      k[1] = w[1]/t;
      k[2] = w[2]/t;
   }
   e[0] = 1.f - c*(k[1]*k[1] + k[2]*k[2]);
   e[1] = -s*k[2] + c*k[0]*k[1];
   e[2] = s*k[1] + c*k[0]*k[2];
   e[3] = s*k[2] + c*k[0]*k[1];
   e[4] = 1.f - c*(k[0]*k[0] + k[2]*k[2]);
   e[5] = -s*k[0] + c*k[1]*k[2];
   e[6] = -s*k[1] + c*k[0]*k[2];
   e[7] = s*k[0] + c*k[1]*k[2];
   e[8] = 1.f - c*(k[0]*k[0] + k[1]*k[1]);
}

//--------------------------------------------------------------------------------
// Angle between two rotations (TMat4 layout 3x3), degrees
static float inspectPanoAngleDeg(const float *a, const float *b)
{
   float tr = a[0]*b[0] + a[1]*b[1] + a[2]*b[2] + a[3]*b[3] + a[4]*b[4] + a[5]*b[5] + a[6]*b[6] + a[7]*b[7] + a[8]*b[8],
         c = fmaxf(-1.f, fminf(1.f, 0.5f*(tr - 1.f)));

   return acosf(c)*57.2957795f;
}

//--------------------------------------------------------------------------------
// Dense symmetric solve by Gaussian elimination with partial pivoting (n small); false when singular
static bool inspectPanoSolve(float *a, float *b, int n)
{
   for (int c = 0; c < n; c++)
   {
      int best = c;

      for (int r = c + 1; r < n; r++)
         if (fabsf(a[(size_t)r*n + c]) > fabsf(a[(size_t)best*n + c]))
            best = r;
      if (fabsf(a[(size_t)best*n + c]) < 1e-12f)
         return false;
      if (best != c)
      {
         for (int k = 0; k < n; k++)
         {
            float t = a[(size_t)c*n + k];

            a[(size_t)c*n + k] = a[(size_t)best*n + k];
            a[(size_t)best*n + k] = t;
         }

         float t = b[c];

         b[c] = b[best];
         b[best] = t;
      }
      for (int r = c + 1; r < n; r++)
      {
         float f = a[(size_t)r*n + c]/a[(size_t)c*n + c];

         if (f == 0.f)
            continue;
         for (int k = c; k < n; k++)
            a[(size_t)r*n + k] -= f*a[(size_t)c*n + k];
         b[r] -= f*b[c];
      }
   }
   for (int r = n - 1; r >= 0; r--)
   {
      float s = b[r];

      for (int k = r + 1; k < n; k++)
         s -= a[(size_t)r*n + k]*b[k];
      b[r] = s/a[(size_t)r*n + r];
   }
   return true;
}

//--------------------------------------------------------------------------------
// One frame's pyramid from its BGR picture: gray, box-shrunk, high-passed (minus a 5x5 box) and normalized
static void inspectPanoPyramid(LPCBYTE bgr, DWORD width, DWORD height, TPanoFrame &p)
{
   int w0 = (int)width/inspectPanoShrink,
       h0 = (int)height/inspectPanoShrink;
   TAlloc<float> gray((size_t)w0*h0);

   for (int y = 0; y < h0; y++)
      for (int x = 0; x < w0; x++)
      {
         float s = 0.f;

         for (int dy = 0; dy < inspectPanoShrink; dy++)
         {
            LPCBYTE q = bgr + ((size_t)(y*inspectPanoShrink + dy)*width + (size_t)x*inspectPanoShrink)*3u;

            for (int dx = 0; dx < inspectPanoShrink; dx++, q += 3)
               s += 0.114f*(float)q[0] + 0.587f*(float)q[1] + 0.299f*(float)q[2];
         }
         gray[(size_t)y*w0 + x] = s/(float)(inspectPanoShrink*inspectPanoShrink);
      }
   for (int l = 0; l < inspectPanoLevels; l++)
   {
      int w = w0 >> l,
          h = h0 >> l;
      TAlloc<float> lv((size_t)w*h);

      for (int y = 0; y < h; y++)
         for (int x = 0; x < w; x++)
         {
            float s = 0.f;
            int   n = 1 << l;

            for (int dy = 0; dy < n; dy++)
               for (int dx = 0; dx < n; dx++)
                  s += gray[(size_t)(y*n + dy)*w0 + x*n + dx];
            lv[(size_t)y*w + x] = s/(float)(n*n);
         }

      float *hp = new float[(size_t)w*h];
      float sum = 0.f,
             sq = 0.f;

      for (int y = 0; y < h; y++)
         for (int x = 0; x < w; x++)
         {
            float s = 0.f;
            int   n = 0;

            for (int dy = -2; dy <= 2; dy++)
               for (int dx = -2; dx <= 2; dx++)
                  if (y + dy >= 0 && y + dy < h && x + dx >= 0 && x + dx < w)
                  {
                     s += lv[(size_t)(y + dy)*w + x + dx];
                     n++;
                  }
            hp[(size_t)y*w + x] = lv[(size_t)y*w + x] - s/(float)n;
            sum += hp[(size_t)y*w + x];
            sq += (float)hp[(size_t)y*w + x]*hp[(size_t)y*w + x];
         }

      float sd = sqrtf(fmaxf(sq/(float)(w*h) - (sum/(float)(w*h))*(sum/(float)(w*h)), 1e-6f)),
            gsum = 0.f;

      for (size_t i = 0; i < (size_t)w*h; i++)
         hp[i] /= sd;
      for (int y = 1; y + 1 < h; y++)
         for (int x = 1; x + 1 < w; x++)
            gsum += fabsf(hp[(size_t)y*w + x + 1] - hp[(size_t)y*w + x - 1]) + fabsf(hp[(size_t)(y + 1)*w + x] - hp[(size_t)(y - 1)*w + x]);
      p.hp[l] = hp;
      p.w[l] = w;
      p.h[l] = h;
      p.edge[l] = gsum/(float)((w - 2)*(h - 2));
   }
}

//--------------------------------------------------------------------------------
// One Gauss-Newton round at a level: accumulates, solves, applies; returns the RMS residual before it
static float inspectPanoRound(int level, float knee, int &residuals)
{
   const bool    shifts = inspectShiftFit && inspectBoxCount;
   const int     rots = inspectPanoCount*3,
                 lever0 = rots + (shifts ? rots : 0),
                 body0 = lever0 + (inspectLeverFit && inspectBoxCount ? 3 : 0),
                 focal0 = body0 + (inspectBodyFit && inspectBoxCount ? 2*inspectBodyBands : 0),
                 n = focal0 + (inspectFocalFit ? 1 : 0);
   TAlloc<float> H((size_t)n*n),
                  g((size_t)n);
   float        sq = 0.f,
                 diag = 0.f;
   int           count = 0;

   memset(H(), 0, sizeof(float)*(size_t)n*n);
   memset(g(), 0, sizeof(float)*(size_t)n);
   for (int i = 0; i < inspectPanoCount; i++)
   {
      const TPanoFrame &a = inspectPano[i];
      const float      *img = a.hp[level]();
      float             s = 1.f/(float)(inspectPanoShrink << level),
                        fx = a.fx*s,
                        fy = a.fy*s,
                        cx = (a.cx + 0.5f)*s - 0.5f,
                        cy = (a.cy + 0.5f)*s - 0.5f;
      int               w = a.w[level],
                        h = a.h[level];
      TVec3             ca = inspectCenter(a.rot, a.shift);

      for (int y = 2; y + 2 < h; y++)
         for (int x = 2; x + 2 < w; x++)
         {
            float v0 = img[(size_t)y*w + x],
                  ex = img[(size_t)y*w + x + 1] - img[(size_t)y*w + x - 1],
                  ey = img[(size_t)(y + 1)*w + x] - img[(size_t)(y - 1)*w + x];

            if (fabsf(ex) + fabsf(ey) < cPanoEdgeShare*a.edge[level])
               continue;

            float rx = ((float)x - cx)/fx,
                  ry = -((float)y - cy)/fy;
            TVec3 d = { a.rot[0]*rx + a.rot[3]*ry - a.rot[6], a.rot[1]*rx + a.rot[4]*ry - a.rot[7],
                        a.rot[2]*rx + a.rot[5]*ry - a.rot[8] },
                  X = {};
            float dl = sqrtf(d.x*d.x + d.y*d.y + d.z*d.z),
                  t = 0.f,
                  nd = 1.f;
            int   plane = -1;

            d.x /= dl;
            d.y /= dl;
            d.z /= dl;

            // with the room's box: the point the ray meets from this camera's own center (else at infinity)
            if (inspectBoxCount && !inspectBoxHit(ca, d, t, plane))
               continue;
            if (plane >= 0)
            {
               X.x = ca.x + t*d.x;
               X.y = ca.y + t*d.y;
               X.z = ca.z + t*d.z;
               nd = inspectBoxN[plane][0]*d.x + inspectBoxN[plane][1]*d.y + inspectBoxN[plane][2]*d.z;
            }
            for (int j = 0; j < inspectPanoCount; j++)
            {
               if (j == i)
                  continue;

               const TPanoFrame &b = inspectPano[j];

               if (b.rot[6]*a.rot[6] + b.rot[7]*a.rot[7] + b.rot[8]*a.rot[8] < cPanoNeighborCos) // forwards' cosine
                  continue;

               TVec3 cb = inspectCenter(b.rot, b.shift),
                     q = d;

               if (plane >= 0)
               {
                  q.x = X.x - cb.x;
                  q.y = X.y - cb.y;
                  q.z = X.z - cb.z;
               }

               TVec3 rc = { b.rot[0]*q.x + b.rot[1]*q.y + b.rot[2]*q.z, b.rot[3]*q.x + b.rot[4]*q.y + b.rot[5]*q.z,
                            b.rot[6]*q.x + b.rot[7]*q.y + b.rot[8]*q.z };

               if (rc.z > -1e-3f)
                  continue;

               float iz = 1.f/-rc.z,
                     u = cx + fx*rc.x*iz,
                     v = cy - fy*rc.y*iz,
                     val,
                     gu,
                     gv;

               if (!inspectPanoAt(b.hp[level](), b.w[level], b.h[level], u, v, val, gu, gv))
                  continue;

               float r = v0 - val,
                     wt = fabsf(r) <= knee ? 1.f : knee/fabsf(r),
                     J[24];
               int   at[24],
                     m = 6;
               // d(u, v)/d(rc), then back to world: gw = R_b (dval/drc)
               TVec3 grc = { gu*fx*iz, -gv*fy*iz, gu*fx*rc.x*iz*iz - gv*fy*rc.y*iz*iz },
                     gw = { b.rot[0]*grc.x + b.rot[3]*grc.y + b.rot[6]*grc.z, b.rot[1]*grc.x + b.rot[4]*grc.y + b.rot[7]*grc.z,
                            b.rot[2]*grc.x + b.rot[5]*grc.y + b.rot[8]*grc.z };

               for (int k = 0; k < 3; k++)
               {
                  at[k] = 3*i + k;
                  at[3 + k] = 3*j + k;
               }
               if (plane < 0) // at infinity: turning a moves the direction, turning b the other way
               {
                  J[0] = gw.y*d.z - gw.z*d.y;
                  J[1] = gw.z*d.x - gw.x*d.z;
                  J[2] = gw.x*d.y - gw.y*d.x;
                  for (int k = 0; k < 3; k++)
                     J[3 + k] = -J[k];
               }
               else
               {
                  /* turning a swings the point about the pivot and slides it along the ray back onto its plane
                     (P y = y - d (n y)/(n d)); turning b turns its view of the point; a shift of a's center slides the
                     point on its plane, one of b's moves its view the other way; the lever moves both centers */
                  const float *pn = inspectBoxN[plane];
                  float        gd = (gw.x*d.x + gw.y*d.y + gw.z*d.z)/nd;
                  TVec3        pg = { gw.x - pn[0]*gd, gw.y - pn[1]*gd, gw.z - pn[2]*gd };

                  J[0] = pg.y*X.z - pg.z*X.y;
                  J[1] = pg.z*X.x - pg.x*X.z;
                  J[2] = pg.x*X.y - pg.y*X.x;
                  J[3] = -(gw.y*X.z - gw.z*X.y);
                  J[4] = -(gw.z*X.x - gw.x*X.z);
                  J[5] = -(gw.x*X.y - gw.y*X.x);
                  if (shifts)
                  {
                     const float pa[3] = { pg.x, pg.y, pg.z },
                                 pb[3] = { gw.x, gw.y, gw.z };

                     for (int k = 0; k < 3; k++)
                     {
                        J[m + k] = -pa[k];
                        at[m + k] = rots + 3*i + k;
                        J[m + 3 + k] = pb[k];
                        at[m + 3 + k] = rots + 3*j + k;
                     }
                     m += 6;
                  }
                  if (body0 > lever0)
                  {
                     for (int k = 0; k < 3; k++)
                     {
                        J[m + k] = -(pg.x*a.rot[3*k] + pg.y*a.rot[3*k + 1] + pg.z*a.rot[3*k + 2])
                                   + gw.x*b.rot[3*k] + gw.y*b.rot[3*k + 1] + gw.z*b.rot[3*k + 2];
                        at[m + k] = lever0 + k;
                     }
                     m += 3;
                  }
                  if (focal0 > body0) // a's center moves the point on its plane (-pg), b's its view (+gw): radius along f_h, height along up
                  {
                     int   ba = inspectBodyBandOf(a),
                           bb = inspectBodyBandOf(b);
                     TVec3 fa = inspectHorizontalForward(a.rot),
                           fb = inspectHorizontalForward(b.rot);

                     J[m] = -(pg.x*fa.x + pg.y*fa.y + pg.z*fa.z);
                     at[m] = body0 + 2*ba;
                     J[m + 1] = -pg.y;
                     at[m + 1] = body0 + 2*ba + 1;
                     J[m + 2] = gw.x*fb.x + gw.y*fb.y + gw.z*fb.z;
                     at[m + 2] = body0 + 2*bb;
                     J[m + 3] = gw.y;
                     at[m + 3] = body0 + 2*bb + 1;
                     m += 4;
                  }
               }
               if (n > focal0)
               {
                  /* the focal scale s (fx, fy x (1 + s), every frame): a's ray leans away from its center (d d_u/ds =
                     -(rx x_cam + ry y_cam), onto its plane when it has one), b's projection spreads from its center
                     (du/ds = u - cx, dv/ds = v - cy) */
                  TVec3 dd = { -(rx*a.rot[0] + ry*a.rot[3]), -(rx*a.rot[1] + ry*a.rot[4]), -(rx*a.rot[2] + ry*a.rot[5]) },
                        pv = gw;
                  float along = 1.f/dl;

                  if (plane >= 0)
                  {
                     const float *pn = inspectBoxN[plane];
                     float        gd = (gw.x*d.x + gw.y*d.y + gw.z*d.z)/nd;

                     pv.x = gw.x - pn[0]*gd;
                     pv.y = gw.y - pn[1]*gd;
                     pv.z = gw.z - pn[2]*gd;
                     along = t/dl;
                  }
                  J[m] = -(along*(pv.x*dd.x + pv.y*dd.y + pv.z*dd.z) + gu*(u - cx) + gv*(v - cy));
                  at[m] = focal0;
                  m++;
               }
               for (int p = 0; p < m; p++)
               {
                  for (int k = 0; k < m; k++)
                     H[(size_t)at[p]*n + at[k]] += wt*J[p]*J[k];
                  g[at[p]] += wt*J[p]*r;
               }
               sq += r*r;
               count++;
            }
         }
   }

   // damping per block (turns, shifts, lever, body, focal): the turns' holds the whole set's free rotation near the gyroscope's
   const int   lo[5] = { 0, rots, lever0, body0, focal0 },
               hi[5] = { rots, lever0, body0, focal0, n };
   // a coarse level's few, blurred residuals hold less: its turns lean harder on the gyroscope (x4 per level)
   const float coarse = (float)(1 << 2*level),
               pull[5] = { cPanoPrior*coarse, cPanoShiftPrior, cPanoPrior, cPanoPrior, cPanoPrior };

   for (int blk = 0; blk < 5; blk++)
   {
      diag = 0.f;
      for (int k = lo[blk]; k < hi[blk]; k++)
         diag += H[(size_t)k*n + k];
      diag = hi[blk] > lo[blk] ? diag/(float)(hi[blk] - lo[blk]) : 1.f;
      for (int k = lo[blk]; k < hi[blk]; k++)
         H[(size_t)k*n + k] += pull[blk]*diag + 1e-12f;
   }
   for (int k = 0; k < n; k++)
      g[k] = -g[k];
   if (count && inspectPanoSolve(H(), g(), n))
   {
      if (focal0 > body0) // the body's pair moves every frame of its band, along the forward it had at this round
      {
         for (int i = 0; i < inspectPanoCount; i++)
         {
            TPanoFrame &a = inspectPano[i];
            int         b = inspectBodyBandOf(a);
            TVec3       fh = inspectHorizontalForward(a.rot);
            float       dr = g[body0 + 2*b],
                        dy = g[body0 + 2*b + 1];

            a.shift[0] += dr*fh.x;
            a.shift[1] += dr*fh.y + dy;
            a.shift[2] += dr*fh.z;
         }
         for (int b = 0; b < inspectBodyBands; b++)
         {
            inspectBody[b][0] += g[body0 + 2*b];
            inspectBody[b][1] += g[body0 + 2*b + 1];
         }
      }
      for (int i = 0; i < inspectPanoCount; i++)
      {
         float w[3] = { g[3*i], g[3*i + 1], g[3*i + 2] },
               e[9],
               old[9];
         TPanoFrame &a = inspectPano[i];

         inspectPanoExp(w, e);
         memcpy(old, a.rot, sizeof(old));
         for (int c = 0; c < 3; c++) // each camera axis (a column) turned in the world
         {
            a.rot[3*c] = e[0]*old[3*c] + e[1]*old[3*c + 1] + e[2]*old[3*c + 2];
            a.rot[3*c + 1] = e[3]*old[3*c] + e[4]*old[3*c + 1] + e[5]*old[3*c + 2];
            a.rot[3*c + 2] = e[6]*old[3*c] + e[7]*old[3*c + 1] + e[8]*old[3*c + 2];
         }
      }
      if (shifts)
         for (int i = 0; i < inspectPanoCount; i++)
            for (int k = 0; k < 3; k++)
               inspectPano[i].shift[k] += g[rots + 3*i + k];
      if (body0 > lever0)
      {
         inspectLever.x += g[lever0];
         inspectLever.y += g[lever0 + 1];
         inspectLever.z += g[lever0 + 2];
      }
      if (n > focal0)
      {
         float f = 1.f + fmaxf(-0.05f, fminf(0.05f, g[focal0])); // a step never beyond 5%

         for (int i = 0; i < inspectPanoCount; i++)
         {
            inspectPano[i].fx *= f;
            inspectPano[i].fy *= f;
         }
         inspectFocalScale *= f;
      }
   }
   residuals = count;
   return count ? sqrtf(sq/(float)count) : 0.f;
}

/*--------------------------------------------------------------------------------
   The plumb of every frame from its own picture (user, 2026-09-28: "the plumb can be set by the
   vanishing points seen in the image"; a frontal rectification turns boxes into rectangles only
   with the camera's true tilt). A frame's vertical vanishing point is its own vertical in camera
   axes; the rotation is turned the least that carries it onto the world's up. Frames without one
   take the camera-to-sensor misalignment the others show: in camera axes the measured vertical v
   and the sensor's up g differ by a small fixed turn, v - g = W x g, W by least squares. A measured
   vertical farther than cPlumbMaxDeg from the sensor's is taken for a wrong detection.
  --------------------------------------------------------------------------------*/
static const float cPlumbMaxDeg = 8.f,   // a measured vertical farther from the sensor's is a wrong detection
                   cPlumbJumpDeg = 1.5f, // an own vertical farther from its window's mean is doubtful
                   cPlumbOwnWeight = 8.f; // the frame's own vanishing-point vertical against each neighbor (user: 4x or 8x)

enum {
   cPlumbWindow = 2 // neighbors on each side in capture order (user: five frames)
};

//--------------------------------------------------------------------------------
static void inspectPanoPlumb(DWORD room)
{
   float H[9] = {},
         g[3] = {},
         W[3] = { 0.f, 0.f, 0.f },
         sum = 0.f,
         worst = 0.f,
         turn[inspectPanoMax][2],  // each frame's plumb turn about world x and z (radians)
         fixed[inspectPanoMax][2]; // the same after the regularity check
   bool  ownV[inspectPanoMax];      // the frame has a vertical vanishing point of its own
   int   measured = 0;

   if (!inspectFrameVr || !inspectPlumbOn)
      return;
   for (int pass = 0; pass < 2; pass++)
   {
      if (pass == 1 && measured >= 5) // the misalignment from every measured frame
      {
         for (int k = 0; k < 3; k++)
            H[4*k] += 1e-4f;
         memcpy(W, g, sizeof(W));
         if (!inspectPanoSolve(H, W, 3))
            memset(W, 0, sizeof(W));
      }
      for (int i = 0; i < inspectPanoCount; i++)
      {
         TPanoFrame          &p = inspectPano[i];
         const TVanishResult &vr = inspectFrameVr[p.index];
         TVec3                up = { p.rot[1], p.rot[4], p.rot[7] }, // the world's up in camera axes
                              v = vr.dirCam[0];
         float                vl = sqrtf(v.x*v.x + v.y*v.y + v.z*v.z);
         bool                 own = (vr.flags & vfVertical) && vl > 0.f;

         if (own)
         {
            v.x /= vl;
            v.y /= vl;
            v.z /= vl;
            if (v.x*up.x + v.y*up.y + v.z*up.z < 0.f)
            {
               v.x = -v.x;
               v.y = -v.y;
               v.z = -v.z;
            }
            own = acosf(fminf(1.f, v.x*up.x + v.y*up.y + v.z*up.z))*57.2957795f <= cPlumbMaxDeg;
         }
         if (pass == 0) // the misalignment: v - g = W x g = -[g]x W
         {
            if (!own)
               continue;

            float A[9] = { 0.f, up.z, -up.y, -up.z, 0.f, up.x, up.y, -up.x, 0.f },
                  d[3] = { v.x - up.x, v.y - up.y, v.z - up.z };

            for (int r = 0; r < 3; r++)
            {
               for (int c = 0; c < 3; c++)
                  for (int k = 0; k < 3; k++)
                     H[3*r + c] += A[3*k + r]*A[3*k + c];
               for (int k = 0; k < 3; k++)
                  g[r] += A[3*k + r]*d[k];
            }
            measured++;
            continue;
         }
         if (!own) // no vertical of its own: the sensor's up turned by the misalignment
         {
            v.x = up.x + (W[1]*up.z - W[2]*up.y);
            v.y = up.y + (W[2]*up.x - W[0]*up.z);
            v.z = up.z + (W[0]*up.y - W[1]*up.x);
            vl = sqrtf(v.x*v.x + v.y*v.y + v.z*v.z);
            v.x /= vl;
            v.y /= vl;
            v.z /= vl;
         }

         // the vertical in the world; the least turn carrying it onto up: about (vw x up), by their angle
         TVec3 vw = { p.rot[0]*v.x + p.rot[3]*v.y + p.rot[6]*v.z, p.rot[1]*v.x + p.rot[4]*v.y + p.rot[7]*v.z,
                      p.rot[2]*v.x + p.rot[5]*v.y + p.rot[8]*v.z };
         float ax = -vw.z,
               az = vw.x,
               al = sqrtf(ax*ax + az*az),
               ang = acosf(fminf(1.f, vw.y));

         turn[i][0] = al > 1e-9f ? ax/al*ang : 0.f;
         turn[i][1] = al > 1e-9f ? az/al*ang : 0.f;
         ownV[i] = own;
      }
   }

   /* the operator keeps a regular hold (user, 2026-09-28): a frame's plumb is the weighted mean of its window - the
      two frames before and the two after (capture order) once each, its own vanishing-point vertical cPlumbOwnWeight
      times (a frame without one weighs once, by the misalignment). A frame whose own turn lies far from that mean
      had a doubtful detection */
   int   jumps = 0;
   float spread = 0.f;

   for (int i = 0; i < inspectPanoCount; i++)
   {
      float sx = 0.f,
            sz = 0.f,
            sw = 0.f;

      for (int j = i - cPlumbWindow; j <= i + cPlumbWindow; j++)
         if (j >= 0 && j < inspectPanoCount)
         {
            float wt = j == i && ownV[i] ? cPlumbOwnWeight : 1.f;

            sx += wt*turn[j][0];
            sz += wt*turn[j][1];
            sw += wt;
         }
      fixed[i][0] = sx/sw;
      fixed[i][1] = sz/sw;
      if (!ownV[i])
         continue;

      float off = sqrtf((turn[i][0] - fixed[i][0])*(turn[i][0] - fixed[i][0])
                        + (turn[i][1] - fixed[i][1])*(turn[i][1] - fixed[i][1]))*57.2957795f;

      spread += off;
      jumps += off > cPlumbJumpDeg ? 1 : 0;
   }
   for (int i = 0; i < inspectPanoCount; i++)
   {
      TPanoFrame &p = inspectPano[i];
      float       w[3] = { fixed[i][0], 0.f, fixed[i][1] },
                  e[9],
                  old[9],
                  deg = sqrtf(w[0]*w[0] + w[2]*w[2])*57.2957795f;

      inspectPanoExp(w, e);
      memcpy(old, p.rot, sizeof(old));
      for (int c = 0; c < 3; c++)
      {
         p.rot[3*c] = e[0]*old[3*c] + e[1]*old[3*c + 1] + e[2]*old[3*c + 2];
         p.rot[3*c + 1] = e[3]*old[3*c] + e[4]*old[3*c + 1] + e[5]*old[3*c + 2];
         p.rot[3*c + 2] = e[6]*old[3*c] + e[7]*old[3*c + 1] + e[8]*old[3*c + 2];
      }
      sum += deg;
      worst = fmaxf(worst, deg);
   }
   printf("  panorama room %lu: plumb from the frames' own vertical vanishing points: %d of %d frames measured,"
          " camera-to-sensor misalignment %.2f %.2f %.2f degrees; own verticals off their window's mean %.2f on average,"
          " %d beyond %.1f;"
          " plumb turn mean %.2f, worst %.2f degrees\n", (unsigned long)room, measured, inspectPanoCount, W[0]*57.2957795f,
          W[1]*57.2957795f, W[2]*57.2957795f, measured ? spread/(float)measured : 0.f, jumps, cPlumbJumpDeg,
          inspectPanoCount ? sum/(float)inspectPanoCount : 0.f, worst);
}

/*--------------------------------------------------------------------------------
   The photometric Gauss-Newton rounds from pyramid level top down to the finest; the rotations in
   inspectPanoRot follow; returns the finest level's last RMS
  --------------------------------------------------------------------------------*/
static float inspectPanoGN(DWORD room, int top)
{
   float fine = 0.f;

   if (inspectVpSolveOn) // the lines hold the rotations: the photometric fit would drag them off (the parallax)
   {
      for (int i = 0; i < inspectPanoCount; i++)
         if (inspectPanoHas[inspectPano[i].index])
            memcpy(inspectPanoRot[inspectPano[i].index], inspectPano[i].rot, sizeof(inspectPano[i].rot));
      return 0.f;
   }

   for (int l = top; l >= 0; l--)
   {
      float knee = 1e9f,
            first = 0.f,
            last = 0.f;
      int   count = 0;

      inspectFocalFit = inspectFocalOn && l <= cPanoFocalLevel; // a 1% focal error moves the frame edges ~1 px from 1/32 on

      for (int it = 0; it < inspectPanoIters; it++)
      {
         float rms = inspectPanoRound(l, knee, count);

         if (it == 0)
            first = rms;
         last = rms;
         knee = cPanoHuber*rms;
      }
      float mean = 0.f,
            worst = 0.f;

      for (int i = 0; i < inspectPanoCount; i++)
      {
         float a = inspectPanoAngleDeg(inspectPano[i].rot, inspectPano[i].gyro);

         mean += a;
         worst = fmaxf(worst, a);
      }
      printf("  panorama room %lu: level 1/%d, %d residuals, RMS %.3f -> %.3f; focal scale %.4f; correction to the gyroscope mean %.2f,"
             " worst %.2f degrees\n", (unsigned long)room, inspectPanoShrink << l, count, first, last, inspectFocalScale,
             inspectPanoCount ? mean/(float)inspectPanoCount : 0.f, worst);
      fine = last;
      inspectFocalFit = false;
   }
   for (int i = 0; i < inspectPanoCount; i++)
      if (inspectPanoHas[inspectPano[i].index])
         memcpy(inspectPanoRot[inspectPano[i].index], inspectPano[i].rot, sizeof(inspectPano[i].rot));
   return fine;
}

/*--------------------------------------------------------------------------------
   The gyroscope's attitude against the picture's time (user, 2026-09-30: every freedom counted - the
   pose-to-image delay among them). The session logs the attitude at the sensor's rate (rtPose); a
   frame's rotation at its picture's stamp plus dt comes from the two samples around it, blended and
   made orthonormal again. --dt-scan measures the registration's residual (level 1/32, the gyroscope's
   rotations as they are) at trial delays; --pose-dt S takes the attitude S seconds off every stamp.
  --------------------------------------------------------------------------------*/
enum {
   inspectPoseMax = 131072
};

static QWORD inspectPoseNs[inspectPoseMax];
static float inspectPoseRot[inspectPoseMax][9]; // camera to world, TMat4 layout 3x3
static int   inspectPoseCount = 0;

//--------------------------------------------------------------------------------
// The attitude at stamp + dt (seconds); false outside the log
static bool inspectPoseAt(QWORD stamp, float dt, float *rot)
{
   LONG  shiftUs = (LONG)(dt*1e6f);
   QWORD t = shiftUs >= 0 ? stamp + (QWORD)shiftUs*1000u : stamp - (QWORD)(-shiftUs)*1000u;
   int   lo = 0,
         hi = inspectPoseCount - 1;

   if (inspectPoseCount < 2 || t < inspectPoseNs[0] || t > inspectPoseNs[hi])
      return false;
   while (hi - lo > 1)
   {
      int m = (lo + hi)/2;

      if (inspectPoseNs[m] <= t)
         lo = m;
      else
         hi = m;
   }

   float span = (float)(inspectPoseNs[hi] - inspectPoseNs[lo]),
         u = span > 0.f ? (float)(t - inspectPoseNs[lo])/span : 0.f;

   for (int k = 0; k < 9; k++)
      rot[k] = inspectPoseRot[lo][k]*(1.f - u) + inspectPoseRot[hi][k]*u;

   // columns orthonormal again (Gram-Schmidt: x, then y off x, z = x cross y)
   float *x = rot,
         *y = rot + 3,
         *z = rot + 6,
         l = sqrtf(x[0]*x[0] + x[1]*x[1] + x[2]*x[2]),
         d = 0.f;

   for (int k = 0; k < 3; k++)
      x[k] /= l;
   d = x[0]*y[0] + x[1]*y[1] + x[2]*y[2];
   for (int k = 0; k < 3; k++)
      y[k] -= d*x[k];
   l = sqrtf(y[0]*y[0] + y[1]*y[1] + y[2]*y[2]);
   for (int k = 0; k < 3; k++)
      y[k] /= l;
   z[0] = x[1]*y[2] - x[2]*y[1];
   z[1] = x[2]*y[0] - x[0]*y[2];
   z[2] = x[0]*y[1] - x[1]*y[0];
   return true;
}

//--------------------------------------------------------------------------------
static void inspectPanoDtScan(DWORD room)
{
   TAlloc<float> keep((size_t)inspectPanoCount*9u);
   float         bestDt = 0.f,
                 bestRms = 1e9f;

   for (int i = 0; i < inspectPanoCount; i++)
      memcpy(&keep[(size_t)i*9u], inspectPano[i].rot, sizeof(inspectPano[i].rot));
   printf("  panorama room %lu: %d attitude samples logged\n", (unsigned long)room, inspectPoseCount);
   for (int s = -cDtScanSteps; s <= cDtScanSteps; s++)
   {
      float dt = (float)s*cDtScanStepSec,
            rms = 0.f;
      int   count = 0,
            placed = 0;

      for (int i = 0; i < inspectPanoCount; i++)
         placed += inspectPoseAt(inspectPano[i].stamp, dt, inspectPano[i].rot) ? 1 : 0;
      rms = inspectPanoRound(2, 1e9f, count);
      printf("  panorama room %lu: pose delay %+.3f s: RMS %.4f (%d residuals, %d frames placed)\n", (unsigned long)room, dt,
             rms, count, placed);
      if (rms < bestRms)
      {
         bestRms = rms;
         bestDt = dt;
      }
      for (int i = 0; i < inspectPanoCount; i++)
         memcpy(inspectPano[i].rot, &keep[(size_t)i*9u], sizeof(inspectPano[i].rot));
   }
   printf("  panorama room %lu: pose delay scan: best %+.3f s (RMS %.4f)\n", (unsigned long)room, bestDt, bestRms);
}

/*--------------------------------------------------------------------------------
   One pair's relative turn by brute force (--pair-scan A,B; diagnosis, user 2026-09-30: the ghosts come
   from freedoms not yet orchestrated). A's rotation is held; B is turned about its own camera axes
   (x: pitch, y: heading, z: roll) on a grid, and at each trial A's strong-edge pixels of one image
   band (rows lo..hi, shares of the height) are compared at infinity (pure rotation) with B's. Two bands
   of the same pair that ask for different turns show a parallax - a translation meeting depths.
  --------------------------------------------------------------------------------*/
static void inspectPairScanBand(const TPanoFrame &a, const TPanoFrame &b, const float *rotB, int level, float lo, float hi, float span,
                                float step, float *best, float &bestRms, int &bestCount)
{
   const float *ia = a.hp[level](),
               *ib = b.hp[level]();
   float        s = 1.f/(float)(inspectPanoShrink << level),
                fx = a.fx*s,
                fy = a.fy*s,
                cx = (a.cx + 0.5f)*s - 0.5f,
                cy = (a.cy + 0.5f)*s - 0.5f;
   int          w = a.w[level],
                h = a.h[level],
                steps = (int)(span/step + 0.5f);

   bestRms = 1e9f;
   bestCount = 0;
   for (int ix = -steps; ix <= steps; ix++)
      for (int iy = -steps; iy <= steps; iy++)
         for (int iz = -steps; iz <= steps; iz++)
         {
            // B turned about its own axes: R' = R_b E (E in camera axes, columns of R_b mixed)
            float wv[3] = { (float)ix*step*0.01745329f, (float)iy*step*0.01745329f, (float)iz*step*0.01745329f },
                  e[9],
                  rb[9],
                  sq = 0.f;
            int   count = 0;

            inspectPanoExp(wv, e);
            for (int c = 0; c < 3; c++)
               for (int r = 0; r < 3; r++)
                  rb[3*c + r] = rotB[r]*e[c] + rotB[3 + r]*e[3 + c] + rotB[6 + r]*e[6 + c];
            for (int y = (int)(lo*(float)h) + 2; y + 2 < h && y < (int)(hi*(float)h); y++)
               for (int x = 2; x + 2 < w; x++)
               {
                  float v0 = ia[(size_t)y*w + x],
                        ex = ia[(size_t)y*w + x + 1] - ia[(size_t)y*w + x - 1],
                        ey = ia[(size_t)(y + 1)*w + x] - ia[(size_t)(y - 1)*w + x];

                  if (fabsf(ex) + fabsf(ey) < cPanoEdgeShare*a.edge[level])
                     continue;

                  float rx = ((float)x - cx)/fx,
                        ry = -((float)y - cy)/fy;
                  TVec3 d = { a.rot[0]*rx + a.rot[3]*ry - a.rot[6], a.rot[1]*rx + a.rot[4]*ry - a.rot[7],
                              a.rot[2]*rx + a.rot[5]*ry - a.rot[8] },
                        rc = { rb[0]*d.x + rb[1]*d.y + rb[2]*d.z, rb[3]*d.x + rb[4]*d.y + rb[5]*d.z,
                               rb[6]*d.x + rb[7]*d.y + rb[8]*d.z };

                  if (rc.z > -1e-3f)
                     continue;

                  float iz2 = 1.f/-rc.z,
                        val,
                        gu,
                        gv;

                  if (!inspectPanoAt(ib, b.w[level], b.h[level], cx + fx*rc.x*iz2, cy - fy*rc.y*iz2, val, gu, gv))
                     continue;
                  sq += (v0 - val)*(v0 - val);
                  count++;
               }
            if (count < 50)
               continue;

            float rms = sqrtf(sq/(float)count);

            if (rms < bestRms)
            {
               bestRms = rms;
               bestCount = count;
               best[0] = wv[0]*57.2957795f;
               best[1] = wv[1]*57.2957795f;
               best[2] = wv[2]*57.2957795f;
            }
         }
}

//--------------------------------------------------------------------------------
static void inspectPairScan(DWORD room)
{
   int ia = -1,
       ib = -1;

   for (int i = 0; i < inspectPanoCount; i++)
   {
      ia = inspectPano[i].index == inspectPairA ? i : ia;
      ib = inspectPano[i].index == inspectPairB ? i : ib;
   }
   if (ia < 0 || ib < 0)
   {
      printf("  pair scan room %lu: frames %d and %d not both in the center spin\n", (unsigned long)room, inspectPairA,
             inspectPairB);
      return;
   }

   const float band[4][2] = { { 0.f, 1.f }, { 0.f, 0.34f }, { 0.34f, 0.67f }, { 0.67f, 1.f } };
   LPCSTR      name[4] = { "whole", "top third", "middle third", "bottom third" };

   for (int k = 0; k < 4; k++)
   {
      float coarse[3] = { 0.f, 0.f, 0.f },
            fine[3] = { 0.f, 0.f, 0.f },
            rms = 0.f;
      int   count = 0;
      const TPanoFrame &b = inspectPano[ib];

      inspectPairScanBand(inspectPano[ia], b, b.rot, 1, band[k][0], band[k][1], 5.f, 0.5f, coarse, rms, count);

      float e[9],
            rot[9],
            wv[3] = { coarse[0]*0.01745329f, coarse[1]*0.01745329f, coarse[2]*0.01745329f };

      inspectPanoExp(wv, e);
      for (int c = 0; c < 3; c++)
         for (int r = 0; r < 3; r++)
            rot[3*c + r] = b.rot[r]*e[c] + b.rot[3 + r]*e[3 + c] + b.rot[6 + r]*e[6 + c];
      inspectPairScanBand(inspectPano[ia], b, rot, 0, band[k][0], band[k][1], 0.5f, 0.05f, fine, rms, count);
      printf("  pair scan room %lu: %d -> %d, %s of %d: B turned pitch %.2f heading %.2f roll %.2f degrees (RMS %.3f, %d px)\n",
             (unsigned long)room, inspectPairA, inspectPairB, name[k], inspectPairA, coarse[0] + fine[0], coarse[1] + fine[1],
             coarse[2] + fine[2], rms, count);
   }
}

/*--------------------------------------------------------------------------------
   Points that must meet (user, 2026-09-30: "a first calibration of points that must fit, reduced
   16x, then gradually refined down to 1x"). Each frame keeps plain gray at 1/2, 1/4 and 1/16. Corners
   (Shi-Tomasi at 1/4, spread over a grid) of A are looked for in B through the homography of the
   current rotations (H = K R_b^T R_a K^-1, pure turn): NCC of A's patch against B's patch warped by
   H, offsets searched wide at 1/16, then narrow at 1/4 and at 1/2, a parabola for the subpixel. The
   matched bearings give B's turn relative to A by RANSAC on two points and Horn's quaternion on all
   the inliers.
  --------------------------------------------------------------------------------*/
enum {
   cPointCorners = 400,  // corners per frame at most
   cPointGridX = 10,     // corner buckets across and down (spread over the picture)
   cPointGridY = 8,
   cPointHalf = 5,       // NCC patch half side (11 x 11)
   cPointCoarseWin = 24, // 1/16 search half window (px: ~7.5 degrees)
   cPointRansac = 256,   // RANSAC trials
   cPointEpiIters = 12   // the two-view model's Gauss-Newton rounds per start
};

static const float cPointMinNcc = 0.85f,         // a match's NCC at 1/2
                   cPointMinCornerShare = 0.01f, // corner response against the frame's best
                   cPointInlierDeg = 0.08f,   // a match that fits: within this angle of the two-view model
                   cPointLooseDeg = 1.5f;     // the pure turn's RANSAC: loose, the baseline's parallax still in

static const int   cPointShrink[3] = { 2, 4, 16 };

//--------------------------------------------------------------------------------
static void inspectPointGrays(LPCBYTE bgr, DWORD width, DWORD height, TPanoFrame &p)
{
   for (int l = 0; l < 3; l++)
   {
      int    s = cPointShrink[l],
             w = (int)width/s,
             h = (int)height/s;
      LPBYTE g = new BYTE[(size_t)w*h];

      for (int y = 0; y < h; y++)
         for (int x = 0; x < w; x++)
         {
            DWORD sum = 0;

            for (int dy = 0; dy < s; dy++)
            {
               LPCBYTE q = bgr + ((size_t)(y*s + dy)*width + (size_t)x*s)*3u;

               for (int dx = 0; dx < s; dx++, q += 3)
                  sum += (DWORD)q[0]*29u + (DWORD)q[1]*150u + (DWORD)q[2]*77u;
            }
            g[(size_t)y*w + x] = (BYTE)(sum/(DWORD)(256*s*s));
         }
      p.gray[l] = g;
      p.gw[l] = w;
      p.gh[l] = h;
   }
}

//--------------------------------------------------------------------------------
// Bilinear gray; false outside
static bool inspectGrayAt(LPCBYTE g, int w, int h, float x, float y, float &v)
{
   if (x < 0.f || y < 0.f || x >= (float)(w - 1) || y >= (float)(h - 1))
      return false;

   int     ix = (int)x,
           iy = (int)y;
   float   fx = x - (float)ix,
           fy = y - (float)iy;
   LPCBYTE q = g + (size_t)iy*w + ix;

   v = (float)q[0]*(1.f - fx)*(1.f - fy) + (float)q[1]*fx*(1.f - fy) + (float)q[w]*(1.f - fx)*fy + (float)q[w + 1]*fx*fy;
   return true;
}

//--------------------------------------------------------------------------------
// A frame's corners at 1/4 (x, y pairs); returns how many
static int inspectPointCorners(const TPanoFrame &p, float *xy)
{
   LPCBYTE       g = p.gray[1]();
   int           w = p.gw[1],
                 h = p.gh[1],
                 found = 0;
   TAlloc<float> resp((size_t)w*h);
   float         top = 0.f;

   memset(resp(), 0, sizeof(float)*(size_t)w*h);
   for (int y = 3; y + 3 < h; y++)
      for (int x = 3; x + 3 < w; x++)
      {
         float a = 0.f,
               b = 0.f,
               c = 0.f;

         for (int dy = -2; dy <= 2; dy++)
            for (int dx = -2; dx <= 2; dx++)
            {
               LPCBYTE q = g + (size_t)(y + dy)*w + x + dx;
               float   gx = (float)q[1] - (float)q[-1],
                       gy = (float)q[w] - (float)q[-w];

               a += gx*gx;
               b += gx*gy;
               c += gy*gy;
            }

         // the smaller eigenvalue of the structure tensor
         float m = 0.5f*(a + c) - sqrtf(0.25f*(a - c)*(a - c) + b*b);

         resp[(size_t)y*w + x] = m;
         top = fmaxf(top, m);
      }

   // per bucket: the strongest local maxima (7 x 7)
   const int perCell = cPointCorners/(cPointGridX*cPointGridY);

   for (int cy = 0; cy < cPointGridY; cy++)
      for (int cx = 0; cx < cPointGridX; cx++)
      {
         int x0 = cx*w/cPointGridX,
             x1 = (cx + 1)*w/cPointGridX,
             y0 = cy*h/cPointGridY,
             y1 = (cy + 1)*h/cPointGridY;

         for (int k = 0; k < perCell && found < cPointCorners; k++)
         {
            float best = cPointMinCornerShare*top;
            int   bx = -1,
                  by = -1;

            for (int y = y0 > 8 ? y0 : 8; y < y1 && y + 8 < h; y++)
               for (int x = x0 > 8 ? x0 : 8; x < x1 && x + 8 < w; x++)
               {
                  float r = resp[(size_t)y*w + x];

                  if (r <= best)
                     continue;

                  bool peak = true;

                  for (int dy = -3; dy <= 3 && peak; dy++)
                     for (int dx = -3; dx <= 3 && peak; dx++)
                        peak = resp[(size_t)(y + dy)*w + x + dx] <= r;
                  if (peak)
                  {
                     best = r;
                     bx = x;
                     by = y;
                  }
               }
            if (bx < 0)
               break;
            xy[2*found] = (float)bx;
            xy[2*found + 1] = (float)by;
            found++;
            for (int dy = -6; dy <= 6; dy++) // taken: its neighborhood out of the next picks
               for (int dx = -6; dx <= 6; dx++)
                  if (by + dy >= 0 && by + dy < h && bx + dx >= 0 && bx + dx < w)
                     resp[(size_t)(by + dy)*w + bx + dx] = 0.f;
         }
      }
   return found;
}

//--------------------------------------------------------------------------------
// A pixel of A (level l coords) carried to B's level l coords by the pure turn; false behind B
static bool inspectPointMap(const TPanoFrame &a, const TPanoFrame &b, const float *rotB, int l, float x, float y,
                            float &u, float &v)
{
   float s = 1.f/(float)cPointShrink[l],
         fx = a.fx*s,
         fy = a.fy*s,
         cx = (a.cx + 0.5f)*s - 0.5f,
         cy = (a.cy + 0.5f)*s - 0.5f,
         rx = (x - cx)/fx,
         ry = -(y - cy)/fy;
   TVec3 d = { a.rot[0]*rx + a.rot[3]*ry - a.rot[6], a.rot[1]*rx + a.rot[4]*ry - a.rot[7], a.rot[2]*rx + a.rot[5]*ry - a.rot[8] },
         rc = { rotB[0]*d.x + rotB[1]*d.y + rotB[2]*d.z, rotB[3]*d.x + rotB[4]*d.y + rotB[5]*d.z,
                rotB[6]*d.x + rotB[7]*d.y + rotB[8]*d.z };

   if (rc.z > -1e-3f)
      return false;
   u = cx + fx*rc.x/-rc.z;
   v = cy - fy*rc.y/-rc.z;
   return true;
}

enum {
   cPointPatchMax = 128 // samples of a patch at most
};

// A's patch at one level and where each of its pixels falls in B (the turn's warp, offset-free)
struct TPointPatch {
   int   level,
         n;
   float va[cPointPatchMax],
         ub[cPointPatchMax],
         vb[cPointPatchMax],
         mean,
         dev;
};

//--------------------------------------------------------------------------------
// Builds A's patch at (x, y), level l; false off A, behind B or flat
static bool inspectPointPatch(const TPanoFrame &a, const TPanoFrame &b, const float *rotB, int l, float x, float y,
                              int half, int stride, TPointPatch &p)
{
   float s = 0.f,
         ss = 0.f;

   p.level = l;
   p.n = 0;
   for (int j = -half; j <= half; j += stride)
      for (int i = -half; i <= half; i += stride)
      {
         if (p.n >= cPointPatchMax)
            return false;

         float v;

         if (!inspectGrayAt(a.gray[l](), a.gw[l], a.gh[l], x + (float)i, y + (float)j, v))
            return false;
         if (!inspectPointMap(a, b, rotB, l, x + (float)i, y + (float)j, p.ub[p.n], p.vb[p.n]))
            return false;
         p.va[p.n++] = v;
         s += v;
         ss += v*v;
      }
   p.mean = s/(float)p.n;
   p.dev = ss/(float)p.n - p.mean*p.mean;
   return p.dev >= 4.f; // flat: no texture to match
}

//--------------------------------------------------------------------------------
// NCC of the patch against B offset by (du, dv) in B's level pixels; -2 when off B or flat there
static float inspectPointNcc(const TPanoFrame &b, const TPointPatch &p, float du, float dv)
{
   LPCBYTE gb = b.gray[p.level]();
   int     w = b.gw[p.level],
           h = b.gh[p.level];
   float   sb = 0.f,
           sbb = 0.f,
           sab = 0.f;

   for (int k = 0; k < p.n; k++)
   {
      float v;

      if (!inspectGrayAt(gb, w, h, p.ub[k] + du, p.vb[k] + dv, v))
         return -2.f;
      sb += v;
      sbb += v*v;
      sab += p.va[k]*v;
   }

   float mb = sb/(float)p.n,
         vb = sbb/(float)p.n - mb*mb;

   if (vb < 4.f)
      return -2.f;
   return (sab/(float)p.n - p.mean*mb)/sqrtf(p.dev*vb);
}

//--------------------------------------------------------------------------------
// Unit bearing of a level-l pixel in the camera's own axes
static TVec3 inspectPointBearing(const TPanoFrame &p, int l, float x, float y)
{
   float s = 1.f/(float)cPointShrink[l],
         rx = (x - ((p.cx + 0.5f)*s - 0.5f))/(p.fx*s),
         ry = -(y - ((p.cy + 0.5f)*s - 0.5f))/(p.fy*s),
         n = sqrtf(rx*rx + ry*ry + 1.f);
   TVec3 d = { rx/n, ry/n, -1.f/n };

   return d;
}

//--------------------------------------------------------------------------------
// Horn: the rotation M (row-major 3x3) minimizing sum w |a - M b|^2, by the quaternion of largest eigenvalue
static void inspectHorn(const TVec3 *a, const TVec3 *b, LPCBYTE use, int n, float *M)
{
   float S[9] = {},
         N[16],
         q[4] = { 1.f, 0.f, 0.f, 0.f },
         len = 0.f;

   for (int k = 0; k < n; k++)
      if (!use || use[k])
      {
         const float bv[3] = { b[k].x, b[k].y, b[k].z },
                     av[3] = { a[k].x, a[k].y, a[k].z };

         for (int r = 0; r < 3; r++)
            for (int c = 0; c < 3; c++)
               S[3*r + c] += bv[r]*av[c];
      }
   N[0] = S[0] + S[4] + S[8];
   N[1] = N[4] = S[5] - S[7];
   N[2] = N[8] = S[6] - S[2];
   N[3] = N[12] = S[1] - S[3];
   N[5] = S[0] - S[4] - S[8];
   N[6] = N[9] = S[1] + S[3];
   N[7] = N[13] = S[6] + S[2];
   N[10] = -S[0] + S[4] - S[8];
   N[11] = N[14] = S[5] + S[7];
   N[15] = -S[0] - S[4] + S[8];

   // cyclic Jacobi on the symmetric 4 x 4 (power iteration stalls when two eigenvalues lie close): V holds the vectors
   float V[16] = { 1.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 1.f };

   for (int sweep = 0; sweep < 12; sweep++)
      for (int p = 0; p < 3; p++)
         for (int r = p + 1; r < 4; r++)
         {
            float apr = N[4*p + r];

            if (fabsf(apr) < 1e-12f)
               continue;

            float th = 0.5f*(N[4*r + r] - N[4*p + p])/apr,
                  t = (th >= 0.f ? 1.f : -1.f)/(fabsf(th) + sqrtf(th*th + 1.f)),
                  c = 1.f/sqrtf(t*t + 1.f),
                  s = t*c;

            for (int k = 0; k < 4; k++) // columns p, r
            {
               float kp = N[4*k + p],
                     kr = N[4*k + r];

               N[4*k + p] = c*kp - s*kr;
               N[4*k + r] = s*kp + c*kr;
            }
            for (int k = 0; k < 4; k++) // rows p, r
            {
               float pk = N[4*p + k],
                     rk = N[4*r + k];

               N[4*p + k] = c*pk - s*rk;
               N[4*r + k] = s*pk + c*rk;
            }
            for (int k = 0; k < 4; k++)
            {
               float kp = V[4*k + p],
                     kr = V[4*k + r];

               V[4*k + p] = c*kp - s*kr;
               V[4*k + r] = s*kp + c*kr;
            }
         }

   int top = 0;

   for (int k = 1; k < 4; k++)
      top = N[5*k] > N[5*top] ? k : top;
   for (int k = 0; k < 4; k++)
      q[k] = V[4*k + top];
   len = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
   for (int k = 0; k < 4; k++)
      q[k] /= len > 0.f ? len : 1.f;

   float w = q[0],
         x = q[1],
         y = q[2],
         z = q[3];

   M[0] = 1.f - 2.f*(y*y + z*z);
   M[1] = 2.f*(x*y - w*z);
   M[2] = 2.f*(x*z + w*y);
   M[3] = 2.f*(x*y + w*z);
   M[4] = 1.f - 2.f*(x*x + z*z);
   M[5] = 2.f*(y*z - w*x);
   M[6] = 2.f*(x*z - w*y);
   M[7] = 2.f*(y*z + w*x);
   M[8] = 1.f - 2.f*(x*x + y*y);
}

//--------------------------------------------------------------------------------
// Angle between a and M b, degrees
static float inspectPointMisDeg(const TVec3 &a, const float *M, const TVec3 &b)
{
   float x = M[0]*b.x + M[1]*b.y + M[2]*b.z,
         y = M[3]*b.x + M[4]*b.y + M[5]*b.z,
         z = M[6]*b.x + M[7]*b.y + M[8]*b.z,
         c = fmaxf(-1.f, fminf(1.f, a.x*x + a.y*y + a.z*z));

   return acosf(c)*57.2957795f;
}

static float inspectQuantile(float *v, int n, float q);

/*--------------------------------------------------------------------------------
   The two-view model (user, 2026-09-30: the ghosts are freedoms not orchestrated - the body sways, the
   furniture stands off the walls). B's center lies along c (unit, A's axes) from A's; a match is right
   when a, M b and c are coplanar: its error is the angle of a off the plane spanned by c and M b. The
   turn M (3) and c (2, on the sphere) are solved by Gauss-Newton (numeric Jacobian, Cauchy weights at
   the median), from the pure turn and from six starting baselines; the best is kept. A pure turn is the
   limit c -> anything with every point far: the error then is the pure turn's own.
  --------------------------------------------------------------------------------*/

//--------------------------------------------------------------------------------
// Signed angle of a off the epipolar plane of (c, M b), degrees; the pure turn's angle when c is zero
static float inspectPointEpiDeg(const TVec3 &a, const float *M, const float *c, const TVec3 &b)
{
   float x = M[0]*b.x + M[1]*b.y + M[2]*b.z,
         y = M[3]*b.x + M[4]*b.y + M[5]*b.z,
         z = M[6]*b.x + M[7]*b.y + M[8]*b.z,
         nx = c[1]*z - c[2]*y,
         ny = c[2]*x - c[0]*z,
         nz = c[0]*y - c[1]*x,
         nl = sqrtf(nx*nx + ny*ny + nz*nz);

   if (nl < 1e-6f)
      return inspectPointMisDeg(a, M, b);

   float s = (a.x*nx + a.y*ny + a.z*nz)/nl;

   return asinf(fmaxf(-1.f, fminf(1.f, s)))*57.2957795f;
}

//--------------------------------------------------------------------------------
// The model's errors (radians) for parameters p (turn increment 3, baseline tilt 2) about (M0, c0, u, v)
static void inspectPointEpiModel(const float *p, const float *M0, const float *c0, const float *u, const float *v,
                                 float *M, float *c)
{
   float e[9];

   inspectPanoExp(p, e); // M = exp(p) M0 (row-major both)
   for (int r = 0; r < 3; r++)
      for (int k = 0; k < 3; k++)
         M[3*r + k] = e[3*r]*M0[k] + e[3*r + 1]*M0[3 + k] + e[3*r + 2]*M0[6 + k];
   for (int k = 0; k < 3; k++)
      c[k] = c0[k] + p[3]*u[k] + p[4]*v[k];

   float l = sqrtf(c[0]*c[0] + c[1]*c[1] + c[2]*c[2]);

   for (int k = 0; k < 3; k++)
      c[k] /= l;
}

//--------------------------------------------------------------------------------
// Solves M and c on the points marked in use (M in and out); returns the median error of those, degrees
static float inspectPointEpipolar(const TVec3 *a, const TVec3 *b, LPCBYTE use, int n, float *M, float *cBest)
{
   const float starts[6][3] = { { 1.f, 0.f, 0.f }, { -1.f, 0.f, 0.f }, { 0.f, 1.f, 0.f }, { 0.f, -1.f, 0.f },
                                { 0.f, 0.f, 1.f }, { 0.f, 0.f, -1.f } };
   TAlloc<float> err((size_t)n);
   float         bestMed = 1e9f,
                 bestM[9];

   memcpy(bestM, M, sizeof(bestM));
   for (int s = 0; s < 6; s++)
   {
      float M0[9],
            c0[3] = { starts[s][0], starts[s][1], starts[s][2] };

      memcpy(M0, M, sizeof(M0));
      for (int it = 0; it < cPointEpiIters; it++)
      {
         // the sphere's tangent at c0: u off an axis c0 is not near, v = c0 x u
         float ref[3] = { 0.f, 0.f, 0.f },
               u[3],
               v[3],
               dot = 0.f,
               ul = 0.f;

         ref[fabsf(c0[0]) < 0.9f ? 0 : 1] = 1.f;
         dot = ref[0]*c0[0] + ref[1]*c0[1] + ref[2]*c0[2];
         for (int k = 0; k < 3; k++)
            u[k] = ref[k] - dot*c0[k];
         ul = sqrtf(u[0]*u[0] + u[1]*u[1] + u[2]*u[2]);
         for (int k = 0; k < 3; k++)
            u[k] /= ul;
         v[0] = c0[1]*u[2] - c0[2]*u[1];
         v[1] = c0[2]*u[0] - c0[0]*u[2];
         v[2] = c0[0]*u[1] - c0[1]*u[0];

         // residuals at p = 0 and their Cauchy scale
         float p0[5] = { 0.f, 0.f, 0.f, 0.f, 0.f },
               Mt[9],
               ct[3];
         int   used = 0;

         inspectPointEpiModel(p0, M0, c0, u, v, Mt, ct);
         for (int k = 0; k < n; k++)
            if (use[k])
               err[used++] = fabsf(inspectPointEpiDeg(a[k], Mt, ct, b[k]));

         float scale = fmaxf(0.01f, 1.5f*inspectQuantile(err(), used, 0.5f)),
               H[25] = {},
               g[5] = {};

         for (int k = 0; k < n; k++)
         {
            if (!use[k])
               continue;

            float r0 = inspectPointEpiDeg(a[k], Mt, ct, b[k]),
                  wt = 1.f/(1.f + (r0/scale)*(r0/scale)),
                  J[5];

            for (int q = 0; q < 5; q++)
            {
               float pq[5] = { 0.f, 0.f, 0.f, 0.f, 0.f },
                     Mq[9],
                     cq[3];

               pq[q] = 1e-4f;
               inspectPointEpiModel(pq, M0, c0, u, v, Mq, cq);
               J[q] = (inspectPointEpiDeg(a[k], Mq, cq, b[k]) - r0)/1e-4f;
            }
            for (int r = 0; r < 5; r++)
            {
               for (int q = 0; q < 5; q++)
                  H[5*r + q] += wt*J[r]*J[q];
               g[r] -= wt*J[r]*r0;
            }
         }
         for (int r = 0; r < 5; r++)
            H[6*r] += 1e-3f*H[6*r] + 1e-9f;
         if (!inspectPanoSolve(H, g, 5))
            break;
         for (int r = 0; r < 5; r++)
            g[r] = fmaxf(-0.2f, fminf(0.2f, g[r]));
         inspectPointEpiModel(g, M0, c0, u, v, Mt, ct);
         memcpy(M0, Mt, sizeof(M0));
         memcpy(c0, ct, sizeof(c0));
      }

      int used = 0;

      for (int k = 0; k < n; k++)
         if (use[k])
            err[used++] = fabsf(inspectPointEpiDeg(a[k], M0, c0, b[k]));

      float med = inspectQuantile(err(), used, 0.5f);

      if (med < bestMed)
      {
         bestMed = med;
         memcpy(bestM, M0, sizeof(bestM));
         memcpy(cBest, c0, sizeof(c0));
      }
   }
   memcpy(M, bestM, sizeof(bestM));
   return bestMed;
}

/*--------------------------------------------------------------------------------
   B's turn w (degrees, its own axes) from a measured M that carries B's camera vectors into A's; the
   current one is R_a^T R_b: R_a^T R_b E = M, so E = (R_a^T R_b)^T M = R_b^T R_a M; w = log E
  --------------------------------------------------------------------------------*/
static void inspectPairTurn(const TPanoFrame &a, const TPanoFrame &b, const float *M, float *wDeg)
{
   float C[9],
         E[9];

   for (int r = 0; r < 3; r++)
      for (int c = 0; c < 3; c++)
         C[3*r + c] = a.rot[3*r]*b.rot[3*c] + a.rot[3*r + 1]*b.rot[3*c + 1] + a.rot[3*r + 2]*b.rot[3*c + 2]; // (R_a^T R_b)(r,c)
   for (int r = 0; r < 3; r++)
      for (int c = 0; c < 3; c++)
         E[3*r + c] = C[r]*M[c] + C[3 + r]*M[3 + c] + C[6 + r]*M[6 + c]; // (C^T M)(r,c)

   float ang = acosf(fmaxf(-1.f, fminf(1.f, 0.5f*(E[0] + E[4] + E[8] - 1.f)))),
         k = ang > 1e-6f ? ang/(2.f*sinf(ang)) : 0.5f;

   wDeg[0] = k*(E[7] - E[5])*57.2957795f;
   wDeg[1] = k*(E[2] - E[6])*57.2957795f;
   wDeg[2] = k*(E[3] - E[1])*57.2957795f;
}

/*--------------------------------------------------------------------------------
   One pair by its points: B's turn w (degrees, about B's own axes) that best carries A's corners onto
   their matches; inliers and their RMS misfit (degrees) out; false when too few meet
  --------------------------------------------------------------------------------*/
static bool inspectPointPair(const TPanoFrame &a, const TPanoFrame &b, const float *cornersA, int corners, float *wDeg,
                             int &inliers, float &misDeg, int &matched, float &rotMedDeg, float &epiMedDeg, float *baseDir,
                             float *pureDeg, float *gyroMedDeg, TVec3 *outA = NULL, TVec3 *outB = NULL, int *outN = NULL,
                             int outCap = 0)
{
   rotMedDeg = epiMedDeg = 0.f;
   baseDir[0] = baseDir[1] = baseDir[2] = 0.f;
   pureDeg[0] = pureDeg[1] = pureDeg[2] = 0.f;
   gyroMedDeg[0] = gyroMedDeg[1] = 0.f;

   TAlloc<TVec3> ba((size_t)corners),
                 bb((size_t)corners);
   TAlloc<BYTE>  use((size_t)corners),
                 best((size_t)corners);
   int           m = 0;

   matched = 0;
   inliers = 0;
   misDeg = 0.f;

   /* the pair's own offset first, at 1/16 (the gyroscope's error is shared by the whole picture): every other
      corner votes its NCC at each offset of a wide window, two pixels a step, then one pixel around the best */
   const int   sampleStep = corners > 60 ? corners/60 : 1;
   float       pairU = 0.f,
               pairV = 0.f,
               pairTop = -1e9f;
   TPointPatch patch;

   for (int pass = 0; pass < 2; pass++)
   {
      int   win = pass == 0 ? cPointCoarseWin : 2,
            step = pass == 0 ? 2 : 1;
      float cu = pass == 0 ? 0.f : pairU,
            cv = pass == 0 ? 0.f : pairV;

      for (int j = -win; j <= win; j += step)
         for (int i = -win; i <= win; i += step)
         {
            float sum = 0.f;

            for (int k = 0; k < corners; k += sampleStep)
            {
               if (!inspectPointPatch(a, b, b.rot, 2, cornersA[2*k]*0.25f, cornersA[2*k + 1]*0.25f, 3, 1, patch))
                  continue;

               float c = inspectPointNcc(b, patch, cu + (float)i, cv + (float)j);

               sum += c > 0.f ? c : 0.f;
            }
            if (sum > pairTop)
            {
               pairTop = sum;
               pairU = cu + (float)i;
               pairV = cv + (float)j;
            }
         }
   }

   // each corner: +-3 around the pair's offset at 1/16, then +-3 at 1/4, +-2 at 1/2, and a parabola
   for (int k = 0; k < corners; k++)
   {
      float x4 = cornersA[2*k],
            y4 = cornersA[2*k + 1],
            du = pairU,
            dv = pairV,
            top = -2.f;
      bool  ok = true;

      for (int l = 2; l >= 0 && ok; l--)
      {
         float sc = l == 2 ? 0.25f : (l == 1 ? 1.f : 2.f),
               c0 = -2.f,
               bu = du,
               bv = dv;
         int   win = l == 0 ? 2 : 3;

         if (!inspectPointPatch(a, b, b.rot, l, x4*sc, y4*sc, l == 2 ? 3 : cPointHalf*(l == 0 ? 2 : 1), l == 0 ? 2 : 1, patch))
         {
            ok = false;
            break;
         }
         for (int j = -win; j <= win; j++)
            for (int i = -win; i <= win; i++)
            {
               float c = inspectPointNcc(b, patch, du + (float)i, dv + (float)j);

               if (c > c0)
               {
                  c0 = c;
                  bu = du + (float)i;
                  bv = dv + (float)j;
               }
            }
         top = c0;
         du = bu;
         dv = bv;
         if (l == 2 && top < 0.6f) // no clear match already at the coarse level
            ok = false;
         if (l > 0) // the offset into the next finer level
         {
            du *= l == 2 ? 4.f : 2.f;
            dv *= l == 2 ? 4.f : 2.f;
         }
      }
      if (!ok || top < cPointMinNcc)
         continue;

      // subpixel at 1/2: a parabola through the neighbors in each direction (the patch is the 1/2 one)
      float cl = inspectPointNcc(b, patch, du - 1.f, dv),
            cr = inspectPointNcc(b, patch, du + 1.f, dv),
            cu = inspectPointNcc(b, patch, du, dv - 1.f),
            cd = inspectPointNcc(b, patch, du, dv + 1.f),
            den = cl - 2.f*top + cr,
            x2 = x4*2.f,
            y2 = y4*2.f,
            u2,
            v2;

      if (cl > -2.f && cr > -2.f && den < -1e-6f)
         du += fmaxf(-0.5f, fminf(0.5f, 0.5f*(cl - cr)/den));
      den = cu - 2.f*top + cd;
      if (cu > -2.f && cd > -2.f && den < -1e-6f)
         dv += fmaxf(-0.5f, fminf(0.5f, 0.5f*(cu - cd)/den));
      if (!inspectPointMap(a, b, b.rot, 0, x2, y2, u2, v2))
         continue;
      ba[m] = inspectPointBearing(a, 0, x2, y2);
      bb[m] = inspectPointBearing(b, 0, u2 + du, v2 + dv);
      m++;
   }
   matched = m;
   if (m < 8)
      return false;

   // RANSAC on two points, then Horn on the inliers (bearings in each camera's own axes: a = M b)
   float    M[9],
            bestM[9];
   int      bestCount = 0;
   DWORD    seed = 12345u;

   for (int t = 0; t < cPointRansac; t++)
   {
      seed = seed*1664525u + 1013904223u;

      int p = (int)((seed >> 8)%(DWORD)m);

      seed = seed*1664525u + 1013904223u;

      int q = (int)((seed >> 8)%(DWORD)m);

      if (p == q)
         continue;
      memset(use(), 0, (size_t)m);
      use[p] = 1;
      use[q] = 1;
      inspectHorn(ba(), bb(), use(), m, M);

      int count = 0;

      for (int k = 0; k < m; k++)
         count += inspectPointMisDeg(ba[k], M, bb[k]) <= cPointLooseDeg ? 1 : 0;
      if (count > bestCount)
      {
         bestCount = count;
         memcpy(bestM, M, sizeof(M));
      }
   }
   if (bestCount < 8)
      return false;
   for (int pass = 0; pass < 2; pass++)
   {
      for (int k = 0; k < m; k++)
         best[k] = inspectPointMisDeg(ba[k], bestM, bb[k]) <= cPointLooseDeg ? 1 : 0;
      inspectHorn(ba(), bb(), best(), m, bestM);
   }

   // how well a pure turn explains the loose inliers (median), then the two-view model with the baseline
   TAlloc<float> err((size_t)m);
   int           loose = 0;

   for (int k = 0; k < m; k++)
      if (best[k])
         err[loose++] = inspectPointMisDeg(ba[k], bestM, bb[k]);
   rotMedDeg = inspectQuantile(err(), loose, 0.5f);
   if (outN) // the matched bearings that fit a loose turn: the parallax still in, for the centers' fit
   {
      *outN = 0;
      for (int k = 0; k < m && *outN < outCap; k++)
         if (best[k])
         {
            outA[*outN] = ba[k];
            outB[*outN] = bb[k];
            (*outN)++;
         }
   }
   inspectPairTurn(a, b, bestM, pureDeg); // the pure turn's own answer, before the baseline takes a share

   /* the body's circle (user, 2026-09-30: the operator turns about the trunk, the arm a little out): the gyroscope's
      turn as it is, each center ahead of the trunk's axis along its own flat forward - the baseline is the chord
      between them (its length, the radius, does not change the epipolar planes). Its median misfit on the same
      points, and the gyroscope's turn alone */
   {
      float Mg[9],
            chord[3],
            fa[3] = { -a.rot[6], 0.f, -a.rot[8] },
            fb[3] = { -b.rot[6], 0.f, -b.rot[8] },
            la = sqrtf(fa[0]*fa[0] + fa[2]*fa[2]),
            lb = sqrtf(fb[0]*fb[0] + fb[2]*fb[2]),
            w[3],
            wl = 0.f;
      int   used = 0;

      for (int r = 0; r < 3; r++) // M = R_a^T R_b (row-major)
         for (int c = 0; c < 3; c++)
            Mg[3*r + c] = a.rot[3*r]*b.rot[3*c] + a.rot[3*r + 1]*b.rot[3*c + 1] + a.rot[3*r + 2]*b.rot[3*c + 2];
      for (int k = 0; k < 3; k++)
         w[k] = fb[k]/(lb > 1e-6f ? lb : 1.f) - fa[k]/(la > 1e-6f ? la : 1.f);
      for (int k = 0; k < 3; k++) // into A's axes: R_a^T w
         chord[k] = a.rot[3*k]*w[0] + a.rot[3*k + 1]*w[1] + a.rot[3*k + 2]*w[2];
      wl = sqrtf(chord[0]*chord[0] + chord[1]*chord[1] + chord[2]*chord[2]);
      for (int k = 0; k < 3; k++)
         chord[k] /= wl > 1e-6f ? wl : 1.f;
      for (int k = 0; k < m; k++)
         if (best[k])
            err[used++] = inspectPointMisDeg(ba[k], Mg, bb[k]);
      gyroMedDeg[0] = inspectQuantile(err(), used, 0.5f);
      used = 0;
      for (int k = 0; k < m; k++)
         if (best[k])
            err[used++] = fabsf(inspectPointEpiDeg(ba[k], Mg, chord, bb[k]));
      gyroMedDeg[1] = inspectQuantile(err(), used, 0.5f);
   }
   epiMedDeg = inspectPointEpipolar(ba(), bb(), best(), m, bestM, baseDir);
   for (int k = 0; k < m; k++)
   {
      float d = fabsf(inspectPointEpiDeg(ba[k], bestM, baseDir, bb[k]));

      if (d <= cPointInlierDeg)
      {
         misDeg += d*d;
         inliers++;
      }
   }
   misDeg = inliers ? sqrtf(misDeg/(float)inliers) : 0.f;

   inspectPairTurn(a, b, bestM, wDeg);
   return true;
}

//--------------------------------------------------------------------------------
// Angle between two row-major rotations, degrees
static float inspectRowAngleDeg(const float *a, const float *b)
{
   float tr = 0.f;

   for (int k = 0; k < 9; k++)
      tr += a[k]*b[k];
   return acosf(fmaxf(-1.f, fminf(1.f, 0.5f*(tr - 1.f))))*57.2957795f;
}

//--------------------------------------------------------------------------------
// out = C E(w) (row-major), w in degrees about the second frame's own axes
static void inspectMatFromTurn(const float *C, const float *wDeg, float *out)
{
   float w[3] = { wDeg[0]*0.01745329f, wDeg[1]*0.01745329f, wDeg[2]*0.01745329f },
         e[9];

   inspectPanoExp(w, e);
   for (int r = 0; r < 3; r++)
      for (int c = 0; c < 3; c++)
         out[3*r + c] = C[3*r]*e[c] + C[3*r + 1]*e[3 + c] + C[3*r + 2]*e[6 + c];
}

/*--------------------------------------------------------------------------------
   A frame's vanishing triad in camera axes, row-major with the directions as columns: the measured
   vertical (toward the world's up by the frame's rotation), a measured room axis made square to it,
   their cross. pick NULL: axis A if measured, else B; else 0..3 = +A, -A, +B, -B (false when not measured)
  --------------------------------------------------------------------------------*/
static bool inspectVanishTriad(const TVanishResult &vr, const float *rot, const int *pick, float *T)
{
   int   which = pick ? *pick : ((vr.flags & vfAxisA) ? 0 : 2),
         dir = which < 2 ? 1 : 2;
   float sign = which%2 ? -1.f : 1.f;

   if (!(vr.flags & (dir == 1 ? vfAxisA : vfAxisB)))
      return false;

   TVec3 v = vr.dirCam[0],
         h = vr.dirCam[dir];
   float up = v.x*rot[1] + v.y*rot[4] + v.z*rot[7], // the vertical against the world's up seen in camera axes
         l = sqrtf(v.x*v.x + v.y*v.y + v.z*v.z);

   if (l < 1e-6f)
      return false;
   v.x /= up < 0.f ? -l : l;
   v.y /= up < 0.f ? -l : l;
   v.z /= up < 0.f ? -l : l;

   float d = h.x*v.x + h.y*v.y + h.z*v.z;

   h.x = sign*(h.x - d*v.x);
   h.y = sign*(h.y - d*v.y);
   h.z = sign*(h.z - d*v.z);
   l = sqrtf(h.x*h.x + h.y*h.y + h.z*h.z);
   if (l < 1e-6f)
      return false;
   h.x /= l;
   h.y /= l;
   h.z /= l;

   TVec3 c = { v.y*h.z - v.z*h.y, v.z*h.x - v.x*h.z, v.x*h.y - v.y*h.x };

   T[0] = v.x;
   T[3] = v.y;
   T[6] = v.z;
   T[1] = h.x;
   T[4] = h.y;
   T[7] = h.z;
   T[2] = c.x;
   T[5] = c.y;
   T[8] = c.z;
   return true;
}

//--------------------------------------------------------------------------------
// A rotation turned about its own camera axes by w degrees: out = in E(w) (TMat4 layout 3x3)
static void inspectTurnInCamera(const float *in, const float *wDeg, float *out)
{
   float w[3] = { wDeg[0]*0.01745329f, wDeg[1]*0.01745329f, wDeg[2]*0.01745329f },
         e[9];

   inspectPanoExp(w, e);
   for (int c = 0; c < 3; c++)
      for (int r = 0; r < 3; r++)
         out[3*c + r] = in[r]*e[c] + in[3 + r]*e[3 + c] + in[6 + r]*e[6 + c];
}

//--------------------------------------------------------------------------------
// A rotation turned by a small world turn w (left): out = exp(w) in (TMat4 layout 3x3)
static void inspectTurnWorld(const float *in, const float *w, float *out)
{
   float e[9];

   inspectPanoExp(w, e);
   for (int c = 0; c < 3; c++)
   {
      out[3*c] = e[0]*in[3*c] + e[1]*in[3*c + 1] + e[2]*in[3*c + 2];
      out[3*c + 1] = e[3]*in[3*c] + e[4]*in[3*c + 1] + e[5]*in[3*c + 2];
      out[3*c + 2] = e[6]*in[3*c] + e[7]*in[3*c + 1] + e[8]*in[3*c + 2];
   }
}

/*--------------------------------------------------------------------------------
   The rotations by the lines (user, 2026-09-30: "the ceiling's and floor's horizontals and the
   verticals of cabinets, doors and windows must always be the needle of your balance; no rectification
   is valid if the ceiling/floor is not horizontal or they are not vertical"). Vanishing points lie at
   infinity: the body's sway moves none of them, so they fix each frame's turn where the matched points
   (parallax) cannot. Least squares on small world turns w_i of every center-spin frame:
   - its measured vertical, carried to the world, onto the world's up (vector residual, J = [e]x);
   - each measured room axis, laid flat, onto the room's axes (heading alpha + 90k, alpha the frames'
     circular mean mod 90): a heading residual, J = (0, 1, 0); an axis beyond cVpAxisGateDeg is not the
     room's (a diagonal tile floor, a piece of furniture) and stays out;
   - the gyroscope between frames close in time (cVpLinkSec): their relative turn kept as it measured
     it, w_b - w_a = R_b log((R_a^T R_b)^T G_ab);
   each residual in degrees over its sigma, Huber on the lines; relinearized cVpRounds times.
  --------------------------------------------------------------------------------*/
static float inspectLineWeight(DWORD support);

static const float cVpSigmaDeg = 1.f,      // a measured vertical or axis
                   cVpLinkSigmaDeg = 0.7f, // the gyroscope between two frames seconds apart
                   cVpHuber = 2.f,         // in sigmas
                   cVpAxisGateDeg = 8.f,
                   cVpLinkSec = 10.f;

enum {
   cVpRounds = 4
};

//--------------------------------------------------------------------------------
// Adds one residual block r (m rows) with Jacobian rows J (m x 3 per unknown frame) to the normal equations
static void inspectVpAdd(float *H, float *g, int n, const int *at, int blocks, const float *J, const float *r, int m, float w)
{
   for (int row = 0; row < m; row++)
      for (int p = 0; p < blocks; p++)
         for (int a = 0; a < 3; a++)
         {
            float jp = J[(row*blocks + p)*3 + a];

            if (jp == 0.f)
               continue;
            g[at[p] + a] -= w*jp*r[row];
            for (int q = 0; q < blocks; q++)
               for (int b = 0; b < 3; b++)
                  H[(size_t)(at[p] + a)*n + at[q] + b] += w*jp*J[(row*blocks + q)*3 + b];
         }
}

//--------------------------------------------------------------------------------
static void inspectPanoVanishSolve(DWORD room)
{
   if (!inspectFrameVr || inspectPanoCount < 2)
      return;

   const int n = inspectPanoCount*3;
   float     sc = 0.f,
             ss = 0.f;

   // the room's axis: the circular mean of every frame's axis heading, mod 90 (x4 on the circle)
   for (int i = 0; i < inspectPanoCount; i++)
   {
      const TVanishResult &vr = inspectFrameVr[inspectPano[i].index];

      if ((vr.flags & (vfAxisA | vfAxisB)) && !isnan(vr.roomAxisDeg))
      {
         sc += cosf(4.f*vr.roomAxisDeg*0.01745329f);
         ss += sinf(4.f*vr.roomAxisDeg*0.01745329f);
      }
   }

   float alpha = atan2f(ss, sc)/4.f; // radians

   inspectVpAlpha = alpha;

   // capture order: the gyroscope links neighbors in time (the log is in processing order)
   TAlloc<int> order((size_t)inspectPanoCount);

   for (int i = 0; i < inspectPanoCount; i++)
      order[i] = i;
   for (int i = 1; i < inspectPanoCount; i++)
   {
      int x = order[i],
          j = i - 1;

      while (j >= 0 && inspectPano[order[j]].stamp > inspectPano[x].stamp)
      {
         order[j + 1] = order[j];
         j--;
      }
      order[j + 1] = x;
   }
   for (int round = 0; round < cVpRounds; round++)
   {
      TAlloc<float> H((size_t)n*n),
                    g((size_t)n);
      float         vSq = 0.f,
                    aSq = 0.f,
                    lSq = 0.f,
                    levelSq = 0.f;
      int           vN = 0,
                    aN = 0,
                    lN = 0,
                    gated = 0;
      const float   s2 = 1.f/(cVpSigmaDeg*0.01745329f*cVpSigmaDeg*0.01745329f),
                    l2 = 1.f/(cVpLinkSigmaDeg*0.01745329f*cVpLinkSigmaDeg*0.01745329f),
                    knee = cVpHuber*cVpSigmaDeg*0.01745329f;

      memset(H(), 0, sizeof(float)*(size_t)n*n);
      memset(g(), 0, sizeof(float)*(size_t)n);
      for (int i = 0; i < inspectPanoCount; i++)
      {
         const TPanoFrame    &p = inspectPano[i];
         const TVanishResult &vr = inspectFrameVr[p.index];
         int                  at[1] = { 3*i };

         if (vr.flags & vfVertical)
         {
            TVec3 v = vr.dirCam[0];
            TVec3 e = { p.rot[0]*v.x + p.rot[3]*v.y + p.rot[6]*v.z, p.rot[1]*v.x + p.rot[4]*v.y + p.rot[7]*v.z,
                        p.rot[2]*v.x + p.rot[5]*v.y + p.rot[8]*v.z };
            float l = sqrtf(e.x*e.x + e.y*e.y + e.z*e.z),
                  sgn = e.y < 0.f ? -1.f : 1.f;

            e.x *= sgn/l;
            e.y *= sgn/l;
            e.z *= sgn/l;

            // r = up - e - w x e = (up - e) + [e]x w
            float r[3] = { -e.x, 1.f - e.y, -e.z },
                  J[9] = { 0.f, -e.z, e.y, e.z, 0.f, -e.x, -e.y, e.x, 0.f },
                  mag = sqrtf(r[0]*r[0] + r[1]*r[1] + r[2]*r[2]),
                  w = s2*(mag <= knee ? 1.f : knee/mag);

            inspectVpAdd(H(), g(), n, at, 1, J, r, 3, w*inspectLineWeight(vr.support[0]));
            vSq += mag*mag;
            vN++;
         }
         for (int d = 1; d <= 2; d++)
         {
            if (!(vr.flags & (d == 1 ? vfAxisA : vfAxisB)))
               continue;

            TVec3 h = vr.dirCam[d];
            float ex = p.rot[0]*h.x + p.rot[3]*h.y + p.rot[6]*h.z,
                  ey = p.rot[1]*h.x + p.rot[4]*h.y + p.rot[7]*h.z,
                  ez = p.rot[2]*h.x + p.rot[5]*h.y + p.rot[8]*h.z,
                  hl = sqrtf(ex*ex + ey*ey + ez*ez),
                  phi = atan2f(ex, -ez),
                  off = phi - alpha;

            // the nearest room axis (mod 90): a line of the room either way along it
            off = off - 1.5707963f*floorf(off/1.5707963f + 0.5f);
            if (fabsf(off)*57.2957795f > cVpAxisGateDeg)
            {
               gated++;
               continue;
            }

            /* two rows (user, 2026-09-30: the canonical horizontals are an absolute compass, and a plumb check): its
               heading onto the room's axis, J = (0, 1, 0); its level - a horizontal line has no height to climb - the
               world y of the unit direction to 0, J = (e.z, 0, -e.x) */
            float ux = ex/hl,
                  uy = ey/hl,
                  uz = ez/hl,
                  r[2] = { -off, -uy },
                  J[6] = { 0.f, 1.f, 0.f, uz, 0.f, -ux },
                  mag = sqrtf(off*off + uy*uy),
                  w = s2*(mag <= knee ? 1.f : knee/mag);

            inspectVpAdd(H(), g(), n, at, 1, J, r, 2, w*inspectLineWeight(vr.support[d]));
            aSq += off*off;
            levelSq += uy*uy;
            aN++;
         }
      }
      for (int k = 0; k + 1 < inspectPanoCount; k++) // the gyroscope between neighbors in time
      {
         const TPanoFrame &a = inspectPano[order[k]],
                          &b = inspectPano[order[k + 1]];

         if ((float)(b.stamp - a.stamp)*1e-9f > cVpLinkSec)
            continue;

         // C = R_a^T R_b now, G = the gyroscope's; B's turn in its own axes w = log(C^T G), in the world R_b w
         float C[9],
               G[9],
               E[9];

         for (int r = 0; r < 3; r++)
            for (int c = 0; c < 3; c++)
            {
               C[3*r + c] = a.rot[3*r]*b.rot[3*c] + a.rot[3*r + 1]*b.rot[3*c + 1] + a.rot[3*r + 2]*b.rot[3*c + 2];
               G[3*r + c] = a.gyro[3*r]*b.gyro[3*c] + a.gyro[3*r + 1]*b.gyro[3*c + 1] + a.gyro[3*r + 2]*b.gyro[3*c + 2];
            }
         for (int r = 0; r < 3; r++)
            for (int c = 0; c < 3; c++)
               E[3*r + c] = C[r]*G[c] + C[3 + r]*G[3 + c] + C[6 + r]*G[6 + c];

         float ang = acosf(fmaxf(-1.f, fminf(1.f, 0.5f*(E[0] + E[4] + E[8] - 1.f)))),
               kk = ang > 1e-6f ? ang/(2.f*sinf(ang)) : 0.5f,
               wb[3] = { kk*(E[7] - E[5]), kk*(E[2] - E[6]), kk*(E[3] - E[1]) },
               ww[3] = { b.rot[0]*wb[0] + b.rot[3]*wb[1] + b.rot[6]*wb[2], b.rot[1]*wb[0] + b.rot[4]*wb[1] + b.rot[7]*wb[2],
                         b.rot[2]*wb[0] + b.rot[5]*wb[1] + b.rot[8]*wb[2] },
               // r = ww - (w_b - w_a): J_a = +I, J_b = -I
               J[18] = { 1.f, 0.f, 0.f, -1.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, -1.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, -1.f };
         int   at[2] = { 3*order[k], 3*order[k + 1] };

         inspectVpAdd(H(), g(), n, at, 2, J, ww, 3, l2);
         lSq += ww[0]*ww[0] + ww[1]*ww[1] + ww[2]*ww[2];
         lN++;
      }

      float diag = 0.f;

      for (int k = 0; k < n; k++)
         diag += H[(size_t)k*n + k];
      diag = diag/(float)n;
      for (int k = 0; k < n; k++)
         H[(size_t)k*n + k] += 1e-6f*diag + 1e-9f;
      printf("  panorama room %lu: lines round %d: verticals %d RMS %.2f, room axes %d RMS %.2f (level %.2f; %d off the axes), gyroscope"
             " links %d RMS %.2f degrees; room axis %.2f\n", (unsigned long)room, round + 1, vN,
             vN ? sqrtf(vSq/(float)vN)*57.2957795f : 0.f, aN, aN ? sqrtf(aSq/(float)aN)*57.2957795f : 0.f,
             aN ? sqrtf(levelSq/(float)aN)*57.2957795f : 0.f, gated, lN,
             lN ? sqrtf(lSq/(float)lN)*57.2957795f : 0.f, alpha*57.2957795f);
      if (!inspectPanoSolve(H(), g(), n))
         break;
      for (int i = 0; i < inspectPanoCount; i++)
      {
         TPanoFrame &a = inspectPano[i];
         float       e[9],
                     old[9];

         inspectPanoExp(&g[3*i], e);
         memcpy(old, a.rot, sizeof(old));
         for (int c = 0; c < 3; c++)
         {
            a.rot[3*c] = e[0]*old[3*c] + e[1]*old[3*c + 1] + e[2]*old[3*c + 2];
            a.rot[3*c + 1] = e[3]*old[3*c] + e[4]*old[3*c + 1] + e[5]*old[3*c + 2];
            a.rot[3*c + 2] = e[6]*old[3*c] + e[7]*old[3*c + 1] + e[8]*old[3*c + 2];
         }
      }
   }

   /* the gyroscope's fluctuation (user, 2026-09-30: it sets the search window of the least squares): each frame's
      correction by the lines, in the world (log of R R_gyro^T); its heading part split into a slow drift (a line over
      the capture time) and what is left shot by shot, its tilt part as it is */
   float st = 0.f,
         sh = 0.f,
         stt = 0.f,
         sth = 0.f,
         tiltSq = 0.f;
   QWORD t0 = inspectPano[order[0]].stamp; // the first shot: times in seconds from it
   TAlloc<float> hd((size_t)inspectPanoCount),
                 tm((size_t)inspectPanoCount);

   for (int i = 0; i < inspectPanoCount; i++)
   {
      const TPanoFrame &p = inspectPano[i];
      float             D[9];

      for (int r = 0; r < 3; r++) // R R_gyro^T, row-major
         for (int c = 0; c < 3; c++)
            D[3*r + c] = p.rot[r]*p.gyro[c] + p.rot[3 + r]*p.gyro[3 + c] + p.rot[6 + r]*p.gyro[6 + c];

      float ang = acosf(fmaxf(-1.f, fminf(1.f, 0.5f*(D[0] + D[4] + D[8] - 1.f)))),
            kk = ang > 1e-6f ? ang/(2.f*sinf(ang)) : 0.5f,
            wx = kk*(D[7] - D[5])*57.2957795f,
            wy = kk*(D[2] - D[6])*57.2957795f,
            wz = kk*(D[3] - D[1])*57.2957795f;

      hd[i] = wy;
      tm[i] = (float)((p.stamp - t0)/1000000u)*1e-3f;
      st += tm[i];
      sh += hd[i];
      stt += tm[i]*tm[i];
      sth += tm[i]*hd[i];
      tiltSq += wx*wx + wz*wz;
   }

   float nf = (float)inspectPanoCount,
         slope = (nf*sth - st*sh)/fmaxf(1e-6f, nf*stt - st*st),
         icpt = (sh - slope*st)/nf,
         left = 0.f;

   for (int i = 0; i < inspectPanoCount; i++)
   {
      float d = hd[i] - (icpt + slope*tm[i]);

      left += d*d;
   }
   printf("  panorama room %lu: the gyroscope against the lines: heading drift %.2f degrees/min over %.0f s, shot by shot"
          " %.2f degrees (sigma) beyond it; tilt %.2f degrees (RMS) - a search window of ~3 sigma\n", (unsigned long)room,
          60.f*slope, tm[order[inspectPanoCount - 1]], sqrtf(left/nf), sqrtf(tiltSq/nf));
}

/*--------------------------------------------------------------------------------
   The centers on the body's circle (user, 2026-09-30: the operator turns about the trunk with the
   arm a little out; stage 2, the turns already held by the lines). Each center-spin frame's camera
   sits r ahead of the trunk's axis along its own flat forward and y_b above the pivot for its band
   (the wrist lifts the phone differently to the ceiling, the horizon and the floor): 4 unknowns for
   the whole spin. The matched points of every neighbor pair are the observations: A's bearing from
   A's center meets the room's box, that point seen from B's center must fall on B's bearing; the
   misfit (radians) weighed by Huber (furniture off the box's planes). Gauss-Newton, numeric Jacobian.
   The centers land in each frame's shift.
  --------------------------------------------------------------------------------*/
enum {
   cCircleMaxObs = 262144,
   cCircleIters = 12,
   cCircleParams = 1 + inspectBodyBands // r, then y per band
};

static const float cCircleHuber = 0.005f; // radians (~0.3 degree)

struct TCircleObs {
   int   a,
         b;
   TVec3 da, // A's bearing, A's camera axes
         db; // its match, B's camera axes
};

//--------------------------------------------------------------------------------
// A frame's center under the circle's parameters (world)
static TVec3 inspectCircleCenter(const TPanoFrame &p, const float *th)
{
   /* the sphere (user, 2026-09-30: the eyes turn about the skull, steady; the camera turns, rises and falls with the
      scene sought - it draws a sphere): the camera r ahead of a steady center along its whole aim, tilt included;
      --circle keeps the aim flat (the trunk's axis only) */
   TVec3 f = inspectCircleFlat ? inspectHorizontalForward(p.rot) : TVec3{ -p.rot[6], -p.rot[7], -p.rot[8] },
         c = { th[0]*f.x, th[0]*f.y + th[1 + inspectBodyBandOf(p)], th[0]*f.z };

   return c;
}

//--------------------------------------------------------------------------------
// One observation's misfit (3 components, ~radians); false when A's ray leaves the box or B looks away
static bool inspectCircleResidual(const TCircleObs &o, const float *th, float *r)
{
   const TPanoFrame &a = inspectPano[o.a],
                    &b = inspectPano[o.b];
   TVec3             ca = inspectCircleCenter(a, th),
                     cb = inspectCircleCenter(b, th),
                     d = { a.rot[0]*o.da.x + a.rot[3]*o.da.y + a.rot[6]*o.da.z, a.rot[1]*o.da.x + a.rot[4]*o.da.y + a.rot[7]*o.da.z,
                           a.rot[2]*o.da.x + a.rot[5]*o.da.y + a.rot[8]*o.da.z };
   float             t = 0.f;
   int               plane = -1;

   if (!inspectBoxHit(ca, d, t, plane))
      return false;

   TVec3 q = { ca.x + t*d.x - cb.x, ca.y + t*d.y - cb.y, ca.z + t*d.z - cb.z },
         qc = { b.rot[0]*q.x + b.rot[1]*q.y + b.rot[2]*q.z, b.rot[3]*q.x + b.rot[4]*q.y + b.rot[5]*q.z,
                b.rot[6]*q.x + b.rot[7]*q.y + b.rot[8]*q.z };
   float l = sqrtf(qc.x*qc.x + qc.y*qc.y + qc.z*qc.z);

   if (l < 1e-6f || qc.z > 0.f)
      return false;
   r[0] = qc.x/l - o.db.x;
   r[1] = qc.y/l - o.db.y;
   r[2] = qc.z/l - o.db.z;
   return true;
}

/*--------------------------------------------------------------------------------
   The joint fit (user, 2026-09-30: the frames correcting one another - the lines, the gyroscope and
   the points together, every freedom orchestrated). Unknowns per center-spin frame: its turn w_i (3)
   and its center c_i (3, inside the small sphere the arm sweeps: the body circle's point plus a sway).
   Residuals, each over its sigma:
   - every matched point: B's ray off the epipolar plane of A's ray and the baseline (depth-free,
     cJointEpiSigmaDeg - the points are that precise);
   - the lines, as in inspectPanoVanishSolve (vertical onto up; room axes: heading mod 90 and level);
   - the gyroscope between neighbors in time;
   - each center's sway from its point on the circle (cJointSwayM; it also holds the free translation).
   Gauss-Newton on the dense normal equations (6 per frame), numeric Jacobians for the points, Huber.
  --------------------------------------------------------------------------------*/
static const float cJointEpiSigmaDeg = 0.15f,
                   cJointEpiHuberDeg = 0.2f,
                   cJointSwayM = 0.03f,
                   cJointMinBaseM = 0.01f; // a pair closer than this (a tilt of the wrist): no epipolar plane to trust

enum {
   cJointRounds = 6
};

//--------------------------------------------------------------------------------
// A point's epipolar misfit (radians, signed) for frame rotations ra, rb and centers ca, cb; false without a baseline
static bool inspectJointEpi(const TCircleObs &o, const float *ra, const float *rb, const TVec3 &ca, const TVec3 &cb, float &r)
{
   TVec3 da = { ra[0]*o.da.x + ra[3]*o.da.y + ra[6]*o.da.z, ra[1]*o.da.x + ra[4]*o.da.y + ra[7]*o.da.z,
                ra[2]*o.da.x + ra[5]*o.da.y + ra[8]*o.da.z },
         db = { rb[0]*o.db.x + rb[3]*o.db.y + rb[6]*o.db.z, rb[1]*o.db.x + rb[4]*o.db.y + rb[7]*o.db.z,
                rb[2]*o.db.x + rb[5]*o.db.y + rb[8]*o.db.z },
         B = { cb.x - ca.x, cb.y - ca.y, cb.z - ca.z },
         n = { B.y*da.z - B.z*da.y, B.z*da.x - B.x*da.z, B.x*da.y - B.y*da.x };
   float bl = sqrtf(B.x*B.x + B.y*B.y + B.z*B.z),
         nl = sqrtf(n.x*n.x + n.y*n.y + n.z*n.z);

   if (bl < cJointMinBaseM || nl < 1e-9f)
      return false;
   r = asinf(fmaxf(-1.f, fminf(1.f, (db.x*n.x + db.y*n.y + db.z*n.z)/nl)));
   return true;
}

//--------------------------------------------------------------------------------
// A line direction's weight by its support (user, 2026-09-30: the lines are the needle): sigma ~ sqrt(1000/support)
static float inspectLineWeight(DWORD support)
{
   float s = sqrtf(1000.f/(float)(support > 1u ? support : 1u));

   s = fmaxf(0.3f, fminf(2.f, s));
   return 1.f/(s*s);
}

//--------------------------------------------------------------------------------
// Each frame's line misfit after the fit: its vertical off the world's up, its room axis off level (degrees)
static void inspectLineReport(DWORD room)
{
   int   over = 0;
   float worst = 0.f;

   for (int i = 0; i < inspectPanoCount && inspectFrameVr; i++)
   {
      const TPanoFrame    &p = inspectPano[i];
      const TVanishResult &vr = inspectFrameVr[p.index];

      if (!(vr.flags & vfVertical))
         continue;

      TVec3 v = vr.dirCam[0];
      float ey = p.rot[1]*v.x + p.rot[4]*v.y + p.rot[7]*v.z,
            l = sqrtf(v.x*v.x + v.y*v.y + v.z*v.z),
            off = acosf(fminf(1.f, fabsf(ey)/l))*57.2957795f;

      worst = fmaxf(worst, off);
      if (off > 1.f || p.index == inspectPairA)
      {
         printf("  faces room %lu: frame %d: its vertical (%lu edges) %.2f degrees off the world's up after the fit\n",
                (unsigned long)room, p.index, (unsigned long)vr.support[0], off);
         over += off > 1.f ? 1 : 0;
      }
   }
   printf("  faces room %lu: verticals after the fit: %d beyond 1 degree, worst %.2f\n", (unsigned long)room, over, worst);

   // the room axes: each measured horizontal's heading after the fit against the set's circular mean (mod 90)
   float sc = 0.f,
         ss = 0.f;

   for (int pass = 0; pass < 2; pass++)
   {
      float mean = atan2f(ss, sc)/4.f;
      int   axisOver = 0;

      for (int i = 0; i < inspectPanoCount && inspectFrameVr; i++)
      {
         const TPanoFrame    &p = inspectPano[i];
         const TVanishResult &vr = inspectFrameVr[p.index];

         for (int d = 1; d <= 2; d++)
         {
            if (!(vr.flags & (d == 1 ? vfAxisA : vfAxisB)))
               continue;

            TVec3 h = vr.dirCam[d];
            float ex = p.rot[0]*h.x + p.rot[3]*h.y + p.rot[6]*h.z,
                  ez = p.rot[2]*h.x + p.rot[5]*h.y + p.rot[8]*h.z,
                  phi = atan2f(ex, -ez);

            if (pass == 0)
            {
               sc += cosf(4.f*phi);
               ss += sinf(4.f*phi);
               continue;
            }

            float off = phi - mean;

            off = (off - 1.5707963f*floorf(off/1.5707963f + 0.5f))*57.2957795f;
            if (p.index == inspectPairA || fabsf(off) > 3.f)
            {
               printf("  faces room %lu: frame %d: its room axis %c (%lu edges) %+.2f degrees off the room's after the fit\n",
                      (unsigned long)room, p.index, d == 1 ? 'A' : 'B', (unsigned long)vr.support[d], off);
               axisOver += fabsf(off) > 3.f ? 1 : 0;
            }
         }
      }
      if (pass == 1)
         printf("  faces room %lu: room axes after the fit: %d beyond 3 degrees\n", (unsigned long)room, axisOver);
   }
}

//--------------------------------------------------------------------------------
static void inspectPanoJoint(DWORD room, const TCircleObs *obs, int count, const float *th)
{
   const int     n = inspectPanoCount*6;
   const float   eps = 1e-4f,
                 epi2 = 1.f/(cJointEpiSigmaDeg*0.01745329f*cJointEpiSigmaDeg*0.01745329f),
                 epiKnee = cJointEpiHuberDeg*0.01745329f,
                 sway2 = 1.f/(cJointSwayM*cJointSwayM);
   TAlloc<float> H((size_t)n*n),
                 g((size_t)n),
                 mags((size_t)count + 1u);

   for (int round = 0; round < cJointRounds; round++)
   {
      int used = 0;

      memset(H(), 0, sizeof(float)*(size_t)n*n);
      memset(g(), 0, sizeof(float)*(size_t)n);

      // the points: 12 unknowns each (A's and B's turn and center)
      for (int k = 0; k < count; k++)
      {
         const TCircleObs &o = obs[k];
         const TPanoFrame &a = inspectPano[o.a],
                          &b = inspectPano[o.b];
         TVec3             ca = { a.shift[0], a.shift[1], a.shift[2] },
                           cb = { b.shift[0], b.shift[1], b.shift[2] };
         float             r0 = 0.f,
                           J[12];
         int               at[12];
         bool              ok = inspectJointEpi(o, a.rot, b.rot, ca, cb, r0);

         for (int q = 0; q < 12 && ok; q++)
         {
            float w[3] = { 0.f, 0.f, 0.f },
                  ra[9],
                  rb[9],
                  rq = 0.f;
            TVec3 cq = q < 6 ? ca : cb;

            memcpy(ra, a.rot, sizeof(ra));
            memcpy(rb, b.rot, sizeof(rb));
            if (q%6 < 3)
            {
               w[q%3] = eps;
               inspectTurnWorld(q < 6 ? a.rot : b.rot, w, q < 6 ? ra : rb);
            }
            else
            {
               float *c = &cq.x;

               c[q%3] += eps;
            }
            ok = inspectJointEpi(o, ra, rb, q < 6 ? cq : ca, q < 6 ? cb : cq, rq);
            J[q] = (rq - r0)/eps;
            at[q] = 6*(q < 6 ? o.a : o.b) + (q%6 < 3 ? q%3 : 3 + q%3);
         }
         if (!ok)
            continue;

         float mag = fabsf(r0),
               wt = epi2*(mag <= epiKnee ? 1.f : epiKnee/mag);

         mags[used++] = mag;
         for (int p = 0; p < 12; p++)
         {
            g[at[p]] -= wt*J[p]*r0;
            for (int q = 0; q < 12; q++)
               H[(size_t)at[p]*n + at[q]] += wt*J[p]*J[q];
         }
      }

      // the lines and the gyroscope: the rotation part (every sixth block's first three)
      const float s2 = 1.f/(cVpSigmaDeg*0.01745329f*cVpSigmaDeg*0.01745329f),
                  l2 = 1.f/(cVpLinkSigmaDeg*0.01745329f*cVpLinkSigmaDeg*0.01745329f),
                  knee = cVpHuber*cVpSigmaDeg*0.01745329f;
      float       sc = 0.f,
                  ss = 0.f;

      for (int i = 0; i < inspectPanoCount; i++)
      {
         const TVanishResult &vr = inspectFrameVr[inspectPano[i].index];

         if ((vr.flags & (vfAxisA | vfAxisB)) && !isnan(vr.roomAxisDeg))
         {
            sc += cosf(4.f*vr.roomAxisDeg*0.01745329f);
            ss += sinf(4.f*vr.roomAxisDeg*0.01745329f);
         }
      }

      float alpha = atan2f(ss, sc)/4.f;

      for (int i = 0; i < inspectPanoCount && inspectFrameVr; i++)
      {
         const TPanoFrame    &p = inspectPano[i];
         const TVanishResult &vr = inspectFrameVr[p.index];
         int                  at[1] = { 6*i };

         if (vr.flags & vfVertical)
         {
            TVec3 v = vr.dirCam[0],
                  e = { p.rot[0]*v.x + p.rot[3]*v.y + p.rot[6]*v.z, p.rot[1]*v.x + p.rot[4]*v.y + p.rot[7]*v.z,
                        p.rot[2]*v.x + p.rot[5]*v.y + p.rot[8]*v.z };
            float l = sqrtf(e.x*e.x + e.y*e.y + e.z*e.z),
                  sgn = e.y < 0.f ? -1.f : 1.f;

            e.x *= sgn/l;
            e.y *= sgn/l;
            e.z *= sgn/l;

            float r[3] = { -e.x, 1.f - e.y, -e.z },
                  J[9] = { 0.f, -e.z, e.y, e.z, 0.f, -e.x, -e.y, e.x, 0.f },
                  mag = sqrtf(r[0]*r[0] + r[1]*r[1] + r[2]*r[2]);

            inspectVpAdd(H(), g(), n, at, 1, J, r, 3, s2*inspectLineWeight(vr.support[0])*(mag <= knee ? 1.f : knee/mag));
         }
         for (int d = 1; d <= 2; d++)
         {
            if (!(vr.flags & (d == 1 ? vfAxisA : vfAxisB)))
               continue;

            TVec3 h = vr.dirCam[d];
            float ex = p.rot[0]*h.x + p.rot[3]*h.y + p.rot[6]*h.z,
                  ey = p.rot[1]*h.x + p.rot[4]*h.y + p.rot[7]*h.z,
                  ez = p.rot[2]*h.x + p.rot[5]*h.y + p.rot[8]*h.z,
                  hl = sqrtf(ex*ex + ey*ey + ez*ez),
                  off = atan2f(ex, -ez) - alpha;

            off = off - 1.5707963f*floorf(off/1.5707963f + 0.5f);
            if (fabsf(off)*57.2957795f > cVpAxisGateDeg)
               continue;

            float r[2] = { -off, -ey/hl },
                  J[6] = { 0.f, 1.f, 0.f, ez/hl, 0.f, -ex/hl },
                  mag = sqrtf(r[0]*r[0] + r[1]*r[1]);

            inspectVpAdd(H(), g(), n, at, 1, J, r, 2, s2*inspectLineWeight(vr.support[d])*(mag <= knee ? 1.f : knee/mag));
         }

         // the sway: the center off its point on the circle, J = I on the center block
         TVec3 circ = inspectCircleCenter(p, th);
         float r[3] = { circ.x - p.shift[0], circ.y - p.shift[1], circ.z - p.shift[2] },
               J[9] = { 1.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 1.f };
         int   atc[1] = { 6*i + 3 };

         for (int k = 0; k < 3; k++)
            r[k] = -r[k]; // r(dc) = (c + dc) - circ
         inspectVpAdd(H(), g(), n, atc, 1, J, r, 3, sway2);
      }
      for (int i = 0; i < inspectPanoCount; i++) // the gyroscope: consecutive in time
         for (int j = 0; j < inspectPanoCount; j++)
         {
            const TPanoFrame &a = inspectPano[i],
                             &b = inspectPano[j];

            if (b.stamp <= a.stamp || (float)(b.stamp - a.stamp)*1e-9f > cVpLinkSec)
               continue;

            bool next = true; // b is a's successor: no frame in between

            for (int k = 0; k < inspectPanoCount && next; k++)
               next = !(inspectPano[k].stamp > a.stamp && inspectPano[k].stamp < b.stamp);
            if (!next)
               continue;

            float C[9],
                  G[9],
                  E[9];

            for (int r = 0; r < 3; r++)
               for (int c = 0; c < 3; c++)
               {
                  C[3*r + c] = a.rot[3*r]*b.rot[3*c] + a.rot[3*r + 1]*b.rot[3*c + 1] + a.rot[3*r + 2]*b.rot[3*c + 2];
                  G[3*r + c] = a.gyro[3*r]*b.gyro[3*c] + a.gyro[3*r + 1]*b.gyro[3*c + 1] + a.gyro[3*r + 2]*b.gyro[3*c + 2];
               }
            for (int r = 0; r < 3; r++)
               for (int c = 0; c < 3; c++)
                  E[3*r + c] = C[r]*G[c] + C[3 + r]*G[3 + c] + C[6 + r]*G[6 + c];

            float ang = acosf(fmaxf(-1.f, fminf(1.f, 0.5f*(E[0] + E[4] + E[8] - 1.f)))),
                  kk = ang > 1e-6f ? ang/(2.f*sinf(ang)) : 0.5f,
                  wb[3] = { kk*(E[7] - E[5]), kk*(E[2] - E[6]), kk*(E[3] - E[1]) },
                  ww[3] = { b.rot[0]*wb[0] + b.rot[3]*wb[1] + b.rot[6]*wb[2], b.rot[1]*wb[0] + b.rot[4]*wb[1] + b.rot[7]*wb[2],
                            b.rot[2]*wb[0] + b.rot[5]*wb[1] + b.rot[8]*wb[2] },
                  J[18] = { 1.f, 0.f, 0.f, -1.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, -1.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, -1.f };
            int   at[2] = { 6*i, 6*j };

            inspectVpAdd(H(), g(), n, at, 2, J, ww, 3, l2);
         }

      float med = inspectQuantile(mags(), used, 0.5f)*57.2957795f,
            diag = 0.f;

      for (int k = 0; k < n; k++)
         diag += H[(size_t)k*n + k];
      diag /= (float)n;
      for (int k = 0; k < n; k++)
         H[(size_t)k*n + k] += 1e-6f*diag + 1e-9f;
      printf("  faces room %lu: joint round %d: %d points, epipolar misfit median %.3f degrees\n", (unsigned long)room,
             round + 1, used, med);
      if (!inspectPanoSolve(H(), g(), n))
         break;

      float turn = 0.f,
            move = 0.f;

      for (int i = 0; i < inspectPanoCount; i++)
      {
         TPanoFrame &p = inspectPano[i];
         float       rot[9],
                     w[3] = { fmaxf(-0.05f, fminf(0.05f, g[6*i])), fmaxf(-0.05f, fminf(0.05f, g[6*i + 1])),
                              fmaxf(-0.05f, fminf(0.05f, g[6*i + 2])) };

         inspectTurnWorld(p.rot, w, rot);
         memcpy(p.rot, rot, sizeof(rot));
         for (int k = 0; k < 3; k++)
            p.shift[k] += fmaxf(-0.05f, fminf(0.05f, g[6*i + 3 + k])); // at most 5 cm a round
         turn += sqrtf(w[0]*w[0] + w[1]*w[1] + w[2]*w[2])*57.2957795f;
         move += sqrtf(g[6*i + 3]*g[6*i + 3] + g[6*i + 4]*g[6*i + 4] + g[6*i + 5]*g[6*i + 5]);
      }
      printf("  faces room %lu: joint round %d: frames turned %.3f degrees, moved %.3f m on average\n", (unsigned long)room,
             round + 1, turn/(float)inspectPanoCount, move/(float)inspectPanoCount);
   }

   float swaySum = 0.f;

   for (int i = 0; i < inspectPanoCount; i++)
   {
      TVec3 circ = inspectCircleCenter(inspectPano[i], th);
      float dx = inspectPano[i].shift[0] - circ.x,
            dy = inspectPano[i].shift[1] - circ.y,
            dz = inspectPano[i].shift[2] - circ.z;

      swaySum += sqrtf(dx*dx + dy*dy + dz*dz);
   }
   printf("  faces room %lu: joint: the centers sway %.3f m from the body circle on average\n", (unsigned long)room,
          swaySum/(float)inspectPanoCount);
   for (int i = 0; i < inspectPanoCount; i++)
      if (inspectPanoHas[inspectPano[i].index])
         memcpy(inspectPanoRot[inspectPano[i].index], inspectPano[i].rot, sizeof(inspectPano[i].rot));
   inspectLineReport(room);
}

//--------------------------------------------------------------------------------
static void inspectPanoCircleSolve(DWORD room)
{
   TAlloc<float>      cornerXY((size_t)inspectPanoCount*cPointCorners*2u);
   TAlloc<int>        cornerN((size_t)inspectPanoCount);
   TAlloc<TCircleObs> obs((size_t)cCircleMaxObs);
   TAlloc<TVec3>      ba((size_t)cPointCorners),
                      bb((size_t)cPointCorners);
   int                count = 0,
                      pairs = 0;

   for (int i = 0; i < inspectPanoCount; i++)
      cornerN[i] = inspectPointCorners(inspectPano[i], &cornerXY[(size_t)i*cPointCorners*2u]);
   for (int i = 0; i < inspectPanoCount; i++)
      for (int j = i + 1; j < inspectPanoCount && count < cCircleMaxObs; j++)
      {
         const TPanoFrame &a = inspectPano[i],
                          &b = inspectPano[j];

         if (a.rot[6]*b.rot[6] + a.rot[7]*b.rot[7] + a.rot[8]*b.rot[8] < cPairCos)
            continue;

         float w[3],
               mis,
               rotMed,
               epiMed,
               base[3],
               pure[3],
               gyroMed[2];
         int   fit = 0,
               matched = 0,
               got = 0;

         if (!inspectPointPair(a, b, &cornerXY[(size_t)i*cPointCorners*2u], cornerN[i], w, fit, mis, matched, rotMed, epiMed,
                               base, pure, gyroMed, ba(), bb(), &got, cPointCorners))
            continue;
         for (int k = 0; k < got && count < cCircleMaxObs; k++)
         {
            obs[count].a = i;
            obs[count].b = j;
            obs[count].da = ba[k];
            obs[count].db = bb[k];
            count++;
         }
         pairs++;
      }

   float th[cCircleParams] = {},
         first = 0.f,
         last = 0.f;

   for (int it = 0; it < cCircleIters; it++)
   {
      float H[cCircleParams*cCircleParams] = {},
            g[cCircleParams] = {};
      TAlloc<float> mags((size_t)count);
      int           used = 0;

      for (int k = 0; k < count; k++)
      {
         float r0[3],
               J[3*cCircleParams];

         if (!inspectCircleResidual(obs[k], th, r0))
            continue;

         bool ok = true;

         for (int q = 0; q < cCircleParams && ok; q++)
         {
            float tq[cCircleParams],
                  rq[3];

            memcpy(tq, th, sizeof(tq));
            tq[q] += 1e-3f;
            ok = inspectCircleResidual(obs[k], tq, rq);
            for (int c = 0; c < 3 && ok; c++)
               J[c*cCircleParams + q] = (rq[c] - r0[c])/1e-3f;
         }
         if (!ok)
            continue;

         float mag = sqrtf(r0[0]*r0[0] + r0[1]*r0[1] + r0[2]*r0[2]),
               wt = mag <= cCircleHuber ? 1.f : cCircleHuber/mag;

         mags[used++] = mag;
         for (int c = 0; c < 3; c++)
            for (int p = 0; p < cCircleParams; p++)
            {
               g[p] -= wt*J[c*cCircleParams + p]*r0[c];
               for (int q = 0; q < cCircleParams; q++)
                  H[p*cCircleParams + q] += wt*J[c*cCircleParams + p]*J[c*cCircleParams + q];
            }
      }

      float med = inspectQuantile(mags(), used, 0.5f)*57.2957795f;

      if (it == 0)
         first = med;
      last = med;
      for (int p = 0; p < cCircleParams; p++)
      {
         H[p*cCircleParams + p] += 1e-6f*H[p*cCircleParams + p] + 1e-9f;
         if (p > 0 && !inspectCircleHeights) // the band heights held at the pivot: free, they traded with the box's height
         {
            for (int q = 0; q < cCircleParams; q++)
               H[p*cCircleParams + q] = H[q*cCircleParams + p] = 0.f;
            H[p*cCircleParams + p] = 1.f;
            g[p] = 0.f;
         }
      }
      if (!used || !inspectPanoSolve(H, g, cCircleParams))
         break;
      for (int p = 0; p < cCircleParams; p++)
         th[p] += fmaxf(-0.1f, fminf(0.1f, g[p])); // at most 10 cm a step
   }
   printf("  faces room %lu: body circle from %d points of %d pairs: median misfit %.3f -> %.3f degrees; the phone %.3f m"
          " ahead of the trunk's axis, %.3f / %.3f / %.3f m above the pivot (floor / horizon / ceiling band)\n",
          (unsigned long)room, count, pairs, first, last, th[0], th[1], th[2], th[3]);
   for (int i = 0; i < inspectPanoCount; i++)
   {
      TVec3 c = inspectCircleCenter(inspectPano[i], th);

      inspectPano[i].shift[0] = c.x;
      inspectPano[i].shift[1] = c.y;
      inspectPano[i].shift[2] = c.z;
   }
   inspectPanoJoint(room, obs(), count, th);
}

/*--------------------------------------------------------------------------------
   The wide basin (user, 2026-09-30: coarse first, the fit gradually refined down to 1x). The photometric
   Gauss-Newton sees only what lies within a pixel or two of its level, and the gyroscope misses by
   degrees (19 -> 20: 2.7). Each pair of neighbors (forwards within cPairCos) is searched by brute force,
   coarse to fine (1/64 +-6 by 1 degree, 1/32 +-1 by 0.25, 1/16 +-0.25 by 0.0625): its relative turn w,
   about B's own axes. All pairs then pull the frames together by least squares on small world turns:
   w_b - w_a = R_b w (the turn B asks for, in the world), each pair weighed by its pixels and by Huber
   on its misfit, a weak pull toward the gyroscope holding the set. Measured and solved cPairRounds times.
  --------------------------------------------------------------------------------*/
static void inspectPanoPairs(DWORD room)
{
   const int     n = inspectPanoCount*3;
   TAlloc<float> cornerXY((size_t)inspectPanoCount*cPointCorners*2u);
   TAlloc<int>   cornerN((size_t)inspectPanoCount);
   int           cornerSum = 0;

   for (int i = 0; i < inspectPanoCount; i++) // corners depend on the picture only: found once
   {
      cornerN[i] = inspectPointCorners(inspectPano[i], &cornerXY[(size_t)i*cPointCorners*2u]);
      cornerSum += cornerN[i];
   }
   printf("  panorama room %lu: %d corners per frame on average\n", (unsigned long)room,
          inspectPanoCount ? cornerSum/inspectPanoCount : 0);

   for (int round = 0; round < cPairRounds; round++)
   {
      TAlloc<float> H((size_t)n*n),
                    g((size_t)n),
                    pw((size_t)inspectPanoCount*inspectPanoCount*4u); // per pair: world turn asked for, weight
      int           pairs = 0,
                    tried = 0,
                    matchSum = 0,
                    medPairs = 0,
                    inlierSum = 0;
      float         sumTurn = 0.f,
                    misSum = 0.f,
                    rotMedSum = 0.f,
                    epiMedSum = 0.f;

      memset(pw(), 0, sizeof(float)*(size_t)inspectPanoCount*inspectPanoCount*4u);
      for (int i = 0; i < inspectPanoCount; i++)
         for (int j = i + 1; j < inspectPanoCount; j++)
         {
            const TPanoFrame &a = inspectPano[i],
                             &b = inspectPano[j];

            if (a.rot[6]*b.rot[6] + a.rot[7]*b.rot[7] + a.rot[8]*b.rot[8] < cPairCos)
               continue;

            float total[3] = { 0.f, 0.f, 0.f },
                  mis = 0.f;
            int   count = 0,
                  matched = 0;

            float rotMed = 0.f,
                  epiMed = 0.f,
                  base[3],
                  pure[3],
                  gyroMed[2];
            bool  met = inspectPointPair(a, b, &cornerXY[(size_t)i*cPointCorners*2u], cornerN[i], total, count, mis, matched,
                                         rotMed, epiMed, base, pure, gyroMed);

            tried++;
            matchSum += matched;
            if (met && round == 0 && inspectPairDebug)
            {
               TVec3 bw = { a.rot[0]*base[0] + a.rot[3]*base[1] + a.rot[6]*base[2],
                            a.rot[1]*base[0] + a.rot[4]*base[1] + a.rot[7]*base[2],
                            a.rot[2]*base[0] + a.rot[5]*base[1] + a.rot[8]*base[2] };

               float gap = (float)(a.stamp > b.stamp ? a.stamp - b.stamp : b.stamp - a.stamp)*1e-9f,
                     turn = sqrtf(total[0]*total[0] + total[1]*total[1] + total[2]*total[2]),
                     pureTurn = sqrtf(pure[0]*pure[0] + pure[1]*pure[1] + pure[2]*pure[2]),
                     rel[9],
                     relW[3] = { 0.f, 0.f, 0.f };

               // the gyroscope's turn from B to A in B's axes (Q = R_b^T R_a): the pure turn's part along it and across it
               for (int r = 0; r < 3; r++)
                  for (int c = 0; c < 3; c++)
                     rel[3*r + c] = b.rot[3*r]*a.rot[3*c] + b.rot[3*r + 1]*a.rot[3*c + 1] + b.rot[3*r + 2]*a.rot[3*c + 2];

               float relAng = acosf(fmaxf(-1.f, fminf(1.f, 0.5f*(rel[0] + rel[4] + rel[8] - 1.f)))),
                     relK = relAng > 1e-6f ? relAng/(2.f*sinf(relAng)) : 0.5f;

               relW[0] = relK*(rel[7] - rel[5]);
               relW[1] = relK*(rel[2] - rel[6]);
               relW[2] = relK*(rel[3] - rel[1]);

               float relL = sqrtf(relW[0]*relW[0] + relW[1]*relW[1] + relW[2]*relW[2]),
                     along = relL > 1e-6f ? (pure[0]*relW[0] + pure[1]*relW[1] + pure[2]*relW[2])/relL : 0.f,
                     across = sqrtf(fmaxf(0.f, pureTurn*pureTurn - along*along));

               /* the vanishing points (user, 2026-09-30: the ceiling's and floor's horizontals and the verticals of
                  cabinets, doors and windows are the balance's needle): at infinity, the body's sway moves none of
                  them. Each frame's triad - its measured vertical, a measured room axis made square to it, their
                  cross - in camera axes; the pair's turn M = T_a T_b^T, the axis label and signs of B taken as the
                  gyroscope's nearest (mod 90) */
               const TVanishResult *va = inspectFrameVr ? &inspectFrameVr[a.index] : NULL,
                                   *vb = inspectFrameVr ? &inspectFrameVr[b.index] : NULL;
               float                vpGyro = NAN,
                                    vpPure = NAN,
                                    vpTwo = NAN;

               if (va && vb && (va->flags & vfVertical) && (vb->flags & vfVertical) && (va->flags & (vfAxisA | vfAxisB))
                   && (vb->flags & (vfAxisA | vfAxisB)))
               {
                  float Ta[9],
                        Mg[9],
                        Mp[9],
                        Mt[9];

                  inspectVanishTriad(*va, a.rot, NULL, Ta);
                  for (int r = 0; r < 3; r++)
                     for (int c = 0; c < 3; c++)
                        Mg[3*r + c] = a.rot[3*r]*b.rot[3*c] + a.rot[3*r + 1]*b.rot[3*c + 1] + a.rot[3*r + 2]*b.rot[3*c + 2];
                  inspectMatFromTurn(Mg, pure, Mp);
                  inspectMatFromTurn(Mg, total, Mt);

                  float best = 1e9f,
                        Mv[9];

                  for (int h = 0; h < 4; h++) // B's horizontal: +-A or +-B, whichever the gyroscope's turn says
                  {
                     float Tb[9],
                           M[9];

                     if (!inspectVanishTriad(*vb, b.rot, &h, Tb))
                        continue;
                     for (int r = 0; r < 3; r++)
                        for (int c = 0; c < 3; c++)
                           M[3*r + c] = Ta[3*r]*Tb[3*c] + Ta[3*r + 1]*Tb[3*c + 1] + Ta[3*r + 2]*Tb[3*c + 2];

                     float d = inspectRowAngleDeg(M, Mg);

                     if (d < best)
                     {
                        best = d;
                        memcpy(Mv, M, sizeof(Mv));
                     }
                  }
                  if (best < 45.f)
                  {
                     vpGyro = best;
                     vpPure = inspectRowAngleDeg(Mv, Mp);
                     vpTwo = inspectRowAngleDeg(Mv, Mt);
                  }
               }
               printf("    pair %d-%d vanishing turn off: the gyroscope %.2f, the points' pure turn %.2f, the two-view model %.2f"
                      " degrees\n", a.index, b.index, vpGyro, vpPure, vpTwo);
               printf("    pair %d-%d gyro turn %.2f degrees: the pictures' pure turn off it %.2f along, %.2f across; points off"
                      " the gyroscope alone %.3f, off the gyroscope on the body's circle %.3f\n", a.index, b.index,
                      relAng*57.2957795f, along, across, gyroMed[0], gyroMed[1]);

               printf("    pair %d-%d (bands %u %u): %d matched, %d fit; median off a pure turn %.3f, off the two-view model"
                      " %.3f degrees; baseline (world) %.2f %.2f %.2f; %.1f s apart, the pictures turn %.2f degrees off"
                      " the gyroscope (a pure turn: %.2f)\n", a.index, b.index, (unsigned)inspectBand[a.index], (unsigned)inspectBand[b.index],
                      matched, count, rotMed, epiMed, bw.x, bw.y, bw.z, gap, turn, pureTurn);
            }
            if (met)
            {
               rotMedSum += rotMed;
               epiMedSum += epiMed;
               medPairs++;
            }
            if (!met || count < cPairMinInliers)
            {
               if (round == 0 && inspectPairDebug)
                  printf("    pair %d-%d (bands %u %u): %d matched, %d inliers\n", a.index, b.index,
                         (unsigned)inspectBand[a.index], (unsigned)inspectBand[b.index], matched, count);
               continue;
            }
            misSum += mis;
            inlierSum += count;

            // the turn in B's axes carried to the world: R_b w
            float  wr[3] = { total[0]*0.01745329f, total[1]*0.01745329f, total[2]*0.01745329f },
                  *q = &pw[((size_t)i*inspectPanoCount + j)*4u];

            q[0] = b.rot[0]*wr[0] + b.rot[3]*wr[1] + b.rot[6]*wr[2];
            q[1] = b.rot[1]*wr[0] + b.rot[4]*wr[1] + b.rot[7]*wr[2];
            q[2] = b.rot[2]*wr[0] + b.rot[5]*wr[1] + b.rot[8]*wr[2];
            q[3] = (float)count/1000.f;
            sumTurn += sqrtf(wr[0]*wr[0] + wr[1]*wr[1] + wr[2]*wr[2])*57.2957795f;
            pairs++;
         }

      // least squares, reweighed: Huber on each pair's misfit (degrees) against the current solution
      TAlloc<float> sol((size_t)n);

      memset(sol(), 0, sizeof(float)*(size_t)n);
      for (int it = 0; it < cPairIters; it++)
      {
         float diag = 0.f;

         memset(H(), 0, sizeof(float)*(size_t)n*n);
         memset(g(), 0, sizeof(float)*(size_t)n);
         for (int i = 0; i < inspectPanoCount; i++)
            for (int j = i + 1; j < inspectPanoCount; j++)
            {
               const float *q = &pw[((size_t)i*inspectPanoCount + j)*4u];

               if (q[3] <= 0.f)
                  continue;

               float mis = 0.f;

               for (int k = 0; k < 3; k++)
               {
                  float d = sol[3*j + k] - sol[3*i + k] - q[k];

                  mis += d*d;
               }
               mis = sqrtf(mis)*57.2957795f;

               float wt = q[3]*(mis <= cPairHuberDeg ? 1.f : cPairHuberDeg/mis);

               for (int k = 0; k < 3; k++)
               {
                  H[(size_t)(3*i + k)*n + 3*i + k] += wt;
                  H[(size_t)(3*j + k)*n + 3*j + k] += wt;
                  H[(size_t)(3*i + k)*n + 3*j + k] -= wt;
                  H[(size_t)(3*j + k)*n + 3*i + k] -= wt;
                  g[3*j + k] += wt*q[k];
                  g[3*i + k] -= wt*q[k];
               }
            }
         for (int k = 0; k < n; k++)
            diag += H[(size_t)k*n + k];
         diag = n ? diag/(float)n : 1.f;
         for (int k = 0; k < n; k++)
            H[(size_t)k*n + k] += cPairGyroPull*diag + 1e-9f;
         if (!inspectPanoSolve(H(), g(), n))
            break;
         memcpy(sol(), g(), sizeof(float)*(size_t)n);
      }

      // the misfit left, then the turns applied (left, in the world, as the Gauss-Newton does)
      float left = 0.f,
            moved = 0.f,
            worst = 0.f;
      int   fits = 0;

      for (int i = 0; i < inspectPanoCount; i++)
         for (int j = i + 1; j < inspectPanoCount; j++)
         {
            const float *q = &pw[((size_t)i*inspectPanoCount + j)*4u];

            if (q[3] <= 0.f)
               continue;

            float mis = 0.f;

            for (int k = 0; k < 3; k++)
            {
               float d = sol[3*j + k] - sol[3*i + k] - q[k];

               mis += d*d;
            }
            left += sqrtf(mis)*57.2957795f;
            fits++;
         }
      for (int i = 0; i < inspectPanoCount; i++)
      {
         TPanoFrame &a = inspectPano[i];
         float       e[9],
                     old[9],
                     t = sqrtf(sol[3*i]*sol[3*i] + sol[3*i + 1]*sol[3*i + 1] + sol[3*i + 2]*sol[3*i + 2])*57.2957795f;

         inspectPanoExp(&sol[3*i], e);
         memcpy(old, a.rot, sizeof(old));
         for (int c = 0; c < 3; c++)
         {
            a.rot[3*c] = e[0]*old[3*c] + e[1]*old[3*c + 1] + e[2]*old[3*c + 2];
            a.rot[3*c + 1] = e[3]*old[3*c] + e[4]*old[3*c + 1] + e[5]*old[3*c + 2];
            a.rot[3*c + 2] = e[6]*old[3*c] + e[7]*old[3*c + 1] + e[8]*old[3*c + 2];
         }
         moved += t;
         worst = fmaxf(worst, t);
      }
      printf("  panorama room %lu: pairs round %d: %d pairs by their points (%d inliers each, their own misfit %.3f"
             " degrees), turn asked %.2f degrees on average, misfit left %.3f; frames turned %.2f on average, worst %.2f"
             " degrees; %d pairs tried, %d points matched each\n", (unsigned long)room, round + 1, pairs,
             pairs ? inlierSum/pairs : 0, pairs ? misSum/(float)pairs : 0.f, pairs ? sumTurn/(float)pairs : 0.f,
             fits ? left/(float)fits : 0.f, inspectPanoCount ? moved/(float)inspectPanoCount : 0.f, worst, tried,
             tried ? matchSum/tried : 0);
      printf("  panorama room %lu: pairs round %d: median misfit of the points, a pure turn %.3f degrees, the two-view"
             " model (turn + baseline) %.3f degrees (%d pairs)\n", (unsigned long)room, round + 1,
             medPairs ? rotMedSum/(float)medPairs : 0.f, medPairs ? epiMedSum/(float)medPairs : 0.f, medPairs);
   }
}

/*--------------------------------------------------------------------------------
   What every frame shows at the same pixel is the camera's, not the room's (vignetting, the lens's
   shading, the exposure's own falloff): at a coarse level it outweighs the scene and pulls the
   neighbors toward no turn at all. Each level's mean over all frames (same size) is that fixed
   pattern; it is taken out, the level renormalized and its edge threshold measured again.
  --------------------------------------------------------------------------------*/
static void inspectPanoFixedPattern(DWORD room)
{
   if (!inspectPanoFixedOn)
      return;
   for (int l = 0; l < inspectPanoLevels; l++)
   {
      int  w = inspectPano[0].w[l],
           h = inspectPano[0].h[l],
           used = 0;
      bool same = true;

      for (int i = 1; i < inspectPanoCount; i++)
         same = same && inspectPano[i].w[l] == w && inspectPano[i].h[l] == h;
      if (!same)
         continue;

      TAlloc<float> mean((size_t)w*h);
      float         sq = 0.f;

      memset(mean(), 0, sizeof(float)*(size_t)w*h);
      for (int i = 0; i < inspectPanoCount; i++)
      {
         const float *hp = inspectPano[i].hp[l]();

         for (size_t k = 0; k < (size_t)w*h; k++)
            mean[k] += hp[k];
         used++;
      }
      for (size_t k = 0; k < (size_t)w*h; k++)
      {
         mean[k] /= (float)used;
         sq += mean[k]*mean[k];
      }
      for (int i = 0; i < inspectPanoCount; i++)
      {
         TPanoFrame &p = inspectPano[i];
         float      *hp = p.hp[l]();
         float       s = 0.f,
                     gsum = 0.f;

         for (size_t k = 0; k < (size_t)w*h; k++)
         {
            hp[k] -= mean[k];
            s += hp[k]*hp[k];
         }

         float sd = sqrtf(fmaxf(s/(float)(w*h), 1e-6f));

         for (size_t k = 0; k < (size_t)w*h; k++)
            hp[k] /= sd;
         for (int y = 1; y + 1 < h; y++)
            for (int x = 1; x + 1 < w; x++)
               gsum += fabsf(hp[(size_t)y*w + x + 1] - hp[(size_t)y*w + x - 1]) + fabsf(hp[(size_t)(y + 1)*w + x] - hp[(size_t)(y - 1)*w + x]);
         p.edge[l] = gsum/(float)((w - 2)*(h - 2));
      }
      printf("  panorama room %lu: level 1/%d, the camera's fixed pattern (mean of %d frames) RMS %.3f of a frame's, taken out\n",
             (unsigned long)room, inspectPanoShrink << l, used, sqrtf(sq/(float)(w*h)));
   }
}

//--------------------------------------------------------------------------------
// Registers the room's center-spin frames (see above); false when too few
static bool inspectPanoRegister(LPCSTR sessionDir, DWORD room, const DWORD *frameInfo, int frames)
{
   TSessionReader reader;
   TRecordView    v;
   int            index = 0;

   for (int i = 0; i < inspectPanoCount; i++)
      for (int l = 0; l < inspectPanoLevels; l++)
         inspectPano[i].hp[l] = NULL;
   for (int i = 0; i < inspectPanoCount; i++)
      for (int l = 0; l < 3; l++)
         inspectPano[i].gray[l] = NULL;
   inspectPanoCount = 0;
   memset(inspectPanoHas, 0, sizeof(inspectPanoHas));
   if (!reader.Open(sessionDir))
      return false;
   inspectPoseCount = 0;
   while (reader.Next(v) && inspectPanoCount < inspectPanoMax)
   {
      TImageRecord img;
      TPoseRecord  pose;

      if (v.type == rtPose && inspectPoseCount < inspectPoseMax && pose.Decode(v.payload, v.length))
      {
         const float *pm = pose.cameraToWorld.m;
         float       *pr = inspectPoseRot[inspectPoseCount];

         pr[0] = pm[0];
         pr[1] = pm[1];
         pr[2] = pm[2];
         pr[3] = pm[4];
         pr[4] = pm[5];
         pr[5] = pm[6];
         pr[6] = pm[8];
         pr[7] = pm[9];
         pr[8] = pm[10];
         inspectPoseNs[inspectPoseCount++] = v.stampNs;
         continue;
      }
      if (v.type != rtImage || !img.Decode(v.payload, v.length))
         continue;

      int f = index++;

      if (f >= frames || f >= inspectMaxFrames || frameInfo[f]/inspectStations != room || frameInfo[f]%inspectStations != 0u)
         continue;
      if (inspectIsSuperseded(inspectStamp[f]) || inspectKind[f] != (BYTE)skCenter)
         continue;

      TAlloc<BYTE> bgr((size_t)img.width*img.height*3u);

      if (!inspectDecode(img.pixels, img.pixelBytes, img.width, img.height, true, bgr))
         continue;

      TPanoFrame  &p = inspectPano[inspectPanoCount++];
      const float *m = img.cameraToWorld.m;
      float        r[9] = { m[0], m[1], m[2], m[4], m[5], m[6], m[8], m[9], m[10] };

      p.index = f;
      p.stamp = v.stampNs;
      p.fx = img.intr.fx*inspectFocalScale;
      p.fy = img.intr.fy*inspectFocalScale;
      p.cx = img.intr.cx;
      p.cy = img.intr.cy;
      memcpy(p.rot, r, sizeof(r));
      memcpy(p.gyro, r, sizeof(r));
      p.shift[0] = p.shift[1] = p.shift[2] = 0.f;
      inspectPanoPyramid(bgr(), img.width, img.height, p);
      inspectPointGrays(bgr(), img.width, img.height, p);
   }
   if (inspectPanoCount < 2)
      return false;
   inspectPanoFixedPattern(room);
   if (inspectDtScan)
      inspectPanoDtScan(room);
   if (inspectPoseDtSec != 0.f) // every frame takes the gyroscope's attitude that far from its picture's stamp
      for (int i = 0; i < inspectPanoCount; i++)
         if (inspectPoseAt(inspectPano[i].stamp, inspectPoseDtSec, inspectPano[i].rot))
            memcpy(inspectPano[i].gyro, inspectPano[i].rot, sizeof(inspectPano[i].rot));
   if (inspectVpSolveOn)
      inspectPanoVanishSolve(room);
   else
      inspectPanoPlumb(room); // the plumb first: the registration then only restores the neighbors' agreement
   if (inspectPairsOn) // the wide basin first; the Gauss-Newton then only from 1/16, within its pixel
   {
      inspectPanoPairs(room);
      inspectPanoGN(room, 1);
   }
   else
      inspectPanoGN(room, inspectPanoTop >= 0 && inspectPanoTop < inspectPanoLevels ? inspectPanoTop : inspectPanoLevels - 1);
   if (inspectPairA >= 0)
      inspectPairScan(room);

   float worst = 0.f,
         mean = 0.f;

   for (int i = 0; i < inspectPanoCount; i++)
   {
      float a = inspectPanoAngleDeg(inspectPano[i].rot, inspectPano[i].gyro);

      memcpy(inspectPanoRot[inspectPano[i].index], inspectPano[i].rot, sizeof(inspectPano[i].rot));
      inspectPanoHas[inspectPano[i].index] = true;
      mean += a;
      worst = fmaxf(worst, a);
   }
   printf("  panorama room %lu: %d frames registered, correction to the gyroscope mean %.2f, worst %.2f degrees\n",
          (unsigned long)room, inspectPanoCount, mean/(float)inspectPanoCount, worst);

   /* each frame's own vertical (its vertical vanishing point, from the picture's lines) carried to the world by the
      gyroscope's rotation and by the registered one: its angle to the world's up is the tilt still wrong in that
      rotation (user, 2026-09-28: frame 72 rectifies not parallel - a gyroscope deviation?). To faces_vertical.csv */
   char path[sessionPathMax];
   int  seen = 0;
   float sumG = 0.f,
         sumR = 0.f,
         maxR = 0.f;

   snprintf(path, sizeof(path), "%s/faces_vertical.csv", inspectOutDir ? inspectOutDir : ".");

   FILE *csv = inspectFrameVr ? fopen(path, "wb") : NULL;

   if (csv)
      fprintf(csv, "frame,band,pitch,tiltGyroDeg,tiltRegisteredDeg,heading,upWorldX,upWorldZ,camVx,camVy,camVz,camGx,camGy,camGz,gyroGx,gyroGy,gyroGz\r\n");
   for (int i = 0; csv && i < inspectPanoCount; i++)
   {
      const TPanoFrame    &p = inspectPano[i];
      const TVanishResult &vr = inspectFrameVr[p.index];

      if (!(vr.flags & vfVertical))
         continue;

      float t[2];
      TVec3 w = {},
            v = vr.dirCam[0];

      for (int s = 0; s < 2; s++)
      {
         const float *r = s ? p.rot : p.gyro;
         float        l;

         w.x = r[0]*v.x + r[3]*v.y + r[6]*v.z;
         w.y = r[1]*v.x + r[4]*v.y + r[7]*v.z;
         w.z = r[2]*v.x + r[5]*v.y + r[8]*v.z;
         l = sqrtf(w.x*w.x + w.y*w.y + w.z*w.z);
         t[s] = l > 0.f ? acosf(fminf(1.f, fabsf(w.y)/l))*57.2957795f : NAN;
         w.x /= l;
         w.y /= l;
         w.z /= l;
      }
      if (w.y < 0.f) // the measured vertical pointing up (world and camera alike)
      {
         w.x = -w.x;
         w.y = -w.y;
         w.z = -w.z;
         v.x = -v.x;
         v.y = -v.y;
         v.z = -v.z;
      }

      TVec3 fwd = { -p.rot[6], -p.rot[7], -p.rot[8] };

      fprintf(csv, "%d,%u,%.2f,%.3f,%.3f,%.2f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f\r\n", p.index,
              (unsigned)inspectBand[p.index], asinf(-p.rot[7])*57.2957795f, t[0], t[1], geomHeadingDeg(fwd), w.x, w.z, v.x, v.y,
              v.z, p.rot[1], p.rot[4], p.rot[7], p.gyro[1], p.gyro[4], p.gyro[7]);
      sumG += t[0];
      sumR += t[1];
      maxR = fmaxf(maxR, t[1]);
      seen++;
   }
   if (csv)
   {
      fclose(csv);
      printf("  panorama room %lu: %d frames with their own vertical: tilt left by the gyroscope mean %.2f, after the"
             " registration mean %.2f, worst %.2f degrees\n", (unsigned long)room, seen, seen ? sumG/(float)seen : 0.f,
             seen ? sumR/(float)seen : 0.f, maxR);
   }
   return true;
}

/*--------------------------------------------------------------------------------
   The center spin as one panorama (--panorama; user, 2026-09-28): the frames are taken turning in
   place, one camera center, so each one maps onto the sphere of directions by its rotation alone
   (H = K R K^-1) - no depth, no parallax. Equirectangular, cPanoDegPerPx per pixel: heading 0..360
   left to right (clockwise, the walking sense), pitch +90 at the top down to -90. Every pixel takes
   each frame seeing it, weighed by how far inside that frame it lies (a feather toward the edges),
   so the overlaps blend. Placed by the gyroscope, or (refined) by inspectPanoRegister's rotations,
   blended then with a sharper feather (cPanoSharp) so each pixel leans on its most central view.
   The way back to the frames (user: a line found on the panorama maps back to its frames):
   panorama_room<N>[_refinado].csv holds each frame's rotation and pinhole, _owner.u16 the frame
   holding each pixel best.
  --------------------------------------------------------------------------------*/
static void inspectPanorama(LPCSTR sessionDir, LPCSTR outDir, DWORD room, const DWORD *frameInfo, int frames, bool refined)
{
   LPCSTR tag = refined ? "_refinado" : "";

   const int      cols = (int)(360.f/cPanoDegPerPx + 0.5f),
                  rows = cols/2;
   TAlloc<float>  acc((size_t)cols*rows*4u),
                  top((size_t)cols*rows), // the strongest weight so far at each pixel
                  colX((size_t)cols),
                  colZ((size_t)cols);
   TAlloc<WORD>   owner((size_t)cols*rows); // the frame holding each pixel best (0xFFFF none)
   TSessionReader reader;
   TRecordView    v;
   int            index = 0,
                  used = 0;
   char           path[sessionPathMax];

   if (!reader.Open(sessionDir))
      return;
   snprintf(path, sizeof(path), "%s/panorama_room%lu%s.csv", outDir, (unsigned long)room, tag);

   FILE *csv = fopen(path, "wb");

   if (!csv)
      return;
   fprintf(csv, "frame,band,bin,heading,pitch,roll,fx,fy,cx,cy,width,height,r00,r01,r02,r10,r11,r12,r20,r21,r22\r\n");
   memset(acc(), 0, (size_t)cols*rows*4u*sizeof(float));
   memset(top(), 0, (size_t)cols*rows*sizeof(float));
   memset(owner(), 0xFF, (size_t)cols*rows*sizeof(WORD));
   for (int c = 0; c < cols; c++)
   {
      float hd = ((float)c + 0.5f)*cPanoDegPerPx*0.01745329f;

      colX[c] = sinf(hd); // east
      colZ[c] = -cosf(hd); // north is -z
   }
   while (reader.Next(v))
   {
      TImageRecord img;

      if (v.type != rtImage || !img.Decode(v.payload, v.length))
         continue;

      int f = index++;

      if (f >= frames || f >= inspectMaxFrames || frameInfo[f]/inspectStations != room || frameInfo[f]%inspectStations != 0u)
         continue;
      if (inspectIsSuperseded(inspectStamp[f]) || inspectKind[f] != (BYTE)skCenter)
         continue;

      TAlloc<BYTE> bgr((size_t)img.width*img.height*3u);

      if (!inspectDecode(img.pixels, img.pixelBytes, img.width, img.height, true, bgr))
         continue;

      TMat4 pose = img.cameraToWorld;

      if (refined && inspectPanoHas[f])
      {
         const float *r = inspectPanoRot[f];

         pose.m[0] = r[0];
         pose.m[1] = r[1];
         pose.m[2] = r[2];
         pose.m[4] = r[3];
         pose.m[5] = r[4];
         pose.m[6] = r[5];
         pose.m[8] = r[6];
         pose.m[9] = r[7];
         pose.m[10] = r[8];
      }

      const float *m = pose.m;
      TVec3        fwd = pose.Forward();
      float        w = (float)img.width,
                   h = (float)img.height,
                   half = atanf(0.5f*sqrtf(w*w/(img.intr.fx*img.intr.fx) + h*h/(img.intr.fy*img.intr.fy))),
                   reach = cosf(half + 0.02f); // directions beyond the frame's half diagonal skip at once

      // world to camera rows: a panorama direction d lands on this frame at rc = R d (then the pinhole)
      fprintf(csv, "%d,%u,%u,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.2f,%lu,%lu,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\r\n", f,
              (unsigned)inspectBand[f], (unsigned)inspectBin[f], geomHeadingDeg(fwd), geomPitchDeg(fwd),
              geomRollDeg(pose), img.intr.fx, img.intr.fy, img.intr.cx, img.intr.cy, (unsigned long)img.width,
              (unsigned long)img.height, m[0], m[1], m[2], m[4], m[5], m[6], m[8], m[9], m[10]);
      for (int r = 0; r < rows; r++)
      {
         float pt = (90.f - ((float)r + 0.5f)*cPanoDegPerPx)*0.01745329f,
               cp = cosf(pt),
               sp = sinf(pt);

         for (int c = 0; c < cols; c++)
         {
            TVec3 d = { cp*colX[c], sp, cp*colZ[c] };

            if (d.x*fwd.x + d.y*fwd.y + d.z*fwd.z < reach)
               continue;

            TVec3 rc = { m[0]*d.x + m[1]*d.y + m[2]*d.z, m[4]*d.x + m[5]*d.y + m[6]*d.z, m[8]*d.x + m[9]*d.y + m[10]*d.z };

            if (rc.z > -1e-3f)
               continue;

            float su = img.intr.cx + img.intr.fx*inspectFocalScale*rc.x/-rc.z,
                  sv = img.intr.cy - img.intr.fy*inspectFocalScale*rc.y/-rc.z;
            int   iu = (int)floorf(su),
                  iv = (int)floorf(sv);

            if (iu < 0 || iv < 0 || iu + 1 >= (int)img.width || iv + 1 >= (int)img.height)
               continue;

            float du = su - (float)iu,
                  dv = sv - (float)iv,
                  wu = 1.f - fabsf(2.f*su/w - 1.f),
                  wv = 1.f - fabsf(2.f*sv/h - 1.f),
                  wgt = refined ? powf(wu*wv, cPanoSharp) : wu*wv,
                  *cell = acc() + ((size_t)r*cols + c)*4u;

            for (int ch = 0; ch < 3; ch++)
            {
               LPCBYTE q = bgr() + ((size_t)iv*img.width + iu)*3u + ch;
               float   top = (float)q[0]*(1.f - du) + (float)q[3]*du,
                       bot = (float)q[(size_t)img.width*3u]*(1.f - du) + (float)q[(size_t)img.width*3u + 3u]*du;

               cell[ch] += wgt*(top*(1.f - dv) + bot*dv);
            }
            cell[3] += wgt;
            if (wgt > top[(size_t)r*cols + c])
            {
               top[(size_t)r*cols + c] = wgt;
               owner[(size_t)r*cols + c] = (WORD)f;
            }
         }
      }
      used++;
   }
   fclose(csv);

   TFloorGrid g = {};

   g.rows = rows;
   g.cols = cols;
   g.acc = acc();
   snprintf(path, sizeof(path), "%s/panorama_room%lu%s.bmp", outDir, (unsigned long)room, tag);
   inspectFloorBMP(g, 1.f/cPanoDegPerPx, NULL, NULL, 0, path);
   snprintf(path, sizeof(path), "%s/panorama_room%lu%s_owner.u16", outDir, (unsigned long)room, tag); // rows x cols, top row first

   FILE *own = fopen(path, "wb");

   if (own)
   {
      fwrite(owner(), sizeof(WORD), (size_t)cols*rows, own);
      fclose(own);
   }
   printf("  panorama room %lu: %d center-spin frames %s, %dx%d px (%.2f degrees per pixel)\n",
          (unsigned long)room, used, refined ? "registered" : "by the gyroscope", cols, rows, cPanoDegPerPx);
}

/*--------------------------------------------------------------------------------
   The five faces from the center spin (--faces; user, 2026-09-28: the panorama's blend, sliding
   instead of turning). The spin turns in place, so every frame maps onto any plane by its rotation
   alone; each wall of the plan (from the spin point, camera at the plan's height) and the floor get
   one straight canvas, cFacePxPerM per meter: a wall seen from inside, left to right along the plan's
   clockwise order and from the ceiling down; the floor from above, N up when a face is named N
   (else the plan's u). Every canvas pixel takes each frame seeing it, weighed by its feather to the
   power cPanoSharp, so each pixel leans on its most central view; lines stay straight, ready for
   the RANSAC. Rotations from inspectPanoRegister when it ran, else the gyroscope's. Written as
   giro_<face>.bmp, with giro_<face>_owner.u16 (the frame holding each pixel best: the way back).
  --------------------------------------------------------------------------------*/
static const float cFacePxPerM = 500.f; // 2 mm per pixel

// One face canvas: pixel (c, r) center is at origin + (c + 0.5) across + (r + 0.5) down, in world meters from the camera
struct TFaceCanvas {
   char          name[16];
   TVec3         origin,
                 across,
                 down;
   int           cols,
                 rows;
   TBlock<float> acc,
                 top;
   TBlock<WORD>  owner;
};

//--------------------------------------------------------------------------------
static void inspectFaceAlloc(TFaceCanvas &fc)
{
   size_t n = (size_t)fc.cols*fc.rows;

   fc.acc = new float[n*4u];
   fc.top = new float[n];
   fc.owner = new WORD[n];
   memset(fc.acc(), 0, n*4u*sizeof(float));
   memset(fc.top(), 0, n*sizeof(float));
   memset(fc.owner(), 0xFF, n*sizeof(WORD));
}

/*--------------------------------------------------------------------------------
   Subpixel placement of the frames on the faces (user, 2026-09-28: every frame has an expected
   position - the gyroscope's - and the subpixel comes from Gauss and least squares, with four
   degrees of freedom per frame: horizontal, vertical, rotation and scale). Each frame is rectified
   alone onto every face it sees (cAlignPxPerM, as its edge magnitude: exposure drops out) and moved
   there by its own similarity about the face's middle: d(P) = (tx + a x - b y, ty + b x + a y), x y
   from the middle - a turn of the camera shifts and rolls its patch, the body's turn (the camera a
   little nearer or farther) scales it. Every overlapping pair of patches is cut into tiles; each
   tile measures the local shift s laying one patch on the other (normalized correlation of the
   edges around the pair's own shift, the peak placed by a Gaussian through its neighbors), so
   d_j(P) - d_i(P) = -s at the tile's middle P. Per face, one least squares on its patches'
   similarities, Huber-reweighted, each pulled weakly toward the expected position (d = 0).
   Measure, solve, render again, measure again: the last round's shifts tell how coherent the net
   is. Everything goes to csv (faces_pares.csv, and faces_quadros.csv at the end) - the way back from
   a face pixel to its frame.
  --------------------------------------------------------------------------------*/
enum {
   inspectPatchMax  = 640,
   inspectAlignLvl  = 4,    // the coarse level: this many fine pixels per coarse one
   inspectAlignRuns = 5,    // measure-solve rounds
   inspectTileMax   = 24000 // tile measures per round
};

static const float cAlignPxPerM = 250.f,      // 4 mm per pixel
                   cAlignEdgeFrame = 0.03f,   // a frame's own border (share of its size) stays out of the patches
                   cAlignMinOverlap = 4000.f, // fine pixels two patches share, else no pair
                   cAlignMinNcc = 0.25f,
                   cAlignCoarseSpan = 8.f,    // the pair's own shift: coarse search, coarse pixels (+-128 mm)
                   cAlignFineSpan = 3.f,      // then fine, around it
                   cAlignTilePx = 80.f,       // tile side (0.32 m)
                   cAlignTileSpan = 2.f,      // a tile's search around the pair's shift, fine pixels
                   cAlignMinTile = 1500.f,    // shared pixels a tile needs
                   cAlignLever = 500.f,       // the similarity's a b are solved as a*lever, b*lever (pixels at the lever)
                   cAlignPrior = 1e-3f,       // pull of the shift toward the expected position, share of the mean diagonal
                   cAlignPriorRS = 1e-2f,     // rotation and scale: small but known freedoms (a gyroscope 2 degrees off), pulled harder
                   cAlignMaxShiftPx = 12.f,   // a frame's shift on a face at most (5 cm), and a pair's
                   cAlignMaxScale = 0.01f,    // its scale off 1 at most
                   cAlignMaxRot = 0.0087f;    // its rotation at most (0.5 degree)

// One frame rectified alone onto one face: edge magnitude inside its bounding box (NAN where the frame does not reach)
struct TFacePatch {
   int           frame,  // index into inspectPano
                 face,
                 x0,
                 y0,
                 w,
                 h,
                 cw,     // coarse level
                 ch;
   TBlock<float> edge,
                 coarse;
};

static TFacePatch inspectPatch[inspectPatchMax];
static int        inspectPatchCount = 0;
static float      inspectSim[inspectPanoMax][layoutMaxVerts + 1][4], // each frame's similarity on each face: tx ty a b
                  inspectSimRef[layoutMaxVerts + 1][2];               // the face's middle (cAlignPxPerM pixels)

//--------------------------------------------------------------------------------
// World direction of a canvas point of a face in cAlignPxPerM pixels (camera at the origin)
static TVec3 inspectAlignPoint(const TFaceCanvas &fc, float c, float r)
{
   float ac = (c + 0.5f)/cAlignPxPerM,
         dn = (r + 0.5f)/cAlignPxPerM;
   TVec3 d = { fc.origin.x + ac*fc.across.x + dn*fc.down.x, fc.origin.y + ac*fc.across.y + dn*fc.down.y,
               fc.origin.z + ac*fc.across.z + dn*fc.down.z };

   return d;
}

//--------------------------------------------------------------------------------
// A canvas point pulled back through a patch's similarity: the frame's content shown at P was rectified at P - d(P)
static void inspectSimBack(const float *sim, const float *ref, float &c, float &r)
{
   float rx = c - ref[0],
         ry = r - ref[1];

   c -= sim[0] + sim[2]*rx - sim[3]*ry;
   r -= sim[1] + sim[3]*rx + sim[2]*ry;
}

//--------------------------------------------------------------------------------
// Gray of a frame at a world direction (bilinear), NAN outside its picture less its own border
static float inspectAlignSample(const TImageRecord &img, LPCBYTE bgr, const float *m, const float *shift, const TVec3 &P)
{
   TVec3 c = inspectCenter(m, shift),
         d = { P.x - c.x, P.y - c.y, P.z - c.z }, // from this camera's own center
         rc = { m[0]*d.x + m[1]*d.y + m[2]*d.z, m[3]*d.x + m[4]*d.y + m[5]*d.z, m[6]*d.x + m[7]*d.y + m[8]*d.z };

   if (rc.z > -1e-3f)
      return NAN;

   float su = img.intr.cx + img.intr.fx*inspectFocalScale*rc.x/-rc.z,
         sv = img.intr.cy - img.intr.fy*inspectFocalScale*rc.y/-rc.z,
         bu = cAlignEdgeFrame*(float)img.width,
         bv = cAlignEdgeFrame*(float)img.height;

   if (su < bu || sv < bv || su >= (float)img.width - 1.f - bu || sv >= (float)img.height - 1.f - bv)
      return NAN;

   int   iu = (int)su,
         iv = (int)sv;
   float du = su - (float)iu,
         dv = sv - (float)iv,
         g[4];

   for (int k = 0; k < 4; k++)
   {
      LPCBYTE q = bgr + ((size_t)(iv + k/2)*img.width + iu + k%2)*3u;

      g[k] = 0.114f*(float)q[0] + 0.587f*(float)q[1] + 0.299f*(float)q[2];
   }
   return (g[0]*(1.f - du) + g[1]*du)*(1.f - dv) + (g[2]*(1.f - du) + g[3]*du)*dv;
}

//--------------------------------------------------------------------------------
// One frame onto one face as an edge patch, moved by its similarity; false when it does not see the face
static bool inspectAlignPatch(const TImageRecord &img, LPCBYTE bgr, const float *m, const float *shift, const TFaceCanvas &fc,
                              int cols, int rows,
                              const float *sim, const float *ref, TFacePatch &p)
{
   int lo[2] = { cols, rows },
       hi[2] = { -1, -1 };

   for (int r = 0; r < rows; r += 8)
      for (int c = 0; c < cols; c += 8)
      {
         float u = (float)c,
               v = (float)r;

         inspectSimBack(sim, ref, u, v);
         if (!isnan(inspectAlignSample(img, bgr, m, shift, inspectAlignPoint(fc, u, v))))
         {
            lo[0] = c < lo[0] ? c : lo[0];
            lo[1] = r < lo[1] ? r : lo[1];
            hi[0] = c > hi[0] ? c : hi[0];
            hi[1] = r > hi[1] ? r : hi[1];
         }
      }
   if (hi[0] < 0)
      return false;
   p.x0 = lo[0] - 8 > 0 ? lo[0] - 8 : 0;
   p.y0 = lo[1] - 8 > 0 ? lo[1] - 8 : 0;
   p.w = (hi[0] + 8 < cols ? hi[0] + 8 : cols - 1) - p.x0 + 1;
   p.h = (hi[1] + 8 < rows ? hi[1] + 8 : rows - 1) - p.y0 + 1;

   TAlloc<float> gray((size_t)p.w*p.h);
   float        *edge = new float[(size_t)p.w*p.h];

   for (int r = 0; r < p.h; r++)
      for (int c = 0; c < p.w; c++)
      {
         float u = (float)(p.x0 + c),
               v = (float)(p.y0 + r);

         inspectSimBack(sim, ref, u, v);
         gray[(size_t)r*p.w + c] = inspectAlignSample(img, bgr, m, shift, inspectAlignPoint(fc, u, v));
      }
   for (int r = 0; r < p.h; r++)
      for (int c = 0; c < p.w; c++)
      {
         float e = NAN;

         if (r > 0 && c > 0 && r + 1 < p.h && c + 1 < p.w)
         {
            const float *g = gray() + (size_t)r*p.w + c;
            float        gx = (g[1 - p.w] + 2.f*g[1] + g[1 + p.w]) - (g[-1 - p.w] + 2.f*g[-1] + g[-1 + p.w]),
                         gy = (g[p.w - 1] + 2.f*g[p.w] + g[p.w + 1]) - (g[-p.w - 1] + 2.f*g[-p.w] + g[-p.w + 1]);

            e = sqrtf(gx*gx + gy*gy); // NAN when any neighbor is outside
         }
         edge[(size_t)r*p.w + c] = e;
      }
   p.edge = edge;
   p.cw = p.w/inspectAlignLvl;
   p.ch = p.h/inspectAlignLvl;

   float *coarse = new float[(size_t)(p.cw > 0 ? p.cw : 1)*(p.ch > 0 ? p.ch : 1)];

   for (int r = 0; r < p.ch; r++)
      for (int c = 0; c < p.cw; c++)
      {
         float s = 0.f;
         int   n = 0;

         for (int k = 0; k < inspectAlignLvl*inspectAlignLvl; k++)
         {
            float e = edge[(size_t)(r*inspectAlignLvl + k/inspectAlignLvl)*p.w + c*inspectAlignLvl + k%inspectAlignLvl];

            if (!isnan(e))
            {
               s += e;
               n++;
            }
         }
         coarse[(size_t)r*p.cw + c] = n == inspectAlignLvl*inspectAlignLvl ? s/(float)n : NAN;
      }
   p.coarse = coarse;
   return true;
}

/*--------------------------------------------------------------------------------
   Correlation of patch a with patch b shifted by (sx, sy) - b's pixel = a's pixel + shift - at a
   level (lv 1 fine, else coarse), over a's local rows [r0, r1) and columns [c0, c1); count of the
   shared pixels
  --------------------------------------------------------------------------------*/
static float inspectAlignNcc(const TFacePatch &a, const TFacePatch &b, int lv, int sx, int sy, int step, int r0, int r1, int c0,
                             int c1, int &count)
{
   const float *ea = lv == 1 ? a.edge() : a.coarse(),
               *eb = lv == 1 ? b.edge() : b.coarse();
   int          wa = lv == 1 ? a.w : a.cw,
                ha = lv == 1 ? a.h : a.ch,
                wb = lv == 1 ? b.w : b.cw,
                hb = lv == 1 ? b.h : b.ch,
                ox = (b.x0 - a.x0)/lv,  // b's origin in a's pixels
                oy = (b.y0 - a.y0)/lv;
   float        s1 = 0.f,
                s2 = 0.f,
                s11 = 0.f,
                s22 = 0.f,
                s12 = 0.f;

   count = 0;
   for (int r = r0 > 0 ? r0 : 0; r < (r1 < ha ? r1 : ha); r += step)
   {
      int rb = r + sy - oy;

      if (rb < 0 || rb >= hb)
         continue;
      for (int c = c0 > 0 ? c0 : 0; c < (c1 < wa ? c1 : wa); c += step)
      {
         int cb = c + sx - ox;

         if (cb < 0 || cb >= wb)
            continue;

         float u = ea[(size_t)r*wa + c],
               v = eb[(size_t)rb*wb + cb];

         if (isnan(u) || isnan(v))
            continue;
         s1 += u;
         s2 += v;
         s11 += u*u;
         s22 += v*v;
         s12 += u*v;
         count++;
      }
   }
   if (count < 50)
      return -1.f;

   float n = (float)count,
         va = s11 - s1*s1/n,
         vb = s22 - s2*s2/n;

   return va > 0.f && vb > 0.f ? (s12 - s1*s2/n)/sqrtf(va*vb) : -1.f;
}

//--------------------------------------------------------------------------------
// Subpixel offset of a peak from its two neighbors: a Gaussian through three positive samples, else a parabola
static float inspectAlignPeak(float before, float at, float after)
{
   if (before > 0.f && at > 0.f && after > 0.f)
   {
      float l0 = logf(before),
            l1 = logf(at),
            l2 = logf(after),
            den = l0 - 2.f*l1 + l2;

      if (den < -1e-6f)
         return fmaxf(-0.5f, fminf(0.5f, 0.5f*(l0 - l2)/den));
   }
   return inspectPeak(before, at, after);
}

/*--------------------------------------------------------------------------------
   Best shift of b against a over a square window around (cx, cy) at a level, in a's region; the
   subpixel shift in fine pixels, false when the peak is weak or on the window's edge
  --------------------------------------------------------------------------------*/
static bool inspectAlignSearch(const TFacePatch &a, const TFacePatch &b, int lv, int cx, int cy, int span, int step, int r0, int r1,
                               int c0, int c1, float minCount, float &sx, float &sy, float &ncc, int &count)
{
   float grid[17][17],
         best = -2.f;
   int   bx = 0,
         by = 0,
         n = 0;

   if (span > 8)
      span = 8;
   for (int y = -span; y <= span; y++)
      for (int x = -span; x <= span; x++)
      {
         float c = inspectAlignNcc(a, b, lv, cx + x, cy + y, step, r0, r1, c0, c1, n);

         grid[y + span][x + span] = c;
         if (c > best)
         {
            best = c;
            bx = x;
            by = y;
            count = n;
         }
      }
   if (best < cAlignMinNcc || (float)(count*step*step*lv*lv) < minCount || bx == -span || bx == span || by == -span || by == span)
      return false;

   int gx = bx + span,
       gy = by + span;

   sx = ((float)(cx + bx) + inspectAlignPeak(grid[gy][gx - 1], grid[gy][gx], grid[gy][gx + 1]))*(float)lv;
   sy = ((float)(cy + by) + inspectAlignPeak(grid[gy - 1][gx], grid[gy][gx], grid[gy + 1][gx]))*(float)lv;
   ncc = best;
   return true;
}

//--------------------------------------------------------------------------------
// Renders every registered frame's patches onto the faces with the current rotations and similarities
static void inspectAlignRender(LPCSTR sessionDir, const TFaceCanvas *fc, int faces, const int *cols, const int *rows)
{
   TSessionReader reader;
   TRecordView    v;
   int            index = 0;

   for (int i = 0; i < inspectPatchCount; i++)
   {
      inspectPatch[i].edge = NULL;
      inspectPatch[i].coarse = NULL;
   }
   inspectPatchCount = 0;
   if (!reader.Open(sessionDir))
      return;
   while (reader.Next(v))
   {
      TImageRecord img;

      if (v.type != rtImage || !img.Decode(v.payload, v.length))
         continue;

      int f = index++,
          k = -1;

      for (int i = 0; i < inspectPanoCount; i++)
         k = inspectPano[i].index == f ? i : k;
      if (k < 0)
         continue;

      TAlloc<BYTE> bgr((size_t)img.width*img.height*3u);

      if (!inspectDecode(img.pixels, img.pixelBytes, img.width, img.height, true, bgr))
         continue;
      for (int i = 0; i < faces && inspectPatchCount < inspectPatchMax; i++)
      {
         TFacePatch &p = inspectPatch[inspectPatchCount];

         p.frame = k;
         p.face = i;
         if (inspectAlignPatch(img, bgr(), inspectPano[k].rot, inspectPano[k].shift, fc[i], cols[i], rows[i], inspectSim[k][i],
                               inspectSimRef[i], p))
            inspectPatchCount++;
      }
   }
}

// One tile measure: which patches, where (cAlignPxPerM canvas pixels), the shift and its weight
struct TAlignTile {
   int   a,       // patch indices
         b;
   float x,
         y,
         sx,
         sy,
         ncc,
         weight,
         residual;
};

static TAlignTile inspectTile[inspectTileMax];
static float      inspectRejected[inspectPatchMax*8][5]; // pairs left out for a shift beyond cAlignMaxShiftPx: a, b, px, py, ncc
static int        inspectRejectedCount = 0;

//--------------------------------------------------------------------------------
// The tiles of every overlapping pair of patches on the faces; returns their count
static int inspectAlignTiles(void)
{
   int tiles = 0,
       span = (int)cAlignTileSpan,
       side = (int)cAlignTilePx;

   inspectRejectedCount = 0;
   for (int i = 0; i < inspectPatchCount; i++)
      for (int j = i + 1; j < inspectPatchCount; j++)
      {
         const TFacePatch &a = inspectPatch[i],
                          &b = inspectPatch[j];

         if (a.face != b.face || a.frame == b.frame)
            continue;

         int ox = a.x0 > b.x0 ? a.x0 : b.x0,
             oy = a.y0 > b.y0 ? a.y0 : b.y0,
             ex = a.x0 + a.w < b.x0 + b.w ? a.x0 + a.w : b.x0 + b.w,
             ey = a.y0 + a.h < b.y0 + b.h ? a.y0 + a.h : b.y0 + b.h,
             n = 0;

         if (ex <= ox || ey <= oy || (float)((ex - ox)*(ey - oy)) < cAlignMinOverlap)
            continue;

         // the pair's own shift first: coarse, then fine around it
         float px,
               py,
               pn;

         if (!inspectAlignSearch(a, b, inspectAlignLvl, 0, 0, (int)cAlignCoarseSpan, 1, 0, 1 << 20, 0, 1 << 20, cAlignMinOverlap,
                                 px, py, pn, n)
             || !inspectAlignSearch(a, b, 1, (int)lroundf(px), (int)lroundf(py), (int)cAlignFineSpan, 2, 0, 1 << 20, 0, 1 << 20,
                                    cAlignMinOverlap, px, py, pn, n))
            continue;

         /* the frames already stand on the scene's own axes (plumb and heading): what is left between two of them is
            small. A pair farther apart is locked on something off the plane (furniture shifted by parallax) or on a
            repeating pattern (a row of doors) - it measures nothing (user, 2026-09-28: small but known freedoms) */
         if (sqrtf(px*px + py*py) > cAlignMaxShiftPx && inspectRejectedCount < inspectPatchMax*8)
         {
            float *q = inspectRejected[inspectRejectedCount++];

            q[0] = (float)i;
            q[1] = (float)j;
            q[2] = px;
            q[3] = py;
            q[4] = pn;
         }
         if (sqrtf(px*px + py*py) > cAlignMaxShiftPx)
            continue;
         for (int ty = oy; ty + side/2 <= ey && tiles < inspectTileMax; ty += side)
            for (int tx = ox; tx + side/2 <= ex && tiles < inspectTileMax; tx += side)
            {
               TAlignTile &t = inspectTile[tiles];
               float       sx,
                           sy,
                           nc;

               if (!inspectAlignSearch(a, b, 1, (int)lroundf(px), (int)lroundf(py), span + 1, 1, ty - a.y0, ty + side - a.y0,
                                       tx - a.x0, tx + side - a.x0, cAlignMinTile, sx, sy, nc, n))
                  continue;
               t.a = i;
               t.b = j;
               t.x = (float)tx + 0.5f*(float)(side < ex - tx ? side : ex - tx);
               t.y = (float)ty + 0.5f*(float)(side < ey - ty ? side : ey - ty);
               t.sx = sx;
               t.sy = sy;
               t.ncc = nc;
               t.weight = nc*nc*sqrtf((float)n);
               t.residual = 0.f;
               tiles++;
            }
      }
   return tiles;
}

//--------------------------------------------------------------------------------
static int inspectFloatOrder(LPCVOID a, LPCVOID b)
{
   float x = *(const float *)a,
         y = *(const float *)b;

   return x < y ? -1 : (x > y ? 1 : 0);
}

//--------------------------------------------------------------------------------
// Quantile of the first n values (sorted in place)
static float inspectQuantile(float *v, int n, float q)
{
   if (n <= 0)
      return 0.f;
   qsort(v, (size_t)n, sizeof(float), inspectFloatOrder);

   int k = (int)(q*(float)n);

   return v[k < n ? k : n - 1];
}

/*--------------------------------------------------------------------------------
   One face's least squares: its patches' similarities from its tiles, d_j(P) - d_i(P) = -s,
   pulled toward d = 0 and reweighted by Cauchy on a scale from the median residual (a tile whose
   peak lies - furniture off the plane, a repeating pattern - fades out instead of pulling); adds
   the solution to inspectSim. Returns the tiles' median residual (fine pixels) and sets their
   residuals; medBefore is the median shift measured, p90 the residuals' 90th percentile.
  --------------------------------------------------------------------------------*/
static float inspectAlignSolveFace(int face, int tiles, float &medBefore, float &p90, int &used)
{
   int var[inspectPatchMax],
       m = 0;

   for (int i = 0; i < inspectPatchCount; i++)
      var[i] = inspectPatch[i].face == face ? m++ : -1;

   const int     n = 4*m;
   TAlloc<float> H((size_t)(n > 0 ? n : 1)*(n > 0 ? n : 1)),
                 g((size_t)(n > 0 ? n : 1)),
                 x((size_t)(n > 0 ? n : 1)),
                 q((size_t)(tiles > 0 ? tiles : 1));
   float         knee = 1e9f,
                 rms = 0.f;

   used = 0;
   medBefore = 0.f;
   p90 = 0.f;
   for (int k = 0; k < tiles; k++)
      if (inspectPatch[inspectTile[k].a].face == face)
         q[used++] = sqrtf(inspectTile[k].sx*inspectTile[k].sx + inspectTile[k].sy*inspectTile[k].sy);
   if (!used || !n)
      return 0.f;
   medBefore = inspectQuantile(q(), used, 0.5f);
   for (int irls = 0; irls < 6; irls++)
   {
      float diag = 0.f;
      int   cnt = 0;

      memset(H(), 0, sizeof(float)*(size_t)n*n);
      memset(g(), 0, sizeof(float)*(size_t)n);
      for (int k = 0; k < tiles; k++)
      {
         const TAlignTile &t = inspectTile[k];

         if (inspectPatch[t.a].face != face)
            continue;

         int   oi = 4*var[t.a],
               oj = 4*var[t.b];
         float rx = (t.x - inspectSimRef[face][0])/cAlignLever,
               ry = (t.y - inspectSimRef[face][1])/cAlignLever,
               wt = t.weight*(irls == 0 ? 1.f : 1.f/(1.f + (t.residual/knee)*(t.residual/knee))),
               rowX[4] = { 1.f, 0.f, rx, -ry },
               rowY[4] = { 0.f, 1.f, ry, rx };

         for (int ax = 0; ax < 2; ax++)
         {
            const float *A = ax ? rowY : rowX;
            float        s = -(ax ? t.sy : t.sx);

            for (int p = 0; p < 4; p++)
            {
               for (int q = 0; q < 4; q++)
               {
                  float aa = wt*A[p]*A[q];

                  H[(size_t)(oj + p)*n + oj + q] += aa;
                  H[(size_t)(oi + p)*n + oi + q] += aa;
                  H[(size_t)(oj + p)*n + oi + q] -= aa;
                  H[(size_t)(oi + p)*n + oj + q] -= aa;
               }
               g[oj + p] += wt*A[p]*s;
               g[oi + p] -= wt*A[p]*s;
            }
         }
      }
      for (int k = 0; k < n; k++)
         diag += H[(size_t)k*n + k];
      diag = diag/(float)n + 1e-6f;
      for (int k = 0; k < n; k++)
         H[(size_t)k*n + k] += (k%4 < 2 ? cAlignPrior : cAlignPriorRS)*diag;
      memcpy(x(), g(), sizeof(float)*(size_t)n);
      if (!inspectPanoSolve(H(), x(), n))
         return -1.f;
      cnt = 0;
      for (int k = 0; k < tiles; k++)
      {
         TAlignTile &t = inspectTile[k];

         if (inspectPatch[t.a].face != face)
            continue;

         const float *pi = &x[4*var[t.a]],
                     *pj = &x[4*var[t.b]];
         float        rx = (t.x - inspectSimRef[face][0])/cAlignLever,
                      ry = (t.y - inspectSimRef[face][1])/cAlignLever,
                      ex = (pj[0] - pi[0]) + (pj[2] - pi[2])*rx - (pj[3] - pi[3])*ry + t.sx,
                      ey = (pj[1] - pi[1]) + (pj[3] - pi[3])*rx + (pj[2] - pi[2])*ry + t.sy;

         t.residual = sqrtf(ex*ex + ey*ey);
         q[cnt++] = t.residual;
      }
      rms = inspectQuantile(q(), used, 0.5f);
      p90 = inspectQuantile(q(), used, 0.9f);
      knee = fmaxf(0.5f, 2.4f*1.4826f*rms); // Cauchy scale from the median residual
   }
   for (int i = 0; i < inspectPatchCount; i++)
   {
      if (var[i] < 0)
         continue;

      float *s = inspectSim[inspectPatch[i].frame][face];

      s[0] += x[4*var[i]];
      s[1] += x[4*var[i] + 1];
      s[2] += x[4*var[i] + 2]/cAlignLever;
      s[3] += x[4*var[i] + 3]/cAlignLever;

      // small freedoms stay small: the shift, the scale and the rotation held to their bounds
      float t = sqrtf(s[0]*s[0] + s[1]*s[1]);

      if (t > cAlignMaxShiftPx)
      {
         s[0] *= cAlignMaxShiftPx/t;
         s[1] *= cAlignMaxShiftPx/t;
      }
      s[2] = fmaxf(-cAlignMaxScale, fminf(cAlignMaxScale, s[2]));
      s[3] = fmaxf(-cAlignMaxRot, fminf(cAlignMaxRot, s[3]));
   }
   return rms;
}

//--------------------------------------------------------------------------------
// The frames' similarities on every face from their tiles (see above), rounds of measure and solve
static void inspectFaceAlign(LPCSTR sessionDir, LPCSTR outDir, DWORD room, const TFaceCanvas *fc, int faces)
{
   int  cols[layoutMaxVerts + 1],
        rows[layoutMaxVerts + 1];
   char path[sessionPathMax];

   memset(inspectSim, 0, sizeof(inspectSim));
   if (inspectPanoCount < 2)
      return;
   for (int i = 0; i < faces; i++)
   {
      cols[i] = (int)((float)fc[i].cols*cAlignPxPerM/cFacePxPerM);
      rows[i] = (int)((float)fc[i].rows*cAlignPxPerM/cFacePxPerM);
      inspectSimRef[i][0] = 0.5f*(float)cols[i];
      inspectSimRef[i][1] = 0.5f*(float)rows[i];
   }
   snprintf(path, sizeof(path), "%s/faces_pares.csv", outDir);

   FILE *log = fopen(path, "wb"); // every tile measure of every round: the least squares' whole input and outcome

   if (log)
      fprintf(log, "round,face,frameA,frameB,tileX,tileY,shiftX,shiftY,ncc,weight,residual\r\n");
   for (int run = 0; run < inspectAlignRuns; run++)
   {
      inspectAlignRender(sessionDir, fc, faces, cols, rows);

      int tiles = inspectAlignTiles();

      for (int f = 0; f < faces; f++)
      {
         float before = 0.f,
               p90 = 0.f,
               after;
         int   used = 0;

         after = inspectAlignSolveFace(f, tiles, before, p90, used);
         printf("  faces room %lu: round %d, %s: %d tiles, median shift %.2f px -> residual median %.2f px, p90 %.2f px"
                " (%.1f -> %.1f mm)\n", (unsigned long)room, run + 1, fc[f].name, used, before, after, p90,
                before*1000.f/cAlignPxPerM, after*1000.f/cAlignPxPerM);
      }
      for (int k = 0; log && k < inspectRejectedCount; k++) // the pairs left out, as tile -1: their own shift only
      {
         const float *q = inspectRejected[k];

         fprintf(log, "%d,%s,%d,%d,-1,-1,%.3f,%.3f,%.4f,0,0\r\n", run + 1, fc[inspectPatch[(int)q[0]].face].name,
                 inspectPano[inspectPatch[(int)q[0]].frame].index, inspectPano[inspectPatch[(int)q[1]].frame].index, q[2], q[3], q[4]);
      }
      for (int k = 0; log && k < tiles; k++)
      {
         const TAlignTile &t = inspectTile[k];

         fprintf(log, "%d,%s,%d,%d,%.1f,%.1f,%.3f,%.3f,%.4f,%.3f,%.3f\r\n", run + 1, fc[inspectPatch[t.a].face].name,
                 inspectPano[inspectPatch[t.a].frame].index, inspectPano[inspectPatch[t.b].frame].index, t.x, t.y, t.sx,
                 t.sy, t.ncc, t.weight, t.residual);
      }
   }
   if (log)
      fclose(log);
}

//--------------------------------------------------------------------------------
// One frame onto one face canvas, moved by its similarity there (sim, ref: cAlignPxPerM pixels)
static void inspectFaceSplat(const TImageRecord &img, LPCBYTE bgr, const float *m, const float *shift, int f, const float *sim,
                             const float *ref,
                             TFaceCanvas &fc)
{
   float w = (float)img.width,
         h = (float)img.height,
         half = atanf(0.5f*sqrtf(w*w/(img.intr.fx*img.intr.fx) + h*h/(img.intr.fy*img.intr.fy))),
         reach = cosf(half + 0.02f);
   TVec3 fwd = { -m[6], -m[7], -m[8] },
         cen = inspectCenter(m, shift);

   for (int r = 0; r < fc.rows; r++)
      for (int c = 0; c < fc.cols; c++)
      {
         float ua = ((float)c + 0.5f)*cAlignPxPerM/cFacePxPerM - 0.5f,
               va = ((float)r + 0.5f)*cAlignPxPerM/cFacePxPerM - 0.5f;

         inspectSimBack(sim, ref, ua, va);

         TVec3 d = inspectAlignPoint(fc, ua, va);

         d.x -= cen.x; // from this camera's own center
         d.y -= cen.y;
         d.z -= cen.z;

         float dl = sqrtf(d.x*d.x + d.y*d.y + d.z*d.z);

         if (dl < 1e-3f || (d.x*fwd.x + d.y*fwd.y + d.z*fwd.z) < reach*dl)
            continue;

         TVec3 rc = { m[0]*d.x + m[1]*d.y + m[2]*d.z, m[3]*d.x + m[4]*d.y + m[5]*d.z, m[6]*d.x + m[7]*d.y + m[8]*d.z };

         if (rc.z > -1e-3f)
            continue;

         float su = img.intr.cx + img.intr.fx*inspectFocalScale*rc.x/-rc.z,
               sv = img.intr.cy - img.intr.fy*inspectFocalScale*rc.y/-rc.z;
         int   iu = (int)floorf(su),
               iv = (int)floorf(sv);

         if (iu < 0 || iv < 0 || iu + 1 >= (int)img.width || iv + 1 >= (int)img.height)
            continue;

         float  du = su - (float)iu,
                dv = sv - (float)iv,
                wgt = powf((1.f - fabsf(2.f*su/w - 1.f))*(1.f - fabsf(2.f*sv/h - 1.f)), cPanoSharp);
         size_t at = (size_t)r*fc.cols + c;
         float *cell = fc.acc() + at*4u;

         for (int ch = 0; ch < 3; ch++)
         {
            LPCBYTE q = bgr + ((size_t)iv*img.width + iu)*3u + ch;
            float   t = (float)q[0]*(1.f - du) + (float)q[3]*du,
                    b = (float)q[(size_t)img.width*3u]*(1.f - du) + (float)q[(size_t)img.width*3u + 3u]*du;

            cell[ch] += wgt*(t*(1.f - dv) + b*dv);
         }
         cell[3] += wgt;
         if (wgt > fc.top[at])
         {
            fc.top[at] = wgt;
            fc.owner[at] = (WORD)f;
         }
      }
}

/*--------------------------------------------------------------------------------
   Baseline zero of the faces (user, 2026-09-28): on every wall the ceiling line is horizontal on
   the top edge and both corners are vertical on the side edges - until then the picture is not
   rectified. The ceiling crease is the guide ("premise zero"): each wall is composed with a margin
   around it (cCreaseMarginM past the corners, above the ceiling and below the floor) and its
   crease found there - the highest near-horizontal Sobel line across the wall's middle (RANSAC; the
   lines below it are the molding's own edges). Every crease point is a ray from the camera that
   meets the ceiling's plane (ceiling above the camera): a point of the wall's true line in the plan,
   whatever plane the canvas assumed. A line through them per wall, the corners where neighbors
   cross, and the faces are rebuilt on those corners: the crease on the top edge, the corners on the
   sides. inspectCreaseCheck then measures what the rebuilt walls show: the crease's tilt and height,
   each corner's lean and place.
  --------------------------------------------------------------------------------*/
static const float cCreaseMarginM = 0.25f,  // measure canvases reach this far around the wall
                   cCreaseBandM = 0.5f,     // the crease is sought this far below the assumed ceiling (and the margin above)
                   cCreaseMaxDropM = 0.2f,  // a line found farther below the assumed top is no crease: the wall stays
                   cCreaseEndM = 0.35f,     // the wall's ends stay out of the crease (the other wall starts there)
                   cCreaseSlope = 0.12f,    // near-horizontal: at most this slope
                   cCreaseTolPx = 2.5f,     // RANSAC inlier distance
                   cCreaseShare = 0.15f,    // a line needs this share of the columns
                   cCreaseStrong = 0.35f,   // the wanted line is chosen among those holding this share of the best-held one
                   cCreaseEdgeMin = 12.f,   // a line's edge points: at least this gray step (the ceiling's texture stays out)
                   cCornerBandM = 0.2f;     // a corner is sought this far on both sides of its expected place

//--------------------------------------------------------------------------------
// The measure canvas of a wall: the wall plus a margin all around, at cFacePxPerM
static void inspectCreaseCanvas(const TFaceCanvas &w, float ceilM, TFaceCanvas &m)
{
   float mg = cCreaseMarginM;

   snprintf(m.name, sizeof(m.name), "%s", w.name);
   m.origin.x = w.origin.x - mg*w.across.x - mg*w.down.x;
   m.origin.y = w.origin.y - mg*w.across.y - mg*w.down.y;
   m.origin.z = w.origin.z - mg*w.across.z - mg*w.down.z;
   m.across = w.across;
   m.down = w.down;
   m.cols = w.cols + (int)(2.f*mg*cFacePxPerM + 0.5f);
   m.rows = (int)((ceilM + 2.f*mg)*cFacePxPerM + 0.5f);
}

//--------------------------------------------------------------------------------
// Every registered frame composed onto the canvases as they are (no similarities)
static void inspectComposeRaw(LPCSTR sessionDir, TFaceCanvas *fc, int count)
{
   TSessionReader reader;
   TRecordView    v;
   int            index = 0;
   const float    none[4] = { 0.f, 0.f, 0.f, 0.f },
                  ref[2] = { 0.f, 0.f };

   for (int i = 0; i < count; i++)
      inspectFaceAlloc(fc[i]);
   if (!reader.Open(sessionDir))
      return;
   while (reader.Next(v))
   {
      TImageRecord img;

      if (v.type != rtImage || !img.Decode(v.payload, v.length))
         continue;

      int f = index++,
          k = -1;

      for (int i = 0; i < inspectPanoCount; i++)
         k = inspectPano[i].index == f ? i : k;
      if (k < 0)
         continue;

      TAlloc<BYTE> bgr((size_t)img.width*img.height*3u);

      if (!inspectDecode(img.pixels, img.pixelBytes, img.width, img.height, true, bgr))
         continue;
      for (int i = 0; i < count; i++)
         inspectFaceSplat(img, bgr(), inspectPano[k].rot, inspectPano[k].shift, f, none, ref, fc[i]);
   }
}

//--------------------------------------------------------------------------------
// Gray of a composed canvas (NAN where no frame reached)
static void inspectCanvasGray(const TFaceCanvas &fc, float *gray)
{
   for (size_t i = 0; i < (size_t)fc.cols*fc.rows; i++)
   {
      const float *c = fc.acc() + i*4u;

      gray[i] = c[3] > 0.f ? (0.114f*c[0] + 0.587f*c[1] + 0.299f*c[2])/c[3] : NAN;
   }
}

/*--------------------------------------------------------------------------------
   The strongest edge points of a canvas region along one axis: vertical = false scans each column
   for the rows where the vertical gradient peaks (horizontal lines), true scans each row for the
   columns where the horizontal gradient peaks (vertical lines). Points go to px, py; returns their
   count.
  --------------------------------------------------------------------------------*/
static int inspectCanvasEdges(const float *gray, int cols, int rows, bool vertical, int c0, int c1, int r0, int r1, float *px,
                              float *py, int max)
{
   int n = 0,
       lines = vertical ? r1 - r0 : c1 - c0;

   for (int l = 0; l < lines && n < max; l += 2)
   {
      int   lo = vertical ? c0 : r0,
            hi = vertical ? c1 : r1;
      float peak = 0.f;

      for (int pass = 0; pass < 2; pass++) // the line's strongest step, then its peaks above a third of it
         for (int t = lo + 1; t + 1 < hi && n < max; t++)
         {
            int   x = vertical ? t : c0 + l,
                  y = vertical ? r0 + l : t;
            float g[3];
            bool  ok = x > 0 && y > 0 && x + 1 < cols && y + 1 < rows;

            for (int k = -1; ok && k <= 1; k++)
            {
               int   xa = vertical ? x + k - 1 : x,
                     ya = vertical ? y : y + k - 1,
                     xb = vertical ? x + k + 1 : x,
                     yb = vertical ? y : y + k + 1;
               float a = xa >= 0 && ya >= 0 && xa < cols && ya < rows ? gray[(size_t)ya*cols + xa] : NAN,
                     b = xb >= 0 && yb >= 0 && xb < cols && yb < rows ? gray[(size_t)yb*cols + xb] : NAN;

               g[k + 1] = fabsf(b - a);
               ok = !isnan(g[k + 1]);
            }
            if (!ok)
               continue;
            if (pass == 0)
            {
               peak = fmaxf(peak, g[1]);
               continue;
            }
            if (g[1] < cCreaseEdgeMin || g[1] < peak/3.f || g[1] < g[0] || g[1] < g[2])
               continue;

            float o = inspectPeak(g[0], g[1], g[2]);

            px[n] = (float)x + (vertical ? o : 0.f);
            py[n] = (float)y + (vertical ? 0.f : o);
            n++;
         }
   }
   return n;
}

/*--------------------------------------------------------------------------------
   Lines of a point cloud by RANSAC: along x (horizontal = true: y = a + b x) or along y (x = a +
   b y), slope within maxSlope. The best-held line first; then, among the lines holding cCreaseShare
   of span (the cloud's extent along the axis, in points every 2 px) and cCreaseStrong of the best,
   the one nearest to "want" (its value at mid) - the highest
   crease wants -1e4. Refined by least squares on its inliers. False when none holds.
  --------------------------------------------------------------------------------*/
static bool inspectCanvasLine(const float *px, const float *py, int n, bool horizontal, float maxSlope, float mid, float want,
                              float span, float &a, float &b, int &inliers)
{
   const float *u = horizontal ? px : py,
               *v = horizontal ? py : px;
   float        bestScore = 1e18f,
                need = cCreaseShare*span/2.f;
   bool         found = false;
   int          most = 0;

   inliers = 0;
   for (int phase = 0; phase < 2; phase++) // the best-held line first; then the wanted one among the strong ones
   {
      unsigned seed = 12345u;

      if (phase == 1)
         need = fmaxf(need, cCreaseStrong*(float)most);
      for (int it = 0; it < 3000 && n >= 2; it++)
      {
         seed = seed*1103515245u + 12345u;

         int i = (int)((seed >> 8)%(unsigned)n);

         seed = seed*1103515245u + 12345u;

         int j = (int)((seed >> 8)%(unsigned)n);

         if (fabsf(u[j] - u[i]) < 0.2f*span)
            continue;

         float sb = (v[j] - v[i])/(u[j] - u[i]),
               sa = v[i] - sb*u[i];
         int   in = 0;

         if (fabsf(sb) > maxSlope)
            continue;
         for (int k = 0; k < n; k++)
            in += fabsf(v[k] - (sa + sb*u[k])) <= cCreaseTolPx ? 1 : 0;
         if (phase == 0)
         {
            most = in > most ? in : most;
            continue;
         }
         if ((float)in < need)
            continue;

         float score = fabsf(sa + sb*mid - want) - 1e-3f*(float)in; // nearest to the wanted place, then the strongest

         if (score < bestScore)
         {
            bestScore = score;
            a = sa;
            b = sb;
            found = true;
         }
      }
   }
   if (!found)
      return false;
   for (int pass = 0; pass < 2; pass++)
   {
      float s1 = 0.f,
            su = 0.f,
            sv = 0.f,
            suu = 0.f,
            suv = 0.f;

      inliers = 0;
      for (int k = 0; k < n; k++)
         if (fabsf(v[k] - (a + b*u[k])) <= cCreaseTolPx)
         {
            s1 += 1.f;
            su += u[k];
            sv += v[k];
            suu += u[k]*u[k];
            suv += u[k]*v[k];
            inliers++;
         }

      float den = s1*suu - su*su;

      if (inliers < 2 || fabsf(den) < 1e-6f)
         break;
      b = (s1*suv - su*sv)/den;
      a = (sv - b*su)/s1;
   }
   return true;
}

/*--------------------------------------------------------------------------------
   The walls from their ceiling creases (see above): fc[0..walls) are rebuilt on the corners; the
   floor canvas fc[walls] is bounded by them. False (the canvases untouched) when a wall's crease
   is not found.
  --------------------------------------------------------------------------------*/
static bool inspectCreaseWalls(LPCSTR sessionDir, DWORD room, TFaceCanvas *fc, int walls, float cam, float ceil)
{
   float       hc = ceil - cam,
               lineP[layoutMaxVerts][2],
               lineD[layoutMaxVerts][2],
               lineW[layoutMaxVerts], // how much a wall's own direction weighs: its crease points times its length squared
               lineT[layoutMaxVerts];
   TFaceCanvas mc[layoutMaxVerts];

   for (int i = 0; i < walls; i++)
      inspectCreaseCanvas(fc[i], ceil, mc[i]);
   inspectComposeRaw(sessionDir, mc, walls);
   for (int i = 0; i < walls; i++)
   {
      const TFaceCanvas &m = mc[i];
      TAlloc<float>      gray((size_t)m.cols*m.rows),
                         px(200000u),
                         py(200000u);
      int                mg = (int)(cCreaseMarginM*cFacePxPerM),
                         end = (int)(cCreaseEndM*cFacePxPerM),
                         c0 = mg + end,
                         c1 = m.cols - mg - end,
                         r1 = mg + (int)(cCreaseBandM*cFacePxPerM),
                         n,
                         in = 0;
      float              a = 0.f,
                         b = 0.f;

      inspectCanvasGray(m, gray());
      if (inspectOutDir) // the measure canvas as composed, for a look at what the crease search sees
      {
         TFloorGrid g = {};
         char       path[sessionPathMax];

         g.rows = m.rows;
         g.cols = m.cols;
         g.acc = m.acc();
         snprintf(path, sizeof(path), "%s/medida_%s.bmp", inspectOutDir, m.name);
         inspectFloorBMP(g, cFacePxPerM, NULL, NULL, 0, path);
      }
      n = inspectCanvasEdges(gray(), m.cols, m.rows, false, c0, c1, 0, r1, px(), py(), 200000);
      if (!inspectCanvasLine(px(), py(), n, true, cCreaseSlope, 0.5f*(float)(c0 + c1), -1e4f, (float)(c1 - c0), a, b, in))
      {
         printf("  faces room %lu: %s: no ceiling crease found (%d edge points)\n", (unsigned long)room, m.name, n);
         return false;
      }

      /* the crease's points onto the ceiling's plane: the wall's true line in the plan (least squares, principal axis).
         The rays leave from where the cameras that saw this wall stood - the mean center of the frames facing it (the
         body's sphere puts them ~0.36 m toward the wall: from the pivot, 101732's walls came out 0.9 m too far) */
      TVec3 nw = { m.across.z, 0.f, -m.across.x },
            eye = { 0.f, 0.f, 0.f };
      float eyeN = 0.f;

      if (nw.x*m.origin.x + nw.z*m.origin.z < 0.f)
      {
         nw.x = -nw.x;
         nw.z = -nw.z;
      }
      for (int f = 0; f < inspectPanoCount; f++)
      {
         TVec3 fh = inspectHorizontalForward(inspectPano[f].rot);

         if (fh.x*nw.x + fh.z*nw.z < 0.7071f)
            continue;
         eye.x += inspectPano[f].shift[0];
         eye.y += inspectPano[f].shift[1];
         eye.z += inspectPano[f].shift[2];
         eyeN += 1.f;
      }
      if (eyeN > 0.f)
      {
         eye.x /= eyeN;
         eye.y /= eyeN;
         eye.z /= eyeN;
      }

      float sx = 0.f,
            sz = 0.f,
            sxx = 0.f,
            szz = 0.f,
            sxz = 0.f;
      int   k = 0;

      for (int c = c0; c < c1; c += 4)
      {
         float r = a + b*(float)c,
               ac = ((float)c + 0.5f)/cFacePxPerM,
               dn = (r + 0.5f)/cFacePxPerM;
         TVec3 d = { m.origin.x + ac*m.across.x + dn*m.down.x, m.origin.y + ac*m.across.y + dn*m.down.y,
                     m.origin.z + ac*m.across.z + dn*m.down.z };

         if (d.y - eye.y < 1e-3f)
            continue;

         float s = (hc - eye.y)/(d.y - eye.y),
               x = eye.x + (d.x - eye.x)*s,
               z = eye.z + (d.z - eye.z)*s;

         sx += x;
         sz += z;
         sxx += x*x;
         szz += z*z;
         sxz += x*z;
         k++;
      }
      if (k < 10)
         return false;

      float fk = (float)k,
            cxx = sxx/fk - (sx/fk)*(sx/fk),
            czz = szz/fk - (sz/fk)*(sz/fk),
            cxz = sxz/fk - (sx/fk)*(sz/fk),
            th = 0.5f*atan2f(2.f*cxz, cxx - czz);

      lineP[i][0] = sx/fk;
      lineP[i][1] = sz/fk;
      lineT[i] = th;
      lineW[i] = (float)in*(0.5f*(cxx + czz) + sqrtf(0.25f*(cxx - czz)*(cxx - czz) + cxz*cxz));
      printf("  faces room %lu: %s: crease %d points (row %.1f + %.5f col), %.2f degrees on the assumed wall, %.0f mm %s its top\n",
             (unsigned long)room, m.name, in, a, b, atanf(b)*57.2957795f,
             fabsf(a + b*0.5f*(float)(c0 + c1) - (float)mg)*1000.f/cFacePxPerM,
             a + b*0.5f*(float)(c0 + c1) < (float)mg ? "above" : "below");

      /* a "crease" far below the wall's top is something under the ceiling (101732: the window wall's crease had
         fallen off the canvas, the window's top was taken for it and the wall pushed 0.86 m): the wall stays put */
      float drop = (a + b*0.5f*(float)(c0 + c1) - (float)mg)/cFacePxPerM;

      if (drop > cCreaseMaxDropM)
      {
         float half = 0.5f*(float)fc[i].cols/cFacePxPerM;

         printf("  faces room %lu: %s: %.0f mm below the top is no ceiling crease: the wall kept\n", (unsigned long)room,
                m.name, drop*1000.f);
         lineP[i][0] = fc[i].origin.x + half*fc[i].across.x;
         lineP[i][1] = fc[i].origin.z + half*fc[i].across.z;
         lineT[i] = atan2f(fc[i].across.z, fc[i].across.x);
         lineW[i] = 0.f;
      }
   }

   /* the room's square (user's plan: a rectangle): one heading for all walls, each wall at it or at right angles,
      the walls with long creases weighing most (a crease half hidden by a wardrobe only gives its wall's place);
      the mean of 4 x heading over the walls, then each wall's own place from its points */
   float s4 = 0.f,
         c4 = 0.f,
         th0;

   for (int i = 0; i < walls; i++)
   {
      s4 += lineW[i]*sinf(4.f*lineT[i]);
      c4 += lineW[i]*cosf(4.f*lineT[i]);
   }
   th0 = 0.25f*atan2f(s4, c4);

   /* the lines hold the turns (--vp-solve): the square's heading is theirs - hundreds of frames' horizontals - not
      the four creases' (101732: the creases turned the walls 4 degrees off the frames, the cabinet parallel to N came
      out a trapezoid). A heading phi from north lies at phi - 90 in this plane's angle: the same mod 90 */
   if (inspectVpSolveOn && inspectFrameVr)
   {
      float fs = 0.f,
            fcs = 0.f;

      for (int k = 0; k < inspectPanoCount; k++)
      {
         const TPanoFrame    &p = inspectPano[k];
         const TVanishResult &vr = inspectFrameVr[p.index];

         for (int d = 1; d <= 2; d++)
            if (vr.flags & (d == 1 ? vfAxisA : vfAxisB))
            {
               TVec3 h = vr.dirCam[d];
               float phi = atan2f(p.rot[0]*h.x + p.rot[3]*h.y + p.rot[6]*h.z, -(p.rot[2]*h.x + p.rot[5]*h.y + p.rot[8]*h.z)),
                     w = (float)vr.support[d];

               fs += w*sinf(4.f*phi);
               fcs += w*cosf(4.f*phi);
            }
      }
      if (fs != 0.f || fcs != 0.f)
      {
         printf("  faces room %lu: the square's heading from the lines %.2f (the creases' own %.2f) degrees\n",
                (unsigned long)room, 0.25f*atan2f(fs, fcs)*57.2957795f, th0*57.2957795f);
         th0 = 0.25f*atan2f(fs, fcs);
      }
   }
   for (int i = 0; i < walls; i++)
   {
      float q = roundf((lineT[i] - th0)/1.5707963f),
            t = th0 + q*1.5707963f;

      printf("  faces room %lu: %s: its own crease %.2f degrees off the room's square (weight %.0f%%)\n", (unsigned long)room,
             mc[i].name, (lineT[i] - t)*57.2957795f, 100.f*lineW[i]/fmaxf(1e-6f, sqrtf(s4*s4 + c4*c4)));
      lineD[i][0] = cosf(t);
      lineD[i][1] = sinf(t);
      if (lineD[i][0]*mc[i].across.x + lineD[i][1]*mc[i].across.z < 0.f) // along the wall's own sense
      {
         lineD[i][0] = -lineD[i][0];
         lineD[i][1] = -lineD[i][1];
      }
   }

   // the corners: wall i starts where wall i - 1 crosses it
   float cx[layoutMaxVerts],
         cz[layoutMaxVerts];

   for (int i = 0; i < walls; i++)
   {
      int   p = (i + walls - 1)%walls;
      float x,
            z;

      if (!inspectCross(lineP[p][0], lineP[p][1], lineD[p][0], lineD[p][1], lineP[i][0], lineP[i][1], lineD[i][0], lineD[i][1],
                        x, z))
         return false;
      cx[i] = x;
      cz[i] = z;
   }
   for (int i = 0; i < walls; i++)
   {
      int   q = (i + 1)%walls;
      float dx = cx[q] - cx[i],
            dz = cz[q] - cz[i],
            len = sqrtf(dx*dx + dz*dz),
            turn = acosf(fmaxf(-1.f, fminf(1.f, lineD[i][0]*lineD[q][0] + lineD[i][1]*lineD[q][1])))*57.2957795f,
            swing = atan2f(fc[i].across.x*dz - fc[i].across.z*dx, fc[i].across.x*dx + fc[i].across.z*dz)*57.2957795f,
            moved = sqrtf((cx[i] - fc[i].origin.x)*(cx[i] - fc[i].origin.x) + (cz[i] - fc[i].origin.z)*(cz[i] - fc[i].origin.z));

      printf("  faces room %lu: %s: turned %.2f degrees from the assumed wall, its start moved %.3f m\n", (unsigned long)room,
             fc[i].name, swing, moved);
      fc[i].origin.x = cx[i];
      fc[i].origin.y = hc;
      fc[i].origin.z = cz[i];
      fc[i].across.x = dx/len;
      fc[i].across.y = 0.f;
      fc[i].across.z = dz/len;
      fc[i].cols = (int)(len*cFacePxPerM + 0.5f);
      fc[i].rows = (int)(ceil*cFacePxPerM + 0.5f);
      printf("  faces room %lu: %s: %.3f m long, corner to the next %.2f degrees\n", (unsigned long)room, fc[i].name, len,
             turn);
   }

   // the floor canvas: bounded by the corners along its own axes
   TFaceCanvas &fl = fc[walls];
   TVec3        up = { -fl.down.x, 0.f, -fl.down.z };
   float        xLo = 1e9f,
                xHi = -1e9f,
                yLo = 1e9f,
                yHi = -1e9f;

   for (int i = 0; i < walls; i++)
   {
      float x = cx[i]*fl.across.x + cz[i]*fl.across.z,
            y = cx[i]*up.x + cz[i]*up.z;

      xLo = fminf(xLo, x);
      xHi = fmaxf(xHi, x);
      yLo = fminf(yLo, y);
      yHi = fmaxf(yHi, y);
   }
   fl.origin.x = xLo*fl.across.x + yHi*up.x;
   fl.origin.z = xLo*fl.across.z + yHi*up.z;
   fl.cols = (int)((xHi - xLo)*cFacePxPerM + 0.5f);
   fl.rows = (int)((yHi - yLo)*cFacePxPerM + 0.5f);
   return true;
}

/*--------------------------------------------------------------------------------
   What the rebuilt walls show (the baseline zero): on each wall's measure canvas, the crease's tilt
   and its distance from the top edge, and each corner's lean and distance from its side edge (the
   corner as the near-vertical line nearest the edge). Printed and written to faces_baliza.csv.
  --------------------------------------------------------------------------------*/
static void inspectCreaseCheck(LPCSTR sessionDir, LPCSTR outDir, DWORD room, const TFaceCanvas *fc, int walls, float ceil)
{
   TFaceCanvas mc[layoutMaxVerts];
   char        path[sessionPathMax];

   for (int i = 0; i < walls; i++)
      inspectCreaseCanvas(fc[i], ceil, mc[i]);
   inspectComposeRaw(sessionDir, mc, walls);
   snprintf(path, sizeof(path), "%s/faces_baliza.csv", outDir);

   FILE *csv = fopen(path, "wb");

   if (csv)
      fprintf(csv, "face,creaseDeg,creaseMm,leftDeg,leftMm,rightDeg,rightMm\r\n");
   for (int i = 0; i < walls; i++)
   {
      const TFaceCanvas &m = mc[i];
      TAlloc<float>      gray((size_t)m.cols*m.rows),
                         px(400000u),
                         py(400000u);
      int                mg = (int)(cCreaseMarginM*cFacePxPerM),
                         end = (int)(cCreaseEndM*cFacePxPerM),
                         band = (int)(cCornerBandM*cFacePxPerM),
                         n,
                         in = 0;
      float              a = 0.f,
                         b = 0.f,
                         out[6] = { NAN, NAN, NAN, NAN, NAN, NAN };

      inspectCanvasGray(m, gray());
      if (inspectOutDir) // the rebuilt wall's measure canvas, for a look at the baseline
      {
         TFloorGrid g = {};
         char       name[sessionPathMax];

         g.rows = m.rows;
         g.cols = m.cols;
         g.acc = m.acc();
         snprintf(name, sizeof(name), "%s/baliza_%s.bmp", inspectOutDir, m.name);
         inspectFloorBMP(g, cFacePxPerM, NULL, NULL, 0, name);
      }
      n = inspectCanvasEdges(gray(), m.cols, m.rows, false, mg + end, m.cols - mg - end, 0,
                             mg + (int)(cCreaseBandM*cFacePxPerM), px(), py(), 400000);
      if (inspectCanvasLine(px(), py(), n, true, cCreaseSlope, 0.5f*(float)m.cols, -1e4f, (float)(m.cols - 2*mg - 2*end), a, b,
                            in))
      {
         printf("  faces room %lu: %s check: crease row %.1f + %.5f col, %d points\n", (unsigned long)room, m.name, a, b, in);
         out[0] = atanf(b)*57.2957795f;
         out[1] = (a + b*0.5f*(float)m.cols - (float)mg)*1000.f/cFacePxPerM;
      }
      for (int s = 0; s < 2; s++) // the corners: near-vertical lines by the side edges, below the crease, above the floor
      {
         int edge = s ? m.cols - mg : mg,
             r0 = mg + (int)(0.3f*cFacePxPerM),
             r1 = m.rows - mg - (int)(0.3f*cFacePxPerM);

         n = inspectCanvasEdges(gray(), m.cols, m.rows, true, edge - band, edge + band, r0, r1, px(), py(), 400000);
         if (inspectCanvasLine(px(), py(), n, false, cCreaseSlope, 0.5f*(float)(r0 + r1), (float)edge, (float)(r1 - r0), a, b, in))
         {
            out[2 + 2*s] = atanf(b)*57.2957795f;
            out[3 + 2*s] = (a + b*0.5f*(float)(r0 + r1) - (float)edge)*1000.f/cFacePxPerM;
         }
      }
      printf("  faces room %lu: %s baseline: crease %.2f degrees, %.1f mm off the top; left corner %.2f degrees, %.1f mm;"
             " right corner %.2f degrees, %.1f mm\n", (unsigned long)room, m.name, out[0], out[1], out[2], out[3], out[4],
             out[5]);
      if (csv)
         fprintf(csv, "%s,%.3f,%.1f,%.3f,%.1f,%.3f,%.1f\r\n", m.name, out[0], out[1], out[2], out[3], out[4], out[5]);
   }
   if (csv)
      fclose(csv);
}

/*--------------------------------------------------------------------------------
   How upright a rectified frame is (user, 2026-09-28: orthogonal axes - many vertical vectors with a
   vanishing point in common set the verticals, the ceiling's line the top line). The straight
   near-vertical and near-horizontal edges of the canvas (RANSAC one after another, each at least
   cLeanMinPx long): out[0] the verticals' common lean at the camera's foot (footX; a turn about the
   plane's normal), out[1] how many verticals agree on it, out[2] their convergence as the plane's
   tilt about its horizontal axis (degrees), out[3] the mean slope of the horizontals, out[4] how
   many, out[5] the ceiling line's slope (degrees; eyeRow: the canvas row at
   the eye's height, NAN off a wall; creaseRow: the ceiling's row; dist: the plane's distance), out[6]
   how many told it.
  --------------------------------------------------------------------------------*/
static float       inspectLean[inspectPanoMax][layoutMaxVerts + 1][6]; // per face: lean, its lines, ceiling slope, its lines, tilt, ceiling over distance
static const float cLeanMinPx = 80.f,   // a line's support along it (canvas pixels)
                   cLeanSlope = 0.15f,
                   cLeanEyeM = 0.2f,    // horizontals this near the eye's height tell no heading
                   cLeanCreaseM = 0.12f, // the heading from the lines this near the ceiling's line (its crease and molding)
                   cLeanAgree = 0.012f;  // a vertical agrees with the common vanishing point within this slope (0.7 degree)

enum {
   inspectLeanLines = 16 // lines per direction kept for the common vanishing point
};

//--------------------------------------------------------------------------------
static void inspectRectLean(const float *acc, int cols, int rows, float ppm, float eyeRow, float creaseRow, float footX,
                            float dist, float *out)
{
   TAlloc<float> gray((size_t)cols*rows),
                 px(300000u),
                 py(300000u);
   TAlloc<BYTE>  used(300000u);

   for (size_t i = 0; i < (size_t)cols*rows; i++)
   {
      const float *c = acc + i*4u;

      gray[i] = c[3] > 0.f ? 0.114f*c[0] + 0.587f*c[1] + 0.299f*c[2] : NAN;
   }
   for (int k = 0; k < 7; k++)
      out[k] = 0.f;
   for (int dir = 0; dir < 2; dir++) // 0: verticals (x = a + b y), 1: horizontals (y = a + b x)
   {
      int   n = inspectCanvasEdges(gray(), cols, rows, dir == 0, 0, cols, 0, rows, px(), py(), 300000);
      float sw = 0.f,
            sa = 0.f,
            sx = 0.f,
            sxx = 0.f,
            sxa = 0.f,
            sp = 0.f,  // the ceiling line's slope, weighed by its points
            spw = 0.f;
      float lb[inspectLeanLines],  // the verticals: slope, place across (px), points
            lx[inspectLeanLines],
            lw[inspectLeanLines];
      int   lines = 0,
            heads = 0;

      memset(used(), 0, (size_t)(n > 0 ? n : 1));
      for (int round = 0; round < 12; round++)
      {
         const float *u = dir == 0 ? py() : px(),
                     *v = dir == 0 ? px() : py();
         unsigned     seed = 777u + (unsigned)round;
         float        ba = 0.f,
                      bb = 0.f;
         int          best = 0;

         for (int it = 0; it < 2000 && n >= 2; it++)
         {
            seed = seed*1103515245u + 12345u;

            int i = (int)((seed >> 8)%(unsigned)n);

            seed = seed*1103515245u + 12345u;

            int j = (int)((seed >> 8)%(unsigned)n);

            if (used[i] || used[j] || fabsf(u[j] - u[i]) < 0.5f*cLeanMinPx)
               continue;

            float b = (v[j] - v[i])/(u[j] - u[i]),
                  a = v[i] - b*u[i];
            int   in = 0;

            if (fabsf(b) > cLeanSlope)
               continue;
            for (int k = 0; k < n; k++)
               in += !used[k] && fabsf(v[k] - (a + b*u[k])) <= 1.5f ? 1 : 0;
            if (in > best)
            {
               best = in;
               ba = a;
               bb = b;
            }
         }
         if ((float)best*2.f < cLeanMinPx) // points every 2 px along the line
            break;

         // least squares on the inliers, then they are used up
         float s1 = 0.f,
               su = 0.f,
               sv = 0.f,
               suu = 0.f,
               suv = 0.f;

         for (int k = 0; k < n; k++)
            if (!used[k] && fabsf(v[k] - (ba + bb*u[k])) <= 1.5f)
            {
               s1 += 1.f;
               su += u[k];
               sv += v[k];
               suu += u[k]*u[k];
               suv += u[k]*v[k];
               used[k] = 1u;
            }

         float den = s1*suu - su*su;

         if (den > 1e-6f)
         {
            bb = (s1*suv - su*sv)/den;
            ba = (sv - bb*su)/s1;
         }

         float deg = atanf(bb)*57.2957795f,
               at = (ba + bb*su/s1)/ppm; // where the line lies across the canvas (m)

         sw += s1;
         sa += s1*deg;
         sx += s1*at;
         sxx += s1*at*at;
         sxa += s1*at*deg;
         if (dir == 0 && lines < inspectLeanLines)
         {
            lb[lines] = bb;
            lx[lines] = at*ppm;
            lw[lines] = s1;
         }
         lines++;

         /* the ceiling's line (user, 2026-09-28: orthogonal axes - the ceiling sets the top line, the verticals the
            plumb): the lines within cLeanCreaseM of it, their slope averaged; what turn it asks (heading and the turn
            about the wall's normal both slope it) is settled in inspectLeanPlumb */
         float h = (eyeRow - at*ppm)/ppm;

         if (dir == 1 && !isnan(eyeRow) && fabsf(h) >= cLeanEyeM && fabsf(at*ppm - creaseRow) <= cLeanCreaseM*ppm)
         {
            sp += s1*atanf(bb); // the ceiling line's own slope: the turn it asks is set in the plumb's least squares
            spw += s1;
            heads++;
         }
      }
      if (sw <= 0.f)
         continue;
      if (dir == 0)
      {
         /* the verticals' common vanishing point (user, 2026-09-28: "many vertical vectors with a vanishing point in
            common" - any upright structure, not one piece of furniture): a line at x (canvas px) has slope
            b = b0 + b1 (x - footX) - b0 the lean at the camera's foot (a turn about the plane's normal), b1 the
            convergence (the plane tilted about its horizontal axis by tau = -b1 D ppm). The model most lines agree
            with (pairs of lines, or one line alone), then least squares on them; lines off it (a leaning box) stay out */
         int   m = lines < inspectLeanLines ? lines : inspectLeanLines,
               bestIn = 0;
         float b0 = 0.f,
               b1 = 0.f,
               bestW = 0.f;

         for (int i = 0; i < m; i++) // the lean most verticals share (their vanishing point at infinity, straight up)
         {
            float w = 0.f;
            int   in = 0;

            for (int k = 0; k < m; k++)
               if (fabsf(lb[k] - lb[i]) <= cLeanAgree)
               {
                  w += lw[k];
                  in++;
               }
            if (w > bestW)
            {
               bestW = w;
               bestIn = in;
               b0 = lb[i];
            }
         }

         float s1 = 0.f,
               su = 0.f,
               sv = 0.f,
               suu = 0.f,
               suv = 0.f;

         for (int k = 0; k < m; k++)
            if (fabsf(lb[k] - (b0 + b1*(lx[k] - footX))) <= cLeanAgree)
            {
               float x = lx[k] - footX;

               s1 += lw[k];
               su += lw[k]*x;
               sv += lw[k]*lb[k];
               suu += lw[k]*x*x;
               suv += lw[k]*x*lb[k];
            }

         float den = s1*suu - su*su;

         // the agreeing verticals' mean lean; their convergence, as a check only (a wall at an angle sees it as lean)
         b1 = bestIn >= 2 && den > 1e-3f*s1*s1*cLeanMinPx*cLeanMinPx ? (s1*suv - su*sv)/den : 0.f;
         out[0] = s1 > 0.f ? atanf(sv/s1)*57.2957795f : 0.f;
         out[1] = (float)bestIn;
         out[2] = atanf(-b1*dist*ppm)*57.2957795f;
      }
      else
      {
         out[3] = sa/sw;
         out[4] = (float)lines;
         out[5] = spw > 0.f ? sp/spw*57.2957795f : 0.f;
         out[6] = (float)heads;
      }
   }
}

/*--------------------------------------------------------------------------------
   Each frame rectified alone onto each face (user, 2026-09-28: "the pure rectification, before any
   panorama, one image per associated frame; no crop, the homography only"): a folder per face
   (outDir/<face>/), in it <frame>.bmp - the whole frame carried onto the face's plane by its
   registered rotation (no blend, no similarity), on a canvas bounding its own footprint, however
   far past the face (up to cFrameRectReachM beyond it: rays grazing the plane would run to the
   horizon). cFrameRectPxPerM per pixel. quadros.csv there gives each image's place: its top-left
   corner in the face's own canvas pixels (at that resolution, may be negative) and its size.
  --------------------------------------------------------------------------------*/
static const float cFrameRectPxPerM = 250.f, // 4 mm per pixel
                   cFrameRectReachM = 1.f;

//--------------------------------------------------------------------------------
static void inspectFaceFrames(LPCSTR sessionDir, LPCSTR outDir, DWORD room, const TFaceCanvas *fc, int faces)
{
   TSessionReader reader;
   TRecordView    v;
   int            index = 0,
                  written = 0;
   FILE          *list[layoutMaxVerts + 1];
   char           path[sessionPathMax];

   memset(inspectLean, 0, sizeof(inspectLean));
   for (int i = 0; i < faces; i++)
   {
      list[i] = NULL;
      if (!outDir) // measure only: the leans land in inspectLean, nothing is written
         continue;
      snprintf(path, sizeof(path), "%s/%s", outDir, fc[i].name);
      inspectMakeDir(path);
      snprintf(path, sizeof(path), "%s/%s/quadros.csv", outDir, fc[i].name);
      list[i] = fopen(path, "wb");
      if (list[i])
         fprintf(list[i], "frame,x0,y0,cols,rows,pxPerM,share,vertDeg,vertLines,keystoneDegPerM,horizDeg,horizLines,"
                          "headingDeg,headingLines\r\n");
   }
   if (reader.Open(sessionDir))
      while (reader.Next(v))
      {
         TImageRecord img;

         if (v.type != rtImage || !img.Decode(v.payload, v.length))
            continue;

         int f = index++,
             k = -1;

         for (int i = 0; i < inspectPanoCount; i++)
            k = inspectPano[i].index == f ? i : k;
         if (k < 0)
            continue;

         TAlloc<BYTE> bgr((size_t)img.width*img.height*3u);
         const float *m = inspectPano[k].rot;
         TVec3        cen = inspectCenter(m, inspectPano[k].shift); // the camera's own center
         float        fx = img.intr.fx*inspectFocalScale,
                      fy = img.intr.fy*inspectFocalScale;

         if (!inspectDecode(img.pixels, img.pixelBytes, img.width, img.height, true, bgr))
            continue;
         for (int i = 0; i < faces; i++)
         {
            const TFaceCanvas &c = fc[i];
            TVec3              n = { c.across.y*c.down.z - c.across.z*c.down.y, c.across.z*c.down.x - c.across.x*c.down.z,
                                     c.across.x*c.down.y - c.across.y*c.down.x };
            float              D = n.x*c.origin.x + n.y*c.origin.y + n.z*c.origin.z,
                               lim[4] = { -cFrameRectReachM, -cFrameRectReachM, (float)c.cols/cFacePxPerM + cFrameRectReachM,
                                          (float)c.rows/cFacePxPerM + cFrameRectReachM },
                               box[4] = { 1e9f, 1e9f, -1e9f, -1e9f };
            int                hits = 0;

            if (D < 0.f)
            {
               n.x = -n.x;
               n.y = -n.y;
               n.z = -n.z;
               D = -D;
            }

            // the frame's border rays onto the plane: its footprint, clamped to the reach
            for (int s = 0; s < 4*64; s++)
            {
               float t = (float)(s%64)/63.f,
                     su = s/64 == 0 ? t*(float)img.width : (s/64 == 1 ? (float)img.width : (s/64 == 2 ? (1.f - t)*(float)img.width : 0.f)),
                     sv = s/64 == 0 ? 0.f : (s/64 == 1 ? t*(float)img.height : (s/64 == 2 ? (float)img.height : (1.f - t)*(float)img.height)),
                     rx = (su - img.intr.cx)/fx,
                     ry = -(sv - img.intr.cy)/fy;
               TVec3 d = { m[0]*rx + m[3]*ry - m[6], m[1]*rx + m[4]*ry - m[7], m[2]*rx + m[5]*ry - m[8] };
               float nd = n.x*d.x + n.y*d.y + n.z*d.z,
                     dl = sqrtf(d.x*d.x + d.y*d.y + d.z*d.z);

               if (nd < 0.02f*dl) // away from the plane or grazing it: toward the reach's edge
               {
                  float ux = d.x*c.across.x + d.y*c.across.y + d.z*c.across.z,
                        uy = d.x*c.down.x + d.y*c.down.y + d.z*c.down.z;

                  if (nd <= 0.f)
                     continue;
                  box[0] = fminf(box[0], ux < 0.f ? lim[0] : box[0]);
                  box[2] = fmaxf(box[2], ux > 0.f ? lim[2] : box[2]);
                  box[1] = fminf(box[1], uy < 0.f ? lim[1] : box[1]);
                  box[3] = fmaxf(box[3], uy > 0.f ? lim[3] : box[3]);
                  continue;
               }

               float tt = D/nd,
                     X[3] = { d.x*tt - c.origin.x, d.y*tt - c.origin.y, d.z*tt - c.origin.z },
                     ux = X[0]*c.across.x + X[1]*c.across.y + X[2]*c.across.z,
                     uy = X[0]*c.down.x + X[1]*c.down.y + X[2]*c.down.z;

               box[0] = fminf(box[0], ux);
               box[1] = fminf(box[1], uy);
               box[2] = fmaxf(box[2], ux);
               box[3] = fmaxf(box[3], uy);
               hits++;
            }
            if (hits < 16)
               continue;
            box[0] = fmaxf(box[0], lim[0]);
            box[1] = fmaxf(box[1], lim[1]);
            box[2] = fminf(box[2], lim[2]);
            box[3] = fminf(box[3], lim[3]);

            int x0 = (int)floorf(box[0]*cFrameRectPxPerM),
                y0 = (int)floorf(box[1]*cFrameRectPxPerM),
                cols = (int)ceilf(box[2]*cFrameRectPxPerM) - x0,
                rows = (int)ceilf(box[3]*cFrameRectPxPerM) - y0;

            if (cols < 8 || rows < 8)
               continue;

            TAlloc<float> acc((size_t)cols*rows*4u);

            memset(acc(), 0, sizeof(float)*(size_t)cols*rows*4u);
            for (int r = 0; r < rows; r++)
               for (int q = 0; q < cols; q++)
               {
                  float ac = ((float)(x0 + q) + 0.5f)/cFrameRectPxPerM,
                        dn = ((float)(y0 + r) + 0.5f)/cFrameRectPxPerM;
                  TVec3 d = { c.origin.x + ac*c.across.x + dn*c.down.x - cen.x, c.origin.y + ac*c.across.y + dn*c.down.y - cen.y,
                              c.origin.z + ac*c.across.z + dn*c.down.z - cen.z }, // from this camera's own center
                        rc = { m[0]*d.x + m[1]*d.y + m[2]*d.z, m[3]*d.x + m[4]*d.y + m[5]*d.z, m[6]*d.x + m[7]*d.y + m[8]*d.z };

                  if (rc.z > -1e-3f)
                     continue;

                  float su = img.intr.cx + fx*rc.x/-rc.z,
                        sv = img.intr.cy - fy*rc.y/-rc.z;
                  int   iu = (int)floorf(su),
                        iv = (int)floorf(sv);

                  if (iu < 0 || iv < 0 || iu + 1 >= (int)img.width || iv + 1 >= (int)img.height)
                     continue;

                  float  du = su - (float)iu,
                         dv = sv - (float)iv,
                        *cell = acc() + ((size_t)r*cols + q)*4u;

                  for (int ch = 0; ch < 3; ch++)
                  {
                     LPCBYTE p = bgr() + ((size_t)iv*img.width + iu)*3u + ch;
                     float   t = (float)p[0]*(1.f - du) + (float)p[3]*du,
                             b = (float)p[(size_t)img.width*3u]*(1.f - du) + (float)p[(size_t)img.width*3u + 3u]*du;

                     cell[ch] = t*(1.f - dv) + b*dv;
                  }
                  cell[3] = 1.f;
               }

            float faceShare = 0.f; // of the face itself
            int   inFace = 0;

            for (int r = 0; r < rows; r++)
               for (int q = 0; q < cols; q++)
                  if (acc()[((size_t)r*cols + q)*4u + 3u] > 0.f && x0 + q >= 0 && y0 + r >= 0
                      && (float)(x0 + q) < (float)c.cols*cFrameRectPxPerM/cFacePxPerM
                      && (float)(y0 + r) < (float)c.rows*cFrameRectPxPerM/cFacePxPerM)
                     inFace++;
            faceShare = (float)inFace/((float)c.cols*(float)c.rows*(cFrameRectPxPerM/cFacePxPerM)*(cFrameRectPxPerM/cFacePxPerM));
            if (faceShare < 0.02f) // a sliver of the face: not one of its frames
               continue;

            float lean[7],
                  eye = c.down.y < -0.5f ? c.origin.y*cFrameRectPxPerM - (float)y0 : NAN; // walls: the eye hc below the top

            // the camera's foot on the plane (D n), across the canvas: where the verticals' convergence is measured from
            float footX = ((n.x*D - c.origin.x)*c.across.x + (n.y*D - c.origin.y)*c.across.y + (n.z*D - c.origin.z)*c.across.z)
                          *cFrameRectPxPerM - (float)x0;

            inspectRectLean(acc(), cols, rows, cFrameRectPxPerM, eye, isnan(eye) ? NAN : -(float)y0, footX, D, lean);
            inspectLean[k][i][0] = lean[0];
            inspectLean[k][i][1] = lean[1];
            inspectLean[k][i][2] = lean[5];
            inspectLean[k][i][3] = lean[6];
            inspectLean[k][i][4] = lean[2];
            inspectLean[k][i][5] = D > 0.f && !isnan(eye) ? c.origin.y/D : 0.f; // the ceiling line's h/D
            if (outDir)
            {
               TFloorGrid g = {};

               g.rows = rows;
               g.cols = cols;
               g.acc = acc();
               snprintf(path, sizeof(path), "%s/%s/%03d.bmp", outDir, c.name, f);
               inspectFloorBMP(g, cFrameRectPxPerM, NULL, NULL, 0, path);
            }
            if (list[i])
               fprintf(list[i], "%d,%d,%d,%d,%d,%.0f,%.4f,%.3f,%.0f,%.3f,%.3f,%.0f,%.3f,%.0f\r\n", f, x0, y0, cols, rows,
                       cFrameRectPxPerM, faceShare, lean[0], lean[1], lean[2], lean[3], lean[4], lean[5], lean[6]);
            written++;
         }
      }
   for (int i = 0; i < faces; i++)
      if (list[i])
         fclose(list[i]);
   if (outDir)
      printf("  faces room %lu: %d frame rectifications written, a folder per face\n", (unsigned long)room, written);
}

/*--------------------------------------------------------------------------------
   The plumb and heading of every frame from the scene's own orthogonal axes (user, 2026-09-28:
   "many vertical vectors with a vanishing point in common" set the verticals - adaptive to any
   scene, not one piece of furniture - and the ceiling's line sets the top line). Each frame is
   rectified onto every wall it sees and measured there (inspectRectLean): the lean t most of its
   verticals share is its turn still missing about the wall's normal n, the slope of the ceiling's
   line its turn about the vertical; least squares over these per frame (two walls in view give
   both horizontal components). Each turn is held to its window - two frames before and two after
   once each, its own measure cLeanOwnWeight times - and at most cLeanMaxDeg; rounds of measure and
   turn until they settle.
  --------------------------------------------------------------------------------*/
static const float cLeanOwnWeight = 8.f,
                   cLeanMaxDeg = 6.f,
                   cLeanMinLines = 3.f,
                   cLeanMinHeads = 1.f,     // ceiling lines a frame's heading needs
                   cLeanHeadingSign = 1.f,  // the turn about the vertical that undoes the heading the ceiling's line shows
                   cLeanTiltSign = 1.f;     // the turn about a wall's horizontal axis that undoes the verticals' convergence

enum {
   inspectLeanRounds = 3,
   inspectLeanBands  = 3 // the center spin's bands (ceiling, horizon, floor): each turns as one
};

//--------------------------------------------------------------------------------
static void inspectLeanPlumb(LPCSTR sessionDir, DWORD room, const TFaceCanvas *fc, int walls)
{
   for (int round = 0; round <= inspectLeanRounds; round++)
   {
      float turn[inspectPanoMax][3],
            sum = 0.f,
            weight = 0.f,
            hsum = 0.f,
            hweight = 0.f,
            tsum = 0.f,
            tweight = 0.f,
            HB[inspectLeanBands][9] = {}, // each band's normal equations
            GB[inspectLeanBands][3] = {},
            bandTurn[inspectLeanBands][3];
      bool  has[inspectPanoMax],
            bandHas[inspectLeanBands];

      inspectFaceFrames(sessionDir, NULL, room, fc, walls);
      for (int k = 0; k < inspectPanoCount; k++)
      {
         /* every measure is one component of the frame's missing turn w along an axis u: u.w = m. The verticals'
            lean about a wall's normal, their convergence about its horizontal axis, the ceiling's line about the
            vertical; least squares over them (two walls seeing the same axis are not counted twice) */
         float H[9] = {},
               g[3] = {},
               wsum = 0.f;

         turn[k][0] = turn[k][1] = turn[k][2] = 0.f;
         has[k] = false;
         for (int i = 0; i < walls; i++)
         {
            const float       *l = inspectLean[k][i];
            const TFaceCanvas &c = fc[i];
            TVec3              n = { c.across.y*c.down.z - c.across.z*c.down.y, c.across.z*c.down.x - c.across.x*c.down.z,
                                     c.across.x*c.down.y - c.across.y*c.down.x },
                               up = { 0.f, 1.f, 0.f };
            /* the ceiling line's slope s asks (n + h/D up).w = -s: the heading tilts it by h/D, a turn about the
               wall's normal by one (a line rotates with the wall's picture) */
            TVec3              cu = { n.x + l[5]*up.x, n.y + l[5]*up.y, n.z + l[5]*up.z };
            const TVec3       *axis[3] = { &n, &c.across, &cu };
            float              meas[3] = { l[0], cLeanTiltSign*l[4], -cLeanHeadingSign*l[2] },
                               wt[3] = { l[1] >= cLeanMinLines ? l[1] : 0.f, 0.f, // the convergence only checked
                                         l[3] >= cLeanMinHeads ? l[3] : 0.f };

            for (int s = 0; s < 3; s++)
            {
               const TVec3 &u = *axis[s];
               float        uu[3] = { u.x, u.y, u.z },
                            m = fmaxf(-cLeanMaxDeg, fminf(cLeanMaxDeg, meas[s]))*0.01745329f;

               if (wt[s] <= 0.f)
                  continue;
               for (int r = 0; r < 3; r++)
               {
                  for (int q = 0; q < 3; q++)
                     H[3*r + q] += wt[s]*uu[r]*uu[q];
                  g[r] += wt[s]*uu[r]*m;
               }
               wsum += wt[s];
            }
            if (wt[0] > 0.f)
            {
               sum += fabsf(l[0])*l[1];
               weight += l[1];
            }
            if (wt[2] > 0.f)
            {
               hsum += fabsf(l[2])*l[3];
               hweight += l[3];
            }
            if (l[1] >= cLeanMinLines + 1.f)
            {
               tsum += fabsf(l[4])*l[1];
               tweight += l[1];
            }
         }
         if (wsum <= 0.f)
            continue;
         for (int r = 0; r < 3; r++)
            H[4*r] += 1e-3f*wsum; // an axis no measure sees stays as it is

         int b = (int)inspectBand[inspectPano[k].index] < inspectLeanBands ? (int)inspectBand[inspectPano[k].index] : 0;

         for (int r = 0; r < 9; r++)
            HB[b][r] += H[r];
         for (int r = 0; r < 3; r++)
            GB[b][r] += g[r];
      }

      /* one turn per band (user, 2026-09-28: the frames' own measures are noisy - few verticals, a short crease -
         and turning each by its own broke the neighbors' agreement, stepping the crease): the band's frames turn
         together, by the least squares over all their measures; the photometric registration keeps them agreeing */
      /* the heading is one for the whole spin (a band turned apart would come loose from the others): the least
         squares over every band's measures sets it, each band then solves only its plumb with that heading held */
      float HA[9] = {},
            GA[3] = {};

      for (int b = 0; b < inspectLeanBands; b++)
      {
         for (int r = 0; r < 9; r++)
            HA[r] += HB[b][r];
         for (int r = 0; r < 3; r++)
            GA[r] += GB[b][r];
      }

      float all[3];

      memcpy(all, GA, sizeof(all));
      if (!(HA[0] + HA[4] + HA[8] > 0.f) || !inspectPanoSolve(HA, all, 3))
         all[1] = 0.f;
      for (int b = 0; b < inspectLeanBands; b++)
      {
         // the band's plumb: its equations with the heading moved to the right side (w_y = the whole spin's)
         float h2[4] = { HB[b][0], HB[b][2], HB[b][6], HB[b][8] },
               g2[2] = { GB[b][0] - HB[b][1]*all[1], GB[b][2] - HB[b][7]*all[1] };

         bandTurn[b][0] = bandTurn[b][2] = 0.f;
         bandTurn[b][1] = all[1];
         bandHas[b] = true;
         if (h2[0] + h2[3] > 0.f && inspectPanoSolve(h2, g2, 2))
         {
            bandTurn[b][0] = g2[0];
            bandTurn[b][2] = g2[1];
         }
         if (bandHas[b])
            printf("  faces room %lu: plumb, round %d: band %d turns %.2f %.2f %.2f degrees\n", (unsigned long)room, round, b,
                   bandTurn[b][0]*57.2957795f, bandTurn[b][1]*57.2957795f, bandTurn[b][2]*57.2957795f);
      }
      for (int k = 0; k < inspectPanoCount; k++)
      {
         int b = (int)inspectBand[inspectPano[k].index] < inspectLeanBands ? (int)inspectBand[inspectPano[k].index] : 0;

         has[k] = bandHas[b];
         memcpy(turn[k], bandTurn[b], sizeof(bandTurn[b]));
      }
      printf("  faces room %lu: plumb, round %d: verticals lean %.2f degrees (%.0f lines), converge by a tilt of %.2f degrees,"
             " the ceiling's line shows a heading of %.2f degrees (%.0f lines)\n", (unsigned long)room, round,
             weight > 0.f ? sum/weight : 0.f, weight, tweight > 0.f ? tsum/tweight : 0.f, hweight > 0.f ? hsum/hweight : 0.f,
             hweight);
      if (round == inspectLeanRounds)
         break;
      for (int k = 0; k < inspectPanoCount; k++)
      {
         float e[9],
               old[9];

         if (!has[k])
            continue;

         const float *w = turn[k];
         TPanoFrame  &p = inspectPano[k];

         inspectPanoExp(w, e);
         memcpy(old, p.rot, sizeof(old));
         for (int c = 0; c < 3; c++)
         {
            p.rot[3*c] = e[0]*old[3*c] + e[1]*old[3*c + 1] + e[2]*old[3*c + 2];
            p.rot[3*c + 1] = e[3]*old[3*c] + e[4]*old[3*c + 1] + e[5]*old[3*c + 2];
            p.rot[3*c + 2] = e[6]*old[3*c] + e[7]*old[3*c + 1] + e[8]*old[3*c + 2];
         }
         memcpy(inspectPanoRot[p.index], p.rot, sizeof(p.rot));
      }
      inspectPanoGN(room, 1); // the neighbors' agreement restored at the fine levels, the axes kept by the whole set
   }
}

/*--------------------------------------------------------------------------------
   The plan again from the solved turns (101732: the plan was found on the gyroscope's raw turns;
   with the lines' turns its wall distances no longer met the frames - the window wall's crease fell
   off its canvas and the window's top was taken for it). Every frame of the room is read again: a
   center-spin frame with its solved turn, a corner station's with its own; the vanishing points and
   their edges measured anew and fed to a fresh layout (no tilt bias: the lines already set the plumb).
  --------------------------------------------------------------------------------*/
static bool inspectReplan(LPCSTR sessionDir, DWORD room, const DWORD *frameInfo, int frames, float ceilingM, TLayoutPlan &plan)
{
   TSessionReader reader;
   TRoomLayout    layout;
   TRecordView    v;
   TVanishConfig  cfg = TVanishConfig::Default();
   TAlloc<float>  rays((size_t)cfg.maxEdges*3u);
   TAlloc<BYTE>   labels((size_t)cfg.maxEdges);
   TVanishEdges   edges = { rays(), labels(), (DWORD)cfg.maxEdges, 0u };
   const TVec3    noBias = { 0.f, 0.f, 0.f };
   int            index = 0,
                  fed = 0;

   if (!reader.Open(sessionDir))
      return false;
   while (reader.Next(v))
   {
      TImageRecord img;

      if (v.type != rtImage || !img.Decode(v.payload, v.length))
         continue;

      int f = index++;

      if (f >= frames || f >= inspectMaxFrames || frameInfo[f]/inspectStations != room || inspectIsSuperseded(inspectStamp[f]))
         continue;

      DWORD station = frameInfo[f]%inspectStations;
      TMat4 pose = img.cameraToWorld;

      if (station == 0u)
      {
         if (!inspectPanoHas[f])
            continue;

         const float *r = inspectPanoRot[f];

         pose.m[0] = r[0];
         pose.m[1] = r[1];
         pose.m[2] = r[2];
         pose.m[4] = r[3];
         pose.m[5] = r[4];
         pose.m[6] = r[5];
         pose.m[8] = r[6];
         pose.m[9] = r[7];
         pose.m[10] = r[8];
      }

      TAlloc<BYTE>  luma((size_t)img.width*img.height);
      TVanishResult vr = {};

      vr.roomAxisDeg = NAN;
      edges.count = 0u;
      if (!inspectDecode(img.pixels, img.pixelBytes, img.width, img.height, false, luma)
          || !vanishDetect(luma(), (int)img.width, (int)img.height, (int)img.width, img.intr, pose, cfg, vr, &edges))
         continue;
      layout.AddFrame(pose, vr, edges, noBias, (int)station);
      fed++;
   }

   bool ok = layout.Solve(layout.AnchorDeg(), ceilingM, plan);

   printf("  replan room %lu: %d frames with the solved turns: %s, %u corners, axis %.2f, camera %.2f m, ceiling %.2f m\n",
          (unsigned long)room, fed, ok && plan.valid ? "a plan" : "no plan", (unsigned)plan.vertexCount, plan.axisDeg,
          plan.cameraHeightM, plan.ceilingM);
   for (int i = 0; ok && i < (int)plan.vertexCount; i++)
      printf("  replan room %lu: corner %d at u %.3f w %.3f\n", (unsigned long)room, i, plan.verts[i].u, plan.verts[i].w);
   return ok && plan.valid;
}

//--------------------------------------------------------------------------------
// The room's box from the faces (outward normals) and the plan's floor and ceiling (see inspectBoxHit)
static void inspectBuildBox(const TFaceCanvas *fc, int walls, float cam, float ceil)
{
   inspectBoxCount = 0;
   for (int i = 0; i < walls && inspectBoxCount < inspectBoxMax - 2; i++)
   {
      float *bn = inspectBoxN[inspectBoxCount],
            bl;

      bn[0] = fc[i].across.z;
      bn[1] = 0.f;
      bn[2] = -fc[i].across.x;
      bl = bn[0]*fc[i].origin.x + bn[2]*fc[i].origin.z;
      if (bl < 0.f)
      {
         bn[0] = -bn[0];
         bn[2] = -bn[2];
         bl = -bl;
      }
      inspectBoxD[inspectBoxCount++] = bl;
   }
   inspectBoxN[inspectBoxCount][0] = 0.f; // the floor
   inspectBoxN[inspectBoxCount][1] = -1.f;
   inspectBoxN[inspectBoxCount][2] = 0.f;
   inspectBoxD[inspectBoxCount++] = cam;
   inspectBoxN[inspectBoxCount][0] = 0.f; // the ceiling
   inspectBoxN[inspectBoxCount][1] = 1.f;
   inspectBoxN[inspectBoxCount][2] = 0.f;
   inspectBoxD[inspectBoxCount++] = ceil - cam;
}

//--------------------------------------------------------------------------------
static void inspectFaces(LPCSTR sessionDir, LPCSTR outDir, DWORD room, const TLayoutPlan &plan, const DWORD *frameInfo, int frames)
{
   if (!plan.valid || plan.vertexCount < 3 || isnan(plan.axisDeg))
   {
      printf("  faces room %lu: no plan\n", (unsigned long)room);
      return;
   }
   inspectLever.x = inspectLever.y = inspectLever.z = 0.f; // each room's spin its own lever, pure rotation until the box
   inspectBoxCount = 0;

   /* the lines aligned every frame to one room axis (their mean): the plan found its walls on its own axis, anchored
      elsewhere in the capture while the gyroscope drifted (101732: the walls rebuilt 6.6 degrees off, the room 3.9 m
      long for 3.3). The whole set is turned, mod 90, onto the plan's axis */
   if (inspectVpSolveOn && !isnan(inspectVpAlpha))
   {
      float delta = plan.axisDeg*0.01745329f - inspectVpAlpha;

      delta -= 1.5707963f*floorf(delta/1.5707963f + 0.5f);

      float w[3] = { 0.f, -delta, 0.f }; // a turn about the world's up raises the heading by -w_y

      for (int i = 0; i < inspectPanoCount; i++)
      {
         float rot[9];

         inspectTurnWorld(inspectPano[i].rot, w, rot);
         memcpy(inspectPano[i].rot, rot, sizeof(rot));
         if (inspectPanoHas[inspectPano[i].index])
            memcpy(inspectPanoRot[inspectPano[i].index], rot, sizeof(rot));
      }
      printf("  faces room %lu: the lines' room axis turned %.2f degrees onto the plan's (%.2f)\n", (unsigned long)room,
             delta*57.2957795f, plan.axisDeg);
   }

   float       a = plan.axisDeg*0.01745329f,
               cam = plan.cameraHeightM > 0.3f ? plan.cameraHeightM : 1.5f,
               ceil = plan.ceilingM > 1.f ? plan.ceilingM : 2.7f,
               upU = 1.f,
               upW = 0.f;
   TVec3       ud = { sinf(a), 0.f, -cosf(a) },
               wd = { cosf(a), 0.f, sinf(a) };
   int         walls = plan.vertexCount,
               faces = walls + 1;
   TFaceCanvas fc[layoutMaxVerts + 1];

   for (int i = 0; i < walls; i++)
   {
      const TPlanPoint &p = plan.verts[i],
                       &q = plan.verts[(i + 1)%walls];
      float             du = q.u - p.u,
                        dw = q.w - p.w,
                        len = sqrtf(du*du + dw*dw);
      char              label[16];

      mosaicFaceLabel(i, label, sizeof(label));
      if (!strcmp(label, "N"))
      {
         bool  constU = fabsf(p.u - q.u) < fabsf(p.w - q.w);
         float side = (constU ? p.u + q.u : p.w + q.w) >= 0.f ? 1.f : -1.f;

         upU = constU ? side : 0.f;
         upW = constU ? 0.f : side;
      }
      snprintf(fc[i].name, sizeof(fc[i].name), "%s", label);
      fc[i].origin.x = p.u*ud.x + p.w*wd.x;
      fc[i].origin.y = ceil - cam;
      fc[i].origin.z = p.u*ud.z + p.w*wd.z;
      fc[i].across.x = (du*ud.x + dw*wd.x)/len;
      fc[i].across.y = 0.f;
      fc[i].across.z = (du*ud.z + dw*wd.z)/len;
      fc[i].down.x = 0.f;
      fc[i].down.y = -1.f;
      fc[i].down.z = 0.f;
      fc[i].cols = (int)(len*cFacePxPerM + 0.5f);
      fc[i].rows = (int)(ceil*cFacePxPerM + 0.5f);
   }

   // the floor from above: up (upU, upW) in the plan, right clockwise of it; the canvas bounds the polygon
   float rightU = -upW,
         rightW = upU,
         xLo = 1e9f,
         xHi = -1e9f,
         yLo = 1e9f,
         yHi = -1e9f;
   TFaceCanvas &fl = fc[walls];
   char         label[16];

   for (int i = 0; i < walls; i++)
   {
      float x = plan.verts[i].u*rightU + plan.verts[i].w*rightW,
            y = plan.verts[i].u*upU + plan.verts[i].w*upW;

      xLo = fminf(xLo, x);
      xHi = fmaxf(xHi, x);
      yLo = fminf(yLo, y);
      yHi = fmaxf(yHi, y);
   }
   mosaicFaceLabel(walls, label, sizeof(label));
   snprintf(fl.name, sizeof(fl.name), "%s", strncmp(label, "parede", 6) ? label : "piso");

   TVec3 rightWorld = { rightU*ud.x + rightW*wd.x, 0.f, rightU*ud.z + rightW*wd.z },
         upWorld = { upU*ud.x + upW*wd.x, 0.f, upU*ud.z + upW*wd.z };

   fl.origin.x = xLo*rightWorld.x + yHi*upWorld.x;
   fl.origin.y = -cam;
   fl.origin.z = xLo*rightWorld.z + yHi*upWorld.z;
   fl.across = rightWorld;
   fl.down.x = -upWorld.x;
   fl.down.y = 0.f;
   fl.down.z = -upWorld.z;
   fl.cols = (int)((xHi - xLo)*cFacePxPerM + 0.5f);
   fl.rows = (int)((yHi - yLo)*cFacePxPerM + 0.5f);
   if (inspectPlumbOn) // the scene's verticals and ceiling line set each frame's plumb and heading, then the walls
      inspectLeanPlumb(sessionDir, room, fc, walls);
   for (int pass = 0; pass < 2; pass++) // twice: each wall's corners hang on its neighbors' creases
      if (!inspectCreaseWalls(sessionDir, room, fc, walls, cam, ceil))
      {
         printf("  faces room %lu: walls kept as they are (pass %d)\n", (unsigned long)room, pass + 1);
         break;
      }

   // the room's box from the faces, then the lever arm with the rotations (see inspectBoxHit)
   inspectBuildBox(fc, walls, cam, ceil);
   if (inspectLeverOn && inspectPanoCount > 1 && inspectLeverScan)
   {
      // the registration's residual with the lever held at each trial, the rotations registered anew (level 1/16)
      TAlloc<float> keep((size_t)inspectPanoCount*9u);

      for (int i = 0; i < inspectPanoCount; i++)
         memcpy(&keep[(size_t)i*9u], inspectPano[i].rot, sizeof(inspectPano[i].rot));
      for (int axis = 0; axis < 3; axis++)
         for (int s = -2; s <= 2; s++)
         {
            float knee = 1e9f,
                  rms = 0.f;
            int   count = 0;

            for (int i = 0; i < inspectPanoCount; i++)
               memcpy(inspectPano[i].rot, &keep[(size_t)i*9u], sizeof(inspectPano[i].rot));
            inspectLever.x = axis == 0 ? 0.2f*(float)s : 0.f;
            inspectLever.y = axis == 1 ? 0.2f*(float)s : 0.f;
            inspectLever.z = axis == 2 ? 0.2f*(float)s : 0.f;
            for (int it = 0; it < inspectPanoIters; it++)
            {
               rms = inspectPanoRound(1, knee, count);
               knee = cPanoHuber*rms;
            }
            printf("  faces room %lu: lever scan %.2f %.2f %.2f m: RMS %.4f (%d residuals)\n", (unsigned long)room,
                   inspectLever.x, inspectLever.y, inspectLever.z, rms, count);
         }
      for (int i = 0; i < inspectPanoCount; i++)
         memcpy(inspectPano[i].rot, &keep[(size_t)i*9u], sizeof(inspectPano[i].rot));
      inspectLever.x = inspectLever.y = inspectLever.z = 0.f;
   }
   if (inspectVpSolveOn && inspectPanoCount > 1) // the turns held by the lines: the centers from the matched points
   {
      inspectPanoCircleSolve(room);

      // the walls again, their crease rays now leaving from the solved centers, and the box they make
      for (int pass = 0; pass < 2; pass++)
         if (!inspectCreaseWalls(sessionDir, room, fc, walls, cam, ceil))
            break;
      inspectBuildBox(fc, walls, cam, ceil);
   }
   else if (inspectLeverOn && inspectPanoCount > 1)
   {
      /* each frame's own center (user, 2026-09-29): the lever came out ~0 (no steady arm) yet the furniture doubles -
         the body sways from shot to shot; three more unknowns per frame, held toward the pivot */
      TAlloc<float> len((size_t)inspectPanoCount);
      float         band[inspectLeanBands][4] = {}; // mean x y z, frames

      memset(inspectBody, 0, sizeof(inspectBody));
      inspectBodyFit = true; // the body's pairs first, from 1/32: the parallax they make is seen there already
      inspectPanoGN(room, 2);
      for (int b = 0; b < inspectBodyBands; b++)
         printf("  faces room %lu: band %d: the phone %.3f m ahead of the turn's axis, %.3f m above the pivot\n",
                (unsigned long)room, b, inspectBody[b][0], inspectBody[b][1]);
      inspectShiftFit = true;
      inspectPanoGN(room, 1);
      inspectShiftFit = false;
      inspectBodyFit = false;
      for (int b = 0; b < inspectBodyBands; b++)
         printf("  faces room %lu: band %d, with the sways: the phone %.3f m ahead, %.3f m above\n", (unsigned long)room, b,
                inspectBody[b][0], inspectBody[b][1]);
      for (int i = 0; i < inspectPanoCount; i++)
      {
         const float *s = inspectPano[i].shift;
         int          b = (int)inspectBand[inspectPano[i].index];

         len[i] = sqrtf(s[0]*s[0] + s[1]*s[1] + s[2]*s[2]);
         if (b >= 0 && b < inspectLeanBands)
         {
            for (int k = 0; k < 3; k++)
               band[b][k] += s[k];
            band[b][3] += 1.f;
         }
      }
      printf("  faces room %lu: own shifts of the camera centers: median %.3f, p90 %.3f, largest %.3f m\n", (unsigned long)room,
             inspectQuantile(len(), inspectPanoCount, 0.5f), inspectQuantile(len(), inspectPanoCount, 0.9f),
             inspectQuantile(len(), inspectPanoCount, 1.f));
      for (int b = 0; b < inspectLeanBands; b++)
         if (band[b][3] > 0.f)
            printf("  faces room %lu: band %d: mean shift %.3f %.3f %.3f m (%d frames)\n", (unsigned long)room, b,
                   band[b][0]/band[b][3], band[b][1]/band[b][3], band[b][2]/band[b][3], (int)band[b][3]);
   }
   else
      inspectBoxCount = 0; // pure rotation, as before
   inspectFaceAlign(sessionDir, outDir, room, fc, faces);
   for (int i = 0; i < faces; i++)
      inspectFaceAlloc(fc[i]);

   TSessionReader reader;
   TRecordView    v;
   int            index = 0,
                  used = 0;

   if (!reader.Open(sessionDir))
      return;
   while (reader.Next(v))
   {
      TImageRecord img;

      if (v.type != rtImage || !img.Decode(v.payload, v.length))
         continue;

      int f = index++;

      if (f >= frames || f >= inspectMaxFrames || frameInfo[f]/inspectStations != room || frameInfo[f]%inspectStations != 0u)
         continue;
      if (inspectIsSuperseded(inspectStamp[f]) || inspectKind[f] != (BYTE)skCenter)
         continue;

      TAlloc<BYTE> bgr((size_t)img.width*img.height*3u);

      if (!inspectDecode(img.pixels, img.pixelBytes, img.width, img.height, true, bgr))
         continue;

      const float *p = img.cameraToWorld.m;
      float        m[9] = { p[0], p[1], p[2], p[4], p[5], p[6], p[8], p[9], p[10] };

      int         k = -1;
      const float none[4] = { 0.f, 0.f, 0.f, 0.f };

      for (int i = 0; i < inspectPanoCount; i++)
         k = inspectPano[i].index == f ? i : k;
      if (inspectPanoHas[f])
         memcpy(m, inspectPanoRot[f], sizeof(m));
      for (int i = 0; i < faces; i++)
         inspectFaceSplat(img, bgr(), m, k >= 0 ? inspectPano[k].shift : NULL, f, k >= 0 ? inspectSim[k][i] : none,
                          inspectSimRef[i], fc[i]);
      used++;
   }

   /* the way back from a face pixel to its frame (user, 2026-09-28): faces_geometria.csv turns a face pixel (c, r) into
      the canvas point u = (c + 0.5) alignPxPerM/facePxPerM - 0.5 (v likewise), pulled back through the frame's
      similarity (x = u - refX, y = v - refY; u -= tx + a x - b y, v -= ty + b x + a y), then the world direction
      origin + (u + 0.5)/alignPxPerM across + (v + 0.5)/alignPxPerM down; faces_quadros.csv turns it into the frame's
      pixel by its rotation rows (rc = R d) and pinhole (su = cx + fx rc.x/-rc.z, sv = cy - fy rc.y/-rc.z) */
   char  path[sessionPathMax];
   FILE *geo,
        *frm;

   snprintf(path, sizeof(path), "%s/faces_geometria.csv", outDir);
   geo = fopen(path, "wb");
   snprintf(path, sizeof(path), "%s/faces_quadros.csv", outDir);
   frm = fopen(path, "wb");
   if (geo)
   {
      fprintf(geo, "face,originX,originY,originZ,acrossX,acrossY,acrossZ,downX,downY,downZ,facePxPerM,alignPxPerM,cols,rows,"
                   "refX,refY,cameraM,ceilingM\r\n");
      for (int i = 0; i < faces; i++)
         fprintf(geo, "%s,%.5f,%.5f,%.5f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.1f,%.1f,%d,%d,%.2f,%.2f,%.3f,%.3f\r\n", fc[i].name,
                 fc[i].origin.x, fc[i].origin.y, fc[i].origin.z, fc[i].across.x, fc[i].across.y, fc[i].across.z, fc[i].down.x,
                 fc[i].down.y, fc[i].down.z, cFacePxPerM, cAlignPxPerM, fc[i].cols, fc[i].rows, inspectSimRef[i][0],
                 inspectSimRef[i][1], cam, ceil);
      fclose(geo);
   }
   if (frm)
   {
      fprintf(frm, "frame,face,tx,ty,a,b,scale,rotDeg,fx,fy,cx,cy,r00,r01,r02,r10,r11,r12,r20,r21,r22,gyroDeg,shiftX,shiftY,"
                   "shiftZ\r\n");
      for (int k = 0; k < inspectPanoCount; k++)
         for (int i = 0; i < faces; i++)
         {
            const TPanoFrame &p = inspectPano[k];
            const float      *s = inspectSim[k][i],
                             *r = p.rot;

            fprintf(frm, "%d,%s,%.4f,%.4f,%.7f,%.7f,%.6f,%.4f,%.2f,%.2f,%.2f,%.2f,%.7f,%.7f,%.7f,%.7f,%.7f,%.7f,%.7f,%.7f,%.7f,"
                         "%.3f,%.4f,%.4f,%.4f\r\n", p.index, fc[i].name, s[0], s[1], s[2], s[3], 1.f + s[2], s[3]*57.2957795f, p.fx, p.fy, p.cx,
                    p.cy, r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7], r[8], inspectPanoAngleDeg(p.rot, p.gyro),
                    p.shift[0], p.shift[1], p.shift[2]);
         }
      fclose(frm);
   }
   for (int i = 0; i < faces; i++)
   {
      TFloorGrid g = {};
      char       path[sessionPathMax];

      g.rows = fc[i].rows;
      g.cols = fc[i].cols;
      g.acc = fc[i].acc();
      snprintf(path, sizeof(path), "%s/giro_%s.bmp", outDir, fc[i].name);
      inspectFloorBMP(g, cFacePxPerM, NULL, NULL, 0, path);
      snprintf(path, sizeof(path), "%s/giro_%s_owner.u16", outDir, fc[i].name);

      FILE *own = fopen(path, "wb");

      if (own)
      {
         fwrite(fc[i].owner(), sizeof(WORD), (size_t)fc[i].cols*fc[i].rows, own);
         fclose(own);
      }
      printf("  faces room %lu: %s %dx%d px (%.2f x %.2f m)\n", (unsigned long)room, fc[i].name, fc[i].cols, fc[i].rows,
             (float)fc[i].cols/cFacePxPerM, (float)fc[i].rows/cFacePxPerM);
   }
   printf("  faces room %lu: %d center-spin frames (%s), camera %.2f m, ceiling %.2f m\n", (unsigned long)room, used,
          inspectPanoCount ? "registered" : "gyroscope", cam, ceil);
   inspectCreaseCheck(sessionDir, outDir, room, fc, walls, ceil);
   if (inspectFaceFramesOn)
      inspectFaceFrames(sessionDir, outDir, room, fc, faces);
}

//--------------------------------------------------------------------------------
// The faces on the plan found again from the solved turns when the lines hold them (--vp-solve), else on the plan given
static void inspectFacesSolved(LPCSTR sessionDir, LPCSTR outDir, DWORD room, const TLayoutPlan &plan, const DWORD *frameInfo,
                               int frames)
{
   static TLayoutPlan replan;

   if (inspectVpSolveOn && inspectReplan(sessionDir, room, frameInfo, frames, plan.ceilingM > 1.f ? plan.ceilingM : 2.8f, replan))
      inspectFaces(sessionDir, outDir, room, replan, frameInfo, frames);
   else
      inspectFaces(sessionDir, outDir, room, plan, frameInfo, frames);
}

/*--------------------------------------------------------------------------------
   Floor top views (--floor), user 2026-09-28: every keyframe below the horizon holds some floor.
   Each frame is re-projected onto the floor plane (camera height from the plan), 4 mm per pixel,
   plan axes (u up, w right: a view from above, not mirrored), and written alone as piso_NNN.bmp -
   the frame's share of the fifth face (four walls and the floor). The center spin stands on the
   spin circle, so its crops are also summed straight into the plan, cut to the polygon:
   piso_room<N>.bmp (walls and spin point drawn) and piso_room<N>_limpo.bmp. A corner station stood
   at a spot only the ceiling creases and corners can fix (the gyroscope is coarse): its crops stay
   frame by frame here, centered on the camera's nadir; the whole floor, every frame placed by the
   bundle adjustment, comes out of --walls. The door phase walks, so it stays out.
  --------------------------------------------------------------------------------*/
static void inspectFloorViews(LPCSTR sessionDir, LPCSTR outDir, DWORD room, const TLayoutPlan &plan, bool solved,
                              float axisDeg, const TVanishResult *frameVr, const DWORD *frameInfo, int frames,
                              float spinRadiusM)
{
   const float    ppm = (float)inspectWallPxPerM;
   float          a = axisDeg*0.01745329f,
                  h = plan.cameraHeightM > 0.3f ? plan.cameraHeightM : 1.5f,
                  uLo = 0.f,
                  uHi = 0.f,
                  wLo = 0.f,
                  wHi = 0.f;
   TVec3          ud = { sinf(a), 0.f, -cosf(a) },
                  wd = { cosf(a), 0.f, sinf(a) };
   bool           hasPlan = solved && plan.vertexCount >= 3;
   int            written = 0,
                  index = 0;
   TSessionReader reader;
   TRecordView    v;

   if (isnan(axisDeg) || !reader.Open(sessionDir))
      return;
   for (int i = 0; i < plan.vertexCount; i++)
   {
      uLo = fminf(uLo, plan.verts[i].u);
      uHi = fmaxf(uHi, plan.verts[i].u);
      wLo = fminf(wLo, plan.verts[i].w);
      wHi = fmaxf(wHi, plan.verts[i].w);
   }

   TFloorGrid mosaic = {};

   mosaic.row0 = (int)floorf(-(uHi + cFloorMarginM)*ppm);
   mosaic.col0 = (int)floorf((wLo - cFloorMarginM)*ppm);
   mosaic.rows = hasPlan ? (int)((uHi - uLo + 2.f*cFloorMarginM)*ppm) + 1 : 1;
   mosaic.cols = hasPlan ? (int)((wHi - wLo + 2.f*cFloorMarginM)*ppm) + 1 : 1;

   TAlloc<float> mosaicAcc((size_t)mosaic.rows*mosaic.cols*4u);
   TPlanPoint    spinPoint = { 0.f, 0.f };

   mosaic.acc = mosaicAcc();
   memset(mosaicAcc(), 0, (size_t)mosaic.rows*mosaic.cols*4u*sizeof(float));
   printf("  floor views room %lu: camera height %.2f m%s, axis %.2f\n", (unsigned long)room, h,
          hasPlan ? "" : " (no plan: frames only)", axisDeg);
   while (reader.Next(v))
   {
      TImageRecord img;

      if (v.type != rtImage || !img.Decode(v.payload, v.length))
         continue;

      int f = index++;

      if (f >= frames || f >= inspectMaxFrames || frameInfo[f]/inspectStations != room)
         continue;
      if (inspectIsSuperseded(inspectStamp[f]) || inspectKind[f] == (BYTE)skDoor)
         continue;

      int        st = (int)(frameInfo[f]%inspectStations);
      bool       onSpin = st == 0;
      TFloorPose p;
      bool       byVanish = inspectFloorPose(img, frameVr[f], axisDeg, onSpin, spinRadiusM, p);
      DWORD      axes = frameVr[f].flags & (vfAxisA | vfAxisB);
      bool       xyz = (frameVr[f].flags & vfVertical) && axes == (vfAxisA | vfAxisB) && byVanish && hasPlan && onSpin
                       && inspectFloorSeesCorner(img, p, plan, ud, wd, h); // the merge takes only these
      float      uMin,
                 uMax,
                 wMin,
                 wMax,
                 share = inspectFloorFootprint(img, p, ud, wd, h, onSpin && hasPlan ? &plan : NULL, uMin, uMax, wMin, wMax);

      if (share < cFloorMinShare)
         continue;

      TAlloc<BYTE> bgr((size_t)img.width*img.height*3u);

      if (!inspectDecode(img.pixels, img.pixelBytes, img.width, img.height, true, bgr))
         continue;

      TFloorGrid one = {};

      one.row0 = (int)floorf(-uMax*ppm);
      one.col0 = (int)floorf(wMin*ppm);
      one.rows = (int)floorf(-uMin*ppm) - one.row0 + 1;
      one.cols = (int)floorf(wMax*ppm) - one.col0 + 1;

      TAlloc<float> oneAcc((size_t)one.rows*one.cols*4u);

      one.acc = oneAcc();
      memset(oneAcc(), 0, (size_t)one.rows*one.cols*4u*sizeof(float));
      inspectFloorSplat(img, bgr(), p, ud, wd, h, ppm, onSpin && hasPlan ? &plan : NULL, one);

      TPlanPoint nadir = { p.camX*ud.x + p.camZ*ud.z, p.camX*wd.x + p.camZ*wd.z };
      char       path[sessionPathMax];

      snprintf(path, sizeof(path), "%s/piso_%03d.bmp", outDir, f);
      inspectFloorBMP(one, ppm, onSpin && hasPlan ? &plan : NULL, &nadir, 1, path);
      written++;
      printf("  floor frame %03d station %d (%s, %s): pitch %.1f, %.0f%% of the frame on the floor, u %.2f..%.2f w %.2f..%.2f"
             " (%dx%d px)\n", f, st, xyz ? "xyz" : cFloorAxes[axes/2u], byVanish ? "vanishing" : "gyroscope", geomPitchDeg(img.cameraToWorld.Forward()),
             100.f*share, uMin, uMax, wMin, wMax, one.cols, one.rows);
      for (int r = 0; onSpin && hasPlan && xyz && r < one.rows; r++) // the merge takes only XYZ frames
         for (int c = 0; c < one.cols; c++)
         {
            int          tr = one.row0 + r - mosaic.row0,
                         tc = one.col0 + c - mosaic.col0;
            const float *src = one.acc + ((size_t)r*one.cols + c)*4u;

            if (tr < 0 || tr >= mosaic.rows || tc < 0 || tc >= mosaic.cols || !(src[3] > 0.f))
               continue;

            float *dst = mosaic.acc + ((size_t)tr*mosaic.cols + tc)*4u;

            for (int ch = 0; ch < 4; ch++)
               dst[ch] += src[ch];
         }
   }
   printf("  floor views room %lu: %d frames written\n", (unsigned long)room, written);
   if (!hasPlan)
      return;

   char path[sessionPathMax];

   snprintf(path, sizeof(path), "%s/piso_room%lu.bmp", outDir, (unsigned long)room);
   inspectFloorBMP(mosaic, ppm, &plan, &spinPoint, 1, path);
   snprintf(path, sizeof(path), "%s/piso_room%lu_limpo.bmp", outDir, (unsigned long)room);
   inspectFloorBMP(mosaic, ppm, NULL, NULL, 0, path);
}

//--------------------------------------------------------------------------------
// Pitch correction of a measured frame: its crease elevation against the median of its wall and line kind
static float inspectRectPitch(const TRectMeasure &m, const float *medianElev)
{
   float target = medianElev[m.wall*2 + (m.floorLine ? 1 : 0)];

   return isnan(target) ? 0.f : cRectPitchSign*(m.elevDeg - target);
}

/*--------------------------------------------------------------------------------
   The gyroscope's correction table (user, 2026-09-30: "images like 79 and 82, once normalized, make a
   table of the gyroscope's corrections, inherited by the frames without as much anchoring"; "most
   frames have neighbors beside, above and below"). A frame with its own vertical and a room axis has
   its true turn from its lines: the vertical onto the world's up, the axis onto the room's (the
   nearest of refAxisDeg + 90k to the gyroscope's reading); its correction Q = R_lines R_gyro^T, kept as
   a rotation vector. A frame short of either takes its neighbors' (same station, aims within
   cInheritAimDeg), weighed by their lines' support, by how close they aim and how close in time.
  --------------------------------------------------------------------------------*/
static const float cInheritAimDeg = 40.f,
                   cInheritAimScaleDeg = 10.f,
                   cInheritTimeScaleSec = 30.f,
                   cAxisGateDeg = 12.f;          // a measured horizontal is the room's within this (mod 90)

static float inspectInheritW[inspectMaxFrames][3]; // the correction's rotation vector (world, radians)
static BYTE  inspectInheritKind[inspectMaxFrames]; // 0 none, 1 its own lines, 2 inherited

/*--------------------------------------------------------------------------------
   A frame's measured horizontal is the room's only within cAxisGateDeg of the room axis (mod 90):
   101732's frame 81 looks at the floor, its strongest horizontal was the diagonal tiles' (44 degrees
   off) and both its views came out square to the tiles, the walls askew
  --------------------------------------------------------------------------------*/
static bool inspectAxisOk(const TVanishResult &vr, float refAxisDeg)
{
   if (!(vr.flags & (vfAxisA | vfAxisB)) || isnan(vr.roomAxisDeg) || isnan(refAxisDeg))
      return false;

   float d = vr.roomAxisDeg - refAxisDeg;

   d -= 90.f*floorf(d/90.f + 0.5f);
   return fabsf(d) <= cAxisGateDeg;
}

//--------------------------------------------------------------------------------
// Row-major 3x3 of a pose's rotation (camera to world)
static void inspectPoseRows(const TMat4 &m, float *R)
{
   for (int r = 0; r < 3; r++)
      for (int c = 0; c < 3; c++)
         R[3*r + c] = m.m[4*c + r];
}

//--------------------------------------------------------------------------------
static void inspectBuildInherit(LPCSTR sessionDir, const TVanishResult *frameVr, const DWORD *frameInfo, int frames,
                                float refAxisDeg)
{
   TSessionReader reader;
   TRecordView    v;
   int            index = 0,
                  own = 0,
                  inherited = 0;
   TAlloc<float>  Rg((size_t)inspectMaxFrames*9u);
   TAlloc<QWORD>  when((size_t)inspectMaxFrames);
   TAlloc<BYTE>   has((size_t)inspectMaxFrames);

   memset(has(), 0, (size_t)inspectMaxFrames);
   memset(inspectInheritKind, 0, sizeof(inspectInheritKind));
   if (isnan(refAxisDeg) || !reader.Open(sessionDir))
      return;
   while (reader.Next(v))
   {
      TImageRecord img;

      if (v.type != rtImage || !img.Decode(v.payload, v.length))
         continue;

      int f = index++;

      if (f >= frames || f >= inspectMaxFrames)
         break;
      if (inspectIsSuperseded(inspectStamp[f]))
         continue;
      inspectPoseRows(img.cameraToWorld, &Rg[(size_t)f*9u]);
      when[f] = v.stampNs;
      has[f] = 1;
   }

   // the anchored frames: their own lines give the true turn
   for (int f = 0; f < frames && f < inspectMaxFrames; f++)
   {
      const TVanishResult &vr = frameVr[f];
      int                  d = (vr.flags & vfAxisA) ? 1 : 2;

      if (!has[f] || !(vr.flags & vfVertical) || !inspectAxisOk(vr, refAxisDeg)) // a diagonal floor's axis anchors nothing
         continue;

      const float *R = &Rg[(size_t)f*9u];
      TVec3        up = vr.dirCam[0],
                   ax = vr.dirCam[d];
      float        l = sqrtf(up.x*up.x + up.y*up.y + up.z*up.z);

      up.x /= l;
      up.y /= l;
      up.z /= l;
      if (R[3]*up.x + R[4]*up.y + R[5]*up.z < 0.f) // toward the world's up
      {
         up.x = -up.x;
         up.y = -up.y;
         up.z = -up.z;
      }

      float dd = ax.x*up.x + ax.y*up.y + ax.z*up.z;

      ax.x -= dd*up.x;
      ax.y -= dd*up.y;
      ax.z -= dd*up.z;
      l = sqrtf(ax.x*ax.x + ax.y*ax.y + ax.z*ax.z);
      if (l < 1e-6f)
         continue;
      ax.x /= l;
      ax.y /= l;
      ax.z /= l;

      // the axis as the gyroscope sees it, onto the nearest room axis
      float wx = R[0]*ax.x + R[1]*ax.y + R[2]*ax.z,
            wz = R[6]*ax.x + R[7]*ax.y + R[8]*ax.z,
            phi = atan2f(wx, -wz)*57.2957795f,
            k = roundf((phi - refAxisDeg)/90.f),
            h = (refAxisDeg + 90.f*k)*0.01745329f;
      TVec3 aw = { sinf(h), 0.f, -cosf(h) },
            cw = { -aw.z, 0.f, aw.x }, // aw x Y, Y = (0, 1, 0)
            cc = { ax.y*up.z - ax.z*up.y, ax.z*up.x - ax.x*up.z, ax.x*up.y - ax.y*up.x };

      // R_true = [aw Y cw] [ax up cc]^T; Q = R_true R_gyro^T
      float Mw[9] = { aw.x, 0.f, cw.x, aw.y, 1.f, cw.y, aw.z, 0.f, cw.z },
            Mc[9] = { ax.x, up.x, cc.x, ax.y, up.y, cc.y, ax.z, up.z, cc.z },
            Rt[9],
            Q[9];

      for (int r = 0; r < 3; r++)
         for (int c = 0; c < 3; c++)
            Rt[3*r + c] = Mw[3*r]*Mc[3*c] + Mw[3*r + 1]*Mc[3*c + 1] + Mw[3*r + 2]*Mc[3*c + 2];
      for (int r = 0; r < 3; r++)
         for (int c = 0; c < 3; c++)
            Q[3*r + c] = Rt[3*r]*R[3*c] + Rt[3*r + 1]*R[3*c + 1] + Rt[3*r + 2]*R[3*c + 2];

      float ang = acosf(fmaxf(-1.f, fminf(1.f, 0.5f*(Q[0] + Q[4] + Q[8] - 1.f)))),
            kk = ang > 1e-6f ? ang/(2.f*sinf(ang)) : 0.5f;

      inspectInheritW[f][0] = kk*(Q[7] - Q[5]);
      inspectInheritW[f][1] = kk*(Q[2] - Q[6]);
      inspectInheritW[f][2] = kk*(Q[3] - Q[1]);
      inspectInheritKind[f] = 1;
      own++;
   }

   // the others: their neighbors' corrections, nearest aims and times weighing most
   for (int f = 0; f < frames && f < inspectMaxFrames; f++)
   {
      if (!has[f] || inspectInheritKind[f] == 1)
         continue;

      const float *R = &Rg[(size_t)f*9u];
      float        sum[3] = { 0.f, 0.f, 0.f },
                   wsum = 0.f;

      for (int k = 0; k < frames && k < inspectMaxFrames; k++)
      {
         if (inspectInheritKind[k] != 1 || frameInfo[k] != frameInfo[f])
            continue;

         const float *Rk = &Rg[(size_t)k*9u];
         float        cosAim = R[2]*Rk[2] + R[5]*Rk[5] + R[8]*Rk[8], // the forwards (-z columns) against each other
                      aim = acosf(fmaxf(-1.f, fminf(1.f, cosAim)))*57.2957795f;

         if (aim > cInheritAimDeg)
            continue;

         float dt = (float)(when[f] > when[k] ? when[f] - when[k] : when[k] - when[f])*1e-9f,
               sup = (float)frameVr[k].support[0],
               wt = sup/(1.f + (aim/cInheritAimScaleDeg)*(aim/cInheritAimScaleDeg))/(1.f + dt/cInheritTimeScaleSec);

         for (int c = 0; c < 3; c++)
            sum[c] += wt*inspectInheritW[k][c];
         wsum += wt;
      }
      if (wsum <= 0.f)
         continue;
      for (int c = 0; c < 3; c++)
         inspectInheritW[f][c] = sum[c]/wsum;
      inspectInheritKind[f] = 2;
      inherited++;
   }
   printf("  correction table: %d frames by their own lines, %d inherited from their neighbors\n", own, inherited);
}

//--------------------------------------------------------------------------------
// A frame's measure with the table's turn where it has no lines of its own: its vertical and room axis in camera axes
static void inspectInheritVanish(const TMat4 &pose, int f, float refAxisDeg, TVanishResult &vr)
{
   if (f < 0 || f >= inspectMaxFrames || inspectInheritKind[f] != 2 || isnan(refAxisDeg))
      return;

   float R[9],
         E[9],
         Rt[9];

   inspectPoseRows(pose, R);
   inspectPanoExp(inspectInheritW[f], E);
   for (int r = 0; r < 3; r++) // R_true = Q R_gyro
      for (int c = 0; c < 3; c++)
         Rt[3*r + c] = E[3*r]*R[c] + E[3*r + 1]*R[3 + c] + E[3*r + 2]*R[6 + c];

   // camera axes of the world's up and of the room axis: R_true^T times them
   float h = refAxisDeg*0.01745329f;
   TVec3 aw = { sinf(h), 0.f, -cosf(h) },
         up = { Rt[3], Rt[4], Rt[5] },
         ax = { Rt[0]*aw.x + Rt[3]*aw.y + Rt[6]*aw.z, Rt[1]*aw.x + Rt[4]*aw.y + Rt[7]*aw.z,
                Rt[2]*aw.x + Rt[5]*aw.y + Rt[8]*aw.z };

   /* what the frame measures stays; only what it lacks is inherited. A measured room axis (a crease) keeps the
      inherited vertical square to it: a horizontal line has no height to climb, so the crease comes out level */
   int own = !inspectAxisOk(vr, refAxisDeg) ? 0 : ((vr.flags & vfAxisA) ? 1 : ((vr.flags & vfAxisB) ? 2 : 0));

   if (!own) // a measured horizontal off the room's (the tiles' diagonal) is not kept
      vr.flags &= (BYTE)~(vfAxisA | vfAxisB);

   if (!(vr.flags & vfVertical))
   {
      if (own)
      {
         TVec3 a = vr.dirCam[own];
         float la = sqrtf(a.x*a.x + a.y*a.y + a.z*a.z),
               d = (up.x*a.x + up.y*a.y + up.z*a.z)/(la*la);

         up.x -= d*a.x;
         up.y -= d*a.y;
         up.z -= d*a.z;

         float lu = sqrtf(up.x*up.x + up.y*up.y + up.z*up.z);

         up.x /= lu;
         up.y /= lu;
         up.z /= lu;
      }
      vr.dirCam[0] = up;
      vr.flags |= vfVertical;
   }
   if (!own)
   {
      vr.dirCam[1] = ax;
      vr.flags |= vfAxisA;
   }
}

/*--------------------------------------------------------------------------------
   --rectify, second pass: the ceiling creases level every frontal view (user, 2026-09-27: "as linhas
   de teto definem uma trajetória linear que precisa participar da normalização"). The first pass
   measured each frame's crease as its own vertical rotated it; now the view turns about its axis
   until the crease is flat (roll), and - center spin only, every frame from the same spot - about
   its horizontal axis until the crease stands at the median elevation of that wall's frames
   (pitch). Corner stations stand elsewhere: roll only. Printed before and after.
  --------------------------------------------------------------------------------*/
static void inspectRectifyLevel(LPCSTR sessionDir, LPCSTR outDir, const TVanishResult *frameVr, const DWORD *frameInfo,
                                int frames, float refAxisDeg)
{
   float medianElev[8]; // per wall, ceiling crease then floor crease

   for (int slot = 0; slot < 8; slot++)
   {
      float v[inspectMaxFrames];
      int   n = 0;

      for (int f = 0; f < frames && f < inspectMaxFrames; f++)
         if (inspectRect[f].ok && !inspectIsSuperseded(inspectStamp[f])
             && inspectRect[f].wall*2 + (inspectRect[f].floorLine ? 1 : 0) == slot
             && frameInfo[f]%inspectStations == 0u)
            v[n++] = inspectRect[f].elevDeg;
      for (int i = 1; i < n; i++)
      {
         float x = v[i];
         int   k = i - 1;

         while (k >= 0 && v[k] > x)
         {
            v[k + 1] = v[k];
            k--;
         }
         v[k + 1] = x;
      }
      medianElev[slot] = n ? v[n/2] : NAN;
   }

   if (inspectRectOwn) // the gyroscope's correction table first: the frames without lines inherit it
      inspectBuildInherit(sessionDir, frameVr, frameInfo, frames, refAxisDeg);
   if (inspectRectOwn && outDir) // each view's squareness, before and after the lines turned it
   {
      char csvPath[sessionPathMax];

      snprintf(csvPath, sizeof(csvPath), "%s/esquadro.csv", outDir);
      inspectSquareCsv = fopen(csvPath, "wb");
      if (inspectSquareCsv)
         fprintf(inspectSquareCsv, "view,leanBefore,slopeBefore,keystoneVBefore,keystoneHBefore,leanAfter,slopeAfter,"
                                   "keystoneVAfter,keystoneHAfter,meetXBefore,meetYBefore,meetXAfter,meetYAfter\r\n");
   }

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

      if (f >= frames || f >= inspectMaxFrames)
         break;

      if (inspectIsSuperseded(inspectStamp[f]))
         continue; // a replaced photo: no frontal view
      img.intr.fx *= inspectIntrScale;
      img.intr.fy *= inspectIntrScale;

      const TRectMeasure &m = inspectRect[f];
      bool                center = frameInfo[f]%inspectStations == 0u;
      int                 slot = m.wall*2 + (m.floorLine ? 1 : 0);
      float               target = medianElev[slot],
                          roll = m.ok ? cRectRollSign*atanf(m.slope)*57.29578f : 0.f,
                          pitch = m.ok && center && !isnan(target) ? cRectPitchSign*(m.elevDeg - target) : 0.f;
      TRectMeasure        after = {};

      /* no crease in this frame (furniture, a plain view): the gyroscope's error drifts slowly, so the corrections
         of the measured neighbors of the same station, a few frames before and after, are interpolated */
      if (!m.ok)
      {
         int prev = -1,
             next = -1;

         for (int k = f - 1; k >= 0 && k >= f - inspectRectReach && prev < 0; k--)
            if (inspectRect[k].ok && frameInfo[k] == frameInfo[f])
               prev = k;
         for (int k = f + 1; k < frames && k < inspectMaxFrames && k <= f + inspectRectReach && next < 0; k++)
            if (inspectRect[k].ok && frameInfo[k] == frameInfo[f])
               next = k;
         if (prev >= 0 && next >= 0)
         {
            float t = (float)(f - prev)/(float)(next - prev),
                  r0 = cRectRollSign*atanf(inspectRect[prev].slope)*57.29578f,
                  r1 = cRectRollSign*atanf(inspectRect[next].slope)*57.29578f;

            roll = r0 + (r1 - r0)*t;
            pitch = center ? inspectRectPitch(inspectRect[prev], medianElev)
                             + (inspectRectPitch(inspectRect[next], medianElev) - inspectRectPitch(inspectRect[prev], medianElev))*t
                           : 0.f;
            printf("  level frame %03d: interpolated from %03d and %03d, roll %+.2f pitch %+.2f deg\n", f, prev, next,
                   roll, pitch);
         }
      }

      /* the frame's own lines only (--rectify-own; user, 2026-09-30: the wall's normal from its horizontals, the plumb
         from its verticals, the homography about the image center): a measured vertical is not levelled again by the
         crease - 101732's frame 82 sees no ceiling, took its neighbors' pitch and its verticals converged */
      TVanishResult own = frameVr[f];

      if (inspectRectOwn) // a frame without lines of its own takes the table's turn
         inspectInheritVanish(img.cameraToWorld, f, refAxisDeg, own);
      if (inspectRectOwn && (own.flags & vfVertical))
      {
         roll = 0.f;
         pitch = 0.f;
      }
      inspectCornerCenter = center;
      inspectCornerFrame = f;
      inspectRectify(img, own, refAxisDeg, outDir, f, roll, pitch, &after, true);
      if (m.ok)
         printf("  level frame %03d wall %d %s: slope %+.4f -> %+.4f, elevation %.2f -> %.2f (median %.2f), roll %+.2f"
                " pitch %+.2f deg\n", f, m.wall, m.floorLine ? "floor" : "ceiling", m.slope,
                after.ok ? after.slope : NAN, m.elevDeg, after.ok ? after.elevDeg : NAN, target, roll, pitch);
   }
   if (inspectSquareCsv)
   {
      fclose(inspectSquareCsv);
      inspectSquareCsv = NULL;
   }
}

//--------------------------------------------------------------------------------
// One pass over the session: every kept --hidden-floor frame measured again with its hint
static void inspectHiddenPass(LPCSTR sessionDir, const THiddenHint *hints)
{
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

      for (int i = 0; i < inspectHiddenCount; i++)
         if (inspectHiddenData[i].index == f)
         {
            THiddenFrame &h = inspectHiddenData[i];
            TVanishEdges  edges = { h.ray(), h.label(), h.count, h.count };

            inspectHiddenFloor(img, h.vr, edges, f, &hints[i], h.res);
         }
   }
}

/*--------------------------------------------------------------------------------
   The station decides the floor lines of its corner (user, 2026-09-28). The frames of a station see
   the same corner from nearly the same spot: the one showing it nearest the picture's middle is
   the central frame, its view the widest. Its corner top, carried by the gyroscope, tells every
   neighbor which corner to look at (a frame turned aside may show another corner's moldings higher).
   Then every frame elects its floor lines among its own candidates, their votes raised by the
   neighbors' candidates the floor plane's homography lands on them (each frame placed against its
   own corner - the ceiling corner is the guide): the homography moves weight, never geometry.
  --------------------------------------------------------------------------------*/
static void inspectHiddenStations(LPCSTR sessionDir)
{
   THiddenHint hints[inspectHiddenMax];
   int         central[inspectHiddenMax];
   TVec3       refA[inspectHiddenMax];

   for (int i = 0; i < inspectHiddenCount; i++)
   {
      float best = 1e9f;

      central[i] = -1;
      for (int j = 0; j < inspectHiddenCount; j++)
      {
         const THiddenFrame &h = inspectHiddenData[j];
         float               off = fabsf(h.res.ccx - 0.5f*(float)h.height);

         if (h.station == inspectHiddenData[i].station && h.res.corner && off < best)
         {
            best = off;
            central[i] = j;
         }
      }
      hints[i].elect[0] = -2;
      hints[i].elect[1] = -2;
      hints[i].predX = NAN;
      hints[i].predY = NAN;
      hints[i].cameraM = NAN;
      hints[i].gateShare = cHiddenGateShare;
      if (central[i] < 0)
         continue;

      const THiddenFrame &c = inspectHiddenData[central[i]],
                         &h = inspectHiddenData[i];
      TVec3               top = c.pose.RotateVector(inspectUprightRay(c.intr, c.height, c.res.ccx, c.res.ccy)),
                          cam = { h.pose.m[0]*top.x + h.pose.m[1]*top.y + h.pose.m[2]*top.z,
                                  h.pose.m[4]*top.x + h.pose.m[5]*top.y + h.pose.m[6]*top.z,
                                  h.pose.m[8]*top.x + h.pose.m[9]*top.y + h.pose.m[10]*top.z };

      refA[i] = c.pose.RotateVector(c.vr.dirCam[1]);
      if (!inspectUprightAt(h.intr, h.height, cam, hints[i].predX, hints[i].predY))
      {
         hints[i].predX = NAN;
         hints[i].predY = NAN;
      }
      printf("  station %d frame %03d: central frame %03d, corner expected at (%.0f, %.0f), blind pass found (%.0f, %.0f)\n",
             h.station, h.index, c.index, hints[i].predX, hints[i].predY, h.res.corner ? h.res.ccx : NAN,
             h.res.corner ? h.res.ccy : NAN);
   }

   printf("  hidden floor: second pass, every frame on its station's corner\n");
   inspectHiddenPass(sessionDir, hints);

   /* the election (user, 2026-09-28): every frame keeps its OWN candidates - their geometry comes from its own edges
      only. The floor plane's homography carries a neighbor's candidates into the frame just to tell which of its own
      lines they are: a neighbor's line landing within cInheritTolShare of the corner's height from an own candidate
      lends it its vote (its strength, its wall's view in its frame, twice from the central frame). A short segment of
      a line seen whole and clean next door thus inherits the neighbor's weight; nothing is ever drawn from it */
   for (int g = 0; g < inspectHiddenCount; g++)
   {
      hints[g].elect[0] = -2;
      hints[g].elect[1] = -2;
      if (central[g] < 0 || !inspectHiddenData[g].res.corner)
         continue;

      const THiddenFrame &t = inspectHiddenData[g];
      float               h = inspectHiddenCameraM,
                          span = isnan(t.res.efy) ? (float)t.width : fabsf(t.res.efy - t.res.ccy),
                          tol = cInheritTolShare*span,
                          own[2][inspectPencilMax],
                          inh[2][inspectPencilMax],
                          from[2][inspectPencilMax][inspectHiddenMax];

      memset(inh, 0, sizeof(inh));
      memset(from, 0, sizeof(from));
      for (int k = 0; k < 2; k++)
      {
         for (int q = 0; q < t.res.cands[k]; q++)
            own[k][q] = t.res.cand[k][q][4]*t.res.share[k]*(g == central[g] ? 2.f : 1.f);
         for (int j = 0; j < inspectHiddenCount; j++)
         {
            const THiddenFrame &f = inspectHiddenData[j];

            if (j == g || central[j] != central[g] || !f.res.corner)
               continue;
            for (int p = 0; p < f.res.cands[k]; p++)
            {
               const float *cd = f.res.cand[k][p];
               float        x0,
                            y0,
                            x1,
                            y1,
                            x,
                            y,
                            closest = tol;
               int          at = -1;

               if (!inspectFloorTransfer(f, t, refA[g], h, cd[0], cd[1], x0, y0)
                   || !inspectFloorTransfer(f, t, refA[g], h, cd[2], cd[3], x1, y1)
                   || !inspectCross(t.res.ccx, t.res.ccy, t.res.vx[0] - t.res.ccx, t.res.vy[0] - t.res.ccy, x0, y0, x1 - x0,
                                    y1 - y0, x, y))
                  continue;
               for (int q = 0; q < t.res.cands[k]; q++) // the nearest own candidate takes it
                  if (fabsf(y - t.res.cand[k][q][1]) <= closest)
                  {
                     closest = fabsf(y - t.res.cand[k][q][1]);
                     at = q;
                  }
               if (at < 0)
                  continue;

               float vote = cd[4]*f.res.share[k]*(j == central[g] ? 2.f : 1.f);

               inh[k][at] += vote;
               from[k][at][j] += vote;
            }
         }
      }

      /* both walls' floor lines meet at one foot: a pair of own candidates crossing the vertical within
         cHiddenAgreeShare of the corner's height scores (sqrt A + sqrt B)^2, a wall alone its own total (the other
         wall hidden here) */
      int   bestA = -1,
            bestB = -1;
      float bestScore = 0.f;

      for (int a = -1; a < t.res.cands[0]; a++)
         for (int b = -1; b < t.res.cands[1]; b++)
         {
            if ((a >= 0 && own[0][a] <= 0.f) || (b >= 0 && own[1][b] <= 0.f)) // the lever lifts a weak own line, never a missing one
               continue;
            if (a >= 0 && b >= 0 && fabsf(t.res.cand[0][a][1] - t.res.cand[1][b][1]) > cHiddenAgreeShare*span)
               continue;

            float ta = a >= 0 ? sqrtf(own[0][a] + inh[0][a]) : 0.f,
                  tb = b >= 0 ? sqrtf(own[1][b] + inh[1][b]) : 0.f,
                  score = (ta + tb)*(ta + tb);

            if (score > bestScore)
            {
               bestScore = score;
               bestA = a;
               bestB = b;
            }
         }
      hints[g].elect[0] = bestA;
      hints[g].elect[1] = bestB;
      for (int k = 0; k < 2; k++)
      {
         int e = k ? bestB : bestA;

         printf("  station %d frame %03d, %s floor:", t.station, t.index, k ? "B" : "A");
         for (int q = 0; q < t.res.cands[k]; q++)
         {
            printf(" %s%.0f px own %.0f + inherited %.0f", q == e ? "[" : "", t.res.cand[k][q][1] - t.res.ccy, own[k][q],
                   inh[k][q]);
            for (int j = 0; j < inspectHiddenCount; j++)
               if (from[k][q][j] > 0.f)
                  printf(" %03d:%.0f", inspectHiddenData[j].index, from[k][q][j]);
            printf("%s;", q == e ? "]" : "");
         }
         printf("%s\n", e < 0 ? " elected none - hidden here" : "");
      }
   }

   printf("  hidden floor: last pass, the station's floor lines\n");
   inspectHiddenPass(sessionDir, hints);
}

/*--------------------------------------------------------------------------------
   A frame list such as "0,3,4,8-15,17-18" (keyframe numbers as capInspect extracts them, frame_NNN.jpg)
   into one flag per frame.
  --------------------------------------------------------------------------------*/
static void inspectParseList(LPCSTR text, LPBYTE flags)
{
   LPCSTR p = text;

   memset(flags, 0, inspectMaxFrames);
   while (*p)
   {
      LPSTR end = NULL;
      long  a = strtol(p, &end, 10),
            b = a;

      if (end == p)
         break;
      p = end;
      if (*p == '-')
      {
         b = strtol(p + 1, &end, 10);
         p = end;
      }
      for (long k = a; k <= b; k++)
         if (k >= 0 && k < inspectMaxFrames)
            flags[k] = 1u;
      if (*p == ',')
         p++;
   }
}

//--------------------------------------------------------------------------------
// --corner-frames: the frames the user named as corner frames
static void inspectParsePick(LPCSTR text)
{
   inspectPicking = true;
   inspectParseList(text, inspectPick);
}

/*--------------------------------------------------------------------------------
   Barrel distortion from the lines known to be straight (--distortion; user, 2026-09-29: a straight
   line of the scene - the ceiling's crease, a door frame, the wardrobe's edges - bends in the
   picture only by the lens: if they bend there is a barrel, else it is negligible). Every frame's
   edges at half resolution (Sobel, thinned across the gradient) are linked into chains whose
   direction turns slowly; the long, nearly straight ones are kept. The radial model undistorts a
   picture point, x_u = x_d (1 + k1 r^2 + k2 r^4) in focal units about the principal point (k1 > 0:
   a barrel), and k1 k2 are the ones that make every chain straightest at once - the sum of each
   chain's squared distances to its own best line (total least squares). Chains still bent after a
   first fit (a curved object) leave, and the fit runs again. Printed with the displacement at the
   picture's corner and edge; distorcao_linhas.csv holds every chain.
  --------------------------------------------------------------------------------*/
static const float cDistEdgeMin = 20.f,  // Sobel magnitude of an edge pixel (half resolution, gray 0..255)
                   cDistTurnCos = 0.94f, // neighbors of a chain turn at most ~20 degrees
                   cDistMinLen = 300.f,  // chain extent, half-resolution pixels
                   cDistMaxBend = 0.03f; // a chain bent beyond this share of its length at k = 0 is no straight line

enum {
   inspectDistChainMax = 6000,
   inspectDistPointMax = 1500000,
   inspectDistChainPx  = 40000 // pixels one chain holds at most
};

struct TDistChain {
   int   first,
         count,
         frame;
   float length, // half-resolution pixels
         radius, // mean distance from the principal point, focal units
         before, // rms distance to its own line, full-resolution pixels (k = 0)
         after;
   bool  used;
};

static TDistChain inspectDistChain[inspectDistChainMax];
static float      inspectDistPt[inspectDistPointMax*2]; // normalized distorted x y
static int        inspectDistChains = 0,
                  inspectDistPoints = 0;

//--------------------------------------------------------------------------------
// A chain's squared distances to its own best line, summed, its points undistorted by k1 k2
static float inspectDistCost(const TDistChain &c, float k1, float k2)
{
   const float *p = inspectDistPt + (size_t)c.first*2u;
   float        mx = 0.f,
                my = 0.f,
                cxx = 0.f,
                cyy = 0.f,
                cxy = 0.f,
                sum = 0.f;

   for (int pass = 0; pass < 3; pass++)
   {
      float nx = 0.f,
            ny = 0.f;

      if (pass == 2) // the major axis: its normal is the line's
      {
         float t = 0.5f*atan2f(2.f*cxy, cxx - cyy);

         nx = -sinf(t);
         ny = cosf(t);
      }
      for (int i = 0; i < c.count; i++)
      {
         float x = p[2*i],
               y = p[2*i + 1],
               r2 = x*x + y*y,
               s = 1.f + k1*r2 + k2*r2*r2;

         x *= s;
         y *= s;
         if (pass == 0)
         {
            mx += x;
            my += y;
         }
         else if (pass == 1)
         {
            cxx += (x - mx)*(x - mx);
            cyy += (y - my)*(y - my);
            cxy += (x - mx)*(y - my);
         }
         else
         {
            float e = (x - mx)*nx + (y - my)*ny;

            sum += e*e;
         }
      }
      if (pass == 0)
      {
         mx /= (float)c.count;
         my /= (float)c.count;
      }
   }
   return sum;
}

//--------------------------------------------------------------------------------
// All used chains' cost at k1 k2: rms distance in full-resolution pixels
static float inspectDistRms(float k1, float k2, float fx)
{
   float sum = 0.f;
   int   n = 0;

   for (int i = 0; i < inspectDistChains; i++)
      if (inspectDistChain[i].used)
      {
         sum += inspectDistCost(inspectDistChain[i], k1, k2);
         n += inspectDistChain[i].count;
      }
   return n ? sqrtf(sum/(float)n)*fx : 0.f;
}

//--------------------------------------------------------------------------------
// One frame's long, nearly straight edge chains into inspectDistChain (gray at half resolution)
static void inspectDistFrame(const float *g, int w, int h, const TImageRecord &img, int frame)
{
   TAlloc<float> ux((size_t)w*h),
                 uy((size_t)w*h),
                 mag((size_t)w*h);
   TAlloc<BYTE>  edge((size_t)w*h);
   TAlloc<int>   chain((size_t)inspectDistChainPx),
                 stack((size_t)inspectDistChainPx);

   memset(mag(), 0, sizeof(float)*(size_t)w*h);
   memset(ux(), 0, sizeof(float)*(size_t)w*h);
   memset(uy(), 0, sizeof(float)*(size_t)w*h);
   memset(edge(), 0, (size_t)w*h);
   for (int y = 1; y + 1 < h; y++)
      for (int x = 1; x + 1 < w; x++)
      {
         const float *q = g + (size_t)y*w + x;
         float        gx = (q[-w + 1] + 2.f*q[1] + q[w + 1]) - (q[-w - 1] + 2.f*q[-1] + q[w - 1]),
                      gy = (q[w - 1] + 2.f*q[w] + q[w + 1]) - (q[-w - 1] + 2.f*q[-w] + q[-w + 1]),
                      m = sqrtf(gx*gx + gy*gy);
         size_t       at = (size_t)y*w + x;

         mag[at] = m;
         ux[at] = m > 0.f ? gx/m : 0.f;
         uy[at] = m > 0.f ? gy/m : 0.f;
      }
   for (int y = 2; y + 2 < h; y++) // thinned across the gradient
      for (int x = 2; x + 2 < w; x++)
      {
         size_t at = (size_t)y*w + x;
         float  m = mag[at],
                ax = fabsf(ux[at]),
                ay = fabsf(uy[at]);
         int    dx = ax > 2.414f*ay ? 1 : (ay > 2.414f*ax ? 0 : (ux[at]*uy[at] > 0.f ? 1 : -1)),
                dy = ax > 2.414f*ay ? 0 : 1,
                off = dy*w + dx;

         if (m >= cDistEdgeMin && m >= mag[(size_t)((int)at + off)] && m > mag[(size_t)((int)at - off)])
            edge[at] = 1;
      }
   for (size_t seed = 0; seed < (size_t)w*h && inspectDistChains < inspectDistChainMax; seed++)
   {
      if (edge[seed] != 1)
         continue;

      int n = 0,
          top = 0;

      edge[seed] = 2;
      stack[top++] = (int)seed;
      while (top > 0)
      {
         int at = stack[--top],
             x = at%w,
             y = at/w;

         if (n < inspectDistChainPx)
            chain[n++] = at;
         for (int dy = -1; dy <= 1; dy++)
            for (int dx = -1; dx <= 1; dx++)
            {
               int nb = (y + dy)*w + x + dx;

               if ((dx || dy) && edge[nb] == 1 && ux[at]*ux[nb] + uy[at]*uy[nb] >= cDistTurnCos && top < inspectDistChainPx)
               {
                  edge[nb] = 2;
                  stack[top++] = nb;
               }
            }
      }
      if ((float)n < cDistMinLen || inspectDistPoints + n > inspectDistPointMax)
         continue;

      // straight enough at k = 0: its extent along the major axis, and its bend across
      float mx = 0.f,
            my = 0.f,
            cxx = 0.f,
            cyy = 0.f,
            cxy = 0.f,
            lo = 1e9f,
            hi = -1e9f,
            bend = 0.f;

      for (int i = 0; i < n; i++)
      {
         mx += (float)(chain[i]%w);
         my += (float)(chain[i]/w);
      }
      mx /= (float)n;
      my /= (float)n;
      for (int i = 0; i < n; i++)
      {
         float x = (float)(chain[i]%w) - mx,
               y = (float)(chain[i]/w) - my;

         cxx += x*x;
         cyy += y*y;
         cxy += x*y;
      }

      float t = 0.5f*atan2f(2.f*cxy, cxx - cyy),
            ax = cosf(t),
            ay = sinf(t);

      for (int i = 0; i < n; i++)
      {
         float x = (float)(chain[i]%w) - mx,
               y = (float)(chain[i]/w) - my,
               a = x*ax + y*ay;

         lo = fminf(lo, a);
         hi = fmaxf(hi, a);
         bend = fmaxf(bend, fabsf(-x*ay + y*ax));
      }
      if (hi - lo < cDistMinLen || bend > cDistMaxBend*(hi - lo))
         continue;

      TDistChain &c = inspectDistChain[inspectDistChains++];
      float       rs = 0.f;

      c.first = inspectDistPoints;
      c.count = n;
      c.frame = frame;
      c.length = hi - lo;
      c.used = true;
      for (int i = 0; i < n; i++)
      {
         float X = (2.f*(float)(chain[i]%w) + 0.5f - img.intr.cx)/img.intr.fx,
               Y = (2.f*(float)(chain[i]/w) + 0.5f - img.intr.cy)/img.intr.fy;

         inspectDistPt[(size_t)inspectDistPoints*2u] = X;
         inspectDistPt[(size_t)inspectDistPoints*2u + 1u] = Y;
         inspectDistPoints++;
         rs += sqrtf(X*X + Y*Y);
      }
      c.radius = rs/(float)n;
   }
}

//--------------------------------------------------------------------------------
// The best k1 k2 on a grid about (k1, k2): steps of s1 and s2, span steps each way
static float inspectDistSearch(float &k1, float &k2, float s1, float s2, int span1, int span2, float fx)
{
   float b1 = k1,
         b2 = k2,
         best = inspectDistRms(k1, k2, fx);

   for (int i = -span1; i <= span1; i++)
      for (int j = -span2; j <= span2; j++)
      {
         float a = k1 + s1*(float)i,
               b = k2 + s2*(float)j,
               e = inspectDistRms(a, b, fx);

         if (e < best)
         {
            best = e;
            b1 = a;
            b2 = b;
         }
      }
   k1 = b1;
   k2 = b2;
   return best;
}

//--------------------------------------------------------------------------------
static void inspectDistortion(LPCSTR sessionDir, LPCSTR outDir)
{
   TSessionReader reader;
   TRecordView    v;
   int            index = 0;
   float          fx = 0.f,
                  cornerR = 0.f,
                  edgeR = 0.f;

   inspectDistChains = 0;
   inspectDistPoints = 0;
   if (!reader.Open(sessionDir))
      return;
   while (reader.Next(v))
   {
      TImageRecord img;

      if (v.type != rtImage || !img.Decode(v.payload, v.length))
         continue;

      int          f = index++,
                   w = (int)img.width/2,
                   h = (int)img.height/2;
      TAlloc<BYTE> bgr((size_t)img.width*img.height*3u);

      if (!inspectDecode(img.pixels, img.pixelBytes, img.width, img.height, true, bgr))
         continue;

      TAlloc<float> g((size_t)w*h);

      for (int y = 0; y < h; y++)
         for (int x = 0; x < w; x++)
         {
            float s = 0.f;

            for (int k = 0; k < 4; k++)
            {
               LPCBYTE q = bgr() + ((size_t)(2*y + k/2)*img.width + 2*x + k%2)*3u;

               s += 0.114f*(float)q[0] + 0.587f*(float)q[1] + 0.299f*(float)q[2];
            }
            g[(size_t)y*w + x] = 0.25f*s;
         }
      inspectDistFrame(g(), w, h, img, f);
      fx = img.intr.fx;
      cornerR = sqrtf((img.intr.cx/img.intr.fx)*(img.intr.cx/img.intr.fx) + (img.intr.cy/img.intr.fy)*(img.intr.cy/img.intr.fy));
      edgeR = img.intr.cy/img.intr.fy;
   }
   if (!inspectDistChains || fx <= 0.f)
   {
      printf("  distortion: no straight chains\n");
      return;
   }

   float k1 = 0.f,
         k2 = 0.f,
         zero = 0.f,
         fit = 0.f;
   int   used = inspectDistChains;

   for (int round = 0; round < 2; round++)
   {
      TAlloc<float> e((size_t)inspectDistChains);
      int           n = 0;

      k1 = 0.f;
      k2 = 0.f;
      inspectDistSearch(k1, k2, 0.005f, 0.f, 60, 0, fx); // k1 alone over -0.3..0.3, then both, finer
      inspectDistSearch(k1, k2, 0.0025f, 0.01f, 12, 30, fx);
      fit = inspectDistSearch(k1, k2, 0.0005f, 0.002f, 6, 6, fx);
      for (int i = 0; i < inspectDistChains; i++)
      {
         TDistChain &c = inspectDistChain[i];

         c.before = sqrtf(inspectDistCost(c, 0.f, 0.f)/(float)c.count)*fx;
         c.after = sqrtf(inspectDistCost(c, k1, k2)/(float)c.count)*fx;
         if (c.used)
            e[n++] = c.after;
      }

      float knee = 3.f*inspectQuantile(e(), n, 0.5f);

      used = 0;
      for (int i = 0; i < inspectDistChains && round == 0; i++) // a chain still bent is a curved object
         inspectDistChain[i].used = inspectDistChain[i].after <= knee;
      for (int i = 0; i < inspectDistChains; i++)
         used += inspectDistChain[i].used ? 1 : 0;
      zero = inspectDistRms(0.f, 0.f, fx);
   }

   // the straightness by distance from the center: a barrel bends the lines toward the edges
   float band[3][3] = {}; // before, after, chains - center, middle, edge

   for (int i = 0; i < inspectDistChains; i++)
   {
      const TDistChain &c = inspectDistChain[i];
      int               b = c.radius < 0.25f ? 0 : (c.radius < 0.5f ? 1 : 2);

      if (!c.used)
         continue;
      band[b][0] += c.before;
      band[b][1] += c.after;
      band[b][2] += 1.f;
   }
   printf("  distortion: %d straight chains (%d kept) from %d frames, %d points\n", inspectDistChains, used, index,
          inspectDistPoints);
   printf("  distortion: k1 %.4f k2 %.4f (undistort x_u = x_d (1 + k1 r^2 + k2 r^4), focal units; k1 > 0 a barrel)\n", k1, k2);
   printf("  distortion: straightness rms %.3f px with no distortion -> %.3f px fitted\n", zero, fit);
   for (int b = 0; b < 3; b++)
      if (band[b][2] > 0.f)
         printf("  distortion: chains %s: %d, rms %.3f -> %.3f px\n",
                b == 0 ? "near the center" : (b == 1 ? "midway" : "near the edges"), (int)band[b][2], band[b][0]/band[b][2],
                band[b][1]/band[b][2]);
   for (int s = 0; s < 2; s++)
   {
      float r = s ? cornerR : edgeR,
            r2 = r*r;

      printf("  distortion: at the %s (r %.3f) a point moves %.1f px\n", s ? "corner" : "top edge's middle", r,
             r*(k1*r2 + k2*r2*r2)*fx);
   }

   char path[sessionPathMax];

   snprintf(path, sizeof(path), "%s/distorcao_linhas.csv", outDir);

   FILE *csv = fopen(path, "wb");

   if (csv)
   {
      fprintf(csv, "chain,frame,points,lengthPx,radius,beforePx,afterPx,used\r\n");
      for (int i = 0; i < inspectDistChains; i++)
      {
         const TDistChain &c = inspectDistChain[i];

         fprintf(csv, "%d,%d,%d,%.1f,%.4f,%.3f,%.3f,%d\r\n", i, c.frame, c.count, 2.f*c.length, c.radius, c.before, c.after,
                 c.used ? 1 : 0);
      }
      fclose(csv);
   }
}

//--------------------------------------------------------------------------------
int main(int argc, LPSTR *argv)
{
   abSetIdlePriority();
   if (argc < 3)
   {
      fprintf(stderr, "usage: capInspect <sessionDir> <outDir> [--vanish] [--ceiling meters] [--mingrad n] [--edges] [--rectify] [--blur]\n");
      return 2;
   }

   TSessionReader reader;
   TRecordView    v;
   char           path[sessionPathMax];
   QWORD          counts[rtDoor + 1] = {},
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
                  overlay = false,
                  measureBlur = false,
                  floorViews = false,
                  panorama = false,
                  faces = false,
                  distortion = false;
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
      else if (!strcmp(argv[i], "--intr-scale") && i + 1 < argc)
         inspectIntrScale = (float)atof(argv[++i]);
      else if (!strcmp(argv[i], "--roll-only"))
         inspectRollOnly = true;
      else if (!strcmp(argv[i], "--diagonal-floor"))
         inspectDiagonalFloor = true;
      else if (!strcmp(argv[i], "--merge"))
         inspectMergeOn = true;
      else if (!strcmp(argv[i], "--third"))
         inspectThirdAxis = true;
      else if (!strcmp(argv[i], "--rectify-own"))
      {
         rectify = true;
         inspectRectOwn = true;
      }
      else if (!strcmp(argv[i], "--rectify"))
         rectify = true;
      else if (!strcmp(argv[i], "--doors-all"))
      {
         inspectDoors = true;
         inspectDoorsAll = true;
      }
      else if (!strcmp(argv[i], "--doors"))
         inspectDoors = true;
      else if (!strcmp(argv[i], "--door-tilt") && i + 1 < argc)
         inspectDoorTiltDeg = (float)atof(argv[++i]);
      else if (!strcmp(argv[i], "--blur-noise") && i + 1 < argc)
         inspectBlurNoise = (float)atof(argv[++i]);
      else if (!strcmp(argv[i], "--walls"))
         walls = true;
      else if (!strcmp(argv[i], "--creases"))
         inspectShowCreases = true;
      else if (!strcmp(argv[i], "--blur"))
         measureBlur = true;
      else if (!strcmp(argv[i], "--corner-frames") && i + 1 < argc)
         inspectParsePick(argv[++i]);
      else if (!strcmp(argv[i], "--face-names") && i + 1 < argc)
         mosaicSetFaceNames(argv[++i]);
      else if (!strcmp(argv[i], "--hidden-floor") && i + 1 < argc)
         inspectParseList(argv[++i], inspectHidden);
      else if (!strcmp(argv[i], "--hidden-heights") && i + 2 < argc)
      {
         inspectHiddenCeilingM = (float)atof(argv[++i]);
         inspectHiddenCameraM = (float)atof(argv[++i]);
      }
      else if (!strcmp(argv[i], "--free-plan"))
         inspectFreePlan = true;
      else if (!strcmp(argv[i], "--focal-scale") && i + 1 < argc)
         inspectFocalScale = (float)atof(argv[++i]);
      else if (!strcmp(argv[i], "--face-frames"))
      {
         inspectFaceFramesOn = true;
         faces = true;
      }
      else if (!strcmp(argv[i], "--no-plumb"))
         inspectPlumbOn = false;
      else if (!strcmp(argv[i], "--pair-scan") && i + 1 < argc && sscanf(argv[i + 1], "%d,%d", &inspectPairA, &inspectPairB) == 2)
         i++;
      else if (!strcmp(argv[i], "--circle"))
         inspectCircleFlat = true;
      else if (!strcmp(argv[i], "--circle-heights"))
         inspectCircleHeights = true;
      else if (!strcmp(argv[i], "--vp-solve"))
         inspectVpSolveOn = true;
      else if (!strcmp(argv[i], "--pair-debug"))
         inspectPairDebug = true;
      else if (!strcmp(argv[i], "--pairs"))
         inspectPairsOn = true;
      else if (!strcmp(argv[i], "--dt-scan"))
         inspectDtScan = true;
      else if (!strcmp(argv[i], "--pose-dt") && i + 1 < argc)
         inspectPoseDtSec = (float)atof(argv[++i]);
      else if (!strcmp(argv[i], "--no-focal"))
         inspectFocalOn = false;
      else if (!strcmp(argv[i], "--no-fixed-pattern"))
         inspectPanoFixedOn = false;
      else if (!strcmp(argv[i], "--pano-top") && i + 1 < argc)
         inspectPanoTop = atoi(argv[++i]);
      else if (!strcmp(argv[i], "--no-lever"))
         inspectLeverOn = false;
      else if (!strcmp(argv[i], "--lever-scan"))
         inspectLeverScan = true;
      else if (!strcmp(argv[i], "--distortion"))
         distortion = true;
      else if (!strcmp(argv[i], "--faces"))
         faces = true;
      else if (!strcmp(argv[i], "--panorama"))
         panorama = true;
      else if (!strcmp(argv[i], "--floor"))
         floorViews = true;
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

   inspectRectOut = rectify;
   inspectOutDir = argv[2];
   inspectReadElections(argv[1]);
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
   if (distortion) // the lens alone: nothing else of the session
   {
      inspectDistortion(argv[1], argv[2]);
#ifdef _WIN32
      CoUninitialize();
#endif
      return 0;
   }
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
                   "edges,fwdX,fwdY,fwdZ,rollDeg,blurPx,blurMinPx,blurTextured,elected\r\n");
   fprintf(poses, "sensorNs,headingDeg,pitchDeg,tracking\r\n");
   while (reader.Next(v))
   {
      if ((int)v.type <= (int)rtDoor)
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
      else if (v.type == rtDoor)
      {
         TDoorRecord dr;

         if (dr.Decode(v.payload, v.length))
            printf("  door %u of room %lu: %s at u %.2f w %.2f, crease/head %.3f (%d frames) -> ceiling line %.2f m\n",
                   (unsigned)dr.index, (unsigned long)dr.roomIndex,
                   dr.state == dsConfirmed ? "confirmed" : (dr.state == dsDropped ? "dropped" : "not shot"), dr.u, dr.w,
                   dr.ratio, dr.ratios, 2.1f*dr.ratio);
      }
      else if (v.type == rtStation)
      {
         TStationRecord st;

         if (st.Decode(v.payload, v.length))
         {
            if (st.kind == skCenter && st.event == seBegin)
               printf("  station %lu: center spin\n", (unsigned long)st.index);
            else if (st.kind == skDoor)
               printf("  station %lu %s: doors\n", (unsigned long)st.index, st.event == seBegin ? "begin" : "end");
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
         img.intr.fx *= inspectIntrScale;
         img.intr.fy *= inspectIntrScale;
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
            snprintf(info.software, sizeof(info.software), "Sorena LiDAR capInspect");
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
         // captures stamped by the app under an earlier name: the EXIF Software tag carries the project's own
         TAlloc<BYTE> renamed(outLen);

         memcpy(renamed(), outBytes, outLen);
         if (inspectEXIFSoftware(renamed(), outLen))
            outBytes = renamed();
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

               bool solvedRoom = inspectPlan(argv[2], statRoom, layout, axis, ceilingM, plan);

               if (panorama || faces)
               {
                  bool registered;

                  if (panorama)
                     inspectPanorama(argv[1], argv[2], statRoom, frameInfo(), images, false);
                  inspectFrameVr = frameVr();
                  registered = inspectPanoRegister(argv[1], statRoom, frameInfo(), images);
                  if (panorama && registered)
                     inspectPanorama(argv[1], argv[2], statRoom, frameInfo(), images, true);
                  if (faces && solvedRoom)
                     inspectFacesSolved(argv[1], argv[2], statRoom, plan, frameInfo(), images);
               }
               if (floorViews)
                  inspectFloorViews(argv[1], argv[2], statRoom, plan, solvedRoom,
                                    solvedRoom ? plan.axisDeg : layout.AnchorDeg(), frameVr(), frameInfo(), images,
                                    spinRadiusM);
            }
            roomStats = TInspectAxis();
            axis.Reset();
            layout.Reset();
            statRoom = room;
         }
         snprintf(path, sizeof(path), "%s/edges_%03d.bmp", argv[2], images);
         inspectMeasure(img, meta, hasMeta, hasAppVanish ? &appVanish : NULL, force, vcfg, axis, edges, layout,
                        tilt, edgeImages ? path : NULL, rectify || inspectDoors ? argv[2] : NULL, images, vm,
                        images < inspectMaxFrames ? frameVr[images] : scratchVr);
         if (images < inspectMaxFrames)
         {
            frameInfo[images] = (DWORD)(hasMeta ? meta.roomIndex : 0u)*inspectStations + (hasMeta ? meta.stationIndex : 0u);
            inspectKind[images] = hasMeta ? meta.stationKind : (BYTE)skCenter;
            inspectBand[images] = hasMeta ? meta.spinBand : 0u;
            inspectBin[images] = hasMeta ? meta.spinBin : 0u;
         }
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
         fprintf(frames, "%u,%u,%.2f,%.2f,%.2f,%.2f,%lu,%lu,%lu,%lu,", (unsigned)vm.flags, (unsigned)vm.verdict,
                 vm.roomAxisDeg, vm.deviationDeg, vm.tiltErrDeg, vm.orthoErrDeg, (unsigned long)vm.support[0],
                 (unsigned long)vm.support[1], (unsigned long)vm.support[2], (unsigned long)vm.edges);

         // the gyroscope attitude as the frame recorded it (older blocks: derived from their pose)
         TVec3 fwd = img.cameraToWorld.Forward();
         float roll = geomRollDeg(img.cameraToWorld);

         if (hasMeta)
         {
            fwd.x = meta.forward[0];
            fwd.y = meta.forward[1];
            fwd.z = meta.forward[2];
            roll = meta.rollDeg;
         }
         fprintf(frames, "%.5f,%.5f,%.5f,%.2f,", fwd.x, fwd.y, fwd.z, roll);

         // blur as the app measured it on the raw luma, or (--blur) measured here on the decoded JPEG
         TBlurResult br = {};

         br.sharpPx = hasMeta ? meta.blurMinPx : NAN;
         br.medianPx = hasMeta ? meta.blurPx : NAN;
         br.textured = -1;
         if (measureBlur)
         {
            TAlloc<BYTE> luma((size_t)img.width*img.height);

            if (inspectDecode(img.pixels, img.pixelBytes, img.width, img.height, false, luma))
            {
               DWORD seed = 777u;

               // --blur-noise: sensor grain added back (the JPEG smoothed it; the app measures the raw luma)
               for (size_t p = 0; inspectBlurNoise > 0.f && p < (size_t)img.width*img.height; p++)
               {
                  float g = 0.f;

                  for (int k = 0; k < 4; k++)
                  {
                     seed = seed*1664525u + 1013904223u;
                     g += (float)(seed >> 8)/16777216.f - 0.5f;
                  }

                  float v = (float)luma[p] + g*1.732f*inspectBlurNoise; // 4 uniforms: variance 1/3, scaled to sigma

                  luma[p] = (BYTE)(v < 0.f ? 0.f : (v > 255.f ? 255.f : v + 0.5f));
               }
               blurMeasure(luma(), (int)img.width, (int)img.height, (int)img.width, br);
               printf("  blur frame %03d: %5.2f px (median %5.2f, %d/%d textured) tiles", images, br.sharpPx,
                      br.medianPx, br.textured, br.tiles);
               for (int t = 0; t < blurGridSide*blurGridSide; t++)
                  printf(" %5.2f", br.tilePx[t]);
               printf("\n");
            }
         }
         fprintf(frames, "%.2f,%.2f,%d,%d\r\n", br.medianPx, br.sharpPx, br.textured,
                 inspectIsSuperseded(v.stampNs) ? 0 : 1); // 0: a retake replaced it, or it lost to the bin's photo
         if (images < inspectMaxFrames)
            inspectStamp[images] = v.stampNs;
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
   if (inspectHiddenCount)
      inspectHiddenStations(argv[1]); // the corner frames of each station decide their floor lines together
   if (statRoom != ~0ul)
   {
      inspectAxisPrint("room", roomStats);

      bool solvedRoom = inspectPlan(argv[2], statRoom, layout, axis, ceilingM, plan);

      if (solvedRoom)
      {
         if (walls)
            inspectComposeWalls(argv[1], argv[2], statRoom, plan, frameVr(), frameInfo(), images, spinRadiusM,
                                refineRounds, bundleRounds, jointRounds);
         if (overlay)
            inspectOverlay(argv[1], argv[2], statRoom, plan, frameVr(), frameInfo(), images, spinRadiusM);
      }
      if (panorama || faces)
      {
         bool registered;

         if (panorama)
            inspectPanorama(argv[1], argv[2], statRoom, frameInfo(), images, false);
         inspectFrameVr = frameVr();
         registered = inspectPanoRegister(argv[1], statRoom, frameInfo(), images);
         if (panorama && registered)
            inspectPanorama(argv[1], argv[2], statRoom, frameInfo(), images, true);
         if (faces && solvedRoom)
            inspectFacesSolved(argv[1], argv[2], statRoom, plan, frameInfo(), images);
      }
      if (floorViews)
         inspectFloorViews(argv[1], argv[2], statRoom, plan, solvedRoom, solvedRoom ? plan.axisDeg : layout.AnchorDeg(),
                           frameVr(), frameInfo(), images, spinRadiusM);
      inspectCornerPlan = solvedRoom ? &plan : NULL;
      if (rectify)
         inspectRectifyLevel(argv[1], argv[2], frameVr(), frameInfo(), images,
                             axis.HasReference() ? axis.ReferenceDeg() : NAN);
      if (rectify && inspectMergeOn)
         inspectMergeWrite(argv[2]);
      inspectAxisPrint("session", allStats);
   }
#ifdef _WIN32
   CoUninitialize();
#endif
   fclose(frames);
   fclose(poses);
   printf("records:");
   for (int t = 1; t <= (int)rtDoor; t++)
      printf(" %s=%llu", inspectTypeName((TRecordType)t), (unsigned long long)counts[t]);
   printf("\nspan: %.1f s, truncated tail: %s\n", (float)(lastNs - firstNs)*1e-9f, reader.Truncated() ? "yes" : "no");
   return 0;
}

//--------------------------------------------------------------------------------
