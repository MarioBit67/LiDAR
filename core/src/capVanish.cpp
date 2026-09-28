#include "capVanish.h"
#include "alloc.h"
#include "libDiscipline.h"

static const float cDegToRad = 0.01745329252f,
                   cRadToDeg = 57.29577951f,
                   cStrongOrthoDeg = 3.f, // a room reference needs both axes this close to orthogonal
                   cMaxBiasDeg = 8.f;     // a tilt correction above this is a wrong vertical, not the camera mounting

enum {
   vanishRefinePasses = 3,
   tiltMinSupport     = 200, // vertical edges a frame needs to teach the tilt bias
   tiltMinFrames      = 3,
   tiltWindow         = 20,
   vanishEigenIters   = 12,
   vanishWindow       = 2  // structure tensor half window (5x5)
};

enum TEdgeLabel {
   elNone     = 0,
   elVertical = 1
};

//--------------------------------------------------------------------------------
TVanishConfig TVanishConfig::Default(void)
{
   TVanishConfig c = {};

   c.maxSide = 800;
   c.minGrad = 128;
   c.minSupport = 60;
   c.maxEdges = 40000;
   c.minCoherence = 0.85f;
   c.tolDeg = 1.5f;
   c.stepDeg = 0.5f;
   c.maxTiltDeg = 4.f;
   return c;
}

//--------------------------------------------------------------------------------
static float vanishDot(const TVec3 &a, const TVec3 &b)
{
   return a.x*b.x + a.y*b.y + a.z*b.z;
}

//--------------------------------------------------------------------------------
static TVec3 vanishCross(const TVec3 &a, const TVec3 &b)
{
   TVec3 r;

   r.x = a.y*b.z - a.z*b.y;
   r.y = a.z*b.x - a.x*b.z;
   r.z = a.x*b.y - a.y*b.x;
   return r;
}

//--------------------------------------------------------------------------------
static bool vanishNormalize(TVec3 &v)
{
   float len = sqrtf(vanishDot(v, v));

   if (len < 1e-12f)
      return false;
   v.x /= len;
   v.y /= len;
   v.z /= len;
   return true;
}

//--------------------------------------------------------------------------------
// Unsigned angle between two lines through the origin, degrees [0, 90]
static float vanishLineAngleDeg(const TVec3 &a, const TVec3 &b)
{
   TVec3 c = vanishCross(a, b);

   return atan2f(sqrtf(vanishDot(c, c)), fabsf(vanishDot(a, b)))*cRadToDeg;
}

//--------------------------------------------------------------------------------
// World direction into camera axes (the transpose of the pose rotation)
static TVec3 vanishWorldToCam(const TMat4 &pose, const TVec3 &w)
{
   TVec3 r;

   r.x = pose.m[0]*w.x + pose.m[1]*w.y + pose.m[2]*w.z;
   r.y = pose.m[4]*w.x + pose.m[5]*w.y + pose.m[6]*w.z;
   r.z = pose.m[8]*w.x + pose.m[9]*w.y + pose.m[10]*w.z;
   return r;
}

//--------------------------------------------------------------------------------
// Horizontal room axis of world heading hDeg, in camera axes, orthogonal to the camera-frame vertical up
static TVec3 vanishAxisAt(const TMat4 &pose, const TVec3 &up, float hDeg)
{
   float h = hDeg*cDegToRad;
   TVec3 w = { sinf(h), 0.f, -cosf(h) },
         a = vanishWorldToCam(pose, w);
   float d = vanishDot(a, up);

   a.x -= d*up.x;
   a.y -= d*up.y;
   a.z -= d*up.z;
   vanishNormalize(a);
   return a;
}

//--------------------------------------------------------------------------------
TVec3 vanishRotate(const TVec3 &v, const TVec3 &w)
{
   float angle = sqrtf(vanishDot(w, w));

   if (angle < 1e-9f)
      return v;

   TVec3 k = { w.x/angle, w.y/angle, w.z/angle },
         kv = vanishCross(k, v);
   float c = cosf(angle),
         s = sinf(angle),
         kd = vanishDot(k, v)*(1.f - c);
   TVec3 r = { v.x*c + kv.x*s + k.x*kd, v.y*c + kv.y*s + k.y*kd, v.z*c + kv.z*s + k.z*kd };

   return r;
}

//--------------------------------------------------------------------------------
TVec3 vanishRotationBetween(const TVec3 &a, const TVec3 &b)
{
   TVec3 axis = vanishCross(a, b);
   float s = sqrtf(vanishDot(axis, axis)),
         angle = atan2f(s, vanishDot(a, b));

   if (s < 1e-9f)
   {
      TVec3 zero = { 0.f, 0.f, 0.f };

      return zero;
   }
   axis.x *= angle/s;
   axis.y *= angle/s;
   axis.z *= angle/s;
   return axis;
}

//--------------------------------------------------------------------------------
float vanishAxisDiffDeg(float a, float b)
{
   float d = fmodf(a - b, 90.f);

   if (d > 45.f)
      d -= 90.f;
   else if (d <= -45.f)
      d += 90.f;
   return d;
}

//--------------------------------------------------------------------------------
static float vanishAxisWrap(float deg)
{
   float d = fmodf(deg, 90.f);

   if (d < 0.f)
      d += 90.f;
   return d;
}

//--------------------------------------------------------------------------------
// Box-downsamples the luma by an integer factor
static void vanishShrink(LPCBYTE luma, int stride, int factor, LPBYTE out, int ow, int oh)
{
   DWORD area = (DWORD)(factor*factor);

   for (int y = 0; y < oh; y++)
      for (int x = 0; x < ow; x++)
      {
         DWORD sum = 0u;

         for (int dy = 0; dy < factor; dy++)
         {
            LPCBYTE row = luma + (size_t)(y*factor + dy)*stride + (size_t)x*factor;

            for (int dx = 0; dx < factor; dx++)
               sum += row[dx];
         }
         out[(size_t)y*ow + x] = (BYTE)(sum/area);
      }
}

//--------------------------------------------------------------------------------
static void vanishScharr(LPCBYTE img, int w, int h, int *gx, int *gy)
{
   memset(gx, 0, sizeof(int)*(size_t)w*h);
   memset(gy, 0, sizeof(int)*(size_t)w*h);
   for (int y = 1; y < h - 1; y++)
      for (int x = 1; x < w - 1; x++)
      {
         LPCBYTE p = img + (size_t)y*w + x;
         int     tl = p[-w - 1],
                 tc = p[-w],
                 tr = p[-w + 1],
                 ml = p[-1],
                 mr = p[1],
                 bl = p[w - 1],
                 bc = p[w],
                 br = p[w + 1];

         gx[(size_t)y*w + x] = 3*(tr - tl) + 10*(mr - ml) + 3*(br - bl);
         gy[(size_t)y*w + x] = 3*(bl - tl) + 10*(bc - tc) + 3*(br - tr);
      }
}

//--------------------------------------------------------------------------------
static int vanishMag(const int *gx, const int *gy, size_t i)
{
   return abs(gx[i]) + abs(gy[i]);
}

//--------------------------------------------------------------------------------
// Thin edges: the magnitude peaks across the (quantized) gradient direction
static bool vanishPeak(const int *gx, const int *gy, int w, int x, int y, int minGrad)
{
   size_t i = (size_t)y*w + x;
   int    m = vanishMag(gx, gy, i),
          ax = abs(gx[i]),
          ay = abs(gy[i]);
   size_t step = 1u;

   if (m < minGrad)
      return false;
   if (ay > 2*ax)
      step = (size_t)w;
   else if (ax <= 2*ay)
      step = (gx[i] > 0) == (gy[i] > 0) ? (size_t)w + 1u : (size_t)w - 1u;
   return m > vanishMag(gx, gy, i - step) && m >= vanishMag(gx, gy, i + step);
}

/*--------------------------------------------------------------------------------
   Edge pixel -> unit normal of its interpretation plane. The tangent comes from the 5x5
   structure tensor (about a degree of accuracy); incoherent pixels (corners, texture) fail.
  --------------------------------------------------------------------------------*/
static bool vanishEdgeNormal(const int *gx, const int *gy, int w, int x, int y, int factor, const TIntrinsics &k,
                             float minCoherence, TVec3 &n, TVec3 &ray)
{
   float jxx = 0.f,
         jxy = 0.f,
         jyy = 0.f;

   for (int dy = -vanishWindow; dy <= vanishWindow; dy++)
      for (int dx = -vanishWindow; dx <= vanishWindow; dx++)
      {
         size_t i = (size_t)(y + dy)*w + (x + dx);
         float  a = (float)gx[i],
                b = (float)gy[i];

         jxx += a*a;
         jxy += a*b;
         jyy += b*b;
      }

   float trace = jxx + jyy,
         spread = sqrtf((jxx - jyy)*(jxx - jyy) + 4.f*jxy*jxy);

   if (trace <= 0.f || spread < minCoherence*trace)
      return false;

   float angle = 0.5f*atan2f(2.f*jxy, jxx - jyy), // dominant gradient direction
         tu = -sinf(angle),
         tv = cosf(angle),
         u = ((float)x + 0.5f)*(float)factor - 0.5f,
         v = ((float)y + 0.5f)*(float)factor - 0.5f;
   TVec3 along = { tu/k.fx, -tv/k.fy, 0.f };

   ray.x = (u - k.cx)/k.fx;
   ray.y = -(v - k.cy)/k.fy;
   ray.z = -1.f;
   n = vanishCross(ray, along);
   return vanishNormalize(n) && vanishNormalize(ray);
}

//--------------------------------------------------------------------------------
// Symmetric 3x3 accumulator of plane normals: xx xy xz yy yz zz
static void vanishAccumulate(float m[6], const TVec3 &n)
{
   m[0] += n.x*n.x;
   m[1] += n.x*n.y;
   m[2] += n.x*n.z;
   m[3] += n.y*n.y;
   m[4] += n.y*n.z;
   m[5] += n.z*n.z;
}

/*--------------------------------------------------------------------------------
   Least-squares direction orthogonal to every accumulated normal: the eigenvector of the
   smallest eigenvalue, by power iteration on the adjugate (its dominant eigenvalue is the
   product of the two largest ones). Starts from the prediction, keeps its sign.
  --------------------------------------------------------------------------------*/
static bool vanishSmallestEigen(const float m[6], TVec3 &dir)
{
   float a00 = m[3]*m[5] - m[4]*m[4],
         a01 = m[2]*m[4] - m[1]*m[5],
         a02 = m[1]*m[4] - m[2]*m[3],
         a11 = m[0]*m[5] - m[2]*m[2],
         a12 = m[1]*m[2] - m[0]*m[4],
         a22 = m[0]*m[3] - m[1]*m[1];
   TVec3 x = dir;

   for (int it = 0; it < vanishEigenIters; it++)
   {
      TVec3 y;

      y.x = a00*x.x + a01*x.y + a02*x.z;
      y.y = a01*x.x + a11*x.y + a12*x.z;
      y.z = a02*x.x + a12*x.y + a22*x.z;
      if (!vanishNormalize(y))
         return false;
      x = y;
   }
   if (vanishDot(x, dir) < 0.f)
   {
      x.x = -x.x;
      x.y = -x.y;
      x.z = -x.z;
   }
   dir = x;
   return true;
}

//--------------------------------------------------------------------------------
// a' M b for the symmetric accumulator (xx xy xz yy yz zz)
static float vanishQuad(const float m[6], const TVec3 &a, const TVec3 &b)
{
   return a.x*(m[0]*b.x + m[1]*b.y + m[2]*b.z) + a.y*(m[1]*b.x + m[3]*b.y + m[4]*b.z)
          + a.z*(m[2]*b.x + m[4]*b.y + m[5]*b.z);
}

/*--------------------------------------------------------------------------------
   Least-squares direction constrained to the plane orthogonal to up: the minor axis of the
   2x2 restriction of the accumulator (closed form). Keeps the sign of the start direction.
  --------------------------------------------------------------------------------*/
static void vanishSmallestInPlane(const float m[6], const TVec3 &up, TVec3 &dir)
{
   float d = vanishDot(dir, up);
   TVec3 e1 = { dir.x - d*up.x, dir.y - d*up.y, dir.z - d*up.z };

   if (!vanishNormalize(e1))
      return;

   TVec3 e2 = vanishCross(up, e1);
   float a = vanishQuad(m, e1, e1),
         b = vanishQuad(m, e1, e2),
         c = vanishQuad(m, e2, e2),
         minor = 0.5f*atan2f(2.f*b, a - c) + 1.57079633f,
         cs = cosf(minor),
         sn = sinf(minor);
   TVec3 r = { cs*e1.x + sn*e2.x, cs*e1.y + sn*e2.y, cs*e1.z + sn*e2.z };

   if (vanishDot(r, e1) < 0.f)
   {
      r.x = -r.x;
      r.y = -r.y;
      r.z = -r.z;
   }
   dir = r;
}

/*--------------------------------------------------------------------------------
   Refines one direction from the edges it explains. Each pass relabels (only free edges or
   edges already carrying this label), tightening the tolerance to tolSin; the result must
   stay within maxDeg of the prediction and gather minSupport edges.
  --------------------------------------------------------------------------------*/
static bool vanishRefine(const float *normals, LPBYTE labels, DWORD count, BYTE label, const TVec3 &predicted,
                         float tolSin, float maxDeg, int minSupport, TVec3 &dir, DWORD &support)
{
   float firstTol = sinf(maxDeg*cDegToRad);

   dir = predicted;
   support = 0u;
   for (int pass = 0; pass < vanishRefinePasses; pass++)
   {
      float m[6] = {},
            tol = pass == 0 ? (firstTol > tolSin ? firstTol : tolSin) : tolSin;
      DWORD n = 0u;

      for (DWORD e = 0; e < count; e++)
      {
         if (labels[e] != elNone && labels[e] != label)
            continue;

         TVec3 nv = { normals[3u*e], normals[3u*e + 1u], normals[3u*e + 2u] };

         if (fabsf(vanishDot(nv, dir)) < tol)
         {
            labels[e] = label;
            vanishAccumulate(m, nv);
            n++;
         }
         else if (labels[e] == label)
            labels[e] = elNone;
      }
      support = n;
      if ((int)n < minSupport || !vanishSmallestEigen(m, dir))
         break;
   }
   if ((int)support >= minSupport && vanishLineAngleDeg(dir, predicted) <= maxDeg)
      return true;
   for (DWORD e = 0; e < count; e++)
      if (labels[e] == label)
         labels[e] = elNone;
   dir = predicted;
   return false;
}

/*--------------------------------------------------------------------------------
   Joint refinement of the three directions: each pass hands every edge to the direction that
   explains it best (within the tolerance, twice as wide on the first pass), then re-solves each
   direction by least squares. A direction must keep minSupport edges and stay within maxDeg of
   its prediction; otherwise it falls back to the prediction, unflagged.
  --------------------------------------------------------------------------------*/
static BYTE vanishRefineJoint(const float *normals, DWORD count, const TVec3 *pred, float tolSin, float maxDeg,
                              int minSupport, TVec3 *dir, LPDWORD support)
{
   BYTE flags = 0u;

   for (int d = 0; d < vanishDirs; d++)
   {
      dir[d] = pred[d];
      support[d] = 0u;
   }
   for (int pass = 0; pass < vanishRefinePasses; pass++)
   {
      float m[vanishDirs][6] = {},
            tol = pass == 0 ? 2.f*tolSin : tolSin;
      DWORD n[vanishDirs] = {};

      for (DWORD e = 0; e < count; e++)
      {
         TVec3 nv = { normals[3u*e], normals[3u*e + 1u], normals[3u*e + 2u] };
         float bestDot = tol;
         int   best = -1;

         for (int d = 0; d < vanishDirs; d++)
         {
            float dot = fabsf(vanishDot(nv, dir[d]));

            if (dot < bestDot)
            {
               bestDot = dot;
               best = d;
            }
         }
         if (best < 0)
            continue;
         vanishAccumulate(m[best], nv);
         n[best]++;
      }
      // the vertical is free; the room axes stay horizontal (one line per image is then enough)
      for (int d = 0; d < vanishDirs; d++)
      {
         support[d] = n[d];
         if ((int)n[d] < minSupport)
            continue;
         if (d == 0)
            vanishSmallestEigen(m[d], dir[d]);
         else
            vanishSmallestInPlane(m[d], (int)n[0] >= minSupport ? dir[0] : pred[0], dir[d]);
      }
   }
   for (int d = 0; d < vanishDirs; d++)
      if ((int)support[d] >= minSupport && vanishLineAngleDeg(dir[d], pred[d]) <= maxDeg)
         flags |= (BYTE)(1 << d);
      else
         dir[d] = pred[d];
   return flags;
}

//--------------------------------------------------------------------------------
// Final edge labels for the caller: 1 + index of the measured direction explaining the edge, 0 none
static void vanishLabelEdges(const float *normals, DWORD count, const TVec3 *dir, BYTE flags, float tolSin,
                             LPBYTE labels)
{
   for (DWORD e = 0; e < count; e++)
   {
      TVec3 nv = { normals[3u*e], normals[3u*e + 1u], normals[3u*e + 2u] };
      float bestDot = tolSin;
      BYTE  best = 0u;

      for (int d = 0; d < vanishDirs; d++)
      {
         float dot = fabsf(vanishDot(nv, dir[d]));

         if ((flags & (1 << d)) && dot < bestDot)
         {
            bestDot = dot;
            best = (BYTE)(d + 1);
         }
      }
      labels[e] = best;
   }
}

//--------------------------------------------------------------------------------
// Heading score: soft count of the free edges one of the two room axes explains
static float vanishScore(const float *normals, LPCBYTE labels, DWORD count, const TVec3 &a, const TVec3 &b,
                         float tolSin)
{
   float score = 0.f,
         inv = 1.f/tolSin;

   for (DWORD e = 0; e < count; e++)
   {
      if (labels[e] != elNone)
         continue;

      const float *n = normals + 3u*e;
      float        da = fabsf(n[0]*a.x + n[1]*a.y + n[2]*a.z),
                   db = fabsf(n[0]*b.x + n[1]*b.y + n[2]*b.z),
                   d = (da < db ? da : db)*inv;

      if (d < 1.f)
         score += 1.f - d*d;
   }
   return score;
}

//--------------------------------------------------------------------------------
// Collects edge normals (and rays when asked): NMS survivors, evenly thinned to maxEdges, then the coherence test
static DWORD vanishCollect(const int *gx, const int *gy, int w, int h, int factor, const TIntrinsics &k,
                           const TVanishConfig &cfg, float *normals, float *rays)
{
   int   lo = vanishWindow + 1;
   DWORD peaks = 0u,
         count = 0u,
         seen = 0u;

   for (int y = lo; y < h - lo; y++)
      for (int x = lo; x < w - lo; x++)
         if (vanishPeak(gx, gy, w, x, y, cfg.minGrad))
            peaks++;

   DWORD keepEvery = peaks > (DWORD)cfg.maxEdges ? (peaks + (DWORD)cfg.maxEdges - 1u)/(DWORD)cfg.maxEdges : 1u;

   for (int y = lo; y < h - lo && count < (DWORD)cfg.maxEdges; y++)
      for (int x = lo; x < w - lo && count < (DWORD)cfg.maxEdges; x++)
      {
         if (!vanishPeak(gx, gy, w, x, y, cfg.minGrad) || seen++%keepEvery)
            continue;

         TVec3 n,
               ray;

         if (!vanishEdgeNormal(gx, gy, w, x, y, factor, k, cfg.minCoherence, n, ray))
            continue;
         normals[3u*count] = n.x;
         normals[3u*count + 1u] = n.y;
         normals[3u*count + 2u] = n.z;
         if (rays)
         {
            rays[3u*count] = ray.x;
            rays[3u*count + 1u] = ray.y;
            rays[3u*count + 2u] = ray.z;
         }
         count++;
      }
   return count;
}

//--------------------------------------------------------------------------------
// Best room-axis heading in [0, 90) for the free edges, with parabolic sub-step refinement
static float vanishSearch(const TMat4 &pose, const TVec3 &up, const float *normals, LPCBYTE labels, DWORD count,
                          const TVanishConfig &cfg)
{
   float tolSin = sinf(cfg.tolDeg*cDegToRad),
         step = cfg.stepDeg > 0.f ? cfg.stepDeg : 0.5f;
   int   steps = (int)(90.f/step + 0.5f),
         best = 0;
   float bestScore = -1.f,
         prevScore = 0.f,
         nextScore = 0.f;

   for (int i = 0; i < steps; i++)
   {
      float hDeg = (float)i*step,
            s = vanishScore(normals, labels, count, vanishAxisAt(pose, up, hDeg), vanishAxisAt(pose, up, hDeg + 90.f),
                            tolSin);

      if (s > bestScore)
      {
         bestScore = s;
         best = i;
      }
   }

   float bestDeg = (float)best*step;

   prevScore = vanishScore(normals, labels, count, vanishAxisAt(pose, up, bestDeg - step),
                           vanishAxisAt(pose, up, bestDeg - step + 90.f), tolSin);
   nextScore = vanishScore(normals, labels, count, vanishAxisAt(pose, up, bestDeg + step),
                           vanishAxisAt(pose, up, bestDeg + step + 90.f), tolSin);

   float curve = prevScore - 2.f*bestScore + nextScore;

   if (curve < 0.f)
      bestDeg += 0.5f*step*(prevScore - nextScore)/curve;
   return vanishAxisWrap(bestDeg);
}

//--------------------------------------------------------------------------------
bool vanishDetect(LPCBYTE luma, int width, int height, int stride, const TIntrinsics &intr,
                  const TMat4 &cameraToWorld, const TVanishConfig &cfg, TVanishResult &out, TVanishEdges *edges)
{
   int   longSide = width > height ? width : height,
         factor = (longSide + cfg.maxSide - 1)/cfg.maxSide;
   TVec3 worldUp = { 0.f, 1.f, 0.f },
         rawUp = vanishWorldToCam(cameraToWorld, worldUp),
         bias = { cfg.upBias[0], cfg.upBias[1], cfg.upBias[2] },
         gravityUp = vanishRotate(rawUp, bias); // the sensor gravity with the calibrated camera tilt

   out = TVanishResult();
   out.roomAxisDeg = NAN;
   out.tiltErrDeg = NAN;
   out.orthoErrDeg = NAN;
   out.dirCam[0] = gravityUp;
   out.dirCam[1] = vanishAxisAt(cameraToWorld, gravityUp, 0.f);
   out.dirCam[2] = vanishAxisAt(cameraToWorld, gravityUp, 90.f);
   if (edges)
      edges->count = 0u;
   if (factor < 1)
      factor = 1;

   int ow = width/factor,
       oh = height/factor;

   if (!luma || ow < 4*vanishWindow + 8 || oh < 4*vanishWindow + 8 || intr.fx <= 0.f || intr.fy <= 0.f)
      return false;

   TAlloc<BYTE>  shrunk((size_t)ow*oh);
   TAlloc<int>   gx((size_t)ow*oh),
                 gy((size_t)ow*oh);
   TAlloc<float> normals((size_t)cfg.maxEdges*3u);
   TAlloc<BYTE>  labels((size_t)cfg.maxEdges);

   vanishShrink(luma, stride, factor, shrunk(), ow, oh);
   vanishScharr(shrunk(), ow, oh, gx(), gy());

   float *rays = edges && edges->capacity >= (DWORD)cfg.maxEdges ? edges->ray : NULL;
   DWORD  count = vanishCollect(gx(), gy(), ow, oh, factor, intr, cfg, normals(), rays);

   out.edges = count;
   if ((int)count < cfg.minSupport)
      return false;
   memset(labels(), elNone, (size_t)count);

   // coarse vertical first (it defines the horizontal plane and keeps vertical edges out of the heading vote)
   float tolSin = sinf(cfg.tolDeg*cDegToRad);
   TVec3 up;
   DWORD upSupport = 0u;

   vanishRefine(normals(), labels(), count, elVertical, gravityUp, tolSin, cfg.maxTiltDeg, cfg.minSupport, up,
                upSupport);

   float hDeg = vanishSearch(cameraToWorld, up, normals(), labels(), count, cfg);
   TVec3 pred[vanishDirs] = { up, vanishAxisAt(cameraToWorld, up, hDeg), vanishAxisAt(cameraToWorld, up, hDeg + 90.f) };

   out.flags = vanishRefineJoint(normals(), count, pred, tolSin, cfg.maxTiltDeg, cfg.minSupport, out.dirCam,
                                 out.support);
   if (out.flags & vfVertical)
   {
      out.tiltErrDeg = vanishLineAngleDeg(out.dirCam[0], rawUp); // against the raw sensor: the check of the sensor
      out.upFix = vanishRotationBetween(rawUp, out.dirCam[0]);
   }
   if (rays)
   {
      vanishLabelEdges(normals(), count, out.dirCam, out.flags, tolSin, edges->label);
      edges->count = count;
   }

   // room-axis heading: support-weighted circular mean of the measured horizontal axes, modulo 90 degrees
   float sx = 0.f,
         sy = 0.f;

   for (int d = 1; d < vanishDirs; d++)
   {
      if (!(out.flags & (1 << d)))
         continue;

      float axis = geomHeadingDeg(cameraToWorld.RotateVector(out.dirCam[d])) - (d == 2 ? 90.f : 0.f),
            ang = vanishAxisWrap(axis)*4.f*cDegToRad;

      sx += (float)out.support[d]*cosf(ang);
      sy += (float)out.support[d]*sinf(ang);
   }
   if (sx != 0.f || sy != 0.f)
      out.roomAxisDeg = vanishAxisWrap(atan2f(sy, sx)*cRadToDeg*0.25f);

   for (int i = 0; i < vanishDirs; i++)
      for (int j = i + 1; j < vanishDirs; j++)
      {
         if (!(out.flags & (1 << i)) || !(out.flags & (1 << j)))
            continue;

         float dev = 90.f - vanishLineAngleDeg(out.dirCam[i], out.dirCam[j]);

         if (isnan(out.orthoErrDeg) || dev > out.orthoErrDeg)
            out.orthoErrDeg = dev;
      }
   return true;
}

//--------------------------------------------------------------------------------
TAxisCheck::TAxisCheck(float tolDeg) : Pvalues(), Ptol(tolDeg), PrefDeg(NAN), PfirstRefDeg(NAN), PlastDev(NAN), Pcount(0), Paligned(0),
   Pmisaligned(0), Punverified(0), Psound(), PhasRef(false)
{
}

//--------------------------------------------------------------------------------
void TAxisCheck::Reset(void)
{
   PrefDeg = NAN;
   PfirstRefDeg = NAN;
   PlastDev = NAN;
   Pcount = 0;
   Paligned = 0;
   Pmisaligned = 0;
   Punverified = 0;
   PhasRef = false;
}

/*--------------------------------------------------------------------------------
   Consensus over the most recent sound frames (a sliding window, so the slow gyroscope drift is
   followed while jumps are still caught): the value with the most neighbors within the tolerance
   (quorum required), then the mean of that group.
  --------------------------------------------------------------------------------*/
void TAxisCheck::updateReference(void)
{
   int lo = Pcount > axisCheckWindow ? Pcount - axisCheckWindow : 0,
       best = -1,
       bestCount = 0;

   for (int i = lo; i < Pcount; i++)
   {
      int n = 0;

      if (!Psound[i])
         continue;
      for (int j = lo; j < Pcount; j++)
         if (Psound[j] && fabsf(vanishAxisDiffDeg(Pvalues[j], Pvalues[i])) <= Ptol)
            n++;
      if (n > bestCount)
      {
         bestCount = n;
         best = i;
      }
   }
   if (bestCount < axisCheckQuorum)
      return; // keeps the last reference (or none yet)

   float sum = 0.f;

   for (int j = lo; j < Pcount; j++)
   {
      float d = vanishAxisDiffDeg(Pvalues[j], Pvalues[best]);

      if (Psound[j] && fabsf(d) <= Ptol)
         sum += d;
   }
   PrefDeg = vanishAxisWrap(Pvalues[best] + sum/(float)bestCount);
   if (!PhasRef)
   {
      PfirstRefDeg = PrefDeg;
      for (int j = lo; j < Pcount; j++) // the frames that waited for the quorum are judged now
         if (fabsf(vanishAxisDiffDeg(Pvalues[j], PrefDeg)) <= Ptol)
            Paligned++;
         else
            Pmisaligned++;
   }
   PhasRef = true;
}

//--------------------------------------------------------------------------------
TAxisVerdict TAxisCheck::Offer(const TVanishResult &r, float *deviationDeg)
{
   bool had = PhasRef,
        sound = isnan(r.orthoErrDeg) || r.orthoErrDeg <= cStrongOrthoDeg;

   *deviationDeg = NAN;
   PlastDev = NAN;
   if (isnan(r.roomAxisDeg))
   {
      Punverified++;
      return avNoLines;
   }
   /* full: the oldest measure leaves (the reference follows the last axisCheckWindow ones - a store that stopped
      taking new ones froze the reference while the gyroscope drifted on, and every later frame turned orange) */
   if (Pcount == axisCheckMax)
   {
      memmove(Pvalues, Pvalues + 1, sizeof(float)*(axisCheckMax - 1));
      memmove(Psound, Psound + 1, sizeof(bool)*(axisCheckMax - 1));
      Pcount--;
   }
   Pvalues[Pcount] = r.roomAxisDeg;
   Psound[Pcount] = sound;
   Pcount++;
   updateReference();
   if (!PhasRef)
      return avPending;

   float d = vanishAxisDiffDeg(r.roomAxisDeg, PrefDeg);

   *deviationDeg = d;
   PlastDev = d;
   if (!had)
      return fabsf(d) > Ptol ? avMisaligned : avReference; // counted with the waiting frames
   if (fabsf(d) > Ptol)
   {
      Pmisaligned++;
      return avMisaligned;
   }
   Paligned++;
   return avAligned;
}

//--------------------------------------------------------------------------------
TTiltBias::TTiltBias(void) : Pmean(), Pcount(0)
{
}

//--------------------------------------------------------------------------------
void TTiltBias::Reset(void)
{
   Pmean = TVec3();
   Pcount = 0;
}

//--------------------------------------------------------------------------------
// Running mean of the measured corrections; only well supported verticals within the sanity bound count
void TTiltBias::Add(const TVanishResult &r)
{
   if (!(r.flags & vfVertical) || (int)r.support[0] < tiltMinSupport || r.tiltErrDeg > cMaxBiasDeg)
      return;

   float n = (float)(Pcount < tiltWindow ? Pcount + 1 : tiltWindow);

   Pmean.x += (r.upFix.x - Pmean.x)/n;
   Pmean.y += (r.upFix.y - Pmean.y)/n;
   Pmean.z += (r.upFix.z - Pmean.z)/n;
   Pcount++;
}

//--------------------------------------------------------------------------------
void TTiltBias::Apply(TVanishConfig &cfg) const
{
   bool ready = Pcount >= tiltMinFrames;

   cfg.upBias[0] = ready ? Pmean.x : 0.f;
   cfg.upBias[1] = ready ? Pmean.y : 0.f;
   cfg.upBias[2] = ready ? Pmean.z : 0.f;
}

//--------------------------------------------------------------------------------
TVec3 TTiltBias::Bias(void) const
{
   TVec3 zero = { 0.f, 0.f, 0.f };

   return Pcount >= tiltMinFrames ? Pmean : zero;
}

//--------------------------------------------------------------------------------
