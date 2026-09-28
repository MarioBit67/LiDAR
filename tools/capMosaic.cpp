#include "capMosaic.h"
#include "capVanish.h"

#include <stdio.h>
#include <string.h>
#include <math.h>

enum {
   mosaicPxPerM      = 250, // wall orthophoto: 4 mm per pixel
   mosaicCoarseReach = 40,  // search +-16 cm on the wall
   mosaicCoarseStep  = 2,
   mosaicCoarseRow   = 4,   // pixel stride of the coarse search
   mosaicFineReach   = 3,
   mosaicPairReach   = 12,  // pair search after the leave-one-out start: +-5 cm
   mosaicMaxFrames   = 256,
   bundleTile        = 125, // 50 cm tiles on the walls: one pair of image points each
   bundleMaxObs      = 6000,
   bundleIterations  = 8,
   bundleCeiling     = -1,  // TBundleObs.g of a single-frame observation: ceiling crease point,
   bundleFloor       = -2,  // floor crease point,
   bundleCorner      = -3,  // corner edge point (wall = the plan vertex)
   mosaicCornerReach = 30,  // corner and floor searches: +-12 cm around the plan
   mosaicFloorReach  = 30,
   mosaicEqualStep   = 4,   // equalization samples: every 16 mm on the walls
   mosaicEqualMinShared = 200, // shared spots a frame needs to get its own factor
   mosaicRatioBins   = 512,
   mosaicPathMax     = 512
};

static const float cMosaicGain = 0.5f,     // per round: two overlapping frames each move half way and meet
                   cMinCoverage = 0.03f,   // a frame must see this share of a wall to be matched on it
                   cMinScore = 0.35f,      // normalized cross-correlation to trust a match
                   cMinOverlap = 4000.f,   // matched pixels
                   cPairScore = 0.25f,     // pair match weight is its score squared; below this it is left out
                   cJointAnchor = 1.f,     // weight of a ceiling anchor (a pair weighs score^2 <= 1)
                   cJointPrior = 0.02f,    // keeps frames without pairs or anchors where they are
                   cBundleScore = 0.3f,    // tile match needed to become a pair of image points
                   cBundleSigmaPixel = 1.f,      // reduced-picture pixels (~0.04 degree)
                   cBundleSigmaAngle = 0.002f,   // radians
                   cBundleSigmaDelta = 0.03f,    // radians
                   cBundleSigmaWall = 0.5f,
                   cBundleSigmaCeilingHeight = 0.3f,
                   cBundleSigmaRadius = 0.15f,
                   cBundleSigmaStation = 0.6f,   // corner station start: a plan corner a quarter of the way in
                   cBundleSigmaGyro = 1000.f,    // radians: keyframe attitudes disagree 1-4 degrees with the pictures (sync)
                   cBundleSigmaCorner = 1.5f,    // reduced-picture pixels across the corner edge
                   cBundleFloorWeight = 0.f,     // a floor crease is often a baseboard top or a furniture foot
                   cBundleCornerWeight = 0.f,
                   cBundleHuber = 2.f,     // in sigmas
                   cRatioRange = 1.2f,     // log ratio range of the vote: factors 0.3 .. 3.3
                   cCornerFill = 0.02f;    // corner-station views where the center spin also sees: a gentle feather

static bool mosaicPlanFixed = true; // the bundle holds walls and camera height on the plan (--free-plan: they move)
static char mosaicFaceNames[layoutMaxVerts + 1][8]; // --face-names: walls in plan order, then the floor ("" = default)
static FILE *mosaicPointsFile = NULL; // open during a corner fit's first measure: the RANSAC cloud of that frame
static LPCSTR mosaicOutDir = NULL;

//--------------------------------------------------------------------------------
void mosaicSetFaceNames(LPCSTR list)
{
   int n = 0;

   memset(mosaicFaceNames, 0, sizeof(mosaicFaceNames));
   for (LPCSTR p = list; *p && n <= layoutMaxVerts; n++)
   {
      int len = 0;

      while (p[len] && p[len] != ',')
         len++;
      snprintf(mosaicFaceNames[n], sizeof(mosaicFaceNames[n]), "%.*s", len < 7 ? len : 7, p);
      p += len;
      if (*p == ',')
         p++;
   }
}

//--------------------------------------------------------------------------------
// A face's name for files and prints: the user's (--face-names), else parede<K> / piso
void mosaicFaceLabel(int face, LPSTR out, size_t cap)
{
   if (face >= 0 && face <= layoutMaxVerts && mosaicFaceNames[face][0])
      snprintf(out, cap, "%s", mosaicFaceNames[face]);
   else
      snprintf(out, cap, "parede%d", face);
}

// One wall of the plan as an orthophoto canvas
struct TMosaicWall {
   bool  constU;
   float offset,
         s0,
         sdir,
         dist;
   int   cols,
         rows;
   TVec3 right;
};

//--------------------------------------------------------------------------------
static void mosaicWallGeometry(const TLayoutPlan &plan, const TVec3 &ud, const TVec3 &wd, TMosaicWall *walls)
{
   for (int i = 0; i < plan.vertexCount; i++)
   {
      const TPlanPoint &p = plan.verts[i],
                       &q = plan.verts[(i + 1)%plan.vertexCount];
      TMosaicWall      &wl = walls[i];

      wl.constU = fabsf(p.u - q.u) < fabsf(p.w - q.w);
      wl.offset = wl.constU ? 0.5f*(p.u + q.u) : 0.5f*(p.w + q.w);
      wl.dist = fabsf(wl.offset);

      float lo = wl.constU ? fminf(p.w, q.w) : fminf(p.u, q.u),
            hi = wl.constU ? fmaxf(p.w, q.w) : fmaxf(p.u, q.u),
            sgn = wl.offset >= 0.f ? 1.f : -1.f;
      TVec3 n = wl.constU ? ud : wd,
            t = wl.constU ? wd : ud;

      n.x *= sgn;
      n.z *= sgn;

      float right = -n.z*t.x + n.x*t.z; // (n x up) . t: seen from inside, left to right

      wl.sdir = right >= 0.f ? 1.f : -1.f;
      wl.s0 = wl.sdir > 0.f ? lo : hi;
      wl.cols = (int)((hi - lo)*(float)mosaicPxPerM);
      wl.rows = (int)(plan.ceilingM*(float)mosaicPxPerM);
      wl.right.x = wl.sdir*t.x;
      wl.right.y = 0.f;
      wl.right.z = wl.sdir*t.z;
   }
}

//--------------------------------------------------------------------------------
// World point of canvas pixel (c, r): spin point at the origin, y relative to the camera height
static TVec3 mosaicPoint(const TMosaicWall &wl, const TVec3 &ud, const TVec3 &wd, float ceilingM, float heightM,
                         float c, float r)
{
   float s = wl.s0 + wl.sdir*(c + 0.5f)/(float)mosaicPxPerM,
         z = ceilingM - (r + 0.5f)/(float)mosaicPxPerM - heightM,
         pu = wl.constU ? wl.offset : s,
         pw = wl.constU ? s : wl.offset;
   TVec3 p = { pu*ud.x + pw*wd.x, z, pu*ud.z + pw*wd.z };

   return p;
}

/*--------------------------------------------------------------------------------
   Samples one frame at a world point: to the frame's center, the refinement rotation, the
   vanishing-point rotation, the pinhole, a bilinear read. Returns the weight (0: not seen), falling
   off steeply toward the picture border so each pixel mostly comes from its most frontal view.
  --------------------------------------------------------------------------------*/
static float mosaicSample(const TMosaicFrame &f, TVec3 p, float bgr[3])
{
   TVec3 back = { -f.delta.x, -f.delta.y, -f.delta.z };

   p.x -= f.camX;
   p.z -= f.camZ;
   p.y -= f.camY;
   p = vanishRotate(p, back);

   float ka = p.x*f.worldA.x + p.z*f.worldA.z,
         kc = p.x*f.worldC.x + p.z*f.worldC.z,
         rx = f.camA.x*ka + f.camUp.x*p.y + f.camC.x*kc,
         ry = f.camA.y*ka + f.camUp.y*p.y + f.camC.y*kc,
         rz = f.camA.z*ka + f.camUp.z*p.y + f.camC.z*kc;

   if (rz > -1e-3f)
      return 0.f;

   float su = f.k.cx + f.k.fx*rx/-rz,
         sv = f.k.cy - f.k.fy*ry/-rz;
   int   iu = (int)floorf(su),
         iv = (int)floorf(sv);

   if (iu < 0 || iv < 0 || iu + 1 >= f.w || iv + 1 >= f.h)
      return 0.f;

   float du = su - (float)iu,
         dv = sv - (float)iv,
         ex = (su - f.k.cx)/f.k.cx,
         ey = (sv - f.k.cy)/f.k.cy,
         wt = 1.f - 0.5f*(ex*ex + ey*ey);

   if (wt <= 0.f)
      return 0.f;
   for (int ch = 0; ch < 3; ch++)
   {
      LPCBYTE q = f.bgr() + ((size_t)iv*f.w + iu)*3u + ch;
      float   t0 = (float)q[0]*(1.f - du) + (float)q[3]*du,
              t1 = (float)q[(size_t)f.w*3u]*(1.f - du) + (float)q[(size_t)f.w*3u + 3u]*du;

      bgr[ch] = t0*(1.f - dv) + t1*dv;
   }
   wt *= wt;
   return wt*wt;
}

/*--------------------------------------------------------------------------------
   Object boundaries of a luma canvas, split by direction (3x3 box, Sobel): |gx| = vertical edges
   (jambs, cabinet sides, frames) fix the shift along the wall, |gy| = horizontal edges (tops,
   baseboards, heads) fix it vertically. Mask: the whole 5x5 neighborhood valid.
  --------------------------------------------------------------------------------*/
static void mosaicEdges(const float *luma, LPCBYTE valid, int w, int h, float *edgesX, float *edgesY, LPBYTE mask)
{
   memset(edgesX, 0, sizeof(float)*(size_t)w*h);
   memset(edgesY, 0, sizeof(float)*(size_t)w*h);
   memset(mask, 0, (size_t)w*h);
   for (int y = 2; y + 2 < h; y++)
      for (int x = 2; x + 2 < w; x++)
      {
         bool ok = true;

         for (int dy = -2; dy <= 2 && ok; dy++)
            for (int dx = -2; dx <= 2 && ok; dx++)
               ok = valid[(size_t)(y + dy)*w + x + dx] != 0u;
         if (!ok)
            continue;

         float b[3][3];

         for (int j = -1; j <= 1; j++)
            for (int i = -1; i <= 1; i++)
            {
               float s = 0.f;

               for (int dy = -1; dy <= 1; dy++)
                  for (int dx = -1; dx <= 1; dx++)
                     s += luma[(size_t)(y + j + dy)*w + x + i + dx];
               b[j + 1][i + 1] = s/9.f;
            }

         float gx = (b[0][2] + 2.f*b[1][2] + b[2][2]) - (b[0][0] + 2.f*b[1][0] + b[2][0]),
               gy = (b[2][0] + 2.f*b[2][1] + b[2][2]) - (b[0][0] + 2.f*b[0][1] + b[0][2]);

         edgesX[(size_t)y*w + x] = fabsf(gx);
         edgesY[(size_t)y*w + x] = fabsf(gy);
         mask[(size_t)y*w + x] = 1u;
      }
}

//--------------------------------------------------------------------------------
// Normalized cross-correlation of a at (x, y) with b at (x + dx, y + dy), over pixels valid in both
static float mosaicNCC(const float *a, LPCBYTE ma, const float *b, LPCBYTE mb, int w, int h, int dx, int dy, int stride,
                       const int *box, float *count)
{
   float sa = 0.f,
         sb = 0.f,
         saa = 0.f,
         sbb = 0.f,
         sab = 0.f,
         n = 0.f;

   for (int y = box[1]; y < box[3]; y += stride)
   {
      int by = y + dy;

      if (by < 0 || by >= h)
         continue;
      for (int x = 0; x < w; x += stride)
      {
         int    bx = x + dx;
         size_t ia = (size_t)y*w + x,
                ib = (size_t)by*w + bx;

         if (bx < 0 || bx >= w || !ma[ia] || !mb[ib])
            continue;

         float va = a[ia],
               vb = b[ib];

         sa += va;
         sb += vb;
         saa += va*va;
         sbb += vb*vb;
         sab += va*vb;
         n += 1.f;
      }
   }
   *count = n*(float)(stride*stride);
   if (n < 50.f)
      return -1.f;

   float cov = sab - sa*sb/n,
         va = saa - sa*sa/n,
         vb = sbb - sb*sb/n;

   return va > 0.f && vb > 0.f ? cov/sqrtf(va*vb) : -1.f;
}

/*--------------------------------------------------------------------------------
   Shift that best lays the frame's edges onto the mosaic's: coarse search (+-16 cm, 2 px steps,
   sparse pixels), fine search (+-3 px, every pixel), parabolic subpixel on each axis.
  --------------------------------------------------------------------------------*/
static float mosaicAlign(const float *a, LPCBYTE ma, const float *b, LPCBYTE mb, int w, int h, int reach, const int *box,
                         float *dx, float *dy, float *overlap)
{
   int   bx = 0,
         by = 0;
   float best = -2.f,
         cnt = 0.f;

   for (int sy = -reach; sy <= reach; sy += mosaicCoarseStep)
      for (int sx = -reach; sx <= reach; sx += mosaicCoarseStep)
      {
         float s = mosaicNCC(a, ma, b, mb, w, h, sx, sy, mosaicCoarseRow, box, &cnt);

         if (s > best)
         {
            best = s;
            bx = sx;
            by = sy;
         }
      }

   float grid[2*mosaicFineReach + 1][2*mosaicFineReach + 1];
   int   fx = 0,
         fy = 0;

   best = -2.f;
   for (int j = -mosaicFineReach; j <= mosaicFineReach; j++)
      for (int i = -mosaicFineReach; i <= mosaicFineReach; i++)
      {
         float s = mosaicNCC(a, ma, b, mb, w, h, bx + i, by + j, 1, box, &cnt);

         grid[j + mosaicFineReach][i + mosaicFineReach] = s;
         if (s > best)
         {
            best = s;
            fx = i;
            fy = j;
            *overlap = cnt;
         }
      }

   float subX = 0.f,
         subY = 0.f;
   int   gx = fx + mosaicFineReach,
         gy = fy + mosaicFineReach;

   if (gx > 0 && gx < 2*mosaicFineReach)
   {
      float l = grid[gy][gx - 1],
            c = grid[gy][gx],
            r = grid[gy][gx + 1],
            den = l - 2.f*c + r;

      subX = den < 0.f ? 0.5f*(l - r)/den : 0.f;
   }
   if (gy > 0 && gy < 2*mosaicFineReach)
   {
      float u = grid[gy - 1][gx],
            c = grid[gy][gx],
            d = grid[gy + 1][gx],
            den = u - 2.f*c + d;

      subY = den < 0.f ? 0.5f*(u - d)/den : 0.f;
   }
   *dx = (float)(bx + fx) + subX;
   *dy = (float)(by + fy) + subY;
   return best;
}

//--------------------------------------------------------------------------------
static void mosaicWriteBMP(LPCSTR path, const float *acc, int w, int h)
{
   FILE *bmp = fopen(path, "wb");

   if (!bmp)
      return;

   int          rowBytes = (w*3 + 3) & ~3;
   DWORD        imageBytes = (DWORD)rowBytes*(DWORD)h,
                header[13] = { 54u + imageBytes, 0u, 54u, 40u, (DWORD)w, (DWORD)h, 0x00180001u, 0u, imageBytes, 2835u,
                               2835u, 0u, 0u };
   BYTE         magic[2] = { 'B', 'M' };
   TAlloc<BYTE> line((size_t)rowBytes);

   fwrite(magic, 1u, 2u, bmp);
   fwrite(header, 4u, 13u, bmp);
   for (int r = h - 1; r >= 0; r--) // bottom-up
   {
      memset(line(), 0, (size_t)rowBytes);
      for (int c = 0; c < w; c++)
      {
         const float *o = acc + ((size_t)r*w + c)*4u;

         for (int ch = 0; ch < 3 && o[3] > 0.f; ch++)
            line[(size_t)c*3u + ch] = (BYTE)fminf(255.f, o[ch]/o[3] + 0.5f); // gains may push past white
      }
      fwrite(line(), 1u, (size_t)rowBytes, bmp);
   }
   fclose(bmp);
}

/*--------------------------------------------------------------------------------
   One refinement round on one wall: each frame's luma picture of the wall against the mosaic of the
   others (leave-one-out, so a frame never pulls toward itself), edges matched with subpixel
   precision. The shift (ds along the wall, dz up) on a wall at distance D is undone by a rotation of
   the frame: beta = -ds/D about the vertical, alpha = dz/D about the wall's right direction.
  --------------------------------------------------------------------------------*/
static void mosaicRefineWall(TMosaicFrame *frames, int count, const TMosaicWall &wl, const TVec3 &ud, const TVec3 &wd,
                             const TLayoutPlan &plan, TVec3 *update, float *updateW, float *sumShift, int *shifts)
{
   size_t        pixels = (size_t)wl.cols*wl.rows;
   TAlloc<float> luma(pixels*(size_t)count),
                 weight(pixels*(size_t)count),
                 sumL(pixels),
                 sumW(pixels),
                 loo(pixels),
                 edgesFX(pixels),
                 edgesFY(pixels),
                 edgesMX(pixels),
                 edgesMY(pixels);
   TAlloc<BYTE>  validF(pixels),
                 validM(pixels),
                 maskF(pixels),
                 maskM(pixels),
                 seen((size_t)count);

   memset(sumL(), 0, sizeof(float)*pixels);
   memset(sumW(), 0, sizeof(float)*pixels);
   for (int f = 0; f < count; f++)
   {
      float *l = luma() + pixels*(size_t)f,
            *wt = weight() + pixels*(size_t)f;
      size_t covered = 0u;

      for (int r = 0; r < wl.rows; r++)
         for (int c = 0; c < wl.cols; c++)
         {
            size_t i = (size_t)r*wl.cols + c;
            float  bgr[3],
                   s = mosaicSample(frames[f], mosaicPoint(wl, ud, wd, plan.ceilingM, plan.cameraHeightM, (float)c, (float)r), bgr);

            wt[i] = s;
            l[i] = s > 0.f ? 0.114f*bgr[0] + 0.587f*bgr[1] + 0.299f*bgr[2] : 0.f;
            if (s > 0.f)
            {
               sumL[i] += s*l[i];
               sumW[i] += s;
               covered++;
            }
         }
      seen[f] = (float)covered >= cMinCoverage*(float)pixels ? 1u : 0u;
   }
   for (int f = 0; f < count; f++)
   {
      if (!seen[f])
         continue;

      const float *l = luma() + pixels*(size_t)f,
                  *wt = weight() + pixels*(size_t)f;

      for (size_t i = 0; i < pixels; i++)
      {
         float rest = sumW[i] - wt[i];

         validF[i] = wt[i] > 0.f ? 1u : 0u;
         validM[i] = rest > 1e-4f ? 1u : 0u;
         loo[i] = validM[i] ? (sumL[i] - wt[i]*l[i])/rest : 0.f;
      }
      mosaicEdges(l, validF(), wl.cols, wl.rows, edgesFX(), edgesFY(), maskF());
      mosaicEdges(loo(), validM(), wl.cols, wl.rows, edgesMX(), edgesMY(), maskM());

      int   full[4] = { 0, 0, wl.cols, wl.rows };
      // the vertical boundaries give the shift along the wall, the horizontal ones the vertical shift
      float dx = 0.f,
            dy = 0.f,
            unusedX = 0.f,
            unusedY = 0.f,
            overlap = 0.f,
            overlapY = 0.f,
            scoreX = mosaicAlign(edgesFX(), maskF(), edgesMX(), maskM(), wl.cols, wl.rows, mosaicCoarseReach, full,
                                 &dx, &unusedY, &overlap),
            scoreY = mosaicAlign(edgesFY(), maskF(), edgesMY(), maskM(), wl.cols, wl.rows, mosaicCoarseReach, full,
                                 &unusedX, &dy, &overlapY),
            score = fminf(scoreX, scoreY);

      if (fabsf(dx) >= (float)(mosaicCoarseReach - 1) || fabsf(dy) >= (float)(mosaicCoarseReach - 1))
         continue; // at the edge of the search: no trustworthy optimum inside

      if (score < cMinScore || overlap < cMinOverlap)
         continue;

      float ds = dx/(float)mosaicPxPerM,  // frame content must move this far right...
            dz = -dy/(float)mosaicPxPerM, // ...and this far up
            beta = -ds/wl.dist,
            alpha = dz/wl.dist,
            wgt = overlap*score;

      update[f].x += wgt*alpha*wl.right.x;
      update[f].y += wgt*beta;
      update[f].z += wgt*alpha*wl.right.z;
      updateW[f] += wgt;
      *sumShift += sqrtf(dx*dx + dy*dy);
      (*shifts)++;
   }
}

//--------------------------------------------------------------------------------
// Adds one weighted equation sum_k c_k . delta_{frame_k} = rhs (up to two frames) to the normal equations
static void mosaicEquation(float *ata, float *atb, int n, int f0, const TVec3 &c0, int f1, const TVec3 &c1, float rhs,
                           float w)
{
   int   idx[6];
   float cof[6];
   int   m = 0;

   for (int k = 0; k < 3 && f0 >= 0; k++)
   {
      idx[m] = 3*f0 + k;
      cof[m++] = k == 0 ? c0.x : (k == 1 ? c0.y : c0.z);
   }
   for (int k = 0; k < 3 && f1 >= 0; k++)
   {
      idx[m] = 3*f1 + k;
      cof[m++] = k == 0 ? c1.x : (k == 1 ? c1.y : c1.z);
   }
   for (int i = 0; i < m; i++)
   {
      atb[idx[i]] += w*cof[i]*rhs;
      for (int j = 0; j < m; j++)
         ata[(size_t)idx[i]*n + idx[j]] += w*cof[i]*cof[j];
   }
}

//--------------------------------------------------------------------------------
// Gaussian elimination with partial pivoting; a (n x n) and b are destroyed, x receives the solution
static bool mosaicSolve(float *a, float *b, float *x, int n)
{
   for (int c = 0; c < n; c++)
   {
      int p = c;

      for (int r = c + 1; r < n; r++)
         if (fabsf(a[(size_t)r*n + c]) > fabsf(a[(size_t)p*n + c]))
            p = r;
      if (fabsf(a[(size_t)p*n + c]) < 1e-12f)
         return false;
      for (int k = 0; k < n && p != c; k++)
      {
         float t = a[(size_t)c*n + k];

         a[(size_t)c*n + k] = a[(size_t)p*n + k];
         a[(size_t)p*n + k] = t;
      }

      float tb = b[c];

      b[c] = b[p];
      b[p] = tb;
      for (int r = c + 1; r < n; r++)
      {
         float f = a[(size_t)r*n + c]/a[(size_t)c*n + c];

         for (int k = c; k < n; k++)
            a[(size_t)r*n + k] -= f*a[(size_t)c*n + k];
         b[r] -= f*b[c];
      }
   }
   for (int r = n - 1; r >= 0; r--)
   {
      float s = b[r];

      for (int k = r + 1; k < n; k++)
         s -= a[(size_t)r*n + k]*x[k];
      x[r] = s/a[(size_t)r*n + r];
   }
   return true;
}

/*--------------------------------------------------------------------------------
   Ceiling anchor of one frame's picture of a wall: the crease must sit on the wall's top row
   (the plan put it at the ceiling height). Returns the row of the strongest horizontal boundary
   within the top 24 cm (subpixel), or -1 when the frame does not show the top of that wall.
  --------------------------------------------------------------------------------*/
static float mosaicCreaseRow(const float *edgesY, LPCBYTE mask, int w, int h)
{
   const int band = mosaicPxPerM*24/100;
   float     rowSum[mosaicPxPerM];
   int       best = -1;
   float     bestV = 0.f,
             mean = 0.f;

   for (int r = 0; r < band && r < h; r++)
   {
      float s = 0.f;
      int   n = 0;

      for (int c = 0; c < w; c++)
         if (mask[(size_t)r*w + c])
         {
            s += edgesY[(size_t)r*w + c];
            n++;
         }
      rowSum[r] = n >= w/4 ? s/(float)n : 0.f;
      mean += rowSum[r];
      if (rowSum[r] > bestV)
      {
         bestV = rowSum[r];
         best = r;
      }
   }
   mean /= (float)band;
   if (best < 0 || bestV < 3.f*mean)
      return -1.f;

   /* the crease is the TOP of the crown molding (where the plan put the ceiling): the highest strong boundary,
      not the strongest one (the molding's lower edge against the wall usually contrasts more) */
   for (int r = 1; r < best; r++)
      if (rowSum[r] >= 0.5f*bestV && rowSum[r] >= rowSum[r - 1] && rowSum[r] >= rowSum[r + 1])
      {
         best = r;
         break;
      }
   if (best < 1 || best + 1 >= band)
      return -1.f;

   float l = rowSum[best - 1],
         c = rowSum[best],
         u = rowSum[best + 1],
         den = l - 2.f*c + u;

   return (float)best + (den < 0.f ? 0.5f*(l - u)/den : 0.f);
}

/*--------------------------------------------------------------------------------
   Joint adjustment of every frame's rotation, anchored on the ceiling. Per wall, every pair of
   frames that overlap gives their relative shift (vertical boundaries -> along the wall, horizontal
   ones -> up); every frame showing the top of a wall gives the absolute shift that puts its crease on
   the wall's top row; a weak prior keeps unconnected frames still. On a wall at distance D with
   right direction t, a rotation delta moves the frame's picture by -D delta.y along it and by
   D (t . delta) up. All equations are solved together (normal equations) and applied in full.
  --------------------------------------------------------------------------------*/
static void mosaicJoint(TMosaicFrame *frames, int count, const TMosaicWall *walls, int wallCount, const TVec3 &ud,
                        const TVec3 &wd, const TLayoutPlan &plan, int round)
{
   int           n = 3*count,
                 pairs = 0,
                 anchors = 0;
   TAlloc<float> ata((size_t)n*n),
                 atb((size_t)n),
                 x((size_t)n);
   float         shiftSum = 0.f,
                 creaseSum = 0.f;

   memset(ata(), 0, sizeof(float)*(size_t)n*n);
   memset(atb(), 0, sizeof(float)*(size_t)n);
   for (int i = 0; i < n; i++)
      ata[(size_t)i*n + i] = cJointPrior;
   for (int wi = 0; wi < wallCount; wi++)
   {
      const TMosaicWall &wl = walls[wi];
      size_t             pixels = (size_t)wl.cols*wl.rows;
      int                seen[mosaicMaxFrames],
                         m = 0;

      if (wl.cols <= 0)
         continue;

      // pictures of the frames that see this wall: edges by direction, masks
      TAlloc<float> ex(pixels*(size_t)count),
                    ey(pixels*(size_t)count),
                    luma(pixels);
      TAlloc<BYTE>  mask(pixels*(size_t)count),
                    valid(pixels);

      for (int f = 0; f < count && m < mosaicMaxFrames; f++)
      {
         size_t covered = 0u;

         for (int r = 0; r < wl.rows; r++)
            for (int c = 0; c < wl.cols; c++)
            {
               size_t i = (size_t)r*wl.cols + c;
               float  bgr[3],
                      s = mosaicSample(frames[f], mosaicPoint(wl, ud, wd, plan.ceilingM, plan.cameraHeightM, (float)c, (float)r), bgr);

               valid[i] = s > 0.f ? 1u : 0u;
               luma[i] = s > 0.f ? 0.114f*bgr[0] + 0.587f*bgr[1] + 0.299f*bgr[2] : 0.f;
               covered += valid[i];
            }
         if ((float)covered < cMinCoverage*(float)pixels)
            continue;
         mosaicEdges(luma(), valid(), wl.cols, wl.rows, ex() + pixels*(size_t)m, ey() + pixels*(size_t)m,
                     mask() + pixels*(size_t)m);
         seen[m++] = f;
      }

      int   full[4] = { 0, 0, wl.cols, wl.rows };
      TVec3 alongC = { 0.f, -wl.dist, 0.f },               // along-wall shift per unit delta
            upC = { wl.dist*wl.right.x, 0.f, wl.dist*wl.right.z }, // vertical shift per unit delta
            zero = { 0.f, 0.f, 0.f },
            negAlong = { 0.f, wl.dist, 0.f },
            negUp = { -upC.x, 0.f, -upC.z };

      for (int i = 0; i < m; i++)
      {
         const float *eyI = ey() + pixels*(size_t)i;
         LPCBYTE      mkI = mask() + pixels*(size_t)i;
         float        row = mosaicCreaseRow(eyI, mkI, wl.cols, wl.rows);

         if (row >= 0.f) // ceiling anchor: move up by row pixels
         {
            mosaicEquation(ata(), atb(), n, seen[i], upC, -1, zero, (row + 0.5f)/(float)mosaicPxPerM, cJointAnchor);
            creaseSum += row;
            anchors++;
         }
         for (int j = i + 1; j < m; j++)
         {
            float dx = 0.f,
                  dy = 0.f,
                  ux = 0.f,
                  uy = 0.f,
                  ovX = 0.f,
                  ovY = 0.f,
                  sx = mosaicAlign(ex() + pixels*(size_t)j, mask() + pixels*(size_t)j, ex() + pixels*(size_t)i, mkI,
                                   wl.cols, wl.rows, mosaicPairReach, full, &dx, &uy, &ovX),
                  sy = mosaicAlign(ey() + pixels*(size_t)j, mask() + pixels*(size_t)j, eyI, mkI, wl.cols, wl.rows,
                                   mosaicPairReach, full, &ux, &dy, &ovY),
                  q = fminf(sx, sy);

            if (q < cPairScore || fminf(ovX, ovY) < cMinOverlap || fabsf(dx) >= (float)(mosaicPairReach - 1)
                || fabsf(dy) >= (float)(mosaicPairReach - 1))
               continue;

            // frame j must move by (dx right, -dy up) to sit on frame i
            float w = q*q;

            mosaicEquation(ata(), atb(), n, seen[j], alongC, seen[i], negAlong, dx/(float)mosaicPxPerM, w);
            mosaicEquation(ata(), atb(), n, seen[j], upC, seen[i], negUp, -dy/(float)mosaicPxPerM, w);
            shiftSum += sqrtf(dx*dx + dy*dy);
            pairs++;
         }
      }
   }
   if (!mosaicSolve(ata(), atb(), x(), n))
      return;
   for (int f = 0; f < count; f++)
   {
      frames[f].delta.x += x[3*f];
      frames[f].delta.y += x[3*f + 1];
      frames[f].delta.z += x[3*f + 2];
   }
   printf("  joint round %d: %d pairs (mean shift %.2f px), %d ceiling anchors (mean crease row %.2f)\n", round, pairs,
          pairs ? shiftSum/(float)pairs : 0.f, anchors, anchors ? creaseSum/(float)anchors : 0.f);
}

/* ==================================================================================================
   Bundle adjustment of the center spin: rotations AND geometry together. Unknowns: every frame's small
   rotation, every wall's offset, e = ceiling above the camera, r = spin radius. Observations are image
   points measured once per round (they do not move while solving): pairs of points of the same detail
   in two frames (from tiles of their wall pictures aligned on edges) and points of the ceiling crease.
   Residuals live on the walls, in meters: the two rays of a pair, each from its frame's REAL center
   (which moves with r), must pierce the wall at the same spot; a crease ray must pierce it at the
   ceiling height. Weak priors hold the current estimate. Gauss-Newton, damped, Huber-robust.
   ================================================================================================== */

// One image observation (reduced-picture pixels)
struct TBundleObs {
   int   f,     // frame
         g,     // second frame of a pair, -1: a ceiling crease point of f
         wall;
   float uf,
         vf,
         ug,
         vg,
         w;     // match quality
};

//--------------------------------------------------------------------------------
// Camera direction -> world with the frame's vanishing-point rotation and a refinement delta
static TVec3 bundleToWorld(const TMosaicFrame &f, const TVec3 &delta, const TVec3 &v)
{
   float a = f.camA.x*v.x + f.camA.y*v.y + f.camA.z*v.z,
         c = f.camC.x*v.x + f.camC.y*v.y + f.camC.z*v.z;
   TVec3 room = { f.worldA.x*a + f.worldC.x*c, f.camUp.x*v.x + f.camUp.y*v.y + f.camUp.z*v.z,
                  f.worldA.z*a + f.worldC.z*c };

   return vanishRotate(room, delta);
}

//--------------------------------------------------------------------------------
// Camera center: the station (sx, sz) plus the spin circle, r out along the horizontal forward
static TVec3 bundleCenter(const TMosaicFrame &f, const TVec3 &delta, float r, float sx, float sz)
{
   TVec3 fwd = { 0.f, 0.f, -1.f },
         w = bundleToWorld(f, delta, fwd);
   float len = sqrtf(w.x*w.x + w.z*w.z);
   TVec3 c = { sx + (len > 1e-6f ? r*w.x/len : 0.f), 0.f, sz + (len > 1e-6f ? r*w.z/len : 0.f) };

   return c;
}

//--------------------------------------------------------------------------------
// Where the ray of image point (u, v) of frame f pierces the plane of a wall (n . P = offset)
static bool bundleHit(const TMosaicFrame &f, const TVec3 &delta, float r, float sx, float sz, float u, float v,
                      const TVec3 &n, float offset, TVec3 &hit)
{
   TVec3 dc = { (u - f.k.cx)/f.k.fx, -(v - f.k.cy)/f.k.fy, -1.f },
         d = bundleToWorld(f, delta, dc),
         c = bundleCenter(f, delta, r, sx, sz);
   float den = n.x*d.x + n.y*d.y + n.z*d.z;

   if (fabsf(den) < 1e-4f)
      return false;

   float t = (offset - (n.x*c.x + n.y*c.y + n.z*c.z))/den;

   if (t <= 0.f)
      return false;
   hit.x = c.x + t*d.x;
   hit.y = c.y + t*d.y;
   hit.z = c.z + t*d.z;
   return true;
}

//--------------------------------------------------------------------------------
// World point -> image point of frame f for a trial delta and spin radius (the model being solved)
static bool bundleProjectAt(const TMosaicFrame &f, const TVec3 &delta, float r, float sx, float sz, TVec3 p, float &u,
                            float &v)
{
   TVec3 c = bundleCenter(f, delta, r, sx, sz),
         back = { -delta.x, -delta.y, -delta.z };

   p.x -= c.x;
   p.z -= c.z;
   p = vanishRotate(p, back);

   float ka = p.x*f.worldA.x + p.z*f.worldA.z,
         kc = p.x*f.worldC.x + p.z*f.worldC.z,
         rx = f.camA.x*ka + f.camUp.x*p.y + f.camC.x*kc,
         ry = f.camA.y*ka + f.camUp.y*p.y + f.camC.y*kc,
         rz = f.camA.z*ka + f.camUp.z*p.y + f.camC.z*kc;

   if (rz > -1e-3f)
      return false;
   u = f.k.cx + f.k.fx*rx/-rz;
   v = f.k.cy - f.k.fy*ry/-rz;
   return true;
}

//--------------------------------------------------------------------------------
// World point -> image point of frame f with its current delta and center (the sampling model)
static bool bundleProject(const TMosaicFrame &f, TVec3 p, float &u, float &v)
{
   TVec3 back = { -f.delta.x, -f.delta.y, -f.delta.z };

   p.x -= f.camX;
   p.z -= f.camZ;
   p.y -= f.camY;
   p = vanishRotate(p, back);

   float ka = p.x*f.worldA.x + p.z*f.worldA.z,
         kc = p.x*f.worldC.x + p.z*f.worldC.z,
         rx = f.camA.x*ka + f.camUp.x*p.y + f.camC.x*kc,
         ry = f.camA.y*ka + f.camUp.y*p.y + f.camC.y*kc,
         rz = f.camA.z*ka + f.camUp.z*p.y + f.camC.z*kc;

   if (rz > -1e-3f)
      return false;
   u = f.k.cx + f.k.fx*rx/-rz;
   v = f.k.cy - f.k.fy*ry/-rz;
   return true;
}

//--------------------------------------------------------------------------------
// World point of a fractional canvas pixel of a wall
static TVec3 bundleCanvasPoint(const TMosaicWall &wl, const TVec3 &ud, const TVec3 &wd, float e, float c, float r)
{
   float s = wl.s0 + wl.sdir*(c + 0.5f)/(float)mosaicPxPerM,
         z = e - (r + 0.5f)/(float)mosaicPxPerM,
         pu = wl.constU ? wl.offset : s,
         pw = wl.constU ? s : wl.offset;
   TVec3 p = { pu*ud.x + pw*wd.x, z, pu*ud.z + pw*wd.z };

   return p;
}

//--------------------------------------------------------------------------------
// Highest strong horizontal boundary in the top band of columns [c0, c1) (the crease), -1 if none
static float bundleCreaseRow(const float *edgesY, LPCBYTE mask, int w, int h, int c0, int c1)
{
   const int band = mosaicPxPerM*24/100;
   float     rowSum[mosaicPxPerM];
   float     bestV = 0.f,
             mean = 0.f;
   int       best = -1;

   for (int r = 0; r < band && r < h; r++)
   {
      float s = 0.f;
      int   n = 0;

      for (int c = c0; c < c1; c++)
         if (mask[(size_t)r*w + c])
         {
            s += edgesY[(size_t)r*w + c];
            n++;
         }
      rowSum[r] = n >= (c1 - c0)/2 ? s/(float)n : 0.f;
      mean += rowSum[r];
      if (rowSum[r] > bestV)
      {
         bestV = rowSum[r];
         best = r;
      }
   }
   mean /= (float)band;
   if (best < 0 || bestV < 3.f*mean)
      return -1.f;
   for (int r = 1; r < best; r++)
      if (rowSum[r] >= 0.5f*bestV && rowSum[r] >= rowSum[r - 1] && rowSum[r] >= rowSum[r + 1])
      {
         best = r;
         break;
      }
   if (best < 1 || best + 1 >= band)
      return -1.f;

   float l = rowSum[best - 1],
         c = rowSum[best],
         u = rowSum[best + 1],
         den = l - 2.f*c + u;

   return (float)best + (den < 0.f ? 0.5f*(l - u)/den : 0.f);
}

//--------------------------------------------------------------------------------
// World direction -> camera direction of frame f for a trial delta (the inverse of bundleToWorld)
static TVec3 bundleToCamera(const TMosaicFrame &f, const TVec3 &delta, TVec3 w)
{
   TVec3 back = { -delta.x, -delta.y, -delta.z };

   w = vanishRotate(w, back);

   float ka = w.x*f.worldA.x + w.z*f.worldA.z,
         kc = w.x*f.worldC.x + w.z*f.worldC.z;
   TVec3 c = { f.camA.x*ka + f.camUp.x*w.y + f.camC.x*kc, f.camA.y*ka + f.camUp.y*w.y + f.camC.y*kc,
               f.camA.z*ka + f.camUp.z*w.y + f.camC.z*kc };

   return c;
}

/*--------------------------------------------------------------------------------
   Luma of frame f over canvas columns [c0, c1) and rows [r0, r1) of a wall plane - extended past the
   canvas, so a corner or the floor a little beyond the current plan is still found. Returns the
   pixels seen.
  --------------------------------------------------------------------------------*/
static int bundleStrip(const TMosaicFrame &f, const TMosaicWall &wl, const TVec3 &ud, const TVec3 &wd, float e, int c0,
                       int c1, int r0, int r1, float *luma, LPBYTE valid)
{
   int w = c1 - c0,
       seen = 0;

   for (int r = r0; r < r1; r++)
      for (int c = c0; c < c1; c++)
      {
         size_t i = (size_t)(r - r0)*w + (c - c0);
         float  bgr[3],
                s = mosaicSample(f, bundleCanvasPoint(wl, ud, wd, e, (float)c, (float)r), bgr);

         valid[i] = s > 0.f ? 1u : 0u;
         luma[i] = s > 0.f ? 0.114f*bgr[0] + 0.587f*bgr[1] + 0.299f*bgr[2] : 0.f;
         seen += valid[i];
      }
   return seen;
}

//--------------------------------------------------------------------------------
// Subpixel peak of a profile at index b (parabola through its neighbors)
static float bundlePeak(const float *p, int b)
{
   float den = p[b - 1] - 2.f*p[b] + p[b + 1];

   return (float)b + (den < 0.f ? 0.5f*(p[b - 1] - p[b + 1])/den : 0.f);
}

/*--------------------------------------------------------------------------------
   The corner edge near canvas column cv of a wall in frame f: the column of strongest lateral
   contrast (two walls take the light differently) over the upper band, from below the molding to
   40% down - above the furniture. col, row: canvas position of the edge; false without a clear peak.
  --------------------------------------------------------------------------------*/
static bool bundleCornerColumn(const TMosaicFrame &f, const TMosaicWall &wl, const TVec3 &ud, const TVec3 &wd, float e,
                               int cv, float &col, float &row)
{
   const int     c0 = cv - mosaicCornerReach,
                 c1 = cv + mosaicCornerReach + 1,
                 r0 = wl.rows/20,
                 r1 = wl.rows*2/5,
                 w = c1 - c0,
                 h = r1 - r0;
   TAlloc<float> luma((size_t)w*h),
                 profile((size_t)w);
   TAlloc<BYTE>  valid((size_t)w*h);

   if (h <= 0 || (float)bundleStrip(f, wl, ud, wd, e, c0, c1, r0, r1, luma(), valid()) < 0.7f*(float)(w*h))
      return false;

   float best = 0.f,
         mean = 0.f;
   int   at = -1;

   for (int c = 1; c + 1 < w; c++)
   {
      float s = 0.f;
      int   n = 0;

      for (int r = 0; r < h; r++)
      {
         size_t i = (size_t)r*w + c;

         if (valid[i - 1u] && valid[i + 1u])
         {
            s += fabsf(luma[i + 1u] - luma[i - 1u]);
            n++;
         }
      }
      profile[c] = n >= h/2 ? s/(float)n : 0.f;
      mean += profile[c];
      if (profile[c] > best)
      {
         best = profile[c];
         at = c;
      }
   }
   mean /= (float)(w - 2);
   if (at < 2 || at + 2 >= w || best < 3.f*mean)
      return false;
   col = (float)c0 + bundlePeak(profile(), at);
   row = 0.5f*(float)(r0 + r1);
   return true;
}

/*--------------------------------------------------------------------------------
   The floor crease of a wall in frame f over columns [c0, c1): the strongest horizontal boundary
   within reach of the plan's floor row. false when furniture hides it (no clear peak).
  --------------------------------------------------------------------------------*/
static bool bundleFloorRow(const TMosaicFrame &f, const TMosaicWall &wl, const TVec3 &ud, const TVec3 &wd, float e,
                           int c0, int c1, float &row)
{
   const int     r0 = wl.rows - mosaicFloorReach,
                 r1 = wl.rows + mosaicFloorReach + 1,
                 w = c1 - c0,
                 h = r1 - r0;
   TAlloc<float> luma((size_t)w*h),
                 profile((size_t)h);
   TAlloc<BYTE>  valid((size_t)w*h);

   if ((float)bundleStrip(f, wl, ud, wd, e, c0, c1, r0, r1, luma(), valid()) < 0.7f*(float)(w*h))
      return false;

   float best = 0.f,
         mean = 0.f;
   int   at = -1;

   for (int r = 1; r + 1 < h; r++)
   {
      float s = 0.f;
      int   n = 0;

      for (int c = 0; c < w; c++)
      {
         size_t i = (size_t)r*w + c;

         if (valid[i - (size_t)w] && valid[i + (size_t)w])
         {
            s += fabsf(luma[i + (size_t)w] - luma[i - (size_t)w]);
            n++;
         }
      }
      profile[r] = n >= w/2 ? s/(float)n : 0.f;
      mean += profile[r];
      if (profile[r] > best)
      {
         best = profile[r];
         at = r;
      }
   }
   mean /= (float)(h - 2);
   if (at < 2 || at + 2 >= h || best < 3.f*mean)
      return false;
   row = (float)r0 + bundlePeak(profile(), at);
   return true;
}

/*--------------------------------------------------------------------------------
   Observations of one round: per wall, the frames' pictures (edges by direction); every overlapping
   pair is split into tiles, each tile of frame g aligned on frame f (subpixel) and turned into a pair
   of image points; every frame showing the top of the wall gives crease points per column band, the
   bottom floor crease points, and each end of the wall its corner edge. One room, forty views: every
   corner and crease is seen from several spin positions and stations, the parallax fixes where it is.
  --------------------------------------------------------------------------------*/
static int bundleObserve(const TMosaicFrame *frames, int count, const TMosaicWall *walls, int wallCount,
                         const TVec3 &ud, const TVec3 &wd, const TLayoutPlan &plan, float e, int reach, TBundleObs *obs,
                         int cap)
{
   int n = 0;

   for (int wi = 0; wi < wallCount && n < cap; wi++)
   {
      const TMosaicWall &wl = walls[wi];
      size_t             pixels = (size_t)wl.cols*wl.rows;
      int                seen[mosaicMaxFrames],
                         m = 0;

      if (wl.cols <= 0)
         continue;

      TAlloc<float> ex(pixels*(size_t)count),
                    ey(pixels*(size_t)count),
                    luma(pixels);
      TAlloc<BYTE>  mask(pixels*(size_t)count),
                    valid(pixels);

      for (int f = 0; f < count && m < mosaicMaxFrames; f++)
      {
         size_t covered = 0u;

         for (int r = 0; r < wl.rows; r++)
            for (int c = 0; c < wl.cols; c++)
            {
               size_t i = (size_t)r*wl.cols + c;
               float  bgr[3],
                      s = mosaicSample(frames[f], bundleCanvasPoint(wl, ud, wd, e, (float)c, (float)r), bgr);

               valid[i] = s > 0.f ? 1u : 0u;
               luma[i] = s > 0.f ? 0.114f*bgr[0] + 0.587f*bgr[1] + 0.299f*bgr[2] : 0.f;
               covered += valid[i];
            }
         if ((float)covered < cMinCoverage*(float)pixels)
            continue;
         mosaicEdges(luma(), valid(), wl.cols, wl.rows, ex() + pixels*(size_t)m, ey() + pixels*(size_t)m,
                     mask() + pixels*(size_t)m);
         seen[m++] = f;
      }
      for (int i = 0; i < m && n < cap; i++)
      {
         const TMosaicFrame &fi = frames[seen[i]];
         LPCBYTE             mi = mask() + pixels*(size_t)i;

         // ceiling crease points along the wall
         for (int c0 = 0; c0 + bundleTile <= wl.cols && n < cap; c0 += bundleTile)
         {
            float row = bundleCreaseRow(ey() + pixels*(size_t)i, mi, wl.cols, wl.rows, c0, c0 + bundleTile),
                  u = 0.f,
                  v = 0.f;

            if (row < 0.f
                || !bundleProject(fi, bundleCanvasPoint(wl, ud, wd, e, (float)(c0 + bundleTile/2), row), u, v))
               continue;
            obs[n].f = seen[i];
            obs[n].g = bundleCeiling;
            obs[n].wall = wi;
            obs[n].uf = u;
            obs[n].vf = v;
            obs[n].w = 1.f;
            n++;
         }

         // floor crease points: with the ceiling they give the distance to the wall from the room height alone
         for (int c0 = 0; c0 + bundleTile <= wl.cols && n < cap; c0 += bundleTile)
         {
            float row = 0.f,
                  u = 0.f,
                  v = 0.f;

            if (!bundleFloorRow(fi, wl, ud, wd, e, c0, c0 + bundleTile, row)
                || !bundleProject(fi, bundleCanvasPoint(wl, ud, wd, e, (float)(c0 + bundleTile/2), row), u, v))
               continue;
            obs[n].f = seen[i];
            obs[n].g = bundleFloor;
            obs[n].wall = wi;
            obs[n].uf = u;
            obs[n].vf = v;
            obs[n].w = cBundleFloorWeight;
            n++;
         }

         // the corner edge at each end of the wall
         for (int end = 0; end < 2 && n < cap; end++)
         {
            int               v = (wi + end)%wallCount;
            const TPlanPoint &p = plan.verts[v];
            float             along = wl.constU ? p.w : p.u,
                              col = 0.f,
                              row = 0.f,
                              u = 0.f,
                              y = 0.f;
            int               cv = (int)floorf((along - wl.s0)*wl.sdir*(float)mosaicPxPerM + 0.5f);

            if (!bundleCornerColumn(fi, wl, ud, wd, e, cv, col, row)
                || !bundleProject(fi, bundleCanvasPoint(wl, ud, wd, e, col, row), u, y))
               continue;
            obs[n].f = seen[i];
            obs[n].g = bundleCorner;
            obs[n].wall = v;
            obs[n].uf = u;
            obs[n].vf = y;
            obs[n].ug = e - (row + 0.5f)/(float)mosaicPxPerM; // height of the measured point above the camera
            obs[n].w = cBundleCornerWeight;
            n++;
         }

         // tiles of every overlapping pair
         for (int j = i + 1; j < m && n < cap; j++)
         {
            const TMosaicFrame &fj = frames[seen[j]];
            LPCBYTE             mj = mask() + pixels*(size_t)j;

            for (int r0 = 0; r0 + bundleTile <= wl.rows && n < cap; r0 += bundleTile)
               for (int c0 = 0; c0 + bundleTile <= wl.cols && n < cap; c0 += bundleTile)
               {
                  size_t both = 0u;
                  int    box[4] = { c0, r0, c0 + bundleTile, r0 + bundleTile };

                  for (int r = r0; r < r0 + bundleTile; r++)
                     for (int c = c0; c < c0 + bundleTile; c++)
                     {
                        size_t k = (size_t)r*wl.cols + c;

                        both += mi[k] && mj[k] ? 1u : 0u;
                     }
                  if ((float)both < 0.6f*(float)(bundleTile*bundleTile))
                     continue;

                  float dx = 0.f,
                        dy = 0.f,
                        ux = 0.f,
                        uy = 0.f,
                        ov = 0.f,
                        sx = mosaicAlign(ex() + pixels*(size_t)j, mj, ex() + pixels*(size_t)i, mi, wl.cols, wl.rows,
                                         reach, box, &dx, &uy, &ov),
                        sy = mosaicAlign(ey() + pixels*(size_t)j, mj, ey() + pixels*(size_t)i, mi, wl.cols, wl.rows,
                                         reach, box, &ux, &dy, &ov),
                        q = fminf(sx, sy),
                        cx = (float)(c0 + bundleTile/2),
                        cy = (float)(r0 + bundleTile/2);

                  if (q < cBundleScore || fabsf(dx) >= (float)(reach - 1) || fabsf(dy) >= (float)(reach - 1))
                     continue;

                  // frame j's detail at the tile center sits in frame i at the center + (dx, dy)
                  float uf = 0.f,
                        vf = 0.f,
                        ug = 0.f,
                        vg = 0.f;

                  if (!bundleProject(fi, bundleCanvasPoint(wl, ud, wd, e, cx + dx, cy + dy), uf, vf)
                      || !bundleProject(fj, bundleCanvasPoint(wl, ud, wd, e, cx, cy), ug, vg))
                     continue;
                  obs[n].f = seen[i];
                  obs[n].g = seen[j];
                  obs[n].wall = wi;
                  obs[n].uf = uf;
                  obs[n].vf = vf;
                  obs[n].ug = ug;
                  obs[n].vg = vg;
                  obs[n].w = q;
                  n++;
               }
         }
      }
   }
   return n;
}

//--------------------------------------------------------------------------------
// Parameter vector: delta per frame (3), wall offsets, e (ceiling above the camera), r (spin radius)
static void bundleResiduals(const TMosaicFrame *frames, int count, const TMosaicWall *walls, int wallCount,
                            const TVec3 &ud, const TVec3 &wd, const TBundleObs *obs, int nObs, LPCBYTE use,
                            float ceilingM, const float *theta, const float *prior, int stations, float *res)
{
   int   base = 3*count,
         stationBase = base + wallCount + 2;
   float e = theta[base + wallCount],
         r = theta[base + wallCount + 1];
   int   k = 0;

   for (int o = 0; o < nObs; o++)
   {
      const TBundleObs  &b = obs[o];
      const TMosaicWall &wl = walls[b.wall];
      const TMosaicFrame &ff = frames[b.f];
      TVec3              n = wl.constU ? ud : wd,
                         df = { ff.delta.x + theta[3*b.f], ff.delta.y + theta[3*b.f + 1], ff.delta.z + theta[3*b.f + 2] },
                         pf;
      float              offset = theta[base + b.wall],
                         sw = sqrtf(b.w),
                         fx = ff.station > 0 ? theta[stationBase + 2*(ff.station - 1)] : 0.f,
                         fz = ff.station > 0 ? theta[stationBase + 2*(ff.station - 1) + 1] : 0.f;
      bool               ok = use[o] && bundleHit(ff, df, r, fx, fz, b.uf, b.vf, n, offset, pf);

      if (b.g >= 0)
      {
         const TMosaicFrame &fg = frames[b.g];
         TVec3               dg = { fg.delta.x + theta[3*b.g], fg.delta.y + theta[3*b.g + 1],
                                    fg.delta.z + theta[3*b.g + 2] };
         float               gx = fg.station > 0 ? theta[stationBase + 2*(fg.station - 1)] : 0.f,
                             gz = fg.station > 0 ? theta[stationBase + 2*(fg.station - 1) + 1] : 0.f;

         /* scale-free: f's hit on the wall reprojected into g, in pixels (meters on the wall would let the whole
            room shrink to cut the residuals) */
         float u = 0.f,
               v = 0.f;

         ok = ok && bundleProjectAt(fg, dg, r, gx, gz, pf, u, v);
         res[k++] = ok ? sw*(u - b.ug)/cBundleSigmaPixel : 0.f;
         res[k++] = ok ? sw*(v - b.vg)/cBundleSigmaPixel : 0.f;
      }
      else if (b.g == bundleCorner)
      {
         // the plan vertex's vertical edge must pass through the measured point: distance across it, in pixels
         int   i = (b.wall + wallCount - 1)%wallCount;
         float offI = theta[base + i],
               offV = theta[base + b.wall],
               pu = walls[i].constU ? offI : offV,
               pw = walls[i].constU ? offV : offI,
               u0 = 0.f,
               v0 = 0.f,
               u1 = 0.f,
               v1 = 0.f;
         TVec3 top = { pu*ud.x + pw*wd.x, b.ug, pu*ud.z + pw*wd.z },
               low = { top.x, b.ug - 0.3f, top.z };
         bool  seen = use[o] && bundleProjectAt(ff, df, r, fx, fz, top, u0, v0)
                      && bundleProjectAt(ff, df, r, fx, fz, low, u1, v1);
         float lx = u1 - u0,
               ly = v1 - v0,
               len = sqrtf(lx*lx + ly*ly);

         res[k++] = seen && len > 1.f ? sw*((b.uf - u0)*ly - (b.vf - v0)*lx)/len/cBundleSigmaCorner : 0.f;
      }
      else
      {
         // the crease must sit at the ceiling (the floor) height: the height error seen as an angle from the camera
         TVec3 c = bundleCenter(ff, df, r, fx, fz);
         float dist = sqrtf((pf.x - c.x)*(pf.x - c.x) + (pf.y - c.y)*(pf.y - c.y) + (pf.z - c.z)*(pf.z - c.z)),
               level = b.g == bundleFloor ? e - ceilingM : e;

         res[k++] = ok && dist > 0.1f ? sw*((pf.y - level)/dist)/cBundleSigmaAngle : 0.f;
      }
   }

   /* the gyroscope between consecutive keyframes of one station: the relative rotation of the model must be the
      attitude's (camera x and z axes of the later frame, seen from the earlier one) */
   for (int f = 0; f + 1 < count; f++)
   {
      const TMosaicFrame &a = frames[f],
                         &g = frames[f + 1];
      bool                linked = a.station == g.station;
      TVec3               da = { a.delta.x + theta[3*f], a.delta.y + theta[3*f + 1], a.delta.z + theta[3*f + 2] },
                          dg = { g.delta.x + theta[3*f + 3], g.delta.y + theta[3*f + 4], g.delta.z + theta[3*f + 5] },
                          axes[2] = { { 1.f, 0.f, 0.f }, { 0.f, 0.f, 1.f } };

      for (int q = 0; q < 2; q++)
      {
         const TVec3 &v = axes[q];
         TVec3        model = bundleToCamera(a, da, bundleToWorld(g, dg, v)),
                      w = { g.gyroX.x*v.x + g.gyroY.x*v.y + g.gyroZ.x*v.z, g.gyroX.y*v.x + g.gyroY.y*v.y + g.gyroZ.y*v.z,
                            g.gyroX.z*v.x + g.gyroY.z*v.y + g.gyroZ.z*v.z };
         float        gx = w.x*a.gyroX.x + w.y*a.gyroX.y + w.z*a.gyroX.z,
                      gy = w.x*a.gyroY.x + w.y*a.gyroY.y + w.z*a.gyroY.z,
                      gz = w.x*a.gyroZ.x + w.y*a.gyroZ.y + w.z*a.gyroZ.z;

         res[k++] = linked ? (model.x - gx)/cBundleSigmaGyro : 0.f;
         res[k++] = linked ? (model.y - gy)/cBundleSigmaGyro : 0.f;
         res[k++] = linked ? (model.z - gz)/cBundleSigmaGyro : 0.f;
      }
   }
   for (int i = 0; i < 3*count; i++)
      res[k++] = theta[i]/cBundleSigmaDelta;
   for (int i = 0; i < wallCount; i++)
      res[k++] = (theta[base + i] - prior[base + i])/cBundleSigmaWall;
   res[k++] = (e - prior[base + wallCount])/cBundleSigmaCeilingHeight;
   res[k++] = (r - prior[base + wallCount + 1])/cBundleSigmaRadius;
   for (int i = 0; i < 2*stations; i++)
      res[k++] = (theta[stationBase + i] - prior[stationBase + i])/cBundleSigmaStation;
}

//--------------------------------------------------------------------------------
static int bundleResidualCount(const TBundleObs *obs, int nObs, int count, int wallCount, int stations)
{
   int m = 3*count + wallCount + 2 + 2*stations + (count > 1 ? 6*(count - 1) : 0); // priors, gyroscope links

   for (int o = 0; o < nObs; o++)
      m += obs[o].g >= 0 ? 2 : 1;
   return m;
}

/*--------------------------------------------------------------------------------
   Gauss-Newton on the parameter vector with a numeric Jacobian (each residual is a cheap ray-plane
   intersection), Levenberg damping and Huber weights (k = 2 sigma). Returns the final cost.
  --------------------------------------------------------------------------------*/
static float bundleSolve(const TMosaicFrame *frames, int count, const TMosaicWall *walls, int wallCount,
                         const TVec3 &ud, const TVec3 &wd, const TBundleObs *obs, int nObs, LPCBYTE use,
                         float ceilingM, float *theta, const float *prior, int stations, int iterations)
{
   int           np = 3*count + wallCount + 2 + 2*stations,
                 nr = bundleResidualCount(obs, nObs, count, wallCount, stations);
   TAlloc<float> res((size_t)nr),
                 trial((size_t)nr),
                 jac((size_t)nr*np),
                 hw((size_t)nr),
                 ata((size_t)np*np),
                 atb((size_t)np),
                 step((size_t)np),
                 moved((size_t)np);
   float         lambda = 1e-3f,
                 cost = 0.f;

   for (int it = 0; it < iterations; it++)
   {
      bundleResiduals(frames, count, walls, wallCount, ud, wd, obs, nObs, use, ceilingM, theta, prior, stations, res());
      cost = 0.f;
      for (int i = 0; i < nr; i++)
      {
         float a = fabsf(res[i]);

         hw[i] = a <= cBundleHuber ? 1.f : cBundleHuber/a;
         cost += hw[i]*res[i]*res[i];
      }
      for (int p = 0; p < np; p++)
      {
         float eps = p < 3*count ? 1e-4f : 1e-3f,
               keep = theta[p];

         theta[p] = keep + eps;
         bundleResiduals(frames, count, walls, wallCount, ud, wd, obs, nObs, use, ceilingM,
                         theta, prior, stations, trial());
         theta[p] = keep;
         for (int i = 0; i < nr; i++)
            jac[(size_t)i*np + p] = (trial[i] - res[i])/eps;
      }

      bool improved = false;

      for (int attempt = 0; attempt < 6 && !improved; attempt++)
      {
         memset(ata(), 0, sizeof(float)*(size_t)np*np);
         memset(atb(), 0, sizeof(float)*(size_t)np);
         for (int i = 0; i < nr; i++)
         {
            const float *row = jac() + (size_t)i*np;

            for (int p = 0; p < np; p++)
            {
               if (row[p] == 0.f)
                  continue;
               atb[p] -= hw[i]*row[p]*res[i];
               for (int q = 0; q < np; q++)
                  ata[(size_t)p*np + q] += hw[i]*row[p]*row[q];
            }
         }
         for (int p = 0; p < np; p++)
            ata[(size_t)p*np + p] *= 1.f + lambda;
         if (!mosaicSolve(ata(), atb(), step(), np))
            break;
         for (int p = 0; p < np; p++) // a held plan: its walls and the ceiling above the camera never step
            moved[p] = mosaicPlanFixed && p >= 3*count && p <= 3*count + wallCount ? theta[p] : theta[p] + step[p];
         bundleResiduals(frames, count, walls, wallCount, ud, wd, obs, nObs, use, ceilingM,
                         moved(), prior, stations, trial());

         float newCost = 0.f;

         for (int i = 0; i < nr; i++)
            newCost += hw[i]*trial[i]*trial[i];
         if (newCost < cost)
         {
            memcpy(theta, moved(), sizeof(float)*(size_t)np);
            lambda = fmaxf(1e-6f, 0.3f*lambda);
            improved = true;
         }
         else
            lambda *= 10.f;
      }
      if (!improved)
         break;
   }
   return cost;
}

/*--------------------------------------------------------------------------------
   Rounds of observe -> solve -> apply: the frames get their deltas and centers, the plan its wall
   offsets (rectangle corners rebuilt from consecutive walls) and camera height.
  --------------------------------------------------------------------------------*/
static void mosaicBundle(TMosaicFrame *frames, int count, TLayoutPlan &plan, float &spinRadiusM, int rounds)
{
   float a = plan.axisDeg*0.01745329f;
   TVec3 ud = { sinf(a), 0.f, -cosf(a) },
         wd = { cosf(a), 0.f, sinf(a) };
   int   wallCount = plan.vertexCount,
         stations = 0,
         base = 3*count,
         stationBase = base + wallCount + 2;

   for (int f = 0; f < count; f++)
      stations = frames[f].station > stations ? frames[f].station : stations;

   int np = stationBase + 2*stations;

   for (int i = 0; i < wallCount; i++)
   {
      int j = (i + 1)%wallCount;

      if ((fabsf(plan.verts[i].u - plan.verts[j].u) < fabsf(plan.verts[i].w - plan.verts[j].w))
          == (fabsf(plan.verts[j].u - plan.verts[(j + 1)%wallCount].u)
              < fabsf(plan.verts[j].w - plan.verts[(j + 1)%wallCount].w)))
      {
         printf("  bundle: consecutive parallel walls - not adjusted\n");
         return;
      }
   }
   for (int round = 0; round < rounds; round++)
   {
      TMosaicWall        walls[layoutMaxVerts];
      TAlloc<TBundleObs> obs((size_t)bundleMaxObs);
      TAlloc<float>      theta((size_t)np),
                         prior((size_t)np);
      float              e = plan.ceilingM - plan.cameraHeightM;

      mosaicWallGeometry(plan, ud, wd, walls);

      // the first round searches wide: the corner stations start from a rough guess of where they stood
      int          nObs = bundleObserve(frames, count, walls, wallCount, ud, wd, plan, e,
                                        round == 0 ? mosaicCoarseReach : mosaicPairReach, obs(), bundleMaxObs),
                   pairs = 0,
                   floors = 0,
                   corners = 0;
      TAlloc<BYTE> use((size_t)(nObs > 0 ? nObs : 1));

      for (int o = 0; o < nObs; o++)
      {
         use[o] = 1u;
         pairs += obs[o].g >= 0 ? 1 : 0;
         floors += obs[o].g == bundleFloor ? 1 : 0;
         corners += obs[o].g == bundleCorner ? 1 : 0;
      }
      memset(theta(), 0, sizeof(float)*(size_t)np);
      for (int i = 0; i < wallCount; i++)
         theta[base + i] = walls[i].offset;
      theta[base + wallCount] = e;
      theta[base + wallCount + 1] = spinRadiusM;
      for (int f = 0; f < count; f++)
         if (frames[f].station > 0)
         {
            theta[stationBase + 2*(frames[f].station - 1)] = frames[f].stationX;
            theta[stationBase + 2*(frames[f].station - 1) + 1] = frames[f].stationZ;
         }
      memcpy(prior(), theta(), sizeof(float)*(size_t)np);

      float cost = bundleSolve(frames, count, walls, wallCount, ud, wd, obs(), nObs, use(), plan.ceilingM, theta(),
                               prior(), stations, bundleIterations);

      // apply: rotations, spin radius, camera centers, walls, camera height
      spinRadiusM = theta[base + wallCount + 1];
      for (int f = 0; f < count; f++)
      {
         TVec3 d = { theta[3*f], theta[3*f + 1], theta[3*f + 2] };

         frames[f].delta.x += d.x;
         frames[f].delta.y += d.y;
         frames[f].delta.z += d.z;

         if (frames[f].station > 0)
         {
            frames[f].stationX = theta[stationBase + 2*(frames[f].station - 1)];
            frames[f].stationZ = theta[stationBase + 2*(frames[f].station - 1) + 1];
         }

         TVec3 c = bundleCenter(frames[f], frames[f].delta, spinRadiusM, frames[f].stationX, frames[f].stationZ);

         frames[f].camX = c.x;
         frames[f].camZ = c.z;
      }
      plan.cameraHeightM = plan.ceilingM - theta[base + wallCount];
      for (int i = 0; i < wallCount; i++)
      {
         int   j = (i + 1)%wallCount;
         float off = theta[base + i],
               next = theta[base + j];

         // corner j joins wall i and wall j
         if (walls[i].constU)
         {
            plan.verts[j].u = off;
            plan.verts[j].w = next;
         }
         else
         {
            plan.verts[j].w = off;
            plan.verts[j].u = next;
         }
      }

      float minU = 1e9f,
            maxU = -1e9f,
            minW = 1e9f,
            maxW = -1e9f;

      for (int i = 0; i < wallCount; i++)
      {
         minU = fminf(minU, plan.verts[i].u);
         maxU = fmaxf(maxU, plan.verts[i].u);
         minW = fminf(minW, plan.verts[i].w);
         maxW = fmaxf(maxW, plan.verts[i].w);
      }
      plan.extentU = maxU - minU;
      plan.extentW = maxW - minW;
      printf("  bundle round %d: %d observations (%d pairs, %d floor, %d corner), cost %.1f -> room %.3f x %.3f m, "
             "camera %.3f m, spin radius %.3f m\n", round + 1, nObs, pairs, floors, corners, cost, plan.extentU,
             plan.extentW, plan.cameraHeightM, spinRadiusM);
   }
}

/*--------------------------------------------------------------------------------
   One luminance factor per frame and wall, elected by a cosine-balanced vote. A wall is evenly lit
   paint seen by several frames: at each spot of a sparse grid the elected color is the mean of the
   frames seeing it weighted by the cosine of their angle of attack (head-on views are the truest);
   each frame's factor is then the MEDIAN, over the spots it shares, of elected / its own - the same
   content on both sides of the ratio, so dark furniture or a shadow does not skew it.
   gains[(wall*count + frame)*3 + channel].
  --------------------------------------------------------------------------------*/
static void mosaicElect(const TMosaicFrame *frames, int count, const TMosaicWall *walls, int wallCount,
                        const TVec3 &ud, const TVec3 &wd, const TLayoutPlan &plan, float *gains)
{
   TAlloc<DWORD> hist((size_t)count*3u*mosaicRatioBins);
   TAlloc<DWORD> shared((size_t)count);
   TAlloc<float> color((size_t)count*3u),
                 cosine((size_t)count);
   TAlloc<BYTE>  seen((size_t)count);

   for (size_t i = 0; i < (size_t)wallCount*count*3u; i++)
      gains[i] = 1.f;
   for (int wi = 0; wi < wallCount; wi++)
   {
      const TMosaicWall &wl = walls[wi];
      TVec3              n = wl.constU ? ud : wd;

      memset(hist(), 0, sizeof(DWORD)*(size_t)count*3u*mosaicRatioBins);
      memset(shared(), 0, sizeof(DWORD)*(size_t)count);
      for (int r = 0; r < wl.rows; r += mosaicEqualStep)
         for (int c = 0; c < wl.cols; c += mosaicEqualStep)
         {
            TVec3 p = mosaicPoint(wl, ud, wd, plan.ceilingM, plan.cameraHeightM, (float)c, (float)r);
            float elected[3] = {},
                  weight = 0.f;
            int   views = 0;

            for (int f = 0; f < count; f++)
            {
               float bgr[3],
                     s = mosaicSample(frames[f], p, bgr),
                     dx = p.x - frames[f].camX,
                     dz = p.z - frames[f].camZ;

               seen[f] = s > 0.f && fminf(bgr[0], fminf(bgr[1], bgr[2])) > 8.f
                         && fmaxf(bgr[0], fmaxf(bgr[1], bgr[2])) < 245.f ? 1u : 0u;
               if (!seen[f])
                  continue;
               cosine[f] = fabsf(dx*n.x + dz*n.z)/sqrtf(dx*dx + p.y*p.y + dz*dz + 1e-9f);
               for (int ch = 0; ch < 3; ch++)
               {
                  color[(size_t)f*3u + ch] = bgr[ch];
                  elected[ch] += cosine[f]*bgr[ch];
               }
               weight += cosine[f];
               views++;
            }
            if (views < 2 || weight <= 0.f)
               continue;
            for (int f = 0; f < count; f++)
            {
               if (!seen[f])
                  continue;
               shared[f]++;
               for (int ch = 0; ch < 3; ch++)
               {
                  float ratio = (elected[ch]/weight)/color[(size_t)f*3u + ch];
                  int   bin = (int)((logf(ratio) + cRatioRange)/(2.f*cRatioRange)*(float)mosaicRatioBins);

                  if (bin >= 0 && bin < mosaicRatioBins)
                     hist[((size_t)f*3u + ch)*mosaicRatioBins + bin]++;
               }
            }
         }
      for (int f = 0; f < count; f++)
      {
         if (shared[f] < (DWORD)mosaicEqualMinShared)
            continue;
         for (int ch = 0; ch < 3; ch++)
         {
            const DWORD *h = hist() + ((size_t)f*3u + ch)*mosaicRatioBins;
            DWORD        total = 0u,
                         acc = 0u;
            int          b = 0;

            for (int k = 0; k < mosaicRatioBins; k++)
               total += h[k];
            while (b < mosaicRatioBins - 1 && acc + h[b] < total/2u)
               acc += h[b++];
            gains[((size_t)wi*count + f)*3u + ch] = expf(((float)b + 0.5f)/(float)mosaicRatioBins*2.f*cRatioRange
                                                          - cRatioRange);
         }
      }
   }
}

/*--------------------------------------------------------------------------------
   The room corner in the picture: the plan vertex whose floor-to-ceiling edge projects inside it
   with most of its length (at least minIn of 10 points, 5% off the border; a tie goes to the edge nearer
   the picture's middle) - the frame shows X, Y and Z of the room at once, the ones the fit trusts
   (user, 2026-09-28). A floor tile grid measures both axes too, but no corner. -1: none.
  --------------------------------------------------------------------------------*/
static int mosaicSeenCorner(const TMosaicFrame &f, const TLayoutPlan &plan, const TVec3 &ud, const TVec3 &wd, int minIn)
{
   int   best = -1,
         bestIn = minIn - 1;
   float bestOff = 1e9f;

   for (int k = 0; k < plan.vertexCount; k++)
   {
      int   inView = 0;
      float off = 0.f;

      for (int s = 0; s < 10; s++)
      {
         TVec3 p = { plan.verts[k].u*ud.x + plan.verts[k].w*wd.x,
                     plan.ceilingM*((float)s + 0.5f)/10.f - plan.cameraHeightM,
                     plan.verts[k].u*ud.z + plan.verts[k].w*wd.z };
         float u,
               v;

         if (bundleProject(f, p, u, v) && u > 0.05f*(float)f.w && u < 0.95f*(float)f.w && v > 0.05f*(float)f.h
             && v < 0.95f*(float)f.h)
         {
            inView++;
            off += fabsf(u - f.k.cx) + fabsf(v - f.k.cy);
         }
      }
      if (inView < minIn)
         continue;
      off /= (float)inView;
      if (inView > bestIn || (inView == bestIn && off < bestOff))
      {
         best = k;
         bestIn = inView;
         bestOff = off;
      }
   }
   return best;
}

//--------------------------------------------------------------------------------
// The plan vertex nearest to where a frame looks, in azimuth from its camera (a named corner frame whose edge falls outside)
static int mosaicAimedCorner(const TMosaicFrame &f, const TLayoutPlan &plan, const TVec3 &ud, const TVec3 &wd)
{
   TVec3 ahead = { 0.f, 0.f, -1.f },
         fw = bundleToWorld(f, f.delta, ahead);
   int   best = -1;
   float bestCos = -2.f;

   for (int k = 0; k < plan.vertexCount; k++)
   {
      float dx = plan.verts[k].u*ud.x + plan.verts[k].w*wd.x - f.camX,
            dz = plan.verts[k].u*ud.z + plan.verts[k].w*wd.z - f.camZ,
            cs = (dx*fw.x + dz*fw.z)/(sqrtf(dx*dx + dz*dz)*sqrtf(fw.x*fw.x + fw.z*fw.z) + 1e-6f);

      if (cs > bestCos)
      {
         bestCos = cs;
         best = k;
      }
   }
   return best;
}

//--------------------------------------------------------------------------------
// Point (u, w) inside the plan polygon (even-odd rule)
static bool mosaicInside(const TLayoutPlan &plan, float u, float w)
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

// The floor canvas seen from above: up and right in plan axes (u, w), the top and left edges, the size
struct TFloorCanvas {
   float upU,
         upW,
         rightU,
         rightW,
         top,
         left;
   int   cols,
         rows;
};

/*--------------------------------------------------------------------------------
   The floor canvas P (user, 2026-09-28: N up). The wall named N (--face-names) is the top edge - up
   is its outward side - and clockwise from it L is on the right, S at the bottom, O on the left, a
   view from above, not mirrored. Unnamed walls: u up, w right. 4 mm per pixel over the plan.
  --------------------------------------------------------------------------------*/
static void mosaicFloorCanvas(const TLayoutPlan &plan, TFloorCanvas &fc)
{
   float hi = -1e9f,
         lo = 1e9f,
         rl = 1e9f,
         rh = -1e9f;

   fc.upU = 1.f;
   fc.upW = 0.f;
   for (int i = 0; i < plan.vertexCount; i++)
      if (!strcmp(mosaicFaceNames[i], "N"))
      {
         const TPlanPoint &p = plan.verts[i],
                          &q = plan.verts[(i + 1)%plan.vertexCount];
         bool              constU = fabsf(p.u - q.u) < fabsf(p.w - q.w);
         float             side = (constU ? p.u + q.u : p.w + q.w) >= 0.f ? 1.f : -1.f;

         fc.upU = constU ? side : 0.f;
         fc.upW = constU ? 0.f : side;
      }
   fc.rightU = -fc.upW; // clockwise from up, seen from above (w is clockwise of u)
   fc.rightW = fc.upU;
   for (int i = 0; i < plan.vertexCount; i++)
   {
      float a = plan.verts[i].u*fc.upU + plan.verts[i].w*fc.upW,
            b = plan.verts[i].u*fc.rightU + plan.verts[i].w*fc.rightW;

      hi = fmaxf(hi, a);
      lo = fminf(lo, a);
      rh = fmaxf(rh, b);
      rl = fminf(rl, b);
   }
   fc.top = hi;
   fc.left = rl;
   fc.cols = (int)((rh - rl)*(float)mosaicPxPerM) + 1;
   fc.rows = (int)((hi - lo)*(float)mosaicPxPerM) + 1;
}

//--------------------------------------------------------------------------------
// Plan point (u, w) of floor canvas pixel (c, r)
static void mosaicFloorAt(const TFloorCanvas &fc, int c, int r, float &pu, float &pw)
{
   float a = fc.top - ((float)r + 0.5f)/(float)mosaicPxPerM,
         b = fc.left + ((float)c + 0.5f)/(float)mosaicPxPerM;

   pu = fc.upU*a + fc.rightU*b;
   pw = fc.upW*a + fc.rightW*b;
}

/*--------------------------------------------------------------------------------
   The fifth face (user, 2026-09-28): the floor seen straight down from the ceiling, 4 mm per pixel,
   plan axes (u up, w right), cut to the polygon. Every frame placed by the refinement (rotations,
   station positions, spin radius and camera height fitted on the ceiling creases and corners - the
   gyroscope only started them) that reaches the floor adds its crop, weighted by the picture falloff
   times sin^3 of the depression: the steep views are the sharp ones and furniture leans least in
   them. piso_<U>x<W>m.bmp. Furniture is laid flat as floor, as it is.
  --------------------------------------------------------------------------------*/
static void mosaicFloor(const TMosaicFrame *frames, int count, const TLayoutPlan &plan, const TVec3 &ud, const TVec3 &wd,
                        LPCSTR outDir)
{
   float        h = plan.cameraHeightM;
   TFloorCanvas fc;

   if (plan.vertexCount < 3 || !(h > 0.3f))
      return;
   mosaicFloorCanvas(plan, fc);

   int           cols = fc.cols,
                 rows = fc.rows,
                 used = 0;
   size_t        pixels = (size_t)cols*rows;
   TAlloc<float> acc(pixels*4u);
   TAlloc<BYTE>  seen((size_t)count);
   char          path[mosaicPathMax];

   memset(acc(), 0, sizeof(float)*pixels*4u);
   memset(seen(), 0, (size_t)count);
   for (int r = 0; r < rows; r++)
      for (int c = 0; c < cols; c++)
      {
         float pu,
               pw;

         mosaicFloorAt(fc, c, r, pu, pw);
         if (!mosaicInside(plan, pu, pw))
            continue;

         TVec3  p = { pu*ud.x + pw*wd.x, -h, pu*ud.z + pw*wd.z };
         float *o = acc() + ((size_t)r*cols + c)*4u;

         for (int f = 0; f < count; f++)
         {
            float bgr[3],
                  s = frames[f].xyz ? mosaicSample(frames[f], p, bgr) : 0.f; // the merge takes only XYZ frames

            if (s <= 0.f)
               continue;

            float dx = p.x - frames[f].camX,
                  dz = p.z - frames[f].camZ,
                  sn = h/sqrtf(dx*dx + h*h + dz*dz);

            s *= sn*sn*sn;
            o[0] += s*bgr[0];
            o[1] += s*bgr[1];
            o[2] += s*bgr[2];
            o[3] += s;
            seen[f] = 1u;
         }
      }
   for (int f = 0; f < count; f++)
      used += seen[f] ? 1 : 0;
   snprintf(path, sizeof(path), "%s/piso_%.2fx%.2fm.bmp", outDir, (float)rows/(float)mosaicPxPerM,
            (float)cols/(float)mosaicPxPerM);
   mosaicWriteBMP(path, acc(), cols, rows);
   printf("  floor: %d of %d frames on the floor, %dx%d px, camera %.3f m\n", used, count, cols, rows, h);
}

enum {
   mosaicFitMaxObs    = 4000,
   mosaicFitRounds    = 1,  // measure once (the elected lines are the observations), solve, then one check measure
   mosaicFitIters     = 30,
   mosaicFitParams    = 6,  // world rotation (3), camera height offset and - a corner station seeing ceiling AND floor
                            // creases (their spread on the wall fixes the distance) - station x, z; else it stays
   mosaicFitCreaseReach = 300, // crease columns this near the corner (1.2 m)
   mosaicFitCornerReach = 60,  // corner edge searched +-24 cm around the plan corner (wider: the jambs win)
   mosaicFitMaxCands  = 8192, // Sobel peaks of one band
   mosaicRansacIters  = 400,
   mosaicRansacLines  = 3,    // lines taken out one after the other
   mosaicFitMinPoints = 12    // a line needs this many peaks
};

static const float cFitSigmaObs = 0.005f,  // meters on the wall
                   cFitSigmaDelta = 0.03f, // radians off the frame's vanishing rotation
                   cFitSigmaHeight = 0.15f,
                   cFitSigmaStation = 0.6f,  // a corner station starts a quarter of the way in from a plan corner
                   cFitEdge = 40.f,        // Sobel response (weights 1-2-1, 2 pixels apart) of a crease or corner edge
                   cFitRansacTol = 1.5f,   // pixels off a RANSAC line
                   cFitMaxSlope = 0.14f,   // lines within 8 degrees of level (creases) or of plumb (corner edge),
                   cFitMaxSlopeFine = 0.035f, // 2 degrees once the first solve set the rotation
                   cFitMinSupport = 0.25f, // share of the strongest line a candidate needs (the true crease may be short)
                   cFitMissing = 50.f,     // error score of a wall without a crease, or without the edge
                   cFitPointWeight = 3.f,  // the corner point where the two creases meet, standing in for a missing edge (9 line points)
                   cFitMinCrossing = 0.2f; // sine of the angle the two creases must cross at in the picture

// The lines a corner frame is measured on
enum TFitKind {
   fkCeiling,
   fkFloor,
   fkCorner
};

// One measured point of the corner in the picture: on a crease or on the corner edge of a wall
struct TFitObs {
   float    u,
            v;
   int      wall;
   TFitKind kind;
   float    weight;  // 1 for a line point; the corner point where two creases meet weighs cFitPointWeight
};

//--------------------------------------------------------------------------------
// Luma of a covered canvas pixel; -1 outside or not seen
static float mosaicCanvasLuma(const float *acc, int cols, int rows, int c, int r)
{
   if (c < 0 || r < 0 || c >= cols || r >= rows)
      return -1.f;

   const float *o = acc + ((size_t)r*cols + c)*4u;

   return o[3] > 0.f ? 0.114f*o[0] + 0.587f*o[1] + 0.299f*o[2] : -1.f;
}

/*--------------------------------------------------------------------------------
   Sobel at a canvas pixel: the vertical luma step (rows, weights 1-2-1 across three columns) and the
   horizontal one (columns, 1-2-1 across three rows); false when any sample is not seen.
  --------------------------------------------------------------------------------*/
static bool mosaicCanvasStep(const float *acc, int cols, int rows, int c, int r, float &along, float &across)
{
   along = 0.f;
   across = 0.f;
   for (int d = -1; d <= 1; d++)
   {
      float up = mosaicCanvasLuma(acc, cols, rows, c + d, r - 1),
            dn = mosaicCanvasLuma(acc, cols, rows, c + d, r + 1),
            lf = mosaicCanvasLuma(acc, cols, rows, c - 1, r + d),
            rt = mosaicCanvasLuma(acc, cols, rows, c + 1, r + d);

      if (up < 0.f || dn < 0.f || lf < 0.f || rt < 0.f)
         return false;
      along += (dn - up)*(d ? 1.f : 2.f); // Sobel
      across += (rt - lf)*(d ? 1.f : 2.f);
   }
   return true;
}

//--------------------------------------------------------------------------------
// Subpixel peak of three samples around a local maximum (parabola), in [-0.5, 0.5]
static float mosaicPeakOffset(float before, float at, float after)
{
   float den = before - 2.f*at + after;

   if (!(den < -1e-6f))
      return 0.f;

   float off = 0.5f*(before - after)/den;

   return off < -0.5f ? -0.5f : (off > 0.5f ? 0.5f : off);
}

//--------------------------------------------------------------------------------
// One frame onto one wall canvas, raw colors (weight 1 where seen)
static void mosaicRenderWall(const TMosaicFrame &f, const TMosaicWall &wl, const TLayoutPlan &plan, const TVec3 &ud,
                             const TVec3 &wd, float *acc)
{
   memset(acc, 0, sizeof(float)*(size_t)wl.cols*wl.rows*4u);
   for (int r = 0; r < wl.rows; r++)
      for (int c = 0; c < wl.cols; c++)
      {
         TVec3  p = mosaicPoint(wl, ud, wd, plan.ceilingM, plan.cameraHeightM, (float)c, (float)r);
         float  bgr[3],
               *o = acc + ((size_t)r*wl.cols + c)*4u;

         if (mosaicSample(f, p, bgr) <= 0.f)
            continue;
         o[0] = bgr[0];
         o[1] = bgr[1];
         o[2] = bgr[2];
         o[3] = 1.f;
      }
}

//--------------------------------------------------------------------------------
// Canvas column of plan vertex k on a wall
static float mosaicCornerColumn(const TMosaicWall &wl, const TLayoutPlan &plan, int k)
{
   float s = wl.constU ? plan.verts[k].w : plan.verts[k].u;

   return (s - wl.s0)*wl.sdir*(float)mosaicPxPerM - 0.5f;
}

/*--------------------------------------------------------------------------------
   RANSAC for one straight line y = a*x + b among the points still free (used == 0), slope within
   maxSlope: pairs of points at least 10 apart propose the line, the one with most points within
   cFitRansacTol wins. Deterministic (fixed seed). Returns the count of its points.
  --------------------------------------------------------------------------------*/
static int mosaicRansacLine(const float *x, const float *y, int n, LPCBYTE used, float maxSlope, float &a, float &b)
{
   DWORD seed = 12345u;
   int   unused = 0,
         best = 0;

   for (int i = 0; i < n; i++)
      unused += used[i] ? 0 : 1;
   if (unused < 2)
      return 0;
   for (int it = 0; it < mosaicRansacIters; it++)
   {
      seed = seed*1664525u + 1013904223u;

      int i = (int)((seed >> 8)%(DWORD)n);

      seed = seed*1664525u + 1013904223u;

      int j = (int)((seed >> 8)%(DWORD)n);

      if (used[i] || used[j] || fabsf(x[j] - x[i]) < 10.f)
         continue;

      float ta = (y[j] - y[i])/(x[j] - x[i]),
            tb = y[i] - ta*x[i];
      int   count = 0;

      if (fabsf(ta) > maxSlope)
         continue;
      for (int p = 0; p < n; p++)
         count += !used[p] && fabsf(y[p] - (ta*x[p] + tb)) <= cFitRansacTol ? 1 : 0;
      if (count > best)
      {
         best = count;
         a = ta;
         b = tb;
      }
   }
   return best;
}

/*--------------------------------------------------------------------------------
   The corner as the frame's own rectified view shows it on one wall (user, 2026-09-28: Sobel peaks,
   RANSAC for the real lines). Candidates are every Sobel peak (to a subpixel) of the band: near the
   corner, the horizontal edges of the top (ceiling crease) or bottom (floor crease) 35% of the
   canvas, and the vertical edges within +-24 cm of the plan corner (corner edge). RANSAC takes out
   up to three lines one after the other; among those with at least cFitMinSupport of the strongest (user:
   on the ceiling the top line wins, the ones below are furniture - wardrobes, shelves - or the
   molding's own lower edge), the crease is the highest (ceiling) or the
   lowest (floor), the corner edge the one nearest the plan corner. Its points are refitted by least
   squares, go back to picture points, and the line is printed against where it must be: creases
   level on the top and bottom canvas rows, the edge vertical on the corner column. error: degrees +
   centimeters off; a wall without any crease or without the edge costs cFitMissing each.
  --------------------------------------------------------------------------------*/
static int mosaicCornerMeasure(const TMosaicFrame &f, const TMosaicWall &wl, int wall, const TLayoutPlan &plan, int k,
                               const TVec3 &ud, const TVec3 &wd, const float *acc, TFitObs *obs, int nObs, LPCSTR name,
                               float maxSlope, float &error, int &creases, int &edges, int &kinds)
{
   static LPCSTR cWhat[3] = { "ceiling crease", "floor crease", "corner edge" };
   int           cols = wl.cols,
                 rows = wl.rows,
                 found[3] = { 0, 0, 0 };
   float         ck = mosaicCornerColumn(wl, plan, k);
   TAlloc<float> px((size_t)mosaicFitMaxCands),
                 py((size_t)mosaicFitMaxCands);
   TAlloc<BYTE>  used((size_t)mosaicFitMaxCands);

   for (int kind = fkCeiling; kind <= fkCorner; kind++)
   {
      int n = 0;

      if (kind != fkCorner)
         for (int c = (int)ck - mosaicFitCreaseReach; c <= (int)ck + mosaicFitCreaseReach; c += 2)
            for (int step = 1; c >= 1 && c + 1 < cols && step < rows*35/100 && n < mosaicFitMaxCands; step++)
            {
               int   r = kind == fkCeiling ? step : rows - 1 - step, // the ceiling from the top, the floor from the bottom
                     dir = kind == fkCeiling ? 1 : -1;
               float along,
                     across,
                     before,
                     after,
                     dummy;

               if (!mosaicCanvasStep(acc, cols, rows, c, r, along, across)
                   || !mosaicCanvasStep(acc, cols, rows, c, r - dir, before, dummy)
                   || !mosaicCanvasStep(acc, cols, rows, c, r + dir, after, dummy))
                  continue;
               if (fabsf(along) < cFitEdge || fabsf(along) < 2.f*fabsf(across)
                   || fabsf(along) < fabsf(before) || fabsf(along) < fabsf(after))
                  continue; // not a peak of a horizontal edge
               px[n] = (float)c;
               py[n] = (float)r + (float)dir*mosaicPeakOffset(fabsf(before), fabsf(along), fabsf(after));
               n++;
            }
      else
         for (int r = rows/10; r < rows*9/10; r += 2)
            for (int c = (int)ck - mosaicFitCornerReach; c <= (int)ck + mosaicFitCornerReach && n < mosaicFitMaxCands; c++)
            {
               float along,
                     across,
                     left,
                     right,
                     dummy;

               if (!mosaicCanvasStep(acc, cols, rows, c, r, along, across)
                   || !mosaicCanvasStep(acc, cols, rows, c - 1, r, dummy, left)
                   || !mosaicCanvasStep(acc, cols, rows, c + 1, r, dummy, right))
                  continue;
               if (fabsf(across) < cFitEdge || fabsf(across) < 2.f*fabsf(along)
                   || fabsf(across) < fabsf(left) || fabsf(across) < fabsf(right))
                  continue; // not a peak of a vertical edge
               px[n] = (float)r; // the edge as column = a*row + b
               py[n] = (float)c + mosaicPeakOffset(fabsf(left), fabsf(across), fabsf(right));
               n++;
            }

      // the real lines, strongest first
      float la[mosaicRansacLines],
            lb[mosaicRansacLines];
      int   lc[mosaicRansacLines],
            lines = 0,
            strongest = 0,
            pick = -1;

      memset(used(), 0, (size_t)n);
      for (int l = 0; l < mosaicRansacLines; l++)
      {
         int count = mosaicRansacLine(px(), py(), n, used(), maxSlope, la[l], lb[l]);

         if (count < mosaicFitMinPoints)
            break;
         lc[l] = count;
         strongest = count > strongest ? count : strongest;
         for (int p = 0; p < n; p++)
            if (fabsf(py[p] - (la[l]*px[p] + lb[l])) <= cFitRansacTol)
               used[p] = 1u;
         lines++;
      }
      for (int l = 0; l < lines; l++)
      {
         float edgeCm = 100.f*(kind == fkCeiling ? la[l]*ck + lb[l] + 0.5f : (float)rows - 0.5f - (la[l]*ck + lb[l]));

         if (kind != fkCorner) // every line the band holds, for the tie-break check
            printf("      %s line %d: %d peaks, slope %+.2f deg, %.1f cm from the %s edge at the corner\n", cWhat[kind], l,
                   lc[l], atanf(la[l])*57.29578f, edgeCm/(float)mosaicPxPerM, kind == fkCeiling ? "top" : "bottom");
         if ((float)lc[l] < cFitMinSupport*(float)strongest)
            continue;
         if (pick < 0)
         {
            pick = l;
            continue;
         }

         float mid = 0.5f*(float)rows,
               at = kind == fkCorner ? fabsf(la[l]*mid + lb[l] - ck) : la[l]*ck + lb[l],
               was = kind == fkCorner ? fabsf(la[pick]*mid + lb[pick] - ck) : la[pick]*ck + lb[pick];

         if ((kind == fkFloor) ? at > was : at < was) // ceiling: highest; floor: lowest; edge: nearest the corner
            pick = l;
      }
      // the cloud and the elected line, back on the full picture (the fit's first measure: pontos_NNN.csv)
      for (int p = 0; mosaicPointsFile && p < n; p++)
      {
         int   line = -1;
         float u,
               v,
               c = kind == fkCorner ? py[p] : px[p],
               r = kind == fkCorner ? px[p] : py[p];
         TVec3 q = mosaicPoint(wl, ud, wd, plan.ceilingM, plan.cameraHeightM, c, r);

         for (int l = 0; l < lines && line < 0; l++)
            if (fabsf(py[p] - (la[l]*px[p] + lb[l])) <= cFitRansacTol)
               line = l;
         if (bundleProject(f, q, u, v))
            fprintf(mosaicPointsFile, "%s,%s,%.1f,%.1f,%d,%d\r\n", cWhat[kind], name, 2.f*u + 0.5f, 2.f*v + 0.5f, line,
                    line >= 0 && line == pick ? 1 : 0);
      }
      if (pick < 0)
      {
         printf("    %s %s: not found (%d peaks)\n", name, cWhat[kind], n);
         continue;
      }

      // least squares on the chosen line's points
      float sx = 0.f,
            sy = 0.f,
            sxx = 0.f,
            sxy = 0.f,
            a = la[pick],
            b = lb[pick];
      int   m = 0;

      for (int p = 0; p < n; p++)
         if (fabsf(py[p] - (la[pick]*px[p] + lb[pick])) <= cFitRansacTol)
         {
            sx += px[p];
            sy += py[p];
            sxx += px[p]*px[p];
            sxy += px[p]*py[p];
            m++;
         }

      float det = (float)m*sxx - sx*sx;

      if (fabsf(det) > 1e-3f)
      {
         a = ((float)m*sxy - sx*sy)/det;
         b = (sy - a*sx)/(float)m;
      }

      float tilt = atanf(a)*57.29578f,
            off = kind == fkCorner ? 100.f*(a*0.5f*(float)rows + b - ck)/(float)mosaicPxPerM
                                   : (kind == fkCeiling ? 100.f*(a*ck + b + 0.5f)/(float)mosaicPxPerM
                                                        : 100.f*((float)rows - 0.5f - (a*ck + b))/(float)mosaicPxPerM);

      found[kind] = 1;
      error += fabsf(tilt) + fabsf(off); // degrees and centimeters off the canvas border
      printf("    %s %s: %d points (%d lines), %s %+.2f deg, %+.1f cm %s\n", name, cWhat[kind], m, lines,
             kind == fkCorner ? "tilt" : "slope", tilt, off,
             kind == fkCorner ? "off the corner at mid-height"
                              : (kind == fkCeiling ? "below the ceiling at the corner" : "above the floor at the corner"));
      for (int p = 0; p < n && nObs < mosaicFitMaxObs; p++)
      {
         if (fabsf(py[p] - (a*px[p] + b)) > cFitRansacTol)
            continue;

         float u,
               v,
               c = kind == fkCorner ? py[p] : px[p],
               r = kind == fkCorner ? px[p] : py[p];
         TVec3 q = mosaicPoint(wl, ud, wd, plan.ceilingM, plan.cameraHeightM, c, r);

         if (!bundleProject(f, q, u, v))
            continue;
         obs[nObs].u = u;
         obs[nObs].v = v;
         obs[nObs].wall = wall;
         obs[nObs].kind = (TFitKind)kind;
         obs[nObs].weight = 1.f;
         nObs++;
      }
   }
   if (!found[fkCeiling] && !found[fkFloor])
      error += cFitMissing;
   if (!found[fkCorner])
      error += cFitMissing;
   creases += found[fkCeiling] + found[fkFloor];
   kinds |= (found[fkCeiling] ? 1 : 0) | (found[fkFloor] ? 2 : 0); // 1: a ceiling crease, 2: a floor crease
   edges += found[fkCorner];
   return nObs;
}

/*--------------------------------------------------------------------------------
   The corner seen where its creases meet (user, 2026-09-28: the vertical crease has little contrast,
   the corner seen on the ceiling confirms it). The picture points of each wall's crease of one kind
   (ceiling or floor) are fitted to a line in the picture (principal axis) and the two lines are
   intersected: the top (or foot) of the corner, where both creases end. It must sit on the plan
   vertex at the ceiling (or floor) height: printed as the offset along wall A and in height, added
   to the error and appended as an observation (along and height on wall A) when it stands in for a
   missing edge (stand); with both edges found it only confirms them. found: the lines
   crossed inside the picture.
  --------------------------------------------------------------------------------*/
static int mosaicCornerPoint(const TMosaicFrame &f, const TMosaicWall *walls, int wallA, int wallB, const TLayoutPlan &plan,
                             int k, const TVec3 &ud, const TVec3 &wd, float radius, TFitKind kind, TFitObs *obs, int nObs,
                             LPCSTR nameA, bool stand, float &error, bool &found)
{
   float mu[2] = { 0.f, 0.f },
         mv[2] = { 0.f, 0.f },
         du[2],
         dv[2];
   int   cnt[2] = { 0, 0 };

   found = false;
   for (int side = 0; side < 2; side++)
   {
      int   wall = side ? wallB : wallA;
      float cuu = 0.f,
            cvv = 0.f,
            cuv = 0.f;

      for (int i = 0; i < nObs; i++)
         if (obs[i].wall == wall && obs[i].kind == kind && obs[i].weight == 1.f)
         {
            mu[side] += obs[i].u;
            mv[side] += obs[i].v;
            cnt[side]++;
         }
      if (cnt[side] < 8)
         return nObs;
      mu[side] /= (float)cnt[side];
      mv[side] /= (float)cnt[side];
      for (int i = 0; i < nObs; i++)
         if (obs[i].wall == wall && obs[i].kind == kind && obs[i].weight == 1.f)
         {
            float a = obs[i].u - mu[side],
                  b = obs[i].v - mv[side];

            cuu += a*a;
            cvv += b*b;
            cuv += a*b;
         }

      float angle = 0.5f*atan2f(2.f*cuv, cuu - cvv);

      du[side] = cosf(angle);
      dv[side] = sinf(angle);
   }

   float det = dv[0]*du[1] - du[0]*dv[1]; // [d0 -d1] (t s) = p1 - p0

   if (fabsf(det) < cFitMinCrossing)
      return nObs; // nearly parallel in the picture: no reliable crossing

   float pu = mu[1] - mu[0],
         pv = mv[1] - mv[0],
         t = (pv*du[1] - pu*dv[1])/det,
         u = mu[0] + t*du[0],
         v = mv[0] + t*dv[0];

   if (u < 0.f || v < 0.f || u >= (float)f.w || v >= (float)f.h)
      return nObs; // the corner falls outside the picture

   const TMosaicWall &wl = walls[wallA];
   TVec3              nrm = wl.constU ? ud : wd,
                      hit;

   if (!bundleHit(f, f.delta, radius, f.stationX, f.stationZ, u, v, nrm, wl.offset, hit))
      return nObs;

   float along = 100.f*((wl.constU ? hit.x*wd.x + hit.z*wd.z : hit.x*ud.x + hit.z*ud.z)
                        - (wl.constU ? plan.verts[k].w : plan.verts[k].u))*wl.sdir,
         height = 100.f*(hit.y + f.camY - (kind == fkCeiling ? plan.ceilingM - plan.cameraHeightM : -plan.cameraHeightM));

   found = true;
   printf("    %s corner where the creases meet: %+.1f cm along %s, %+.1f cm in height%s\n",
          kind == fkCeiling ? "ceiling" : "floor", along, nameA, height, stand ? "" : " (check only: the edges were found)");
   if (!stand)
      return nObs; // the faint vertical edge was found on both walls: the crossing only confirms it
   error += fabsf(along) + fabsf(height);
   for (int c = 0; c < 2 && nObs < mosaicFitMaxObs; c++)
   {
      obs[nObs].u = u;
      obs[nObs].v = v;
      obs[nObs].wall = wallA;
      obs[nObs].kind = c ? kind : fkCorner;
      obs[nObs].weight = cFitPointWeight;
      nObs++;
   }
   return nObs;
}

//--------------------------------------------------------------------------------
// Residuals of the corner fit: observations on the walls (meters / sigma), then the priors
static int mosaicFitResiduals(const TMosaicFrame &f, const TMosaicWall *walls, const TLayoutPlan &plan, int k,
                              const TVec3 &ud, const TVec3 &wd, const TFitObs *obs, int nObs, float radius,
                              const float *theta, const float *start, int np, float *res)
{
   TVec3 delta = { theta[0], theta[1], theta[2] };
   int   n = 0;

   for (int i = 0; i < nObs; i++)
   {
      const TMosaicWall &wl = walls[obs[i].wall];
      TVec3              nrm = wl.constU ? ud : wd,
                         hit;
      float              r = 0.f;

      if (bundleHit(f, delta, radius, np > 4 ? theta[4] : f.stationX, np > 4 ? theta[5] : f.stationZ, obs[i].u, obs[i].v,
                    nrm, wl.offset, hit))
      {
         if (obs[i].kind == fkCorner)
            r = ((wl.constU ? hit.x*wd.x + hit.z*wd.z : hit.x*ud.x + hit.z*ud.z)
                 - (wl.constU ? plan.verts[k].w : plan.verts[k].u))/cFitSigmaObs;
         else if (obs[i].kind == fkCeiling)
            r = (hit.y + theta[3] - (plan.ceilingM - plan.cameraHeightM))/cFitSigmaObs;
         else
            r = (hit.y + theta[3] + plan.cameraHeightM)/cFitSigmaObs;
      }
      res[n++] = r*obs[i].weight;
   }
   for (int i = 0; i < 3; i++)
      res[n++] = (theta[i] - start[i])/cFitSigmaDelta;
   res[n++] = theta[3]/cFitSigmaHeight;
   for (int i = 4; i < np; i++)
      res[n++] = (theta[i] - start[i])/cFitSigmaStation;
   return n;
}

/*--------------------------------------------------------------------------------
   Corner fit of one XYZ frame (user, 2026-09-28: the frame shows the three axes of the corner, so
   its rectification fixes rotation as well as perspective - the ceiling crease level, the corner
   edge vertical). The frame is rendered on the two walls of its corner, the crease and the edge are
   measured there and taken back to picture points; its world rotation and camera height are then
   solved (Gauss-Newton, Levenberg damping) so those points land on the plan's crease line and corner
   line. The plan stays; so does the station, unless the frame belongs to a corner station and shows a
   ceiling and a floor crease (user: frame 42 - the ceiling gives the corner, the floor line from the
   left meets it): their spread on the wall fixes the distance, and the station x, z are solved too.
   One measure on the vanishing-point pose (user: the RANSAC picks there are the right ones), a solve on
   those fixed observations, then a check measure that is printed and never replaces the solve.
  --------------------------------------------------------------------------------*/
static void mosaicCornerFit(TMosaicFrame &f, int k, const TMosaicWall *walls, const TLayoutPlan &plan, const TVec3 &ud,
                            const TVec3 &wd)
{
   int   n = plan.vertexCount,
         wallA = (k + n - 1)%n,
         wallB = k,
         maxPixels = walls[wallA].cols*walls[wallA].rows > walls[wallB].cols*walls[wallB].rows
                     ? walls[wallA].cols*walls[wallA].rows : walls[wallB].cols*walls[wallB].rows,
         np = 4; // rotation and camera height; a corner station seeing both creases frees its x, z (6)
   float radius = sqrtf((f.camX - f.stationX)*(f.camX - f.stationX) + (f.camZ - f.stationZ)*(f.camZ - f.stationZ)),
         start[mosaicFitParams] = { f.delta.x, f.delta.y, f.delta.z, 0.f, f.stationX, f.stationZ };
   char  nameA[16],
         nameB[16];

   TAlloc<float>   acc((size_t)maxPixels*4u);
   TAlloc<TFitObs> obs((size_t)mosaicFitMaxObs);

   mosaicFaceLabel(wallA, nameA, sizeof(nameA));
   mosaicFaceLabel(wallB, nameB, sizeof(nameB));

   TVec3 bestDelta = f.delta; // a round counts only if its own re-measure is better (a new view can lock on another edge)
   float bestCamX = f.camX,
         bestCamZ = f.camZ,
         bestCamY = f.camY,
         bestStationX = f.stationX,
         bestStationZ = f.stationZ,
         bestError = 1e9f;

   printf("  corner fit frame %03d: corner %d, walls %s and %s\n", f.index, k, nameA, nameB);
   for (int round = 0; round <= mosaicFitRounds; round++)
   {
      int   nObs = 0,
            creases = 0,
            edges = 0,
            kinds = 0;
      float error = 0.f,
            slope = round ? cFitMaxSlopeFine : cFitMaxSlope;
      bool  top = false,
            foot = false;

      printf("   round %d:\n", round);
      if (round == 0 && mosaicOutDir)
      {
         char path[mosaicPathMax];

         snprintf(path, sizeof(path), "%s/pontos_%03d.csv", mosaicOutDir, f.index);
         mosaicPointsFile = fopen(path, "wb");
         if (mosaicPointsFile)
            fprintf(mosaicPointsFile, "kind,wall,u,v,line,elected\r\n"); // full-picture pixels, stored orientation
      }
      mosaicRenderWall(f, walls[wallA], plan, ud, wd, acc());
      nObs = mosaicCornerMeasure(f, walls[wallA], wallA, plan, k, ud, wd, acc(), obs(), nObs, nameA, slope, error, creases,
                                 edges, kinds);
      mosaicRenderWall(f, walls[wallB], plan, ud, wd, acc());
      nObs = mosaicCornerMeasure(f, walls[wallB], wallB, plan, k, ud, wd, acc(), obs(), nObs, nameB, slope, error, creases,
                                 edges, kinds);
      if (mosaicPointsFile)
      {
         fclose(mosaicPointsFile);
         mosaicPointsFile = NULL;
      }
      nObs = mosaicCornerPoint(f, walls, wallA, wallB, plan, k, ud, wd, radius, fkCeiling, obs(), nObs, nameA, edges < 2,
                               error, top);
      nObs = mosaicCornerPoint(f, walls, wallA, wallB, plan, k, ud, wd, radius, fkFloor, obs(), nObs, nameA, edges < 2,
                               error, foot);
      if (top || foot)
         error -= cFitMissing*(float)(2 - edges); // confirmed where its creases meet: the faint vertical edge may be missing
      printf("    error %.2f (degrees + centimeters over the lines found)\n", error);
      if (round > 0 && round < mosaicFitRounds && error >= bestError) // the check measure never undoes the solve
      {
         printf("    worse than round %d: its pose is kept\n", round - 1);
         f.delta = bestDelta;
         f.camX = bestCamX;
         f.camZ = bestCamZ;
         f.camY = bestCamY;
         f.stationX = bestStationX;
         f.stationZ = bestStationZ;
         break;
      }
      bestError = error;
      bestDelta = f.delta;
      bestCamX = f.camX;
      bestCamZ = f.camZ;
      bestCamY = f.camY;
      bestStationX = f.stationX;
      bestStationZ = f.stationZ;
      if (round == 0 && !creases) // a crease holds the rotation; the edge alone would let it spin
      {
         printf("    no crease: the frame keeps its vanishing-point pose\n");
         break;
      }
      if (round == 0 && f.station > 0 && (kinds & 3) == 3)
      {
         np = 6; // ceiling and floor creases on the walls: their spread tells the distance, the station is found too
         printf("    ceiling and floor creases in view: the station position is solved too\n");
      }
      if (round == mosaicFitRounds || nObs < 20)
         break; // the last round only measures the result

      int           nr = nObs + np;
      float         theta[mosaicFitParams] = { f.delta.x, f.delta.y, f.delta.z, f.camY, f.stationX, f.stationZ },
                    lambda = 1e-3f;
      TAlloc<float> res((size_t)nr),
                    trial((size_t)nr),
                    jac((size_t)nr*np);

      for (int it = 0; it < mosaicFitIters; it++)
      {
         float cost = 0.f,
               ata[mosaicFitParams*mosaicFitParams],
               atb[mosaicFitParams],
               step[mosaicFitParams],
               moved[mosaicFitParams];

         mosaicFitResiduals(f, walls, plan, k, ud, wd, obs(), nObs, radius, theta, start, np, res());
         for (int i = 0; i < nr; i++)
            cost += res[i]*res[i];
         for (int p = 0; p < np; p++)
         {
            float eps = p < 3 ? 1e-4f : 1e-3f,
                  keepTheta = theta[p];

            theta[p] = keepTheta + eps;
            mosaicFitResiduals(f, walls, plan, k, ud, wd, obs(), nObs, radius, theta, start, np, trial());
            theta[p] = keepTheta;
            for (int i = 0; i < nr; i++)
               jac[(size_t)i*np + p] = (trial[i] - res[i])/eps;
         }

         bool improved = false;

         for (int attempt = 0; attempt < 6 && !improved; attempt++)
         {
            memset(ata, 0, sizeof(ata));
            memset(atb, 0, sizeof(atb));
            for (int i = 0; i < nr; i++)
               for (int p = 0; p < np; p++)
               {
                  float jp = jac[(size_t)i*np + p];

                  atb[p] -= jp*res[i];
                  for (int q = 0; q < np; q++)
                     ata[p*np + q] += jp*jac[(size_t)i*np + q];
               }
            for (int p = 0; p < np; p++)
               ata[p*np + p] *= 1.f + lambda;
            if (!mosaicSolve(ata, atb, step, np))
               break;
            memcpy(moved, theta, sizeof(moved));
            for (int p = 0; p < np; p++)
               moved[p] = theta[p] + step[p];
            mosaicFitResiduals(f, walls, plan, k, ud, wd, obs(), nObs, radius, moved, start, np, trial());

            float newCost = 0.f;

            for (int i = 0; i < nr; i++)
               newCost += trial[i]*trial[i];
            if (newCost < cost)
            {
               memcpy(theta, moved, sizeof(theta));
               lambda = fmaxf(1e-6f, 0.3f*lambda);
               improved = true;
            }
            else
               lambda *= 10.f;
         }
         if (!improved)
            break;
      }
      f.delta.x = theta[0];
      f.delta.y = theta[1];
      f.delta.z = theta[2];
      f.camY = theta[3];
      f.stationX = theta[4];
      f.stationZ = theta[5];

      TVec3 c = bundleCenter(f, f.delta, radius, f.stationX, f.stationZ);

      f.camX = c.x;
      f.camZ = c.z;
      printf("    -> %d points; rotation change %.2f deg, camera height %+.1f cm", nObs,
             57.29578f*sqrtf((theta[0] - start[0])*(theta[0] - start[0]) + (theta[1] - start[1])*(theta[1] - start[1])
                             + (theta[2] - start[2])*(theta[2] - start[2])),
             100.f*f.camY);
      if (np > 4)
         printf(", station (%+.2f, %+.2f) m, moved %.2f m", f.stationX, f.stationZ,
                sqrtf((f.stationX - start[4])*(f.stationX - start[4]) + (f.stationZ - start[5])*(f.stationZ - start[5])));
      printf("\n");
   }
}

/*--------------------------------------------------------------------------------
   Frame by frame, its share of the room's five target images (user, 2026-09-28): four walls and the
   floor, each on the canvas of its final orthophoto (4 mm per pixel; walls: column along the wall
   seen from inside, row from the ceiling down; floor: the canvas P, N up (mosaicFloorCanvas),
   cut to the polygon), sampled with the refined pose, raw colors (no gains). A face the
   frame covers at least cMinCoverage of becomes comp_NNN_<face>.bmp (with --face-names: <frame><name>.bmp,
   e.g. 8N.bmp) on the WHOLE canvas, black where unseen, so the components stack as they are; comp.csv
   tells what each covers (bounding box on the canvas) and whether the frame is XYZ (merged) or
   partial (placed, waiting for the subpixel matching).
  --------------------------------------------------------------------------------*/
static void mosaicComponents(const TMosaicFrame *frames, int count, const TMosaicWall *walls, const TLayoutPlan &plan,
                             const TVec3 &ud, const TVec3 &wd, const int *corner, LPCSTR outDir)
{
   TFloorCanvas fc;
   char         path[mosaicPathMax];

   mosaicFloorCanvas(plan, fc);

   int floorCols = fc.cols,
       floorRows = fc.rows,
       maxPixels = floorCols*floorRows,
       written = 0;

   for (int i = 0; i < plan.vertexCount; i++)
      maxPixels = walls[i].cols*walls[i].rows > maxPixels ? walls[i].cols*walls[i].rows : maxPixels;

   TAlloc<float> canvas((size_t)maxPixels*4u);

   snprintf(path, sizeof(path), "%s/comp.csv", outDir);

   FILE *csv = fopen(path, "wb");

   if (!csv)
      return;
   fprintf(csv, "frame,station,class,face,col0,row0,cols,rows,coveredPx,share,file\r\n");
   for (int f = 0; f < count; f++)
   {
      const TMosaicFrame &fr = frames[f];
      char                line[256];
      size_t              used = 0u;

      line[0] = '\0';
      for (int face = 0; face <= plan.vertexCount; face++)
      {
         bool onFloor = face == plan.vertexCount;
         int  cols = onFloor ? floorCols : walls[face].cols,
              rows = onFloor ? floorRows : walls[face].rows,
              c0 = cols,
              c1 = -1,
              r0 = rows,
              r1 = -1,
              covered = 0,
              n = plan.vertexCount,
              k = corner[f];

         if (cols <= 0 || rows <= 0 || (k >= 0 && !onFloor && face != k && face != (k + n - 1)%n)) // a corner: its two walls
            continue;
         memset(canvas(), 0, sizeof(float)*(size_t)cols*rows*4u);
         for (int r = 0; r < rows; r++)
            for (int c = 0; c < cols; c++)
            {
               TVec3 p;

               if (onFloor)
               {
                  float pu,
                        pw;

                  mosaicFloorAt(fc, c, r, pu, pw);
                  if (!mosaicInside(plan, pu, pw))
                     continue;
                  p.x = pu*ud.x + pw*wd.x;
                  p.y = -plan.cameraHeightM;
                  p.z = pu*ud.z + pw*wd.z;
               }
               else
                  p = mosaicPoint(walls[face], ud, wd, plan.ceilingM, plan.cameraHeightM, (float)c, (float)r);

               float  bgr[3],
                      s = mosaicSample(fr, p, bgr),
                     *o = canvas() + ((size_t)r*cols + c)*4u;

               if (s <= 0.f)
                  continue;
               o[0] = bgr[0];
               o[1] = bgr[1];
               o[2] = bgr[2];
               o[3] = 1.f;
               covered++;
               c0 = c < c0 ? c : c0;
               c1 = c > c1 ? c : c1;
               r0 = r < r0 ? r : r0;
               r1 = r > r1 ? r : r1;
            }

         float share = (float)covered/(float)(cols*rows);

         if (share < cMinCoverage)
            continue;

         int  cw = c1 - c0 + 1,
              ch = r1 - r0 + 1;
         char faceName[16],
              file[64];

         if (mosaicFaceNames[face][0])
            snprintf(faceName, sizeof(faceName), "%s", mosaicFaceNames[face]);
         else if (onFloor)
            snprintf(faceName, sizeof(faceName), "piso");
         else
            snprintf(faceName, sizeof(faceName), "parede%d", face);
         if (mosaicFaceNames[face][0])
            snprintf(file, sizeof(file), "%d%s.bmp", fr.index, faceName); // the user's naming: 8N, 8L...
         else
            snprintf(file, sizeof(file), "comp_%03d_%s.bmp", fr.index, faceName);
         snprintf(path, sizeof(path), "%s/%s", outDir, file);
         mosaicWriteBMP(path, canvas(), cols, rows); // the whole target canvas: the components stack as they are
         fprintf(csv, "%d,%d,%s,%s,%d,%d,%d,%d,%d,%.4f,%s\r\n", fr.index, fr.station, fr.xyz ? "xyz" : "partial", faceName, c0,
                 r0, cw, ch, covered, share, file);
         written++;
         used += (size_t)snprintf(line + used, sizeof(line) - used, " %s %.0f%%", faceName, 100.f*share);
         if (used >= sizeof(line))
            used = sizeof(line) - 1u;
      }
      printf("  components frame %03d station %d (%s):%s\n", fr.index, fr.station, fr.xyz ? "xyz" : "partial",
             line[0] ? line : " none");
   }
   fclose(csv);
   printf("  components: %d crops from %d frames (comp.csv)\n", written, count);
}

/*--------------------------------------------------------------------------------
   The preliminary merge (user, 2026-09-28): only the XYZ corner frames, and from each only its three
   trusted images - the floor and the two walls that meet at the corner it sees (vertex k ends wall
   k-1 and starts wall k). Walls weigh the picture falloff times the cosine of the angle of attack,
   the floor times sin^3 of the depression; raw colors, no gains. prelim_parede_K_<length>m.bmp and
   prelim_piso_<U>x<W>m.bmp, with how much of each canvas the corners already fill.
  --------------------------------------------------------------------------------*/
static void mosaicCornerMerge(const TMosaicFrame *frames, int count, const int *corner, const TMosaicWall *walls,
                              const TLayoutPlan &plan, const TVec3 &ud, const TVec3 &wd, LPCSTR outDir)
{
   int          n = plan.vertexCount;
   float        h = plan.cameraHeightM;
   char         path[mosaicPathMax];
   TFloorCanvas fc;

   mosaicFloorCanvas(plan, fc);
   for (int face = 0; face <= n; face++)
   {
      bool          onFloor = face == n;
      int           cols = onFloor ? fc.cols : walls[face].cols,
                    rows = onFloor ? fc.rows : walls[face].rows,
                    used = 0,
                    filled = 0,
                    inside = 0;
      TAlloc<float> acc((size_t)(cols > 0 ? cols : 1)*(rows > 0 ? rows : 1)*4u);

      if (cols <= 0 || rows <= 0)
         continue;
      memset(acc(), 0, sizeof(float)*(size_t)cols*rows*4u);
      for (int f = 0; f < count; f++)
      {
         int k = corner[f];

         if (k < 0 || (!onFloor && face != k && face != (k + n - 1)%n))
            continue;
         used++;
         for (int r = 0; r < rows; r++)
            for (int c = 0; c < cols; c++)
            {
               TVec3 p;

               if (onFloor)
               {
                  float pu,
                        pw;

                  mosaicFloorAt(fc, c, r, pu, pw);
                  if (!mosaicInside(plan, pu, pw))
                     continue;
                  p.x = pu*ud.x + pw*wd.x;
                  p.y = -h;
                  p.z = pu*ud.z + pw*wd.z;
               }
               else
                  p = mosaicPoint(walls[face], ud, wd, plan.ceilingM, h, (float)c, (float)r);

               float  bgr[3],
                      s = mosaicSample(frames[f], p, bgr),
                     *o = acc() + ((size_t)r*cols + c)*4u;

               if (s <= 0.f)
                  continue;

               float dx = p.x - frames[f].camX,
                     dz = p.z - frames[f].camZ,
                     dist = sqrtf(dx*dx + p.y*p.y + dz*dz + 1e-9f);

               if (onFloor)
                  s *= (h/dist)*(h/dist)*(h/dist);
               else
               {
                  TVec3 nrm = walls[face].constU ? ud : wd;

                  s *= fabsf(dx*nrm.x + dz*nrm.z)/dist;
               }
               o[0] += s*bgr[0];
               o[1] += s*bgr[1];
               o[2] += s*bgr[2];
               o[3] += s;
            }
      }
      for (int r = 0; r < rows; r++)
         for (int c = 0; c < cols; c++)
         {
            float pu = 0.f,
                  pw = 0.f;

            if (onFloor)
               mosaicFloorAt(fc, c, r, pu, pw);

            bool in = !onFloor || mosaicInside(plan, pu, pw);

            inside += in ? 1 : 0;
            filled += in && acc[((size_t)r*cols + c)*4u + 3u] > 0.f ? 1 : 0;
         }
      if (onFloor)
         snprintf(path, sizeof(path), "%s/prelim_piso_%.2fx%.2fm.bmp", outDir, (float)rows/(float)mosaicPxPerM,
                  (float)cols/(float)mosaicPxPerM);
      else
         snprintf(path, sizeof(path), "%s/prelim_parede_%d_%.2fm.bmp", outDir, face, (float)cols/(float)mosaicPxPerM);
      mosaicWriteBMP(path, acc(), cols, rows);
      printf("  preliminary %s%d: %d corner frames, %.0f%% filled\n", onFloor ? "floor" : "wall ", onFloor ? 0 : face,
             used, inside ? 100.f*(float)filled/(float)inside : 0.f);
   }
}

//--------------------------------------------------------------------------------
void mosaicWalls(TMosaicFrame *frames, int count, int centers, const TLayoutPlan &given, int rounds,
                 int bundleRounds, int jointRounds, float spinRadiusM, bool fixPlan, LPCSTR outDir)
{
   TLayoutPlan plan = given; // the bundle adjustment refines the plan too
   float       a = plan.axisDeg*0.01745329f;
   TVec3       ud = { sinf(a), 0.f, -cosf(a) },
               wd = { cosf(a), 0.f, sinf(a) };
   TMosaicWall walls[layoutMaxVerts];

   mosaicPlanFixed = fixPlan;
   mosaicOutDir = outDir;
   mosaicWallGeometry(plan, ud, wd, walls);
   for (int it = 0; it < rounds; it++) // each frame against the mosaic of the others
   {
      TAlloc<TVec3> update((size_t)count);
      TAlloc<float> updateW((size_t)count);
      float         sumShift = 0.f;
      int           shifts = 0;

      memset(update(), 0, sizeof(TVec3)*(size_t)count);
      memset(updateW(), 0, sizeof(float)*(size_t)count);
      for (int i = 0; i < plan.vertexCount; i++)
         mosaicRefineWall(frames, centers, walls[i], ud, wd, plan, update(), updateW(), &sumShift, &shifts);
      for (int f = 0; f < centers; f++)
         if (updateW[f] > 0.f)
         {
            frames[f].delta.x += cMosaicGain*update[f].x/updateW[f];
            frames[f].delta.y += cMosaicGain*update[f].y/updateW[f];
            frames[f].delta.z += cMosaicGain*update[f].z/updateW[f];
         }
      printf("  mosaic round %d: %d frame-wall matches, mean shift %.2f px (%.1f mm)\n", it + 1, shifts,
             shifts ? sumShift/(float)shifts : 0.f, shifts ? 1000.f*sumShift/(float)shifts/(float)mosaicPxPerM : 0.f);
   }
   if (bundleRounds > 0)
   {
      mosaicBundle(frames, count, plan, spinRadiusM, bundleRounds);
      mosaicWallGeometry(plan, ud, wd, walls);
   }
   /* optional: all frames together, anchored on the ceiling. Off by default - anchoring every crease on the
      APPROXIMATE plan fights the pairs (202726: molding pushed out, window doubled); the geometry must be
      adjusted together with the rotations (bundle adjustment) before the ceiling can be a hard anchor */
   for (int it = 0; it < jointRounds; it++)
      mosaicJoint(frames, centers, walls, plan.vertexCount, ud, wd, plan, it + 1);

   TAlloc<float> gains((size_t)plan.vertexCount*count*3u);
   TAlloc<int>   corner((size_t)count);
   int           merged = 0;

   /* the merge takes the XYZ frames only: both room axes measured AND a room corner in the picture, seen with the
      refined pose. The XZ / YZ ones (a stretch of one wall, the floor alone) are placed but wait for the subpixel
      matching on furniture and decoration */
   for (int f = 0; f < count; f++)
   {
      corner[f] = frames[f].xyz ? mosaicSeenCorner(frames[f], plan, ud, wd, 3) : -1;
      if (frames[f].picked) // named by the user: the corner most in view, else the one it aims at
      {
         corner[f] = mosaicSeenCorner(frames[f], plan, ud, wd, 1);
         if (corner[f] < 0)
            corner[f] = mosaicAimedCorner(frames[f], plan, ud, wd);
      }
      frames[f].xyz = corner[f] >= 0;
      if (corner[f] >= 0)
         printf("  corner frame %03d station %d: corner %d, walls %d and %d%s\n", frames[f].index, frames[f].station,
                corner[f], (corner[f] + plan.vertexCount - 1)%plan.vertexCount, corner[f],
                frames[f].picked ? " (named)" : "");
      merged += frames[f].xyz ? 1 : 0;
   }
   printf("  merge: %d of %d frames are XYZ (a room corner in view)\n", merged, count);
   for (int f = 0; f < count; f++) // each corner frame fixed by its own three axes before it gives its components
      if (corner[f] >= 0)
         mosaicCornerFit(frames[f], corner[f], walls, plan, ud, wd);
   mosaicComponents(frames, count, walls, plan, ud, wd, corner(), outDir);
   mosaicCornerMerge(frames, count, corner(), walls, plan, ud, wd, outDir);
   mosaicElect(frames, count, walls, plan.vertexCount, ud, wd, plan, gains());

   // final orthophotos with the refined rotations
   for (int i = 0; i < plan.vertexCount; i++)
   {
      const TMosaicWall &wl = walls[i];
      size_t             pixels = (size_t)wl.cols*wl.rows;
      TAlloc<float>      acc(pixels*4u + 1u);
      char               path[mosaicPathMax];

      if (wl.cols <= 0)
         continue;
      memset(acc(), 0, sizeof(float)*(pixels*4u + 1u));
      for (int f = 0; f < count; f++)
         for (int r = 0; r < wl.rows; r++)
            for (int c = 0; c < wl.cols; c++)
            {
               TVec3  p = mosaicPoint(wl, ud, wd, plan.ceilingM, plan.cameraHeightM, (float)c, (float)r);
               float  bgr[3],
                      s = frames[f].xyz ? mosaicSample(frames[f], p, bgr) : 0.f,
                     *o = acc() + ((size_t)r*wl.cols + c)*4u;

               if (s <= 0.f)
                  continue;

               // walls have an even luminance: the more head-on the view (cosine of the angle of attack), the truer
               TVec3 n = wl.constU ? ud : wd;
               float dx = p.x - frames[f].camX,
                     dz = p.z - frames[f].camZ,
                     cs = fabsf(dx*n.x + dz*n.z)/sqrtf(dx*dx + p.y*p.y + dz*dz + 1e-9f);

               s *= cs;
               s *= frames[f].station > 0 ? cCornerFill : 1.f; // corner views fill the holes, the spin point decides

               const float *gain = gains() + ((size_t)i*count + f)*3u;

               o[0] += s*gain[0]*bgr[0];
               o[1] += s*gain[1]*bgr[1];
               o[2] += s*gain[2]*bgr[2];
               o[3] += s;
            }
      snprintf(path, sizeof(path), "%s/parede_%d_%.2fm.bmp", outDir, i, (float)wl.cols/(float)mosaicPxPerM);
      mosaicWriteBMP(path, acc(), wl.cols, wl.rows);
   }
   mosaicFloor(frames, count, plan, ud, wd, outDir);
}
