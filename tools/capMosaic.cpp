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
                         int c, int r)
{
   float s = wl.s0 + wl.sdir*((float)c + 0.5f)/(float)mosaicPxPerM,
         z = ceilingM - ((float)r + 0.5f)/(float)mosaicPxPerM - heightM,
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
                   s = mosaicSample(frames[f], mosaicPoint(wl, ud, wd, plan.ceilingM, plan.cameraHeightM, c, r), bgr);

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
                      s = mosaicSample(frames[f], mosaicPoint(wl, ud, wd, plan.ceilingM, plan.cameraHeightM, c, r), bgr);

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
         for (int p = 0; p < np; p++)
            moved[p] = theta[p] + step[p];
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
            TVec3 p = mosaicPoint(wl, ud, wd, plan.ceilingM, plan.cameraHeightM, c, r);
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

//--------------------------------------------------------------------------------
void mosaicWalls(TMosaicFrame *frames, int count, int centers, const TLayoutPlan &given, int rounds,
                 int bundleRounds, int jointRounds, float spinRadiusM, LPCSTR outDir)
{
   TLayoutPlan plan = given; // the bundle adjustment refines the plan too
   float       a = plan.axisDeg*0.01745329f;
   TVec3       ud = { sinf(a), 0.f, -cosf(a) },
               wd = { cosf(a), 0.f, sinf(a) };
   TMosaicWall walls[layoutMaxVerts];

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
               TVec3  p = mosaicPoint(wl, ud, wd, plan.ceilingM, plan.cameraHeightM, c, r);
               float  bgr[3],
                      s = mosaicSample(frames[f], p, bgr),
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
}
