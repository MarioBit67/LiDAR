#include "capDoor.h"
#include "alloc.h"
#include "libDiscipline.h"

enum {
   doorRunGap   = 24  // rows a jamb edge may miss (a hinge, a switch plate, a stretch where both sides match) and run on
};

static const float cViewFocal = 0.3f,     // view focal over the frame's: the whole frame fits, a door keeps ~150 px
                   cEdgeMin = 48.f,       // Sobel magnitude of an edge on the view luma
                   cJambMinPx = 150.f,    // a jamb run spans at least this (a door 7 m away stands ~265 px tall)
                   cAspectMin = 0.25f,    // opening width over height: 0.6-0.9 m over ~2.1 m, casing included
                   cAspectMax = 0.55f,
                   cHeadMinFrac = 0.5f,   // columns between the jambs that see the head edge
                   cOverrunMax = 0.4f,    // columns beside the casing where the head edge runs on (a divider)
                   cRunOnMax = 0.3f,      // rows above the head where a jamb's edge runs on (a panel gap)
                   cAboveEdgesMax = 0.10f, // edge pixels on the wall above the head (124744: a plain wall reads 0.07)
                   cColorGapMax = 10.f,   // wall above vs wall beside the door
                   cKnobRowLo = 0.40f,    // knob band, in opening heights above the floor (~1 m of 2.1)
                   cKnobRowHi = 0.52f,
                   cCameraMinHeads = 0.55f, // camera height in opening heights: ~1.15-2.0 m at 2.10 (a phone held up)
                   cCameraMaxHeads = 0.95f,
                   cCutFootMinDeg = 30.f,   // a door foot cut by the photo's edge is accepted when the photo reaches this far down
                   cDoorAspectInv = 2.6f,   // height over width of a door opening (2.10 over ~0.8)
                   cCreaseClearHeads = 0.12f, // the ceiling line stands this far above the head at least (the casing top: ~0.03)
                   cCasingNearHeads = 0.012f, // the casing top stands this far above the head (in openings)...
                   cCasingFarHeads = 0.12f,   // ...up to this far (a 3-10 cm casing over 2.10; a cut door's height is estimated)
                   cCreaseMinHeads = 1.1f,  // ceiling line in opening heights: ~2.3-3.8 m
                   cCreaseMaxHeads = 1.8f;

//--------------------------------------------------------------------------------
void doorFrontal(const TYUVImage &img, const TIntrinsics &k, const TVec3 &upCam, const TVec3 &nCam, TDoorView &view)
{
   const int   ow = doorViewW,
               oh = doorViewH;
   const float f = cViewFocal*k.fx;

   // virtual camera: looks along n (into the wall), y = true up, x = y cross z
   TVec3 up = upCam,
         zv = { -nCam.x, -nCam.y, -nCam.z },
         xv = { up.y*zv.z - up.z*zv.y, up.z*zv.x - up.x*zv.z, up.x*zv.y - up.y*zv.x };
   float elev = asinf(fmaxf(-0.9f, fminf(0.9f, -up.z))), // the real camera looks down -z
         yaw = atan2f(-xv.z, zv.z),
         cxv = 0.5f*(float)ow - f*tanf(fmaxf(-0.9f, fminf(0.9f, yaw))),
         cyv = 0.5f*(float)oh + f*tanf(elev);

   view.focalPx = f;
   view.cx = cxv;
   view.horizonRow = cyv;
   for (int vv = 0; vv < oh; vv++)
      for (int uu = 0; uu < ow; uu++)
      {
         size_t o = (size_t)vv*ow + uu;
         float  rx = ((float)uu - cxv)/f,
                ry = -((float)vv - cyv)/f,
                cx = xv.x*rx + up.x*ry - zv.x,
                cy = xv.y*rx + up.y*ry - zv.y,
                cz = xv.z*rx + up.z*ry - zv.z;

         view.y[o] = 0u;
         view.u[o] = 128u;
         view.v[o] = 128u;
         view.valid[o] = 0u;
         if (cz > -1e-6f)
            continue;

         float su = k.cx + k.fx*cx/-cz,
               sv = k.cy - k.fy*cy/-cz;
         int   iu = (int)floorf(su),
               iv = (int)floorf(sv);

         if (iu < 0 || iv < 0 || iu + 1 >= img.width || iv + 1 >= img.height)
            continue;

         float   du = su - (float)iu,
                 dv = sv - (float)iv;
         LPCBYTE p = img.y + (size_t)iv*img.yStride + iu;
         float   top = (float)p[0]*(1.f - du) + (float)p[1]*du,
                 bot = (float)p[img.yStride]*(1.f - du) + (float)p[img.yStride + 1]*du;
         size_t  c = (size_t)(iv/2)*img.uvRowStride + (size_t)(iu/2)*img.uvPixelStride;

         view.y[o] = (BYTE)(top*(1.f - dv) + bot*dv + 0.5f);
         view.u[o] = img.u[c];
         view.v[o] = img.v[c];
         view.valid[o] = 1u;
      }
}

//--------------------------------------------------------------------------------
bool doorFrameWall(const TVanishResult &vr, TVec3 &upCam, TVec3 &nCam)
{
   TVec3 up = vr.dirCam[0],
         a = vr.dirCam[1];

   if (!(vr.flags & (vfAxisA | vfAxisB)))
      return false;
   if (!(vr.flags & vfAxisA)) // only B measured: A is B turned about the vertical
   {
      const TVec3 &bm = vr.dirCam[2];

      a.x = bm.y*up.z - bm.z*up.y;
      a.y = bm.z*up.x - bm.x*up.z;
      a.z = bm.x*up.y - bm.y*up.x;
   }

   float d = a.x*up.x + a.y*up.y + a.z*up.z;

   a.x -= d*up.x;
   a.y -= d*up.y;
   a.z -= d*up.z;

   float len = sqrtf(a.x*a.x + a.y*a.y + a.z*a.z);

   if (!(len > 1e-6f)) // also NaN
      return false;
   a.x /= len;
   a.y /= len;
   a.z /= len;

   TVec3 b = { up.y*a.z - up.z*a.y, up.z*a.x - up.x*a.z, up.x*a.y - up.y*a.x };
   float fa = -a.z, // camera forward (0, 0, -1) along each axis
         fb = -b.z;
   bool  facingA = fabsf(fa) >= fabsf(fb),
         corner = atan2f(fminf(fabsf(fa), fabsf(fb)), fmaxf(fabsf(fa), fabsf(fb))) >= 25.f*0.01745329f;

   // aimed at a corner: the wall it shows more of - a wall holds the lines running along it (A-facing: B edges)
   if (corner && (vr.flags & vfAxisA) && (vr.flags & vfAxisB))
      facingA = vr.support[2] >= vr.support[1];

   TVec3 n = facingA ? a : b;
   float s = (facingA ? fa : fb) >= 0.f ? 1.f : -1.f;

   upCam = up;
   nCam.x = n.x*s;
   nCam.y = n.y*s;
   nCam.z = n.z*s;
   return true;
}

//--------------------------------------------------------------------------------
int doorFrameWalls(const TVanishResult &vr, TVec3 &upCam, TVec3 *nCam)
{
   if (!doorFrameWall(vr, upCam, nCam[0]))
      return 0;

   // the other wall: the dominant normal turned 90 degrees about the vertical, toward the camera's forward
   TVec3 n = nCam[0],
         o = { upCam.y*n.z - upCam.z*n.y, upCam.z*n.x - upCam.x*n.z, upCam.x*n.y - upCam.y*n.x };
   float along = -n.z, // camera forward (0, 0, -1) along each
         across = -o.z;

   if (atan2f(fabsf(across), fabsf(along)) < 25.f*0.01745329f)
      return 1; // facing one wall: the other is barely seen
   if (across < 0.f)
   {
      o.x = -o.x;
      o.y = -o.y;
      o.z = -o.z;
   }
   nCam[1] = o;
   return 2;
}

//--------------------------------------------------------------------------------
float doorColumnHeadingDeg(const TDoorView &view, const TVec3 &upCam, const TVec3 &nCam, const TMat4 &cameraToWorld,
                           float col)
{
   TVec3 zv = { -nCam.x, -nCam.y, -nCam.z },
         xv = { upCam.y*zv.z - upCam.z*zv.y, upCam.z*zv.x - upCam.x*zv.z, upCam.x*zv.y - upCam.y*zv.x };
   float rx = (col - view.cx)/view.focalPx;
   TVec3 ray = { xv.x*rx - zv.x, xv.y*rx - zv.y, xv.z*rx - zv.z };

   return geomHeadingDeg(cameraToWorld.RotateVector(ray));
}

//--------------------------------------------------------------------------------
struct TDoorJamb {
   int col,
       top,
       bottom;
};

//--------------------------------------------------------------------------------
static bool doorVertical(const int *gx, const int *gy, size_t o)
{
   int ax = gx[o] < 0 ? -gx[o] : gx[o],
       ay = gy[o] < 0 ? -gy[o] : gy[o];

   return (float)ax >= cEdgeMin && ax >= 2*ay;
}

//--------------------------------------------------------------------------------
static bool doorHorizontal(const int *gx, const int *gy, size_t o)
{
   int ax = gx[o] < 0 ? -gx[o] : gx[o],
       ay = gy[o] < 0 ? -gy[o] : gy[o];

   return (float)ay >= cEdgeMin && ay >= 2*ax;
}

//--------------------------------------------------------------------------------
// Share of the columns c0..c1 with a horizontal edge within one row of row r
static float doorRowFrac(const int *gx, const int *gy, int w, int h, int r, int c0, int c1)
{
   int hits = 0,
       n = 0;

   c0 = c0 < 1 ? 1 : c0;
   c1 = c1 > w - 2 ? w - 2 : c1;
   if (r < 2 || r > h - 3 || c1 < c0)
      return 0.f;
   for (int c = c0; c <= c1; c++)
   {
      bool hit = false;

      for (int d = -1; d <= 1; d++)
         hit = hit || doorHorizontal(gx, gy, (size_t)(r + d)*w + c);
      hits += hit ? 1 : 0;
      n++;
   }
   return (float)hits/(float)n;
}

//--------------------------------------------------------------------------------
// Share of the rows r0..r1 with a vertical edge within one column of column c
static float doorColFrac(const int *gx, const int *gy, int w, int h, int c, int r0, int r1)
{
   int hits = 0,
       n = 0;

   r0 = r0 < 1 ? 1 : r0;
   r1 = r1 > h - 2 ? h - 2 : r1;
   if (c < 2 || c > w - 3 || r1 < r0)
      return 0.f;
   for (int r = r0; r <= r1; r++)
   {
      bool hit = false;

      for (int d = -1; d <= 1; d++)
         hit = hit || doorVertical(gx, gy, (size_t)r*w + c + d);
      hits += hit ? 1 : 0;
      n++;
   }
   return (float)hits/(float)n;
}

//--------------------------------------------------------------------------------
// Mean Y, U, V of the smooth, valid pixels of a box; false when too few
static bool doorBoxColor(const TDoorView &view, const int *gx, const int *gy, int c0, int c1, int r0, int r1,
                         float *yuv, float *edgeFrac)
{
   const int w = doorViewW,
             h = doorViewH;
   float     sy = 0.f,
             su = 0.f,
             sv = 0.f;
   int       n = 0,
             all = 0,
             edges = 0;

   c0 = c0 < 1 ? 1 : c0;
   c1 = c1 > w - 2 ? w - 2 : c1;
   r0 = r0 < 1 ? 1 : r0;
   r1 = r1 > h - 2 ? h - 2 : r1;
   for (int r = r0; r <= r1; r++)
      for (int c = c0; c <= c1; c++)
      {
         size_t o = (size_t)r*w + c;
         int    m = (gx[o] < 0 ? -gx[o] : gx[o]) + (gy[o] < 0 ? -gy[o] : gy[o]);

         if (!view.valid[o])
            continue;
         all++;
         if ((float)m >= cEdgeMin)
         {
            edges++;
            continue;
         }
         sy += (float)view.y[o];
         su += (float)view.u[o];
         sv += (float)view.v[o];
         n++;
      }
   if (edgeFrac)
      *edgeFrac = all ? (float)edges/(float)all : 1.f;
   if (n < 20)
      return false;
   yuv[0] = sy/(float)n;
   yuv[1] = su/(float)n;
   yuv[2] = sv/(float)n;
   return true;
}

/*--------------------------------------------------------------------------------
   The knob: at ~1 m, just inside one jamb, a compact blob darker than the leaf around it (a closed
   door; an open leaf stands elsewhere and gives none - a tie-breaker, never a requirement).
  --------------------------------------------------------------------------------*/
static bool doorKnob(const TDoorView &view, int c0, int c1, int r0, int r1)
{
   const int w = doorViewW;
   float     mean = 0.f;
   int       n = 0,
             dark = 0,
             lo = c0 < c1 ? c0 : c1,
             hi = c0 < c1 ? c1 : c0;

   c0 = lo < 0 ? 0 : lo;
   c1 = hi > w - 1 ? w - 1 : hi;
   r0 = r0 < 0 ? 0 : r0;
   r1 = r1 > doorViewH - 1 ? doorViewH - 1 : r1;
   if (c1 <= c0 || r1 <= r0)
      return false;
   for (int r = r0; r <= r1; r++)
      for (int c = c0; c <= c1; c++)
         if (view.valid[(size_t)r*w + c])
         {
            mean += (float)view.y[(size_t)r*w + c];
            n++;
         }
   if (n < 30)
      return false;
   mean /= (float)n;
   for (int r = r0; r <= r1; r++)
      for (int c = c0; c <= c1; c++)
         if (view.valid[(size_t)r*w + c] && (float)view.y[(size_t)r*w + c] < mean - 30.f)
            dark++;
   return dark >= n/500 + 3 && dark <= n/12;
}

//--------------------------------------------------------------------------------
// Diagnosis: a jamb pair and the check that stopped it (0: it made a door)
static void doorTried(TDoorStats &st, int a, int b, int why, float value = 0.f)
{
   if (st.tried >= doorMaxTried)
      return;
   st.triedA[st.tried] = a;
   st.triedB[st.tried] = b;
   st.triedWhy[st.tried] = why;
   st.triedValue[st.tried] = value;
   st.tried++;
}

//--------------------------------------------------------------------------------
int doorDetect(const TDoorView &view, TDoor *doors, int cap, TDoorStats *stats)
{
   const int  w = doorViewW,
              h = doorViewH;
   TAlloc<int> gx((size_t)w*h),
               gy((size_t)w*h),
               colTop((size_t)w),
               colBot((size_t)w);
   TDoorJamb  jambs[doorMaxJambs];
   TDoorStats st = {};
   int        jambCount = 0,
              found = 0;

   memset(gx(), 0, sizeof(int)*(size_t)w*h);
   memset(gy(), 0, sizeof(int)*(size_t)w*h);
   for (int r = 1; r < h - 1; r++)
      for (int c = 1; c < w - 1; c++)
      {
         size_t  o = (size_t)r*w + c;
         LPCBYTE p = view.y + o;

         if (!view.valid[o - w - 1] || !view.valid[o - w] || !view.valid[o - w + 1] || !view.valid[o - 1] || !view.valid[o + 1]
             || !view.valid[o + w - 1] || !view.valid[o + w] || !view.valid[o + w + 1]) // the frame's own edge is no edge
            continue;
         gx[o] = (int)p[-w + 1] + 2*(int)p[1] + (int)p[w + 1] - (int)p[-w - 1] - 2*(int)p[-1] - (int)p[w - 1];
         gy[o] = (int)p[w - 1] + 2*(int)p[w] + (int)p[w + 1] - (int)p[-w - 1] - 2*(int)p[-w] - (int)p[-w + 1];
      }

   // the longest vertical run of each column (a jamb, a panel edge, a room corner)
   for (int c = 2; c < w - 2; c++)
   {
      int start = -1,
          last = -1,
          bestLen = 0;

      colTop[c] = 0;
      colBot[c] = 0;
      for (int r = 2; r < h - 2; r++)
      {
         bool hit = false;

         for (int d = -1; d <= 1; d++)
            hit = hit || doorVertical(gx(), gy(), (size_t)r*w + c + d);
         if (!hit)
            continue;
         if (start < 0 || r - last > doorRunGap)
            start = r;
         last = r;
         if (last - start > bestLen)
         {
            bestLen = last - start;
            colTop[c] = start;
            colBot[c] = last;
         }
      }
   }
   for (int c = 3; c < w - 3 && jambCount < doorMaxJambs; c++)
   {
      int len = colBot[c] - colTop[c];
      bool peak = true;

      if ((float)len < cJambMinPx)
         continue;
      for (int d = -3; d <= 3; d++)
         if (d && colBot[c + d] - colTop[c + d] > len)
            peak = false;
      if (!peak || (jambCount && c - jambs[jambCount - 1].col < 4))
         continue;
      jambs[jambCount].col = c;
      jambs[jambCount].top = colTop[c];
      jambs[jambCount].bottom = colBot[c];
      jambCount++;
   }

   st.jambs = jambCount;
   for (int i = 0; i < jambCount; i++)
   {
      st.jambCol[i] = jambs[i].col;
      st.jambTop[i] = jambs[i].top;
      st.jambBottom[i] = jambs[i].bottom;
   }

   // pairs of jambs that frame an opening
   for (int i = 0; i < jambCount; i++)
      for (int j = i + 1; j < jambCount; j++)
      {
         const TDoorJamb &a = jambs[i],
                         &b = jambs[j];
         /* the lower top is the head (a leaf swung open, closer, rises above it); the lower foot bounds the opening, as
            an edge may stop short where its background changes (083920 frames 29 and 30) */
         float            top = fmaxf((float)a.top, (float)b.top),
                          bot = fmaxf((float)a.bottom, (float)b.bottom),
                          oh = bot - top,
                          ow = (float)(b.col - a.col);

         st.pairs++;
         if (oh < 50.f || ow < 20.f || ow/oh < cAspectMin || ow/oh > cAspectMax)
         {
            st.shape++;
            doorTried(st, a.col, b.col, 1);
            continue;
         }
         if (fabsf((float)(a.top - b.top)) > 0.15f*oh || fabsf((float)(a.bottom - b.bottom)) > 0.45f*oh)
         {
            st.shape++;
            doorTried(st, a.col, b.col, 1);
            continue;
         }
         if (top >= view.horizonRow || bot <= view.horizonRow)
         {
            st.shape++;
            doorTried(st, a.col, b.col, 1);
            continue;
         }

         // the head: a horizontal edge across the opening, near the jambs' tops
         int   headRow = -1;
         float headFrac = 0.f;

         for (int r = (a.top < b.top ? a.top : b.top) - 12; r <= (int)top + 6; r++) // between the two tops: a leaf may rise past the head
         {
            float fr = doorRowFrac(gx(), gy(), w, h, r, a.col + 4, b.col - 4);

            if (fr > headFrac)
            {
               headFrac = fr;
               headRow = r;
            }
         }
         if (headFrac < cHeadMinFrac)
         {
            st.head++;
            doorTried(st, a.col, b.col, 4);
            continue;
         }

         /* the floor: the lower foot of a jamb that starts at the head. An edge may stop short where its background
            changes (the casing against a dark hall, then against its light floor: 083920 frame 30 stopped at 858 of
            971); the hinge edge of an open leaf lies on the wall and reaches the true floor */
         float floorRow = fminf((float)a.bottom, (float)b.bottom);

         for (int s = 0; s < 2; s++)
         {
            const TDoorJamb &jb = s == 0 ? a : b;

            if (fabsf((float)(jb.top - headRow)) <= 0.05f*oh && (float)jb.bottom > floorRow)
               floorRow = (float)jb.bottom;
         }

         /* a door shows its foot. Each jamb is judged in its own column, since the photo's bottom edge runs slanted
            across the frontal view (153013 frame 102: the near door's left jamb ended at the photo's edge, its right
            one below it): seen - the photo goes on below it; cut - it ends where the photo does. A tall window from
            the ceiling band has both cut with the photo barely below the horizon (124744 frame 1). A door with both
            feet cut is taken when the photo reaches well below the horizon (a door close by), as a candidate only */
         bool  footSeen = false,
               footCut = false,
               cutFar = false,
               anyCut = false;
         float seenFloor = 0.f,
               cutBottom = 0.f;

         for (int k = 0; k < 2; k++)
         {
            const TDoorJamb &jb = k == 0 ? a : b;
            int              lowest = -1;

            for (int r = h - 1; r >= jb.bottom && lowest < 0; r--)
               if (view.valid[(size_t)r*w + jb.col])
                  lowest = r;
            if (lowest < 0)
               continue;
            if (lowest - jb.bottom >= 12) // the photo goes on below this foot
            {
               footSeen = true;
               seenFloor = fmaxf(seenFloor, (float)jb.bottom);
            }
            else
            {
               anyCut = true;
               cutBottom = fmaxf(cutBottom, (float)jb.bottom);
               cutFar = cutFar || atan2f(view.horizonRow - (float)lowest, view.focalPx)*57.29578f <= -cCutFootMinDeg;
            }
         }

         /* a foot seen above a jamb cut lower by the photo's edge is something farther ending there - a door in the
            hall seen through the opening, a leaf's edge (153013 frame 102, 132058 frame 37) - not this door's floor */
         if (footSeen && anyCut && seenFloor < cutBottom - 12.f)
            footSeen = false;
         if (footSeen)
            floorRow = seenFloor; // the foot seen is the floor (a cut jamb only reaches the photo's edge)
         else if (cutFar)
            footCut = true;
         else
         {
            st.shape++;
            doorTried(st, a.col, b.col, 2);
            continue;
         }

         // from here the opening itself: head to floor (the runs may reach past both)
         float open = footCut ? fminf(floorRow - (float)headRow, cDoorAspectInv*ow) // cut: the height its width implies
                              : floorRow - (float)headRow,
               camera = (floorRow - view.horizonRow)/open;

         if (!footCut && (camera < cCameraMinHeads || camera > cCameraMaxHeads))
         {
            st.shape++;
            doorTried(st, a.col, b.col, 3);
            continue; // a camera held below knee height or above a head: not an opening at door scale
         }

         // the highest ceiling line over the door: the crease of the floor plan, molding included
         int creaseRow = -1;

         for (int r = 2; r < headRow - (int)(cCreaseClearHeads*open) && creaseRow < 0; r++) // clear of the casing top
            if (doorRowFrac(gx(), gy(), w, h, r, a.col - (int)(0.2f*ow), b.col + (int)(0.2f*ow)) >= 0.5f)
               creaseRow = r;
         if (!footCut && creaseRow >= 0 && ((floorRow - (float)creaseRow)/open < cCreaseMinHeads
                                || (floorRow - (float)creaseRow)/open > cCreaseMaxHeads))
         {
            st.shape++;
            doorTried(st, a.col, b.col, 3);
            continue; // a ceiling within a hand of the head, or twice its height: something else
         }

         /* an open leaf swung parallel to this wall looks just like a door in its frontal view (user, 2026-09-27:
            "deu positivo na folha aberta"). A door, open or shut, sits in its casing: the casing's top is a second
            line just above the head, across the whole opening; above a bare leaf there is only the wall */
         bool casing = false;
         int  casingFrom = headRow - (int)(cCasingFarHeads*open),
              casingTo = headRow - (int)(cCasingNearHeads*open);

         for (int r = casingFrom; r <= casingTo && !casing; r++)
            casing = doorRowFrac(gx(), gy(), w, h, r, a.col + 4, b.col - 4) >= cHeadMinFrac;
         if (!casing)
         {
            st.shape++;
            doorTried(st, a.col, b.col, 8);
            continue;
         }

         // a wardrobe's divider runs on beside the "casing"; a door head stops
         float overL = doorRowFrac(gx(), gy(), w, h, headRow, a.col - (int)(0.6f*ow), a.col - (int)(0.2f*ow)),
               overR = doorRowFrac(gx(), gy(), w, h, headRow, b.col + (int)(0.2f*ow), b.col + (int)(0.6f*ow)),
               overrun = fmaxf(overL, overR);

         /* a door's frame ends at the head; a wardrobe's panel gaps run on up to the ceiling (083920 frame 41). One
            jamb may run on: the leaf of an open door */
         float onA = doorColFrac(gx(), gy(), w, h, a.col, headRow - (int)(0.2f*open), headRow - (int)(0.04f*open)),
               onB = doorColFrac(gx(), gy(), w, h, b.col, headRow - (int)(0.2f*open), headRow - (int)(0.04f*open));

         if (overrun > cOverrunMax || fminf(onA, onB) > cRunOnMax)
         {
            st.overrun++;
            doorTried(st, a.col, b.col, 5);
            continue;
         }

         /* above the head: plain wall, the color of the wall beside the door - high enough to clear an open leaf's top,
            low enough to stay under the molding */
         int   r0 = headRow - (int)(0.35f*open),
               r1 = headRow - (int)(0.12f*open);

         if (creaseRow >= 0 && r0 < creaseRow + (int)(0.08f*open))
            r0 = creaseRow + (int)(0.08f*open);

         float above[3],
               left[3],
               right[3],
               aboveEdges = 1.f;
         bool  hasAbove = doorBoxColor(view, gx(), gy(), a.col, b.col, r0, r1, above, &aboveEdges),
               hasLeft = doorBoxColor(view, gx(), gy(), a.col - (int)(0.5f*ow), a.col - (int)(0.1f*ow), r0, r1, left, NULL),
               hasRight = doorBoxColor(view, gx(), gy(), b.col + (int)(0.1f*ow), b.col + (int)(0.5f*ow), r0, r1, right, NULL);

         if (!hasAbove || aboveEdges > cAboveEdgesMax || (!hasLeft && !hasRight))
         {
            st.above++;
            doorTried(st, a.col, b.col, 6, !hasAbove ? -1.f : ((!hasLeft && !hasRight) ? -2.f : aboveEdges)); // -1 no box, -2 no sides
            continue;
         }

         float gap = 1e9f;

         for (int s = 0; s < 2; s++)
         {
            const float *side = s == 0 ? left : right;

            if (s == 0 ? !hasLeft : !hasRight)
               continue;

            float du = above[1] - side[1],
                  dv = above[2] - side[2],
                  g = sqrtf(du*du + dv*dv) + 0.25f*fabsf(above[0] - side[0]);

            gap = fminf(gap, g);
         }
         if (gap > cColorGapMax)
         {
            st.color++;
            doorTried(st, a.col, b.col, 7, gap);
            continue;
         }

         /* tie-breakers: the knob, a room corner close by. The knob sits at ~1 m of its leaf: a closed leaf fills the
            opening, an open one stands across the wall and its edge is one of the "jambs" (user, 2026-09-27: "a
            maçaneta ficou na folha perpendicular") - so each jamb is searched on both sides, in the knob band of its
            own run (the open leaf is closer: taller and lower in the view than the opening) */
         bool knob = false,
              corner = false;

         for (int s = 0; s < 2; s++)
         {
            const TDoorJamb &jb = s == 0 ? a : b;
            float            run = (float)(jb.bottom - jb.top);
            int              kr0 = (int)((float)jb.bottom - cKnobRowHi*run),
                             kr1 = (int)((float)jb.bottom - cKnobRowLo*run);

            int in = s == 0 ? 1 : -1; // toward the opening; outward, an open leaf reaches a whole width (knob at its free edge)

            knob = knob || doorKnob(view, jb.col + in*(int)(0.05f*ow), jb.col + in*(int)(0.35f*ow), kr0, kr1)
                   || doorKnob(view, jb.col - in*(int)(0.05f*ow), jb.col - in*(int)(1.0f*ow), kr0, kr1);
         }

         for (int c = a.col - (int)(0.6f*ow); c <= b.col + (int)(0.6f*ow); c++)
         {
            if (c < 2 || c >= w - 2 || (c > a.col - (int)(0.02f*ow) && c < b.col + (int)(0.02f*ow)))
               continue;
            if ((float)(colBot[c] - colTop[c]) >= 1.15f*open && (float)colTop[c] < (float)headRow - 0.1f*open)
               corner = true;
         }

         TDoor d = {};

         d.leftCol = (float)a.col;
         d.rightCol = (float)b.col;
         d.headRow = (float)headRow;
         d.floorRow = floorRow;
         d.creaseRow = creaseRow >= 0 ? (float)creaseRow : NAN;
         d.creaseOverHead = creaseRow >= 0 && !footCut ? (floorRow - (float)creaseRow)/(floorRow - (float)headRow) : NAN;
         d.cameraOverHead = footCut ? NAN : camera;
         d.colorGap = gap;
         d.knob = knob;
         d.footCut = footCut;
         d.nearCorner = corner;
         d.score = headFrac + (1.f - overrun) + (cAboveEdgesMax - aboveEdges)*10.f + (cColorGapMax - gap)/cColorGapMax
                   + (knob ? 0.3f : 0.f) + (corner ? 0.3f : 0.f);

         doorTried(st, a.col, b.col, 0);

         // keep the best of overlapping openings (outer and inner casing edges frame the same door)
         int slot = -1;

         for (int k = 0; k < found; k++)
            if (fminf(d.rightCol, doors[k].rightCol) - fmaxf(d.leftCol, doors[k].leftCol)
                > 0.5f*fminf(d.rightCol - d.leftCol, doors[k].rightCol - doors[k].leftCol))
               slot = k;
         if (slot >= 0)
         {
            if (d.score > doors[slot].score)
               doors[slot] = d;
            continue;
         }
         if (found < cap)
            doors[found++] = d;
         else
         {
            int worst = 0;

            for (int k = 1; k < found; k++)
               if (doors[k].score < doors[worst].score)
                  worst = k;
            if (d.score > doors[worst].score)
               doors[worst] = d;
         }
      }

   if (stats)
      *stats = st;

   // best first
   for (int i = 1; i < found; i++)
   {
      TDoor d = doors[i];
      int   k = i - 1;

      while (k >= 0 && doors[k].score < d.score)
      {
         doors[k + 1] = doors[k];
         k--;
      }
      doors[k + 1] = d;
   }
   return found;
}
