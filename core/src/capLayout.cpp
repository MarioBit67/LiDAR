#include "capLayout.h"
#include "libDiscipline.h"

static const float cDegToRad = 0.01745329252f,
                   cRadToDeg = 57.29577951f,
                   cElevBinDeg = 0.2f,       // line Hough: perpendicular elevation bin
                   cCreaseFrac = 0.15f,      // crease = the HIGHEST line with this share of its frame's longest (beams, cabinets lie below)
                   cWallFrac = 0.2f,         // a second wall on a side needs this share of the strongest
                   cMinRayY = 0.05f,         // rays this close to the horizon are ill-conditioned
                   cMinAcross = 0.17f,       // sin 10 degrees between the ray and the line it lies on
                   cMinHeightM = 0.9f,       // camera height search range
                   cMaxHeightM = 1.9f,
                   cHeightStepM = 0.02f,
                   cDefaultHeightM = 1.45f,  // chest height when the floor creases stay hidden
                   cMaxAxisDevDeg = 20.f,    // a line farther than this from both room axes is ignored
                   cMinWallM = 0.3f,
                   cMaxFrameJumpDeg = 5.f,   // a keyframe axis farther than this from the recent median is an error, not drift
                   cMinCeilPitchDeg = 5.f,   // frames aimed at the ceiling band give creases at any elevation
                   cCreaseViewDeg = 25.f,    // level frames: up to this above their aim (half the view less a margin)
                   cMaxAimDeg = 35.f,        // corner station aim: at most this off the corner bisector (the diagonal)
                   cSecondAimDeg = 25.f,     // a second aim from the same corner: at least this apart from the first
                   cMinAreaM2 = 1.f,
                   cDoorHeadM = 2.10f,       // door and window heads: the most reliable height of a building
                   cMinHeadM = 1.6f,
                   cHeadBinM = 0.01f,
                   cMoldingM = 0.15f,        // crown molding band below the ceiling crease
                   cMinDoorScale = 0.8f,
                   cMaxDoorScale = 1.2f,
                   cCornerRatioTol = 0.03f;  // floor creases must meet below the ceiling corner: q ratios within 3%

enum {
   layoutMaxEdges   = 400000,  // 8000 per keyframe, 50 keyframes
   layoutFrameEdges = 8000,   // per keyframe (evenly thinned)
   layoutMinPairs   = 150,    // coinciding ceiling / floor votes needed to trust the solved height
   layoutHeadBins   = 110,    // 1.6..2.7 m, 1 cm
   layoutMinHeadPts = 30,
   layoutElevBins   = 450,    // 0..90 degrees
   layoutMinLinePts = 25,     // edge pixels of one line in one frame
   layoutMaxLines   = 8000,
   layoutMinCornerPairs = 3   // consistent floor-crease pairs needed to trust the corner height
};

// One wall: axis-aligned segment. kind 0: constant u (runs along w); kind 1: constant w (runs along u)
struct TPlanWall {
   int   kind;
   float offset,
         a0,
         a1,
         weight,
         midAngle;
};

//--------------------------------------------------------------------------------
TRoomLayout::TRoomLayout(void) : Pcount(0u), Pcap(0u), PframeStart(), PframePitch(), PframeCenter(), PframeAxis(),
   Pframes(0), PanchorDeg(NAN), Precent(), PrecentCount(0)
{
}

//--------------------------------------------------------------------------------
void TRoomLayout::Reset(void)
{
   Pcount = 0u;
   Pframes = 0;
   PanchorDeg = NAN;
   PrecentCount = 0;
}

//--------------------------------------------------------------------------------
static float layoutMedian(const float *v, int n)
{
   float s[layoutMaxFrames];

   n = n < layoutMaxFrames ? n : layoutMaxFrames;
   for (int i = 0; i < n; i++)
   {
      int at = i;

      while (at > 0 && s[at - 1] > v[i])
      {
         s[at] = s[at - 1];
         at--;
      }
      s[at] = v[i];
   }
   return n ? s[n/2] : 0.f;
}

/*--------------------------------------------------------------------------------
   Stores the horizontal edges of one center-spin keyframe DE-DRIFTED: the frame's own room axis
   (vanishing points) is unwrapped against the previous frame and the whole frame is turned about
   the vertical so that its axis lands on the anchor (first frame). Gyroscope drift between the
   ceiling band and the floor band would otherwise pull their creases apart. Every frame is kept:
   which ones jump (diagonal floor tiles at 45 degrees, spurious axes) is decided by Solve against
   its neighbors on BOTH sides - a wrong first frame must not become the reference (2026-09-27: the
   L room lost its whole ceiling band to a first axis 5.8 degrees off).
  --------------------------------------------------------------------------------*/
void TRoomLayout::AddFrame(const TMat4 &cameraToWorld, const TVanishResult &r, const TVanishEdges &edges,
                           const TVec3 &tiltBias, bool centerSpin)
{
   if (isnan(r.roomAxisDeg) || Pframes >= layoutMaxFrames)
      return;
   if (isnan(PanchorDeg))
   {
      PanchorDeg = r.roomAxisDeg;
      Precent[0] = r.roomAxisDeg;
      PrecentCount = 1;
   }

   float ref = layoutMedian(Precent, PrecentCount),
         jump = vanishAxisDiffDeg(r.roomAxisDeg, ref),
         frameAxis = ref + jump; // this frame's own axis, unwrapped

   PframeAxis[Pframes] = frameAxis;

   if (PrecentCount == layoutDriftWindow)
   {
      for (int i = 1; i < layoutDriftWindow; i++)
         Precent[i - 1] = Precent[i];
      PrecentCount--;
   }
   Precent[PrecentCount++] = frameAxis;
   if (!Pcap)
   {
      TAlloc<float> data((size_t)layoutMaxEdges*4u);

      data.Drop(Pdata);
      Pcap = layoutMaxEdges;
   }

   // calibrated camera tilt (stable, unlike one frame's vertical): rays turn from the true vertical onto gravity
   TVec3 fix = { -tiltBias.x, -tiltBias.y, -tiltBias.z };
   float delta = PanchorDeg - frameAxis,
         cs = cosf(delta*cDegToRad),
         sn = sinf(delta*cDegToRad),
         heading[vanishDirs];

   for (int d = 1; d < vanishDirs; d++)
      heading[d] = fmodf(geomHeadingDeg(cameraToWorld.RotateVector(vanishRotate(r.dirCam[d], fix))) + delta + 360.f,
                         180.f);

   DWORD horizontal = 0u;

   for (DWORD e = 0; e < edges.count; e++)
      if (edges.label[e] >= 2u)
         horizontal++;

   DWORD every = horizontal > (DWORD)layoutFrameEdges ? (horizontal + layoutFrameEdges - 1u)/layoutFrameEdges : 1u,
         seen = 0u;
   float *out = Pdata();

   PframePitch[Pframes] = geomPitchDeg(cameraToWorld.Forward());
   PframeCenter[Pframes] = centerSpin;
   PframeStart[Pframes] = Pcount;
   for (DWORD e = 0; e < edges.count && Pcount < Pcap; e++)
   {
      int d = (int)edges.label[e] - 1;

      if (d < 1 || !(r.flags & (1 << d)) || seen++%every)
         continue;

      TVec3 rc = { edges.ray[3u*e], edges.ray[3u*e + 1u], edges.ray[3u*e + 2u] },
            rw = cameraToWorld.RotateVector(vanishRotate(rc, fix));

      out[4u*Pcount] = rw.x*cs - rw.z*sn; // heading turned by delta (clockwise positive)
      out[4u*Pcount + 1u] = rw.y;
      out[4u*Pcount + 2u] = rw.z*cs + rw.x*sn;
      out[4u*Pcount + 3u] = heading[d];
      Pcount++;
   }
   Pframes++;
   PframeStart[Pframes] = Pcount;
}

/*--------------------------------------------------------------------------------
   Plan quantities of one stored edge: kind of the wall it belongs to (0: wall of constant u,
   1: constant w), q = offset/ray height and p = along/ray height (scaled by the plane distance
   later), and whether it lies on the ceiling. False for unusable edges.
  --------------------------------------------------------------------------------*/
static bool layoutEdge(const float *e, float axisDeg, int &kind, float &q, float &p, bool &ceiling)
{
   float ry = e[1],
         diff = fmodf(e[3] - axisDeg + 360.f, 180.f),
         a = axisDeg*cDegToRad;

   if (fabsf(ry) < cMinRayY)
      return false;
   if (diff > 90.f)
      diff = 180.f - diff; // [0, 90]: 0 = the line runs along u
   if (diff <= cMaxAxisDevDeg)
      kind = 1;             // runs along u: the wall has constant w
   else if (diff >= 90.f - cMaxAxisDevDeg)
      kind = 0;
   else
      return false;

   float ru = e[0]*sinf(a) - e[2]*cosf(a),
         rw = e[0]*cosf(a) + e[2]*sinf(a),
         across = fabsf(kind == 0 ? ru : rw),
         flat = sqrtf(ru*ru + rw*rw);

   if (across < cMinAcross*flat)
      return false; // looking along the line (near its vanishing point): the offset is ill-conditioned
   q = (kind == 0 ? ru : rw)/ry;
   p = (kind == 0 ? rw : ru)/ry;
   ceiling = ry > 0.f;
   return true;
}

//--------------------------------------------------------------------------------
static float layoutSmooth(const float *hist, int i, int n)
{
   float s = hist[i];

   if (i > 0)
      s += hist[i - 1];
   if (i + 1 < n)
      s += hist[i + 1];
   return s;
}

/*--------------------------------------------------------------------------------
   Highest elevation at which a keyframe's topmost line can be taken for the crease. Aimed at the
   ceiling band: any. Aimed near the horizon: only well inside the view - a far wall's crease is,
   a near wall's crease may be out of it and its topmost line a door head (2026-09-27: the far
   walls of the L room were seen only from level frames). Corner stations never give walls.
  --------------------------------------------------------------------------------*/
static float layoutCreaseTop(float pitchDeg, bool center)
{
   if (!center)
      return -90.f;
   return pitchDeg >= cMinCeilPitchDeg ? 90.f : pitchDeg + cCreaseViewDeg;
}

// One horizontal line seen by one keyframe: a peak of that frame's elevation histogram
struct TPlanLine {
   int   frame,
         half;    // kind*2 + (offset < 0): the side of the room the line lies on
   bool  ceiling, // above the horizon
         top,     // the topmost long ceiling-side line of its frame and side (any frame: height pairs)
         crease;  // the same, in a center-spin frame aimed at the ceiling: a wall / ceiling crease for the walls
   float q,       // |offset| per unit of distance to its plane (1/tan of its perpendicular elevation)
         r0,      // along / offset range covered by its edges
         r1,
         weight;  // edge pixels
};

//--------------------------------------------------------------------------------
static int layoutHist(bool ceiling, int half)
{
   return (ceiling ? 0 : 4) + half;
}

//--------------------------------------------------------------------------------
// Histogram row and elevation bin of one stored edge; false for unusable edges
static bool layoutEdgeBin(const float *e, float axisDeg, int &row, int &bin, float &q, float &p)
{
   int  kind;
   bool onCeiling;

   if (!layoutEdge(e, axisDeg, kind, q, p, onCeiling))
      return false;

   int negative = (onCeiling ? q < 0.f : q > 0.f) ? 1 : 0;

   row = layoutHist(onCeiling, kind*2 + negative);
   bin = (int)(atanf(1.f/fabsf(q))*cRadToDeg/cElevBinDeg);
   return bin < layoutElevBins;
}

/*--------------------------------------------------------------------------------
   Lines of one keyframe: a 1D Hough per room side and plane. For a horizontal line running along a
   room axis, every edge pixel on it shares the perpendicular elevation atan(1/|q|), so each physical
   line (crease, door head, furniture top, tile joint) is one peak. The topmost long ceiling-side
   line of each side is the crease (fixtures are short, wall features lie below it) - trusted only
   up to creaseTopDeg, where a higher line could still have been seen.
  --------------------------------------------------------------------------------*/
static int layoutFrameLines(const float *data, DWORD first, DWORD last, float axisDeg, int frame, float creaseTopDeg,
                            TPlanLine *lines, int n, int cap)
{
   TAlloc<float> hist((size_t)8*layoutElevBins);
   int           start = n,
                 row,
                 bin;
   float         q,
                 p;

   memset(hist(), 0, sizeof(float)*8u*layoutElevBins);
   for (DWORD e = first; e < last; e++)
      if (layoutEdgeBin(data + 4u*e, axisDeg, row, bin, q, p))
         hist[(size_t)row*layoutElevBins + bin] += 1.f;
   for (int h = 0; h < 8 && n < cap; h++)
   {
      const float *hr = hist() + (size_t)h*layoutElevBins;

      for (int b = 1; b + 1 < layoutElevBins && n < cap; b++)
      {
         float v = layoutSmooth(hr, b, layoutElevBins);

         if (v < (float)layoutMinLinePts || v < layoutSmooth(hr, b - 1, layoutElevBins)
             || v <= layoutSmooth(hr, b + 1, layoutElevBins))
            continue;

         TPlanLine &line = lines[n];
         float      sumQ = 0.f;

         line.frame = frame;
         line.half = h%4;
         line.ceiling = h < 4;
         line.crease = false;
         line.top = false;
         line.r0 = 1e9f;
         line.r1 = -1e9f;
         line.weight = 0.f;
         for (DWORD e = first; e < last; e++)
         {
            if (!layoutEdgeBin(data + 4u*e, axisDeg, row, bin, q, p) || row != h || bin < b - 1 || bin > b + 1)
               continue;

            float ratio = p/q;

            sumQ += fabsf(q);
            line.weight += 1.f;
            line.r0 = ratio < line.r0 ? ratio : line.r0;
            line.r1 = ratio > line.r1 ? ratio : line.r1;
         }
         if (line.weight <= 0.f)
            continue;
         line.q = sumQ/line.weight;
         n++;
      }
   }
   for (int half = 0; half < 4; half++)
   {
      float maxW = 0.f;
      int   best = -1;

      for (int i = start; i < n; i++)
         if (lines[i].ceiling && lines[i].half == half && lines[i].weight > maxW)
            maxW = lines[i].weight;
      for (int i = start; i < n; i++)
         if (lines[i].ceiling && lines[i].half == half && lines[i].weight >= cCreaseFrac*maxW
             && (best < 0 || lines[i].q < lines[best].q))
            best = i;
      if (best >= 0)
      {
         lines[best].top = true;
         lines[best].crease = atanf(1.f/fabsf(lines[best].q))*cRadToDeg <= creaseTopDeg;
      }
   }
   return n;
}

/*--------------------------------------------------------------------------------
   Camera height from room corners seen whole in ONE image (center spin frames that catch two
   walls, corner-station frames aimed at the opposite corner). The two walls' ceiling creases fix
   the corner's azimuth: q_c1/q_c0 = D1/D0. The two floor creases must meet right below, at the same
   azimuth: q_f1/q_f0 = D1/D0 too - a furniture edge or a tile joint almost never does. Then
   k = q_f/q_c = (H - h)/h on both walls and h = H/(1 + k). Same image, same instant: no drift.
  --------------------------------------------------------------------------------*/
static float layoutPairHeight(const TPlanLine *lines, int n, float ceilingM, bool &solved, int &pairs)
{
   int           heightBins = (int)((cMaxHeightM - cMinHeightM)/cHeightStepM + 0.5f) + 1;
   TAlloc<float> votes((size_t)heightBins);
   TAlloc<int>   count((size_t)heightBins);

   memset(votes(), 0, sizeof(float)*(size_t)heightBins);
   memset(count(), 0, sizeof(int)*(size_t)heightBins);
   for (int fs = 0; fs < n;)
   {
      int fe = fs;

      while (fe < n && lines[fe].frame == lines[fs].frame)
         fe++;
      for (int s0 = 0; s0 < 2; s0++)
         for (int s1 = 2; s1 < 4; s1++)
         {
            int c0 = -1,
                c1 = -1;

            for (int i = fs; i < fe; i++)
               if (lines[i].top && lines[i].half == s0)
                  c0 = i;
               else if (lines[i].top && lines[i].half == s1)
                  c1 = i;
            if (c0 < 0 || c1 < 0)
               continue;

            float rc = lines[c1].q/lines[c0].q;

            for (int f0 = fs; f0 < fe; f0++)
            {
               if (lines[f0].ceiling || lines[f0].half != s0)
                  continue;
               for (int f1 = fs; f1 < fe; f1++)
               {
                  if (lines[f1].ceiling || lines[f1].half != s1)
                     continue;

                  float rf = lines[f1].q/lines[f0].q;

                  if (fabsf(rf/rc - 1.f) > cCornerRatioTol)
                     continue;

                  float k = 0.5f*(lines[f0].q/lines[c0].q + lines[f1].q/lines[c1].q),
                        h = ceilingM/(1.f + k),
                        w = lines[f0].weight < lines[f1].weight ? lines[f0].weight : lines[f1].weight;
                  int   hb = (int)((h - cMinHeightM)/cHeightStepM + 0.5f);

                  if (hb < 0 || hb >= heightBins)
                     continue;
                  votes[hb] += w;
                  count[hb]++;
               }
            }
         }
      fs = fe;
   }

   float bestH = cDefaultHeightM,
         bestVote = 0.f;
   int   bestBin = -1;

   for (int hb = 0; hb < heightBins; hb++)
   {
      float v = layoutSmooth(votes(), hb, heightBins);

      if (v > bestVote)
      {
         bestVote = v;
         bestBin = hb;
         bestH = cMinHeightM + (float)hb*cHeightStepM;
      }
   }
   pairs = 0;
   for (int hb = bestBin - 1; bestBin >= 0 && hb <= bestBin + 1; hb++)
      if (hb >= 0 && hb < heightBins)
         pairs += count[hb];
   solved = pairs >= layoutMinCornerPairs;
   return solved ? bestH : cDefaultHeightM;
}

/*--------------------------------------------------------------------------------
   Camera height by wall agreement, on lines: per side, the ceiling crease is the heaviest cluster of
   the frames' creases (|q| within 4%); every floor-side line of that side proposes
   h = H*q_c/(q_c + q_f). Each side votes at most its crease weight per height bin, so floor tiles and
   furniture tops (whose proposals differ from wall to wall) lose to the height all walls agree on.
  --------------------------------------------------------------------------------*/
static float layoutLineHeight(const TPlanLine *lines, int n, float ceilingM, bool &solved)
{
   int           heightBins = (int)((cMaxHeightM - cMinHeightM)/cHeightStepM + 0.5f) + 1;
   TAlloc<float> votes((size_t)heightBins),
                 side((size_t)heightBins);

   memset(votes(), 0, sizeof(float)*(size_t)heightBins);
   for (int half = 0; half < 4; half++)
   {
      float qc = 0.f,
            wc = 0.f;

      for (int i = 0; i < n; i++)
      {
         if (!lines[i].crease || lines[i].half != half)
            continue;

         float w = 0.f;

         for (int j = 0; j < n; j++)
            if (lines[j].crease && lines[j].half == half && fabsf(lines[j].q - lines[i].q) <= 0.04f*lines[i].q)
               w += lines[j].weight;
         if (w > wc)
         {
            wc = w;
            qc = lines[i].q;
         }
      }
      if (wc <= 0.f)
         continue;
      memset(side(), 0, sizeof(float)*(size_t)heightBins);
      for (int i = 0; i < n; i++)
      {
         if (lines[i].ceiling || lines[i].half != half)
            continue;

         float h = ceilingM*qc/(qc + lines[i].q),
               w = lines[i].weight < wc ? lines[i].weight : wc;
         int   hb = (int)((h - cMinHeightM)/cHeightStepM + 0.5f);

         if (hb >= 0 && hb < heightBins)
            side[hb] += w;
      }
      for (int hb = 0; hb < heightBins; hb++)
         votes[hb] += side[hb] < wc ? side[hb] : wc;
   }

   float bestH = cDefaultHeightM,
         bestVote = 0.f;

   for (int hb = 0; hb < heightBins; hb++)
   {
      float v = layoutSmooth(votes(), hb, heightBins);

      if (v > bestVote)
      {
         bestVote = v;
         bestH = cMinHeightM + (float)hb*cHeightStepM;
      }
   }
   solved = bestVote >= (float)layoutMinPairs;
   return solved ? bestH : cDefaultHeightM;
}

/*--------------------------------------------------------------------------------
   Walls from the creases: per side the creases are sorted by distance and clustered (gap above
   8 cm or 4%); a cluster is a wall whose along-range is the union of its creases. Weaker clusters
   survive only on along-ranges no stronger wall of that side claims (steps of an L).
  --------------------------------------------------------------------------------*/
static int layoutLineWalls(const TPlanLine *lines, int n, float ceilingM, float heightM, TPlanWall *walls, int cap)
{
   int found = 0;

   for (int half = 0; half < 4; half++)
   {
      float d[layoutMaxFrames],
            w[layoutMaxFrames],
            a0[layoutMaxFrames],
            a1[layoutMaxFrames];
      int   m = 0;
      float sign = (half & 1) ? -1.f : 1.f,
            scale = ceilingM - heightM;

      for (int i = 0; i < n && m < layoutMaxFrames; i++)
      {
         if (!lines[i].crease || lines[i].half != half)
            continue;

         float dist = lines[i].q*scale,
               lo = lines[i].r0*sign*dist,
               hi = lines[i].r1*sign*dist;
         int   at = m++;

         while (at > 0 && d[at - 1] > dist)
         {
            d[at] = d[at - 1];
            w[at] = w[at - 1];
            a0[at] = a0[at - 1];
            a1[at] = a1[at - 1];
            at--;
         }
         d[at] = dist;
         w[at] = lines[i].weight;
         a0[at] = lo < hi ? lo : hi;
         a1[at] = lo < hi ? hi : lo;
      }

      // chain the sorted creases into clusters
      TPlanWall cl[layoutMaxWalls];
      float     lastD = 0.f;
      int       k = 0;

      for (int i = 0; i < m; i++)
      {
         bool join = k > 0 && d[i] - lastD <= fmaxf(0.08f, 0.04f*d[i]);

         if (!join && k >= layoutMaxWalls)
            break;
         if (!join)
         {
            cl[k].kind = half/2;
            cl[k].offset = 0.f;
            cl[k].weight = 0.f;
            cl[k].a0 = a0[i];
            cl[k].a1 = a1[i];
            cl[k].midAngle = 0.f;
            k++;
         }

         TPlanWall &c = cl[k - 1];

         c.offset += d[i]*w[i];
         c.weight += w[i];
         c.a0 = a0[i] < c.a0 ? a0[i] : c.a0;
         c.a1 = a1[i] > c.a1 ? a1[i] : c.a1;
         lastD = d[i];
      }

      float strongest = 0.f;

      for (int i = 0; i < k; i++)
      {
         cl[i].offset = sign*cl[i].offset/cl[i].weight;
         strongest = cl[i].weight > strongest ? cl[i].weight : strongest;
      }

      // strongest first; a weaker wall only where no stronger wall of the same side is
      for (int pass = 0; pass < k && found < cap; pass++)
      {
         int best = -1;

         for (int i = 0; i < k; i++)
            if (cl[i].weight > 0.f && (best < 0 || cl[i].weight > cl[best].weight))
               best = i;
         if (best < 0 || cl[best].weight < cWallFrac*strongest)
            break;

         TPlanWall c = cl[best];
         bool      shadowed = c.a1 - c.a0 < cMinWallM;

         cl[best].weight = 0.f;
         for (int i = 0; i < found && !shadowed; i++)
         {
            float lo = walls[i].a0 > c.a0 ? walls[i].a0 : c.a0,
                  hi = walls[i].a1 < c.a1 ? walls[i].a1 : c.a1;

            if (walls[i].kind == c.kind && (walls[i].offset < 0.f) == (c.offset < 0.f) && hi - lo > 0.2f*(c.a1 - c.a0))
               shadowed = true;
         }
         if (!shadowed)
            walls[found++] = c;
      }
   }
   return found;
}

//--------------------------------------------------------------------------------
// Map coordinates of a plan point: x = w (right), y = u (up)
static float layoutCross(const TPlanPoint &a, const TPlanPoint &b, const TPlanPoint &c)
{
   return (b.w - a.w)*(c.u - b.u) - (b.u - a.u)*(c.w - b.w);
}

//--------------------------------------------------------------------------------
static bool layoutSegmentsCross(const TPlanPoint &p, const TPlanPoint &q, const TPlanPoint &a, const TPlanPoint &b)
{
   float d1 = layoutCross(a, b, p),
         d2 = layoutCross(a, b, q),
         d3 = layoutCross(p, q, a),
         d4 = layoutCross(p, q, b);

   return ((d1 > 1e-4f && d2 < -1e-4f) || (d1 < -1e-4f && d2 > 1e-4f))
          && ((d3 > 1e-4f && d4 < -1e-4f) || (d3 < -1e-4f && d4 > 1e-4f));
}

//--------------------------------------------------------------------------------
static bool layoutInside(const TLayoutPlan &plan, const TPlanPoint &p)
{
   bool in = false;

   for (int i = 0; i < plan.vertexCount; i++)
   {
      const TPlanPoint &a = plan.verts[i],
                       &b = plan.verts[(i + plan.vertexCount - 1)%plan.vertexCount];

      if ((a.u > p.u) != (b.u > p.u) && p.w < (b.w - a.w)*(p.u - a.u)/(b.u - a.u) + a.w)
         in = !in;
   }
   return in;
}

//--------------------------------------------------------------------------------
// Clear line of sight between two corners: crosses no wall and runs inside the room
static bool layoutSees(const TLayoutPlan &plan, int from, int to)
{
   const TPlanPoint &p = plan.verts[from],
                    &q = plan.verts[to];
   TPlanPoint        mid = { 0.5f*(p.u + q.u), 0.5f*(p.w + q.w) };

   for (int i = 0; i < plan.vertexCount; i++)
   {
      int j = (i + 1)%plan.vertexCount;

      if (i == from || j == from || i == to || j == to)
         continue;
      if (layoutSegmentsCross(p, q, plan.verts[i], plan.verts[j]))
         return false;
   }
   return layoutInside(plan, mid);
}

//--------------------------------------------------------------------------------
static void layoutAddVertex(TLayoutPlan &plan, float u, float w)
{
   if (plan.vertexCount > 0)
   {
      const TPlanPoint &last = plan.verts[plan.vertexCount - 1];

      if (fabsf(last.u - u) < 0.05f && fabsf(last.w - w) < 0.05f)
         return;
   }
   if (plan.vertexCount >= layoutMaxVerts)
      return;
   plan.verts[plan.vertexCount].u = u;
   plan.verts[plan.vertexCount].w = w;
   plan.vertexCount++;
}

/*--------------------------------------------------------------------------------
   Walks the walls clockwise around the spin point. Perpendicular neighbors meet at a corner;
   parallel neighbors (a step of the room, or a wall hidden from the spin point) are joined by
   a perpendicular connector halfway across the gap between their ends.
  --------------------------------------------------------------------------------*/
static void layoutPolygon(TPlanWall *walls, int n, TLayoutPlan &plan)
{
   for (int i = 0; i < n; i++)
   {
      float mid = 0.5f*(walls[i].a0 + walls[i].a1),
            u = walls[i].kind == 0 ? walls[i].offset : mid,
            w = walls[i].kind == 0 ? mid : walls[i].offset;

      walls[i].midAngle = atan2f(w, u);
   }
   for (int i = 1; i < n; i++)
   {
      TPlanWall key = walls[i];
      int       j = i - 1;

      while (j >= 0 && walls[j].midAngle > key.midAngle)
      {
         walls[j + 1] = walls[j];
         j--;
      }
      walls[j + 1] = key;
   }
   plan.vertexCount = 0;
   for (int i = 0; i < n; i++)
   {
      const TPlanWall &s = walls[i],
                      &t = walls[(i + 1)%n];

      if (s.kind != t.kind)
      {
         float u = s.kind == 0 ? s.offset : t.offset,
               w = s.kind == 0 ? t.offset : s.offset;

         layoutAddVertex(plan, u, w);
         continue;
      }

      // parallel neighbors: connector at the along-coordinate between the facing ends
      float sEnd = fabsf(s.a1 - t.a0) < fabsf(s.a0 - t.a1) ? s.a1 : s.a0,
            tEnd = sEnd == s.a1 ? t.a0 : t.a1,
            c = 0.5f*(sEnd + tEnd);

      if (s.kind == 0)
      {
         layoutAddVertex(plan, s.offset, c);
         layoutAddVertex(plan, t.offset, c);
      }
      else
      {
         layoutAddVertex(plan, c, s.offset);
         layoutAddVertex(plan, c, t.offset);
      }
      plan.complete = false;
   }
   if (plan.vertexCount > 1)
   {
      const TPlanPoint &a = plan.verts[0],
                       &b = plan.verts[plan.vertexCount - 1];

      if (fabsf(a.u - b.u) < 0.05f && fabsf(a.w - b.w) < 0.05f)
         plan.vertexCount--;
   }
}

/*--------------------------------------------------------------------------------
   Door heads (and window heads, set at the same height): horizontal lines ON a wall, below
   the ceiling. Every edge above the horizon is placed on each wall of its orientation; its
   height above the floor is histogrammed and the strongest cluster between 1.6 m and the crown
   molding, in the ASSUMED scale, is the head. Since heads are almost always at 2.10 m (ceilings
   vary), 2.10/head rescales the whole plan and turns the ceiling height into a measure.
  --------------------------------------------------------------------------------*/
static bool layoutDoorHead(const float *data, DWORD count, float axisDeg, float ceilingM, float heightM,
                           const TPlanWall *walls, int n, float &headM)
{
   TAlloc<float> hist((size_t)layoutHeadBins);
   float         best = 0.f;
   int           bestBin = -1;

   memset(hist(), 0, sizeof(float)*layoutHeadBins);
   for (DWORD e = 0; e < count; e++)
   {
      int   kind;
      float q,
            p;
      bool  onCeiling;

      if (!layoutEdge(data + 4u*e, axisDeg, kind, q, p, onCeiling) || !onCeiling)
         continue;
      for (int i = 0; i < n; i++)
      {
         const TPlanWall &w = walls[i];

         if (w.kind != kind || (w.offset < 0.f) != (q < 0.f))
            continue;

         float above = w.offset/q,
               along = w.offset*p/q,
               y = heightM + above;
         int   bin = (int)((y - cMinHeadM)/cHeadBinM);

         if (along < w.a0 - 0.1f || along > w.a1 + 0.1f || y > ceilingM - cMoldingM || bin < 0 || bin >= layoutHeadBins)
            continue;
         hist[bin] += 1.f;
      }
   }
   for (int b = 0; b < layoutHeadBins; b++)
   {
      float s = layoutSmooth(hist(), b, layoutHeadBins);

      if (s > best)
      {
         best = s;
         bestBin = b;
      }
   }
   if (bestBin < 0 || best < (float)layoutMinHeadPts)
      return false;
   headM = cMinHeadM + ((float)bestBin + 0.5f)*cHeadBinM;
   return true;
}

//--------------------------------------------------------------------------------
// Convex corners become stations in walking order; each aims along its diagonal at the farthest corner it sees
static void layoutStations(TLayoutPlan &plan)
{
   plan.stationCount = 0;
   for (int i = 0; i < plan.vertexCount; i++)
   {
      const TPlanPoint &prev = plan.verts[(i + plan.vertexCount - 1)%plan.vertexCount],
                       &next = plan.verts[(i + 1)%plan.vertexCount];

      plan.convex[i] = layoutCross(prev, plan.verts[i], next) < 0.f ? 1u : 0u; // clockwise: right turns
   }
   for (int i = 0; i < plan.vertexCount && plan.stationCount < layoutMaxStations; i++)
   {
      /* the aim is the diagonal: the target must lie within cMaxAimDeg of the corner's inward bisector (a nearly
         adjacent corner along one wall shows that wall edge-on); the farthest such corner wins, and only when none
         qualifies the farthest visible one does. A second corner in that fan, well apart from the first, gets a
         second station on the same spot (the corner facing both arms of an L). A reflex corner (the one jutting
         in) aims the same way, along its inward bisector (user, 2026-09-27: the L is 1->3 and 1->5, 2->6, 3->1, 4->1,
         5->1, 6->2 counterclockwise from the corner facing the reflex one, the reflex being 4) */
      const TPlanPoint &prev = plan.verts[(i + plan.vertexCount - 1)%plan.vertexCount],
                       &next = plan.verts[(i + 1)%plan.vertexCount];
      float             pu = prev.u - plan.verts[i].u,
                        pw = prev.w - plan.verts[i].w,
                        nu = next.u - plan.verts[i].u,
                        nw = next.w - plan.verts[i].w,
                        pl = sqrtf(pu*pu + pw*pw) + 1e-6f,
                        nl = sqrtf(nu*nu + nw*nw) + 1e-6f,
                        turn = plan.convex[i] ? 1.f : -1.f, // a reflex corner: the edges open away from the room
                        bu = turn*(pu/pl + nu/nl),
                        bw = turn*(pw/pl + nw/nl),
                        bl = sqrtf(bu*bu + bw*bw) + 1e-6f,
                        minCos = cosf(cMaxAimDeg*cDegToRad),
                        bestD = 0.f,
                        anyD = 0.f;
      int               best = -1,
                        any = -1;

      for (int j = 0; j < plan.vertexCount; j++)
      {
         float du = plan.verts[j].u - plan.verts[i].u,
               dw = plan.verts[j].w - plan.verts[i].w,
               d = du*du + dw*dw;

         if (j == i || !layoutSees(plan, i, j))
            continue;
         if (d > anyD)
         {
            anyD = d;
            any = j;
         }
         if (d > bestD && (du*bu + dw*bw)/(sqrtf(d)*bl) >= minCos)
         {
            bestD = d;
            best = j;
         }
      }
      bool inFan = best >= 0;

      best = inFan ? best : any;
      if (best < 0)
         continue;
      plan.stationVertex[plan.stationCount] = (BYTE)i;
      plan.targetVertex[plan.stationCount] = (BYTE)best;
      plan.stationCount++;
      if (!inFan || !plan.convex[i] || plan.stationCount >= layoutMaxStations)
         continue;

      // a second arm in the fan: another far corner well apart from the first aim
      float au = plan.verts[best].u - plan.verts[i].u,
            aw = plan.verts[best].w - plan.verts[i].w,
            al = sqrtf(au*au + aw*aw) + 1e-6f,
            maxCos = cosf(cSecondAimDeg*cDegToRad),
            secondD = 0.f;
      int   second = -1;

      for (int j = 0; j < plan.vertexCount; j++)
      {
         float du = plan.verts[j].u - plan.verts[i].u,
               dw = plan.verts[j].w - plan.verts[i].w,
               d = du*du + dw*dw,
               len = sqrtf(d) + 1e-6f;

         if (j == i || j == best || d < 0.5f*bestD || d <= secondD || !layoutSees(plan, i, j)
             || (du*bu + dw*bw)/(len*bl) < minCos || (du*au + dw*aw)/(len*al) > maxCos)
            continue;
         secondD = d;
         second = j;
      }
      if (second < 0)
         continue;
      plan.stationVertex[plan.stationCount] = (BYTE)i;
      plan.targetVertex[plan.stationCount] = (BYTE)second;
      plan.stationCount++;
   }

   // the corner a reflex corner aims at looks back at it: the jutting corner seen head-on (user, 2026-09-27)
   int planned = plan.stationCount;

   for (int s = 0; s < planned && plan.stationCount < layoutMaxStations; s++)
   {
      int  r = plan.stationVertex[s],
           t = plan.targetVertex[s];
      bool have = false;

      if (plan.convex[r])
         continue;
      for (int k = 0; k < plan.stationCount && !have; k++)
         have = plan.stationVertex[k] == t && plan.targetVertex[k] == r;
      if (have)
         continue;
      plan.stationVertex[plan.stationCount] = (BYTE)t;
      plan.targetVertex[plan.stationCount] = (BYTE)r;
      plan.stationCount++;
   }

   // walking order: by corner, the stations of one corner together (stable)
   for (int s = 1; s < plan.stationCount; s++)
   {
      BYTE v = plan.stationVertex[s],
           t = plan.targetVertex[s];
      int  at = s;

      while (at > 0 && plan.stationVertex[at - 1] > v)
      {
         plan.stationVertex[at] = plan.stationVertex[at - 1];
         plan.targetVertex[at] = plan.targetVertex[at - 1];
         at--;
      }
      plan.stationVertex[at] = v;
      plan.targetVertex[at] = t;
   }
}

//--------------------------------------------------------------------------------
float TRoomLayout::AnchorDeg(void) const
{
   return Pframes ? layoutMedian(PframeAxis, Pframes) : PanchorDeg;
}

//--------------------------------------------------------------------------------
// A keyframe whose axis is more than the jump limit from the median of its neighbors (both sides) is an error
bool TRoomLayout::FrameKept(int f) const
{
   int lo = f - layoutDriftWindow < 0 ? 0 : f - layoutDriftWindow,
       hi = f + layoutDriftWindow + 1 > Pframes ? Pframes : f + layoutDriftWindow + 1;

   return fabsf(PframeAxis[f] - layoutMedian(PframeAxis + lo, hi - lo)) <= cMaxFrameJumpDeg;
}

/*--------------------------------------------------------------------------------
   axisDeg: the room axis in the world (AnchorDeg, or a correction of it). The stored edges live on
   the anchor frame, which sits (anchor - median) away from the world.
  --------------------------------------------------------------------------------*/
bool TRoomLayout::Solve(float axisDeg, float ceilingM, TLayoutPlan &out) const
{
   const float *data = Pdata();

   out = TLayoutPlan();
   out.axisDeg = axisDeg;
   out.ceilingM = ceilingM;
   out.cameraHeightM = cDefaultHeightM;
   out.failure = pfNoData;
   if (!data || !Pcount || isnan(axisDeg))
      return false;
   out.failure = pfNone;
   axisDeg += PanchorDeg - AnchorDeg();

   // lines per keyframe, the ceiling creases give the walls; the floor only gives the camera height
   TAlloc<TPlanLine> lines((size_t)layoutMaxLines);
   int               lineCount = 0;

   for (int f = 0; f < Pframes; f++)
      if (FrameKept(f))
         lineCount = layoutFrameLines(data, PframeStart[f], PframeStart[f + 1], axisDeg, f,
                                      layoutCreaseTop(PframePitch[f], PframeCenter[f]), lines(), lineCount,
                                      layoutMaxLines);
   out.cameraHeightM = layoutPairHeight(lines(), lineCount, ceilingM, out.heightSolved, out.heightCorners);
   if (!out.heightSolved)
      out.cameraHeightM = layoutLineHeight(lines(), lineCount, ceilingM, out.heightSolved);

   TPlanWall walls[layoutMaxWalls];
   int       n = layoutLineWalls(lines(), lineCount, ceilingM, out.cameraHeightM, walls, layoutMaxWalls);

   out.wallCount = n;
   for (int i = 0; i < n; i++)
   {
      out.wallKind[i] = walls[i].kind;
      out.wallOffset[i] = walls[i].offset;
      out.wallA0[i] = walls[i].a0;
      out.wallA1[i] = walls[i].a1;
      out.wallWeight[i] = walls[i].weight;
   }
   if (n < 4)
   {
      out.failure = pfFewWalls;
      return false;
   }
   out.complete = true;
   layoutPolygon(walls, n, out);
   if (out.vertexCount < 4)
   {
      out.failure = pfNoPolygon;
      return false;
   }

   float area = 0.f,
         minU = out.verts[0].u,
         maxU = minU,
         minW = out.verts[0].w,
         maxW = minW;

   for (int i = 0; i < out.vertexCount; i++)
   {
      const TPlanPoint &a = out.verts[i],
                       &b = out.verts[(i + 1)%out.vertexCount];

      area += a.w*b.u - b.w*a.u;
      minU = a.u < minU ? a.u : minU;
      maxU = a.u > maxU ? a.u : maxU;
      minW = a.w < minW ? a.w : minW;
      maxW = a.w > maxW ? a.w : maxW;
   }
   out.areaM2 = 0.5f*fabsf(area);
   out.extentU = maxU - minU;
   out.extentW = maxW - minW;
   if (out.areaM2 < cMinAreaM2)
   {
      out.failure = pfTooSmall;
      return false;
   }

   // walls must enclose the spin point from every side
   TPlanPoint origin = { 0.f, 0.f };

   if (!layoutInside(out, origin))
   {
      out.failure = pfOutside;
      return false;
   }

   /* door heads at 2.10 m would fix the scale better than an assumed ceiling, but a head found as "the
      strongest wall line below the molding" is often a cabinet slab or a door trim (office 173630: head read
      at 1.94 m, scale +8.5% the wrong way). Measured and reported only; applied once doors are recognized
      by their head WITH the two jambs below it. */
   float head = 0.f;

   out.assumedCeilingM = ceilingM;
   out.doorScale = 1.f;
   if (layoutDoorHead(data, Pcount, axisDeg, ceilingM, out.cameraHeightM, walls, n, head))
   {
      float s = cDoorHeadM/head;

      if (s >= cMinDoorScale && s <= cMaxDoorScale)
      {
         out.doorFound = true;
         out.doorScale = s; // not applied
      }
   }
   layoutStations(out);
   if (!out.stationCount)
      out.failure = pfNoStations;
   out.valid = out.stationCount > 0;
   return out.valid;
}

//--------------------------------------------------------------------------------
