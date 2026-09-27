#include "capCanvas.h"
#include "libDiscipline.h"

//--------------------------------------------------------------------------------
DWORD canvasRGBA(int r, int g, int b, int a)
{
   return (DWORD)(r & 0xFF) | ((DWORD)(g & 0xFF) << 8) | ((DWORD)(b & 0xFF) << 16) | ((DWORD)(a & 0xFF) << 24);
}

//--------------------------------------------------------------------------------
static DWORD canvasBlend(DWORD dst, DWORD src)
{
   DWORD a = src >> 24;

   if (a == 255u)
      return src | 0xFF000000u;
   if (a == 0u)
      return dst;

   DWORD out = 0xFF000000u;

   for (int sh = 0; sh < 24; sh += 8)
   {
      DWORD d = (dst >> sh) & 0xFFu,
            c = (src >> sh) & 0xFFu;

      out |= ((c*a + d*(255u - a))/255u) << sh;
   }
   return out;
}

//--------------------------------------------------------------------------------
void canvasFillRect(TSurface &s, int x, int y, int w, int h, DWORD rgba)
{
   int x0 = x < 0 ? 0 : x,
       y0 = y < 0 ? 0 : y,
       x1 = x + w > s.width ? s.width : x + w,
       y1 = y + h > s.height ? s.height : y + h;

   for (int py = y0; py < y1; py++)
   {
      LPDWORD row = s.pixels + (size_t)py*s.stride;

      for (int px = x0; px < x1; px++)
         row[px] = canvasBlend(row[px], rgba);
   }
}

//--------------------------------------------------------------------------------
void canvasFillCircle(TSurface &s, int cx, int cy, int radius, DWORD rgba)
{
   int r2 = radius*radius;

   for (int dy = -radius; dy <= radius; dy++)
   {
      int py = cy + dy;

      if (py < 0 || py >= s.height)
         continue;

      LPDWORD row = s.pixels + (size_t)py*s.stride;

      for (int dx = -radius; dx <= radius; dx++)
      {
         int px = cx + dx;

         if (px >= 0 && px < s.width && dx*dx + dy*dy <= r2)
            row[px] = canvasBlend(row[px], rgba);
      }
   }
}

//--------------------------------------------------------------------------------
void canvasRing(TSurface &s, int cx, int cy, int radius, int thickness, DWORD rgba)
{
   int outer = radius*radius,
       inner = (radius - thickness)*(radius - thickness);

   for (int dy = -radius; dy <= radius; dy++)
   {
      int py = cy + dy;

      if (py < 0 || py >= s.height)
         continue;

      LPDWORD row = s.pixels + (size_t)py*s.stride;

      for (int dx = -radius; dx <= radius; dx++)
      {
         int px = cx + dx,
             d2 = dx*dx + dy*dy;

         if (px >= 0 && px < s.width && d2 <= outer && d2 >= inner)
            row[px] = canvasBlend(row[px], rgba);
      }
   }
}

//--------------------------------------------------------------------------------
void canvasBlit(TSurface &s, int x, int y, int w, int h, const DWORD *img, int iw, int ih)
{
   if (!img || iw <= 0 || ih <= 0 || w <= 0 || h <= 0)
      return;

   // scale to fill (cover), keep aspect, crop the overflow evenly (Q16 fixed point)
   QWORD sx = ((QWORD)iw << 16)/(QWORD)w,
         sy = ((QWORD)ih << 16)/(QWORD)h,
         step = sx < sy ? sx : sy,
         offX = (((QWORD)iw << 16) - step*(QWORD)w)/2u,
         offY = (((QWORD)ih << 16) - step*(QWORD)h)/2u;

   for (int py = 0; py < h; py++)
   {
      int sy2 = (int)((offY + step*(QWORD)py) >> 16),
          ty = y + py;

      if (ty < 0 || ty >= s.height)
         continue;

      LPDWORD      row = s.pixels + (size_t)ty*s.stride;
      const DWORD *src = img + (size_t)(sy2 < ih ? sy2 : ih - 1)*iw;

      for (int px = 0; px < w; px++)
      {
         int sx2 = (int)((offX + step*(QWORD)px) >> 16),
             tx = x + px;

         if (tx >= 0 && tx < s.width)
            row[tx] = src[sx2 < iw ? sx2 : iw - 1] | 0xFF000000u;
      }
   }
}

//--------------------------------------------------------------------------------
void canvasLine(TSurface &s, int x0, int y0, int x1, int y1, int thickness, DWORD rgba)
{
   int dx = x1 - x0,
       dy = y1 - y0,
       steps = abs(dx) > abs(dy) ? abs(dx) : abs(dy),
       half = thickness/2;

   if (!steps)
      steps = 1;
   for (int i = 0; i <= steps; i++)
      canvasFillRect(s, x0 + dx*i/steps - half, y0 + dy*i/steps - half, thickness, thickness, rgba);
}

/*--------------------------------------------------------------------------------
   Convex quadrilateral (corners in order), scanline filled: for each row the span between the
   leftmost and rightmost crossing of the four edges.
  --------------------------------------------------------------------------------*/
void canvasFillQuad(TSurface &s, const int *xs, const int *ys, DWORD rgba)
{
   int top = ys[0],
       bottom = ys[0];

   for (int i = 1; i < 4; i++)
   {
      top = ys[i] < top ? ys[i] : top;
      bottom = ys[i] > bottom ? ys[i] : bottom;
   }
   top = top < 0 ? 0 : top;
   bottom = bottom >= s.height ? s.height - 1 : bottom;
   for (int y = top; y <= bottom; y++)
   {
      int lo = INT_MAX,
          hi = INT_MIN;

      for (int i = 0; i < 4; i++)
      {
         int x0 = xs[i],
             y0 = ys[i],
             x1 = xs[(i + 1)%4],
             y1 = ys[(i + 1)%4];

         if (y0 == y1 || (y < y0 && y < y1) || (y > y0 && y > y1))
            continue; // horizontal edges: their end points belong to the neighbors

         int x = x0 + (x1 - x0)*(y - y0)/(y1 - y0);

         lo = x < lo ? x : lo;
         hi = x > hi ? x : hi;
      }
      if (lo > hi)
         continue;
      lo = lo < 0 ? 0 : lo;
      hi = hi >= s.width ? s.width - 1 : hi;

      LPDWORD row = s.pixels + (size_t)y*s.stride;

      for (int x = lo; x <= hi; x++)
         row[x] = canvasBlend(row[x], rgba);
   }
}

//--------------------------------------------------------------------------------
