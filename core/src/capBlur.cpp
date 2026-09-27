#include "capBlur.h"
#include "alloc.h"
#include "libDiscipline.h"

static const float cPi = 3.14159265f,
                   cLowF0 = 0.03f,   // low band, cycles per pixel: the scene itself
                   cLowF1 = 0.06f,
                   cHighF0 = 0.10f,  // high band: what blur takes away first
                   cHighF1 = 0.20f,
                   cNoiseF0 = 0.40f, // noise band: sensor grain is white, the scene is gone there
                   cNoiseF1 = 0.50f,
                   cMinStdDev = 6.f; // luma standard deviation of a tile with texture enough to judge
static const int   cReduce = 4;      // box reduction first: focus and motion blur live at a few pixels, the grain and
                                     // the ISP sharpening at one or two (both on 42 and 44 of 083920)

//--------------------------------------------------------------------------------
// Mean power over an annulus of a 1/f^2 spectrum (samples grow with f in 2D)
static float blurBandPower(float f0, float f1)
{
   return logf(f1/f0)/(0.5f*(f1*f1 - f0*f0));
}

//--------------------------------------------------------------------------------
// Mean f^2 over an annulus, weighted by the power of a 1/f^2 spectrum
static float blurBandF2(float f0, float f1)
{
   return 0.5f*(f1*f1 - f0*f0)/logf(f1/f0);
}

//--------------------------------------------------------------------------------
// In-place radix-2 FFT of n complex samples (n a power of two); cosT/sinT hold n/2 twiddles
static void blurFFT(float *re, float *im, int n, const float *cosT, const float *sinT)
{
   int j = 0;

   for (int i = 1; i < n; i++)
   {
      int bit = n >> 1;

      while (j & bit)
      {
         j ^= bit;
         bit >>= 1;
      }
      j ^= bit;
      if (i < j)
      {
         float tr = re[i],
               ti = im[i];

         re[i] = re[j];
         im[i] = im[j];
         re[j] = tr;
         im[j] = ti;
      }
   }
   for (int len = 2; len <= n; len <<= 1)
   {
      int half = len >> 1,
          step = n/len;

      for (int i = 0; i < n; i += len)
         for (int k = 0; k < half; k++)
         {
            int   a = i + k,
                  b = a + half;
            float wr = cosT[k*step],
                  wi = -sinT[k*step],
                  xr = re[b]*wr - im[b]*wi,
                  xi = re[b]*wi + im[b]*wr;

            re[b] = re[a] - xr;
            im[b] = im[a] - xi;
            re[a] += xr;
            im[a] += xi;
         }
   }
}

//--------------------------------------------------------------------------------
static float blurMedian(float *v, int n)
{
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
   return n%2 ? v[n/2] : 0.5f*(v[n/2 - 1] + v[n/2]);
}

/*--------------------------------------------------------------------------------
   One tile: mean removed, Hann window, 2D FFT, then the mean power of the low and the high band.
   Their ratio against the 1/f^2 ratio is the Gaussian attenuation exp(-4 pi^2 s^2 f^2) between the
   two bands, which yields the sigma s. A tile at least as rich in high frequencies as 1/f^2 is sharp
   (s = 0). NaN when the tile is plain.
  --------------------------------------------------------------------------------*/
static float blurTile(const float *plane, int stride, int x0, int y0, float *re, float *im, float *colRe, float *colIm,
                      const float *window, const float *cosT, const float *sinT)
{
   const int n = blurTileSide;
   float     mean = 0.f,
             var = 0.f;

   for (int y = 0; y < n; y++)
      for (int x = 0; x < n; x++)
         mean += plane[(size_t)(y0 + y)*stride + x0 + x];
   mean /= (float)(n*n);
   for (int y = 0; y < n; y++)
      for (int x = 0; x < n; x++)
      {
         float v = plane[(size_t)(y0 + y)*stride + x0 + x] - mean;

         var += v*v;
         re[y*n + x] = v*window[x]*window[y];
         im[y*n + x] = 0.f;
      }
   if (sqrtf(var/(float)(n*n)) < cMinStdDev)
      return NAN;
   for (int y = 0; y < n; y++)
      blurFFT(re + y*n, im + y*n, n, cosT, sinT);
   for (int x = 0; x < n; x++)
   {
      for (int y = 0; y < n; y++)
      {
         colRe[y] = re[y*n + x];
         colIm[y] = im[y*n + x];
      }
      blurFFT(colRe, colIm, n, cosT, sinT);
      for (int y = 0; y < n; y++)
      {
         re[y*n + x] = colRe[y];
         im[y*n + x] = colIm[y];
      }
   }

   float  low = 0.f,
          high = 0.f,
          noise = 0.f;
   int    lowN = 0,
          highN = 0,
          noiseN = 0;

   for (int ky = 0; ky < n; ky++)
      for (int kx = 0; kx < n; kx++)
      {
         float fx = (float)(kx < n/2 ? kx : kx - n)/(float)n,
               fy = (float)(ky < n/2 ? ky : ky - n)/(float)n,
               f = sqrtf(fx*fx + fy*fy),
               p = re[ky*n + kx]*re[ky*n + kx] + im[ky*n + kx]*im[ky*n + kx];

         if (f >= cLowF0 && f < cLowF1)
         {
            low += p;
            lowN++;
         }
         else if (f >= cHighF0 && f < cHighF1)
         {
            high += p;
            highN++;
         }
         else if (f >= cNoiseF0 && f < cNoiseF1)
         {
            noise += p;
            noiseN++;
         }
      }
   if (!lowN || !highN || !noiseN || low <= 0.f)
      return NAN;

   // the white grain floor is taken off both bands: what is left is the scene through the blur
   float grain = noise/(float)noiseN,
         lowP = low/(float)lowN - grain,
         highP = high/(float)highN - grain;

   if (lowP <= 0.f)
      return NAN;

   float ratio = highP/lowP,
         ideal = blurBandPower(cHighF0, cHighF1)/blurBandPower(cLowF0, cLowF1),
         df2 = blurBandF2(cHighF0, cHighF1) - blurBandF2(cLowF0, cLowF1);

   if (ratio >= ideal)
      return 0.f;
   if (ratio <= 0.f)
      return 99.f;
   return sqrtf(logf(ideal/ratio)/(4.f*cPi*cPi*df2));
}

//--------------------------------------------------------------------------------
bool blurMeasure(LPCBYTE luma, int width, int height, int stride, TBlurResult &out)
{
   const int     n = blurTileSide,
                 g = blurGridSide,
                 rw = width/cReduce,
                 rh = height/cReduce;
   TAlloc<float> reduced((size_t)(rw > 0 ? rw : 1)*(rh > 0 ? rh : 1)),
                 re((size_t)n*n),
                 im((size_t)n*n),
                 colRe((size_t)n),
                 colIm((size_t)n),
                 window((size_t)n),
                 cosT((size_t)n/2),
                 sinT((size_t)n/2),
                 textured((size_t)g*g);

   out.sharpPx = NAN;
   out.medianPx = NAN;
   out.tiles = 0;
   out.textured = 0;
   for (int i = 0; i < g*g; i++)
      out.tilePx[i] = NAN;
   if (!luma || rw < n || rh < n)
      return false;
   for (int y = 0; y < rh; y++)
      for (int x = 0; x < rw; x++)
      {
         int sum = 0;

         for (int dy = 0; dy < cReduce; dy++)
            for (int dx = 0; dx < cReduce; dx++)
               sum += luma[(size_t)(y*cReduce + dy)*stride + x*cReduce + dx];
         reduced[(size_t)y*rw + x] = (float)sum/(float)(cReduce*cReduce);
      }
   for (int i = 0; i < n; i++)
      window[i] = 0.5f - 0.5f*cosf(2.f*cPi*(float)i/(float)(n - 1));
   for (int i = 0; i < n/2; i++)
   {
      cosT[i] = cosf(2.f*cPi*(float)i/(float)n);
      sinT[i] = sinf(2.f*cPi*(float)i/(float)n);
   }

   // tile centers spread over the middle 90% of the image
   for (int ty = 0; ty < g; ty++)
      for (int tx = 0; tx < g; tx++)
      {
         int cx = (int)((float)rw*(0.05f + 0.9f*((float)tx + 0.5f)/(float)g)),
             cy = (int)((float)rh*(0.05f + 0.9f*((float)ty + 0.5f)/(float)g)),
             x0 = cx - n/2,
             y0 = cy - n/2;

         x0 = x0 < 0 ? 0 : (x0 + n > rw ? rw - n : x0);
         y0 = y0 < 0 ? 0 : (y0 + n > rh ? rh - n : y0);

         float s = blurTile(reduced(), rw, x0, y0, re(), im(), colRe(), colIm(), window(), cosT(), sinT());

         s *= (float)cReduce; // back to pixels of the full image

         out.tilePx[ty*g + tx] = s;
         out.tiles++;
         if (isnan(s))
            continue;
         textured[out.textured] = s;
         out.textured++;
         if (isnan(out.sharpPx) || s < out.sharpPx)
            out.sharpPx = s;
      }
   if (!out.textured)
      return false;
   out.medianPx = blurMedian(textured(), out.textured);
   return true;
}
