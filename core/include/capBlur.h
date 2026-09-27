#ifndef CAPBLUR_H
#define CAPBLUR_H
#include "winTypes.h"

/* Blur of a keyframe from the spectrum of its luma (user, 2026-09-27: "a FFT da imagem evidencia o
 * grau de borrão"). A grid of full-resolution tiles is windowed and transformed; the radial power
 * spectrum of a natural scene falls as 1/f^2, and blur (motion or focus) cuts the high frequencies on
 * top of that. The ratio of a high band to a low band, against the 1/f^2 ratio, gives the width of the
 * equivalent Gaussian blur. The luma is box-reduced first: focus and motion blur live at a few pixels,
 * the grain and the ISP sharpening at one or two. The median over the textured tiles speaks for the
 * frame (one strong edge makes a single tile look sharp whatever the focus); plain tiles are skipped.
 * 083920 as the operator judged it: 42 excellent 0.0, 32 almost good 2.6, 44 and 15 out of focus 5.7
 * and 6.3. */

enum {
   blurTileSide = 256, // pixels of a tile (power of two)
   blurGridSide = 3    // tiles per side of the grid over the image center
};

struct TBlurResult {
   float medianPx,   // equivalent Gaussian sigma in full-image pixels, median over the textured tiles (NaN: none)
         sharpPx;    // the sharpest textured tile alone (diagnosis)
   int   tiles,      // tiles measured
         textured;   // tiles with texture enough to judge
   float tilePx[blurGridSide*blurGridSide]; // per tile, row-major (NaN: plain)
};

// Measures the blur of an 8-bit luma plane; false when no tile had texture enough
bool blurMeasure(LPCBYTE luma, int width, int height, int stride, TBlurResult &out);

#endif // CAPBLUR_H
