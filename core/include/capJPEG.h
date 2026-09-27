#ifndef CAPJPEG_H
#define CAPJPEG_H
#include "winTypes.h"
#include "capBuf.h"

/* Baseline JPEG encoder (JFIF, 4:2:0, Annex K tables) fed straight from camera YUV planes, so
 * a keyframe never goes through an RGB round trip. Portable C++, the same on every platform. */

// A camera frame in YUV 4:2:0 with arbitrary strides (YUV_420_888, NV12 and NV21 all fit)
struct TYUVImage {
   int     width, height;
   LPCBYTE y;
   int     yStride;
   LPCBYTE u, v; // Cb, Cr
   int     uvRowStride,
           uvPixelStride; // 1 planar, 2 interleaved
};

class TJPEGEncoder
{
 public:
   explicit TJPEGEncoder(int quality); // 1..100

   // Appends a complete .jpg to out; app11 (optional) is written as an APP11 segment right after APP0
   bool Encode(const TYUVImage &img, TByteBuf &out, LPCBYTE app11 = NULL, size_t app11Len = 0u) const;

   TJPEGEncoder(const TJPEGEncoder &) = delete;
   TJPEGEncoder &operator=(const TJPEGEncoder &) = delete;

 private:
   BYTE  Pquant[2][64]; // luma / chroma, natural order
   float Pscale[2][64]; // 1 / (quant * DCT normalization), natural order
};

#endif // CAPJPEG_H
