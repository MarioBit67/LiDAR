#include "capJPEG.h"
#include "libDiscipline.h"

static const BYTE cZigzag[64] = {
   0, 1, 8, 16, 9, 2, 3, 10, 17, 24, 32, 25, 18, 11, 4, 5,
   12, 19, 26, 33, 40, 48, 41, 34, 27, 20, 13, 6, 7, 14, 21, 28,
   35, 42, 49, 56, 57, 50, 43, 36, 29, 22, 15, 23, 30, 37, 44, 51,
   58, 59, 52, 45, 38, 31, 39, 46, 53, 60, 61, 54, 47, 55, 62, 63
};

static const BYTE cLumaQuant[64] = {
   16, 11, 10, 16, 24, 40, 51, 61,
   12, 12, 14, 19, 26, 58, 60, 55,
   14, 13, 16, 24, 40, 57, 69, 56,
   14, 17, 22, 29, 51, 87, 80, 62,
   18, 22, 37, 56, 68, 109, 103, 77,
   24, 35, 55, 64, 81, 104, 113, 92,
   49, 64, 78, 87, 103, 121, 120, 101,
   72, 92, 95, 98, 112, 100, 103, 99
};

static const BYTE cChromaQuant[64] = {
   17, 18, 24, 47, 99, 99, 99, 99,
   18, 21, 26, 66, 99, 99, 99, 99,
   24, 26, 56, 99, 99, 99, 99, 99,
   47, 66, 99, 99, 99, 99, 99, 99,
   99, 99, 99, 99, 99, 99, 99, 99,
   99, 99, 99, 99, 99, 99, 99, 99,
   99, 99, 99, 99, 99, 99, 99, 99,
   99, 99, 99, 99, 99, 99, 99, 99
};

static const BYTE cDCLumaBits[16] = { 0, 1, 5, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0 },
                  cDCChromaBits[16] = { 0, 3, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0 },
                  cDCVals[12] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 },
                  cACLumaBits[16] = { 0, 2, 1, 3, 3, 2, 4, 3, 5, 5, 4, 4, 0, 0, 1, 0x7d },
                  cACChromaBits[16] = { 0, 2, 1, 2, 4, 4, 3, 4, 7, 5, 4, 4, 0, 1, 2, 0x77 };

static const BYTE cACLumaVals[162] = {
   0x01, 0x02, 0x03, 0x00, 0x04, 0x11, 0x05, 0x12, 0x21, 0x31, 0x41, 0x06, 0x13, 0x51, 0x61, 0x07,
   0x22, 0x71, 0x14, 0x32, 0x81, 0x91, 0xa1, 0x08, 0x23, 0x42, 0xb1, 0xc1, 0x15, 0x52, 0xd1, 0xf0,
   0x24, 0x33, 0x62, 0x72, 0x82, 0x09, 0x0a, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x25, 0x26, 0x27, 0x28,
   0x29, 0x2a, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49,
   0x4a, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5a, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69,
   0x6a, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7a, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89,
   0x8a, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9a, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7,
   0xa8, 0xa9, 0xaa, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6, 0xb7, 0xb8, 0xb9, 0xba, 0xc2, 0xc3, 0xc4, 0xc5,
   0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xd2, 0xd3, 0xd4, 0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda, 0xe1, 0xe2,
   0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9, 0xea, 0xf1, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8,
   0xf9, 0xfa
};

static const BYTE cACChromaVals[162] = {
   0x00, 0x01, 0x02, 0x03, 0x11, 0x04, 0x05, 0x21, 0x31, 0x06, 0x12, 0x41, 0x51, 0x07, 0x61, 0x71,
   0x13, 0x22, 0x32, 0x81, 0x08, 0x14, 0x42, 0x91, 0xa1, 0xb1, 0xc1, 0x09, 0x23, 0x33, 0x52, 0xf0,
   0x15, 0x62, 0x72, 0xd1, 0x0a, 0x16, 0x24, 0x34, 0xe1, 0x25, 0xf1, 0x17, 0x18, 0x19, 0x1a, 0x26,
   0x27, 0x28, 0x29, 0x2a, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48,
   0x49, 0x4a, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5a, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68,
   0x69, 0x6a, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7a, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87,
   0x88, 0x89, 0x8a, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9a, 0xa2, 0xa3, 0xa4, 0xa5,
   0xa6, 0xa7, 0xa8, 0xa9, 0xaa, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6, 0xb7, 0xb8, 0xb9, 0xba, 0xc2, 0xc3,
   0xc4, 0xc5, 0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xd2, 0xd3, 0xd4, 0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda,
   0xe2, 0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9, 0xea, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8,
   0xf9, 0xfa
};

// Canonical Huffman code for one table: code/length per symbol
struct THuffCode {
   WORD code[256];
   BYTE size[256];
};

// MSB-first entropy writer with 0xFF byte stuffing
struct TBitWriter {
   TByteBuf &out;
   DWORD     acc;
   int       count;

   explicit TBitWriter(TByteBuf &o) : out(o), acc(0u), count(0) {}

   void Put(DWORD bits, int n)
   {
      acc = (acc << n) | (bits & ((1u << n) - 1u));
      count += n;
      while (count >= 8)
      {
         BYTE b = (BYTE)(acc >> (count - 8));

         out.PutByte(b);
         if (b == 0xFF)
            out.PutByte(0u);
         count -= 8;
      }
   }

   void Finish(void)
   {
      if (count > 0)
         Put(0x7Fu, 8 - count); // pad with ones
   }
};

//--------------------------------------------------------------------------------
static void jpegBuildHuff(const BYTE bits[16], LPCBYTE vals, THuffCode &h)
{
   WORD code = 0u;
   int  k = 0;

   memset(&h, 0, sizeof(h));
   for (int len = 1; len <= 16; len++)
   {
      for (int i = 0; i < bits[len - 1]; i++)
      {
         h.code[vals[k]] = code;
         h.size[vals[k]] = (BYTE)len;
         code++;
         k++;
      }
      code = (WORD)(code << 1);
   }
}

//--------------------------------------------------------------------------------
static void jpegMarker(TByteBuf &out, BYTE marker, WORD length)
{
   out.PutByte(0xFF);
   out.PutByte(marker);
   out.PutByte((BYTE)(length >> 8));
   out.PutByte((BYTE)length);
}

//--------------------------------------------------------------------------------
static void jpegPutHuffTable(TByteBuf &out, BYTE tableClassId, const BYTE bits[16], LPCBYTE vals, int nVals)
{
   out.PutByte(tableClassId);
   out.PutBytes(bits, 16u);
   out.PutBytes(vals, (size_t)nVals);
}

//--------------------------------------------------------------------------------
static int jpegBitLength(int v)
{
   int n = 0;

   if (v < 0)
      v = -v;
   while (v)
   {
      n++;
      v >>= 1;
   }
   return n;
}

//--------------------------------------------------------------------------------
static void jpegPutValue(TBitWriter &bw, int v, int n)
{
   if (v < 0)
      v += (1 << n) - 1;
   bw.Put((DWORD)v, n);
}

//--------------------------------------------------------------------------------
static void jpegFDCT(const float in[64], float out[64])
{
   static float cosTab[8][8];
   static bool  ready = false;
   float        tmp[64];

   if (!ready)
   {
      for (int u = 0; u < 8; u++)
         for (int x = 0; x < 8; x++)
            cosTab[u][x] = cosf((float)(2*x + 1)*(float)u*3.14159265f/16.f);
      ready = true;
   }
   for (int y = 0; y < 8; y++)
      for (int u = 0; u < 8; u++)
      {
         float s = 0.f;

         for (int x = 0; x < 8; x++)
            s += in[y*8 + x]*cosTab[u][x];
         tmp[y*8 + u] = s;
      }
   for (int u = 0; u < 8; u++)
      for (int v = 0; v < 8; v++)
      {
         float s = 0.f;

         for (int y = 0; y < 8; y++)
            s += tmp[y*8 + u]*cosTab[v][y];
         out[v*8 + u] = s;
      }
}

//--------------------------------------------------------------------------------
static void jpegEncodeBlock(TBitWriter &bw, const float block[64], const float scale[64], int &prevDC,
                            const THuffCode &dc, const THuffCode &ac)
{
   float coef[64];
   int   q[64];

   jpegFDCT(block, coef);
   for (int i = 0; i < 64; i++)
   {
      int   pos = cZigzag[i];
      float c = coef[pos]*scale[pos];

      q[i] = (int)(c < 0.f ? c - 0.5f : c + 0.5f);
   }

   int diff = q[0] - prevDC,
       n = jpegBitLength(diff);

   prevDC = q[0];
   bw.Put(dc.code[n], dc.size[n]);
   if (n)
      jpegPutValue(bw, diff, n);

   int run = 0;

   for (int i = 1; i < 64; i++)
   {
      if (!q[i])
      {
         run++;
         continue;
      }
      while (run > 15)
      {
         bw.Put(ac.code[0xF0], ac.size[0xF0]); // ZRL
         run -= 16;
      }
      n = jpegBitLength(q[i]);

      int sym = (run << 4) | n;

      bw.Put(ac.code[sym], ac.size[sym]);
      jpegPutValue(bw, q[i], n);
      run = 0;
   }
   if (run)
      bw.Put(ac.code[0x00], ac.size[0x00]); // EOB
}

//--------------------------------------------------------------------------------
static int jpegClamp(int v, int hi)
{
   if (v < 0)
      return 0;
   if (v > hi)
      return hi;
   return v;
}

//--------------------------------------------------------------------------------
// 8x8 luma block at (x0, y0), level-shifted, edge-replicated
static void jpegLumaBlock(const TYUVImage &img, int x0, int y0, float out[64])
{
   for (int y = 0; y < 8; y++)
   {
      LPCBYTE row = img.y + (size_t)jpegClamp(y0 + y, img.height - 1)*img.yStride;

      for (int x = 0; x < 8; x++)
         out[y*8 + x] = (float)row[jpegClamp(x0 + x, img.width - 1)] - 128.f;
   }
}

//--------------------------------------------------------------------------------
// 8x8 chroma block at chroma coordinates (x0, y0) from plane p
static void jpegChromaBlock(const TYUVImage &img, LPCBYTE p, int x0, int y0, float out[64])
{
   int cw = (img.width + 1)/2,
       ch = (img.height + 1)/2;

   for (int y = 0; y < 8; y++)
   {
      LPCBYTE row = p + (size_t)jpegClamp(y0 + y, ch - 1)*img.uvRowStride;

      for (int x = 0; x < 8; x++)
         out[y*8 + x] = (float)row[(size_t)jpegClamp(x0 + x, cw - 1)*img.uvPixelStride] - 128.f;
   }
}

//--------------------------------------------------------------------------------
TJPEGEncoder::TJPEGEncoder(int quality)
{
   int q = quality < 1 ? 1 : (quality > 100 ? 100 : quality),
       s = q < 50 ? 5000/q : 200 - 2*q;

   for (int i = 0; i < 64; i++)
   {
      int row = i/8,
          col = i%8,
          l = jpegClamp((cLumaQuant[i]*s + 50)/100, 255),
          c = jpegClamp((cChromaQuant[i]*s + 50)/100, 255);
      float cu = col ? 0.5f : 0.35355339f, // C(u)/2
            cv = row ? 0.5f : 0.35355339f;

      Pquant[0][i] = (BYTE)(l ? l : 1);
      Pquant[1][i] = (BYTE)(c ? c : 1);
      Pscale[0][i] = cu*cv/(float)Pquant[0][i];
      Pscale[1][i] = cu*cv/(float)Pquant[1][i];
   }
}

//--------------------------------------------------------------------------------
bool TJPEGEncoder::Encode(const TYUVImage &img, TByteBuf &out, LPCBYTE app11, size_t app11Len) const
{
   static_assert(sizeof(cACLumaVals) == 162 && sizeof(cACChromaVals) == 162, "Annex K AC tables");

   THuffCode dcL, dcC, acL, acC;
   BYTE      zq[64];

   if (img.width <= 0 || img.height <= 0 || img.width > 65535 || img.height > 65535 || app11Len > 65533u)
      return false;
   jpegBuildHuff(cDCLumaBits, cDCVals, dcL);
   jpegBuildHuff(cDCChromaBits, cDCVals, dcC);
   jpegBuildHuff(cACLumaBits, cACLumaVals, acL);
   jpegBuildHuff(cACChromaBits, cACChromaVals, acC);

   out.PutByte(0xFF);
   out.PutByte(0xD8); // SOI
   jpegMarker(out, 0xE0, 16u); // APP0 JFIF 1.1, no density, no thumbnail
   out.PutBytes((LPCBYTE)"JFIF", 5u);
   out.PutByte(1u);
   out.PutByte(1u);
   out.PutByte(0u);
   out.PutWord(0x0100u);
   out.PutWord(0x0100u);
   out.PutByte(0u);
   out.PutByte(0u);
   if (app11 && app11Len)
   {
      jpegMarker(out, 0xEB, (WORD)(app11Len + 2u)); // APP11: our frame metadata
      out.PutBytes(app11, app11Len);
   }

   jpegMarker(out, 0xDB, 132u); // DQT, both tables in zigzag order
   for (int t = 0; t < 2; t++)
   {
      out.PutByte((BYTE)t);
      for (int i = 0; i < 64; i++)
         zq[i] = Pquant[t][cZigzag[i]];
      out.PutBytes(zq, 64u);
   }

   jpegMarker(out, 0xC0, 17u); // SOF0: 8-bit, Y 2x2, Cb 1x1, Cr 1x1
   out.PutByte(8u);
   out.PutByte((BYTE)(img.height >> 8));
   out.PutByte((BYTE)img.height);
   out.PutByte((BYTE)(img.width >> 8));
   out.PutByte((BYTE)img.width);
   out.PutByte(3u);

   BYTE comps[9] = { 1, 0x22, 0, 2, 0x11, 1, 3, 0x11, 1 };

   out.PutBytes(comps, 9u);

   jpegMarker(out, 0xC4, 418u); // DHT
   jpegPutHuffTable(out, 0x00, cDCLumaBits, cDCVals, 12);
   jpegPutHuffTable(out, 0x10, cACLumaBits, cACLumaVals, 162);
   jpegPutHuffTable(out, 0x01, cDCChromaBits, cDCVals, 12);
   jpegPutHuffTable(out, 0x11, cACChromaBits, cACChromaVals, 162);

   jpegMarker(out, 0xDA, 12u); // SOS
   out.PutByte(3u);

   BYTE scan[9] = { 1, 0x00, 2, 0x11, 3, 0x11, 0, 63, 0 };

   out.PutBytes(scan, 9u);

   TBitWriter bw(out);
   float      block[64];
   int        dcY = 0,
              dcU = 0,
              dcV = 0;

   for (int my = 0; my < img.height; my += 16)
      for (int mx = 0; mx < img.width; mx += 16)
      {
         jpegLumaBlock(img, mx, my, block);
         jpegEncodeBlock(bw, block, Pscale[0], dcY, dcL, acL);
         jpegLumaBlock(img, mx + 8, my, block);
         jpegEncodeBlock(bw, block, Pscale[0], dcY, dcL, acL);
         jpegLumaBlock(img, mx, my + 8, block);
         jpegEncodeBlock(bw, block, Pscale[0], dcY, dcL, acL);
         jpegLumaBlock(img, mx + 8, my + 8, block);
         jpegEncodeBlock(bw, block, Pscale[0], dcY, dcL, acL);
         jpegChromaBlock(img, img.u, mx/2, my/2, block);
         jpegEncodeBlock(bw, block, Pscale[1], dcU, dcC, acC);
         jpegChromaBlock(img, img.v, mx/2, my/2, block);
         jpegEncodeBlock(bw, block, Pscale[1], dcV, dcC, acC);
      }
   bw.Finish();
   out.PutByte(0xFF);
   out.PutByte(0xD9); // EOI
   return out.Ok();
}

//--------------------------------------------------------------------------------
