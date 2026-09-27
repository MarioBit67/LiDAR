#ifndef SHARED_SHA256_H
#define SHARED_SHA256_H
#include "winTypes.h" // winint typedefs (BYTE/DWORD/QWORD/LPCBYTE/LPSTR/LPCSTR)

#include <stddef.h> // size_t
#include <stdio.h>  // fopen/fread — sha256HexFile streams a file through the core

/* THE SHA-256 (FIPS 180-4) - every SHA operation in the codebase lives behind this one header
   (goldenRule 28: no per-engine copy). Plain integer C++: no OS crypto, no libc alloc, no float, so the
   SAME code and the SAME digests serve Windows, Android and iOS - which is precisely why an OS-provided
   hasher cannot be the canonical one. The context is a plain struct and therefore COPYABLE: copy it to
   snapshot a midstate and resume from that snapshot (what the proof-of-work sweep does per candidate). */

typedef struct
{
   DWORD state[8];
   QWORD bitlen;
   BYTE  buf[64];
   DWORD buflen;
} TSHA256Ctx;

/* The engine is header-inline ON PURPOSE: every test target hashes through testCache.h, and a linked
   object would force each of them to carry an extra source. Inline keeps the ONE definition in the ONE
   file and costs nothing at a call site. */

// -- SHA-256 (FIPS 180-4), plain C, copyable context -------------------------
static const DWORD K256[64] = {
   0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
   0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
   0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
   0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
   0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
   0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
   0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
   0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2};

#define ror(x, n) (((x) >> (n)) | ((x) << (32 - (n))))

//--------------------------------------------------------------------------------
static inline void sha256Block(DWORD st[8], LPCBYTE p)
{
   DWORD w[64],
         a,
         b,
         c,
         d,
         e,
         f,
         g,
         h,
         t1,
         t2;
   int i;

   for (i = 0; i < 16; i++)
      w[i] = ((DWORD)p[i*4] << 24) | ((DWORD)p[i*4 + 1] << 16) | ((DWORD)p[i*4 + 2] << 8) | (DWORD)p[i*4 + 3];
   for (i = 16; i < 64; i++)
   {
      DWORD s0 = ror(w[i - 15], 7) ^ ror(w[i - 15], 18) ^ (w[i - 15] >> 3),
            s1 = ror(w[i - 2], 17) ^ ror(w[i - 2], 19) ^ (w[i - 2] >> 10);

      w[i] = w[i - 16] + s0 + w[i - 7] + s1;
   }
   a = st[0];
   b = st[1];
   c = st[2];
   d = st[3];
   e = st[4];
   f = st[5];
   g = st[6];
   h = st[7];
   for (i = 0; i < 64; i++)
   {
      DWORD S1 = ror(e, 6) ^ ror(e, 11) ^ ror(e, 25),
            ch = (e & f) ^ (~e & g);

      t1 = h + S1 + ch + K256[i] + w[i];

      DWORD S0 = ror(a, 2) ^ ror(a, 13) ^ ror(a, 22),
            maj = (a & b) ^ (a & c) ^ (b & c);

      t2 = S0 + maj;
      h = g;
      g = f;
      f = e;
      e = d + t1;
      d = c;
      c = b;
      b = a;
      a = t1 + t2;
   }
   st[0] += a;
   st[1] += b;
   st[2] += c;
   st[3] += d;
   st[4] += e;
   st[5] += f;
   st[6] += g;
   st[7] += h;
}

//--------------------------------------------------------------------------------
static inline void sha256Init(TSHA256Ctx *c)
{
   c->state[0] = 0x6a09e667;
   c->state[1] = 0xbb67ae85;
   c->state[2] = 0x3c6ef372;
   c->state[3] = 0xa54ff53a;
   c->state[4] = 0x510e527f;
   c->state[5] = 0x9b05688c;
   c->state[6] = 0x1f83d9ab;
   c->state[7] = 0x5be0cd19;
   c->bitlen = 0;
   c->buflen = 0;
}

//--------------------------------------------------------------------------------
static inline void sha256Update(TSHA256Ctx *c, LPCBYTE data, size_t len)
{
   size_t i = 0;

   if (c->buflen)
   {
      while (i < len && c->buflen < 64)
         c->buf[c->buflen++] = data[i++];
      if (c->buflen == 64)
      {
         sha256Block(c->state, c->buf);
         c->bitlen += 512;
         c->buflen = 0;
      }
   }
   for (; i + 64 <= len; i += 64)
   {
      sha256Block(c->state, data + i);
      c->bitlen += 512;
   }
   while (i < len)
      c->buf[c->buflen++] = data[i++];
}

//--------------------------------------------------------------------------------
static inline void sha256Final(TSHA256Ctx *c, BYTE out[32])
{
   QWORD total = c->bitlen + (QWORD)c->buflen*8u;

   c->buf[c->buflen++] = 0x80;
   if (c->buflen > 56)
   {
      while (c->buflen < 64)
         c->buf[c->buflen++] = 0;
      sha256Block(c->state, c->buf);
      c->buflen = 0;
   }
   while (c->buflen < 56)
      c->buf[c->buflen++] = 0;
   for (int i = 7; i >= 0; i--)
      c->buf[c->buflen++] = (BYTE)(total >> (i*8));
   sha256Block(c->state, c->buf);
   for (int i = 0; i < 8; i++)
   {
      out[i*4 + 0] = (BYTE)(c->state[i] >> 24);
      out[i*4 + 1] = (BYTE)(c->state[i] >> 16);
      out[i*4 + 2] = (BYTE)(c->state[i] >> 8);
      out[i*4 + 3] = (BYTE)(c->state[i]);
   }
}

//--------------------------------------------------------------------------------
static inline void sha256Buffer(LPCBYTE data, size_t n, BYTE out[32])
{
   TSHA256Ctx c;

   sha256Init(&c);
   sha256Update(&c, data, n);
   sha256Final(&c, out);
}

//--------------------------------------------------------------------------------
static inline void sha256ToHex(const BYTE dig[32], LPSTR outHex)
{
   static const char hex[] = "0123456789abcdef";

   for (int i = 0; i < 32; i++)
   {
      outHex[i*2]     = hex[(dig[i] >> 4) & 0xF];
      outHex[i*2 + 1] = hex[dig[i] & 0xF];
   }
   outHex[64] = '\0';
}

//--------------------------------------------------------------------------------
static inline int sha256HexBuffer(LPCBYTE data, size_t len, LPSTR outHex)
{
   BYTE dig[32];

   if (!outHex)
      return 1;
   sha256Buffer(data, len, dig);
   sha256ToHex(dig, outHex);
   return 0;
}

/*--------------------------------------------------------------------------------
   Stream a file through the core. Chunked so an image of any size costs one fixed buffer, and the
   buffer is a local (no allocator involved on any platform).
  --------------------------------------------------------------------------------*/
static inline int sha256HexFile(LPCSTR path, LPSTR outHex)
{
   enum { kChunk = 16384 };

   BYTE       buf[kChunk],
              dig[32];
   TSHA256Ctx c;
   FILE      *f = NULL;

   if (!path || !outHex)
      return 1;
   f = fopen(path, "rb");
   if (!f)
      return 1;
   sha256Init(&c);

   size_t n;

   while ((n = fread(buf, 1, sizeof buf, f)) > 0)
      sha256Update(&c, buf, n);
   fclose(f);
   sha256Final(&c, dig);
   sha256ToHex(dig, outHex);
   return 0;
}

/*--------------------------------------------------------------------------------
   PCCSHA — the digest of load||suffix. The stamper sweeps the suffix until the result opens with the
   required run of zero nibbles; a file whose digest still clears that run therefore cannot have been
   touched since it was stamped. A null/empty suffix degenerates to a plain digest of the load.
  --------------------------------------------------------------------------------*/
static inline void sha256PCC(LPCBYTE load, size_t loadLen, LPCBYTE suffix, size_t suffixLen, BYTE out[32])
{
   TSHA256Ctx c;

   sha256Init(&c);
   if (load && loadLen)
      sha256Update(&c, load, loadLen);
   if (suffix && suffixLen)
      sha256Update(&c, suffix, suffixLen);
   sha256Final(&c, out);
}

//--------------------------------------------------------------------------------
static inline int sha256PCCZeros(const BYTE dig[32])
{
   int z = 0;

   while (z < 64 && ((dig[z/2] >> (z%2 ? 0 : 4)) & 0xF) == 0)
      z++;
   return z;
}

//--------------------------------------------------------------------------------
static inline bool sha256PCCOk(const BYTE dig[32], int zeroNibbles)
{
   return sha256PCCZeros(dig) >= zeroNibbles;
}

/* SHA-256 of the RUNNING executable as lowercase 64-hex + NUL into out (cap >= 65). Returns out, or
   "(unavailable)" if the image cannot be read. One routine so every app prints the SAME "[SHA] ..."
   entry line - a build fingerprint that tells a dev binary from a stale prod copy at a glance. */
LPCSTR appSelfSHA(LPSTR out, int cap);

/* -- PCCSHA: the pristine-file proof -----------------------------------------------
   PCCSHA is SHA-256 over the payload with a SUFFIX appended to the load - the stamper sweeps that
   suffix until the digest opens with the required run of zero nibbles, so a matching digest can only
   belong to an untouched original. sha256PCC hashes load||suffix in one call (suffix may be null/0,
   which degenerates to a plain digest); sha256PCCZeros counts the digest's leading zero nibbles and
   sha256PCCOk tests it against the required run. */
void sha256PCC(LPCBYTE load, size_t loadLen, LPCBYTE suffix, size_t suffixLen, BYTE out[32]);
int  sha256PCCZeros(const BYTE dig[32]);                 // how many leading nibbles are zero (0..64)
bool sha256PCCOk(const BYTE dig[32], int zeroNibbles);   // does it clear the required run?

#endif // SHARED_SHA256_H
