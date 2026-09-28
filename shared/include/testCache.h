/*
 * testCache.h — shared SHA-cache test engine.
 *
 * A test that hasn't changed should not re-run. At start, abTestBegin() computes a SHA-256
 * over everything that determines the test's result, looks it up in a per-test cache file, and
 * if the stored SHA matches it EARLY-EXITS with the cached exit code. On a miss the test runs and
 * abTestEnd() stores SHA -> exit code.
 *
 * The SHA covers (union of the spec):
 *   - the test's own executable .text AND .rdata sections (timestamp-independent codegen + const/string
 *     signature, so a codegen-neutral rebuild still HITS but a .rdata-only change — an embedded descriptor,
 *     a literal table — correctly MISSES; statically-linked library code is included);
 *   - the full command line (argv) — the sole case discriminator (tests take NO env, see arg.h);
 *   - each declared input file's size + last-write-time (metadata, not content — a HIT stays near-zero;
 *     a real edit changes size/mtime and invalidates);
 *   - each declared external application fired by the test: its .text + that invocation's cmdline;
 *   - an optional caller `extra` fingerprint string (e.g. TFrameCodec::FeatureDescriptor) — folds an
 *     out-of-band determinant (active model / beacon field hash / cs* build selection) into the SHA so a
 *     change there MISSES even when the exe .text/.rdata do not relink (the stale-lib class).
 *
 * 🔴 THE PRECONDITION: A CACHED TEST MUST BE A FUNCTION OF WHAT THE SHA COVERS. Caching is memoization,
 * so a test whose result also depends on the clock, an RNG, a thread interleaving or an undeclared file
 * is not cacheable at all — the cache will freeze one draw's verdict and replay it as truth, which is
 * indistinguishable from a flaky test. If a test needs a per-run nonce (the usual reason: so a stale
 * artifact can never be mistaken for a fresh result), seed it from TAbTestCache::Nonce() rather than the
 * clock — see the note there. Anything else the result depends on goes in `inputs` or `extra`.
 *
 * Header-only + self-contained (bundled SHA-256), so any target reaching shared/include can use it
 * with no link changes. RAII usage (preferred — ctor checks, dtor saves):
 *     int main(int argc, char **argv) {
 *        abArgInit(argc, argv); abSetIdlePriority();
 *        const char *inputs[] = { coverPngPath, NULL };             // or NULL
 *        TAbTestCache tc(argc, argv, inputs);                       // ctor: SHA + cache lookup
 *        if (tc.isHit()) return tc.code();                          // HIT -> skip
 *        int code = run_the_test();
 *        return tc.commit(code);                                    // dtor persists SHA -> code
 *     }
 * Legacy free-function form (still supported): TAbTestCache tc; if (abTestBegin(&tc, argc, argv,
 * inputs, externals, extra)) return tc.cachedCode; ... return abTestEnd(&tc, code);  (extra = 0 for none)
 */
#ifndef TEST_CACHE_H
#define TEST_CACHE_H
#include "winTypes.h" // winint typedefs (BYTE/WORD/DWORD/...)

#include <stdint.h>
#include <stdio.h>
#include "sha256.h" // THE shared SHA-256 — the cache owns no engine of its own
#include <stdlib.h>
#include <string.h>

#if defined(_WIN32)
   #include <windows.h>
#endif

/* -- SHA-256 ---------------------------------------------------------------------------------
   No engine here: the cache hashes with the ONE shared SHA-256 (goldenRule 28). These names stay so the
   cache's own call sites read unchanged. */
typedef TSHA256Ctx TAbtcSHA;

//--------------------------------------------------------------------------------
static inline void abtcSHAInit(TAbtcSHA *s)
{
   sha256Init(s);
}

//--------------------------------------------------------------------------------
static inline void abtcSHAUpdate(TAbtcSHA *s, LPCVOID data, size_t n)
{
   sha256Update(s, (LPCBYTE)data, n);
}

//--------------------------------------------------------------------------------
static inline void abtcSHAFinal(TAbtcSHA *s, BYTE out[32])
{
   sha256Final(s, out);
}

// -- hashing helpers ----------------------------------------------------------------------

// Hash a file's full content into the running SHA. Returns 0 ok, -1 if unreadable.
static inline int abtcHashFile(TAbtcSHA *sha, LPCSTR path)
{
   FILE *f = fopen(path, "rb");

   if (!f)
      return -1;

   BYTE buf[8192];
   size_t r;

   while ((r = fread(buf, 1, sizeof(buf), f)) > 0)
      abtcSHAUpdate(sha, buf, r);
   fclose(f);
   return 0;
}

/* Hash a file's size + last-write-time (metadata, NOT content) — near-instant, no read. A real edit
   changes size or mtime -> the SHA changes -> the cache invalidates. Used for declared INPUT data files
   so a cache HIT stays near-zero (content hashing multi-MB corpora would dominate the hit). Returns 0 ok,
   -1 if the file is missing/unstattable (caller treats as "cannot cache"). */

//--------------------------------------------------------------------------------
static inline int abtcHashFileMeta(TAbtcSHA *sha, LPCSTR path)
{
#if defined(_WIN32)
   WIN32_FILE_ATTRIBUTE_DATA fad;

   if (!GetFileAttributesExA(path, GetFileExInfoStandard, &fad))
      return -1;

   QWORD sz = ((QWORD)fad.nFileSizeHigh << 32) | (QWORD)fad.nFileSizeLow,
         mt = ((QWORD)fad.ftLastWriteTime.dwHighDateTime << 32) | (QWORD)fad.ftLastWriteTime.dwLowDateTime;

   abtcSHAUpdate(sha, &sz, sizeof sz);
   abtcSHAUpdate(sha, &mt, sizeof mt);
   return 0;
#else
   (void)sha;
   (void)path;
   return -1;
#endif
}

//--------------------------------------------------------------------------------
static inline int abtcHashPeText(TAbtcSHA *sha, LPCSTR path)
{
   FILE *f = fopen(path, "rb");

   if (!f)
      return -1;

   BYTE dos[64];

   if (fread(dos, 1, 64, f) != 64 || dos[0] != 'M' || dos[1] != 'Z')
   {
      fclose(f);
      return -1;
   }
   DWORD peoff = (DWORD)dos[60] | ((DWORD)dos[61] << 8) |
                    ((DWORD)dos[62] << 16) | ((DWORD)dos[63] << 24);

   if (fseek(f, (long)peoff, SEEK_SET) != 0)
   {
      fclose(f);
      return -1;
   }
   BYTE coff[24]; // PE sig(4) + COFF header(20)

   if (fread(coff, 1, 24, f) != 24 || coff[0] != 'P' || coff[1] != 'E')
   {
      fclose(f);
      return -1;
   }
   WORD nsec = (WORD)(coff[6] | (coff[7] << 8)),
        optsz = (WORD)(coff[20] | (coff[21] << 8));

   if (fseek(f, (long)(peoff + 24 + optsz), SEEK_SET) != 0) // to section table
   {
      fclose(f);
      return -1;
   }
   /* Collect BOTH .text (codegen) AND .rdata (string literals / const tables) — behaviour is determined by
      code AND its constants, so hashing .text alone let a .rdata-only change (e.g. an embedded JSON descriptor
      or a literal) sail through as a false cache HIT. Hash them in a FIXED order (.text then .rdata) so the
      SHA is stable. Cacheable iff .text was found (rc=0); .rdata is folded when present. */
   DWORD tPtr = 0,
         tSz  = 0,
         rPtr = 0,
         rSz  = 0;

   for (WORD i = 0; i < nsec; i++)
   {
      BYTE sh[40];

      if (fread(sh, 1, 40, f) != 40)
         break;

      DWORD rawsz = (DWORD)sh[16] | ((DWORD)sh[17] << 8) |
                       ((DWORD)sh[18] << 16) | ((DWORD)sh[19] << 24);
      DWORD rawptr = (DWORD)sh[20] | ((DWORD)sh[21] << 8) |
                        ((DWORD)sh[22] << 16) | ((DWORD)sh[23] << 24);

      if (memcmp(sh, ".text\0\0\0", 8) == 0)
      {
         tPtr = rawptr;
         tSz  = rawsz;
      }
      else if (memcmp(sh, ".rdata\0\0", 8) == 0)
      {
         rPtr = rawptr;
         rSz  = rawsz;
      }
   }

   int   rc          = -1;
   DWORD secPtr[2]   = {tPtr, rPtr},
         secSz[2]    = {tSz, rSz};

   for (int s = 0; s < 2; s++)
   {
      if (secPtr[s] == 0 || secSz[s] == 0 || fseek(f, (long)secPtr[s], SEEK_SET) != 0)
         continue;

      BYTE  buf[8192];
      DWORD left = secSz[s];
      int   ok   = 1;

      while (left > 0)
      {
         size_t want = left < sizeof(buf) ? left : sizeof(buf),
                got  = fread(buf, 1, want, f);

         if (got == 0)
         {
            ok = 0;
            break;
         }
         abtcSHAUpdate(sha, buf, got);
         left -= (DWORD)got;
      }
      if (s == 0 && ok) // .text hashed cleanly → the run is cacheable (.rdata is a bonus determinant)
         rc = 0;
   }
   fclose(f);
   return rc;
}

// -- engine -------------------------------------------------------------------------------

typedef struct TAbTestCache
{
   BYTE sha[32];

   char shaHex[65],
        cacheFile[1024];
   int hit,
       cachedCode,
       active,     // 1 if caching is in effect (own .text hashed ok)
       code_,      // RAII: the committed result, persisted by the destructor
       committed_, // RAII: 1 once commit() has been called
       raii_;      // RAII: 1 if built via the (argc,argv,...) ctor -> dtor auto-persists

#ifdef __cplusplus
   /* RAII front-end (see file header). Default ctor = the legacy free-function flow
    * (raii_ stays 0 -> the dtor is inert; caller drives abTestBegin/abTestEnd). The
    * (argc,argv,...) ctor computes the SHA + cache lookup; on a miss the caller runs
    * the test and calls commit(code), and the dtor persists it on scope exit. */
   TAbTestCache() { memset(this, 0, sizeof(*this)); }
   TAbTestCache(int argc, LPSTR *argv, LPCSTR const *inputs = 0, LPCSTR const *externals = 0, LPCSTR extra = 0);

   ~TAbTestCache();

   bool isHit() const { return hit != 0; }       // HIT -> caller returns code()
   int code() const { return cachedCode; }        // the cached exit code (valid on a hit)

   /* Nonce - THE seed for any per-run value a test would otherwise take from the clock or an RNG.
      It is this run's cache key: 64 hex chars fingerprinting the exe, argv and the declared inputs,
      or "" when caching is off (then the caller may fall back to a clock).

      🔴 WHY THIS EXISTS. A cache MEMOIZES A FUNCTION. If a test's result depends on something the key
      does not cover - a random LID, a time(NULL) payload, thread scheduling - it is not a function, and
      the cache faithfully freezes whichever verdict one run happened to produce and replays it forever.
      That reads exactly like a flaky test and is nearly impossible to tell apart from one. MEASURED
      2026-09-11: testRouteImageQ1planesweep Lena/G, identical binary and argv, cache cleared each time,
      came out 0,1,0,0,1,1,1,0 - because the test injects a clock-seeded MING payload, so every run
      watermarks a DIFFERENT image and a marginal case lands either side of the line.

      Seeding from this instead keeps the property such a nonce is there for - it still differs whenever
      the code, the arguments or the inputs differ, so a stale artifact from an older build can never be
      mistaken for a fresh result - while making the run a function of its declared inputs. Same build,
      same case, same answer; different build, different nonce. */
   LPCSTR Nonce() const { return active ? shaHex : ""; }
   int commit(int c) // return tc.commit(rc); dtor stores it
   {
      code_ = c;
      committed_ = 1;
      return c;
   }
#endif
} TAbTestCache;

//--------------------------------------------------------------------------------
static inline void abtcToHex(const BYTE h[32], char out[65])
{
   static const char d[] = "0123456789abcdef";

   for (int i = 0; i < 32; i++)
   {
      out[i*2] = d[h[i] >> 4];
      out[i*2 + 1] = d[h[i] & 15];
   }
   out[64] = 0;
}

/* inputs / externals: NULL-terminated arrays, or NULL. Each externals[] entry is
   "exepath|commandline"; the exe's .text and the cmdline string both enter the SHA.
   Returns 1 on cache HIT (tc->cachedCode set); 0 on miss (run the test). */

//--------------------------------------------------------------------------------
static inline int abTestBegin(TAbTestCache *tc, int argc, LPSTR *argv,
                              LPCSTR const *inputs, LPCSTR const *externals, LPCSTR extra)
{
   memset(tc, 0, sizeof(*tc));

   TAbtcSHA sha;

   abtcSHAInit(&sha);

   char selfPath[1024] = {0};
#if defined(_WIN32)
   DWORD n = GetModuleFileNameA(NULL, selfPath, (DWORD)sizeof(selfPath));
   if (n == 0 || n >= sizeof(selfPath))
      return 0; // cannot locate self -> no caching, always run
#else
   return 0;
#endif
   if (abtcHashPeText(&sha, selfPath) != 0)
      return 0; // cannot read own .text -> no caching

   // command line
   for (int i = 1; i < argc; i++)
   {
      abtcSHAUpdate(&sha, argv[i], strlen(argv[i]));
      abtcSHAUpdate(&sha, "\x1f", 1);
   }
   // declared input files (content)
   if (inputs)
      for (int i = 0; inputs[i]; i++)
      {
         abtcSHAUpdate(&sha, inputs[i], strlen(inputs[i]));
         if (abtcHashFileMeta(&sha, inputs[i]) != 0) // size+mtime (near-instant), not content
            return 0; // a declared input is unreadable -> don't risk a stale cache
      }
   if (extra)           // caller feature fingerprint (e.g. TFrameCodec::FeatureDescriptor): a field/model/cs* change flips the SHA even when the exe .text does not relink
      abtcSHAUpdate(&sha, extra, strlen(extra));
   // declared external apps: exe .text + cmdline
   if (externals)
      for (int i = 0; externals[i]; i++)
      {
         LPCSTR spec = externals[i],
                bar = strchr(spec, '|');

         char exep[1024];
         size_t el = bar ? (size_t)(bar - spec) : strlen(spec);

         if (el >= sizeof(exep))
            return 0;
         memcpy(exep, spec, el);
         exep[el] = 0;
         if (abtcHashPeText(&sha, exep) != 0)
            return 0;
         if (bar)
            abtcSHAUpdate(&sha, bar + 1, strlen(bar + 1));
      }

   abtcSHAFinal(&sha, tc->sha);
   abtcToHex(tc->sha, tc->shaHex);
   tc->active = 1;

   // cache file = <selfdir>/.abtestcache/<selfbase>_<first 12 hex>.cache
   char dir[1024];

   snprintf(dir, sizeof(dir), "%s", selfPath);

   LPSTR slash = strrchr(dir, '\\'),
         fwd = strrchr(dir, '/');

   if (fwd > slash)
      slash = fwd;

   LPCSTR base = slash ? slash + 1 : dir;
   char baseCopy[256];

   snprintf(baseCopy, sizeof(baseCopy), "%s", base);
   if (slash)
      *slash = 0;
   else
      dir[0] = 0;

   char cdir[1100];

   snprintf(cdir, sizeof(cdir), "%s%s.abtestcache", dir, dir[0] ? "\\" : "");
#if defined(_WIN32)
   CreateDirectoryA(cdir, NULL);
#endif
   snprintf(tc->cacheFile, sizeof(tc->cacheFile), "%s\\%s_%.12s.cache", cdir, baseCopy, tc->shaHex);

   // look up
   FILE *cf = fopen(tc->cacheFile, "rb");

   if (cf)
   {
      char line[256] = {0};
      if (fgets(line, sizeof(line), cf) && strncmp(line, "sha=", 4) == 0)
      {
         char hex[65] = {0};
         int i = 0;

         for (; i < 64 && line[4 + i] && line[4 + i] != ' '; i++)
            hex[i] = line[4 + i];
         hex[i] = 0;

         LPCSTR cp = strstr(line, "code=");

         if (i == 64 && cp && strcmp(hex, tc->shaHex) == 0)
         {
            tc->hit = 1;
            tc->cachedCode = atoi(cp + 5);
         }
      }
      fclose(cf);
   }
   if (tc->hit)
   {
      printf("[abtestcache] HIT %s -> exit %d (skipped, SHA %.12s)\n",
             baseCopy, tc->cachedCode, tc->shaHex);
      fflush(stdout);
      return 1;
   }
   return 0;
}

// Wide-argv (wmain / UNICODE) variant: flattens argv to narrow ASCII then delegates.

//--------------------------------------------------------------------------------
static inline int abTestBeginW(TAbTestCache *tc, int argc, wchar_t **argv,
                               LPCSTR const *inputs, LPCSTR const *externals, LPCSTR extra)
{
   static LPSTR narrow[64];
   static char store[8192];
   size_t off = 0;
   int n = (argc < 64) ? argc : 64;

   for (int i = 0; i < n; i++)
   {
      narrow[i] = store + off;

      size_t j = 0;

      for (; argv[i][j] && off + j + 1 < sizeof(store); j++)
         store[off + j] = (char)((unsigned)argv[i][j] & 0xFF);
      store[off + j] = 0;
      off += j + 1;
   }
   return abTestBegin(tc, n, narrow, inputs, externals, extra);
}

// Store SHA -> exit code (only when caching is active). Returns `code` unchanged.

//--------------------------------------------------------------------------------
static inline int abTestEnd(TAbTestCache *tc, int code)
{
   if (tc->active)
   {
      FILE *cf = fopen(tc->cacheFile, "wb");

      if (cf)
      {
         fprintf(cf, "sha=%s code=%d\n", tc->shaHex, code);
         fclose(cf);
      }
   }
   return code;
}

#ifdef __cplusplus
// -- RAII front-end: ctor does begin(), dtor does end() on a committed result. ---------------------

inline TAbTestCache::TAbTestCache(int argc, LPSTR *argv, LPCSTR const *inputs, LPCSTR const *externals, LPCSTR extra)
{
   abTestBegin(this, argc, argv, inputs, externals, extra); // memsets, then fills sha/hit/cachedCode/active/cacheFile
   raii_ = 1;                                                // set AFTER (abTestBegin zeroed the struct)
}

inline TAbTestCache::~TAbTestCache()
{
   if (raii_ && committed_)
      abTestEnd(this, code_); // persist SHA -> exit code on scope exit (every path after commit)
}
#endif

#endif
