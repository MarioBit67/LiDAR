/* sha256.cpp — the canonical shared SHA-256 (see sha256.h). Lifted intact from the proof-of-work
   stamper, which is where it was first proven, and promoted here so ONE implementation serves every
   engine (goldenRule 28) on every platform — the codec ships to Android and iOS, where an OS-crypto
   hasher is not available. */

#include "sha256.h"

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h> // GetModuleFileNameA — the running image path for appSelfSHA
#endif

#include <stdio.h> // fopen/fread/snprintf — file streaming + the "(unavailable)" fallback

#include "libDiscipline.h" // allocator/fp64 poison — last include (no libc alloc / no double here)

//--------------------------------------------------------------------------------
LPCSTR appSelfSHA(LPSTR out, int cap)
{
   if (cap < 65)
   {
      if (cap > 0)
         out[0] = '\0';
      return out;
   }
   snprintf(out, (size_t)cap, "(unavailable)");

#ifdef _WIN32
   char path[1024];

   if (GetModuleFileNameA(NULL, path, (DWORD)sizeof path) != 0)
      (void)sha256HexFile(path, out); // leaves "(unavailable)" in place if the image cannot be read
#endif // _WIN32 — a mobile library has no executable of its own to fingerprint
   return out;
}

