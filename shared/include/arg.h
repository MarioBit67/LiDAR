/*
 * arg.h — command-line parameter access for tests (drop-in for getenv).
 *
 * Part of the shared/ test model. Tests take ALL parameters from the command line, never
 * from the environment: the SHA-cache engine hashes argv but cannot perfectly capture the
 * environment, so any getenv read would silently defeat the cache. Call abArgInit(argc, argv)
 * once at the top of main(), then use abArg("NAME") exactly where getenv("NAME") was:
 *   --NAME=value  -> abArg("NAME")    returns "value"
 *   --NAME        -> abArg("NAME")    returns ""        (bare flag present)
 *   (absent)      -> abArg("NAME")    returns NULL      (same as getenv miss)
 */
#ifndef ARG_H
#define ARG_H
#include "winTypes.h" // winint typedefs (BYTE/WORD/DWORD/...)

#include <string.h>
#include <stdlib.h>

static int abArgArgc = 0;
static LPSTR *abArgArgv = NULL;

//--------------------------------------------------------------------------------
static inline void abArgInit(int argc, LPSTR *argv)
{
   abArgArgc = argc;
   abArgArgv = argv;
}

/* Wide-argv programs (MSVC wmain / UNICODE entry) call this instead of abArgInit: it
   flattens the wide argv into narrow ASCII once (flag args are ASCII) so abArg/abArgW work. */

//--------------------------------------------------------------------------------
static inline void abArgInitW(int argc, wchar_t **argv)
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
   abArgArgc = n;
   abArgArgv = narrow;
}

//--------------------------------------------------------------------------------
static inline LPCSTR abArg(LPCSTR name)
{
   size_t n = strlen(name);

   for (int i = 1; i < abArgArgc; i++)
   {
      LPCSTR a = abArgArgv[i];

      if (a[0] == '-' && a[1] == '-' && strncmp(a + 2, name, n) == 0)
      {
         if (a[2 + n] == '=')
            return a + 3 + n;
         if (a[2 + n] == '\0')
            return "";
      }
   }
   return NULL;
}

//--------------------------------------------------------------------------------
static inline int abArgInt(LPCSTR name, int def)
{
   LPCSTR v = abArg(name);

   return (v && *v) ? atoi(v) : def;
}

/* Wide variant for paths consumed as wchar_t (drop-in for _wgetenv). Returns a pointer into a
   rotating static buffer (startup arg parse is single-threaded), or NULL when absent. */

//--------------------------------------------------------------------------------
static inline const wchar_t *abArgW(LPCSTR name)
{
   static wchar_t buf[1024];
   LPCSTR v = abArg(name);

   if (!v)
      return NULL;

   size_t i = 0;

   for (; v[i] && i < 1023; i++)
      buf[i] = (wchar_t)(unsigned char)v[i];
   buf[i] = 0;
   return buf;
}

#endif
