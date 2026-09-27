#ifndef WINTYPES_H
#define WINTYPES_H

/* Cross-platform Windows-notation integer / pointer typedefs (winint discipline, goldenRule 35).
   ppCheck rewrites the TOKENS (uint8_t -> BYTE, uint32_t -> DWORD, void * -> LPVOID, ...); THIS header
   DEFINES them, so a migrated .h resolves the names even before libDiscipline.h (the "last include") is
   pulled. On Windows every typedef is byte-identical to <windows.h>, so a TU that also includes windows.h
   sees ONE consistent set (a redundant identical typedef is legal; a differing DWORD would conflict).
   On Linux the same names map to the fixed-width <stdint.h> equivalents. Lean by design: no windows.h,
   no macro pollution. Include this FIRST in any header/source that carries winint notation. */

#include <stdint.h>

#ifdef _WIN32
typedef unsigned char  BYTE;    // <windows.h>: unsigned char
typedef unsigned short WORD;    // <windows.h>: unsigned short
typedef unsigned long  DWORD;   // <windows.h>: unsigned long (LLP64: 32-bit, DISTINCT from uint32_t)
typedef long           LONG;    // <windows.h>: long (LLP64: 32-bit signed, DISTINCT from int32_t)
typedef void          *LPVOID;  // <windows.h>: void *
typedef const void    *LPCVOID; // <windows.h>: const void *
typedef const char    *LPCSTR;  // <windows.h>: const char *
typedef char          *LPSTR;   // <windows.h>: char *
typedef unsigned char *LPBYTE;  // <windows.h>: BYTE *
typedef const unsigned char *LPCBYTE; // <windows.h>: CONST BYTE *
typedef unsigned short *LPWORD; // <windows.h>: WORD *
typedef unsigned long *LPDWORD; // <windows.h>: DWORD *
typedef unsigned long long  QWORD,    // 64-bit unsigned (SDK: unsigned __int64 / ULONGLONG)
                           *LPQWORD;  // QWORD *
typedef const unsigned long long *LPCQWORD; // CONST QWORD *
typedef long long           LONGLONG; // 64-bit signed (SDK: __int64 / LONGLONG) — same type MSVC gets from winnt
#else
typedef uint8_t     BYTE;
typedef uint16_t    WORD;
typedef uint32_t    DWORD;
typedef int32_t     LONG;
typedef void       *LPVOID;
typedef const void *LPCVOID;
typedef const char *LPCSTR;
typedef char       *LPSTR;
typedef BYTE    *LPBYTE;
typedef const BYTE *LPCBYTE;
typedef WORD    *LPWORD;
typedef DWORD   *LPDWORD;
typedef uint64_t     QWORD;
typedef QWORD       *LPQWORD;
typedef const QWORD *LPCQWORD;
typedef int64_t      LONGLONG; // 64-bit signed (matches the SDK's LONGLONG on the non-Windows shim path)
#endif

#endif // WINTYPES_H
