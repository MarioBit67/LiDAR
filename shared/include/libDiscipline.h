#ifndef SHARED_LIB_DISCIPLINE_H
#define SHARED_LIB_DISCIPLINE_H

/*
 * libDiscipline.h — workspace-wide library discipline header.
 *
 * Included by EVERY library .c file across all engines (abImage,
 * abDoc, abAudio, abWorker). Activates compile-time enforcement of
 * the allocator discipline and any future workspace-wide restrictions.
 *
 * INCLUDE RULES:
 *   Library .c files in src/      → MUST include this header (SHARED_LIBRARY_BUILD).
 *   Test files in test/            → MUST include this header (SHARED_TEST_BUILD): the allocator
 *                                   discipline is NOT tolerated to lapse in tests — every test
 *                                   allocation goes through the pool (abAlloc/abFree), same as the
 *                                   library, so the tests genuinely exercise the one shared pool and
 *                                   never split-brain the heap (libc malloc + pool) under threads.
 *   Tool/script files              → MUST NOT include this header.
 *   shared/src/poolRegistry.c    → defines SHARED_POOL_INTERNAL_ALLOC
 *                                   before including this header.
 *
 * BUILD SYSTEM:
 *   The CMake library target for each engine defines SHARED_LIBRARY_BUILD; the test targets define
 *   SHARED_TEST_BUILD. Either one arms ALL poisons below — allocator, fp64, and env — unconditionally.
 *   There is no per-poison arming flag and no opt-out. If a target with NEITHER define includes this
 *   header (e.g. a tool), the #error below fires immediately.
 *
 * ADDING NEW DISCIPLINES:
 *   Add new poison blocks inside this file, not in module headers, so the full workspace discipline
 *   stays auditable in one place. Poisons are unconditional in a library/test build; the sole
 *   permitted exemption is poolRegistry.c's SHARED_POOL_INTERNAL_ALLOC (the pool must reach the raw
 *   allocator). Do not add fp64/env-style opt-out flags.
 */

#if !defined(SHARED_LIBRARY_BUILD) && !defined(SHARED_TEST_BUILD)
#error "libDiscipline.h included outside a library or test build target.   \
        Define SHARED_LIBRARY_BUILD (library) or SHARED_TEST_BUILD (test) in your CMake target.   \
        Tool translation units must not include this header."
#endif

/* The fp64 ban is UNCONDITIONAL in every library/test build (we are past the guard above) — there is no
 * opt-in flag and no silent build-level exemption. Pull the FULL standard/platform header set FIRST —
 * before the poisons below redefine double/malloc/getenv — so no system header (e.g. <windows.h> ->
 * <malloc.h>, or the NDK's <time.h>/<wchar.h>) is parsed with those macros active. Include guards make
 * every later (re)include inert, so only first-party tokens get rewritten. */
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <wchar.h>
#include <math.h>
#include <time.h>
#include <limits.h>
#include <float.h>
#include <errno.h>
#include <locale.h>
#include <inttypes.h>
#if defined(_M_X64) || defined(_M_IX86) || defined(__x86_64__) || defined(__i386__)
#include <immintrin.h>
#endif
#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
/* Win32 COM/OLE-Automation/WIC SDK headers declare the platform `DOUBLE`/`DATE`
 * typedefs and use the `double` keyword directly (VARIANT.dblVal, oleauto.h and
 * wincodec.h prototypes). WIN32_LEAN_AND_MEAN strips them from <windows.h>, so a
 * COM/WIC translation unit that includes them AFTER this header would have the
 * poison rewrite `double` inside the SDK. Pull the full set in HERE, before the
 * poison; include guards make each TU's later (re)include inert. */
#include <objbase.h>
#include <oaidl.h>
#include <oleauto.h>
#include <wincodec.h>
#endif // _WIN32

/* C++20 threading backend (thread.h seam) — pulled UNCONDITIONALLY, before ANY poison,
 * because the poisons below break these too: on the NDK's libc++ <thread> drags in
 * <format>/<locale>/<ostream>, whose allocator references hit the malloc/operator-new poison; and on
 * MSVC <xkeycheck.h> #errors if `double` is already a macro. Pull them HERE, before ANY poison, so a
 * later (re)include via thread.h is inert. No exemption — every poison build (we are past the
 * library/test guard above) gets the STL threading headers parsed clean. */
#ifdef __cplusplus
#include <thread>
#include <mutex>
#include <condition_variable>
#endif

/* -- Allocator discipline -------------------------------------------
 *
 * No library translation unit may call malloc/calloc/realloc/free
 * or any aligned allocation variant directly. All allocation must
 * go through the pool facade (pool.h) or, in the case of
 * poolRegistry.c itself, through the raw allocator via the
 * SHARED_POOL_INTERNAL_ALLOC exemption below.
 *
 * Exemption: shared/src/poolRegistry.c defines
 * SHARED_POOL_INTERNAL_ALLOC before including this header.
 * No other library file may define this macro.
 */
#ifndef SHARED_POOL_INTERNAL_ALLOC

/* Pull in system allocator headers so their declarations are processed
 * before we redefine the function-like macros below.  Without this,
 * any translation unit that includes us before <stdlib.h> would get
 * garbled declarations (posixMemalign, etc.) from the macro expansion. */
#include <stdlib.h>
#include <string.h> /* declare strdup/_strdup (with their __declspec(allocator)) BEFORE poisoning them,
                     * or the function-like #define mangles ucrt string.h's own declaration (C2495). */
#ifdef __SSE2__
#include <immintrin.h>
#endif

/* Redefine allocator functions to cause a link/compile error on any call.
 * #define is used instead of #pragma GCC poison so that headers which
 * define wrapper macros (e.g. abiAlignedFree) are not themselves poisoned
 * at the token level — only actual call-sites are blocked. */
#define malloc(s) POISON_use_pool_alloc_instead
#define calloc(n, s) POISON_use_pool_alloc_instead
#define realloc(p, s) POISON_use_pool_alloc_instead
#define free(p) POISON_use_pool_free_instead
/* strdup/_strdup allocate via the CRT — a pool-freed strdup is the allocator mismatch that crashed the
 * video analyzer (psRelease on a foreign pointer). Poison both; use abwStrdup (pool-backed) instead. */
#define strdup(s) POISON_use_abwStrdup_instead
#define _strdup(s) POISON_use_abwStrdup_instead
#define alignedAlloc(a, s) POISON_use_pool_alloc_instead
#define posixMemalign(p, a, s) POISON_use_pool_alloc_instead
/* VS2026 MSVC 19.51+ defines _mm_malloc as a macro in <immintrin.h>.
 * Undef before redefining to avoid C4005 under /WX. */
#ifdef _mm_malloc
#undef _mm_malloc
#endif
#define _mm_malloc(s, a) POISON_use_pool_alloc_instead
#define _aligned_malloc(s, a) POISON_use_pool_alloc_instead
#define _aligned_free(p) POISON_use_pool_free_instead

#endif // SHARED_POOL_INTERNAL_ALLOC

/* -- FP64 discipline ------------------------------------------------
 *
 * double / fp64 is FORBIDDEN. Use float (fp32) ONLY where a Q-format integer
 * cannot equally match it; otherwise use Q-format integers.
 *
 * MSVC has no #pragma-poison for a keyword, so we redefine the `double` token
 * to an undeclared identifier — a hard compile error at any use site. This is
 * safe because every standard header is include-guarded: the double-bearing
 * std/platform headers are pulled in HERE first (above, before any poison), so
 * any later (re)include is an inert no-op and only YOUR `double` tokens get
 * rewritten.
 *
 * There is NO exemption. The ban is unconditional in every library/test build;
 * no flag turns it off and no file may opt out. Convert the `double` to float or
 * a Q-format integer — that is the only path.
 */
#define double DOUBLE_FP64_IS_POISONED__use_float_or_Qint
#define float64 FLOAT64_IS_POISONED__use_float_or_Qint

/* -- Environment discipline -----------------------------------------
 *
 * No library translation unit may call getenv directly. All environment reads route
 * through abGetEnv (abiEnv.h) so the fEnv kill-switch can force the whole codebase
 * env-free in ONE place (abGetEnv returns NULL → hardcoded production defaults; e.g.
 * abworker, which has no environment). <stdlib.h> is pulled in above (before any
 * poison), so this just rewrites first-party `getenv` tokens to a hard compile error.
 *
 * The wrapper itself (shared/src/abiEnv.c) is the one TU permitted to call getenv; it stays
 * free of this header, so it needs no exemption. There is no opt-out flag.
 */
#define getenv(s) GETENV_IS_POISONED__use_abGetEnv_instead

/* -- Future discipline slots ----------------------------------------
 *
 * FFT discipline:
 *   #ifndef SHARED_FFT_INTERNAL
 *   #ifdef __GNUC__
 *   #pragma GCC poison kiss_fft kiss_fftr
 *   #endif
 *   #endif
 *
 * RNG discipline:
 *   #ifndef SHARED_RNG_INTERNAL
 *   #ifdef __GNUC__
 *   #pragma GCC poison rand srand random srandom
 *   #endif
 *   #endif
 *
 * Logging discipline:
 *   #ifndef SHARED_LOG_INTERNAL
 *   #ifdef __GNUC__
 *   #pragma GCC poison printf fprintf
 *   #endif
 *   #endif
 *
 * Add discipline blocks here as they are approved and activated.
 * Do not add them in module headers or engine-local files.
 */

#endif // SHARED_LIB_DISCIPLINE_H
