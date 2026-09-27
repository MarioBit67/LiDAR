/*
   abNew.cpp — global operator new / delete routed through the ONE shared pool.

   Every allocation in the monorepo goes through abAlloc/abFree (alloc.h) -> poolShared.
   Once a translation unit is C++, an unqualified new/delete must NOT reach the CRT heap, or
   it split-brains the heap against the pool — the same allocator-mismatch class the C
   malloc/free poison forbids. So we REPLACE the global allocation functions: every
   new / new[] / delete / delete[], including the sized, over-aligned, and nothrow overloads,
   forwards to the pool. One heap exists: the shared pool.

   C++-only TU introduced by the C++20 migration. It intentionally calls the pool (never
   libc), so it does not include libDiscipline.h — there is nothing here to poison.
*/
#define csPoolOwner // this TU IS the pool owner (operator new/delete) -> reach TPool's private members
#include <new>
#include <cstring>
#include "alloc.h"

/*--------------------------------------------------------------------------------
   Pool allocation core — always ZEROED (so every C++-allocated object starts on cleared memory before its
   constructor runs) and THREAD-TRACKED: new/new[] route through thdCalloc / thdAllocAligned so the
   per-thread GC ring owns every heap object, and delete/delete[] route through thdFree (free + untrack).
   A zero-size request still yields a unique non-null block — a new expression must never return null for
   size 0. Over-aligned requests (C++17 aligned new) use the pool's aligned tracked path + an explicit
   zero-fill (thdAllocAligned does not zero; the default block is already 64-byte aligned, covering
   align <= 64).

   Thread-tracking means an object is reclaimed by its allocating thread's per-thread reset (TThread's
   run() fires TThdPoolSpan at thread end). Any object that must OUTLIVE that reset — e.g. handed to
   another thread — MUST be pinned: use TAlloc<T> (or an explicit TPool::Lock), the sanctioned outliving
   mechanism. (An earlier build parked new on the untracked global pool to dodge this cross-thread
   use-after-free; it is back on the tracked path now that TAlloc/Lock make the outliving contract explicit.)
  --------------------------------------------------------------------------------*/
static LPVOID abNewAlloc(std::size_t size, std::size_t align)
{
   std::size_t n = size ? size : 1;

   if (align)
   {
      LPVOID p = TPool::thdAllocAligned(n, align);

      if (p)
         memset(p, 0, n);
      return p;
   }
   return TPool::thdCalloc(n, 1);
}

/*--------------------------------------------------------------------------------
   OOM policy (codebase choice): EVERY operator new - throwing form included - simply RETURNS NULL on pool
   exhaustion. No throw (no std::bad_alloc, no /EHa unwind dependency), no abort. Pool exhaustion means the
   process is doomed; callers guard their allocations explicitly (TMemAlloc `if (!m)`, plain `if (!p)`). So
   there is no separate throwing path - all overloads share abNewAlloc.
  --------------------------------------------------------------------------------*/

//--------------------------------------------------------------------------------
LPVOID operator new(std::size_t size)
{
   return abNewAlloc(size, 0);
}

//--------------------------------------------------------------------------------
LPVOID operator new[](std::size_t size)
{
   return abNewAlloc(size, 0);
}

//--------------------------------------------------------------------------------
LPVOID operator new(std::size_t size, std::align_val_t align)
{
   return abNewAlloc(size, static_cast<std::size_t>(align));
}

//--------------------------------------------------------------------------------
LPVOID operator new[](std::size_t size, std::align_val_t align)
{
   return abNewAlloc(size, static_cast<std::size_t>(align));
}

//--------------------------------------------------------------------------------
LPVOID operator new(std::size_t size, const std::nothrow_t &) noexcept
{
   return abNewAlloc(size, 0);
}

//--------------------------------------------------------------------------------
LPVOID operator new[](std::size_t size, const std::nothrow_t &) noexcept
{
   return abNewAlloc(size, 0);
}

//--------------------------------------------------------------------------------
LPVOID operator new(std::size_t size, std::align_val_t align, const std::nothrow_t &) noexcept
{
   return abNewAlloc(size, static_cast<std::size_t>(align));
}

//--------------------------------------------------------------------------------
LPVOID operator new[](std::size_t size, std::align_val_t align, const std::nothrow_t &) noexcept
{
   return abNewAlloc(size, static_cast<std::size_t>(align));
}

//--------------------------------------------------------------------------------
void operator delete(LPVOID p) noexcept
{
   TPool::thdFree(p);
}

//--------------------------------------------------------------------------------
void operator delete[](LPVOID p) noexcept
{
   TPool::thdFree(p);
}

//--------------------------------------------------------------------------------
void operator delete(LPVOID p, std::size_t) noexcept
{
   TPool::thdFree(p);
}

//--------------------------------------------------------------------------------
void operator delete[](LPVOID p, std::size_t) noexcept
{
   TPool::thdFree(p);
}

//--------------------------------------------------------------------------------
void operator delete(LPVOID p, std::align_val_t) noexcept
{
   TPool::thdFree(p);
}

//--------------------------------------------------------------------------------
void operator delete[](LPVOID p, std::align_val_t) noexcept
{
   TPool::thdFree(p);
}

//--------------------------------------------------------------------------------
void operator delete(LPVOID p, std::size_t, std::align_val_t) noexcept
{
   TPool::thdFree(p);
}

//--------------------------------------------------------------------------------
void operator delete[](LPVOID p, std::size_t, std::align_val_t) noexcept
{
   TPool::thdFree(p);
}

//--------------------------------------------------------------------------------
void operator delete(LPVOID p, const std::nothrow_t &) noexcept
{
   TPool::thdFree(p);
}

//--------------------------------------------------------------------------------
void operator delete[](LPVOID p, const std::nothrow_t &) noexcept
{
   TPool::thdFree(p);
}

//--------------------------------------------------------------------------------
void operator delete(LPVOID p, std::align_val_t, const std::nothrow_t &) noexcept
{
   TPool::thdFree(p);
}

//--------------------------------------------------------------------------------
void operator delete[](LPVOID p, std::align_val_t, const std::nothrow_t &) noexcept
{
   TPool::thdFree(p);
}
