/*
 * abPool.cpp - THE one allocator for the whole monorepo (single layer, no forwarding facade).
 *
 * This file absorbs the former TPoolShared (buddy recycling allocator) + poolBase (global bind) +
 * poolThread (per-thread GC) + abAlloc (the ab* public surface) into ONE translation unit. Every
 * subsystem allocates through the ab* functions declared in alloc.h; those operate the ONE global
 * pool directly - there is no vtable, no handle passed around, and no thin wrapper layer.
 *
 * Buddy core (concepts ported from AntiMemFrag.cpp): power-of-2 size classes (free[PS_ORDERS]). A
 * freed block is NOT returned to libc - it is RETAINED, marked not-busy, and recycled by a later
 * allocation; libc Free() happens only at destroy. Each block carries a FIXED 64-byte prefix, so the
 * user pointer is always (block + PS_HEADER) and the prefix is recovered as (ptr - PS_HEADER) - a
 * constant offset, no back-pointer slot. Reference-counted in a GC sense: alloc = refCount 1, abLock
 * = ++, abFree = if (!--) recycle; abReset reclaims every unpinned in-use block.
 *
 * Per-thread GC (formerly poolThread) is now a SENTINEL-HEADED CIRCULAR doubly-linked ring threaded
 * through two header links (the embedded TThdLink) - PURE membership; the global buddy table keeps all
 * control (refCount / PS_IN_USE / order / chunk). Track = O(1) splice after the sentinel; free = O(1)
 * head-agnostic splice (any order, cross-thread safe under the pool mutex, no head/owner pointer);
 * abThrReset = O(1)/node walk reclaiming the unpinned (refCount==1) blocks and detaching the abLock'd
 * survivors, leaving the ring empty (so no block references a dead thread's sentinel). This replaces
 * the former O(n) linear-scan swap-remove table.
 *
 * malloc/free are used legitimately here (this IS the allocator) - SHARED_POOL_INTERNAL_ALLOC exempts
 * the file from the libDiscipline allocator poison. The only OS dependency is the mutex (the Linux
 * port swaps just that). C++20 (codebase standard); PrettyPrint for every authored line.
 */
#define SHARED_POOL_INTERNAL_ALLOC
#define csPoolOwner // pool-owner TU: this file DEFINES TPool's private methods -> must share the owner's
                    /* all-public view so its symbols mangle to match abNew.cpp's calls (MSVC encodes access
                       in the mangled name). Rule 37: csPoolOwner is the two pool-owner TUs (this + abNew.cpp). */
#ifndef SHARED_LIBRARY_BUILD // portable: consumers (abDoc/abAudio/...) compile shared/src without the target define
#define SHARED_LIBRARY_BUILD
#endif

#include <stdlib.h>
#include <string.h>
#include "alloc.h"
#include "libDiscipline.h" // allocator poison (last); this file is exempt via the macro above
#ifdef ABC_MEMTAG
#include <stdio.h> // peak-memory profiler: track-file dump (Windows-only diagnostic build)
#endif

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#else
#include <pthread.h>
#endif

enum {
   psOrders     = 48, // buddy size classes 0..47 (a block spans 1<<order bytes); 1<<47 ceiling
   psChunkOrder = 20  // default chunk = 1 MiB; buddies coalesce within a chunk
};

// next-field sentinel: the block is allocated (a free block's next is a free-list link or NULL)
#define PS_IN_USE ((TPsBlock *)~(size_t)0u)

/* Fixed block header: the TPsBlock prefix padded up to one cache line. The user pointer is ALWAYS
 * (block + PS_HEADER), so recovery is `ptr - PS_HEADER` - a constant offset, no back-pointer slot.
 * 64 = one cache line and the practical SIMD maximum (AVX-512); production callers request at most
 * 32-byte alignment, so a 64-aligned block satisfies all of them. The prefix (5 base fields + the
 * 2-pointer ring link) is 56 bytes, inside the 64. Under ABC_MEMTAG the profiler's tag fields push
 * it past 64, so the diagnostic build widens the header to 128 (only that non-shipping build). */
#ifdef ABC_MEMTAG
#define PS_HEADER ((size_t)128u)
#else
#define PS_HEADER ((size_t)64u)
#endif

typedef struct TPsChunk {
   struct TPsChunk *next; // chunk list (destroy frees these)
   LPVOID raw;           // the unaligned malloc result (freed on destroy); `base` is raw rounded to 64
   LPSTR base;          // buddy region start (64-aligned): 1<<order bytes, fully partitioned to blocks
   size_t order;         // chunk spans 1<<order bytes
} TPsChunk;

/* Per-thread ring link. The per-thread live-list is a SENTINEL-HEADED CIRCULAR doubly-linked ring:
 * membership ONLY - the global buddy table keeps all control (refCount / PS_IN_USE / order / chunk).
 * Insert = splice after the sentinel; remove = a pure head-agnostic splice (never touches a head
 * pointer or an owner back-pointer), so a cross-thread free is innocuous under the pool lock. */
typedef struct TThdLink {
   struct TThdLink *prev,
                   *next;
} TThdLink;

typedef struct TPsBlock {
   struct TPsBlock *next;     // free[order] link when free; PS_IN_USE when allocated
   struct TPsChunk *chunk;    // owning chunk (buddy base + bounds)
   size_t          order,    // block spans 1<<order bytes
                   refCount, // GC: alloc=1, lock=++, release=if(!--)recycle
                   align;    // alignment used (preserved across realloc)
   TThdLink        thd;      // per-thread ring membership (prev/next); self-linked when off-ring
#ifdef ABC_MEMTAG
   LPCSTR tagFile; // alloc-site __FILE__ (static literal, pointer stored - no copy). ABC_MEMTAG only.
   int         tagLine; // alloc-site __LINE__. Fits the spare room in the widened 128-byte header.
#endif
} TPsBlock;

/* The prefix MUST fit inside the fixed header, or its fields would overlap the user's bytes (user =
 * block + PS_HEADER). This guards the ring links + the ABC_MEMTAG tag against a silent overflow that
 * corrupts the returned memory. */
static_assert(sizeof(TPsBlock) <= PS_HEADER, "TPsBlock prefix must fit within PS_HEADER");

// Recover the owning block from an embedded ring link.
#define blockOfLink(lp) ((TPsBlock *)(LPVOID)((LPSTR)(lp) - offsetof(TPsBlock, thd)))

struct TPoolShared {
   TPsBlock *free[psOrders]; // per-order free-lists of RETAINED (recyclable) blocks
   TPsChunk *chunks;         // all chunks (destroy frees these)
   size_t   align;          // default alignment when a call requests none
#ifdef ABC_MEMTAG
   int pendGrew; // set when a new chunk was just added (arena high-water) -> dump the peak after the alloc
#endif
#ifdef _WIN32
   CRITICAL_SECTION mtx;
#else
   pthread_mutex_t mtx;
#endif
};

// -- mutex helpers (the only OS-specific code) -----------------------------
#ifdef _WIN32
#define PS_LOCK(p) EnterCriticalSection(&(p)->mtx)
#define PS_UNLOCK(p) LeaveCriticalSection(&(p)->mtx)
#else
#define PS_LOCK(p) pthread_mutex_lock(&(p)->mtx)
#define PS_UNLOCK(p) pthread_mutex_unlock(&(p)->mtx)
#endif

/* THE one pool — TPool's private static member: statically created, opaque, process-long-lasting. It
   always exists; EnsureReady() does the one-time mutex+free-list init (gReady) lazily on first use. */
TPoolShared TPool::gPool;
bool       TPool::gReady = false;

#ifdef _WIN32
#define AB_TLS __declspec(thread)
#else
#define AB_TLS __thread
#endif

/* Opaque per-thread GC ring (membership only; the global buddy table keeps all control). TLS state — the
 * sentinel + a lazy-ready flag — is zero-initialised, so init() is the SOLE fReady owner: idempotent,
 * fired by every tracked alloc; the first call on a thread arms the empty ring, the rest are a flag test.
 * No explicit thread-init anywhere. Caller holds the pool lock. */
static int thdRingArmExit(void); // fwd: thread-exit drain hook (defined after Drain)

class TThdRing {
public:
   static void Init(void) // sole fReady manager: arm the empty ring once, then a cheap no-op
   {
      if (gReady)
         return;
      thdRingArmExit(); // same moment the ring is armed, arm the exit drain that ends its ownership
      gRing.prev = &gRing;
      gRing.next = &gRing;
      gReady     = true;
   }
   static TThdLink *Head(void) { return &gRing; } // sentinel, for the reset walks

   static void Link(TPsBlock *b);      // Init() + splice a fresh block after the sentinel (O(1))
   static void SpliceOut(TPsBlock *b); // head-agnostic remove (safe cross-thread; no-op if off-ring)
   static void Drain(void);            // thread exit: end this ring's ownership of every survivor

private:
   static AB_TLS TThdLink gRing;
   static AB_TLS bool     gReady;
};

AB_TLS TThdLink TThdRing::gRing;   // zero-init: prev/next NULL until init() arms it
AB_TLS bool     TThdRing::gReady;  // zero-init: false until the first tracked alloc

#ifdef ABC_MEMTAG
/* -- peak-memory profiler (ABC_MEMTAG - Windows diagnostic build only) -----------------------------------
 * _abAlloc stamps each block's alloc site via psTag (thread-local). When an alloc forces a NEW chunk (the
 * arena reaches a new high-water - the running-counter approach can't survive buddy split/coalesce/reset,
 * so we trigger on chunk growth instead), we aggregate the LIVE blocks by (file:line) and rewrite
 * abcMemPeak.log - the exact peak composition is always on disk (survives a later crash) for offline
 * evaluation. Never compiled in release. */
#if defined(_MSC_VER)
#define PS_TLS __declspec(thread)
#else
#define PS_TLS _Thread_local
#endif
static PS_TLS LPCSTR gTagFile = 0;
static PS_TLS int gTagLine = 0;

//--------------------------------------------------------------------------------
static void psTag(LPCSTR file, int line)
{
   gTagFile = file;
   gTagLine = line;
}

//--------------------------------------------------------------------------------
static void psSnapshotPeak(TPoolShared *p)
{
   typedef struct {
      LPCSTR f;
      int         l;
      size_t      bytes;
      unsigned    n;
   } Ent;
   static Ent tbl[256];
   int    nt = 0;
   size_t inUseTotal = 0,    // sum of in-use block spans at this moment (walked = ground truth)
          reservedTotal = 0; // sum of chunk spans = arena high-water (what the OS holds)

   TPsChunk *c;

   for (c = p->chunks; c; c = c->next) // aggregate every in-use block by (file,line)
   {
      LPSTR q   = c->base,
           end = c->base + ((size_t)1u << c->order);

      reservedTotal += (size_t)1u << c->order;
      while (q < end)
      {
         TPsBlock *b  = (TPsBlock *)q;
         size_t   sp = (size_t)1u << b->order;

         q += sp;
         if (b->next != PS_IN_USE)
            continue;
         inUseTotal += sp;

         int i;

         for (i = 0; i < nt; i++)
            if (tbl[i].f == b->tagFile && tbl[i].l == b->tagLine)
               break;
         if (i == nt && nt < 256)
         {
            tbl[nt].f     = b->tagFile;
            tbl[nt].l     = b->tagLine;
            tbl[nt].bytes = 0;
            tbl[nt].n     = 0;
            nt++;
         }
         if (i < 256)
         {
            tbl[i].bytes += sp;
            tbl[i].n++;
         }
      }
   }

   FILE *fp = fopen("abcMemPeak.log", "w");

   if (!fp)
      return;
   fprintf(fp, "ARENA high-water = %zu KB reserved (%zu MB) | in-use now = %zu KB (%zu MB) across %d call sites\n",
           reservedTotal >> 10, reservedTotal >> 20, inUseTotal >> 10, inUseTotal >> 20, nt);
   for (int a = 0; a < nt; a++) // selection sort by bytes desc, then print basename:line
   {
      int mx = a;

      for (int b = a + 1; b < nt; b++)
         if (tbl[b].bytes > tbl[mx].bytes)
            mx = b;

      Ent t = tbl[a];

      tbl[a]  = tbl[mx];
      tbl[mx] = t;

      LPCSTR f  = tbl[a].f ? tbl[a].f : "?",
                 bn = f;

      for (LPCSTR s = f; *s; s++)
         if (*s == '/' || *s == '\\')
            bn = s + 1;
      fprintf(fp, "  %8zu KB  x%-5u  %s:%d\n", tbl[a].bytes >> 10, tbl[a].n, bn, tbl[a].l);
   }
   fclose(fp);
}
#endif

//--------------------------------------------------------------------------------
static size_t psRoundUp(size_t v, size_t a)
{
   if (a <= 1u)
      return v;
   return (v + (a - 1u)) & ~(a - 1u);
}

/*--------------------------------------------------------------------------------
   Smallest order whose block (1<<order) holds the fixed header plus the payload. Alignment does not
   enter the size: every block start is 64-aligned, so user = block+PS_HEADER meets any request <= 64.
  --------------------------------------------------------------------------------*/
static size_t psOrderOf(size_t size, size_t align)
{
   size_t need;

   (void)align; // alignment is satisfied structurally (aligned blocks), not by extra slack

   need = PS_HEADER + size;

   size_t k = 0u;

   while (((size_t)1u << k) < need)
      k++;
   return k;
}

//--------------------------------------------------------------------------------
// Recover the prefix from a user pointer: it sits exactly PS_HEADER bytes before the user bytes.

//--------------------------------------------------------------------------------
static TPsBlock *psBlockOf(LPVOID userPtr)
{
   return (TPsBlock *)(LPVOID)((LPSTR)userPtr - PS_HEADER);
}

//--------------------------------------------------------------------------------
// Remove `target` from its order's free-list; return 1 if it was present.

//--------------------------------------------------------------------------------
static int psUnlinkFree(TPoolShared *p, size_t order, TPsBlock *target)
{
   TPsBlock **pp = &p->free[order];

   while (*pp)
   {
      if (*pp == target)
      {
         *pp = target->next;
         return 1;
      }
      pp = &(*pp)->next;
   }
   return 0;
}

//--------------------------------------------------------------------------------
// Malloc a fresh chunk of `order` bytes (a single free block of that order). Caller holds the lock.

//--------------------------------------------------------------------------------
static int psNewChunk(TPoolShared *p, size_t order)
{
   TPsChunk *c;
   TPsBlock *b;
   LPVOID raw;

   if (order >= psOrders)
      return 0;
   c = (TPsChunk *)malloc(sizeof(TPsChunk));
   if (!c)
      return 0;
   /* over-allocate by one header so the buddy region can start on a 64-aligned boundary; every block
    * start is then a multiple of its own (>= header) size from an aligned base, hence itself aligned. */
   raw = malloc(((size_t)1u << order) + PS_HEADER);
   if (!raw)
   {
      free(c); // libc: pool internals malloc/free chunks directly (SHARED_POOL_INTERNAL_ALLOC exempt)
      return 0;
   }
   c->raw     = raw;
   c->base    = (LPSTR)psRoundUp((size_t)raw, PS_HEADER);
   c->order   = order;
   c->next    = p->chunks;
   p->chunks  = c;
#ifdef ABC_MEMTAG
   p->pendGrew = 1; // arena grew -> the next alloc's completion dumps the peak composition (profiler)
#endif

   b = (TPsBlock *)c->base; // the whole region is one free block of `order`
   b->chunk    = c;
   b->order    = order;
   b->refCount = 0u;
   b->align    = 0u;
   b->next     = p->free[order];
   p->free[order] = b;
   return 1;
}

//--------------------------------------------------------------------------------
// Ensure free[order] is non-empty by splitting a larger free block (or carving a new chunk).

//--------------------------------------------------------------------------------
static int psEnsure(TPoolShared *p, size_t order)
{
   size_t k;

   if (p->free[order])
      return 1;
   for (k = order + 1u; k < psOrders && !p->free[k]; k++)
   {
   }
   if (k >= psOrders)
   {
      size_t co = order > (size_t)psChunkOrder ? order : (size_t)psChunkOrder;

      if (!psNewChunk(p, co))
         return 0;
      for (k = co; k < psOrders && !p->free[k]; k++)
      {
      }
      if (k >= psOrders)
         return 0;
   }
   // split the order-k block down to `order`, pushing each freed buddy onto its class
   while (k > order)
   {
      TPsBlock *b = p->free[k],
               *buddy;

      p->free[k] = b->next; // pop
      k--;
      buddy = (TPsBlock *)((LPSTR)b + ((size_t)1u << k));
      b->order        = k;
      buddy->order    = k;
      buddy->chunk    = b->chunk;
      buddy->refCount = 0u;
      buddy->align    = 0u;
      buddy->next     = p->free[k];
      p->free[k]      = buddy;
      b->next         = p->free[k];
      p->free[k]      = b;
   }
   return p->free[order] != NULL;
}

//--------------------------------------------------------------------------------
// Mark a block free, coalesce with its buddy as far as possible, then push onto its class.

//--------------------------------------------------------------------------------
static void psFreeBlock(TPoolShared *p, TPsBlock *b)
{
   size_t   order = b->order;
   TPsChunk *c     = b->chunk;

   for (;;)
   {
      size_t   off   = (size_t)((LPSTR)b - c->base);
      TPsBlock *buddy = (TPsBlock *)(c->base + (off ^ ((size_t)1u << order)));

      if (order >= c->order) // reached the chunk's top order
         break;
      if (buddy->next == PS_IN_USE || buddy->order != order)
         break; // buddy busy or subdivided
      if (!psUnlinkFree(p, order, buddy))
         break; // buddy not actually a free block of this order
      if (buddy < b)
         b = buddy; // the merged block starts at the lower address
      order++;
      b->order = order;
   }
   b->refCount = 0u;
   b->next     = p->free[order];
   p->free[order] = b;
}

/*--------------------------------------------------------------------------------
   Splice a freshly allocated block onto the CURRENT thread's ring, right after the sentinel (O(1)).
   init() arms the ring lazily (fReady) before the first splice. Caller holds the pool lock.
  --------------------------------------------------------------------------------*/
void TThdRing::Link(TPsBlock *b)
{
   TThdLink *n = &b->thd;

   Init();
   n->prev          = &gRing;
   n->next          = gRing.next;
   gRing.next->prev = n;
   gRing.next       = n;
}

/*--------------------------------------------------------------------------------
   Splice a block OUT of whatever ring it sits on, then self-link it (off-ring). A pure head-agnostic
   splice: it never touches a head pointer, so a cross-thread free is safe under the pool lock, and it
   is a harmless no-op on an already-self-linked (off-ring) block. Caller holds the pool lock.
  --------------------------------------------------------------------------------*/
void TThdRing::SpliceOut(TPsBlock *b)
{
   TThdLink *n = &b->thd;

   n->prev->next = n->next;
   n->next->prev = n->prev;
   n->prev = n; // self-link => off-ring; a subsequent splice is a no-op
   n->next = n;
}

/*--------------------------------------------------------------------------------
   Drain - the thread is ending, so its ring must stop owning anything.

   The sentinel gRing is AB_TLS: it dies WITH the thread. A block still linked to it keeps prev/next
   pointing into that dead TLS, and the next Free - from any thread, at any later time - executes
   `n->prev->next = n->next` straight into released memory. MEASURED 2026-09-13: abworker faulted at
   exactly that store (TThdRing::SpliceOut+0xb, `mov qword ptr [rcx+8],rax`) when the MAIN thread ran
   finishJob and freed a buffer the job thread had allocated. Page heap turned the silent corruption
   into a clean fault; without it the write just poisoned the heap free-list and surfaced later as an
   unrelated AV - and when that AV landed inside TPool::Free, the unwinding thread left the pool lock
   owned by a dead thread and the whole worker deadlocked.

   SpliceOut is genuinely safe cross-THREAD (it is head-agnostic and takes the pool lock). What it could
   not be safe against is cross-LIFETIME: a neighbour that no longer exists. So membership ends here,
   while the sentinel is still alive. Every survivor is self-linked, which is the documented off-ring
   state, so a later Free is the no-op splice it already handles.

   The blocks themselves are NOT touched. A block that outlives its allocating thread is exactly the
   case this exists for: it goes on being owned by whoever holds it (a TBlock pin, another thread, the
   process) and is freed correctly later. Ending membership is the whole operation.

   Trade-off, stated plainly: unpinned scratch this thread abandoned is no longer reachable by any
   thrReset, so it stays in the pool until the block is explicitly freed. That is a leak of genuinely
   abandoned memory, and it is the correct trade - the alternative is reclaiming blocks a caller may
   still hold, which is the very class of bug this fixes.
  --------------------------------------------------------------------------------*/
void TThdRing::Drain(void)
{
   if (!gReady)
      return;

   PS_LOCK(&TPool::gPool); // Link/SpliceOut both run under it; the walk must not race a splice

   TThdLink *n = gRing.next;

   while (n != &gRing)
   {
      TThdLink *next = n->next;

      n->prev = n; // off-ring, exactly as SpliceOut leaves a removed node
      n->next = n;
      n = next;
   }
   gRing.prev = &gRing;
   gRing.next = &gRing;
   gReady     = false;
   PS_UNLOCK(&TPool::gPool);
}

/* The thread-exit hook. A function-local thread_local is constructed on the first call and destroyed
   when the thread ends - which is what AB_TLS (__declspec(thread)) cannot give us, since a raw TLS
   variable carries no destructor. Constructed from Init() on this thread's FIRST tracked allocation,
   so it is destroyed LATE in the reverse-order teardown: other thread_locals (a TBlock that frees its
   buffer, say) unwind first, while the ring is still valid, and the drain then ends whatever remains. */
class TThdRingExit {
public:
   TThdRingExit(void) : Parmed(1) {}

   ~TThdRingExit(void) { TThdRing::Drain(); }

   int Armed(void) const { return Parmed; }

private:
   int Parmed;
};

//--------------------------------------------------------------------------------
static int thdRingArmExit(void)
{
   static thread_local TThdRingExit g;

   return g.Armed();
}

// -- allocation (per-call alignment) ---------------------------------------

//--------------------------------------------------------------------------------
static LPVOID psAllocAlignedLocked(TPoolShared *p, size_t size, size_t align)
{
   size_t   a = align,
            order;

   TPsBlock *b;
   LPSTR user;

   if (a < p->align)
      a = p->align;
   if (a < sizeof(LPVOID))
      a = sizeof(LPVOID);
   if (a > PS_HEADER)
      a = PS_HEADER; // the fixed header is the max alignment it can guarantee (production max is 32)

   order = psOrderOf(size, a);
   if (order >= psOrders || !psEnsure(p, order))
      return NULL;

   b = p->free[order];
   p->free[order] = b->next; // pop
   b->next     = PS_IN_USE;  // mark allocated
   b->order    = order;
   b->refCount = 1u;
   b->align    = a;
   b->thd.prev = &b->thd; // off-ring (self-linked) unless a thd* alloc splices it in below
   b->thd.next = &b->thd;
#ifdef ABC_MEMTAG
   b->tagFile = gTagFile; // stamp the alloc site (set by _abAlloc via psTag)
   b->tagLine = gTagLine;
   if (p->pendGrew) // this alloc forced a new chunk = a new arena high-water -> dump the composition NOW
   {                // (after stamping, so the triggering block is included)
      p->pendGrew = 0;
      psSnapshotPeak(p);
   }
#endif

   user = (LPSTR)b + PS_HEADER; // fixed offset: block is aligned, so user is too - no back-pointer
   return (LPVOID)user;
}

/*--------------------------------------------------------------------------------
   True iff `addr` lies inside one of the pool's committed chunk regions. Pure validation - never
   dereferences `addr` - so it is safe to call on a foreign or wholly random pointer.
  --------------------------------------------------------------------------------*/
static int psAddrInChunks(const TPoolShared *p, LPCVOID addr)
{
   const TPsChunk *c = p->chunks;

   while (c)
   {
      if ((LPCSTR)addr >= c->base && (LPCSTR)addr < c->base + ((size_t)1u << c->order))
         return 1;
      c = c->next;
   }
   return 0;
}

/*--------------------------------------------------------------------------------
   Release a user pointer to the pool. Two range-check layers guard a foreign/random pointer (no
   dereference until the address is proven ours), and the PS_IN_USE sentinel guards a double-free of a
   real block - so an over-free is always a harmless no-op. Caller holds the pool lock.

   HARD INVARIANT: the THREAD ring and the GLOBAL buddy table are independent and NON-GATING. Once the
   block is validated as genuinely in-use, BOTH deletions run, each standing alone with no early return
   between them: (1) splice off the thread ring (pure membership; a no-op if off-ring), then (2)
   refCount-- and, at zero, recycle to the buddy free-lists (the authoritative reclamation). A no-op in
   one table never blocks the other - which is exactly what makes a cross-thread free innocuous.
  --------------------------------------------------------------------------------*/
static void psReleaseLocked(TPoolShared *p, LPVOID ptr)
{
   TPsBlock *b;

   if (!psAddrInChunks(p, ptr))
      return;
   b = psBlockOf(ptr);
   if (!psAddrInChunks(p, b))
      return;
   if (b->next != PS_IN_USE)
      return; // already free (double-free): both tables already settled - nothing to delete

   TThdRing::SpliceOut(b); // (1) ring membership delete - unconditional (no-op if off-ring)
   if (--b->refCount == 0u) // (2) buddy reclaim - independent of the ring op above
      psFreeBlock(p, b);
}

// -- lifecycle -------------------------------------------------------------

/*--------------------------------------------------------------------------------
   One-time lazy init of THE pool: ready its OS mutex + buddy free-lists, guarded by gReady — the same
   zeroed-flag arm as TThdRing::Init. gPool is a static member that ALWAYS exists (no bind, no pointer,
   nothing that can be absent or rebound); this only readies it on first use. Align is a fixed 32
   (AVX-safe, covers every engine's need). pthread failure leaves gReady false → the next alloc retries.
  --------------------------------------------------------------------------------*/
void TPool::EnsureReady(void)
{
   if (gReady)
      return;

   memset(gPool.free, 0, sizeof(gPool.free));
   gPool.chunks = NULL;
   gPool.align  = 32u;
#ifdef ABC_MEMTAG
   gPool.pendGrew = 0;
#endif
#ifdef _WIN32
   InitializeCriticalSection(&gPool.mtx);
#else
   if (pthread_mutex_init(&gPool.mtx, NULL) != 0)
      return;
#endif
   gReady = true;
}

/*--------------------------------------------------------------------------------
   Free every chunk to libc (regardless of refCount) and un-ready the pool. Tests/shutdown only; the next
   allocation re-arms via EnsureReady. Not called on the production process-lifetime pool.
  --------------------------------------------------------------------------------*/
void TPool::Destroy(void)
{
   TPsChunk *c;

   if (!gReady)
      return;
   PS_LOCK(&gPool);
   c = gPool.chunks;
   while (c)
   {
      TPsChunk *next = c->next;

      ::free(c->raw); // libc: pool internals malloc/free chunks directly (SHARED_POOL_INTERNAL_ALLOC exempt)
      ::free(c);
      c = next;
   }
   gPool.chunks = NULL;
   memset(gPool.free, 0, sizeof(gPool.free));
   PS_UNLOCK(&gPool);
#ifdef _WIN32
   DeleteCriticalSection(&gPool.mtx);
#else
   pthread_mutex_destroy(&gPool.mtx);
#endif
   gReady = false;
}

// -- the one allocation family (single shared pool) ------------------------

//--------------------------------------------------------------------------------
LPVOID TPool::Alloc(size_t size, LPCSTR file, int line)
{
   LPVOID r;

   EnsureReady();
   if (size == 0u)
      return NULL;
   PS_LOCK(&gPool);
#ifdef ABC_MEMTAG
   psTag(file, line); // stamp the next block's alloc site for the peak profiler
#else
   (void)file;
   (void)line;
#endif
   r = psAllocAlignedLocked(&gPool, size, 32u);
   PS_UNLOCK(&gPool);
   return r;
}

//--------------------------------------------------------------------------------
LPVOID TPool::AllocAligned(size_t size, size_t align, LPCSTR file, int line)
{
   LPVOID r;

   EnsureReady();
   if (size == 0u)
      return NULL;
   PS_LOCK(&gPool);
#ifdef ABC_MEMTAG
   psTag(file, line);
#else
   (void)file;
   (void)line;
#endif
   r = psAllocAlignedLocked(&gPool, size, align < 32u ? 32u : align);
   PS_UNLOCK(&gPool);
   return r;
}

//--------------------------------------------------------------------------------
LPVOID TPool::Calloc(size_t count, size_t size)
{
   size_t n;
   LPVOID r;

   EnsureReady();
   if (size != 0u && count > (size_t)-1/size) // overflow guard
      return NULL;
   n = count*size;
   if (n == 0u)
      return NULL;
   PS_LOCK(&gPool);
   r = psAllocAlignedLocked(&gPool, n, 32u);
   PS_UNLOCK(&gPool);
   if (r)
      memset(r, 0, n);
   return r;
}

//--------------------------------------------------------------------------------
LPVOID TPool::Realloc(LPVOID ptr, size_t newSize)
{
   TPsBlock *ob;
   size_t   oldAlign, oldCap, copy;
   LPVOID nw;

   EnsureReady();
   if (!ptr)
      return AllocAligned(newSize, 0u, 0, 0);

   // read old block info under a brief lock; do NOT hold it across alloc (non-recursive mutex)
   PS_LOCK(&gPool);
   ob       = psBlockOf(ptr);
   oldAlign = ob->align;
   oldCap   = ((size_t)1u << ob->order) - (size_t)((LPSTR)ptr - (LPSTR)ob); // usable bytes from ptr
   PS_UNLOCK(&gPool);

   nw = AllocAligned(newSize, oldAlign, 0, 0); // preserve the original alignment
   if (nw)
   {
      copy = oldCap < newSize ? oldCap : newSize;
      memcpy(nw, ptr, copy);

      /* SWAP the refCounts of the two blocks: the pin (a TBlock owner's refCount >= 2) rides to `nw`,
       * and the old block drops to `nw`'s fresh count of 1 — so the single Free below reclaims the old
       * block cleanly (effective deletion on the PREVIOUS block). Without this the pinned owner would be
       * orphaned onto a fresh rc-1 block (GC-reclaimable → the use-after-free the ownership model
       * prevents) and the old block would leak at rc >= 1. */
      PS_LOCK(&gPool);
      {
         size_t swap = psBlockOf(nw)->refCount;

         psBlockOf(nw)->refCount  = psBlockOf(ptr)->refCount;
         psBlockOf(ptr)->refCount = swap;
      }
      PS_UNLOCK(&gPool);

      Free(ptr); // old block: refCount 1 -> 0, reclaimed
   }
   return nw;
}

//--------------------------------------------------------------------------------
void TPool::Free(LPVOID ptr)
{
   if (!ptr)
      return;
   PS_LOCK(&gPool);
   psReleaseLocked(&gPool, ptr);
   PS_UNLOCK(&gPool);
}

//--------------------------------------------------------------------------------
LPVOID TPool::Lock(LPVOID ptr)
{
   if (!ptr)
      return ptr;
   PS_LOCK(&gPool);
   psBlockOf(ptr)->refCount++; // pin: refCount>=2 survives reset
   PS_UNLOCK(&gPool);
   return ptr; // hand back the pinned pointer so `return TPool::Lock(p)` chains the handoff
}

/*--------------------------------------------------------------------------------
   Undo Lock: the raw refcount decrement (reclaim at 0), running NO destructor — the pin's public exit. Same
   pool op as the operator-delete backend (Free); named for the pin/unpin pair so a pinned block's teardown
   is `Unlock(w)` (2->1, no dtor) then `delete w` (dtor + reclaim, 1->0). Over-unpin is harmless.
  --------------------------------------------------------------------------------*/
void TPool::Unlock(LPVOID ptr)
{
   Free(ptr);
}

//--------------------------------------------------------------------------------
size_t TPool::RefCount(LPVOID ptr)
{
   size_t rc;

   if (!ptr)
      return 0u;
   PS_LOCK(&gPool);
   rc = psBlockOf(ptr)->refCount;
   PS_UNLOCK(&gPool);
   return rc;
}

/*--------------------------------------------------------------------------------
   Locate the user base of the block that holds `p`, an interior pointer — specifically a `new T[n]`
   ELEMENT pointer, which sits a compiler array-cookie past the block base. Every user base is 64-aligned
   (block + PS_HEADER, PS_HEADER a multiple of 64), so the base is the highest 64-aligned address <= p whose
   prefix is a live in-use block that spans p. offset is set to (p - base) — the cookie size, MEASURED not
   assumed, so TBlock stays ABI-agnostic across win/linux/ios/android. Foreign/untracked p → returns p,
   offset 0 (identity: a bare pointer is its own base). PS_HEADER-aligned round-down means <= a couple of
   steps for any real cookie (8 for the common case, more only for over-aligned T).
  --------------------------------------------------------------------------------*/
LPVOID TPool::Find(LPVOID p, size_t &offset)
{
   LPSTR  cb;
   LPVOID found = NULL;
   int    guard;

   offset = 0u;
   if (!p)
      return NULL;

   PS_LOCK(&gPool);
   cb = (LPSTR)(LPVOID)((size_t)p & ~(PS_HEADER - 1u)); // candidate base: p rounded down to PS_HEADER
   for (guard = 0; guard < 16 && !found; guard++)
   {
      TPsBlock *b = psBlockOf(cb); // cb - PS_HEADER; within the same chunk, so always readable

      if (b->next == PS_IN_USE && b->order < psOrders)
      {
         LPSTR hi = cb + (((size_t)1u << b->order) - PS_HEADER); // one-past the block's user bytes

         if ((LPSTR)p >= cb && (LPSTR)p < hi)
            found = cb;
      }
      if (!found)
         cb -= PS_HEADER; // step down one alignment unit (over-aligned cookie)
   }
   PS_UNLOCK(&gPool);

   if (!found)
      return p; // untracked: identity, offset 0
   offset = (size_t)((LPSTR)p - (LPSTR)found);
   return found;
}

/*--------------------------------------------------------------------------------
   GLOBAL GC reset (orchestrator-only, job boundary) - the coarse backstop. Reclaims every unpinned
   in-use block across ALL chunks and ALL threads, coalescing buddies back into large free runs; abLock'd
   blocks (refCount>=2) survive in place. Scope contrast: abThrReset is CURRENT-THREAD-ONLY (walks just
   this thread's ring); abReset is the whole pool.

   Ring interaction: the global reclaim returns blocks to the buddy free-lists, so it must not leave a
   thread ring pointing at a recycled block. (a) It empties the CALLING thread's ring (detaching each
   node - those blocks are being reclaimed anyway). (b) OTHER threads' rings are ASSUMED QUIESCENT
   (workers thdReset per job); a live ring entry on another thread surviving a global reset is a caller
   error - same class as an unpinned block surviving (fix via abLock or reset-first) - NOT chased across
   another thread's TLS here.

   Two phases for the buddy reclaim because coalescing rewrites block orders, which would corrupt a
   simultaneous linear walk: phase 1 gathers the live blocks onto a temp list (orders intact); phase 2
   frees each (coalescing on the way).
  --------------------------------------------------------------------------------*/
void TPool::Reset(void)
{
   TPoolShared *p = &gPool;
   TPsChunk    *c;
   TPsBlock    *pending = NULL; // temp list (threaded through ->next) of blocks to re-free
   TThdLink   *n;

   if (!gReady)
      return; // nothing allocated yet -> nothing to reset
   PS_LOCK(p);

   /* (a) empty the CALLING thread's ring: detach (self-link) every node so no live entry points at a
    *     block the reclaim below may recycle. Other threads' rings are assumed quiescent (see above). */
   {
      TThdLink *ring = TThdRing::Head();

      if (ring->next != NULL) // armed on this thread?
      {
         n = ring->next;
         while (n != ring)
         {
            TThdLink *next = n->next;

            n->prev = n; // self-link => off-ring
            n->next = n;
            n = next;
         }
         ring->prev = ring;
         ring->next = ring;
      }
   }

   // (b) global buddy reclaim across all chunks/threads (the poolSharedReset behavior).
   for (c = p->chunks; c; c = c->next)
   {
      LPSTR q   = c->base,
            end = c->base + ((size_t)1u << c->order);

      while (q < end)
      {
         TPsBlock *b     = (TPsBlock *)q;
         size_t   order = b->order;

         q += (size_t)1u << order; // advance BEFORE ->next is repurposed as the temp link
         if (b->next == PS_IN_USE && b->refCount <= 1u)
         {
            b->refCount = 0u;
            b->next     = pending; // gather; do not coalesce yet (orders must stay intact for the walk)
            pending     = b;
         }
      }
   }
   while (pending)
   {
      TPsBlock *b = pending;

      pending = b->next;
      psFreeBlock(p, b); // coalesces with any buddy already returned to a class list
   }
   PS_UNLOCK(p);
}

//--------------------------------------------------------------------------------
LPSTR TPool::Strdup(LPCSTR s)
{
   size_t n;
   LPSTR p;

   if (!s)
      return NULL;
   n = strlen(s) + 1u;
   p = (LPSTR)Alloc(n);
   if (p)
      memcpy(p, s, n);
   return p;
}

// -- per-thread GC over the same shared pool -------------------------------

/* NO explicit thrInit: the ring sentinel is armed lazily from its zeroed TLS state by TThdRing::Init() on
   the first tracked alloc, so any regular alloc inside a thread auto-fires its bookkeeping. thrReset
   also self-arms if it runs on a never-allocated thread. */

/*--------------------------------------------------------------------------------
   Allocate + track on this thread's ring (one lock, no separate table alloc).
  --------------------------------------------------------------------------------*/
LPVOID TPool::thdAllocCore(size_t size, size_t align)
{
   LPVOID user;

   EnsureReady();
   if (size == 0u)
      return NULL;
   PS_LOCK(&gPool);
   user = psAllocAlignedLocked(&gPool, size, align < 32u ? 32u : align);
   if (user)
      TThdRing::Link(psBlockOf(user));
   PS_UNLOCK(&gPool);
   return user;
}

//--------------------------------------------------------------------------------
LPVOID TPool::thdAlloc(size_t size)
{
   return thdAllocCore(size, 32u);
}

//--------------------------------------------------------------------------------
LPVOID TPool::thdAllocAligned(size_t size, size_t align)
{
   return thdAllocCore(size, align);
}

/*--------------------------------------------------------------------------------
   Thread-tracked AND zeroed (the real impl, not a facade). new/new[] route here in the C++ build so
   every heap object is tracked for the per-thread GC and starts on cleared memory.
  --------------------------------------------------------------------------------*/
LPVOID TPool::thdCalloc(size_t count, size_t size)
{
   size_t n;
   LPVOID r;

   if (size != 0u && count > (size_t)-1/size) // overflow guard
      return NULL;
   n = count*size;
   r = thdAllocCore(n, 32u);
   if (r)
      memset(r, 0, n);
   return r;
}

//--------------------------------------------------------------------------------
void TPool::thdFree(LPVOID ptr)
{
   Free(ptr); // same path: psReleaseLocked splices off the ring + recycles to the buddy pool
}

/*--------------------------------------------------------------------------------
   CURRENT-THREAD-ONLY reset - never touches another thread's ring (scope contrast: abReset is the
   whole pool). A thin loop over the PUBLIC abFree: for each member of this thread's ring, call
   Free(p) - which already splices the node off the ring AND releases one global ref, coalescing back
   to the buddy pool at refCount 0. Disposal is natural: an unpinned block (refCount 1) is reclaimed; an
   abLock'd survivor (refCount > 1) stays alive for its holder but leaves this ring. `next` is saved
   BEFORE each abFree, since it splices the node and may recycle the block header. Ring ends empty, so no
   block references this thread's sentinel (safe at thread exit). O(1) per node. abThrReset does NOT
   hold the pool lock - abFree owns the locking per call (no re-entrancy).
  --------------------------------------------------------------------------------*/
void TPool::thrReset(void)
{
   TThdLink *ring = TThdRing::Head(),
            *n;

   TThdRing::Init(); // ensure armed (no-op if already); a never-used thread just gets an empty ring
   n = ring->next;
   while (n != ring)
   {
      TThdLink *next = n->next; // save BEFORE free splices the node / recycles the block header
      TPsBlock  *b    = blockOfLink(n);

      Free((LPSTR)b + PS_HEADER); // public path: unpin from the ring + decrement + coalesce at 0
      n = next;
   }
   ring->prev = ring; // ring now empty
   ring->next = ring;
}

// -- occupancy snapshot (diagnostics: the apps' 'm' key, memReport) --------

/*--------------------------------------------------------------------------------
   Walk every chunk under the lock and tally occupancy by buddy size class - a single consistent
   read-only snapshot. A block is allocated when next is the PS_IN_USE sentinel, otherwise it is a
   retained (recyclable) free block. No allocation.
  --------------------------------------------------------------------------------*/
void TPool::Stats(TAbPoolStats *out)
{
   TPoolShared *p = &gPool;
   TPsChunk    *c;

   if (!out)
      return;

   memset(out, 0, sizeof(*out));
   if (!gReady)
      return; // pool never armed -> zeroed snapshot

   PS_LOCK(p);
   for (c = p->chunks; c != NULL; c = c->next)
   {
      LPSTR q   = c->base,
            end = c->base + ((size_t)1u << c->order);

      out->chunks++;
      out->reservedBytes += (size_t)1u << c->order;

      while (q < end)
      {
         TPsBlock *b     = (TPsBlock *)q;

         size_t order = b->order,
                span  = (size_t)1u << order;

         if (b->next == PS_IN_USE)
         {
            out->inUseBlocks++;
            out->inUseBytes += span;
            if (order < (size_t)abPoolOrders)
               out->inUseByOrder[order]++;
            if (b->refCount >= 2u)
               out->lockedBlocks++;
         }
         else
         {
            out->freeBlocks++;
            out->freeBytes += span;
            if (order < (size_t)abPoolOrders)
               out->freeByOrder[order]++;
            if (order > out->largestFreeOrder)
               out->largestFreeOrder = order;
         }
         q += span;
      }
   }
   PS_UNLOCK(p);
}
