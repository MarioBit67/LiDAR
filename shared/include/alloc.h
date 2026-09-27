/*
 * alloc.h — THE one allocation API for the whole monorepo: class TPool.
 *
 * Every subsystem (abImage / abAudio / abDoc / abWorker / abBalance / abProxy) allocates through
 * TPool's static methods, which operate the ONE shared pool directly. There is no per-engine pool, no
 * vtable, and no handle — the four-pool virtualization layer (registry / poolBase facade) was needed
 * only while each engine had its own arena; with a single consolidated pool it is pure indirection and
 * is being removed. Only the pool implementation is aware of "a pool"; all other code calls TPool::*.
 *
 * Reference-counted in a GC sense: alloc starts a block at refCount=1, lock pins it (refCount++),
 * free is `if (!--refCount) reclaim`. A Locked block survives until every holder frees it, and a
 * reset reclaims only refCount==1 blocks. The pool returns 64-byte-aligned memory (one cache line),
 * so alloc covers every SIMD need; allocAligned is kept for explicit per-call alignment intent.
 *
 * Thread variants (thd / thr) add per-thread GC ownership tracking over the SAME shared pool:
 * thrReset reclaims a thread's own unshared scratch (refCount==1) without a global reset. The
 * memory is always the one global pool; the thread table is just bookkeeping.
 *
 * C++ (codebase standard); PrettyPrint for any authored line.
 */
#ifndef SHARED_ALLOC_H
#define SHARED_ALLOC_H
#include "winTypes.h" // winint typedefs (BYTE/WORD/DWORD/...)

#include <stddef.h>  /* size_t */
#include <stdbool.h> /* bool (TPool::bound) */

// -- occupancy snapshot of the one pool (diagnostics: the apps' 'm' key, memReport) --------------
enum { abPoolOrders = 48 }; /* buddy size classes 0..47 (a block spans 1<<order bytes) */

/* Region bytes (1<<order), not the caller's requested sizes, so inUseBytes counts rounded-up buddy
 * spans. `largestFreeOrder` is meaningful only when freeBlocks > 0. */
typedef struct TAbPoolStats {
   // byte totals over rounded-up buddy regions: reserved from libc | held by in-use blocks | recyclable
   size_t        reservedBytes, inUseBytes, freeBytes;
   // block counts: chunks obtained | in-use | recyclable | Lock-pinned (refCount >= 2)
   unsigned long chunks, inUseBlocks, freeBlocks, lockedBlocks;
   size_t        largestFreeOrder; /* highest order with a free block (largest recyclable run) */
   unsigned long inUseByOrder[abPoolOrders],
                 freeByOrder[abPoolOrders]; /* per-size-class histograms */
} TAbPoolStats;

struct TPoolShared; // opaque buddy-pool storage (defined in abPool.cpp); TPool owns the one instance

/* THE one allocation family (single shared pool). All static — TPool is a namespace with access
 * control, never instantiated. The ONE pool is a private static member (gPool) — statically created,
 * opaque, process-long-lasting: it always exists, so there is no bind / no "bound?" test.
 *
 * Alloc-site tagging (profiler): under ABC_MEMTAG (a Windows-only profiling build) the caller's
 * __FILE__/__LINE__ are passed into alloc/allocAligned, which stamp the block so TPoolShared can dump
 * the peak-memory composition (by call site) to a track file on every new high-water. Default
 * (file=0,line=0) ⇒ no tag, a plain forward — zero cost, byte-identical release. __FILE__ is a static
 * literal (pointer stored, never copied). */
#ifdef __cplusplus /* the pool API is a C++ class; C TUs (e.g. vendored miniz, force-fed types.h) skip it */
class TPool {
public:
   /* Pool ops callable directly by user code; ordinary allocation flows through global operator
    * new/new[]/delete (→ the private members). Alloc/Init/thread-arm are all LAZY (auto on first use
    * from zeroed state), so only the deliberate ops are public: Realloc, Lock, Unlock, and the read-only
    * Stats snapshot. BOTH GC resets are protected — fired only by their RAII spans (TPoolSpan /
    * TThdPoolSpan), never called explicitly. General methods are PascalCase; the thread-scoped thd/thr
    * prefixes stay lowercase (house standard). */
   static LPVOID Realloc(LPVOID ptr, size_t newSize);  /* grow/shrink, preserving bytes */
   static LPVOID Lock(LPVOID ptr);                    /* pin a long-lived block (refCount++); returns ptr (chains) */
   static void  Unlock(LPVOID ptr);                   /* undo Lock: refCount-- (reclaim at 0), NO destructor */
   /* Find the block's USER BASE for any pointer that lies inside it, and report the interior offset
    * (out param). For a `new T[n]` element pointer this recovers the base BEFORE the array cookie and
    * measures the cookie AGNOSTICALLY — no ABI/compiler cookie-size assumption. Returns p itself
    * (offset 0) for a foreign/untracked pointer. Lets TBlock run pool ops (Lock/Unlock/Realloc) on the
    * true base while keeping the element pointer for delete[]/indexing. */
   static LPVOID Find(LPVOID p, size_t &offset);
   static void  Stats(TAbPoolStats *out);            /* read-only occupancy snapshot (the apps' 'm' key; no owner needed) */

protected:
   /* GC resets — NOT called directly by app code; each is fired only by its RAII span's dtor (the
    * sanctioned trigger; the span reaches these via private inheritance, no friend needed). Protected
    * keeps the triggers off the public surface (opacity):
    *   Reset()    global, job boundary       -> TPoolSpan
    *   thrReset() this thread's own scratch  -> TThdPoolSpan */
   static void Reset(void);
   static void thrReset(void);

public: // baseline for the pool-owner view: the guard below only adds `private` for consumers
#ifndef csPoolOwner
private: // consumers: fully opaque (the whole pool surface below is unreachable). Owners keep the public
         // baseline above (guard skipped) -> full access, symbols mangle public (S).
#endif
   // -- the one pool: static, opaque, always-exists (no bind, no pointer, no "bound?" test) -----
   static TPoolShared gPool;  /* THE pool storage — the single process-long-lasting instance */
   static bool       gReady; /* one-time mutex+free-list init flag (lazy, like TThdRing::gReady) */

   // -- internal helpers (member scope so they reach the private API without csPoolOwner) --------
   static void  EnsureReady(void);                       /* one-time lazy init of gPool (mutex + free-lists) */
   static LPVOID thdAllocCore(size_t size, size_t align); /* alloc + ring-track under one lock */

   // -- allocation (single shared pool) -----------------------------------------
   static LPVOID Alloc(size_t size, LPCSTR file = 0, int line = 0);                      /* 64-aligned block; refCount=1 */
   static LPVOID AllocAligned(size_t size, size_t align, LPCSTR file = 0, int line = 0); /* explicit per-call alignment */
   static LPVOID Calloc(size_t count, size_t size);  /* zeroed; overflow-guarded */
   static void   Free(LPVOID ptr);                    /* refCount--; reclaim at 0 (over-free is harmless) */
   static size_t RefCount(LPVOID ptr);                /* current block refcount; 0 if unknown */
   static LPSTR Strdup(LPCSTR s);              /* pool-backed strdup (NULL-safe) */

   /* -- per-thread GC over the same shared pool --
      An O(1) intrusive live-list (threaded through each block's header): thd* push a block onto this
      thread's list, Free/thdFree unlink it on recycle (any order, cross-thread safe under the pool
      mutex), and thrReset reclaims this thread's unpinned scratch (refCount==1) while leaving the
      Lock'd survivors (refCount>=2) for their holder. NO explicit thread-init: the ring is armed lazily
      from its zeroed TLS sentinel on the first tracked alloc, so any regular alloc inside a thread
      auto-fires its bookkeeping. */
   static LPVOID thdAlloc(size_t size);                      /* alloc + track (default align) */
   static LPVOID thdAllocAligned(size_t size, size_t align); /* alloc + track, per-call alignment */
   static LPVOID thdCalloc(size_t count, size_t size);       /* alloc + track + zero (new/new[] route here) */
   static void  thdFree(LPVOID ptr);                         /* free + untrack */

   // -- pool lifecycle (owner-only; no app-facing bind — the pool always exists) --
   static void Destroy(void); /* free every chunk + reset gReady (tests/shutdown only) */
};

/* TPoolSpan — RAII job-boundary guard: its destructor fires Reset() (the global GC reset). Reset is
 * protected (off the public surface); this span is the sanctioned trigger — construct one for the job's
 * duration and the pool is reclaimed when it destructs. Privately derives TPool ("implemented in terms
 * of"): not-a-pool to the outside, but reaches the protected Reset through the base — no friend needed. */
class TPoolSpan : private TPool {
public:
   TPoolSpan(void) = default;

   ~TPoolSpan(void) { Reset(); }

   TPoolSpan(const TPoolSpan &) = delete;
   TPoolSpan &operator=(const TPoolSpan &) = delete;
};

/* TThdPoolSpan — RAII per-thread GC guard: its destructor fires thrReset() (reclaim THIS thread's own
 * refCount==1 scratch, sparing Lock'd survivors) without a global reset. Thread sibling of TPoolSpan;
 * privately derives TPool to reach the protected thrReset — no friend, publishes nothing. Construct one
 * for a thread's work span and its scratch is reclaimed when it destructs. */
class TThdPoolSpan : private TPool {
public:
   TThdPoolSpan(void) = default;

   ~TThdPoolSpan(void) { thrReset(); }

   TThdPoolSpan(const TThdPoolSpan &) = delete;
   TThdPoolSpan &operator=(const TThdPoolSpan &) = delete;
};

/* TBlock<T> / TAlloc<T> — RAII owners of a POOL buffer, ownership ALWAYS under RAII (never a raw pushed
 * pointer). TBlock is the base + the RECEIVER of a transfer: adopt/assign pins the block (TPool::Lock →
 * GC-durable while held) + releases any prior; the dtor unpins + frees (Unlock + delete[]); Drop(b) MOVES
 * ownership into another TBlock. TAlloc DERIVES from TBlock and ALLOCATES its own block in the ctor (the
 * producer). Hand off with `producer.Drop(receiverBlock)`. Non-copyable, single owner. `x()` / `operator
 * T*` = raw ptr; operator[]/operator-> index/dereference. */
/* TBlockMap<T> — scoped gateway between a `new T[n]` ELEMENT pointer and its pool block base. On entry it
 * Finds the true base (Ptr) and the array-cookie gap (Offset), resolved case-by-case — no stored/assumed
 * cookie size, ABI-agnostic (win/linux/ios/android). The pool op runs on Ptr inside the scope (and MAY move
 * it, e.g. Realloc); on exit the dtor re-advances the caller's own pointer past the cookie. Both the
 * pointer-moving path and the read-only pin (Ptr unchanged → Data restored to itself) go through the one gate. */
template<class T> struct TBlockMap {
   LPSTR Ptr; // block user base — what every TPool op wants

   T *&Data; // the caller's element pointer, restored on scope exit

   size_t Offset; // Data - Ptr, the measured cookie gap

   explicit TBlockMap(T *&p) : Data(p) { Ptr = (LPSTR)TPool::Find(Data, Offset); }

   ~TBlockMap(void) { Data = (T *)(Ptr + Offset); }

   TBlockMap(const TBlockMap &)            = delete;
   TBlockMap &operator=(const TBlockMap &) = delete;
   TBlockMap(TBlockMap &&)                 = delete;
   TBlockMap &operator=(TBlockMap &&)      = delete;
};

template<class T> class TBlock {
protected:
   T *Data; // the ELEMENT pointer (what `new T[n]` returns, what the user indexes)

   void adopt(T *p)
   {
      Data = p;
      if (Data)
      {
         TBlockMap<T> m(Data); // gateway: pin the true base (refCount 1 -> 2), GC-durable while held

         TPool::Lock(m.Ptr);
      }
   }

   /* Unpin + free, ADAPTIVE to `new` vs `new[]`: TBlockMap measures the array-cookie gap (Offset) at the
      true base. Offset != 0 => an array cookie is present (this was `new T[n]`) => delete[] (the compiler
      reads the cookie to run n destructors). Offset == 0 => no cookie (a single `new T`, or a trivial-T
      array with no cookie) => plain delete. Both routes hit the SAME pool free; the distinction only governs
      which/how-many destructors run. So a TBlock owns EITHER a `new`'d object or a `new[]`'d array. */
   void freeData(void)
   {
      if (!Data)
         return;

      size_t off = 0u;
      {
         TBlockMap<T> m(Data); // gateway: unpin the true base (2 -> 1)

         TPool::Unlock(m.Ptr);
         off = m.Offset;
      }
      if (off)
         delete[] Data;
      else
         delete Data;
      Data = NULL;
   }

public:
   explicit TBlock(T *p = NULL) : Data(NULL) { adopt(p); }

   ~TBlock(void) { freeData(); }

   TBlock &operator=(T *p)
   {
      freeData();
      adopt(p);
      return *this;
   }

   /* Move ownership from THIS owner into `b`, RAII throughout. Created at refCount 2 (Alloc + this owner's
    * pin); `b = Data` adds b's pin (->3); Unlock undoes THIS pin (->2) so b holds it durably at 2; Data =
    * NULL removes it from this owner (dtor then no-ops). refCount-mastered — never a raw unowned pointer. */
   T *Drop(TBlock &b)
   {
      b = Data;
      {
         TBlockMap<T> m(Data); // gateway: unpin THIS pin (3 -> 2), b keeps it durable

         TPool::Unlock(m.Ptr);
      }
      Data = NULL;
      return b;
   }

   /* Grow/shrink a POD buffer in place, preserving the pin (TPool::Realloc swaps the refCount onto the new
    * block). The gateway maps to the true base, the realloc MAY move it (Ptr updated), and the dtor
    * re-advances Data past the cookie to the new element ptr. Trivial T only (cookie 0 → Data == base). */
   void Realloc(size_t n)
   {
      static_assert(__is_trivially_destructible(T), "TBlock::Realloc is for trivially-destructible (POD) buffers only");
      if (Data)
      {
         TBlockMap<T> m(Data); // gateway: Ptr = true base, Offset = cookie gap; dtor restores Data = Ptr + Offset

         m.Ptr = TPool::Realloc(m.Ptr, n*sizeof(T) + m.Offset); // realloc TRUE base; refCount swaps over
      }
   }

   T   *operator->(void) { return Data; }
   T   &operator[](size_t n) { return Data[n]; }
   const T &operator[](size_t n) const { return Data[n]; } // const indexing (read a segment off a const block)
   T   *operator()(void) const { return Data; } // raw pointer: b()
   operator T *(void) const { return Data; }    // pass-through + `if (b)` null test

   TBlock(const TBlock &)            = delete; // copy is forbidden — two owners of one block = double free
   TBlock &operator=(const TBlock &) = delete;

   /* MOVE = the sanctioned ownership transfer. Steal the source's element pointer and blank it; the block
    * keeps its existing pin (refCount unchanged), only WHICH object owns it changes. Exactly one owner at
    * all times, buffer never raw. This is how an owner crosses a boundary (e.g. a TWSFrame moved through a
    * typed handoff): the memberwise move cascades to this. (memcpy bypasses it — the transport must move.) */
   TBlock(TBlock &&o) : Data(o.Data) { o.Data = NULL; }

   TBlock &operator=(TBlock &&o)
   {
      if (this != &o)
      {
         freeData(); // release what we currently own before taking o's (adaptive new/new[])
         Data   = o.Data;
         o.Data = NULL;
      }
      return *this;
   }
};

/* TAlloc<T> — a TBlock that ALLOCATES its own block: the scope-bound producer. `new T[n]` for any T; the
 * base adopts + pins (refCount 2 → GC-durable for the scope); every manual pool op routes through TBlockMap
 * (TPool::Find) to the true base past the array cookie a non-trivial T carries. dtor unpin+free, Drop,
 * indexing, and pointer decay inherited from TBlock. Non-copyable (via the base). */
template<class T> class TAlloc : public TBlock<T> {
public:
   explicit TAlloc(size_t n) : TBlock<T>(new T[n]) {} // array-new; TBlock::offCookie corrects pool ops
};
#endif /* __cplusplus */

#endif /* SHARED_ALLOC_H */
