#ifndef SHARED_THREAD_H
#define SHARED_THREAD_H
#include "winTypes.h" // winint typedefs (BYTE/WORD/DWORD/...)

/* thread.h -- THE threading seam for the whole monorepo (threading exclusivity mandate).
 *
 * TMutex / TSemaphore (+ their scoped RAII guards TMutexLock / TSemLock) and, in a later sub-stage,
 * TThread wrap the ONE backend seam. NO other code anywhere may master threads directly: the C++20 std::
 * primitives (std::thread / std::recursive_mutex / std::condition_variable) appear ONLY inside this
 * module (thread.h + thread.cpp) -- the ONE TU ppCheck exempts from the STL poison (goldenRule 36).
 * Everything else uses these classes. C11 <threads.h> was retired here (pure C++20; no <threads.h> dep).
 *
 * TMutex     -- RAII recursive mutex; Enter / Leave / TryEnter + diagnostics (Locker / OwningThread /
 *               IsMainThread). Ported from mbLib TCritical (the CRITICAL_SECTION lineage was recursive).
 * TMutexLock -- scoped lock guard: ctor Enter (or TryEnter when fTry), dtor Leave, double-leave guarded,
 *               records the call-site `from` breadcrumb as the mutex's Locker (feeds abFault attribution).
 * TSemaphore -- in-process counting semaphore with a TIMED, interruptible Wait (goldenRule 24). Built on
 *               std::mutex + std::condition_variable (wait_until). Ported from mbLib TSemaphore.
 * TSemLock   -- scoped acquire/release guard (Wait in ctor, Release in dtor -- only if the Wait succeeded).
 * TThread    -- Delphi-style template-method thread: derived implements Execute(); the base entry sets IDLE
 *               priority, resets per-thread pool scratch, and wraps Execute() in try/catch -> abFault.
 *
 * thisInfo   -- portable call-site breadcrumb macro (compiler pretty-function), passed to the guards.
 */

#include <thread>             // std::thread — the ONE backend (this module + thread.cpp are the sole users)
#include <mutex>              // std::mutex / std::recursive_mutex
#include <condition_variable> // std::condition_variable (TSemaphore's wait)

/* Portable call-site breadcrumb -- the function signature of the site taking a lock. Stored as a
 * mutex/semaphore Locker ("who holds me") and fed to abFault attribution when a fault is captured. */
#if defined(__GNUC__) || defined(__clang__)
   #define thisInfo __PRETTY_FUNCTION__
#elif defined(_MSC_VER)
   #define thisInfo __FUNCSIG__
#else
   #define thisInfo __func__
#endif

// Timeout sentinel for the timed waits (TSemaphore::Wait / TSemLock): block until acquired.
enum { threadInfinite = -1 };

// Logical core count (>= 1), for band / pool sizing. Part of the seam (platform threading knowledge).
int threadHardwareThreads(void);

/* Monotonic wall-clock in microseconds (C++20 steady_clock, encapsulated in the seam so callers stay
   STL-free; C11 timespec_get / clock_gettime retired, goldenRule 36). Wraps at 2^64 us (~584k years). */
QWORD threadNowUs(void);

//---------------------------------------------------------------------------------------------------

/* TMutex -- recursive mutex over the C++20 std:: backend. RAII: the ctor inits the primitive, the dtor destroys
 * it. Enter / Leave / TryEnter master the lock; Locker / OwningThread / IsMainThread are diagnostics that
 * feed fault attribution. Prefer the scoped TMutexLock over hand Enter/Leave. */
class TMutex
{
 public:
   TMutex(void);

   ~TMutex(void);

   void Enter(void);    // block until held (recursive: the owning thread may re-enter)
   void Leave(void);    // release one level
   bool TryEnter(void); // non-blocking; true if the lock was taken

   LPCSTR Locker(void) const { return Plocker; } // call-site breadcrumb of the current holder
   std::thread::id OwningThread(void) const { return Powner; }
   bool            HeldByCurrent(void) const; // is the calling thread the owner?
   static bool IsMainThread(void);        // is the caller the process's primary thread?

   TMutex(const TMutex &) = delete; // owns a live primitive -- not copyable
   TMutex &operator=(const TMutex &) = delete;

 private:
   friend class TMutexLock; // the guard writes Plocker with the call-site `from`
   friend class TCondVar;   // TCondVar::Wait adopts Pmtx into a unique_lock for condition_variable_any

   std::recursive_mutex Pmtx; // recursive: the CRITICAL_SECTION lineage (mbLib TCritical) re-enters
   LPCSTR Plocker; // who holds me (thisInfo of the guard that took it), "" when free
   std::thread::id Powner; // owning thread while held
   bool            Pheld;  // best-effort held flag (diagnostic; recursion counts once)
};

//---------------------------------------------------------------------------------------------------

/* TMutexLock -- scoped RAII lock guard. Ctor Enters (or TryEnters when fTry); dtor Leaves exactly once
 * (double-leave guarded). Records the call-site `from` (pass thisInfo) as the mutex's Locker. On a fTry
 * that fails, Locked() is false and the dtor does nothing. */
class TMutexLock
{
 public:
   TMutexLock(TMutex &m, LPCSTR from, bool fTry = false);

   ~TMutexLock(void);

   bool Locked(void) const { return Pheld; } // false only when a fTry acquire failed

   TMutexLock(const TMutexLock &) = delete;
   TMutexLock &operator=(const TMutexLock &) = delete;

 private:
   TMutex &Pmutex;
   bool    Pheld;
};

//---------------------------------------------------------------------------------------------------

/* TSemaphore -- in-process counting semaphore built on std::mutex + std::condition_variable. Wait has a
 * TIMEOUT (goldenRule 24: bounded / interruptible blocking) -- it returns false on timeout instead of
 * wedging the thread. count = the maximum level; initial = the starting available count (< 0 => count,
 * i.e. fully available). Release raises the level (capped at max) and wakes waiters. */
class TSemaphore
{
 public:
   explicit TSemaphore(int count = 1, int initial = -1);
   ~TSemaphore(void);

   bool Wait(int mSec = threadInfinite); // true = acquired one; false = timed out
   void Release(int ct = 1);             // raise the level by ct (capped at max) and wake waiters
   int  Level(void) const { return Pcount; } // current available count (diagnostic)

   TSemaphore(const TSemaphore &) = delete;
   TSemaphore &operator=(const TSemaphore &) = delete;

 private:
   std::mutex              Pmtx;
   std::condition_variable Pcv;
   int                     Pcount,
                           Pmax;
};

//---------------------------------------------------------------------------------------------------

/* TSemLock -- scoped acquire/release guard. Ctor Waits (mSec timeout); dtor Releases -- but ONLY if the
 * Wait actually acquired (Acquired() reports it), so a timed-out acquire never over-releases. */
class TSemLock
{
 public:
   explicit TSemLock(TSemaphore &s, int mSec = threadInfinite);
   ~TSemLock(void);

   bool Acquired(void) const { return Pacquired; }

   TSemLock(const TSemLock &) = delete;
   TSemLock &operator=(const TSemLock &) = delete;

 private:
   TSemaphore &Psem;
   bool        Pacquired;
};

//---------------------------------------------------------------------------------------------------

/* TCondVar -- condition variable over the C++20 std:: backend, paired with a TMutex. Wait() atomically
 * releases the held mutex, blocks until Signal()/Broadcast(), then reacquires it, so the caller re-checks
 * its predicate in a `while (!ready) cv.Wait(m);` loop -- the faithful stand-in for C11 cnd_wait /
 * cnd_signal / cnd_broadcast. Built on std::condition_variable_any so it composes with TMutex's recursive
 * backend. The wait is UNBOUNDED (matches the message-pipe / encode-pipeline consumers it backs); reach for
 * TSemaphore instead where goldenRule-24 timed / interruptible blocking is required. */
class TCondVar
{
 public:
   TCondVar(void);

   ~TCondVar(void);

   void Wait(TMutex &m);  // release m, block until signalled, reacquire m (re-check the predicate on return)
   void Signal(void);     // wake one waiter
   void Broadcast(void);  // wake all waiters

   TCondVar(const TCondVar &) = delete;
   TCondVar &operator=(const TCondVar &) = delete;

 private:
   std::condition_variable_any Pcv; // pairs with TMutex (recursive) -- _any, not the unique_lock-only cv
};

//---------------------------------------------------------------------------------------------------

/* TThread -- Delphi-style template-method thread over the C++20 std:: backend. A derived class implements the pure
 * virtual Execute() (the work body) and calls Ping() at its safe points; Ping() observes the cooperative
 * Terminate() request internally, so derived code NEVER hand-rolls a `while (!terminated)` loop. The base
 * thread-entry does the boilerplate ONCE, so derived code writes none of it: set IDLE priority (goldenRule
 * 5a) -> reset this thread's pool scratch (poolThrInit / poolThrReset, the shared per-thread GC) ->
 * try { Execute(); } catch (...) { abFault(tag); } (goldenRule-26 thread-root guard). Start() spawns and
 * carries a thisInfo breadcrumb; the dtor cooperatively Terminates + Joins (RAII start/join lifecycle). */
class TThread
{
 public:
   explicit TThread(LPCSTR name = "thread");
   virtual ~TThread(void);

   bool Start(LPCSTR from = nullptr); // spawn; `from` = thisInfo breadcrumb; false if create failed
   void Terminate(void);                    // request cooperative stop (Ping() then returns false)
   void Join(void);                         // block until the thread's Execute() has returned
   bool Finished(void) const { return Pfinished; }

   TThread(const TThread &) = delete; // owns a live thread -- not copyable
   TThread &operator=(const TThread &) = delete;

 protected:
   virtual void Execute(void) = 0; // the work body -- derived MUST implement

   bool Ping(void);                                    // cooperative checkpoint: false => stop now
   bool IsTerminated(void) const { return Pterminated; } // observe the flag directly if finer control is needed

   /* Lifecycle hooks (default no-ops). TerminatedSet() fires from Terminate() AFTER the flag is set -- a
      dormant worker overrides it to wake itself (e.g. release its wait semaphore) so shutdown is instant at
      zero idle CPU. OnTerminate() fires at the END of Run() ON THE WORKER THREAD with the object still alive
      (the finished / funeral notification) -- NOT from ~TThread, so a derived can react to its own exit. */
   virtual void TerminatedSet(void) {}
   virtual void OnTerminate(void) {}

 private:
   void Run(void); // IDLE + pool-scratch reset + try/catch(abFault) + Execute (std::thread entry)
   void sehRun(void); // csFaultTrace only: SEH trampoline — catches a worker AV /EHsc can't, dumps the TStack chain

   std::thread   Pthr;

   LPCSTR Pname,
          Pfrom; // thisInfo breadcrumb captured at Start

   volatile bool Pterminated,
                 Pfinished;
   bool          Pstarted,
                 Pjoined;
};

//---------------------------------------------------------------------------------------------------

/* CORM worker tuning. cormIdleMs = the idle re-check tick (ms) a dormant worker wakes on to re-observe
   Ping()/terminate even absent a Post() -- a liveness belt over the instant TerminatedSet() wake.
   cormMaxPending = the wake semaphore's cap (outstanding Post()+terminate wakes; the pool posts once per
   dispatch and joins before the next, so at most a couple are ever pending). */
enum {
   cormIdleMs     = 1000,
   cormMaxPending = 8
};

/* TCormWorker -- reusable CORM (create-once-reuse-many) worker: a TThread that stays DORMANT between jobs
 * instead of being recreated per job. The BASE owns the dormant loop; a leaf implements ONLY DoJob() (the
 * per-job work) and optionally OnJobDone() (fired after the per-job pool-scratch recycle -- e.g. to signal a
 * pool's done-count). Post() wakes the worker to run one DoJob; Terminate() wakes it instantly (via the
 * TerminatedSet override) so it exits at zero idle CPU. This is the shape a pooled band worker AND a
 * fire-and-forget saver (e.g. a snapshot writer, minus the done-count) both build on. */
class TCormWorker : public TThread
{
 public:
   explicit TCormWorker(LPCSTR name = "cormWorker");

   void Post(void); // wake the dormant worker to run one DoJob()

 protected:
   virtual void DoJob(void) = 0;   // the per-job work body -- leaf implements
   virtual void OnJobDone(void) {} // fired after DoJob() + the per-job scratch recycle (default no-op)

   void Execute(void) override;       // the dormant loop (base-owned; a leaf does NOT override this)
   void TerminatedSet(void) override; // wake the dormant worker so Terminate() is instant

 private:
   TSemaphore Pwake; // Post() / TerminatedSet() release; the loop waits on it (idle-timed re-check)
};

/* -- CORM band fork/join pool (was threadpool.h; merged here — one threading seam) -----------------
 * Process-wide create-once-reuse-many pool (impl in abImage camblock). cbRun splits [0,n) into
 * <=maxbands (0 = all cores) contiguous bands; the caller runs band 0 at normal priority, workers run
 * IDLE; the join is the barrier. Reductions are the caller's (index-ordered for determinism). Per-band
 * scratch is caller-provided — the pool is not an allocator. Serial fallback when threads/parts<=1.
 * See memory project_abimage_multithreading. */
typedef void (*TCbKernel)(int lo, int hi, int band, LPVOID ctx);

// Run fn over [0,n) in <=maxbands bands (0 = cbHwCores()); caller runs band 0; barrier on return.
void cbRun(int n, TCbKernel fn, LPVOID ctx, int maxbands);

// Participant cap = ABI_CB_THREADS (via abiCamblockSetThreads) else all hardware cores, <= cbPoolMax.
int cbHwCores(void);

#endif // SHARED_THREAD_H
