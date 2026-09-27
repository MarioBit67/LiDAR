/* thread.cpp -- THE threading backend seam (see thread.h). TMutex / TMutexLock / TSemaphore / TSemLock
 * implemented over C++20 std:: (std::thread / std::recursive_mutex / std::condition_variable). This is
 * the ONLY translation unit that touches the raw threading primitives directly (+ the one ppCheck exempts
 * from the STL poison, goldenRule 36). C11 <threads.h> is retired -> pure C++20, one code path for
 * win/linux/ios/android (no __STDC_NO_THREADS__ shim). The semaphore is a mutex + condition variable
 * (the standard has std::counting_semaphore in C++20, but we keep the timed/interruptible Wait ourselves,
 * goldenRule 24: bounded / interruptible blocking). */

#ifndef SHARED_LIBRARY_BUILD
#define SHARED_LIBRARY_BUILD
#endif

#ifdef _WIN32
   #ifndef WIN32_LEAN_AND_MEAN
      #define WIN32_LEAN_AND_MEAN
   #endif
   #include <windows.h> // SetThreadPriority / THREAD_PRIORITY_IDLE (goldenRule 5a per-thread belt)
#else
   #include <unistd.h> // sysconf (core count)
#endif

#include <stdio.h> // snprintf (fault tag)
#include <chrono>  // std::chrono for the timed wait deadline

#include "thread.h"
#include "fault.h"    // goldenRule-26 thread-root guard target (stub for now)
#include "alloc.h" // abThrInit / abThrReset -- the shared per-thread scratch GC (memThread reset)
#include "libDiscipline.h" // allocator/fp64/env poison -- last (this TU uses none of them)

/* The primary thread, captured at image load (dynamic initializers run on the main thread before main()).
 * IsMainThread compares against it -- the portable stand-in for mbLib's MainThreadID. */
static const std::thread::id gMainThread = std::this_thread::get_id();

//---------------------------------------------------------------------------------------------------

int threadHardwareThreads(void)
{
#ifdef _WIN32
   SYSTEM_INFO si;

   GetNativeSystemInfo(&si);
   return (si.dwNumberOfProcessors >= 1u) ? (int)si.dwNumberOfProcessors : 1;
#else
   long n = sysconf(_SC_NPROCESSORS_ONLN);

   return (n >= 1) ? (int)n : 1;
#endif
}

//---------------------------------------------------------------------------------------------------

QWORD threadNowUs(void)
{
   return (QWORD)std::chrono::duration_cast<std::chrono::microseconds>(
              std::chrono::steady_clock::now().time_since_epoch())
       .count();
}

//---------------------------------------------------------------------------------------------------

TMutex::TMutex(void) : Plocker(""), Powner(), Pheld(false)
{
   // std::recursive_mutex default-constructs ready to use -- no init/destroy needed.
}

//---------------------------------------------------------------------------------------------------

TMutex::~TMutex(void)
{
}

//---------------------------------------------------------------------------------------------------

void TMutex::Enter(void)
{
   Pmtx.lock();
   Powner = std::this_thread::get_id();
   Pheld = true;
}

//---------------------------------------------------------------------------------------------------

void TMutex::Leave(void)
{
   Pheld = false;
   Plocker = "";
   Pmtx.unlock();
}

//---------------------------------------------------------------------------------------------------

bool TMutex::TryEnter(void)
{
   if (!Pmtx.try_lock())
      return false;

   Powner = std::this_thread::get_id();
   Pheld = true;
   return true;
}

//---------------------------------------------------------------------------------------------------

bool TMutex::HeldByCurrent(void) const
{
   return Pheld && Powner == std::this_thread::get_id();
}

//---------------------------------------------------------------------------------------------------

bool TMutex::IsMainThread(void)
{
   return std::this_thread::get_id() == gMainThread;
}

//---------------------------------------------------------------------------------------------------

TMutexLock::TMutexLock(TMutex &m, LPCSTR from, bool fTry) : Pmutex(m), Pheld(false)
{
   if (!fTry)
   {
      Pmutex.Enter();
      Pheld = true;
   }
   else
      Pheld = Pmutex.TryEnter();

   if (Pheld)
      Pmutex.Plocker = (from != nullptr) ? from : "";
}

//---------------------------------------------------------------------------------------------------

TMutexLock::~TMutexLock(void)
{
   if (Pheld)
   {
      Pheld = false; // double-leave guard
      Pmutex.Leave();
   }
}

//---------------------------------------------------------------------------------------------------

TSemaphore::TSemaphore(int count, int initial)
    : Pcount((initial < 0) ? count : initial), Pmax(count)
{
   // std::mutex + std::condition_variable default-construct ready to use.
}

//---------------------------------------------------------------------------------------------------

TSemaphore::~TSemaphore(void)
{
}

//---------------------------------------------------------------------------------------------------

bool TSemaphore::Wait(int mSec)
{
   std::unique_lock<std::mutex> lk(Pmtx);

   if (mSec == threadInfinite)
   {
      Pcv.wait(lk, [this] { return Pcount > 0; });
   }
   else
   {
      auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(mSec);

      if (!Pcv.wait_until(lk, deadline, [this] { return Pcount > 0; }))
         return false; // timed out (lk released by the unique_lock dtor)
   }

   Pcount--;
   return true;
}

//---------------------------------------------------------------------------------------------------

void TSemaphore::Release(int ct)
{
   {
      std::lock_guard<std::mutex> lk(Pmtx);

      Pcount += ct;
      if (Pmax > 0 && Pcount > Pmax)
         Pcount = Pmax;
   }
   for (int i = 0; i < ct; i++)
      Pcv.notify_one();
}

//---------------------------------------------------------------------------------------------------

TSemLock::TSemLock(TSemaphore &s, int mSec) : Psem(s), Pacquired(false)
{
   Pacquired = Psem.Wait(mSec);
}

//---------------------------------------------------------------------------------------------------

TSemLock::~TSemLock(void)
{
   if (Pacquired)
      Psem.Release(1);
}

//---------------------------------------------------------------------------------------------------

TCondVar::TCondVar(void)
{
   // std::condition_variable_any default-constructs ready to use -- no init/destroy needed.
}

//---------------------------------------------------------------------------------------------------

TCondVar::~TCondVar(void)
{
}

//---------------------------------------------------------------------------------------------------

void TCondVar::Wait(TMutex &m)
{
   /* The caller already holds m (Enter'd once). Adopt its primitive into a unique_lock so
      condition_variable_any can release it while blocked and reacquire it on wake, then detach the lock
      (release, NOT unlock) so the caller's own Leave() still balances the level. m's owner/held
      diagnostics are mirrored across the wait. */
   std::unique_lock<std::recursive_mutex> lk(m.Pmtx, std::adopt_lock);

   m.Pheld = false;
   Pcv.wait(lk);
   m.Powner = std::this_thread::get_id();
   m.Pheld  = true;
   lk.release();
}

//---------------------------------------------------------------------------------------------------

void TCondVar::Signal(void)
{
   Pcv.notify_one();
}

//---------------------------------------------------------------------------------------------------

void TCondVar::Broadcast(void)
{
   Pcv.notify_all();
}

//---------------------------------------------------------------------------------------------------

TThread::TThread(LPCSTR name)
    : Pname((name != nullptr) ? name : "thread"), Pfrom(nullptr),
      Pterminated(false), Pfinished(false), Pstarted(false), Pjoined(false)
{
   // Pthr (std::thread) default-constructs to a non-joinable empty handle.
}

//---------------------------------------------------------------------------------------------------

TThread::~TThread(void)
{
   Terminate(); // RAII: cooperatively stop...
   Join();      // ...and reap the thread before we die (bounded by the derived Ping() contract)
}

//---------------------------------------------------------------------------------------------------

bool TThread::Start(LPCSTR from)
{
   if (Pstarted)
      return true;

   Pfrom = from;
   try
   {
      Pthr = std::thread(&TThread::Run, this); // spawns; runs Run() on the new thread
   }
   catch (...) // std::thread throws std::system_error if the OS refuses the thread
   {
      Pfinished = true; // never ran
      return false;
   }
   Pstarted = true;
   return true;
}

//---------------------------------------------------------------------------------------------------

void TThread::Terminate(void)
{
   Pterminated = true;
   TerminatedSet(); // hook: a dormant worker wakes itself here so shutdown is instant (default no-op)
}

//---------------------------------------------------------------------------------------------------

void TThread::Join(void)
{
   if (Pstarted && !Pjoined)
   {
      if (Pthr.joinable())
         Pthr.join();
      Pjoined = true;
   }
}

//---------------------------------------------------------------------------------------------------

/* Cooperative checkpoint the derived Execute() polls. Returns false once Terminate() was requested, so
 * derived code writes `while (Ping()) { ... }` instead of a hand-rolled `while (!terminated)` loop. Future:
 * also block here on a pause/sync request; for now it only reports the terminate state. */
bool TThread::Ping(void)
{
   return !Pterminated;
}

//---------------------------------------------------------------------------------------------------

/* The base thread body: all the per-thread boilerplate derived code must NOT write. IDLE priority
 * (goldenRule 5a belt -- the process is already idle, so spawned threads inherit it; this is the
 * per-thread backstop), reset this thread's pool-scratch tracking (the shared per-thread GC), run the
 * derived Execute() under the goldenRule-26 thread-root guard, then reclaim the thread's scratch. */
void TThread::Run(void)
{
#ifdef _WIN32
   SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_IDLE);
#endif
   /* NO explicit thread-init: the per-thread GC ring auto-arms from its zeroed TLS state on the first
      tracked alloc (TThdRing lazy init), so nothing is needed here. */

   {
      TThdPoolSpan span; // reclaim this thread's scratch when Execute() returns or throws (RAII thrReset)

#ifdef csFaultTrace
      sehRun(); // fault-trace build: SEH trampoline catches a worker access-violation -> DumpRaw chain + abFault
#else
      try
      {
         Execute();
      }
      catch (...)
      {
         char tag[192];

         snprintf(tag, sizeof tag, "TThread '%s' @ %s", Pname, (Pfrom != nullptr) ? Pfrom : "?");
         abFault(tag); // stub records + returns -> no crash-through; the real handler drops in later
      }
#endif
   }
   OnTerminate(); // funeral hook: worker thread, object still alive (NOT from ~TThread)
   Pfinished = true;
}

/* sehRun -- csFaultTrace worker-thread SEH trampoline. Under /EHsc a C++ catch(...) does NOT catch a
   structured exception (an access violation), so a worker AV would die unattributed. This PoD-bodied helper
   (no C++ objects needing unwinding -> __try is legal, no C2712) wraps Execute() in __try/__except: on a
   hardware fault it prints the code + this thread's TStack chain (crash-safe DumpRaw) + tid, then records via
   abFault. Windows-only; elsewhere a passthrough. Compiled ONLY under csFaultTrace (ships OFF, zero cost). */
#ifdef csFaultTrace
#ifdef _WIN32

//---------------------------------------------------------------------------------------------------
void TThread::sehRun(void)
{
   __try
   {
      Execute();
   }
   __except (EXCEPTION_EXECUTE_HANDLER)
   {
      char tag[224];

      snprintf(tag, sizeof tag, "SEH 0x%lx in TThread '%s' @ %s (tid %lu)",
               (unsigned long)GetExceptionCode(), (Pname != nullptr) ? Pname : "?",
               (Pfrom != nullptr) ? Pfrom : "?", (unsigned long)GetCurrentThreadId());
      fprintf(stderr, "[abFault] %s\n", tag);
      fflush(stderr);
      TStack::DumpRaw(); // crash-safe chain FIRST (survives a corrupt heap; abFault's std::string Dump may not)
      abFault(tag);
   }
}
#else

//---------------------------------------------------------------------------------------------------
void TThread::sehRun(void)
{
   Execute();
}
#endif
#endif

//---------------------------------------------------------------------------------------------------

TCormWorker::TCormWorker(LPCSTR name) : TThread(name), Pwake(cormMaxPending, 0)
{
}

//---------------------------------------------------------------------------------------------------

void TCormWorker::Post(void)
{
   Pwake.Release(); // hand the dormant worker one job to run
}

//---------------------------------------------------------------------------------------------------

void TCormWorker::TerminatedSet(void)
{
   Pwake.Release(); // wake the dormant worker so it observes IsTerminated() and exits its loop at once
}

//---------------------------------------------------------------------------------------------------

/* The CORM dormant loop the base owns (a leaf never writes this). Wait for a Post() (idle-timed so it also
 * re-checks the cooperative stop every cormIdleMs even if a wake was missed); run one DoJob(); recycle this
 * thread's per-job pool scratch; then notify OnJobDone(). Terminate() releases the wake semaphore (via
 * TerminatedSet), so a dormant worker exits immediately -- zero idle CPU, instant shutdown. */
void TCormWorker::Execute(void)
{
   while (Ping())
   {
      if (!Pwake.Wait(cormIdleMs))
         continue; // no job this tick -- loop to re-observe Ping() / IsTerminated()

      if (IsTerminated())
         break;

      {
         TThdPoolSpan span; // per-job scratch recycle (RAII thrReset): untracked jobs no-op, tracked reclaimed

         DoJob();
      }
      OnJobDone();
   }
}
