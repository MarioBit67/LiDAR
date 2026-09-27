#ifndef SHARED_FAULT_H
#define SHARED_FAULT_H
#include "winTypes.h" // winint typedefs (BYTE/WORD/DWORD/...)

/* fault.h -- MINIMAL STUB of the decided fault-handling endpoint (goldenRule 26).
 *
 * goldenRule 26 fixes the error-resilience architecture as `abFault`: a central capture/minidump handler
 * with a MANDATORY thread-root try/catch -> abFault(tag) on every production thread. That subsystem is
 * DESIGN-ONLY and not built yet (sequenced after the live abiCamblock agents). This stub exists so the
 * thread-root hook in TThread has a real symbol to call NOW: it RECORDS the tag (so a fault is attributable
 * / testable) and RETURNS -- it does NOT crash-through during bring-up. The real handler drops in here later
 * (reserved-stack minidump, /EHa capture, payload-content-free attribution, recovery decision).
 *
 * TODO(roadmapAbFault): replace this stub with the central handler; keep this signature stable.
 */

#include <string> // std::string (the one allowed STL) -- TStack::Dump()

/* -- FAULT-TRACE build switch (mirrors profiler.h's csProfile / PROFIT) --------------------------------
   Uncomment to compile the SEGFAULT-TRACKING machinery IN: the TThread SEH trampoline (catches a worker
   access-violation that /EHsc catch(...) cannot, prints the faulting thread's TStack chain + tid via
   DumpRaw, then records it through abFault) and the abTrace() frame markers. Ships OFF = ZERO cost: abTrace
   is a no-op and TThread::Run runs Execute() directly (an AV crashes through to WER as normal). Flip this ONE
   line to turn on-device / on-load fault attribution when a rare crash recurs. Removal: see
   memory `referenceFaultTraceMachinery`. */
// #define csFaultTrace

unsigned long abDbgTid(void); // current OS thread id (0 off-Windows) — tags the fault dump per thread

void abFault(LPCSTR tag); // record `tag`, dump the faulting thread's TStack chain, record the fault

// Diagnostics / tests: the most recently captured fault tag, or NULL if none has fired.
LPCSTR abFaultLastTag(void);

/* Portable pretty-function name for a TStack frame (mirrors thread.h's thisInfo, but kept local so the
   abFault core carries no dependency on the threading seam). */
#if defined(__GNUC__) || defined(__clang__)
   #define AB_STACK_FN __PRETTY_FUNCTION__
#elif defined(_MSC_VER)
   #define AB_STACK_FN __FUNCSIG__
#else
   #define AB_STACK_FN __func__
#endif

/* TStack -- per-thread RAII call-stack tracer (abFault core #15, stage 1). A TStack IS the list node: the
 * ctor links this frame as the head of THIS thread's chain, the dtor pops it. `root` lives in thread-local
 * storage (NULL at each thread's start), so every thread keeps its OWN independent chain -- and a fault,
 * caught on the same thread that raised it (/EHa on Windows, a synchronous signal on Linux), dumps exactly
 * that thread's chain. No fixed depth and no allocation on push/pop -- two pointer writes. Dump() renders the
 * current thread's chain innermost-frame-first ("inner/outer/"). Instrument a routine with `abTrace();`. */
class TStack {
   static thread_local TStack *root; // head of THIS thread's chain (innermost frame); NULL at thread start
   LPCSTR func;                 // frame name (a stable string literal / __PRETTY_FUNCTION__)
   TStack     *next;                 // the next-outer frame on this thread

 public:
   explicit TStack(LPCSTR fn) : func(fn), next(root)
   {
      root = this;
   }

   ~TStack()
   {
      root = next;
   }

   TStack(const TStack &) = delete;
   TStack &operator=(const TStack &) = delete;

   static std::string Dump(void); // this thread's chain, innermost first: "inner/outer/"
   static void DumpRaw(void);     // crash-safe chain: fprintf each frame (NO heap alloc; survives a corrupt heap)
};

/* abTrace() — push this function's name onto the thread's TStack for the scope. Gated on csFaultTrace so a
   non-fault-trace build pays nothing (mirrors PROFIT). The frames it leaves are what DumpRaw prints when the
   SEH trampoline catches an AV. */
#ifdef csFaultTrace
   #define abTrace() TStack _abTrace_(AB_STACK_FN)
#else
   #define abTrace() ((void)0)
#endif

#endif // SHARED_FAULT_H
