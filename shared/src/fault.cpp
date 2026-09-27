/* fault.cpp -- MINIMAL STUB (see fault.h) + the TStack call-stack tracer (abFault core #15, stage 1).
 * abFault records the tag and dumps the FAULTING thread's TStack chain (the fault is caught on the same
 * thread that raised it), then returns -- no crash-through, so a caught fault stays attributable during
 * bring-up. TODO(roadmapAbFault): swap in the central capture/minidump handler here. */

#ifndef SHARED_LIBRARY_BUILD
#define SHARED_LIBRARY_BUILD
#endif

#include <stdio.h>
#include <string.h>
#ifdef _WIN32
   #include <windows.h> // GetCurrentThreadId for abDbgTid (before the poison)
#endif

#include "fault.h"
#include "libDiscipline.h" // allocator/fp64/env poison -- last (this TU uses none of them)

//--------------------------------------------------------------------------------------------------
unsigned long abDbgTid(void)
{
#ifdef _WIN32
   return (unsigned long)GetCurrentThreadId();
#else
   return 0ul;
#endif
}

static char gLastFaultTag[256];
static bool gHaveFault = false;
// -- TStack: per-thread RAII call-stack tracer (intrusive node; root in thread-local storage) ------
thread_local TStack *TStack::root = nullptr;

//--------------------------------------------------------------------------------------------------
std::string TStack::Dump(void)
{
   std::string s;

   for (TStack *p = root; p != nullptr; p = p->next) // innermost (root) first, out to the entry frame
   {
      s += (p->func != nullptr) ? p->func : "?";
      s += "/";
   }
   return s;
}

/* DumpRaw -- crash-safe variant of Dump(): fprintf each frame directly (frame names are static string
   literals / __FUNCSIG__, safe to read) with a per-line flush and NO heap allocation, so the chain survives
   even when the fault corrupted the heap (which would make Dump()'s std::string allocation itself fault). */

//--------------------------------------------------------------------------------------------------
void TStack::DumpRaw(void)
{
   int depth = 0;

   for (TStack *p = root; p != nullptr && depth < 64; p = p->next, depth++)
   {
      fprintf(stderr, "[abFault] frame[%d]: %s\n", depth, (p->func != nullptr) ? p->func : "?");
      fflush(stderr);
   }
}

//--------------------------------------------------------------------------------------------------
void abFault(LPCSTR tag)
{
   snprintf(gLastFaultTag, sizeof gLastFaultTag, "%s", (tag != nullptr) ? tag : "(null)");
   gHaveFault = true;

   fprintf(stderr, "[abFault] %s\n", gLastFaultTag);
   fprintf(stderr, "[abFault] stack: %s\n", TStack::Dump().c_str());
   fflush(stderr);

   /* STUB: record + dump + return. The real handler (goldenRule 26) will capture a minidump on a reserved
    * stack and decide recovery vs clean abort; until then a recorded fault does not crash-through, so the
    * locator can catch the fault, dump the chain, and keep running. */
}

//--------------------------------------------------------------------------------------------------
LPCSTR abFaultLastTag(void)
{
   return gHaveFault ? gLastFaultTag : nullptr;
}
