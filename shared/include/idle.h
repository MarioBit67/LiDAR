/*
 * idle.h — demote the current process to IDLE scheduling priority.
 *
 * Part of the shared/ integration layer. Call abSetIdlePriority() ONCE at the top
 * of main()/wmain(). Per goldenRules 5a, an idle host process means every thread it
 * spawns AND every child process it launches INHERITS idle — so heavy compute never
 * makes the host unresponsive. This is the per-process backstop to the host-idle
 * inheritance the parallel test runner relies on.
 */
#ifndef IDLE_H
#define IDLE_H

#if defined(_WIN32)

   #include <windows.h>

//--------------------------------------------------------------------------------
   static inline void abSetIdlePriority(void)
   {
      SetPriorityClass(GetCurrentProcess(), IDLE_PRIORITY_CLASS);
   }

#else

   #include <unistd.h>

//--------------------------------------------------------------------------------
   static inline void abSetIdlePriority(void)
   {
      (void)nice(19);
   }

#endif

#endif
