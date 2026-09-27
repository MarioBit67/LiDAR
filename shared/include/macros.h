/*
 * macros.h — small, dependency-free utility macros shared across the monorepo.
 *
 * Part of the shared/ integration layer. Include where you need ARRAYSIZE etc.
 */
#ifndef MACROS_H
#define MACROS_H

/* ARRAYSIZE(v) — element count of a fixed-size (non-decayed) array. Guarded: the
   Windows SDK (winnt.h) defines an identical macro, so this folds to a no-op when
   windows.h is already included. */
#ifndef ARRAYSIZE
   #define ARRAYSIZE(v) (sizeof(v)/sizeof(*(v)))
#endif

#endif
