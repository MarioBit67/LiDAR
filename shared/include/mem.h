#ifndef SHARED_MEM_H
#define SHARED_MEM_H
#include "winTypes.h" // winint typedefs (BYTE/WORD/DWORD/...)

#include <stddef.h>

/* TMemAlloc — RAII scoped pool buffer (the memory analog of the class arc / goldenRule 22: constructors and
 * destructors own the resource lifecycle). Owns ONE new[]'d block; the destructor delete[]'s it on EVERY
 * exit path — early return, exception, end of scope — so a routine that allocates / processes / releases
 * never leaks and never hand-frees. Pool-backed (new[]/delete[] route through the global operator new/delete
 * to the one shared pool; libc malloc/free are poisoned). Move-only: it holds a raw owner,
 * so copying would double-free (copy is deleted; move transfers ownership). OOM-aware: after construction
 * `!m` (or Get()==NULL) means the pool was dry — return abiErrPoolOom, nothing to clean up.
 *
 *   TMemAlloc m(n);
 *   if (!m)                       // pool OOM
 *      return abiErrPoolOom;
 *   TMyType *p = m.as<TMyType>(); // typed view of the block
 *   ... use p; return from anywhere — m frees on the way out ...
 */
class TMemAlloc
{
 public:
   TMemAlloc() : Pp(nullptr), Pn(0) {}                 // null/empty owner — the moved-from + failure-return state
   explicit TMemAlloc(size_t n) : Pp(new unsigned char[n]), Pn(n) {}
   ~TMemAlloc()
   {
      delete[] Pp;
   }

   TMemAlloc(const TMemAlloc &) = delete; // raw owner — not copyable (would double-free)
   TMemAlloc &operator=(const TMemAlloc &) = delete;

   TMemAlloc(TMemAlloc &&o) noexcept : Pp(o.Pp), Pn(o.Pn)
   {
      o.Pp = nullptr;
      o.Pn = 0;
   }
   TMemAlloc &operator=(TMemAlloc &&o) noexcept
   {
      if (this != &o)
      {
         delete[] Pp;
         Pp = o.Pp;
         Pn = o.Pn;
         o.Pp = nullptr;
         o.Pn = 0;
      }
      return *this;
   }

   LPVOID Get() const { return Pp; }                     // raw block (NULL on OOM)
   LPSTR Data() const { return (LPSTR)Pp; }            // byte view
   template <class T> T *As() const { return (T *)Pp; } // typed view: m.as<TMyType>()
   size_t Size() const { return Pn; }
   explicit operator bool() const { return Pp != nullptr; }

 private:
   unsigned char *Pp;
   size_t         Pn;
};

#endif // SHARED_MEM_H
