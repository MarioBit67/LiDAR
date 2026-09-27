#ifndef CAPBUF_H
#define CAPBUF_H
#include "winTypes.h"
#include "alloc.h"

#include <stddef.h>

/* Little-endian serialization for session records. TByteBuf grows a pool block; once an
 * allocation fails Ok() stays false and further puts are ignored. TByteReader is bounds
 * checked: an overrun latches Ok() false and every later read returns zero. */

class TByteBuf
{
 public:
   TByteBuf(void);

   void Clear(void) { Psize = 0u; }
   void PutByte(BYTE v);
   void PutWord(WORD v);
   void PutDword(DWORD v);
   void PutLong(LONG v);
   void PutQword(QWORD v);
   void PutFloat(float v);
   void PutBytes(LPCBYTE p, size_t n);
   void PutBlob(LPCBYTE p, size_t n); // DWORD length prefix + bytes

   LPCBYTE Data(void) const { return Pdata(); }
   size_t  Size(void) const { return Psize; }
   bool    Ok(void) const { return Pok; }

   TByteBuf(const TByteBuf &) = delete;
   TByteBuf &operator=(const TByteBuf &) = delete;

 private:
   bool reserveFor(size_t extra);

   TBlock<BYTE> Pdata;
   size_t       Psize,
                Pcap;
   bool         Pok;
};

class TByteReader
{
 public:
   TByteReader(LPCBYTE p, size_t n);

   BYTE    GetByte(void);
   WORD    GetWord(void);
   DWORD   GetDword(void);
   LONG    GetLong(void);
   QWORD   GetQword(void);
   float   GetFloat(void);
   bool    GetBytes(LPBYTE out, size_t n);
   LPCBYTE GetView(size_t n);                     // pointer into the source, NULL on overrun
   LPCBYTE GetBlob(DWORD &n);                     // DWORD length prefix, then a view
   DWORD   GetCount(size_t elemSize);             // element count that must fit in what is left

   bool Ok(void) const { return Pok; }
   bool AtEnd(void) const { return Ppos == Pn; }

 private:
   QWORD getLE(size_t n);

   LPCBYTE Pp;
   size_t  Pn,
           Ppos;
   bool    Pok;
};

#endif // CAPBUF_H
