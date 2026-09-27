#ifndef CAPLOG_H
#define CAPLOG_H
#include "winTypes.h"
#include "alloc.h"
#include "capRecord.h"

#include <stdio.h>

/* Append-only, time-ordered log of typed records. A crash mid-write only loses the truncated
 * tail record; everything before it stays readable.
 *   file header: magic "LREC" DWORD | version DWORD
 *   record:      stampNs QWORD | type WORD | reserved WORD | length DWORD | payload[length] */

enum {
   logMagic   = 0x4345524C, // "LREC"
   logVersion = 1
};

// One record as read back; payload is valid until the next Next()
struct TRecordView {
   QWORD       stampNs;
   TRecordType type;
   LPCBYTE     payload;
   DWORD       length;
};

class TRecordLogWriter
{
 public:
   TRecordLogWriter(void);

   ~TRecordLogWriter(void);

   bool  Open(LPCSTR path);
   bool  Write(QWORD stampNs, TRecordType type, LPCBYTE payload, size_t length);
   void  Flush(void);
   void  Close(void);
   QWORD Bytes(void) const { return Pbytes; }

   TRecordLogWriter(const TRecordLogWriter &) = delete;
   TRecordLogWriter &operator=(const TRecordLogWriter &) = delete;

 private:
   FILE *Pfile;
   QWORD Pbytes;
};

class TRecordLogReader
{
 public:
   TRecordLogReader(void);

   ~TRecordLogReader(void);

   bool Open(LPCSTR path);
   bool Next(TRecordView &view); // false at end of file or on a truncated tail
   bool Truncated(void) const { return Ptruncated; }
   void Close(void);

   TRecordLogReader(const TRecordLogReader &) = delete;
   TRecordLogReader &operator=(const TRecordLogReader &) = delete;

 private:
   FILE        *Pfile;
   TBlock<BYTE> Pbuf;
   size_t       Pcap;
   bool         Ptruncated;
};

#endif // CAPLOG_H
