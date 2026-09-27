#include "capLog.h"
#include "libDiscipline.h"

enum {
   logFileHeader   = 8,
   logRecordHeader = 16,
   logMaxRecord    = 256 << 20
};

//--------------------------------------------------------------------------------
static void logPutLE(LPBYTE p, QWORD v, int n)
{
   for (int i = 0; i < n; i++)
      p[i] = (BYTE)(v >> (8*i));
}

//--------------------------------------------------------------------------------
static QWORD logGetLE(LPCBYTE p, int n)
{
   QWORD v = 0u;

   for (int i = 0; i < n; i++)
      v |= (QWORD)p[i] << (8*i);
   return v;
}

//--------------------------------------------------------------------------------
TRecordLogWriter::TRecordLogWriter(void) : Pfile(NULL), Pbytes(0u)
{
}

//--------------------------------------------------------------------------------
TRecordLogWriter::~TRecordLogWriter(void)
{
   Close();
}

//--------------------------------------------------------------------------------
bool TRecordLogWriter::Open(LPCSTR path)
{
   BYTE header[logFileHeader];

   Close();
   Pfile = fopen(path, "wb");
   if (!Pfile)
      return false;
   logPutLE(header, logMagic, 4);
   logPutLE(header + 4, logVersion, 4);
   if (fwrite(header, 1u, sizeof(header), Pfile) != sizeof(header))
   {
      Close();
      return false;
   }
   Pbytes = sizeof(header);
   return true;
}

//--------------------------------------------------------------------------------
bool TRecordLogWriter::Write(QWORD stampNs, TRecordType type, LPCBYTE payload, size_t length)
{
   BYTE header[logRecordHeader];

   if (!Pfile || length > (size_t)logMaxRecord)
      return false;
   logPutLE(header, stampNs, 8);
   logPutLE(header + 8, (QWORD)type, 2);
   logPutLE(header + 10, 0u, 2);
   logPutLE(header + 12, (QWORD)length, 4);
   if (fwrite(header, 1u, sizeof(header), Pfile) != sizeof(header))
      return false;
   if (length && fwrite(payload, 1u, length, Pfile) != length)
      return false;
   Pbytes += sizeof(header) + length;
   return true;
}

//--------------------------------------------------------------------------------
void TRecordLogWriter::Flush(void)
{
   if (Pfile)
      fflush(Pfile);
}

//--------------------------------------------------------------------------------
void TRecordLogWriter::Close(void)
{
   if (Pfile)
   {
      fclose(Pfile);
      Pfile = NULL;
   }
}

//--------------------------------------------------------------------------------
TRecordLogReader::TRecordLogReader(void) : Pfile(NULL), Pcap(0u), Ptruncated(false)
{
}

//--------------------------------------------------------------------------------
TRecordLogReader::~TRecordLogReader(void)
{
   Close();
}

//--------------------------------------------------------------------------------
bool TRecordLogReader::Open(LPCSTR path)
{
   BYTE header[logFileHeader];

   Close();
   Ptruncated = false;
   Pfile = fopen(path, "rb");
   if (!Pfile)
      return false;
   if (fread(header, 1u, sizeof(header), Pfile) != sizeof(header)
       || logGetLE(header, 4) != (QWORD)logMagic
       || logGetLE(header + 4, 4) != (QWORD)logVersion)
   {
      Close();
      return false;
   }
   return true;
}

//--------------------------------------------------------------------------------
bool TRecordLogReader::Next(TRecordView &view)
{
   BYTE   header[logRecordHeader];
   size_t got;

   if (!Pfile)
      return false;
   got = fread(header, 1u, sizeof(header), Pfile);
   if (got == 0u)
      return false;
   if (got != sizeof(header))
   {
      Ptruncated = true;
      return false;
   }

   DWORD length = (DWORD)logGetLE(header + 12, 4);

   if (length > (DWORD)logMaxRecord)
   {
      Ptruncated = true;
      return false;
   }
   if (length > Pcap)
   {
      TAlloc<BYTE> fresh(length);

      fresh.Drop(Pbuf);
      Pcap = length;
   }
   if (length && fread(Pbuf(), 1u, length, Pfile) != length)
   {
      Ptruncated = true;
      return false;
   }
   view.stampNs = logGetLE(header, 8);
   view.type = (TRecordType)logGetLE(header + 8, 2);
   view.payload = Pbuf();
   view.length = length;
   return true;
}

//--------------------------------------------------------------------------------
void TRecordLogReader::Close(void)
{
   if (Pfile)
   {
      fclose(Pfile);
      Pfile = NULL;
   }
}

//--------------------------------------------------------------------------------
