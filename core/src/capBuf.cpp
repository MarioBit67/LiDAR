#include "capBuf.h"
#include "libDiscipline.h"

//--------------------------------------------------------------------------------
TByteBuf::TByteBuf(void) : Psize(0u), Pcap(0u), Pok(true)
{
}

//--------------------------------------------------------------------------------
bool TByteBuf::reserveFor(size_t extra)
{
   if (!Pok)
      return false;
   if (Psize + extra <= Pcap)
      return true;

   size_t cap = Pcap ? Pcap : 256u;

   while (cap < Psize + extra)
      cap *= 2u;

   TAlloc<BYTE> fresh(cap);

   if (!fresh)
   {
      Pok = false;
      return false;
   }
   if (Psize)
      memcpy(fresh(), Pdata(), Psize);
   fresh.Drop(Pdata);
   Pcap = cap;
   return true;
}

//--------------------------------------------------------------------------------
void TByteBuf::PutBytes(LPCBYTE p, size_t n)
{
   if (!n || !reserveFor(n))
      return;
   memcpy(Pdata() + Psize, p, n);
   Psize += n;
}

//--------------------------------------------------------------------------------
void TByteBuf::PutByte(BYTE v)
{
   PutBytes(&v, 1u);
}

//--------------------------------------------------------------------------------
void TByteBuf::PutWord(WORD v)
{
   BYTE b[2];

   b[0] = (BYTE)v;
   b[1] = (BYTE)(v >> 8);
   PutBytes(b, 2u);
}

//--------------------------------------------------------------------------------
void TByteBuf::PutDword(DWORD v)
{
   BYTE b[4];

   for (int i = 0; i < 4; i++)
      b[i] = (BYTE)(v >> (8*i));
   PutBytes(b, 4u);
}

//--------------------------------------------------------------------------------
void TByteBuf::PutLong(LONG v)
{
   PutDword((DWORD)v);
}

//--------------------------------------------------------------------------------
void TByteBuf::PutQword(QWORD v)
{
   BYTE b[8];

   for (int i = 0; i < 8; i++)
      b[i] = (BYTE)(v >> (8*i));
   PutBytes(b, 8u);
}

//--------------------------------------------------------------------------------
void TByteBuf::PutFloat(float v)
{
   DWORD bits;

   memcpy(&bits, &v, 4u);
   PutDword(bits);
}

//--------------------------------------------------------------------------------
void TByteBuf::PutBlob(LPCBYTE p, size_t n)
{
   PutDword((DWORD)n);
   PutBytes(p, n);
}

//--------------------------------------------------------------------------------
TByteReader::TByteReader(LPCBYTE p, size_t n) : Pp(p), Pn(n), Ppos(0u), Pok(true)
{
}

//--------------------------------------------------------------------------------
LPCBYTE TByteReader::GetView(size_t n)
{
   if (!Pok || Pn - Ppos < n)
   {
      Pok = false;
      return NULL;
   }

   LPCBYTE v = Pp + Ppos;

   Ppos += n;
   return v;
}

//--------------------------------------------------------------------------------
QWORD TByteReader::getLE(size_t n)
{
   LPCBYTE v = GetView(n);
   QWORD   r = 0u;

   if (!v)
      return 0u;
   for (size_t i = 0; i < n; i++)
      r |= (QWORD)v[i] << (8u*i);
   return r;
}

//--------------------------------------------------------------------------------
BYTE TByteReader::GetByte(void)
{
   return (BYTE)getLE(1u);
}

//--------------------------------------------------------------------------------
WORD TByteReader::GetWord(void)
{
   return (WORD)getLE(2u);
}

//--------------------------------------------------------------------------------
DWORD TByteReader::GetDword(void)
{
   return (DWORD)getLE(4u);
}

//--------------------------------------------------------------------------------
LONG TByteReader::GetLong(void)
{
   return (LONG)GetDword();
}

//--------------------------------------------------------------------------------
QWORD TByteReader::GetQword(void)
{
   return getLE(8u);
}

//--------------------------------------------------------------------------------
float TByteReader::GetFloat(void)
{
   DWORD bits = GetDword();
   float v;

   memcpy(&v, &bits, 4u);
   return v;
}

//--------------------------------------------------------------------------------
bool TByteReader::GetBytes(LPBYTE out, size_t n)
{
   LPCBYTE v = GetView(n);

   if (!v)
      return false;
   memcpy(out, v, n);
   return true;
}

//--------------------------------------------------------------------------------
LPCBYTE TByteReader::GetBlob(DWORD &n)
{
   n = GetDword();
   return GetView(n);
}

//--------------------------------------------------------------------------------
DWORD TByteReader::GetCount(size_t elemSize)
{
   DWORD n = GetDword();

   if (!Pok || (Pn - Ppos)/elemSize < n)
   {
      Pok = false;
      return 0u;
   }
   return n;
}

//--------------------------------------------------------------------------------
