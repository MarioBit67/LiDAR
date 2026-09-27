#include "capEXIF.h"
#include "libDiscipline.h"

enum {
   exifMaxTags   = 12,
   exifMaxData   = 64,
   exifTypeByte  = 1,
   exifTypeAscii = 2,
   exifTypeShort = 3,
   exifTypeLong  = 4,
   exifTypeRatio = 5
};

// One IFD entry with its value bytes (inline when <= 4 bytes, else in the IFD data area)
struct TEXIFTag {
   WORD  tag, type;
   DWORD count, len;
   BYTE  data[exifMaxData];
};

// One image file directory being assembled
struct TEXIFDir {
   TEXIFTag tags[exifMaxTags];
   int      n;

   void Ascii(WORD tag, LPCSTR s);
   void Short(WORD tag, WORD v);
   void Long(WORD tag, DWORD v);
   void Bytes(WORD tag, LPCBYTE v, DWORD count);
   void Rationals(WORD tag, const DWORD *numDen, DWORD count); // count rationals = 2*count DWORDs
   DWORD Size(void) const;
   void Write(TByteBuf &out, DWORD offset) const; // offset of this IFD from the TIFF header
};

//--------------------------------------------------------------------------------
static void exifLE(LPBYTE p, DWORD v, int n)
{
   for (int i = 0; i < n; i++)
      p[i] = (BYTE)(v >> (8*i));
}

//--------------------------------------------------------------------------------
void TEXIFDir::Ascii(WORD tag, LPCSTR s)
{
   TEXIFTag &t = tags[n++];
   size_t    len = strnlen(s, exifMaxData - 1);

   t.tag = tag;
   t.type = exifTypeAscii;
   memcpy(t.data, s, len);
   t.data[len] = 0u;
   t.count = (DWORD)len + 1u;
   t.len = t.count;
}

//--------------------------------------------------------------------------------
void TEXIFDir::Short(WORD tag, WORD v)
{
   TEXIFTag &t = tags[n++];

   t.tag = tag;
   t.type = exifTypeShort;
   t.count = 1u;
   t.len = 2u;
   exifLE(t.data, v, 2);
}

//--------------------------------------------------------------------------------
void TEXIFDir::Long(WORD tag, DWORD v)
{
   TEXIFTag &t = tags[n++];

   t.tag = tag;
   t.type = exifTypeLong;
   t.count = 1u;
   t.len = 4u;
   exifLE(t.data, v, 4);
}

//--------------------------------------------------------------------------------
void TEXIFDir::Bytes(WORD tag, LPCBYTE v, DWORD count)
{
   TEXIFTag &t = tags[n++];

   t.tag = tag;
   t.type = exifTypeByte;
   t.count = count;
   t.len = count;
   memcpy(t.data, v, count);
}

//--------------------------------------------------------------------------------
void TEXIFDir::Rationals(WORD tag, const DWORD *numDen, DWORD count)
{
   TEXIFTag &t = tags[n++];

   t.tag = tag;
   t.type = exifTypeRatio;
   t.count = count;
   t.len = count*8u;
   for (DWORD i = 0; i < 2u*count; i++)
      exifLE(t.data + i*4u, numDen[i], 4);
}

//--------------------------------------------------------------------------------
DWORD TEXIFDir::Size(void) const
{
   DWORD size = 2u + 12u*(DWORD)n + 4u;

   for (int i = 0; i < n; i++)
      if (tags[i].len > 4u)
         size += (tags[i].len + 1u) & ~1u;
   return size;
}

//--------------------------------------------------------------------------------
void TEXIFDir::Write(TByteBuf &out, DWORD offset) const
{
   DWORD dataAt = offset + 2u + 12u*(DWORD)n + 4u;

   out.PutWord((WORD)n);
   for (int i = 0; i < n; i++)
   {
      const TEXIFTag &t = tags[i];
      BYTE            value[4] = { 0, 0, 0, 0 };

      out.PutWord(t.tag);
      out.PutWord(t.type);
      out.PutDword(t.count);
      if (t.len <= 4u)
      {
         memcpy(value, t.data, t.len);
         out.PutBytes(value, 4u);
      }
      else
      {
         out.PutDword(dataAt);
         dataAt += (t.len + 1u) & ~1u;
      }
   }
   out.PutDword(0u); // no next IFD
   for (int i = 0; i < n; i++)
      if (tags[i].len > 4u)
      {
         out.PutBytes(tags[i].data, tags[i].len);
         if (tags[i].len & 1u)
            out.PutByte(0u);
      }
}

//--------------------------------------------------------------------------------
BYTE exifOrientation(const TMat4 &cameraToWorld)
{
   // world up expressed in camera axes (x right, y up): the columns of R dotted with (0, 1, 0)
   float ux = cameraToWorld.m[1],
         uy = cameraToWorld.m[5];

   if (fabsf(uy) >= fabsf(ux))
      return uy >= 0.f ? 1u : 3u;
   return ux < 0.f ? 6u : 8u; // left edge up: rotate 90 clockwise; right edge up: 90 counter-clockwise
}

//--------------------------------------------------------------------------------
static void exifDegrees(LONG e7, DWORD out[6])
{
   DWORD v = (DWORD)(e7 < 0 ? -e7 : e7),
         deg = v/10000000u,
         remMin = (v%10000000u)*60u,           // minutes * 1e7
         minutes = remMin/10000000u,
         secMilli = (DWORD)(((QWORD)(remMin%10000000u)*60u*1000u)/10000000u);

   out[0] = deg;
   out[1] = 1u;
   out[2] = minutes;
   out[3] = 1u;
   out[4] = secMilli;
   out[5] = 1000u;
}

//--------------------------------------------------------------------------------
void exifBuild(const TEXIFInfo &info, TByteBuf &out)
{
   TEXIFDir   ifd0 = {},
              exif = {},
              gps = {};
   char       stamp[20];
   time_t     t = (time_t)(info.wallNs/1000000000u);
   struct tm *lt = localtime(&t);

   snprintf(stamp, sizeof(stamp), "%04d:%02d:%02d %02d:%02d:%02d", lt->tm_year + 1900, lt->tm_mon + 1, lt->tm_mday,
            lt->tm_hour, lt->tm_min, lt->tm_sec);

   ifd0.Ascii(0x0110, info.model);           // Model
   ifd0.Short(0x0112, info.orientation);     // Orientation
   ifd0.Ascii(0x0131, info.software);        // Software
   ifd0.Ascii(0x0132, stamp);                // DateTime
   ifd0.Long(0x8769, 0u);                    // ExifIFDPointer, patched below
   if (info.hasGPS)
      ifd0.Long(0x8825, 0u);                 // GPSInfo, patched below

   exif.Ascii(0x9003, stamp);                // DateTimeOriginal
   if (info.focalMm > 0.f)
   {
      DWORD focal[2] = { (DWORD)(info.focalMm*100.f + 0.5f), 100u };

      exif.Rationals(0x920A, focal, 1u);     // FocalLength
   }
   exif.Long(0xA002, info.width);            // PixelXDimension
   exif.Long(0xA003, info.height);           // PixelYDimension

   if (info.hasGPS)
   {
      BYTE  version[4] = { 2, 3, 0, 0 },
            altRef = info.altMm < 0 ? 1u : 0u;
      DWORD lat[6],
            lon[6],
            alt[2] = { (DWORD)(info.altMm < 0 ? -info.altMm : info.altMm), 1000u };

      exifDegrees(info.latE7, lat);
      exifDegrees(info.lonE7, lon);
      gps.Bytes(0x0000, version, 4u);
      gps.Ascii(0x0001, info.latE7 < 0 ? "S" : "N");
      gps.Rationals(0x0002, lat, 3u);
      gps.Ascii(0x0003, info.lonE7 < 0 ? "W" : "E");
      gps.Rationals(0x0004, lon, 3u);
      gps.Bytes(0x0005, &altRef, 1u);
      gps.Rationals(0x0006, alt, 1u);
   }

   DWORD ifd0At = 8u,
         exifAt = ifd0At + ifd0.Size(),
         gpsAt = exifAt + exif.Size();

   exifLE(ifd0.tags[4].data, exifAt, 4);
   if (info.hasGPS)
      exifLE(ifd0.tags[5].data, gpsAt, 4);

   out.PutBytes((LPCBYTE)"Exif\0", 6u);
   out.PutBytes((LPCBYTE)"II", 2u);
   out.PutWord(42u);
   out.PutDword(ifd0At);
   ifd0.Write(out, ifd0At);
   exif.Write(out, exifAt);
   if (info.hasGPS)
      gps.Write(out, gpsAt);
}

//--------------------------------------------------------------------------------
bool jpegInsertSegment(LPCBYTE jpg, size_t n, BYTE marker, LPCBYTE payload, size_t len, TByteBuf &out)
{
   if (n < 2u || jpg[0] != 0xFF || jpg[1] != 0xD8 || len > 65533u)
      return false;
   out.PutByte(0xFF);
   out.PutByte(0xD8);
   out.PutByte(0xFF);
   out.PutByte(marker);
   out.PutByte((BYTE)((len + 2u) >> 8));
   out.PutByte((BYTE)(len + 2u));
   out.PutBytes(payload, len);
   out.PutBytes(jpg + 2u, n - 2u);
   return out.Ok();
}

//--------------------------------------------------------------------------------
