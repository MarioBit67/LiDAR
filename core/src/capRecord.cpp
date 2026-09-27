#include "capRecord.h"
#include "libDiscipline.h"

//--------------------------------------------------------------------------------
static void putMat(TByteBuf &out, const TMat4 &m)
{
   for (int i = 0; i < 16; i++)
      out.PutFloat(m.m[i]);
}

//--------------------------------------------------------------------------------
static TMat4 getMat(TByteReader &r)
{
   TMat4 m;

   for (int i = 0; i < 16; i++)
      m.m[i] = r.GetFloat();
   return m;
}

//--------------------------------------------------------------------------------
static void putCompass(TByteBuf &out, const TCompass &c)
{
   out.PutFloat(c.magneticDeg);
   out.PutFloat(c.trueDeg);
   out.PutFloat(c.accuracyDeg);
}

//--------------------------------------------------------------------------------
static TCompass getCompass(TByteReader &r)
{
   TCompass c;

   c.magneticDeg = r.GetFloat();
   c.trueDeg = r.GetFloat();
   c.accuracyDeg = r.GetFloat();
   return c;
}

//--------------------------------------------------------------------------------
static void putIntr(TByteBuf &out, const TIntrinsics &k)
{
   out.PutFloat(k.fx);
   out.PutFloat(k.fy);
   out.PutFloat(k.cx);
   out.PutFloat(k.cy);
}

//--------------------------------------------------------------------------------
static TIntrinsics getIntr(TByteReader &r)
{
   TIntrinsics k;

   k.fx = r.GetFloat();
   k.fy = r.GetFloat();
   k.cx = r.GetFloat();
   k.cy = r.GetFloat();
   return k;
}

//--------------------------------------------------------------------------------
static DWORD depthSampleBytes(TPixFmt f)
{
   if (f == pfDepthF32)
      return 4u;
   if (f == pfDepthU16)
      return 2u;
   return 0u;
}

//--------------------------------------------------------------------------------
void TPoseRecord::Encode(TByteBuf &out) const
{
   putMat(out, cameraToWorld);
   putCompass(out, compass);
   out.PutByte((BYTE)tracking);
}

//--------------------------------------------------------------------------------
bool TPoseRecord::Decode(LPCBYTE p, size_t n)
{
   TByteReader r(p, n);

   cameraToWorld = getMat(r);
   compass = getCompass(r);
   tracking = (TTrackState)r.GetByte();
   return r.Ok() && r.AtEnd();
}

//--------------------------------------------------------------------------------
void TImageRecord::Encode(TByteBuf &out) const
{
   out.PutDword(width);
   out.PutDword(height);
   out.PutByte((BYTE)format);
   putIntr(out, intr);
   for (int i = 0; i < 5; i++)
      out.PutFloat(distortion[i]);
   putMat(out, cameraToWorld);
   putCompass(out, compass);
   out.PutBlob(pixels, pixelBytes);
}

//--------------------------------------------------------------------------------
bool TImageRecord::Decode(LPCBYTE p, size_t n)
{
   TByteReader r(p, n);

   width = r.GetDword();
   height = r.GetDword();
   format = (TPixFmt)r.GetByte();
   intr = getIntr(r);
   for (int i = 0; i < 5; i++)
      distortion[i] = r.GetFloat();
   cameraToWorld = getMat(r);
   compass = getCompass(r);
   pixels = r.GetBlob(pixelBytes);
   return r.Ok() && r.AtEnd();
}

//--------------------------------------------------------------------------------
float TDepthRecord::DepthAt(DWORD x, DWORD y) const
{
   size_t i = (size_t)y*width + x;

   if (format == pfDepthF32)
   {
      float v;

      memcpy(&v, samples + i*4u, 4u);
      return v;
   }

   WORD mm;

   memcpy(&mm, samples + i*2u, 2u);
   return (float)mm*0.001f;
}

//--------------------------------------------------------------------------------
void TDepthRecord::Encode(TByteBuf &out) const
{
   out.PutDword(width);
   out.PutDword(height);
   out.PutByte((BYTE)format);
   putIntr(out, intr);
   putMat(out, cameraToWorld);
   out.PutBlob(samples, sampleBytes);
   out.PutBlob(confidence, confidence ? confidenceBytes : 0u);
}

//--------------------------------------------------------------------------------
bool TDepthRecord::Decode(LPCBYTE p, size_t n)
{
   TByteReader r(p, n);

   width = r.GetDword();
   height = r.GetDword();
   format = (TPixFmt)r.GetByte();
   intr = getIntr(r);
   cameraToWorld = getMat(r);
   samples = r.GetBlob(sampleBytes);
   confidence = r.GetBlob(confidenceBytes);
   if (!confidenceBytes)
      confidence = NULL;
   if (!r.Ok() || !r.AtEnd())
      return false;

   QWORD pixels = (QWORD)width*height,
         sample = depthSampleBytes(format);

   if (!sample || sampleBytes != pixels*sample)
      return false;
   return !confidence || confidenceBytes == pixels;
}

//--------------------------------------------------------------------------------
TVec3 TMeshRecord::Vertex(DWORD i) const
{
   TVec3 v;

   memcpy(&v.x, vertices + (size_t)i*12u, 4u);
   memcpy(&v.y, vertices + (size_t)i*12u + 4u, 4u);
   memcpy(&v.z, vertices + (size_t)i*12u + 8u, 4u);
   return v;
}

//--------------------------------------------------------------------------------
DWORD TMeshRecord::Index(DWORD i) const
{
   DWORD v;

   memcpy(&v, indices + (size_t)i*4u, 4u);
   return v;
}

//--------------------------------------------------------------------------------
void TMeshRecord::Encode(TByteBuf &out) const
{
   out.PutBytes(id, 16u);
   putMat(out, localToWorld);
   out.PutDword(vertexCount);
   out.PutBytes(vertices, (size_t)vertexCount*12u);
   out.PutDword(indexCount);
   out.PutBytes(indices, (size_t)indexCount*4u);
}

//--------------------------------------------------------------------------------
bool TMeshRecord::Decode(LPCBYTE p, size_t n)
{
   TByteReader r(p, n);

   r.GetBytes(id, 16u);
   localToWorld = getMat(r);
   vertexCount = r.GetCount(12u);
   vertices = r.GetView((size_t)vertexCount*12u);
   indexCount = r.GetCount(4u);
   indices = r.GetView((size_t)indexCount*4u);
   if (!r.Ok() || !r.AtEnd() || indexCount%3u != 0u)
      return false;
   for (DWORD i = 0; i < indexCount; i++)
      if (Index(i) >= vertexCount)
         return false;
   return true;
}

//--------------------------------------------------------------------------------
void TLocationRecord::Encode(TByteBuf &out) const
{
   out.PutLong(latE7);
   out.PutLong(lonE7);
   out.PutLong(altMm);
   out.PutDword(horizAccMm);
   out.PutDword(vertAccMm);
   out.PutQword(fixNs);
}

//--------------------------------------------------------------------------------
bool TLocationRecord::Decode(LPCBYTE p, size_t n)
{
   TByteReader r(p, n);

   latE7 = r.GetLong();
   lonE7 = r.GetLong();
   altMm = r.GetLong();
   horizAccMm = r.GetDword();
   vertAccMm = r.GetDword();
   fixNs = r.GetQword();
   return r.Ok() && r.AtEnd();
}

//--------------------------------------------------------------------------------
void TStationRecord::Encode(TByteBuf &out) const
{
   out.PutByte((BYTE)event);
   out.PutDword(roomIndex);
   out.PutDword(index);
   out.PutByte((BYTE)kind);
   out.PutByte(corner);
   out.PutByte(target);
}

//--------------------------------------------------------------------------------
bool TStationRecord::Decode(LPCBYTE p, size_t n)
{
   TByteReader r(p, n);

   event = (TStationEvent)r.GetByte();
   roomIndex = r.GetDword();
   index = r.GetDword();
   kind = (TStationKind)r.GetByte();
   corner = r.GetByte();
   target = r.GetByte();
   if (!r.Ok() || !r.AtEnd())
      return false;
   return (event == seBegin || event == seEnd) && (kind == skCenter || kind == skCorner);
}

//--------------------------------------------------------------------------------
void TRoomRecord::Encode(TByteBuf &out) const
{
   size_t len = strnlen(name, roomNameMax - 1);

   out.PutByte((BYTE)event);
   out.PutDword(index);
   out.PutBlob((LPCBYTE)name, len);
   out.PutFloat(cameraHeightM);
}

//--------------------------------------------------------------------------------
bool TRoomRecord::Decode(LPCBYTE p, size_t n)
{
   TByteReader r(p, n);
   DWORD       len = 0u;

   event = (TRoomEvent)r.GetByte();
   index = r.GetDword();

   LPCBYTE text = r.GetBlob(len);

   cameraHeightM = r.GetFloat();
   if (!r.Ok() || !r.AtEnd() || len >= roomNameMax)
      return false;
   if (event != reBegin && event != reEnd)
      return false;
   memcpy(name, text, len);
   name[len] = '\0';
   return true;
}

//--------------------------------------------------------------------------------
void TVanishRecord::Encode(TByteBuf &out) const
{
   out.PutDword(roomIndex);
   out.PutDword(seq);
   out.PutByte(flags);
   out.PutByte(verdict);
   out.PutFloat(roomAxisDeg);
   out.PutFloat(deviationDeg);
   out.PutFloat(tiltErrDeg);
   out.PutFloat(orthoErrDeg);
   for (int i = 0; i < 3; i++)
   {
      out.PutFloat(dirCam[i].x);
      out.PutFloat(dirCam[i].y);
      out.PutFloat(dirCam[i].z);
   }
   for (int i = 0; i < 3; i++)
      out.PutDword(support[i]);
   out.PutDword(edges);
}

//--------------------------------------------------------------------------------
bool TVanishRecord::Decode(LPCBYTE p, size_t n)
{
   TByteReader r(p, n);

   roomIndex = r.GetDword();
   seq = r.GetDword();
   flags = r.GetByte();
   verdict = r.GetByte();
   roomAxisDeg = r.GetFloat();
   deviationDeg = r.GetFloat();
   tiltErrDeg = r.GetFloat();
   orthoErrDeg = r.GetFloat();
   for (int i = 0; i < 3; i++)
   {
      dirCam[i].x = r.GetFloat();
      dirCam[i].y = r.GetFloat();
      dirCam[i].z = r.GetFloat();
   }
   for (int i = 0; i < 3; i++)
      support[i] = r.GetDword();
   edges = r.GetDword();
   return r.Ok() && r.AtEnd();
}

//--------------------------------------------------------------------------------
void TLayoutRecord::Encode(TByteBuf &out) const
{
   BYTE verts = vertexCount < layoutRecMaxVerts ? vertexCount : (BYTE)layoutRecMaxVerts,
        stations = stationCount < layoutRecMaxStations ? stationCount : (BYTE)layoutRecMaxStations;

   out.PutDword(roomIndex);
   out.PutByte(flags);
   out.PutByte(verts);
   out.PutByte(stations);
   out.PutFloat(axisDeg);
   out.PutFloat(ceilingM);
   out.PutFloat(assumedCeilingM);
   out.PutFloat(cameraHeightM);
   out.PutFloat(doorScale);
   for (int i = 0; i < verts; i++)
   {
      out.PutFloat(u[i]);
      out.PutFloat(w[i]);
   }
   for (int i = 0; i < stations; i++)
   {
      out.PutByte(stationVertex[i]);
      out.PutByte(targetVertex[i]);
   }
}

//--------------------------------------------------------------------------------
bool TLayoutRecord::Decode(LPCBYTE p, size_t n)
{
   TByteReader r(p, n);

   roomIndex = r.GetDword();
   flags = r.GetByte();
   vertexCount = r.GetByte();
   stationCount = r.GetByte();
   if (vertexCount > layoutRecMaxVerts || stationCount > layoutRecMaxStations)
      return false;
   axisDeg = r.GetFloat();
   ceilingM = r.GetFloat();
   assumedCeilingM = r.GetFloat();
   cameraHeightM = r.GetFloat();
   doorScale = r.GetFloat();
   for (int i = 0; i < vertexCount; i++)
   {
      u[i] = r.GetFloat();
      w[i] = r.GetFloat();
   }
   for (int i = 0; i < stationCount; i++)
   {
      stationVertex[i] = r.GetByte();
      targetVertex[i] = r.GetByte();
   }
   return r.Ok() && r.AtEnd();
}

//--------------------------------------------------------------------------------
