#include "capFrameMeta.h"
#include "libDiscipline.h"

static const BYTE cMetaId[frameMetaIdLen] = { 'L', 'I', 'D', 'A', 'R', 'C', 'A', 'P', 0 };

//--------------------------------------------------------------------------------
static void metaPutMat(TByteBuf &out, const TMat4 &m)
{
   for (int i = 0; i < 16; i++)
      out.PutFloat(m.m[i]);
}

//--------------------------------------------------------------------------------
static TMat4 metaGetMat(TByteReader &r)
{
   TMat4 m;

   for (int i = 0; i < 16; i++)
      m.m[i] = r.GetFloat();
   return m;
}

//--------------------------------------------------------------------------------
void TFrameMeta::Encode(TByteBuf &out) const
{
   size_t start = out.Size();

   out.PutBytes(cMetaId, frameMetaIdLen);
   out.PutWord(frameMetaVersion);
   out.PutBytes(sessionId, 16u);
   out.PutDword(roomIndex);
   out.PutDword(seq);
   out.PutQword(sensorNs);
   out.PutQword(wallNs);
   metaPutMat(out, cameraToWorld);
   out.PutFloat(compass.magneticDeg);
   out.PutFloat(compass.trueDeg);
   out.PutFloat(compass.accuracyDeg);
   out.PutByte((BYTE)headingRef);
   out.PutFloat(intr.fx);
   out.PutFloat(intr.fy);
   out.PutFloat(intr.cx);
   out.PutFloat(intr.cy);
   for (int i = 0; i < 5; i++)
      out.PutFloat(distortion[i]);
   out.PutFloat(headingDeg);
   out.PutFloat(pitchDeg);
   out.PutByte(spinBand);
   out.PutByte(spinBin);
   out.PutLong(latE7);
   out.PutLong(lonE7);
   out.PutLong(altMm);
   out.PutDword(horizAccMm);
   out.PutQword(fixAgeNs);
   out.PutDword(refineFlags);
   metaPutMat(out, refinedCameraToWorld);
   out.PutDword(modelRoomId);
   out.PutDword(modelFaceMask);
   out.PutByte(stationIndex);
   out.PutByte(stationKind);
   out.PutByte(cornerIndex);
   out.PutByte(targetCorner);
   out.PutFloat(focalMm);
   out.PutFloat(roomAxisDeg);
   out.PutFloat(axisDevDeg);
   out.PutWord(tiltErrCdeg);
   out.PutWord(orthoErrCdeg);
   out.PutByte(vanishFlags);
   out.PutByte(axisVerdict);
   while (out.Ok() && out.Size() - start < (size_t)frameMetaSize)
      out.PutByte(0u); // reserved
}

//--------------------------------------------------------------------------------
bool TFrameMeta::Decode(LPCBYTE p, size_t n)
{
   TByteReader r(p, n);
   BYTE        id[frameMetaIdLen];

   if (n != (size_t)frameMetaSize || !r.GetBytes(id, frameMetaIdLen) || memcmp(id, cMetaId, frameMetaIdLen))
      return false;
   if (r.GetWord() != frameMetaVersion)
      return false;
   r.GetBytes(sessionId, 16u);
   roomIndex = r.GetDword();
   seq = r.GetDword();
   sensorNs = r.GetQword();
   wallNs = r.GetQword();
   cameraToWorld = metaGetMat(r);
   compass.magneticDeg = r.GetFloat();
   compass.trueDeg = r.GetFloat();
   compass.accuracyDeg = r.GetFloat();
   headingRef = (THeadingRef)r.GetByte();
   intr.fx = r.GetFloat();
   intr.fy = r.GetFloat();
   intr.cx = r.GetFloat();
   intr.cy = r.GetFloat();
   for (int i = 0; i < 5; i++)
      distortion[i] = r.GetFloat();
   headingDeg = r.GetFloat();
   pitchDeg = r.GetFloat();
   spinBand = r.GetByte();
   spinBin = r.GetByte();
   latE7 = r.GetLong();
   lonE7 = r.GetLong();
   altMm = r.GetLong();
   horizAccMm = r.GetDword();
   fixAgeNs = r.GetQword();
   refineFlags = r.GetDword();
   refinedCameraToWorld = metaGetMat(r);
   modelRoomId = r.GetDword();
   modelFaceMask = r.GetDword();
   stationIndex = r.GetByte(); // reserved zeros in blocks written before the station fields
   stationKind = r.GetByte();
   cornerIndex = r.GetByte();
   targetCorner = r.GetByte();
   focalMm = r.GetFloat();
   roomAxisDeg = r.GetFloat();
   axisDevDeg = r.GetFloat();
   tiltErrCdeg = r.GetWord();
   orthoErrCdeg = r.GetWord();
   vanishFlags = r.GetByte();
   axisVerdict = r.GetByte();
   return r.Ok();
}

//--------------------------------------------------------------------------------
LPCBYTE TFrameMeta::Find(LPCBYTE jpg, size_t n, size_t *len)
{
   size_t pos = 2u;

   if (n < 4u || jpg[0] != 0xFF || jpg[1] != 0xD8)
      return NULL;
   while (pos + 4u <= n && jpg[pos] == 0xFF)
   {
      BYTE   marker = jpg[pos + 1u];
      size_t segLen = ((size_t)jpg[pos + 2u] << 8) | jpg[pos + 3u];

      if (marker == 0xDA || segLen < 2u || pos + 2u + segLen > n)
         break;
      if (marker == 0xEB && segLen - 2u >= frameMetaIdLen && !memcmp(jpg + pos + 4u, cMetaId, frameMetaIdLen))
      {
         *len = segLen - 2u;
         return jpg + pos + 4u;
      }
      pos += 2u + segLen;
   }
   return NULL;
}

//--------------------------------------------------------------------------------
