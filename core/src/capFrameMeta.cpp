#include "capFrameMeta.h"
#include "libDiscipline.h"

static const BYTE cMetaId[frameMetaIdLen] = { 'L', 'I', 'D', 'A', 'R', 'C', 'A', 'P', 0 },
                  cPad[4] = { 0, 0, 0, 0 }; // the reserved tail of the 300-byte blocks

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
// Blur in centipixels plus one: 0 is "not measured" (NaN), so the zeros of older blocks read right
static WORD metaBlurWord(float px)
{
   if (isnan(px) || px < 0.f)
      return 0u;
   return px >= 655.f ? (WORD)0xFFFFu : (WORD)(px*100.f + 1.5f);
}

//--------------------------------------------------------------------------------
static float metaBlurPx(WORD w)
{
   return w ? (float)(w - 1u)/100.f : NAN;
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
   out.PutBytes(cPad, 4u);
   for (int i = 0; i < 3; i++)
      out.PutFloat(forward[i]);
   out.PutFloat(rollDeg);
   out.PutWord(metaBlurWord(blurPx)); // the last 4 bytes of the block
   out.PutWord(metaBlurWord(blurMinPx));
   while (out.Ok() && out.Size() - start < (size_t)frameMetaSize)
      out.PutByte(0u); // reserved
}

//--------------------------------------------------------------------------------
bool TFrameMeta::Decode(LPCBYTE p, size_t n)
{
   TByteReader r(p, n);
   BYTE        id[frameMetaIdLen];

   if ((n != (size_t)frameMetaSize && n != (size_t)frameMetaSizeV1) || !r.GetBytes(id, frameMetaIdLen)
       || memcmp(id, cMetaId, frameMetaIdLen))
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
   if (n == (size_t)frameMetaSize)
   {
      BYTE pad[4];

      r.GetBytes(pad, 4u);
      for (int i = 0; i < 3; i++)
         forward[i] = r.GetFloat();
      rollDeg = r.GetFloat();
      blurPx = metaBlurPx(r.GetWord()); // zeros in blocks written before the blur
      blurMinPx = metaBlurPx(r.GetWord());
   }
   else
   {
      // a 300-byte block: the same attitude from its pose
      TVec3 f = cameraToWorld.Forward();

      forward[0] = f.x;
      forward[1] = f.y;
      forward[2] = f.z;
      rollDeg = geomRollDeg(cameraToWorld);
      blurPx = NAN;
      blurMinPx = NAN;
   }
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
