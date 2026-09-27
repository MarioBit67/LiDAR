#ifndef CAPFRAMEMETA_H
#define CAPFRAMEMETA_H
#include "winTypes.h"
#include "capGeom.h"
#include "capRecord.h"

/* Our per-frame metadata, embedded in every keyframe JPEG as an APP11 segment ("LIDARCAP"), so a
 * frame stays self-describing outside its session. It carries the frame-to-model correlation in two
 * layers: the DEVICE layer (filled by the app at capture time: identity, clocks, pose, compass, GNSS,
 * lens, spin band/bin) and the SERVER layer (zero at capture; a later pipeline refines the pose and
 * binds the frame to the room model). The block has a FIXED size, so the server rewrites its layer
 * in place without re-encoding or moving a single byte of the image. */

enum {
   frameMetaVersion = 1,
   frameMetaIdLen   = 9, // "LIDARCAP\0"
   frameMetaSize    = 300
};

enum THeadingRef {
   hrArbitrary = 0, // no magnetometer: yaw relative to the session start
   hrMagnetic  = 1,
   hrTrue      = 2
};

struct TFrameMeta {
   // -- device layer --
   BYTE        sessionId[16];
   DWORD       roomIndex, seq; // seq: keyframe number within the session
   QWORD       sensorNs, wallNs;
   TMat4       cameraToWorld;
   TCompass    compass;
   THeadingRef headingRef;
   TIntrinsics intr;
   float       distortion[5],
               headingDeg,     // camera forward, clockwise from the heading reference
               pitchDeg;       // camera forward above the horizon
   BYTE        spinBand, spinBin;
   LONG        latE7, lonE7, altMm; // last GNSS fix (0 when none)
   DWORD       horizAccMm;          // 0 when no fix
   QWORD       fixAgeNs;            // frame time minus fix time
   // -- server layer (zero at capture) --
   DWORD       refineFlags;         // bit 0: refined pose valid, bit 1: model binding valid
   TMat4       refinedCameraToWorld;
   DWORD       modelRoomId,
               modelFaceMask;       // which room surfaces the frame sees (bit per wall, floor, ceiling)
   // -- device layer, appended in the reserved tail (older blocks read these as 0) --
   BYTE        stationIndex,
               stationKind,         // TStationKind
               cornerIndex,         // corner the operator stands in (corner stations)
               targetCorner;        // corner aimed at (corner stations)
   float       focalMm;             // lens focal length (0 in older blocks)
   float       roomAxisDeg,         // vanishing points: world heading of room axis A, [0, 90)
               axisDevDeg;          // against the room reference (NaN when not judged)
   WORD        tiltErrCdeg,         // measured vertical vs gravity, centidegrees (0xFFFF unknown)
               orthoErrCdeg;        // orthogonality error, centidegrees (0xFFFF unknown)
   BYTE        vanishFlags,         // TVanishFlag bits; 0 = not measured (older blocks)
               axisVerdict;         // TAxisVerdict

   void Encode(TByteBuf &out) const;           // exactly frameMetaSize bytes
   bool Decode(LPCBYTE p, size_t n);
   static LPCBYTE Find(LPCBYTE jpg, size_t n, size_t *len); // locate the block inside a JPEG
};

#endif // CAPFRAMEMETA_H
