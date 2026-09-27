#ifndef CAPRECORD_H
#define CAPRECORD_H
#include "winTypes.h"
#include "capGeom.h"
#include "capBuf.h"

/* Typed payloads of a capture session (conventions in capGeom.h). Pixel, depth and mesh
 * buffers are non-owning VIEWS: on Encode they point at the caller's buffer, after Decode
 * they point into the payload passed in, which must outlive the record. */

enum TRecordType {
   rtPose     = 1, // camera attitude (+ position when tracked) with compass, sensor rate
   rtImage    = 2, // kept camera frame
   rtDepth    = 3, // depth map aligned with an image
   rtMesh     = 4, // reconstructed surface chunk (ARKit mesh anchor)
   rtLocation = 5, // GNSS fix
   rtRoom     = 6, // room begin / end marker
   rtStation  = 7, // capture station inside a room (center spin, corner views)
   rtVanish   = 8, // vanishing directions measured on a keyframe (room axes, gravity and intrinsics checks)
   rtLayout   = 9  // floor plan of a room estimated after its center spin (editable later in the property plan)
};

enum TTrackState {
   tsNone    = 0,
   tsLimited = 1,
   tsNormal  = 2
};

enum TPixFmt {
   pfJPEG    = 1,
   pfNV21    = 2,
   pfDepthF32 = 10, // meters, float
   pfDepthU16 = 11  // millimeters, WORD
};

// NaN marks an unknown value
struct TCompass {
   float magneticDeg, trueDeg, accuracyDeg;
};

struct TPoseRecord {
   TMat4       cameraToWorld;
   TCompass    compass;
   TTrackState tracking;

   void Encode(TByteBuf &out) const;
   bool Decode(LPCBYTE p, size_t n);
};

struct TImageRecord {
   DWORD       width, height;
   TPixFmt     format;
   TIntrinsics intr;
   float       distortion[5]; // k1 k2 k3 p1 p2; zero when the ISP already corrected the image
   TMat4       cameraToWorld;
   TCompass    compass;
   LPCBYTE     pixels; // view
   DWORD       pixelBytes;

   void Encode(TByteBuf &out) const;
   bool Decode(LPCBYTE p, size_t n);
};

struct TDepthRecord {
   DWORD       width, height;
   TPixFmt     format;
   TIntrinsics intr;
   TMat4       cameraToWorld;
   LPCBYTE     samples;    // view, width*height depth samples
   DWORD       sampleBytes;
   LPCBYTE     confidence; // view, one byte per pixel (0 low, 1 medium, 2 high), may be NULL
   DWORD       confidenceBytes;

   float DepthAt(DWORD x, DWORD y) const; // meters
   void  Encode(TByteBuf &out) const;
   bool  Decode(LPCBYTE p, size_t n);
};

struct TMeshRecord {
   BYTE    id[16]; // stable across updates of the same chunk
   TMat4   localToWorld;
   LPCBYTE vertices; // view, vertexCount * 3 floats (local frame)
   DWORD   vertexCount;
   LPCBYTE indices;  // view, indexCount DWORDs, triangles
   DWORD   indexCount;

   TVec3 Vertex(DWORD i) const;
   DWORD Index(DWORD i) const;
   void  Encode(TByteBuf &out) const;
   bool  Decode(LPCBYTE p, size_t n);
};

// Q-format integers: degrees * 1e7 (~1 cm), millimeters
struct TLocationRecord {
   LONG  latE7, lonE7, altMm; // altitude above the WGS84 ellipsoid
   DWORD horizAccMm, vertAccMm;
   QWORD fixNs;               // when the receiver computed the fix

   void Encode(TByteBuf &out) const;
   bool Decode(LPCBYTE p, size_t n);
};

enum TRoomEvent {
   reBegin = 1,
   reEnd   = 2
};

enum {
   roomNameMax = 64
};

struct TRoomRecord {
   TRoomEvent event;
   DWORD      index;             // increments per room within the session
   char       name[roomNameMax]; // NUL-terminated UTF-8
   float      cameraHeightM;     // above the floor (NaN unknown); metric scale of the crease layout

   void Encode(TByteBuf &out) const;
   bool Decode(LPCBYTE p, size_t n);
};

enum TStationEvent {
   seBegin = 1,
   seEnd   = 2
};

enum TStationKind {
   skCenter = 0, // in-place spin from the middle of the room
   skCorner = 1  // standing in a corner, aiming at the opposite one: the parallax baseline
};

// One capture station of a room; corners are numbered clockwise from the first one visited
struct TStationRecord {
   TStationEvent event;
   DWORD         roomIndex, index;
   TStationKind  kind;
   BYTE          corner, target; // corner the operator stands in / corner aimed at (skCorner only)

   void Encode(TByteBuf &out) const;
   bool Decode(LPCBYTE p, size_t n);
};

/* Vanishing directions of one keyframe (capVanish) and its verdict against the room axes (TAxisVerdict).
 * Directions are camera axes: 0 vertical, 1 room axis A, 2 room axis B; NaN angles are unknown. */
struct TVanishRecord {
   DWORD roomIndex, seq;  // keyframe the measure belongs to (seq as in its LIDARCAP block)
   BYTE  flags,           // TVanishFlag bits of the measured directions
         verdict;         // TAxisVerdict
   float roomAxisDeg,     // world heading of room axis A, [0, 90)
         deviationDeg,    // against the room reference, signed
         tiltErrDeg,      // measured vertical vs gravity
         orthoErrDeg;     // worst deviation from orthogonality between measured directions
   TVec3 dirCam[3];
   DWORD support[3],
         edges;

   void Encode(TByteBuf &out) const;
   bool Decode(LPCBYTE p, size_t n);
};

enum {
   layoutRecMaxVerts    = 16,
   layoutRecMaxStations = 8
};

enum TLayoutFlag {
   lfValid        = 1,
   lfComplete     = 2, // the walls cover every heading around the spin point
   lfHeightSolved = 4, // camera height from paired ceiling / floor creases
   lfDoorScaled   = 8  // scale taken from door heads at 2.10 m (the ceiling height is then a measure)
};

/* Floor plan of one room (capLayout), in plan meters: origin at the spin point, u along the room axis at
 * world heading axisDeg, w along axisDeg + 90; corners clockwise seen from above. The property plan
 * places each room later (the operator may re-position rooms), so the plan stays in its own frame. */
struct TLayoutRecord {
   DWORD roomIndex;
   BYTE  flags,            // TLayoutFlag
         vertexCount,
         stationCount;
   float axisDeg,
         ceilingM,         // measured when lfDoorScaled, else the assumption
         assumedCeilingM,
         cameraHeightM,
         doorScale,
         u[layoutRecMaxVerts],
         w[layoutRecMaxVerts];
   BYTE  stationVertex[layoutRecMaxStations],
         targetVertex[layoutRecMaxStations];

   void Encode(TByteBuf &out) const;
   bool Decode(LPCBYTE p, size_t n);
};

#endif // CAPRECORD_H
