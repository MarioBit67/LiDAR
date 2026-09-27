#ifndef CAPEXIF_H
#define CAPEXIF_H
#include "winTypes.h"
#include "capGeom.h"
#include "capBuf.h"

/* Standard EXIF (APP1) for our keyframes, so any viewer shows them upright and dated while the
 * pixels stay in the native sensor orientation the intrinsics refer to. Orientation comes from the
 * actual pose (which image edge gravity points to), not from an assumed way of holding the phone. */

struct TEXIFInfo {
   char  model[64],
         software[32];
   QWORD wallNs;             // capture time, Unix epoch
   BYTE  orientation;        // EXIF 1, 3, 6 or 8
   float focalMm;            // 0 = omit
   DWORD width, height;
   bool  hasGPS;
   LONG  latE7, lonE7, altMm;
};

// EXIF orientation that shows the frame upright: from where world-up falls in the image
BYTE exifOrientation(const TMat4 &cameraToWorld);

// The APP1 payload ("Exif\0\0" + little-endian TIFF), without the marker and length
void exifBuild(const TEXIFInfo &info, TByteBuf &out);

// Copies a JPEG inserting one APPn segment right after SOI (no re-encode)
bool jpegInsertSegment(LPCBYTE jpg, size_t n, BYTE marker, LPCBYTE payload, size_t len, TByteBuf &out);

#endif // CAPEXIF_H
