#ifndef CAPSESSION_H
#define CAPSESSION_H
#include "winTypes.h"
#include "thread.h"
#include "capLog.h"

/* A capture session is one directory:
 *   session.json  manifest (device, status, record counts)
 *   capture.lrec  every record, interleaved in arrival order
 * Every platform writes the same format, so one offline pipeline rebuilds every property.
 * The writer is thread-safe: platform callbacks may arrive on different threads. */

enum {
   sessionPathMax = 512
};

struct TDeviceInfo {
   char platform[16],   // "android", "ios"
        model[64],
        osVersion[32],
        appVersion[16],
        camera[48],     // e.g. "ultra-wide 0.5x"
        depthSource[16], // "lidar", "tof", "none"
        headingRef[16];  // "true", "magnetic", "arbitrary" (no magnetometer: one global yaw solved offline)
};

struct TSessionCounts {
   DWORD rooms;
   QWORD poses, images, depth, meshes, locations, stations, vanish, layouts, bytes;
};

class TSessionWriter
{
 public:
   TSessionWriter(void);

   ~TSessionWriter(void);

   // Creates dir (its parent must exist); refuses a directory that already holds a session
   bool Open(LPCSTR dir, const TDeviceInfo &dev, QWORD wallNs);

   bool WritePose(QWORD stampNs, const TPoseRecord &r);
   bool WriteImage(QWORD stampNs, const TImageRecord &r);
   bool WriteDepth(QWORD stampNs, const TDepthRecord &r);
   bool WriteMesh(QWORD stampNs, const TMeshRecord &r);
   bool WriteLocation(QWORD stampNs, const TLocationRecord &r);
   bool WriteStation(QWORD stampNs, const TStationRecord &r);
   bool WriteVanish(QWORD stampNs, const TVanishRecord &r);
   bool WriteLayout(QWORD stampNs, const TLayoutRecord &r);

   // A room brackets one 360-degree spin; beginning a room ends the open one
   DWORD BeginRoom(QWORD stampNs, LPCSTR name);
   void  EndRoom(QWORD stampNs, float cameraHeightM);
   bool  RoomOpen(void) const;

   // Ends the open room, flushes and marks the manifest complete; idempotent
   void Close(QWORD wallNs);

   bool           IsOpen(void) const;
   void           SessionId(BYTE out[16]) const; // derived at Open (SHA-256 of dir, time, device)
   TSessionCounts Counts(void) const;

   TSessionWriter(const TSessionWriter &) = delete;
   TSessionWriter &operator=(const TSessionWriter &) = delete;
   TSessionWriter(TSessionWriter &&) = delete;
   TSessionWriter &operator=(TSessionWriter &&) = delete;

 private:
   bool writeLocked(QWORD stampNs, TRecordType type, LPQWORD counter);
   void endRoomLocked(QWORD stampNs, float cameraHeightM);
   bool writeManifest(bool complete, QWORD wallNs);

   mutable TMutex   Pmutex;
   TRecordLogWriter Plog;
   TByteBuf         Pscratch;
   TDeviceInfo      Pdev;
   TSessionCounts   Pcounts;
   TRoomRecord      Proom;
   char             Pdir[sessionPathMax];
   BYTE             Pid[16];
   QWORD            PcreatedNs,
                    ProomMarkers;
   bool             Popen,
                    ProomOpen;
};

class TSessionReader
{
 public:
   bool Open(LPCSTR dir);
   bool Next(TRecordView &view) { return Plog.Next(view); }
   bool Truncated(void) const { return Plog.Truncated(); }

 private:
   TRecordLogReader Plog;
};

#endif // CAPSESSION_H
