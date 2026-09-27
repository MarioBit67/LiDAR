#include "capSession.h"
#include "sha256.h"
#ifdef _WIN32
#include <direct.h>
#else
#include <sys/stat.h>
#endif
#include "libDiscipline.h"

static LPCSTR cManifestFile = "session.json",
              cRecordFile = "capture.lrec";

//--------------------------------------------------------------------------------
static bool sessionMakeDir(LPCSTR dir)
{
#ifdef _WIN32
   int rc = _mkdir(dir);
#else
   int rc = mkdir(dir, 0755);
#endif

   return rc == 0 || errno == EEXIST;
}

//--------------------------------------------------------------------------------
static bool sessionExists(LPCSTR path)
{
   FILE *f = fopen(path, "rb");

   if (!f)
      return false;
   fclose(f);
   return true;
}

//--------------------------------------------------------------------------------
static void sessionJoin(LPSTR out, LPCSTR dir, LPCSTR name)
{
   snprintf(out, sessionPathMax, "%s/%s", dir, name);
}

//--------------------------------------------------------------------------------
static void sessionCopy(LPSTR dst, size_t cap, LPCSTR src)
{
   size_t n = src ? strnlen(src, cap - 1) : 0u;

   if (n)
      memcpy(dst, src, n);
   dst[n] = '\0';
}

//--------------------------------------------------------------------------------
// JSON string literal (quotes included) for a NUL-terminated UTF-8 string
static void sessionPutJSON(FILE *f, LPCSTR s)
{
   fputc('"', f);
   for (LPCSTR p = s; *p; p++)
   {
      unsigned char c = (unsigned char)*p;

      if (c == '"' || c == '\\')
      {
         fputc('\\', f);
         fputc(c, f);
      }
      else if (c < 0x20)
         fprintf(f, "\\u%04x", c);
      else
         fputc(c, f);
   }
   fputc('"', f);
}

//--------------------------------------------------------------------------------
TSessionWriter::TSessionWriter(void) : Pdev(), Pcounts(), Proom(), Pid(), PcreatedNs(0u), ProomMarkers(0u),
   Popen(false), ProomOpen(false)
{
   Pdir[0] = '\0';
}

//--------------------------------------------------------------------------------
TSessionWriter::~TSessionWriter(void)
{
   Close(PcreatedNs);
}

//--------------------------------------------------------------------------------
bool TSessionWriter::Open(LPCSTR dir, const TDeviceInfo &dev, QWORD wallNs)
{
   TMutexLock lock(Pmutex, thisInfo);
   char       path[sessionPathMax];

   if (Popen || !sessionMakeDir(dir))
      return false;
   sessionCopy(Pdir, sizeof(Pdir), dir);
   sessionJoin(path, Pdir, cManifestFile);
   if (sessionExists(path))
      return false;
   sessionJoin(path, Pdir, cRecordFile);
   if (sessionExists(path) || !Plog.Open(path))
      return false;
   Pdev = dev;

   TSHA256Ctx sha;
   BYTE       digest[32];

   sha256Init(&sha);
   sha256Update(&sha, (LPCBYTE)Pdir, strlen(Pdir));
   sha256Update(&sha, (LPCBYTE)&wallNs, sizeof(wallNs));
   sha256Update(&sha, (LPCBYTE)dev.model, strnlen(dev.model, sizeof(dev.model)));
   sha256Final(&sha, digest);
   memcpy(Pid, digest, sizeof(Pid));
   Pcounts = TSessionCounts();
   PcreatedNs = wallNs;
   ProomMarkers = 0u;
   ProomOpen = false;
   Popen = true;
   return writeManifest(false, wallNs);
}

//--------------------------------------------------------------------------------
bool TSessionWriter::writeLocked(QWORD stampNs, TRecordType type, LPQWORD counter)
{
   if (!Popen || !Pscratch.Ok())
      return false;
   if (!Plog.Write(stampNs, type, Pscratch.Data(), Pscratch.Size()))
      return false;
   (*counter)++;
   Pcounts.bytes = Plog.Bytes();
   return true;
}

//--------------------------------------------------------------------------------
bool TSessionWriter::WritePose(QWORD stampNs, const TPoseRecord &r)
{
   TMutexLock lock(Pmutex, thisInfo);

   Pscratch.Clear();
   r.Encode(Pscratch);
   return writeLocked(stampNs, rtPose, &Pcounts.poses);
}

//--------------------------------------------------------------------------------
bool TSessionWriter::WriteImage(QWORD stampNs, const TImageRecord &r)
{
   TMutexLock lock(Pmutex, thisInfo);

   Pscratch.Clear();
   r.Encode(Pscratch);
   return writeLocked(stampNs, rtImage, &Pcounts.images);
}

//--------------------------------------------------------------------------------
bool TSessionWriter::WriteDepth(QWORD stampNs, const TDepthRecord &r)
{
   TMutexLock lock(Pmutex, thisInfo);

   Pscratch.Clear();
   r.Encode(Pscratch);
   return writeLocked(stampNs, rtDepth, &Pcounts.depth);
}

//--------------------------------------------------------------------------------
bool TSessionWriter::WriteMesh(QWORD stampNs, const TMeshRecord &r)
{
   TMutexLock lock(Pmutex, thisInfo);

   Pscratch.Clear();
   r.Encode(Pscratch);
   return writeLocked(stampNs, rtMesh, &Pcounts.meshes);
}

//--------------------------------------------------------------------------------
bool TSessionWriter::WriteLocation(QWORD stampNs, const TLocationRecord &r)
{
   TMutexLock lock(Pmutex, thisInfo);

   Pscratch.Clear();
   r.Encode(Pscratch);
   return writeLocked(stampNs, rtLocation, &Pcounts.locations);
}

//--------------------------------------------------------------------------------
bool TSessionWriter::WriteStation(QWORD stampNs, const TStationRecord &r)
{
   TMutexLock lock(Pmutex, thisInfo);

   Pscratch.Clear();
   r.Encode(Pscratch);
   return writeLocked(stampNs, rtStation, &Pcounts.stations);
}

//--------------------------------------------------------------------------------
bool TSessionWriter::WriteVanish(QWORD stampNs, const TVanishRecord &r)
{
   TMutexLock lock(Pmutex, thisInfo);

   Pscratch.Clear();
   r.Encode(Pscratch);
   return writeLocked(stampNs, rtVanish, &Pcounts.vanish);
}

//--------------------------------------------------------------------------------
bool TSessionWriter::WriteLayout(QWORD stampNs, const TLayoutRecord &r)
{
   TMutexLock lock(Pmutex, thisInfo);

   Pscratch.Clear();
   r.Encode(Pscratch);
   return writeLocked(stampNs, rtLayout, &Pcounts.layouts);
}

//--------------------------------------------------------------------------------
bool TSessionWriter::WriteElect(QWORD stampNs, const TElectRecord &r)
{
   TMutexLock lock(Pmutex, thisInfo);

   Pscratch.Clear();
   r.Encode(Pscratch);
   return writeLocked(stampNs, rtElect, &Pcounts.elects);
}

//--------------------------------------------------------------------------------
bool TSessionWriter::WriteDoor(QWORD stampNs, const TDoorRecord &r)
{
   TMutexLock lock(Pmutex, thisInfo);

   Pscratch.Clear();
   r.Encode(Pscratch);
   return writeLocked(stampNs, rtDoor, &Pcounts.doors);
}

/*--------------------------------------------------------------------------------
   The log copied record by record into capture.lrec.tmp, the dropped images left out, then put in
   place of the original and reopened for appending. The original goes only once the copy is whole.
  --------------------------------------------------------------------------------*/
int TSessionWriter::Compact(LPCQWORD drop, int count)
{
   TMutexLock       lock(Pmutex, thisInfo);
   char             path[sessionPathMax],
                    tmp[sessionPathMax];
   TRecordLogReader in;
   TRecordLogWriter out;
   TRecordView      v;
   int              dropped = 0;
   bool             ok = true;

   if (!Popen || !drop || count <= 0)
      return 0;
   sessionJoin(path, Pdir, cRecordFile);
   snprintf(tmp, sizeof(tmp), "%s.tmp", path);
   Plog.Close();
   if (!in.Open(path) || !out.Open(tmp))
   {
      Plog.Append(path);
      return 0;
   }
   while (ok && in.Next(v))
   {
      bool skip = false;

      for (int i = 0; v.type == rtImage && i < count && !skip; i++)
         skip = drop[i] == v.stampNs;
      if (skip)
      {
         dropped++;
         continue;
      }
      ok = out.Write(v.stampNs, v.type, v.payload, v.length);
   }
   in.Close();
   out.Close();
   if (!ok)
   {
      remove(tmp);
      Plog.Append(path);
      return 0;
   }
   remove(path); // rename does not replace on every platform
   rename(tmp, path);
   Plog.Append(path);
   Pcounts.images -= (QWORD)dropped;
   Pcounts.bytes = Plog.Bytes();
   return dropped;
}

//--------------------------------------------------------------------------------
DWORD TSessionWriter::BeginRoom(QWORD stampNs, LPCSTR name)
{
   TMutexLock lock(Pmutex, thisInfo);

   endRoomLocked(stampNs, NAN);
   Proom.event = reBegin;
   Proom.index = Pcounts.rooms++;
   sessionCopy(Proom.name, sizeof(Proom.name), name);
   Proom.cameraHeightM = NAN;
   Pscratch.Clear();
   Proom.Encode(Pscratch);
   ProomOpen = writeLocked(stampNs, rtRoom, &ProomMarkers);
   return Proom.index;
}

//--------------------------------------------------------------------------------
void TSessionWriter::EndRoom(QWORD stampNs, float cameraHeightM)
{
   TMutexLock lock(Pmutex, thisInfo);

   endRoomLocked(stampNs, cameraHeightM);
}

//--------------------------------------------------------------------------------
void TSessionWriter::endRoomLocked(QWORD stampNs, float cameraHeightM)
{
   if (!ProomOpen)
      return;
   ProomOpen = false;
   Proom.event = reEnd;
   Proom.cameraHeightM = cameraHeightM;
   Pscratch.Clear();
   Proom.Encode(Pscratch);
   writeLocked(stampNs, rtRoom, &ProomMarkers);
   Plog.Flush();
}

//--------------------------------------------------------------------------------
bool TSessionWriter::RoomOpen(void) const
{
   TMutexLock lock(Pmutex, thisInfo);

   return ProomOpen;
}

//--------------------------------------------------------------------------------
void TSessionWriter::Close(QWORD wallNs)
{
   TMutexLock lock(Pmutex, thisInfo);

   if (!Popen)
      return;
   endRoomLocked(wallNs, NAN);
   Plog.Flush();
   Plog.Close();
   writeManifest(true, wallNs);
   Popen = false;
}

//--------------------------------------------------------------------------------
bool TSessionWriter::IsOpen(void) const
{
   TMutexLock lock(Pmutex, thisInfo);

   return Popen;
}

//--------------------------------------------------------------------------------
void TSessionWriter::SessionId(BYTE out[16]) const
{
   TMutexLock lock(Pmutex, thisInfo);

   memcpy(out, Pid, sizeof(Pid));
}

//--------------------------------------------------------------------------------
TSessionCounts TSessionWriter::Counts(void) const
{
   TMutexLock lock(Pmutex, thisInfo);

   return Pcounts;
}

//--------------------------------------------------------------------------------
// Write-then-rename so a crash never leaves a half-written manifest
bool TSessionWriter::writeManifest(bool complete, QWORD wallNs)
{
   char  tmp[sessionPathMax],
         path[sessionPathMax];
   FILE *f;

   sessionJoin(path, Pdir, cManifestFile);
   snprintf(tmp, sizeof(tmp), "%s.tmp", path);
   f = fopen(tmp, "wb");
   if (!f)
      return false;
   fprintf(f, "{\r\n");
   fprintf(f, "  \"format\": \"lidar-session\",\r\n");
   fprintf(f, "  \"version\": 1,\r\n");
   fprintf(f, "  \"sessionId\": \"");
   for (int i = 0; i < 16; i++)
      fprintf(f, "%02x", Pid[i]);
   fprintf(f, "\",\r\n");
   fprintf(f, "  \"status\": \"%s\",\r\n", complete ? "complete" : "recording");
   fprintf(f, "  \"createdUnixNs\": %llu,\r\n", (unsigned long long)PcreatedNs);
   fprintf(f, "  \"closedUnixNs\": %llu,\r\n", (unsigned long long)(complete ? wallNs : 0u));
   fprintf(f, "  \"records\": \"%s\",\r\n", cRecordFile);
   fprintf(f, "  \"world\": \"x east, y up, -z north\",\r\n");
   fprintf(f, "  \"pose\": \"camera to world, column-major; camera x right, y up, -z forward\",\r\n");
   fprintf(f, "  \"device\": {\r\n    \"platform\": ");
   sessionPutJSON(f, Pdev.platform);
   fprintf(f, ",\r\n    \"model\": ");
   sessionPutJSON(f, Pdev.model);
   fprintf(f, ",\r\n    \"osVersion\": ");
   sessionPutJSON(f, Pdev.osVersion);
   fprintf(f, ",\r\n    \"appVersion\": ");
   sessionPutJSON(f, Pdev.appVersion);
   fprintf(f, ",\r\n    \"camera\": ");
   sessionPutJSON(f, Pdev.camera);
   fprintf(f, ",\r\n    \"depthSource\": ");
   sessionPutJSON(f, Pdev.depthSource);
   fprintf(f, ",\r\n    \"headingRef\": ");
   sessionPutJSON(f, Pdev.headingRef);
   fprintf(f, "\r\n  },\r\n");
   fprintf(f, "  \"counts\": {\"rooms\": %lu, \"images\": %llu, \"depth\": %llu, \"meshes\": %llu, ",
           (unsigned long)Pcounts.rooms, (unsigned long long)Pcounts.images, (unsigned long long)Pcounts.depth,
           (unsigned long long)Pcounts.meshes);
   fprintf(f, "\"poses\": %llu, \"locations\": %llu, \"stations\": %llu, ", (unsigned long long)Pcounts.poses,
           (unsigned long long)Pcounts.locations, (unsigned long long)Pcounts.stations);
   fprintf(f, "\"vanish\": %llu, \"layouts\": %llu, \"elects\": %llu, \"doors\": %llu, \"bytes\": %llu}\r\n",
           (unsigned long long)Pcounts.vanish, (unsigned long long)Pcounts.layouts, (unsigned long long)Pcounts.elects,
           (unsigned long long)Pcounts.doors, (unsigned long long)Pcounts.bytes);
   fprintf(f, "}\r\n");

   bool ok = fclose(f) == 0;

   remove(path);
   return ok && rename(tmp, path) == 0;
}

//--------------------------------------------------------------------------------
bool TSessionReader::Open(LPCSTR dir)
{
   char path[sessionPathMax];

   sessionJoin(path, dir, cRecordFile);
   return Plog.Open(path);
}

//--------------------------------------------------------------------------------
