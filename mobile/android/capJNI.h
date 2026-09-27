#ifndef CAPJNI_H
#define CAPJNI_H
#include "winTypes.h"

/* ==================================================================================================
   capJNI - the ONE Java-bridge seam of the Android adapter. A generic, literal-free reflection layer:
   TAndroid passes every class name, method name and JVM signature as a PARAMETER, so the only file that
   touches the VM (capJNI.cpp) carries no string or number literal. Objects cross as opaque LPVOID
   handles; local references live inside a jniFrameBegin/jniFrameEnd pair, long-lived ones are globals.
   A pending Java exception is cleared and reported through jniFailed().
   ================================================================================================== */

enum {
   jniMaxArgs = 8
};

enum TJArgType {
   jatInt,
   jatLong,
   jatFloat,
   jatBool,
   jatString, // NUL-terminated UTF-8, converted to a java.lang.String
   jatObject
};

struct TJArg {
   TJArgType type;
   QWORD     i; // int / long / bool payload (two's complement)
   float     f;
   LPCSTR    s;
   LPVOID    o;
};

TJArg jniInt(QWORD v, TJArgType type); // jatInt / jatLong / jatBool
TJArg jniFloat(float v);
TJArg jniStr(LPCSTR s);
TJArg jniObjArg(LPVOID o);

bool   jniInit(LPVOID javaVM, LPVOID activity); // the ANativeActivity vm + clazz
bool   jniAttach(void);                         // attach the calling thread (idempotent)
LPVOID jniActivity(void);                       // global ref to the activity
void   jniFrameBegin(int capacity);             // local-reference scope
void   jniFrameEnd(void);                       // releases every local of the scope
LPVOID jniGlobal(LPVOID local);                 // promote a local to a global
void   jniDropGlobal(LPVOID global);            // release a global
bool   jniFailed(void);                         // a call raised (and cleared) an exception; resets on read

LPVOID jniNew(LPCSTR cls, LPCSTR ctor, LPCSTR sig, const TJArg *args, int n);   // ctor = "<init>"
// Instance calls on obj; name + JVM signature come from the caller
LPVOID jniCallObj(LPVOID obj, LPCSTR name, LPCSTR sig, const TJArg *args, int n); // object result
int    jniCallInt(LPVOID obj, LPCSTR name, LPCSTR sig, const TJArg *args, int n); // int result
QWORD  jniCallLong(LPVOID obj, LPCSTR name, LPCSTR sig, const TJArg *args, int n); // long result
float  jniCallFloat(LPVOID obj, LPCSTR name, LPCSTR sig, const TJArg *args, int n); // float result
bool   jniCallBool(LPVOID obj, LPCSTR name, LPCSTR sig, const TJArg *args, int n); // boolean result
LONG   jniCallFixed(LPVOID obj, LPCSTR name, LPCSTR sig, LONG scale);            // no-arg fp64 getter * scale, rounded
void   jniCallVoid(LPVOID obj, LPCSTR name, LPCSTR sig, const TJArg *args, int n); // no result
LPVOID jniStaticObj(LPCSTR cls, LPCSTR name, LPCSTR sig, const TJArg *args, int n); // static method, object result
LPVOID jniStaticField(LPCSTR cls, LPCSTR name, LPCSTR sig);                      // static object field
LPVOID jniStringArray(LPCSTR const *strs, int n);                                // java.lang.String[]
bool   jniLockBitmap(LPVOID bitmap, LPBYTE *pixels, int *width, int *height, int *stride); // ARGB_8888 = R, G, B, A bytes
void   jniUnlockBitmap(LPVOID bitmap);                                           // pairs jniLockBitmap

#endif // CAPJNI_H
