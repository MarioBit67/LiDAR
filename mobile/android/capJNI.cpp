#include <jni.h>
#include <android/bitmap.h>
#include "capJNI.h"
#include "libDiscipline.h"

static JavaVM          *gVM = NULL;
static jobject          gActivity = NULL;
static thread_local JNIEnv *gEnv = NULL;
static thread_local bool    gFailed = false;

//--------------------------------------------------------------------------------
TJArg jniInt(QWORD v, TJArgType type)
{
   TJArg a = {};

   a.type = type;
   a.i = v;
   return a;
}

//--------------------------------------------------------------------------------
TJArg jniFloat(float v)
{
   TJArg a = {};

   a.type = jatFloat;
   a.f = v;
   return a;
}

//--------------------------------------------------------------------------------
TJArg jniStr(LPCSTR s)
{
   TJArg a = {};

   a.type = jatString;
   a.s = s;
   return a;
}

//--------------------------------------------------------------------------------
TJArg jniObjArg(LPVOID o)
{
   TJArg a = {};

   a.type = jatObject;
   a.o = o;
   return a;
}

//--------------------------------------------------------------------------------
bool jniInit(LPVOID javaVM, LPVOID activity)
{
   gVM = (JavaVM *)javaVM;
   if (!jniAttach())
      return false;
   gActivity = gEnv->NewGlobalRef((jobject)activity);
   return gActivity != NULL;
}

//--------------------------------------------------------------------------------
bool jniAttach(void)
{
   if (gEnv)
      return true;
   if (!gVM)
      return false;
   if (gVM->GetEnv((LPVOID *)&gEnv, JNI_VERSION_1_6) == JNI_OK)
      return true;
   gEnv = NULL;
   return gVM->AttachCurrentThread(&gEnv, NULL) == JNI_OK;
}

//--------------------------------------------------------------------------------
LPVOID jniActivity(void)
{
   return gActivity;
}

//--------------------------------------------------------------------------------
void jniFrameBegin(int capacity)
{
   if (jniAttach())
      gEnv->PushLocalFrame(capacity);
}

//--------------------------------------------------------------------------------
void jniFrameEnd(void)
{
   if (gEnv)
      gEnv->PopLocalFrame(NULL);
}

//--------------------------------------------------------------------------------
LPVOID jniGlobal(LPVOID local)
{
   if (!local || !jniAttach())
      return NULL;
   return gEnv->NewGlobalRef((jobject)local);
}

//--------------------------------------------------------------------------------
void jniDropGlobal(LPVOID global)
{
   if (global && jniAttach())
      gEnv->DeleteGlobalRef((jobject)global);
}

//--------------------------------------------------------------------------------
static bool jniCheck(void)
{
   if (!gEnv->ExceptionCheck())
      return true;
   gEnv->ExceptionClear();
   gFailed = true;
   return false;
}

//--------------------------------------------------------------------------------
bool jniFailed(void)
{
   bool f = gFailed;

   gFailed = false;
   return f;
}

//--------------------------------------------------------------------------------
static void jniPack(const TJArg *args, int n, jvalue *out)
{
   for (int i = 0; i < n && i < jniMaxArgs; i++)
      switch (args[i].type)
      {
         case jatInt :
            out[i].i = (jint)args[i].i;
            break;

         case jatLong :
            out[i].j = (jlong)args[i].i;
            break;

         case jatFloat :
            out[i].f = args[i].f;
            break;

         case jatBool :
            out[i].z = args[i].i ? JNI_TRUE : JNI_FALSE;
            break;

         case jatString :
            out[i].l = gEnv->NewStringUTF(args[i].s);
            break;

         default :
            out[i].l = (jobject)args[i].o;
            break;
      }
}

//--------------------------------------------------------------------------------
static jmethodID jniMethod(jobject obj, LPCSTR name, LPCSTR sig)
{
   if (!obj || !jniAttach())
      return NULL;

   jclass    cls = gEnv->GetObjectClass(obj);
   jmethodID m = gEnv->GetMethodID(cls, name, sig);

   gEnv->DeleteLocalRef(cls);
   if (!jniCheck())
      return NULL;
   return m;
}

//--------------------------------------------------------------------------------
LPVOID jniNew(LPCSTR cls, LPCSTR ctorName, LPCSTR sig, const TJArg *args, int n)
{
   jvalue v[jniMaxArgs];

   if (!jniAttach())
      return NULL;

   jclass c = gEnv->FindClass(cls);

   if (!jniCheck() || !c)
      return NULL;

   jmethodID ctor = gEnv->GetMethodID(c, ctorName, sig);

   if (!jniCheck() || !ctor)
      return NULL;
   jniPack(args, n, v);

   jobject o = gEnv->NewObjectA(c, ctor, v);

   return jniCheck() ? o : NULL;
}

//--------------------------------------------------------------------------------
LPVOID jniCallObj(LPVOID obj, LPCSTR name, LPCSTR sig, const TJArg *args, int n)
{
   jvalue    v[jniMaxArgs];
   jmethodID m = jniMethod((jobject)obj, name, sig);

   if (!m)
      return NULL;
   jniPack(args, n, v);

   jobject r = gEnv->CallObjectMethodA((jobject)obj, m, v);

   return jniCheck() ? r : NULL;
}

//--------------------------------------------------------------------------------
int jniCallInt(LPVOID obj, LPCSTR name, LPCSTR sig, const TJArg *args, int n)
{
   jvalue    v[jniMaxArgs];
   jmethodID m = jniMethod((jobject)obj, name, sig);

   if (!m)
      return 0;
   jniPack(args, n, v);

   jint r = gEnv->CallIntMethodA((jobject)obj, m, v);

   return jniCheck() ? (int)r : 0;
}

//--------------------------------------------------------------------------------
QWORD jniCallLong(LPVOID obj, LPCSTR name, LPCSTR sig, const TJArg *args, int n)
{
   jvalue    v[jniMaxArgs];
   jmethodID m = jniMethod((jobject)obj, name, sig);

   if (!m)
      return 0;
   jniPack(args, n, v);

   jlong r = gEnv->CallLongMethodA((jobject)obj, m, v);

   if (!jniCheck())
      return 0;
   return (QWORD)r;
}

//--------------------------------------------------------------------------------
float jniCallFloat(LPVOID obj, LPCSTR name, LPCSTR sig, const TJArg *args, int n)
{
   jvalue    v[jniMaxArgs];
   jmethodID m = jniMethod((jobject)obj, name, sig);

   if (!m)
      return 0;
   jniPack(args, n, v);

   jfloat r = gEnv->CallFloatMethodA((jobject)obj, m, v);

   return jniCheck() ? r : 0;
}

//--------------------------------------------------------------------------------
bool jniCallBool(LPVOID obj, LPCSTR name, LPCSTR sig, const TJArg *args, int n)
{
   jvalue    v[jniMaxArgs];
   jmethodID m = jniMethod((jobject)obj, name, sig);

   if (!m)
      return false;
   jniPack(args, n, v);

   jboolean r = gEnv->CallBooleanMethodA((jobject)obj, m, v);

   return jniCheck() && r == JNI_TRUE;
}

//--------------------------------------------------------------------------------
LONG jniCallFixed(LPVOID obj, LPCSTR name, LPCSTR sig, LONG scale)
{
   jmethodID m = jniMethod((jobject)obj, name, sig);

   if (!m)
      return 0;

   jdouble r = gEnv->CallDoubleMethodA((jobject)obj, m, NULL);

   if (!jniCheck())
      return 0;
   return (LONG)llround(r*(jdouble)scale);
}

//--------------------------------------------------------------------------------
void jniCallVoid(LPVOID obj, LPCSTR name, LPCSTR sig, const TJArg *args, int n)
{
   jvalue    v[jniMaxArgs];
   jmethodID m = jniMethod((jobject)obj, name, sig);

   if (!m)
      return;
   jniPack(args, n, v);
   gEnv->CallVoidMethodA((jobject)obj, m, v);
   jniCheck();
}

//--------------------------------------------------------------------------------
LPVOID jniStaticObj(LPCSTR cls, LPCSTR name, LPCSTR sig, const TJArg *args, int n)
{
   jvalue v[jniMaxArgs];

   if (!jniAttach())
      return NULL;

   jclass c = gEnv->FindClass(cls);

   if (!jniCheck() || !c)
      return NULL;

   jmethodID m = gEnv->GetStaticMethodID(c, name, sig);

   if (!jniCheck() || !m)
      return NULL;
   jniPack(args, n, v);

   jobject r = gEnv->CallStaticObjectMethodA(c, m, v);

   return jniCheck() ? r : NULL;
}

//--------------------------------------------------------------------------------
LPVOID jniStaticField(LPCSTR cls, LPCSTR name, LPCSTR sig)
{
   if (!jniAttach())
      return NULL;

   jclass c = gEnv->FindClass(cls);

   if (!jniCheck() || !c)
      return NULL;

   jfieldID f = gEnv->GetStaticFieldID(c, name, sig);

   if (!jniCheck() || !f)
      return NULL;

   jobject r = gEnv->GetStaticObjectField(c, f);

   return jniCheck() ? r : NULL;
}

//--------------------------------------------------------------------------------
LPVOID jniStringArray(LPCSTR const *strs, int n)
{
   if (!jniAttach() || n <= 0)
      return NULL;

   jstring      first = gEnv->NewStringUTF(strs[0]);
   jclass       cls = gEnv->GetObjectClass(first);
   jobjectArray arr = gEnv->NewObjectArray(n, cls, first);

   for (int i = 1; i < n; i++)
      gEnv->SetObjectArrayElement(arr, i, gEnv->NewStringUTF(strs[i]));
   return jniCheck() ? arr : NULL;
}

//--------------------------------------------------------------------------------
bool jniLockBitmap(LPVOID bitmap, LPBYTE *pixels, int *width, int *height, int *stride)
{
   AndroidBitmapInfo info;

   LPVOID p = NULL;

   if (!bitmap || !jniAttach())
      return false;
   if (AndroidBitmap_getInfo(gEnv, (jobject)bitmap, &info) != ANDROID_BITMAP_RESULT_SUCCESS)
      return false;
   if (AndroidBitmap_lockPixels(gEnv, (jobject)bitmap, &p) != ANDROID_BITMAP_RESULT_SUCCESS)
      return false;
   *pixels = (LPBYTE)p;
   *width = (int)info.width;
   *height = (int)info.height;
   *stride = (int)info.stride;
   return true;
}

//--------------------------------------------------------------------------------
void jniUnlockBitmap(LPVOID bitmap)
{
   if (bitmap && jniAttach())
      AndroidBitmap_unlockPixels(gEnv, (jobject)bitmap);
}

//--------------------------------------------------------------------------------
