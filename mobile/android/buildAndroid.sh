#!/bin/bash
# buildAndroid.sh - build, package, sign and (optionally) install the LiDAR capture APK.
#
#   bash mobile/android/buildAndroid.sh [--install]
#
# Every translation unit compiles THROUGH ppCompile (launcher mode over the NDK clang) - regraOuro 3.
# Only the link and packaging steps call the toolchain directly. Outputs live in build/android only.
# Toolchains come from E:/Projetos/Claude/third-party (read/execute only).
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
TP="/e/Projetos/Claude/third-party"
PPCOMPILE="/e/Projetos/Claude/shared/tools/ppCheck/build/Release/ppCompile.exe"
NDK="$TP/android-ndk-r27c"
LLVM="$NDK/toolchains/llvm/prebuilt/windows-x86_64/bin"
CLANGXX="$LLVM/clang++.exe"
CLANG="$LLVM/clang.exe"
SDK="$TP/androidSdk"
BT="$SDK/build-tools/34.0.0"
PLATFORM="$SDK/platforms/android-34/android.jar"
JDK="$TP/jdk17/jdk-17.0.19+10"
API=26
TARGET="aarch64-linux-android$API"
OUT="$ROOT/build/android"
OBJ="$OUT/obj"
KEYSTORE="$ROOT/keys/lidarDebug.jks"
GLUE="$NDK/sources/android/native_app_glue"

mkdir -p "$OBJ" "$OUT/apk/lib/arm64-v8a" "$ROOT/keys"

INC=(-I"$ROOT/shared/include" -I"$ROOT/core/include" -I"$ROOT/mobile/include" -I"$ROOT/mobile/android" -I"$GLUE")
CXXFLAGS=(--target=$TARGET -std=c++20 -O2 -fPIC -fvisibility=hidden -DSHARED_LIBRARY_BUILD -Wall)

SRCS=(
   shared/src/abNew.cpp shared/src/abPool.cpp shared/src/thread.cpp shared/src/fault.cpp shared/src/sha256.cpp
   core/src/capGeom.cpp core/src/capOrient.cpp core/src/capSpin.cpp core/src/capBuf.cpp core/src/capRecord.cpp
   core/src/capLog.cpp core/src/capSession.cpp core/src/capJPEG.cpp core/src/capFrameMeta.cpp core/src/capEXIF.cpp
   core/src/capVanish.cpp core/src/capLayout.cpp core/src/capBlur.cpp core/src/capDoor.cpp
   mobile/app/capCanvas.cpp mobile/app/capApp.cpp
   mobile/android/capJNI.cpp mobile/android/TAndroid.cpp mobile/android/androidMain.cpp
)

OBJS=()
for s in "${SRCS[@]}"; do
   o="$OBJ/$(basename "${s%.cpp}").o"
   echo "cc  $s"
   "$PPCOMPILE" "$CLANGXX" "${CXXFLAGS[@]}" "${INC[@]}" -c "$ROOT/$s" -o "$o"
   OBJS+=("$o")
done

echo "cc  native_app_glue (vendored)"
"$PPCOMPILE" "$CLANG" --target=$TARGET -O2 -fPIC -c "$GLUE/android_native_app_glue.c" -o "$OBJ/glue.o"
OBJS+=("$OBJ/glue.o")

echo "ld  liblidar.so"
"$CLANGXX" --target=$TARGET -shared -o "$OUT/apk/lib/arm64-v8a/liblidar.so" "${OBJS[@]}" \
   -static-libstdc++ -landroid -llog -lcamera2ndk -lmediandk -ljnigraphics -lm \
   -u ANativeActivity_onCreate -Wl,--defsym=android_main=androidMain -Wl,--no-undefined -Wl,-z,max-page-size=16384

echo "pkg lidar.apk"
rm -f "$OUT/unsigned.apk" "$OUT/aligned.apk" "$OUT/lidar.apk"
"$BT/aapt2.exe" link -o "$OUT/unsigned.apk" --manifest "$ROOT/mobile/android/AndroidManifest.xml" -I "$PLATFORM"
(cd "$OUT/apk" && "$BT/aapt.exe" add "$OUT/unsigned.apk" lib/arm64-v8a/liblidar.so > /dev/null)
"$BT/zipalign.exe" -p -f 4 "$OUT/unsigned.apk" "$OUT/aligned.apk"

if [ ! -f "$KEYSTORE" ]; then
   "$JDK/bin/keytool.exe" -genkeypair -keystore "$KEYSTORE" -alias lidar -keyalg RSA -keysize 2048 \
      -validity 10000 -storepass lidardebug -keypass lidardebug -dname "CN=LiDAR Debug, O=Sorena" > /dev/null
fi
JAVA_HOME="$(cygpath -w "$JDK")" "$BT/apksigner.bat" sign --ks "$(cygpath -w "$KEYSTORE")" --ks-pass pass:lidardebug \
   --out "$(cygpath -w "$OUT/lidar.apk")" "$(cygpath -w "$OUT/aligned.apk")"
echo "ok  $OUT/lidar.apk"

if [ "${1:-}" = "--install" ]; then
   "$SDK/platform-tools/adb.exe" install -r "$(cygpath -w "$OUT/lidar.apk")"
fi
