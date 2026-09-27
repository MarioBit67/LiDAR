# ppGate.cmake - route EVERY MSBuild CL compile through the ORIGINAL ppCompile (regraOuro 3).
#
# Replicated from E:/Projetos/Claude/shared/cmake/ppGate.cmake, pointing at the original tool by absolute
# path. On the VS18/MSBuild generator CMAKE_*_COMPILER_LAUNCHER is a no-op, so the gate is wired through
# MSBuild's <CLToolExe>/<CLToolPath>: the CL task invokes ppCompile.exe flags-first (STANDIN mode), which
# gates each source TU with ppCheck and then hands off to the real cl. A non-green TU fails the build.
# There is no fallback: without ppCompile the configure stops.

set(LIDAR_PPCOMPILE_DIR "E:/Projetos/Claude/shared/tools/ppCheck/build/Release")

if(NOT MSVC)
   message(FATAL_ERROR "ppGate: the host build is MSVC/MSBuild only")
endif()

if(NOT EXISTS "${LIDAR_PPCOMPILE_DIR}/ppCompile.exe")
   message(FATAL_ERROR "ppGate: ppCompile.exe not found in ${LIDAR_PPCOMPILE_DIR}")
endif()

file(TO_NATIVE_PATH "${LIDAR_PPCOMPILE_DIR}" _ppcDir)
set(CMAKE_VS_GLOBALS
    "CLToolExe=ppCompile.exe"
    "CLToolPath=${_ppcDir}")
message(STATUS "ppGate: MSBuild CL routed through ppCompile.exe (${_ppcDir})")
unset(_ppcDir)
