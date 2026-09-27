# LiDAR capture - host build (core + shared + tests). VS18 / MSBuild only; every TU through ppCompile.
cmake_minimum_required(VERSION 3.25)
project(lidar CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/ppGate.cmake)

add_compile_options(/W3 /WX /utf-8 /EHsc)
add_compile_definitions(_CRT_SECURE_NO_WARNINGS)

# shared (replicated from E:/Projetos/Claude/shared)
add_library(lidarShared STATIC
   shared/src/abNew.cpp
   shared/src/abPool.cpp
   shared/src/thread.cpp
   shared/src/fault.cpp
   shared/src/sha256.cpp
   shared/src/json.cpp)
target_include_directories(lidarShared PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/shared/include)
target_compile_definitions(lidarShared PRIVATE SHARED_LIBRARY_BUILD)

# portable capture core
add_library(lidarCore STATIC
   core/src/capGeom.cpp
   core/src/capOrient.cpp
   core/src/capSpin.cpp
   core/src/capBuf.cpp
   core/src/capRecord.cpp
   core/src/capLog.cpp
   core/src/capSession.cpp
   core/src/capJPEG.cpp
   core/src/capFrameMeta.cpp
   core/src/capEXIF.cpp
   core/src/capVanish.cpp
   core/src/capLayout.cpp)
target_include_directories(lidarCore PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/core/include)
target_compile_definitions(lidarCore PRIVATE SHARED_LIBRARY_BUILD)
target_link_libraries(lidarCore PUBLIC lidarShared)

enable_testing()
add_executable(capTest test/capTest.cpp)
target_compile_definitions(capTest PRIVATE SHARED_TEST_BUILD)
target_link_libraries(capTest PRIVATE lidarCore windowscodecs ole32)
add_test(NAME capTest COMMAND capTest WORKING_DIRECTORY ${CMAKE_BINARY_DIR})

# desktop tools (no libDiscipline: tool TUs stay outside the poison build)
add_executable(capInspect tools/capInspect.cpp tools/capMosaic.cpp)
target_link_libraries(capInspect PRIVATE lidarCore windowscodecs ole32)
