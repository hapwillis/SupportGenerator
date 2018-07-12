cmake_minimum_required(VERSION 3.5)
project(SupportTest)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(NANOGUI_BUILD_EXAMPLE OFF CACHE BOOL " " FORCE)
set(NANOGUI_BUILD_PYTHON  OFF CACHE BOOL " " FORCE)
set(NANOGUI_INSTALL       OFF CACHE BOOL " " FORCE)

set(HEADERS headers/Camera.h headers/ConnectionPoints.h headers/DefaultShader.h headers/Graph.h headers/Model.h headers/NavigationMesh.h headers/Octree.h headers/SupportPaths.h)
set(SOURCES src/Camera.cpp src/ConnectionPoints.cpp src/DefaultShader.cpp src/Graph.cpp src/Model.cpp src/NavigationMesh.cpp src/Octree.cpp src/placeSupportPoints.cpp src/SupportPaths.cpp)
set(RESOURCES resources/default.fs resources/default.vs)

source_group("headers" FILES ${HEADERS})
source_group("src" FILES ${SOURCES})
source_group("resources" FILES ${RESOURCES})
source_group("test" FILES )

if ( CMAKE_CXX_COMPILER_ID MATCHES "Clang|AppleClang|GNU" )
    target_compile_options( vector-test PRIVATE -Wall -Wextra -Wunreachable-code -Wpedantic)
endif()
if ( CMAKE_CXX_COMPILER_ID MATCHES "Clang" )
    target_compile_options( vector-test PRIVATE -Wweak-vtables -Wexit-time-destructors -Wglobal-constructors -Wmissing-noreturn )
endif()

include_directories(headers)
include_directories(include)

add_executable(mytest test.cpp ${HEADERS} ${SOURCES} ${RESOURCES})
target_link_libraries(mytest lib/assimp-vc140-mt)
