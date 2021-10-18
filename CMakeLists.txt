#MAC export PKG_CONFIG_PATH="/usr/local/opt/opencv@2/lib/pkgconfig:/usr/local/lib/pkgconfig"

cmake_minimum_required(VERSION 3.10)
find_package(PkgConfig)

# set the project name
project(LiveScanClient)

pkg_check_modules(OPENCV REQUIRED opencv4)
pkg_check_modules(ZSTD REQUIRED libzstd)
#pkg_check_modules(NANOFLANN REQUIRED nanoflann)
pkg_check_modules(FREENECT REQUIRED libfreenect)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/LiveScanClient)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${OPENCV_INCLUDE_DIRS})
include_directories(${ZSTD_INCLUDE_DIRS})
include_directories(${NANOFLANN_INCLUDE_DIRS})
include_directories(${FREENECT_INCLUDE_DIRS})

link_directories(${OPENCV_LIBRARY_DIRS})

# add the executable
file(GLOB SOURCES "src/LiveScanClient/*.cpp")
# remove windows specific image renderer for now
list(REMOVE_ITEM SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/LiveScanClient/imageRenderer.cpp)
list(REMOVE_ITEM SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/LiveScanClient/kinectCapture.cpp)

list(APPEND SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/socketCS.cpp)

add_executable(client ${SOURCES})

# this code won't build on newer g++ (17)
set_property(TARGET client PROPERTY CXX_STANDARD 11)
target_link_libraries (client ${OPENCV_LINK_LIBRARIES} ${ZSTD_LINK_LIBRARIES} ${FREENECT_LINK_LIBRARIES} Threads::Threads)

