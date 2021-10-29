
cmake_minimum_required(VERSION 3.10)
find_package(PkgConfig)

# set the project name
project(LiveScanClient)

pkg_check_modules(OPENCV REQUIRED opencv4)
pkg_check_modules(ZSTD REQUIRED libzstd)
pkg_check_modules(FREENECT2 REQUIRED freenect2)
pkg_check_modules(GLFW3 REQUIRED glfw3)
pkg_check_modules(GFLAGS REQUIRED gflags)
set(OpenGL_GL_PREFERENCE "GLVND")
find_package(OpenGL REQUIRED)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/LiveScanClient)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${OPENCV_INCLUDE_DIRS})
include_directories(${ZSTD_INCLUDE_DIRS})
include_directories(${NANOFLANN_INCLUDE_DIRS})
include_directories(${FREENECT2_INCLUDE_DIRS})
include_directories(${GLFW3_INCLUDE_DIRS})

link_directories(${OPENCV_LIBRARY_DIRS})

# add the executable
file(GLOB SOURCES "src/LiveScanClient/*.cpp")
# remove windows specific image renderer for now
list(REMOVE_ITEM SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/LiveScanClient/imageRenderer.cpp)
list(REMOVE_ITEM SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/LiveScanClient/kinectCapture.cpp)

list(APPEND SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/socketCS.cpp)

LIST(APPEND RESOURCES
  src/LiveScanClient/shader/debug.fs
  src/LiveScanClient/shader/default.vs
  src/LiveScanClient/shader/filter1.fs
  src/LiveScanClient/shader/filter2.fs
  src/LiveScanClient/shader/stage1.fs
  src/LiveScanClient/shader/stage2.fs
)

add_executable(client ${SOURCES})

# this code won't build on newer g++ (17)
set_property(TARGET client PROPERTY CXX_STANDARD 11)
target_link_libraries (client ${OPENCV_LINK_LIBRARIES}
						      ${ZSTD_LINK_LIBRARIES} 
                              ${FREENECT2_LINK_LIBRARIES}
                              ${GLFW3_LINK_LIBRARIES}
                              OpenGL::GL
                              Threads::Threads
                              ${GFLAGS_LINK_LIBRARIES}
                      )

