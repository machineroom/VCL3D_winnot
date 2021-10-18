cmake_minimum_required(VERSION 3.10)

# set the project name
project(LiveScanClient)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/LiveScanClient)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(/usr/local/opt/opencv@2/include)
include_directories(/usr/local/include)


# add the executable
file(GLOB SOURCES "src/LiveScanClient/*.cpp")
# remove windows specific image renderer for now
list(REMOVE_ITEM SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/LiveScanClient/imageRenderer.cpp)
list(REMOVE_ITEM SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/LiveScanClient/kinectCapture.cpp)

list(APPEND SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/socketCS.cpp)

add_executable(client ${SOURCES})
# this code won't build on newer g++ (17)
set_property(TARGET client PROPERTY CXX_STANDARD 11)
target_link_libraries (client opencv2)

