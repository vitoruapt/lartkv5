cmake_minimum_required(VERSION 2.8.3)
project(tcp_client)

find_package(catkin REQUIRED COMPONENTS
    roscpp
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES tcp_client
 CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(${catkin_INCLUDE_DIRS} include)

## Declare a cpp library
add_library(tcp_client src/class_tcp.cpp)
target_link_libraries(tcp_client ${catkin_LIBRARIES})