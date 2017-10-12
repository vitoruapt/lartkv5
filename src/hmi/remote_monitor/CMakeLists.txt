cmake_minimum_required(VERSION 2.8.3)
project(remote_monitor)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(BUG_WITH_DATABASE_INTERFACE_SOLVED false)

if(BUS_WITH_DATABASE_INTERFACE_SOLVED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  database_interface
  atlascar_base
)

#check for libpq's configure tool
find_program(PG_CONFIG NAMES pg_config DOC "libpq config tool")
if (NOT PG_CONFIG)
  message(FATAL_ERROR "Couldn't find pg_config. Is libpq installed?")
endif(NOT PG_CONFIG)

find_package(PkgConfig)
pkg_check_modules(SQLITE3 REQUIRED sqlite3)
include_directories(${SQLITE3_INCLUDE_DIRS})
link_directories(${SQLITE3_LIBRARY_DIRS})

#get the libpq include and lib directories
execute_process( 
  COMMAND pg_config --includedir
  ERROR_VARIABLE IGNORE_VAR 
  OUTPUT_VARIABLE PQ_INCLUDE_DIR 
  OUTPUT_STRIP_TRAILING_WHITESPACE)
execute_process( 
  COMMAND pg_config --libdir
  ERROR_VARIABLE IGNORE_VAR 
  OUTPUT_VARIABLE PQ_LIB_DIR 
  OUTPUT_STRIP_TRAILING_WHITESPACE)

include(CheckIncludeFiles)
set(CMAKE_REQUIRED_INCLUDES ${PQ_INCLUDE_DIR})
check_include_files(libpq-fe.h HAVE_LIBPQ)
if (NOT HAVE_LIBPQ)
  message(FATAL_ERROR "Error: PostgreSQL implementation cannot find libpq-fe.h")
endif(NOT HAVE_LIBPQ)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES remote_monitor
 CATKIN_DEPENDS roscpp std_msgs database_interface atlascar_base
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)

include_directories(${catkin_INCLUDE_DIRS})

link_directories(${PQ_LIB_DIR})
include_directories(${PQ_INCLUDE_DIR})

add_executable(remote_monitor src/remote_monitor.cpp)
target_link_libraries(remote_monitor pq ${catkin_LIBRARIES})
else()
message(WARNING "THis package will not compile due to cascating failure of database_interface")
endif()
## Declare a cpp library
# add_library(remote_monitor
#   src/${PROJECT_NAME}/remote_monitor.cpp
# )

## Declare a cpp executable
# add_executable(remote_monitor_node src/remote_monitor_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(remote_monitor_node remote_monitor_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(remote_monitor_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS remote_monitor remote_monitor_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_remote_monitor.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
