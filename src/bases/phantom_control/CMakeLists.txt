cmake_minimum_required(VERSION 2.8.3)
project(phantom_control)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
#   phua_haptic
)

find_path(LIBHD_INCLUDE_DIRS HD/hd.h)
find_path(LIBHL_INCLUDE_DIRS HL/hl.h)
find_path(LIBHDU_INCLUDE_DIRS HDU/hdu.h)
find_path(LIBHLU_INCLUDE_DIRS HLU/hlu.h)

find_library(LIBHD_LIBRARIES NAMES HD libHD libHD.so PATHS /usr/lib /usr/lib64)
find_library(LIBHL_LIBRARIES NAMES HL libHL libHL.so PATHS /usr/lib /usr/lib64)
find_library(LIBHDU_LIBRARIES NAMES HDU libHDU libHDU.so PATHS /usr/lib /usr/lib64)
find_library(LIBHLU_LIBRARIES NAMES HLU libHLU libHLU.so PATHS /usr/lib /usr/lib64)
find_library(LIBPHANTOM_LIBRARIES NAMES PHANToMIO libPHANToMIO libPHANToMIO.so PATHS /usr/lib /usr/lib64)

set(COMPILE true)

if(${LIBHD_LIBRARIES} STREQUAL LIBHD_LIBRARIES-NOTFOUND)
    message(WARNING "OpenHaptics libraries are not installed! please install both OpenHaptics-AE 3.0 and PHANTOM Device Drivers located in the source folder of phua_haptic package. Not compiling this module.")
    set(COMPILE false)
endif()

if(${LIBHL_LIBRARIES} STREQUAL LIBHD_LIBRARIES-NOTFOUND)
    message(WARNING "OpenHaptics libraries are not installed! please install both OpenHaptics-AE 3.0 and PHANTOM Device Drivers located in the source folder of phua_haptic package. Not compiling this module.")
    set(COMPILE false)
endif()

if(${LIBHDU_LIBRARIES} STREQUAL LIBHD_LIBRARIES-NOTFOUND)
    message(WARNING "OpenHaptics libraries are not installed! please install both OpenHaptics-AE 3.0 and PHANTOM Device Drivers located in the source folder of phua_haptic package. Not compiling this module.")
    set(COMPILE false)
endif()

if(${LIBHLU_LIBRARIES} STREQUAL LIBHD_LIBRARIES-NOTFOUND)
    message(WARNING "OpenHaptics libraries are not installed! please install both OpenHaptics-AE 3.0 and PHANTOM Device Drivers located in the source folder of phua_haptic package. Not compiling this module.")
    set(COMPILE false)
endif()

if(${LIBPHANTOM_LIBRARIES} STREQUAL LIBHD_LIBRARIES-NOTFOUND)
    message(WARNING "OpenHaptics libraries are not installed! please install both OpenHaptics-AE 3.0 and PHANTOM Device Drivers located in the source folder of phua_haptic package. Not compiling this module.")
    set(COMPILE false)
endif()
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
add_message_files(
  FILES
  State.msg
#   Message2.msg
)

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
generate_messages(
  DEPENDENCIES
  std_msgs
)

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
 INCLUDE_DIRS include
#  LIBRARIES phantom_control
 CATKIN_DEPENDS roscpp std_msgs message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

if(COMPILE)
    
    include_directories(${catkin_INCLUDE_DIRS} include)
    include_directories(${LIBHD_INCLUDE_DIRS})
    include_directories(${LIBHL_INCLUDE_DIRS})
    include_directories(${LIBHDU_INCLUDE_DIRS})
    include_directories(${LIBHLU_INCLUDE_DIRS})

    add_executable(phantom_control src/phantom_control.cpp src/conio.cpp )

    add_dependencies(phantom_control phantom_control_generate_messages_cpp)
    # target_link_libraries(phantom_control HD HDU)
    target_link_libraries(phantom_control ${catkin_LIBRARIES})
    target_link_libraries(phantom_control ${LIBHD_LIBRARIES})
    target_link_libraries(phantom_control ${LIBHL_LIBRARIES})
    target_link_libraries(phantom_control ${LIBHDU_LIBRARIES})
    target_link_libraries(phantom_control ${LIBHLU_LIBRARIES})
    target_link_libraries(phantom_control ${LIBPHANTOM_LIBRARIES})
    target_link_libraries(phantom_control ${catkin_LIBRARIES})
    target_link_libraries(phantom_control ${LIBRAW1394_LIBRARIES})
endif()

## Declare a cpp library
# add_library(phantom_control
#   src/${PROJECT_NAME}/phantom_control.cpp
# )

## Declare a cpp executable
# add_executable(phantom_control_node src/phantom_control_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(phantom_control_node phantom_control_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(phantom_control_node
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
# install(TARGETS phantom_control phantom_control_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_phantom_control.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
