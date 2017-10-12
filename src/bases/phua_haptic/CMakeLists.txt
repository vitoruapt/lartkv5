cmake_minimum_required(VERSION 2.8.3)
project(phua_haptic)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  hitec5980sg
  roslib
  tf
)

set_property(GLOBAL PROPERTY FIND_LIBRARY_USE_LIB64_PATHS ON) 

find_package(PkgConfig)

find_package(GTK2 2.6 REQUIRED gtk)
find_package(Eigen3 REQUIRED)

pkg_check_modules(RAW1394 libraw1394)

find_library(ARMA_LIBRARY NAMES armadillo )

find_path(LIBHD_INCLUDE_DIRS HD/hd.h)
find_path(LIBHL_INCLUDE_DIRS HL/hl.h)
find_path(LIBHDU_INCLUDE_DIRS HDU/hdu.h)
find_path(LIBHLU_INCLUDE_DIRS HLU/hlu.h)

find_library(LIBHD_LIBRARIES NAMES HD libHD libHD.so PATHS /usr/lib /usr/lib64)
find_library(LIBHL_LIBRARIES NAMES HL libHL libHL.so PATHS /usr/lib /usr/lib64)
find_library(LIBHDU_LIBRARIES NAMES HDU libHDU libHDU.so PATHS /usr/lib /usr/lib64)
find_library(LIBHLU_LIBRARIES NAMES HLU libHLU libHLU.so PATHS /usr/lib /usr/lib64)
find_library(LIBPHANTOM_LIBRARIES NAMES PHANToMIO libPHANToMIO libPHANToMIO.so PATHS /usr/lib /usr/lib64)

# set(LIBRAW1394_LIBRARIES ${PROJECT_SOURCE_DIR}/lib/libraw1394.so.8)

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
    message(WARNING "OpenHaptics libraries are not installed! please install both OpenHaptics-AE 3.0 and PHANTOM Device Drivers located in the source folder of this package. Not compiling this module.")
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
 INCLUDE_DIRS include
#  LIBRARIES ${PROJECT_SOURCE_DIR}/lib/libraw1394.so.8
 CATKIN_DEPENDS roscpp std_msgs hitec5980sg roslib tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)

if(COMPILE)
    include_directories(${catkin_INCLUDE_DIRS} include)
    include_directories(${EIGEN3_INCLUDE_DIR})
    include_directories(${GTK2_INCLUDE_DIRS})
    include_directories(${LIBHD_INCLUDE_DIRS})
    include_directories(${LIBHL_INCLUDE_DIRS})
    include_directories(${LIBHDU_INCLUDE_DIRS})
    include_directories(${LIBHLU_INCLUDE_DIRS})
    include_directories(${RAW1394_INCLUDE_DIRS})
    
    add_executable(phua_haptic_interface src/main.cpp src/gtk_interface.cpp src/humanoid_functions.cpp src/miscellaneous.cpp src/conio.cpp src/humanoid_control_functions.cpp src/gtk_aux.cpp src/hd_hl_apis_callbacks.cpp src/haptic_rendering_funx.cpp src/hd_hl_apis_aux.cpp)

    target_link_libraries(phua_haptic_interface ${GTK2_LIBRARIES} ${ARMA_LIBRARY})
    target_link_libraries(phua_haptic_interface ${LIBHD_LIBRARIES})
    target_link_libraries(phua_haptic_interface ${LIBHL_LIBRARIES})
    target_link_libraries(phua_haptic_interface ${LIBHDU_LIBRARIES})
    target_link_libraries(phua_haptic_interface ${LIBHLU_LIBRARIES})
    target_link_libraries(phua_haptic_interface ${LIBPHANTOM_LIBRARIES})
    target_link_libraries(phua_haptic_interface ${RAW1394_LIBRARIES})
    target_link_libraries(phua_haptic_interface ${catkin_LIBRARIES})
    target_link_libraries(phua_haptic_interface ${LIBRAW1394_LIBRARIES})
    
    
endif()

## Declare a cpp library
# add_library(phua_haptic
#   src/${PROJECT_NAME}/phua_haptic.cpp
# )

## Declare a cpp executable
# add_executable(phua_haptic_node src/phua_haptic_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(phua_haptic_node phua_haptic_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(phua_haptic_node
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
# install(TARGETS phua_haptic phua_haptic_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_phua_haptic.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
