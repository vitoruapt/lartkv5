cmake_minimum_required(VERSION 2.8.3)
project(lidar_egomotion)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  pcl_ros
  atlascar_base
  kfilter
  pcl_conversions
  laser_geometry
)

find_package(PkgConfig REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

pkg_check_modules(PLPLOT plplotd)
pkg_check_modules(CSM_LIB csm)

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
#  LIBRARIES lidar_egomotion
 CATKIN_DEPENDS roscpp std_msgs pcl_ros atlascar_base kfilter pcl_conversions laser_geometry
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)


if(PLPLOT_FOUND AND CSM_LIB_FOUND)
    include_directories(${PLPLOT_INCLUDE_DIRS})
    include_directories(${CSM_LIB_INCLUDE_DIRS})
    include_directories(${catkin_INCLUDE_DIRS})
    include_directories(${EIGEN3_INCLUDE_DIR})
    include_directories(${OpenCV_INCLUDE_DIRS})    

    set(PSM_ALGORITHM psm)
    set(MBICP_ALGORITHM mbicp)

    add_library(${PSM_ALGORITHM} src/psm/polar_match.cpp)
    add_library(${MBICP_ALGORITHM} src/mbicp/calcul.c src/mbicp/MbICP.c src/mbicp/mbicp_aux.c src/mbicp/percolate.c src/mbicp/sp_matrix.c)

    target_link_libraries(${PSM_ALGORITHM} ${catkin_LIBRARIES})
    target_link_libraries(${MBICP_ALGORITHM} ${catkin_LIBRARIES})
    
    add_executable(lidar_egomotion src/lidar_egomotion.cpp src/motion_model.cpp src/plplot_graph.cpp src/types_implementation.cpp)
    target_link_libraries(lidar_egomotion ${PLPLOT_LIBRARIES} ${CSM_LIB_LIBRARIES} ${PSM_ALGORITHM} ${MBICP_ALGORITHM} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
else()
    message(WARNING "\nTo correctly compile this module, plplotd library and CSM are required.\nWithout these libraries nothing will be compiled.\nPlplot may be installed using sudo apt-get install libplplot-dev \nThe C(anonical) Scan Matcher (CSM) may be found in https://github.com/AndreaCensi/csm \nCSM must be installed system wide and be accessible via pkg-config. The file csm-master.zip located in the source folder of this package, it contains the needed source code for the CSM library, compile and install this code please! (using cmake)\nInstructions to compile:\nunzip the file csm-master.zip\ncd csm-master\ncmake .\nmake\nsudo make install\nThe CSM package will need the gsl library: sudo apt-get install libgsl0-dev to install.\n\n")
endif()


## Declare a cpp library
# add_library(lidar_egomotion
#   src/${PROJECT_NAME}/lidar_egomotion.cpp
# )

## Declare a cpp executable
# add_executable(lidar_egomotion_node src/lidar_egomotion_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(lidar_egomotion_node lidar_egomotion_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(lidar_egomotion_node
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
# install(TARGETS lidar_egomotion lidar_egomotion_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_lidar_egomotion.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
