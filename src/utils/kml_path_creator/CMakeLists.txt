cmake_minimum_required(VERSION 2.8.3)
project(kml_path_creator)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

find_path(LIBKML_INCLUDE_DIR kml/dom.h)

find_library(LIBKMLDOM_LIBRARY NAMES kmldom)
find_library(LIBKMLBASE_LIBRARY NAMES kmlbase)
find_library(LIBKMLCONVENIENCE_LIBRARY NAMES kmlconvenience)
find_library(LIBKMLENGINE_LIBRARY NAMES kmlengine)

set(DO_KML true)

if(${LIBKML_INCLUDE_DIR} STREQUAL LIBKML_INCLUDE_DIR-NOTFOUND)
    message(WARNING "Libkml is not installed please do: sudo apt-get install libkml-dev\nkml/dom.h not found")
    set(DO_KML false)
endif()

if(${LIBKMLDOM_LIBRARY} STREQUAL LIBKMLDOM_LIBRARY-NOTFOUND)
    message(WARNING "Libkml is not installed please do: sudo apt-get install libkml-dev\nlibkmldom not found")
    set(DO_KML false)
endif()

if(${LIBKMLBASE_LIBRARY} STREQUAL LIBKMLBASE_LIBRARY-NOTFOUND)
    message(WARNING "Libkml is not installed please do: sudo apt-get install libkml-dev\nlibkmlbase not found")
    set(DO_KML false)
endif()

if(${LIBKMLCONVENIENCE_LIBRARY} STREQUAL LIBKMLCONVENIENCE_LIBRARY-NOTFOUND)
    message(WARNING "Libkml is not installed please do: sudo apt-get install libkml-dev\nlibkmlconvenience not found")
    set(DO_KML false)
endif()

if(${LIBKMLENGINE_LIBRARY} STREQUAL LIBKMLENGINE_LIBRARY-NOTFOUND)
    message(WARNING "Libkml is not installed please do: sudo apt-get install libkml-dev\nlibkmlengine not found")
    set(DO_KML false)
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
#  INCLUDE_DIRS include
#  LIBRARIES kmlPathCreator
 CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)

if(DO_KML)
    include_directories(${LIBKML_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

    add_executable(kml_path_creator src/kml_path_creator.cpp)

    target_link_libraries(kml_path_creator ${LIBKMLDOM_LIBRARY} ${LIBKMLBASE_LIBRARY} ${LIBKMLCONVENIENCE_LIBRARY} ${LIBKMLCONVENIENCE_LIBRARY} ${LIBKMLENGINE_LIBRARY} ${catkin_LIBRARIES})
else()
    message(WARNING "This package will not compile")
endif()


## Declare a cpp library
# add_library(kmlPathCreator
#   src/${PROJECT_NAME}/kmlPathCreator.cpp
# )

## Declare a cpp executable
# add_executable(kmlPathCreator_node src/kmlPathCreator_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(kmlPathCreator_node kmlPathCreator_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(kmlPathCreator_node
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
# install(TARGETS kmlPathCreator kmlPathCreator_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_kmlPathCreator.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
