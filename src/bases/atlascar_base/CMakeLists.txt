cmake_minimum_required(VERSION 2.8.3)
project(atlascar_base)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    message_generation
    tcp_client
    topic_priority
    human_driver_monitor
    pcl_ros
    interactive_markers
    kfilter
)

find_package(PkgConfig REQUIRED)

pkg_check_modules(GTKMM gtkmm-3.0)


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
  GearboxCommand.msg
  ManagerCommand.msg
  PlcCommand.msg
  ThrottleCommand.msg
  GearboxStatus.msg
  ManagerStatus.msg
  PlcStatus.msg
  ThrottleStatus.msg
)

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
#  LIBRARIES atlascar_base
 CATKIN_DEPENDS roscpp std_msgs message_runtime tcp_client topic_priority human_driver_monitor pcl_ros interactive_markers kfilter
#  DEPENDS
)

###########
## Build ##
###########

include_directories(${catkin_INCLUDE_DIRS} include)

## Declare a cpp executable
add_executable(state_example src/state_example.cpp)
# add_executable(odometry src/odometry.cpp)
add_executable(plc src/plc_node.cpp src/plc.cpp)
add_executable(throttle src/throttle_node.cpp src/throttle.cpp)
add_executable(gearbox src/gearbox_node.cpp src/gearbox.cpp)
add_executable(manager src/manager_node.cpp src/manager.cpp)

add_dependencies(state_example atlascar_base_generate_messages_cpp)
# add_dependencies(odometry atlascar_base_generate_messages_cpp)
add_dependencies(plc atlascar_base_generate_messages_cpp)
add_dependencies(throttle atlascar_base_generate_messages_cpp)
add_dependencies(gearbox atlascar_base_generate_messages_cpp)
add_dependencies(manager atlascar_base_generate_messages_cpp)

target_link_libraries(state_example ${catkin_LIBRARIES})
# target_link_libraries(odometry ${catkin_LIBRARIES})
target_link_libraries(plc ${catkin_LIBRARIES})
target_link_libraries(throttle ${catkin_LIBRARIES})
target_link_libraries(gearbox ${catkin_LIBRARIES})
target_link_libraries(manager ${catkin_LIBRARIES})

if(GTKMM_FOUND)
    include_directories(${GTKMM_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
    
    add_executable(manager_gui src/manager_gui.cpp src/manager_gui_node.cpp)
    add_executable(maneuvers src/maneuvers.cpp  src/maneuvers_node.cpp)
    
    add_dependencies(manager_gui atlascar_base_generate_messages_cpp)
    add_dependencies(maneuvers atlascar_base_generate_messages_cpp)
    
    target_link_libraries(manager_gui ${catkin_LIBRARIES} ${GTKMM_LIBRARIES})    
    target_link_libraries(maneuvers ${catkin_LIBRARIES} ${GTKMM_LIBRARIES})
else()
    message(WARNING "Gtkmm and Sigc++ not installed, manager_gui will not be built!, if the GUI is required, please install the gtkmm lib using: sudo apt-get install libgtkmm-3.0-dev libsigc++-2.0-dev")
endif()

find_package(LibXml2 REQUIRED LibXml2)

include_directories(${LIBXML2_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
add_executable(show_axes src/show_axes.cpp)
add_dependencies(show_axes atlascar_base_generate_messages_cpp)
target_link_libraries(show_axes ${catkin_LIBRARIES} ${LIBXML2_LIBRARIES})
