cmake_minimum_required(VERSION 2.8.3)
project(polygon_primitives_extraction)

set(COMPILE_BASE false)

if(COMPILE_BASE)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin COMPONENTS
  roscpp
  std_msgs
  pcl_ros
  bo_polygon2d
  mit_darpa_logs_player
  colormap
)

set (CMAKE_CXX_FLAGS "-frounding-math -Wno-deprecated -Wno-deprecated-declarations -O3")


find_package(CGAL QUIET)
find_package(OpenCV REQUIRED)

find_package(PkgConfig REQUIRED)

find_package(wrapper_collada)

pkg_check_modules(COLLADA collada-dom-150)


# find_path(CGAL_INCLUDE_DIRS CGAL/Exact_predicates_inexact_constructions_kernel.h)

# find_library(LIBHD_LIBRARIES NAMES HD libHD libHD.so PATHS /usr/lib /usr/lib64)



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
#  LIBRARIES polygon_primitives_extraction
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
set(compile false)

if(CGAL_FOUND)
else()
    message(WARNING "CGAL not installed, to compile this package please install it using:\n  sudo apt-get install libcgal-dev\nDo this only if you are really sure you need this package, it takes several minutes to compile.\nNot compiling the polygon_primitives_extraction package.")
    set(compile false)
endif()

if(COLLADA_FOUND)
else()
    message(WARNING "COLLADA not installed, to compile this package please install it using:\n  sudo apt-get install collada-dom-dev\nDo this only if you are really sure you need this package, it takes several minutes to compile.\nNot compiling the polygon_primitives_extraction package.")
    set(compile false)
endif()


if(compile)
    set(COLLADA_INCLUDE_DIRS ${COLLADA_INCLUDEDIR} ${COLLADA_INCLUDEDIR}/1.5)

    include_directories(${catkin_INCLUDE_DIRS})
    include_directories(${CGAL_INCLUDE_DIRS})
    include_directories(${OpenCV_INCLUDE_DIRS})
    include_directories(${COLLADA_INCLUDE_DIRS})

    add_library(projection_mesh src/projection_mesh.cpp)
    add_dependencies(projection_mesh polygon_primitive_msg_generate_messages_cpp)
    target_link_libraries(projection_mesh ${catkin_LIBRARIES})

    #target_link_libraries(projection_mesh /usr/local/lib/libCGAL.so)
    #target_link_libraries(projection_mesh /usr/local/lib/libCGAL_Core.so)
    #target_link_libraries(polygon_primitive /usr/lib/libmpfr.so)
    #target_link_libraries(polygon_primitive /usr/lib/libgmp.so)

    add_library(constrained_delaunay_triangulation src/constrained_delaunay_triangulation.cpp src/constrained_delaunay_triangulation_utils.cpp) 
    add_dependencies(constrained_delaunay_triangulation polygon_primitive_msg_generate_messages_cpp)
    target_link_libraries(constrained_delaunay_triangulation ${catkin_LIBRARIES} ${CGAL_LIBRARIES})


    add_library(pinhole_projection src/class_pinhole_projection.cpp) 
    add_library(distortion_correction src/distortion_correction.cpp) 
    add_library(textured_triangle src/textured_triangle.cpp)
    
    add_dependencies(pinhole_projection polygon_primitive_msg_generate_messages_cpp)
    add_dependencies(distortion_correction polygon_primitive_msg_generate_messages_cpp)
    add_dependencies(textured_triangle polygon_primitive_msg_generate_messages_cpp)

    target_link_libraries(pinhole_projection ${catkin_LIBRARIES})
    target_link_libraries(distortion_correction ${catkin_LIBRARIES})
    target_link_libraries(textured_triangle ${catkin_LIBRARIES})

    add_library(camera_projection src/camera_projection.cpp src/camera_parameters.cpp) 
    target_link_libraries(camera_projection projection_mesh constrained_delaunay_triangulation pinhole_projection distortion_correction textured_triangle)
    target_link_libraries(camera_projection ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

    # # Must install CGAL, libmpfr* and libgmp*
    add_library(polygon_primitive src/polygon_primitive_auxiliary.cpp src/polygon_primitive_planefitting.cpp src/polygon_primitive_operations.cpp src/polygon_primitive_communication.cpp src/polygon_primitive_offset.cpp src/polygon_primitive_2dhulls.cpp src/polygon_primitive.cpp)
    add_dependencies(polygon_primitive polygon_primitive_msg_generate_messages_cpp)
    target_link_libraries(polygon_primitive ${catkin_LIBRARIES})
    target_link_libraries(polygon_primitive ${CGAL_LIBRARIES})

    # target_link_libraries(polygon_primitive /usr/lib/x86_64-linux-gnu/libmpfr.so.4)
    # target_link_libraries(polygon_primitive /usr/lib/x86_64-linux-gnu/libgmp.so.10)

    add_library(polygon_primitive_with_texture src/polygon_primitive_with_texture.cpp src/polygon_primitive_with_texture_rviz.cpp)
    add_dependencies(polygon_primitive_with_texture polygon_primitive_msg_generate_messages_cpp)
    target_link_libraries(polygon_primitive_with_texture polygon_primitive)
    target_link_libraries(polygon_primitive_with_texture ${catkin_LIBRARIES})
    target_link_libraries(polygon_primitive_with_texture ${CGAL_LIBRARIES})

    # 
    # ##BINARIES
    add_executable(extract_polygon_primitives src/extract_polygon_primitives.cpp src/extract_polygon_primitives_rvizmarkers.cpp src/extract_polygon_primitives_handlers.cpp src/extract_polygon_primitives_auxiliary.cpp)
    add_dependencies(extract_polygon_primitives polygon_primitive_msg_generate_messages_cpp)
    target_link_libraries(extract_polygon_primitives polygon_primitive)
    target_link_libraries(extract_polygon_primitives ${catkin_LIBRARIES})
    target_link_libraries(extract_polygon_primitives ${COLLADA_LIBRARIES})
    # 
    # #rosbuild_add_executable(texturize_polygon_primitives src/pre_compile.cpp src/texturize_polygon_primitives.cpp ) 
    add_executable(texturize_polygon_primitives src/texturize_polygon_primitives.cpp ) 
    add_dependencies(texturize_polygon_primitives polygon_primitive_msg_generate_messages_cpp)
    target_link_libraries(texturize_polygon_primitives polygon_primitive_with_texture)  
    target_link_libraries(texturize_polygon_primitives bo_polygon2d)  
    target_link_libraries(texturize_polygon_primitives colormap)
    target_link_libraries(texturize_polygon_primitives distortion_correction)
    target_link_libraries(texturize_polygon_primitives camera_projection)
    target_link_libraries(texturize_polygon_primitives ${catkin_LIBRARIES})  

else()
    message(WARNING "Not compiling the polygon_primitives_extraction package.")
endif()

else()
    message(WARNING "Not compiling the polygon_primitives_extraction package. Base error!!")
endif()

## Declare a cpp library
# add_library(polygon_primitives_extraction
#   src/${PROJECT_NAME}/polygon_primitives_extraction.cpp
# )

## Declare a cpp executable
# add_executable(polygon_primitives_extraction_node src/polygon_primitives_extraction_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(polygon_primitives_extraction_node polygon_primitives_extraction_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(polygon_primitives_extraction_node
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
# install(TARGETS polygon_primitives_extraction polygon_primitives_extraction_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_polygon_primitives_extraction.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
