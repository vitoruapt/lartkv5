cmake_minimum_required(VERSION 2.8.3)
project(arrow_detection)

find_package(visp_ros QUIET)
find_package(Boost REQUIRED COMPONENTS system)

if(NOT visp_ros_FOUND)
    message(WARNING "visp_ros package not found!\nPackage arrow_detection will not compile")
else()
   

    find_package(catkin REQUIRED COMPONENTS
    message_generation
    roscpp
    std_msgs
    geometry_msgs
    cv_bridge
    image_transport
    visp_ros
    tcp_client
    )

    find_package(OpenCV REQUIRED)
    
    catkin_package(
        INCLUDE_DIRS include
        #  LIBRARIES find_arrow_v11
        CATKIN_DEPENDS roscpp std_msgs geometry_msgs
        #  DEPENDS system_lib
    )
    
    add_definitions(${VISP_DEFINITIONS})

    include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} include)

    ## Declare a cpp executable
    #add_executable(arrow_detection src/findArrow.cpp src/rotateImage.cpp src/test1.cpp src/arrow_detection.cpp)
    add_executable(arrow_detection src/findArrow.cpp src/rotateImage.cpp src/arrow_detection.cpp)
    add_executable(fanuc src/fanuc.cpp)
    
    add_executable(track_arrow src/track_arrow.cpp)
    add_executable(keyboard src/keyboard.cpp)

    ## Add cmake target dependencies of the executable/library
    ## as an example, message headers may need to be generated before nodes
    # add_dependencies(arrow_detection_node arrow_detection_generate_messages_cpp)

    ## Specify libraries to link a library or executable target against
    target_link_libraries(track_arrow ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
    target_link_libraries(arrow_detection ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
    target_link_libraries(fanuc ${catkin_LIBRARIES} ${Boost_LIBRARIES})
    target_link_libraries(keyboard ${catkin_LIBRARIES} ${Boost_LIBRARIES})

endif()
