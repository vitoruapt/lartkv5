cmake_minimum_required(VERSION 2.8.3)
project(inertial_correct_cam)

find_package(visp_ros QUIET)
find_package(Boost REQUIRED COMPONENTS system)

if(NOT visp_ros_FOUND)
    message(WARNING "
                     |===========================================================| 
                     |===========================================================|  
                     |===            MAKEFILE WAS NOT CREATED!                ===|
                     |===               VISP_ROS NOT FOUND                    ===|
                     |===========================================================|   
                     |===========================================================|")
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
        CATKIN_DEPENDS roscpp std_msgs #geometry_msgs image_transport cv_bridge
    )
 
    
    add_definitions(${VISP_DEFINITIONS})
    include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} include)

    
    ## Declare a cpp executable
    add_executable(inertial_correct_cam src/rotateImage.cpp)


    ## Specify libraries to link a library or executable target against
    target_link_libraries(inertial_correct_cam ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})



    message(WARNING "
                 |~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~|
                 |~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~| 
                 |~~~                 INERTIAL_CORRECT_CAM                 ~~~|
                 |~~~           MAKEFILE WAS SUCESSFULLY CREATED!          ~~~|
                 |~~~                    CONGRATULATIONS                   ~~~|
                 |~~~ joao.peixoto@ua.pt                         Março2015 ~~~|
                 |~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~|
                 |~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~|") 
endif()