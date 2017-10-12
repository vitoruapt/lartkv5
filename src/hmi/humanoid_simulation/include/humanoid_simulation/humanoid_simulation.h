#ifndef HUMANOID_SIMULATION_H
#define HUMANOID_SIMULATION_H

#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <cassert>
#include <cmath>
#include <vector>
#include <algorithm> 
#include <fstream>
#include <time.h>
#include <std_msgs/String.h>

#include <HD/hd.h>
#include <HDU/hduVector.h>
#include <HDU/hduMatrix.h>
#include <HDU/hduError.h>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PolygonStamped.h>

#include <vrep_common/VrepInfo.h>
#include <vrep_common/ObjectGroupData.h>
#include <vrep_common/JointSetStateData.h>
#include <vrep_common/ForceSensorData.h>
#include <vrep_common/simRosGetObjectGroupData.h>
#include <vrep_common/ImuData.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include "../v_repConst.h"
#include "../conio.h"

typedef struct
{
    HDdouble device_position[3];
    HDdouble joint_angle[3];
    HDdouble gimbal_angle[3];
    HDboolean button_state[2];
    
} device_state;

#endif