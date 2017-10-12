/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, ISR University of Coimbra.
*  Copyright (c) 2011-2012, INRIA, CNRS, all rights reserved
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the ISR University of Coimbra nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Gonçalo Cabrita on 08/11/2010
*
* Author: Nicolas Vignard on 2 MAY 2012
* Notes: Add the ability to control the Mti-G
*********************************************************************/
#include "MTi/MTi.h"
#include <ros/ros.h>


/**
 * @brief
 *
 * @param argc
 * @param argv
 * @return int
 */
int main(int argc, char** argv)
{
    ros::init(argc, argv, "mti_node");    
    ros::NodeHandle pn("~");

    // Params
    std::string portname;
    int baudrate;
    std::string frame_id;
    pn.param<std::string>("port", portname, "/dev/ttyUSB0");
    pn.param("baudrate", baudrate, 115200);
    pn.param<std::string>("frame_id", frame_id, Xsens::IMU_FRAME_ID);

    Xsens::MTi::outputMode outputMode;
    pn.param<bool>("temperature", outputMode.temperatureData,false);
    pn.param<bool>("calibrated", outputMode.calibratedData,true);
    pn.param<bool>("orientation", outputMode.orientationData,true);
    pn.param<bool>("auxiliary", outputMode.auxiliaryData,false);
    pn.param<bool>("position", outputMode.positionData,false);
    pn.param<bool>("velocity", outputMode.velocityData,false);
    pn.param<bool>("status", outputMode.statusData,false);
    pn.param<bool>("rawGPS", outputMode.rawGPSData,false);
    pn.param<bool>("rawInertial", outputMode.rawInertialData,false);

    Xsens::MTi::outputSettings outputSettings;
    pn.param<bool>("timeStamp", outputSettings.timeStamp,false);
    int orientationMode;
    pn.param<int>("orientationMode", orientationMode,Xsens::Quaternion);
    outputSettings.orientationMode = (Xsens::MTOrientationMode)orientationMode;
    pn.param<bool>("enableAcceleration", outputSettings.enableAcceleration,false);
    pn.param<bool>("enableRateOfTurn", outputSettings.enableRateOfTurn,false);
    pn.param<bool>("enableMagnetometer", outputSettings.enableMagnetometer,false);
    pn.param<bool>("velocityModeNED", outputSettings.velocityModeNED,false);

    int scenarioAsInt;
    pn.param<int>("scenario",scenarioAsInt,Xsens::Automotive);
    Xsens::Scenario scenario = (Xsens::Scenario)scenarioAsInt;
    double GPSLeverArm_X, GPSLeverArm_Y, GPSLeverArm_Z;
    pn.param<double>("GPSLeverArm_X",GPSLeverArm_X,0.0);
    pn.param<double>("GPSLeverArm_Y",GPSLeverArm_Y,0.0);//0.25
    pn.param<double>("GPSLeverArm_Z",GPSLeverArm_Z,0.0);//0.70
    Xsens::MTi::Position GPS_lever_arm;
    GPS_lever_arm.x = GPSLeverArm_X;
    GPS_lever_arm.y = GPSLeverArm_Y;
    GPS_lever_arm.z = GPSLeverArm_Z;

    Xsens::MTi * mti = new Xsens::MTi();

    if(!mti->openPort((char*)portname.c_str(), baudrate))
    {
        ROS_FATAL("MTi -- Unable to connect to the MTi.");
        ROS_BREAK();
    }
    ROS_INFO("MTi -- Successfully connected to the MTi!");

    std::string prefix = "";
    pn.getParamCached("tf_prefix",prefix);
    ROS_INFO("tf_prefix: %s",prefix.c_str());
    if(!mti->setSettings(outputMode, outputSettings, scenario, prefix, frame_id, GPS_lever_arm, 1000))
    {
        ROS_FATAL("MTi -- Unable to set the output mode and settings.");
        ROS_BREAK();
    }
    ROS_INFO("MTi -- Setup complete! Initiating data streaming...");

    ros::Publisher mti_pub = pn.advertise<sensor_msgs::Imu>("imu/data", 10);
    ros::Publisher navsat_pub = pn.advertise<sensor_msgs::NavSatFix>("fix", 10);
    ros::Publisher odomPub = pn.advertise<nav_msgs::Odometry>("odom",10);

    tf::TransformBroadcaster odom_broadcaster;
    tf::TransformListener listener;

    ros::Rate r(20);
    while(ros::ok())
    {
        ros::Time now = ros::Time::now();
        sensor_msgs::Imu imu_msg = mti->fillImuMessage(now);
        mti_pub.publish(imu_msg);

        sensor_msgs::NavSatFix nav_fix_msg = mti->fillNavFixMessage(now);
        navsat_pub.publish(nav_fix_msg);

        nav_msgs::Odometry odom_msg = mti->fillOdometryMessage(listener, odom_broadcaster, now);
        odomPub.publish(odom_msg);

        ros::spinOnce();
        r.sleep();
    }

    return(0);
}

// EOF

