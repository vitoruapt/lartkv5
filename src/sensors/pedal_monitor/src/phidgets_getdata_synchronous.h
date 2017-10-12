/**************************************************************************************************
 Software License Agreement (BSD License)

 Copyright (c) 2011-2014, LAR toolkit developers - University of Aveiro - http://lars.mec.ua.pt
 All rights reserved.

 Redistribution and use in source and binary forms, with or without modification, are permitted
 provided that the following conditions are met:

  *Redistributions of source code must retain the above copyright notice, this list of
   conditions and the following disclaimer.
  *Redistributions in binary form must reproduce the above copyright notice, this list of
   conditions and the following disclaimer in the documentation and/or other materials provided
   with the distribution.
  *Neither the name of the University of Aveiro nor the names of its contributors may be used to
   endorse or promote products derived from this software without specific prior written permission.
 
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***************************************************************************************************/

/**
\file
\brief Include file for the program to monitor and publish the data from the pedals
*/

#ifndef SIMPLE_H
#define SIMPLE_H

#include <stdio.h>

#include <phidget21.h>

#include <ros/ros.h>

#include <ros/package.h>

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>

#include <sensor_msgs/Range.h>
#include <pressure_cells/SenVal.h>
#include <visualization_msgs/Marker.h>


class PhidgetClass
{
    public:
        
        PhidgetClass(boost::asio::io_service& io_service,ros::NodeHandle& nh_);
        
        void start();
        int AttachHandler(CPhidgetHandle IFK);
        int DetachHandler(CPhidgetHandle IFK);
        int ErrorHandler(CPhidgetHandle IFK, int ErrorCode, const char *unknown);
        
        void irHandler(boost::system::error_code const& cError);
        void highRateHandler(boost::system::error_code const& cError);
        
        int readIRSensors();
        int calibrateIRSensors();
        void publishIRValues();   
        int readFSRSensors();
        int calibrateFSRSensors();
        void publishFSRValues();
        void FSR_pub();
        
        int readLCSensors();
        int calibrateLCSensors();
        void publishLCValues();
        void LC_pub();

        int ir1_value;
        int ir2_value;
        int fsr1_value;
        int fsr2_value;
        int fsr3_value;
        int fsr4_value;
        int lc1_value;
        int lc2_value;
        
        double LC1_data, LC2_data, LC3_data; 
        double FSR1_data, FSR2_data, FSR3_data;
        double IR1_data, IR2_data, IR3_data;
        
        double ir1_max, ir1_min, ir1_vmax, ir1_vmin, ir1_v; 
        double ir2_max, ir2_min, ir2_vmax, ir2_vmin, ir2_v; 
        double ir3_max, ir3_min, ir3_vmax, ir3_vmin, ir3_v; 
        
        double fsr1_max, fsr1_min, fsr1_vmax, fsr1_vmin, fsr1_v; 
        double fsr2_max, fsr2_min, fsr2_vmax, fsr2_vmin, fsr2_v; 
        double fsr3_max, fsr3_min, fsr3_vmax, fsr3_vmin, fsr3_v; 
          
        double lc1_max, lc1_min, lc1_vmax, lc1_vmin, lc1_v; 
        double lc2_max, lc2_min, lc2_vmax, lc2_vmin, lc2_v;
        double lc3_max, lc3_min, lc3_vmax, lc3_vmin, lc3_v;
        
    private:
        
        sensor_msgs::Range ir1;
        sensor_msgs::Range ir2;
        pressure_cells::SenVal ForceVal;
        
        uint ir1_index;
        uint ir2_index;

        uint fsr1_index;
        uint fsr2_index;
        uint fsr3_index;
        uint fsr4_index;
        
        uint lc12_index;
        uint lc34_index;
            
        uint ir_period;
        uint high_rate_period;
        
        boost::asio::deadline_timer ir_timer;
        boost::asio::deadline_timer high_rate_timer;
       
        
        ros::Publisher ir1_publisher;
        ros::Publisher ir2_publisher;

        ros::Publisher ForceVal_publisher;
        
        visualization_msgs::Marker LC_marker;
        visualization_msgs::Marker FSR_marker;

        ros::Publisher LC_marker_pub;
        ros::Publisher FSR_marker_pub;

        ros::NodeHandle nh;
        
        CPhidgetInterfaceKitHandle ifKit;
};

#endif //SIMPLE_H