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
\brief Read data from bag and save to file.

This script reads the data,from the bag, and saves the data to .txt file.
*/

#include "ros/ros.h"
#include <mtt/TargetList.h>
#include <sensor_msgs/Range.h>
#include <pressure_cells/SenVal.h>
#include <odometer/OdometerStatus.h>
#include "cmath"
#include <iostream>
#include <vector>
#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>
#include <fstream>
#include <iostream>

using namespace std;

// void  laserGather(const mtt::TargetList& msg)
// {
// 	for(int i=0; i<msg.Targets.size();i++)
// 	{
//         if (msg.Targets[i].velocity.linear.x >= 0.05)
//         {
//             if (msg.Targets[i].id == 302 || msg.Targets[i].id == 27)
//             {     
//                 cout << "Time: " << msg.Targets[i].header.stamp << " | ID: " << msg.Targets[i].id << " | VEL: " << msg.Targets[i].velocity.linear.x << endl;
//                 
//                 ofstream file("/media/Data/Data_Record/velocity.txt",ios::app);
//                 
//                 file << "Time: " << msg.Targets[i].header.stamp << " | ID: " << msg.Targets[i].id << " | VEL: " << msg.Targets[i].velocity.linear.x << endl;
//                                
//                 file.close(); 
//         
//             }
//         }
// 	}       
// }

void  forcevalGather(const pressure_cells::SenVal& msg)
{
//     cout << msg.header.stamp << ":LC 1:" << msg.sen1 << ":FSR 1:" << msg.sen3 << endl;
//     cout << msg.header.stamp << ":LC 2:" << msg.sen2 << ":FSR 1:" << msg.sen4 << endl;
    ofstream file("/media/Data/Data_Record/newtests/force_data.txt",ios::app);  
    file << "Time:" << msg.header.stamp << ":LC1:" << msg.sen1 << ":FSR1:" << msg.sen3 << ":LC2:" << msg.sen2 << ":FSR2:" << msg.sen4 << endl;               
    file.close(); 
}

void  ir1Gather(const sensor_msgs::Range& msg)
{
//     cout << msg.header.stamp << " IR1: " << msg.range << endl;
    ofstream file("/media/Data/Data_Record/newtests/ir1_data.txt",ios::app);  
    file << "Time:" << msg.header.stamp << ":IR1:" << msg.range << endl;               
    file.close();
}

void  ir2Gather(const sensor_msgs::Range& msg)
{
//     cout << msg.header.stamp << " IR2: " << msg.range << endl;
    ofstream file("/media/Data/Data_Record/newtests/ir2_data.txt",ios::app);  
    file << "Time:" << msg.header.stamp << ":IR2:" << msg.range << endl;               
    file.close();
}

void  odometerGather(const odometer::OdometerStatus& msg)
{
//     cout << msg.header.stamp << " Velocity: " << msg.velocity << endl;
    
    ofstream file("/media/Data/Data_Record/newtests/odometer_data.txt",ios::app);    
    file << "Time:" << msg.header.stamp << ":VEL:" << msg.velocity << endl;                    
    file.close(); 
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "data_gather");
    
    ros::NodeHandle n;

//     ros::Subscriber laser_data = n.subscribe("/targets", 1000, laserGather);
    ros::Subscriber forceval_data = n.subscribe("/pedal_monitor/ForceVal", 1000, forcevalGather);
    ros::Subscriber ir1_data = n.subscribe("/pedal_monitor/IR_1", 1000, ir1Gather);
    ros::Subscriber ir2_data = n.subscribe("/pedal_monitor/IR_2", 1000, ir2Gather);
    ros::Subscriber odometer_data = n.subscribe("/pedal_monitor/odometer", 1000, odometerGather);
    
    
    
    ros::spin();

    return 0;
}