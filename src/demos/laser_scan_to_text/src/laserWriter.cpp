/**************************************************************************************************
 Software License Agreement (BSD License)

 Copyright (c) 2011-2013, LAR toolkit developers - University of Aveiro - http://lars.mec.ua.pt
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
/**@file class_xbox_ptu_teleop.cpp
 * @brief class class_xbox_ptu_teleop source code that implements major part of communication code for atlascar teleop node
 */

// #include <laser_scan_to_text/laserWriter.h>

#include <ros/ros.h>
#include <string>
#include <iostream>
#include <fstream>
#include <vector>
#include <sensor_msgs/LaserScan.h>

class LaserWriter{
    public:
        int version;
        bool firstScan;
        std::string outFilePath;
        
        LaserWriter(const ros::NodeHandle& nh_,std::string outFile):
            nh(nh_),
            firstScan(true),
            outFilePath(outFile),
            version(1)
        {}
            
        ~LaserWriter()
        {}
        
        void setupMessaging()
        {
            laserSubscriber = nh.subscribe<sensor_msgs::LaserScan>("laserScan", 1000, &LaserWriter::laserScanHandler, this);
        }
        
        void laserScanHandler(const sensor_msgs::LaserScan::ConstPtr& scan)
        {
            std::ofstream outFile;
            
            if(firstScan){
                outFile.open(outFilePath, std::ios_base::out);
                outFile << "#Laser Scan in Text Format, version: " << version<< std::endl;
                outFile << "#Created by Jorge Almeida from ros/bag"<< std::endl;
                outFile << "angle_min: " << scan->angle_min<<" #rad "<<std::endl;
                outFile << "angle_max: " << scan->angle_max<<" #rad "<<std::endl;
                outFile << "angle_increment: " << scan->angle_increment<<" #rad "<<std::endl;
                outFile << "time_increment: " << scan->time_increment<<" #sec"<<std::endl;
                outFile << "scan_time: " << scan->scan_time<<" #sec"<<std::endl;
                outFile << "range_min: " << scan->range_min<<" #m"<< std::endl;
                outFile << "range_max: " << scan->range_max<<" #m"<< std::endl;
                outFile <<"#Format of each scan: TIMESTAMP RANGE_1 RANGE_2 ... [RANGE_N]"<<std::endl;
                firstScan = false;
            }else{
                outFile.open(outFilePath, std::ios_base::app);
            }
            
            
            outFile << scan->header.stamp << " ";
            for(float range : scan->ranges){
                outFile << range << " ";
            }
            
            outFile << std::endl;
        }
        
        void loop()
        {
            ros::spin();
        }
        
        ///Ros node handler
        ros::NodeHandle nh;
        ///Ros command subscriber
        ros::Subscriber laserSubscriber;
};

int main(int argc, char** argv)
{
    std::cout<<"Starting laserWriter"<<std::endl;
    ros::init(argc, argv, "laserWriter", ros::init_options::AnonymousName);
    ros::NodeHandle nh;

    LaserWriter laserWriter(nh,"/home/jorge14/Desktop/test1.txt");
    
    //Setup message advertise and subscription
    laserWriter.setupMessaging();
    
    std::cout<<"In main loop"<<std::endl;
    //Main program loop
    laserWriter.loop();

    return true;
};