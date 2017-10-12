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

/*! 
 *  @file   head_motion.cpp
 *  @brief  subscribe points from arrow pose and move the head 
 *  
 *  @author     César Sousa, cesarsousa@ua.pt
 *  @date 4-6-2014
 *  @version V0.0
 *  @internal
 * 
 *      Revision    ---
 *      Compiler    gcc
 *      Company     DEM - Universidade de Aveiro
 *      Copyright   Copyright (c) 2014, César Sousa
 * 
 *      Info:       
 *  
 *  command to make doxyfile: "make doxyfile" than "make doc"
 * 
 */
 
#include <ros/ros.h>

#include <hitec5980sg/hitec5980sg.h>
#include <geometry_msgs/Point.h>

using namespace ros;
using namespace std;

short unsigned int x_before = 1950;
short unsigned int y_before = 1470;


class Head
{
    public:
        
        Head(std::string device):
        servos(device.c_str())
        {
            
        }
        
        void positionHandler(const geometry_msgs::Point& point)
        {
            if ((point.z <= 0) && (point.y <= 0))
            {
                servos.SetPosition(53,x_before);
                //std::cout << "x_before: " << x_before << "y_before" << y_before << std::endl;
                //servos.SetSpeedPosition(53,0);
                servos.SetPosition(41,y_before);
                //servos.SetSpeedPosition(41,0);
            }
            else
            {
                if ((point.z >= 0) && (point.z <= 255) && (point.y >= 0) && (point.y <= 255))//validate velocity
                {
                    if (point.x == 1)
                    {
                        servos.SetPosition(53,1600);
                        x_before = servos.SetSpeedPosition(53,point.y);
                        
                        servos.SetPosition(41,2000);
                        y_before = servos.SetSpeedPosition(41,point.z);
                    }
                    if (point.x == 2)
                    {
                        servos.SetPosition(53,1600);
                        x_before = servos.SetSpeedPosition(53,point.y);
                        
                        servos.SetPosition(41,600);
                        y_before = servos.SetSpeedPosition(41,point.z);  
                    }
                    if (point.x == 3)
                    {
                        servos.SetPosition(53,2000);
                        x_before = servos.SetSpeedPosition(53,point.y);
                        
                        servos.SetPosition(41,2000);
                        y_before = servos.SetSpeedPosition(41,point.z); 
                    }
                    if (point.x == 4)
                    {
                        servos.SetPosition(53,2000);
                        x_before = servos.SetSpeedPosition(53,point.y);
                        
                        servos.SetPosition(41,600);
                        y_before = servos.SetSpeedPosition(41,point.z);
                    }
                }
                else
                {
                    std::cout << "invalid  velocity: " << point.z << std::endl;
                }
            }
        }
        
        hitec_5980SG servos;
};

int main(int argc, char** argv)
{
    ros::init( argc, argv, "head_motion" );//nome
    
    ros::NodeHandle n("~");
    
    Head head("/dev/ttyUSB0");
    std::cout << "initializing head motion..." << std::endl;
    
    ros::Subscriber sub = n.subscribe("/find_arrow_state", 1,&Head::positionHandler,&head);
    
    ros::spin();
    
    return 0;
}
