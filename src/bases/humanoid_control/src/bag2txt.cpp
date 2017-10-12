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

#include <unistd.h>

#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <fstream>

#include <ros/ros.h>

#include <tf/transform_listener.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <haptic_force/Force.h>
#include <pressure_cells/Cop.h>
#include <phantom_control/State.h>
#include <humanoid_control/Humanoid.h>
#include <pressure_cells/SenVal.h>

using namespace std;
using namespace ros;

class Conversion
{
    public:
        
        Conversion():
        n_("~")
        {
            arduino_1_values_report_name_ = ros::names::remap("arduino_1_values_report");
            arduino_2_values_report_name_ = ros::names::remap("arduino_2_values_report");
            
            cop_left_right_report_name_ = ros::names::remap("cop_left_right_report");
            cop_left_report_name_ = ros::names::remap("cop_left_report");
            cop_right_report_name_ = ros::names::remap("cop_right_report");

            force_report_name_ = ros::names::remap("force_report_name_");
            humanoid_state_report_name_ = ros::names::remap("humanoid_state_report");
            phantom_state_report_name_ = ros::names::remap("phantom_state_report");
            
            remove(arduino_1_values_report_name_.c_str());
            remove(arduino_2_values_report_name_.c_str());
            
            remove(cop_left_right_report_name_.c_str());
            remove(cop_left_report_name_.c_str());
            remove(cop_right_report_name_.c_str());

            remove(force_report_name_.c_str());
            remove(humanoid_state_report_name_.c_str());
            remove(phantom_state_report_name_.c_str());
            
            arduino_1_handler_ = n_.subscribe<pressure_cells::SenVal> ("/arduino_1_values",1000,boost::bind(&Conversion::ArduinoHandler,this,_1,arduino_1_values_report_name_));
            arduino_2_handler_ = n_.subscribe<pressure_cells::SenVal> ("/arduino_2_values",1000,boost::bind(&Conversion::ArduinoHandler,this,_1,arduino_2_values_report_name_));
            
            cop_left_right_handler_ = n_.subscribe<pressure_cells::Cop> ("/cop_left_right",1000,boost::bind(&Conversion::COPHandler,this,_1,cop_left_right_report_name_));
            cop_left_handler_ = n_.subscribe<pressure_cells::Cop> ("/cop_left",1000,boost::bind(&Conversion::COPHandler,this,_1,cop_left_report_name_));
            cop_right_handler_ = n_.subscribe<pressure_cells::Cop> ("/cop_right",1000,boost::bind(&Conversion::COPHandler,this,_1,cop_right_report_name_));
            
            force_report_handler_ = n_.subscribe ("/force",1000, &Conversion::ForceHandler, this);
            
            humanoid_state_handler_ = n_.subscribe("/humanoid_state",1000, &Conversion::HumanoidHandler, this);
            
            phantom_state_handler_ = n_.subscribe ("/phantom_state",1000, &Conversion::PhantomHandler, this);
        }
        
        ~Conversion()
        {
            
        }
        
        void ArduinoHandler(const pressure_cells::SenVal::ConstPtr& ard_values, string report_name)
        {
            ofstream report_ard;
            report_ard.open(report_name.c_str(),ios::app);
            
            boost::format fm("%.6f");// %.13f %.13f %.13f %.13f");
            fm % ard_values->header.stamp.toSec();// % ard_values.sen1 % ard_values.sen2 % ard_values.sen3 % ard_values.sen4;
    
            report_ard<<ard_values->header.seq<<" "<<fm.str()<<" "<< ard_values->sen1<<" "<< ard_values->sen2 <<" "<< ard_values->sen3 <<" "<<ard_values->sen4<<endl;
            
            report_ard.close();
        }
        
        void COPHandler(const pressure_cells::Cop::ConstPtr& cop_values, string report_name)
        {
            ofstream report_cop;
            report_cop.open(report_name.c_str(),ios::app);
            
            boost::format fm("%.6f");
            fm % cop_values->header.stamp.toSec() ;
    
            report_cop<<cop_values->header.seq<<" "<<fm.str()<<" "<< cop_values->copx <<" "<<cop_values->copy<<endl;
            
            report_cop.close();
        }
        
        void ForceHandler(const haptic_force::ForcePtr& force_values)
        {
            ofstream report_force;
            report_force.open(force_report_name_.c_str(),ios::app);
            
            boost::format fm("%.6f");
            fm % force_values->header.stamp.toSec();
    
            report_force<<force_values->header.seq<<" "<<fm.str()<<" "<< force_values->force[0]<<" "<< force_values->force[1]<<" "<< force_values->force[2]<<endl;
            
            report_force.close();
            
        }

        void HumanoidHandler(const humanoid_control::HumanoidPtr& humanoid_values)
        {
            ofstream report_humanoid;
            report_humanoid.open(humanoid_state_report_name_.c_str(),ios::app);
            
            boost::format fm("%.6f");
            fm % humanoid_values->header.stamp.toSec();
    
            report_humanoid<<humanoid_values->header.seq<<" "<<fm.str();
            
            for (int i = 0; i < 12; i++)
            {
                report_humanoid<<" "<<humanoid_values->speed_wanted[i];
            }
            for (int i = 0; i < 12; i++)
            {
                report_humanoid<<" "<<humanoid_values->joint_now[i];
            }
            for (int i = 0; i < 12; i++)
            {
                report_humanoid<<" "<<humanoid_values->joint_wanted[i];
            }

            report_humanoid<<endl;

            report_humanoid.close();
            
        }

        void PhantomHandler(const phantom_control::StatePtr& phantom_values)
        {
            ofstream report_phantom;
            report_phantom.open(phantom_state_report_name_.c_str(),ios::app);
            
            boost::format fm("%.6f");
            fm % phantom_values->header.stamp.toSec();
    
            report_phantom<<phantom_values->header.seq<<" "<<fm.str();
            
            for (int i = 0; i < 3; i++)
            {
                report_phantom<<" "<<phantom_values->position[i];
            }
            for (int i = 0; i < 3; i++)
            {
                report_phantom<<" "<<phantom_values->rot[i];
            }
            for (int i = 0; i < 3; i++)
            {
                report_phantom<<" "<<phantom_values->joints[i];
            }
            for (int i = 0; i < 3; i++)
            {
                report_phantom<<" "<<phantom_values->force[i];
            }
            for (int i = 0; i < 2; i++)
            {
                report_phantom<<" "<<phantom_values->buttons[i];
            }
            
            report_phantom<<" "<<phantom_values->home;
            
            for (int i = 0; i < 3; i++)
            {
                report_phantom<<" "<<phantom_values->home_pos[i];
            }

            report_phantom<<endl;

            report_phantom.close();
            
        }
        
        string arduino_1_values_report_name_;
        string arduino_2_values_report_name_;
        
        string cop_left_right_report_name_;
        string cop_left_report_name_;
        string cop_right_report_name_;

        string force_report_name_;
        string humanoid_state_report_name_;
        string phantom_state_report_name_;
        
        ros::Subscriber arduino_1_handler_; 
        ros::Subscriber arduino_2_handler_;
        
        ros::Subscriber cop_left_right_handler_;
        ros::Subscriber cop_left_handler_;
        ros::Subscriber cop_right_handler_;
        
        ros::Subscriber force_report_handler_;
        ros::Subscriber humanoid_state_handler_;
        ros::Subscriber phantom_state_handler_;
        
        ros::NodeHandle n_;
};

int main(int argc,char**argv)
{
    ros::init(argc, argv, "bag2txt");
            
    Conversion convert;
    
    cout<<"spining ..."<<endl;
    ros::spin();
    
    return 0;
}
