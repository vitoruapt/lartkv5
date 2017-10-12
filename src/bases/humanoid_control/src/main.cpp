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
/**
 * @file main.cpp
 * @author Emílio Estrelinha nº 38637 (emilio.estrelinha@ua.pt)
 * @brief Reteives data and commands Humanoid robot
 */
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <sstream>
#include <unistd.h>
#include <iostream>
#include <cmath>  

#include <sys/types.h>
#include <unistd.h>
#include <stdlib.h>

#include <humanoid_control/servohumanoid.h>
#include <hitec5980sg/hitec5980sg.h>

#include <phantom_control/State.h>
#include <humanoid_control/Humanoid.h>


using namespace ros;
using namespace std;


ServoHumanoid *g_human;

phantom_control::State g_PState;
humanoid_control::Humanoid g_RState;


const double g_ZZ=THIGH_LENGTH+LEG_LENGTH;
short int firsttme=1;

//const short int id_list[12]={11,12,13,15,16,21,22,23,25,26,31,32};
const short int id_list[12]={11,21,12,22,13,23,15,25,16,26,31,32};      // bottom to top servo's command

void PhantomCallBk ( const phantom_control::State &phantom_state )
{
    double joint_angle, current_POS;
    double  z_scale = 0.5, x_scale = 5, y_scale=0.8;

    //cout<<"Pos XX: "<<phantom_state.position[0]<<"\tPos YY: "<<phantom_state.position[1]<<"\tPos ZZ: "<<phantom_state.position[2]<<endl;
    
    g_PState=phantom_state;
    
    if(g_human->RoboState.op_mode==1)
        {
        current_POS=g_PState.position[1];
        
        joint_angle = acos ((g_ZZ + current_POS * z_scale)/ g_ZZ) * 180.0 / PI;
        
        if(isnan(joint_angle)) joint_angle=0;
        if(joint_angle>20.) joint_angle=20.0;
        
        g_human->RoboState.joint_wanted[2]=joint_angle;
        g_human->RoboState.joint_wanted[3]=joint_angle;
        g_human->RoboState.joint_wanted[4]=joint_angle*2.;
        g_human->RoboState.joint_wanted[5]=joint_angle*2.;
        g_human->RoboState.joint_wanted[8]=joint_angle;
        g_human->RoboState.joint_wanted[9]=joint_angle;

        cout<<"Ankle angle = "<<joint_angle<<endl;
        }
    else if(g_human->RoboState.op_mode==2)
    {
        current_POS=g_PState.position[0];
        
        joint_angle = atan2(current_POS * x_scale, g_ZZ) * 180.0 / PI;
        
        if(isnan(joint_angle)) joint_angle=0;
        if(joint_angle>30.) joint_angle=30.0;
        if(joint_angle<-30.) joint_angle=-30.0;
        
        g_human->RoboState.joint_wanted[0]=-joint_angle;
        g_human->RoboState.joint_wanted[1]=joint_angle;
        g_human->RoboState.joint_wanted[6]=joint_angle;
        g_human->RoboState.joint_wanted[7]=-joint_angle;
        
        cout<<"Ankle angle = "<<joint_angle<<endl;
    }
    else if(g_human->RoboState.op_mode==3)
    {
        if (firsttme==1)
        {
            cout<<"First Time"<<endl;
            
            g_human->RoboState.joint_wanted[2]=20.;
            g_human->RoboState.joint_wanted[3]=20.;
            g_human->RoboState.joint_wanted[4]=40.;
            g_human->RoboState.joint_wanted[5]=40.;
            g_human->RoboState.joint_wanted[8]=20.;
            g_human->RoboState.joint_wanted[9]=20.;

            firsttme=0;
        }
        else
        {
            current_POS=-g_PState.position[2];
            
            joint_angle = asin((current_POS * y_scale)/THIGH_LENGTH) * 180.0 / PI + 20.;
            
            //if(isnan(joint_angle)) joint_angle=0;
            if(joint_angle>0.) g_human->RoboState.joint_wanted[4]=joint_angle*2.;
            if(joint_angle<0.) g_human->RoboState.joint_wanted[4]=joint_angle/2.;
            
            g_human->RoboState.joint_wanted[2]=20.;
            g_human->RoboState.joint_wanted[3]=20.;
            //g_human->RoboState.joint_wanted[4]=joint_angle*2.;
            g_human->RoboState.joint_wanted[5]=g_human->RoboState.joint_wanted[4];
            g_human->RoboState.joint_wanted[8]=joint_angle;
            g_human->RoboState.joint_wanted[9]=joint_angle;
            
            cout<<"Hip angle = "<<joint_angle<<endl;
    
        }
    }
    else if(g_human->RoboState.op_mode==0)
    {
        
        

        for (int i = 0; i < 13; i++)
        {
            g_human->RoboState.joint_wanted[i]=0;
        }
    }
}

/**
* @fn int main ( int argc, char **argv )
* Main 
*/
int main(int argc, char** argv)
{
    short unsigned int resp=65535;
    
    // ROS
    ros::init( argc, argv, "humanoid_node" );//nome
    
    ros::NodeHandle n("~");
    
    ros::Publisher pub_state= n.advertise< humanoid_control::Humanoid >( "/humanoid_state", 1000 );
    
    ros::Subscriber sub_force = n.subscribe ( "/phantom_state", 1, PhantomCallBk );

    int op_mode = 0;
    int op_mode_default = 0;
    n.param("op_mode", op_mode, op_mode_default );
    
    g_human = (ServoHumanoid*) new ServoHumanoid("/dev/ttyUSB0");

    if(op_mode==0)
    {
        ROS_INFO("Operation mode of humanoid robot is 0");
        ROS_INFO("STOP MODE");
        
        g_human->RoboState.op_mode=op_mode;
        g_human->RoboState.joint_wanted[0]=0.;
        g_human->RoboState.joint_wanted[1]=0.;
        g_human->RoboState.joint_wanted[2]=10.;
        g_human->RoboState.joint_wanted[3]=10.;
        g_human->RoboState.joint_wanted[4]=20.;
        g_human->RoboState.joint_wanted[5]=20.;
        g_human->RoboState.joint_wanted[6]=0.;
        g_human->RoboState.joint_wanted[7]=0.;
        g_human->RoboState.joint_wanted[8]=15.;
        g_human->RoboState.joint_wanted[9]=15.;
        g_human->RoboState.joint_wanted[10]=0.;
        g_human->RoboState.joint_wanted[11]=0.;
    
    }else if(op_mode==1)
    {
        ROS_INFO("Operation mode of humanoid robot is 1");
        ROS_INFO("BOUNCY MODE");
        g_human->RoboState.op_mode=op_mode;
    }else if(op_mode==2)
    {
        ROS_INFO("Operation mode of humanoid robot is 2");
        ROS_INFO("SHAKY MODE");
        g_human->RoboState.op_mode=op_mode;
    }else if(op_mode==3)
    {
        ROS_INFO("Operation mode of humanoid robot is 3");
        ROS_INFO("HUMPING MODE");
        g_human->RoboState.op_mode=op_mode;
    }
    
    for (int i = 0; i < 12; i++)
    {
        g_human->RoboState.speed_wanted[i]=50;
    }
    //g_human->RoboState.speed_wanted[2]=3*50;
    //g_human->RoboState.speed_wanted[3]=3*50;
    
    g_human->HomePosition();

    
//     // test loop
//     short int id;
//     double pos;
//     while(1)
//     {
//         cout<<"Set servo id:"<<endl;
//         cin>>id;
//         if(id==555) exit(0);
//         cout<<"Set angle to move servo "<<id<<" :"<<endl;
//         cin>>pos;
//         resp = g_human->MoveJoint(id, pos);
//         cout<<"The servo responded "<<resp<<" to the command."<<endl;
//     }
//     return 0;
//     // end test loop

    
    g_RState.speed_g_static=g_human->RoboState.speed_g_static;

    int pid = getpid(), rpid;

    boost::format fmt("sudo renice -10 %d");
    fmt % pid;
    
    rpid = std::system(fmt.str().c_str());
    
    while(ros::ok())
    {       
        for (uint i = 0; i <12 ; i++)
        {
            if(  (g_human->RoboState.joint_now[i] > (g_human->RoboState.joint_wanted[i] + 1 )) || (g_human->RoboState.joint_now[i] < (g_human->RoboState.joint_wanted[i] - 1 )))
            {
                resp = g_human->MoveJoint(id_list[i], g_human->RoboState.joint_wanted[i]);
                
                //cout<<"Robot needs to change position in joint "<<id_list[i]<<" : position now "<<g_human->RoboState.joint_now[i]<<"   position wanted "<<g_human->RoboState.joint_wanted[i]<<endl;
            }
            
            resp = g_human->SetJointSpeed(id_list[i], g_human->RoboState.speed_wanted[i]);
            g_human->RoboState.joint_now[i]=g_human->ConvertServoValueByID(id_list[i], resp);
        }
        
        for (int i = 0; i < 13 ; i++)
        {
            g_RState.joint_now[i]=g_human->RoboState.joint_now[i];
            g_RState.speed_wanted[i]=g_human->RoboState.speed_wanted[i];
            g_RState.speed_wanted[i]=g_human->RoboState.speed_wanted[i];
        }
        
        g_RState.header.stamp=ros::Time::now();
        // publish HumanState
        pub_state.publish( g_RState );

        
        ros::spinOnce (  );
    }

    delete g_human;

    
    return 0;
}
