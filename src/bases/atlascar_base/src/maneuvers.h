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
#ifndef _MANEUVERS_H_
#define _MANEUVERS_H_

/**
\file
\brief Atlascar Maneuvers class declaration
*/

#include <ros/ros.h>
#include <atlascar_base/Gamepad.h>
#include <atlascar_base/ManagerCommand.h>
#include <atlascar_base/ManagerStatus.h>
#include <fstream>

using namespace std;

namespace atlascar_base
{
class Maneuvers
{
    public:        
        Maneuvers(const ros::NodeHandle& nh_):
        nh(nh_)
        {       
//             command_msgs.header

            command_msg.priority = 1;
            command_msg.lifetime = std::numeric_limits<double>::infinity();

            command_msg.velocity = 0;

            command_msg.auto_ignition = 1;
            command_msg.auto_brake = 1;
            command_msg.auto_direction = 1;
            command_msg.auto_clutch = 1;
            command_msg.auto_throttle = 1;

            command_msg.direct_control = 2;
        }
        
        /**
        \brief Initialize the class
        
        Add the safety message to the command queue.
        */
        void init();
        
        /**
        \brief Start ros message subscribing and advertising
        
        Subscribe command messages and advertise status messages.
        */
        void setupMessaging();
        
        /**
        \brief State Manager
        
        Analyzes the state of state machine
        */
        void stateManager();
        
        /**
        \brief Start main control loop
        
        Loop the module sending command messages (velocity) to the manager. It currently loops at about 100Hz.
        */
        void loop();
        
        /**
        \brief Car state machine status
        
        Define all states of macine 
        */      
        typedef enum 
        {
            UNKNOWN,
            INIT,
            
            POSITIVE_VELOCITY,
            NEUTRAL_VELOCITY,
            NEGATIVE_VELOCITY,
            PARALLEL_PARKING,
            REVERSING,
            NEUTRAL_VELOCITY_PARKING,
        } CarState;
        
        typedef enum 
        {
            FIRST,
            SECOND,
            THIRD,
            FOURTH,
            FIFTH,
            SIXTH,
            SEVENTH,
            EIGHTH,
            NINETH,
            TENTH
        } sequentialState;
        
        /**
        \brief Parking status
        
        Variable to define parking satus 
        */ 
        int maneuver;  
        
    protected:  
        
		double map(double value,double min_value,double max_value,double min_required,double max_required);
		
        void saveData(string report_name);
        /**
        \brief Callback - Change and send message to manager
        
        Impose a positive velocity
        */        
        void positive_velocity(int value);
        
        /**
        \brief Callback - Change and send message to manager
        
        Impose a neutral velocity 0 km/h
        */ 
        void neutral_velocity(int value);
        
        /**
        \brief Callback - Change and send message to manager
        
        Impose a neutral gear 
        */         
        void neutral_velocity_parking(int value);
        /**
        \brief Callback - Change and send message to manager
        
        Impose a negative velocity
        */ 
        void negative_velocity(int value);
        
        /**
        \brief Callback - Change and send message to manager
        
        Impose a sequence of velocity and direction
        */ 
        void parallel_parking(int value);
		
        /**
        \brief Callback - Change and send message to manager
        
        Impose a sequence of velocity and direction
        */ 
        void reversing(int value);

        /**
        \brief Callback - Status of all paramters 
        
        Allow check all parameters from manager
        */ 
        void statusCallback(const atlascar_base::ManagerStatusPtr& status);
        
        /**
        \brief Parallel
        
        Redefine values of variables to be send
        */
        void parallel();
        
         /**
        \brief Reverse
        
        Redefine values of variables to be send
        */
        void reverse();
        
        /**
        \name Class variables
        The class variables can be both public or private. They should preferably be private whenever
        possible.
        @{*/
        
        ///Current state of state machine
        CarState current_state; 
        
        ///Moving to stopped state 
        sequentialState pparking;
        
        ///Moving to stopped state 
        sequentialState rreverse;
        
        ///Ros node handler
        ros::NodeHandle nh;
        
        ///Ros command publisher
        ros::Publisher command_pub;
        
        ///Ros command subscriber
        ros::Subscriber command_sub_;
        
        ///Ros status subscriber
        ros::Subscriber status_sub;
            
        ///Command message
        atlascar_base::ManagerCommand command_msg;
        
        ///Status message
        atlascar_base::ManagerStatus status_msg;
		
        ///Status message memory
		atlascar_base::ManagerStatus status_memory;

        ///Gamepad communication class
        Gamepad gamepad;
		
        /**
        @}
        */
};

}
#endif