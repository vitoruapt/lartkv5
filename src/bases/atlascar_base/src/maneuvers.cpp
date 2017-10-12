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
\file
\brief Atlascar Maneuvers class implementation
*/

#include "maneuvers.h"

namespace atlascar_base
{    
    void Maneuvers::init()
    {
        maneuver=0;
        //Initialize communications with the gamepad
        int ret = gamepad.startComm("/dev/input/js0");
        
        //Error Message
        if(ret<0)
        {
            cout<<"Cannot open communications with gamepad on: "<<"/dev/input/js0"<<endl;
            return;
        }
        
        current_state = INIT;
		
		command_msg.lifetime = INFINITY;
		command_msg.priority = 1;
        
        //Register all gamepad callbacks
        gamepad.buttons(0)->callback = sigc::mem_fun<int>(this,&Maneuvers::positive_velocity);
        gamepad.buttons(2)->callback = sigc::mem_fun<int>(this,&Maneuvers::neutral_velocity_parking);
        gamepad.buttons(1)->callback = sigc::mem_fun<int>(this,&Maneuvers::neutral_velocity);
        gamepad.buttons(3)->callback = sigc::mem_fun<int>(this,&Maneuvers::negative_velocity);
        gamepad.buttons(12)->callback = sigc::mem_fun<int>(this,&Maneuvers::parallel_parking);
        gamepad.buttons(14)->callback = sigc::mem_fun<int>(this,&Maneuvers::reversing);
	}
	
	//used to write a txt
	/*
    void Maneuvers::saveData(string report_name)
    {
        ofstream file(report_name.c_str(),ios::app);
        file<<maneuver<<" "<<status_msg.header.stamp<<" "<<status_msg.velocity<<" "<<status_msg.count<<" "<<status_msg.gear<<" "<<status_msg.clutch<<" "<<status_msg.brake<<" "<<status_msg.steering_wheel<<" "<<status_msg.rpm<<" "<<status_msg.throttle_pedal<<" "<<status_msg.throttle_pressure<<" "<<status_msg.brake_pressure<<" "<<status_msg.clutch_pressure<<endl; 
        file.close(); 
    }*/
        
    void Maneuvers::setupMessaging()
    {
        //Subscribe to the status message
        status_sub = nh.subscribe("status", 1, &Maneuvers::statusCallback, this);
        //Advertise command messages
        command_pub = nh.advertise<atlascar_base::ManagerCommand>("command_msg", 1);
    }    
        
    void Maneuvers::statusCallback(const atlascar_base::ManagerStatusPtr& status)
    {
        //Save the status message
        status_msg.velocity=status->velocity;
        status_msg.count=status->count;
        status_msg.pulses_sec=status->pulses_sec;
        status_msg.revolutions_sec=status->revolutions_sec;
		status_msg.header.stamp=status->header.stamp;
		status_msg.gear=status->gear;
		status_msg.clutch=status->clutch;
		status_msg.brake=status->brake;
		status_msg.steering_wheel=status->steering_wheel;
		status_msg.rpm=status->rpm;
		status_msg.throttle_pedal=status->throttle_pedal;
		status_msg.velocity=status->velocity;
		status_msg.count=status->count;
		status_msg.throttle_pressure=status->throttle_pressure;
		status_msg.brake_pressure=status->brake_pressure;
		status_msg.clutch_pressure=status->clutch_pressure;	
        
        //used to write a txt
        /*
        string file_name = "/home/atlascar/Desktop/teste.txt"; 
        saveData(file_name);
        */
    }   
	
    void Maneuvers::positive_velocity(int value)
    {    
        //sets number of maneuver 
         maneuver=1;
         
        //safety
        if(value==0)
            return;
        //sets the current state
        if (current_state==INIT)
            current_state = POSITIVE_VELOCITY;
    }
    
   
    void Maneuvers::negative_velocity(int value)
    {   
        //sets number of maneuver 
        maneuver=2;
        
        //safety
        if(value==0)
            return;        
        //sets the current state
        if (current_state==INIT)
            current_state = NEGATIVE_VELOCITY;
    }
    
    void Maneuvers::parallel_parking(int value)
    {   
        //sets number of maneuver 
        maneuver=3;
        
        //safety
        if(value==0)
            return;
        //sets the current state
        if (current_state==INIT)
		{
			status_memory.count = status_msg.count;
			pparking=FIRST;
            current_state = PARALLEL_PARKING;			
		}
    }
    
    void Maneuvers::reversing(int value)
    {   
        //sets number of maneuver 
        maneuver=4;
        
        //safety
        if(value==0)
            return;
        //sets the current state
        if (current_state==INIT)
        {
            status_memory.count = status_msg.count;
			rreverse=FIRST;
            current_state = REVERSING;
        }
    }
    
    void Maneuvers::neutral_velocity(int value)
    { 
        //sets number of maneuver 
        maneuver=0;
        
        //safety
        if(value==0)
            return;
        //sets the current state
        current_state = NEUTRAL_VELOCITY;
    }
    
    void Maneuvers::neutral_velocity_parking(int value)
    { 
        //sets number of maneuver 
        maneuver=0;
        
        //safety
        if(value==0)
            return;
        //sets the current state
        if (current_state = INIT)
            current_state = NEUTRAL_VELOCITY_PARKING;        
    }
 
    void Maneuvers::reverse()
    {
        switch(rreverse)
        {
            //first stage 
            case FIRST:
                command_msg.velocity=2;
                command_msg.steering_wheel=0.45;
                
                //exit stage
                if (status_msg.count>status_memory.count+10)
					rreverse=SECOND;
				
                break;
                
            //second stage (safety)              
            case SECOND:
                command_msg.velocity=0;
                command_msg.steering_wheel=0.45;
                 
                //exit stage
                if (abs(status_msg.velocity)<0.01)
				{
					status_memory.count=status_msg.count;
					rreverse=THIRD;
				}

                break;
            
            //third stage
            case THIRD: 
                command_msg.velocity=-2;
                command_msg.steering_wheel=-0.45;
				
                //exit stage
                if (status_msg.count<status_memory.count-2)
                    rreverse=FOURTH;
				
                break;
            
            //fourth stage (safety)    
            case FOURTH:
                command_msg.velocity=0;
                command_msg.steering_wheel=-0.45;                
                
                //exit stage
                if (abs(status_msg.velocity)<0.01)
				{
					status_memory.count=status_msg.count;
					rreverse=FIFTH;
				}
				
                break;
                
            //fifth satge
            case FIFTH:  
                command_msg.velocity=2;
                command_msg.steering_wheel=0.45;
                
                //exit stage
                if (status_msg.count>status_memory.count+2)
                    rreverse=SIXTH;
				
                break;
                
            //sixth stage (safety)    
            case SIXTH:
                command_msg.velocity=0;
                command_msg.steering_wheel=0.45;                
                
                //exit stage
                if (abs(status_msg.velocity)<0.01)
				{
					status_memory.count=status_msg.count;
					rreverse=SEVENTH;
				}
				
				break;
                     
            //seventh stage    
            case SEVENTH:  
                
                command_msg.velocity=-2;
                command_msg.steering_wheel=-0.45;
                
                //exit stage
                if (status_msg.count<status_memory.count-2)
                    rreverse=EIGHTH;
				
                break;
                
            //eighth stage (safety)    
            case EIGHTH:
                command_msg.velocity=0;
                command_msg.steering_wheel=-0.45;                
                
                //exit stage
                if (abs(status_msg.velocity)<0.01)
				{
					status_memory.count=status_msg.count;
					rreverse=NINETH;
				}
				
                break;
               
            //nineth stage    
            case NINETH:                
                command_msg.velocity=2;
                command_msg.steering_wheel=0.45;
				
                //exit stage
                if (status_msg.count>status_memory.count+5)
                    rreverse=TENTH;
				
                break;
               
            //tenth stage (end state)    
            case TENTH:
                command_msg.velocity=0;
                command_msg.steering_wheel=0;
                
                //exit stage
                if (abs(status_msg.velocity)<0.01)
                {
                    maneuver=0;
                    //end state
                    current_state=INIT;
                }           
                
                break;                  
        }
    }
    
    void Maneuvers::parallel()
    {
        switch(pparking)
        {
            //first stage
            case FIRST:
                command_msg.velocity=-2;
                command_msg.steering_wheel=-0.40;
				
                //exit stage
                if (status_msg.count<status_memory.count-10)
                {
                    status_memory.count=status_msg.count;
                    pparking=SECOND;
                }
                break;
               
            //second stage (safety)    
            case SECOND:
				command_msg.velocity=0;
				command_msg.steering_wheel=-0.40;
				
                //exit stage
                if (abs(status_msg.velocity)<0.01)
				{
					status_memory.count=status_msg.count;
					pparking=THIRD;
				}
				
                break;				                
             
            //third stage    
            case THIRD:
				command_msg.velocity=-2;
                command_msg.steering_wheel=0.40;
				
                //exit stage
                if (status_msg.count<status_memory.count-2)
                    pparking=FOURTH;
				
                break;
				
            //fouth stage (safety)    
			case FOURTH:			
				command_msg.velocity=0;
                command_msg.steering_wheel=0.40;
				
                //exit stage
                if (abs(status_msg.velocity)<0.01)
                {
                    status_memory.count=status_msg.count;
                    pparking=FIFTH;
                }   
                
                break;
        
            //fifth stage    
			case FIFTH:
				command_msg.velocity=2;
                command_msg.steering_wheel=-0.40;
				
                //exit stage
                if (status_msg.count>status_memory.count+1)                           
                    pparking=SIXTH;
                
                break;
                
            //sixth stage (end state)    
			case SIXTH:
				 command_msg.velocity=0;
                command_msg.steering_wheel=0;
				
                //exit stage
                if (abs(status_msg.velocity)<0.01)
                {
                    maneuver=0;
                    //end state
                    current_state=INIT;
                } 
                
                break;
        }
    }

    void Maneuvers::stateManager()
    {        
        //state machine
        switch(current_state)
        {        
            case UNKNOWN:
                break;
                
            case INIT:            
                break;
                
            case POSITIVE_VELOCITY:
                //stage
                command_msg.parking=0;
				command_msg.velocity = 2;
				command_msg.steering_wheel=0;
                
                //end state
                current_state=INIT;
                break;
               
            case NEUTRAL_VELOCITY:
                //stage
                command_msg.parking=0;
				command_msg.velocity = 0;
				command_msg.steering_wheel=0;
                
                //end state
                current_state=INIT;
                break;
                
            case NEGATIVE_VELOCITY:
                //stage
                command_msg.parking=0;
				command_msg.velocity = -2;
				command_msg.steering_wheel=0;
                
                //end state
                current_state=INIT;
                break;
                
            case PARALLEL_PARKING:
                command_msg.parking=0;
                parallel();                
                break;  
                
            case REVERSING:
                command_msg.parking=0;
                reverse();                
                break;
                
            case NEUTRAL_VELOCITY_PARKING:
                //stage
                command_msg.parking=1;
                
                //end state
                current_state=INIT;
                break;
                
            default:
                cout<<"This state was not defined: "<<current_state<<endl;
        };
    }
    
    void Maneuvers::loop()
    {
        //Frequency
        ros::Rate r(100);//Hz

        do
        {
            //Verify the status of gamepad 
            gamepad.dispatch();
            
            stateManager();
            
            command_msg.header.stamp = ros::Time::now();
            //Publish the message
            command_pub.publish(command_msg);
            
            //Spin ros and sleep the desired amount
            ros::spinOnce();
            r.sleep();
            
        }while(ros::ok());        
    }
}

