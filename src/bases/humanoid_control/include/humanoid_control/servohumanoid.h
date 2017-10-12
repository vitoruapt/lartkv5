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
 * @file servohumanoid.h
 * @author Emílio Estrelinha nº 38637 (emilio.estrelinha@ua.pt)
 * @brief Class develop to control PHUA robot's servomotors using hitec_5980SG Lar3's' module
 */

#ifndef SERVOHUMANOID_H
#define SERVOHUMANOID_H

#define PI 3.141592653589793238462643383279502884197

//Defines for left or right side limbs
#define LEFT     1
#define RIGHT   -1

// Define for dimension extraction
#define ARM_LENGTH  100
#define FOREARM_LENGTH  110
#define LEG_LENGTH  200
#define THIGH_LENGTH    210
#define FOOT_LENGTH     220
#define ANKLE_HEIGHT    230
#define FOOT_WIDTH  240

// #define PFLN {printf("Passing through File [%s] @ Line [%d].\n", __FILE__, __LINE__);}

//includes
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <ros/ros.h>

#include <Eigen/Dense>
#include <hitec5980sg/hitec5980sg.h>


class ServoHumanoid
{
    
    public:
        ServoHumanoid(const char* path);
   
        void HomePosition(void);

        short unsigned int MoveJoint(short int id, double joint_angle);
        
        short unsigned int SetJointSpeed(short int id, unsigned int speed);

        short unsigned int Torso_Rotation_Movement(double joint_angle);
        
        short unsigned int Torso_Flexion_Extension_Movement(double joint_angle);
        
        short unsigned int Torso_Lateral_Flexion_Extension_Movement(double joint_angle);
        
        short unsigned int Ankle_Flexion_Movement(short int id, double joint_angle);
        
        short unsigned int Ankle_Inversion_Eversion_Movement(short int id, double joint_angle);
        
        short unsigned int Knee_Movement(short int id, double joint_angle);

        short unsigned int Hip_Abduction_Hiperabduction_Movement(short int id, double joint_angle);

        short unsigned int Hip_Flexion_Movement(short int id, double joint_angle);

        double ConvertServoValueByID(int id, short unsigned int servo_value);
        
        double Hip_Flexion_ServoValue_Conversion(short int id, short unsigned int servo_value);
        
        double Hip_Abduction_Hiperabduction_ServoValue_Conversion(short int id, short unsigned int servo_value);
        
        double Knee_Flexion_ServoValue_Conversion(short int id, short unsigned int servo_value);
        
        double Ankle_Flexion_ServoValue_Conversion(short int id, short unsigned int servo_value);
        
        double Ankle_Inversion_Eversion_ServoValue_Conversion(short int id, short unsigned int servo_value);
        
        double Torso_Flexion_Extension_ServoValue_Conversion(short unsigned int servo_value);
        
        double Torso_Rotation_ServoValue_Conversion(short unsigned int servo_value);

        double Torso_Lateral_Flexion_Extension_ServoValue_Conversion(short unsigned int servo_value);

        //void Inv_Kin_3DOF_Detached_Leg(double X, double Y, double Z, double *req_angle);
        
        struct RoboState
        {
            // general speed configurations
            short unsigned int op_mode;
            
            double speed_g_static;
            //double speed_now[13];
            double speed_wanted[13];
            
            // Actual positions
            double joint_now[13];
        
            // wanted positions
            double joint_wanted[13];
            
        } RoboState;
        
    private:

        // servomotor Class
        hitec_5980SG* hitec;
        
        // Func to set parameters
        void SetParameters(void);
        
        /*~~~~~~~~~~~~~~~~~~~~~~~~~ 
        || miscellaneous variables ||
          ~~~~~~~~~~~~~~~~~~~~~~~~~*/
        //definition of PI as a double for calculations
        //double PI;
        
        /*~~~~~~~~~~~~~~~~~~~~~~~~~~~ 
        ||      robot dimensions     ||
          ~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
        /** 
        * @brief Structure to hold robot dimensional data.
        */
        struct{
        ///robot arm lenght (mm)
        double arm_length;
        
        ///robot forearm lenght (mm)
        double forearm_length;
        
        ///robot leg lenght (mm)
        double shin_length;
        
        ///robot thigh lenght (mm)
        double thigh_length;
        
        ///robot foot lenght (mm)
        double foot_length;
        
        ///robot foot lenght (mm)
        double foot_width;
        
        ///robot ankle height (mm)
        double ankle_height;
        } robot_dimensions;
        
        //   ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 
        // || robot joint limitation variables ||
        //   ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        /** 
        * @brief Structure to hold current robot joint limits.
        */
        struct{
        // <<<< HEAD >>>>   
        // Motion range [0...10] vertical to tilted
        
        ///head minimum angular limits
        double head_min_tilt;
        
        ///head maximum angular limits
        double head_max_tilt;
        
        /* <<<< ARMS >>>>*/
        /* Shoulder flexion
        Motion range [-40...140] back to front
        */
        ///shoulder flexion minimum angular limits
        double shoulder_min_flexion;
        
        ///shoulder flexion maximum angular limits
        double shoulder_max_flexion;
        
        /* Shoulder abduction
        Motion range [0...180] back to front
        */
        ///shoulder abduction minimum angular limits
        double shoulder_min_abduction;
        
        ///shoulder abduction maximum angular limits
        double shoulder_max_abduction;
        
        /* Elbow flexion
        Motion range [0...120] streched to flected
        */
        ///elbow flexion minimum angular limits
        double elbow_min_flexion;
        
        ///elbow flexion maximum angular limits
        double elbow_max_flexion;
        
        /* <<<< TORSO >>>>*/
        /* Torso rotation
        Motion range [-90...90] left to right
        */    
        ///torso rotation minimum angular limits
        double torso_min_rotation;
        
        ///torso rotation maximum angular limits
        double torso_max_rotation;
        
        /* Torso flexion
        Motion range [-15...90] left to right
        */    
        ///torso flexion minimum angular limits
        double torso_min_flexion;
        
        ///torso flexion maximum angular limits
        double torso_max_flexion;
        
        /* Torso lateral flexion
        Motion range [-90...90] left to right
        */
        ///torso lateral flexion minimum angular limits
        double torso_min_lateral_flexion;
        
        ///torso lateral flexion maximum angular limits
        double torso_max_lateral_flexion;
        
        /* Ankle inversion
        Motion range [-30...45] inside to outside
        */
        ///ankle inversion minimum angular limits
        double ankle_min_inversion;
        
        ///ankle inversion maximum angular limits
        double ankle_max_inversion;
        
        /* Ankle flexion
        Motion range [-40...20] inside to outside
        */
        ///ankle flexion minimum angular limits
        double ankle_min_flexion;
        
        ///ankle flexion maximum angular limits
        double ankle_max_flexion;
        
        /* Knee flexion
        Motion range [0...130] inside to outside
        */
        ///ankle inversion minimum angular limits
        double knee_min_flexion;
        
        ///ankle inversion maximum angular limits
        double knee_max_flexion;
        
        /* Hip abduction
        Motion range [-40...45] inside to outside
        */
        ///hip abduction minimum angular limits
        double hip_min_abduction;
        
        ///hip abduction maximum angular limits
        double hip_max_abduction;
        
        /* hip flexion
        Motion range [-30...120] inside to outside
        */
        ///hip inversion minimum angular limits
        double hip_min_flexion;
        
        ///hip inversion maximum angular limits
        double hip_max_flexion;
        } robot_joint_limits;
        
        /** 
        * @brief Structure to hold servomotor digital value conversion constants.
        */
        struct{
          ///head servo offset
          double head_servo_offset;
    
          ///shoulder flexion offset left
          double shoulder_flexion_servo_offset_left;
    
          ///shoulder flexion offset right
          double shoulder_flexion_servo_offset_right;
    
          ///shoulder abduction offset left
          double shoulder_abduction_servo_offset_left;
    
          ///shoulder abduction offset right
          double shoulder_abduction_servo_offset_right;
    
          ///elbow flexion offset left
          double elbow_flexion_servo_offset_left;
    
          ///elbow flexion offset right
          double elbow_flexion_servo_offset_right;
    
          ///torso rotation offset
          double torso_rotation_servo_offset;
    
          ///torso flexion offset
          double torso_flexion_servo_offset;
    
          ///torso lateral flexion offset
          double torso_lateral_flexion_servo_offset;
    
          ///knee flexion adjust curve b value left
          double knee_flexion_b_value_left;
    
          ///knee flexion adjust curve b value right
          double knee_flexion_b_value_right;
    
          ///hip flexion adjust curve b value left
          double hip_flexion_b_value_left;
    
          ///hip flexion adjust curve b value right
          double hip_flexion_b_value_right;
        } robot_servo_conversion;

        /** 
        * @brief Structure to hold the robot's joints offsets due to the mechanical assembling.
        */
        struct {
            ///head joint offset
          double head_joint_offset;
    
          ///shoulder flexion joint offset left
          double shoulder_flexion_joint_offset_left;
    
          ///shoulder flexion joint offset right
          double shoulder_flexion_joint_offset_right;
    
          ///shoulder abduction joint offset left
          double shoulder_abduction_joint_offset_left;
    
          ///shoulder abduction joint offset right
          double shoulder_abduction_joint_offset_right;
    
          ///elbow flexion joint offset left
          double elbow_flexion_joint_offset_left;
    
          ///elbow flexion joint offset right
          double elbow_flexion_joint_offset_right;
    
          ///torso rotation joint offset
          double torso_rotation_joint_offset;
    
          ///torso flexion joint offset
          double torso_flexion_joint_offset;
    
          ///torso lateral flexion joint offset
          double torso_lateral_flexion_joint_offset;
    
          ///knee flexion joint offset left
          double knee_flexion_joint_offset_left;
    
          ///knee flexion joint offset right
          double knee_flexion_joint_offset_right;
    
          ///hip flexion joint offset left
          double hip_flexion_joint_offset_left;
    
          ///hip flexion joint offset right
          double hip_flexion_joint_offset_right;

          ///hip abduction joint offset left
          double hip_abduction_joint_offset_left;
    
          ///hip abduction joint offset right
          double hip_abduction_joint_offset_right;

          ///feet flexion joint offset left
          double feet_flexion_joint_offset_left;
    
          ///feet flexion joint offset right
          double feet_flexion_joint_offset_right;

          ///feet inversion joint offset left
          double feet_inversion_joint_offset_left;
    
          ///feet inversion joint offset right
          double feet_inversion_joint_offset_right;
            
        } joint_offset;

        
        /** 
        * @brief Structure to hold the robot limb/element masses.
        */
        struct{
          ///foot mass
          double foot_mass;
          
          ///shin mass
          double shin_mass;
          
          ///thigh mass
          double thigh_mass;
          
        } robot_element_mass;
        
        /** 
        * @brief Structure to hold the robot limb centers of mass, in their own frames.
        */
        struct{
          ///foot COM
          Eigen::Vector3d foot_CoM;
          
          ///shin COM
          Eigen::Vector3d shin_CoM;
          
          ///thigh COM
          Eigen::Vector3d thigh_CoM;
          
        } robot_relative_COM;
        
    
};

#endif /* SERVOHUMANOID_H */ 
