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
/** @file
 * @brief humanoid_functions.h file for this module. Contains includes, prototypes and defines.
 * @author pedro_cruz
 * @version 2.0
 * @date 2 May 2012
 *@{
 */
#ifndef _HUMANOID_FUNCTIONS_H_
#define _HUMANOID_FUNCTIONS_H_

//Defines for left or right side limbs
#define LEFT	 1
#define RIGHT	-1

// Define for dimension extraction
#define ARM_LENGTH 	100
#define FOREARM_LENGTH 	110
#define LEG_LENGTH	200
#define THIGH_LENGTH 	210
#define FOOT_LENGTH 	220
#define ANKLE_HEIGHT 	230
#define FOOT_WIDTH	240

// #define PFLN {printf("Passing through File [%s] @ Line [%d].\n", __FILE__, __LINE__);}

//includes
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>

#include <Eigen/Dense>

/**
\brief Class to implement humanoid related functions.

This modules contains several functions used to work with kinematics (direct/inverse), convert joint angle to servo values and some other minor functions. \n
This class stores inside the robot dimensions, accessible thru a dedicated function. 
*/
class humanoid
{
  public:
    /**
    * @brief Constructor
    
    This functions sets the default values for internal variables.
    * @return void
    */
    humanoid(void);
    
    /**
    * @brief De-constructor
    
      This terminates the class.
    * @return void
    */
    ~humanoid(void);
    
    //FUNCTIONS FOR INVERSE/DIRECT KINEMATICS
    /** 
    * @brief Calculates humanoid arm end position given joint values.
    
    This function calculates humanoid arm end position and RPY values given joint values in joint space.
    * @param[in] t1 Shoulder Flexion value in degrees.
    * @param[in] t2 Shoulder Abduction value in degrees.
    * @param[in] t3 Elbow Flexion value in degrees.
    * @param[in] SIDE LEFT or RIGHT.
    * @param[out] end_pos_n_RPY 6 element array for storing the result.
    * @return array with the world position and RPY values.
    */
    void Dir_Kin_3DOF_Arm(double t1, double t2, double t3, short int SIDE, double *end_pos_n_RPY);
    
    /** 
    * @brief Calculates humanoid arm joint values given required position.
    
    This function calculates humanoid arm joint values given required end position in world space.
    (NOT TESTED FOR ALL RELEVANT OCTANTS! SEEMS TO BE WORKING FOR THE TIME BEEING...)
    * @param[in] X [X] required end position.
    * @param[in] Y [Y] required end position.
    * @param[in] Z [Z] required end position.
    * @param[out] req_angle 3 element array for storing the result.
    * @return array with all three angle values if possible / array with value 1000 if not.
    */
    void Inv_Kin_3DOF_Arm(double X, double Y, double Z, double *req_angle);
    
    /** 
    * @brief Calculates humanoid arm jacobian matrix for a given position.
    
    This function calculates the humanoid arm jacobian matrix given the joint angles.
    * @param[in] theta_1 value for the shoulder flexion joint.
    * @param[in] theta_2 value for the shoulder abduction joint.
    * @param[in] theta_3 value for the elbow flexion joint.
    * @param[out] jacobian matrix for storing the result.
    * @return none..
    */
    void CalculateArmJacobian(double theta_1, double theta_2, double theta_3, Eigen::Matrix3d *jacobian);
    
    /** 
    * @brief Calculates humanoid arm alternate jacobian matrix for no \\theta3 positions.
    
    This function calculates the humanoid arm alternate jacobian matrix for no deltaZ positions given the joint angles.
    Tis function is called on the points where the inverse jacobian matrix is impossible.
    * @param[in] theta_1 value for the shoulder flexion joint.
    * @param[in] theta_2 value for the shoulder abduction joint.
    * @param[in] theta_3 value for the elbow flexion joint.
    * @param[out] jacobian matrix for storing the result.
    * @return none.
    */
    void CalculateArmAlternateJacobian_No_Theta3_Singularity(double theta_1, double theta_2, double theta_3, Eigen::MatrixXd *jacobian);
    
    /** 
    * @brief Calculates humanoid detached leg end position given joint values.
    
    This function calculates humanoid detached leg end position and RPY values given joint values in joint space.
    * @param[in] t1 Ankle inversion value in degrees.
    * @param[in] t2 Ankle flexion value in degrees.
    * @param[in] t3 Knee flexion value in degrees.
    * @param[in] t4 Hip abduction value in degrees.
    * @param[in] SIDE LEFT or RIGHT.
    * @param[out] end_pos_n_RPY 6 element array for storing the result.
    * @return array with the world position and RPY values.
    */
    void Dir_Kin_4DOF_Detached_Leg(double t1, double t2, double t3, double t4, short int SIDE, double *end_pos_n_RPY);
    
    /** 
    * @brief Calculates humanoid detached leg joint values given required position.
    
    This function calculates humanoid detached leg joint values given required end position in world space.
    * @param[in] X [X] required end position.
    * @param[in] Y [Y] required end position.
    * @param[in] Z [Z] required end position.
    * @param[out] req_angle 3 element array for storing the result.
    * @return array with all three angle values if possible / array with value 1000 if not.
    */
    void Inv_Kin_3DOF_Detached_Leg(double X, double Y, double Z, double *req_angle);
    
    void Calculate_Detached_Leg_3DOF_Jacobian(double theta_1, double theta_2, double theta_3, Eigen::Matrix3d *jacobian);
    
    void Calculate_Detached_Leg_3DOF_COG(double theta_1, double theta_2, double theta_3, int SIDE, Eigen::Vector3d *COG);
    
    //THESE FUNCTIONS TRANSFORM JOINT ANGLE VALUES TO THEIR EQUIVALENT SERVOMOTOR VALUES
    /* ++++++++++++++++++++++++++++++++++++++++++
	++++++++++++++++++ HEAD ++++++++++++++++++
      ++++++++++++++++++++++++++++++++++++++++++*/
    /** 
    * @brief Converts joint angle for the head.
    
    This function converts joint angle for the head [0...10] to servomotor values [2300...2400].
    * @param[in] joint_angle a double with requested joint angle value in degrees.
    * @return returns respective servomotor value.
    */
    short unsigned int Head_Pan_Tilt_Conversion(double joint_angle);
    
    /* ++++++++++++++++++++++++++++++++++++++++++
	++++++++++++++++++ ARMS ++++++++++++++++++
	++++++++++++++++++++++++++++++++++++++++++*/
    /** 
    * @brief Converts joint angle for the shoulder flexion.
    
    This function converts joint angle for the shoulder flexion/extension [-40...140] to servomotor values [600...2400].
    * @param[in] joint_angle a double with requested joint angle value in degrees.
    * @param[in] SIDE LEFT or RIGHT
    * @return returns respective servomotor value.
    */
    short unsigned int Shoulder_Flexion_Extension_Conversion(short int SIDE, double joint_angle);

    /** 
    * @brief Converts joint angle for the shoulder abduction.
    
    This function converts joint angle for the shoulder abduction/adduction [0...180] to servomotor values [600...2400].
    * @param[in] joint_angle a double with requested joint angle value in degrees.
    * @param[in] SIDE LEFT or RIGHT.
    * @return returns respective servomotor value.
    */
    short unsigned int Shoulder_Abduction_Adduction_Conversion(short int SIDE, double joint_angle);

    /** 
    * @brief Converts joint angle for the left arm elbow flection.
    
    This function converts joint angle for the left arm elbow flection/extension [0...180] to servomotor values [2400...600].
    * @param[in] joint_angle a double with requested joint angle value in degrees.
    * @param[in] SIDE LEFT or RIGHT
    * @return returns respective servomotor value.
    */
    short unsigned int Elbow_Flexion_Extension_Conversion(short int SIDE, double joint_angle);

    /* ++++++++++++++++++++++++++++++++++++++++++
	++++++++++++++++++ TORSO +++++++++++++++++
	++++++++++++++++++++++++++++++++++++++++++*/
    /** 
    * @brief Converts joint angle for the torso rotation.
    
    This function converts joint angle for the torso rotation [-90...90] to servomotor values [600...2400].
    * @param[in] joint_angle a double with requested joint angle value in degrees.
    * @return returns respective servomotor value.
    */
    short unsigned int Torso_Rotation_Conversion(double joint_angle);

    /** 
    * @brief Converts joint angle for the torso flexion/extension.
    
    This function converts joint angle for the torso flexion [-15...90] to servomotor values [600...2400].
    * @param[in] joint_angle a double with requested joint angle value in degrees.
    * @return returns respective servomotor value.
    */
    short unsigned int Torso_Flexion_Extension_Conversion(double joint_angle);

    /** 
    * @brief Converts joint angle for the torso lateral flexion/extension.
    
    This function converts joint angle for the torso lateral flexion [-90...90] to servomotor values [600...2400].
    * @param[in] joint_angle a double with requested joint angle value
    * @return returns respective servomotor value.
    */
    short unsigned int Torso_Lateral_Flexion_Extension_Conversion(double joint_angle);
  
    /* ++++++++++++++++++++++++++++++++++++++++++
	++++++++++++++++++ LEGS ++++++++++++++++++
	++++++++++++++++++++++++++++++++++++++++++*/
    /** 
    * @brief Converts joint angle for ankle inversion/eversion.
    
    This function converts joint angle for the ankle eversion [-30...45] to servomotor values [600...2400].
    * @param[in] joint_angle a double with requested joint angle value in degrees.
    * @param[in] SIDE LEFT or RIGHT.
    * @return returns a character from input.
    */
    short unsigned int Ankle_Inversion_Eversion_Conversion(short int SIDE, double joint_angle);
    
    /** 
    * @brief Converts joint angle for ankle flexion.
    
    This function converts joint angle for the ankle flexion [-40...20] to servomotor values [600...2400].
    * @param[in] joint_angle a double with requested joint angle value in degrees.
    * @param[in] SIDE LEFT or RIGHT
    * @return returns a character from input.
    */
    short unsigned int Ankle_Flexion_Conversion(short int SIDE, double joint_angle);

    /** 
    * @brief Converts joint angle for the knee flexion.
    
    This function converts joint angle for the knee flexion [0...130] to servomotor values [600...2400].
    * @param[in] joint_angle a double with requested joint angle value
    * @param[in] SIDE LEFT or RIGHT
    * @return returns a character from input.
    */
    short unsigned int Knee_Conversion(short int SIDE, double joint_angle);

    /** 
    * @brief Converts joint angle for the hip abduction.
    
    This function converts joint angle for the hip hiperabduction/abduction [-40...45] to servomotor values [2400...600].
    * @param[in] joint_angle a double with requested joint angle value in degrees.
    * @param[in] SIDE LEFT or RIGHT.
    * @return returns a character from input.
    */
    short unsigned int Hip_Abduction_Hiperabduction_Conversion(short int SIDE, double joint_angle);
    
    /** 
    * @brief Converts joint angle for the hip flexion.
    
    This function converts joint angle for the hip flexion [-30...120] to servomotor values [600...2400].
    * @param[in] joint_angle a double with requested joint angle value in degrees.
    * @param[in] SIDE LEFT or RIGHT.
    * @return returns a character from input.
    */
    short unsigned int Hip_Flexion_Conversion(short int SIDE, double joint_angle);
    
    //THESE FUNCTIONS TRANSFORM SERVOMOTOR VALUES TO THEIR EQUIVALENT JOINT ANGLE VALUES 
    /* ++++++++++++++++++++++++++++++++++++++++++
	++++++++++++++++++ HEAD ++++++++++++++++++
      ++++++++++++++++++++++++++++++++++++++++++*/
    /** 
    * @brief Converts servo value for the head.
    
    This function converts servomotor values [2300...2400] to joint angle for the head [0...10].
    * @param[in] servo_value requested servo value.
    * @return returns respective joint value.
    */
    double Head_Pan_Tilt_ServoValue_Conversion(short unsigned int servo_value);
    
    /* ++++++++++++++++++++++++++++++++++++++++++
	++++++++++++++++++ ARMS ++++++++++++++++++
	++++++++++++++++++++++++++++++++++++++++++*/
    /** 
    * @brief Converts servo value for the shoulder flexion.
    
    This function converts servomotor values [600...2400] to joint angle for the shoulder flexion/extension [-40...140].
    * @param[in] servo_value requested servo value.
    * @param[in] SIDE LEFT or RIGHT
    * @return returns respective joint value.
    */
    double Shoulder_Flexion_Extension_ServoValue_Conversion(short int SIDE, short unsigned int servo_value);

    /** 
    * @brief Converts servo value for the shoulder abduction.
    
    This function converts servomotor values [600...2400] to joint angle for the shoulder abduction/adduction [0...180].
    * @param[in] servo_value requested servo value.
    * @param[in] SIDE LEFT or RIGHT
    * @return returns respective joint value.
    */
    double Shoulder_Abduction_Adduction_ServoValue_Conversion(short int SIDE, short unsigned int servo_value);

    /** 
    * @brief Converts servo value for the left arm elbow flection.
    
    This function converts servomotor values [2400...600] to joint angle for the left arm elbow flection/extension [0...180].
    * @param[in] servo_value requested servo value.
    * @param[in] SIDE LEFT or RIGHT
    * @return returns respective joint value.
    */
    double Elbow_Flexion_Extension_ServoValue_Conversion(short int SIDE, short unsigned int servo_value);
    
    /* ++++++++++++++++++++++++++++++++++++++++++
       ++++++++++++++++++ TORSO +++++++++++++++++
       ++++++++++++++++++++++++++++++++++++++++++*/
    /** 
    * @brief Converts servo value for the torso rotation.
    
    This function converts servomotor values [600...2400] to joint angle for the torso rotation [-90...90].
    * @param[in] servo_value requested servo value.
    * @return returns respective joint value.
    */
    double Torso_Rotation_ServoValue_Conversion(short unsigned int servo_value);

    /** 
    * @brief Converts servo value for the torso flexion/extension.
    
    This function converts servomotor values [600...2400] to joint angle for the torso flexion [-15...90] .
    * @param[in] servo_value requested servo value.
    * @return returns respective joint value.
    */
    double Torso_Flexion_Extension_ServoValue_Conversion(short unsigned int servo_value);

    /** 
    * @brief Converts servo value for the torso lateral flexion/extension.
    
    This function converts servomotor values [600...2400] to joint angle for the torso lateral flexion [-90...90].
    * @param[in] servo_value requested servo value.
    * @return returns respective joint value.
    */
    double Torso_Lateral_Flexion_Extension_ServoValue_Conversion(short unsigned int servo_value);
    
    /* ++++++++++++++++++++++++++++++++++++++++++
	++++++++++++++++++ LEGS ++++++++++++++++++
	++++++++++++++++++++++++++++++++++++++++++*/
    /** 
    * @brief Converts servo value for ankle inversion/eversion.
    
    This function converts servomotor values [600...2400] to joint angle for the ankle eversion [-30...45].
    * @param[in] servo_value requested servo value.
    * @param[in] SIDE LEFT or RIGHT
    * @return returns respective joint value.
    */
    double Ankle_Inversion_Eversion_ServoValue_Conversion(short int SIDE, short unsigned int servo_value);
    
    /** 
    * @brief Converts servo value for ankle flexion.
    
    This function converts servomotor values [600...2400] to joint angle for the ankle flexion [-40...20].
    * @param[in] servo_value requested servo value.
    * @param[in] SIDE LEFT or RIGHT
    * @return returns respective joint value.
    */
    double Ankle_Flexion_ServoValue_Conversion(short int SIDE, short unsigned int servo_value);

    /** 
    * @brief Converts servo value for the knee flexion.
    
    This function converts servomotor values [600...2400] to joint angle for the knee flexion [0...130].
    * @param[in] servo_value requested servo value.
    * @param[in] SIDE LEFT or RIGHT
    * @return returns respective joint value.
    */
    double Knee_Flexion_ServoValue_Conversion(short int SIDE, short unsigned int servo_value);

    /** 
    * @brief Converts servo value for the hip abduction.
    
    This function converts servomotor values [2400...600] to joint angle for the hip hiperabduction/abduction [-40...45].
    * @param[in] servo_value requested servo value.
    * @param[in] SIDE LEFT or RIGHT
    * @return returns respective joint value.
    */
    double Hip_Abduction_Hiperabduction_ServoValue_Conversion(short int SIDE, short unsigned int servo_value);
    
    /** 
    * @brief Converts servo value for the hip flexion.
    
    This function converts servomotor values [600...2400] to joint angle for the hip flexion [-30...120].
    * @param[in] servo_value requested servo value.
    * @param[in] SIDE LEFT or RIGHT
    * @return returns respective joint value.
    */
    double Hip_Flexion_ServoValue_Conversion(short int SIDE, short unsigned int servo_value);
    
    /* ++++++++++++++++++++++++++++++++++++++++++
	+++++++++ GET ROBOT DIMENSIONS ++++++++++
	++++++++++++++++++++++++++++++++++++++++++*/
    /** 
    * @brief Outputs selected robot dimension length.
    
    This function retrieves the selected robot length specified.\n
    The following defines can be used:\n
    ARM_LENGTH \n
    FOREARM_LENGTH \n
    LEG_LENGTH \n
    THIGH_LENGTH \n
    FOOT_LENGTH \n
    FOOT_WIDTH \n
    ANKLE_HEIGHT \n
    * @param[in] dimension specified robot dimension.
    * @return returns respective length.
    */
    double GetRobotDimension(int dimension);
    
  private:
    
    /*~~~~~~~~~~~~~~~~~~~~~~~~~ 
    || miscellaneous variables ||
      ~~~~~~~~~~~~~~~~~~~~~~~~~*/
    ///definition of PI as a double for calculations
    double PI;
    
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
      Eigen::Vector3d foot_COM;
      
      ///shin COM
      Eigen::Vector3d shin_COM;
      
      ///thigh COM
      Eigen::Vector3d thigh_COM;
      
    } robot_relative_COM;
    
};

#endif
/**
 *@}
*/
