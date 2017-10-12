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
 * @brief humanoid_control_functions.h file for this module. Contains includes and prototypes.
 * @author pedro_cruz
 * @version 1.0
 * @date 11 May 2012
 *@{
 */
#ifndef __HUMANOID_CONTROL_FUNCTIONS_H_
#define __HUMANOID_CONTROL_FUNCTIONS_H_

#include <phua_haptic/humanoid_functions.h>
#include <phua_haptic/miscellaneous.h>
#include <phua_haptic/types.h>

#include <Eigen/Dense>
#include <armadillo>

#define PATH_FOLLOWING_POSITION_ERROR 1 //double

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~   
||  shared data definition  ||
~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

/** 
* @brief Function to convert a joint angle for a given humanoid servo ID.

This function converts joint angles to servo values for a given servomotor ID.
Depends on external class type "humanoid".
* @param[in] id ID of the servo to convert.
* @param[in] angle_position angle in degrees of the desired joint angle position.
* @return the corresponding servomotor value.
*/
short unsigned int ConvertJointAngleByID(int id, double angle_position);

/** 
* @brief Function to convert a servo value for a given humanoid servo ID.

This function converts servo values to joint values for a given servomotor ID.
Depends on external class type "humanoid".
* @param[in] id ID of the servo to convert.
* @param[in] servo_value servomotor position value.
* @return the corresponding joint angle value.
*/
double ConvertServoValueByID(int id, short unsigned int servo_value);

/** 
* @brief Function to send commands to set the robot to its home position.

This function sends a go to zero command to all joints of the robot.
Depends on external class type "hitec5980sg".
* @param[in] RobotVars structure defined in types.h with the joint values.
* @return none.
*/
void SetRobotHomePosition(shared_vars_t*RobotVars);

/** 
* @brief Function to update the arm end point.

This function updates the arm end point values with the current arm joint values.
* @param[in] RobotVars structure defined in types.h with the joint values.
* @return none.
*/
void UpdateArmsDirKinData(shared_vars_t*RobotVars);

/** 
* @brief Function to update the detached legs end point.

This function updates the detached legs end point values with the current detached legs joint values.
* @param[in] RobotVars structure defined in types.h with the joint values.
* @return none.
*/
void UpdateDetachedLegsDirKinData(shared_vars_t*RobotVars);

/** 
* @brief Function to set speed to arm servos.

This function sets a new speed to all the arm servos.
* @param[in] SIDE LEFT or RIGHT.
* @param[in] speed servo digital speed to set.
* @param[in] RobotVars structure defined in types.h with the joint values.
* @return none.
*/
void SetArmSpeed(short int SIDE, short int speed, shared_vars_t*RobotVars);

/** 
* @brief Function to set speed to leg servos.

This function sets a new speed to all the leg servos.
* @param[in] SIDE LEFT or RIGHT.
* @param[in] speed servo digital speed to set.
* @param[in] RobotVars structure defined in types.h with the joint values.
* @return none.
*/
void SetLegSpeed(short int SIDE, short int speed, shared_vars_t*RobotVars);

/** 
* @brief Function to move arm end point to given coordinates.

This function calculates joint value for given 3D space coordinates for the arms and sends commands to the servos to reach that position.
* @param[in] X xx axis desired position.
* @param[in] Y yy axis desired position.
* @param[in] Z zz axis desired position.
* @param[in] SIDE LEFT or RIGHT.
* @param[in] RobotVars structure defined in types.h with the joint values.
* @return none.
*/
void MoveArmToCartesianPosition(double X, double Y, double Z, short int SIDE, shared_vars_t*RobotVars);

/** 
* @brief Function to move detached leg end point to given coordinates.

This function calculates joint value for given 3D space coordinates for the detached legs and sends commands to the servos to reach that position.
* @param[in] X xx axis desired position.
* @param[in] Y yy axis desired position.
* @param[in] Z zz axis desired position.
* @param[in] t4 hip abduction desired angle.
* @param[in] SIDE LEFT or RIGHT.
* @param[in] RobotVars structure defined in types.h with the joint values.
* @return none.
*/
void MoveDetachedLegToCartesianPosition(double X, double Y, double Z, double t4, short int SIDE, shared_vars_t*RobotVars);

/** 
* @brief Function to update joint value in the data struture from supplied ID.

This function updates the joint values in the data struture from by means of the servo ID.
If parameter id is 1000, the function will scan all the robot joints and update the structure.
* @param[in] id ID of the joint servo.
* @param[in] joint_angle joint value to update.
* @param[in] RobotVars structure defined in types.h with the joint values.
* @return none.
*/
void UpdateJointDataByID(short int id, double joint_angle, shared_vars_t*RobotVars);

/** 
* @brief Function set different automatic speeds to arm servomotors.

This function sends commands to the arms servomotors to update the current speed and updates the information in the data structure.
* @param[in] SIDE LEFT or RIGHT.
* @param[in] RobotVars structure defined in types.h with the joint values.
* @return none.
*/
void SetArmAutomaticSpeeds(short int SIDE, shared_vars_t*RobotVars);

/** 
* @brief Function to calculate joint angular speed given 3d space velocity vector.

This function uses the inverse jacobian to calculate joint angular speed.
* @param[in] avg_speed 3d speed components.
* @param[in] RobotVars structure defined in types.h with the joint values.
* @return none.
*/
void CalculateDifferencialArmServoSpeed(double *avg_speed, shared_vars_t*RobotVars);

/** 
* @brief Function to stop robot at its current joint configuration.

This function sends commands to read current robot joint coordinates followed by commands for that position.
* @param[in] RobotVars structure defined in types.h with the joint values.
* @return none.
*/
void StopRobotMovement(shared_vars_t*RobotVars);

/** 
* @brief Main robot arm speed control type function.

This function is the main function for the arms speed control type.
* @param[in] avg_speed average joystick cartesian speed vector.
* @param[in] RobotVars structure defined in types.h with the joint values.
* @return none.
*/
void ArmsDifferencialSpeedControl(double *avg_speed, shared_vars_t*RobotVars);

/** 
* @brief Function to write points to file.
* @param[in] RobotVars structure defined in types.h with the joint values.
* @return none.
*/
void DemoUserPath_WritePoints(shared_vars_t*RobotVars);

/** 
* @brief Function to read points from file.
* @param[in] RobotVars structure defined in types.h with the joint values.
* @return none.
*/
void DemoUserPath_ReadPoints(shared_vars_t*RobotVars);

/** 
* @brief Function to run points read from file.
* @param[in] RobotVars structure defined in types.h with the joint values.
* @return none.
*/
void PathFollowingExecute(shared_vars_t*RobotVars);

/** 
* @brief Function to update shared structure with the current direct kinematic of the robot.
* @param[in] RobotVars structure defined in types.h with the joint values.
* @return none.
*/
void UpdateKinematicModelDirKin(shared_vars_t*RobotVars);

#endif
/**
 *@}
*/
