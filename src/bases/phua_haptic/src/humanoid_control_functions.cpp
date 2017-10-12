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
 * \file
 * \brief Humanoid control functions implementation
 */

#include <phua_haptic/humanoid_control_functions.h>
#include <phua_haptic/gtk_aux.h>

using namespace std;

short unsigned int ConvertJointAngleByID(int id, double angle_position)
{
  //create pointer to class to acess setPosition function
  humanoid *humanoid_f=(humanoid*) new humanoid();
  short unsigned int servo_position=0;
  switch (id){
	case 61:
	{
	  servo_position=humanoid_f->Head_Pan_Tilt_Conversion(angle_position);
	  break;
	}
	case 54:
	{
	  servo_position=humanoid_f->Shoulder_Flexion_Extension_Conversion(RIGHT,angle_position);
	  break;
	}
	case 53:
	{
	  servo_position=humanoid_f->Shoulder_Abduction_Adduction_Conversion(RIGHT,angle_position);
	  break;
	}
	case 51:
	{
	  servo_position=humanoid_f->Elbow_Flexion_Extension_Conversion(RIGHT,angle_position);
	  break;
	}
	case 44:
	{
	  servo_position=humanoid_f->Shoulder_Flexion_Extension_Conversion(LEFT,angle_position);
	  break;
	}
	case 43:
	{
	  servo_position=humanoid_f->Shoulder_Abduction_Adduction_Conversion(LEFT,angle_position);
	  break;
	}
	case 41:
	{
	  servo_position=humanoid_f->Elbow_Flexion_Extension_Conversion(LEFT,angle_position);
	  break;
	}
	case 31:
	{
	  servo_position=humanoid_f->Torso_Rotation_Conversion(angle_position);
	  break;
	}
	case 32:
	{
	  servo_position=humanoid_f->Torso_Flexion_Extension_Conversion(angle_position);
	  break;
	}
	case 33:
	{
	  servo_position=humanoid_f->Torso_Lateral_Flexion_Extension_Conversion(angle_position);
	  break;
	}
	case 21:
	{
	  servo_position=humanoid_f->Ankle_Inversion_Eversion_Conversion(LEFT,angle_position);
	  break;
	}
	case 22:
	{
	  servo_position=humanoid_f->Ankle_Flexion_Conversion(LEFT,angle_position);
	  break;
	}
	case 23:
	{
	  servo_position=humanoid_f->Knee_Conversion(LEFT,angle_position);
	  break;
	}
	case 25:
	{
	  servo_position=humanoid_f->Hip_Abduction_Hiperabduction_Conversion(LEFT,angle_position);
	  break;
	}
	case 26:
	{
	  servo_position=humanoid_f->Hip_Flexion_Conversion(LEFT,angle_position);
	  break;
	}
	case 11:
	{
	  servo_position=humanoid_f->Ankle_Inversion_Eversion_Conversion(RIGHT,angle_position);
	  break;
	}
	case 12:
	{
	  servo_position=humanoid_f->Ankle_Flexion_Conversion(RIGHT,angle_position);
	  break;
	}
	case 13:
	{
	  servo_position=humanoid_f->Knee_Conversion(RIGHT,angle_position);
	  break;
	}
	case 15:
	{
	  servo_position=humanoid_f->Hip_Abduction_Hiperabduction_Conversion(RIGHT,angle_position);
	  break;
	}
	case 16:
	{
	  servo_position=humanoid_f->Hip_Flexion_Conversion(RIGHT,angle_position);
	  break;
	}
	default:
	{
	  servo_position=0xFFFF;
	  break;
	}
      }
  delete humanoid_f;
  return servo_position;
}

void UpdateArmsDirKinData(shared_vars_t*RobotVars)
{
  //calc dir kin
  double *end_pos_n_RPY= new double[6];
  
  //left
  RobotVars->humanoid_f->Dir_Kin_3DOF_Arm(RobotVars->robot_kin_data.LeftShoulderFlexion,
			       RobotVars->robot_kin_data.LeftShoulderAbduction,
			       RobotVars->robot_kin_data.LeftElbowFlexion,
			       LEFT,
			       end_pos_n_RPY);
			       
  RobotVars->robot_kin_data.X_arm_end_left=end_pos_n_RPY[0];
  RobotVars->robot_kin_data.Y_arm_end_left=end_pos_n_RPY[1];
  RobotVars->robot_kin_data.Z_arm_end_left=end_pos_n_RPY[2];
  RobotVars->robot_kin_data.ROLL_arm_end_left=end_pos_n_RPY[3];
  RobotVars->robot_kin_data.PITCH_arm_end_left=end_pos_n_RPY[4];
  RobotVars->robot_kin_data.YAW_arm_end_left=end_pos_n_RPY[5];
  //right
  RobotVars->humanoid_f->Dir_Kin_3DOF_Arm(RobotVars->robot_kin_data.RightShoulderFlexion,
			       RobotVars->robot_kin_data.RightShoulderAbduction,
			       RobotVars->robot_kin_data.RightElbowFlexion,
			       RIGHT,
			       end_pos_n_RPY);
			       
  RobotVars->robot_kin_data.X_arm_end_right=end_pos_n_RPY[0];
  RobotVars->robot_kin_data.Y_arm_end_right=end_pos_n_RPY[1];
  RobotVars->robot_kin_data.Z_arm_end_right=end_pos_n_RPY[2];
  RobotVars->robot_kin_data.ROLL_arm_end_right=end_pos_n_RPY[3];
  RobotVars->robot_kin_data.PITCH_arm_end_right=end_pos_n_RPY[4];
  RobotVars->robot_kin_data.YAW_arm_end_right=end_pos_n_RPY[5];
  delete end_pos_n_RPY;
}

void UpdateDetachedLegsDirKinData(shared_vars_t*RobotVars)
{
  //calc dir kin
  double *end_pos_n_RPY= new double[6];
  
  //left
  RobotVars->humanoid_f->Dir_Kin_4DOF_Detached_Leg(RobotVars->robot_kin_data.LeftAnkleInversion,
						  RobotVars->robot_kin_data.LeftAnkleFlexion,
						  RobotVars->robot_kin_data.LeftKneeFlexion,
						  RobotVars->robot_kin_data.LeftHipAbduction,
						  LEFT,
						  end_pos_n_RPY);
  
  RobotVars->robot_kin_data.detached_leg_pos_left[0] = end_pos_n_RPY[0];
  RobotVars->robot_kin_data.detached_leg_pos_left[1] = end_pos_n_RPY[1];
  RobotVars->robot_kin_data.detached_leg_pos_left[2] = end_pos_n_RPY[2];
  RobotVars->robot_kin_data.detached_leg_rpy_left[0] = end_pos_n_RPY[3];
  RobotVars->robot_kin_data.detached_leg_rpy_left[1] = end_pos_n_RPY[4];
  RobotVars->robot_kin_data.detached_leg_rpy_left[2] = end_pos_n_RPY[5];
  
  //update COG
  RobotVars->humanoid_f->Calculate_Detached_Leg_3DOF_COG(RobotVars->robot_kin_data.LeftAnkleInversion,
							RobotVars->robot_kin_data.LeftAnkleFlexion,
							RobotVars->robot_kin_data.LeftKneeFlexion,
							LEFT,
							&RobotVars->robot_kin_data.COG_detached_leg_left);
  
  //right
  RobotVars->humanoid_f->Dir_Kin_4DOF_Detached_Leg(RobotVars->robot_kin_data.RightAnkleInversion,
						  RobotVars->robot_kin_data.RightAnkleFlexion,
						  RobotVars->robot_kin_data.RightKneeFlexion,
						  RobotVars->robot_kin_data.RightHipAbduction,
						  RIGHT,
						  end_pos_n_RPY);
  
  RobotVars->robot_kin_data.detached_leg_pos_right[0] = end_pos_n_RPY[0];
  RobotVars->robot_kin_data.detached_leg_pos_right[1] = end_pos_n_RPY[1];
  RobotVars->robot_kin_data.detached_leg_pos_right[2] = end_pos_n_RPY[2];
  RobotVars->robot_kin_data.detached_leg_rpy_right[0] = end_pos_n_RPY[3];
  RobotVars->robot_kin_data.detached_leg_rpy_right[1] = end_pos_n_RPY[4];
  RobotVars->robot_kin_data.detached_leg_rpy_right[2] = end_pos_n_RPY[5];
  
  //update COG
  RobotVars->humanoid_f->Calculate_Detached_Leg_3DOF_COG(RobotVars->robot_kin_data.RightAnkleInversion,
						      RobotVars->robot_kin_data.RightAnkleFlexion,
						      RobotVars->robot_kin_data.RightKneeFlexion,
						      RIGHT,
						      &RobotVars->robot_kin_data.COG_detached_leg_right);
  
  delete end_pos_n_RPY;
}

void SetRobotHomePosition(shared_vars_t*RobotVars)
{
  static short int speed;
  double default_angle_position = 0.0;
  double default_arms_angle_position = 20.0;
  int ret;
  int id;
  speed = RobotVars->parameters.robot_home_position_base_speed;
  if(RobotVars->parameters.kinematic_model==4)
  {
    id=21;
    ret=RobotVars->servo->SetSpeedPosition(id,speed);
    ret=RobotVars->servo->SetPosition(id,RobotVars->humanoid_f->Ankle_Inversion_Eversion_Conversion(LEFT,default_angle_position));
    id=22;
    ret=RobotVars->servo->SetSpeedPosition(id,speed);
    ret=RobotVars->servo->SetPosition(id,RobotVars->humanoid_f->Ankle_Flexion_Conversion(LEFT,default_angle_position));
    id=23;
    ret=RobotVars->servo->SetSpeedPosition(id,speed);
    ret=RobotVars->servo->SetPosition(id,RobotVars->humanoid_f->Knee_Conversion(LEFT,default_angle_position));
    id=25;
    ret=RobotVars->servo->SetSpeedPosition(id,speed);
    ret=RobotVars->servo->SetPosition(id,RobotVars->humanoid_f->Hip_Abduction_Hiperabduction_Conversion(LEFT,default_angle_position));
    id=26;
    ret=RobotVars->servo->SetSpeedPosition(id,speed);
    ret=RobotVars->servo->SetPosition(id,RobotVars->humanoid_f->Hip_Flexion_Conversion(LEFT,default_angle_position));
  }
  else if(RobotVars->parameters.kinematic_model==5)
  {
    id=11;
    ret=RobotVars->servo->SetSpeedPosition(id,speed);
    ret=RobotVars->servo->SetPosition(id,RobotVars->humanoid_f->Ankle_Inversion_Eversion_Conversion(RIGHT,default_angle_position));
    id=12;
    ret=RobotVars->servo->SetSpeedPosition(id,speed);
    ret=RobotVars->servo->SetPosition(id,RobotVars->humanoid_f->Ankle_Flexion_Conversion(RIGHT,default_angle_position));
    id=13;
    ret=RobotVars->servo->SetSpeedPosition(id,speed);
    ret=RobotVars->servo->SetPosition(id,RobotVars->humanoid_f->Knee_Conversion(RIGHT,default_angle_position));
    id=15;
    ret=RobotVars->servo->SetSpeedPosition(id,speed);
    ret=RobotVars->servo->SetPosition(id,RobotVars->humanoid_f->Hip_Abduction_Hiperabduction_Conversion(RIGHT,default_angle_position));
    id=16;
    ret=RobotVars->servo->SetSpeedPosition(id,speed);
    ret=RobotVars->servo->SetPosition(id,RobotVars->humanoid_f->Hip_Flexion_Conversion(RIGHT,default_angle_position));
  }
  else
  {
    id=61;
    ret=RobotVars->servo->SetSpeedPosition(id,speed);
    ret=RobotVars->servo->SetPosition(id,RobotVars->humanoid_f->Head_Pan_Tilt_Conversion(default_angle_position));
    id=44;
    ret=RobotVars->servo->SetSpeedPosition(id,speed);
    ret=RobotVars->servo->SetPosition(id,RobotVars->humanoid_f->Shoulder_Flexion_Extension_Conversion(LEFT,default_angle_position));
    id=43;
    ret=RobotVars->servo->SetSpeedPosition(id,speed);
    ret=RobotVars->servo->SetPosition(id,RobotVars->humanoid_f->Shoulder_Abduction_Adduction_Conversion(LEFT,default_arms_angle_position));
    id=41;
    ret=RobotVars->servo->SetSpeedPosition(id,speed);
    ret=RobotVars->servo->SetPosition(id,RobotVars->humanoid_f->Elbow_Flexion_Extension_Conversion(LEFT,default_arms_angle_position));
    id=54;
    ret=RobotVars->servo->SetSpeedPosition(id,speed);
    ret=RobotVars->servo->SetPosition(id,RobotVars->humanoid_f->Shoulder_Flexion_Extension_Conversion(RIGHT,default_angle_position));
    RobotVars->parameters.arm_auto_speed[0]=speed;
    id=53;
    ret=RobotVars->servo->SetSpeedPosition(id,speed);
    ret=RobotVars->servo->SetPosition(id,RobotVars->humanoid_f->Shoulder_Abduction_Adduction_Conversion(RIGHT,default_arms_angle_position));
    RobotVars->parameters.arm_auto_speed[1]=speed;
    id=51;
    ret=RobotVars->servo->SetSpeedPosition(id,speed);
    ret=RobotVars->servo->SetPosition(id,RobotVars->humanoid_f->Elbow_Flexion_Extension_Conversion(RIGHT,default_arms_angle_position));
    RobotVars->parameters.arm_auto_speed[2]=speed;
    id=31;
    ret=RobotVars->servo->SetSpeedPosition(id,speed);
    ret=RobotVars->servo->SetPosition(id,RobotVars->humanoid_f->Torso_Rotation_Conversion(default_angle_position));
    id=32;
    ret=RobotVars->servo->SetSpeedPosition(id,speed);
    ret=RobotVars->servo->SetPosition(id,RobotVars->humanoid_f->Torso_Flexion_Extension_Conversion(default_angle_position));
    id=33;
    ret=RobotVars->servo->SetSpeedPosition(id,speed);
    ret=RobotVars->servo->SetPosition(id,RobotVars->humanoid_f->Torso_Lateral_Flexion_Extension_Conversion(default_angle_position));
    id=11;
    ret=RobotVars->servo->SetSpeedPosition(id,speed);
    ret=RobotVars->servo->SetPosition(id,RobotVars->humanoid_f->Ankle_Inversion_Eversion_Conversion(RIGHT,default_angle_position));
    id=12;
    ret=RobotVars->servo->SetSpeedPosition(id,speed);
    ret=RobotVars->servo->SetPosition(id,RobotVars->humanoid_f->Ankle_Flexion_Conversion(RIGHT,default_angle_position));
    id=13;
    ret=RobotVars->servo->SetSpeedPosition(id,speed);
    ret=RobotVars->servo->SetPosition(id,RobotVars->humanoid_f->Knee_Conversion(RIGHT,default_angle_position));
    id=15;
    ret=RobotVars->servo->SetSpeedPosition(id,speed);
    ret=RobotVars->servo->SetPosition(id,RobotVars->humanoid_f->Hip_Abduction_Hiperabduction_Conversion(RIGHT,default_angle_position));
    id=16;
    ret=RobotVars->servo->SetSpeedPosition(id,speed);
    ret=RobotVars->servo->SetPosition(id,RobotVars->humanoid_f->Hip_Flexion_Conversion(RIGHT,default_angle_position));
    id=21;
    ret=RobotVars->servo->SetSpeedPosition(id,speed);
    ret=RobotVars->servo->SetPosition(id,RobotVars->humanoid_f->Ankle_Inversion_Eversion_Conversion(LEFT,default_angle_position));
    id=22;
    ret=RobotVars->servo->SetSpeedPosition(id,speed);
    ret=RobotVars->servo->SetPosition(id,RobotVars->humanoid_f->Ankle_Flexion_Conversion(LEFT,default_angle_position));
    id=23;
    ret=RobotVars->servo->SetSpeedPosition(id,speed);
    ret=RobotVars->servo->SetPosition(id,RobotVars->humanoid_f->Knee_Conversion(LEFT,default_angle_position));
    id=25;
    ret=RobotVars->servo->SetSpeedPosition(id,speed);
    ret=RobotVars->servo->SetPosition(id,RobotVars->humanoid_f->Hip_Abduction_Hiperabduction_Conversion(LEFT,default_angle_position));
    id=26;
    ret=RobotVars->servo->SetSpeedPosition(id,speed);
    ret=RobotVars->servo->SetPosition(id,RobotVars->humanoid_f->Hip_Flexion_Conversion(LEFT,default_angle_position));
  }
  RobotVars->parameters.manual_speed = speed;
}

double ConvertServoValueByID(int id, short unsigned int servo_value)
{
  //create pointer to class to acess setPosition function
  humanoid *humanoid_f=(humanoid*) new humanoid();
  double joint_angle=0xFFFF;
  switch (id){
	case 61:
	{
	  joint_angle=humanoid_f->Head_Pan_Tilt_ServoValue_Conversion(servo_value);
	  break;
	}
	case 54:
	{
	  joint_angle=humanoid_f->Shoulder_Flexion_Extension_ServoValue_Conversion(RIGHT, servo_value);
	  break;
	}
	case 53:
	{
	  joint_angle=humanoid_f->Shoulder_Abduction_Adduction_ServoValue_Conversion(RIGHT, servo_value);
	  break;
	}
	case 51:
	{
	  joint_angle=humanoid_f->Elbow_Flexion_Extension_ServoValue_Conversion(RIGHT, servo_value);
	  break;
	}
	case 44:
	{
	  joint_angle=humanoid_f->Shoulder_Flexion_Extension_ServoValue_Conversion(LEFT, servo_value);;
	  break;
	}
	case 43:
	{
	  joint_angle=humanoid_f->Shoulder_Abduction_Adduction_ServoValue_Conversion(LEFT, servo_value);;
	  break;
	}
	case 41:
	{
	  joint_angle=humanoid_f->Elbow_Flexion_Extension_ServoValue_Conversion(LEFT, servo_value);
	  break;
	}
	case 31:
	{
	  joint_angle=humanoid_f->Torso_Rotation_ServoValue_Conversion(servo_value);
	  break;
	}
	case 32:
	{
	  joint_angle=humanoid_f->Torso_Flexion_Extension_ServoValue_Conversion(servo_value);
	  break;
	}
	case 33:
	{
	  joint_angle=humanoid_f->Torso_Lateral_Flexion_Extension_ServoValue_Conversion(servo_value);
	  break;
	}
	case 21:
	{
	  joint_angle=humanoid_f->Ankle_Inversion_Eversion_ServoValue_Conversion(LEFT,servo_value);
	  break;
	}
	case 22:
	{
	  joint_angle=humanoid_f->Ankle_Flexion_ServoValue_Conversion(LEFT,servo_value);
	  break;
	}
	case 23:
	{
	  joint_angle=humanoid_f->Knee_Flexion_ServoValue_Conversion(LEFT,servo_value);
	  break;
	}
	case 25:
	{
	  joint_angle=humanoid_f->Hip_Abduction_Hiperabduction_ServoValue_Conversion(LEFT,servo_value);
	  break;
	}
	case 26:
	{
	  joint_angle=humanoid_f->Hip_Flexion_ServoValue_Conversion(LEFT,servo_value);
	  break;
	}
	case 11:
	{
	  joint_angle=humanoid_f->Ankle_Inversion_Eversion_ServoValue_Conversion(RIGHT,servo_value);
	  break;
	}
	case 12:
	{
	  joint_angle=humanoid_f->Ankle_Flexion_ServoValue_Conversion(RIGHT,servo_value);
	  break;
	}
	case 13:
	{
	  joint_angle=humanoid_f->Knee_Flexion_ServoValue_Conversion(RIGHT,servo_value);
	  break;
	}
	case 15:
	{
	  joint_angle=humanoid_f->Hip_Abduction_Hiperabduction_ServoValue_Conversion(RIGHT,servo_value);
	  break;
	}
	case 16:
	{
	  joint_angle=humanoid_f->Hip_Flexion_ServoValue_Conversion(RIGHT,servo_value);
	  break;
	}
// 	default:
// 	{
// 	  joint_angle=0xFFFF;
// 	  break;
// 	}
      }
  delete humanoid_f;
  return joint_angle;
}

void SetArmSpeed(short int SIDE, short int speed, shared_vars_t*RobotVars)
{
  short int id;
  int ret;
  if(SIDE==RIGHT)
  {
    id=54;
    ret=RobotVars->servo->SetSpeedPosition(id,speed);
    id=53;
    ret=RobotVars->servo->SetSpeedPosition(id,speed);
    id=51;
    ret=RobotVars->servo->SetSpeedPosition(id,speed);
    return;
  }
  else if(SIDE==LEFT)
  {
    id=44;
    ret=RobotVars->servo->SetSpeedPosition(id,speed);
    id=43;
    ret=RobotVars->servo->SetSpeedPosition(id,speed);
    id=41;
    ret=RobotVars->servo->SetSpeedPosition(id,speed);
    return;
  }
  else
  {
    return;
  }
}

void SetLegSpeed(short int SIDE, short int speed, shared_vars_t*RobotVars)
{
  short int id;
  int ret;
  if(SIDE==RIGHT)
  {
    id=11;
    ret=RobotVars->servo->SetSpeedPosition(id,speed);
    id=12;
    ret=RobotVars->servo->SetSpeedPosition(id,speed);
    id=13;
    ret=RobotVars->servo->SetSpeedPosition(id,speed);
    id=15;
    ret=RobotVars->servo->SetSpeedPosition(id,speed);
    id=16;
    ret=RobotVars->servo->SetSpeedPosition(id,speed);
    return;
  }
  else if(SIDE==LEFT)
  {
    id=21;
    ret=RobotVars->servo->SetSpeedPosition(id,speed);
    id=22;
    ret=RobotVars->servo->SetSpeedPosition(id,speed);
    id=23;
    ret=RobotVars->servo->SetSpeedPosition(id,speed);
    id=25;
    ret=RobotVars->servo->SetSpeedPosition(id,speed);
    id=26;
    ret=RobotVars->servo->SetSpeedPosition(id,speed);
    return;
  }
  else
  {
    return;
  }
}

void MoveArmToCartesianPosition(double X, double Y, double Z, short int SIDE, shared_vars_t*RobotVars)
{
  short int id;
  int ret;
  double *thetas= new double[3];
  RobotVars->humanoid_f->Inv_Kin_3DOF_Arm(X , Y , Z , thetas);
  if(thetas[0]!=1000. && thetas[1]!=1000. && thetas[2]!=1000.)
  {
    if(SIDE==RIGHT)
    {
      id=54;
      ret=RobotVars->servo->SetPosition(id,RobotVars->humanoid_f->Shoulder_Flexion_Extension_Conversion(RIGHT,thetas[0]));
      id=53;
      ret=RobotVars->servo->SetPosition(id,RobotVars->humanoid_f->Shoulder_Abduction_Adduction_Conversion(RIGHT,thetas[1]));
      id=51;
      ret=RobotVars->servo->SetPosition(id,RobotVars->humanoid_f->Elbow_Flexion_Extension_Conversion(RIGHT,thetas[2]));
      //update joint values
      RobotVars->robot_kin_data.RightShoulderFlexion=thetas[0];
      RobotVars->robot_kin_data.RightShoulderAbduction=thetas[1];
      RobotVars->robot_kin_data.RightElbowFlexion=thetas[2];
    }
    else if(SIDE==LEFT)
    {
      id=44;
      ret=RobotVars->servo->SetPosition(id,RobotVars->humanoid_f->Shoulder_Flexion_Extension_Conversion(LEFT,thetas[0]));
      id=43;
      ret=RobotVars->servo->SetPosition(id,RobotVars->humanoid_f->Shoulder_Abduction_Adduction_Conversion(LEFT,thetas[1]));
      id=41;
      ret=RobotVars->servo->SetPosition(id,RobotVars->humanoid_f->Elbow_Flexion_Extension_Conversion(LEFT,thetas[2]));
      //update joint values
      RobotVars->robot_kin_data.LeftShoulderFlexion=thetas[0];
      RobotVars->robot_kin_data.LeftShoulderAbduction=thetas[1];
      RobotVars->robot_kin_data.LeftElbowFlexion=thetas[2];
    }
  }
  delete thetas;
}

void MoveDetachedLegToCartesianPosition(double X, double Y, double Z, double t4, short int SIDE, shared_vars_t*RobotVars)
{
  short int id;
  int ret;
  double *thetas= new double[3];
  RobotVars->humanoid_f->Inv_Kin_3DOF_Detached_Leg(X , Y , Z , thetas);
  
  
  
  if(thetas[0]!=1000. && thetas[1]!=1000. && thetas[2]!=1000.)
  {
    if(SIDE==RIGHT)
    {
      thetas[0] = thetas[0]*(double)SIDE;
      id=11;
      ret=RobotVars->servo->SetPosition(id,RobotVars->humanoid_f->Ankle_Inversion_Eversion_Conversion(RIGHT,thetas[0]));
      id=12;
      ret=RobotVars->servo->SetPosition(id,RobotVars->humanoid_f->Ankle_Flexion_Conversion(RIGHT,thetas[1]));
      id=13;
      ret=RobotVars->servo->SetPosition(id,RobotVars->humanoid_f->Knee_Conversion(RIGHT,thetas[2]));
      id=15;
      ret=RobotVars->servo->SetPosition(id,RobotVars->humanoid_f->Hip_Abduction_Hiperabduction_Conversion(RIGHT,t4));
      
      //update joint values
      RobotVars->robot_kin_data.RightAnkleInversion = 	thetas[0];
      RobotVars->robot_kin_data.RightAnkleFlexion = 	thetas[1];
      RobotVars->robot_kin_data.RightKneeFlexion = 	thetas[2];
      RobotVars->robot_kin_data.RightHipAbduction = 	t4;
    }
    else if(SIDE==LEFT)
    {
//       thetas[0] = thetas[0]*(double)SIDE;
      id=21;
      ret=RobotVars->servo->SetPosition(id,RobotVars->humanoid_f->Ankle_Inversion_Eversion_Conversion(LEFT,thetas[0]));
      id=22;
      ret=RobotVars->servo->SetPosition(id,RobotVars->humanoid_f->Ankle_Flexion_Conversion(LEFT,thetas[1]));
      id=23;
      ret=RobotVars->servo->SetPosition(id,RobotVars->humanoid_f->Knee_Conversion(LEFT,thetas[2]));
      id=25;
      ret=RobotVars->servo->SetPosition(id,RobotVars->humanoid_f->Hip_Abduction_Hiperabduction_Conversion(LEFT,t4));
      
      
      //update joint values
      RobotVars->robot_kin_data.LeftAnkleInversion = 	thetas[0];
      RobotVars->robot_kin_data.LeftAnkleFlexion = 	thetas[1];
      RobotVars->robot_kin_data.LeftKneeFlexion = 	thetas[2];
      RobotVars->robot_kin_data.LeftHipAbduction = 	t4;
    }
  }
  delete thetas;
}

void UpdateJointDataByID(short int id, double joint_angle, shared_vars_t*RobotVars)
{
  switch (id){
    case 61:
      RobotVars->robot_kin_data.HeadTilt=joint_angle;break;
    case 54:
      RobotVars->robot_kin_data.RightShoulderFlexion=joint_angle;break;
    case 53:
      RobotVars->robot_kin_data.RightShoulderAbduction=joint_angle;break;
    case 51:
      RobotVars->robot_kin_data.RightElbowFlexion=joint_angle;break;
    case 44:
      RobotVars->robot_kin_data.LeftShoulderFlexion=joint_angle;break;
    case 43:
      RobotVars->robot_kin_data.LeftShoulderAbduction=joint_angle;break;
    case 41:
      RobotVars->robot_kin_data.LeftElbowFlexion=joint_angle;break;
    case 31:
      RobotVars->robot_kin_data.TorsoRotation=joint_angle;break;
    case 32:
      RobotVars->robot_kin_data.TorsoFlexion=joint_angle;break;
    case 33:
      RobotVars->robot_kin_data.TorsoLateralFlexion=joint_angle;break;
    case 11:
      RobotVars->robot_kin_data.RightAnkleInversion=joint_angle;break;
    case 12:
      RobotVars->robot_kin_data.RightAnkleFlexion=joint_angle;break;
    case 13:
      RobotVars->robot_kin_data.RightKneeFlexion=joint_angle;break;
    case 15:
      RobotVars->robot_kin_data.RightHipAbduction=joint_angle;break;
    case 16:
      RobotVars->robot_kin_data.RightHipFlexion=joint_angle;break;
    case 21:
      RobotVars->robot_kin_data.LeftAnkleInversion=joint_angle;break;
    case 22:
      RobotVars->robot_kin_data.LeftAnkleFlexion=joint_angle;break;
    case 23:
      RobotVars->robot_kin_data.LeftKneeFlexion=joint_angle;break;
    case 25:
      RobotVars->robot_kin_data.LeftHipAbduction=joint_angle;break;
    case 26:
      RobotVars->robot_kin_data.LeftHipFlexion=joint_angle;break;
    case 1000:
    {
      //update all!
      const int speed=50;
      int position;
      double angle_position;
      const short int id_list[20]={61,54,53,51,44,43,41,31,32,33,21,22,23,25,26,11,12,13,15,16};
      for(int i=0;i<20;i++)
      {
	position=RobotVars->servo->SetSpeedPosition(id_list[i],speed);
	if(position>=606 && position<=2406)
	{
	  angle_position=ConvertServoValueByID(id_list[i],position);
	  UpdateJointDataByID(id_list[i],angle_position,RobotVars);
	}
	else
	{
	  UpdateJointDataByID(id_list[i],0.,RobotVars);
	}
      }
      break;
    }
    default:
      break;}
}

void SetArmAutomaticSpeeds(short int SIDE, shared_vars_t*RobotVars)
{
  short int id;
  int ret;
  short unsigned int speed;
  if(SIDE==RIGHT)
  {
    id=54;
    speed=RobotVars->servo->ConvertAngularSpeedToServoSpeed(RobotVars->parameters.arm_auto_angular_speed[0], HSR_5980SG);
    if(speed!=0xFFFF){
      if(speed!=RobotVars->parameters.arm_auto_speed[0])
      {
	ret=RobotVars->servo->SetSpeedPosition(id,speed);
	RobotVars->parameters.arm_auto_speed[0]=speed;
      }
    }
    else{
      ret=RobotVars->servo->SetPosition(id,
					RobotVars->humanoid_f->Shoulder_Flexion_Extension_Conversion(SIDE,
											  RobotVars->robot_kin_data.RightShoulderFlexion));
      RobotVars->parameters.arm_auto_speed[0]=0.;
    }
    id=53;
    speed=RobotVars->servo->ConvertAngularSpeedToServoSpeed(RobotVars->parameters.arm_auto_angular_speed[1], HSR_5498SG);
    if(speed!=0xFFFF){
      if(speed!=RobotVars->parameters.arm_auto_speed[1])
      {
	ret=RobotVars->servo->SetSpeedPosition(id,speed);
	RobotVars->parameters.arm_auto_speed[1]=speed;
      }
    }
    else{
      ret=RobotVars->servo->SetPosition(id,
					RobotVars->humanoid_f->Shoulder_Abduction_Adduction_Conversion(SIDE,
											    RobotVars->robot_kin_data.RightShoulderAbduction));
      RobotVars->parameters.arm_auto_speed[1]=0.;
    }
    id=51;
    speed=RobotVars->servo->ConvertAngularSpeedToServoSpeed(RobotVars->parameters.arm_auto_angular_speed[2], HSR_5498SG);
    if(speed!=0xFFFF){
      if(speed!=RobotVars->parameters.arm_auto_speed[2])
      {
	ret=RobotVars->servo->SetSpeedPosition(id,speed);
	RobotVars->parameters.arm_auto_speed[2]=speed;
      }
    }
    else{
      ret=RobotVars->servo->SetPosition(id,
					RobotVars->humanoid_f->Elbow_Flexion_Extension_Conversion(SIDE,
										       RobotVars->robot_kin_data.RightElbowFlexion));
      RobotVars->parameters.arm_auto_speed[2]=0.;
    }
    return;
  }
  else if(SIDE==LEFT)
  {
    id=44;
    speed=RobotVars->servo->ConvertAngularSpeedToServoSpeed(RobotVars->parameters.arm_auto_angular_speed[0], HSR_5980SG);
    if(speed!=0xFFFF){
      if(speed!=RobotVars->parameters.arm_auto_speed[0])
      {
	ret=RobotVars->servo->SetSpeedPosition(id,speed);
	RobotVars->parameters.arm_auto_speed[0]=speed;
      }
    }
    else{
      ret=RobotVars->servo->SetPosition(id,
					RobotVars->humanoid_f->Shoulder_Flexion_Extension_Conversion(SIDE,
											  RobotVars->robot_kin_data.LeftShoulderFlexion));
    }
    id=43;
    speed=RobotVars->servo->ConvertAngularSpeedToServoSpeed(RobotVars->parameters.arm_auto_angular_speed[1], HSR_5498SG);
    if(speed!=0xFFFF){
      if(speed!=RobotVars->parameters.arm_auto_speed[1])
      {
	ret=RobotVars->servo->SetSpeedPosition(id,speed);
	RobotVars->parameters.arm_auto_speed[1]=speed;
      }
    }
    else{
      ret=RobotVars->servo->SetPosition(id,
					RobotVars->humanoid_f->Shoulder_Abduction_Adduction_Conversion(SIDE,
											    RobotVars->robot_kin_data.LeftShoulderAbduction));
    }
    id=41;
    speed=RobotVars->servo->ConvertAngularSpeedToServoSpeed(RobotVars->parameters.arm_auto_angular_speed[2], HSR_5498SG);
    if(speed!=0xFFFF){
      if(speed!=RobotVars->parameters.arm_auto_speed[2]){
	cout<<"setting theta3 speed: "<<speed<<endl;fflush(stdout);
	ret=RobotVars->servo->SetSpeedPosition(id,speed);
	RobotVars->parameters.arm_auto_speed[2]=speed;}
      else{
	cout<<"theta 3 speed not changed"<<endl;fflush(stdout);}
    }
    else{
      ret=RobotVars->servo->SetPosition(id,
					RobotVars->humanoid_f->Elbow_Flexion_Extension_Conversion(SIDE,
										       RobotVars->robot_kin_data.LeftElbowFlexion));
      cout<<"setting theta3 position"<<endl;fflush(stdout);
    }
    return;
  }
  else
  {
    return;
  }
}

void CalculateDifferencialArmServoSpeed(double *avg_speed, shared_vars_t*RobotVars)
{
  //speeds
  static Eigen::Vector3d speeds_3d;
  speeds_3d<< avg_speed[0]/(RobotVars->parameters.pos_coord_scale), avg_speed[1]/(RobotVars->parameters.pos_coord_scale), avg_speed[2]/(RobotVars->parameters.pos_coord_scale);
  
  cout<<"************************************************************"<<endl<<endl<<"3D SPEEDS [mm/s]"<<endl<<speeds_3d<<endl;
  
  //calculate jacobian
  static Eigen::Matrix3d jacobian;
  static Eigen::Matrix3d jacobian_inverted;
  static Eigen::MatrixXd jacobian_alternate(3,2);
  static Eigen::MatrixXd jacobian_alternate_inverted(2,3);
  
  RobotVars->humanoid_f->CalculateArmJacobian(RobotVars->robot_kin_data.LeftShoulderFlexion,
				   RobotVars->robot_kin_data.LeftShoulderAbduction,
				   RobotVars->robot_kin_data.LeftElbowFlexion,
				   &jacobian);
  
  //test to discover max 3d speed from servo max speed = 340.5*PI/180.
//   Eigen::Vector3d maxspeed(340.5*3.1415926/180., 340.5*3.1415926/180., 340.5*3.1415926/180.);
//   cout<<"MAX SPEED AT MAX ANGULAR SPEED:"<<endl<<jacobian * maxspeed<<endl;
  
  cout<<"ANGLES READ"<<endl<<RobotVars->robot_kin_data.LeftShoulderFlexion<<endl<<RobotVars->robot_kin_data.LeftShoulderAbduction<<endl<<RobotVars->robot_kin_data.LeftElbowFlexion<<endl;
  
  cout<<"JACOBIAN MATRIX"<<endl<<jacobian<<endl;
  
  cout<<"JACOBIAN MATRIX DETERMINANT: "<<jacobian.determinant()<<endl;
  
  //inverted jacobian
  jacobian_inverted = jacobian.inverse();
  cout<<"INVERTED JACOBIAN MATRIX"<<endl<<jacobian_inverted<<endl;
  
  //calculate speed in deg/s
  //Eigen::Vector3d speeds_angular = ((jacobian_inverted * speeds_3d) * (180./arma::datum::pi));
  //Previous line commentd by V. Santos, 27-Mai-2013,23:30 and replaced by next:
  Eigen::Vector3d speeds_angular = ((jacobian_inverted * speeds_3d) * (180./M_PI));
  
  cout<<"FULL JACOBIAN ANGULAR SPEEDS [deg/s]"<<endl<<speeds_angular<<endl<<endl;
  
  if(!isnan(speeds_angular[0]) || !isnan(speeds_angular[1]) || !isnan(speeds_angular[2]))
  {
    if(fabs(speeds_angular[0])<350. && fabs(speeds_angular[1])<350. && fabs(speeds_angular[2])<350.)
    {
    cout<<endl<<"-----> NON SINGULARITY!!!!!!!!"<<endl;
    RobotVars->parameters.arm_auto_angular_speed[0]=speeds_angular(0);
    RobotVars->parameters.arm_auto_angular_speed[1]=speeds_angular(1);
    RobotVars->parameters.arm_auto_angular_speed[2]=speeds_angular(2);
    return;
    }
  }
  
  cout<<endl<<"-----> SINGULARITY!!!!!!!!"<<endl;
  // home position singularity OR forward streched arm singularity OR sidewards streched arm singularity
  
  RobotVars->humanoid_f->CalculateArmAlternateJacobian_No_Theta3_Singularity(RobotVars->robot_kin_data.LeftShoulderFlexion,
								  RobotVars->robot_kin_data.LeftShoulderAbduction,
								  RobotVars->robot_kin_data.LeftElbowFlexion,
								  &jacobian_alternate);
  
  
  //convert Eigen Matrix to Armadillo Matrix
  arma::mat jacobian_non_square(3,2);
  for(int i=0; i<3; i++)
  {
    for(int j=0; j<2; j++)
    {
      jacobian_non_square(i,j)=jacobian_alternate(i,j);
    }
  }
  //calculate pseudo-inverse
  arma::mat jacobian_non_square_pseudo_inverted = arma::pinv(jacobian_non_square);
  //convert Armadillo Matrix to Eigen Matrix
  for(int i=0; i<2; i++)
  {
    for(int j=0; j<3; j++)
    {
      jacobian_alternate_inverted(i,j)=jacobian_non_square_pseudo_inverted(i,j);
    }
  }
  
  cout<<"REDUCED JACOBIAN MATRIX"<<endl<<jacobian_alternate<<endl;

  cout<<"PSEUDO-INVERTED JACOBIAN MATRIX"<<endl<<jacobian_alternate_inverted<<endl;
  
  //calculate speed in deg/s
  //Eigen::Vector2d reduced_speeds_angular = ((jacobian_alternate_inverted * speeds_3d) * (180./arma::datum::pi));
  //Previous line commented by V. Santos, 27-Mai-2013,23:31 and replaced by next:
  Eigen::Vector2d reduced_speeds_angular = ((jacobian_alternate_inverted * speeds_3d) * (180./M_PI));
  
  cout<<"REDUCED JACOBIAN ANGULAR SPEEDS [deg/s]"<<endl<<reduced_speeds_angular<<endl<<endl<<"************************************************************"<<endl<<endl;
  
  if(!isnan(reduced_speeds_angular(0)) || !isnan(reduced_speeds_angular(1)))
  {
    if(fabs(reduced_speeds_angular[0])<350. && fabs(reduced_speeds_angular[1])<350.)
    {
      RobotVars->parameters.arm_auto_angular_speed[0]=reduced_speeds_angular(0);
      RobotVars->parameters.arm_auto_angular_speed[1]=reduced_speeds_angular(1);
      RobotVars->parameters.arm_auto_angular_speed[2]=15.;
      return;
    }
  }


//   RobotVars->parameters.arm_auto_angular_speed[0]=0.;
//   RobotVars->parameters.arm_auto_angular_speed[1]=0.;
//   RobotVars->parameters.arm_auto_angular_speed[2]=0.;
}

void StopRobotMovement(shared_vars_t*RobotVars)
{
  cout<<"Setting Kinematic Model STOP!"<<endl;
  static double curr_jnt_val;
  int ret;
  if(RobotVars->parameters.kinematic_model==1)
  {
    //RIGHT
    //update robot joint information
    //shoulder flexion
    curr_jnt_val = ConvertServoValueByID(54,
					RobotVars->servo->SetSpeedPosition(54,
									    RobotVars->parameters.arm_auto_speed[0]));
    ret=RobotVars->servo->SetPosition(54,RobotVars->humanoid_f->Shoulder_Flexion_Extension_Conversion(RIGHT,curr_jnt_val));
    
    //shoulder abduction
    curr_jnt_val = ConvertServoValueByID(53,
					RobotVars->servo->SetSpeedPosition(53,
									    RobotVars->parameters.arm_auto_speed[1]));
    ret=RobotVars->servo->SetPosition(53,RobotVars->humanoid_f->Shoulder_Abduction_Adduction_Conversion(RIGHT,curr_jnt_val));
									    
    //elbow flexion
    curr_jnt_val=ConvertServoValueByID(51,
				      RobotVars->servo->SetSpeedPosition(51,
									  RobotVars->parameters.arm_auto_speed[2]));
    ret=RobotVars->servo->SetPosition(51,RobotVars->humanoid_f->Elbow_Flexion_Extension_Conversion(RIGHT,curr_jnt_val));
  }
  else if(RobotVars->parameters.kinematic_model==2)
  {
    //LEFT
    //update robot joint information
    //shoulder flexion
    curr_jnt_val = ConvertServoValueByID(44,
					RobotVars->servo->SetSpeedPosition(44,
									    RobotVars->parameters.arm_auto_speed[0]));
    ret=RobotVars->servo->SetPosition(44,RobotVars->humanoid_f->Shoulder_Flexion_Extension_Conversion(LEFT,curr_jnt_val));
    
    //shoulder abduction
    curr_jnt_val = ConvertServoValueByID(43,
					RobotVars->servo->SetSpeedPosition(43,
									    RobotVars->parameters.arm_auto_speed[1]));
    ret=RobotVars->servo->SetPosition(43,RobotVars->humanoid_f->Shoulder_Abduction_Adduction_Conversion(LEFT,curr_jnt_val));

    //elbow flexion
    curr_jnt_val=ConvertServoValueByID(41,
				      RobotVars->servo->SetSpeedPosition(41,
									  RobotVars->parameters.arm_auto_speed[2]));
    ret=RobotVars->servo->SetPosition(41,RobotVars->humanoid_f->Elbow_Flexion_Extension_Conversion(LEFT,curr_jnt_val));
  }
  else if(RobotVars->parameters.kinematic_model==3)
  {
    //RIGHT
    //update robot joint information
    //shoulder flexion
    curr_jnt_val = ConvertServoValueByID(54,
					RobotVars->servo->SetSpeedPosition(54,
									    RobotVars->parameters.arm_auto_speed[0]));
    ret=RobotVars->servo->SetPosition(54,RobotVars->humanoid_f->Shoulder_Flexion_Extension_Conversion(RIGHT,curr_jnt_val));
    
    //shoulder abduction
    curr_jnt_val = ConvertServoValueByID(53,
					RobotVars->servo->SetSpeedPosition(53,
									    RobotVars->parameters.arm_auto_speed[1]));
    ret=RobotVars->servo->SetPosition(53,RobotVars->humanoid_f->Shoulder_Abduction_Adduction_Conversion(RIGHT,curr_jnt_val));
									    
    //elbow flexion
    curr_jnt_val=ConvertServoValueByID(51,
				      RobotVars->servo->SetSpeedPosition(51,
									  RobotVars->parameters.arm_auto_speed[2]));
    ret=RobotVars->servo->SetPosition(51,RobotVars->humanoid_f->Elbow_Flexion_Extension_Conversion(RIGHT,curr_jnt_val));

    //LEFT
    //update robot joint information
    //shoulder flexion
    curr_jnt_val = ConvertServoValueByID(44,
					RobotVars->servo->SetSpeedPosition(44,
									    RobotVars->parameters.arm_auto_speed[0]));
    ret=RobotVars->servo->SetPosition(44,RobotVars->humanoid_f->Shoulder_Flexion_Extension_Conversion(LEFT,curr_jnt_val));
    
    //shoulder abduction
    curr_jnt_val = ConvertServoValueByID(43,
					RobotVars->servo->SetSpeedPosition(43,
									    RobotVars->parameters.arm_auto_speed[1]));
    ret=RobotVars->servo->SetPosition(43,RobotVars->humanoid_f->Shoulder_Abduction_Adduction_Conversion(LEFT,curr_jnt_val));

    //elbow flexion
    curr_jnt_val=ConvertServoValueByID(41,
				      RobotVars->servo->SetSpeedPosition(41,
									  RobotVars->parameters.arm_auto_speed[2]));
    ret=RobotVars->servo->SetPosition(41,RobotVars->humanoid_f->Elbow_Flexion_Extension_Conversion(LEFT,curr_jnt_val));
  }
}

void ArmsDifferencialSpeedControl(double *avg_speed, shared_vars_t*RobotVars)
{
  static double curr_jnt_val;
  static double jnt_direction_right[3];
  static double jnt_direction_left[3];
  int ret;

  if(RobotVars->parameters.kinematic_model==1)
  {
    //RIGHT
    //update robot joint information
    //shoulder flexion
    curr_jnt_val = ConvertServoValueByID(54,
					 RobotVars->servo->SetSpeedPosition(54,
									    RobotVars->parameters.arm_auto_speed[0]));
    
    if(fabs(curr_jnt_val)<0.5 && fabs(curr_jnt_val)>0.)
      curr_jnt_val=0.;
    else if(fabs(curr_jnt_val)<90.5 && fabs(curr_jnt_val)>89.5)
      curr_jnt_val=90.;
    else if(fabs(curr_jnt_val)>179.5)
      curr_jnt_val=180.;
    
    RobotVars->robot_kin_data.RightShoulderFlexion = curr_jnt_val;
    //shoulder abduction
    curr_jnt_val = ConvertServoValueByID(53,
					 RobotVars->servo->SetSpeedPosition(53,
									    RobotVars->parameters.arm_auto_speed[1]));
    
    if(fabs(curr_jnt_val)<0.5 && fabs(curr_jnt_val)>0.)
      curr_jnt_val=0.;
    else if(fabs(curr_jnt_val)<90.5 && fabs(curr_jnt_val)>89.5)
      curr_jnt_val=90.;
    else if(fabs(curr_jnt_val)>179.5)
      curr_jnt_val=180.;
    
    RobotVars->robot_kin_data.RightShoulderAbduction = curr_jnt_val;
    //elbow flexion
    curr_jnt_val=ConvertServoValueByID(51,
				       RobotVars->servo->SetSpeedPosition(51,
									  RobotVars->parameters.arm_auto_speed[2]));
    
    if(fabs(curr_jnt_val)<0.5 && fabs(curr_jnt_val)>0.)
      curr_jnt_val=0.;
    else if(fabs(curr_jnt_val)<90.5 && fabs(curr_jnt_val)>89.5)
      curr_jnt_val=90.;
    else if(fabs(curr_jnt_val)>179.5)
      curr_jnt_val=180.;
    
    RobotVars->robot_kin_data.RightElbowFlexion = curr_jnt_val;
    
    //calculate differencial speeds
    avg_speed[2]=-avg_speed[2];
    CalculateDifferencialArmServoSpeed(avg_speed,RobotVars);
    
    //update speeds
    SetArmAutomaticSpeeds(RIGHT, RobotVars);
    
    //send servos to max or min if required
    if(RobotVars->parameters.arm_auto_speed[0]!=0.)
    {
      //theta1
      if(get_sign((double) RobotVars->parameters.arm_auto_angular_speed[0])!=jnt_direction_right[0])
      {
	jnt_direction_right[0]=get_sign((double) RobotVars->parameters.arm_auto_angular_speed[0]);
	if(jnt_direction_right[0]>0)
	  ret=RobotVars->servo->SetPosition(54,RobotVars->humanoid_f->Shoulder_Flexion_Extension_Conversion(RIGHT,135.));
	else
	  ret=RobotVars->servo->SetPosition(54,RobotVars->humanoid_f->Shoulder_Flexion_Extension_Conversion(RIGHT,-45));
      }
    }
    if(RobotVars->parameters.arm_auto_speed[1]!=0.)
    {
      //theta2
      if(get_sign((double) RobotVars->parameters.arm_auto_angular_speed[1])!=jnt_direction_right[1])
      {
	jnt_direction_right[1]=get_sign((double) RobotVars->parameters.arm_auto_angular_speed[1]);
	if(jnt_direction_right[1]>0)
	  ret=RobotVars->servo->SetPosition(53,RobotVars->humanoid_f->Shoulder_Abduction_Adduction_Conversion(RIGHT,180.));
	else
	  ret=RobotVars->servo->SetPosition(53,RobotVars->humanoid_f->Shoulder_Abduction_Adduction_Conversion(RIGHT,0.));
      }
    }
    if(RobotVars->parameters.arm_auto_speed[2]!=0.)
    {
      //theta3
      if(get_sign((double) RobotVars->parameters.arm_auto_angular_speed[2])!=jnt_direction_right[2])
      {
	jnt_direction_right[2]=get_sign((double) RobotVars->parameters.arm_auto_angular_speed[2]);
	if(jnt_direction_right[2]>0)
	  ret=RobotVars->servo->SetPosition(51,RobotVars->humanoid_f->Elbow_Flexion_Extension_Conversion(RIGHT,120.));
	else
	  ret=RobotVars->servo->SetPosition(51,RobotVars->humanoid_f->Elbow_Flexion_Extension_Conversion(RIGHT,0.));
      }
    }
  }
  else if(RobotVars->parameters.kinematic_model==2)
  {
    //LEFT
    //update robot joint information
    //shoulder flexion
    curr_jnt_val = ConvertServoValueByID(44,
					 RobotVars->servo->SetSpeedPosition(44,
									    RobotVars->parameters.arm_auto_speed[0]));
									    
    if(fabs(curr_jnt_val)<0.5 && fabs(curr_jnt_val)>0.)
      curr_jnt_val=0.;
    else if(fabs(curr_jnt_val)<90.5 && fabs(curr_jnt_val)>89.5)
      curr_jnt_val=90.;
    else if(fabs(curr_jnt_val)>179.5 && fabs(curr_jnt_val)<180.5)
      curr_jnt_val=180.;
    else if(fabs(curr_jnt_val)>180.5)
      cout<<"Servo responce error..."<<endl;
    
    RobotVars->robot_kin_data.LeftShoulderFlexion = curr_jnt_val;
    //shoulder abduction
    curr_jnt_val = ConvertServoValueByID(43,
					 RobotVars->servo->SetSpeedPosition(43,
									    RobotVars->parameters.arm_auto_speed[1]));
									    
    if(fabs(curr_jnt_val)<0.5 && fabs(curr_jnt_val)>0.)
      curr_jnt_val=0.;
    else if(fabs(curr_jnt_val)<90.5 && fabs(curr_jnt_val)>89.5)
      curr_jnt_val=90.;
    else if(fabs(curr_jnt_val)>179.5 && fabs(curr_jnt_val)<180.5)
      curr_jnt_val=180.;
    else if(fabs(curr_jnt_val)>180.5)
      cout<<"Servo responce error..."<<endl;
    
    RobotVars->robot_kin_data.LeftShoulderAbduction = curr_jnt_val;
    //elbow flexion
    curr_jnt_val=ConvertServoValueByID(41,
				       RobotVars->servo->SetSpeedPosition(41,
									  RobotVars->parameters.arm_auto_speed[2]));
									  
    if(fabs(curr_jnt_val)<0.5 && fabs(curr_jnt_val)>0.)
      curr_jnt_val=0.;
    else if(fabs(curr_jnt_val)<90.5 && fabs(curr_jnt_val)>89.5)
      curr_jnt_val=90.;
    else if(fabs(curr_jnt_val)>179.5 && fabs(curr_jnt_val)<180.5)
      curr_jnt_val=180.;
    else if(fabs(curr_jnt_val)>180.5)
      cout<<"Servo responce error..."<<endl;
    
    
    RobotVars->robot_kin_data.LeftElbowFlexion = curr_jnt_val;
    
    //calculate differencial speeds
    CalculateDifferencialArmServoSpeed(avg_speed,RobotVars);
    
    //update speeds
    SetArmAutomaticSpeeds(LEFT, RobotVars);
    
    //send servos to max or min if required //
    //theta1
    if(get_sign((double) RobotVars->parameters.arm_auto_angular_speed[0])!=jnt_direction_left[0])
    {
      jnt_direction_left[0]=get_sign((double) RobotVars->parameters.arm_auto_angular_speed[0]);
      if(jnt_direction_left[0]>0)
	ret=RobotVars->servo->SetPosition(44,RobotVars->humanoid_f->Shoulder_Flexion_Extension_Conversion(LEFT,135.));
      else
	ret=RobotVars->servo->SetPosition(44,RobotVars->humanoid_f->Shoulder_Flexion_Extension_Conversion(LEFT,-45));
    }
    //theta2
    if(get_sign((double) RobotVars->parameters.arm_auto_angular_speed[1])!=jnt_direction_left[1])
    {
      jnt_direction_left[1]=get_sign((double) RobotVars->parameters.arm_auto_angular_speed[1]);
      if(jnt_direction_left[1]>0)
	ret=RobotVars->servo->SetPosition(43,RobotVars->humanoid_f->Shoulder_Abduction_Adduction_Conversion(LEFT,180.));
      else
	ret=RobotVars->servo->SetPosition(43,RobotVars->humanoid_f->Shoulder_Abduction_Adduction_Conversion(LEFT,0.));;
    }
    //theta3
    if(get_sign((double) RobotVars->parameters.arm_auto_angular_speed[2])!=jnt_direction_left[2])
    {
      jnt_direction_left[2]=get_sign((double) RobotVars->parameters.arm_auto_angular_speed[2]);
      if(jnt_direction_left[2]>0)
	ret=RobotVars->servo->SetPosition(41,RobotVars->humanoid_f->Elbow_Flexion_Extension_Conversion(LEFT,120.));
      else
	ret=RobotVars->servo->SetPosition(41,RobotVars->humanoid_f->Elbow_Flexion_Extension_Conversion(LEFT,0.));
    }
  }
  else if(RobotVars->parameters.kinematic_model==3)
  {
    //update robot joint information
    //shoulder flexion
    curr_jnt_val = ConvertServoValueByID(44,
					 RobotVars->servo->SetSpeedPosition(44,
									    RobotVars->parameters.arm_auto_speed[0]));
									    
    if(fabs(curr_jnt_val)<0.5 && fabs(curr_jnt_val)>0.)
      curr_jnt_val=0.;
    else if(fabs(curr_jnt_val)<90.5 && fabs(curr_jnt_val)>89.5)
      curr_jnt_val=90.;
    else if(fabs(curr_jnt_val)>179.5)
      curr_jnt_val=180.;
    
    RobotVars->robot_kin_data.LeftShoulderFlexion = curr_jnt_val;
    //shoulder abduction
    curr_jnt_val = ConvertServoValueByID(43,
					 RobotVars->servo->SetSpeedPosition(43,
									    RobotVars->parameters.arm_auto_speed[1]));
									    
    if(fabs(curr_jnt_val)<0.5 && fabs(curr_jnt_val)>0.)
      curr_jnt_val=0.;
    else if(fabs(curr_jnt_val)<90.5 && fabs(curr_jnt_val)>89.5)
      curr_jnt_val=90.;
    else if(fabs(curr_jnt_val)>179.5)
      curr_jnt_val=180.;
    
    RobotVars->robot_kin_data.LeftShoulderAbduction = curr_jnt_val;
    //elbow flexion
    curr_jnt_val=ConvertServoValueByID(41,
				       RobotVars->servo->SetSpeedPosition(41,
									  RobotVars->parameters.arm_auto_speed[2]));
									  
    if(fabs(curr_jnt_val)<0.5 && fabs(curr_jnt_val)>0.)
      curr_jnt_val=0.;
    else if(fabs(curr_jnt_val)<90.5 && fabs(curr_jnt_val)>89.5)
      curr_jnt_val=90.;
    else if(fabs(curr_jnt_val)>179.5)
      curr_jnt_val=180.;
    
    RobotVars->robot_kin_data.LeftElbowFlexion = curr_jnt_val;
    
    //calculate differencial speeds
    CalculateDifferencialArmServoSpeed(avg_speed,RobotVars);
    //update speeds
    SetArmAutomaticSpeeds(LEFT, RobotVars);
    
    //LEFT
    //send servos to max or min if required
    //theta1
    if(get_sign((double) RobotVars->parameters.arm_auto_angular_speed[0])!=jnt_direction_left[0])
    {
      jnt_direction_left[0]=get_sign((double) RobotVars->parameters.arm_auto_angular_speed[0]);
      if(jnt_direction_left[0]>0)
	ret=RobotVars->servo->SetPosition(44,RobotVars->humanoid_f->Shoulder_Flexion_Extension_Conversion(LEFT,135.));
      else
	ret=RobotVars->servo->SetPosition(44,RobotVars->humanoid_f->Shoulder_Flexion_Extension_Conversion(LEFT,-45));
    }
    //theta2
    if(get_sign((double) RobotVars->parameters.arm_auto_angular_speed[1])!=jnt_direction_left[1])
    {
      jnt_direction_left[1]=get_sign((double) RobotVars->parameters.arm_auto_angular_speed[1]);
      if(jnt_direction_left[1]>0)
	ret=RobotVars->servo->SetPosition(43,RobotVars->humanoid_f->Shoulder_Abduction_Adduction_Conversion(LEFT,180.));
      else
	ret=RobotVars->servo->SetPosition(43,RobotVars->humanoid_f->Shoulder_Abduction_Adduction_Conversion(LEFT,0.));
    }
    //theta3
    if(get_sign((double) RobotVars->parameters.arm_auto_angular_speed[2])!=jnt_direction_left[2])
    {
      jnt_direction_left[2]=get_sign((double) RobotVars->parameters.arm_auto_angular_speed[2]);
      if(jnt_direction_left[2]>0)
	ret=RobotVars->servo->SetPosition(41,RobotVars->humanoid_f->Elbow_Flexion_Extension_Conversion(LEFT,120.));
      else
	ret=RobotVars->servo->SetPosition(41,RobotVars->humanoid_f->Elbow_Flexion_Extension_Conversion(LEFT,0.));
    }
    
    //RIGHT
    //update robot joint information
    //shoulder flexion
    curr_jnt_val = ConvertServoValueByID(54,
					 RobotVars->servo->SetSpeedPosition(54,
									    RobotVars->parameters.arm_auto_speed[0]));
									    
    if(fabs(curr_jnt_val)<0.5 && fabs(curr_jnt_val)>0.)
      curr_jnt_val=0.;
    else if(fabs(curr_jnt_val)<90.5 && fabs(curr_jnt_val)>89.5)
      curr_jnt_val=90.;
    else if(fabs(curr_jnt_val)>179.5)
      curr_jnt_val=180.;
    
    RobotVars->robot_kin_data.RightShoulderFlexion = curr_jnt_val;
    //shoulder abduction
    curr_jnt_val = ConvertServoValueByID(53,
					 RobotVars->servo->SetSpeedPosition(53,
									    RobotVars->parameters.arm_auto_speed[1]));
									    
    if(fabs(curr_jnt_val)<0.5 && fabs(curr_jnt_val)>0.)
      curr_jnt_val=0.;
    else if(fabs(curr_jnt_val)<90.5 && fabs(curr_jnt_val)>89.5)
      curr_jnt_val=90.;
    else if(fabs(curr_jnt_val)>179.5)
      curr_jnt_val=180.;
    
    RobotVars->robot_kin_data.RightShoulderAbduction = curr_jnt_val;
    //elbow flexion
    curr_jnt_val=ConvertServoValueByID(51,
				       RobotVars->servo->SetSpeedPosition(51,
									  RobotVars->parameters.arm_auto_speed[2]));
									  
    if(fabs(curr_jnt_val)<0.5 && fabs(curr_jnt_val)>0.)
      curr_jnt_val=0.;
    else if(fabs(curr_jnt_val)<90.5 && fabs(curr_jnt_val)>89.5)
      curr_jnt_val=90.;
    else if(fabs(curr_jnt_val)>179.5)
      curr_jnt_val=180.;
    
    RobotVars->robot_kin_data.RightElbowFlexion = curr_jnt_val;
    
    //calculate differencial speeds
    avg_speed[2]=-avg_speed[2];
    CalculateDifferencialArmServoSpeed(avg_speed,RobotVars);
    
    //update speeds
    SetArmAutomaticSpeeds(RIGHT, RobotVars);
    
    //RIGHT
    //send servos to max or min if required
    //theta1
    if(get_sign((double) RobotVars->parameters.arm_auto_angular_speed[0])!=jnt_direction_right[0])
    {
      jnt_direction_right[0]=get_sign((double) RobotVars->parameters.arm_auto_angular_speed[0]);
      if(jnt_direction_right[0]>0)
	ret=RobotVars->servo->SetPosition(54,RobotVars->humanoid_f->Shoulder_Flexion_Extension_Conversion(RIGHT,135.));
      else
	ret=RobotVars->servo->SetPosition(54,RobotVars->humanoid_f->Shoulder_Flexion_Extension_Conversion(RIGHT,-45));
    }
    //theta2
    if(get_sign((double) RobotVars->parameters.arm_auto_angular_speed[1])!=jnt_direction_right[1])
    {
      jnt_direction_right[1]=get_sign((double) RobotVars->parameters.arm_auto_angular_speed[1]);
      if(jnt_direction_right[1]>0)
	ret=RobotVars->servo->SetPosition(53,RobotVars->humanoid_f->Shoulder_Abduction_Adduction_Conversion(RIGHT,180.));
      else
	ret=RobotVars->servo->SetPosition(53,RobotVars->humanoid_f->Shoulder_Abduction_Adduction_Conversion(RIGHT,0.));
    }
    //theta3
    if(get_sign((double) RobotVars->parameters.arm_auto_angular_speed[2])!=jnt_direction_right[2])
    {
      jnt_direction_right[2]=get_sign((double) RobotVars->parameters.arm_auto_angular_speed[2]);
      if(jnt_direction_right[2]>0)
	ret=RobotVars->servo->SetPosition(51,RobotVars->humanoid_f->Elbow_Flexion_Extension_Conversion(RIGHT,120.));
      else
	ret=RobotVars->servo->SetPosition(51,RobotVars->humanoid_f->Elbow_Flexion_Extension_Conversion(RIGHT,0.));
    }
  }
}


void DemoUserPath_WritePoints(shared_vars_t*RobotVars)
{
  static string file_path = ros::package::getPath("phua_haptic") + "/pnts/" + POINTS_FILE_NAME_STRING;
  static ofstream pnts_file;
  
  pnts_file.open(file_path.c_str(),ios::app);
  if(pnts_file)
  {
    // add kin chain to file
    if(RobotVars->parameters.kinematic_model==1)
    {
      pnts_file << RIGHT_ARM_SAVE_STRING;
    }
    else if(RobotVars->parameters.kinematic_model==2)
    {
      pnts_file << LEFT_ARM_SAVE_STRING;
    }
    else if(RobotVars->parameters.kinematic_model==3)
    {
      pnts_file << BOTH_ARMS_SAVE_STRING;
    }
    else if(RobotVars->parameters.kinematic_model==4)
    {
      pnts_file << LEFT_D_LEG_SAVE_STRING;
    }
    else if(RobotVars->parameters.kinematic_model==5)
    {
      pnts_file << RIGHT_D_LEG_SAVE_STRING;
    }
    //add point
    if(RobotVars->parameters.kinematic_model==1)
    {
      //add lines
      pnts_file << RobotVars->robot_kin_data.X_arm_end_right << " "
      << RobotVars->robot_kin_data.Y_arm_end_right << " "
      << RobotVars->robot_kin_data.Z_arm_end_right << endl;
    }
    else if(RobotVars->parameters.kinematic_model==2)
    {
      //add lines
      pnts_file <<RobotVars->robot_kin_data.X_arm_end_left << " "
      << RobotVars->robot_kin_data.Y_arm_end_left << " "
      << RobotVars->robot_kin_data.Z_arm_end_left << endl;
    }
    else if(RobotVars->parameters.kinematic_model==3)
    {
      //add lines
      pnts_file <<RobotVars->robot_kin_data.X_arm_end_left << " "
      << RobotVars->robot_kin_data.Y_arm_end_left << " "
      << RobotVars->robot_kin_data.Z_arm_end_left << endl;
    }
    else if(RobotVars->parameters.kinematic_model==4)
    {
      //add lines
      pnts_file << RobotVars->robot_kin_data.detached_leg_pos_left[0] << " "
      << RobotVars->robot_kin_data.detached_leg_pos_left[1] << " "
      << RobotVars->robot_kin_data.detached_leg_pos_left[2] << endl;
    }
    else if(RobotVars->parameters.kinematic_model==5)
    {
      //add lines
      pnts_file << RobotVars->robot_kin_data.detached_leg_pos_right[0] << " "
      << RobotVars->robot_kin_data.detached_leg_pos_right[1] << " "
      << RobotVars->robot_kin_data.detached_leg_pos_right[2] << endl;
    }
    //close file
    pnts_file.close();
  }
  else
  {
    perror("Points file error");
  }
    
}

void DemoUserPath_ReadPoints(shared_vars_t*RobotVars)
{
  static string file_path = ros::package::getPath("phua_haptic") + "/pnts/" + POINTS_FILE_NAME_STRING;
  static ifstream pnts_file;
  string STRING;
  
  int line = 1;
  
  pnts_file.open(file_path.c_str());
  
  if(pnts_file)
  {
    //clear vector to relocate values
    RobotVars->haptics_data.user_path_X_pnts.clear();
    RobotVars->haptics_data.user_path_Y_pnts.clear();
    RobotVars->haptics_data.user_path_Z_pnts.clear();
    
    while(!pnts_file.eof()) // To get you all the lines.
    {
      getline(pnts_file,STRING);
      string::iterator it1,it2;
      it1 = STRING.begin();
      // extract points
      if(STRING.compare(RIGHT_ARM_SAVE_STRING) > 1 && RobotVars->parameters.kinematic_model==1)
      {
	it2 = STRING.begin() + strlen(RIGHT_ARM_SAVE_STRING);
	STRING.erase(it1,it2);
// 	cout<<STRING<<endl;
	std::istringstream iss(STRING);
	double value;
	int i = 0;
	while ( iss >> value )
	{
	  if(i == 0)
	    RobotVars->haptics_data.user_path_X_pnts.push_back(value);
	  else if(i ==1)
	    RobotVars->haptics_data.user_path_Y_pnts.push_back(value);
	  else if(i == 2)
	    RobotVars->haptics_data.user_path_Z_pnts.push_back(value);
	  i++;
	}
      }
      else if(STRING.compare(LEFT_ARM_SAVE_STRING) > 1 && RobotVars->parameters.kinematic_model==2)
      {
	it2 = STRING.begin() + strlen(LEFT_ARM_SAVE_STRING);
	STRING.erase(it1,it2);
// 	cout<<STRING<<endl;
	std::istringstream iss(STRING);
	double value;
	int i = 0;
	while ( iss >> value )
	{
	  if(i == 0)
	    RobotVars->haptics_data.user_path_X_pnts.push_back(value);
	  else if(i ==1)
	    RobotVars->haptics_data.user_path_Y_pnts.push_back(value);
	  else if(i == 2)
	    RobotVars->haptics_data.user_path_Z_pnts.push_back(value);
	  i++;
	}
      }
      else if(STRING.compare(BOTH_ARMS_SAVE_STRING) > 1 && RobotVars->parameters.kinematic_model==3)
      {
	it2 = STRING.begin() + strlen(BOTH_ARMS_SAVE_STRING);
	STRING.erase(it1,it2);
// 	cout<<STRING<<endl;
	std::istringstream iss(STRING);
	double value;
	int i = 0;
	while ( iss >> value )
	{
	  if(i == 0)
	    RobotVars->haptics_data.user_path_X_pnts.push_back(value);
	  else if(i ==1)
	    RobotVars->haptics_data.user_path_Y_pnts.push_back(value);
	  else if(i == 2)
	    RobotVars->haptics_data.user_path_Z_pnts.push_back(value);
	  i++;
	}
      }
      else if(STRING.compare(RIGHT_D_LEG_SAVE_STRING) > 1 && RobotVars->parameters.kinematic_model==5)
      {
	it2 = STRING.begin() + strlen(RIGHT_D_LEG_SAVE_STRING);
	STRING.erase(it1,it2);
// 	cout<<STRING<<endl;
	std::istringstream iss(STRING);
	double value;
	int i = 0;
	while ( iss >> value )
	{
	  if(i == 0)
	    RobotVars->haptics_data.user_path_X_pnts.push_back(value);
	  else if(i ==1)
	    RobotVars->haptics_data.user_path_Y_pnts.push_back(value);
	  else if(i == 2)
	    RobotVars->haptics_data.user_path_Z_pnts.push_back(value);
	  i++;
	}
      }
      else if(STRING.compare(LEFT_D_LEG_SAVE_STRING) > 1 && RobotVars->parameters.kinematic_model==4)
      {
	it2 = STRING.begin() + strlen(LEFT_D_LEG_SAVE_STRING);
	STRING.erase(it1,it2);
// 	cout<<STRING<<endl;
	std::istringstream iss(STRING);
	double value;
	int i = 0;
	while ( iss >> value )
	{
	  if(i == 0)
	    RobotVars->haptics_data.user_path_X_pnts.push_back(value);
	  else if(i ==1)
	    RobotVars->haptics_data.user_path_Y_pnts.push_back(value);
	  else if(i == 2)
	    RobotVars->haptics_data.user_path_Z_pnts.push_back(value);
	  i++;
	}
      }
      else
      {
	if(STRING.size()>0)
	{
	  cout << "Points file error @ line " << line << ".";
	  RobotVars->haptics_data.demo_user_path_run_start = FALSE;
	  pnts_file.close();
	  return;
	}
	
      }
      line++;
    }
    pnts_file.close();
    if(RobotVars->haptics_data.user_path_X_pnts.size()>0 && RobotVars->haptics_data.user_path_Y_pnts.size()>0 && RobotVars->haptics_data.user_path_Z_pnts.size()>0)
    {
      //run can start
      RobotVars->haptics_data.demo_user_path_run_start = TRUE;
    }
    else
    {
      cout << "NO POINTS READ!" << endl;
      RobotVars->haptics_data.demo_user_path_run_start = FALSE;
    }
  }
  else
  {
    perror("Points file error");
    RobotVars->haptics_data.demo_user_path_run_start = FALSE;
  }
}

void PathFollowingExecute(shared_vars_t*RobotVars)
{
  static unsigned int incr = 0;
  static double next_point[3];
  static double curr_point[3];
  
  if((RobotVars->haptics_data.user_path_X_pnts.size()>0 && RobotVars->haptics_data.user_path_Y_pnts.size()>0 && RobotVars->haptics_data.user_path_Z_pnts.size()>0)
    &&
    RobotVars->haptics_data.chosen_demo == 3
    &&
    RobotVars->haptics_data.demo_user_path_run_start)
  {
    next_point[0] = RobotVars->haptics_data.user_path_X_pnts[incr];
    next_point[1] = RobotVars->haptics_data.user_path_Y_pnts[incr];
    next_point[2] = RobotVars->haptics_data.user_path_Z_pnts[incr];
    
    double margin = (double)PATH_FOLLOWING_POSITION_ERROR;
    
    //update robot information
    UpdateJointDataByID(1000, 0., RobotVars);
    
    //retrieve current end-effector position
    if(RobotVars->parameters.kinematic_model==1)
    {
      //right arm
      UpdateArmsDirKinData(RobotVars);
      curr_point[0] = RobotVars->robot_kin_data.X_arm_end_right;
      curr_point[1] = RobotVars->robot_kin_data.Y_arm_end_right;
      curr_point[2] = RobotVars->robot_kin_data.Z_arm_end_right;
    }
    else if(RobotVars->parameters.kinematic_model==2)
    {
      //left arm
      UpdateArmsDirKinData(RobotVars);
      curr_point[0] = RobotVars->robot_kin_data.X_arm_end_left;
      curr_point[1] = RobotVars->robot_kin_data.Y_arm_end_left;
      curr_point[2] = RobotVars->robot_kin_data.Z_arm_end_left;
    }
    else if(RobotVars->parameters.kinematic_model==3)
    {
      //both arms
      UpdateArmsDirKinData(RobotVars);
      curr_point[0] = RobotVars->robot_kin_data.X_arm_end_right;
      curr_point[1] = RobotVars->robot_kin_data.Y_arm_end_right;
      curr_point[2] = RobotVars->robot_kin_data.Z_arm_end_right;
    }
    else if(RobotVars->parameters.kinematic_model==4)
    {
      //left detached leg
      UpdateDetachedLegsDirKinData(RobotVars);
      memcpy(&curr_point, RobotVars->robot_kin_data.detached_leg_pos_left, sizeof(RobotVars->robot_kin_data.detached_leg_pos_left));
    }
    else if(RobotVars->parameters.kinematic_model==5)
    {
      //right detached leg
      UpdateDetachedLegsDirKinData(RobotVars);
      memcpy(&curr_point, RobotVars->robot_kin_data.detached_leg_pos_right, sizeof(RobotVars->robot_kin_data.detached_leg_pos_right));
    }
    
    //check if the point has been reached
    if(IsWithinRange(curr_point[0], (next_point[0] - margin), (next_point[0] + margin))
      &&
      IsWithinRange(curr_point[1], (next_point[1] - margin), (next_point[1] + margin))
      &&
      IsWithinRange(curr_point[2], (next_point[2] - margin), (next_point[2] + margin)))
    {
      //point has been reached
      cout<<"-> Reached point "<<incr+1<<": X["<<curr_point[0]<<"] Y["<<curr_point[1]<<"] Z["<<curr_point[2]<<"]"<<endl;
      //increment
      incr++;
    }//end of point reached
    else
    {
      cout<<"-> Sending next point: X["<<next_point[0]<<"] Y["<<next_point[1]<<"] Z["<<next_point[2]<<"]"<<endl;
      //send command to move to point
      if(RobotVars->parameters.kinematic_model==1)
      {
	MoveArmToCartesianPosition(next_point[0],
					  next_point[1],
					  next_point[2],
					  RIGHT,
					  RobotVars);
      }
      else if(RobotVars->parameters.kinematic_model==2)
      {
	MoveArmToCartesianPosition(next_point[0],
					  next_point[1],
					  next_point[2],
					  LEFT,
					  RobotVars);
      }
      else if(RobotVars->parameters.kinematic_model==3)
      {
	MoveArmToCartesianPosition(next_point[0],
					  next_point[1],
					  next_point[2],
					  RIGHT,
					  RobotVars);
	MoveArmToCartesianPosition(next_point[0],
					  next_point[1],
					  next_point[2],
					  LEFT,
					  RobotVars);
      }
      else if(RobotVars->parameters.kinematic_model==4)
      {
	MoveDetachedLegToCartesianPosition(next_point[0],
					  next_point[1],
					  next_point[2],
					  0.,
					  LEFT,
					  RobotVars);
      }
      else if(RobotVars->parameters.kinematic_model==5)
      {
	MoveDetachedLegToCartesianPosition(next_point[0],
					  next_point[1],
					  next_point[2],
					  0.,
					  RIGHT,
					  RobotVars);
      }
    }//end of send command
    
    cout<<"INCR: "<<incr<<endl;
    cout<<"SIZE: "<<RobotVars->haptics_data.user_path_X_pnts.size()<<endl;
      cout<<"CHECK INCR: "<<(incr > RobotVars->haptics_data.user_path_X_pnts.size() ? "TRUE" : "FALSE" )<<endl;
    
    if(incr >= RobotVars->haptics_data.user_path_X_pnts.size())
    {
      //last point was already run
      if(RobotVars->haptics_data.demo_user_path_is_run_once)
      {
	// run once
	incr = 0;
	RobotVars->haptics_data.demo_user_path_run_start = FALSE;
	cout<<"-> End!"<<endl;
// 	if(RobotVars->parameters.kinematic_model==1)
// 	{
// 	  //right arm
// 	  MoveArmToCartesianPosition(0.,0.,-229.,
// 					  RIGHT,
// 					  RobotVars);
// 	}
// 	else if(RobotVars->parameters.kinematic_model==2)
// 	{
// 	  //left arm
// 	  MoveArmToCartesianPosition(0.,0.,-229.,
// 					  LEFT,
// 					  RobotVars);
// 	}
// 	else if(RobotVars->parameters.kinematic_model==3)
// 	{
// 	  //both arms
// 	  MoveArmToCartesianPosition(0.,0.,-229.,
// 					  RIGHT,
// 					  RobotVars);
// 	  MoveArmToCartesianPosition(0.,0.,-229.,
// 					  LEFT,
// 					  RobotVars);
// 	}
// 	else if(RobotVars->parameters.kinematic_model==4)
// 	{
// 	  //left detached leg
// 	  MoveDetachedLegToCartesianPosition(0.,0.,332.,
// 					  0.,
// 					  LEFT,
// 					  RobotVars);
// 	}
// 	else if(RobotVars->parameters.kinematic_model==5)
// 	{
// 	  //right detached leg
// 	  MoveDetachedLegToCartesianPosition(0.,0.,332.,
// 					  0.,
// 					  RIGHT,
// 					  RobotVars);
// 	}
	SetRobotHomePosition(RobotVars);
	return;
      }
      else
      {
	// run in loop
	incr = 0;
	cout<<"-> Restarting routine."<<endl;
	return;
      }
    }
  }
  else
  {
    incr = 0;
    return;
  }
}

void UpdateKinematicModelDirKin(shared_vars_t*RobotVars)
{
  //retrieve current end-effector position
  if(RobotVars->parameters.kinematic_model==1)
  {
    //right arm
    UpdateArmsDirKinData(RobotVars);
  }
  else if(RobotVars->parameters.kinematic_model==2)
  {
    //left arm
    UpdateArmsDirKinData(RobotVars);
  }
  else if(RobotVars->parameters.kinematic_model==3)
  {
    //both arms
    UpdateArmsDirKinData(RobotVars);
  }
  else if(RobotVars->parameters.kinematic_model==4)
  {
    //left detached leg
    UpdateDetachedLegsDirKinData(RobotVars);
  }
  else if(RobotVars->parameters.kinematic_model==5)
  {
    //right detached leg
    UpdateDetachedLegsDirKinData(RobotVars);
  }
  else
  {
    UpdateArmsDirKinData(RobotVars);
    UpdateDetachedLegsDirKinData(RobotVars);
  }
}
