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
 * @file servohumanoid.cpp
 * @author Emílio Estrelinha nº 38637 (emilio.estrelinha@ua.pt)
 * @brief Class develop to control PHUA robot's servomotors using hitec_5980SG Lar3's' module
 */

#include <humanoid_control/servohumanoid.h>


ServoHumanoid::ServoHumanoid(const char* path)
{
    SetParameters();
    
    hitec=(hitec_5980SG*) new hitec_5980SG(path);

    //robot_state.speed_g_static=50;
    
    int speed=25;
    int position;
    //double angle_position;
    //const short int id_list[20]={61,54,53,51,44,43,41,31,32,33,21,22,23,25,26,11,12,13,15,16};
    const short int id_list[12]={11,21,12,22,13,23,15,25,16,26,31,32};   
    for(int i=0;i<12;i++)
    {
        position=SetJointSpeed(id_list[i],speed);
        ROS_INFO("Servo %d --> responce : %d", id_list[i], position);
        RoboState.joint_wanted[i]=0;
    }
}

void ServoHumanoid::SetParameters(void)
{
    //robot dimensions
    robot_dimensions.arm_length =   123.; // mm
    robot_dimensions.forearm_length =   106.; // mm
    robot_dimensions.shin_length =  139.; //mm
    robot_dimensions.thigh_length =     143.; //mm
    robot_dimensions.foot_length =  150.; //mm
    robot_dimensions.foot_width =   50.; //mm
    robot_dimensions.ankle_height =     50.; //mm
    // definition of the number PI
    //PI=3.141592653589793238462643383279502884197;
    // joint servo offsets/conversion constants
    robot_servo_conversion.head_servo_offset =              2400.;
    robot_servo_conversion.shoulder_flexion_servo_offset_left =         1050.;
    robot_servo_conversion.shoulder_flexion_servo_offset_right =        1975.;
    robot_servo_conversion.shoulder_abduction_servo_offset_left =   600.;
    robot_servo_conversion.shoulder_abduction_servo_offset_right =  2400.;
    robot_servo_conversion.elbow_flexion_servo_offset_left =        2400.;
    robot_servo_conversion.elbow_flexion_servo_offset_right =       600.;
    robot_servo_conversion.torso_rotation_servo_offset =            1500.;
    robot_servo_conversion.torso_flexion_servo_offset =         750.;
    robot_servo_conversion.torso_lateral_flexion_servo_offset =     1500.;
    robot_servo_conversion.knee_flexion_b_value_left =          600.;
    robot_servo_conversion.knee_flexion_b_value_right =         2400.;
    robot_servo_conversion.hip_flexion_b_value_left =           900.;
    robot_servo_conversion.hip_flexion_b_value_right =          1600.;
    // joint limits
    robot_joint_limits.head_min_tilt =          0.; // degree
    robot_joint_limits.head_max_tilt =          10.; // degree
    robot_joint_limits.shoulder_min_flexion =       -45.; // degree
    robot_joint_limits.shoulder_max_flexion =       135.; // degree
    robot_joint_limits.shoulder_min_abduction =     0.; // degree
    robot_joint_limits.shoulder_max_abduction =     180.; // degree
    robot_joint_limits.elbow_min_flexion =      0.; // degree
    robot_joint_limits.elbow_max_flexion =      120.; // degree
    robot_joint_limits.torso_min_rotation =     -90.; // degree
    robot_joint_limits.torso_max_rotation =     90.; // degree
    robot_joint_limits.torso_min_flexion =      -15.;  // degree
    robot_joint_limits.torso_max_flexion =      90.;  // degree
    robot_joint_limits.torso_min_lateral_flexion =  -90.; // degree
    robot_joint_limits.torso_max_lateral_flexion =  90.; // degree
    robot_joint_limits.ankle_min_inversion =        -30.; // degree
    robot_joint_limits.ankle_max_inversion =        45.; // degree
    robot_joint_limits.ankle_min_flexion =      -40.; // degree
    robot_joint_limits.ankle_max_flexion =      20.; // degree
    robot_joint_limits.knee_min_flexion =           0.; // degree
    robot_joint_limits.knee_max_flexion =           130.; // degree
    robot_joint_limits.hip_min_abduction =      -40.; // degree
    robot_joint_limits.hip_max_abduction =      45; // degree
    robot_joint_limits.hip_min_flexion =            -30.; // degree
    robot_joint_limits.hip_max_flexion =            120.; // degree
    //robot's masses
    robot_element_mass.foot_mass =  0.444; //kg
    robot_element_mass.shin_mass =  0.343; //kg
    robot_element_mass.thigh_mass = 0.243; //kg -> hip servos/joints don't count, added to pelvis
    //robot's COM
    robot_relative_COM.foot_CoM << 9.06, 2.746, 31.7; //mm
    robot_relative_COM.shin_CoM << 0.182, 15.861, 88.519; //mm
    robot_relative_COM.thigh_CoM << -2.262, 1.418, 101.284; //mm
    
    // Joint off sets to compensate the mechanical settings
    joint_offset.torso_rotation_joint_offset=0.;
    joint_offset.torso_flexion_joint_offset=-20.;
    joint_offset.torso_lateral_flexion_joint_offset=0.;
    joint_offset.knee_flexion_joint_offset_left=0. + 5.;
    joint_offset.knee_flexion_joint_offset_right=-3. + 5.;
    joint_offset.hip_flexion_joint_offset_left=-5. - 5;
    joint_offset.hip_flexion_joint_offset_right=5. - 5; // 3. - 5
    joint_offset.hip_abduction_joint_offset_left=-3.;
    joint_offset.hip_abduction_joint_offset_right=0.;
    joint_offset.feet_flexion_joint_offset_left=5. ;
    joint_offset.feet_flexion_joint_offset_right=5. ;
    joint_offset.feet_inversion_joint_offset_left=0.;
    joint_offset.feet_inversion_joint_offset_right=20.;
    
    // RoboState's inicializing
    RoboState.speed_g_static=20;
    for (int i = 0; i < 12; i++)
    {
        //RoboState.speed_now[i]=RoboState.speed_g_static;
        RoboState.speed_wanted[i]=RoboState.speed_g_static;
    }
}

void ServoHumanoid::HomePosition(void)
{
    const short int id_list[12]={11,21,12,22,13,23,15,25,16,26,31,32};    
    short unsigned int resp = 65535;
    
    for (uint i = 0 ; i < 12 ; i++)
    {
        resp = MoveJoint(id_list[i], 0.0);
        ROS_INFO("Servo with ID %d was set to Home position and gave the responce %d", id_list[i], resp);
    }
    for (uint i = 0 ; i < 12 ; i++)
    {
        resp=SetJointSpeed(id_list[i],RoboState.speed_wanted[i]);
        RoboState.joint_now[i]=ConvertServoValueByID(id_list[i], resp);
    }

}


short unsigned int ServoHumanoid::MoveJoint(short int id, double joint_angle)
{
    short unsigned int resp = 65535;
    
    switch(id)
    {
        case 31:
        {
            resp = Torso_Rotation_Movement(joint_angle + joint_offset.torso_rotation_joint_offset);
            break;
        }
        case 32:
        {
            resp = Torso_Flexion_Extension_Movement(joint_angle + joint_offset.torso_flexion_joint_offset);
            break;
        }
        //case 33:
        //{
            //resp = Torso_Lateral_Flexion_Extension_Movement(joint_angle + joint_offset.torso_lateral_flexion_joint_offset);
            //break;
        //}
        case 21:
        {
            resp = Ankle_Inversion_Eversion_Movement(id,joint_angle + joint_offset.feet_inversion_joint_offset_left);
            break;
        }
        case 22:
        {
            resp = Ankle_Flexion_Movement(id,joint_angle + joint_offset.feet_flexion_joint_offset_left);
            break;
        }
        case 23:
        {
            resp = Knee_Movement(id,joint_angle + joint_offset.knee_flexion_joint_offset_left);
            break;
        }
        case 25:
        {
            resp = Hip_Abduction_Hiperabduction_Movement(id,joint_angle + joint_offset.hip_abduction_joint_offset_left);
            break;
        }
        case 26:
        {
            resp = Hip_Flexion_Movement(id,joint_angle + joint_offset.hip_flexion_joint_offset_left);
            break;
        }
        case 11:
        {
            resp = Ankle_Inversion_Eversion_Movement(id,joint_angle + joint_offset.feet_inversion_joint_offset_right);
            break;
        }
        case 12:
        {
            resp = Ankle_Flexion_Movement(id,joint_angle + joint_offset.feet_flexion_joint_offset_right);
            break;
        }
        case 13:
        {
            resp = Knee_Movement(id,joint_angle + joint_offset.knee_flexion_joint_offset_right);
            break;
        }
        case 15:
        {
            resp = Hip_Abduction_Hiperabduction_Movement(id,joint_angle + joint_offset.hip_abduction_joint_offset_right);
            break;
        }
        case 16:
        {
            resp = Hip_Flexion_Movement(id,joint_angle + joint_offset.hip_flexion_joint_offset_right);
            break;
        }
        case 1000:
        {
            HomePosition();
            //resp = 0;
        }
        default:
        {
            ROS_INFO("Servo ID %d not recognized", id);
            break;
        }
    }
        
    return resp;
}


short unsigned int ServoHumanoid::SetJointSpeed(short int id, unsigned int speed)
{
    short unsigned int resp;
    
    switch(id)
    {
        case 31:
        {
            resp = hitec->SetSpeedPosition(id, speed);
            break;
        }
        case 32:
        {
            resp = hitec->SetSpeedPosition(id, speed);
            break;
        }
        case 33:
        {
            resp = hitec->SetSpeedPosition(id, speed);
            break;
        }
        case 21:
        {
            resp = hitec->SetSpeedPosition(id, speed*4./2.);
            break;
        }
        case 22:
        {
            resp = hitec->SetSpeedPosition(id, speed*4./2.);
            break;
        }
        case 23:
        {
            resp = hitec->SetSpeedPosition(id, speed*(30./22.));
            break;
        }
        case 25:
        {
            resp = hitec->SetSpeedPosition(id, speed*4./2.);
            break;
        }
        case 26:
        {
            resp = hitec->SetSpeedPosition(id, speed*(30./22.));
            break;
        }
        case 11:
        {
            resp = hitec->SetSpeedPosition(id, speed*4./2.);
            break;
        }
        case 12:
        {
            resp = hitec->SetSpeedPosition(id, speed*4./2.);
            break;
        }
        case 13:
        {
            resp = hitec->SetSpeedPosition(id, speed*(30./22.));
            break;
        }
        case 15:
        {
            resp = hitec->SetSpeedPosition(id, speed*4./2.);
            break;
        }
        case 16:
        {
            resp = hitec->SetSpeedPosition(id, speed*(30./22.));
            break;
        }
        case 1000:
        {
            const short int id_list[12]={11,21,12,22,13,23,15,25,16,26,31,32};     
            for(int i=0;i<12;i++)
            {
                resp=SetJointSpeed(id_list[i],speed);
            }
            ROS_INFO("Setting all Servo's speed to %d", speed);                 
        }
        default:
        {
            ROS_INFO("Servo ID %d not recognized", id);
            resp = 65535;
            break;
        }
    }
    return resp;
}


double ServoHumanoid::ConvertServoValueByID(int id, short unsigned int servo_value)
{
    
    double joint_angle=0xFFFF;
    switch (id)
    {
        case 31:
        {
            joint_angle=Torso_Rotation_ServoValue_Conversion(servo_value) - joint_offset.torso_rotation_joint_offset;
            break;
        }
        case 32:
        {
            joint_angle=Torso_Flexion_Extension_ServoValue_Conversion(servo_value) - joint_offset.torso_flexion_joint_offset;
            break;
        }
        //case 33:
        //{
            //joint_angle=Torso_Lateral_Flexion_Extension_ServoValue_Conversion(servo_value) - joint_offset.torso_lateral_flexion_joint_offset;
            //break;
        //}
        case 21:
        {
            joint_angle=Ankle_Inversion_Eversion_ServoValue_Conversion(id,servo_value) - joint_offset.feet_inversion_joint_offset_left;
            break;
        }
        case 22:
        {
            joint_angle=Ankle_Flexion_ServoValue_Conversion(id,servo_value) - joint_offset.feet_flexion_joint_offset_left;
            break;
        }
        case 23:
        {
            joint_angle=Knee_Flexion_ServoValue_Conversion(id,servo_value) - joint_offset.knee_flexion_joint_offset_left;
            break;
        }
        case 25:
        {
            joint_angle=Hip_Abduction_Hiperabduction_ServoValue_Conversion(id,servo_value) - joint_offset.hip_abduction_joint_offset_left;
            break;
        }
        case 26:
        {
            joint_angle=Hip_Flexion_ServoValue_Conversion(id,servo_value) - joint_offset.hip_flexion_joint_offset_left;
            break;
        }
        case 11:
        {
            joint_angle=Ankle_Inversion_Eversion_ServoValue_Conversion(id,servo_value) - joint_offset.feet_inversion_joint_offset_right;
            break;
        }
        case 12:
        {
            joint_angle=Ankle_Flexion_ServoValue_Conversion(id,servo_value) - joint_offset.feet_flexion_joint_offset_right;
            break;
        }
        case 13:
        {
            joint_angle=Knee_Flexion_ServoValue_Conversion(id,servo_value) - joint_offset.knee_flexion_joint_offset_right;
            break;
        }
        case 15:
        {
            joint_angle=Hip_Abduction_Hiperabduction_ServoValue_Conversion(id,servo_value) - joint_offset.hip_abduction_joint_offset_right;
            break;
        }
        case 16:
        {
            joint_angle=Hip_Flexion_ServoValue_Conversion(id,servo_value) - joint_offset.hip_flexion_joint_offset_right;
            break;
        }
        default:
        {
            joint_angle=0xFFFF;
            break;
        }
    }
    
    return joint_angle;
}


short unsigned int ServoHumanoid::Torso_Rotation_Movement(double joint_angle)
{
    short unsigned int move, resp = 65535;
    
    //   Motion range [-90...90] left to right
    //   Equivalent servo range [600...2400]
    
    // Check range of request
    if(joint_angle<robot_joint_limits.torso_min_rotation || joint_angle>robot_joint_limits.torso_max_rotation)
        return 0;
    
    //compute servo range output
    move = (robot_servo_conversion.torso_rotation_servo_offset+joint_angle*10.);
    
    resp = hitec->SetPosition(31,move);
    
    return resp;

}

short unsigned int ServoHumanoid::Torso_Flexion_Extension_Movement(double joint_angle)
{
    short unsigned int move, resp = 65535;
    
    //   Motion range [-15...90] left to right
    //   Equivalent servo range [600...2400]
    
    // Check range of request
    if(joint_angle<robot_joint_limits.torso_min_flexion || joint_angle>robot_joint_limits.torso_max_flexion)
        return 0;
    
    //compute servo range output
    move = (robot_servo_conversion.torso_flexion_servo_offset+joint_angle*10.);
    
    
    resp = hitec->SetPosition(32,move);
    
    return resp;
    
}


short unsigned int ServoHumanoid::Torso_Lateral_Flexion_Extension_Movement(double joint_angle)
{
    short unsigned int move, resp = 65535;
    
    //   Motion range [-90...90] left to right
    //   Equivalent servo range [600...2400]
    
    // Check range of request
    if(joint_angle<robot_joint_limits.torso_min_lateral_flexion || joint_angle>robot_joint_limits.torso_max_lateral_flexion)
        return 0;
    
    //compute servo range output
    move = (robot_servo_conversion.torso_lateral_flexion_servo_offset+joint_angle*10.);
    
    resp = hitec->SetPosition(33,move);
    
    return resp;
    
}


short unsigned int ServoHumanoid::Ankle_Flexion_Movement(short int id, double joint_angle)
{
    short unsigned int move, resp = 65535;
    
    // Check range of request
    if(joint_angle<robot_joint_limits.ankle_min_flexion || joint_angle>robot_joint_limits.ankle_max_flexion)
        return 0;
    
    double sign;
    if(id== 22)
    {
        // LEFT
        //   Motion range [-40...20] inside to outside
        //   Equivalent servo range [600...2400]
        sign=(double) LEFT;
    }
    else if(id== 12)
    {
        // RIGHT
        //   Motion range [-40...20] inside to outside
        //   Equivalent servo range [2400...600]
        sign=(double) RIGHT;
    }
    else
        return resp;
    
    
    double b=1500.;
    
    if(joint_angle==0)
        move = 1500;
    else if(joint_angle<0)
    {
        //   Conversion
        double m=(1500.-600.)/(-robot_joint_limits.ankle_min_flexion);
        move = round(sign*(m*joint_angle)+b);
    }
    else
    {
        //   Conversion
        double m=(2400.-1500.)/(robot_joint_limits.ankle_max_flexion);
        move = round(sign*(m*joint_angle)+b);
    }
    
    resp = hitec->SetPosition(id,move);
    
    return resp;
}


short unsigned int ServoHumanoid::Ankle_Inversion_Eversion_Movement(short int id, double joint_angle)
{
    short unsigned int move, resp = 65535;
    
    // Check range of request
    //   Motion range [-30...45] inside to outside
    if(joint_angle<robot_joint_limits.ankle_min_inversion || joint_angle>robot_joint_limits.ankle_max_inversion)
        return 0;
    
    double sign;
    if(id==21)
    {
        // LEFT
        //   Equivalent servo range [600...2400]
        sign=(double) LEFT;
    }
    else if(id==11)
    {
        // RIGHT
        //   Equivalent servo range [2400...600]
        sign=(double) RIGHT;
    }
    else
        return resp;
    
    double b=1500.;
    
    //   Conversion
    if(joint_angle==0)
        move = 1500;
    else if(joint_angle<0)
    {
        double m=(1500.-600.)/(-robot_joint_limits.ankle_min_inversion);
        move = round(sign*(m*joint_angle)+b);
    }
    else
    {
        double m=(2400.-1500.)/(robot_joint_limits.ankle_max_inversion);
        move = round(sign*(m*joint_angle)+b);
    }
    
    resp = hitec->SetPosition(id,move);
    
    return resp;
}


short unsigned int ServoHumanoid::Knee_Movement(short int id, double joint_angle)
{
    short unsigned int move, resp = 65535;
    
  // Check range of request
  if(joint_angle<robot_joint_limits.knee_min_flexion || joint_angle>robot_joint_limits.knee_max_flexion)
    return 0;
  
//   //detected very high current consumption when knee is bellow 5degrees
//   if(joint_angle<=5.)
//     joint_angle=5.;
  
  double sign;
  double b;
  if(id==23)
  {
    // LEFT
    //   Motion range [0...130] inside to outside
    //   Equivalent servo range [600...2400]
    b=robot_servo_conversion.knee_flexion_b_value_left;
    sign=(double) LEFT;
  }
  else if(id==13)
  {
    // RIGHT
    //   Motion range [0...130] inside to outside
    //   Equivalent servo range [2400...600]
    b=robot_servo_conversion.knee_flexion_b_value_right;
    sign=(double) RIGHT;
  }
  else
    return resp;
  
  //   Conversion
  double m=sign*(2400.-600.)/(robot_joint_limits.knee_max_flexion-robot_joint_limits.knee_min_flexion);
  move = round((m*joint_angle)+b);

  resp = hitec->SetPosition(id,move);

  return resp;
}


short unsigned int ServoHumanoid::Hip_Abduction_Hiperabduction_Movement(short int id, double joint_angle)
{
    short unsigned int move, resp = 65535;
    
  // this servo has a gain of 40/10 (pulley teeth)
  //TODO: set the angle conversion straight w_a = (N_b/N_a) * w_b
  
  // Check range of request
  if(joint_angle<robot_joint_limits.hip_min_abduction || joint_angle>robot_joint_limits.hip_max_abduction)
    return 0;
  
  double sign;
  if(id==25)
  {
    // LEFT
    //   Motion range [-40...45] inside to outside
    //   Equivalent servo range [2400...600]
    sign=(double) RIGHT;
  }
  else if(id==15)
  {
    // RIGHT
    //   Motion range [-40...45] inside to outside
    //   Equivalent servo range [2400...600]
    sign=(double) LEFT;
  }
  else
    return resp;
  
  double b=1500.;
  //   Conversion
  if(joint_angle==0)
    move = 1500;
  else if(joint_angle<0)
  {
    double m=900./(-robot_joint_limits.hip_min_abduction);
    move = round(sign*(m*joint_angle)+b);
  }
  else
  {
    double m=900./(robot_joint_limits.hip_max_abduction);
    move = round(sign*(m*joint_angle)+b);
  }

  resp = hitec->SetPosition(id,move);

  return resp;
}



short unsigned int ServoHumanoid::Hip_Flexion_Movement(short int id, double joint_angle)
{
    short unsigned int move, resp = 65535;
    
  // this servo has a gain of 30/22 (pulley teeth)
  //TODO: set the angle conversion straight
  
  // Check range of request
  if(joint_angle<robot_joint_limits.hip_min_flexion || joint_angle>robot_joint_limits.hip_max_flexion)
    return 0;
  
  double sign;
  double offset;
  if(id==26)
  {
    // LEFT
    //   Motion range [-30...120] inside to outside
    //   Equivalent servo range [600...2400]
    offset=robot_servo_conversion.hip_flexion_b_value_left;
    sign=(double) LEFT;
  }
  else if(id==16)
  {
    // RIGHT
    //   Motion range [-30...120] inside to outside
    //   Equivalent servo range [2400...600]
    offset=robot_servo_conversion.hip_flexion_b_value_right;
    sign=(double) RIGHT;
  }
  else
    return resp;
  
  //compute servo range output
  move = offset+sign*joint_angle*10.;


  resp = hitec->SetPosition(id,move);

  return resp;


}


double ServoHumanoid::Torso_Rotation_ServoValue_Conversion(short unsigned int servo_value)
{
  if(servo_value<595 || servo_value>2405)
  {
    return 0xFFFF;
  }
  //   Motion range [-90...90] left to right
  //   Equivalent servo range [600...2400]
  
  //compute servo range output
  return ((double)servo_value-robot_servo_conversion.torso_rotation_servo_offset)/10.;
}

double ServoHumanoid::Torso_Flexion_Extension_ServoValue_Conversion(short unsigned int servo_value)
{
  if(servo_value<595 || servo_value>2405)
  {
    return 0xFFFF;
  }
  //   Motion range [-15...90] left to right
  //   Equivalent servo range [600...2400]
  
  //compute servo range output
  return ((double)servo_value-robot_servo_conversion.torso_flexion_servo_offset)/10.;
}

double ServoHumanoid::Torso_Lateral_Flexion_Extension_ServoValue_Conversion(short unsigned int servo_value)
{
  if(servo_value<595 || servo_value>2405)
  {
    return 0xFFFF;
  }
  //   Motion range [-90...90] left to right
  //   Equivalent servo range [600...2400]
  
  //compute servo range output
  return ((double)servo_value-robot_servo_conversion.torso_lateral_flexion_servo_offset)/10.;
}



double ServoHumanoid::Ankle_Inversion_Eversion_ServoValue_Conversion(short int id, short unsigned int servo_value)
{
  // Check range of request
    if(servo_value<595 || servo_value>2405)
  {
    return 0xFFFF;
  }
  //   Motion range [-30...45] inside to outside
  
  if(servo_value==1500)
    return 0.;
  
  if(id==11)
  {
    // LEFT
    //   Equivalent servo range [600...2400]
    if(servo_value<1500)
    {
      return servo_value*(robot_joint_limits.ankle_max_inversion/-900.)-(robot_joint_limits.ankle_max_inversion/-900.)*1500.;
    }
    else
    {
      return servo_value*(robot_joint_limits.ankle_min_inversion/900.)-(robot_joint_limits.ankle_min_inversion/900.)*1500.;
    }
  }
  else if(id==21)
  {
    if(servo_value<1500)
    {
      return servo_value*(robot_joint_limits.ankle_min_inversion/-900.)-(robot_joint_limits.ankle_min_inversion/-900.)*1500.;
    }
    else
    {
      return servo_value*(robot_joint_limits.ankle_max_inversion/900.)-(robot_joint_limits.ankle_max_inversion/900.)*1500.;
    }
  }
  else
  {
    return 0xFFFF;
  }
}
    
double ServoHumanoid::Ankle_Flexion_ServoValue_Conversion(short int id, short unsigned int servo_value)
{
  // Check range of request
    if(servo_value<595 || servo_value>2405)
  {
    return 0xFFFF;
  }
  //   Motion range [-40...20] inside to outside
  
  if(servo_value==1500)
    return 0.;
  
  if(id==12)
  {
    // LEFT
    //   Equivalent servo range [600...2400]
    if(servo_value<1500)
    {
      return (double)servo_value*(robot_joint_limits.ankle_max_flexion/-900.)-(robot_joint_limits.ankle_max_flexion/-900.)*1500.;
    }
    else
    {
      return (double)servo_value*(robot_joint_limits.ankle_min_flexion/900.)-(robot_joint_limits.ankle_min_flexion/900.)*1500.;
    }
  }
  else if(id==22)
  {
    if(servo_value<1500)
    {
      return (double)servo_value*(robot_joint_limits.ankle_min_flexion/-900.)-(robot_joint_limits.ankle_min_flexion/-900.)*1500.;
    }
    else
    {
      return (double)servo_value*(robot_joint_limits.ankle_max_flexion/900.)-(robot_joint_limits.ankle_max_flexion/900.)*1500.;
    }
  }
  else
  {
    return 0xFFFF;
  }
}

double ServoHumanoid::Knee_Flexion_ServoValue_Conversion(short int id, short unsigned int servo_value)
{
  // Check range of request
    if(servo_value<595 || servo_value>2405)
  {
    return 0xFFFF;
  }
  
  double sign;
  if(id==23)
  {
    // LEFT
    //   Motion range [0...130] inside to outside
    //   Equivalent servo range [600...2400]
    sign=(double) LEFT;
    return ((double)servo_value-robot_servo_conversion.knee_flexion_b_value_left)/(sign*(2400.-600.)/(robot_joint_limits.knee_max_flexion-robot_joint_limits.knee_min_flexion));
  }
  else if(id==13)
  {
    // RIGHT
    //   Motion range [0...130] inside to outside
    //   Equivalent servo range [2400...600]
    sign=(double) RIGHT;
    return ((double)servo_value-robot_servo_conversion.knee_flexion_b_value_right)/(sign*(2400.-600.)/(robot_joint_limits.knee_max_flexion-robot_joint_limits.knee_min_flexion));
  }
  else
    return 0xFFFF;
}

double ServoHumanoid::Hip_Abduction_Hiperabduction_ServoValue_Conversion(short int id, short unsigned int servo_value)
{
  // Check range of request
  if(servo_value<595 || servo_value>2405)
  {
    return 0xFFFF;
  }
  
  if(servo_value==1500)
    return 0.;
  
  double sign;
  if(id==25)
  {
    // LEFT
    //   Motion range [-40...45] inside to outside
    //   Equivalent servo range [2400...600]
    sign=(double) RIGHT;
    if(servo_value<1500)
    {
      double m=900./(-robot_joint_limits.hip_min_abduction);
      return ((double)servo_value-1500.)/(sign*m);
    }
    else
    {
      double m=900./(robot_joint_limits.hip_max_abduction);
      return ((double)servo_value-1500.)/(sign*m);
    }
  }
  else if(id==15)
  {
    // RIGHT
    //   Motion range [-40...45] inside to outside
    //   Equivalent servo range [2400...600]
    sign=(double) LEFT;
    if(servo_value<1500)
    {
      double m=900./(-robot_joint_limits.hip_min_abduction);
      return ((double)servo_value-1500.)/(sign*m);
    }
    else
    {
      double m=900./(robot_joint_limits.hip_max_abduction);
      return ((double)servo_value-1500.)/(sign*m);
    }
  }
  else
    return 0xFFFF;
}
    
double ServoHumanoid::Hip_Flexion_ServoValue_Conversion(short int id, short unsigned int servo_value)
{
  // Check range of request
  if(servo_value<595 || servo_value>2405)
  {
    return 0xFFFF;
  }
  
  double sign;
  double offset;
  if(id==26)
  {
    // LEFT
    //   Motion range [-30...120] inside to outside
    //   Equivalent servo range [600...2400]
    offset=robot_servo_conversion.hip_flexion_b_value_left;
    sign=(double) LEFT;
    if(servo_value==offset)
      return 0.;
  }
  else if(id==16)
  {
    // RIGHT
    //   Motion range [-30...120] inside to outside
    //   Equivalent servo range [2400...600]
    offset=robot_servo_conversion.hip_flexion_b_value_right;
    sign=(double) RIGHT;
    if(servo_value==offset)
      return 0.;
  }
  else
    return 0xFFFF;
  
  //compute servo range output
  return sign*((double)servo_value-offset)/10.;
}


//void ServoHumanoid::Inv_Kin_3DOF_Detached_Leg(double X, double Y, double Z, double *req_angle)
//{
   ////vars
  //double L1 = robot_dimensions.shin_length;
  //double L2 = robot_dimensions.thigh_length;
  //double z_clearance = robot_dimensions.ankle_height;

  ////WILLIAM LAGE CODE FOR dsPIC
  //// adjusted for left leg and frame orientations...
 
  //double A,q1,q2,q3;
  //double k1,k2,k3,kk_aux;
  
  
  //// Ankle joint (lateral plane)
  //q1 = atan2(Y,(Z - z_clearance));
  
  //if((q1 > PI/4. && q1 < 3. * PI/4.) || (q1 <  - PI/4. && q1 >  - 3. * PI/4.))
      //A = Y/sin(q1);
  //else
      //A =  - (Z - z_clearance)/cos(q1);
  
  //// Ankle joint (sagittal plane) 
  //k1 =  2. * L1 * X;
  //k2 =  - 2. * L1 * A;
  //k3 = pow(X,2.) + pow(A,2.) + pow(L1,2.) - pow(L2,2.);
  //kk_aux = pow(k1,2.) + pow(k2,2.) - pow(k3,2.);

  //if(kk_aux < 0.)
  //{
    //req_angle[0] = 1000.;
    //req_angle[1] = 1000.;
    //req_angle[2] = 1000.;
    //return;
  //}
  //q2 = atan2(k1,k2) + atan2(sqrt(pow(k1,2.) + pow(k2,2.) - pow(k3,2.)), k3);
    
  ////Angkle in the interval  - pi to pi
  //if(q2 > PI)
    //q2 = q2 - 2. * PI;

  //if(q2 <  - PI)
    //q2 = q2 + 2. * PI;
    
  ////Knee joint
  //k1 = 0.;
  //k2 = 2. * L1 * L2;
  //k3 = pow(X,2.) + pow(A,2.) - pow(L1,2.) - pow(L2,2.);
  //kk_aux = pow(k2,2.) - pow(k3,2.);

  //if(kk_aux < 0.)
  //{
    //req_angle[0] = 1000.;
    //req_angle[1] = 1000.;
    //req_angle[2] = 1000.;
    //return;
  //}
  //q3 = atan2(sqrt(pow(k2,2.) - pow(k3,2.)), k3);
  
  ////cout<<"THETA1: "<<q1*(180. / PI)<<" THETA2:"<<q2*(180. / PI)<<" THETA3:"<<q3*(180. / PI)<<endl;
  
  //if(!isnan(q1 * q2 * q3))
  //{
    
////     //adjust to servo tolerance
////     if(q1 < (robot_joint_limits.ankle_min_inversion *PI/180.) && q1 > ((robot_joint_limits.ankle_min_inversion-5.)*PI/180. + 0.0034))
////       q1=robot_joint_limits.ankle_min_inversion*PI/180.;
////     else if(q1 < (((robot_joint_limits.ankle_max_inversion+5.)*PI/180.)+0.0034) && q1 > (robot_joint_limits.ankle_max_inversion*PI/180.))
////       q1=robot_joint_limits.ankle_max_inversion*PI/180.;
    ////check joint limits
    //if(q1 >=(robot_joint_limits.ankle_min_inversion *PI/180.) && q1 <= (robot_joint_limits.ankle_max_inversion*PI/180.))
      //req_angle[0]=q1*180./PI;
    //else
      //req_angle[0] = 1000.;
    
////     //adjust to servo tolerance
////     if(q2 < (robot_joint_limits.ankle_min_flexion *PI/180.) && q2 > ((robot_joint_limits.ankle_min_flexion-5.)*PI/180. + 0.0034))
////       q2=robot_joint_limits.ankle_min_flexion*PI/180.;
////     else if(q2 < (((robot_joint_limits.ankle_max_flexion+5.)*PI/180.)+0.0034) && q2 > (robot_joint_limits.ankle_max_flexion*PI/180.))
////       q2=robot_joint_limits.ankle_max_flexion*PI/180.;
    ////check joint limits
    //if(q2 >=(robot_joint_limits.ankle_min_flexion *PI/180.) && q2 <= (robot_joint_limits.ankle_max_flexion*PI/180.))
      //req_angle[1]=q2*180./PI;
    //else
      //req_angle[1] = 1000.;
    
////     //adjust to servo tolerance
////     if(q3 < (robot_joint_limits.knee_min_flexion *PI/180.) && q3 > ((robot_joint_limits.knee_min_flexion-5.)*PI/180. + 0.0034))
////       q1=robot_joint_limits.knee_min_flexion;
////     else if(q3 < (((robot_joint_limits.knee_max_flexion+5.)*PI/180.)+0.0034) && q3 > (robot_joint_limits.knee_max_flexion*PI/180.))
////       q3=robot_joint_limits.knee_max_flexion*PI/180.;
    ////check joint limits
    //if(q3 >=robot_joint_limits.knee_min_flexion && q3 <= (robot_joint_limits.knee_max_flexion*PI/180.))
      //req_angle[2]=q3*180./PI;
    //else
      //req_angle[2] = 1000.;
    
    
////         req_angle[0]=q1*180./PI;
////     req_angle[1]=q2*180./PI;
////     req_angle[2]=q3*180./PI;
    
////     cout<<"THETA1: "<<req_angle[0]<<" THETA2:"<<req_angle[1]<<" THETA3:"<<req_angle[2]<<endl;

    //return;
  //}
  //else
  //{
    //req_angle[0] = 1000.;
    //req_angle[1] = 1000.;
    //req_angle[2] = 1000.;
    //return;
  //}
//}

