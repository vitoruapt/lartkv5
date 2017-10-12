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
 * \brief Humanoid functions implementation
 */

#include <phua_haptic/humanoid_functions.h>

using namespace std;

humanoid::humanoid(void)
{
  //robot dimensions
  robot_dimensions.arm_length = 	123.; // mm
  robot_dimensions.forearm_length = 	106.; // mm
  robot_dimensions.shin_length = 	139.; //mm
  robot_dimensions.thigh_length = 	143.; //mm
  robot_dimensions.foot_length = 	150.; //mm
  robot_dimensions.foot_width = 	50.; //mm
  robot_dimensions.ankle_height = 	50.; //mm
  // definition of the number PI
  PI=3.1415926535898;
  // joint servo offsets/conversion constants
  robot_servo_conversion.head_servo_offset = 				2400.;
  robot_servo_conversion.shoulder_flexion_servo_offset_left = 		1050.;
  robot_servo_conversion.shoulder_flexion_servo_offset_right = 		1975.;
  robot_servo_conversion.shoulder_abduction_servo_offset_left = 	600.;
  robot_servo_conversion.shoulder_abduction_servo_offset_right =	2400.;
  robot_servo_conversion.elbow_flexion_servo_offset_left =		2400.;
  robot_servo_conversion.elbow_flexion_servo_offset_right =		600.;
  robot_servo_conversion.torso_rotation_servo_offset =			1500.;
  robot_servo_conversion.torso_flexion_servo_offset =			750.;
  robot_servo_conversion.torso_lateral_flexion_servo_offset =		1500.;
  robot_servo_conversion.knee_flexion_b_value_left =			600.;
  robot_servo_conversion.knee_flexion_b_value_right =			2400.;
  robot_servo_conversion.hip_flexion_b_value_left =			900.;
  robot_servo_conversion.hip_flexion_b_value_right =			1600.;
  // joint limits
  robot_joint_limits.head_min_tilt =			0.; // degree
  robot_joint_limits.head_max_tilt =			10.; // degree
  robot_joint_limits.shoulder_min_flexion =		-45.; // degree
  robot_joint_limits.shoulder_max_flexion =		135.; // degree
  robot_joint_limits.shoulder_min_abduction =		0.; // degree
  robot_joint_limits.shoulder_max_abduction =		180.; // degree
  robot_joint_limits.elbow_min_flexion =		0.; // degree
  robot_joint_limits.elbow_max_flexion =		120.; // degree
  robot_joint_limits.torso_min_rotation =		-90.; // degree
  robot_joint_limits.torso_max_rotation =		90.; // degree
  robot_joint_limits.torso_min_flexion =		-15.;  // degree
  robot_joint_limits.torso_max_flexion =		90.;  // degree
  robot_joint_limits.torso_min_lateral_flexion =	-90.; // degree
  robot_joint_limits.torso_max_lateral_flexion =	90.; // degree
  robot_joint_limits.ankle_min_inversion =		-30.; // degree
  robot_joint_limits.ankle_max_inversion =		45.; // degree
  robot_joint_limits.ankle_min_flexion =		-40.; // degree
  robot_joint_limits.ankle_max_flexion =		20.; // degree
  robot_joint_limits.knee_min_flexion =			0.; // degree
  robot_joint_limits.knee_max_flexion =			130.; // degree
  robot_joint_limits.hip_min_abduction =		-40.; // degree
  robot_joint_limits.hip_max_abduction =		45; // degree
  robot_joint_limits.hip_min_flexion =			-30.; // degree
  robot_joint_limits.hip_max_flexion =			120.; // degree
  //robot's masses
  robot_element_mass.foot_mass =	0.444; //kg
  robot_element_mass.shin_mass =	0.343; //kg
  robot_element_mass.thigh_mass =	0.243; //kg -> hip servos/joints don't count, added to pelvis
  //robot's COM
  robot_relative_COM.foot_COM << 9.06, 2.746, 31.7; //mm
  robot_relative_COM.shin_COM << 0.182, 15.861, 88.519; //mm
  robot_relative_COM.thigh_COM << -2.262, 1.418, 101.284; //mm
  
  return;
}

humanoid::~humanoid(void)
{
  memset(&robot_dimensions, 0., sizeof(robot_dimensions));
  memset(&robot_servo_conversion, 0., sizeof(robot_servo_conversion));
  memset(&robot_joint_limits, 0., sizeof(robot_joint_limits));
  memset(&robot_element_mass, 0., sizeof(robot_element_mass));
//   cout<<"Deconstructing humanoid class"<<endl;
  return;
}

void humanoid::Dir_Kin_3DOF_Arm(double t1, double t2, double t3, short int SIDE, double *end_pos_n_RPY)
{
  //vars
  double L1 = robot_dimensions.arm_length;
  double L2 = robot_dimensions.forearm_length;
  double X, Y, Z, ROLL, PITCH, YAW;
  
  double theta1 = t1 * PI / 180.;
  double theta2 = t2 * PI / 180.;
  double theta3 = t3 * PI / 180.;
  
  //X [mm]
  X = L1 * cos(theta2) * sin(theta1) + L2 * (cos(theta1) * sin(theta3) + cos(theta2) * cos(theta3) * sin(theta1));
  //Y [mm]
  Y = (double)SIDE * (sin(theta2) * (L1 + L2 * cos(theta3)));
  //Z [mm]
  Z = L2 * (sin(theta1) * sin(theta3) - cos(theta1) * cos(theta2) * cos(theta3)) - L1 * cos(theta1) * cos(theta2);
  //PITCH [rad]
  PITCH = asin(cos(theta1) * cos(theta2) * cos(theta3) - sin(theta1) * sin(theta3));
  //ROLL [rad]
  ROLL = asin(((double)SIDE * cos(theta3)*sin(theta2))/cos(PITCH));
  //YAW [rad]
  YAW = acos(((double)SIDE * -1. * cos(theta1) * sin(theta2)) / cos(PITCH));
  
  end_pos_n_RPY[0] = X;
  end_pos_n_RPY[1] = Y;
  end_pos_n_RPY[2] = Z;
  end_pos_n_RPY[3] = ROLL * 180. / PI; // ROLL [deg]
  end_pos_n_RPY[4] = PITCH * 180. / PI; // PITCH [deg]
  end_pos_n_RPY[5] = YAW * 180. / PI; // YAW [deg]
}

void humanoid::Inv_Kin_3DOF_Arm(double X, double Y, double Z, double *req_angle)
{
  //vars
  double L1 = robot_dimensions.arm_length;
  double L2 = robot_dimensions.forearm_length;
  
  static Eigen::MatrixXd solutions(3,8);


// 					redundancy tree:
// 			t3_pos 						   t3_neg
// 		t2_pos		t2_neg				    t2_pos	 t2_neg
// 	  t1_pos  t1_neg    t1_pos  t1_neg		      t1_pos  t1_neg   t1_pos  t1_neg
//   
//   
//   
// 			possible combinations:
//   t3_pos | t3_pos | t3_pos | t3_pos | t3_neg | t3_neg | t3_neg | t3_neg
//   t2_pos | t2_pos | t2_neg | t2_neg | t2_pos | t2_pos | t2_neg | t2_neg
//   t1_pos | t1_neg | t1_pos | t1_neg | t1_pos | t1_neg | t1_pos | t1_neg
//   ---------------------------------------------------------------------
//    comb1 | comb2  | comb3  | comb4  | comb5  | comb6  | comb7  | comb8
//      0   |   1    |   2    |    3   |    4   |    5   |    6   |   7
  
  double t1_pos, t2_pos, t3_pos;
  double t1_neg, t2_neg, t3_neg;
  int i;
  double k1, k2 ,k3, yy, xx;
  /*
  t3_pos=acos((pow(X,2.)+pow(Y,2.)+pow(Z,2.)-pow(L1,2.)-pow(L2,2.))/(2.*L1*L2));
  if(!(t3_pos>=0. && t3_pos<=120.*PI/180.))
  {
    req_angle[0] = 1000.;
    req_angle[1] = 1000.;
    req_angle[2] = 1000.;
    cout<<"ERROR!"<<endl;
    return;
  }
  
  cout<<"THETA3:"<<t3_pos<<endl;
  
  xx = X + (L2 * sin(t3_pos));
  yy = Z + sqrt( pow(X,2.) + pow(Z,2.) - pow(L2,2.) * (pow(-1.*cos(t3_pos),2.) + 1.));
  
  cout<<"sqrt:"<<sqrt( pow(X,2.) + pow(Z,2.) - pow((L2 * sin(t3_pos)),2.))<<endl;
  
  t1_pos=2.*atan2(yy,xx);
  
  cout<<"THETA1:"<<t1_pos<<endl;
  
  if(!(t1_pos>=-40.*PI/180. && t1_pos<=140.*PI/180.))
  {
    req_angle[0] = 1000.;
    req_angle[1] = 1000.;
    req_angle[2] = 1000.;
    cout<<"ERROR!"<<endl;
    return;
  }
  
  t2_pos = atan2( Y * cos(t1_pos), L2 * sin(t1_pos) * sin(t3_pos) - Z);
  
  cout<<"THETA2:"<<t2_pos<<endl;
  
  if(!(t2_pos>=0. && t2_pos<=PI))
  {
    req_angle[0] = 1000.;
    req_angle[1] = 1000.;
    req_angle[2] = 1000.;
    cout<<"ERROR!"<<endl;
    return;
  }

  if(!isnan(t1_pos * t2_pos * t3_pos))
  {
    req_angle[0]=t1_pos*180./PI;
    req_angle[1]=t2_pos*180./PI;
    req_angle[2]=t3_pos*180./PI;
  }
  else
  {
    req_angle[0] = 1000.;
    req_angle[1] = 1000.;
    req_angle[2] = 1000.;
    cout<<"ERROR!"<<endl;
  }
  
  printf("X:%f[mm] | Y:%f[mm] | Z:%f[mm]\n",X,Y,Z);
  printf("Theta1:%g[deg] | Theta2:%f[deg] | Theta3:%f[deg]\n\n",req_angle[0],req_angle[1],req_angle[2]); fflush(stdout);*/
  
  
  //  ******* ******* theta3 positive *******  ******* 
  t3_pos=acos((pow(X,2.)+pow(Y,2.)+pow(Z,2.)-pow(L1,2.)-pow(L2,2.))/(2.*L1*L2));
  if(!isnan(t3_pos))
  {
    //tolerance in rad
    if(fabs(t3_pos)<(double) 0.0034)
    {
      t3_pos=robot_joint_limits.elbow_min_flexion;
    }
    else if(fabs(t3_pos)<(((robot_joint_limits.elbow_max_flexion+5.)*PI/180.)+0.0034) && fabs(t3_pos)>(robot_joint_limits.elbow_max_flexion*PI/180.))
    {
      t3_pos=robot_joint_limits.elbow_max_flexion*PI/180.;
    }
    //check limits
    if(!((t3_pos*180./PI)>=robot_joint_limits.elbow_min_flexion && (t3_pos*180./PI)<=robot_joint_limits.elbow_max_flexion))
    {
      //if not a solution place nan
      for(i=0;i<4;i++)
	solutions(0,i)=0./0.;
    }
    else
    {
      //if correct place value
      for(i=0;i<4;i++)
	solutions(0,i)=t3_pos;
    }
  }
  else
  {
    //if not a solution place nan
    for(i=0;i<4;i++)
      solutions(0,i)=0./0.;
  }
  //  ******* ******* theta3 negative *******  *******
  t3_neg=-1.*acos((pow(X,2.)+pow(Y,2.)+pow(Z,2.)-pow(L1,2.)-pow(L2,2.))/(2.*L1*L2));
  if(!isnan(t3_neg))
  {
    //tolerance in rad
    if(fabs(t3_neg)<(double) 0.0034)
    {
      t3_pos=robot_joint_limits.elbow_min_flexion;
    }
    else if(fabs(t3_neg)<(((robot_joint_limits.elbow_max_flexion+5.)*PI/180.)+0.0034) && fabs(t3_neg)>(robot_joint_limits.elbow_max_flexion*PI/180.))
    {
      t3_neg=robot_joint_limits.elbow_max_flexion*PI/180.;
    }
    //check limits
    if(!((t3_neg*180./PI)>=robot_joint_limits.elbow_min_flexion && (t3_neg*180./PI)<=robot_joint_limits.elbow_max_flexion))
    {
      //if not a solution place nan
      for(i=4;i<8;i++)
	solutions(0,i)=0./0.;
    }
    else
    {
      //if correct place value
      for(i=4;i<8;i++)
	solutions(0,i)=t3_neg;
    }
  }
  else
  {
    //if not a solution place nan
    for(i=4;i<8;i++)
      solutions(0,i)=0./0.;
  }
  //  ******* ******* theta2 positive with theta3 positive *******  ******* 
  t2_pos=asin(Y / ( L1 + L2 * cos(solutions(0,0))));
  if(!isnan(t2_pos))
  {
    //tolerance in rad
    if(fabs(t2_pos)<(double) 0.0034)
    {
      t2_pos=robot_joint_limits.shoulder_min_abduction;
    }
    else if(t2_pos<(((robot_joint_limits.shoulder_max_abduction+5.)*PI/180.)+0.0034) && t2_pos>(robot_joint_limits.shoulder_max_abduction*PI/180.))
    {
      t2_pos=robot_joint_limits.shoulder_max_abduction*PI/180.;
    }
    //check limits
    if(!(t2_pos>=robot_joint_limits.shoulder_min_abduction && t2_pos<=(robot_joint_limits.shoulder_max_abduction*PI/180.)))
    {
      //if not a solution place nan
      solutions(1,0)=0./0.;
      solutions(1,1)=0./0.;
    }
    else
    {
      //if correct place value
      solutions(1,0)=t2_pos;
      solutions(1,1)=t2_pos;
    }
  }
  else
  {
    //if not a solution place nan
    solutions(1,0)=0./0.;
    solutions(1,1)=0./0.;
  }
  //  ******* ******* theta2 negative with theta3 positive *******  ******* 
  t2_neg=-1. * asin(Y / ( L1 + L2 * cos(solutions(0,0))));
  if(!isnan(t2_neg))
  {
    //tolerance in rad
    if(fabs(t2_neg)<(double) 0.0034)
    {
      t2_pos=robot_joint_limits.shoulder_min_abduction;
    }
    else if(t2_neg<(((robot_joint_limits.shoulder_max_abduction+5.)*PI/180.)+0.0034) && t2_neg>(robot_joint_limits.shoulder_max_abduction*PI/180.))
    {
      t2_neg=robot_joint_limits.shoulder_max_abduction*PI/180.;
    }
    //check limits
    if(!(t2_neg>=robot_joint_limits.shoulder_min_abduction && t2_neg<=(robot_joint_limits.shoulder_max_abduction*PI/180.)))
    {
      //if not a solution place nan
      solutions(1,2)=0./0.;
      solutions(1,3)=0./0.;
    }
    else
    {
      //if correct place value
      solutions(1,2)=t2_neg;
      solutions(1,3)=t2_neg;
    }
  }
  else
  {
    //if not a solution place nan
    solutions(1,2)=0./0.;
    solutions(1,3)=0./0.;
  }
  //  ******* ******* theta2 positive with theta3 negative *******  ******* 
  t2_pos=asin(Y / ( L1 + L2 * cos(solutions(0,4))));
  if(!isnan(t2_pos))
  {
    //tolerance in rad
    if(fabs(t2_pos)<(double) 0.0034)
    {
      t2_pos=robot_joint_limits.shoulder_min_abduction;
    }
    else if(t2_pos<(((robot_joint_limits.shoulder_max_abduction+5.)*PI/180.)+0.0034) && t2_pos>(robot_joint_limits.shoulder_max_abduction*PI/180.))
    {
      t2_pos=robot_joint_limits.shoulder_max_abduction*PI/180.;
    }
    //check limits
    if(!(t2_pos>=robot_joint_limits.shoulder_min_abduction && t2_pos<=(robot_joint_limits.shoulder_max_abduction*PI/180.)))
    {
      //if not a solution place nan
      solutions(1,4)=0./0.;
      solutions(1,5)=0./0.;
    }
    else
    {
      //if correct place value
      solutions(1,4)=t2_pos;
      solutions(1,5)=t2_pos;
    }
  }
  else
  {
    //if not a solution place nan
    solutions(1,4)=0./0.;
    solutions(1,5)=0./0.;
  }
  //  ******* ******* theta2 negative with theta3 negative *******  ******* 
  t2_neg=-1. * asin(Y / ( L1 + L2 * cos(solutions(0,4))));
  if(!isnan(t2_neg))
  {
    //tolerance in rad
    if(fabs(t2_neg)<(double) 0.0034)
    {
      t2_pos=robot_joint_limits.shoulder_min_abduction;
    }
    else if(t2_neg<(((robot_joint_limits.shoulder_max_abduction+5.)*PI/180.)+0.0034) && t2_neg>(robot_joint_limits.shoulder_max_abduction*PI/180.))
    {
      t2_neg=robot_joint_limits.shoulder_max_abduction*PI/180.;
    }
    //check limits
    if(!(t2_neg>=robot_joint_limits.shoulder_min_abduction && t2_neg<=(robot_joint_limits.shoulder_max_abduction*PI/180.)))
    {
      //if not a solution place nan
      solutions(1,6)=0./0.;
      solutions(1,7)=0./0.;
    }
    else
    {
      //if correct place value
      solutions(1,6)=t2_neg;
      solutions(1,7)=t2_neg;
    }
  }
  else
  {
    //if not a solution place nan
    solutions(1,6)=0./0.;
    solutions(1,7)=0./0.;
  }
  //  ******* ******* theta1 positive with previous in matrix *******  *******
  for(i=0;i<8;i++)
  {
    k1=L2 * sin(solutions(0,i));
    k2=L1 * cos(solutions(1,i)) + L2 * cos(solutions(1,i)) * cos(solutions(0,i));
    k3=X;
    xx=k1 + k3;
    yy=k2 + sqrt(pow(k1,2.) + pow(k2,2.) - pow(k3,2.));
    t1_pos=2.*atan2(yy,xx);
    if(!isnan(t1_pos))
    {
      //tolerance in rad
      if(t1_pos<robot_joint_limits.shoulder_min_flexion*PI/180. && t1_pos>((robot_joint_limits.shoulder_min_flexion-5.)*PI/180.+0.0034))
      {
	t1_pos=robot_joint_limits.shoulder_min_flexion*PI/180.;
      }
      else if(t1_pos<(((robot_joint_limits.shoulder_max_flexion+5.)*PI/180.)+0.0034) && t1_pos>(robot_joint_limits.shoulder_max_flexion*PI/180.))
      {
	t1_pos=robot_joint_limits.shoulder_max_flexion*PI/180.;
      }
      //check limits
      if(!(t1_pos>=robot_joint_limits.shoulder_min_flexion*PI/180. && t1_pos<=robot_joint_limits.shoulder_max_flexion*PI/180.))
      {
	//if not a solution place nan
	solutions(2,i)=0./0.;
      }
      else
      {
	//if correct place value
	solutions(2,i)=t1_pos;
      }
    }
    else
    {
      //if not a solution place nan
      solutions(2,i)=0./0.;
    }
    i++;
  }
  //    ******* ******* theta1 negative with previous in matrix *******  *******
  for(i=1;i<8;i++)
  {
    k1=L2 * sin(solutions(0,i));
    k2=L1 * cos(solutions(1,i)) + L2 * cos(solutions(1,i)) * cos(solutions(0,i));
    k3=X;
    xx=k1 + k3;
    yy=k2 - sqrt(pow(k1,2.) + pow(k2,2.) - pow(k3,2.));
    t1_neg=2.*atan2(yy,xx);
    if(!isnan(t1_neg))
    {
      //tolerance in rad
      if(t1_neg<robot_joint_limits.shoulder_min_flexion*PI/180. && t1_neg>((robot_joint_limits.shoulder_min_flexion-5.)*PI/180.+0.0034))
      {
	t1_neg=robot_joint_limits.shoulder_min_flexion*PI/180.;
      }
      else if(t1_neg<(((robot_joint_limits.shoulder_max_flexion+5.)*PI/180.)+0.0034) && t1_neg>(robot_joint_limits.shoulder_max_flexion*PI/180.))
      {
	t1_neg=robot_joint_limits.shoulder_max_flexion*PI/180.;
      }
      //check limits
      if(!(t1_neg>=robot_joint_limits.shoulder_min_flexion*PI/180. && t1_neg<=robot_joint_limits.shoulder_max_flexion*PI/180.))
      {
	//if not a solution place nan
	solutions(2,i)=0./0.;
      }
      else
      {
	//if correct place value
	solutions(2,i)=t1_neg;
      }
    }
    else
    {
      //if not a solution place nan
      solutions(2,i)=0./0.;
    }
    i++;
  }
  
//   cout<<"  X   Y   Z"<<endl<<"["<<X<<"]["<<Y<<"]["<<Z<<"]"<<endl;
//   cout<<solutions*180./PI<<endl;
  
  //decide on wich redundancy to use
  req_angle[0] = 1000.;
  req_angle[1] = 1000.;
  req_angle[2] = 1000.;
  
  double theta1;
  double theta2;
  double theta3;
  
  //choose kin solution from octant
  if(Z<=0. && Y>=0.)
  {
    theta1=solutions(2,1);
    theta2=solutions(1,1);
    theta3=solutions(0,1);
    if(!isnan(theta1 * theta2 * theta3))
    {
      req_angle[0]=theta1*180./PI;
      req_angle[1]=theta2*180./PI;
      req_angle[2]=theta3*180./PI;
      return;
    }
  }
  else if(Z>=0. && Y>=0. && X>=0.)
  {
    theta1=solutions(2,0);
    theta2=solutions(1,0);
    theta3=solutions(0,0);
    if(!isnan(theta1 * theta2 * theta3))
    {
      req_angle[0]=theta1*180./PI;
      req_angle[1]=theta2*180./PI;
      req_angle[2]=theta3*180./PI;
      return;
    }
  }
  else
  {
    theta1=solutions(2,4);
    theta2=solutions(1,4);
    theta3=solutions(0,4);
    if(!isnan(theta1 * theta2 * theta3))
    {
      req_angle[0]=theta1*180./PI;
      req_angle[1]=theta2*180./PI;
      req_angle[2]=theta3*180./PI;
      return;
    }
//     else
//       cout<<"ELSE"<<endl<<endl;
  }
  
}

void humanoid::CalculateArmJacobian(double theta_1, double theta_2, double theta_3, Eigen::Matrix3d *jacobian)
{
  //vars
  double L1 = robot_dimensions.arm_length;
  double L2 = robot_dimensions.forearm_length;
  
  double theta1=theta_1*PI/180.;
  double theta2=theta_2*PI/180.;
  double theta3=theta_3*PI/180.;
  
  //calculate coefficients
  double dx_dtheta1 = cos(theta1) * (L1 * cos(theta2) + L2 * cos(theta2) * cos(theta3)) - L2 * sin(theta1) * sin(theta3);
  
  double dx_dtheta2 = -sin(theta2) * sin(theta1) * (L1 + L2 * cos(theta3));
  
  double dx_dtheta3 = L2 * (cos(theta3) * cos(theta1) - sin(theta3) * sin(theta1) * cos(theta2));
  
  double dy_dtheta1 = 0.;
  
  double dy_dtheta2 = cos(theta2) * (L1 + L2 * cos(theta3));
  
  double dy_dtheta3 = -L2 * sin(theta3) * sin(theta2);
  
  double dz_dtheta1 = L2 * cos(theta1) * sin(theta3) + sin(theta1) * cos(theta2) * (L1 + L2 * cos(theta3));
  
  double dz_dtheta2 = sin(theta2) * cos(theta1) * (L1 + L2 * cos(theta3));
  
  double dz_dtheta3 = L2 * (cos(theta3) * sin(theta1) + sin(theta3) * cos(theta1) * cos(theta2));
  
  //place coefficients in jacobian matrix
  Eigen::Matrix3d jacobian_temp;
  jacobian_temp << dx_dtheta1, dx_dtheta2, dx_dtheta3,
		   dy_dtheta1, dy_dtheta2, dy_dtheta3,
		   dz_dtheta1, dz_dtheta2, dz_dtheta3;
  //replace matrix
  memcpy(jacobian,&jacobian_temp,sizeof(jacobian_temp));
}

void humanoid::CalculateArmAlternateJacobian_No_Theta3_Singularity(double theta_1, double theta_2, double theta_3, Eigen::MatrixXd *jacobian)
{
  //vars
  double L1 = robot_dimensions.arm_length;
  double L2 = robot_dimensions.forearm_length;
  
  double theta1=theta_1*PI/180.;
  double theta2=theta_2*PI/180.;
  double theta3=theta_3*PI/180.;
  
  //calculate coefficients
  double dx_dtheta1 = cos(theta1) * (L1 * cos(theta2) + L2 * cos(theta2) * cos(theta3)) - L2 * sin(theta1) * sin(theta3);
  
  double dx_dtheta2 = -sin(theta2) * sin(theta1) * (L1 + L2 * cos(theta3));
  
  double dy_dtheta1 = 0.;
  
  double dy_dtheta2 = cos(theta2) * (L1 + L2 * cos(theta3));
  
  double dz_dtheta1 = L2 * cos(theta1) * sin(theta3) + sin(theta1) * cos(theta2) * (L1 + L2 * cos(theta3));
  
  double dz_dtheta2 = sin(theta2) * cos(theta1) * (L1 + L2 * cos(theta3));
  
  //place coefficients in jacobian matrix
  static Eigen::MatrixXd jacobian_temp(3,2);
  jacobian_temp << dx_dtheta1, dx_dtheta2,
		   dy_dtheta1, dy_dtheta2,
		   dz_dtheta1, dz_dtheta2;
  //replace matrix
  memcpy(jacobian,&jacobian_temp,sizeof(jacobian_temp));
}

void humanoid::Dir_Kin_4DOF_Detached_Leg(double t1, double t2, double t3, double t4, short int SIDE, double *end_pos_n_RPY)
{
  //vars
//   double L1 = robot_dimensions.shin_length;
//   double L2 = robot_dimensions.thigh_length;
//   double z_clearance = robot_dimensions.ankle_height;
  
  double theta1 = (t1 * PI / 180.) * (double)SIDE;
  double theta2 = t2 * PI / 180.;
  double theta3 = t3 * PI / 180.;
  double theta4 = t4 * PI / 180.;

  end_pos_n_RPY[0] = sin(theta2) * ((robot_dimensions.shin_length) + (robot_dimensions.thigh_length) * cos(theta3)) -
		    (robot_dimensions.thigh_length) * cos(theta2) * sin(theta3); //X [mm]
  
  end_pos_n_RPY[1] = (cos(theta2) * sin(theta1) * ((robot_dimensions.shin_length) + (robot_dimensions.thigh_length) * cos(theta3)) +
		    (robot_dimensions.thigh_length) * sin(theta1) * sin(theta2) * sin(theta3));// * (double)SIDE; //Y [mm]
  
  end_pos_n_RPY[2] = (robot_dimensions.ankle_height) + cos(theta1) * cos(theta2) * ((robot_dimensions.shin_length) + (robot_dimensions.thigh_length) * cos(theta3)) +
		      (robot_dimensions.thigh_length) * cos(theta1) * sin(theta2) * sin(theta3); //Z [mm]
  
  end_pos_n_RPY[3] = asin(-(cos(theta4) * (cos(theta1) * sin(theta2) * sin(theta3) + cos(theta1) * cos(theta2) * cos(theta3)) - sin(theta1)*sin(theta4))) *
																			    (180. / PI); // ROLL [deg]
																			    
  end_pos_n_RPY[4] = atan2(cos(theta1) * sin(theta2) * cos(theta3) - cos(theta1) * cos(theta2) * cos(theta3),
			  cos(theta4) * sin(theta1) + sin(theta4) * (cos(theta1) * sin(theta2) * sin(theta3) + cos(theta1) * cos(theta2) * cos(theta3))) *
																			    (180. / PI); // PITCH [deg]
																			    
  end_pos_n_RPY[5] = atan2(sin(theta4) * cos(theta1) + cos(theta4) * (sin(theta1) * sin(theta2) * sin(theta3) + cos(theta2) * cos(theta3) * sin(theta1)),
			    -cos(theta4) * (sin(theta3) * cos(theta2) - sin(theta2) * cos(theta3))) *
												      (180. / PI); // YAW [deg]
}

void humanoid::Inv_Kin_3DOF_Detached_Leg(double X, double Y, double Z, double *req_angle)
{
   //vars
  double L1 = robot_dimensions.shin_length;
  double L2 = robot_dimensions.thigh_length;
  double z_clearance = robot_dimensions.ankle_height;

  //WILLIAM LAGE CODE FOR dsPIC
  // adjusted for left leg and frame orientations...
 
  double A,q1,q2,q3;
  double k1,k2,k3,kk_aux;
  
  
  // Ankle joint (lateral plane)
  q1 = atan2(Y,(Z - z_clearance));
  
  if((q1 > PI/4. && q1 < 3. * PI/4.) || (q1 <  - PI/4. && q1 >  - 3. * PI/4.))
      A = Y/sin(q1);
  else
      A =  - (Z - z_clearance)/cos(q1);
  
  // Ankle joint (sagittal plane) 
  k1 =  2. * L1 * X;
  k2 =  - 2. * L1 * A;
  k3 = pow(X,2.) + pow(A,2.) + pow(L1,2.) - pow(L2,2.);
  kk_aux = pow(k1,2.) + pow(k2,2.) - pow(k3,2.);

  if(kk_aux < 0.)
  {
    req_angle[0] = 1000.;
    req_angle[1] = 1000.;
    req_angle[2] = 1000.;
    return;
  }
  q2 = atan2(k1,k2) + atan2(sqrt(pow(k1,2.) + pow(k2,2.) - pow(k3,2.)), k3);
    
  //Angkle in the interval  - pi to pi
  if(q2 > PI)
    q2 = q2 - 2. * PI;

  if(q2 <  - PI)
    q2 = q2 + 2. * PI;
    
  //Knee joint
  k1 = 0.;
  k2 = 2. * L1 * L2;
  k3 = pow(X,2.) + pow(A,2.) - pow(L1,2.) - pow(L2,2.);
  kk_aux = pow(k2,2.) - pow(k3,2.);

  if(kk_aux < 0.)
  {
    req_angle[0] = 1000.;
    req_angle[1] = 1000.;
    req_angle[2] = 1000.;
    return;
  }
  q3 = atan2(sqrt(pow(k2,2.) - pow(k3,2.)), k3);
  
  //cout<<"THETA1: "<<q1*(180. / PI)<<" THETA2:"<<q2*(180. / PI)<<" THETA3:"<<q3*(180. / PI)<<endl;
  
  if(!isnan(q1 * q2 * q3))
  {
    
//     //adjust to servo tolerance
//     if(q1 < (robot_joint_limits.ankle_min_inversion *PI/180.) && q1 > ((robot_joint_limits.ankle_min_inversion-5.)*PI/180. + 0.0034))
//       q1=robot_joint_limits.ankle_min_inversion*PI/180.;
//     else if(q1 < (((robot_joint_limits.ankle_max_inversion+5.)*PI/180.)+0.0034) && q1 > (robot_joint_limits.ankle_max_inversion*PI/180.))
//       q1=robot_joint_limits.ankle_max_inversion*PI/180.;
    //check joint limits
    if(q1 >=(robot_joint_limits.ankle_min_inversion *PI/180.) && q1 <= (robot_joint_limits.ankle_max_inversion*PI/180.))
      req_angle[0]=q1*180./PI;
    else
      req_angle[0] = 1000.;
    
//     //adjust to servo tolerance
//     if(q2 < (robot_joint_limits.ankle_min_flexion *PI/180.) && q2 > ((robot_joint_limits.ankle_min_flexion-5.)*PI/180. + 0.0034))
//       q2=robot_joint_limits.ankle_min_flexion*PI/180.;
//     else if(q2 < (((robot_joint_limits.ankle_max_flexion+5.)*PI/180.)+0.0034) && q2 > (robot_joint_limits.ankle_max_flexion*PI/180.))
//       q2=robot_joint_limits.ankle_max_flexion*PI/180.;
    //check joint limits
    if(q2 >=(robot_joint_limits.ankle_min_flexion *PI/180.) && q2 <= (robot_joint_limits.ankle_max_flexion*PI/180.))
      req_angle[1]=q2*180./PI;
    else
      req_angle[1] = 1000.;
    
//     //adjust to servo tolerance
//     if(q3 < (robot_joint_limits.knee_min_flexion *PI/180.) && q3 > ((robot_joint_limits.knee_min_flexion-5.)*PI/180. + 0.0034))
//       q1=robot_joint_limits.knee_min_flexion;
//     else if(q3 < (((robot_joint_limits.knee_max_flexion+5.)*PI/180.)+0.0034) && q3 > (robot_joint_limits.knee_max_flexion*PI/180.))
//       q3=robot_joint_limits.knee_max_flexion*PI/180.;
    //check joint limits
    if(q3 >=robot_joint_limits.knee_min_flexion && q3 <= (robot_joint_limits.knee_max_flexion*PI/180.))
      req_angle[2]=q3*180./PI;
    else
      req_angle[2] = 1000.;
    
    
//         req_angle[0]=q1*180./PI;
//     req_angle[1]=q2*180./PI;
//     req_angle[2]=q3*180./PI;
    
//     cout<<"THETA1: "<<req_angle[0]<<" THETA2:"<<req_angle[1]<<" THETA3:"<<req_angle[2]<<endl;

    return;
  }
  else
  {
    req_angle[0] = 1000.;
    req_angle[1] = 1000.;
    req_angle[2] = 1000.;
    return;
  }
}

void humanoid::Calculate_Detached_Leg_3DOF_Jacobian(double theta_1, double theta_2, double theta_3, Eigen::Matrix3d *jacobian)
{
   //vars
  double L1 = robot_dimensions.shin_length;
  double L2 = robot_dimensions.thigh_length;
  
  double theta1=theta_1*PI/180.;
  double theta2=theta_2*PI/180.;
  double theta3=theta_3*PI/180.;
  
  //place coefficients in jacobian matrix
  Eigen::Matrix3d jacobian_temp;
  
  jacobian_temp << 0., L1*cos(theta2)+L2*cos(theta2+theta3), L2*cos(theta2+theta3),
  
		   -L1*cos(theta1)*cos(theta2)-L2*cos(theta1)*cos(theta2+theta3), L1*sin(theta1)*sin(theta2)+L2*sin(theta1)*sin(theta2+theta3), L2*sin(theta1)*sin(theta2+theta3),
		   
		   -L1*sin(theta1)*cos(theta2)-L2*sin(theta1)*cos(theta2+theta3), -L1*cos(theta1)*sin(theta2)-L2*cos(theta1)*sin(theta2+theta3), -L2*cos(theta1)*sin(theta2+theta3);
		   
  //replace matrix
  memcpy(jacobian,&jacobian_temp,sizeof(jacobian_temp));
}

void humanoid::Calculate_Detached_Leg_3DOF_COG(double theta_1, double theta_2, double theta_3, int SIDE, Eigen::Vector3d *COG)
{
  static Eigen::Vector3d COM_shin;
  static Eigen::Vector3d COM_thigh;
  static Eigen::Vector3d leg_COG;
  static double cog_x;
  static double cog_y;
  static double cog_z;
  
  //vars
  double L1 = robot_dimensions.shin_length;
  double theta1 = theta_1*PI/180.;
  double theta2 = theta_2*PI/180.;
  double theta3 = theta_3*PI/180.;
  
  //LEFT
  if(SIDE == LEFT)
  {
    //shin
    cog_x = robot_relative_COM.shin_COM[0];
    cog_y = robot_relative_COM.shin_COM[1];
    cog_z = robot_relative_COM.shin_COM[2];
    
    COM_shin = Eigen::Vector3d(cog_x*cos(theta2) + cog_z*sin(theta2),
			      cog_x*sin(theta1)*sin(theta2) + cog_y*cos(theta1) - cog_z*sin(theta1)*cos(theta2),
			      -cog_x*cos(theta1)*sin(theta2) + cog_y*sin(theta1) + cog_z*cos(theta1)*cos(theta2) + robot_dimensions.ankle_height);
    
    //thigh
    cog_x = robot_relative_COM.thigh_COM[0];
    cog_y = robot_relative_COM.thigh_COM[1];
    cog_z = robot_relative_COM.thigh_COM[2];
    
    COM_thigh = Eigen::Vector3d(cog_x*(cos(theta2)*cos(theta3)+sin(theta2)*sin(theta3)) - cog_z*(cos(theta2)*sin(theta3) - sin(theta2)*cos(theta3)) + L1*sin(theta2),
					      
				cog_x*sin(theta1)*(cos(theta2)*sin(theta3) - sin(theta2)*cos(theta3)) + cog_y*cos(theta1) + (cog_z*(cos(theta2)*cos(theta3) + sin(theta2)*sin(theta3)) + L1*cos(theta2))*sin(theta1),
					      
				cog_x*cos(theta1)*(cos(theta2)*sin(theta3) - sin(theta2)*cos(theta3)) - cog_y*sin(theta1) + cog_z*cos(theta1)*(cos(theta2)*cos(theta3) + sin(theta2)*sin(theta3)) + L1*cos(theta1)*cos(theta2) + robot_dimensions.ankle_height);
    
    // calculate leg COG
    leg_COG = (1./(robot_element_mass.foot_mass + robot_element_mass.shin_mass + robot_element_mass.thigh_mass)) *
	      ((robot_element_mass.foot_mass)*robot_relative_COM.foot_COM + (robot_element_mass.shin_mass)*COM_shin + (robot_element_mass.thigh_mass)*COM_thigh);
    
    //replace matrix
    memcpy( COG, &leg_COG, sizeof(leg_COG));
    return;
  }
  else if(SIDE == RIGHT)
  {
    //foot
    Eigen::Vector3d COM_foot = Eigen::Vector3d(robot_relative_COM.foot_COM[0],robot_relative_COM.foot_COM[1],robot_relative_COM.foot_COM[2]);
    
    //shin
    cog_x = robot_relative_COM.shin_COM[0];
    cog_y = robot_relative_COM.shin_COM[1];
    cog_z = robot_relative_COM.shin_COM[2];
    
    COM_shin = Eigen::Vector3d(cog_x*cos(theta2) + cog_z*sin(theta2),
			      cog_x*sin(theta1)*sin(theta2) + cog_y*cos(theta1) - cog_z*sin(theta1)*cos(theta2),
			      -cog_x*cos(theta1)*sin(theta2) + cog_y*sin(theta1) + cog_z*cos(theta1)*cos(theta2) + robot_dimensions.ankle_height);
    
    //thigh
    cog_x = robot_relative_COM.thigh_COM[0];
    cog_y = robot_relative_COM.thigh_COM[1];
    cog_z = robot_relative_COM.thigh_COM[2];
    
    COM_thigh = Eigen::Vector3d(cog_x*(cos(theta2)*cos(theta3)+sin(theta2)*sin(theta3)) - cog_z*(cos(theta2)*sin(theta3) - sin(theta2)*cos(theta3)) + L1*sin(theta2),
					      
				cog_x*sin(theta1)*(cos(theta2)*sin(theta3) - sin(theta2)*cos(theta3)) + cog_y*cos(theta1) + (cog_z*(cos(theta2)*cos(theta3) + sin(theta2)*sin(theta3)) + L1*cos(theta2))*sin(theta1),
					      
				cog_x*cos(theta1)*(cos(theta2)*sin(theta3) - sin(theta2)*cos(theta3)) - cog_y*sin(theta1) + cog_z*cos(theta1)*(cos(theta2)*cos(theta3) + sin(theta2)*sin(theta3)) + L1*cos(theta1)*cos(theta2) + robot_dimensions.ankle_height);

    // calculate leg COG
    leg_COG = (1./(robot_element_mass.foot_mass + robot_element_mass.shin_mass + robot_element_mass.thigh_mass)) *
	      ((robot_element_mass.foot_mass)*COM_foot + (robot_element_mass.shin_mass)*COM_shin + (robot_element_mass.thigh_mass)*COM_thigh);
    
    //replace matrix
    memcpy( COG, &leg_COG, sizeof(leg_COG));
    return;
  }
  else
  {
    return;
  }
}

short unsigned int humanoid::Head_Pan_Tilt_Conversion(double joint_angle)
{
  //   Motion range [0...10] vertical to tilted
  //   Equivalent servo range [2300...2400]
  
  // Check range of request
  if(joint_angle<robot_joint_limits.head_min_tilt || joint_angle>robot_joint_limits.head_max_tilt)
  {
    return 0;
  }
  
  //compute servo range output  
  return (robot_servo_conversion.head_servo_offset-(joint_angle*10.));
}

short unsigned int humanoid::Shoulder_Flexion_Extension_Conversion(short int SIDE, double joint_angle)
{
  // Check range of request
  if(joint_angle<robot_joint_limits.shoulder_min_flexion || joint_angle>robot_joint_limits.shoulder_max_flexion)
  {
    return 0;
  }
  
  double offset;
  double sign;
  if(SIDE==LEFT)
  {
    // LEFT
    //   Motion range [-40...140] back to front
    //   Equivalent servo range [600...2400]
    offset=robot_servo_conversion.shoulder_flexion_servo_offset_left;
    sign=(double) LEFT;
  }
  else if(SIDE==RIGHT)
  {
    // RIGHT
    //   Motion range [-40...140] back to front
    //   Equivalent servo range [2400...600]
    offset=robot_servo_conversion.shoulder_flexion_servo_offset_right;
    sign=(double) RIGHT;
  }
  else
  {
    return 0;
  }
  
  //compute servo range output
  return (offset+sign*joint_angle*10.);
}

short unsigned int humanoid::Shoulder_Abduction_Adduction_Conversion(short int SIDE, double joint_angle)
{
  double offset;
  double sign;
  if(SIDE==LEFT)
  {
    // LEFT
    //   Motion range [0...180] back to front
    //   Equivalent servo range [600...2400]
    offset=robot_servo_conversion.shoulder_abduction_servo_offset_left;
    sign=(double) LEFT;
  }
  else if(SIDE==RIGHT)
  {
    // RIGHT
    //   Motion range [0...180] back to front
    //   Equivalent servo range [2400...600]
    offset=robot_servo_conversion.shoulder_abduction_servo_offset_right;
    sign=(double) RIGHT;
  }
  else
  {
    return 0;
  }

    // Check range of request
  if(joint_angle<robot_joint_limits.shoulder_min_abduction || joint_angle>robot_joint_limits.shoulder_max_abduction)
  {
    return 0;
  }
  
  //compute servo range output
  return (offset+sign*joint_angle*10.);
}

short unsigned int humanoid::Elbow_Flexion_Extension_Conversion(short int SIDE, double joint_angle)
{
  double offset;
  double sign;
  if(SIDE==LEFT)
  {
    // LEFT
    //   Motion range [0...120] streched to flected
    //   Equivalent servo range [2400...1200]
    offset=robot_servo_conversion.elbow_flexion_servo_offset_left;
    sign=(double) RIGHT; /* <-- NOT TO WORRY, IT'S CORRECT...*/
  }
  else if(SIDE==RIGHT)
  {
    // RIGHT
    //   Motion range [0...120] streched to flected
    //   Equivalent servo range [2400...1200]
    offset=robot_servo_conversion.elbow_flexion_servo_offset_right;
    sign=(double) LEFT; /* <-- NOT TO WORRY, IT'S CORRECT...*/
  }
  else
  {
    return 0;
  }
    
    // Check range of request
  if(joint_angle<robot_joint_limits.elbow_min_flexion || joint_angle>robot_joint_limits.elbow_max_flexion)
  {
    return 0;
  }
  
  //compute servo range output
  return (offset+sign*joint_angle*10.);
}

short unsigned int humanoid::Torso_Rotation_Conversion(double joint_angle)
{
  //   Motion range [-90...90] left to right
  //   Equivalent servo range [600...2400]
  
    // Check range of request
  if(joint_angle<robot_joint_limits.torso_min_rotation || joint_angle>robot_joint_limits.torso_max_rotation)
    return 0;
  
  //compute servo range output
  return (robot_servo_conversion.torso_rotation_servo_offset+joint_angle*10.);
}

short unsigned int humanoid::Torso_Flexion_Extension_Conversion(double joint_angle)
{
  //   Motion range [-15...90] left to right
  //   Equivalent servo range [600...2400]
  
    // Check range of request
  if(joint_angle<robot_joint_limits.torso_min_flexion || joint_angle>robot_joint_limits.torso_max_flexion)
    return 0;
  
  //compute servo range output
  return (robot_servo_conversion.torso_flexion_servo_offset+joint_angle*10.);
}


short unsigned int humanoid::Torso_Lateral_Flexion_Extension_Conversion(double joint_angle)
{
  //   Motion range [-90...90] left to right
  //   Equivalent servo range [600...2400]
  
    // Check range of request
  if(joint_angle<robot_joint_limits.torso_min_lateral_flexion || joint_angle>robot_joint_limits.torso_max_lateral_flexion)
    return 0;
  
  //compute servo range output
  return (robot_servo_conversion.torso_lateral_flexion_servo_offset+joint_angle*10.);
}
	
short unsigned int humanoid::Ankle_Inversion_Eversion_Conversion(short int SIDE, double joint_angle)
{
  // Check range of request
  //   Motion range [-30...45] inside to outside
  if(joint_angle<robot_joint_limits.ankle_min_inversion || joint_angle>robot_joint_limits.ankle_max_inversion)
    return 0;
  
  double sign;
  if(SIDE==LEFT)
  {
    // LEFT
    //   Equivalent servo range [600...2400]
    sign=(double) LEFT;
  }
  else if(SIDE==RIGHT)
  {
    // RIGHT
    //   Equivalent servo range [2400...600]
    sign=(double) RIGHT;
  }
  else
  {
    return 0;
  }
  
  double b=1500.;
  //   Conversion
  if(joint_angle==0)
    return 1500;
  else if(joint_angle<0)
  {
    double m=(1500.-600.)/(-robot_joint_limits.ankle_min_inversion);
    return round(sign*(m*joint_angle)+b);
  }
  else
  {
    double m=(2400.-1500.)/(robot_joint_limits.ankle_max_inversion);
    return round(sign*(m*joint_angle)+b);
  }
}

short unsigned int humanoid::Ankle_Flexion_Conversion(short int SIDE, double joint_angle)
{
  // Check range of request
  if(joint_angle<robot_joint_limits.ankle_min_flexion || joint_angle>robot_joint_limits.ankle_max_flexion)
    return 0;
  
  double sign;
  if(SIDE==LEFT)
  {
    // LEFT
    //   Motion range [-40...20] inside to outside
    //   Equivalent servo range [600...2400]
    sign=(double) LEFT;
  }
  else if(SIDE==RIGHT)
  {
    // RIGHT
    //   Motion range [-40...20] inside to outside
    //   Equivalent servo range [2400...600]
    sign=(double) RIGHT;
  }
  else
    return 0;
  
  double b=1500.;
  if(joint_angle==0)
    return 1500;
  else if(joint_angle<0)
  {
    //   Conversion
    double m=(1500.-600.)/(-robot_joint_limits.ankle_min_flexion);
    return round(sign*(m*joint_angle)+b);
  }
  else
  {
    //   Conversion
    double m=(2400.-1500.)/(robot_joint_limits.ankle_max_flexion);
    return round(sign*(m*joint_angle)+b);
  }
}

short unsigned int humanoid::Knee_Conversion(short int SIDE, double joint_angle)
{
  // Check range of request
  if(joint_angle<robot_joint_limits.knee_min_flexion || joint_angle>robot_joint_limits.knee_max_flexion)
    return 0;
  
//   //detected very high current consumption when knee is bellow 5degrees
//   if(joint_angle<=5.)
//     joint_angle=5.;
  
  double sign;
  double b;
  if(SIDE==LEFT)
  {
    // LEFT
    //   Motion range [0...130] inside to outside
    //   Equivalent servo range [600...2400]
    b=robot_servo_conversion.knee_flexion_b_value_left;
    sign=(double) LEFT;
  }
  else if(SIDE==RIGHT)
  {
    // RIGHT
    //   Motion range [0...130] inside to outside
    //   Equivalent servo range [2400...600]
    b=robot_servo_conversion.knee_flexion_b_value_right;
    sign=(double) RIGHT;
  }
  else
    return 0;
  
  //   Conversion
  double m=sign*(2400.-600.)/(robot_joint_limits.knee_max_flexion-robot_joint_limits.knee_min_flexion);
  return round((m*joint_angle)+b);
}

short unsigned int humanoid::Hip_Abduction_Hiperabduction_Conversion(short int SIDE, double joint_angle)
{
  // this servo has a gain of 40/10 (pulley teeth)
  //TODO: set the angle conversion straight w_a = (N_b/N_a) * w_b
  
  // Check range of request
  if(joint_angle<robot_joint_limits.hip_min_abduction || joint_angle>robot_joint_limits.hip_max_abduction)
    return 0;
  
  double sign;
  if(SIDE==LEFT)
  {
    // LEFT
    //   Motion range [-40...45] inside to outside
    //   Equivalent servo range [2400...600]
    sign=(double) RIGHT;
  }
  else if(SIDE==RIGHT)
  {
    // RIGHT
    //   Motion range [-40...45] inside to outside
    //   Equivalent servo range [2400...600]
    sign=(double) LEFT;
  }
  else
    return 0;
  
  double b=1500.;
  //   Conversion
  if(joint_angle==0)
    return 1500;
  else if(joint_angle<0)
  {
    double m=900./(-robot_joint_limits.hip_min_abduction);
    return round(sign*(m*joint_angle)+b);
  }
  else
  {
    double m=900./(robot_joint_limits.hip_max_abduction);
    return round(sign*(m*joint_angle)+b);
  }
}

short unsigned int humanoid::Hip_Flexion_Conversion(short int SIDE, double joint_angle)
{
  // this servo has a gain of 30/22 (pulley teeth)
  //TODO: set the angle conversion straight
  
  // Check range of request
  if(joint_angle<robot_joint_limits.hip_min_flexion || joint_angle>robot_joint_limits.hip_max_flexion)
    return 0;
  
  double sign;
  double offset;
  if(SIDE==LEFT)
  {
    // LEFT
    //   Motion range [-30...120] inside to outside
    //   Equivalent servo range [600...2400]
    offset=robot_servo_conversion.hip_flexion_b_value_left;
    sign=(double) LEFT;
  }
  else if(SIDE==RIGHT)
  {
    // RIGHT
    //   Motion range [-30...120] inside to outside
    //   Equivalent servo range [2400...600]
    offset=robot_servo_conversion.hip_flexion_b_value_right;
    sign=(double) RIGHT;
  }
  else
    return 0;
  
  //compute servo range output
  return offset+sign*joint_angle*10.;
}

double humanoid::Head_Pan_Tilt_ServoValue_Conversion(short unsigned int servo_value)
{
  //   Motion range [0...10] vertical to tilted
  //   Equivalent servo range [2300...2400]
  
  //check servo range
    if(servo_value<2306 || servo_value>2406)
  {
    return 0xFFFF;
  }
  
  //compute servo range output  
  return (2400.-(double)servo_value)/10.;
}

double humanoid::Shoulder_Flexion_Extension_ServoValue_Conversion(short int SIDE, short unsigned int servo_value)
{
  if(servo_value<595 || servo_value>2405)
  {
    return 0xFFFF;
  }
  double offset;
  double sign;
  if(SIDE==LEFT)
  {
    // LEFT
    //   Motion range [-40...140] back to front
    //   Equivalent servo range [600...2400]
      //check servo range
    offset=robot_servo_conversion.shoulder_flexion_servo_offset_left;
    sign=(double) LEFT;
  }
  else if(SIDE==RIGHT)
  {
    // RIGHT
    //   Motion range [-40...140] back to front
    //   Equivalent servo range [2400...600]
    offset=robot_servo_conversion.shoulder_flexion_servo_offset_right;
    sign=(double) RIGHT;
  }
  else
  {
    return 0xFFFF;
  }
  
  //compute joint angle output
  return sign*((double)servo_value-offset)/10.;
}

double humanoid::Shoulder_Abduction_Adduction_ServoValue_Conversion(short int SIDE, short unsigned int servo_value)
{
  if(servo_value<595 || servo_value>2405)
  {
    return 0xFFFF;
  } 
  double offset;
  double sign;
  if(SIDE==LEFT)
  {
    // LEFT
    //   Motion range [0...180] back to front
    //   Equivalent servo range [600...2400]
    offset=robot_servo_conversion.shoulder_abduction_servo_offset_left;
    sign=(double) LEFT;
  }
  else if(SIDE==RIGHT)
  {
    // RIGHT
    //   Motion range [0...180] back to front
    //   Equivalent servo range [2400...600]
    offset=robot_servo_conversion.shoulder_abduction_servo_offset_right;
    sign=(double) RIGHT;
  }
  else
  {
    return 0xFFFF;
  }
  
  //compute joint angle output
  return sign*((double)servo_value-offset)/10.;
}

double humanoid::Elbow_Flexion_Extension_ServoValue_Conversion(short int SIDE, short unsigned int servo_value)
{
  if(servo_value<595 || servo_value>2405)
  {
    return 0xFFFF;
  }
  double offset;
  double sign;
  if(SIDE==LEFT)
  {
    // LEFT
    //   Motion range [0...120] streched to flected
    //   Equivalent servo range [2400...1200]
    offset=robot_servo_conversion.elbow_flexion_servo_offset_left;
    sign=(double) RIGHT; /* <-- NOT TO WORRY, IT'S CORRECT...*/
  }
  else if(SIDE==RIGHT)
  {
    // RIGHT
    //   Motion range [0...120] streched to flected
    //   Equivalent servo range [2400...1200]
    offset=robot_servo_conversion.elbow_flexion_servo_offset_right;
    sign=(double) LEFT; /* <-- NOT TO WORRY, IT'S CORRECT...*/
  }
  else
  {
    return 0xFFFF;
  }
  
  //compute joint angle output
  return sign*((double)servo_value-offset)/10.;
}

double humanoid::Torso_Rotation_ServoValue_Conversion(short unsigned int servo_value)
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

double humanoid::Torso_Flexion_Extension_ServoValue_Conversion(short unsigned int servo_value)
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

double humanoid::Torso_Lateral_Flexion_Extension_ServoValue_Conversion(short unsigned int servo_value)
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

double humanoid::Ankle_Inversion_Eversion_ServoValue_Conversion(short int SIDE, short unsigned int servo_value)
{
  // Check range of request
    if(servo_value<595 || servo_value>2405)
  {
    return 0xFFFF;
  }
  //   Motion range [-30...45] inside to outside
  
  if(servo_value==1500)
    return 0.;
  
  if(SIDE==RIGHT)
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
  else if(SIDE==LEFT)
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
    
double humanoid::Ankle_Flexion_ServoValue_Conversion(short int SIDE, short unsigned int servo_value)
{
  // Check range of request
    if(servo_value<595 || servo_value>2405)
  {
    return 0xFFFF;
  }
  //   Motion range [-40...20] inside to outside
  
  if(servo_value==1500)
    return 0.;
  
  if(SIDE==RIGHT)
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
  else if(SIDE==LEFT)
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

double humanoid::Knee_Flexion_ServoValue_Conversion(short int SIDE, short unsigned int servo_value)
{
  // Check range of request
    if(servo_value<595 || servo_value>2405)
  {
    return 0xFFFF;
  }
  
  double sign;
  if(SIDE==LEFT)
  {
    // LEFT
    //   Motion range [0...130] inside to outside
    //   Equivalent servo range [600...2400]
    sign=(double) LEFT;
    return ((double)servo_value-robot_servo_conversion.knee_flexion_b_value_left)/(sign*(2400.-600.)/(robot_joint_limits.knee_max_flexion-robot_joint_limits.knee_min_flexion));
  }
  else if(SIDE==RIGHT)
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

double humanoid::Hip_Abduction_Hiperabduction_ServoValue_Conversion(short int SIDE, short unsigned int servo_value)
{
  // Check range of request
  if(servo_value<595 || servo_value>2405)
  {
    return 0xFFFF;
  }
  
  if(servo_value==1500)
    return 0.;
  
  double sign;
  if(SIDE==LEFT)
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
  else if(SIDE==RIGHT)
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
    
double humanoid::Hip_Flexion_ServoValue_Conversion(short int SIDE, short unsigned int servo_value)
{
  // Check range of request
  if(servo_value<595 || servo_value>2405)
  {
    return 0xFFFF;
  }
  
  double sign;
  double offset;
  if(SIDE==LEFT)
  {
    // LEFT
    //   Motion range [-30...120] inside to outside
    //   Equivalent servo range [600...2400]
    offset=robot_servo_conversion.hip_flexion_b_value_left;
    sign=(double) LEFT;
    if(servo_value==offset)
      return 0.;
  }
  else if(SIDE==RIGHT)
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

/* ++++++++++++++++++++++++++++++++++++++++++
    +++++++++ GET ROBOT DIMENSIONS ++++++++++
    ++++++++++++++++++++++++++++++++++++++++++*/
double humanoid::GetRobotDimension(int dimension)
{
  switch (dimension)
  {
    case ARM_LENGTH:
      return robot_dimensions.arm_length;
    case FOREARM_LENGTH:
      return robot_dimensions.forearm_length;
    case LEG_LENGTH:
      return robot_dimensions.shin_length;
    case THIGH_LENGTH:
      return robot_dimensions.thigh_length;
    case FOOT_LENGTH:
      return robot_dimensions.foot_length;
    case ANKLE_HEIGHT:
      return robot_dimensions.ankle_height;
    case FOOT_WIDTH:
      return robot_dimensions.foot_width;
    default:
      return FP_NAN;
  }
}




