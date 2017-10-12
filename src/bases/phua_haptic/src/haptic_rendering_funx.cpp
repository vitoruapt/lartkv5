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
 * \brief Haptic rendering functions implementation
 */

#include <phua_haptic/haptic_rendering_funx.h>

hduVector3Dd CalculateWorkspaceDemoForce(void *pUserData)
{
  shared_vars_t*RobotVars=(shared_vars_t*)pUserData;
  //vectors
  hduVector3Dd forceDirection;
  //force vector initialization
  hduVector3Dd sent_force(0.0,0.0,0.0);
  //joystick position;
  static hduVector3Dd phantom_position;
  
  
  // ***** Calculations *****
  if( (RobotVars->parameters.kinematic_model>=1 && RobotVars->parameters.kinematic_model<=3) && RobotVars->parameters.manual_speed_control)
  {
    // *** ARMS ***    
    //inside sphere properties
    double min_sphereRadius = ((double)sqrt(13327.) / (double)RobotVars->parameters.pos_coord_scale); // mm 
    RobotVars->haptics_data.arm_main_sphere_min_Radius = min_sphereRadius;
    //outside sphere properties
    double max_sphereRadius = (RobotVars->humanoid_f->GetRobotDimension(ARM_LENGTH)+RobotVars->humanoid_f->GetRobotDimension(FOREARM_LENGTH))
			      /
			      ((double)RobotVars->parameters.pos_coord_scale); // mm
    RobotVars->haptics_data.arm_main_sphere_max_Radius = max_sphereRadius;
    //back sphere properties
    double back_sphereRadius = RobotVars->humanoid_f->GetRobotDimension(FOREARM_LENGTH) / ((double)RobotVars->parameters.pos_coord_scale); // mm
    RobotVars->haptics_data.arm_back_sphereRadius = back_sphereRadius;
    
    
    // center of the main sphere
    hduVector3Dd main_spheresPosition = RobotVars->haptics_data.arm_main_spheresPosition;
    // center of the back sphere
    hduVector3Dd back_spheresPosition = RobotVars->haptics_data.arm_back_spherePosition;
    
    //phantom position
    phantom_position = RobotVars->phantom_data.m_devicePosition;
    
    //force direction is constant in Y axle / joystick X axle
    if(RobotVars->parameters.kinematic_model==1)
    {
      forceDirection = TransformWorldCoordinatesToPHaNToMCoordinates(hduVector3Dd(0.,-1.,0.));
      // YY PLANE FORCE
      sent_force = sent_force + (CalculatePolyForceMagnitude(-(phantom_position[1] - main_spheresPosition[1])/ (double)RobotVars->parameters.pos_coord_scale) * forceDirection);
    }
    else
    {
      forceDirection = TransformWorldCoordinatesToPHaNToMCoordinates(hduVector3Dd(0.,1.,0.));
      // YY PLANE FORCE
      sent_force = sent_force + (CalculatePolyForceMagnitude((phantom_position[1] - main_spheresPosition[1])/ (double)RobotVars->parameters.pos_coord_scale) * forceDirection);
    }
    
    // MAIN SPHERES:
    double radius = (phantom_position - main_spheresPosition).magnitude();
    // SMALL CENTER SPHERE
    forceDirection = TransformWorldCoordinatesToPHaNToMCoordinates(((phantom_position - main_spheresPosition)/radius));
    sent_force = sent_force + ((CalculatePolyForceMagnitude(radius - min_sphereRadius)) * forceDirection);
    // BIG CENTER SPHERE
    forceDirection = TransformWorldCoordinatesToPHaNToMCoordinates((-1. * (phantom_position - main_spheresPosition)/radius));
    sent_force = sent_force + ((CalculatePolyForceMagnitude(max_sphereRadius - radius)) * forceDirection);
    // BACK SPHERE
    radius = (phantom_position - back_spheresPosition).magnitude();
    forceDirection = TransformWorldCoordinatesToPHaNToMCoordinates(((phantom_position - back_spheresPosition)/radius));
    sent_force = sent_force + ((CalculatePolyForceMagnitude(radius - back_sphereRadius)) * forceDirection);
  }
  else if((RobotVars->parameters.kinematic_model>=4 && RobotVars->parameters.kinematic_model<=5) && RobotVars->parameters.manual_speed_control)
  {
    //LEGS
    //outside sphere properties
    double max_sphereRadius = ((RobotVars->humanoid_f->GetRobotDimension(THIGH_LENGTH)
				+
				RobotVars->humanoid_f->GetRobotDimension(LEG_LENGTH)) / (double)RobotVars->parameters.pos_coord_scale); // mm
    RobotVars->haptics_data.leg_main_sphere_max_Radius = max_sphereRadius;
    //smaller sphere properties
    double min_sphereRadius = ((double)119.233 / (double)RobotVars->parameters.pos_coord_scale); // mm
    RobotVars->haptics_data.leg_main_sphere_min_Radius = min_sphereRadius;
    //front small sphere properties
    double min_front_sphereRadius = (RobotVars->humanoid_f->GetRobotDimension(LEG_LENGTH) / (double)RobotVars->parameters.pos_coord_scale); // mm
    RobotVars->haptics_data.leg_back_sphereRadius = min_front_sphereRadius;
    //back outer sphere properties
    double max_back_sphereRadius = (RobotVars->humanoid_f->GetRobotDimension(LEG_LENGTH) / (double)RobotVars->parameters.pos_coord_scale); // mm
    if(max_back_sphereRadius){}
    
    phantom_position = RobotVars->phantom_data.m_devicePosition;
    
    // centers of the spheres
    hduVector3Dd main_spheresPosition = RobotVars->haptics_data.leg_main_spheresPosition;
    hduVector3Dd back_spheresPosition = RobotVars->haptics_data.leg_back_spherePosition;
    hduVector3Dd front_spheresPosition = RobotVars->haptics_data.leg_front_spherePosition;
    
    // OUTER SPHERE:
    double radius = (phantom_position - main_spheresPosition).magnitude();
    forceDirection = TransformWorldCoordinatesToPHaNToMCoordinates((-1. * (phantom_position - main_spheresPosition)/radius));
    sent_force = sent_force + ((CalculatePolyForceMagnitude(max_sphereRadius - radius)) * forceDirection);
    // INNER SPHERE
    forceDirection = TransformWorldCoordinatesToPHaNToMCoordinates(((phantom_position - main_spheresPosition)/radius));
    sent_force = sent_force + ((CalculatePolyForceMagnitude(radius - min_sphereRadius)) * forceDirection);
    //PLANES
    //vector u
    hduVector3Dd u = hduVector3Dd(1.0,0.0,0.0);
    //vector v30
    hduVector3Dd v30 = hduVector3Dd(0.0,-sin(DegToRad(30.0)),cos(DegToRad(30.0)));
    //vector v45
    hduVector3Dd v45 = hduVector3Dd(0.0,sin(DegToRad(45.0)),cos(DegToRad(45.0)));    

//     if(RobotVars->parameters.kinematic_model==4)
//     {
//       //LEFT LEG
//       
//       
//     }
//     else
//     {
//       //RIGHT LEG
//       // vector normal to plane on the left
//       hduVector3Dd n_left = crossProduct(u,v30);
//       // equation of a plane s = a*x + b*y + c*z + d
//       double d = dotProduct( -main_spheresPosition + hduVector3Dd(0.0,-sin(DegToRad(30.0)),cos(DegToRad(30.0))), n_left );
//       n_left = -n_left;
//       //distance to the plane s
//       double s = (n_left[0] * phantom_position[0] + n_left[1] * phantom_position[1] + n_left[2] * phantom_position[2] + d) / sqrt(pow(n_left[0],2.) + pow(n_left[1],2.) + pow(n_left[2],2.));
//       s = s / (double)RobotVars->parameters.pos_coord_scale;
//       std::cout<<"DISTANCE TO PLANE"<<s<<std::endl;
//       n_left.normalize();
//       forceDirection = n_left;
//       sent_force = sent_force + ((CalculatePolyForceMagnitude(s)) * forceDirection);
      
//       // vector normal to plane on the right
//       hduVector3Dd n_right = crossProduct(u,v45);
//       // equation of a plane s = a*x + b*y + c*z + d
//       d = dotProduct( -hduVector3Dd(0.0,sin(DegToRad(45.0)),cos(DegToRad(45.0))), n_right );
//       //distance to the plane s
//       s = (n_right[0] * phantom_position[0] + n_right[1] * phantom_position[1] + n_right[2] * phantom_position[2] + d) / sqrt(pow(n_right[0],2.) + pow(n_right[1],2.) + pow(n_right[2],2.));
//       s = s / (double)RobotVars->parameters.pos_coord_scale;
//       n_right.normalize();
//       forceDirection = n_right;
//       sent_force = sent_force + ((CalculatePolyForceMagnitude(s)) * forceDirection);
//     }
//     // FRONT SPHERE
//     radius = (phantom_position - front_spheresPosition).magnitude();
//     forceDirection = TransformWorldCoordinatesToPHaNToMCoordinates(((phantom_position - front_spheresPosition)/radius));
//     sent_force = sent_force + ((CalculatePolyForceMagnitude(radius - min_front_sphereRadius)) * forceDirection);
  }
  else
  {
    //OTHERS
  }
  
  return (sent_force);
}

hduVector3Dd CalculatePlaneDrawingDemoForce(void *pUserData)
{
  shared_vars_t*RobotVars=(shared_vars_t*)pUserData;
  
  hduVector3Dd joystick_position = RobotVars->phantom_data.m_devicePosition;
  
  HDdouble s = RobotVars->haptics_data.demo_2_Plane.perpDistance(joystick_position);
  
  RobotVars->haptics_data.demo2_distance_to_plane  = fabs((double)s / (double)RobotVars->parameters.pos_coord_scale);
  hduVector3Dd n = RobotVars->haptics_data.demo_2_Plane.normal();
  n.normalize();
  
  // force calculation
  return TransformWorldCoordinatesToPHaNToMCoordinates(CalculatePlaneForceMagnitude(RobotVars->haptics_data.demo2_distance_to_plane) * -n);
}

void Demo2_BuildPlane(void *pUserData)
{
  shared_vars_t*RobotVars=(shared_vars_t*)pUserData;
  // use plane class from openhaptics
  RobotVars->haptics_data.demo_2_Plane = hduPlaned(RobotVars->haptics_data.demo_2_point_1,
						   RobotVars->haptics_data.demo_2_point_2,
						   RobotVars->haptics_data.demo_2_point_3);  
}

hduVector3Dd Leg_COP_Monitoring(void *pUserData)
{
  shared_vars_t*RobotVars=(shared_vars_t*)pUserData;
  hduVector3Dd Fx,Fy;
  double x,y;
  static double x_border_p, x_border_m, y_border_p, y_border_m;
  
  if(RobotVars->parameters.kinematic_model==4)
  {
    x = RobotVars->robot_kin_data.COG_detached_leg_left[0];
    y = RobotVars->robot_kin_data.COG_detached_leg_left[1];    
  }
  else if(RobotVars->parameters.kinematic_model==5)
  {
    x = RobotVars->robot_kin_data.COG_detached_leg_right[0];
    y = RobotVars->robot_kin_data.COG_detached_leg_right[1];
  }
  else
    return hduVector3Dd(0.0,0.0,0.0);
  
  std::cout<<"X: "<<x<<std::endl;
  std::cout<<"Y: "<<y<<std::endl;
  
  if( fabs(y) >= 10.0)
  {
    Fy = hduVector3Dd(0.0,-CalculateExponentialForceMagnitude(0.0),0.0);
  }
  if( fabs(x) >= 35.0)
  {
    Fx = hduVector3Dd(CalculateExponentialForceMagnitude(1),0.0,0.0);
  }
  
//   x_border_p = RobotVars->haptics_data.leg_main_spheresPosition[0] + (80.0)/2.0;
//   x_border_m = RobotVars->haptics_data.leg_main_spheresPosition[0] - (80.0)/2.0;
//   y_border_p = RobotVars->haptics_data.leg_main_spheresPosition[1] + (RobotVars->humanoid_f->GetRobotDimension(FOOT_WIDTH))/2.0;
//   y_border_m = RobotVars->haptics_data.leg_main_spheresPosition[1] - (RobotVars->humanoid_f->GetRobotDimension(FOOT_WIDTH))/2.0;
//   
//   double s;
//   //check x
//   s = x_border_p - x;
//   Fx = hduVector3Dd(-1*get_sign(x)*CalculateExponentialForceMagnitude(s),0.0,0.0);
//   s = x-x_border_m;
//   Fx += hduVector3Dd(get_sign(x)*CalculateExponentialForceMagnitude(s),0.0,0.0);
// 
//   //check y
//   s = y_border_p - y;
//   Fy = hduVector3Dd(0.0,-1*get_sign(y)*CalculateExponentialForceMagnitude(s),0.0);
//   s = y-y_border_m;
//   Fy += hduVector3Dd(0.0,get_sign(y)*CalculateExponentialForceMagnitude(s),0.0);
//   
//   std::cout<<Fx+Fy<<std::endl;
  
  return Fx+Fy;
}




