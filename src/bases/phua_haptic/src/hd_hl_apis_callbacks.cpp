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
 * \brief Auxiliary haptic callbacks implementation
 */

#include <phua_haptic/hd_hl_apis_callbacks.h>

using namespace std;

extern DeviceData RawDeviceData;
/*~~~~~~~~~~~~~~~~~ 
||   CALLBACKS    ||
~~~~~~~~~~~~~~~~~~*/

HDCallbackCode HDCALLBACK updateDeviceCallback(void *pUserData)
{
  DeviceData *RawDeviceData=(DeviceData*)pUserData;
  
  int nButtons = 0;
  
  static HDdouble T[16];
  static hduMatrix M;
  
  hdBeginFrame(hdGetCurrentDevice());
  
  /* Retrieve the current button(s). */
  hdGetIntegerv(HD_CURRENT_BUTTONS, &nButtons);
  
  /* In order to get the specific button 1 state, we use a bitmask to test for the HD_DEVICE_BUTTON_1 bit. */
  RawDeviceData->m_button1State = (nButtons & HD_DEVICE_BUTTON_1) ? HD_TRUE : HD_FALSE;
  
  /* In order to get the specific button 2 state, we use a bitmask to test for the HD_DEVICE_BUTTON_2 bit. */
  RawDeviceData->m_button2State = (nButtons & HD_DEVICE_BUTTON_2) ? HD_TRUE : HD_FALSE;
  
  /* Get the current location of the device.*/
  hdGetDoublev(HD_CURRENT_POSITION, RawDeviceData->m_devicePosition); /*milimeters*/
  
  /*Get joint angles*/
  hdGetDoublev(HD_CURRENT_JOINT_ANGLES, RawDeviceData->jointAngles); /*rad*/
  
  /*Get gimbal angles*/
  hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, RawDeviceData->gimbalAngles); /*rad*/
  
  // Get motor temperatures [0...1]
  hdGetDoublev(HD_MOTOR_TEMPERATURE, RawDeviceData->motor_temperature);
  
  // Get transform
  hdGetDoublev(HD_CURRENT_TRANSFORM, T);
  M = hduMatrix(T);
  M.transpose();
  RawDeviceData->end_effector_transform = M;
  
  /* Also check the error state of HDAPI. */
  RawDeviceData->m_error = hdGetError();
  
  /* Copy the position into our device_data structure. */
  hdEndFrame(hdGetCurrentDevice());
  
  return HD_CALLBACK_CONTINUE;    
}

HDCallbackCode HDCALLBACK updateDeviceParametersCallback(void *pUserData)
{
  shared_vars_t*RobotVars=(shared_vars_t*)pUserData;
  
  // read max stifness value from device
  hdGetDoublev(HD_NOMINAL_MAX_STIFFNESS, &(RobotVars->phantom_data.max_stiffness));
  
  // read max damping value from device
  hdGetDoublev(HD_NOMINAL_MAX_DAMPING, &(RobotVars->phantom_data.max_damping));
  
  // read max force value from device
  hdGetDoublev(HD_NOMINAL_MAX_FORCE, &(RobotVars->phantom_data.max_force)); //N
  
  // read max continuous force value from device
  hdGetDoublev(HD_NOMINAL_MAX_CONTINUOUS_FORCE, &(RobotVars->phantom_data.max_continuous_force)); //N

  return HD_CALLBACK_DONE;    
}

HDCallbackCode HDCALLBACK copyDeviceDataCallback(void *pUserData)
{
  shared_vars_t*RobotVars=(shared_vars_t*)pUserData;
  static double rotVals[4][4];
#if ROS_VERSION_MINIMUM(1, 8, 0)   //At least FUERTE, V. Santos, 27-Mai-2013,23:34
  static tf::Quaternion q;
  static tf::Matrix3x3 mat;
#else
  static btQuaternion q;
  static btMatrix3x3 mat;
#endif
  
  if(!RobotVars->parameters.exit_status)
  {
    //alternate button 1 clicked state
    if((bool)RawDeviceData.m_button1State==TRUE && (bool)RobotVars->phantom_data.m_button1State==FALSE)
    {
      RobotVars->phantom_data.m_button1Clicked=!RobotVars->phantom_data.m_button1Clicked;
    }
    
    //alternate button 2 clicked state
    if((bool)RawDeviceData.m_button2State==TRUE && (bool)RobotVars->phantom_data.m_button2State==FALSE)
    {
      RobotVars->phantom_data.m_button2Clicked=!RobotVars->phantom_data.m_button2Clicked;
    }
    
    //check button 1
    if(RawDeviceData.m_button1State!=RobotVars->phantom_data.m_button1State)
    {
      RobotVars->phantom_data.m_button1State=RawDeviceData.m_button1State;
    }
    //check button 2
    if(RawDeviceData.m_button2State!=RobotVars->phantom_data.m_button2State)
    {
      RobotVars->phantom_data.m_button2State=RawDeviceData.m_button2State;
    }
    
    //check position
    if(RawDeviceData.m_devicePosition[0]!=RobotVars->phantom_data.m_devicePosition[0] ||
      RawDeviceData.m_devicePosition[1]!=RobotVars->phantom_data.m_devicePosition[1] ||
      RawDeviceData.m_devicePosition[2]!=RobotVars->phantom_data.m_devicePosition[2])
    {
      //update position
      if(RobotVars->parameters.coordinate_resolution==1.)
      {
	// resolution by milimeters
	RobotVars->phantom_data.m_devicePosition[0]=round(RawDeviceData.m_devicePosition[2]);
	RobotVars->phantom_data.m_devicePosition[1]=round(RawDeviceData.m_devicePosition[0]);
	RobotVars->phantom_data.m_devicePosition[2]=round(RawDeviceData.m_devicePosition[1]);
      }
      else
      {
	// all other resolutions 
	RobotVars->phantom_data.m_devicePosition[0]=RetrievePrecisionFromDouble(RawDeviceData.m_devicePosition[2],RobotVars->parameters.coordinate_resolution);
	RobotVars->phantom_data.m_devicePosition[1]=RetrievePrecisionFromDouble(RawDeviceData.m_devicePosition[0],RobotVars->parameters.coordinate_resolution);
	RobotVars->phantom_data.m_devicePosition[2]=RetrievePrecisionFromDouble(RawDeviceData.m_devicePosition[1],RobotVars->parameters.coordinate_resolution);
      }
    }
    if(RobotVars->parameters.cntrl_pos_back_front<0.)
    {
      RobotVars->phantom_data.m_devicePosition[0] = -RobotVars->phantom_data.m_devicePosition[0];
      RobotVars->phantom_data.m_devicePosition[1] = -RobotVars->phantom_data.m_devicePosition[1];
    }
    
    
    //check joint angles
    if(RawDeviceData.jointAngles[0]!=RobotVars->phantom_data.jointAngles[0] || RawDeviceData.jointAngles[1]!=RobotVars->phantom_data.jointAngles[1] ||RawDeviceData.jointAngles[2]!=RobotVars->phantom_data.jointAngles[2])
    {
      //RobotVars->phantom_data.jointAngles=RawDeviceData.jointAngles * (180./arma::datum::pi);
      //V. Santos, 27-Mai-2013,23:38 replace previous by next
      RobotVars->phantom_data.jointAngles=RawDeviceData.jointAngles * (180./M_PI);
    }
    
    //check gimbal angles
    if(RawDeviceData.gimbalAngles[0]!=RobotVars->phantom_data.gimbalAngles[0] || RawDeviceData.gimbalAngles[1]!=RobotVars->phantom_data.gimbalAngles[1] ||RawDeviceData.gimbalAngles[2]!=RobotVars->phantom_data.gimbalAngles[2])
    {
      //RobotVars->phantom_data.gimbalAngles = RawDeviceData.gimbalAngles * (180./arma::datum::pi);
      //V. Santos, 27-Mai-2013,23:38 replace previous by next
      RobotVars->phantom_data.gimbalAngles = RawDeviceData.gimbalAngles * (180./M_PI);
    }
    
    if(RawDeviceData.motor_temperature[0] != RobotVars->phantom_data.motor_temperature[0] ||
      RawDeviceData.motor_temperature[1] != RobotVars->phantom_data.motor_temperature[1] ||
      RawDeviceData.motor_temperature[2] != RobotVars->phantom_data.motor_temperature[2])
    {
      // motor temperatures
      memcpy(&(RobotVars->phantom_data.motor_temperature),&(RawDeviceData.motor_temperature),sizeof(RawDeviceData.motor_temperature));
    }
    
    // Get transform matrixes
   
    RobotVars->phantom_data.phantom_to_base_point.setOrigin( tf::Vector3( RawDeviceData.m_devicePosition[0], RawDeviceData.m_devicePosition[1], RawDeviceData.m_devicePosition[2]) );

    RawDeviceData.end_effector_transform.get(rotVals);

#if ROS_VERSION_MINIMUM(1, 8, 0)   //At least FUERTE, V. Santos, 27-Mai-2013,23:34
    mat = tf::Matrix3x3(rotVals[0][0], rotVals[0][1], rotVals[0][2],
		      rotVals[1][0], rotVals[1][1], rotVals[1][2],
		      rotVals[2][0], rotVals[2][1], rotVals[2][2]);
#else
    mat = btMatrix3x3(rotVals[0][0], rotVals[0][1], rotVals[0][2],
		      rotVals[1][0], rotVals[1][1], rotVals[1][2],
		      rotVals[2][0], rotVals[2][1], rotVals[2][2]);

#endif
    mat.getRotation(q);

    RobotVars->phantom_data.phantom_to_base_point.setRotation( q );
    
    RobotVars->phantom_data.base_point_to_pen_tip.setOrigin( tf::Vector3( 0,0,-40));

#if ROS_VERSION_MINIMUM(1, 8, 0)   //At least FUERTE, V. Santos, 27-Mai-2013,23:34
    mat = tf::Matrix3x3(1, 0,0,
		      0,1,0,
		      0,0,1);
#else
    mat = btMatrix3x3(1, 0,0,
		      0,1,0,
		      0,0,1);
#endif
    mat.getRotation(q);
    RobotVars->phantom_data.base_point_to_pen_tip.setRotation( q );
    // end of transform matrixes
    
    /* Copy error state of HDAPI. */
    RobotVars->phantom_data.m_error = RawDeviceData.m_error;
  }
  
  return HD_CALLBACK_DONE;
}

HDCallbackCode HDCALLBACK forcefeedbackCallback(void *pUserData)
{
  shared_vars_t*RobotVars=(shared_vars_t*)pUserData;
  static timespec loop_rate_start,loop_rate_stop;
  long double time_in_seconds;
  static int counter=0;
  static hduVector3Dd TOTAL_FORCE_1=hduVector3Dd(0.,0.,0.), TOTAL_FORCE_2=hduVector3Dd(0.,0.,0.);
  static bool first_run = FALSE;
//   static float phantom_pen_apparent_mass = 0.045 + 0.01;
  static hduVector3Dd GRAVITY_COMPENSATION_FORCE = TransformWorldCoordinatesToPHaNToMCoordinates(hduVector3Dd(0,0,(PHANToM_TOOLTIP_MASS * 9.81)));
  
  // variable for error handling
  HDErrorInfo error;
  
  // ****** START OF DEMO 1: WORKSPACE LIMITS ******
  if(RobotVars->haptics_data.chosen_demo == 1 && RobotVars->haptic_loop_start)
  {
    //compute force for distance to workspace limits
    TOTAL_FORCE_1 = CalculateWorkspaceDemoForce(RobotVars);
    
    //apply forces
    if(TOTAL_FORCE_1.magnitude() > 0.)
    {
      hlBeginFrame();
      hdSetDoublev(HD_CURRENT_FORCE, TOTAL_FORCE_1);
      hlEndFrame();
      RobotVars->haptics_data.applied_force = TransformPHANToMCoordinatesToWorldCoordinates(TOTAL_FORCE_1); //NEWTONS
    }
    else
    {
      //gravity compensation
      hlBeginFrame();
      hdSetDoublev(HD_CURRENT_FORCE, GRAVITY_COMPENSATION_FORCE);
      hlEndFrame();
      RobotVars->haptics_data.applied_force = TransformPHANToMCoordinatesToWorldCoordinates(GRAVITY_COMPENSATION_FORCE);
    }
    //error handling
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
	hduPrintError(stderr, &error, "Error during force feedback callback\n");

	if (hduIsSchedulerError(&error))
	{
	    return HD_CALLBACK_DONE;
	}        
    }
    
    //calculate computational frequency
    if(counter<100)
      counter++;
    else
    {
      clock_gettime(CLOCK_REALTIME, &loop_rate_stop);
      time_in_seconds =
      ((long double)loop_rate_stop.tv_sec + ((long double)loop_rate_stop.tv_nsec / (long double)1e9)) -
      ((long double)loop_rate_start.tv_sec + ((long double)loop_rate_start.tv_nsec / (long double)1e9));
      RobotVars->parameters.haptics_rate = round((long double)counter / (time_in_seconds));
      clock_gettime(CLOCK_REALTIME, &loop_rate_start);
      counter = 0;
    }
  }// ****** END OF DEMO 1: WORKSPACE LIMITS ******
  // ****** START OF DEMO 2: DRAWING ******
  else if(RobotVars->haptics_data.chosen_demo == 2 && RobotVars->haptic_loop_start)
  {
    //compute force for distance to workspace limits
    TOTAL_FORCE_1 = CalculateWorkspaceDemoForce(RobotVars);
//     TOTAL_FORCE_1 = 2. * TOTAL_FORCE_1 / 3.;
    
    if(RobotVars->haptics_data.demo_2_point_1.magnitude() > 0. &&
      RobotVars->haptics_data.demo_2_point_2.magnitude() > 0. &&
      RobotVars->haptics_data.demo_2_point_3.magnitude() > 0. &&
      first_run == FALSE)
    {
      std::cout<<"Building Plane...";
      Demo2_BuildPlane(RobotVars);
      std::cout<<"...DONE!"<<std::endl;
      first_run = TRUE;
    }
    else if(RobotVars->haptics_data.demo_2_point_1.magnitude() > 0. &&
      RobotVars->haptics_data.demo_2_point_2.magnitude() > 0. &&
      RobotVars->haptics_data.demo_2_point_3.magnitude() > 0.&&
      first_run == TRUE)
    {
      //compute force for distance to plane
      TOTAL_FORCE_2 = CalculatePlaneDrawingDemoForce(RobotVars);
    }
    else
    {
      //keep waiting for points
      if(RobotVars->phantom_data.m_button2Clicked)
      {
	// pen tip
	// hduVector3Dd joystick_position = RobotVars->phantom_data.m_PenTipPosition;
	// wrist point
	hduVector3Dd joystick_position = RobotVars->phantom_data.m_devicePosition;
	//point tests
	if(RobotVars->haptics_data.demo_2_point_1.magnitude() == 0. &&
	  RobotVars->haptics_data.demo_2_point_2.magnitude() == 0. && 
	  RobotVars->haptics_data.demo_2_point_3.magnitude() == 0.)
	{
	  RobotVars->haptics_data.demo_2_point_1 = joystick_position;
	  cout<<"Setting Point 1 to: "<<RobotVars->haptics_data.demo_2_point_1<<endl;
	  RobotVars->phantom_data.m_button2Clicked=!RobotVars->phantom_data.m_button2Clicked;
	}
	if(RobotVars->haptics_data.demo_2_point_1.magnitude() != 0. &&
	  RobotVars->haptics_data.demo_2_point_2.magnitude() == 0. && 
	  RobotVars->haptics_data.demo_2_point_3.magnitude() == 0. &&
	  RobotVars->haptics_data.demo_2_point_1 != joystick_position)
	{
	  RobotVars->haptics_data.demo_2_point_2 = joystick_position;
	  cout<<"Setting Point 2 to: "<<RobotVars->haptics_data.demo_2_point_2<<endl;
	  RobotVars->phantom_data.m_button2Clicked=!RobotVars->phantom_data.m_button2Clicked;
	}
	if(RobotVars->haptics_data.demo_2_point_1.magnitude() != 0. &&
	  RobotVars->haptics_data.demo_2_point_2.magnitude() != 0. && 
	  RobotVars->haptics_data.demo_2_point_3.magnitude() == 0. &&
	  RobotVars->haptics_data.demo_2_point_1 != joystick_position &&
	  RobotVars->haptics_data.demo_2_point_2 != joystick_position)
	{
	  RobotVars->haptics_data.demo_2_point_3 = joystick_position;
	  cout<<"Setting Point 3 to: "<<RobotVars->haptics_data.demo_2_point_3<<endl;
	  RobotVars->phantom_data.m_button2Clicked=!RobotVars->phantom_data.m_button2Clicked;
	}
      }
      first_run = FALSE;
    }
    
//     cout<<"workspace:"<<TOTAL_FORCE_1<<endl;
//     cout<<"demo 2:"<<TOTAL_FORCE_2<<endl;
    
    //apply forces
    if(TOTAL_FORCE_1.magnitude() > 0. || TOTAL_FORCE_2.magnitude() > 0.)
    {
      hlBeginFrame();
      hdSetDoublev(HD_CURRENT_FORCE, TOTAL_FORCE_1+TOTAL_FORCE_2);
      hlEndFrame();
      RobotVars->haptics_data.applied_force = TransformPHANToMCoordinatesToWorldCoordinates(TOTAL_FORCE_1+TOTAL_FORCE_2); //NEWTONS
    }
    else
    {
      //gravity compensation
      hlBeginFrame();
      hdSetDoublev(HD_CURRENT_FORCE, GRAVITY_COMPENSATION_FORCE);
      hlEndFrame();
      RobotVars->haptics_data.applied_force = TransformPHANToMCoordinatesToWorldCoordinates(GRAVITY_COMPENSATION_FORCE);
    }
    //error handling
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
	hduPrintError(stderr, &error, "Error during force feedback callback\n");

	if (hduIsSchedulerError(&error))
	{
	    return HD_CALLBACK_DONE;
	}        
    }
    
    //calculate computational frequency
    if(counter<100)
      counter++;
    else
    {
      clock_gettime(CLOCK_REALTIME, &loop_rate_stop);
      time_in_seconds =
      ((long double)loop_rate_stop.tv_sec + ((long double)loop_rate_stop.tv_nsec / (long double)1e9)) -
      ((long double)loop_rate_start.tv_sec + ((long double)loop_rate_start.tv_nsec / (long double)1e9));
      RobotVars->parameters.haptics_rate = round((long double)counter / (time_in_seconds));
      clock_gettime(CLOCK_REALTIME, &loop_rate_start);
      counter = 0;
    }
    
    
  }// ****** END OF DEMO 2: DRAWING ******
  // ****** START OF DEMO 3: USER PATH ******
  else if(RobotVars->haptics_data.chosen_demo == 3 && RobotVars->haptic_loop_start)
  {
    //gravity compensation
    hlBeginFrame();
    hdSetDoublev(HD_CURRENT_FORCE, GRAVITY_COMPENSATION_FORCE);
    hlEndFrame();
    RobotVars->haptics_data.applied_force = TransformPHANToMCoordinatesToWorldCoordinates(GRAVITY_COMPENSATION_FORCE);
    //calculate computational frequency
    if(counter<100)
      counter++;
    else
    {
      clock_gettime(CLOCK_REALTIME, &loop_rate_stop);
      time_in_seconds =
      ((long double)loop_rate_stop.tv_sec + ((long double)loop_rate_stop.tv_nsec / (long double)1e9)) -
      ((long double)loop_rate_start.tv_sec + ((long double)loop_rate_start.tv_nsec / (long double)1e9));
      RobotVars->parameters.haptics_rate = round((long double)counter / (time_in_seconds));
      clock_gettime(CLOCK_REALTIME, &loop_rate_start);
      counter = 0;
    }
  }// ****** END OF DEMO 3: USER PATH ******
  // ****** START OF DEMO 4: BALANCING ******
  else if(RobotVars->haptics_data.chosen_demo == 4 && RobotVars->haptic_loop_start)
  {
    //compute force for distance to workspace limits
//     TOTAL_FORCE_1 = CalculateWorkspaceDemoForce(RobotVars);
    hduVector3Dd COP_force = Leg_COP_Monitoring(RobotVars);
    
    if(COP_force.magnitude() > 0.0 || TOTAL_FORCE_1.magnitude() > 0.0)
    {
      hlBeginFrame();
      hdSetDoublev(HD_CURRENT_FORCE, TransformWorldCoordinatesToPHaNToMCoordinates(COP_force) + TOTAL_FORCE_1);
      hlEndFrame();
      RobotVars->haptics_data.applied_force = COP_force + TransformPHANToMCoordinatesToWorldCoordinates(TOTAL_FORCE_1);
    }
    else
    {
      //gravity compensation
      hlBeginFrame();
      hdSetDoublev(HD_CURRENT_FORCE, GRAVITY_COMPENSATION_FORCE);
      hlEndFrame();
      RobotVars->haptics_data.applied_force = TransformPHANToMCoordinatesToWorldCoordinates(GRAVITY_COMPENSATION_FORCE);
    }
    //calculate computational frequency
    if(counter<100)
      counter++;
    else
    {
      clock_gettime(CLOCK_REALTIME, &loop_rate_stop);
      time_in_seconds =
      ((long double)loop_rate_stop.tv_sec + ((long double)loop_rate_stop.tv_nsec / (long double)1e9)) -
      ((long double)loop_rate_start.tv_sec + ((long double)loop_rate_start.tv_nsec / (long double)1e9));
      RobotVars->parameters.haptics_rate = round((long double)counter / (time_in_seconds));
      clock_gettime(CLOCK_REALTIME, &loop_rate_start);
      counter = 0;
    }
  }// ****** END OF DEMO 4: BALANCING ******
  else if(RobotVars->haptics_data.chosen_demo == 0 && RobotVars->haptic_loop_start)
  {
    //gravity compensation
    hlBeginFrame();
    hdSetDoublev(HD_CURRENT_FORCE, GRAVITY_COMPENSATION_FORCE);
    hlEndFrame();
    RobotVars->haptics_data.applied_force = TransformPHANToMCoordinatesToWorldCoordinates(GRAVITY_COMPENSATION_FORCE);
    
    //calculate computational frequency
    if(counter<100)
      counter++;
    else
    {
      clock_gettime(CLOCK_REALTIME, &loop_rate_stop);
      time_in_seconds =
      ((long double)loop_rate_stop.tv_sec + ((long double)loop_rate_stop.tv_nsec / (long double)1e9)) -
      ((long double)loop_rate_start.tv_sec + ((long double)loop_rate_start.tv_nsec / (long double)1e9));
      RobotVars->parameters.haptics_rate = round((long double)counter / (time_in_seconds));
      clock_gettime(CLOCK_REALTIME, &loop_rate_start);
      counter = 0;
    }
  }
  else
  {
    RobotVars->parameters.haptics_rate = 0.;
    counter = 0;
    if(RobotVars->haptics_data.demo_2_point_1.magnitude() != 0.)
    {
    RobotVars->haptics_data.demo_2_point_1 = hduVector3Dd(0,0,0);
    RobotVars->haptics_data.demo_2_point_2 = hduVector3Dd(0,0,0);
    RobotVars->haptics_data.demo_2_point_3 = hduVector3Dd(0,0,0);
    TOTAL_FORCE_1=hduVector3Dd(0.,0.,0.);
    TOTAL_FORCE_2=hduVector3Dd(0.,0.,0.);
    first_run = FALSE;
    }
    RobotVars->haptics_data.applied_force = hduVector3Dd(0,0,0);
  }
  
  
  if(!RobotVars->parameters.exit_status)
  {
    // this assures the callback is run at every servo loop tick
    return HD_CALLBACK_CONTINUE;
  }
  else
  {
    return HD_CALLBACK_DONE;
  }
}

HDCallbackCode HDCALLBACK CalibrationCallback(void *pUserData)
{
  shared_vars_t*RobotVars=(shared_vars_t*)pUserData;

  if(RobotVars->phantom_data.need_update && RobotVars->phantom_data.phantom_on)
  {
    int calibrationStyle= HD_CALIBRATION_INKWELL;

    hdBeginFrame(hdGetCurrentDevice());
    HDenum status = hdCheckCalibration();
    hdEndFrame(hdGetCurrentDevice());
    
    if (status == HD_CALIBRATION_OK)
    {
      printf("PHANToM calibration OK! Will not update.\n");
    }
    else
    {
      hdUpdateCalibration(calibrationStyle);
      printf("PHANToM successfully calibrated!\n");
    }
    RobotVars->phantom_data.need_update=FALSE;
  } 
  // this assures the callback is run at every servo loop tick
  return HD_CALLBACK_CONTINUE;
}
