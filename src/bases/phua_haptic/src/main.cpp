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
 * \brief Main source code, for the humanoid haptics
 */

#include <phua_haptic/main.h>

using namespace std;

/*~~~~~~~~~~~~~~~~~~~~~~~~ 
||      global vars      ||
~~~~~~~~~~~~~~~~~~~~~~~~~*/
/// global variable needed for rapid data collection in the sevo loop thread
DeviceData RawDeviceData;

/*~~~~~~~~~~~~~~~~~ 
||      main      ||
~~~~~~~~~~~~~~~~~~*/
/** 
* @brief Main program function.

This function starts execution of the application.
In here the threads are defined and structures are initialized.
Program call arguments are also checked.
* @param argc counter of call arguments.
* @param argv array with call arguments.
* @return 0 at normal execution, -1 at error.
*/
int main(int argc, char *argv[])
{
  shared_vars_t RobotVars;
  memset(&RobotVars, 0, sizeof(RobotVars));
  
  
  HDSchedulerHandle hUpdateHandle = NULL;
  HDSchedulerHandle hForceHandle = NULL;
  HDSchedulerHandle hCalibrationHandle = NULL;
  HDErrorInfo error;

  //check for inline inputs
  //must be path to USB port
  if(argc>2){
    cout<<"********************************************************************"<<endl;
    cout<<"    Program execution stopped. Please insert only one argument."<<endl;
    cout<<"********************************************************************"<<endl;
    return EXIT_FAILURE;
  }
  else if(argc==1){
    //if no user argument inserted the program attempts standard USB port
    cout<<"***********************************************************************"<<endl;
    cout<<" No argument inserted. Using standard path to USB port [/dev/ttyUSB0]."<<endl;
    cout<<"***********************************************************************"<<endl;
    RobotVars.servo=(hitec_5980SG*) new hitec_5980SG("/dev/ttyUSB0");
    //if port not open
    if(!RobotVars.servo->IsActive())
    {
      cout<<"***********************************************************************"<<endl;
      return EXIT_FAILURE;
    }
    /* Initialize the device, must be done before attempting to call any hd functions. */
    RobotVars.phantom_data.hHD = hdInitDevice(HD_DEFAULT_DEVICE);
    /*check for device error*/
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
      hduPrintError(stderr, &error, "Failed to initialize the device");
      RobotVars.phantom_data.phantom_on=FALSE;
      return EXIT_FAILURE;
    }
    else
    {
      RobotVars.phantom_data.phantom_on=TRUE;
    }
  }
  else{
    // check for other arguments 
    string input_str (argv[1]);
    string dbg_str ("--debug");
    string hlp_str ("--help");
    string dir_str ("/");
    // check for ´--help´
    //check if user placed "--" in the beggining
    if(input_str==hlp_str)
    {
      print_help();
      return EXIT_SUCCESS;
    }
    else if(input_str==dbg_str)
    {
      //check for ´debug´
      // debug argument allows for program execution to run without USB/FireWire connection
      RobotVars.servo=(hitec_5980SG*) new hitec_5980SG("/dev/ttyUSB0"); //no matter if comm not sucessful
      /* Initialize the device, must be done before attempting to call any hd functions. */
      RobotVars.phantom_data.hHD = hdInitDevice(HD_DEFAULT_DEVICE);
      /*check for device error*/
      if (HD_DEVICE_ERROR(error = hdGetError()))
      {
	RobotVars.phantom_data.phantom_on=FALSE;
      }
      else
      {
	RobotVars.phantom_data.phantom_on=TRUE;
      }
    }
    else
    {
      //separate gibberish from unix paths
      if(input_str.find(dir_str)==0)
      {
	//try to open port
	cout<<"*********************************************************"<<endl;
	cout<<"   Using inserted path to USB port ["<<argv[1]<<"]."<<endl;
	cout<<"*********************************************************"<<endl;
	RobotVars.servo=(hitec_5980SG*) new hitec_5980SG(argv[1]);
	if(!RobotVars.servo->IsActive())
	{
	  cout<<"*********************************************************"<<endl;
	  return EXIT_FAILURE;
	}
	/* Initialize the device, must be done before attempting to call any hd functions. */
	RobotVars.phantom_data.hHD = hdInitDevice(HD_DEFAULT_DEVICE);
	/*check for device error*/
	if (HD_DEVICE_ERROR(error = hdGetError()))
	{
	  hduPrintError(stderr, &error, "Failed to initialize the device");
	  RobotVars.phantom_data.phantom_on=FALSE;
	  return EXIT_FAILURE;           
	}
	else
	{
	  RobotVars.phantom_data.phantom_on=TRUE;
	}
      }
      else
      {
	cout<<"******************************************************************"<<endl;
	cout<<" Invalid program syntax call. Use ´--help´ for more information"<<endl;
	cout<<"******************************************************************"<<endl;
	return EXIT_FAILURE;
      }
    }
  }
  
  if(RobotVars.phantom_data.phantom_on)
  {
    // start haptic context
    RobotVars.phantom_data.hHLRC = hlCreateContext(RobotVars.phantom_data.hHD);
    hlMakeCurrent(RobotVars.phantom_data.hHLRC);
    hlDisable(HL_USE_GL_MODELVIEW);// VERY IMPORTANT!!! IF NOT DISABLED PROGRAM SEGFAULTS DUE TO NO REAL USE OF OPENGL PIXEL BUFFER
    
    //variable that is used so store phantom information on the fly
    memset(&RawDeviceData, 0, sizeof(DeviceData));
    // Schedule the main scheduler callback that updates the device state and the haptics callback
    hUpdateHandle = hdScheduleAsynchronous(updateDeviceCallback, &RawDeviceData, HD_MAX_SCHEDULER_PRIORITY);
    hForceHandle = hdScheduleAsynchronous(forcefeedbackCallback, &RobotVars, HD_DEFAULT_SCHEDULER_PRIORITY);
    hCalibrationHandle = hdScheduleAsynchronous(CalibrationCallback, &RobotVars, HD_MIN_SCHEDULER_PRIORITY);

    // Start the servo loop scheduler.
    hdStartScheduler();
    //check for errors
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
	hduPrintError(stderr, &error, "Failed to start the scheduler");
	return EXIT_FAILURE;           
    }
    // run the callback to update device parameters
    hdScheduleSynchronous(updateDeviceParametersCallback,&RobotVars,HD_DEFAULT_SCHEDULER_PRIORITY);
  }
  //clean screen
  ScreenClear();
  //greeting
  print_greeting(&RobotVars);
  
  if(RobotVars.phantom_data.phantom_on)
    cout<<"* OpenHaptics scheduler callbacks running."<<endl;
  
  //init humanoid class
  RobotVars.humanoid_f=(humanoid*) new humanoid();
  
  //parameter initialization
  RobotVars.parameters.exit_status=FALSE;
  RobotVars.parameters.coordinate_resolution=1.;
  RobotVars.parameters.pos_coord_scale = 2.;
  RobotVars.parameters.manual_speed_control=TRUE;
  RobotVars.parameters.automatic_speed_control=FALSE;
  RobotVars.parameters.robot_home_position_base_speed = 15; // [ 1...255 ]
  RobotVars.parameters.automatic_speed_control_freq = 100.; //Hz
  
  RobotVars.update_labels=FALSE;
  RobotVars.parameters.graphics_refresh_rate = 22.; //HZ
  
  RobotVars.phantom_data.need_update=FALSE;
  
  RobotVars.haptics_data.chosen_demo = 0;
  RobotVars.haptics_data.demo_2_point_1 = hduVector3Dd(0,0,0);
  RobotVars.haptics_data.demo_2_point_2 = hduVector3Dd(0,0,0);
  RobotVars.haptics_data.demo_2_point_3 = hduVector3Dd(0,0,0);
  RobotVars.haptics_data.demo_2_Plane = hduPlaned(hduVector3Dd(0,0,0),0.0);
  RobotVars.haptics_data.demo_user_path_point_storing = FALSE;
  RobotVars.haptics_data.demo_user_path_is_run_once = TRUE;
  RobotVars.haptics_data.demo_user_path_run_start = FALSE;
  
  cout<<"* Robot parameters initialized."<<endl;
  
  //ROS INIT ###########################################################
  ros::init(argc, argv, "phua"); // name of the system
  ros::NodeHandle n;
//   ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::MarkerArray>("workspace_sphere", 1);
  
  
  visualization_msgs::MarkerArray markersMsg;
  //   #################################################################
  static tf::TransformBroadcaster broadcaster;
  static tf::TransformListener listener;
  
  //define phantom pre transform
#if ROS_VERSION_MINIMUM(1, 8, 0)   //At least FUERTE, V. Santos, 27-Mai-2013
  tf::Quaternion q;
  tf::Matrix3x3 mat;
  mat = tf::Matrix3x3(0, 0, 1,
		    1,0,0,
		    0, 1, 0); //rotz(90)*rotx(90)
#else
  btQuaternion q;
  btMatrix3x3 mat;
  mat = btMatrix3x3(0, 0, 1,
		    1,0,0,
		    0, 1, 0); //rotz(90)*rotx(90)
#endif

  RobotVars.phantom_data.world_to_phantom.setOrigin( tf::Vector3( 0,0,0 ));

  mat.getRotation(q);
  RobotVars.phantom_data.world_to_phantom.setRotation( q );
  
  cout<<"* ROS components initialized."<<endl;
  
  // mutex
  pthread_mutex_init(&(RobotVars.mutex_gtk),NULL);
  //create thread for handling gtk interface
  pthread_t thread_gtk;
  //run interface
  pthread_create( &thread_gtk, NULL, interface_init, &RobotVars);
  
  //vars used for calculations and checks
  short unsigned int prev_speed = 0;
  hduVector3Dd prev_position;
  double arm_curr_pos[3];
  memset(&arm_curr_pos, 0., sizeof(arm_curr_pos));
  double detached_leg_curr_pos[3];
  memset(&detached_leg_curr_pos, 0., sizeof(detached_leg_curr_pos));
  double hip_abduction_angle;
  hduVector3Dd pos_diff;
  hduVector3Dd instant_diff;
  hduVector3Dd last_position;
  double *avg_speed=new double[3];
  timespec tf,interval_time_i,loop_rate_start,loop_rate_stop;
  long double time_in_seconds;
  long double interval;
  int k_tick=0;
  int counter = 0;

  
  if(RobotVars.phantom_data.phantom_on)
  {
    cout<<"* Main loop started."<<endl;
    // cycle that updates device data using parameter set
    bool exit=RobotVars.parameters.exit_status;
    while(!exit)
    {
      /* Perform a synchronous call to update the most current device state.*/
      hdScheduleSynchronous(copyDeviceDataCallback,&RobotVars,HD_DEFAULT_SCHEDULER_PRIORITY);
      
      prev_position = RobotVars.phantom_data.m_devicePosition;
      
      //calculate pen tip coordinates/velocity
      ROS_CalculatePHANToMPenFrame(&broadcaster,&listener, &RobotVars);
      
      ROS_UpdateMarkers(markersMsg.markers, &RobotVars );
      marker_pub.publish(markersMsg);
      
      //keep setting current time
      clock_gettime(CLOCK_REALTIME, &interval_time_i);
      
      if(RobotVars.haptic_loop_start)
      {
	//entering here means the loop will start so vars and checks must begin
	UpdateJointDataByID(1000, 0., &RobotVars);
	//detect kinematic model
	if(RobotVars.parameters.kinematic_model==1)
	{
	  cout<<" :: Controlling Right Arm ::"<<endl;
	  //retrive current arm end pos;
	  arm_curr_pos[0] = RobotVars.robot_kin_data.X_arm_end_right;
	  arm_curr_pos[1] = RobotVars.robot_kin_data.Y_arm_end_right;
	  arm_curr_pos[2] = RobotVars.robot_kin_data.Z_arm_end_right;
	  //Determine haptic spheres center position
	  RobotVars.haptics_data.arm_main_spheresPosition = (hduVector3Dd)RobotVars.phantom_data.m_devicePosition -
							    ((hduVector3Dd)arm_curr_pos / (double)RobotVars.parameters.pos_coord_scale);
							    
	  RobotVars.haptics_data.arm_back_spherePosition = RobotVars.haptics_data.arm_main_spheresPosition
	  + 
	  (
	  hduVector3Dd(-RobotVars.humanoid_f->GetRobotDimension(ARM_LENGTH)*sin(DegToRad(45.0)),
		       0.0,
		       -RobotVars.humanoid_f->GetRobotDimension(ARM_LENGTH)*cos(DegToRad(45.0)))
	  /
	  (double)RobotVars.parameters.pos_coord_scale
	  );
	}
	else if(RobotVars.parameters.kinematic_model==2)
	{
	  cout<<" :: Controlling Left Arm ::"<<endl;
	  //retrive current arm end pos;
	  arm_curr_pos[0] = RobotVars.robot_kin_data.X_arm_end_left;
	  arm_curr_pos[1] = RobotVars.robot_kin_data.Y_arm_end_left;
	  arm_curr_pos[2] = RobotVars.robot_kin_data.Z_arm_end_left;
	  //Determine haptic spheres center position
	  RobotVars.haptics_data.arm_main_spheresPosition = (hduVector3Dd)RobotVars.phantom_data.m_devicePosition -
	  ((hduVector3Dd)arm_curr_pos / (double)RobotVars.parameters.pos_coord_scale);
	  
	  RobotVars.haptics_data.arm_back_spherePosition = RobotVars.haptics_data.arm_main_spheresPosition
	  + 
	  (
	  hduVector3Dd(-RobotVars.humanoid_f->GetRobotDimension(ARM_LENGTH)*sin(DegToRad(45.0)),
		       0.0,
		       -RobotVars.humanoid_f->GetRobotDimension(ARM_LENGTH)*cos(DegToRad(45.0)))
		       /
		       (double)RobotVars.parameters.pos_coord_scale
		       );
	}
	else if(RobotVars.parameters.kinematic_model==3)
	{
	  cout<<" :: Controlling Both Arms ::"<<endl;
	  //retrive current arm end pos;
	  arm_curr_pos[0] = RobotVars.robot_kin_data.X_arm_end_left;
	  arm_curr_pos[1] = RobotVars.robot_kin_data.Y_arm_end_left;
	  arm_curr_pos[2] = RobotVars.robot_kin_data.Z_arm_end_left;
	  //Determine haptic spheres center position
	  RobotVars.haptics_data.arm_main_spheresPosition = (hduVector3Dd)RobotVars.phantom_data.m_devicePosition -
	  ((hduVector3Dd)arm_curr_pos / (double)RobotVars.parameters.pos_coord_scale);
	  
	  RobotVars.haptics_data.arm_back_spherePosition = RobotVars.haptics_data.arm_main_spheresPosition
	  + 
	  (
	  hduVector3Dd(-RobotVars.humanoid_f->GetRobotDimension(ARM_LENGTH)*sin(DegToRad(45.0)),
		       0.0,
		       -RobotVars.humanoid_f->GetRobotDimension(ARM_LENGTH)*cos(DegToRad(45.0)))
		       /
		       (double)RobotVars.parameters.pos_coord_scale
		       );
	}
	else if(RobotVars.parameters.kinematic_model==4)
	{
	  cout<<" :: Controlling Left Detached Leg ::"<<endl;
	  memcpy(&detached_leg_curr_pos, &RobotVars.robot_kin_data.detached_leg_pos_left, sizeof(RobotVars.robot_kin_data.detached_leg_pos_left));
	  //calc centers
	  RobotVars.haptics_data.leg_main_spheresPosition = (hduVector3Dd)RobotVars.phantom_data.m_devicePosition -
	  (hduVector3Dd(RobotVars.robot_kin_data.detached_leg_pos_left[0], RobotVars.robot_kin_data.detached_leg_pos_left[1], RobotVars.robot_kin_data.detached_leg_pos_left[2]) / (double)RobotVars.parameters.pos_coord_scale);
	  RobotVars.haptics_data.leg_back_spherePosition = RobotVars.haptics_data.leg_main_spheresPosition
							  + 
							  (
							  hduVector3Dd(-RobotVars.humanoid_f->GetRobotDimension(LEG_LENGTH)*cos(DegToRad(40.0)),
									0.0,
									RobotVars.humanoid_f->GetRobotDimension(LEG_LENGTH)*sin(DegToRad(40.0)))
									/
									(double)RobotVars.parameters.pos_coord_scale
							  );
	  RobotVars.haptics_data.leg_front_spherePosition = RobotVars.haptics_data.leg_main_spheresPosition
							  + 
							  (
							  hduVector3Dd(RobotVars.humanoid_f->GetRobotDimension(LEG_LENGTH)*cos(DegToRad(20.0)),
								      0.0,
								      RobotVars.humanoid_f->GetRobotDimension(LEG_LENGTH)*sin(DegToRad(20.0)))
								      /
								      (double)RobotVars.parameters.pos_coord_scale
							  );
	}
	else if(RobotVars.parameters.kinematic_model==5)
	{
	  cout<<" :: Controlling Right Detached Leg ::"<<endl;
	  memcpy(&detached_leg_curr_pos, &RobotVars.robot_kin_data.detached_leg_pos_right, sizeof(RobotVars.robot_kin_data.detached_leg_pos_right));
	  //calc centers
	  RobotVars.haptics_data.leg_main_spheresPosition = (hduVector3Dd)RobotVars.phantom_data.m_devicePosition -
	  (hduVector3Dd(RobotVars.robot_kin_data.detached_leg_pos_right[0], RobotVars.robot_kin_data.detached_leg_pos_right[1], RobotVars.robot_kin_data.detached_leg_pos_right[2]-RobotVars.humanoid_f->GetRobotDimension(ANKLE_HEIGHT)) / (double)RobotVars.parameters.pos_coord_scale);
	  RobotVars.haptics_data.leg_back_spherePosition = RobotVars.haptics_data.leg_main_spheresPosition
	  + 
	  (
	  hduVector3Dd(-RobotVars.humanoid_f->GetRobotDimension(LEG_LENGTH)*cos(DegToRad(40.0)),
		       0.0,
		       RobotVars.humanoid_f->GetRobotDimension(LEG_LENGTH)*sin(DegToRad(40.0)))
		       /
		       (double)RobotVars.parameters.pos_coord_scale
		       );
	  RobotVars.haptics_data.leg_front_spherePosition = RobotVars.haptics_data.leg_main_spheresPosition
		       + 
		       (
		       hduVector3Dd(RobotVars.humanoid_f->GetRobotDimension(LEG_LENGTH)*cos(DegToRad(20.0)),
				    0.0,
				    RobotVars.humanoid_f->GetRobotDimension(LEG_LENGTH)*sin(DegToRad(20.0)))
				    /
				    (double)RobotVars.parameters.pos_coord_scale
				    );
	}
	else
	{
	  //does not do anything if no kinematic model selected
	  continue;
	}
	
	
	if(RobotVars.parameters.manual_speed_control)// ***************** MANUAL SPEED CONTROL MODE ***************** 
	  cout<<"-> Running on Manual Servo Speed."<<endl;
	else// ***************** AUTOMATIC SPEED CONTROL MODE ***************** 
	  cout<<"-> Running on Automatic Servo Speed."<<endl;
	
	//get instant phantom position
	//get coordinate difference from start point
	memcpy(&last_position, &RobotVars.phantom_data.m_devicePosition, sizeof(RobotVars.phantom_data.m_devicePosition));
	
	clock_gettime(CLOCK_REALTIME, &loop_rate_start);
	
	if(!hdIsEnabled(HD_FORCE_OUTPUT))
	  hdEnable(HD_FORCE_OUTPUT);
	
	//CONTROL LOOPS RUN HERE!!!
	while(RobotVars.haptic_loop_start && !exit)
	{
	  /* Perform a synchronous call to update the most current device state.*/
	  hdScheduleSynchronous(copyDeviceDataCallback,&RobotVars,HD_DEFAULT_SCHEDULER_PRIORITY);
	  
	  //calculate pen tip coordinates
	  ROS_CalculatePHANToMPenFrame(&broadcaster,&listener, &RobotVars);
	  
	  ROS_UpdateMarkers(markersMsg.markers, &RobotVars );
	  marker_pub.publish(markersMsg);
	  
	  // ***************** POSITION CONTROL *****************
	  //get movement increment
	  instant_diff = RobotVars.parameters.pos_coord_scale * (RobotVars.phantom_data.m_devicePosition-last_position);
	  
	  // redefine servo speed if necessary
	  if(RobotVars.parameters.manual_speed_control && RobotVars.parameters.manual_speed!=prev_speed)
	  {
	    if(RobotVars.parameters.kinematic_model==1)
	    {
	      cout<<"-> Setting New Arm Servomotor Speed:"<<RobotVars.parameters.manual_speed<<"."<<endl;
	      SetArmSpeed(RIGHT, RobotVars.parameters.manual_speed, &RobotVars);
	    }
	    else if(RobotVars.parameters.kinematic_model==2)
	    {
	      cout<<"-> Setting New Arm Servomotor Speed:"<<RobotVars.parameters.manual_speed<<"."<<endl;
	      SetArmSpeed(LEFT, RobotVars.parameters.manual_speed, &RobotVars);
	    }
	    else if(RobotVars.parameters.kinematic_model==3)
	    {
	      cout<<"-> Setting New Arms Servomotors Speed:"<<RobotVars.parameters.manual_speed<<"."<<endl;
	      SetArmSpeed(RIGHT, RobotVars.parameters.manual_speed, &RobotVars);
	      SetArmSpeed(LEFT, RobotVars.parameters.manual_speed, &RobotVars);
	    }
	    else if(RobotVars.parameters.kinematic_model==4)
	    {
	      cout<<"-> Setting New Leg Servomotor Speed:"<<RobotVars.parameters.manual_speed<<"."<<endl;
	      SetLegSpeed(LEFT, RobotVars.parameters.manual_speed, &RobotVars);
	    }
	    else if(RobotVars.parameters.kinematic_model==5)
	    {
	      cout<<"-> Setting New Leg Servomotor Speed:"<<RobotVars.parameters.manual_speed<<"."<<endl;
	      SetLegSpeed(RIGHT, RobotVars.parameters.manual_speed, &RobotVars);
	    }
	    prev_speed=RobotVars.parameters.manual_speed;
	  }
	  
	  //robot instructions will only proceed if position has changed over resolution or running user path
	  if((fabs(instant_diff[0]) >= (RobotVars.parameters.coordinate_resolution) || 
	    fabs(instant_diff[1]) >= (RobotVars.parameters.coordinate_resolution) || 
	    fabs(instant_diff[2]) >= (RobotVars.parameters.coordinate_resolution) )
	    ||
	    RobotVars.haptics_data.demo_user_path_run_start)
	  {
	    //get coordinate difference from start point
	    pos_diff = RobotVars.parameters.pos_coord_scale * (RobotVars.phantom_data.m_devicePosition - prev_position);
	    //choose manual or automatic speed control
	    if(RobotVars.parameters.manual_speed_control)
	    {
	      // ***************** MANUAL SPEED CONTROL MODE ***************** 
	      
	      //move corresponding limbs
	      if(RobotVars.parameters.kinematic_model==1 && !RobotVars.haptics_data.demo_user_path_run_start)
	      {
		
		MoveArmToCartesianPosition(RetrievePrecisionFromDouble((arm_curr_pos[0] + pos_diff[0]),RobotVars.parameters.coordinate_resolution),
					   RetrievePrecisionFromDouble((arm_curr_pos[1] - pos_diff[1]),RobotVars.parameters.coordinate_resolution),
					   RetrievePrecisionFromDouble((arm_curr_pos[2] + pos_diff[2]),RobotVars.parameters.coordinate_resolution),
					   RIGHT,
					   &RobotVars);
	      }
	      else if(RobotVars.parameters.kinematic_model==2 && !RobotVars.haptics_data.demo_user_path_run_start)
	      {
		MoveArmToCartesianPosition(RetrievePrecisionFromDouble((arm_curr_pos[0] + pos_diff[0]),RobotVars.parameters.coordinate_resolution),
					   RetrievePrecisionFromDouble((arm_curr_pos[1] + pos_diff[1]),RobotVars.parameters.coordinate_resolution),
					   RetrievePrecisionFromDouble((arm_curr_pos[2] + pos_diff[2]),RobotVars.parameters.coordinate_resolution),
					   LEFT,
					   &RobotVars);
	      }
	      else if(RobotVars.parameters.kinematic_model==3 && !RobotVars.haptics_data.demo_user_path_run_start)
	      {
		MoveArmToCartesianPosition(RetrievePrecisionFromDouble((arm_curr_pos[0] + pos_diff[0]),RobotVars.parameters.coordinate_resolution),
					   RetrievePrecisionFromDouble((arm_curr_pos[1] + pos_diff[1]),RobotVars.parameters.coordinate_resolution),
					   RetrievePrecisionFromDouble((arm_curr_pos[2] + pos_diff[2]),RobotVars.parameters.coordinate_resolution),
					   RIGHT,
					   &RobotVars);
		MoveArmToCartesianPosition(RetrievePrecisionFromDouble((arm_curr_pos[0] + pos_diff[0]),RobotVars.parameters.coordinate_resolution),
					   RetrievePrecisionFromDouble((arm_curr_pos[1] + pos_diff[1]),RobotVars.parameters.coordinate_resolution),
					   RetrievePrecisionFromDouble((arm_curr_pos[2] + pos_diff[2]),RobotVars.parameters.coordinate_resolution),
					   LEFT,
					   &RobotVars);
	      }
	      else if(RobotVars.parameters.kinematic_model==4 && !RobotVars.haptics_data.demo_user_path_run_start)
	      {
		hip_abduction_angle = (RobotVars.phantom_data.gimbalAngles[2]+180.) * RobotVars.parameters.pos_coord_scale;
		  MoveDetachedLegToCartesianPosition(RetrievePrecisionFromDouble((detached_leg_curr_pos[0] + pos_diff[0]),RobotVars.parameters.coordinate_resolution),
						  RetrievePrecisionFromDouble((detached_leg_curr_pos[1] + pos_diff[1]),RobotVars.parameters.coordinate_resolution),
						  RetrievePrecisionFromDouble((detached_leg_curr_pos[2] + pos_diff[2]),RobotVars.parameters.coordinate_resolution),
						  hip_abduction_angle,
						  LEFT,
						  &RobotVars);
	      }
	      else if(RobotVars.parameters.kinematic_model==5 && !RobotVars.haptics_data.demo_user_path_run_start)
	      {
		hip_abduction_angle = (RobotVars.phantom_data.gimbalAngles[2]+180.) * RobotVars.parameters.pos_coord_scale;
		  MoveDetachedLegToCartesianPosition(RetrievePrecisionFromDouble((detached_leg_curr_pos[0] + pos_diff[0]),RobotVars.parameters.coordinate_resolution),
						  RetrievePrecisionFromDouble((detached_leg_curr_pos[1] - pos_diff[1]),RobotVars.parameters.coordinate_resolution),
						  RetrievePrecisionFromDouble((detached_leg_curr_pos[2] + pos_diff[2]),RobotVars.parameters.coordinate_resolution),
						  hip_abduction_angle,
						  RIGHT,
						  &RobotVars);
	      }
	      else if( RobotVars.parameters.kinematic_model > 0 && RobotVars.haptics_data.chosen_demo == 3 && RobotVars.haptics_data.demo_user_path_run_start)
	      {
		PathFollowingExecute(&RobotVars);
	      }
	      else
	      {
		//does not do anything if no kinematic model selected | redundant
		continue;
	      }
	      // ****************************************************************
	    }
	    else
	    {
	      // ***************** AUTOMATIC SPEED CONTROL MODE *****************
	      clock_gettime(CLOCK_REALTIME, &tf);
	      interval = ((long double)tf.tv_sec + (long double)tf.tv_nsec / (long double)1e9) - ((long double)interval_time_i.tv_sec + (long double)interval_time_i.tv_nsec / (long double)1e9);
	      printf("Interval: %9.9Lf\n",interval);
	      if(interval>=(1./RobotVars.parameters.automatic_speed_control_freq))
	      {
		cout<<"Speed Control!"<<endl;
		
		avg_speed[0] = RobotVars.phantom_data.average_speed[0];
		avg_speed[1] = RobotVars.phantom_data.average_speed[1];
		avg_speed[2] = RobotVars.phantom_data.average_speed[2];
		
		ArmsDifferencialSpeedControl(avg_speed, &RobotVars);
		clock_gettime(CLOCK_REALTIME, &interval_time_i);
	      }
	      // ****************************************************************
	    }
	    //update direct kinematics at every iteration
	    if(RobotVars.parameters.kinematic_model>=1 && RobotVars.parameters.kinematic_model<=3)
	    {
	      UpdateArmsDirKinData(&RobotVars);
	    }
	    else if(RobotVars.parameters.kinematic_model==4 || RobotVars.parameters.kinematic_model==5)
	    {
	      UpdateDetachedLegsDirKinData(&RobotVars);
	    }
	    else
	    {
	      //does not do anything if no kinematic model selected | redundant
	      continue;
	    }
	    
	    k_tick=0;
	    
	    //get coordinate difference from start point
	    memcpy(&last_position, &RobotVars.phantom_data.m_devicePosition, sizeof(RobotVars.phantom_data.m_devicePosition));
	  }
	  else
	  {
	    //count ticks to stop robot
	    k_tick++;
	    if(k_tick==500)
	    {
	      UpdateJointDataByID(1000, 0., &RobotVars);
	      if(RobotVars.parameters.kinematic_model>=1 && RobotVars.parameters.kinematic_model<=3)
	      {
		UpdateArmsDirKinData(&RobotVars);
	      }
	      else if(RobotVars.parameters.kinematic_model==4 || RobotVars.parameters.kinematic_model==5)
	      {
		UpdateDetachedLegsDirKinData(&RobotVars);
	      }
	      StopRobotMovement(&RobotVars);
	    }
	    else if(k_tick>2500)
	      k_tick=500-1;
	  }
	  //update labels
	  RobotVars.update_labels=TRUE;
	  // ****************************************************
	  exit=RobotVars.parameters.exit_status;
	  //calculate control rate
	  if(counter<100)
	    counter++;
	  else
	  {
	    clock_gettime(CLOCK_REALTIME, &loop_rate_stop);
	    time_in_seconds =
	    ((long double)loop_rate_stop.tv_sec + ((long double)loop_rate_stop.tv_nsec / (long double)1e9)) -
	    ((long double)loop_rate_start.tv_sec + ((long double)loop_rate_start.tv_nsec / (long double)1e9));

	    RobotVars.parameters.control_rate = round((long double)1. / (time_in_seconds/(long double)counter));
	    counter = 0;
	    clock_gettime(CLOCK_REALTIME, &loop_rate_start);
	  }
	}
	StopRobotMovement(&RobotVars);
	cout<<" :: Terminating haptic loop!... ::"<<endl;
	RobotVars.parameters.control_rate = 0.;
	if(hdIsEnabled(HD_FORCE_OUTPUT))
	  hdDisable(HD_FORCE_OUTPUT);
	if(RobotVars.haptics_data.chosen_demo == 2)
	{
	  RobotVars.haptics_data.demo_2_point_1=hduVector3Dd(0,0,0);
	  RobotVars.haptics_data.demo_2_point_2=hduVector3Dd(0,0,0);
	  RobotVars.haptics_data.demo_2_point_3=hduVector3Dd(0,0,0);
	}
      } //end of haptic/control loop
      // update interface labels
      RobotVars.update_labels=TRUE;
      exit=RobotVars.parameters.exit_status;
    }
  }
  
  if(RobotVars.phantom_data.phantom_on || RobotVars.servo->IsActive())
  {
    cout<<"* Application termination initiated."<<endl<<"* Processing shutdown routines:"<<endl;
    cout<<"---> Main loop stopped."<<endl;
    cout<<"---> Waiting for GTK+ interface thread to close...";
  }
  
  // wait for interface to close
  pthread_join( thread_gtk, NULL);
  if(RobotVars.phantom_data.phantom_on || RobotVars.servo->IsActive())
    cout<<"DONE!"<<endl;
  else
  {
    cout<<"* Application termination initiated."<<endl<<"* Processing shutdown routines:"<<endl;
    cout<<"---> Main loop stopped."<<endl;
    cout<<"---> GTK+ interface closed."<<endl;
  }
  
  if(RobotVars.phantom_data.phantom_on)
  {
    cout<<"---> Cleaning OpenHaptics Toolkit variables:"<<endl;
    // For cleanup, unschedule callbacks and stop the servo loop.
    cout<<"     >> Deleting haptic context...";
    // hlDeleteContext(hlGetCurrentContext());
    // OK, SO THIS IS GETTING WEIRDER... NOW, CREATING AND DELETING HAPTIC CONTEXT IS GIVING SEGFAULT... TODO: TRY AND SOLVE THIS CRAP...
    hlMakeCurrent(NULL);
    cout<<"DONE!"<<endl;
    cout<<"     >> Stopping servo loop thread and scheduler...";
    hdUnschedule(hForceHandle);
    hdUnschedule(hUpdateHandle);
    hdStopScheduler();
    cout<<"DONE!"<<endl;
    cout<<"     >> Disabling PHANToM device...";
    hdDisableDevice(RobotVars.phantom_data.hHD);
    cout<<"DONE!"<<endl;
    
    //EXIT message
    cout<<"* Exiting now."<<endl<<">> [Cleanup errors may appear bellow this line...]"<<endl;
  }
  else
  {
    cout<<"* Exiting now."<<endl;
  }
  
  return EXIT_SUCCESS;
}

void ROS_CalculatePHANToMPenFrame(tf::TransformBroadcaster *broadcaster, tf::TransformListener *listener, void *pUserData)
{
  shared_vars_t*RobotVars=(shared_vars_t*)pUserData;
  
  static tf::StampedTransform world_to_pen_tip_transform;
  static timespec ts,tf;
  static long double time_in_seconds;
  static uint counter;
  static hduVector3Dd pos;
  double *avg_speed=new double[3];
  
  
  ros::Time time_ros;
  time_ros = ros::Time::now();
    
  broadcaster->sendTransform(tf::StampedTransform(RobotVars->phantom_data.world_to_phantom, time_ros, "/phantom_world_corrected", "/phantom_world"));
  
  broadcaster->sendTransform(tf::StampedTransform(RobotVars->phantom_data.phantom_to_base_point, time_ros, "/phantom_world", "/phantom_base_point"));
  
  broadcaster->sendTransform(tf::StampedTransform(RobotVars->phantom_data.base_point_to_pen_tip, time_ros, "/phantom_base_point", "/phantom_pen_tip"));
  
  try
  {
    listener->lookupTransform("/phantom_world_corrected", "/phantom_pen_tip", ros::Time(0),world_to_pen_tip_transform);
  }
  catch (tf::TransformException ex)
  {
    if(listener->waitForTransform("/phantom_world_corrected", "/phantom_pen_tip",ros::Time(0), ros::Duration(2.0)))
    {
      try
      {
	listener->lookupTransform("/phantom_world_corrected", "/phantom_pen_tip", ros::Time(0),world_to_pen_tip_transform);
      }
      catch (tf::TransformException ex)
      {
	ROS_ERROR("Joystick Transforms: Could not lookup transform after waiting 2 secs\n.%s",ex.what());
      }
    }
    else
    {
      ROS_ERROR("Joystick Transforms: Could not find valid transform after waiting 2 seconds\n.%s",ex.what());
    }
  }
  
  ros::spinOnce();
  
  RobotVars->phantom_data.m_PenTipPosition[0] = world_to_pen_tip_transform.getOrigin().x();
  RobotVars->phantom_data.m_PenTipPosition[1] = world_to_pen_tip_transform.getOrigin().y();
  RobotVars->phantom_data.m_PenTipPosition[2] = world_to_pen_tip_transform.getOrigin().z();
  
  if(counter>=15)
  {
  //calculate speed
  clock_gettime(CLOCK_REALTIME, &tf);
  time_in_seconds = ((long double)tf.tv_sec + ((long double)tf.tv_nsec / (long double)1e9)) - ((long double)ts.tv_sec + ((long double)ts.tv_nsec / (long double)1e9));
  
  
  CalculateAverageCartesianSpeed((RobotVars->phantom_data.m_PenTipPosition[0] - pos[0]),
				(RobotVars->phantom_data.m_PenTipPosition[1] - pos[1]),
				(RobotVars->phantom_data.m_PenTipPosition[2] - pos[2]),
				time_in_seconds,
				avg_speed);
  
  //update speed
  RobotVars->phantom_data.average_speed[0] = avg_speed[0];
  RobotVars->phantom_data.average_speed[1] = avg_speed[1];
  RobotVars->phantom_data.average_speed[2] = avg_speed[2];
  
  counter=0;
  pos = RobotVars->phantom_data.m_PenTipPosition;
  clock_gettime(CLOCK_REALTIME, &ts);
  }
  else
  {
    counter++;
  }
  delete avg_speed;
}

void ROS_UpdateMarkers(vector<visualization_msgs::Marker>& marker_vector, void *pUserData)
{
  shared_vars_t*RobotVars=(shared_vars_t*)pUserData;
  string text;
  geometry_msgs::Point p;
  geometry_msgs::Point p2;
  //clear the whole vector
  marker_vector.clear();
  
  //TAGS
  //Base frame
  visualization_msgs::Marker marker_BASE_TAG;
  marker_BASE_TAG.header.frame_id = "/phantom_world_corrected";
  //marker info
  marker_BASE_TAG.header.stamp = ros::Time::now();
  marker_BASE_TAG.ns = "text_BASE_TAG";
  marker_BASE_TAG.action = visualization_msgs::Marker::ADD;
  //marker type
  marker_BASE_TAG.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  text = "/phantom_world";
  marker_BASE_TAG.text = text;
  marker_BASE_TAG.pose.position.x = 0.0;
  marker_BASE_TAG.pose.position.y = 0.0;
  marker_BASE_TAG.pose.position.z = 0.0;
  marker_BASE_TAG.scale.z = 10.0; 
  //marker color
  marker_BASE_TAG.color.r=0.5;
  marker_BASE_TAG.color.g=0.5;
  marker_BASE_TAG.color.b=0.5;
  marker_BASE_TAG.color.a=1;
  marker_BASE_TAG.id=0;
  marker_BASE_TAG.lifetime = ros::Duration();
  marker_vector.push_back(marker_BASE_TAG);
  //wirst frame
  visualization_msgs::Marker marker_WRIST_TAG;
  marker_WRIST_TAG.header.frame_id = "/phantom_base_point";
  //marker info
  marker_WRIST_TAG.header.stamp = ros::Time::now();
  marker_WRIST_TAG.ns = "text_wrist_TAG";
  marker_WRIST_TAG.action = visualization_msgs::Marker::ADD;
  //marker type
  marker_WRIST_TAG.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  text = "/phantom_wrist";
  marker_WRIST_TAG.text = text;
  marker_WRIST_TAG.pose.position = marker_BASE_TAG.pose.position;
  marker_WRIST_TAG.scale = marker_BASE_TAG.scale;
  //marker color
  marker_WRIST_TAG.color = marker_BASE_TAG.color;
  marker_WRIST_TAG.id=0;
  marker_WRIST_TAG.lifetime = ros::Duration();
  marker_vector.push_back(marker_WRIST_TAG);
  //Tip frame
  visualization_msgs::Marker marker_TIP_TAG;
  marker_TIP_TAG.header.frame_id = "/phantom_pen_tip";
  //marker info
  marker_TIP_TAG.header.stamp = ros::Time::now();
  marker_TIP_TAG.ns = "text_tip_TAG";
  marker_TIP_TAG.action = visualization_msgs::Marker::ADD;
  //marker type
  marker_TIP_TAG.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  text = "/phantom_tip";
  marker_TIP_TAG.text = text;
  marker_TIP_TAG.pose.position = marker_BASE_TAG.pose.position;
  marker_TIP_TAG.scale = marker_BASE_TAG.scale;
  //marker color
  marker_TIP_TAG.color = marker_BASE_TAG.color;
  marker_TIP_TAG.id=0;
  marker_TIP_TAG.lifetime = ros::Duration();
  marker_vector.push_back(marker_TIP_TAG);
  
  if(RobotVars->haptics_data.chosen_demo == 1 && RobotVars->haptic_loop_start)
  {
    if((RobotVars->parameters.kinematic_model>=4 && RobotVars->parameters.kinematic_model<=5) && RobotVars->parameters.manual_speed_control)
    {
      // |||||||||||||||||||||||||||||||||||||||||||||||||||||| LEG WORKSPACE DEMO CONSTRUCTIONS ||||||||||||||||||||||||||||||||||||||||||||||||||||||
      //OUTER SPHERE
      visualization_msgs::Marker marker_outer_sphere;
      marker_outer_sphere.header.frame_id = "/phantom_world_corrected";
      //marker info
      marker_outer_sphere.header.stamp = ros::Time::now();
      marker_outer_sphere.ns = "outer_sphere";
      marker_outer_sphere.action = visualization_msgs::Marker::ADD;
      //marker type
      marker_outer_sphere.type = visualization_msgs::Marker::SPHERE;
      //marker scale
      marker_outer_sphere.scale.x = 2.0*RobotVars->haptics_data.leg_main_sphere_max_Radius; 
      marker_outer_sphere.scale.y = 2.0*RobotVars->haptics_data.leg_main_sphere_max_Radius; 
      marker_outer_sphere.scale.z = 2.0*RobotVars->haptics_data.leg_main_sphere_max_Radius; 
      //marker pose
      marker_outer_sphere.pose.position.x = RobotVars->haptics_data.leg_main_spheresPosition[0];
      marker_outer_sphere.pose.position.y = RobotVars->haptics_data.leg_main_spheresPosition[1];
      marker_outer_sphere.pose.position.z = RobotVars->haptics_data.leg_main_spheresPosition[2];
      marker_outer_sphere.pose.orientation.x = 0.0;
      marker_outer_sphere.pose.orientation.y = 0.0;
      marker_outer_sphere.pose.orientation.z = 0.0;
      marker_outer_sphere.pose.orientation.w = 1.0;
      
      marker_outer_sphere.color.r=0;
      marker_outer_sphere.color.g=1;
      marker_outer_sphere.color.b=0;
      marker_outer_sphere.color.a=0.2;
      marker_outer_sphere.id=0;
      marker_outer_sphere.lifetime = ros::Duration();
      
      marker_vector.push_back(marker_outer_sphere);
      
      //INNER SPHERE
      visualization_msgs::Marker marker_inner_sphere;
      marker_inner_sphere.header.frame_id = "/phantom_world_corrected";
      //marker info
      marker_inner_sphere.header.stamp = ros::Time::now();
      marker_inner_sphere.ns = "inner_sphere";
      marker_inner_sphere.action = visualization_msgs::Marker::ADD;
      //marker type
      marker_inner_sphere.type = visualization_msgs::Marker::SPHERE;
      //marker scale
      marker_inner_sphere.scale.x = 2.0*RobotVars->haptics_data.leg_main_sphere_min_Radius; 
      marker_inner_sphere.scale.y = 2.0*RobotVars->haptics_data.leg_main_sphere_min_Radius; 
      marker_inner_sphere.scale.z = 2.0*RobotVars->haptics_data.leg_main_sphere_min_Radius; 
      //marker pose
      marker_inner_sphere.pose.position.x = RobotVars->haptics_data.leg_main_spheresPosition[0];
      marker_inner_sphere.pose.position.y = RobotVars->haptics_data.leg_main_spheresPosition[1];
      marker_inner_sphere.pose.position.z = RobotVars->haptics_data.leg_main_spheresPosition[2];
      marker_inner_sphere.pose.orientation.x = 0.0;
      marker_inner_sphere.pose.orientation.y = 0.0;
      marker_inner_sphere.pose.orientation.z = 0.0;
      marker_inner_sphere.pose.orientation.w = 1.0;
      //marker color
      marker_inner_sphere.color.r=0;
      marker_inner_sphere.color.g=0;
      marker_inner_sphere.color.b=1;
      marker_inner_sphere.color.a=0.2;
      marker_inner_sphere.id=0;
      marker_inner_sphere.lifetime = ros::Duration();
      marker_vector.push_back(marker_inner_sphere);
      
      
      //FORCE VECTOR
      visualization_msgs::Marker marker_force_vector;
      marker_force_vector.header.frame_id = "/phantom_world_corrected";
      //marker info
      marker_force_vector.header.stamp = ros::Time::now();
      marker_force_vector.ns = "force_vector";
      marker_force_vector.action = visualization_msgs::Marker::ADD;
      //marker type
      marker_force_vector.type = visualization_msgs::Marker::ARROW;
      //marker points
      marker_force_vector.points.clear();
      p.x = RobotVars->phantom_data.m_devicePosition[0];
      p.y = RobotVars->phantom_data.m_devicePosition[1];
      p.z = RobotVars->phantom_data.m_devicePosition[2];
      marker_force_vector.points.push_back(p);
      
      hduVector3Dd Fdir = RobotVars->haptics_data.applied_force;
      if(Fdir.magnitude() > 3.3)
      {
	Fdir.normalize();
	Fdir = Fdir * 3.3;
      }
      p2.x = RobotVars->phantom_data.m_devicePosition[0] + Fdir[0]*10;
      p2.y = RobotVars->phantom_data.m_devicePosition[1] + Fdir[1]*10;
      p2.z = RobotVars->phantom_data.m_devicePosition[2] + Fdir[2]*10;
      marker_force_vector.points.push_back(p2);
      //marker scale
      marker_force_vector.scale.x = Fdir.magnitude()*5; 
      marker_force_vector.scale.y = Fdir.magnitude()*6;
      marker_force_vector.scale.z = Fdir.magnitude()*7;
      //marker color
      marker_force_vector.color.r=0;
      marker_force_vector.color.g=0;
      marker_force_vector.color.b=0;
      marker_force_vector.color.a=1;
      marker_force_vector.id=0;
      marker_force_vector.lifetime = ros::Duration();
      marker_vector.push_back(marker_force_vector);
      
      //vector magnitude
      visualization_msgs::Marker marker_force_magnitude;
      marker_force_magnitude.header.frame_id = "/phantom_world_corrected";
      //marker info
      marker_force_magnitude.header.stamp = ros::Time::now();
      marker_force_magnitude.ns = "text_force";
      marker_force_magnitude.action = visualization_msgs::Marker::ADD;
      //marker type
      marker_force_magnitude.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      text = str(boost::format("%3.1f") % AvoidMinusZero(Fdir.magnitude())) + " N";
      marker_force_magnitude.text = text;
      marker_force_magnitude.pose.position.x = RobotVars->phantom_data.m_devicePosition[0] + Fdir[0]*10;
      marker_force_magnitude.pose.position.y = RobotVars->phantom_data.m_devicePosition[1] + Fdir[1]*10;
      marker_force_magnitude.pose.position.z = RobotVars->phantom_data.m_devicePosition[2] + Fdir[2]*10;
      marker_force_magnitude.scale.z = 10.0; 
      //marker color
      marker_force_magnitude.color = marker_force_vector.color;
      marker_force_magnitude.color.a=0.8;
      marker_force_magnitude.id=0;
      marker_force_magnitude.lifetime = ros::Duration();
      marker_vector.push_back(marker_force_magnitude);   
    }
    else if((RobotVars->parameters.kinematic_model>=1 && RobotVars->parameters.kinematic_model<=3) && RobotVars->parameters.manual_speed_control)
    {
      // |||||||||||||||||||||||||||||||||||||||||||||||||||||| ARM WORKSPACE DEMO CONSTRUCTIONS ||||||||||||||||||||||||||||||||||||||||||||||||||||||
      //OUTER SPHERE
      visualization_msgs::Marker marker_outer_sphere;
      marker_outer_sphere.header.frame_id = "/phantom_world_corrected";
      //marker info
      marker_outer_sphere.header.stamp = ros::Time::now();
      marker_outer_sphere.ns = "outer_sphere";
      marker_outer_sphere.action = visualization_msgs::Marker::ADD;
      //marker type
      marker_outer_sphere.type = visualization_msgs::Marker::SPHERE;
      //marker scale
      marker_outer_sphere.scale.x = 2.0*RobotVars->haptics_data.arm_main_sphere_max_Radius; 
      marker_outer_sphere.scale.y = 2.0*RobotVars->haptics_data.arm_main_sphere_max_Radius; 
      marker_outer_sphere.scale.z = 2.0*RobotVars->haptics_data.arm_main_sphere_max_Radius; 
      //marker pose
      marker_outer_sphere.pose.position.x = RobotVars->haptics_data.arm_main_spheresPosition[0];
      marker_outer_sphere.pose.position.y = RobotVars->haptics_data.arm_main_spheresPosition[1];
      marker_outer_sphere.pose.position.z = RobotVars->haptics_data.arm_main_spheresPosition[2];
      marker_outer_sphere.pose.orientation.x = 0.0;
      marker_outer_sphere.pose.orientation.y = 0.0;
      marker_outer_sphere.pose.orientation.z = 0.0;
      marker_outer_sphere.pose.orientation.w = 1.0;
      
      marker_outer_sphere.color.r=0;
      marker_outer_sphere.color.g=1;
      marker_outer_sphere.color.b=0;
      marker_outer_sphere.color.a=0.2;
      marker_outer_sphere.id=0;
      marker_outer_sphere.lifetime = ros::Duration();
      
      marker_vector.push_back(marker_outer_sphere);
      
      //INNER SPHERE
      visualization_msgs::Marker marker_inner_sphere;
      marker_inner_sphere.header.frame_id = "/phantom_world_corrected";
      //marker info
      marker_inner_sphere.header.stamp = ros::Time::now();
      marker_inner_sphere.ns = "inner_sphere";
      marker_inner_sphere.action = visualization_msgs::Marker::ADD;
      //marker type
      marker_inner_sphere.type = visualization_msgs::Marker::SPHERE;
      //marker scale
      marker_inner_sphere.scale.x = 2.0*RobotVars->haptics_data.arm_main_sphere_min_Radius; 
      marker_inner_sphere.scale.y = 2.0*RobotVars->haptics_data.arm_main_sphere_min_Radius; 
      marker_inner_sphere.scale.z = 2.0*RobotVars->haptics_data.arm_main_sphere_min_Radius; 
      //marker pose
      marker_inner_sphere.pose.position.x = RobotVars->haptics_data.arm_main_spheresPosition[0];
      marker_inner_sphere.pose.position.y = RobotVars->haptics_data.arm_main_spheresPosition[1];
      marker_inner_sphere.pose.position.z = RobotVars->haptics_data.arm_main_spheresPosition[2];
      marker_inner_sphere.pose.orientation = marker_outer_sphere.pose.orientation;
      //marker color
      marker_inner_sphere.color.r=0;
      marker_inner_sphere.color.g=0;
      marker_inner_sphere.color.b=1;
      marker_inner_sphere.color.a=1;
      marker_inner_sphere.id=0;
      marker_inner_sphere.lifetime = ros::Duration();
      marker_vector.push_back(marker_inner_sphere);
      
      //INNER SPHERE
      visualization_msgs::Marker marker_back_sphere;
      marker_back_sphere.header.frame_id = "/phantom_world_corrected";
      //marker info
      marker_back_sphere.header.stamp = ros::Time::now();
      marker_back_sphere.ns = "back_sphere";
      marker_back_sphere.action = visualization_msgs::Marker::ADD;
      //marker type
      marker_back_sphere.type = visualization_msgs::Marker::SPHERE;
      //marker scale
      marker_back_sphere.scale.x = 2.0*RobotVars->haptics_data.arm_back_sphereRadius; 
      marker_back_sphere.scale.y = 2.0*RobotVars->haptics_data.arm_back_sphereRadius; 
      marker_back_sphere.scale.z = 2.0*RobotVars->haptics_data.arm_back_sphereRadius; 
      //marker pose
      marker_back_sphere.pose.position.x = RobotVars->haptics_data.arm_back_spherePosition[0];
      marker_back_sphere.pose.position.y = RobotVars->haptics_data.arm_back_spherePosition[1];
      marker_back_sphere.pose.position.z = RobotVars->haptics_data.arm_back_spherePosition[2];
      marker_back_sphere.pose.orientation = marker_outer_sphere.pose.orientation;
      //marker color
      marker_back_sphere.color.r=0;
      marker_back_sphere.color.g=0;
      marker_back_sphere.color.b=1;
      marker_back_sphere.color.a=1;
      marker_back_sphere.id=0;
      marker_back_sphere.lifetime = ros::Duration();
      marker_vector.push_back(marker_back_sphere);
      
      //PLANE
      visualization_msgs::Marker marker_plane;
      marker_plane.header.frame_id = "/phantom_world_corrected";
      //marker info
      marker_plane.header.stamp = ros::Time::now();
      marker_plane.ns = "plane";
      marker_plane.action = visualization_msgs::Marker::ADD;
      //marker type
      marker_plane.type = visualization_msgs::Marker::CUBE;
      //marker scale
      marker_plane.scale.x = 2.0*RobotVars->haptics_data.arm_main_sphere_max_Radius; 
      marker_plane.scale.y = 0.01; 
      marker_plane.scale.z = 2.0*RobotVars->haptics_data.arm_main_sphere_max_Radius; 
      //marker pose
      marker_plane.pose.position.x = RobotVars->haptics_data.arm_main_spheresPosition[0];
      marker_plane.pose.position.y = RobotVars->haptics_data.arm_main_spheresPosition[1];
      marker_plane.pose.position.z = RobotVars->haptics_data.arm_main_spheresPosition[2];
      marker_plane.pose.orientation = marker_outer_sphere.pose.orientation;
      //marker color
      marker_plane.color.r=1;
      marker_plane.color.g=1;
      marker_plane.color.b=0;
      marker_plane.color.a=0.8;
      marker_plane.id=0;
      marker_plane.lifetime = ros::Duration();
      marker_vector.push_back(marker_plane);
      
      //FORCE VECTOR
      visualization_msgs::Marker marker_force_vector;
      marker_force_vector.header.frame_id = "/phantom_world_corrected";
      //marker info
      marker_force_vector.header.stamp = ros::Time::now();
      marker_force_vector.ns = "force_vector";
      marker_force_vector.action = visualization_msgs::Marker::ADD;
      //marker type
      marker_force_vector.type = visualization_msgs::Marker::ARROW;
      //marker points
      marker_force_vector.points.clear();
      p.x = RobotVars->phantom_data.m_devicePosition[0];
      p.y = RobotVars->phantom_data.m_devicePosition[1];
      p.z = RobotVars->phantom_data.m_devicePosition[2];
      marker_force_vector.points.push_back(p);
      
      hduVector3Dd Fdir = RobotVars->haptics_data.applied_force;
      if(Fdir.magnitude() > 3.3)
      {
	Fdir.normalize();
	Fdir = Fdir * 3.3;
      }
      p2.x = RobotVars->phantom_data.m_devicePosition[0] + Fdir[0]*10;
      p2.y = RobotVars->phantom_data.m_devicePosition[1] + Fdir[1]*10;
      p2.z = RobotVars->phantom_data.m_devicePosition[2] + Fdir[2]*10;
      marker_force_vector.points.push_back(p2);
      //marker scale
      marker_force_vector.scale.x = Fdir.magnitude()*5; 
      marker_force_vector.scale.y = Fdir.magnitude()*6;
      marker_force_vector.scale.z = Fdir.magnitude()*7;
      //marker color
      marker_force_vector.color.r=0;
      marker_force_vector.color.g=0;
      marker_force_vector.color.b=0;
      marker_force_vector.color.a=1;
      marker_force_vector.id=0;
      marker_force_vector.lifetime = ros::Duration();
      marker_vector.push_back(marker_force_vector);
      
      //vector magnitude
      visualization_msgs::Marker marker_force_magnitude;
      marker_force_magnitude.header.frame_id = "/phantom_world_corrected";
      //marker info
      marker_force_magnitude.header.stamp = ros::Time::now();
      marker_force_magnitude.ns = "text_force";
      marker_force_magnitude.action = visualization_msgs::Marker::ADD;
      //marker type
      marker_force_magnitude.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      text = str(boost::format("%3.1f") % AvoidMinusZero(Fdir.magnitude())) + " N";
      marker_force_magnitude.text = text;
      marker_force_magnitude.pose.position.x = RobotVars->phantom_data.m_devicePosition[0] + Fdir[0]*10;
      marker_force_magnitude.pose.position.y = RobotVars->phantom_data.m_devicePosition[1] + Fdir[1]*10;
      marker_force_magnitude.pose.position.z = RobotVars->phantom_data.m_devicePosition[2] + Fdir[2]*10;
      marker_force_magnitude.scale.z = 10.0; 
      //marker color
      marker_force_magnitude.color = marker_force_vector.color;
      marker_force_magnitude.color.a=0.8;
      marker_force_magnitude.id=0;
      marker_force_magnitude.lifetime = ros::Duration();
      marker_vector.push_back(marker_force_magnitude);
    }
  }
  else if(RobotVars->haptics_data.chosen_demo == 2 && RobotVars->haptic_loop_start)
  {
    if((RobotVars->parameters.kinematic_model>=1 && RobotVars->parameters.kinematic_model<=3) && RobotVars->parameters.manual_speed_control)
    {
      // |||||||||||||||||||||||||||||||||||||||||||||||||||||| PLANE DRAWING DEMO CONSTRUCTIONS ||||||||||||||||||||||||||||||||||||||||||||||||||||||
      
      // ******************************* workspace information *******************************
      //OUTER SPHERE
      visualization_msgs::Marker marker_outer_sphere;
      marker_outer_sphere.header.frame_id = "/phantom_world_corrected";
      //marker info
      marker_outer_sphere.header.stamp = ros::Time::now();
      marker_outer_sphere.ns = "outer_sphere";
      marker_outer_sphere.action = visualization_msgs::Marker::ADD;
      //marker type
      marker_outer_sphere.type = visualization_msgs::Marker::SPHERE;
      //marker scale
      marker_outer_sphere.scale.x = 2.0*RobotVars->haptics_data.arm_main_sphere_max_Radius; 
      marker_outer_sphere.scale.y = 2.0*RobotVars->haptics_data.arm_main_sphere_max_Radius; 
      marker_outer_sphere.scale.z = 2.0*RobotVars->haptics_data.arm_main_sphere_max_Radius; 
      //marker pose
      marker_outer_sphere.pose.position.x = RobotVars->haptics_data.arm_main_spheresPosition[0];
      marker_outer_sphere.pose.position.y = RobotVars->haptics_data.arm_main_spheresPosition[1];
      marker_outer_sphere.pose.position.z = RobotVars->haptics_data.arm_main_spheresPosition[2];
      marker_outer_sphere.pose.orientation.x = 0.0;
      marker_outer_sphere.pose.orientation.y = 0.0;
      marker_outer_sphere.pose.orientation.z = 0.0;
      marker_outer_sphere.pose.orientation.w = 1.0;
      
      marker_outer_sphere.color.r=0;
      marker_outer_sphere.color.g=1;
      marker_outer_sphere.color.b=0;
      marker_outer_sphere.color.a=0.2;
      marker_outer_sphere.id=0;
      marker_outer_sphere.lifetime = ros::Duration();
      
      marker_vector.push_back(marker_outer_sphere);
      
      //INNER SPHERE
      visualization_msgs::Marker marker_inner_sphere;
      marker_inner_sphere.header.frame_id = "/phantom_world_corrected";
      //marker info
      marker_inner_sphere.header.stamp = ros::Time::now();
      marker_inner_sphere.ns = "inner_sphere";
      marker_inner_sphere.action = visualization_msgs::Marker::ADD;
      //marker type
      marker_inner_sphere.type = visualization_msgs::Marker::SPHERE;
      //marker scale
      marker_inner_sphere.scale.x = 2.0*RobotVars->haptics_data.arm_main_sphere_min_Radius; 
      marker_inner_sphere.scale.y = 2.0*RobotVars->haptics_data.arm_main_sphere_min_Radius; 
      marker_inner_sphere.scale.z = 2.0*RobotVars->haptics_data.arm_main_sphere_min_Radius; 
      //marker pose
      marker_inner_sphere.pose.position.x = RobotVars->haptics_data.arm_main_spheresPosition[0];
      marker_inner_sphere.pose.position.y = RobotVars->haptics_data.arm_main_spheresPosition[1];
      marker_inner_sphere.pose.position.z = RobotVars->haptics_data.arm_main_spheresPosition[2];
      marker_inner_sphere.pose.orientation = marker_outer_sphere.pose.orientation;
      //marker color
      marker_inner_sphere.color.r=0;
      marker_inner_sphere.color.g=0;
      marker_inner_sphere.color.b=1;
      marker_inner_sphere.color.a=1;
      marker_inner_sphere.id=0;
      marker_inner_sphere.lifetime = ros::Duration();
      marker_vector.push_back(marker_inner_sphere);
      
      //INNER SPHERE
      visualization_msgs::Marker marker_back_sphere;
      marker_back_sphere.header.frame_id = "/phantom_world_corrected";
      //marker info
      marker_back_sphere.header.stamp = ros::Time::now();
      marker_back_sphere.ns = "back_sphere";
      marker_back_sphere.action = visualization_msgs::Marker::ADD;
      //marker type
      marker_back_sphere.type = visualization_msgs::Marker::SPHERE;
      //marker scale
      marker_back_sphere.scale.x = 2.0*RobotVars->haptics_data.arm_back_sphereRadius; 
      marker_back_sphere.scale.y = 2.0*RobotVars->haptics_data.arm_back_sphereRadius; 
      marker_back_sphere.scale.z = 2.0*RobotVars->haptics_data.arm_back_sphereRadius; 
      //marker pose
      marker_back_sphere.pose.position.x = RobotVars->haptics_data.arm_back_spherePosition[0];
      marker_back_sphere.pose.position.y = RobotVars->haptics_data.arm_back_spherePosition[1];
      marker_back_sphere.pose.position.z = RobotVars->haptics_data.arm_back_spherePosition[2];
      marker_back_sphere.pose.orientation = marker_outer_sphere.pose.orientation;
      //marker color
      marker_back_sphere.color.r=0;
      marker_back_sphere.color.g=0;
      marker_back_sphere.color.b=1;
      marker_back_sphere.color.a=1;
      marker_back_sphere.id=0;
      marker_back_sphere.lifetime = ros::Duration();
      marker_vector.push_back(marker_back_sphere);
      
      //PLANE
      visualization_msgs::Marker marker_plane;
      marker_plane.header.frame_id = "/phantom_world_corrected";
      //marker info
      marker_plane.header.stamp = ros::Time::now();
      marker_plane.ns = "plane";
      marker_plane.action = visualization_msgs::Marker::ADD;
      //marker type
      marker_plane.type = visualization_msgs::Marker::CUBE;
      //marker scale
      marker_plane.scale.x = 2.0*RobotVars->haptics_data.arm_main_sphere_max_Radius; 
      marker_plane.scale.y = 0.01; 
      marker_plane.scale.z = 2.0*RobotVars->haptics_data.arm_main_sphere_max_Radius; 
      //marker pose
      marker_plane.pose.position.x = RobotVars->haptics_data.arm_main_spheresPosition[0];
      marker_plane.pose.position.y = RobotVars->haptics_data.arm_main_spheresPosition[1];
      marker_plane.pose.position.z = RobotVars->haptics_data.arm_main_spheresPosition[2];
      marker_plane.pose.orientation = marker_outer_sphere.pose.orientation;
      //marker color
      marker_plane.color.r=1;
      marker_plane.color.g=1;
      marker_plane.color.b=0;
      marker_plane.color.a=0.8;
      marker_plane.id=0;
      marker_plane.lifetime = ros::Duration();
      marker_vector.push_back(marker_plane);
      
      //FORCE VECTOR
      visualization_msgs::Marker marker_force_vector;
      marker_force_vector.header.frame_id = "/phantom_world_corrected";
      //marker info
      marker_force_vector.header.stamp = ros::Time::now();
      marker_force_vector.ns = "force_vector";
      marker_force_vector.action = visualization_msgs::Marker::ADD;
      //marker type
      marker_force_vector.type = visualization_msgs::Marker::ARROW;
      //marker points
      marker_force_vector.points.clear();
      p.x = RobotVars->phantom_data.m_devicePosition[0];
      p.y = RobotVars->phantom_data.m_devicePosition[1];
      p.z = RobotVars->phantom_data.m_devicePosition[2];
      marker_force_vector.points.push_back(p);
      
      hduVector3Dd Fdir = RobotVars->haptics_data.applied_force;
      if(Fdir.magnitude() > 3.3)
      {
	Fdir.normalize();
	Fdir = Fdir * 3.3;
      }
      p2.x = RobotVars->phantom_data.m_devicePosition[0] + Fdir[0]*10;
      p2.y = RobotVars->phantom_data.m_devicePosition[1] + Fdir[1]*10;
      p2.z = RobotVars->phantom_data.m_devicePosition[2] + Fdir[2]*10;
      marker_force_vector.points.push_back(p2);
      //marker scale
      marker_force_vector.scale.x = Fdir.magnitude()*5; 
      marker_force_vector.scale.y = Fdir.magnitude()*6;
      marker_force_vector.scale.z = Fdir.magnitude()*7;
      //marker color
      marker_force_vector.color.r=0;
      marker_force_vector.color.g=0;
      marker_force_vector.color.b=0;
      marker_force_vector.color.a=1;
      marker_force_vector.id=0;
      marker_force_vector.lifetime = ros::Duration();
      marker_vector.push_back(marker_force_vector);
      
      //vector magnitude
      visualization_msgs::Marker marker_force_magnitude;
      marker_force_magnitude.header.frame_id = "/phantom_world_corrected";
      //marker info
      marker_force_magnitude.header.stamp = ros::Time::now();
      marker_force_magnitude.ns = "text_force";
      marker_force_magnitude.action = visualization_msgs::Marker::ADD;
      //marker type
      marker_force_magnitude.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      text = str(boost::format("%3.1f") % AvoidMinusZero(Fdir.magnitude())) + " N";
      marker_force_magnitude.text = text;
      marker_force_magnitude.pose.position.x = RobotVars->phantom_data.m_devicePosition[0] + Fdir[0]*10;
      marker_force_magnitude.pose.position.y = RobotVars->phantom_data.m_devicePosition[1] + Fdir[1]*10;
      marker_force_magnitude.pose.position.z = RobotVars->phantom_data.m_devicePosition[2] + Fdir[2]*10;
      marker_force_magnitude.scale.z = 10.0; 
      //marker color
      marker_force_magnitude.color = marker_force_vector.color;
      marker_force_magnitude.color.a=0.8;
      marker_force_magnitude.id=0;
      marker_force_magnitude.lifetime = ros::Duration();
      marker_vector.push_back(marker_force_magnitude);
      // *************************************************************************************
      // PLANE AND POINTS
      //point1
      if(RobotVars->haptics_data.demo_2_point_1.magnitude() > 0.0)
      {
	visualization_msgs::Marker marker_point_sphere;
	marker_point_sphere.header.frame_id = "/phantom_world_corrected";
	//marker info
	marker_point_sphere.header.stamp = ros::Time::now();
	marker_point_sphere.ns = "point_1";
	marker_point_sphere.action = visualization_msgs::Marker::ADD;
	//marker type
	marker_point_sphere.type = visualization_msgs::Marker::SPHERE;
	//marker scale
	marker_point_sphere.scale.x = 0.1*RobotVars->haptics_data.arm_main_sphere_min_Radius; 
	marker_point_sphere.scale.y = marker_point_sphere.scale.x; 
	marker_point_sphere.scale.z = marker_point_sphere.scale.x; 
	//marker pose
	marker_point_sphere.pose.position.x = RobotVars->haptics_data.demo_2_point_1[0];
	marker_point_sphere.pose.position.y = RobotVars->haptics_data.demo_2_point_1[1];
	marker_point_sphere.pose.position.z = RobotVars->haptics_data.demo_2_point_1[2];
	marker_point_sphere.pose.orientation = marker_outer_sphere.pose.orientation;
	//marker color
	marker_point_sphere.color.r=153.0/255.0;
	marker_point_sphere.color.g=51.0/255.0;
	marker_point_sphere.color.b=51.0/255.0;
	marker_point_sphere.color.a=1;
	marker_point_sphere.id=0;
	marker_point_sphere.lifetime = ros::Duration();
	marker_vector.push_back(marker_point_sphere);
	//text
	visualization_msgs::Marker marker_point_text;
	marker_point_text.header.frame_id = "/phantom_world_corrected";
	//marker info
	marker_point_text.header.stamp = ros::Time::now();
	marker_point_text.ns = "point_1_text";
	marker_point_text.action = visualization_msgs::Marker::ADD;
	//marker type
	marker_point_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	text = "Point 1";
	marker_point_text.text = text;
	marker_point_text.pose.position = marker_point_sphere.pose.position;
	marker_point_text.scale.z = 7.0;
	//marker color
	marker_point_text.color = marker_point_sphere.color;
	marker_point_text.color.a=1;
	marker_point_text.id=0;
	marker_point_text.lifetime = ros::Duration();
	marker_vector.push_back(marker_point_text);
      }
      //point2
      if(RobotVars->haptics_data.demo_2_point_2.magnitude() > 0.0)
      {
	visualization_msgs::Marker marker_point2_sphere;
	marker_point2_sphere.header.frame_id = "/phantom_world_corrected";
	//marker info
	marker_point2_sphere.header.stamp = ros::Time::now();
	marker_point2_sphere.ns = "point_2";
	marker_point2_sphere.action = visualization_msgs::Marker::ADD;
	//marker type
	marker_point2_sphere.type = visualization_msgs::Marker::SPHERE;
	//marker scale
	marker_point2_sphere.scale.x = 0.1*RobotVars->haptics_data.arm_main_sphere_min_Radius; 
	marker_point2_sphere.scale.y = marker_point2_sphere.scale.x; 
	marker_point2_sphere.scale.z = marker_point2_sphere.scale.x; 
	//marker pose
	marker_point2_sphere.pose.position.x = RobotVars->haptics_data.demo_2_point_2[0];
	marker_point2_sphere.pose.position.y = RobotVars->haptics_data.demo_2_point_2[1];
	marker_point2_sphere.pose.position.z = RobotVars->haptics_data.demo_2_point_2[2];
	marker_point2_sphere.pose.orientation = marker_outer_sphere.pose.orientation;
	//marker color
	marker_point2_sphere.color.r=153.0/255.0;
	marker_point2_sphere.color.g=51.0/255.0;
	marker_point2_sphere.color.b=51.0/255.0;
	marker_point2_sphere.color.a=1;
	marker_point2_sphere.id=0;
	marker_point2_sphere.lifetime = ros::Duration();
	marker_vector.push_back(marker_point2_sphere);
	//text
	visualization_msgs::Marker marker_point2_text;
	marker_point2_text.header.frame_id = "/phantom_world_corrected";
	//marker info
	marker_point2_text.header.stamp = ros::Time::now();
	marker_point2_text.ns = "point_2_text";
	marker_point2_text.action = visualization_msgs::Marker::ADD;
	//marker type
	marker_point2_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	text = "Point 2";
	marker_point2_text.text = text;
	marker_point2_text.pose.position = marker_point2_sphere.pose.position;
	marker_point2_text.scale.z = 7.0;
	//marker color
	marker_point2_text.color = marker_point2_sphere.color;
	marker_point2_text.color.a=1;
	marker_point2_text.id=0;
	marker_point2_text.lifetime = ros::Duration();
	marker_vector.push_back(marker_point2_text);
      }
      //point3
      if(RobotVars->haptics_data.demo_2_point_3.magnitude() > 0.0)
      {
	visualization_msgs::Marker marker_point3_sphere;
	marker_point3_sphere.header.frame_id = "/phantom_world_corrected";
	//marker info
	marker_point3_sphere.header.stamp = ros::Time::now();
	marker_point3_sphere.ns = "point_3";
	marker_point3_sphere.action = visualization_msgs::Marker::ADD;
	//marker type
	marker_point3_sphere.type = visualization_msgs::Marker::SPHERE;
	//marker scale
	marker_point3_sphere.scale.x = 0.1*RobotVars->haptics_data.arm_main_sphere_min_Radius; 
	marker_point3_sphere.scale.y = marker_point3_sphere.scale.x; 
	marker_point3_sphere.scale.z = marker_point3_sphere.scale.x; 
	//marker pose
	marker_point3_sphere.pose.position.x = RobotVars->haptics_data.demo_2_point_3[0];
	marker_point3_sphere.pose.position.y = RobotVars->haptics_data.demo_2_point_3[1];
	marker_point3_sphere.pose.position.z = RobotVars->haptics_data.demo_2_point_3[2];
	marker_point3_sphere.pose.orientation = marker_outer_sphere.pose.orientation;
	//marker color
	marker_point3_sphere.color.r=153.0/255.0;
	marker_point3_sphere.color.g=51.0/255.0;
	marker_point3_sphere.color.b=51.0/255.0;
	marker_point3_sphere.color.a=1;
	marker_point3_sphere.id=0;
	marker_point3_sphere.lifetime = ros::Duration();
	marker_vector.push_back(marker_point3_sphere);
	//text
	visualization_msgs::Marker marker_point_text;
	marker_point_text.header.frame_id = "/phantom_world_corrected";
	//marker info
	marker_point_text.header.stamp = ros::Time::now();
	marker_point_text.ns = "point_3_text";
	marker_point_text.action = visualization_msgs::Marker::ADD;
	//marker type
	marker_point_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	text = "Point 3";
	marker_point_text.text = text;
	marker_point_text.pose.position = marker_point3_sphere.pose.position;
	marker_point_text.scale.z = 7.0;
	//marker color
	marker_point_text.color = marker_point3_sphere.color;
	marker_point_text.color.a=1;
	marker_point_text.id=0;
	marker_point_text.lifetime = ros::Duration();
	marker_vector.push_back(marker_point_text);
      }
      hduVector3Dd normalLocal = RobotVars->haptics_data.demo_2_Plane.normal();
      //plane
      if(normalLocal.magnitude() > 0.0)
      {
	hduVector3Dd a = normalLocal;
	hduVector3Dd n = RobotVars->haptics_data.demo_2_point_2 - ((RobotVars->haptics_data.demo_2_point_1 + RobotVars->haptics_data.demo_2_point_2 + RobotVars->haptics_data.demo_2_point_3) / 3.0);
	hduVector3Dd s = crossProduct(a,n);
	
	n.normalize();
	s.normalize();
	a.normalize();

#if ROS_VERSION_MINIMUM(1, 8, 0)   //At least FUERTE, V. Santos, 27-Mai-2013
	tf::Quaternion plane_quat;
	tf::Matrix3x3 mat = tf::Matrix3x3(n[0], s[0], a[0],
			  n[1], s[1], a[1],
			  n[2], s[2], a[2]);
#else
	btQuaternion plane_quat;
	btMatrix3x3 mat = btMatrix3x3(n[0], s[0], a[0],
			  n[1], s[1], a[1],
			  n[2], s[2], a[2]);
#endif
	mat.getRotation(plane_quat);

	//PLANE
	visualization_msgs::Marker marker_plane;
	marker_plane.header.frame_id = "/phantom_world_corrected";
	//marker info
	marker_plane.header.stamp = ros::Time::now();
	marker_plane.ns = "drawing_plane";
	marker_plane.action = visualization_msgs::Marker::ADD;
	//marker type
	marker_plane.type = visualization_msgs::Marker::CUBE;
	//marker scale
	marker_plane.scale.x = 200;
	marker_plane.scale.y = 200;
	marker_plane.scale.z = 0.01;
	
	//marker pose
	marker_plane.pose.position.x = (RobotVars->haptics_data.demo_2_point_1[0] + RobotVars->haptics_data.demo_2_point_2[0] + RobotVars->haptics_data.demo_2_point_3[0]) / 3.0;
	marker_plane.pose.position.y = (RobotVars->haptics_data.demo_2_point_1[1] + RobotVars->haptics_data.demo_2_point_2[1] + RobotVars->haptics_data.demo_2_point_3[1]) / 3.0;
	marker_plane.pose.position.z = (RobotVars->haptics_data.demo_2_point_1[2] + RobotVars->haptics_data.demo_2_point_2[2] + RobotVars->haptics_data.demo_2_point_3[2]) / 3.0;
	
	tf::quaternionTFToMsg(plane_quat.normalize(),
			      marker_plane.pose.orientation);		
	
	//marker color
	marker_plane.color.r=0;
	marker_plane.color.g=245.0/255.0;
	marker_plane.color.b=255.0/255.0;
	marker_plane.color.a=0.8;
	marker_plane.id=0;
	marker_plane.lifetime = ros::Duration();
	marker_vector.push_back(marker_plane);
/*	
	visualization_msgs::Marker marker_n_vector;
	marker_n_vector.header.frame_id = "/phantom_world_corrected";
	//marker info
	marker_n_vector.header.stamp = ros::Time::now();
	marker_n_vector.ns = "n_vector";
	marker_n_vector.action = visualization_msgs::Marker::ADD;
	//marker type
	marker_n_vector.type = visualization_msgs::Marker::ARROW;
	//marker points
	marker_n_vector.points.clear();
	p = marker_plane.pose.position;
	marker_n_vector.points.push_back(p);
	p2.x = p.x + normalLocal[0]/10.0;
	p2.y = p.y + normalLocal[1]/10.0;
	p2.z = p.z + normalLocal[2]/10.0;
	marker_n_vector.points.push_back(p2);
	//marker scale
	marker_n_vector.scale.x = 3; 
	marker_n_vector.scale.y = 4;
	marker_n_vector.scale.z = 3;
	//marker color
	marker_n_vector.color.r=0;
	marker_n_vector.color.g=0;
	marker_n_vector.color.b=0;
	marker_n_vector.color.a=1;
	marker_n_vector.id=0;
	marker_n_vector.lifetime = ros::Duration();
	marker_vector.push_back(marker_n_vector);*/
      }
    }
  }
  else if(RobotVars->haptics_data.chosen_demo == 4 && RobotVars->haptic_loop_start)
  {
    // |||||||||||||||||||||||||||||||||||||||||||||||||||||| LEG WORKSPACE DEMO CONSTRUCTIONS ||||||||||||||||||||||||||||||||||||||||||||||||||||||
    //OUTER SPHERE
    visualization_msgs::Marker marker_outer_sphere;
    marker_outer_sphere.header.frame_id = "/phantom_world_corrected";
    //marker info
    marker_outer_sphere.header.stamp = ros::Time::now();
    marker_outer_sphere.ns = "outer_sphere";
    marker_outer_sphere.action = visualization_msgs::Marker::ADD;
    //marker type
    marker_outer_sphere.type = visualization_msgs::Marker::SPHERE;
    //marker scale
    marker_outer_sphere.scale.x = 2.0*RobotVars->haptics_data.leg_main_sphere_max_Radius; 
    marker_outer_sphere.scale.y = 2.0*RobotVars->haptics_data.leg_main_sphere_max_Radius; 
    marker_outer_sphere.scale.z = 2.0*RobotVars->haptics_data.leg_main_sphere_max_Radius; 
    //marker pose
    marker_outer_sphere.pose.position.x = RobotVars->haptics_data.leg_main_spheresPosition[0];
    marker_outer_sphere.pose.position.y = RobotVars->haptics_data.leg_main_spheresPosition[1];
    marker_outer_sphere.pose.position.z = RobotVars->haptics_data.leg_main_spheresPosition[2];
    marker_outer_sphere.pose.orientation.x = 0.0;
    marker_outer_sphere.pose.orientation.y = 0.0;
    marker_outer_sphere.pose.orientation.z = 0.0;
    marker_outer_sphere.pose.orientation.w = 1.0;
    
    marker_outer_sphere.color.r=0;
    marker_outer_sphere.color.g=1;
    marker_outer_sphere.color.b=0;
    marker_outer_sphere.color.a=0.1;
    marker_outer_sphere.id=0;
    marker_outer_sphere.lifetime = ros::Duration();
    
    marker_vector.push_back(marker_outer_sphere);
    
    //INNER SPHERE
    visualization_msgs::Marker marker_inner_sphere;
    marker_inner_sphere.header.frame_id = "/phantom_world_corrected";
    //marker info
    marker_inner_sphere.header.stamp = ros::Time::now();
    marker_inner_sphere.ns = "inner_sphere";
    marker_inner_sphere.action = visualization_msgs::Marker::ADD;
    //marker type
    marker_inner_sphere.type = visualization_msgs::Marker::SPHERE;
    //marker scale
    marker_inner_sphere.scale.x = 2.0*RobotVars->haptics_data.leg_main_sphere_min_Radius; 
    marker_inner_sphere.scale.y = 2.0*RobotVars->haptics_data.leg_main_sphere_min_Radius; 
    marker_inner_sphere.scale.z = 2.0*RobotVars->haptics_data.leg_main_sphere_min_Radius; 
    //marker pose
    marker_inner_sphere.pose.position.x = RobotVars->haptics_data.leg_main_spheresPosition[0];
    marker_inner_sphere.pose.position.y = RobotVars->haptics_data.leg_main_spheresPosition[1];
    marker_inner_sphere.pose.position.z = RobotVars->haptics_data.leg_main_spheresPosition[2];
    marker_inner_sphere.pose.orientation.x = 0.0;
    marker_inner_sphere.pose.orientation.y = 0.0;
    marker_inner_sphere.pose.orientation.z = 0.0;
    marker_inner_sphere.pose.orientation.w = 1.0;
    //marker color
    marker_inner_sphere.color.r=0;
    marker_inner_sphere.color.g=0;
    marker_inner_sphere.color.b=1;
    marker_inner_sphere.color.a=0.15;
    marker_inner_sphere.id=0;
    marker_inner_sphere.lifetime = ros::Duration();
    marker_vector.push_back(marker_inner_sphere);
    
    // |||||||||||||||||||||||||||||||||||||||||||||||||||||| LEG BALANCING DEMO CONSTRUCTIONS ||||||||||||||||||||||||||||||||||||||||||||||||||||||
    //FORCE VECTOR
    visualization_msgs::Marker marker_force_vector;
    marker_force_vector.header.frame_id = "/phantom_world_corrected";
    //marker info
    marker_force_vector.header.stamp = ros::Time::now();
    marker_force_vector.ns = "force_vector";
    marker_force_vector.action = visualization_msgs::Marker::ADD;
    //marker type
    marker_force_vector.type = visualization_msgs::Marker::ARROW;
    //marker points
    marker_force_vector.points.clear();
    p.x = RobotVars->phantom_data.m_devicePosition[0];
    p.y = RobotVars->phantom_data.m_devicePosition[1];
    p.z = RobotVars->phantom_data.m_devicePosition[2];
    marker_force_vector.points.push_back(p);
    
    hduVector3Dd Fdir = RobotVars->haptics_data.applied_force;
    if(Fdir.magnitude() > 3.3)
    {
      Fdir.normalize();
      Fdir = Fdir * 3.3;
    }
    p2.x = RobotVars->phantom_data.m_devicePosition[0] + Fdir[0]*10;
    p2.y = RobotVars->phantom_data.m_devicePosition[1] + Fdir[1]*10;
    p2.z = RobotVars->phantom_data.m_devicePosition[2] + Fdir[2]*10;
    marker_force_vector.points.push_back(p2);
    //marker scale
    marker_force_vector.scale.x = Fdir.magnitude()*5; 
    marker_force_vector.scale.y = Fdir.magnitude()*6;
    marker_force_vector.scale.z = Fdir.magnitude()*7;
    //marker color
    marker_force_vector.color.r=0;
    marker_force_vector.color.g=0;
    marker_force_vector.color.b=0;
    marker_force_vector.color.a=1;
    marker_force_vector.id=0;
    marker_force_vector.lifetime = ros::Duration();
    marker_vector.push_back(marker_force_vector);
    
    //vector magnitude
    visualization_msgs::Marker marker_force_magnitude;
    marker_force_magnitude.header.frame_id = "/phantom_world_corrected";
    //marker info
    marker_force_magnitude.header.stamp = ros::Time::now();
    marker_force_magnitude.ns = "text_force";
    marker_force_magnitude.action = visualization_msgs::Marker::ADD;
    //marker type
    marker_force_magnitude.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text = str(boost::format("%3.1f") % AvoidMinusZero(Fdir.magnitude())) + " N";
    marker_force_magnitude.text = text;
    marker_force_magnitude.pose.position.x = RobotVars->phantom_data.m_devicePosition[0] + Fdir[0]*10;
    marker_force_magnitude.pose.position.y = RobotVars->phantom_data.m_devicePosition[1] + Fdir[1]*10;
    marker_force_magnitude.pose.position.z = RobotVars->phantom_data.m_devicePosition[2] + Fdir[2]*10;
    marker_force_magnitude.scale.z = 10.0; 
    //marker color
    marker_force_magnitude.color = marker_force_vector.color;
    marker_force_magnitude.color.a=0.9;
    marker_force_magnitude.id=0;
    marker_force_magnitude.lifetime = ros::Duration();
    marker_vector.push_back(marker_force_magnitude);
    
    
    //Center of Gravity
    visualization_msgs::Marker marker_COG;
    marker_COG.header.frame_id = "/phantom_world_corrected";
    //marker info
    marker_COG.header.stamp = ros::Time::now();
    marker_COG.ns = "COG_sphere";
    marker_COG.action = visualization_msgs::Marker::ADD;
    //marker type
    marker_COG.type = visualization_msgs::Marker::SPHERE;
    //marker scale
    marker_COG.scale.x = 10.0; 
    marker_COG.scale.y = marker_COG.scale.x; 
    marker_COG.scale.z = marker_COG.scale.x; 
    //marker pose
    if(RobotVars->parameters.kinematic_model==4)
    {  
      marker_COG.pose.position.x = RobotVars->haptics_data.leg_main_spheresPosition[0] + RobotVars->robot_kin_data.COG_detached_leg_left[0];
      marker_COG.pose.position.y = RobotVars->haptics_data.leg_main_spheresPosition[1] + RobotVars->robot_kin_data.COG_detached_leg_left[1];
      marker_COG.pose.position.z = RobotVars->haptics_data.leg_main_spheresPosition[2] + RobotVars->robot_kin_data.COG_detached_leg_left[2];
    }
    else
    {   
      marker_COG.pose.position.x = RobotVars->haptics_data.leg_main_spheresPosition[0] + RobotVars->robot_kin_data.COG_detached_leg_right[0];
      marker_COG.pose.position.y = RobotVars->haptics_data.leg_main_spheresPosition[1] + RobotVars->robot_kin_data.COG_detached_leg_right[1];
      marker_COG.pose.position.z = RobotVars->haptics_data.leg_main_spheresPosition[2] + RobotVars->robot_kin_data.COG_detached_leg_right[2];
    }
    marker_COG.pose.orientation.x = 0.0;
    marker_COG.pose.orientation.y = marker_COG.pose.orientation.x;
    marker_COG.pose.orientation.z = marker_COG.pose.orientation.x;
    marker_COG.pose.orientation.w = 1.0;
    //marker color
    marker_COG.color.r=1;
    marker_COG.color.g=1;
    marker_COG.color.b=0;
    marker_COG.color.a=0.9;
    marker_COG.id=0;
    marker_COG.lifetime = ros::Duration();
    marker_vector.push_back(marker_COG);
    
    //cog_tag
    visualization_msgs::Marker marker_cog_tag;
    marker_cog_tag.header.frame_id = "/phantom_world_corrected";
    //marker info
    marker_cog_tag.header.stamp = ros::Time::now();
    marker_cog_tag.ns = "cog_tag";
    marker_cog_tag.action = visualization_msgs::Marker::ADD;
    //marker type
    marker_cog_tag.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text = "COG";
    marker_cog_tag.text = text;
    marker_cog_tag.pose.position = marker_COG.pose.position;
    marker_cog_tag.scale.z = 7.0; 
    //marker color
    marker_cog_tag.color.r = 0;
    marker_cog_tag.color.g = 0;
    marker_cog_tag.color.b = 0;
    marker_cog_tag.color.a=1.0;
    marker_cog_tag.id=0;
    marker_cog_tag.lifetime = ros::Duration();
    marker_vector.push_back(marker_cog_tag);
    
    //Foot AREA PLANE
    visualization_msgs::Marker marker_foot_area_plane;
    marker_foot_area_plane.header.frame_id = "/phantom_world_corrected";
    //marker info
    marker_foot_area_plane.header.stamp = ros::Time::now();
    marker_foot_area_plane.ns = "foot_area_plane";
    marker_foot_area_plane.action = visualization_msgs::Marker::ADD;
    //marker type
    marker_foot_area_plane.type = visualization_msgs::Marker::CUBE;
    //marker scale
    marker_foot_area_plane.scale.x = RobotVars->humanoid_f->GetRobotDimension(FOOT_LENGTH)/ (double)RobotVars->parameters.pos_coord_scale; 
    marker_foot_area_plane.scale.y = RobotVars->humanoid_f->GetRobotDimension(FOOT_WIDTH)/ (double)RobotVars->parameters.pos_coord_scale; 
    marker_foot_area_plane.scale.z = 0.01;
    //marker pose    
    marker_foot_area_plane.pose.position.x = RobotVars->haptics_data.leg_main_spheresPosition[0];
    marker_foot_area_plane.pose.position.y = RobotVars->haptics_data.leg_main_spheresPosition[1];
    marker_foot_area_plane.pose.position.z = RobotVars->haptics_data.leg_main_spheresPosition[2]- (RobotVars->humanoid_f->GetRobotDimension(ANKLE_HEIGHT)/ (double)RobotVars->parameters.pos_coord_scale);
    marker_foot_area_plane.pose.orientation = marker_COG.pose.orientation;
    //marker color
    marker_foot_area_plane.color.r=70.0/255.0;
    marker_foot_area_plane.color.g=130.0/255.0;
    marker_foot_area_plane.color.b=180.0/255.0;
    marker_foot_area_plane.color.a=0.75;
    marker_foot_area_plane.id=0;
    marker_foot_area_plane.lifetime = ros::Duration();
    marker_vector.push_back(marker_foot_area_plane);
    
    // plane tag
    visualization_msgs::Marker marker_plane_tag;
    marker_plane_tag.header.frame_id = "/phantom_world_corrected";
    //marker info
    marker_plane_tag.header.stamp = ros::Time::now();
    marker_plane_tag.ns = "foot_plane_tag";
    marker_plane_tag.action = visualization_msgs::Marker::ADD;
    //marker type
    marker_plane_tag.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text = "Foot Area";
    marker_plane_tag.text = text;
    marker_plane_tag.pose.position.x = marker_foot_area_plane.pose.position.x + marker_foot_area_plane.scale.x/2.0;
    marker_plane_tag.pose.position.y = marker_foot_area_plane.pose.position.y + marker_foot_area_plane.scale.y/2.0;
    marker_plane_tag.pose.position.z = marker_foot_area_plane.pose.position.z;
    marker_plane_tag.scale.z = 5.0; 
    //marker color
    marker_plane_tag.color.r = 0;
    marker_plane_tag.color.g = 0;
    marker_plane_tag.color.b = 0;
    marker_plane_tag.color.a=1.0;
    marker_plane_tag.id=0;
    marker_plane_tag.lifetime = ros::Duration();
    marker_vector.push_back(marker_plane_tag);
    
    //Center of Pressure
    visualization_msgs::Marker marker_COP;
    marker_COP.header.frame_id = "/phantom_world_corrected";
    //marker info
    marker_COP.header.stamp = ros::Time::now();
    marker_COP.ns = "COP_sphere";
    marker_COP.action = visualization_msgs::Marker::ADD;
    //marker type
    marker_COP.type = visualization_msgs::Marker::SPHERE;
    //marker scale
    marker_COP.scale.x = marker_COG.scale.x; 
    marker_COP.scale.y = marker_COP.scale.x; 
    marker_COP.scale.z = 1; 
    //marker pose
    marker_COP.pose.position = marker_COG.pose.position;
    marker_COP.pose.position.z = marker_foot_area_plane.pose.position.z;
    marker_COP.pose.orientation = marker_COG.pose.orientation;
    //marker color
    marker_COP.color.r=178.0/255.0;
    marker_COP.color.g=34.0/255.0;
    marker_COP.color.b=34.0/255.0;
    marker_COP.color.a=1;
    marker_COP.id=0;
    marker_COP.lifetime = ros::Duration();
    marker_vector.push_back(marker_COP);
    
    //cop_tag
    visualization_msgs::Marker marker_cop_tag;
    marker_cop_tag.header.frame_id = "/phantom_world_corrected";
    //marker info
    marker_cop_tag.header.stamp = ros::Time::now();
    marker_cop_tag.ns = "cop_tag";
    marker_cop_tag.action = visualization_msgs::Marker::ADD;
    //marker type
    marker_cop_tag.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text = "COP";
    marker_cop_tag.text = text;
    marker_cop_tag.pose.position = marker_COP.pose.position;
    marker_cop_tag.scale.z = 10.0; 
    //marker color
    marker_cop_tag.color.r = 0;
    marker_cop_tag.color.g = 0;
    marker_cop_tag.color.b = 0;
    marker_cop_tag.color.a=1.0;
    marker_cop_tag.id=0;
    marker_cop_tag.lifetime = ros::Duration();
    marker_vector.push_back(marker_cop_tag);
  }
  else
  {
    
  }
}
