/**
 * @file phantom_control_vrep.cpp
 * @brief Node responsible for handling the communication between the PHANToM device and V-REP.
 *        Connects to V-REP and joystick simultaneously, returning the device status to the simulator
 *        and updating the received forces. 
 *        Implements joint-by-joint robot control and inverse kinematics control when combined with the joystick, 
 *        or inverse kinematics when alone, using a polynomial-like trajectory generation. 
 *        The operation mode depends on the V-REP scene, and on the state selected by Button 2.
 *        It uses both ROS and OpenHaptics. 
 * @version v1.0
 * @author João O. Barros
 * @date February 2015
 */


#include <humanoid_simulation/humanoid_simulation.h>

#define PI 3.14159265

using namespace std;

static device_state state;

HDdouble base_torque[3];
HDdouble base_force[3];

HDSchedulerHandle gCallbackHandle = HD_INVALID_HANDLE;
HDCallbackCode HDCALLBACK UpdateCalibrationCallback(void *pUserData);
HDboolean CheckCalibration();

ros::ServiceClient object_handles_srv;
ros::ServiceClient joint_handles_srv;
ros::Publisher command_pelvis_position_pub;
ros::Publisher command_pelvis_position_stamped_pub;
ros::Publisher command_joints_position_pub;
ros::Publisher command_left_joints_position_pub;
ros::Publisher command_right_joints_position_pub;
ros::Subscriber feedback_force_filtered_sub;
ros::Subscriber simulation_state_sub;

vrep_common::simRosGetObjectGroupData joint_handles;
geometry_msgs::Point pelvis_target;
geometry_msgs::Point pelvis_target_init;
geometry_msgs::PointStamped pelvis_target_stamped;
vrep_common::JointSetStateData joint_target;
vrep_common::JointSetStateData left_joint_target;
vrep_common::JointSetStateData right_joint_target;

vector<int> left_joint_handle;
vector<int> right_joint_handle;

/*const double L_1 = 135;
const double L_2 = 135;
const double L_3 = 25;
const double A = 35;
const double L_4 = L_1 + A;*/

int command_method (0);
int previous_state_0 (0);
int previous_state_1 (0);

int simulator_state(4);
double simulationTime (0);
double time_inc = 0.05;
double poly_time;
string move_type = "move_down";
int scale_div = 1700;


HDCallbackCode HDCALLBACK SchedulerCallback(void *pUserData)
{
    /* Scheduler callback responsible for device data extraction and force rendering.
     * Calling operations within a haptic frame ensures consistency for the data being used, since the device
     * state remains the same within the frame. */
     
    HDErrorInfo error;
    
    HDint nButtons = 0;

    hdBeginFrame(hdGetCurrentDevice());
    
    hdGetDoublev(HD_CURRENT_POSITION, state.device_position); 
    hdGetDoublev(HD_CURRENT_JOINT_ANGLES, state.joint_angle);

    hdGetIntegerv(HD_CURRENT_BUTTONS, &nButtons);
    hdSetDoublev(HD_CURRENT_FORCE, base_force);   /* hdSetDoublev(HD_CURRENT_JOINT_TORQUE, base_torque); */

    
    /* ============ Joystick angles cross-checking with literature: PHANToM OMNI Haptic Device: Kinematic and Manipulability ============ */
    
    /*double theta_1 = -atan2(state.device_position[0], state.device_position[2] + L_4);
    
    double R = sqrt((state.device_position[0]*state.device_position[0]) + ((state.device_position[2] + L_4)*(state.device_position[2] + L_4)));
    double r = sqrt((state.device_position[0]*state.device_position[0]) + ((state.device_position[2] + L_4)*(state.device_position[2] + L_4)) + ((state.device_position[1] - L_3)*(state.device_position[1] - L_3)));
    double beta = atan2((state.device_position[1] - L_3), R);
    double gamma = acos((L_1*L_1 + r*r - L_2*L_2) / (2*L_1*r));
    double theta_2 = gamma + beta;
    
    double alpha = acos((L_1*L_1 + L_2*L_2 - r*r) / (2*L_1*L_2));
    double theta_3 = theta_2 + alpha - (M_PI/2);*/
    
    
    state.button_state[0] = 
        (nButtons & HD_DEVICE_BUTTON_1) ? HD_TRUE : HD_FALSE;
    
    state.button_state[1] = 
        (nButtons & HD_DEVICE_BUTTON_2) ? HD_TRUE : HD_FALSE;              
        
    if (!state.button_state[1])
        previous_state_1 = 0;
    else if (state.button_state[1] && previous_state_1 == 0)
    {
        if (command_method == 0)
        {
	  command_method = 1;
	  cout << "Left leg configuration selected." << endl;
        }
        else if (command_method == 1)
        {
	  command_method = 2;
	  cout << "Right leg configuration selected." << endl;
        }
        else if (command_method == 2)
        {
	  /* Inverse kinematics operation or torque operation for both legs, depending on the V-REP scene launched. */
	  
	  command_method = 0;
	  cout << "Dual leg configuration selected." << endl;
        }
        
        previous_state_1 = 1;
    }
    else if (state.button_state[1] && previous_state_1 == 1)
    {
        if (command_method == 0)
	  command_method = 0;
        else if (command_method == 1)
	  command_method = 1;
        else if (command_method == 2)
	  command_method = 2;
	 
        previous_state_1 = 1;
    } 
   
    hdEndFrame(hdGetCurrentDevice());
    
    
    /* Joint angles and end-effector coordinates transformation to V-REP reference frame. */
    
    if (command_method == 0)
        state.joint_angle[0] = -state.joint_angle[0];
    else
        state.joint_angle[0] = state.joint_angle[0]; 
   
    state.joint_angle[1] = state.joint_angle[1];
        
    double betaa = M_PI/2 - state.joint_angle[1];
    state.joint_angle[2] = -(M_PI - betaa - state.joint_angle[2]);
    
    
    state.device_position[0] = -state.device_position[0];
    
    double real_z = state.device_position[1];
    state.device_position[1] = state.device_position[2];
    state.device_position[2] = real_z;
    
    if (!state.button_state[0])
        previous_state_0 = 0;      
    else if (state.button_state[0] && previous_state_0 == 0) 
    {
        pelvis_target_init.x = ((-state.device_position[0]))/scale_div;
        pelvis_target_init.y = ((-state.device_position[1]))/scale_div;
        pelvis_target_init.z = ((-state.device_position[2]))/scale_div;
        
        previous_state_0 = 1; 
        
        cout << "New reference chosen." << endl;
    }
    
    /*cout << "Joint angles: " << "(" << state.joint_angle[0]*(180/M_PI) << ", " << state.joint_angle[1]*(180/M_PI) << "," << state.joint_angle[2]*(180/M_PI) << ")" << endl;
    cout << "Device position: " << "(" << state.device_position[0]/1700 << ", " << state.device_position[1]/1700 << "," << state.device_position[2]/1700 << ")" << endl;*/
 
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, "Error during main scheduler callback\n");
        if (hduIsSchedulerError(&error))
            return HD_CALLBACK_DONE;
    }

    return HD_CALLBACK_CONTINUE;
}
     

void setRobotPosition(double& time)
{  
    /* ============================================================================ */
    /* ===================== Inverse Kinematics control mode ====================== */
    /* ============================================================================ */
    
    double z0=-0.4032;
    double zi=-0.4072;
    double zf=-0.3572;
    double tf=2;

    /*double y0=0;
    double yi=0.05;
    double yf=-0.05;   
    double tf=2;*/
 
    if (time <= tf)
    {
        pelvis_target.x = 0;
        pelvis_target.y = 0;
        pelvis_target.z = (6*(zi-z0)*pow(time,5))/pow(tf,5) - (15*(zi-z0)*pow(time,4))/pow(tf,4) + (10*(zi-z0)*pow(time,3))/pow(tf,3) + z0;
    }
    else
    {
        if (poly_time <= tf)
	  poly_time = poly_time + time_inc;
        else
	  poly_time = time_inc;     
       
        /*if (pelvis_target.y > -0.05 && move_type == "move_down")
	  pelvis_target.y = (6*(yf-yi)*pow(poly_time,5))/pow(tf,5) - (15*(yf-yi)*pow(poly_time,4))/pow(tf,4) + (10*(yf-yi)*pow(poly_time,3))/pow(tf,3) + yi; 
        else if (pelvis_target.y < 0.05 && move_type == "move_up")
	  pelvis_target.y = (6*(yi-yf)*pow(poly_time,5))/pow(tf,5) - (15*(yi-yf)*pow(poly_time,4))/pow(tf,4) + (10*(yi-yf)*pow(poly_time,3))/pow(tf,3) + yf;*/

        if (pelvis_target.z < zf && move_type == "move_down")
	  pelvis_target.z = (6*(zf-zi)*pow(poly_time,5))/pow(tf,5) - (15*(zf-zi)*pow(poly_time,4))/pow(tf,4) + (10*(zf-zi)*pow(poly_time,3))/pow(tf,3) + zi; 
        else if (pelvis_target.z > -zi && move_type == "move_up")
	  pelvis_target.z = (6*(zi-zf)*pow(poly_time,5))/pow(tf,5) - (15*(zi-zf)*pow(poly_time,4))/pow(tf,4) + (10*(zi-zf)*pow(poly_time,3))/pow(tf,3) + zf;
        
        
        if (poly_time < tf)
	  move_type = move_type;
        else
        {
	  if (move_type == "move_down")
	      move_type = "move_up";
	  else
	      move_type = "move_down";
        }
    }
    
    pelvis_target.x = ((-state.device_position[0]))/scale_div - pelvis_target_init.x;
    pelvis_target.y = ((-state.device_position[1]))/scale_div - pelvis_target_init.y;
    pelvis_target.z = ((-state.device_position[2]))/scale_div - pelvis_target_init.z + z0;
       
    if (pelvis_target.x > 0.05)
        pelvis_target.x = 0.05;
    if (pelvis_target.x < -0.05)
        pelvis_target.x = -0.05;
    
    if (pelvis_target.y > 0.05)
        pelvis_target.y = 0.05;
    if (pelvis_target.y < -0.05)
        pelvis_target.y = -0.05;
    
    if (pelvis_target.z > zf)
        pelvis_target.z = zf;
    if (pelvis_target.z < zi)
        pelvis_target.z = zi;
    
    pelvis_target_stamped.header.stamp.sec = simulationTime*100000;
    pelvis_target_stamped.point.x = pelvis_target.x;
    pelvis_target_stamped.point.y = pelvis_target.y;
    pelvis_target_stamped.point.z = pelvis_target.z;
    
    
    /* ============================================================================= */
    /* =================== Joint-by-joint (Torque) control mode ==================== */
    /* ============================================================================= */
        
    joint_target.handles.data.clear();
    joint_target.setModes.data.clear();
    joint_target.values.data.clear();
    
    left_joint_target.handles.data.clear();
    left_joint_target.setModes.data.clear();
    left_joint_target.values.data.clear();
    
    right_joint_target.handles.data.clear();
    right_joint_target.setModes.data.clear();
    right_joint_target.values.data.clear();

    double alpha = state.joint_angle[1];
    double beta = (-state.joint_angle[2]);
    
    /* Joint-by-joint mode - left leg position commands */
    left_joint_target.handles.data.push_back(left_joint_handle[1]);
    left_joint_target.setModes.data.push_back(1);
    left_joint_target.values.data.push_back(-state.joint_angle[0]);
    
    left_joint_target.handles.data.push_back(left_joint_handle[5]);
    left_joint_target.setModes.data.push_back(1);
    left_joint_target.values.data.push_back(state.joint_angle[0]);
    
    left_joint_target.handles.data.push_back(left_joint_handle[0]);
    left_joint_target.setModes.data.push_back(1);
    left_joint_target.values.data.push_back(alpha);
    
    left_joint_target.handles.data.push_back(left_joint_handle[3]);
    left_joint_target.setModes.data.push_back(1);
    left_joint_target.values.data.push_back(beta);
    
    left_joint_target.handles.data.push_back(left_joint_handle[4]);
    left_joint_target.setModes.data.push_back(1);
    left_joint_target.values.data.push_back((beta-alpha));
    
    /* Joint-by-joint mode - right leg position commands */
    right_joint_target.handles.data.push_back(right_joint_handle[1]);
    right_joint_target.setModes.data.push_back(1);
    right_joint_target.values.data.push_back(state.joint_angle[0]);
    
    right_joint_target.handles.data.push_back(right_joint_handle[5]);
    right_joint_target.setModes.data.push_back(1);
    right_joint_target.values.data.push_back(-state.joint_angle[0]);
    
    right_joint_target.handles.data.push_back(right_joint_handle[0]);
    right_joint_target.setModes.data.push_back(1);
    right_joint_target.values.data.push_back(alpha);
    
    right_joint_target.handles.data.push_back(right_joint_handle[3]);
    right_joint_target.setModes.data.push_back(1);
    right_joint_target.values.data.push_back(beta);
    
    right_joint_target.handles.data.push_back(right_joint_handle[4]);
    right_joint_target.setModes.data.push_back(1);
    right_joint_target.values.data.push_back((beta-alpha));
    
    
    /* ============================================================================= */
    /* ================= Pelvis position / robot joints publishing ================= */
    /* ============================================================================= */
    
    if (state.button_state[0] && command_method == 0)
    {
        command_pelvis_position_pub.publish(pelvis_target);
        command_pelvis_position_stamped_pub.publish(pelvis_target_stamped);
             
        command_left_joints_position_pub.publish(left_joint_target);
        command_right_joints_position_pub.publish(right_joint_target);
    } 
    else if (state.button_state[0] && command_method == 1)
        command_left_joints_position_pub.publish(left_joint_target);
    else if (state.button_state[0] && command_method == 2)
        command_right_joints_position_pub.publish(right_joint_target);

}


void simulationCallback(const vrep_common::VrepInfo& msg)
{
    if (simulator_state == 5 && msg.simulatorState.data == 4) /* Simulation stopped. */
      ros::shutdown();
	  
    simulator_state = msg.simulatorState.data;	 
    simulationTime = msg.simulationTime.data;
 
    setRobotPosition(simulationTime);
}
   

void feedbackCallback(const geometry_msgs::Vector3Stamped& msg)
{
    if (state.button_state[0])
    {
        base_force[0] = -msg.vector.x; /* Force.x */ 
        base_force[1] = msg.vector.z;  /* Force.z */
        base_force[2] = msg.vector.y;  /* Force.y */
    }
    else
    {
        base_force[0] = 0; /* Force.x */ 
        base_force[1] = 0; /* Force.z */
        base_force[2] = 0; /* Force.y */
    }
}


void objectHandles()
{
    joint_handles.request.objectType = sim_object_joint_type;
    joint_handles.request.dataType = 0;
    if(joint_handles_srv.call(joint_handles))
    {
        left_joint_handle.clear();
        right_joint_handle.clear();
    
        for (uint i=0; i<7; i++)
        {
	  right_joint_handle.push_back(joint_handles.response.handles[i]);
	  left_joint_handle.push_back(joint_handles.response.handles[i+7]);
        }
    }
    else
        ROS_ERROR("Failed to call service joint_handles");  
}


void setupMessaging(ros::NodeHandle nh)
{
    joint_handles_srv = nh.serviceClient<vrep_common::simRosGetObjectGroupData>("/vrep/simRosGetObjectGroupData"); 
    
    simulation_state_sub = nh.subscribe("/vrep/info", 1000, simulationCallback);
    
    command_pelvis_position_pub = nh.advertise<geometry_msgs::Point>("/vrep/command_pelvis_position",1000);
    command_pelvis_position_stamped_pub = nh.advertise<geometry_msgs::PointStamped>("command_pelvis_position_stamped",1000);
    command_joints_position_pub = nh.advertise<vrep_common::JointSetStateData>("/vrep/command_joints_position",1000);
    command_left_joints_position_pub = nh.advertise<vrep_common::JointSetStateData>("/vrep/command_left_joints_position",1000);
    command_right_joints_position_pub = nh.advertise<vrep_common::JointSetStateData>("/vrep/command_right_joints_position",1000);
    
    feedback_force_filtered_sub = nh.subscribe("/feedback_force_filtered", 1000, feedbackCallback); 
}

    
void mainLoop(int argc, char **argv)
{
    /* ROS main loop: Commanding orders for the model teleoperation are published, 
     * and information about the force values to be rendered by the joystick is subscribed. */ 
     
    ros::init(argc, argv, "phantom_dual");
    ros::NodeHandle nh;
   
    setupMessaging(nh);
    
    objectHandles();
    
    ros::spin();
}


HDCallbackCode HDCALLBACK UpdateCalibrationCallback(void *pUserData)
{
    HDenum *calibrationStyle = (HDenum *) pUserData;
   
    if (hdCheckCalibration() == HD_CALIBRATION_NEEDS_UPDATE)
    {
        hdUpdateCalibration(*calibrationStyle);
    }

    return HD_CALLBACK_DONE;
}


HDboolean CheckCalibration()
{   
    HDint supportedCalibrationStyles;
    HDenum calibrationStyle;
    
    /* Choose a calibration style. Some devices may support multiple types of calibration. 
     * In that case, prefer auto calibration over inkwell calibration, and prefer inkwell calibration over reset encoders. */
    
    hdGetIntegerv(HD_CALIBRATION_STYLE, &supportedCalibrationStyles);
    
    if (supportedCalibrationStyles & HD_CALIBRATION_ENCODER_RESET)
    {
        calibrationStyle = HD_CALIBRATION_ENCODER_RESET;
        /* Calibration style encoder reset: the device needs to be put in a reset position to be calibrated. */
    }
    
    if (supportedCalibrationStyles & HD_CALIBRATION_INKWELL)
    {
        calibrationStyle = HD_CALIBRATION_INKWELL;
        /* Calibration style inkwell: the device needs to be put into a fixture, inkwell, before calibration can be performed. */
    }
    
    if (supportedCalibrationStyles & HD_CALIBRATION_AUTO)
    {
        calibrationStyle = HD_CALIBRATION_AUTO;
        /* Calibration style auto: the device will gather new calibration information as the armature is moved. */
    }
    
    /* Some haptic devices are calibrated when the gimbal is placed into the device inkwell and updateCalibration is called.  
     * This form of calibration is always performed after the servoloop has started running. For devices that require inkwell reset,
     * such as the PHANToM Omni, calibration is successfully performed whenever the device is put into the inkwell. */
    
    HDenum status = hdCheckCalibration();
 
    if (status == HD_CALIBRATION_OK)
    {
        return HD_TRUE;
    }
    else if (status == HD_CALIBRATION_NEEDS_MANUAL_INPUT)
    {
        cout << "Calibration requires manual input." << endl;
        return HD_FALSE;
    }
    else if (status == HD_CALIBRATION_NEEDS_UPDATE)
    {
        /* Synchronous scheduler callback to check the joystick internal calibration parameters. */
        hdScheduleSynchronous(UpdateCalibrationCallback, &calibrationStyle, HD_DEFAULT_SCHEDULER_PRIORITY);
        
        if (HD_DEVICE_ERROR(hdGetError()))
        {
            cout << "\nFailed to update calibration.\n" << endl;
            return HD_FALSE;
        }
        else
        {
            cout << "\nCalibration updated successfully.\n" << endl;
            return HD_TRUE;
        }
    }
    else
    {
        assert(!"Unknown calibration status");
        return HD_FALSE;
    }
}

 
int main(int argc, char **argv)
{
    HDErrorInfo error;
    
    /* General pattern of use for the HDAPI: */
    
    /* Initialize the device. */
    HHD hHD = hdInitDevice(HD_DEFAULT_DEVICE);
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, "Failed to initialize the device");
        fprintf(stderr, "\nPress any key to quit.\n");
        getch();
        
        return -1;           
    }
    ROS_INFO("Found %s.\n\n", hdGetString(HD_DEVICE_MODEL_TYPE));
    
    /* Create asynchronous scheduler callback. */
    gCallbackHandle = hdScheduleAsynchronous(SchedulerCallback, 0, HD_MAX_SCHEDULER_PRIORITY);
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, "Failed to schedule servoloop callback");

        hdDisableDevice(hHD);
	
        fprintf(stderr, "\nPress any key to quit.\n");
        getch();
        
        return -1;    
    }
        
    /* Enable force output. */    
    hdEnable(HD_FORCE_OUTPUT);
    
    /* Start the scheduler - responsible for managing a high frequency thread 
     *for sending forces and retrieving state information from the device. */
    hdSetSchedulerRate(1000);
    hdStartScheduler();
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, "Failed to start the scheduler");
	
        fprintf(stderr, "\nPress any key to quit.\n");
        getch();
        return -1;           
    }
    
    cout << "Dual leg configuration selected." << endl;
    
    /* A parallel loop is started where the ROS node will be running, if the device is properly calibrated. */
    if (CheckCalibration())
        mainLoop(argc, argv);
   
    /* When the application is terminated (ROS node is shutted down), the device and scheduler are cleanup. */
    hdStopScheduler();
    hdUnschedule(gCallbackHandle);
    hdDisableDevice(hHD);
    
    return 0;   
}
