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
 * @file phantom_control.cpp
 * @author Emílio Estrelinha nº 38637 (emilio.estrelinha@ua.pt)
 * @brief Reteives data and commands feedback to the Phantom Omni
 */

#include <phantom_control/omni.h>
#include <phantom_control/conio.h>

#include <sys/types.h>
#include <unistd.h>
#include <stdlib.h>

#include <haptic_force/Force.h>

#include <phantom_control/State.h>

HDSchedulerHandle gCallbackHandle = 0;
OmniState state;
bool g_new=0;
phantom_control::State g_SS;

hduVector3Dd g_home_pos(HOME_XX, HOME_YY, HOME_ZZ);

void ee_convert(void);
void ForceCallBk ( const haptic_force::Force &force );

void HHD_Auto_Calibration(void);
HDCallbackCode HDCALLBACK MonitorCallback(void *pUserData);

void SetHomePos(void);


/**
* @fn int main ( int argc, char **argv )
* Main 
*/
int main ( int argc, char **argv )
{
    // Phantom device state pointer
    OmniState *ss=&state;
    
    
    /*******************************************************************
     * Initialize Phantom device
    *******************************************************************/
    HDErrorInfo error;
    HHD hHD = hdInitDevice(HD_DEFAULT_DEVICE);
    
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, "Failed to initialize haptic device");
        fprintf(stderr, "\nPress any key to quit.\n");
        getch();
        return -1;
    }
    ROS_INFO("Found %s.\n\n", hdGetString(HD_DEVICE_MODEL_TYPE));
    
    /* Schedule the haptic callback function for continuously monitoring the
       button state and rendering the anchored spring force. */
    gCallbackHandle = hdScheduleAsynchronous(MonitorCallback, ss, HD_MAX_SCHEDULER_PRIORITY);
    
    hdEnable(HD_FORCE_OUTPUT);

    hdStartScheduler();
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, "Failed to start scheduler");
        fprintf(stderr, "\nPress any key to quit.\n");
        getch();
        return -1;
    }
    
    HHD_Auto_Calibration();

    
    /*******************************************************************
     * Initialize state
    *******************************************************************/
    ss->buttons[0] = 0;
    ss->buttons[1] = 0;
    ss->home_pos = g_home_pos;
    ss->home=0;
    
    /*******************************************************************
     * Initialize ROS
    *******************************************************************/
    ros::init( argc, argv, "phantom_node" );
    
    ros::NodeHandle n("~");
    
    ros::Publisher pub_state= n.advertise< phantom_control::State >( "/phantom_state", 1000 );
    
    ros::Subscriber sub_force = n.subscribe ( "/force", 1000, ForceCallBk );

    ros::Rate loop_rate( 2.5*1300 );

    ee_convert();

    int pid = getpid(), rpid;

    boost::format fmt("sudo renice -10 %d");
    fmt % pid;
    
    rpid = std::system(fmt.str().c_str());
    
    while ( ros::ok () )
    {
        if(g_new)
        {
            ee_convert();
            
            pub_state.publish( g_SS );
            
            g_new=0;
        }

        // IF button 1
        
        // IF button 2 => Sets a new home position
        if(g_SS.buttons[1])
        {
            g_home_pos[0]=g_SS.position[0];
            g_home_pos[1]=g_SS.position[1];
            g_home_pos[2]=g_SS.position[2];
            ss->home_pos[0]=g_SS.position[0];
            ss->home_pos[1]=g_SS.position[1];
            ss->home_pos[2]=g_SS.position[2];
        }
        
        ros::spinOnce (  );
        
        loop_rate.sleep (  );
    }
    /* Cleanup by stopping the haptics loop, unscheduling the asynchronous
    callback, disabling the device. */
    ROS_INFO("Ending Session....\n");
    hdStopScheduler();
    hdUnschedule(gCallbackHandle);
    hdDisableDevice(hHD);
    
    return 0;
}

/**
* @fn void ee_convert(void)
* Function to get/convert the HHD lib vars to ROS vars
* @param void
* @return void
*/
void ee_convert(void)
{
    // Phantom device state pointer
    OmniState *ss=&state;

    g_SS.pos_hist1[0]=g_SS.position[0];
    g_SS.pos_hist1[1]=g_SS.position[1];
    g_SS.pos_hist1[2]=g_SS.position[2];
    
    g_SS.position[0]=ss->position[0];
    g_SS.position[1]=ss->position[1];
    g_SS.position[2]=ss->position[2];

    g_SS.rot[0]=ss->rot[0];
    g_SS.rot[1]=ss->rot[1];
    g_SS.rot[2]=ss->rot[2];

    g_SS.joints[0]=ss->joints[0];
    g_SS.joints[1]=ss->joints[1];
    g_SS.joints[2]=ss->joints[2];

    g_SS.temp[0]=ss->temp[0];
    g_SS.temp[1]=ss->temp[1];
    g_SS.temp[2]=ss->temp[2];

    g_SS.buttons[0]=ss->buttons[0];
    g_SS.buttons[1]=ss->buttons[1];

    g_SS.home=ss->home;

    g_SS.home_pos[0]=ss->home_pos[0];
    g_SS.home_pos[1]=ss->home_pos[1];
    g_SS.home_pos[2]=ss->home_pos[2];

    g_SS.force[0]=ss->force[0];
    g_SS.force[1]=ss->force[1];
    g_SS.force[2]=ss->force[2];
    
    g_SS.header.stamp = ros::Time::now (  );
}

/**
* @fn void ForceCallBk ( const phantom_control::Force &force )
* Callback for updating force feedback
* @param force : Message that this module subscribes
* @return void
*/
void ForceCallBk ( const haptic_force::Force &force )
{
    state.force[0]=force.force[0];
    state.force[1]=force.force[1];
    state.force[2]=force.force[2];
}

/**
* @fn void HHD_Auto_Calibration(void)
* Automatic Calibration of Phantom Device - No character inputs
* @param void
* @return void
*/
void HHD_Auto_Calibration(void)
{
    int calibrationStyle;
    int supportedCalibrationStyles;
    HDErrorInfo error;
    
    hdGetIntegerv(HD_CALIBRATION_STYLE, &supportedCalibrationStyles);
    if (supportedCalibrationStyles & HD_CALIBRATION_ENCODER_RESET)
    {
        calibrationStyle = HD_CALIBRATION_ENCODER_RESET;
        ROS_INFO("HD_CALIBRATION_ENCODER_RESE..\n\n");
    }
    if (supportedCalibrationStyles & HD_CALIBRATION_INKWELL)
    {
        calibrationStyle = HD_CALIBRATION_INKWELL;
        ROS_INFO("HD_CALIBRATION_INKWELL..\n\n");
    }
    if (supportedCalibrationStyles & HD_CALIBRATION_AUTO)
    {
        calibrationStyle = HD_CALIBRATION_AUTO;
        ROS_INFO("HD_CALIBRATION_AUTO..\n\n");
    }
    
    do 
    {
        hdUpdateCalibration(calibrationStyle);
        ROS_INFO("Calibrating.. (put stylus in well)\n");
        if (HD_DEVICE_ERROR(error = hdGetError()))
        {
            hduPrintError(stderr, &error, "Reset encoders reset failed.");
            break;           
        }
    }   while (hdCheckCalibration() != HD_CALIBRATION_OK);
    
    ROS_INFO("...\n...\nCalibration complete.\n");
}

/**
* @fn HDCallbackCode HDCALLBACK MonitorCallback(void *pUserData)
* Gets Phantom current state of device and sets the force feedback
* @param pUserData : var to access program data
* @return HDCallbackCode : see HHD ref lib
*/
HDCallbackCode HDCALLBACK MonitorCallback(void *pUserData)
{
    OmniState *omni_state = static_cast<OmniState *>(pUserData);
    
    HDErrorInfo error;
    
    int nButtons = 0;

    hduVector3Dd position;
    hduVector3Dd force;
    
    omni_state->force=state.force;
    
    hdBeginFrame(hdGetCurrentDevice());
    
    hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, omni_state->rot);      
    hdGetDoublev(HD_CURRENT_POSITION,      omni_state->position); /*milimeters*/
    hdGetDoublev(HD_CURRENT_JOINT_ANGLES,  omni_state->joints);
    
    //Get buttons
    hdGetIntegerv(HD_CURRENT_BUTTONS, &nButtons);
    omni_state->buttons[0] = (nButtons & HD_DEVICE_BUTTON_1) ? 1 : 0;
    omni_state->buttons[1] = (nButtons & HD_DEVICE_BUTTON_2) ? 1 : 0;

    // Get motor temperatures [0...1]
    hdGetDoublev(HD_MOTOR_TEMPERATURE, omni_state->temp);

    if(omni_state->buttons[0]!=1)
    {
        SetHomePos();
    }
    else
    {
        hdSetDoublev(HD_CURRENT_FORCE,  omni_state->force);
        omni_state->home=0;
    }
    
    hdEndFrame(hdGetCurrentDevice());

    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, "Error during main scheduler callback\n");
        if (hduIsSchedulerError(&error))
        return HD_CALLBACK_DONE;
    }
    
    // Get thetas
    float t[7] = {0.0, omni_state->joints[0], omni_state->joints[1], omni_state->joints[2]-omni_state->joints[1], omni_state->rot[0], omni_state->rot[1], omni_state->rot[2]};
    
    for (int i = 0; i < 7; i++) omni_state->thetas[i] = t[i];
    
    g_new=1;

    return HD_CALLBACK_CONTINUE;
}


/**
* @fn void SetHomePos(void)
* Function to send tip of device to HOME_POSITION
* @param void
* @return void
*/
void SetHomePos(void)
{
    hduVector3Dd home_pos = g_home_pos;
    hduVector3Dd position;
    
    position[0]=g_SS.position[0];
    position[1]=g_SS.position[1];
    position[2]=g_SS.position[2];
    
    if( g_SS.position == g_SS.home_pos)
    {
        g_SS.home=1;
        hduVector3Dd force2home(0, 0.49, 0);
        
        hdSetDoublev(HD_CURRENT_FORCE, force2home);
    }
    if(!g_SS.home)
    {
        hduVector3Dd force2home = (home_pos - position);
        force2home.normalize();
        force2home*=0.975;
        
        hdSetDoublev(HD_CURRENT_FORCE, force2home);
    }
    
}
