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
 * @brief types.h file for this module. Contains types and structures used by the application.
 * @author pedro_cruz
 * @version 1.0
 * @date 16 May 2012
 *@{
 */
#ifndef _TYPES_H_
#define _TYPES_H_

#include <hitec5980sg/hitec5980sg.h>

#include <iostream>
#include <pthread.h>
#include <gtk/gtk.h>
#include <glib.h>

#include <boost/lexical_cast.hpp>
#include <boost/format.hpp>

#include <HD/hd.h>
#include <HDU/hduVector.h>
#include <HDU/hduError.h>
#include <HDU/hduMatrix.h>
#include <HDU/hduPlane.h>
#include <HL/hl.h>

#include <phua_haptic/humanoid_functions.h>

#include <ros/ros.h>
#include "tf/transform_datatypes.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

/** 
* @brief Structure to hold current robot joint/cartesian space information.
*/
typedef struct{
  //robot joints
  double HeadTilt;
  double RightShoulderFlexion;
  double RightShoulderAbduction;
  double RightElbowFlexion;
  double LeftShoulderFlexion;
  double LeftShoulderAbduction;
  double LeftElbowFlexion;
  double TorsoRotation;
  double TorsoFlexion;
  double TorsoLateralFlexion;
  double RightAnkleInversion;
  double RightAnkleFlexion;
  double RightKneeFlexion;
  double RightHipAbduction;
  double RightHipFlexion;
  double LeftAnkleInversion;
  double LeftAnkleFlexion;
  double LeftKneeFlexion;
  double LeftHipAbduction;
  double LeftHipFlexion;
  // -> arm coordinates & RPY | shoulder referencial
  double X_arm_end_left;
  double Y_arm_end_left;
  double Z_arm_end_left;
  double ROLL_arm_end_left;
  double PITCH_arm_end_left;
  double YAW_arm_end_left;
  double X_arm_end_right;
  double Y_arm_end_right;
  double Z_arm_end_right;
  double ROLL_arm_end_right;
  double PITCH_arm_end_right;
  double YAW_arm_end_right;
  // -> detached legs coordinates & RPY | foot referencial
  double detached_leg_pos_left[3];
  double detached_leg_rpy_left[3];
  double detached_leg_pos_right[3];
  double detached_leg_rpy_right[3];
  //Centers of gravity
  Eigen::Vector3d COG_detached_leg_left;
  Eigen::Vector3d COG_detached_leg_right;
}RobotKinData;

/** 
* @brief Collection of interface widgets that are updated/changed on runtime or needed for some other matter.
*/
typedef struct{
  // robot joint space labels
  GtkWidget *joints_left_side;
  GtkWidget *joints_right_side;
  GtkWidget *x_label_left;
  GtkWidget *y_label_left;
  GtkWidget *z_label_left;
  GtkWidget *roll_label_left;
  GtkWidget *pitch_label_left;
  GtkWidget *yaw_label_left;
  GtkWidget *x_label_right;
  GtkWidget *y_label_right;
  GtkWidget *z_label_right;
  GtkWidget *roll_label_right;
  GtkWidget *pitch_label_right;
  GtkWidget *yaw_label_right;
  GtkWidget *head_tilt_label;
  GtkWidget *right_arm_label;
  GtkWidget *left_arm_label;
  GtkWidget *torso_label;
  GtkWidget *right_leg_label;
  GtkWidget *left_leg_label;
  // haptic device labels
  GtkWidget *phantom_x_label;
  GtkWidget *phantom_y_label;
  GtkWidget *phantom_z_label;
  GtkWidget *phantom_pen_tip_x_label;
  GtkWidget *phantom_pen_tip_y_label;
  GtkWidget *phantom_pen_tip_z_label;
  GtkWidget *phantom_speed_label;
  GtkWidget *phantom_jnt1_label;
  GtkWidget *phantom_jnt2_label;
  GtkWidget *phantom_jnt3_label;
  GtkWidget *phantom_gbl1_label;
  GtkWidget *phantom_gbl2_label;
  GtkWidget *phantom_gbl3_label;
  // speed control progress bars
  GtkWidget *auto_speed_arm_theta1_bar;
  GtkWidget *auto_speed_arm_theta2_bar;
  GtkWidget *auto_speed_arm_theta3_bar;
  //haptic loop button
  GtkWidget *start_loop_button;
  //kinematic model combobox
  GtkWidget *kin_model_combobox;
  // haptic workspace interface label
  GtkWidget *workspace_zone_label;
  // haptic drawing demo
  GtkWidget *drawing_demo_label;
  GtkWidget *drawing_demo_plane_label;
  // leg balancing demo
  GtkWidget *leg_balancing_demo_label;
  //user path demo
  GtkWidget *user_path_label;
  GtkWidget *user_path_run_button;
  // force feedback progress bars
  GtkWidget *pos_x_force_bar;
  GtkWidget *neg_x_force_bar;
  GtkWidget *pos_y_force_bar;
  GtkWidget *neg_y_force_bar;
  GtkWidget *pos_z_force_bar;
  GtkWidget *neg_z_force_bar;
  // force magnitude
  GtkWidget *force_magnitude_label;
  // motor temperature labels
  GtkWidget *motor_1_temp;
  GtkWidget *motor_2_temp;
  GtkWidget *motor_3_temp;
  // status bar
  GtkWidget *status_bar;
  //demos
  GtkWidget *demo1_checkbox;
  GtkWidget *demo2_checkbox;
  GtkWidget *demo3_checkbox;
  GtkWidget *demo4_checkbox;
  //stop/go
  GtkWidget *stop_button;
  GtkWidget *go_button;
}WidgetCollection;

/** 
* @brief Holds data/device information retrieved from HDAPI(OpenHaptics3).
*/
typedef struct 
{
  HHLRC hHLRC; //context for haptic feedbac/control
  HHD hHD; //current device (PHANToM)
  HDboolean m_button1State;
  HDboolean m_button2State;
  bool m_button1Clicked;
  bool m_button2Clicked;
  hduVector3Dd m_devicePosition; //mm
  hduVector3Dd jointAngles; //rad
  hduVector3Dd gimbalAngles; //rad
  hduVector3Dd average_speed; //mm/s
  HDErrorInfo m_error;
  bool phantom_on;
  HDdouble max_stiffness;
  HDdouble max_damping;
  HDdouble max_force; //Newton
  HDdouble max_continuous_force; //Newton
  HDdouble motor_temperature[3];
  bool need_update;
  hduMatrix end_effector_transform;
  hduVector3Dd m_PenTipPosition;
  tf::Transform world_to_phantom;
  tf::Transform phantom_to_base_point;
  tf::Transform base_point_to_pen_tip;
} DeviceData;

/** 
* @brief A set of application/funcionality related parameters.
*/
typedef struct 
{
  bool exit_status;
  double coordinate_resolution;
  double automatic_speed_control_freq;
  int kinematic_model;
  bool manual_speed_control;
  short unsigned int manual_speed;
  bool automatic_speed_control;
  double arm_auto_angular_speed[3];
  short unsigned int arm_auto_speed[3];
  double pos_coord_scale;
  unsigned int selected_notebook_page;
  double cntrl_pos_back_front;
  short unsigned int robot_home_position_base_speed;
  unsigned int control_rate;
  double graphics_refresh_rate;
  unsigned int haptics_rate;
} ParameterSet;

/** 
* @brief Holds haptics related data and variables.
*/
typedef struct 
{
  hduVector3Dd arm_main_spheresPosition;
  hduVector3Dd arm_back_spherePosition;
  double arm_main_sphere_max_Radius;
  double arm_main_sphere_min_Radius;
  double arm_back_sphereRadius;
  hduVector3Dd leg_main_spheresPosition;
  hduVector3Dd leg_back_spherePosition;
  hduVector3Dd leg_front_spherePosition;
  double leg_main_sphere_max_Radius;
  double leg_main_sphere_min_Radius;
  double leg_back_sphereRadius;
  hduVector3Dd applied_force;
  short unsigned int force_threshold;
  short unsigned int chosen_demo;
  hduVector3Dd demo_2_point_1;
  hduVector3Dd demo_2_point_2;
  hduVector3Dd demo_2_point_3;
  hduPlaned demo_2_Plane;
  double demo2_distance_to_plane;
  bool demo_user_path_point_storing;
  bool demo_user_path_is_run_once;
  bool demo_user_path_run_start;
  std::vector<double> user_path_X_pnts;
  std::vector<double> user_path_Y_pnts;
  std::vector<double> user_path_Z_pnts;
} HapticsData;

/** 
* @brief Shared struture that holds robot/device information.
*/
typedef struct
{
  hitec_5980SG *servo;
  RobotKinData robot_kin_data;
  pthread_mutex_t mutex_gtk;
  WidgetCollection updt_labels;
  DeviceData phantom_data;
  ParameterSet parameters;
  bool haptic_loop_start;
  bool update_labels;
  HapticsData haptics_data;
  humanoid *humanoid_f;
}shared_vars_t;

#endif
/**
 *@}
*/
