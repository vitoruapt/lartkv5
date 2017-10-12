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
 * \brief GTK auxiliary functions implementation
 */

#include <phua_haptic/gtk_aux.h>

using namespace std;

void UpdateLabels(shared_vars_t*RobotVars)
{
  string text;
  if(RobotVars->parameters.selected_notebook_page==0)
  {
    if(RobotVars->parameters.kinematic_model>=1 && RobotVars->parameters.kinematic_model<=3)
    {
      // -> XYZRPY left
      text="X: "+ str(boost::format("%3.1f") % AvoidMinusZero(RobotVars->robot_kin_data.X_arm_end_left)) +" [mm]";
      gtk_label_set_text((GtkLabel *)RobotVars->updt_labels.x_label_left,text.c_str());
      
      text="Y: "+ str(boost::format("%3.1f") % AvoidMinusZero(RobotVars->robot_kin_data.Y_arm_end_left)) +" [mm]";
      gtk_label_set_text((GtkLabel *)RobotVars->updt_labels.y_label_left,text.c_str());
      
      text="Z: "+ str(boost::format("%3.1f") % AvoidMinusZero(RobotVars->robot_kin_data.Z_arm_end_left)) +" [mm]";
      gtk_label_set_text((GtkLabel *)RobotVars->updt_labels.z_label_left,text.c_str());
      
      text="R: "+ str(boost::format("%3.1f") % AvoidMinusZero(RobotVars->robot_kin_data.ROLL_arm_end_left)) +" [deg]";
      gtk_label_set_text((GtkLabel *)RobotVars->updt_labels.roll_label_left,text.c_str());
      
      text="P: "+ str(boost::format("%3.1f") % AvoidMinusZero(RobotVars->robot_kin_data.PITCH_arm_end_left)) +" [deg]";
      gtk_label_set_text((GtkLabel *)RobotVars->updt_labels.pitch_label_left,text.c_str());
      
      text="Y: "+ str(boost::format("%3.1f") % AvoidMinusZero(RobotVars->robot_kin_data.YAW_arm_end_left)) +" [deg]";
      gtk_label_set_text((GtkLabel *)RobotVars->updt_labels.yaw_label_left,text.c_str());
      
      // -> XYZRPY right
      text="X: "+ str(boost::format("%3.1f") % AvoidMinusZero(RobotVars->robot_kin_data.X_arm_end_right)) +" [mm]";
      gtk_label_set_text((GtkLabel *)RobotVars->updt_labels.x_label_right,text.c_str());
      
      text="Y: "+ str(boost::format("%3.1f") % AvoidMinusZero(RobotVars->robot_kin_data.Y_arm_end_right)) +" [mm]";
      gtk_label_set_text((GtkLabel *)RobotVars->updt_labels.y_label_right,text.c_str());
      
      text="Z: "+ str(boost::format("%3.1f") % AvoidMinusZero(RobotVars->robot_kin_data.Z_arm_end_right)) +" [mm]";
      gtk_label_set_text((GtkLabel *)RobotVars->updt_labels.z_label_right,text.c_str());
      
      text="R: "+ str(boost::format("%3.1f") % AvoidMinusZero(RobotVars->robot_kin_data.ROLL_arm_end_right)) +" [deg]";
      gtk_label_set_text((GtkLabel *)RobotVars->updt_labels.roll_label_right,text.c_str());
      
      text="P: "+ str(boost::format("%3.1f") % AvoidMinusZero(RobotVars->robot_kin_data.PITCH_arm_end_right)) +" [deg]";
      gtk_label_set_text((GtkLabel *)RobotVars->updt_labels.pitch_label_right,text.c_str());
      
      text="Y: "+ str(boost::format("%3.1f") % AvoidMinusZero(RobotVars->robot_kin_data.YAW_arm_end_right)) +" [deg]";
      gtk_label_set_text((GtkLabel *)RobotVars->updt_labels.yaw_label_right,text.c_str());
    }
    else if(RobotVars->parameters.kinematic_model==4)
    {
      // -> XYZRPY left
      text="X: "+ str(boost::format("%3.1f") % AvoidMinusZero(RobotVars->robot_kin_data.detached_leg_pos_left[0])) +" [mm]";
      gtk_label_set_text((GtkLabel *)RobotVars->updt_labels.x_label_left,text.c_str());
      
      text="Y: "+ str(boost::format("%3.1f") % AvoidMinusZero(RobotVars->robot_kin_data.detached_leg_pos_left[1])) +" [mm]";
      gtk_label_set_text((GtkLabel *)RobotVars->updt_labels.y_label_left,text.c_str());
      
      text="Z: "+ str(boost::format("%3.1f") % AvoidMinusZero(RobotVars->robot_kin_data.detached_leg_pos_left[2])) +" [mm]";
      gtk_label_set_text((GtkLabel *)RobotVars->updt_labels.z_label_left,text.c_str());
      
      text="R: "+ str(boost::format("%3.1f") % AvoidMinusZero(RobotVars->robot_kin_data.detached_leg_rpy_left[0])) +" [deg]";
      gtk_label_set_text((GtkLabel *)RobotVars->updt_labels.roll_label_left,text.c_str());
      
      text="P: "+ str(boost::format("%3.1f") % AvoidMinusZero(RobotVars->robot_kin_data.detached_leg_rpy_left[1])) +" [deg]";
      gtk_label_set_text((GtkLabel *)RobotVars->updt_labels.pitch_label_left,text.c_str());
      
      text="Y: "+ str(boost::format("%3.1f") % AvoidMinusZero(RobotVars->robot_kin_data.detached_leg_rpy_left[2])) +" [deg]";
      gtk_label_set_text((GtkLabel *)RobotVars->updt_labels.yaw_label_left,text.c_str());
      
      // -> XYZRPY right
      gtk_label_set_text((GtkLabel *)RobotVars->updt_labels.x_label_right,"X: ---- [mm]");
      
      gtk_label_set_text((GtkLabel *)RobotVars->updt_labels.y_label_right,"Y: ---- [mm]");
      
      gtk_label_set_text((GtkLabel *)RobotVars->updt_labels.z_label_right,"Z: ---- [mm]");
      
      gtk_label_set_text((GtkLabel *)RobotVars->updt_labels.roll_label_right,"R: ---- [deg]");
      
      gtk_label_set_text((GtkLabel *)RobotVars->updt_labels.pitch_label_right,"P: ---- [deg]");
      
      gtk_label_set_text((GtkLabel *)RobotVars->updt_labels.yaw_label_right,"Y: ---- [deg]");
    }
    else if(RobotVars->parameters.kinematic_model==5)
    {
      // -> XYZRPY left
      gtk_label_set_text((GtkLabel *)RobotVars->updt_labels.x_label_left,"X: ---- [mm]");
      
      gtk_label_set_text((GtkLabel *)RobotVars->updt_labels.y_label_left,"Y: ---- [mm]");
      
      gtk_label_set_text((GtkLabel *)RobotVars->updt_labels.z_label_left,"Z: ---- [mm]");
      
      gtk_label_set_text((GtkLabel *)RobotVars->updt_labels.roll_label_left,"R: ---- [deg]");
      
      gtk_label_set_text((GtkLabel *)RobotVars->updt_labels.pitch_label_left,"P: ---- [deg]");
      
      gtk_label_set_text((GtkLabel *)RobotVars->updt_labels.yaw_label_left,"Y: ---- [deg]");
      
      // -> XYZRPY right
      text="X: "+ str(boost::format("%3.1f") % AvoidMinusZero(RobotVars->robot_kin_data.detached_leg_pos_right[0])) +" [mm]";
      gtk_label_set_text((GtkLabel *)RobotVars->updt_labels.x_label_right,text.c_str());
      
      text="Y: "+ str(boost::format("%3.1f") % AvoidMinusZero(RobotVars->robot_kin_data.detached_leg_pos_right[1])) +" [mm]";
      gtk_label_set_text((GtkLabel *)RobotVars->updt_labels.y_label_right,text.c_str());
      
      text="Z: "+ str(boost::format("%3.1f") % AvoidMinusZero(RobotVars->robot_kin_data.detached_leg_pos_right[2])) +" [mm]";
      gtk_label_set_text((GtkLabel *)RobotVars->updt_labels.z_label_right,text.c_str());
      
      text="R: "+ str(boost::format("%3.1f") % AvoidMinusZero(RobotVars->robot_kin_data.detached_leg_rpy_right[0])) +" [deg]";
      gtk_label_set_text((GtkLabel *)RobotVars->updt_labels.roll_label_right,text.c_str());
      
      text="P: "+ str(boost::format("%3.1f") % AvoidMinusZero(RobotVars->robot_kin_data.detached_leg_rpy_right[1])) +" [deg]";
      gtk_label_set_text((GtkLabel *)RobotVars->updt_labels.pitch_label_right,text.c_str());
      
      text="Y: "+ str(boost::format("%3.1f") % AvoidMinusZero(RobotVars->robot_kin_data.detached_leg_rpy_right[2])) +" [deg]";
      gtk_label_set_text((GtkLabel *)RobotVars->updt_labels.yaw_label_right,text.c_str());
    }
    else
    {
      // -> XYZRPY left
      gtk_label_set_text((GtkLabel *)RobotVars->updt_labels.x_label_left,"X: ---- [mm]");
      
      gtk_label_set_text((GtkLabel *)RobotVars->updt_labels.y_label_left,"Y: ---- [mm]");
      
      gtk_label_set_text((GtkLabel *)RobotVars->updt_labels.z_label_left,"Z: ---- [mm]");
      
      gtk_label_set_text((GtkLabel *)RobotVars->updt_labels.roll_label_left,"R: ---- [deg]");
      
      gtk_label_set_text((GtkLabel *)RobotVars->updt_labels.pitch_label_left,"P: ---- [deg]");
      
      gtk_label_set_text((GtkLabel *)RobotVars->updt_labels.yaw_label_left,"Y: ---- [deg]");
      
      // -> XYZRPY right
      gtk_label_set_text((GtkLabel *)RobotVars->updt_labels.x_label_right,"X: ---- [mm]");
      
      gtk_label_set_text((GtkLabel *)RobotVars->updt_labels.y_label_right,"Y: ---- [mm]");
      
      gtk_label_set_text((GtkLabel *)RobotVars->updt_labels.z_label_right,"Z: ---- [mm]");
      
      gtk_label_set_text((GtkLabel *)RobotVars->updt_labels.roll_label_right,"R: ---- [deg]");
      
      gtk_label_set_text((GtkLabel *)RobotVars->updt_labels.pitch_label_right,"P: ---- [deg]");
      
      gtk_label_set_text((GtkLabel *)RobotVars->updt_labels.yaw_label_right,"Y: ---- [deg]");
    }
    
    // -> head
    text="Head Tilt: "+ str(boost::format("%3.1f") % AvoidMinusZero(RobotVars->robot_kin_data.HeadTilt)) +" [deg]";
    gtk_label_set_text((GtkLabel *)RobotVars->updt_labels.head_tilt_label,text.c_str());
    
    // -> arms
    text="Right Shoulder Flexion:	    " + str(boost::format("%3.1f") % AvoidMinusZero(RobotVars->robot_kin_data.RightShoulderFlexion)) + " [deg]\nRight Shoulder Abduction:  "+ str(boost::format("%3.1f") % AvoidMinusZero(RobotVars->robot_kin_data.RightShoulderAbduction)) +" [deg]\nRight Elbow Flexion:	    "+ str(boost::format("%3.1f") % AvoidMinusZero(RobotVars->robot_kin_data.RightElbowFlexion)) +" [deg]";
    gtk_label_set_text((GtkLabel *)RobotVars->updt_labels.right_arm_label,text.c_str());
    text="Left Shoulder Flexion:	  " + str(boost::format("%3.1f") % AvoidMinusZero(RobotVars->robot_kin_data.LeftShoulderFlexion)) + " [deg]\nLeft Shoulder Abduction:  "+ str(boost::format("%3.1f") % AvoidMinusZero(RobotVars->robot_kin_data.LeftShoulderAbduction)) +" [deg]\nLeft Elbow Flexion:		   "+ str(boost::format("%3.1f") % AvoidMinusZero(RobotVars->robot_kin_data.LeftElbowFlexion)) +" [deg]";
    gtk_label_set_text((GtkLabel *)RobotVars->updt_labels.left_arm_label,text.c_str());
    
    // -> torso
    text="Torso Rotation:	     " + str(boost::format("%3.1f") % AvoidMinusZero(RobotVars->robot_kin_data.TorsoRotation)) + " [deg]\nTorso Flexion:		     "+ str(boost::format("%3.1f") % AvoidMinusZero(RobotVars->robot_kin_data.TorsoFlexion)) +" [deg]\nTorso Lateral Flexion:  " + str(boost::format("%3.1f") % AvoidMinusZero(RobotVars->robot_kin_data.TorsoLateralFlexion)) +" [deg]";
    gtk_label_set_text((GtkLabel *)RobotVars->updt_labels.torso_label,text.c_str());
    
    //-> legs
    text="Right Hip Flexion:	      " + str(boost::format("%3.1f") % AvoidMinusZero(RobotVars->robot_kin_data.RightHipFlexion)) + " [deg]\nRight Hip Abduction:     " + str(boost::format("%3.1f") % AvoidMinusZero(RobotVars->robot_kin_data.RightHipAbduction)) +" [deg]\nRight Knee Flexion:      " + str(boost::format("%3.1f") % AvoidMinusZero(RobotVars->robot_kin_data.RightKneeFlexion)) + " [deg]\nRight Ankle Flexion:     " + str(boost::format("%3.1f") % AvoidMinusZero(RobotVars->robot_kin_data.RightAnkleFlexion)) + " [deg]\nRight Ankle Inversion:  " + str(boost::format("%3.1f") % AvoidMinusZero(RobotVars->robot_kin_data.RightAnkleInversion)) + " [deg]";
    gtk_label_set_text((GtkLabel *)RobotVars->updt_labels.right_leg_label,text.c_str());
    text="Left Hip Flexion:	    " + str(boost::format("%3.1f") % AvoidMinusZero(RobotVars->robot_kin_data.LeftHipFlexion)) + " [deg]\nLeft Hip Abduction:     "+ str(boost::format("%3.1f") % AvoidMinusZero(RobotVars->robot_kin_data.LeftHipAbduction)) +" [deg]\nLeft Knee Flexion:      " + str(boost::format("%3.1f") % AvoidMinusZero(RobotVars->robot_kin_data.LeftKneeFlexion)) + " [deg]\nLeft Ankle Flexion:     " + str(boost::format("%3.1f") % AvoidMinusZero(RobotVars->robot_kin_data.LeftAnkleFlexion)) + " [deg]\nLeft Ankle Inversion:  " + str(boost::format("%3.1f") % AvoidMinusZero(RobotVars->robot_kin_data.LeftAnkleInversion)) + " [deg]";
    gtk_label_set_text((GtkLabel *)RobotVars->updt_labels.left_leg_label,text.c_str());
  }
  //joystick
  if(RobotVars->phantom_data.phantom_on)
  {
    if(RobotVars->parameters.selected_notebook_page==1)
    {
      //position
      if(RobotVars->parameters.coordinate_resolution>0.01)
      {
	text="X: "+ str(boost::format("%3.1f") % AvoidMinusZero(RobotVars->phantom_data.m_devicePosition[0])) +" [mm]";
	gtk_label_set_text((GtkLabel *)RobotVars->updt_labels.phantom_x_label,text.c_str());
	text="Y: "+ str(boost::format("%3.1f") % AvoidMinusZero(RobotVars->phantom_data.m_devicePosition[1])) +" [mm]";
	gtk_label_set_text((GtkLabel *)RobotVars->updt_labels.phantom_y_label,text.c_str());
	text="Z: "+ str(boost::format("%3.1f") % AvoidMinusZero(RobotVars->phantom_data.m_devicePosition[2])) +" [mm]";
	gtk_label_set_text((GtkLabel *)RobotVars->updt_labels.phantom_z_label,text.c_str());
      }
      else if(RobotVars->parameters.coordinate_resolution==0.01)
      {
	text="X: "+ str(boost::format("%3.2f") % AvoidMinusZero(RobotVars->phantom_data.m_devicePosition[0])) +" [mm]";
	gtk_label_set_text((GtkLabel *)RobotVars->updt_labels.phantom_x_label,text.c_str());
	text="Y: "+ str(boost::format("%3.2f") % AvoidMinusZero(RobotVars->phantom_data.m_devicePosition[1])) +" [mm]";
	gtk_label_set_text((GtkLabel *)RobotVars->updt_labels.phantom_y_label,text.c_str());
	text="Z: "+ str(boost::format("%3.2f") % AvoidMinusZero(RobotVars->phantom_data.m_devicePosition[2])) +" [mm]";
	gtk_label_set_text((GtkLabel *)RobotVars->updt_labels.phantom_z_label,text.c_str());
      }
      else
      {
	text="X: "+ str(boost::format("%3.3f") % AvoidMinusZero(RobotVars->phantom_data.m_devicePosition[0])) +" [mm]";
	gtk_label_set_text((GtkLabel *)RobotVars->updt_labels.phantom_x_label,text.c_str());
	text="Y: "+ str(boost::format("%3.3f") % AvoidMinusZero(RobotVars->phantom_data.m_devicePosition[1])) +" [mm]";
	gtk_label_set_text((GtkLabel *)RobotVars->updt_labels.phantom_y_label,text.c_str());
	text="Z: "+ str(boost::format("%3.3f") % AvoidMinusZero(RobotVars->phantom_data.m_devicePosition[2])) +" [mm]";
	gtk_label_set_text((GtkLabel *)RobotVars->updt_labels.phantom_z_label,text.c_str());
      }
      
      //pen tip position
      if(RobotVars->parameters.coordinate_resolution>0.01)
      {
	text="X: "+ str(boost::format("%3.1f") % AvoidMinusZero(RobotVars->phantom_data.m_PenTipPosition[0])) +" [mm]";
	gtk_label_set_text((GtkLabel *)RobotVars->updt_labels.phantom_pen_tip_x_label,text.c_str());
	text="Y: "+ str(boost::format("%3.1f") % AvoidMinusZero(RobotVars->phantom_data.m_PenTipPosition[1])) +" [mm]";
	gtk_label_set_text((GtkLabel *)RobotVars->updt_labels.phantom_pen_tip_y_label,text.c_str());
	text="Z: "+ str(boost::format("%3.1f") % AvoidMinusZero(RobotVars->phantom_data.m_PenTipPosition[2])) +" [mm]";
	gtk_label_set_text((GtkLabel *)RobotVars->updt_labels.phantom_pen_tip_z_label,text.c_str());
      }
      else if(RobotVars->parameters.coordinate_resolution==0.01)
      {
	text="X: "+ str(boost::format("%3.2f") % AvoidMinusZero(RobotVars->phantom_data.m_PenTipPosition[0])) +" [mm]";
	gtk_label_set_text((GtkLabel *)RobotVars->updt_labels.phantom_pen_tip_x_label,text.c_str());
	text="Y: "+ str(boost::format("%3.2f") % AvoidMinusZero(RobotVars->phantom_data.m_PenTipPosition[1])) +" [mm]";
	gtk_label_set_text((GtkLabel *)RobotVars->updt_labels.phantom_pen_tip_y_label,text.c_str());
	text="Z: "+ str(boost::format("%3.2f") % AvoidMinusZero(RobotVars->phantom_data.m_PenTipPosition[2])) +" [mm]";
	gtk_label_set_text((GtkLabel *)RobotVars->updt_labels.phantom_pen_tip_z_label,text.c_str());
      }
      else
      {
	text="X: "+ str(boost::format("%3.3f") % AvoidMinusZero(RobotVars->phantom_data.m_PenTipPosition[0])) +" [mm]";
	gtk_label_set_text((GtkLabel *)RobotVars->updt_labels.phantom_pen_tip_x_label,text.c_str());
	text="Y: "+ str(boost::format("%3.3f") % AvoidMinusZero(RobotVars->phantom_data.m_PenTipPosition[1])) +" [mm]";
	gtk_label_set_text((GtkLabel *)RobotVars->updt_labels.phantom_pen_tip_y_label,text.c_str());
	text="Z: "+ str(boost::format("%3.3f") % AvoidMinusZero(RobotVars->phantom_data.m_PenTipPosition[2])) +" [mm]";
	gtk_label_set_text((GtkLabel *)RobotVars->updt_labels.phantom_pen_tip_z_label,text.c_str());
      }
      
      //joint angles
      text = str(boost::format("%3.1f") % AvoidMinusZero(RobotVars->phantom_data.jointAngles[0]));
      gtk_label_set_markup (GTK_LABEL (RobotVars->updt_labels.phantom_jnt1_label),
			    g_markup_printf_escaped ("\u03B8<sub>1</sub>: %s<sup>o</sup>", text.c_str()));
      
      text = str(boost::format("%3.1f") % AvoidMinusZero(RobotVars->phantom_data.jointAngles[1]));
      gtk_label_set_markup (GTK_LABEL (RobotVars->updt_labels.phantom_jnt2_label),
			    g_markup_printf_escaped ("\u03B8<sub>2</sub>: %s<sup>o</sup>", text.c_str()));
      
      text = str(boost::format("%3.1f") % AvoidMinusZero(RobotVars->phantom_data.jointAngles[2]));
      gtk_label_set_markup (GTK_LABEL (RobotVars->updt_labels.phantom_jnt3_label),
			    g_markup_printf_escaped ("\u03B8<sub>3</sub>: %s<sup>o</sup>", text.c_str()));
    
      //gimbal angles
      text=str(boost::format("%3.1f") % AvoidMinusZero(RobotVars->phantom_data.gimbalAngles[0]));
      gtk_label_set_markup (GTK_LABEL (RobotVars->updt_labels.phantom_gbl1_label),
			    g_markup_printf_escaped ("\u03B8<sub>4</sub>: %s<sup>o</sup>", text.c_str()));
      
      text=str(boost::format("%3.1f") % AvoidMinusZero(RobotVars->phantom_data.gimbalAngles[1]));
      gtk_label_set_markup (GTK_LABEL (RobotVars->updt_labels.phantom_gbl2_label),
			    g_markup_printf_escaped ("\u03B8<sub>5</sub>: %s<sup>o</sup>", text.c_str()));
      
      text=str(boost::format("%3.1f") % AvoidMinusZero(RobotVars->phantom_data.gimbalAngles[2]));
      gtk_label_set_markup (GTK_LABEL (RobotVars->updt_labels.phantom_gbl3_label),
			    g_markup_printf_escaped ("\u03B8<sub>6</sub>: %s<sup>o</sup>", text.c_str()));
      
      //motor temperatures
      text = str(boost::format("%3.1f") % AvoidMinusZero(RobotVars->phantom_data.motor_temperature[0]));
      if(RobotVars->phantom_data.motor_temperature[0]<(1./3.))
      {
	//green
	gtk_label_set_markup (GTK_LABEL (RobotVars->updt_labels.motor_1_temp), g_markup_printf_escaped ("Motor 1: <span foreground=\"green\"><b>%s</b></span> ", text.c_str()));
      }
      else if(RobotVars->phantom_data.motor_temperature[0]<(2./3.) && RobotVars->phantom_data.motor_temperature[0]>=(1./3.))
      {
	//yellow
	gtk_label_set_markup (GTK_LABEL (RobotVars->updt_labels.motor_1_temp), g_markup_printf_escaped ("Motor 1: <span foreground=\"gold\"><b>%s</b></span> ", text.c_str()));
      }
      else
      {
	//red
	gtk_label_set_markup (GTK_LABEL (RobotVars->updt_labels.motor_1_temp), g_markup_printf_escaped ("Motor 1: <span foreground=\"red\"><b>%s</b></span> ", text.c_str()));
      }
      text = str(boost::format("%3.1f") % AvoidMinusZero(RobotVars->phantom_data.motor_temperature[1]));
      if(RobotVars->phantom_data.motor_temperature[1]<(1./3.))
      {
	//green
	gtk_label_set_markup (GTK_LABEL (RobotVars->updt_labels.motor_2_temp), g_markup_printf_escaped ("Motor 2: <span foreground=\"green\"><b>%s</b></span> ", text.c_str()));
      }
      else if(RobotVars->phantom_data.motor_temperature[1]<(2./3.) && RobotVars->phantom_data.motor_temperature[1]>=(1./3.))
      {
	//yellow
	gtk_label_set_markup (GTK_LABEL (RobotVars->updt_labels.motor_2_temp), g_markup_printf_escaped ("Motor 2: <span foreground=\"gold\"><b>%s</b></span> ", text.c_str()));
      }
      else
      {
	//red
	gtk_label_set_markup (GTK_LABEL (RobotVars->updt_labels.motor_2_temp), g_markup_printf_escaped ("Motor 2: <span foreground=\"red\"><b>%s</b></span> ", text.c_str()));
      }
      text = str(boost::format("%3.1f") % AvoidMinusZero(RobotVars->phantom_data.motor_temperature[2]));
      if(RobotVars->phantom_data.motor_temperature[2]<(1./3.))
      {
	//green
	gtk_label_set_markup (GTK_LABEL (RobotVars->updt_labels.motor_3_temp), g_markup_printf_escaped ("Motor 3: <span foreground=\"green\"><b>%s</b></span> ", text.c_str()));
      }
      else if(RobotVars->phantom_data.motor_temperature[2]<(2./3.) && RobotVars->phantom_data.motor_temperature[2]>=(1./3.))
      {
	//yellow
	gtk_label_set_markup (GTK_LABEL (RobotVars->updt_labels.motor_3_temp), g_markup_printf_escaped ("Motor 3: <span foreground=\"gold\"><b>%s</b></span> ", text.c_str()));
      }
      else
      {
	//red
	gtk_label_set_markup (GTK_LABEL (RobotVars->updt_labels.motor_3_temp), g_markup_printf_escaped ("Motor 3: <span foreground=\"red\"><b>%s</b></span> ", text.c_str()));
      }
      
      //phantom speed
      text = "Magnitude: " +
      str(boost::format("%3.1f") % AvoidMinusZero(RobotVars->phantom_data.average_speed.magnitude())) +
      " [mm/s] (" + str(boost::format("%3.1f") % AvoidMinusZero(RobotVars->phantom_data.average_speed[0])) +
      "," + str(boost::format("%3.1f") % AvoidMinusZero(RobotVars->phantom_data.average_speed[1])) +
      "," + str(boost::format("%3.1f") % AvoidMinusZero(RobotVars->phantom_data.average_speed[2])) + ")";
      
      gtk_label_set_text((GtkLabel *)RobotVars->updt_labels.phantom_speed_label, text.c_str());
    } //end of notebook page == 1
    else if(RobotVars->parameters.selected_notebook_page==0)
    {
      if(RobotVars->haptics_data.chosen_demo == 1)
      {
	gtk_label_set_text((GtkLabel *)RobotVars->updt_labels.workspace_zone_label, "Workspace Demonstration ONLINE");
      }
      else if(RobotVars->haptics_data.chosen_demo == 2)
      {
	hduVector3Dd normal = RobotVars->haptics_data.demo_2_Plane.normal();
	text = "Plane navigation: At " +
	str(boost::format("%3.1f") % AvoidMinusZero((double)RobotVars->haptics_data.demo2_distance_to_plane)) +
	" mm to plane surface\nPlane equation: (" +
	str(boost::format("%3.1f") % AvoidMinusZero(normal[0])) +
	").X + (" +
	str(boost::format("%3.1f") % AvoidMinusZero(normal[1])) +
	").Y + (" +
	str(boost::format("%3.1f") % AvoidMinusZero(normal[2])) +
	").Z + (" +
	str(boost::format("%3.1f") % AvoidMinusZero(RobotVars->haptics_data.demo_2_Plane.d())) + 
	") = 0";
	gtk_label_set_markup (GTK_LABEL (RobotVars->updt_labels.drawing_demo_label), g_markup_printf_escaped ("%s", text.c_str()));
      }
      else if(RobotVars->haptics_data.chosen_demo == 4)
      {
	if(RobotVars->parameters.kinematic_model == 4)
	{
	  text = "Balancing Support Leg:\nCOG = X:" + 
	  str(boost::format("%3.1f") % AvoidMinusZero((double)RobotVars->robot_kin_data.COG_detached_leg_left[0])) +
	  " [mm] Y:" + 
	  str(boost::format("%3.1f") % AvoidMinusZero((double)RobotVars->robot_kin_data.COG_detached_leg_left[1])) +
	  " [mm] Z:" + 
	  str(boost::format("%3.1f") % AvoidMinusZero((double)RobotVars->robot_kin_data.COG_detached_leg_left[2])) +
	  " [mm]";
	  gtk_label_set_markup (GTK_LABEL (RobotVars->updt_labels.leg_balancing_demo_label), g_markup_printf_escaped ("%s", text.c_str()));
	}
	else if(RobotVars->parameters.kinematic_model == 5)
	{
	  text = "Balancing Support Leg:\nCOG = X:" + 
	  str(boost::format("%3.1f") % AvoidMinusZero((double)RobotVars->robot_kin_data.COG_detached_leg_right[0])) +
	  " [mm] Y:" + 
	  str(boost::format("%3.1f") % AvoidMinusZero((double)RobotVars->robot_kin_data.COG_detached_leg_right[1])) +
	  " [mm] Z:" + 
	  str(boost::format("%3.1f") % AvoidMinusZero((double)RobotVars->robot_kin_data.COG_detached_leg_right[2])) +
	  " [mm]";
	  gtk_label_set_markup (GTK_LABEL (RobotVars->updt_labels.leg_balancing_demo_label), g_markup_printf_escaped ("%s", text.c_str()));
	}
      }
      else
      {
	
      }
      
      // force magnitude label
      text = str(boost::format("%3.1f") % AvoidMinusZero((double)RobotVars->haptics_data.applied_force.magnitude()));
      gtk_label_set_markup (GTK_LABEL (RobotVars->updt_labels.force_magnitude_label), g_markup_printf_escaped ("<u>Force magnitude</u>: <b>%s</b> [N] (sent to the device)", text.c_str()));
      
    }//end of notebook page == 0
    else
    {
      gtk_label_set_text((GtkLabel *)RobotVars->updt_labels.workspace_zone_label, "Workspace location: [Start haptic loop to update]");
      gtk_label_set_text((GtkLabel *)RobotVars->updt_labels.force_magnitude_label, "Force magnitude: --- [N] (sent to the device)");
      gtk_label_set_text((GtkLabel *)RobotVars->updt_labels.drawing_demo_label, "Plane navigation: -------------\nPlane equation: ax + by + cz + d = 0");
    }
  }
  return;
}

void UpdateForceBars(shared_vars_t*RobotVars)
{
  // X force bars
  if(RobotVars->haptics_data.applied_force[0] < 0.)
  {
    gtk_progress_bar_set_fraction((GtkProgressBar *)RobotVars->updt_labels.pos_x_force_bar, 0.);
    if(RobotVars->haptics_data.applied_force[0] < -RobotVars->phantom_data.max_force)
      gtk_progress_bar_set_fraction((GtkProgressBar *)RobotVars->updt_labels.neg_x_force_bar, 1.);
    else
      gtk_progress_bar_set_fraction((GtkProgressBar *)RobotVars->updt_labels.neg_x_force_bar, RobotVars->haptics_data.applied_force[0] / (-RobotVars->phantom_data.max_force));
  }
  else
  {
    gtk_progress_bar_set_fraction((GtkProgressBar *)RobotVars->updt_labels.neg_x_force_bar, 0.);
    if(RobotVars->haptics_data.applied_force[0] > RobotVars->phantom_data.max_force)
      gtk_progress_bar_set_fraction((GtkProgressBar *)RobotVars->updt_labels.pos_x_force_bar, 1.);
    else
      gtk_progress_bar_set_fraction((GtkProgressBar *)RobotVars->updt_labels.pos_x_force_bar, RobotVars->haptics_data.applied_force[0] / RobotVars->phantom_data.max_force);
  }
  // Y force bars
  if(RobotVars->haptics_data.applied_force[1] < 0.)
  {
    gtk_progress_bar_set_fraction((GtkProgressBar *)RobotVars->updt_labels.pos_y_force_bar, 0.);
    if(RobotVars->haptics_data.applied_force[1] < -RobotVars->phantom_data.max_force)
      gtk_progress_bar_set_fraction((GtkProgressBar *)RobotVars->updt_labels.neg_y_force_bar, 1.);
    else
      gtk_progress_bar_set_fraction((GtkProgressBar *)RobotVars->updt_labels.neg_y_force_bar, RobotVars->haptics_data.applied_force[1] / (-RobotVars->phantom_data.max_force));
  }
  else
  {
    gtk_progress_bar_set_fraction((GtkProgressBar *)RobotVars->updt_labels.neg_y_force_bar, 0.);
    if(RobotVars->haptics_data.applied_force[1] > RobotVars->phantom_data.max_force)
      gtk_progress_bar_set_fraction((GtkProgressBar *)RobotVars->updt_labels.pos_y_force_bar, 1.);
    else
      gtk_progress_bar_set_fraction((GtkProgressBar *)RobotVars->updt_labels.pos_y_force_bar, RobotVars->haptics_data.applied_force[1] / RobotVars->phantom_data.max_force);
  }
  // Z force bars
  if(RobotVars->haptics_data.applied_force[2] < 0.)
  {
    gtk_progress_bar_set_fraction((GtkProgressBar *)RobotVars->updt_labels.pos_z_force_bar, 0.);
    if(RobotVars->haptics_data.applied_force[2] < -RobotVars->phantom_data.max_force)
      gtk_progress_bar_set_fraction((GtkProgressBar *)RobotVars->updt_labels.neg_z_force_bar, 1.);
    else
      gtk_progress_bar_set_fraction((GtkProgressBar *)RobotVars->updt_labels.neg_z_force_bar, RobotVars->haptics_data.applied_force[2] / (-RobotVars->phantom_data.max_force));
  }
  else
  {
    gtk_progress_bar_set_fraction((GtkProgressBar *)RobotVars->updt_labels.neg_z_force_bar, 0.);
    if(RobotVars->haptics_data.applied_force[2] > RobotVars->phantom_data.max_force)
      gtk_progress_bar_set_fraction((GtkProgressBar *)RobotVars->updt_labels.pos_z_force_bar, 1.);
    else
      gtk_progress_bar_set_fraction((GtkProgressBar *)RobotVars->updt_labels.pos_z_force_bar, RobotVars->haptics_data.applied_force[2] / RobotVars->phantom_data.max_force);
  }
}

void UpdateStatusBar(shared_vars_t*RobotVars)
{
  string text;

    // throw statusbar messages
    
  if(RobotVars->servo->IsActive() && !RobotVars->phantom_data.phantom_on)
  {
    gtk_statusbar_push(GTK_STATUSBAR(RobotVars->updt_labels.status_bar),
		       gtk_statusbar_get_context_id(GTK_STATUSBAR(RobotVars->updt_labels.status_bar),"statusbar_info"),
		       ":: Hitec servomotor COMM active :: PHANToM OMNI inactive ::");
  }
  else if(RobotVars->servo->IsActive() && RobotVars->phantom_data.phantom_on)
  {
    text=":: Hitec servomotor COMM active :: PHANToM OMNI active :: Control Frequency: " + str(boost::format("%d") % AvoidMinusZero(RobotVars->parameters.control_rate)) + " [Hz] :: Haptic Rendering Frequency: " + str(boost::format("%d") % AvoidMinusZero(RobotVars->parameters.haptics_rate)) + " [Hz] ::";
    
    gtk_statusbar_push(GTK_STATUSBAR(RobotVars->updt_labels.status_bar),
		       gtk_statusbar_get_context_id(GTK_STATUSBAR(RobotVars->updt_labels.status_bar),"statusbar_info"),
		       text.c_str());
  }
  else if(!RobotVars->servo->IsActive() && RobotVars->phantom_data.phantom_on)
  {
    gtk_statusbar_push(GTK_STATUSBAR(RobotVars->updt_labels.status_bar),
		       gtk_statusbar_get_context_id(GTK_STATUSBAR(RobotVars->updt_labels.status_bar),"statusbar_info"),
		       ":: Hitec servomotor COMM inactive :: PHANToM OMNI active ::");
  }
  else
  {
    gtk_statusbar_push(GTK_STATUSBAR(RobotVars->updt_labels.status_bar),
		       gtk_statusbar_get_context_id(GTK_STATUSBAR(RobotVars->updt_labels.status_bar),"statusbar_info"),
		       ":: Communications inactive ::");
  }
}
