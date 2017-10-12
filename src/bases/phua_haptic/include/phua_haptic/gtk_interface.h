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
 * @brief gtk_interface.h file for this module. Contains includes, prototypes and defines.
 * @author pedro_cruz
 * @version 2.0
 * @date 7 May 2012
 *@{
 */
#ifndef __GTK_INTERFACE_H_
#define __GTK_INTERFACE_H_

#include <gtk/gtk.h>
#include <glib.h>
#include <gdk/gdkkeysyms.h>

#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <time.h>
#include <string.h>
#include <math.h>

// #include <ros/ros.h>
#include <ros/package.h>

#include <boost/lexical_cast.hpp>
#include <boost/format.hpp>

#include <phua_haptic/humanoid_control_functions.h>
#include <phua_haptic/miscellaneous.h>
#include <phua_haptic/gtk_aux.h>
#include <phua_haptic/hd_hl_apis_callbacks.h>

#define STR_LEN 512
#define BACK_FACING -1
#define FRONT_FACING 1

//CALLBACKS
/** 
* @brief Callback for destroy signal.

This function executes any time a destroy signal is caught by the interface.
* @param object a GtkObject reference.
* @param user_data a gpointer type data.
* @return none.
*/
void on_application_exit(GtkObject *object, gpointer user_data);

/** 
* @brief Callback for menu "about" dialog window construction "activate" event.

This function executes any time the user presses the "About" menu in the menu bar.
Contains developer information and contacts.
* @param object a GtkObject reference.
* @param user_data a gpointer type data.
* @return none.
*/
void on_about_menu_item_activate(GtkObject *object, gpointer user_data);

/** 
* @brief Callback for robot home position button "clicked" event.

This function executes any time the user presses the robot home position button.
It executes necessary commands for setting the robot to its home position.
* @param widget a GtkWidget reference.
* @param user_data a gpointer type data.
* @return none.
*/
void on_button_set_robot_home_pos_clicked(GtkWidget * widget, gpointer user_data);

/** 
* @brief Callback for loop start button "clicked" event.

This function executes any time the user presses the start/stop loop button.
It executes required functions for haptic/control loops.
* @param widget a GtkWidget reference.
* @param user_data a gpointer type data.
* @return none.
*/
void on_button_start_loop_clicked(GtkWidget * widget, gpointer user_data);

/** 
* @brief Callback for control type combo box item "changed" event.

This function executes any time the user changes the control type combo box.
* @param widget a GtkWidget reference.
* @param user_data a gpointer type data.
* @return none.
*/
void on_page1_left_top_frame_combobox_changed_event(GtkWidget *widget, gpointer user_data);

/** 
* @brief Callback for constant servo speed hscale "changed" event.

This function executes any time the user changes the hscale value.
* @param adjustment a GtkAdjustment reference.
* @param user_data a gpointer type data.
* @return none.
*/
void on_ajustament_hscale_value_changed_event(GtkAdjustment *adjustment, gpointer user_data);

/** 
* @brief Callback for constant speed checkbox "toggled" event.

This function executes any time the user toggles the checkbox.
It hides and shows items on the frame the checkbox is on.
* @param widget a GtkWidget reference.
* @param other_checkbox a gpointer to the other checkbox widget.
* @return none.
*/
void on_constant_speed_radio_checkbox_toggled(GtkWidget *widget, gpointer other_checkbox);

/** 
* @brief Callback for controled speed checkbox "toggled" event.

This function executes any time the user toggles the checkbox.
It hides and shows items on the frame the checkbox is on.
* @param widget a GtkWidget reference.
* @param other_checkbox a gpointer to the other checkbox widget.
* @return none.
*/
void on_controled_speed_radio_checkbox_toggled(GtkWidget *widget, gpointer other_checkbox);

/** 
* @brief Callback for the set position id selection combo box "changed" event.

This function executes any time the user toggles the combo box for id selection to set position of a joint.
* @param widget a GtkWidget reference.
* @param label_data a gpointer to the angle limitation label widget.
* @return none.
*/
void on_page2_select_id_combobox_for_position_changed_event(GtkWidget *widget, gpointer label_data);

/** 
* @brief Callback for the set position button "clicked" event.

This function executes any time the user clicks the set position button.
It sends a set position command for the joint selected in the combo box with the position converted from the entry box input.
* @param widget a GtkWidget reference.
* @param user_data a gpointer.
* @return none.
*/
void on_button_setpos_clicked(GtkWidget *widget, gpointer user_data);

/** 
* @brief Callback for the release all servos button "clicked" event.

This function executes any time the user clicks the release all servomotors button and sends the command accordingly.
* @param widget a GtkWidget reference.
* @param user_data a gpointer.
* @return none.
*/
void on_button_release_all_clicked(GtkWidget *widget, gpointer user_data);

/** 
* @brief Callback for the set go button "toggled" event.

This function executes any time the user toggles the set go button and sends the command accordingly.
Toggling this button untoggles the other.
* @param widget a GtkWidget reference.
* @param other_toggle_button a gpointer to the set stop button.
* @return none.
*/
void on_button_go_toggled(GtkWidget *widget, gpointer other_toggle_button);

/** 
* @brief Callback for the set stop button "toggled" event.

This function executes any time the user toggles the set stop button and sends the command accordingly.
Toggling this button untoggles the other.
* @param widget a GtkWidget reference.
* @param other_toggle_button a gpointer to the set stop go button.
* @return none.
*/
void on_button_stop_toggled(GtkWidget *widget, gpointer other_toggle_button);

/** 
* @brief Callback for the set position to all servomotors "clicked" event.

This function executes any time the user presses the button send a position to all the servos.
The functions will atempt to convert the position. If the position is not reachable, the joint holds still.
* @param widget a GtkWidget reference.
* @param user_data a gpointer.
* @return none.
*/
void on_button_set_pos_all_clicked(GtkWidget *widget, gpointer user_data);

/** 
* @brief Callback for the set speed button "clicked" event.

This function executes any time the user presses the button to set speed.
* @param widget a GtkWidget reference.
* @param user_data a gpointer.
* @return none.
*/
void on_button_setspeed_clicked(GtkWidget *widget, gpointer user_data);

/** 
* @brief Callback for update robot data button "clicked" event.

This function executes any time the user presses the update robot data button, sending commands to reads joint values and updating limb end coordinates.
* @param widget a GtkWidget reference.
* @param user_data a gpointer.
* @return none.
*/
void on_button_vbuttonbox_update_robot_data_clicked(GtkWidget *widget, gpointer user_data);

/** 
* @brief Callback for control resolution checkboxes "toggled" event.

This function updates the data structure with chosen control resolution.
* @param widget a GtkWidget reference.
* @param user_data a gpointer.
* @return none.
*/
void on_control_resolution_toggled(GtkWidget *widget, gpointer user_data);

/** 
* @brief Callback for workspace scaling checkboxes "toggled" event.

This function updates the data structure with chosen workspace scale.
* @param widget a GtkWidget reference.
* @param user_data a gpointer.
* @return none.
*/
void on_workspace_scaling_toggled(GtkWidget *widget, gpointer user_data);

/** 
* @brief Callback for test arm inverse kinematics button "clicked" event.

This function tries X, Y and Z coordinates thru inverse kinematics defined in entry text boxes.
* @param widget a GtkWidget reference.
* @param user_data a gpointer.
* @return none.
*/
void on_button_test_invkin_clicked(GtkWidget *widget, gpointer user_data);

/** 
* @brief Callback for calibration button "clicked" event.

This function launches a small window wich is used to check the joystick calibration.
Instructions are given in the window text.
* @param widget a GtkWidget reference.
* @param user_data a gpointer.
* @return none.
*/
void on_button_calibration_clicked(GtkWidget *widget, gpointer user_data);

/** 
* @brief Callback for notebook page "changed" event.

This function is called by the "changed" event from the main page notebook.
* @param notebook pointer to the notebook GtkWidget.
* @param page pointer to the notebook page.
* @param page_num the page number from the selected page.
* @param user_data a gpointer.
* @return none.
*/
void on_notebook_change_current_page(GtkNotebook *notebook, GtkNotebookPage *page, guint page_num, gpointer user_data);

/** 
* @brief Callback for back facing menu item choice "activate" event.

[NOT WORKING]This function updates the control type to back-facing.
* @param menuitem a GtkMenuItem reference.
* @param user_data a gpointer.
* @return none.
*/
void on_back_facing_menu_item_activate(GtkMenuItem *menuitem, gpointer user_data);

/** 
* @brief Callback for front facing menu item choice "activate" event.

[DEFAULT]This function updates the control type to front-facing.
* @param menuitem a GtkMenuItem reference.
* @param user_data a gpointer.
* @return none.
*/
void on_front_facing_menu_item_activate(GtkMenuItem *menuitem, gpointer user_data);

/** 
* @brief Callback for demo choice checkboxes "toggled" event.

This function is called after the user choses a demo.
* @param togglebutton a GtkToggleButton reference.
* @param user_data a gpointer.
* @return none.
*/
void on_demo_checkboxes_toggled(GtkToggleButton *togglebutton, gpointer user_data);

/** 
* @brief Callback for inverse kinematics combobox "changed" event.

This function is called by the inverse kinematics combobox "changed" event signal.
* @param widget a GtkWidget reference.
* @param user_data a gpointer.
* @return none.
*/
void select_inv_kin_combobox_changed_event(GtkWidget *widget, gpointer user_data);

/** 
* @brief Callback for user path store points button "clicked" event.

This function is called by the clicking of the store points button for path following.
* @param widget a GtkWidget reference.
* @param user_data a gpointer.
* @return none.
*/
void on_user_path_demo_point_store_button_clicked(GtkWidget *widget, gpointer user_data);

/** 
* @brief Callback for path following run once/loop checkboxes "toggled" event.

This function is called after the user choses to run the path once or in loop.
* @param togglebutton a GtkToggleButton reference.
* @param user_data a gpointer.
* @return none.
*/
void user_path_demo_run_checkbox_toggled(GtkToggleButton *togglebutton, gpointer user_data);

/** 
* @brief Callback for user path run points button "clicked" event.

This function is called by the clicking of the run points button for path following.
* @param widget a GtkWidget reference.
* @param user_data a gpointer.
* @return none.
*/
void on_user_path_demo_run_button_clicked(GtkWidget *widget, gpointer user_data);

/** 
* @brief Callback for clear path points button "clicked" event.

This function is called by the clicking of the clear button for path following.
* @param widget a GtkWidget reference.
* @param user_data a gpointer.
* @return none.
*/
void on_user_path_demo_clear_button_clicked(GtkWidget *widget, gpointer user_data);

/** 
* @brief Function launched in timeout for taking care of label updates.

This function loops wainting for requests to update the labels.
* @param data_struct a data structure passed on.
* @return NULL.
*/
gboolean update_watcher(gpointer data_struct);

//MAIN FUNCTION FOR INTERFACE BUILD
/** 
* @brief Main interface function. Object/Widget building.

This function builds the interface and the widgets.
It also connects the signals and events and initializes main GTK loop.
* @param dummy a dummy void type input (required for threading).
* @return NULL.
*/
void *interface_init(void *dummy);

#endif
/**
 *@}
*/
