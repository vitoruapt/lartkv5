/**
 * \file
 * \brief GTK interface functions
 */

#include <phua_haptic/gtk_interface.h>

using namespace std;

/*~~~~~~~~~~~~~~~~~~~~~~~~~ 
||  _____CALLBACKS_____   ||
 ~~~~~~~~~~~~~~~~~~~~~~~~~*/
void on_application_exit (GtkObject *object, gpointer user_data)
{
  shared_vars_t*RobotVars=(shared_vars_t*)user_data;
  pthread_mutex_lock(&(RobotVars->mutex_gtk));
  RobotVars->parameters.exit_status=TRUE;
  pthread_mutex_unlock(&(RobotVars->mutex_gtk));
  GtkWidget *toplevel = gtk_widget_get_toplevel((GtkWidget *)object);
  gtk_widget_hide_all(toplevel);
  gtk_main_quit();
}

void on_button_set_robot_home_pos_clicked(GtkWidget * widget, gpointer user_data)
{
  shared_vars_t*RobotVars=(shared_vars_t*)user_data;
  cout<<"-> Setting robot at home position";fflush(stdout);
  SetRobotHomePosition(RobotVars);sleep(0.25);
  cout<<"...";fflush(stdout);sleep(0.25);
  UpdateJointDataByID(1000, 0., RobotVars);
  cout<<"...";fflush(stdout);
  UpdateKinematicModelDirKin(RobotVars);
  cout<<"...";fflush(stdout);
  RobotVars->update_labels=TRUE;
  cout<<"...";fflush(stdout);
  cout<<"Done!"<<endl;fflush(stdout);
}

void on_button_start_loop_clicked(GtkWidget * widget, gpointer user_data)
{
  shared_vars_t*RobotVars=(shared_vars_t*)user_data;
  //update button & label
  const gchar start_text[STR_LEN]="\nSTART HAPTICS/CONTROL LOOP\n";
  const gchar stop_text[STR_LEN]="\nSTOP HAPTICS/CONTROL LOOP\n";
  
  if(!strcmp(gtk_button_get_label((GtkButton *)widget),start_text))
  {
    if(RobotVars->parameters.kinematic_model!=0)
    {
      //execute on start
      gtk_button_set_label((GtkButton *)widget,stop_text);
      pthread_mutex_lock(&(RobotVars->mutex_gtk));
      //update robot information
      UpdateJointDataByID(1000, 0., RobotVars);
      UpdateKinematicModelDirKin(RobotVars);
      RobotVars->haptic_loop_start=TRUE;
      pthread_mutex_unlock(&(RobotVars->mutex_gtk));
    }
  }
  else
  {
    //execute on stop
    gtk_button_set_label((GtkButton *)widget,start_text);
    pthread_mutex_lock(&(RobotVars->mutex_gtk));
    RobotVars->haptic_loop_start=FALSE;
    RobotVars->haptics_data.demo_user_path_point_storing = FALSE;
    RobotVars->haptics_data.demo_user_path_run_start = FALSE;
    pthread_mutex_unlock(&(RobotVars->mutex_gtk));
  }
}

void on_page1_left_top_frame_combobox_changed_event(GtkWidget *widget, gpointer user_data)
{
  shared_vars_t*RobotVars=(shared_vars_t*)user_data;
  gint selection=gtk_combo_box_get_active((GtkComboBox *)widget);
  GtkWidget *toplevel = gtk_widget_get_toplevel (widget);
  g_object_set_data((GObject *)toplevel,"kinematic_selection",&selection);
  RobotVars->parameters.kinematic_model=selection;
  
  //update labels
  if(selection<=3 && selection>0)
  {
    gtk_label_set_markup ((GtkLabel *)(RobotVars->updt_labels.joints_left_side),
			  g_markup_printf_escaped ("<span underline=\"single\"><b>%s</b></span>", "Robot Left Arm end coordinates / Roll/Pitch/Yaw angles:"));
    gtk_label_set_markup ((GtkLabel *)(RobotVars->updt_labels.joints_right_side),
			  g_markup_printf_escaped ("<span underline=\"single\"><b>%s</b></span>", "Robot Right Arm end coordinates / Roll/Pitch/Yaw angles:"));
  }
  else if(selection>=4 && selection<=5)
  {
    gtk_label_set_markup ((GtkLabel *)(RobotVars->updt_labels.joints_left_side),
			  g_markup_printf_escaped ("<span underline=\"single\"><b>%s</b></span>", "Robot Left Detached Leg end coordinates / Roll/Pitch/Yaw angles:"));
    gtk_label_set_markup ((GtkLabel *)(RobotVars->updt_labels.joints_right_side),
			  g_markup_printf_escaped ("<span underline=\"single\"><b>%s</b></span>", "Robot Right Detached Leg end coordinates / Roll/Pitch/Yaw angles:"));
  }
  
}

void on_ajustament_hscale_value_changed_event(GtkAdjustment *adjustment, gpointer user_data)
{
  shared_vars_t*RobotVars=(shared_vars_t*)user_data;
  static gdouble manual_speed_value;
  manual_speed_value=gtk_adjustment_get_value((GtkAdjustment *)adjustment);
  pthread_mutex_lock(&(RobotVars->mutex_gtk));
  RobotVars->parameters.manual_speed=manual_speed_value;
  pthread_mutex_unlock(&(RobotVars->mutex_gtk));
}

void on_constant_speed_radio_checkbox_toggled(GtkWidget *widget, gpointer other_checkbox)
{
  static bool man;
  static bool autom;
  GtkWidget *toplevel = gtk_widget_get_toplevel (widget);
  GList *child_list=gtk_container_get_children((GtkContainer *)gtk_widget_get_parent((GtkWidget *)widget));
  if(gtk_toggle_button_get_active((GtkToggleButton *)widget))
  {
    //show both the label and the scale bar
    gtk_widget_set_visible((GtkWidget *)g_list_nth_data(child_list,1),TRUE);
    //hide the progress bar
    gtk_widget_set_visible((GtkWidget *)g_list_nth_data(child_list,3),FALSE);
    gtk_toggle_button_set_active((GtkToggleButton *)other_checkbox,FALSE);
    man=TRUE;
    autom=FALSE;
  }
  else
  {
    //hide both the label and the scale bar
    gtk_widget_set_visible((GtkWidget *)g_list_nth_data(child_list,1),FALSE);
    //show the progress bar
    gtk_widget_set_visible((GtkWidget *)g_list_nth_data(child_list,3),TRUE);
    gtk_toggle_button_set_active((GtkToggleButton *)other_checkbox,TRUE);
    man=FALSE;
    autom=TRUE;
  }
  while (g_main_context_iteration (NULL, FALSE));
  g_list_free(child_list);
  g_object_set_data((GObject *)toplevel,"manual_speed_control",&man);
  g_object_set_data((GObject *)toplevel,"automatic_speed_control",&autom);
}

void on_controled_speed_radio_checkbox_toggled(GtkWidget *widget, gpointer other_checkbox)
{
  static bool man;
  static bool autom;
  GtkWidget *toplevel = gtk_widget_get_toplevel (widget);
  GList *child_list=gtk_container_get_children((GtkContainer *)gtk_widget_get_parent((GtkWidget *)widget));
  if(gtk_toggle_button_get_active((GtkToggleButton *)widget))
  {
    //hide both the label and the scale bar
    gtk_widget_set_visible((GtkWidget *)g_list_nth_data(child_list,1),FALSE);
    //show the progress bar
    gtk_widget_set_visible((GtkWidget *)g_list_nth_data(child_list,3),TRUE);
    gtk_toggle_button_set_active((GtkToggleButton *)other_checkbox,FALSE);
    man=FALSE;
    autom=TRUE;
  }
  else
  {
    //show both the label and the scale bar
    gtk_widget_set_visible((GtkWidget *)g_list_nth_data(child_list,1),TRUE);
    //hide the progress bar
    gtk_widget_set_visible((GtkWidget *)g_list_nth_data(child_list,3),FALSE);
    gtk_toggle_button_set_active((GtkToggleButton *)other_checkbox,TRUE);
    man=TRUE;
    autom=FALSE;
  }
  while (g_main_context_iteration (NULL, FALSE));
  g_list_free(child_list);
  g_object_set_data((GObject *)toplevel,"manual_speed_control",&man);
  g_object_set_data((GObject *)toplevel,"automatic_speed_control",&autom);
}

void on_page2_select_id_combobox_for_position_changed_event(GtkWidget *widget, gpointer label_data)
{
  //access child list
  GList *child_list=gtk_container_get_children((GtkContainer *)gtk_widget_get_parent((GtkWidget *)widget));
  cout<<"On individual joint control:"<<endl<<"Selected option "<<gtk_combo_box_get_active_text((GtkComboBox *)widget)<<"."<<endl;
  gint selection=gtk_combo_box_get_active((GtkComboBox *)widget);
  
  switch (selection)
  {
    case 0://none
      gtk_label_set_text((GtkLabel *)label_data,"[---]");gtk_spin_button_set_range((GtkSpinButton *) g_list_next(g_list_next(g_list_next(child_list)))->data,0.,0.);break;
    case 1://head tilt
      gtk_label_set_text((GtkLabel *)label_data,"[0...10 deg]");gtk_spin_button_set_range((GtkSpinButton *) g_list_next(g_list_next(g_list_next(child_list)))->data,0.,10.);break;
    case 2://Right Shoulder Flexion
      gtk_label_set_text((GtkLabel *)label_data,"[-40...140 deg]");gtk_spin_button_set_range((GtkSpinButton *) g_list_next(g_list_next(g_list_next(child_list)))->data,-40.,140.);break;
    case 3://Right Shoulder Abduction
      gtk_label_set_text((GtkLabel *)label_data,"[0...180 deg]");gtk_spin_button_set_range((GtkSpinButton *) g_list_next(g_list_next(g_list_next(child_list)))->data,0.,180.);break;
    case 4://Right Elbow Flexion
      gtk_label_set_text((GtkLabel *)label_data,"[0...120 deg]");gtk_spin_button_set_range((GtkSpinButton *) g_list_next(g_list_next(g_list_next(child_list)))->data,0.,120.);break;
    case 5://Left Shoulder Flexion
      gtk_label_set_text((GtkLabel *)label_data,"[-40...140 deg]");gtk_spin_button_set_range((GtkSpinButton *) g_list_next(g_list_next(g_list_next(child_list)))->data,-40.,140.);break;
    case 6://Left Shoulder Abduction
      gtk_label_set_text((GtkLabel *)label_data,"[0...180 deg]");gtk_spin_button_set_range((GtkSpinButton *) g_list_next(g_list_next(g_list_next(child_list)))->data,0.,180.);break;
    case 7://Left Elbow Flexion
      gtk_label_set_text((GtkLabel *)label_data,"[0...120 deg]");gtk_spin_button_set_range((GtkSpinButton *) g_list_next(g_list_next(g_list_next(child_list)))->data,0.,120.);break;
    case 8://Torso Rotation
      gtk_label_set_text((GtkLabel *)label_data,"[-90...90 deg]");gtk_spin_button_set_range((GtkSpinButton *) g_list_next(g_list_next(g_list_next(child_list)))->data,-90.,90.);break;
    case 9://Torso Flexion
      gtk_label_set_text((GtkLabel *)label_data,"[-15...90 deg]");gtk_spin_button_set_range((GtkSpinButton *) g_list_next(g_list_next(g_list_next(child_list)))->data,-15.,90.);break;
    case 10://Torso Lateral Flexion
      gtk_label_set_text((GtkLabel *)label_data,"[-45...45 deg]");gtk_spin_button_set_range((GtkSpinButton *) g_list_next(g_list_next(g_list_next(child_list)))->data,-45.,45.);break;
    case 11://Right Ankle Inversion
      gtk_label_set_text((GtkLabel *)label_data,"[-30...45 deg]");gtk_spin_button_set_range((GtkSpinButton *) g_list_next(g_list_next(g_list_next(child_list)))->data,-30.,45.);break;
    case 12://Right Ankle Flexion
      gtk_label_set_text((GtkLabel *)label_data,"[-40...20 deg]");gtk_spin_button_set_range((GtkSpinButton *) g_list_next(g_list_next(g_list_next(child_list)))->data,-40.,20.);break;
    case 13://Right Knee Flexion
      gtk_label_set_text((GtkLabel *)label_data,"[0...130 deg]");gtk_spin_button_set_range((GtkSpinButton *) g_list_next(g_list_next(g_list_next(child_list)))->data,0.,130.);break;
    case 14://Right Hip Abduction
      gtk_label_set_text((GtkLabel *)label_data,"[-40...45 deg]");gtk_spin_button_set_range((GtkSpinButton *) g_list_next(g_list_next(g_list_next(child_list)))->data,-40.,45.);break;
    case 15://Right Hip Flexion
      gtk_label_set_text((GtkLabel *)label_data,"[-30...120 deg]");gtk_spin_button_set_range((GtkSpinButton *) g_list_next(g_list_next(g_list_next(child_list)))->data,-30.,120.);break;
    case 16://Left Ankle Inversion
      gtk_label_set_text((GtkLabel *)label_data,"[-30...45 deg]");gtk_spin_button_set_range((GtkSpinButton *) g_list_next(g_list_next(g_list_next(child_list)))->data,-30.,45.);break;
    case 17://Left Ankle Flexion
      gtk_label_set_text((GtkLabel *)label_data,"[-40...20 deg]");gtk_spin_button_set_range((GtkSpinButton *) g_list_next(g_list_next(g_list_next(child_list)))->data,-40.,20.);break;
    case 18://Left Knee Flexion
      gtk_label_set_text((GtkLabel *)label_data,"[0...130 deg]");gtk_spin_button_set_range((GtkSpinButton *) g_list_next(g_list_next(g_list_next(child_list)))->data,0.,130.);break;
    case 19://Left Hip Abduction
      gtk_label_set_text((GtkLabel *)label_data,"[-40...45 deg]");gtk_spin_button_set_range((GtkSpinButton *) g_list_next(g_list_next(g_list_next(child_list)))->data,-40.,45.);break;
    case 20://Left Hip Flexion
      gtk_label_set_text((GtkLabel *)label_data,"[-30...120 deg]");gtk_spin_button_set_range((GtkSpinButton *) g_list_next(g_list_next(g_list_next(child_list)))->data,-30.,120.);break;}
  
  gtk_spin_button_set_value ((GtkSpinButton *) g_list_next(g_list_next(g_list_next(child_list)))->data, 0.);
  
  while (g_main_context_iteration (NULL, FALSE));
}

void on_button_setpos_clicked(GtkWidget *widget, gpointer user_data)
{
  shared_vars_t*RobotVars=(shared_vars_t*)user_data;
  
  if(RobotVars->servo->IsActive())
  {
    //access child list
    GList *child_list=gtk_container_get_children((GtkContainer *)gtk_widget_get_parent((GtkWidget *)widget));
    GList *child1=g_list_first(child_list);
    //retrieve id from combobox
    gint selection=gtk_combo_box_get_active((GtkComboBox *)child1->data);
    if(selection==0){
      g_list_free(child_list);
      return;
    }
    else
    {
      int id;
      short unsigned int servo_position;
      //read value
      double angle_position = gtk_spin_button_get_value((GtkSpinButton *) g_list_next(g_list_next(g_list_next(child_list)))->data);
      //convert selection to id
      switch (selection){
	case 1:
	  id=61;servo_position=ConvertJointAngleByID(id,angle_position);RobotVars->robot_kin_data.HeadTilt=angle_position;break;
	case 2:
	  id=54;servo_position=ConvertJointAngleByID(id,angle_position);RobotVars->robot_kin_data.RightShoulderFlexion=angle_position;break;
	case 3:
	  id=53;servo_position=ConvertJointAngleByID(id,angle_position);RobotVars->robot_kin_data.RightShoulderAbduction=angle_position;break;
	case 4:
	  id=51;servo_position=ConvertJointAngleByID(id,angle_position);RobotVars->robot_kin_data.RightElbowFlexion=angle_position;break;
	case 5:
	  id=44;servo_position=ConvertJointAngleByID(id,angle_position);RobotVars->robot_kin_data.LeftShoulderFlexion=angle_position;break;
	case 6:
	  id=43;servo_position=ConvertJointAngleByID(id,angle_position);RobotVars->robot_kin_data.LeftShoulderAbduction=angle_position;break;
	case 7:
	  id=41;servo_position=ConvertJointAngleByID(id,angle_position);RobotVars->robot_kin_data.LeftElbowFlexion=angle_position;break;
	case 8:
	  id=31;servo_position=ConvertJointAngleByID(id,angle_position);RobotVars->robot_kin_data.TorsoRotation=angle_position;break;
	case 9:
	  id=32;servo_position=ConvertJointAngleByID(id,angle_position);RobotVars->robot_kin_data.TorsoFlexion=angle_position;break;
	case 10:
	  id=33;servo_position=ConvertJointAngleByID(id,angle_position);RobotVars->robot_kin_data.TorsoLateralFlexion=angle_position;break;
	case 11:
	  id=11;servo_position=ConvertJointAngleByID(id,angle_position);RobotVars->robot_kin_data.RightAnkleInversion=angle_position;break;
	case 12:
	  id=12;servo_position=ConvertJointAngleByID(id,angle_position);RobotVars->robot_kin_data.RightAnkleFlexion=angle_position;break;
	case 13:
	  id=13;servo_position=ConvertJointAngleByID(id,angle_position);RobotVars->robot_kin_data.RightKneeFlexion=angle_position;break;
	case 14:
	  id=15;servo_position=ConvertJointAngleByID(id,angle_position);RobotVars->robot_kin_data.RightHipAbduction=angle_position;break;
	case 15:
	  id=16;servo_position=ConvertJointAngleByID(id,angle_position);RobotVars->robot_kin_data.RightHipFlexion=angle_position;break;
	case 16:
	  id=21;servo_position=ConvertJointAngleByID(id,angle_position);RobotVars->robot_kin_data.LeftAnkleInversion=angle_position;break;
	case 17:
	  id=22;servo_position=ConvertJointAngleByID(id,angle_position);RobotVars->robot_kin_data.LeftAnkleFlexion=angle_position;break;
	case 18:
	  id=23;servo_position=ConvertJointAngleByID(id,angle_position);RobotVars->robot_kin_data.LeftKneeFlexion=angle_position;break;
	case 19:
	  id=25;servo_position=ConvertJointAngleByID(id,angle_position);RobotVars->robot_kin_data.LeftHipAbduction=angle_position;break;
	case 20:
	  id=26;servo_position=ConvertJointAngleByID(id,angle_position); RobotVars->robot_kin_data.LeftHipFlexion=angle_position;break;
	default:
	  id=0;servo_position=ConvertJointAngleByID(id,angle_position);break;
      }
      //send command
      cout<<"Sending request to position ["<<servo_position<<"]"<<"["<<angle_position<<" deg]"<<" to servomotor id ["<<id<<"]."<<endl;
      int ret = RobotVars->servo->SetPosition(id,servo_position);
      cout<<"Servo response: "<<ret<<".";
      if(ret==0)
      {
	cout<<" Did not respond."<<endl;
      }
      else if(ret==0xFFFF)
      {
	cout<<" Some error occured. Value out of range or servomotor problem."<<endl;
      }
      else
      {
	cout<<endl;
      }
      
      if(id!=0)
      {
	if(id>40 && id<55)
	  UpdateArmsDirKinData(RobotVars);
	else if(id<29)
	  UpdateDetachedLegsDirKinData(RobotVars);
	  
	RobotVars->update_labels=TRUE;
      }
      //end of selection else
    }
    g_list_free(child_list);
  }
}


void on_button_release_all_clicked(GtkWidget *widget, gpointer user_data)
{
  shared_vars_t*RobotVars=(shared_vars_t*)user_data;
  cout<<"Sending command release to all servos."<<endl;
  int ret=RobotVars->servo->ReleaseServos();
  cout<<"Servos responded: "<<ret<<"."<<endl;
}

void on_button_go_toggled(GtkWidget *widget, gpointer user_data)
{
  shared_vars_t*RobotVars=(shared_vars_t*)user_data;
  if(gtk_toggle_button_get_active((GtkToggleButton *)RobotVars->updt_labels.go_button))
  {
    gtk_toggle_button_set_active((GtkToggleButton *)RobotVars->updt_labels.stop_button,FALSE);
    cout<<"Sending command GO to all servos."<<endl;
    int ret=RobotVars->servo->SetGoStop(1);
    cout<<"Servos responded: "<<ret<<"."<<endl;
  }
//   while (g_main_context_iteration (NULL, FALSE));
}

void on_button_stop_toggled(GtkWidget *widget, gpointer user_data)
{
  shared_vars_t*RobotVars=(shared_vars_t*)user_data;
  if(gtk_toggle_button_get_active((GtkToggleButton *)RobotVars->updt_labels.stop_button))
  {
    gtk_toggle_button_set_active((GtkToggleButton *)RobotVars->updt_labels.go_button,FALSE);
    cout<<"Sending command STOP to all servos."<<endl;
    int ret=RobotVars->servo->SetGoStop(0);
    cout<<"Servos responded: "<<ret<<"."<<endl;
  }
//   while (g_main_context_iteration (NULL, FALSE));
}

void on_about_menu_item_activate(GtkObject *object, gpointer user_data)
{
  const gchar * const author[STR_LEN] = {"Master's Degree Student @University of Aveiro:\nPedro Cruz <pmbc@ua.pt>",NULL};
  const gchar copyright[STR_LEN] ="No copyright at all \xc2\xa9 2012 Pedro Cruz";
  const gchar comments[STR_LEN] = "PHUA Project Haptic Interface";
  const gchar license[STR_LEN] = "There is no license for this product!\nJust get the friggin' thing to work!\n :-D";
  gtk_show_about_dialog (NULL,"authors", author,"comments", comments,
			  "copyright", copyright,"license",license,
			  "version", "v2.0","website", "http://lars.mec.ua.pt",
			  "website-label", "Link to LARS website and database.",
			  "program-name", "PHUA Haptic Interface",
			  "logo-icon-name", GTK_STOCK_EXECUTE,NULL); 
}

void on_button_set_pos_all_clicked(GtkWidget *widget, gpointer user_data)
{
  shared_vars_t*RobotVars=(shared_vars_t*)user_data;
  if(RobotVars->servo->IsActive())
  {
    //acess child list
    GList *child_list=gtk_container_get_children((GtkContainer *)gtk_widget_get_parent((GtkWidget *)widget));
    //read value from entry
    const gchar *angle_text=gtk_entry_get_text((GtkEntry *)g_list_next(g_list_first(child_list))->data);
    //convert position
    if(isNumeric(angle_text))
    {
      short unsigned int position = atoi(angle_text);
      //check range
      if(position>=600 && position<=2400)
      {
	//send command
	cout<<"Sending request to position "<<position<<" to all servomotors in the bus."<<endl;
	int ret=RobotVars->servo->SetPositionAllServos(position);
	cout<<"Servo responded: "<<ret<<"."<<endl;
	UpdateJointDataByID(1000,NULL,RobotVars);
	UpdateArmsDirKinData(RobotVars);
	RobotVars->update_labels=TRUE;
      }
      else
      {
	cout<<"Invalid value for instruction."<<endl;
      }
    }
    g_list_free(child_list);
  }
}

void on_button_setspeed_clicked(GtkWidget *widget, gpointer user_data)
{
  shared_vars_t*RobotVars=(shared_vars_t*)user_data;
	
  if(RobotVars->servo->IsActive())
  {
    //acess child list
    GList *child_list=gtk_container_get_children((GtkContainer *)gtk_widget_get_parent((GtkWidget *)widget));
    //retrieve selection from combobox
    gint selection=gtk_combo_box_get_active((GtkComboBox *)g_list_first(child_list)->data);
    if(selection==0){
      g_list_free(child_list);
      return;
    }
    else
    {
      int id;
      //read value from entry
      const gchar *speed_text=gtk_entry_get_text((GtkEntry *)g_list_next(g_list_next(g_list_next(child_list)))->data);
      if(isNumeric(speed_text) && atoi(speed_text)!=0)
      {
	int speed = atoi(speed_text);
	//check range
	if(speed>0 && speed<256)
	{
	  //convert selection to id
	  switch (selection){
	    case 1:
	      id=61;break;
	    case 2:
	      id=54;break;
	    case 3:
	      id=53;break;
	    case 4:
	      id=51;break;
	    case 5:
	      id=44;break;
	    case 6:
	      id=43;break;
	    case 7:
	      id=41;break;
	    case 8:
	      id=31;break;
	    case 9:
	      id=32;break;
	    case 10:
	      id=33;break;
	    case 11:
	      id=11;break;
	    case 12:
	      id=12;break;
	    case 13:
	      id=13;break;
	    case 14:
	      id=15;break;
	    case 15:
	      id=16;break;
	    case 16:
	      id=21;break;
	    case 17:
	      id=22;break;
	    case 18:
	      id=23;break;
	    case 19:
	      id=25;break;
	    case 20:
	      id=26;break;
	    default:
	      id=0;break;}
	  //send command
	  cout<<"Sending request to set speed "<<speed<<" to servomotor id ["<<id<<"]."<<endl;
	  int position=RobotVars->servo->SetSpeedPosition(id,speed);
	  cout<<"Servo current position: "<<position<<"."<<endl;
	  //update label
	  if(position>=606 && position<=2406)
	  {
	    double joint_angle=ConvertServoValueByID(id,position);
	    boost::format fmter("%3.1f");
	    fmter % joint_angle;
	    string position_label="Position read:\n" + boost::lexical_cast<string>(fmter.str())+"[deg]";
	    gtk_label_set_text((GtkLabel *)g_list_last(child_list)->data,position_label.c_str());
	    if(id!=0)
	    {
	      UpdateJointDataByID(id,joint_angle,RobotVars);
	      if(id>40 && id<55)
	      {
		UpdateArmsDirKinData(RobotVars);
	      }
	      else if(id<29)
		UpdateDetachedLegsDirKinData(RobotVars);
	      
	      RobotVars->update_labels=TRUE;
	    }
	  }
	  else
	  {
	    string position_label="Position read:\nERROR";
	    gtk_label_set_text((GtkLabel *)g_list_last(child_list)->data,position_label.c_str());
	  }
	}
	else
	{
	  cout<<"Invalid value for instruction."<<endl;
	}
      }
    }
    g_list_free(child_list);
  }
}

void on_button_vbuttonbox_update_robot_data_clicked(GtkWidget *widget, gpointer user_data)
{
  shared_vars_t*RobotVars=(shared_vars_t*)user_data;
  pthread_mutex_lock(&RobotVars->mutex_gtk);
  UpdateJointDataByID(1000, 0., RobotVars);
  UpdateArmsDirKinData(RobotVars);
  UpdateDetachedLegsDirKinData(RobotVars);
  RobotVars->update_labels=TRUE;
  pthread_mutex_unlock(&RobotVars->mutex_gtk);
}

void on_control_resolution_toggled(GtkWidget *widget, gpointer user_data)
{
  shared_vars_t*RobotVars=(shared_vars_t*)user_data;
  GList *child_list=gtk_container_get_children((GtkContainer *)gtk_widget_get_parent((GtkWidget *)widget));

  bool is_1mm=(bool)gtk_toggle_button_get_active((GtkToggleButton *)(g_list_first(child_list)->data));
  bool is_0_1mm=(bool)gtk_toggle_button_get_active((GtkToggleButton *)(g_list_next(g_list_first(child_list))->data));
  bool is_0_0_1mm=(bool)gtk_toggle_button_get_active((GtkToggleButton *)(g_list_next(g_list_next(g_list_first(child_list)))->data));
  bool is_0_0_0_1mm=(bool)gtk_toggle_button_get_active((GtkToggleButton *)(g_list_last(child_list)->data));
  
  //change respective values
  if(is_1mm && (bool)gtk_toggle_button_get_active((GtkToggleButton *)widget) && RobotVars->parameters.coordinate_resolution!=1.)
  {
    pthread_mutex_lock(&(RobotVars->mutex_gtk));
    RobotVars->parameters.coordinate_resolution=1.;
    pthread_mutex_unlock(&(RobotVars->mutex_gtk));
    gtk_toggle_button_set_active((GtkToggleButton *)(g_list_next(g_list_first(child_list))->data),FALSE);
    gtk_toggle_button_set_active((GtkToggleButton *)(g_list_next(g_list_next(g_list_first(child_list)))->data),FALSE);
    gtk_toggle_button_set_active((GtkToggleButton *)(g_list_last(child_list)->data),FALSE);
    g_list_free(child_list);
    cout<<"Setting new control resolution to: "<<RobotVars->parameters.coordinate_resolution<<"[mm]."<<endl;
    return;
  }
  else if(is_0_1mm && (bool)gtk_toggle_button_get_active((GtkToggleButton *)widget)&& RobotVars->parameters.coordinate_resolution!=0.1)
  {
    pthread_mutex_lock(&(RobotVars->mutex_gtk));
    RobotVars->parameters.coordinate_resolution=0.1;
    pthread_mutex_unlock(&(RobotVars->mutex_gtk));
    gtk_toggle_button_set_active((GtkToggleButton *)(g_list_first(child_list)->data),FALSE);
    gtk_toggle_button_set_active((GtkToggleButton *)(g_list_next(g_list_next(g_list_first(child_list)))->data),FALSE);
    gtk_toggle_button_set_active((GtkToggleButton *)(g_list_last(child_list)->data),FALSE);
    g_list_free(child_list);
    cout<<"Setting new control resolution to: "<<RobotVars->parameters.coordinate_resolution<<"[mm]."<<endl;
    return;
  }
  else if(is_0_0_1mm && (bool)gtk_toggle_button_get_active((GtkToggleButton *)widget)&& RobotVars->parameters.coordinate_resolution!=0.01)
  {
    pthread_mutex_lock(&(RobotVars->mutex_gtk));
    RobotVars->parameters.coordinate_resolution=0.01;
    pthread_mutex_unlock(&(RobotVars->mutex_gtk));
    gtk_toggle_button_set_active((GtkToggleButton *)(g_list_first(child_list)->data),FALSE);
    gtk_toggle_button_set_active((GtkToggleButton *)(g_list_next(g_list_first(child_list))->data),FALSE);
    gtk_toggle_button_set_active((GtkToggleButton *)(g_list_last(child_list)->data),FALSE);
    g_list_free(child_list);
    cout<<"Setting new control resolution to: "<<RobotVars->parameters.coordinate_resolution<<"[mm]."<<endl;
    return;
  }
  else if(is_0_0_0_1mm && (bool)gtk_toggle_button_get_active((GtkToggleButton *)widget)&& RobotVars->parameters.coordinate_resolution!=0.001)
  {
    pthread_mutex_lock(&(RobotVars->mutex_gtk));
    RobotVars->parameters.coordinate_resolution=0.001;
    pthread_mutex_unlock(&(RobotVars->mutex_gtk));
    gtk_toggle_button_set_active((GtkToggleButton *)(g_list_first(child_list)->data),FALSE);
    gtk_toggle_button_set_active((GtkToggleButton *)(g_list_next(g_list_first(child_list))->data),FALSE);
    gtk_toggle_button_set_active((GtkToggleButton *)(g_list_next(g_list_next(g_list_first(child_list)))->data),FALSE);
    g_list_free(child_list);
    cout<<"Setting new control resolution to: "<<RobotVars->parameters.coordinate_resolution<<"[mm]."<<endl;
    return;
  }
}

void on_workspace_scaling_toggled(GtkWidget *widget, gpointer user_data)
{
  shared_vars_t*RobotVars=(shared_vars_t*)user_data;
  GList *child_list=gtk_container_get_children((GtkContainer *)gtk_widget_get_parent((GtkWidget *)widget));

  bool x1=(bool)gtk_toggle_button_get_active((GtkToggleButton *)(g_list_first(child_list)->data));
  bool x2=(bool)gtk_toggle_button_get_active((GtkToggleButton *)(g_list_next(g_list_first(child_list))->data));
  bool x3=(bool)gtk_toggle_button_get_active((GtkToggleButton *)(g_list_next(g_list_next(g_list_first(child_list)))->data));
  bool x4=(bool)gtk_toggle_button_get_active((GtkToggleButton *)(g_list_last(child_list)->data));
  
  //change respective values
  if(x1 && (bool)gtk_toggle_button_get_active((GtkToggleButton *)widget) && RobotVars->parameters.pos_coord_scale!=1.)
  {
    pthread_mutex_lock(&(RobotVars->mutex_gtk));
    RobotVars->parameters.pos_coord_scale=1.;
    pthread_mutex_unlock(&(RobotVars->mutex_gtk));
    gtk_toggle_button_set_active((GtkToggleButton *)(g_list_next(g_list_first(child_list))->data),FALSE);
    gtk_toggle_button_set_active((GtkToggleButton *)(g_list_next(g_list_next(g_list_first(child_list)))->data),FALSE);
    gtk_toggle_button_set_active((GtkToggleButton *)(g_list_last(child_list)->data),FALSE);
    g_list_free(child_list);
    cout<<"Setting workspace scaling to: x"<<RobotVars->parameters.pos_coord_scale<<"."<<endl;
    return;
  }
  else if(x2 && (bool)gtk_toggle_button_get_active((GtkToggleButton *)widget)&& RobotVars->parameters.pos_coord_scale!=2.)
  {
    pthread_mutex_lock(&(RobotVars->mutex_gtk));
    RobotVars->parameters.pos_coord_scale=2.;
    pthread_mutex_unlock(&(RobotVars->mutex_gtk));
    gtk_toggle_button_set_active((GtkToggleButton *)(g_list_first(child_list)->data),FALSE);
    gtk_toggle_button_set_active((GtkToggleButton *)(g_list_next(g_list_next(g_list_first(child_list)))->data),FALSE);
    gtk_toggle_button_set_active((GtkToggleButton *)(g_list_last(child_list)->data),FALSE);
    g_list_free(child_list);
    cout<<"Setting workspace scaling to: x"<<RobotVars->parameters.pos_coord_scale<<"."<<endl;
    return;
  }
  else if(x3 && (bool)gtk_toggle_button_get_active((GtkToggleButton *)widget)&& RobotVars->parameters.pos_coord_scale!=3.)
  {
    pthread_mutex_lock(&(RobotVars->mutex_gtk));
    RobotVars->parameters.pos_coord_scale=3.;
    pthread_mutex_unlock(&(RobotVars->mutex_gtk));
    gtk_toggle_button_set_active((GtkToggleButton *)(g_list_first(child_list)->data),FALSE);
    gtk_toggle_button_set_active((GtkToggleButton *)(g_list_next(g_list_first(child_list))->data),FALSE);
    gtk_toggle_button_set_active((GtkToggleButton *)(g_list_last(child_list)->data),FALSE);
    g_list_free(child_list);
    cout<<"Setting workspace scaling to: x"<<RobotVars->parameters.pos_coord_scale<<"."<<endl;
    return;
  }
  else if(x4 && (bool)gtk_toggle_button_get_active((GtkToggleButton *)widget)&& RobotVars->parameters.pos_coord_scale!=4.)
  {
    pthread_mutex_lock(&(RobotVars->mutex_gtk));
    RobotVars->parameters.pos_coord_scale=4.;
    pthread_mutex_unlock(&(RobotVars->mutex_gtk));
    gtk_toggle_button_set_active((GtkToggleButton *)(g_list_first(child_list)->data),FALSE);
    gtk_toggle_button_set_active((GtkToggleButton *)(g_list_next(g_list_first(child_list))->data),FALSE);
    gtk_toggle_button_set_active((GtkToggleButton *)(g_list_next(g_list_next(g_list_first(child_list)))->data),FALSE);
    g_list_free(child_list);
    cout<<"Setting workspace scaling to: x"<<RobotVars->parameters.pos_coord_scale<<"."<<endl;
    return;
  }
}

void on_button_test_invkin_clicked(GtkWidget *widget, gpointer user_data)
{
  shared_vars_t*RobotVars=(shared_vars_t*)user_data;
  if(RobotVars->servo->IsActive())
  {
    //acess child list
    GList *child_list = gtk_container_get_children((GtkContainer *)gtk_widget_get_parent((GtkWidget *)widget));
    //check selection
    gint selection=gtk_combo_box_get_active((GtkComboBox *)g_list_first(child_list)->data);
    if(selection==0){
      g_list_free(child_list);
      return;
    }
    else
    {
      //read values from spinbuttons
      double X_pos=gtk_spin_button_get_value((GtkSpinButton *)g_list_next(g_list_next(g_list_first(child_list)))->data);
      double Y_pos=gtk_spin_button_get_value((GtkSpinButton *)g_list_next(g_list_next(g_list_next(g_list_next(g_list_next(g_list_next(g_list_first(child_list)))))))->data);
      double Z_pos=gtk_spin_button_get_value((GtkSpinButton *)g_list_next(g_list_next(g_list_next(g_list_next(g_list_next(g_list_next(g_list_next(g_list_next(g_list_next(g_list_next(g_list_first(child_list)))))))))))->data);
      
      cout<<"POINTS READ: "<<X_pos<<"|"<<Y_pos<<"|"<<Z_pos<<" AT SELECTION: "<<selection<<endl;
      
      //parse the points to movement functions
      if(selection==1) //arms, in the shoulder referencial
      {
	MoveArmToCartesianPosition(X_pos, Y_pos, Z_pos, LEFT, RobotVars);
	MoveArmToCartesianPosition(X_pos, Y_pos, Z_pos, RIGHT, RobotVars);
	UpdateArmsDirKinData(RobotVars);
      }
      else if(selection==2) //arms, in the main referencial
      {
	//transform the points to the shoulder referencial
	//TODO: I STILL NEED TO CREATE THESE FUNCTIONS!!!!!
	
	MoveArmToCartesianPosition(X_pos, Y_pos, Z_pos, LEFT, RobotVars);
	MoveArmToCartesianPosition(X_pos, Y_pos, Z_pos, RIGHT, RobotVars);
	UpdateArmsDirKinData(RobotVars);
      }
      else if(selection==3) //detached legs, ground referencial
      {
	MoveDetachedLegToCartesianPosition(X_pos, Y_pos, Z_pos, 0., LEFT, RobotVars);
	MoveDetachedLegToCartesianPosition(X_pos, Y_pos, Z_pos, 0., RIGHT, RobotVars);
	UpdateDetachedLegsDirKinData(RobotVars);
      }
      else //any other thing
      {
	
      }
      
      g_list_free(child_list);
    }
    //end of else
  }
}

void on_button_calibration_clicked(GtkWidget *widget, gpointer user_data)
{
  shared_vars_t*RobotVars=(shared_vars_t*)user_data;
  GtkWidget *dialog;
  dialog = gtk_message_dialog_new(GTK_WINDOW(gtk_widget_get_toplevel ((GtkWidget *)widget)),
				  GTK_DIALOG_DESTROY_WITH_PARENT,
				  GTK_MESSAGE_WARNING,
				  GTK_BUTTONS_OK_CANCEL,
				  "Place the joystick pen tip pointer in the device inkwell and press OK to calibrate.");
  gtk_window_set_title(GTK_WINDOW(dialog), "PHANToM OMNI CALIBRATION CHECK");

  gint result = gtk_dialog_run(GTK_DIALOG(dialog));
  switch (result)
  {
    case GTK_RESPONSE_OK:
    {
      pthread_mutex_lock(&RobotVars->mutex_gtk);
      RobotVars->phantom_data.need_update=TRUE;
      pthread_mutex_unlock(&RobotVars->mutex_gtk);
      break;
    }
    default:
    {
      pthread_mutex_lock(&RobotVars->mutex_gtk);
      RobotVars->phantom_data.need_update=FALSE;
      pthread_mutex_unlock(&RobotVars->mutex_gtk);
      break;
    }
  }
  gtk_widget_destroy(dialog);
}

void on_notebook_change_current_page(GtkNotebook *notebook, GtkNotebookPage *page, guint page_num, gpointer user_data)
{
  shared_vars_t*RobotVars=(shared_vars_t*)user_data;  
  pthread_mutex_lock(&RobotVars->mutex_gtk);
  RobotVars->parameters.selected_notebook_page = page_num;
  pthread_mutex_unlock(&RobotVars->mutex_gtk);
}

void on_back_facing_menu_item_activate(GtkMenuItem *menuitem, gpointer user_data)
{
//   cout<<"Setting control positioning to back-facing."<<endl;
//   shared_vars_t*RobotVars=(shared_vars_t*)user_data;
//   pthread_mutex_lock(&RobotVars->mutex_gtk);
//   RobotVars->parameters.cntrl_pos_back_front = (double)BACK_FACING;
//   pthread_mutex_unlock(&RobotVars->mutex_gtk);
}

void on_front_facing_menu_item_activate(GtkMenuItem *menuitem, gpointer user_data)
{
//   cout<<"Setting control positioning to front-facing."<<endl;
  shared_vars_t*RobotVars=(shared_vars_t*)user_data;
  pthread_mutex_lock(&RobotVars->mutex_gtk);
  RobotVars->parameters.cntrl_pos_back_front = (double)FRONT_FACING;
  pthread_mutex_unlock(&RobotVars->mutex_gtk);
}

void on_demo_checkboxes_toggled(GtkToggleButton *togglebutton, gpointer user_data)
{
  shared_vars_t*RobotVars=(shared_vars_t*)user_data;
  static bool demo1;
  static bool demo2;
  static bool demo3;
  static bool demo4;
  
  demo1 = (bool)gtk_toggle_button_get_active((GtkToggleButton *)RobotVars->updt_labels.demo1_checkbox);
  demo2 = (bool)gtk_toggle_button_get_active((GtkToggleButton *)RobotVars->updt_labels.demo2_checkbox);
  demo3 = (bool)gtk_toggle_button_get_active((GtkToggleButton *)RobotVars->updt_labels.demo3_checkbox);
  demo4 = (bool)gtk_toggle_button_get_active((GtkToggleButton *)RobotVars->updt_labels.demo4_checkbox);
  
  if((GtkToggleButton *)RobotVars->updt_labels.demo1_checkbox == togglebutton && demo1)
  {
    gtk_toggle_button_set_active((GtkToggleButton *)RobotVars->updt_labels.demo2_checkbox,FALSE);
    gtk_toggle_button_set_active((GtkToggleButton *)RobotVars->updt_labels.demo3_checkbox,FALSE);
    gtk_toggle_button_set_active((GtkToggleButton *)RobotVars->updt_labels.demo4_checkbox,FALSE);
    RobotVars->haptics_data.chosen_demo = 1;
    cout<<"Chosen Haptic Demonstration "<<RobotVars->haptics_data.chosen_demo<<"."<<endl;
    //enable/disable hbox
    gtk_widget_set_visible((GtkWidget *)gtk_widget_get_parent((GtkWidget *)RobotVars->updt_labels.workspace_zone_label),
			   TRUE);
    gtk_widget_set_visible((GtkWidget *)gtk_widget_get_parent((GtkWidget *)RobotVars->updt_labels.drawing_demo_label),
			   FALSE);
    gtk_widget_set_visible((GtkWidget *)gtk_widget_get_parent((GtkWidget *)RobotVars->updt_labels.leg_balancing_demo_label),
			   FALSE);
    gtk_widget_set_visible((GtkWidget *)gtk_widget_get_parent((GtkWidget *)RobotVars->updt_labels.user_path_label),
			   FALSE);
    
  }
  else if((GtkToggleButton *)RobotVars->updt_labels.demo2_checkbox == togglebutton && demo2)
  {
    gtk_toggle_button_set_active((GtkToggleButton *)RobotVars->updt_labels.demo1_checkbox,FALSE);
    gtk_toggle_button_set_active((GtkToggleButton *)RobotVars->updt_labels.demo3_checkbox,FALSE);
    gtk_toggle_button_set_active((GtkToggleButton *)RobotVars->updt_labels.demo4_checkbox,FALSE);
    RobotVars->haptics_data.chosen_demo = 2;
    cout<<"Chosen Haptic Demonstration "<<RobotVars->haptics_data.chosen_demo<<"."<<endl;
    //enable/disable hbox
    gtk_widget_set_visible((GtkWidget *)gtk_widget_get_parent((GtkWidget *)RobotVars->updt_labels.workspace_zone_label),
			   FALSE);
    gtk_widget_set_visible((GtkWidget *)gtk_widget_get_parent((GtkWidget *)RobotVars->updt_labels.drawing_demo_label),
			   TRUE);
    gtk_widget_set_visible((GtkWidget *)gtk_widget_get_parent((GtkWidget *)RobotVars->updt_labels.leg_balancing_demo_label),
			   FALSE);
    gtk_widget_set_visible((GtkWidget *)gtk_widget_get_parent((GtkWidget *)RobotVars->updt_labels.user_path_label),
			   FALSE);
  }
  else if((GtkToggleButton *)RobotVars->updt_labels.demo3_checkbox == togglebutton && demo3)
  {
    gtk_toggle_button_set_active((GtkToggleButton *)RobotVars->updt_labels.demo2_checkbox,FALSE);
    gtk_toggle_button_set_active((GtkToggleButton *)RobotVars->updt_labels.demo1_checkbox,FALSE);
    gtk_toggle_button_set_active((GtkToggleButton *)RobotVars->updt_labels.demo4_checkbox,FALSE);
    RobotVars->haptics_data.chosen_demo = 3;
    cout<<"Chosen Haptic Demonstration "<<RobotVars->haptics_data.chosen_demo<<"."<<endl;
    //enable/disable hbox
    gtk_widget_set_visible((GtkWidget *)gtk_widget_get_parent((GtkWidget *)RobotVars->updt_labels.workspace_zone_label),
			   FALSE);
    gtk_widget_set_visible((GtkWidget *)gtk_widget_get_parent((GtkWidget *)RobotVars->updt_labels.drawing_demo_label),
			   FALSE);
    gtk_widget_set_visible((GtkWidget *)gtk_widget_get_parent((GtkWidget *)RobotVars->updt_labels.leg_balancing_demo_label),
			   FALSE);
    gtk_widget_set_visible((GtkWidget *)gtk_widget_get_parent((GtkWidget *)RobotVars->updt_labels.user_path_label),
			   TRUE);
  }
  else if((GtkToggleButton *)RobotVars->updt_labels.demo4_checkbox == togglebutton && demo4)
  {
    gtk_toggle_button_set_active((GtkToggleButton *)RobotVars->updt_labels.demo2_checkbox,FALSE);
    gtk_toggle_button_set_active((GtkToggleButton *)RobotVars->updt_labels.demo1_checkbox,FALSE);
    gtk_toggle_button_set_active((GtkToggleButton *)RobotVars->updt_labels.demo3_checkbox,FALSE);
    RobotVars->haptics_data.chosen_demo = 4;
    cout<<"Chosen Haptic Demonstration "<<RobotVars->haptics_data.chosen_demo<<"."<<endl;
    //enable/disable hbox
    gtk_widget_set_visible((GtkWidget *)gtk_widget_get_parent((GtkWidget *)RobotVars->updt_labels.workspace_zone_label),
			   FALSE);
    gtk_widget_set_visible((GtkWidget *)gtk_widget_get_parent((GtkWidget *)RobotVars->updt_labels.drawing_demo_label),
			   FALSE);
    gtk_widget_set_visible((GtkWidget *)gtk_widget_get_parent((GtkWidget *)RobotVars->updt_labels.leg_balancing_demo_label),
			   TRUE);
    gtk_widget_set_visible((GtkWidget *)gtk_widget_get_parent((GtkWidget *)RobotVars->updt_labels.user_path_label),
			   FALSE);
  }
  else if((GtkToggleButton *)RobotVars->updt_labels.demo1_checkbox == togglebutton && !demo1 && !demo2 && !demo3 && !demo4)
  {
    RobotVars->haptics_data.chosen_demo = 0;
    cout<<"No Haptic Demonstration Chosen."<<endl;
    //enable/disable hbox
    gtk_widget_set_visible((GtkWidget *)gtk_widget_get_parent((GtkWidget *)RobotVars->updt_labels.workspace_zone_label),
			   FALSE);
    gtk_widget_set_visible((GtkWidget *)gtk_widget_get_parent((GtkWidget *)RobotVars->updt_labels.drawing_demo_label),
			   FALSE);
    gtk_widget_set_visible((GtkWidget *)gtk_widget_get_parent((GtkWidget *)RobotVars->updt_labels.leg_balancing_demo_label),
			   FALSE);
    gtk_widget_set_visible((GtkWidget *)gtk_widget_get_parent((GtkWidget *)RobotVars->updt_labels.user_path_label),
			   FALSE);
  }
  else if((GtkToggleButton *)RobotVars->updt_labels.demo2_checkbox == togglebutton && !demo1 && !demo2 && !demo3 && !demo4)
  {
    RobotVars->haptics_data.chosen_demo = 0;
    cout<<"No Haptic Demonstration Chosen."<<endl;
    //enable/disable hbox
    gtk_widget_set_visible((GtkWidget *)gtk_widget_get_parent((GtkWidget *)RobotVars->updt_labels.workspace_zone_label),
			   FALSE);
    gtk_widget_set_visible((GtkWidget *)gtk_widget_get_parent((GtkWidget *)RobotVars->updt_labels.drawing_demo_label),
			   FALSE);
    gtk_widget_set_visible((GtkWidget *)gtk_widget_get_parent((GtkWidget *)RobotVars->updt_labels.leg_balancing_demo_label),
			   FALSE);
    gtk_widget_set_visible((GtkWidget *)gtk_widget_get_parent((GtkWidget *)RobotVars->updt_labels.user_path_label),
			   FALSE);
  }
  else if((GtkToggleButton *)RobotVars->updt_labels.demo3_checkbox == togglebutton && !demo1 && !demo2 && !demo3 && !demo4)
  {
    RobotVars->haptics_data.chosen_demo = 0;
    cout<<"No Haptic Demonstration Chosen."<<endl;
    //enable/disable hbox
    gtk_widget_set_visible((GtkWidget *)gtk_widget_get_parent((GtkWidget *)RobotVars->updt_labels.workspace_zone_label),
			   FALSE);
    gtk_widget_set_visible((GtkWidget *)gtk_widget_get_parent((GtkWidget *)RobotVars->updt_labels.drawing_demo_label),
			   FALSE);
    gtk_widget_set_visible((GtkWidget *)gtk_widget_get_parent((GtkWidget *)RobotVars->updt_labels.leg_balancing_demo_label),
			   FALSE);
    gtk_widget_set_visible((GtkWidget *)gtk_widget_get_parent((GtkWidget *)RobotVars->updt_labels.user_path_label),
			   FALSE);
  }
  else if((GtkToggleButton *)RobotVars->updt_labels.demo4_checkbox == togglebutton && !demo1 && !demo2 && !demo3 && !demo4)
  {
    RobotVars->haptics_data.chosen_demo = 0;
    cout<<"No Haptic Demonstration Chosen."<<endl;
    //enable/disable hbox
    gtk_widget_set_visible((GtkWidget *)gtk_widget_get_parent((GtkWidget *)RobotVars->updt_labels.workspace_zone_label),
			   FALSE);
    gtk_widget_set_visible((GtkWidget *)gtk_widget_get_parent((GtkWidget *)RobotVars->updt_labels.drawing_demo_label),
			   FALSE);
    gtk_widget_set_visible((GtkWidget *)gtk_widget_get_parent((GtkWidget *)RobotVars->updt_labels.leg_balancing_demo_label),
			   FALSE);
    gtk_widget_set_visible((GtkWidget *)gtk_widget_get_parent((GtkWidget *)RobotVars->updt_labels.user_path_label),
			   FALSE);
  }
}

void select_inv_kin_combobox_changed_event(GtkWidget *widget, gpointer user_data)
{
  shared_vars_t*RobotVars=(shared_vars_t*)user_data;
  gint selection=gtk_combo_box_get_active((GtkComboBox *)widget);
  //update inv kin spinbuttons information
  if(selection==1) //arms, shoulder ref
  {
    GList *child_list=gtk_container_get_children((GtkContainer *)gtk_widget_get_parent((GtkWidget *)widget));
    
    gtk_spin_button_set_value ((GtkSpinButton *)g_list_nth_data(child_list,2), RobotVars->robot_kin_data.X_arm_end_left);
    
    gtk_spin_button_set_value ((GtkSpinButton *)g_list_nth_data(child_list,6), RobotVars->robot_kin_data.Y_arm_end_left);
    
    gtk_spin_button_set_value ((GtkSpinButton *)g_list_nth_data(child_list,10), RobotVars->robot_kin_data.Z_arm_end_left);
    
    g_list_free(child_list);
  }
  else if(selection==2) //arms, main ref
  {
    //coordinates must be transformed to main ref
    
  }
  else if(selection==3) //detached legs
  {
    GList *child_list=gtk_container_get_children((GtkContainer *)gtk_widget_get_parent((GtkWidget *)widget));
    
    gtk_spin_button_set_value ((GtkSpinButton *)g_list_nth_data(child_list,2), RobotVars->robot_kin_data.detached_leg_pos_left[0]);
    gtk_spin_button_set_value ((GtkSpinButton *)g_list_nth_data(child_list,6), RobotVars->robot_kin_data.detached_leg_pos_left[1]);
    gtk_spin_button_set_value ((GtkSpinButton *)g_list_nth_data(child_list,10), RobotVars->robot_kin_data.detached_leg_pos_left[2]);
    
    g_list_free(child_list);
  }
  else
  {
    
  }
}

void on_user_path_demo_point_store_button_clicked(GtkWidget *widget, gpointer user_data)
{
  shared_vars_t*RobotVars=(shared_vars_t*)user_data;
  if(RobotVars->haptic_loop_start)
  {
    RobotVars->haptics_data.demo_user_path_point_storing = ! RobotVars->haptics_data.demo_user_path_point_storing;
  }
  else
  {
    RobotVars->haptics_data.demo_user_path_point_storing = FALSE;
  }
  //change button text
  if(RobotVars->haptics_data.demo_user_path_point_storing)
  {
    gtk_button_set_label((GtkButton *)widget,"Stop Point\nAcquisition");
  }
  else
  {
    gtk_button_set_label((GtkButton *)widget,"Store Points");
  }
}

void user_path_demo_run_checkbox_toggled(GtkToggleButton *togglebutton, gpointer user_data)
{
  static bool run_once;
  static bool run_loop;
  
  shared_vars_t*RobotVars=(shared_vars_t*)user_data;
  GList *child_list=gtk_container_get_children((GtkContainer *)gtk_widget_get_parent((GtkWidget *)togglebutton));
  
  run_once = (bool)gtk_toggle_button_get_active((GtkToggleButton *)g_list_first(child_list)->data);
  run_loop = (bool)gtk_toggle_button_get_active((GtkToggleButton *)g_list_last(child_list)->data);
  
  if((GtkToggleButton *)g_list_first(child_list)->data == togglebutton && run_once)
  {
    gtk_toggle_button_set_active((GtkToggleButton *)g_list_last(child_list)->data,FALSE);
  }
  else if((GtkToggleButton *)g_list_last(child_list)->data == togglebutton && run_loop)
  {
    gtk_toggle_button_set_active((GtkToggleButton *)g_list_first(child_list)->data,FALSE);
  }
  else if((GtkToggleButton *)g_list_first(child_list)->data == togglebutton && !run_once && !run_loop)
  {
    gtk_toggle_button_set_active((GtkToggleButton *)g_list_last(child_list)->data,TRUE);
  }
  else if((GtkToggleButton *)g_list_last(child_list)->data == togglebutton && !run_once && !run_loop)
  {
    gtk_toggle_button_set_active((GtkToggleButton *)g_list_first(child_list)->data,TRUE);
  }
  
  RobotVars->haptics_data.demo_user_path_is_run_once = run_once;
  
  g_list_free(child_list);
}

void on_user_path_demo_run_button_clicked(GtkWidget *widget, gpointer user_data)
{
  shared_vars_t*RobotVars=(shared_vars_t*)user_data;
  
  if(RobotVars->haptic_loop_start && !RobotVars->haptics_data.demo_user_path_run_start)
  {
    //read from file
    DemoUserPath_ReadPoints(RobotVars);
  }
  else
  {
    RobotVars->haptics_data.demo_user_path_run_start = FALSE;
  }
  //change button text
  if(RobotVars->haptics_data.demo_user_path_run_start)
  {
    gtk_button_set_label((GtkButton *)widget,"STOP Run");
  }
  else
  {
    gtk_button_set_label((GtkButton *)widget,"Run Points");
  }
}

void on_user_path_demo_clear_button_clicked(GtkWidget *widget, gpointer user_data)
{
  shared_vars_t*RobotVars=(shared_vars_t*)user_data;
  if(!RobotVars->haptic_loop_start && !RobotVars->haptics_data.demo_user_path_run_start)
  {
    cout<<"-> Deleting points file...";
    std::string file_remove_command = "rm " + ros::package::getPath("phua_haptic") + "/pnts/" + POINTS_FILE_NAME_STRING;
    int ret = system(file_remove_command.c_str());
    if(ret < 0)
    {
      cout<<endl;
      perror("File remove error!");
    }
    else
      cout<<"DONE!"<<endl;
    
    cout<<"-> Clearing points from memory...";
    RobotVars->haptics_data.user_path_X_pnts.clear();
    RobotVars->haptics_data.user_path_Y_pnts.clear();
    RobotVars->haptics_data.user_path_Z_pnts.clear();
    cout<<"DONE!"<<endl;
  }
}

gboolean update_watcher(gpointer data_struct)
{
  shared_vars_t*RobotVars=(shared_vars_t*)data_struct;
  
  if(RobotVars->parameters.exit_status==TRUE)
  {
    return FALSE;
  }
  else
  {
    // update labels
    if(RobotVars->update_labels)
    {
      UpdateLabels(RobotVars);
      pthread_mutex_lock(&RobotVars->mutex_gtk);
      RobotVars->update_labels=FALSE;
      pthread_mutex_unlock(&RobotVars->mutex_gtk);
    }
    // check for speed control type checkboxes
    GtkWidget *toplevel = gtk_widget_get_toplevel ((GtkWidget *)RobotVars->updt_labels.head_tilt_label);
    bool *man=(bool*)g_object_get_data((GObject *)toplevel,"manual_speed_control");
    bool *autom=(bool*)g_object_get_data((GObject *)toplevel,"automatic_speed_control");
    if(*man!=RobotVars->parameters.manual_speed_control)
    {
      RobotVars->parameters.manual_speed_control=*man;
    }
    if(*autom!=RobotVars->parameters.automatic_speed_control)
    {
      RobotVars->parameters.automatic_speed_control=*autom;
    }
    if(!RobotVars->parameters.manual_speed_control)
    {
      //update auto_speed progress bars
      gtk_progress_bar_set_fraction((GtkProgressBar *)RobotVars->updt_labels.auto_speed_arm_theta1_bar,(double)RobotVars->parameters.arm_auto_speed[0]/255.);
      gtk_progress_bar_set_fraction((GtkProgressBar *)RobotVars->updt_labels.auto_speed_arm_theta2_bar,(double)RobotVars->parameters.arm_auto_speed[1]/255.);
      gtk_progress_bar_set_fraction((GtkProgressBar *)RobotVars->updt_labels.auto_speed_arm_theta3_bar,(double)RobotVars->parameters.arm_auto_speed[2]/255.);
    }
    //check for joystick buttons ON
    if(RobotVars->phantom_data.phantom_on)
    {
      //BUTTON 1
      if(RobotVars->phantom_data.m_button1Clicked)
      {
	//START/STOP LOOP
	pthread_mutex_lock(&RobotVars->mutex_gtk);
	RobotVars->phantom_data.m_button1Clicked=FALSE;
	pthread_mutex_unlock(&RobotVars->mutex_gtk);
	on_button_start_loop_clicked((GtkWidget *)(RobotVars->updt_labels.start_loop_button),RobotVars);
      }
      //BUTTON 2
      if(RobotVars->phantom_data.m_button2Clicked)
      {
	if(!RobotVars->haptic_loop_start)
	{
	  //CHANGE KIN MODEL CHECKBOX SELECT IF NOT IN LOOP
	  pthread_mutex_lock(&RobotVars->mutex_gtk);
	  RobotVars->phantom_data.m_button2Clicked=FALSE;
	  pthread_mutex_unlock(&RobotVars->mutex_gtk);
	  gint sel=gtk_combo_box_get_active((GtkComboBox *)(RobotVars->updt_labels.kin_model_combobox));
	  if(sel==5)
	    gtk_combo_box_set_active((GtkComboBox *)(RobotVars->updt_labels.kin_model_combobox),1);
	  else
	    gtk_combo_box_set_active((GtkComboBox *)(RobotVars->updt_labels.kin_model_combobox),sel+1);
	}
	else if(RobotVars->haptic_loop_start && RobotVars->haptics_data.demo_user_path_point_storing)
	{
	  pthread_mutex_lock(&RobotVars->mutex_gtk);
	  RobotVars->phantom_data.m_button2Clicked = FALSE;
	  pthread_mutex_unlock(&RobotVars->mutex_gtk);
	  cout<<"-> Storing point for User Path Demonstration...";
	  DemoUserPath_WritePoints(RobotVars);
	  cout<<"DONE!"<<endl;
	}
      }
      //update force feedback progress bars
      UpdateForceBars(RobotVars);
    }
    //status bar
    UpdateStatusBar(RobotVars);
    //update user path run button status
    if(RobotVars->haptics_data.demo_user_path_run_start)
    {
      gtk_button_set_label((GtkButton *)RobotVars->updt_labels.user_path_run_button,"STOP Run");
    }
    else
    {
      gtk_button_set_label((GtkButton *)RobotVars->updt_labels.user_path_run_button,"Run Points");
    }
    return TRUE;
  }
}

/*~~~~~~~~~~~~~~~~~~~~~~~~~ 
|| ____MAIN_FUNCTION____  ||
 ~~~~~~~~~~~~~~~~~~~~~~~~~*/
void *interface_init(void *dummy)
{
  shared_vars_t*RobotVars=(shared_vars_t*)dummy;
  
  //initialize gtk variables | input NULL
  gtk_init(NULL,NULL);
  
  //error handling variable | use g_error_free(GError *error); to clear
  GError *err = NULL;
  
  // color allocation
  GdkColor red = {0, 0xffff, 0x0000, 0x0000};
  GdkColor green = {0, 0x0000, 0xffff, 0x0000};
  GdkColor blue = {0, 0x0000, 0x0000, 0xffff};
  
  char *markup;
  
  /*~~~~~~~~~~~~~~~~~~~~~~~~~ 
  ||        main window      ||
    ~~~~~~~~~~~~~~~~~~~~~~~~~*/
  GtkWidget *main_window=gtk_window_new(GTK_WINDOW_TOPLEVEL);
  //set main window postion to center of screen
  gtk_window_set_position(GTK_WINDOW(main_window), GTK_WIN_POS_CENTER);
  //set window to not resize
  gtk_window_set_resizable ((GtkWindow *)main_window,FALSE);
  //window title
  gtk_window_set_title(GTK_WINDOW(main_window), "PHUA Haptic Interface v2.0");
  // application icon
  string cmplt_icon_path = ros::package::getPath("phua_haptic") + "/image/robot.png";
  gtk_window_set_icon_from_file(GTK_WINDOW(main_window),cmplt_icon_path.c_str(),&err);
  if(err!=NULL)
  {
    cout<<"Error loading application icon."<<endl<<err->message<<endl;
    g_error_free(err);
  }
  //create acceleration group [SHORTCUTS]
  GtkAccelGroup *accel_group=gtk_accel_group_new();
  gtk_window_add_accel_group(GTK_WINDOW(main_window), accel_group);
  
  /*~~~~~~~~~~~~~~~~~~~~~~~~~ 
  ||         main vbox       ||
    ~~~~~~~~~~~~~~~~~~~~~~~~~*/
  //main vertical box
  GtkWidget *main_vbox=gtk_vbox_new(FALSE, 5);

  /*~~~~~~~~~~~~~~~~~~~~~~~~~ 
  ||        statusbar        ||
    ~~~~~~~~~~~~~~~~~~~~~~~~~*/
  GtkWidget *statusbar=gtk_statusbar_new();
  // throw statusbar messages
  if(RobotVars->servo->IsActive() && !RobotVars->phantom_data.phantom_on)
    gtk_statusbar_push(GTK_STATUSBAR(statusbar),gtk_statusbar_get_context_id(GTK_STATUSBAR(statusbar),"statusbar_info"),":: Hitec servomotor COMM active :: PHANToM OMNI inactive ::");
  else if(RobotVars->servo->IsActive() && RobotVars->phantom_data.phantom_on)
  {
    gtk_statusbar_push(GTK_STATUSBAR(statusbar),gtk_statusbar_get_context_id(GTK_STATUSBAR(statusbar),"statusbar_info"),":: Hitec servomotor COMM active :: PHANToM OMNI active ::");
  }
  else if(!RobotVars->servo->IsActive() && RobotVars->phantom_data.phantom_on)
  {
    gtk_statusbar_push(GTK_STATUSBAR(statusbar),gtk_statusbar_get_context_id(GTK_STATUSBAR(statusbar),"statusbar_info"),":: Hitec servomotor COMM inactive :: PHANToM OMNI active ::");
  }
  else
    gtk_statusbar_push(GTK_STATUSBAR(statusbar),gtk_statusbar_get_context_id(GTK_STATUSBAR(statusbar),"statusbar_info"),":: Communications inactive ::");
  
  RobotVars->updt_labels.status_bar = statusbar;
  
  /*~~~~~~~~~~~~~~~~~~~~~~~~~ 
  ||        menu bar         ||
    ~~~~~~~~~~~~~~~~~~~~~~~~~*/
  GtkWidget *menu_bar=gtk_menu_bar_new();
  GtkWidget *menu_options=gtk_menu_new();
  //menu items
  GtkWidget *menu_item_options=gtk_menu_item_new_with_mnemonic("Options");
  GtkWidget *menu_item_quit=gtk_image_menu_item_new_from_stock(GTK_STOCK_QUIT, accel_group);
  GtkWidget *menu_item_about=gtk_menu_item_new_with_mnemonic("About");
  // back/front-facing control type menu items
  GtkWidget *menu_item_back_front=gtk_menu_item_new_with_mnemonic("Control Positioning");
  GtkWidget *menu_back_front=gtk_menu_new();
  GtkWidget *menu_item_back_facing=gtk_menu_item_new_with_mnemonic("Control Robot Back-Facing");
  GtkWidget *menu_item_front_facing=gtk_menu_item_new_with_mnemonic("Control Robot Front-Facing");
  //create menu structure
  gtk_menu_item_set_submenu(GTK_MENU_ITEM(menu_item_options), menu_options);
  gtk_menu_shell_append(GTK_MENU_SHELL(menu_options), menu_item_back_front);
  gtk_menu_item_set_submenu((GtkMenuItem *)menu_item_back_front,menu_back_front);
  gtk_menu_shell_append(GTK_MENU_SHELL(menu_back_front), menu_item_back_facing);
  gtk_menu_shell_append(GTK_MENU_SHELL(menu_back_front), menu_item_front_facing);
  gtk_menu_shell_append(GTK_MENU_SHELL(menu_options), menu_item_quit);
  gtk_menu_shell_append(GTK_MENU_SHELL(menu_bar), menu_item_options);
  gtk_menu_shell_append(GTK_MENU_SHELL(menu_bar), menu_item_about);  
  
  gtk_menu_item_set_use_underline((GtkMenuItem *)menu_item_options, TRUE);
  gtk_menu_item_set_use_underline((GtkMenuItem *)menu_item_about, TRUE);
  gtk_menu_item_set_use_underline((GtkMenuItem *)menu_item_back_front, TRUE);
  gtk_menu_item_set_use_underline((GtkMenuItem *)menu_item_back_facing, TRUE);
  gtk_menu_item_set_use_underline((GtkMenuItem *)menu_item_front_facing, TRUE);
  
  
  /*~~~~~~~~~~~~~~~~~~~~~~~~~ 
  ||   bottom buttons hbox   ||
    ~~~~~~~~~~~~~~~~~~~~~~~~~*/
  GtkWidget *bottom_buttons_hbox=gtk_hbox_new(FALSE, 3);
    //start loop button
  GtkWidget *button_start_loop=gtk_button_new_with_label("\nSTART HAPTICS/CONTROL LOOP\n");
  gtk_button_set_focus_on_click((GtkButton *)button_start_loop,TRUE);
  RobotVars->updt_labels.start_loop_button=button_start_loop;
  // update robot data button
  GtkWidget *button_vbuttonbox_update_robot_data=gtk_button_new_with_label("UPDATE HUMANOID ROBOT JOINT DATA");
  // set robot home position button
  GtkWidget *button_set_robot_home_pos=gtk_button_new_with_label("SET HUMANOID ROBOT @ HOME POSITION");
  gtk_button_set_focus_on_click((GtkButton *)button_set_robot_home_pos,TRUE);
  // button quit
  GtkWidget *button_quit = gtk_button_new_from_stock(GTK_STOCK_QUIT);
  
  /*~~~~~~~~~~~~~~~~~~~~~~~~~ 
  ||        notebook         ||
    ~~~~~~~~~~~~~~~~~~~~~~~~~*/
  GtkWidget *notebook=gtk_notebook_new();
    //label notebook main page
  GtkWidget *label_notebook_page_1=gtk_label_new("Control Humanoid Robot");
  //main vertical box
  GtkWidget *page1_main_vertbox=gtk_vbox_new(FALSE, 3);
  //main horizontal box
  GtkWidget *page1_main_hbox=gtk_hbox_new(FALSE, 5);
  gtk_box_pack_start(GTK_BOX(page1_main_vertbox), page1_main_hbox,TRUE, TRUE, 0);
  //create notebook main page
  gtk_notebook_append_page ((GtkNotebook *)notebook,page1_main_vertbox,label_notebook_page_1);
  
  //horizontal box for notebook miscellaneous
  GtkWidget *page2_main_hbox=gtk_hbox_new(FALSE, 5);
  //label page 2
  GtkWidget *label_notebook_page_2=gtk_label_new("Joystick Information and Miscellaneous Commands");
  //create notebook page 2
  gtk_notebook_append_page((GtkNotebook *)notebook,page2_main_hbox,label_notebook_page_2);
  
  // horizontal box for notebook haptic demos [ARM!]
  GtkWidget *page3_main_hbox_arm_demos=gtk_hbox_new(FALSE, 3);
  //label page 3
  GtkWidget *label_notebook_page_3=gtk_label_new("Select Demonstrations");
  //create notebook page 3
  gtk_notebook_append_page((GtkNotebook *)notebook, page3_main_hbox_arm_demos, label_notebook_page_3);
  
  
  /*~~~~~~~~~~~~~~~~~~~~~~~~~ 
  || notebook main page left ||
    ~~~~~~~~~~~~~~~~~~~~~~~~~*/
  //vertical box for page 1 on the left
  GtkWidget *notebook_vbox_left_page1=gtk_vbox_new(FALSE, 5);
  //control type frame
  GtkWidget *page1_left_top_frame=gtk_frame_new("Control Type:");
  gtk_frame_set_shadow_type((GtkFrame *)page1_left_top_frame,GTK_SHADOW_OUT);
  //label widget (combo_box) definition
  GtkWidget *page1_left_top_frame_combobox=gtk_combo_box_new_text();
  RobotVars->updt_labels.kin_model_combobox=page1_left_top_frame_combobox;
  const gchar cmbbox_start[]="-> Select control type: <-";
  const gchar cmbbox_list1[]="Control Right Arm:";
  const gchar cmbbox_list2[]="Control Left Arm:";
  const gchar cmbbox_list3[]="Control Both Arms:";
  const gchar cmbbox_list4[]="Control Left Detached Leg:";
  const gchar cmbbox_list5[]="Control Right Detached Leg:";
  gtk_combo_box_append_text((GtkComboBox *)page1_left_top_frame_combobox,cmbbox_start);
  gtk_combo_box_append_text((GtkComboBox *)page1_left_top_frame_combobox,cmbbox_list1);
  gtk_combo_box_append_text((GtkComboBox *)page1_left_top_frame_combobox,cmbbox_list2);
  gtk_combo_box_append_text((GtkComboBox *)page1_left_top_frame_combobox,cmbbox_list3);
  gtk_combo_box_append_text((GtkComboBox *)page1_left_top_frame_combobox,cmbbox_list4);
  gtk_combo_box_append_text((GtkComboBox *)page1_left_top_frame_combobox,cmbbox_list5);
  gtk_combo_box_set_active((GtkComboBox *)page1_left_top_frame_combobox,0);
  gtk_frame_set_label_widget((GtkFrame *)page1_left_top_frame,page1_left_top_frame_combobox);
  //options frame
  GtkWidget *page1_left_bottom_frame=gtk_frame_new(NULL);
  gtk_frame_set_shadow_type((GtkFrame *)page1_left_bottom_frame,GTK_SHADOW_OUT);
  GtkWidget *label_options_frame=gtk_label_new(NULL);
  markup = g_markup_printf_escaped ("<span size=\"large\"> :: <u>%s</u> :: </span>", "Control Type Options");
  gtk_label_set_markup (GTK_LABEL (label_options_frame), markup);
  gtk_frame_set_label_widget((GtkFrame *)page1_left_bottom_frame, label_options_frame);

  //items in frame
  // vbox for all the items
  GtkWidget *page1_left_top_frame_vbox=gtk_vbox_new(FALSE, 5);
  
  //label arm coordinates LEFT
  GtkWidget *label_coordinates=gtk_label_new("Robot Left Arm end coordinates / Roll/Pitch/Yaw angles:");
  gtk_label_set_justify((GtkLabel *)label_coordinates,GTK_JUSTIFY_CENTER);
  markup = g_markup_printf_escaped ("<span underline=\"single\"><b>%s</b></span>", "Robot Left Arm end coordinates / Roll/Pitch/Yaw angles:");
  gtk_label_set_markup (GTK_LABEL (label_coordinates), markup);
  RobotVars->updt_labels.joints_left_side = label_coordinates;
  
  //hbox for labels
  GtkWidget *hbox_coordinates_left_arm=gtk_hbox_new(FALSE, 5);
  //x coord
  const gchar init_val[]="---- [mm]";
  GString *x_text=g_string_new("X: ");
  g_string_append(x_text,init_val);
  GtkWidget *label_X=gtk_label_new(x_text->str);
  gtk_label_set_justify((GtkLabel *)label_X,GTK_JUSTIFY_CENTER);
  RobotVars->updt_labels.x_label_left=label_X;
  //y coord
  GString *y_text=g_string_new("Y: ");
  g_string_append(y_text,init_val);
  GtkWidget *label_Y=gtk_label_new(y_text->str);
  gtk_label_set_justify((GtkLabel *)label_Y,GTK_JUSTIFY_CENTER);
  RobotVars->updt_labels.y_label_left=label_Y;
  //z coord
  GString *z_text=g_string_new("Z: ");
  g_string_append(z_text,init_val);
  GtkWidget *label_Z=gtk_label_new(z_text->str);
  gtk_label_set_justify((GtkLabel *)label_Z,GTK_JUSTIFY_CENTER);
  RobotVars->updt_labels.z_label_left=label_Z;
   //ROLL
  const gchar init_val_deg[]="---- [deg]";
  GString *roll_text=g_string_new("R: ");
  g_string_append(roll_text,init_val_deg);
  GtkWidget *label_roll=gtk_label_new(roll_text->str);
  gtk_label_set_justify((GtkLabel *)label_roll,GTK_JUSTIFY_CENTER);
  RobotVars->updt_labels.roll_label_left=label_roll;
  //PITCH
  GString *pitch_text=g_string_new("P: ");
  g_string_append(pitch_text,init_val_deg);
  GtkWidget *label_pitch=gtk_label_new(pitch_text->str);
  gtk_label_set_justify((GtkLabel *)label_pitch,GTK_JUSTIFY_CENTER);
  RobotVars->updt_labels.pitch_label_left=label_pitch;
  //YAW
  GString *yaw_text=g_string_new("Y: ");
  g_string_append(yaw_text,init_val_deg);
  GtkWidget *label_yaw=gtk_label_new(yaw_text->str);
  gtk_label_set_justify((GtkLabel *)label_yaw,GTK_JUSTIFY_CENTER);
  RobotVars->updt_labels.yaw_label_left=label_yaw;
  
  //label arm coordinates RIGHT
  GtkWidget *label_coordinates_2=gtk_label_new("Robot Right Arm end coordinates / Roll/Pitch/Yaw angles:");
  gtk_label_set_justify((GtkLabel *)label_coordinates_2,GTK_JUSTIFY_CENTER);
  markup = g_markup_printf_escaped ("<span underline=\"single\"><b>%s</b></span>", "Robot Right Arm end coordinates / Roll/Pitch/Yaw angles:");
  gtk_label_set_markup (GTK_LABEL (label_coordinates_2), markup);
 RobotVars->updt_labels.joints_right_side = label_coordinates_2;
  
  //hbox for labels
  GtkWidget *hbox_coordinates_right_arm=gtk_hbox_new(FALSE, 5);
  //x coord
  const gchar init_val_2[]="---- [mm]";
  GString *x_text_2=g_string_new("X: ");
  g_string_append(x_text_2,init_val_2);
  GtkWidget *label_X_2=gtk_label_new(x_text_2->str);
  gtk_label_set_justify((GtkLabel *)label_X_2,GTK_JUSTIFY_CENTER);
  RobotVars->updt_labels.x_label_right=label_X_2;
  //y coord
  GString *y_text_2=g_string_new("Y: ");
  g_string_append(y_text_2,init_val_2);
  GtkWidget *label_Y_right=gtk_label_new(y_text_2->str);
  gtk_label_set_justify((GtkLabel *)label_Y_right,GTK_JUSTIFY_CENTER);
  RobotVars->updt_labels.y_label_right=label_Y_right;
  //z coord
  GString *z_text_2=g_string_new("Z: ");
  g_string_append(z_text_2,init_val_2);
  GtkWidget *label_Z_right=gtk_label_new(z_text_2->str);
  gtk_label_set_justify((GtkLabel *)label_Z_right,GTK_JUSTIFY_CENTER);
  RobotVars->updt_labels.z_label_right=label_Z_right;
   //ROLL
  const gchar init_val_2_deg[]="---- [deg]";
  GString *roll_text_2=g_string_new("R: ");
  g_string_append(roll_text_2,init_val_2_deg);
  GtkWidget *label_roll_right=gtk_label_new(roll_text_2->str);
  gtk_label_set_justify((GtkLabel *)label_roll_right,GTK_JUSTIFY_CENTER);
  RobotVars->updt_labels.roll_label_right=label_roll_right;
  //PITCH
  GString *pitch_text_2=g_string_new("P: ");
  g_string_append(pitch_text_2,init_val_2_deg);
  GtkWidget *label_pitch_right=gtk_label_new(pitch_text_2->str);
  gtk_label_set_justify((GtkLabel *)label_pitch_right,GTK_JUSTIFY_CENTER);
  RobotVars->updt_labels.pitch_label_right=label_pitch_right;
  //YAW
  GString *yaw_text_2=g_string_new("Y: ");
  g_string_append(yaw_text_2,init_val_2_deg);
  GtkWidget *label_yaw_right=gtk_label_new(yaw_text_2->str);
  gtk_label_set_justify((GtkLabel *)label_yaw_right,GTK_JUSTIFY_CENTER);
  RobotVars->updt_labels.yaw_label_right=label_yaw_right;
  
  //robot joint values label
  GtkWidget *label_joint=gtk_label_new("Robot joint values (deg):");
  gtk_label_set_justify((GtkLabel *)label_joint,GTK_JUSTIFY_CENTER);
  markup = g_markup_printf_escaped ("<span underline=\"single\"><b>%s</b></span>", "Robot joint values (deg):");
  gtk_label_set_markup (GTK_LABEL (label_joint), markup);
  // HEAD
  const gchar init_val_jnt_[]="--- [deg]";
  GString *head_label_text=g_string_new("Head Tilt: ");
  g_string_append(head_label_text,init_val_jnt_);
  GtkWidget *label_head=gtk_label_new(head_label_text->str);
  gtk_label_set_justify((GtkLabel *)label_head,GTK_JUSTIFY_CENTER);
  RobotVars->updt_labels.head_tilt_label=label_head;
  //hbox for the arms
  GtkWidget *hbox_arm_jnts=gtk_hbox_new(FALSE, 5);
  // RIGHT ARM
  const gchar init_val_jnt[]="--- [deg]\n";
  GString *right_arm_label_text=g_string_new("Right Shoulder Flexion:	    ");
  g_string_append(right_arm_label_text,init_val_jnt);
  g_string_append(right_arm_label_text,"Right Shoulder Abduction:  ");
  g_string_append(right_arm_label_text,init_val_jnt);
  g_string_append(right_arm_label_text,"Right Elbow Flexion:	    ");
  g_string_append(right_arm_label_text,init_val_jnt_);
  GtkWidget *label_right_arm=gtk_label_new(right_arm_label_text->str);
  gtk_label_set_justify((GtkLabel *)label_right_arm,GTK_JUSTIFY_CENTER);
  RobotVars->updt_labels.right_arm_label=label_right_arm;
  // LEFT ARM
  GString *left_arm_label_text=g_string_new("Left Shoulder Flexion:	  ");
  g_string_append(left_arm_label_text,init_val_jnt);
  g_string_append(left_arm_label_text,"Left Shoulder Abduction:  ");
  g_string_append(left_arm_label_text,init_val_jnt);
  g_string_append(left_arm_label_text,"Left Elbow Flexion:		  ");
  g_string_append(left_arm_label_text,init_val_jnt_);
  GtkWidget *label_left_arm=gtk_label_new(left_arm_label_text->str);
  gtk_label_set_justify((GtkLabel *)label_left_arm,GTK_JUSTIFY_CENTER);
  RobotVars->updt_labels.left_arm_label=label_left_arm;
  // TORSO
  GString *torso_label_text=g_string_new("Torso Rotation:	     ");
  g_string_append(torso_label_text,init_val_jnt);
  g_string_append(torso_label_text,"Torso Flexion:		     ");
  g_string_append(torso_label_text,init_val_jnt);
  g_string_append(torso_label_text,"Torso Lateral Flexion:  ");
  g_string_append(torso_label_text,init_val_jnt_);
  GtkWidget *label_torso=gtk_label_new(torso_label_text->str);
  gtk_label_set_justify((GtkLabel *)label_torso,GTK_JUSTIFY_CENTER);
  RobotVars->updt_labels.torso_label=label_torso;
  //hbox for the legs
  GtkWidget *hbox_leg_jnts=gtk_hbox_new(FALSE, 5);
  // RIGHT LEG
  GString *right_leg_label_text=g_string_new("Right Hip Flexion:	      ");
  g_string_append(right_leg_label_text,init_val_jnt);
  g_string_append(right_leg_label_text,"Right Hip Abduction:     ");
  g_string_append(right_leg_label_text,init_val_jnt);
  g_string_append(right_leg_label_text,"Right Knee Flexion:      ");
  g_string_append(right_leg_label_text,init_val_jnt);
  g_string_append(right_leg_label_text,"Right Ankle Flexion:     ");
  g_string_append(right_leg_label_text,init_val_jnt);
  g_string_append(right_leg_label_text,"Right Ankle Inversion:  ");
  g_string_append(right_leg_label_text,init_val_jnt_);
  GtkWidget *label_right_leg=gtk_label_new(right_leg_label_text->str);
  gtk_label_set_justify((GtkLabel *)label_right_leg,GTK_JUSTIFY_CENTER);
  RobotVars->updt_labels.right_leg_label=label_right_leg;
  // LEFT LEG
  GString *left_leg_label_text=g_string_new("Left Hip Flexion:	    ");
  g_string_append(left_leg_label_text,init_val_jnt);
  g_string_append(left_leg_label_text,"Left Hip Abduction:     ");
  g_string_append(left_leg_label_text,init_val_jnt);
  g_string_append(left_leg_label_text,"Left Knee Flexion:      ");
  g_string_append(left_leg_label_text,init_val_jnt);
  g_string_append(left_leg_label_text,"Left Ankle Flexion:     ");
  g_string_append(left_leg_label_text,init_val_jnt);
  g_string_append(left_leg_label_text,"Left Ankle Inversion:  ");
  g_string_append(left_leg_label_text,init_val_jnt_);
  GtkWidget *label_left_leg=gtk_label_new(left_leg_label_text->str);
  gtk_label_set_justify((GtkLabel *)label_left_leg,GTK_JUSTIFY_CENTER);
  RobotVars->updt_labels.left_leg_label=label_left_leg;
  //bottom vbox (OPTIONS)
  //vbox for option type
  GtkWidget *page1_left_bottom_frame_vbox=gtk_vbox_new(FALSE, 5);
  //option constant speed
  GtkWidget *constant_speed_radio_checkbox=gtk_check_button_new_with_label("Position Control Mode");
  gtk_toggle_button_set_active((GtkToggleButton *)constant_speed_radio_checkbox,TRUE);
  //hbox for scale bar and label
  GtkWidget *hbox_scrll_lbl=gtk_hbox_new(FALSE, 5);
  //label
  GtkWidget *label_speed_scale_bar=gtk_label_new("Overall Servomotor Speed:");
  gtk_label_set_justify((GtkLabel *)label_speed_scale_bar,GTK_JUSTIFY_LEFT);
  //hscale
  GtkObject *ajustament_hscale=gtk_adjustment_new(50,1,280,1,25,25);
  GtkWidget *set_speed_hscale=gtk_hscale_new((GtkAdjustment *)ajustament_hscale);
  gtk_scale_set_digits((GtkScale *)set_speed_hscale,0);
  gtk_scale_set_value_pos((GtkScale *)set_speed_hscale,GTK_POS_RIGHT);
  //for loop to set the ticks on the scale bar
  int tick;
  for(tick=0;tick<255;tick=tick+50)
    gtk_scale_add_mark((GtkScale *)set_speed_hscale,tick,GTK_POS_BOTTOM,NULL);
  // controled speed checkbox
  GtkWidget *controled_speed_radio_checkbox=gtk_check_button_new_with_label("Speed Control Mode");
  gtk_toggle_button_set_active((GtkToggleButton *)controled_speed_radio_checkbox,FALSE);
  //items for controled speed options
  //vbox for bars
  GtkWidget *hbox_prog_bars_auto_speed=gtk_hbox_new(FALSE, 5);
  gtk_widget_set_visible((GtkWidget *)hbox_prog_bars_auto_speed,FALSE);
  //progress bars
  GtkWidget *arm_theta1_speed_prog_bar=gtk_progress_bar_new();
  gtk_progress_bar_set_text((GtkProgressBar *)arm_theta1_speed_prog_bar,"Shoulder Flexion Speed");
  gtk_progress_bar_set_fraction((GtkProgressBar *)arm_theta1_speed_prog_bar,1./255.);
  RobotVars->updt_labels.auto_speed_arm_theta1_bar=arm_theta1_speed_prog_bar;
  GtkWidget *arm_theta2_speed_prog_bar=gtk_progress_bar_new();
  gtk_progress_bar_set_text((GtkProgressBar *)arm_theta2_speed_prog_bar,"Shoulder Abduction Speed");
  gtk_progress_bar_set_fraction((GtkProgressBar *)arm_theta2_speed_prog_bar,1./255.);
  RobotVars->updt_labels.auto_speed_arm_theta2_bar=arm_theta2_speed_prog_bar;
  GtkWidget *arm_theta3_speed_prog_bar=gtk_progress_bar_new();
  gtk_progress_bar_set_text((GtkProgressBar *)arm_theta3_speed_prog_bar,"Elbow Flexion Speed");
  gtk_progress_bar_set_fraction((GtkProgressBar *)arm_theta3_speed_prog_bar,1./255.);
  RobotVars->updt_labels.auto_speed_arm_theta3_bar=arm_theta3_speed_prog_bar;
  
  /*~~~~~~~~~~~~~~~~~~~~~~~~~~~ 
  || notebook main page right ||
    ~~~~~~~~~~~~~~~~~~~~~~~~~~*/
  //frame for items
  GtkWidget *page1_middle_frame=gtk_frame_new(" :: Haptics and Control Parameters :: ");
  gtk_frame_set_shadow_type((GtkFrame *)page1_middle_frame,GTK_SHADOW_OUT);
  gtk_frame_set_label_align((GtkFrame *)page1_middle_frame,0.5,0.5);
  GtkWidget *label_haptics_data_frame=gtk_label_new(NULL);
  markup = g_markup_printf_escaped (" :: <b>%s</b> :: ", "Haptics and Control Parameters");
  gtk_label_set_markup (GTK_LABEL (label_haptics_data_frame), markup);
  gtk_frame_set_label_widget((GtkFrame *)page1_middle_frame, label_haptics_data_frame);
  
  //vbox for items
  GtkWidget *vbox_phantom=gtk_vbox_new(FALSE, 3);
  
//   *****************************************
  //frame for haptics data
  GtkWidget *page1_haptics_data_frame=gtk_frame_new("Haptics Data:");
  gtk_frame_set_shadow_type((GtkFrame *)page1_haptics_data_frame,GTK_SHADOW_OUT);
  //vbox for haptic information
  GtkWidget *vbox_haptic_information=gtk_vbox_new(FALSE, 0);
  //hbox for workspace limit interaction
  GtkWidget *hbox_workspace_interaction=gtk_hbox_new(FALSE, 3);
  //label for workspace interaction
  GtkWidget *label_workspace_interaction=gtk_label_new("Workspace Demonstration --");
  gtk_label_set_justify((GtkLabel *)label_workspace_interaction,GTK_JUSTIFY_LEFT);
  RobotVars->updt_labels.workspace_zone_label=label_workspace_interaction;
  
  //hbox for drawing demo
  GtkWidget *hbox_drawing_demo=gtk_hbox_new(FALSE, 3);
  //label for workspace interaction
  GtkWidget *label_drawing_demo=gtk_label_new("Plane navigation: -------------\nPlane equation: ax + by + cz + d = 0");
  gtk_label_set_justify((GtkLabel *)label_drawing_demo,GTK_JUSTIFY_LEFT);
  RobotVars->updt_labels.drawing_demo_label=label_drawing_demo;
  
  //hbox for user path demo
  GtkWidget *hbox_user_path_demo=gtk_hbox_new(FALSE, 3);
  //label for workspace interaction
  GtkWidget *label_user_path_demo=gtk_label_new("User path following\nDEMO.");
  gtk_label_set_justify((GtkLabel *)label_user_path_demo,GTK_JUSTIFY_LEFT);
  RobotVars->updt_labels.user_path_label=label_user_path_demo;
  //button for point storing
  GtkWidget *user_path_demo_point_store_button = gtk_button_new_with_label("Store Points");
  //vbox for run options
  GtkWidget *vbox_user_path_run_options = gtk_vbox_new(FALSE, 3);
  //button to run loop
  GtkWidget *user_path_demo_run_button = gtk_button_new_with_label("Run Points");
  RobotVars->updt_labels.user_path_run_button = user_path_demo_run_button;
  //box for checkboxes
  GtkWidget *hbox_user_path_checkboxes = gtk_hbox_new(FALSE, 3);
  //checkboxes for run once / loop
  GtkWidget *user_path_demo_run_once_checkbox = gtk_check_button_new_with_label("Once");
  gtk_toggle_button_set_active((GtkToggleButton *)user_path_demo_run_once_checkbox,TRUE);
  GtkWidget *user_path_demo_run_loop_checkbox = gtk_check_button_new_with_label("Loop");
  gtk_toggle_button_set_active((GtkToggleButton *)user_path_demo_run_loop_checkbox,FALSE);
  //button to clear
  GtkWidget *user_path_demo_clear_button = gtk_button_new_with_label("Clear");
  
  //hbox for leg balancing
  GtkWidget *hbox_leg_balancing_demo=gtk_hbox_new(FALSE, 3);
  //label for leg balancing / COG
  GtkWidget *label_leg_balancing_cog_demo=gtk_label_new("Balancing Support Leg:\nCOG: X:---  Y:---  Z:---");
  gtk_label_set_justify((GtkLabel *)label_leg_balancing_cog_demo,GTK_JUSTIFY_LEFT);
  RobotVars->updt_labels.leg_balancing_demo_label=label_leg_balancing_cog_demo;
  
  // label for force bars
  GtkWidget *label_force_feedback=gtk_label_new(NULL);
  markup = g_markup_printf_escaped ("<span underline=\"single\"><b>%s</b></span>", "Force Feedback");
  gtk_label_set_markup (GTK_LABEL (label_force_feedback), markup);
  gtk_label_set_justify((GtkLabel *)label_force_feedback,GTK_JUSTIFY_CENTER);
  //vbox for force bars
  GtkWidget *vbox_force_feedback=gtk_vbox_new(FALSE, 0);
  //hbox for X force
  GtkWidget *hbox_haptic_x_force=gtk_hbox_new(FALSE, 3);
  // X force progress bars
  GtkWidget *x_force_bar=gtk_progress_bar_new();
  gtk_progress_bar_set_fraction((GtkProgressBar *)x_force_bar, 0.);
  RobotVars->updt_labels.pos_x_force_bar=x_force_bar;
  GtkWidget *x_neg_force_bar=gtk_progress_bar_new();
  gtk_progress_bar_set_fraction((GtkProgressBar *)x_neg_force_bar, 0.);
  gtk_progress_bar_set_orientation((GtkProgressBar *)x_neg_force_bar,GTK_PROGRESS_RIGHT_TO_LEFT);
  RobotVars->updt_labels.neg_x_force_bar=x_neg_force_bar;
  // X force label
  GtkWidget *label_x_force=gtk_label_new("X");
  gtk_label_set_justify((GtkLabel *)label_x_force,GTK_JUSTIFY_CENTER);
  gtk_widget_modify_fg(label_x_force, GTK_STATE_NORMAL, &red);
  //hbox for Y force
  GtkWidget *hbox_haptic_y_force=gtk_hbox_new(FALSE, 3);
  // Y force progress bars
  GtkWidget *y_force_bar=gtk_progress_bar_new();
  gtk_progress_bar_set_fraction((GtkProgressBar *)y_force_bar, 0.);
  RobotVars->updt_labels.pos_y_force_bar = y_force_bar;
  GtkWidget *y_neg_force_bar=gtk_progress_bar_new();
  gtk_progress_bar_set_fraction((GtkProgressBar *)y_neg_force_bar, 0.);
  gtk_progress_bar_set_orientation((GtkProgressBar *)y_neg_force_bar,GTK_PROGRESS_RIGHT_TO_LEFT);
  RobotVars->updt_labels.neg_y_force_bar = y_neg_force_bar;
  // Y force label
  GtkWidget *label_y_force=gtk_label_new("Y");
  gtk_label_set_justify((GtkLabel *)label_y_force,GTK_JUSTIFY_CENTER);
  gtk_widget_modify_fg(label_y_force, GTK_STATE_NORMAL, &green);
  //vbox for Z force
  GtkWidget *hbox_haptic_z_force=gtk_hbox_new(FALSE, 3);
  // Z force progress bars
  GtkWidget *z_force_bar=gtk_progress_bar_new();
  gtk_progress_bar_set_fraction((GtkProgressBar *)z_force_bar, 0.);
  RobotVars->updt_labels.pos_z_force_bar = z_force_bar;
  GtkWidget *z_neg_force_bar=gtk_progress_bar_new();
  gtk_progress_bar_set_fraction((GtkProgressBar *)z_neg_force_bar, 0.);
  gtk_progress_bar_set_orientation((GtkProgressBar *)z_neg_force_bar,GTK_PROGRESS_RIGHT_TO_LEFT);
  RobotVars->updt_labels.neg_z_force_bar = z_neg_force_bar;
  // Z force label
  GtkWidget *label_z_force=gtk_label_new("Z");
  gtk_label_set_justify((GtkLabel *)label_z_force,GTK_JUSTIFY_CENTER);
  gtk_widget_modify_fg(label_z_force, GTK_STATE_NORMAL, &blue);
  // force magnitude label
  GtkWidget *label_force_magnitude=gtk_label_new("Force magnitude: --- [N] (sent to the device)");
  gtk_label_set_justify((GtkLabel *)label_force_magnitude,GTK_JUSTIFY_CENTER);
  RobotVars->updt_labels.force_magnitude_label = label_force_magnitude;
  
//   *****************************************
  //frame for parameters
  GtkWidget *page1_control_parameters_frame=gtk_frame_new("Control Parameters:");
  gtk_frame_set_shadow_type((GtkFrame *)page1_control_parameters_frame,GTK_SHADOW_OUT);
  //vbox for parameters
  GtkWidget *vbox_control_parameters=gtk_vbox_new(FALSE, 0);
  //label resolution
  GtkWidget *label_control_resolution=gtk_label_new("Control Resolution:");
  gtk_label_set_justify((GtkLabel *)label_control_resolution,GTK_JUSTIFY_LEFT);
  //hbox for checkboxes
  GtkWidget *hbox_control_parameters=gtk_hbox_new(FALSE, 3);
  //resolution checkboxes
  GtkWidget *control_resolution_checkb_1mm=gtk_check_button_new_with_label("1 mm");
  gtk_toggle_button_set_active((GtkToggleButton *)control_resolution_checkb_1mm,TRUE);
  GtkWidget *control_resolution_checkb_0_1mm=gtk_check_button_new_with_label("0.1 mm");
  gtk_toggle_button_set_active((GtkToggleButton *)control_resolution_checkb_0_1mm,FALSE);
  GtkWidget *control_resolution_checkb_0_0_1mm=gtk_check_button_new_with_label("0.01 mm");
  gtk_toggle_button_set_active((GtkToggleButton *)control_resolution_checkb_0_0_1mm,FALSE);
  GtkWidget *control_resolution_checkb_0_0_0_1mm=gtk_check_button_new_with_label("0.001 mm");
  gtk_toggle_button_set_active((GtkToggleButton *)control_resolution_checkb_0_0_0_1mm,FALSE);
  //label workspace scale
  GtkWidget *label_workspace_scale=gtk_label_new("Workspace Scaling:");
  gtk_label_set_justify((GtkLabel *)label_workspace_scale,GTK_JUSTIFY_LEFT);
  //hbox for checkboxes
  GtkWidget *hbox_workspace_scale=gtk_hbox_new(FALSE, 3);
  //resolution checkboxes
  GtkWidget *scale_1=gtk_check_button_new_with_label("x1");
  gtk_toggle_button_set_active((GtkToggleButton *)scale_1,FALSE);
  GtkWidget *scale_2=gtk_check_button_new_with_label("x2");
  gtk_toggle_button_set_active((GtkToggleButton *)scale_2,TRUE);
  GtkWidget *scale_3=gtk_check_button_new_with_label("x3");
  gtk_toggle_button_set_active((GtkToggleButton *)scale_3,FALSE);
  GtkWidget *scale_4=gtk_check_button_new_with_label("x4");
  gtk_toggle_button_set_active((GtkToggleButton *)scale_4,FALSE);
  
  /*~~~~~~~~~~~~~~~~~~~~~~~~~ 
  || notebook miscellaneous  ||
    ~~~~~~~~~~~~~~~~~~~~~~~~~*/
  // box of page 2    | page2_main_hbox |
  //left frame
  GtkWidget *page2_left_frame=gtk_frame_new(NULL);
  gtk_frame_set_shadow_type((GtkFrame *)page2_left_frame,GTK_SHADOW_OUT);
  GtkWidget *label_indivdual_control_frame=gtk_label_new(NULL);
  markup = g_markup_printf_escaped (" :: <b>%s</b> :: ", "Miscellaneous Humanoid Robot Control");
  gtk_label_set_markup (GTK_LABEL (label_indivdual_control_frame), markup);
  gtk_frame_set_label_widget((GtkFrame *)page2_left_frame, label_indivdual_control_frame);
  
  // vbox for funcionalities
  GtkWidget *page2_vbox_funcs=gtk_vbox_new(FALSE, 5);
  //label setPosition
  GtkWidget *label_page_2_setpos=gtk_label_new(NULL);
  markup = g_markup_printf_escaped ("<u>%s</u>:", "Set Individual Joint Angle");
  gtk_label_set_markup (GTK_LABEL (label_page_2_setpos), markup);
  //hbox for setPosition
  GtkWidget *page2_setpos_hbox=gtk_hbox_new(FALSE, 5);
  //combobox for id
  GtkWidget *page2_select_id_combobox=gtk_combo_box_new_text();
  const gchar id_cmbbox_start[]=" >> Select Joint: <<";
  const gchar id_cmbbox_list1[]="Head Tilt";
  const gchar id_cmbbox_list2[]="Right Shoulder Flexion";
  const gchar id_cmbbox_list3[]="Right Shoulder Abduction";
  const gchar id_cmbbox_list4[]="Right Elbow Flexion";
  const gchar id_cmbbox_list5[]="Left Shoulder Flexion";
  const gchar id_cmbbox_list6[]="Left Shoulder Abduction";
  const gchar id_cmbbox_list7[]="Left Elbow Flexion";
  const gchar id_cmbbox_list8[]="Torso Rotation";
  const gchar id_cmbbox_list9[]="Torso Flexion";
  const gchar id_cmbbox_list10[]="Torso Lateral Flexion";
  const gchar id_cmbbox_list11[]="Right Ankle Inversion";
  const gchar id_cmbbox_list12[]="Right Ankle Flexion";
  const gchar id_cmbbox_list13[]="Right Knee Flexion";
  const gchar id_cmbbox_list14[]="Right Hip Abduction";
  const gchar id_cmbbox_list15[]="Right Hip Flexion";
  const gchar id_cmbbox_list16[]="Left Ankle Inversion";
  const gchar id_cmbbox_list17[]="Left Ankle Flexion";
  const gchar id_cmbbox_list18[]="Left Knee Flexion";
  const gchar id_cmbbox_list19[]="Left Hip Abduction";
  const gchar id_cmbbox_list20[]="Left Hip Flexion";
  gtk_combo_box_append_text((GtkComboBox *)page2_select_id_combobox,id_cmbbox_start);
  gtk_combo_box_append_text((GtkComboBox *)page2_select_id_combobox,id_cmbbox_list1);
  gtk_combo_box_append_text((GtkComboBox *)page2_select_id_combobox,id_cmbbox_list2);
  gtk_combo_box_append_text((GtkComboBox *)page2_select_id_combobox,id_cmbbox_list3);
  gtk_combo_box_append_text((GtkComboBox *)page2_select_id_combobox,id_cmbbox_list4);
  gtk_combo_box_append_text((GtkComboBox *)page2_select_id_combobox,id_cmbbox_list5);
  gtk_combo_box_append_text((GtkComboBox *)page2_select_id_combobox,id_cmbbox_list6);
  gtk_combo_box_append_text((GtkComboBox *)page2_select_id_combobox,id_cmbbox_list7);
  gtk_combo_box_append_text((GtkComboBox *)page2_select_id_combobox,id_cmbbox_list8);
  gtk_combo_box_append_text((GtkComboBox *)page2_select_id_combobox,id_cmbbox_list9);
  gtk_combo_box_append_text((GtkComboBox *)page2_select_id_combobox,id_cmbbox_list10);
  gtk_combo_box_append_text((GtkComboBox *)page2_select_id_combobox,id_cmbbox_list11);
  gtk_combo_box_append_text((GtkComboBox *)page2_select_id_combobox,id_cmbbox_list12);
  gtk_combo_box_append_text((GtkComboBox *)page2_select_id_combobox,id_cmbbox_list13);
  gtk_combo_box_append_text((GtkComboBox *)page2_select_id_combobox,id_cmbbox_list14);
  gtk_combo_box_append_text((GtkComboBox *)page2_select_id_combobox,id_cmbbox_list15);
  gtk_combo_box_append_text((GtkComboBox *)page2_select_id_combobox,id_cmbbox_list16);
  gtk_combo_box_append_text((GtkComboBox *)page2_select_id_combobox,id_cmbbox_list17);
  gtk_combo_box_append_text((GtkComboBox *)page2_select_id_combobox,id_cmbbox_list18);
  gtk_combo_box_append_text((GtkComboBox *)page2_select_id_combobox,id_cmbbox_list19);
  gtk_combo_box_append_text((GtkComboBox *)page2_select_id_combobox,id_cmbbox_list20);
  gtk_combo_box_set_active((GtkComboBox *)page2_select_id_combobox,0);
  //label
  GtkWidget *label_page_2_joint_angle=gtk_label_new("Joint Angle:");
//   //entry
//   GtkWidget *entry_angle=gtk_entry_new();
//   gtk_entry_set_max_length((GtkEntry *)entry_angle,6);
  
  //spinbutton
  GtkWidget *spinbutton_angle = gtk_spin_button_new_with_range ( -360., 360., 0.05);
  gtk_spin_button_set_value ((GtkSpinButton *)spinbutton_angle, 0.);
  
  //label for angle limitations
  GtkWidget *label_page_2_angle=gtk_label_new("[---]");
  //button to submit
  GtkWidget *button_setpos=gtk_button_new_with_label("Submit Angle");
  
  //label setSpeedReadPos
  GtkWidget *label_page_2_setspeed=gtk_label_new(NULL);
  markup = g_markup_printf_escaped ("<u>%s</u> / <u>%s</u>:", "Set Individual Joint Speed","Read Position");
  gtk_label_set_markup (GTK_LABEL (label_page_2_setspeed), markup);
  //hbox for setSpeed
  GtkWidget *page2_setspeed_hbox=gtk_hbox_new(FALSE, 3);
  //combobox for id
  GtkWidget *page2_select_id_combobox2=gtk_combo_box_new_text();
  gtk_combo_box_append_text((GtkComboBox *)page2_select_id_combobox2,id_cmbbox_start);
  gtk_combo_box_append_text((GtkComboBox *)page2_select_id_combobox2,id_cmbbox_list1);
  gtk_combo_box_append_text((GtkComboBox *)page2_select_id_combobox2,id_cmbbox_list2);
  gtk_combo_box_append_text((GtkComboBox *)page2_select_id_combobox2,id_cmbbox_list3);
  gtk_combo_box_append_text((GtkComboBox *)page2_select_id_combobox2,id_cmbbox_list4);
  gtk_combo_box_append_text((GtkComboBox *)page2_select_id_combobox2,id_cmbbox_list5);
  gtk_combo_box_append_text((GtkComboBox *)page2_select_id_combobox2,id_cmbbox_list6);
  gtk_combo_box_append_text((GtkComboBox *)page2_select_id_combobox2,id_cmbbox_list7);
  gtk_combo_box_append_text((GtkComboBox *)page2_select_id_combobox2,id_cmbbox_list8);
  gtk_combo_box_append_text((GtkComboBox *)page2_select_id_combobox2,id_cmbbox_list9);
  gtk_combo_box_append_text((GtkComboBox *)page2_select_id_combobox2,id_cmbbox_list10);
  gtk_combo_box_append_text((GtkComboBox *)page2_select_id_combobox2,id_cmbbox_list11);
  gtk_combo_box_append_text((GtkComboBox *)page2_select_id_combobox2,id_cmbbox_list12);
  gtk_combo_box_append_text((GtkComboBox *)page2_select_id_combobox2,id_cmbbox_list13);
  gtk_combo_box_append_text((GtkComboBox *)page2_select_id_combobox2,id_cmbbox_list14);
  gtk_combo_box_append_text((GtkComboBox *)page2_select_id_combobox2,id_cmbbox_list15);
  gtk_combo_box_append_text((GtkComboBox *)page2_select_id_combobox2,id_cmbbox_list16);
  gtk_combo_box_append_text((GtkComboBox *)page2_select_id_combobox2,id_cmbbox_list17);
  gtk_combo_box_append_text((GtkComboBox *)page2_select_id_combobox2,id_cmbbox_list18);
  gtk_combo_box_append_text((GtkComboBox *)page2_select_id_combobox2,id_cmbbox_list19);
  gtk_combo_box_append_text((GtkComboBox *)page2_select_id_combobox2,id_cmbbox_list20);
  gtk_combo_box_set_active((GtkComboBox *)page2_select_id_combobox2,0);
  //label
  GtkWidget *label_page_2_speed_entry=gtk_label_new("Speed:");
  //spinbutton
  GtkWidget *spinbutton_speed = gtk_spin_button_new_with_range ( 1., 255., 1.);
  gtk_spin_button_set_value ((GtkSpinButton *)spinbutton_speed, 1.);
  
  //label for speed limitations
  GtkWidget *label_page_2_speed=gtk_label_new("[1...255]");
  //button to submit
  GtkWidget *button_setspeed=gtk_button_new_with_label("Submit Speed");
  //label for position read
  GtkWidget *label_page_2_pos_read=gtk_label_new("Position read:\n---[deg]");
  gtk_label_set_justify((GtkLabel *)label_page_2_pos_read,GTK_JUSTIFY_CENTER);
  
  //label for set position of all servos
  GtkWidget *label_page_2_set_pos_all=gtk_label_new(NULL);
  markup = g_markup_printf_escaped ("<u>%s</u>:", "Set Servomotor Position Off All The Joints");
  gtk_label_set_markup (GTK_LABEL (label_page_2_set_pos_all), markup);
  
  //hbox for set position of all servos
  GtkWidget *page2_set_pos_all_hbox=gtk_hbox_new(FALSE, 3);
  //label
  GtkWidget *label_page_2_setposall=gtk_label_new("	Digital Position:");
  //entry
  GtkWidget *entry_pos_all=gtk_entry_new();
  gtk_entry_set_max_length((GtkEntry *)entry_pos_all,4);
  //label for set position and warning
  GtkWidget *label_page_2_limits=gtk_label_new("Value range [600...2400]");
  gtk_label_set_justify((GtkLabel *)label_page_2_limits,GTK_JUSTIFY_CENTER);
  //button to submit
  GtkWidget *button_set_pos_all=gtk_button_new_with_label("Submit Position");
  
  //label for inverse kinematics
  GtkWidget *label_page_2_invkin=gtk_label_new(NULL);
  markup = g_markup_printf_escaped ("<u>%s</u>:", "Inverse Kinematics");
  gtk_label_set_markup (GTK_LABEL (label_page_2_invkin), markup);
  //hbox for inverse kinematics test
  GtkWidget *page2_invkin_hbox=gtk_hbox_new(FALSE, 3);
  //combobox
  GtkWidget *select_inv_kin_combobox=gtk_combo_box_new_text();
  gtk_combo_box_append_text((GtkComboBox *)select_inv_kin_combobox, " >> Kinematic Model: <<");
  gtk_combo_box_append_text((GtkComboBox *)select_inv_kin_combobox, "Arm (Shoulder ref.)");
  gtk_combo_box_append_text((GtkComboBox *)select_inv_kin_combobox, "Arm (Main ref.)");
  gtk_combo_box_append_text((GtkComboBox *)select_inv_kin_combobox, "Detached Leg");
  gtk_combo_box_set_active((GtkComboBox *)select_inv_kin_combobox,0);
  
  //spinbuttons & labels
  GtkWidget *label_invkin_X=gtk_label_new("X:");
  
  GtkWidget *spinbutton_invkin_X = gtk_spin_button_new_with_range ( -1000, 1000, 0.1);
  gtk_spin_button_set_value ((GtkSpinButton *)spinbutton_invkin_X, 0);
  
  GtkWidget *label_mm_1=gtk_label_new("[mm] ");
  GtkWidget *label_invkin_Y=gtk_label_new("Y:");
  
  GtkWidget *spinbutton_invkin_Y = gtk_spin_button_new_with_range ( -1000, 1000, 0.1);
  gtk_spin_button_set_value ((GtkSpinButton *)spinbutton_invkin_Y, 0);
  
  GtkWidget *label_mm_2=gtk_label_new("[mm] ");
  GtkWidget *label_invkin_Z=gtk_label_new("Z:");
  
  GtkWidget *spinbutton_invkin_Z = gtk_spin_button_new_with_range ( -1000, 1000, 0.1);
  gtk_spin_button_set_value ((GtkSpinButton *)spinbutton_invkin_Z, 0);
  
  GtkWidget *label_mm_3=gtk_label_new("[mm] ");
  //button to test inverse kinematics
  GtkWidget *button_test_invkin=gtk_button_new_with_label("Test I.K.");
  
  //hbox for utilities
  GtkWidget *page2_utilities_hbox=gtk_hbox_new(FALSE, 5);
  //label for utilities
  GtkWidget *label_page_2_utils=gtk_label_new(NULL);
  markup = g_markup_printf_escaped ("<b>%s</b>", "Utilities");
  gtk_label_set_markup (GTK_LABEL (label_page_2_utils), markup);
  //release all servos
  GtkWidget *button_release=gtk_button_new_with_label("Release Servos");
  //set go/stop
  GtkWidget *button_go=gtk_toggle_button_new_with_label ("Set Go");
  RobotVars->updt_labels.go_button = button_go;
  GtkWidget *button_stop=gtk_toggle_button_new_with_label("Set Stop");
  RobotVars->updt_labels.stop_button = button_stop;
  
  //right frame
  GtkWidget *page2_right_frame=gtk_frame_new(NULL);
  gtk_frame_set_shadow_type((GtkFrame *)page2_right_frame,GTK_SHADOW_OUT);
  GtkWidget *label_phantom_data_frame=gtk_label_new(NULL);
  markup = g_markup_printf_escaped (" :: <b>%s</b> :: ", "PHANToM OMNI Data and Parameters");
  gtk_label_set_markup (GTK_LABEL (label_phantom_data_frame), markup);
  gtk_frame_set_label_widget((GtkFrame *)page2_right_frame, label_phantom_data_frame);
  
  // vbox for phantom related stuff
  GtkWidget *page2_vbox_phantom=gtk_vbox_new(FALSE, 5);
  //calibration text
  GtkWidget *label_calibration=gtk_label_new(NULL);
  markup = g_markup_printf_escaped ("<i>%s</i>","Normal usage of the PHANToM haptic device will cause a small decalibration to incrementally appear. Please take the time to check it, following the instructions from the small window shown after pressing the button below.");
  gtk_label_set_markup (GTK_LABEL (label_calibration), markup);
  gtk_label_set_justify((GtkLabel *)label_calibration,GTK_JUSTIFY_CENTER);
  gtk_label_set_line_wrap((GtkLabel *)label_calibration,TRUE);
  //calibration button
  GtkWidget *button_calibration = gtk_toggle_button_new();
  GtkWidget *calibration_button_label = gtk_label_new(NULL);
  markup = g_markup_printf_escaped ("<span size=\"larger\"><i><b>%s</b>\n<b>%s</b></i></span>","PHANToM OMNI DEVICE","CALIBRATION TEST");
  gtk_label_set_markup (GTK_LABEL (calibration_button_label), markup);
  gtk_label_set_justify((GtkLabel *)calibration_button_label,GTK_JUSTIFY_CENTER);
  gtk_container_add(GTK_CONTAINER(button_calibration),calibration_button_label);
  
  //   *****************************************
  // PHANTOM RAW DATA
  //frame for joystick data
  GtkWidget *page2_joystick_raw_data_frame=gtk_frame_new("PHANToM Raw Data:");
  gtk_frame_set_shadow_type((GtkFrame *)page2_joystick_raw_data_frame,GTK_SHADOW_OUT);
  //vbox for joystick data
  GtkWidget *vbox_phantom_raw=gtk_vbox_new(FALSE, 0);
  //joystick position label
  GtkWidget *joystick_position_label=gtk_label_new("PHANToM OMNI Coordinates:");
  gtk_label_set_justify((GtkLabel *)joystick_position_label,GTK_JUSTIFY_CENTER);
  //hbox for joystick coordinates
  GtkWidget *joystick_position_hbox=gtk_hbox_new(FALSE, 5);
  //joystick X
  GtkWidget *joystick_X_label=gtk_label_new("X: --- [mm]");
  RobotVars->updt_labels.phantom_x_label=joystick_X_label;
  //joystick Y
  GtkWidget *joystick_Y_label=gtk_label_new("Y: --- [mm]");
  RobotVars->updt_labels.phantom_y_label=joystick_Y_label;
  //joystick Z
  GtkWidget *joystick_Z_label=gtk_label_new("Z: --- [mm]");
  RobotVars->updt_labels.phantom_z_label=joystick_Z_label;
  
  //joystick pen tip position label
  GtkWidget *joystick_pen_tip_position_label=gtk_label_new("PHANToM OMNI Pen Tip Coordinates:");
  gtk_label_set_justify((GtkLabel *)joystick_pen_tip_position_label,GTK_JUSTIFY_CENTER);
  //hbox for joystick coordinates
  GtkWidget *joystick_pen_tip_position_hbox=gtk_hbox_new(FALSE, 5);
  //joystick X
  GtkWidget *joystick_pen_tip_X_label=gtk_label_new("X: --- [mm]");
  RobotVars->updt_labels.phantom_pen_tip_x_label=joystick_pen_tip_X_label;
  //joystick Y
  GtkWidget *joystick_pen_tip_Y_label=gtk_label_new("Y: --- [mm]");
  RobotVars->updt_labels.phantom_pen_tip_y_label=joystick_pen_tip_Y_label;
  //joystick Z
  GtkWidget *joystick_pen_tip_Z_label=gtk_label_new("Z: --- [mm]");
  RobotVars->updt_labels.phantom_pen_tip_z_label=joystick_pen_tip_Z_label;
  
  //joystick speed
  GtkWidget *joystick_speed_label=gtk_label_new("PHANToM OMNI Cartesian Speed:");
  gtk_label_set_justify((GtkLabel *)joystick_speed_label,GTK_JUSTIFY_CENTER);
  //hbox for joystick speed
  GtkWidget *joystick_speed_hbox=gtk_hbox_new(FALSE, 5);
  //joystick speed
  GtkWidget *joystick_speed_values_label=gtk_label_new("Magnitude: --- [mm/s] (---,---,---)");
  RobotVars->updt_labels.phantom_speed_label=joystick_speed_values_label;
  
  //joystick joint angles label
  GtkWidget *joystick_joint_angles_label=gtk_label_new("PHANToM OMNI Joint Angles:");
  gtk_label_set_justify((GtkLabel *)joystick_joint_angles_label,GTK_JUSTIFY_CENTER);
  //hbox for joystick coordinates
  GtkWidget *joystick_joint_angles_hbox=gtk_hbox_new(FALSE, 5);
  //joystick joint1
  GtkWidget *joystick_joint1_label=gtk_label_new(NULL);
  gtk_label_set_markup (GTK_LABEL (joystick_joint1_label),
			    g_markup_printf_escaped ("\u03B8<sub>1</sub>: %s<sup>o</sup>", "0"));
  RobotVars->updt_labels.phantom_jnt1_label=joystick_joint1_label;
  //joystick Y
  GtkWidget *joystick_joint2_label=gtk_label_new(NULL);
  gtk_label_set_markup (GTK_LABEL (joystick_joint2_label),
			    g_markup_printf_escaped ("\u03B8<sub>2</sub>: %s<sup>o</sup>", "0"));
  RobotVars->updt_labels.phantom_jnt2_label=joystick_joint2_label;
  //joystick Z
  GtkWidget *joystick_joint3_label=gtk_label_new(NULL);
  gtk_label_set_markup (GTK_LABEL (joystick_joint3_label),
			g_markup_printf_escaped ("\u03B8<sub>3</sub>: %s<sup>o</sup>", "0"));
  RobotVars->updt_labels.phantom_jnt3_label=joystick_joint3_label;
  //joystick gimbal angles label
  GtkWidget *joystick_gimbal_angles_label=gtk_label_new("PHANToM OMNI Gimbal Angles:");
  gtk_label_set_justify((GtkLabel *)joystick_gimbal_angles_label,GTK_JUSTIFY_CENTER);
  //hbox for joystick coordinates
  GtkWidget *joystick_gimbal_angles_hbox=gtk_hbox_new(FALSE, 5);
  //joystick gimbal1
  GtkWidget *joystick_gimbal1_label=gtk_label_new(NULL);
  gtk_label_set_markup (GTK_LABEL (joystick_gimbal1_label),
			g_markup_printf_escaped ("\u03B8<sub>4</sub>: %s<sup>o</sup>", "0"));
  RobotVars->updt_labels.phantom_gbl1_label=joystick_gimbal1_label;
  //joystick Y
  GtkWidget *joystick_gimbal2_label=gtk_label_new(NULL);
  gtk_label_set_markup (GTK_LABEL (joystick_gimbal2_label),
		      g_markup_printf_escaped ("\u03B8<sub>5</sub>: %s<sup>o</sup>", "0"));
  RobotVars->updt_labels.phantom_gbl2_label=joystick_gimbal2_label;
  //joystick Z
  GtkWidget *joystick_gimbal3_label=gtk_label_new(NULL);
  gtk_label_set_markup (GTK_LABEL (joystick_gimbal3_label),
		      g_markup_printf_escaped ("\u03B8<sub>6</sub>: %s<sup>o</sup>", "0"));
  RobotVars->updt_labels.phantom_gbl3_label=joystick_gimbal3_label;
  // motor temperature
  GtkWidget *motor_temp_label=gtk_label_new("PHANToM OMNI Motor Temperatures:\n[0...1] coolest to warmest");
  gtk_label_set_justify((GtkLabel *)motor_temp_label,GTK_JUSTIFY_CENTER);
  //hbox
  GtkWidget *motor_temp_hbox=gtk_hbox_new(FALSE, 5);
  // temperature labels
  GtkWidget *motor_temp_1_label=gtk_label_new("Motor 1: --- ");
  RobotVars->updt_labels.motor_1_temp=motor_temp_1_label;
  GtkWidget *motor_temp_2_label=gtk_label_new("Motor 2: --- ");
  RobotVars->updt_labels.motor_2_temp=motor_temp_2_label;
  GtkWidget *motor_temp_3_label=gtk_label_new("Motor 3: --- ");
  RobotVars->updt_labels.motor_3_temp=motor_temp_3_label;
  
  //   *****************************************
  
  
  /*~~~~~~~~~~~~~~~~~~~~~~~~ 
  || notebook haptic demos ||
  ~~~~~~~~~~~~~~~~~~~~~~~~~*/
  //*******************************************
  //humanoid image
  GtkWidget *image_humanoid=gtk_image_new();
  //use ROS environment to reach folder and buid string for image path
  string cmplt_img_path = ros::package::getPath("phua_haptic") + "/image/humanoid2.jpg";
  gtk_image_set_from_file((GtkImage *)image_humanoid,cmplt_img_path.c_str());
  //*******************************************
  
  // DEMO 1 (DEFAULT)
  // demo 1 frame
  GtkWidget *demo_workspace_limits_frame=gtk_frame_new(NULL);
  gtk_frame_set_shadow_type((GtkFrame *)demo_workspace_limits_frame,GTK_SHADOW_OUT);
  gtk_frame_set_label_align((GtkFrame *)demo_workspace_limits_frame,0.5,0.5);
  GtkWidget *label_demo_workspace_limits_TITLE=gtk_label_new(NULL);
  markup = g_markup_printf_escaped ("<span size=\"larger\"><u><b>%s</b></u></span>", "WORKSPACE LIMITS");
  gtk_label_set_markup (GTK_LABEL (label_demo_workspace_limits_TITLE), markup);
  gtk_frame_set_label_widget((GtkFrame *)demo_workspace_limits_frame, label_demo_workspace_limits_TITLE);
  // vbox for demo 1
  GtkWidget *vbox_demo_workspace_limits=gtk_vbox_new(FALSE, 3);
  // label for description
  char description[] = "\
    This demonstration allows the user\n\
  to command a robot kinematic model\n\
  with the ability to feel the workspace \n\
  limits rendered as a force vector.\n\
\n\
    The application renders an approach\n\
  force as the user reaches any one of\n\
  the workspace limit zones, and will\n\
  apply a step in force magnitude to\n\
  represent the end of the reachable\n\
  space for the selected kinematic\n\
  model.\n\
\n\
    This demonstration attempts to\n\
  exemplify how an haptic interface can\n\
  be used to control humanoid/industrial \n\
  robots giving the user an enhanced\n\
  perception of the robots workspace\n\
  limitations.\n";
  GtkWidget *label_demo_workspace_limits_description=gtk_label_new(description);
  gtk_label_set_justify((GtkLabel *)label_demo_workspace_limits_description,GTK_JUSTIFY_LEFT);
  // checkbox to select demonstration
  GtkWidget *demo_workspace_limits_checkbox=gtk_check_button_new_with_label("\n\nSELECT WORKSPACE LIMITS DEMO\n\n");
//   gtk_toggle_button_set_active((GtkToggleButton *)demo_workspace_limits_checkbox,TRUE);
  RobotVars->updt_labels.demo1_checkbox = demo_workspace_limits_checkbox;
  
  // DEMO 2
  // demo 2 frame
  GtkWidget *demo_plane_drawing_frame=gtk_frame_new(NULL);
  gtk_frame_set_shadow_type((GtkFrame *)demo_plane_drawing_frame,GTK_SHADOW_OUT);
  gtk_frame_set_label_align((GtkFrame *)demo_plane_drawing_frame,0.5,0.5);
  GtkWidget *label_demo_plane_drawing_TITLE=gtk_label_new(NULL);
  markup = g_markup_printf_escaped ("<span size=\"larger\"><u><b>%s</b></u></span>", "UA SUMMER ACADEMY: DRAWING");
  gtk_label_set_markup (GTK_LABEL (label_demo_plane_drawing_TITLE), markup);
  gtk_frame_set_label_widget((GtkFrame *)demo_plane_drawing_frame, label_demo_plane_drawing_TITLE);
  //vbox for demo 2
  GtkWidget *vbox_demo_plane_drawing=gtk_vbox_new(FALSE, 3);
  // label for description
  char description_2[] = "\
    This demonstration allows the user\n\
  to command a robot arm in order to\n\
  draw some lines on a vertical board.\n\
\n\
    The user is able to determine the\n\
  plane equation by giving the input of\n\
  three points belonging to the board.\n\
  This plane will then be used to create\n\
  a planar force field that resembles\n\
  the board plane.\n\
    This force field is then used as a\n\
  reference object to try and draw\n\
  simple shapes.\n\
\n\
    This demonstration attempts to show\n\
  how real world elements can be\n\
  virtualized as force fields in the\n\
  joystick workspace, allowing for a\n\
  better integration of the robot with\n\
  its surroundings.\n";
  GtkWidget *label_demo_plane_drawing_description=gtk_label_new(description_2);
  gtk_label_set_justify((GtkLabel *)label_demo_plane_drawing_description,GTK_JUSTIFY_LEFT);
  // checkbox to select demonstration
  GtkWidget *demo_plane_drawing_checkbox=gtk_check_button_new_with_label("\n\nSELECT PLANE DRAWING DEMO\n\n");
  gtk_toggle_button_set_active((GtkToggleButton *)demo_plane_drawing_checkbox, FALSE);
  RobotVars->updt_labels.demo2_checkbox = demo_plane_drawing_checkbox;
  
  // DEMO 3
  // demo 3 frame
  GtkWidget *demo_user_path_spring_force_frame=gtk_frame_new(NULL);
  gtk_frame_set_shadow_type((GtkFrame *)demo_user_path_spring_force_frame,GTK_SHADOW_OUT);
  gtk_frame_set_label_align((GtkFrame *)demo_user_path_spring_force_frame,0.5,0.5);
  GtkWidget *label_user_path_spring_force_TITLE=gtk_label_new(NULL);
  markup = g_markup_printf_escaped ("<span size=\"larger\"><u><b>%s</b></u></span>", "USER DEFINED PATH");
  gtk_label_set_markup (GTK_LABEL (label_user_path_spring_force_TITLE), markup);
  gtk_frame_set_label_widget((GtkFrame *)demo_user_path_spring_force_frame, label_user_path_spring_force_TITLE);
  //vbox for demo 3
  GtkWidget *vbox_demo_user_path_spring_force=gtk_vbox_new(FALSE, 3);
  // label for description
  char description_3[] = "\
    This demonstration allows the user\n\
  to store a set of points in the\n\
  workspace of the kinematic model\n\
  selected and set to robot to keep \n\
  reproducing them over time.\n\
\n\
    The user selects a point set to be\n\
  stored for reproduction by the robot.\n\
  The three dimensional points are then\n\
  executed in order running once or in\n\
  a loop.\n\
  \n\
    This demonstration is used to put\n\
  the robot executing defined paths,\n\
  for demonstration purposes.\n\
  \n";
  GtkWidget *label_demo_user_path_spring_force_description=gtk_label_new(description_3);
  gtk_label_set_justify((GtkLabel *)label_demo_user_path_spring_force_description,GTK_JUSTIFY_LEFT);
  
  // checkbox to select demonstration
  GtkWidget *demo_user_path_spring_force_checkbox=gtk_check_button_new_with_label("\n\nSELECT USER PATH DEMO\n\n");
  gtk_toggle_button_set_active((GtkToggleButton *)demo_user_path_spring_force_checkbox, FALSE);
  RobotVars->updt_labels.demo3_checkbox = demo_user_path_spring_force_checkbox;
  
  
    // DEMO 4
  // demo 4 frame
  GtkWidget *demo_detached_legs_balance_frame=gtk_frame_new(NULL);
  gtk_frame_set_shadow_type((GtkFrame *)demo_detached_legs_balance_frame,GTK_SHADOW_OUT);
  gtk_frame_set_label_align((GtkFrame *)demo_detached_legs_balance_frame,0.5,0.5);
  
  
  GtkWidget *label_detached_legs_balance_TITLE=gtk_label_new(NULL);
  markup = g_markup_printf_escaped ("<span size=\"larger\"><u><b>%s</b></u></span>", "LEG BALANCING");
  gtk_label_set_markup (GTK_LABEL (label_detached_legs_balance_TITLE), markup);
  gtk_frame_set_label_widget((GtkFrame *)demo_detached_legs_balance_frame, label_detached_legs_balance_TITLE);
  
  
  //vbox for demo 4
  GtkWidget *vbox_demo_detached_legs_balance=gtk_vbox_new(FALSE, 3);
  // label for description
  char description_4[] = "\
    This demonstration allows the user 	\n\
  to control the detached leg while the\n\
  system monitors its centre-of-gravity\n\
  (COG).\n\
  \n\
    During the control loop, the system\n\
  will signal the user using the joystick\n\
  when the centre-of-gravity projection,\n\
  the centre-of-pressure (COP) leaves the\n\
  area of the robots foot.\n\
  \n\
    This demonstration attempts to show \n\
  the possiblity of using the haptic feed-\n\
  back to collect enriched data of for \n\
  teaching purposes.\n\
					\n";
  GtkWidget *label_demo_detached_legs_balance_description=gtk_label_new(description_4);
  gtk_label_set_justify((GtkLabel *)label_demo_detached_legs_balance_description,GTK_JUSTIFY_LEFT);
  
  // checkbox to select demonstration
  GtkWidget *demo_detached_legs_balance_checkbox=gtk_check_button_new_with_label("\n\nSELECT LEG BALANCING DEMO\n\n");
  gtk_toggle_button_set_active((GtkToggleButton *)demo_detached_legs_balance_checkbox, FALSE);
  RobotVars->updt_labels.demo4_checkbox = demo_detached_legs_balance_checkbox;
  
  
  /*~~~~~~~~~~~~~~~~~~~~~~~~~ 
  ||     Connect signals     ||
    ~~~~~~~~~~~~~~~~~~~~~~~~~*/
  g_signal_connect(G_OBJECT(main_window), "destroy", G_CALLBACK(on_application_exit), RobotVars);
  g_signal_connect(G_OBJECT(button_quit), "clicked", G_CALLBACK(on_application_exit), RobotVars);
  g_signal_connect(G_OBJECT(notebook), "switch-page", G_CALLBACK(on_notebook_change_current_page), RobotVars);
  
  g_signal_connect(G_OBJECT(menu_item_back_facing), "activate",G_CALLBACK(on_back_facing_menu_item_activate), RobotVars);
  g_signal_connect(G_OBJECT(menu_item_front_facing), "activate",G_CALLBACK(on_front_facing_menu_item_activate), RobotVars);
  
  g_signal_connect(G_OBJECT(menu_item_quit), "activate",G_CALLBACK(on_application_exit), RobotVars);
  g_signal_connect(G_OBJECT(menu_item_about), "activate",G_CALLBACK(on_about_menu_item_activate), NULL);
  g_signal_connect(G_OBJECT(button_set_robot_home_pos), "clicked",G_CALLBACK(on_button_set_robot_home_pos_clicked),RobotVars);
  g_signal_connect(G_OBJECT(button_start_loop), "clicked",G_CALLBACK(on_button_start_loop_clicked), RobotVars);
  g_signal_connect(G_OBJECT(page1_left_top_frame_combobox), "changed",G_CALLBACK(on_page1_left_top_frame_combobox_changed_event),RobotVars);
  g_signal_connect(G_OBJECT(constant_speed_radio_checkbox), "toggled",G_CALLBACK(on_constant_speed_radio_checkbox_toggled), (gpointer) controled_speed_radio_checkbox);
  g_signal_connect(G_OBJECT(controled_speed_radio_checkbox), "toggled",G_CALLBACK(on_controled_speed_radio_checkbox_toggled), (gpointer) constant_speed_radio_checkbox);
  g_signal_connect(G_OBJECT(ajustament_hscale), "value-changed",G_CALLBACK(on_ajustament_hscale_value_changed_event),RobotVars);
  g_signal_connect(G_OBJECT(page2_select_id_combobox), "changed",G_CALLBACK(on_page2_select_id_combobox_for_position_changed_event), (gpointer) label_page_2_angle);
  g_signal_connect(G_OBJECT(button_release), "clicked",G_CALLBACK(on_button_release_all_clicked), RobotVars);
  g_signal_connect(G_OBJECT(button_go), "toggled",G_CALLBACK(on_button_go_toggled), RobotVars);
  g_signal_connect(G_OBJECT(button_stop), "toggled",G_CALLBACK(on_button_stop_toggled), RobotVars);
  
  g_signal_connect(G_OBJECT(button_setpos), "clicked",G_CALLBACK(on_button_setpos_clicked),RobotVars);
  g_signal_connect(G_OBJECT(spinbutton_angle), "value-changed", G_CALLBACK(on_button_setpos_clicked), RobotVars);
  
  g_signal_connect(G_OBJECT(button_set_pos_all), "clicked",G_CALLBACK(on_button_set_pos_all_clicked),RobotVars);
  g_signal_connect(G_OBJECT(button_setspeed), "clicked",G_CALLBACK(on_button_setspeed_clicked),RobotVars);
  
  g_signal_connect(G_OBJECT(spinbutton_speed), "value-changed", G_CALLBACK(on_button_setspeed_clicked), RobotVars);
  
  g_signal_connect(G_OBJECT(entry_pos_all), "activate", G_CALLBACK(on_button_set_pos_all_clicked), RobotVars);
  g_signal_connect(G_OBJECT(button_vbuttonbox_update_robot_data), "clicked",G_CALLBACK(on_button_vbuttonbox_update_robot_data_clicked),RobotVars);
  g_signal_connect(G_OBJECT(control_resolution_checkb_1mm), "toggled",G_CALLBACK(on_control_resolution_toggled), RobotVars);
  g_signal_connect(G_OBJECT(control_resolution_checkb_0_1mm), "toggled",G_CALLBACK(on_control_resolution_toggled), RobotVars);
  g_signal_connect(G_OBJECT(control_resolution_checkb_0_0_1mm), "toggled",G_CALLBACK(on_control_resolution_toggled), RobotVars);
  g_signal_connect(G_OBJECT(control_resolution_checkb_0_0_0_1mm), "toggled",G_CALLBACK(on_control_resolution_toggled), RobotVars);
  g_signal_connect(G_OBJECT(scale_1), "toggled",G_CALLBACK(on_workspace_scaling_toggled), RobotVars);
  g_signal_connect(G_OBJECT(scale_2), "toggled",G_CALLBACK(on_workspace_scaling_toggled), RobotVars);
  g_signal_connect(G_OBJECT(scale_3), "toggled",G_CALLBACK(on_workspace_scaling_toggled), RobotVars);
  g_signal_connect(G_OBJECT(scale_4), "toggled",G_CALLBACK(on_workspace_scaling_toggled), RobotVars);
  g_signal_connect(G_OBJECT(button_test_invkin), "clicked",G_CALLBACK(on_button_test_invkin_clicked),RobotVars);
  
  g_signal_connect(G_OBJECT(spinbutton_invkin_X), "value-changed", G_CALLBACK(on_button_test_invkin_clicked),RobotVars);
  g_signal_connect(G_OBJECT(spinbutton_invkin_Y), "value-changed", G_CALLBACK(on_button_test_invkin_clicked),RobotVars);
  g_signal_connect(G_OBJECT(spinbutton_invkin_Z), "value-changed", G_CALLBACK(on_button_test_invkin_clicked),RobotVars);
  
  g_signal_connect(G_OBJECT(select_inv_kin_combobox), "changed",G_CALLBACK(select_inv_kin_combobox_changed_event),RobotVars);
  
  g_signal_connect(G_OBJECT(button_calibration), "clicked",G_CALLBACK(on_button_calibration_clicked),RobotVars);
  g_signal_connect(G_OBJECT(demo_workspace_limits_checkbox), "toggled",G_CALLBACK(on_demo_checkboxes_toggled), RobotVars);
  g_signal_connect(G_OBJECT(demo_plane_drawing_checkbox), "toggled",G_CALLBACK(on_demo_checkboxes_toggled), RobotVars);
  g_signal_connect(G_OBJECT(demo_user_path_spring_force_checkbox), "toggled",G_CALLBACK(on_demo_checkboxes_toggled), RobotVars);
  g_signal_connect(G_OBJECT(demo_detached_legs_balance_checkbox), "toggled",G_CALLBACK(on_demo_checkboxes_toggled), RobotVars);
  
  g_signal_connect(G_OBJECT(user_path_demo_point_store_button), "clicked",G_CALLBACK(on_user_path_demo_point_store_button_clicked), RobotVars);
  
  g_signal_connect(G_OBJECT(user_path_demo_run_once_checkbox), "toggled",G_CALLBACK(user_path_demo_run_checkbox_toggled), RobotVars);
  g_signal_connect(G_OBJECT(user_path_demo_run_loop_checkbox), "toggled",G_CALLBACK(user_path_demo_run_checkbox_toggled), RobotVars);
  
  g_signal_connect(G_OBJECT(user_path_demo_run_button), "clicked",G_CALLBACK(on_user_path_demo_run_button_clicked), RobotVars);
  g_signal_connect(G_OBJECT(user_path_demo_clear_button), "clicked",G_CALLBACK(on_user_path_demo_clear_button_clicked), RobotVars);
  
  
  /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 
  || box pack/container addings ||
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // main page haptics data
  gtk_box_pack_start(GTK_BOX(vbox_phantom), page1_haptics_data_frame,TRUE, TRUE, 0);
  gtk_container_add(GTK_CONTAINER(page1_haptics_data_frame), vbox_haptic_information);
  gtk_box_pack_start(GTK_BOX(vbox_haptic_information), hbox_workspace_interaction,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(hbox_workspace_interaction), label_workspace_interaction,TRUE, TRUE, 0);
  
  gtk_box_pack_start(GTK_BOX(vbox_haptic_information), hbox_drawing_demo,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(hbox_drawing_demo), label_drawing_demo,TRUE, TRUE, 0);
  
  gtk_box_pack_start(GTK_BOX(vbox_haptic_information), hbox_leg_balancing_demo,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(hbox_leg_balancing_demo), label_leg_balancing_cog_demo,TRUE, TRUE, 0);
  
  gtk_box_pack_start(GTK_BOX(vbox_haptic_information), hbox_user_path_demo,FALSE, FALSE, 0);
  gtk_box_pack_start(GTK_BOX(hbox_user_path_demo), label_user_path_demo,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(hbox_user_path_demo), user_path_demo_point_store_button,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(hbox_user_path_demo), vbox_user_path_run_options,TRUE, TRUE, 0);
  
  gtk_box_pack_start(GTK_BOX(hbox_user_path_checkboxes), user_path_demo_run_once_checkbox,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(hbox_user_path_checkboxes), user_path_demo_run_loop_checkbox,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(vbox_user_path_run_options), user_path_demo_run_button,FALSE, FALSE, 0);
  gtk_box_pack_start(GTK_BOX(vbox_user_path_run_options), hbox_user_path_checkboxes,FALSE, FALSE, 0);
  
  gtk_box_pack_start(GTK_BOX(hbox_user_path_demo), user_path_demo_clear_button,TRUE, TRUE, 0);
  
  
  gtk_box_pack_start(GTK_BOX(vbox_haptic_information), gtk_hseparator_new(),FALSE, FALSE, 0);
  gtk_box_pack_start(GTK_BOX(vbox_haptic_information), label_force_feedback,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(vbox_haptic_information), vbox_force_feedback,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(vbox_force_feedback), hbox_haptic_x_force,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(vbox_force_feedback), gtk_hseparator_new(), TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(vbox_force_feedback), hbox_haptic_y_force,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(vbox_force_feedback), gtk_hseparator_new(), TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(vbox_force_feedback), hbox_haptic_z_force,TRUE, TRUE, 0);
  string max = "+ 3.3 N";
  string min = "- 3.3 N";
  gtk_box_pack_start(GTK_BOX(hbox_haptic_x_force), gtk_label_new(min.c_str()),TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(hbox_haptic_x_force), x_neg_force_bar,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(hbox_haptic_x_force), label_x_force,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(hbox_haptic_x_force), x_force_bar,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(hbox_haptic_x_force), gtk_label_new(max.c_str()),TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(hbox_haptic_y_force), gtk_label_new(min.c_str()),TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(hbox_haptic_y_force), y_neg_force_bar,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(hbox_haptic_y_force), label_y_force,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(hbox_haptic_y_force), y_force_bar,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(hbox_haptic_y_force), gtk_label_new(max.c_str()),TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(hbox_haptic_z_force), gtk_label_new(min.c_str()),TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(hbox_haptic_z_force), z_neg_force_bar,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(hbox_haptic_z_force), label_z_force,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(hbox_haptic_z_force), z_force_bar,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(hbox_haptic_z_force), gtk_label_new(max.c_str()),TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(vbox_force_feedback), label_force_magnitude,TRUE, TRUE, 0);
  // main page | control parameters
  gtk_box_pack_start(GTK_BOX(vbox_phantom), page1_control_parameters_frame,TRUE, TRUE, 0);
  gtk_container_add(GTK_CONTAINER(page1_control_parameters_frame), vbox_control_parameters);
  gtk_box_pack_start(GTK_BOX(vbox_control_parameters), label_control_resolution,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(vbox_control_parameters), hbox_control_parameters,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(hbox_control_parameters), control_resolution_checkb_1mm,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(hbox_control_parameters), control_resolution_checkb_0_1mm,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(hbox_control_parameters), control_resolution_checkb_0_0_1mm,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(hbox_control_parameters), control_resolution_checkb_0_0_0_1mm,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(vbox_control_parameters), gtk_hseparator_new(), FALSE, FALSE, 0);
  gtk_box_pack_start(GTK_BOX(vbox_control_parameters), label_workspace_scale,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(vbox_control_parameters), hbox_workspace_scale,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(hbox_workspace_scale), scale_1,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(hbox_workspace_scale), scale_2,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(hbox_workspace_scale), scale_3,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(hbox_workspace_scale), scale_4,TRUE, TRUE, 0);
  gtk_container_add(GTK_CONTAINER(page1_middle_frame), vbox_phantom);
  // page miscellaneous | misc control funcs
  gtk_box_pack_start(GTK_BOX(page2_vbox_funcs), label_page_2_setpos,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(page2_vbox_funcs), page2_setpos_hbox,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(page2_setpos_hbox), page2_select_id_combobox,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(page2_setpos_hbox), gtk_vseparator_new(), TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(page2_setpos_hbox), label_page_2_joint_angle,FALSE, FALSE, 0);
  gtk_box_pack_start(GTK_BOX(page2_setpos_hbox), spinbutton_angle,FALSE, FALSE, 0);
  gtk_box_pack_start(GTK_BOX(page2_setpos_hbox), label_page_2_angle,FALSE, FALSE, 0);
  gtk_box_pack_start(GTK_BOX(page2_setpos_hbox), gtk_vseparator_new(), TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(page2_setpos_hbox), button_setpos,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(page2_vbox_funcs), gtk_hseparator_new(),TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(page2_vbox_funcs), label_page_2_setspeed,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(page2_vbox_funcs), page2_setspeed_hbox,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(page2_setspeed_hbox), page2_select_id_combobox2,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(page2_setspeed_hbox), gtk_vseparator_new(), TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(page2_setspeed_hbox), label_page_2_speed_entry,FALSE, FALSE, 0);
  gtk_box_pack_start(GTK_BOX(page2_setspeed_hbox), spinbutton_speed,FALSE, FALSE, 0);
  gtk_box_pack_start(GTK_BOX(page2_setspeed_hbox), label_page_2_speed,FALSE, FALSE, 0);
  gtk_box_pack_start(GTK_BOX(page2_setspeed_hbox), gtk_vseparator_new(), TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(page2_setspeed_hbox), button_setspeed,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(page2_setspeed_hbox), label_page_2_pos_read,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(page2_vbox_funcs), gtk_hseparator_new(),TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(page2_vbox_funcs), label_page_2_set_pos_all,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(page2_set_pos_all_hbox), label_page_2_setposall, FALSE, FALSE, 0);
  gtk_box_pack_start(GTK_BOX(page2_set_pos_all_hbox), entry_pos_all, FALSE, FALSE, 0);
  gtk_box_pack_start(GTK_BOX(page2_set_pos_all_hbox), label_page_2_limits, FALSE, FALSE, 0);
  gtk_box_pack_start(GTK_BOX(page2_set_pos_all_hbox), gtk_vseparator_new(), TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(page2_set_pos_all_hbox), button_set_pos_all,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(page2_vbox_funcs), page2_set_pos_all_hbox,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(page2_vbox_funcs), gtk_hseparator_new(),TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(page2_vbox_funcs), label_page_2_invkin,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(page2_vbox_funcs), page2_invkin_hbox,TRUE, TRUE, 0);
  
  gtk_box_pack_start(GTK_BOX(page2_invkin_hbox), select_inv_kin_combobox,TRUE, TRUE, 0);
  
  gtk_box_pack_start(GTK_BOX(page2_invkin_hbox), label_invkin_X,TRUE, TRUE, 0);
  
//   gtk_box_pack_start(GTK_BOX(page2_invkin_hbox), entry_invkin_X,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(page2_invkin_hbox), spinbutton_invkin_X,TRUE, TRUE, 0);
  
  gtk_box_pack_start(GTK_BOX(page2_invkin_hbox), label_mm_1,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(page2_invkin_hbox), gtk_vseparator_new(),TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(page2_invkin_hbox), label_invkin_Y,TRUE, TRUE, 0);
  
//   gtk_box_pack_start(GTK_BOX(page2_invkin_hbox), entry_invkin_Y,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(page2_invkin_hbox), spinbutton_invkin_Y,TRUE, TRUE, 0);
  
  gtk_box_pack_start(GTK_BOX(page2_invkin_hbox), label_mm_2,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(page2_invkin_hbox), gtk_vseparator_new(),TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(page2_invkin_hbox), label_invkin_Z,TRUE, TRUE, 0);
  
//   gtk_box_pack_start(GTK_BOX(page2_invkin_hbox), entry_invkin_Z,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(page2_invkin_hbox), spinbutton_invkin_Z,TRUE, TRUE, 0);
  
  gtk_box_pack_start(GTK_BOX(page2_invkin_hbox), label_mm_3,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(page2_invkin_hbox), gtk_vseparator_new(),TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(page2_invkin_hbox), button_test_invkin,TRUE, TRUE, 0);
  
  gtk_box_pack_start(GTK_BOX(page2_vbox_funcs), gtk_hseparator_new(),TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(page2_vbox_funcs), label_page_2_utils,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(page2_vbox_funcs), page2_utilities_hbox,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(page2_utilities_hbox), button_release,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(page2_utilities_hbox), button_go,TRUE, TRUE, 0);
  gtk_box_pack_end(GTK_BOX(page2_utilities_hbox), button_stop,TRUE, TRUE, 0);
  gtk_container_add(GTK_CONTAINER(page2_left_frame), page2_vbox_funcs);
  gtk_box_pack_start(GTK_BOX(page2_main_hbox), page2_left_frame,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(page2_main_hbox), page2_right_frame,TRUE, TRUE, 0);
  // page miscellaneous | calibration
  gtk_container_add(GTK_CONTAINER(page2_right_frame), page2_vbox_phantom);
  
  gtk_box_pack_start(GTK_BOX(page2_vbox_phantom), label_calibration, FALSE, FALSE, 0);
  gtk_box_pack_start(GTK_BOX(page2_vbox_phantom), button_calibration, TRUE, TRUE, 0);
  
  // page miscellaneous | phantom raw data
  gtk_box_pack_start(GTK_BOX(page2_vbox_phantom), page2_joystick_raw_data_frame,TRUE, TRUE, 0);
  gtk_container_add(GTK_CONTAINER(page2_joystick_raw_data_frame), vbox_phantom_raw);
  gtk_box_pack_start(GTK_BOX(vbox_phantom_raw), joystick_position_label,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(vbox_phantom_raw), joystick_position_hbox,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(joystick_position_hbox), joystick_X_label,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(joystick_position_hbox), gtk_vseparator_new(), FALSE, FALSE, 0);
  gtk_box_pack_start(GTK_BOX(joystick_position_hbox), joystick_Y_label,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(joystick_position_hbox), gtk_vseparator_new(), FALSE, FALSE, 0);
  gtk_box_pack_start(GTK_BOX(joystick_position_hbox), joystick_Z_label,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(vbox_phantom_raw), gtk_hseparator_new(),TRUE, TRUE, 0);
  
  gtk_box_pack_start(GTK_BOX(vbox_phantom_raw), joystick_pen_tip_position_label,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(vbox_phantom_raw), joystick_pen_tip_position_hbox,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(joystick_pen_tip_position_hbox), joystick_pen_tip_X_label,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(joystick_pen_tip_position_hbox), gtk_vseparator_new(), FALSE, FALSE, 0);
  gtk_box_pack_start(GTK_BOX(joystick_pen_tip_position_hbox), joystick_pen_tip_Y_label,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(joystick_pen_tip_position_hbox), gtk_vseparator_new(), FALSE, FALSE, 0);
  gtk_box_pack_start(GTK_BOX(joystick_pen_tip_position_hbox), joystick_pen_tip_Z_label,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(vbox_phantom_raw), gtk_hseparator_new(),TRUE, TRUE, 0);
  
  gtk_box_pack_start(GTK_BOX(vbox_phantom_raw), joystick_speed_label,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(vbox_phantom_raw), joystick_speed_hbox,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(joystick_speed_hbox), joystick_speed_values_label,TRUE, TRUE, 0);
  
  gtk_box_pack_start(GTK_BOX(vbox_phantom_raw), gtk_hseparator_new(),TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(vbox_phantom_raw), joystick_joint_angles_label,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(vbox_phantom_raw), joystick_joint_angles_hbox,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(joystick_joint_angles_hbox), joystick_joint1_label,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(joystick_joint_angles_hbox), gtk_vseparator_new(), FALSE, FALSE, 0);
  gtk_box_pack_start(GTK_BOX(joystick_joint_angles_hbox), joystick_joint2_label,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(joystick_joint_angles_hbox), gtk_vseparator_new(), FALSE, FALSE, 0);
  gtk_box_pack_start(GTK_BOX(joystick_joint_angles_hbox), joystick_joint3_label,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(vbox_phantom_raw), gtk_hseparator_new(),TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(vbox_phantom_raw), joystick_gimbal_angles_label,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(vbox_phantom_raw), joystick_gimbal_angles_hbox,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(joystick_gimbal_angles_hbox), joystick_gimbal1_label,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(joystick_gimbal_angles_hbox), gtk_vseparator_new(), FALSE, FALSE, 0);
  gtk_box_pack_start(GTK_BOX(joystick_gimbal_angles_hbox), joystick_gimbal2_label,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(joystick_gimbal_angles_hbox), gtk_vseparator_new(), FALSE, FALSE, 0);
  gtk_box_pack_start(GTK_BOX(joystick_gimbal_angles_hbox), joystick_gimbal3_label,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(vbox_phantom_raw), gtk_hseparator_new(),TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(vbox_phantom_raw), motor_temp_label,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(vbox_phantom_raw), motor_temp_hbox,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(motor_temp_hbox), motor_temp_1_label,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(motor_temp_hbox), gtk_vseparator_new(), FALSE, FALSE, 0);
  gtk_box_pack_start(GTK_BOX(motor_temp_hbox), motor_temp_2_label,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(motor_temp_hbox), gtk_vseparator_new(), FALSE, FALSE, 0);
  gtk_box_pack_start(GTK_BOX(motor_temp_hbox), motor_temp_3_label,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(motor_temp_hbox), gtk_vseparator_new(), FALSE, FALSE, 0);
  // robot data | main page
  gtk_box_pack_start(GTK_BOX(page1_left_top_frame_vbox), label_coordinates,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(hbox_coordinates_left_arm), label_X,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(hbox_coordinates_left_arm), gtk_vseparator_new(), FALSE, FALSE, 0);
  gtk_box_pack_start(GTK_BOX(hbox_coordinates_left_arm), label_Y,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(hbox_coordinates_left_arm), gtk_vseparator_new(), FALSE, FALSE, 0);
  gtk_box_pack_start(GTK_BOX(hbox_coordinates_left_arm), label_Z,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(hbox_coordinates_left_arm), gtk_vseparator_new(), FALSE, FALSE, 0);
  gtk_box_pack_start(GTK_BOX(hbox_coordinates_left_arm), label_roll,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(hbox_coordinates_left_arm), gtk_vseparator_new(), FALSE, FALSE, 0);
  gtk_box_pack_start(GTK_BOX(hbox_coordinates_left_arm), label_pitch,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(hbox_coordinates_left_arm), gtk_vseparator_new(), FALSE, FALSE, 0);
  gtk_box_pack_start(GTK_BOX(hbox_coordinates_left_arm), label_yaw,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(page1_left_top_frame_vbox), hbox_coordinates_left_arm,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(page1_left_top_frame_vbox), gtk_hseparator_new(),TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(page1_left_top_frame_vbox), label_coordinates_2,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(hbox_coordinates_right_arm), label_X_2,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(hbox_coordinates_right_arm), gtk_vseparator_new(), FALSE, FALSE, 0);
  gtk_box_pack_start(GTK_BOX(hbox_coordinates_right_arm), label_Y_right,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(hbox_coordinates_right_arm), gtk_vseparator_new(), FALSE, FALSE, 0);
  gtk_box_pack_start(GTK_BOX(hbox_coordinates_right_arm), label_Z_right,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(hbox_coordinates_right_arm), gtk_vseparator_new(), FALSE, FALSE, 0);
  gtk_box_pack_start(GTK_BOX(hbox_coordinates_right_arm), label_roll_right,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(hbox_coordinates_right_arm), gtk_vseparator_new(), FALSE, FALSE, 0);
  gtk_box_pack_start(GTK_BOX(hbox_coordinates_right_arm), label_pitch_right,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(hbox_coordinates_right_arm), gtk_vseparator_new(), FALSE, FALSE, 0);
  gtk_box_pack_start(GTK_BOX(hbox_coordinates_right_arm), label_yaw_right,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(page1_left_top_frame_vbox), hbox_coordinates_right_arm,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(page1_left_top_frame_vbox), gtk_hseparator_new(),TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(page1_left_top_frame_vbox), label_joint,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(page1_left_top_frame_vbox), label_head,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(hbox_arm_jnts), label_right_arm,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(hbox_arm_jnts), gtk_vseparator_new(), FALSE, FALSE, 0);
  gtk_box_pack_start(GTK_BOX(hbox_arm_jnts), label_left_arm,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(page1_left_top_frame_vbox), hbox_arm_jnts,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(page1_left_top_frame_vbox), label_torso,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(hbox_leg_jnts), label_right_leg,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(hbox_leg_jnts), gtk_vseparator_new(), FALSE, FALSE, 0);
  gtk_box_pack_start(GTK_BOX(hbox_leg_jnts), label_left_leg,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(page1_left_top_frame_vbox), hbox_leg_jnts,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(page1_left_top_frame_vbox), gtk_hseparator_new(),TRUE, TRUE, 0);
  //main page control options
  gtk_box_pack_start(GTK_BOX(page1_left_bottom_frame_vbox), constant_speed_radio_checkbox,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(page1_left_bottom_frame_vbox), hbox_scrll_lbl,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(hbox_scrll_lbl), label_speed_scale_bar,FALSE, FALSE, 0);
  gtk_box_pack_start(GTK_BOX(hbox_scrll_lbl), set_speed_hscale,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(page1_left_bottom_frame_vbox), controled_speed_radio_checkbox,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(hbox_prog_bars_auto_speed), arm_theta1_speed_prog_bar,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(hbox_prog_bars_auto_speed), arm_theta2_speed_prog_bar,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(hbox_prog_bars_auto_speed), arm_theta3_speed_prog_bar,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(page1_left_bottom_frame_vbox), hbox_prog_bars_auto_speed,TRUE, TRUE, 0);
  gtk_container_add(GTK_CONTAINER(page1_left_top_frame), page1_left_top_frame_vbox);
  gtk_container_add(GTK_CONTAINER(page1_left_bottom_frame), page1_left_bottom_frame_vbox);
  gtk_box_pack_start(GTK_BOX(notebook_vbox_left_page1), page1_left_top_frame,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(notebook_vbox_left_page1), page1_left_bottom_frame,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(page1_main_hbox), notebook_vbox_left_page1,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(page1_main_hbox), page1_middle_frame,TRUE, TRUE, 0);
  //main page bottom buttons
  gtk_box_pack_start(GTK_BOX(page1_main_vertbox), bottom_buttons_hbox,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(bottom_buttons_hbox), button_start_loop,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(bottom_buttons_hbox), gtk_vseparator_new(),FALSE, FALSE, 0);
  gtk_box_pack_start(GTK_BOX(bottom_buttons_hbox), button_vbuttonbox_update_robot_data,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(bottom_buttons_hbox), gtk_vseparator_new(),FALSE, FALSE, 0);
  gtk_box_pack_start(GTK_BOX(bottom_buttons_hbox), button_set_robot_home_pos,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(bottom_buttons_hbox), gtk_vseparator_new(),FALSE, FALSE, 0);
  gtk_box_pack_start(GTK_BOX(bottom_buttons_hbox), button_quit,TRUE, TRUE, 0);
  //notebook page 3
  gtk_box_pack_start(GTK_BOX(page3_main_hbox_arm_demos), demo_workspace_limits_frame ,TRUE, TRUE, 0);
  gtk_container_add(GTK_CONTAINER(demo_workspace_limits_frame), vbox_demo_workspace_limits);
  gtk_box_pack_start(GTK_BOX(vbox_demo_workspace_limits), label_demo_workspace_limits_description ,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(vbox_demo_workspace_limits), gtk_hseparator_new() ,FALSE, FALSE, 0);
  gtk_box_pack_start(GTK_BOX(vbox_demo_workspace_limits), demo_workspace_limits_checkbox ,FALSE, FALSE, 0);
  gtk_box_pack_start(GTK_BOX(page3_main_hbox_arm_demos), gtk_vseparator_new() , FALSE, FALSE, 0);
  gtk_box_pack_start(GTK_BOX(page3_main_hbox_arm_demos), demo_plane_drawing_frame ,TRUE, TRUE, 0);
  gtk_container_add(GTK_CONTAINER(demo_plane_drawing_frame), vbox_demo_plane_drawing);
  gtk_box_pack_start(GTK_BOX(vbox_demo_plane_drawing), label_demo_plane_drawing_description ,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(vbox_demo_plane_drawing), gtk_hseparator_new() ,FALSE, FALSE, 0);
  gtk_box_pack_start(GTK_BOX(vbox_demo_plane_drawing), demo_plane_drawing_checkbox ,FALSE, FALSE, 0);
  gtk_box_pack_start(GTK_BOX(page3_main_hbox_arm_demos), gtk_vseparator_new() , FALSE, FALSE, 0);
  gtk_box_pack_start(GTK_BOX(page3_main_hbox_arm_demos), demo_user_path_spring_force_frame ,TRUE, TRUE, 0);
  gtk_container_add(GTK_CONTAINER(demo_user_path_spring_force_frame), vbox_demo_user_path_spring_force);
  gtk_box_pack_start(GTK_BOX(vbox_demo_user_path_spring_force), label_demo_user_path_spring_force_description,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(vbox_demo_user_path_spring_force), gtk_hseparator_new() ,FALSE, FALSE, 0);
  gtk_box_pack_start(GTK_BOX(vbox_demo_user_path_spring_force), demo_user_path_spring_force_checkbox,FALSE, FALSE, 0);
  gtk_box_pack_start(GTK_BOX(page3_main_hbox_arm_demos), gtk_vseparator_new() , FALSE, FALSE, 0);
  gtk_box_pack_start(GTK_BOX(page3_main_hbox_arm_demos), demo_detached_legs_balance_frame ,TRUE, TRUE, 0);
  gtk_container_add(GTK_CONTAINER(demo_detached_legs_balance_frame), vbox_demo_detached_legs_balance);
  gtk_box_pack_start(GTK_BOX(vbox_demo_detached_legs_balance), label_demo_detached_legs_balance_description,TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(vbox_demo_detached_legs_balance), gtk_hseparator_new() ,FALSE, FALSE, 0);
  gtk_box_pack_start(GTK_BOX(vbox_demo_detached_legs_balance), demo_detached_legs_balance_checkbox,FALSE, FALSE, 0);
  
//   gtk_box_pack_start(GTK_BOX(page3_main_hbox_arm_demos), gtk_vseparator_new() , FALSE, FALSE, 0);
//   gtk_box_pack_start(GTK_BOX(page3_main_hbox_arm_demos), image_humanoid ,TRUE, TRUE, 0);
  
  //program window addings
  gtk_box_pack_start(GTK_BOX(main_vbox), menu_bar, FALSE, FALSE, 0);
  gtk_box_pack_start(GTK_BOX(main_vbox), notebook, TRUE, TRUE, 0);
  gtk_box_pack_end(GTK_BOX(main_vbox), statusbar, FALSE, TRUE, 1);
  gtk_container_add(GTK_CONTAINER(main_window), main_vbox);
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  
  //run to initialize pointers
  on_page1_left_top_frame_combobox_changed_event((GtkWidget *)page1_left_top_frame_combobox,(gpointer)button_start_loop);
  //run to initialize speed
  on_ajustament_hscale_value_changed_event((GtkAdjustment *)ajustament_hscale, (gpointer)RobotVars);
  //run to choose front-facing control positioning
  on_front_facing_menu_item_activate((GtkMenuItem *)menu_item_front_facing, (gpointer)RobotVars);
  
  /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 
  ||        show widgets        ||
  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
  gtk_widget_show_all(main_vbox);
  gtk_widget_show_all(main_window);
  
  //run to hide progress bar
  on_controled_speed_radio_checkbox_toggled((GtkWidget *)controled_speed_radio_checkbox,(gpointer) constant_speed_radio_checkbox);  
  
  gtk_widget_set_visible((GtkWidget*)hbox_drawing_demo, FALSE);
  gtk_widget_set_visible((GtkWidget*)hbox_leg_balancing_demo, FALSE);
  gtk_widget_set_visible((GtkWidget*)hbox_user_path_demo, FALSE);
  
  //launch label update watcher
  g_timeout_add((1000/RobotVars->parameters.graphics_refresh_rate),update_watcher,RobotVars);
  
  cout<<"* User interface built."<<endl;
  //start gtk loop
  gtk_main();
  RobotVars->parameters.exit_status = TRUE;
  g_free(markup);
  return NULL;
}