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
 * @file ptu_interactive_control.cpp
 * @brief This code publishes rviz interactive markers for ptu direct control
 * @author Miguel Oliveira
 * @version v0
 * @date 2012-03-02
 */

#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
//commented next line isince it appears not needed and was not found in path! V. Santos, 24-Mai-2013,10:51
//#include <interactive_markers/interactive_marker_client.h>
#include <interactive_markers/menu_handler.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <math.h>
#include <sensor_msgs/JointState.h>

using namespace visualization_msgs;


// %Tag(vars)%
boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
float marker_pos = 0;
interactive_markers::MenuHandler menu_handler;
ros::Publisher pub;
// %EndTag(vars)%

// %Tag(frameCallback)%
void frameCallback(const ros::TimerEvent&)
{
	//static uint32_t counter = 0;

	//static tf::TransformBroadcaster br;

	//tf::Transform t;

	//ros::Time time = ros::Time::now();

	//t.setOrigin(tf::Vector3(0.0, 0.0, sin(float(counter)/140.0) * 2.0));
	//t.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
	//br.sendTransform(tf::StampedTransform(t, time, "base_link", "moving_frame"));

	//t.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
	//t.setRotation(tf::createQuaternionFromRPY(0.0, float(counter)/140.0, 0.0));
	//br.sendTransform(tf::StampedTransform(t, time, "base_link", "rotating_frame"));

	//++counter;
}
// %EndTag(frameCallback)%

// %Tag(processFeedback)%
void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
	std::ostringstream s;
	s << "Feedback from marker '" << feedback->marker_name << "' "
		<< " / control '" << feedback->control_name << "'";

	std::ostringstream mouse_point_ss;
	if( feedback->mouse_point_valid )
	{
		mouse_point_ss << " at " << feedback->mouse_point.x
			<< ", " << feedback->mouse_point.y
			<< ", " << feedback->mouse_point.z
			<< " in frame " << feedback->header.frame_id;
	}

	switch ( feedback->event_type )
	{
		//case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
		//ROS_INFO_STREAM( s.str() << ": button click" << mouse_point_ss.str() << "." );
		//break;

		//case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
		//ROS_INFO_STREAM( s.str() << ": menu item " << feedback->menu_entry_id << " clicked" << mouse_point_ss.str() << "." );
		//break;

		case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:

			tf::Quaternion q;
			q[0] = feedback->pose.orientation.x;
			q[1] = feedback->pose.orientation.y;
			q[2] = feedback->pose.orientation.z;
			q[3] = feedback->pose.orientation.w;
			tf::Matrix3x3 M(q);                                                                                                                
			double r,p,y;
#if ROS_VERSION_MINIMUM(1, 8, 0) // if current ros version is >= 1.8.0
			M.getEulerYPR(y, p, r); //Changed by V. Santos, 15-Abr-2013,09:25
#else // Code for earlier versions (Electric, diamondback, ...)
			M.getRPY(r,p,y);
#endif

			sensor_msgs::JointState joint_state;
			joint_state.header.stamp = ros::Time::now();
			joint_state.name.resize(2);
			joint_state.position.resize(2);
			joint_state.name[0] =ros::names::remap("pan");
			joint_state.position[0] = y;
			joint_state.name[1] =ros::names::remap("tilt");
			joint_state.position[1] = p;
			pub.publish(joint_state);

			//btQuaternion q = feedback->pose.orientation;	

			ROS_INFO_STREAM( s.str() << ": pose changed"
			<< "\nposition = "
			<< feedback->pose.position.x
			<< ", " << feedback->pose.position.y
			<< ", " << feedback->pose.position.z
			<< "\norientation = "
			<< feedback->pose.orientation.w
			<< ", " << feedback->pose.orientation.x
			<< ", " << feedback->pose.orientation.y
			<< ", " << feedback->pose.orientation.z
			<< "\nframe: " << feedback->header.frame_id
			<< " time: " << feedback->header.stamp.sec << "sec, "
			<< feedback->header.stamp.nsec << " nsec" );
			break;

			//case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
			//ROS_INFO_STREAM( s.str() << ": mouse down" << mouse_point_ss.str() << "." );
			//break;

			//case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
			//ROS_INFO_STREAM( s.str() << ": mouse up" << mouse_point_ss.str() << "." );
			//break;
	}

	server->applyChanges();
}
// %EndTag(processFeedback)%

// %Tag(alignMarker)%
void alignMarker( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
	geometry_msgs::Pose pose = feedback->pose;

	pose.position.x = round(pose.position.x-0.5)+0.5;
	pose.position.y = round(pose.position.y-0.5)+0.5;

	ROS_INFO_STREAM( feedback->marker_name << ":"
			<< " aligning position = "
			<< feedback->pose.position.x
			<< ", " << feedback->pose.position.y
			<< ", " << feedback->pose.position.z
			<< " to "
			<< pose.position.x
			<< ", " << pose.position.y
			<< ", " << pose.position.z );

	server->setPose( feedback->marker_name, pose );
	server->applyChanges();
}
// %EndTag(alignMarker)%

double rand( double min, double max )
{
	double t = (double)rand() / (double)RAND_MAX;
	return min + t*(max-min);
}

// %Tag(Box)%
Marker makeBox( InteractiveMarker &msg )
{
	Marker marker;

	marker.type = Marker::SPHERE;
	marker.scale.x = msg.scale * 0.45;
	marker.scale.y = msg.scale * 0.45;
	marker.scale.z = msg.scale * 0.45;
	marker.color.r = 0;
	marker.color.g = 0.7;
	marker.color.b = 0;
	marker.color.a = 0.6;

	return marker;
}

InteractiveMarkerControl& makeBoxControl( InteractiveMarker &msg )
{
	InteractiveMarkerControl control;
	control.always_visible = true;
	control.markers.push_back( makeBox(msg) );
	msg.controls.push_back( control );

	return msg.controls.back();
}
// %EndTag(Box)%

void saveMarker( InteractiveMarker int_marker )
{
	server->insert(int_marker);
	server->setCallback(int_marker.name, &processFeedback);
}

////////////////////////////////////////////////////////////////////////////////////

// %Tag(6DOF)%
void make6DofMarker( bool fixed )
{
	InteractiveMarker int_marker;
	int_marker.header.frame_id = "/atc/vehicle/center_bumper";
	int_marker.pose.position.x = 2;
	int_marker.pose.position.y = 0;
	int_marker.pose.position.z = 0.5;
	int_marker.scale = 0.5;

	int_marker.name = "Fovetation Control Target";
	int_marker.description = "Foveation_control will try \nto aim the PTU towards\n the marker";

	// insert a box
	makeBoxControl(int_marker);
	InteractiveMarkerControl control;

	if ( fixed )
	{
		//int_marker.name += "_fixed";
		//int_marker.description += "\n(fixed orientation)";
		//control.orientation_mode = InteractiveMarkerControl::FIXED;
	}

	control.orientation.w = 1;
	control.orientation.x = 1;
	control.orientation.y = 0;
	control.orientation.z = 0;
	//control.name = "rotate_x";
	//control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
	//int_marker.controls.push_back(control);
	control.name = "move_x";
	control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
	int_marker.controls.push_back(control);

	control.orientation.w = 1;
	control.orientation.x = 0;
	control.orientation.y = 1;
	control.orientation.z = 0;
	//control.name = "rotate_z";
	//control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
	//int_marker.controls.push_back(control);
	control.name = "move_z";
	control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
	int_marker.controls.push_back(control);

	control.orientation.w = 1;
	control.orientation.x = 0;
	control.orientation.y = 0;
	control.orientation.z = 1;
	//control.name = "rotate_y";
	//control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
	//int_marker.controls.push_back(control);
	control.name = "move_y";
	control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
	int_marker.controls.push_back(control);

	server->insert(int_marker);
	server->setCallback(int_marker.name, &processFeedback);
}
// %EndTag(6DOF)%

// %Tag(RandomDof)%
void makeRandomDofMarker( )
{
	InteractiveMarker int_marker;
	int_marker.header.frame_id = "/base_link";
	int_marker.pose.position.y = -3.0 * marker_pos++;;
	int_marker.scale = 1;

	int_marker.name = "6dof_random_axes";
	int_marker.description = "6-DOF\n(Arbitrary Axes)";

	makeBoxControl(int_marker);

	InteractiveMarkerControl control;

	for ( int i=0; i<3; i++ )
	{
		control.orientation.w = rand(-1,1);
		control.orientation.x = rand(-1,1);
		control.orientation.y = rand(-1,1);
		control.orientation.z = rand(-1,1);
		control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
		int_marker.controls.push_back(control);
		control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
		int_marker.controls.push_back(control);
	}

	server->insert(int_marker);
	server->setCallback(int_marker.name, &processFeedback);
}
// %EndTag(RandomDof)%


// %Tag(ViewFacing)%
void makeViewFacingMarker( )
{
	InteractiveMarker int_marker;
	int_marker.header.frame_id = "/base_link";
	int_marker.pose.position.y = -3.0 * marker_pos++;;
	int_marker.scale = 1;

	int_marker.name = "view_facing";
	int_marker.description = "View Facing 6-DOF";

	InteractiveMarkerControl control;

	// make a control that rotates around the view axis
	control.orientation_mode = InteractiveMarkerControl::VIEW_FACING;
	control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
	control.orientation.w = 1;
	control.name = "rotate";

	int_marker.controls.push_back(control);

	// create a box in the center which should not be view facing,
	// but move in the camera plane.
	control.orientation_mode = InteractiveMarkerControl::VIEW_FACING;
	control.interaction_mode = InteractiveMarkerControl::MOVE_PLANE;
	control.independent_marker_orientation = true;
	control.name = "move";

	control.markers.push_back( makeBox(int_marker) );
	control.always_visible = true;

	int_marker.controls.push_back(control);

	server->insert(int_marker);
	server->setCallback(int_marker.name, &processFeedback);
}
// %EndTag(ViewFacing)%


// %Tag(Quadrocopter)%
void makeQuadrocopterMarker( )
{
	InteractiveMarker int_marker;
	int_marker.header.frame_id = "/base_link";
	int_marker.pose.position.y = -3.0 * marker_pos++;;
	int_marker.scale = 1;

	int_marker.name = "quadrocopter";
	int_marker.description = "Quadrocopter";

	makeBoxControl(int_marker);

	InteractiveMarkerControl control;

	control.orientation.w = 1;
	control.orientation.x = 0;
	control.orientation.y = 1;
	control.orientation.z = 0;
	control.interaction_mode = InteractiveMarkerControl::MOVE_ROTATE;
	int_marker.controls.push_back(control);
	control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
	int_marker.controls.push_back(control);

	server->insert(int_marker);
	server->setCallback(int_marker.name, &processFeedback);
}
// %EndTag(Quadrocopter)%

// %Tag(ChessPiece)%
void makeChessPieceMarker( )
{
	InteractiveMarker int_marker;
	int_marker.header.frame_id = "/base_link";
	int_marker.pose.position.y = -3.0 * marker_pos++;;
	int_marker.scale = 1;

	int_marker.name = "chess_piece";
	int_marker.description = "Chess Piece\n(2D Move + Alignment)";

	InteractiveMarkerControl control;

	control.orientation.w = 1;
	control.orientation.x = 0;
	control.orientation.y = 1;
	control.orientation.z = 0;
	control.interaction_mode = InteractiveMarkerControl::MOVE_PLANE;
	int_marker.controls.push_back(control);

	// make a box which also moves in the plane
	control.markers.push_back( makeBox(int_marker) );
	control.always_visible = true;
	int_marker.controls.push_back(control);

	// we want to use our special callback function
	server->insert(int_marker);
	server->setCallback(int_marker.name, &processFeedback);

	// set different callback for POSE_UPDATE feedback
	server->setCallback(int_marker.name, &alignMarker, visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE );
}
// %EndTag(ChessPiece)%

// %Tag(PanTilt)%
void makePanTiltMarker( )
{
	InteractiveMarker int_marker;
	int_marker.header.frame_id = "/atc/ptu/base";
	int_marker.pose.position.x = 0;
	int_marker.pose.position.y = 0;
	int_marker.pose.position.z = 0;
	int_marker.scale = 0.5;

	int_marker.name = "/marker";
	int_marker.description = " Pan/Tilt control";

	makeBoxControl(int_marker);

	InteractiveMarkerControl control;

	control.orientation.w = 1;
	control.orientation.x = 0;
	control.orientation.y = 1;
	control.orientation.z = 0;
	control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
	control.orientation_mode = InteractiveMarkerControl::FIXED;
	int_marker.controls.push_back(control);

	control.orientation.w = 1;
	control.orientation.x = 0;
	control.orientation.y = 0;
	control.orientation.z = 1;
	control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
	control.orientation_mode = InteractiveMarkerControl::INHERIT;
	int_marker.controls.push_back(control);

	server->insert(int_marker);
	server->setCallback(int_marker.name, &processFeedback);
}
// %EndTag(PanTilt)%

// %Tag(Menu)%
void makeMenuMarker()
{
	InteractiveMarker int_marker;
	int_marker.header.frame_id = "/atc/ptu/base";
	int_marker.pose.position.y = -3.0 * marker_pos++;;
	int_marker.scale = 1;

	int_marker.name = "context_menu";
	int_marker.description = "Context Menu\n(Right Click)";

	InteractiveMarkerControl control;

	// make one control using default visuals
	control.interaction_mode = InteractiveMarkerControl::MENU;
	control.description="Options";
	control.name = "menu_only_control";
	int_marker.controls.push_back(control);

	// make one control showing a box
	Marker marker = makeBox( int_marker );
	control.markers.push_back( marker );
	control.always_visible = true;
	int_marker.controls.push_back(control);

	server->insert(int_marker);
	server->setCallback(int_marker.name, &processFeedback);
	menu_handler.apply( *server, int_marker.name );
}
// %EndTag(Menu)%

// %Tag(Moving)%
void makeMovingMarker()
{
	InteractiveMarker int_marker;
	int_marker.header.frame_id = "/moving_frame";
	int_marker.pose.position.y = -3.0 * marker_pos++;;
	int_marker.scale = 1;

	int_marker.name = "moving";
	int_marker.description = "Marker Attached to a\nMoving Frame";

	InteractiveMarkerControl control;

	control.orientation.w = 1;
	control.orientation.x = 1;
	control.orientation.y = 0;
	control.orientation.z = 0;
	control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
	int_marker.controls.push_back(control);

	control.interaction_mode = InteractiveMarkerControl::MOVE_PLANE;
	control.always_visible = true;
	control.markers.push_back( makeBox(int_marker) );
	int_marker.controls.push_back(control);

	server->insert(int_marker);
	server->setCallback(int_marker.name, &processFeedback);
}
// %EndTag(Moving)%

// %Tag(main)%
int main(int argc, char** argv)
{
	ros::init(argc, argv, "ptu_interactive_control_node");
	ros::NodeHandle n;

	//declare the publisher
	pub = n.advertise<sensor_msgs::JointState>("/ptu_cmd", 1);


	// create a timer to update the published transforms
	//ros::Timer frame_timer = n.createTimer(ros::Duration(0.01), frameCallback);

	server.reset( new interactive_markers::InteractiveMarkerServer("direct/im","",false) );

	ros::Duration(0.1).sleep();


	menu_handler.insert( "First Entry", &processFeedback );
	menu_handler.insert( "Second Entry", &processFeedback );
	interactive_markers::MenuHandler::EntryHandle sub_menu_handle = menu_handler.insert( "Submenu" );
	menu_handler.insert( sub_menu_handle, "First Entry", &processFeedback );
	menu_handler.insert( sub_menu_handle, "Second Entry", &processFeedback );

	//make6DofMarker( false );
	//make6DofMarker( true );
	//makeRandomDofMarker( );
	//makeViewFacingMarker( );
	//makeQuadrocopterMarker( );
	//makeChessPieceMarker( );
	makePanTiltMarker( );
	//makeMenuMarker( );
	//makeMovingMarker( );

	server->applyChanges();

	ros::spin();

	server.reset();
}
// %EndTag(main)%



