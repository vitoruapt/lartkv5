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
 * @file publish_target.cpp
 * @brief This code publishes and target as an rviz interactive marker for the
 * foveation control to be tested
 * @author Miguel Oliveira
 * @version v0
 * @date 2012-03-02
 */

//#include <tf/tf.h>

//#include <math.h>
//#include <interactive_markers/interactive_marker_server.h>
//#include <stdio.h>
//#include <ros/ros.h>
//#include <laser_geometry/laser_geometry.h>
//#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
//#include <tf/transform_listener.h>
//#include <pcl_ros/transforms.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
//#include <pcl/segmentation/extract_polygonal_prism_data.h>
//#include <pcl/filters/extract_indices.h>
//#include <pcl/filters/project_inliers.h>

#include <ros/ros.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

#include <math.h>

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
			pcl::PointCloud<pcl::PointXYZ> pc;
			pcl::PointXYZ pt;	
			pt.x = feedback->pose.position.x; pt.y = feedback->pose.position.y;	pt.z = feedback->pose.position.z;
			pc.points.push_back(pt);

			sensor_msgs::PointCloud2 pcmsg_out;
			pcl::toROSMsg(pc, pcmsg_out);
			pcmsg_out.header.stamp = ros::Time::now();
			pcmsg_out.header.frame_id = "/atc/vehicle/center_bumper";
			//ROS_INFO("Publishing Target position x=%f y=%f z=%f", feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z);
			pub.publish(pcmsg_out);

			//ROS_INFO_STREAM( s.str() << ": pose changed"
			//<< "\nposition = "
			//<< feedback->pose.position.x
			//<< ", " << feedback->pose.position.y
			//<< ", " << feedback->pose.position.z
			//<< "\norientation = "
			//<< feedback->pose.orientation.w
			//<< ", " << feedback->pose.orientation.x
			//<< ", " << feedback->pose.orientation.y
			//<< ", " << feedback->pose.orientation.z
			//<< "\nframe: " << feedback->header.frame_id
			//<< " time: " << feedback->header.stamp.sec << "sec, "
			//<< feedback->header.stamp.nsec << " nsec" );
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
	int_marker.header.frame_id = "/base_link";
	int_marker.pose.position.y = -3.0 * marker_pos++;;
	int_marker.scale = 1;

	int_marker.name = "pan_tilt";
	int_marker.description = "Pan / Tilt";

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
	int_marker.header.frame_id = "/base_link";
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
	ros::init(argc, argv, "foveation_control_target_generator");
	ros::NodeHandle n;


	pub = n.advertise<sensor_msgs::PointCloud2>("/target", 1);

	// create a timer to update the published transforms
	//ros::Timer frame_timer = n.createTimer(ros::Duration(0.01), frameCallback);

	server.reset( new interactive_markers::InteractiveMarkerServer("bytarget/im","",false) );

	ros::Duration(0.1).sleep();


	menu_handler.insert( "First Entry", &processFeedback );
	menu_handler.insert( "Second Entry", &processFeedback );
	interactive_markers::MenuHandler::EntryHandle sub_menu_handle = menu_handler.insert( "Submenu" );
	menu_handler.insert( sub_menu_handle, "First Entry", &processFeedback );
	menu_handler.insert( sub_menu_handle, "Second Entry", &processFeedback );

	//make6DofMarker( false );
	make6DofMarker( true );
	//makeRandomDofMarker( );
	//makeViewFacingMarker( );
	//makeQuadrocopterMarker( );
	//makeChessPieceMarker( );
	//makePanTiltMarker( );
	//makeMenuMarker( );
	//makeMovingMarker( );

	server->applyChanges();

	ros::spin();

	server.reset();
}
// %EndTag(main)%


//#include <interactive_markers/interactive_marker_server.h>
//#include <interactive_markers/menu_handler.h>

//#include <tf/transform_broadcaster.h>
//#include <tf/tf.h>

//#include <math.h>
//#include <interactive_markers/interactive_marker_server.h>
//#include <stdio.h>
//#include <ros/ros.h>
//#include <laser_geometry/laser_geometry.h>
//#include <sensor_msgs/LaserScan.h>
//#include <sensor_msgs/PointCloud2.h>
//#include <tf/transform_listener.h>
//#include <pcl_ros/transforms.h>
//#include <pcl/ros/conversions.h>
//#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>
//#include <pcl/segmentation/extract_polygonal_prism_data.h>
//#include <pcl/filters/extract_indices.h>
//#include <pcl/filters/project_inliers.h>


//#define PFLN printf("file %s line %d\n",__FILE__,__LINE__);

//void processFeedback(
//const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
//{
//ROS_INFO_STREAM( feedback->marker_name << " is now at "
//<< feedback->pose.position.x << ", " << feedback->pose.position.y
//<< ", " << feedback->pose.position.z );
//}

//int main(int argc, char **argv)
//{




//ros::init(argc, argv, "foveation_control_target_generator");
//ros::NodeHandle n("~");
//tf::TransformListener listener(n,ros::Duration(10));
//ros::Publisher pub = n.advertise<sensor_msgs::PointCloud2>("/target", 1);


//// create an interactive marker server on the topic namespace simple_marker
//interactive_markers::InteractiveMarkerServer server("foveation_control_target_generator");

//// create an interactive marker for our server
//visualization_msgs::InteractiveMarker int_marker;
//int_marker.header.frame_id = "/atc/vehicle/center_bumper";
//int_marker.name = "my_marker";
//int_marker.description = "Simple 1-DOF Control";

//// create a grey box marker
//visualization_msgs::Marker box_marker;
//box_marker.type = visualization_msgs::Marker::CUBE;
//box_marker.scale.x = 0.45;
//box_marker.scale.y = 0.45;
//box_marker.scale.z = 0.45;
//box_marker.color.r = 0.5;
//box_marker.color.g = 0.5;
//box_marker.color.b = 0.5;
//box_marker.color.a = 1.0;

//// create a non-interactive control which contains the box
//visualization_msgs::InteractiveMarkerControl box_control;
//box_control.always_visible = true;
//box_control.markers.push_back( box_marker );

//// add the control to the interactive marker
////int_marker.controls.push_back( box_control );

//// create a control which will move the box
//// this control does not contain any markers,
//// which will cause RViz to insert two arrows
////visualization_msgs::InteractiveMarkerControl rotate_control;
////rotate_control.name = "move_x";
////rotate_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;

////// add the control to the interactive marker
////int_marker.controls.push_back(rotate_control);
//visualization_msgs::InteractiveMarkerControl control;

//control.orientation.w = 1;
//control.orientation.x = 1;
//control.orientation.y = 0;
//control.orientation.z = 0;
//control.name = "rotate_x";
//control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
//int_marker.controls.push_back(control);
//control.name = "move_x";
//control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
//int_marker.controls.push_back(control);

//control.orientation.w = 1;
//control.orientation.x = 0;
//control.orientation.y = 1;
//control.orientation.z = 0;
//control.name = "rotate_z";
//control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
//int_marker.controls.push_back(control);
//control.name = "move_z";
//control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
//int_marker.controls.push_back(control);

//control.orientation.w = 1;
//control.orientation.x = 0;
//control.orientation.y = 0;
//control.orientation.z = 1;
//control.name = "rotate_y";
//control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
//int_marker.controls.push_back(control);
//control.name = "move_y";
//control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
//int_marker.controls.push_back(control);

//server.insert(int_marker);
//server.setCallback(int_marker.name, &processFeedback);



//// add the interactive marker to our collection &
//// tell the server to call processFeedback() when feedback arrives for it
////server.insert(int_marker, &processFeedback);

//// 'commit' changes and send to all clients
//server.applyChanges();


//ros::Rate loop_rate(10);
//ros::spin();
////while (n.ok())
////{

////float x,y,z;

////printf("Insert the X of target\n");
////int ret=scanf("%f",&x);
////printf("Insert the Y of target\n");
////ret=scanf("%f",&y);
////printf("Insert the Z of target\n");
////ret=scanf("%f",&z);



////pcl::PointCloud<pcl::PointXYZ> pc;
////pcl::PointXYZ pt;	
////pt.x = x; pt.y = y;	pt.z = z;
////pc.points.push_back(pt);

////sensor_msgs::PointCloud2 pcmsg_out;
////pcl::toROSMsg(pc, pcmsg_out);
////pcmsg_out.header.stamp = ros::Time::now();
////pcmsg_out.header.frame_id = "/atc/vehicle/center_bumper";
////ROS_INFO("Publishing Target position x=%f y=%f z=%f", x,y,z);
////pub.publish(pcmsg_out);

////ros::spinOnce();

////loop_rate.sleep();	
////}
//}
