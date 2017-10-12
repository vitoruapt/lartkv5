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
#ifndef _trajectory_planner_nodelet_CPP_
#define _trajectory_planner_nodelet_CPP_
/**
 * @file trajectory_planner_nodelet.cpp
 * @brief Uses the c-trajectory class, to publish trajectories and send the message to follow one of them
 * @author Joel Pereira
 * @version v0
 * @date 2012-04-19
 */
#include "trajectory_planner_nodelet.h"

bool plan_trajectory=false;
bool have_plan=false;
geometry_msgs::PoseStamped pose_in;
geometry_msgs::PoseStamped pose_transformed;

std::vector< pcl::PointCloud<pcl::PointXYZ> > pc_v;


/** 
 * @brief Set attractor point coordinates
 * @param trajectory_planner::coordinates msg
 * @return void
 */
void set_coordinates(trajectory_planner::coordinates msg)
{	

// 	cout<<"stat Message received!!!"<<endl;
	// Change parameters
	pose_in.pose.position.x=msg.x;
	pose_in.pose.position.y=msg.y;
	pose_in.pose.orientation.w=cos(msg.theta/2.0);
	pose_in.pose.orientation.x=0.0;
	pose_in.pose.orientation.y=0.0;
	pose_in.pose.orientation.z=sin(msg.theta/2.0);

	pose_in.header.frame_id="/world";

	manage_vt->set_attractor_point(msg.x,msg.y,msg.theta);
	plan_trajectory=true;
}


/** 
 * @brief Set mtt points
 * @param mtt::TargetListPC msg
 * @return void
 */
void mtt_callback(mtt::TargetListPC msg)
{
	ROS_INFO("received mtt num obs=%ld num lines first obs =%d frame_id=%s",msg.id.size(), msg.obstacle_lines[0].width, msg.header.frame_id.c_str());

	//copy to global variable
	pc_v.erase(pc_v.begin(), pc_v.end());
	for (size_t i=0; i<msg.id.size(); ++i)
	{
		pcl::PointCloud<pcl::PointXYZ> tmp;
		pcl::fromROSMsg(msg.obstacle_lines[i], tmp);
		tmp.header.frame_id=msg.header.frame_id;
// 		tmp.header.stamp=msg.header.stamp;

		ROS_INFO("Obstacle %ld has %ld lines", i, tmp.points.size());
		pc_v.push_back(tmp);
	}	
}


/** 
 * @brief Determine the speed vector
 * @param c_trajectoryPtr t
 * @return vector<double>
 */
vector<double> set_speed_vector(boost::shared_ptr<c_trajectory> t)
{
	vector<double> speed_setted;
	for(size_t i=0;i<_NUM_NODES_;++i)
	{
		if(i < (_NUM_NODES_ - 1))
		{
			if((t->arc[i])*(t->arc[i+1])<0.0)
			{
				speed_setted.push_back((t->arc[i]/fabs(t->arc[i]))*_SPEED_SAFFETY_); // This is the speed set to reverse/forward or forward/reverse
			}
			else
			{
				speed_setted.push_back((t->arc[i]/fabs(t->arc[i]))*_SPEED_REQUIRED_);
			}
		}
		else
		{
			speed_setted.push_back((t->arc[i]/fabs(t->arc[i]))*_SPEED_REQUIRED_);
		}

	}
	return speed_setted;
}


/** 
 * @brief Main code of the nodelet
 * @details Publishes the trajectories message and the command message
 * @param int argc
 * @param char **argv
 * @return int
 */
int main(int argc, char **argv)
{
	ros::init(argc, argv, "trajectory_planner_nodelet");
	ros::NodeHandle n("~");
	p_n=&n;

	//Define the publishers and subscribers
	tf::TransformBroadcaster broadcaster;
	tf::TransformListener listener;
	p_listener = &listener;
	ros::Publisher array_pub = n.advertise<visualization_msgs::MarkerArray>( "/array_of_markers", 1 );
	ros::Subscriber sub = n.subscribe("/msg_coordinates", 1, set_coordinates);
	ros::Subscriber mtt_sub = n.subscribe("/mtt_targets", 1, mtt_callback);
	ros::Rate loop_rate(10);

	//   ___________________________________
	//   |                                 |
	//   |        Define the trajectories  |
	//   |_________________________________| 
	//Declare the traj manager class	
	manage_vt = (c_manage_trajectoryPtr) new c_manage_trajectory();

	//initialize attractor point
	t_desired_coordinates AP;
	AP.x=-1.0;
	AP.y=0.0;
	AP.theta=-M_PI/8;
	manage_vt->set_attractor_point(AP.x, AP.y, AP.theta);


	//initialize vehicle description
	manage_vt->set_vehicle_description(_VEHICLE_WIDTH_, _VEHICLE_LENGHT_BACK_, _VEHICLE_LENGHT_FRONT_, 
			_VEHICLE_HEIGHT_TOP_, _VEHICLE_HEIGHT_BOTTOM_);

	//initialize inter axis distance
	manage_vt->set_inter_axis_distance(_D_);


	//Compute the trajectory parameters
	for (int i=0; i< _NUM_TRAJ_; i++)
	{
		vector<double> v_a;
		vector<double> v_arc;
		for (int j=0; j<_NUM_NODES_; ++j)
		{
			v_a.push_back(M_PI/180.*a[i][j]);	
			v_arc.push_back(arc[i][j]);	
		}
		manage_vt->create_new_trajectory(v_a,v_arc,v_a);
	}

	// Choose trajectory  (0 is trajectory 1)
	int val = 0;	 // This will be commented

	if (val>=(int)manage_vt->vt.size())
	{
		ROS_ERROR("Chosen trajectory does not exist");
		exit(0);
	}
	manage_vt->set_chosen_traj(val);

	//create the static visualisation markers

	commandPublisher = n.advertise<trajectory_planner::traj_info>("/trajectory_information", 1000);

	
	
	
	
	while(ros::ok())
	{

		if(plan_trajectory==true)
		{
			//aqui recebi um comando para defenir uma trajectoria
			
			plan_trajectory = false;
			ROS_INFO("Going to plan a trajectory");
			// _________________________________
			//|                                 |
			//|    Set the parking view frame   |
			//|_________________________________| 
			bool have_transform=true;
			//Set the frame where to draw the trajectories
			try
			{
				p_listener->lookupTransform("/world","/vehicle_odometry",ros::Time(0), transformw);
			}
			catch (tf::TransformException ex)
			{
				ROS_ERROR("%s",ex.what());
				have_transform=false;
			}

			if (have_transform)
			{
				ros::Time time=ros::Time::now();
				broadcaster.sendTransform(tf::StampedTransform(transformw, time,"/world", "/parking_frame"));
				ros::spinOnce();
				broadcaster.sendTransform(tf::StampedTransform(transformw, time+ros::Duration(5),"/world", "/parking_frame")); 
				ros::spinOnce();			
				// 				cout<<"stat Publishing transform"<<endl;

				ros::Duration(0.1).sleep();

				//   ___________________________________
				//   |                                 |
				//   |        Trajectory evaluation    |
				//   |_________________________________| 

				//Transform attractor point to /parking_frame
				tf::Transformer tt;
				pose_in.header.stamp=time+ros::Duration(0.1);
				p_listener->transformPose ("/parking_frame", pose_in, pose_transformed); 

				ROS_INFO("pose_in frame_id=%s pose_transformed frame_id=%s",pose_in.header.frame_id.c_str(), pose_transformed.header.frame_id.c_str());
				//Set transformed attractor point
				manage_vt->set_attractor_point(pose_transformed.pose.position.x,pose_transformed.pose.position.y,atan2(2.0*(pose_transformed.pose.orientation.w*pose_transformed.pose.orientation.z),1-(2*(pose_transformed.pose.orientation.z*pose_transformed.pose.orientation.z))));


				//transform mtt to /parking_frame	
				pcl::PointCloud<pcl::PointXYZ>  pct;
				mtt::TargetListPC msg_transformed;



				for (size_t i=0; i<pc_v.size(); ++i)
				{

					if (i==0)
					{
						try
						{
// 							p_listener->lookupTransform(pc_v[i].header.frame_id,"/parking_frame", pc_v[i].header.stamp, transform_mtt);
						}
						catch (tf::TransformException ex)
						{
							ROS_ERROR("%s",ex.what());
						}
					}

					ROS_INFO("pc_v[%ld] frame_id=%s pcp frame_id=%s",i, pc_v[i].header.frame_id.c_str(), pct.header.frame_id.c_str());
					pcl_ros::transformPointCloud(pc_v[i],pct,transform_mtt.inverse());

					sensor_msgs::PointCloud2 pc_msg;
					pcl::toROSMsg(pct, pc_msg);
					pc_msg.header.frame_id = "/parking_frame";
					pc_msg.header.stamp = ros::Time::now();
					msg_transformed.obstacle_lines.push_back(pc_msg);
				}
				manage_vt->set_obstacles(msg_transformed); 



				//   ___________________________________
				//   |                                 |
				//   |        Draw                     |
				//   |_________________________________|
				manage_vt->create_static_markers();
				
				
				ros::Time st=ros::Time::now();
				
				manage_vt->compute_trajectories_scores();

				cout<<"Compute scores: "<<(ros::Time::now()-st).toSec();
				
				ROS_INFO("manage_vt chosen traj= %d",manage_vt->chosen_traj.index);


				trajectory_planner::traj_info info;

				manage_vt->get_traj_info_msg_from_chosen(&info);

				commandPublisher.publish(info);


				visualization_msgs::MarkerArray marker_array;
				manage_vt->compute_vis_marker_array(&marker_array);
				array_pub.publish(marker_array);
				cout<<"ja size:"<<marker_array.markers.size()<<endl;


				have_plan=true;

			}
		}

		if (have_plan==true)
		{	
			// !!!! The previous 'if' must have a weight evaluation !!!!  if ... && GlobalScore>=0.74
			
			broadcaster.sendTransform(tf::StampedTransform(transformw, ros::Time::now(),"/world", "/parking_frame")); 
			cout<<"stat Publishing transform"<<endl;
		}

		//printf("CURRENT NODE-> %d\n",node);
		//printf("Distance Travelled-> %f\n",base_status.distance_traveled);

		loop_rate.sleep();
		ros::spinOnce();
	}
}
#endif
