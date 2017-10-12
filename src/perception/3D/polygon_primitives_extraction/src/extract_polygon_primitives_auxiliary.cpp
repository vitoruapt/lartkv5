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
 * @addtogroup extract_polygon_primitives 
 * @file 
 * @brief Auxiliary functions for the extract_polygon_primitives binary
 * @author Miguel Oliveira
 * @version v0
 * @date 2011-12-19
 *@{
 */
#ifndef _extract_polygon_primitives_AUXILIARY_CPP_
#define _extract_polygon_primitives_AUXILIARY_CPP_


#include "extract_polygon_primitives.h"

/**
 * @brief Gets the vehicle position by querying for the propper transform
 * @param listener listerner class to liikup transform
 * @param vehicle_tf the output transform
 * @param t time
 * @return 
 */
int get_vehicle_position(ros::NodeHandle *pn, tf::TransformListener* listener, tf::StampedTransform* vehicle_tf, ros::Time* t)
{
	ros::spinOnce();
	bool cont=true;

	while(cont && pn->ok())
	{
		cont=false;
		//Find the tf /world to vehicle at timestamp of velodyne date receival
		try
		{
			//listener->lookupTransform("/tf_vehicle", "/world", *t, *vehicle_tf);
			listener->lookupTransform("/world", "/tf_vehicle", ros::Time(0), *vehicle_tf);
		}
		catch (tf::TransformException ex)
		{
			ROS_ERROR("%s",ex.what());
			cont = true;
			ros::Duration(0.1).sleep();
		}
	}

	return 1;
}

/**
 * @brief Saves a composite point cloud with normals to a file. After saving you can use "rosrun pcl pcd_viewer <filename> -normal 10 -normal_scale 1" to view the normals.
 * @param pc_in point cloud with points
 * @param normals_in point cloud with normals
 * @param name the filename
 * @return 1 success
 */
int save_pc_PointNormal_to_pcd(pcl::PointCloud<pcl::PointXYZ>* pc_in, pcl::PointCloud<pcl::Normal>* normals_in, std::string name)
{
	pcl::PointCloud<pcl::PointNormal> pc; 
	pcl::concatenateFields (*pc_in, *normals_in, pc);
	sensor_msgs::PointCloud2 msg;
	pcl::toROSMsg(pc, msg);
	pcl::io::savePCDFile(name.c_str(), msg);
	return 1;
}

/**
 * @brief Estimate normals for pc_in using either radius or K 
 *
 * @param pc_in the point cloud to estimate the normals from
 * @param vx the viewpoint x coordinate
 * @param vyvthe viewpoint y coordinate
 * @param vz the viewpoint z coordinate
 * @param radius radius of the neighborhood to estimate the normals for each point
 * @param K number of neighbors to define the neighborhood to estimate the normals for each point
 * @param pc_out the normals out. Same size as pc_in
 * @return 1 if sucess or 0 if failure
 */
int compute_normals(pcl::PointCloud<pcl::PointXYZ>* pc_in, float vx, float vy, float vz, float radius, int K, pcl::PointCloud<pcl::Normal>* pc_out)
{
	ros::Time t = ros::Time::now();
	// Compute the normals for the cloud_to_procesvs
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne; // Create the normal estimation obje
	//pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ> ()); //tree to compute normals
//	pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ> ()); //tree to compute normals
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>); //tree to compute normals

	ne.setSearchMethod(tree);
	ne.setViewPoint(vx,vy,vz); 	
	ne.setInputCloud(pc_in->makeShared());

	if (K==0)
		ne.setRadiusSearch(radius);
	else if (radius==0)
		ne.setKSearch(K);
	else
	{
		ROS_ERROR("Cannot compute normals. Either radius=%f or K=%d must be 0, so that one is selected", radius, K);
		return 0;
	}

	ne.compute(*pc_out);

	for (size_t i=0; i<pc_out->points.size(); ++i)
	{
		pcl::flipNormalTowardsViewpoint( pc_in->points[i], vx,vy,vz,pc_out->points[i].normal[0],  pc_out->points[i].normal[1],pc_out->points[i].normal[2]);
	}

	ROS_INFO("Estimated normals in %f secs. pc_in with %ld points. radius=%f K=%d", (ros::Time::now() -t).toSec(), pc_in->points.size(), radius, K);
	return 1;
}

void filter_uncoherent_points(pcl::PointCloud<pcl::PointXYZ>::Ptr pcin, pcl::PointCloud<pcl::PointXYZ> *pcout, pcl::PointCloud<pcl::PointXYZ>* pcelim,  pcl::PointCloud<pcl::Normal>::Ptr n ,double radius, float vx, float vy, float vz)
{
	pcl::PointCloud<pcl::PointNormal>::Ptr tmp_in = pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud<pcl::PointNormal>)  ; 
	pcl::concatenateFields(*pcin, *n, *tmp_in);
	pcl::PointCloud<pcl::PointNormal>::Ptr tmp_out = pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud<pcl::PointNormal>); 
	pcl::PointCloud<pcl::PointNormal>::Ptr tmp_elim = pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud<pcl::PointNormal>); 

	ros::Time t = ros::Time::now();
	filter_uncoherent_points(tmp_in, radius, tmp_out, tmp_elim, vx, vy, vz);

	pcout->points.erase(pcout->points.begin(), pcout->points.end());
	for (size_t i=0; i< tmp_out->points.size(); i++)
	{
		pcl::PointXYZ pt(tmp_out->points[i].x, tmp_out->points[i].y, tmp_out->points[i].z);	
		pcout->points.push_back(pt);
	}

	pcout->height = 1; //1 since its and unordered pc
	pcout->is_dense=0;
	pcout->header.frame_id = pcin->header.frame_id;
	pcout->width = pcout->points.size();

	pcelim->points.erase(pcelim->points.begin(), pcelim->points.end());
	for (size_t i=0; i< tmp_elim->points.size(); i++)
	{
		pcl::PointXYZ pt(tmp_elim->points[i].x, tmp_elim->points[i].y, tmp_elim->points[i].z);	
		pcelim->points.push_back(pt);
	}

	pcelim->height = 1; //1 since its and unordered pc
	pcelim->is_dense=0;
	pcelim->header.frame_id = pcin->header.frame_id;
	pcelim->width = pcelim->points.size();


	ROS_INFO("Normal orientation entropy filter: pts in %ld pts out %ld removed %ld, (in %f secs)",pcin->points.size(), pcout->points.size(), pcelim->points.size(), (ros::Time::now() -t).toSec());
	tmp_in.reset();
	tmp_out.reset();
	tmp_elim.reset();
}

void get_limits(double mean, double std, double factor, double *high, double *low)
{
	(*high) = mean + factor*std;
	(*low) = mean - factor*std;
}

//void draw_markers(int k, btQuaternion* q_avg, std::vector<btQuaternion>* qv, pcl::PointCloud<pcl::PointNormal>::Ptr pc, std::vector<int>* pointIdxRadiusSearch)
//{
//static ros::Publisher markerarray_pub = pn->advertise<visualization_msgs::MarkerArray>("/Marker_neighbours", 1);

//static size_t total_normals=0;
//std::vector<visualization_msgs::Marker> marker_vec;
//geometry_msgs::Point p;
//std_msgs::ColorRGBA color;
//visualization_msgs::Marker msg;

////Draw the K point
//msg.header.frame_id = std::string("/world");
//msg.header.stamp = ros::Time::now();
//msg.ns = "K"; msg.id = 0;
//msg.action = visualization_msgs::Marker::ADD;
//msg.pose.orientation.w = 1.0;
//msg.type = visualization_msgs::Marker::SPHERE;
//msg.pose.position.x = pc->points[k].x;
//msg.pose.position.y = pc->points[k].y;
//msg.pose.position.z = pc->points[k].z;
//ROS_INFO("Position x=%f y=%f z=%f",msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
//msg.scale.x = 5; msg.scale.y = 5; msg.scale.z = 5; 
//msg.color.r = 0; msg.color.g = 0; msg.color.b = 0.3; msg.color.a = 0.2;
//marker_vec.push_back(msg);

////Draw the neighbor points
//msg.header.frame_id = std::string("/world");
//msg.header.stamp = ros::Time::now();
//msg.ns = "neighbors"; msg.id = 0;
//msg.action = visualization_msgs::Marker::ADD;
//msg.pose.position.x = 0; msg.pose.position.y = 0; msg.pose.position.z = 0; 
//msg.pose.orientation.x = 0;	msg.pose.orientation.y = 0;	msg.pose.orientation.z = 0; msg.pose.orientation.w = 1.0;
//msg.type = visualization_msgs::Marker::POINTS;
//msg.scale.x = 0.03; msg.scale.y = 0.03; msg.scale.z = 0.03; 
//msg.color.r = 0; msg.color.g = 0; msg.color.b = 1; msg.color.a = 1;

//msg.points.erase(msg.points.begin(), msg.points.end());
//for (size_t i = 0; i<qv->size(); i++)
//{
//p.x = pc->points[pointIdxRadiusSearch->at(i)].x;
//p.y = pc->points[pointIdxRadiusSearch->at(i)].y;
//p.z = pc->points[pointIdxRadiusSearch->at(i)].z;
//msg.points.push_back(p);
//}
//marker_vec.push_back(msg);

////Draw an arrow with the mean orientation
//msg.header.frame_id = std::string("/world");
//msg.header.stamp = ros::Time::now();
//msg.ns = "mean";
//msg.action = visualization_msgs::Marker::ADD;
//msg.pose.position.x = 0; msg.pose.position.y = 0; msg.pose.position.z = 0; 
//msg.pose.orientation.x = 0;	msg.pose.orientation.y = 0;	msg.pose.orientation.z = 0; msg.pose.orientation.w = 1.0;
//msg.type = visualization_msgs::Marker::ARROW;
//msg.id = 0;
//msg.scale.x = 0.08; msg.scale.y = 0.2; 
//msg.color.r = 1; msg.color.g = 0;	msg.color.b = 0.3;	msg.color.a = 0.6;
//msg.points.erase(msg.points.begin(), msg.points.end());	

//p.x = pc->points[k].x; p.y = pc->points[k].y; p.z = pc->points[k].z;
//msg.points.push_back(p);
//p.x = pc->points[k].x + (*q_avg)[0]; p.y = pc->points[k].y + (*q_avg)[1]; p.z = pc->points[k].z + (*q_avg)[2];
//msg.points.push_back(p);
//marker_vec.push_back(msg);

////Draw the neighbors normals
//msg.header.frame_id = std::string("/world");
//msg.header.stamp = ros::Time::now();
//msg.ns = "n_normals";
//msg.action = visualization_msgs::Marker::ADD;
//msg.pose.position.x = 0; msg.pose.position.y = 0; msg.pose.position.z = 0; 
//msg.pose.orientation.x = 0;	msg.pose.orientation.y = 0;	msg.pose.orientation.z = 0; msg.pose.orientation.w = 1.0;
//msg.type = visualization_msgs::Marker::ARROW;
//msg.scale.x = 0.01;//arrow shaft radius
//msg.scale.y = 0.05; //arrow head radius
//msg.color.r = 0; msg.color.g = 1; msg.color.b = 0.3; msg.color.a = 0.3;

//for (size_t i = 0; i < qv->size (); ++i)
//{
//msg.points.erase(msg.points.begin(), msg.points.end());	
//msg.id = i;
//p.x = pc->points[k].x;
//p.y = pc->points[k].y;
//p.z = pc->points[k].z;
//msg.points.push_back(p);

//p.x = pc->points[k].x + (*qv)[i][0];
//p.y = pc->points[k].y + (*qv)[i][1];
//p.z = pc->points[k].z + (*qv)[i][2];
//msg.points.push_back(p);
//marker_vec.push_back(msg);
//}

////Remove excessive markers
//for (size_t i = qv->size(); i <total_normals; ++i)
//{
//msg.type = visualization_msgs::Marker::ARROW;
//msg.id = i;
//msg.ns = "n_normals";
//msg.action = visualization_msgs::Marker::DELETE;
//marker_vec.push_back(msg);
//}
//total_normals = qv->size();

////Send the marker array msg to rviz
//visualization_msgs::MarkerArray marker_array_msg;
//ROS_INFO("marker_vec = %ld",marker_vec.size());
//marker_array_msg.set_markers_vec(marker_vec);	
//markerarray_pub.publish(marker_array_msg);	
//ros::spinOnce();


//}

void filter_uncoherent_points(pcl::PointCloud<pcl::PointNormal>::Ptr pc, double radius, pcl::PointCloud<pcl::PointNormal>::Ptr pcout, pcl::PointCloud<pcl::PointNormal>::Ptr pcelim, float vx, float vy, float vz)
{
	bool cont=true;
	//initialize a kdtree to pc
	pcl::KdTreeFLANN<pcl::PointNormal> kdtree;
	kdtree.setInputCloud(pc);

	//make sure pcout has no points
	pcout->points.erase(pcout->points.begin(), pcout->points.end());
	pcelim->points.erase(pcelim->points.begin(), pcelim->points.end());
	//_______________________________________________________
	//cycle all points an do the test for each	
	for (int k=0; k< (int)pc->points.size(); k++)
	{
		std::vector<int> pointIdxRadiusSearch;
		std::vector<float> pointRadiusSquaredDistance;
		std::vector<Eigen::Vector4f> ev_v; //list of quaternions from the normals of each neighbor
		Eigen::Vector4f ev_k;
		std::vector<float> angle_to_avg_v;
		double mean, std;
		//int acc=1; //an accumulator to compute the ratio

		if(	kdtree.radiusSearch(pc->points[k], radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) <= 0)
		{
			ROS_INFO("pt %d error in kdtree",k);
		}
		else
		{
			//_______________________________________________________
			ev_k[0] = pc->points[k].normal[0]; 
			ev_k[1] = pc->points[k].normal[1]; 
			ev_k[2] = pc->points[k].normal[2]; 
			ev_k[3] = 0;

			//_______________________________________________________
			for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i) 
			{
				pcl::flipNormalTowardsViewpoint(pc->points[k], vx, vy, vz, pc->points[pointIdxRadiusSearch[i]].normal[0], pc->points[pointIdxRadiusSearch[i]].normal[1], pc->points[pointIdxRadiusSearch[i]].normal[2]);
				Eigen::Vector4f ev; 
				ev[0] = pc->points[pointIdxRadiusSearch[i]].normal[0]; 
				ev[1] = pc->points[pointIdxRadiusSearch[i]].normal[1]; 
				ev[2] = pc->points[pointIdxRadiusSearch[i]].normal[2]; 
				ev[3] = 0;
				ev_v.push_back(ev);
			}

			//_______________________________________________________
			Eigen::Vector4f ev_avg; 
			float x_acc=0, y_acc=0, z_acc=0;
			for (size_t i = 0; i<ev_v.size(); i++) //cycle all neighbor quaternions and use slerp with dynamic weight
			{
				x_acc += ev_v[i][0];
				y_acc += ev_v[i][1];
				z_acc += ev_v[i][2];
			}

			ev_avg[0] = x_acc/(double)ev_v.size();
			ev_avg[1] = y_acc/(double)ev_v.size();
			ev_avg[2] = z_acc/(double)ev_v.size();
			ev_avg[3] = 0;

			double norm = sqrt(ev_avg[0]*ev_avg[0] + ev_avg[1]*ev_avg[1] + ev_avg[2]*ev_avg[2]);
			ev_avg[0] = ev_avg[0]/norm;
			ev_avg[1] = ev_avg[1]/norm;
			ev_avg[2] = ev_avg[2]/norm;

			//______________________________________________________
			//Check if normals must be flipped
			std::vector<Eigen::Vector4f> ev_v2; 
			std::vector<bool> ev_is_fliped; 

			for (size_t i = 0; i<ev_v.size(); i++) //cycle all neighbor quaternions and use slerp with dynamic weight
			{
				double angle_straight = ((float)(pcl::getAngle3D(ev_avg, ev_v[i])*180.0/M_PI));
				Eigen::Vector4f ev_fliped; 
				ev_fliped[0] = -ev_v[i][0];
				ev_fliped[1] = -ev_v[i][1];
				ev_fliped[2] = -ev_v[i][2];
				ev_fliped[3] = -ev_v[i][3];
				double angle_fliped = ((float)(pcl::getAngle3D(ev_avg, ev_fliped)*180.0/M_PI));

				if (fabs(angle_straight) < fabs(angle_fliped))
				{
					ev_v2.push_back(ev_v[i]);	
					ev_is_fliped.push_back(false);
				}
				else
				{
					ev_v2.push_back(ev_fliped);	
					ev_is_fliped.push_back(true);
				}

			}

			//_______________________________________________________
			//Recompute the average after fliping vectors
			Eigen::Vector4f ev2_avg; 
			x_acc=0, y_acc=0, z_acc=0;
			for (size_t i = 0; i<ev_v2.size(); i++) //cycle all neighbor quaternions and use slerp with dynamic weight
			{
				x_acc += ev_v2[i][0];
				y_acc += ev_v2[i][1];
				z_acc += ev_v2[i][2];
			}

			ev2_avg[0] = x_acc/(double)ev_v2.size();
			ev2_avg[1] = y_acc/(double)ev_v2.size();
			ev2_avg[2] = z_acc/(double)ev_v2.size();
			ev2_avg[3] = 0;

			norm = sqrt(ev2_avg[0]*ev2_avg[0] + ev2_avg[1]*ev2_avg[1] + ev2_avg[2]*ev2_avg[2]);
			ev2_avg[0] = ev2_avg[0]/norm;
			ev2_avg[1] = ev2_avg[1]/norm;
			ev2_avg[2] = ev2_avg[2]/norm;

			//_______________________________________________________
			//Get eigen vectors from quaternions
			for (size_t i = 0; i<ev_v2.size(); i++)
			{
				angle_to_avg_v.push_back((float)(pcl::getAngle3D(ev2_avg, ev_v2[i])*180.0/M_PI));
			}

			//_______________________________________________________
			//Get mean and standard deviation from the eigen vectors
			pcl::getMeanStd(angle_to_avg_v, mean, std);
			//double angle_to_avg = (float)(pcl::getAngle3D( ev2_avg, ev_k)*180.0/M_PI); 

			if(std>25 || mean>35)
			{
				pcelim->points.push_back(pc->points[k]);
			}
			else
			{
				pcout->points.push_back(pc->points[k]);
			}

			//  ______________________________________
			//  |                                    |
			//  | 			Draw markers			 |
			//  |____________________________________|
			if(k%1100==0 && cont &&0)
			{

				//ROS_INFO("q_avg = %f %f %f %f",q_avg[0], q_avg[1], q_avg[2], q_avg[3]);
				ROS_INFO("Point %d has %ld neighbors",k,ev_v.size());
				for (size_t i = 0; i<ev_v.size(); i++)
				{
					ROS_INFO("Neighbor %ld has %f (degrees)", i, angle_to_avg_v[i]);
				}
				ROS_INFO("Angle to Average Mean = %f std = %f",mean, std);

				draw_markers(k, &ev2_avg, &ev_v2, pc, &pointIdxRadiusSearch, &ev_avg, &ev_is_fliped);

				printf("olaolaola!\n");
				char ola[1024];
				int ret = scanf("%s",ola);  ret++;
				if (ola[0]=='q') exit(0);
				else if (ola[0]=='c') cont=false;


			}



		}
	}

	ROS_INFO("pcin =%d points", (int) pc->points.size());
	ROS_INFO("pcout =%d points", (int) pcout->points.size());
}

void draw_markers(int k, Eigen::Vector4f* q_avg, std::vector<Eigen::Vector4f>* qv, pcl::PointCloud<pcl::PointNormal>::Ptr pc, std::vector<int>* pointIdxRadiusSearch, Eigen::Vector4f* q_avg0, std::vector<bool>* is_fliped)
{
	static ros::Publisher markerarray_pub = pn->advertise<visualization_msgs::MarkerArray>("/Marker_neighbours", 1);

	static size_t total_normals=0;
	//std::vector<visualization_msgs::Marker> marker_vec;
	geometry_msgs::Point p;
	std_msgs::ColorRGBA color;
	visualization_msgs::Marker msg;
	visualization_msgs::MarkerArray marker_vec;


	//Draw the K point
	msg.header.frame_id = std::string("/world");
	msg.header.stamp = ros::Time::now();
	msg.ns = "K"; msg.id = 0;
	msg.action = visualization_msgs::Marker::ADD;
	msg.pose.orientation.w = 1.0;
	msg.type = visualization_msgs::Marker::SPHERE;
	msg.pose.position.x = pc->points[k].x;
	msg.pose.position.y = pc->points[k].y;
	msg.pose.position.z = pc->points[k].z;
	msg.scale.x = 5; msg.scale.y = 5; msg.scale.z = 5; 
	msg.color.r = 0; msg.color.g = 0; msg.color.b = 0.3; msg.color.a = 0.2;
	marker_vec.markers.push_back(msg);

	//Draw the neighbor points
	msg.header.frame_id = std::string("/world");
	msg.header.stamp = ros::Time::now();
	msg.ns = "neighbors"; msg.id = 0;
	msg.action = visualization_msgs::Marker::ADD;
	msg.pose.position.x = 0; msg.pose.position.y = 0; msg.pose.position.z = 0; 
	msg.pose.orientation.x = 0;	msg.pose.orientation.y = 0;	msg.pose.orientation.z = 0; msg.pose.orientation.w = 1.0;
	msg.type = visualization_msgs::Marker::POINTS;
	msg.scale.x = 0.03; msg.scale.y = 0.03; msg.scale.z = 0.03; 
	msg.color.r = 0; msg.color.g = 0; msg.color.b = 1; msg.color.a = 1;

	msg.points.erase(msg.points.begin(), msg.points.end());
	for (size_t i = 0; i<qv->size(); i++)
	{
		p.x = pc->points[pointIdxRadiusSearch->at(i)].x;
		p.y = pc->points[pointIdxRadiusSearch->at(i)].y;
		p.z = pc->points[pointIdxRadiusSearch->at(i)].z;
		msg.points.push_back(p);
	}
	marker_vec.markers.push_back(msg);

	//Draw an arrow with the iteration0 mean orientation
	msg.header.frame_id = std::string("/world");
	msg.header.stamp = ros::Time::now();
	msg.ns = "it0_mean";
	msg.action = visualization_msgs::Marker::ADD;
	msg.pose.position.x = 0; msg.pose.position.y = 0; msg.pose.position.z = 0; 
	msg.pose.orientation.x = 0;	msg.pose.orientation.y = 0;	msg.pose.orientation.z = 0; msg.pose.orientation.w = 1.0;
	msg.type = visualization_msgs::Marker::ARROW;
	msg.id = 0;
	msg.scale.x = 0.08; msg.scale.y = 0.2; 
	msg.color.r = 0; msg.color.g = 0.2;	msg.color.b = 0.7;	msg.color.a = 0.6;
	msg.points.erase(msg.points.begin(), msg.points.end());	

	p.x = pc->points[k].x; p.y = pc->points[k].y; p.z = pc->points[k].z;
	msg.points.push_back(p);
	p.x = pc->points[k].x + (*q_avg0)[0]; p.y = pc->points[k].y + (*q_avg0)[1]; p.z = pc->points[k].z + (*q_avg0)[2];
	msg.points.push_back(p);
	marker_vec.markers.push_back(msg);

	//Draw an arrow with the mean orientation
	msg.header.frame_id = std::string("/world");
	msg.header.stamp = ros::Time::now();
	msg.ns = "mean";
	msg.action = visualization_msgs::Marker::ADD;
	msg.pose.position.x = 0; msg.pose.position.y = 0; msg.pose.position.z = 0; 
	msg.pose.orientation.x = 0;	msg.pose.orientation.y = 0;	msg.pose.orientation.z = 0; msg.pose.orientation.w = 1.0;
	msg.type = visualization_msgs::Marker::ARROW;
	msg.id = 0;
	msg.scale.x = 0.08; msg.scale.y = 0.2; 
	msg.color.r = 1; msg.color.g = 0;	msg.color.b = 0.3;	msg.color.a = 0.6;
	msg.points.erase(msg.points.begin(), msg.points.end());	

	p.x = pc->points[k].x; p.y = pc->points[k].y; p.z = pc->points[k].z;
	msg.points.push_back(p);
	p.x = pc->points[k].x + (*q_avg)[0]; p.y = pc->points[k].y + (*q_avg)[1]; p.z = pc->points[k].z + (*q_avg)[2];
	msg.points.push_back(p);
	marker_vec.markers.push_back(msg);

	//Draw the neighbors normals
	msg.header.frame_id = std::string("/world");
	msg.header.stamp = ros::Time::now();
	msg.ns = "n_normals";
	msg.action = visualization_msgs::Marker::ADD;
	msg.pose.position.x = 0; msg.pose.position.y = 0; msg.pose.position.z = 0; 
	msg.pose.orientation.x = 0;	msg.pose.orientation.y = 0;	msg.pose.orientation.z = 0; msg.pose.orientation.w = 1.0;
	msg.type = visualization_msgs::Marker::ARROW;
	msg.scale.x = 0.01;//arrow shaft radius
	msg.scale.y = 0.05; //arrow head radius

	for (size_t i = 0; i < qv->size (); ++i)
	{
		if ((*is_fliped)[i]==true)
		{
			msg.color.r = 9; msg.color.g = 0; msg.color.b = 0.0; msg.color.a = 0.3;
		}
		else
		{
			msg.color.r = 0; msg.color.g = 1; msg.color.b = 0.3; msg.color.a = 0.3;
		}

		msg.points.erase(msg.points.begin(), msg.points.end());	
		msg.id = i;
		p.x = pc->points[k].x;
		p.y = pc->points[k].y;
		p.z = pc->points[k].z;
		msg.points.push_back(p);

		p.x = pc->points[k].x + (*qv)[i][0];
		p.y = pc->points[k].y + (*qv)[i][1];
		p.z = pc->points[k].z + (*qv)[i][2];
		msg.points.push_back(p);
		marker_vec.markers.push_back(msg);
	}

	//Remove excessive markers
	for (size_t i = qv->size(); i <total_normals; ++i)
	{
		msg.type = visualization_msgs::Marker::ARROW;
		msg.id = i;
		msg.ns = "n_normals";
		msg.action = visualization_msgs::Marker::DELETE;
		marker_vec.markers.push_back(msg);
	}
	total_normals = qv->size();

	//Send the marker array msg to rviz
		markerarray_pub.publish(marker_vec);	
	ros::spinOnce();


}

//void filter_uncoherent_points(pcl::PointCloud<pcl::PointNormal>::Ptr pc, double radius, pcl::PointCloud<pcl::PointNormal>::Ptr pcout, float vx, float vy, float vz)
//{
////initialize a kdtree to pc
//pcl::KdTreeFLANN<pcl::PointNormal> kdtree;
//kdtree.setInputCloud(pc);


////make sure pcout has no points
//pcout->points.erase(pcout->points.begin(), pcout->points.end());

//ROS_INFO("pcin =%d points", (int) pc->points.size());

////_______________________________________________________
////cycle all points an do the test for each	
//for (int k=0; k< (int)pc->points.size(); k++)
//{
//std::vector<int> pointIdxRadiusSearch;
//std::vector<float> pointRadiusSquaredDistance;
//std::vector<btQuaternion> qv; //list of quaternions from the normals of each neighbor
//std::vector<float> angle_to_avg_v;
//double mean, std;
//int acc=1; //an accumulator to compute the ratio

//if(	kdtree.radiusSearch(pc->points[k], radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) <= 0)
//{
//ROS_INFO("pt %d error in kdtree",k);
//}
//else
//{
////_______________________________________________________
////Get the quaternion from the current point's normal
//btQuaternion q_value(pc->points[k].normal[0], pc->points[k].normal[1], pc->points[k].normal[2], 0);
//q_value.normalize();
//Eigen::Vector4f ev_value; 
//ev_value[0] = q_value[0]; ev_value[1] = q_value[1];	ev_value[2] = q_value[2]; ev_value[3] = q_value[3];

////_______________________________________________________
////Create a list of quaternions from the normals neighbours
//for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i) 
//{
//pcl::flipNormalTowardsViewpoint(pc->points[k], vx, vy, vz, pc->points[pointIdxRadiusSearch[i]].normal[0], pc->points[pointIdxRadiusSearch[i]].normal[1], pc->points[pointIdxRadiusSearch[i]].normal[2]);
//btQuaternion q(pc->points[pointIdxRadiusSearch[i]].normal[0], pc->points[pointIdxRadiusSearch[i]].normal[1], pc->points[pointIdxRadiusSearch[i]].normal[2], 0);
//q.normalize();
//qv.push_back(q);
//}


////_______________________________________________________
////Compute the average quaternion
//btQuaternion q_avg(qv[0]); //initialize the average quaternion as the first quaternion on the list

//for (size_t i = 1; i<qv.size(); i++) //cycle all neighbor quaternions and use slerp with dynamic weight
//{
//if (k%100==0)
//{
//std::vector<int> partial_pointIdxRadiusSearch;
//std::vector<btQuaternion> partial_qv; 
//for (size_t j = 0; j<=i; ++j) 
//{
//partial_pointIdxRadiusSearch.push_back(pointIdxRadiusSearch[j]);
//partial_qv.push_back(qv[j]);
//}

//draw_markers(k, &q_avg, &partial_qv, pc, &partial_pointIdxRadiusSearch);

//printf("olaolaola!\n");
//char ola[1024];
//scanf("%s",ola);  
//if (ola[0]=='q') exit(0);
//}

//double ratio = 1-(double)acc/((double)acc+1);
//q_avg = slerp(q_avg, qv[i], ratio);
//acc++;
//}

////_______________________________________________________
////Get eigen vectors from quaternions
//Eigen::Vector4f ev_avg; 
//ev_avg[0] = q_avg[0]; ev_avg[1] = q_avg[1]; ev_avg[2] = q_avg[2]; ev_avg[3] = q_avg[3];

//for (size_t i = 0; i<qv.size(); i++)
//{
//Eigen::Vector4f v; 
//v[0] = qv[i][0]; v[1] = qv[i][1]; v[2] = qv[i][2]; v[3] = qv[i][3];
//angle_to_avg_v.push_back((float)(pcl::getAngle3D(ev_avg, v)*180.0/M_PI));
//}

////_______________________________________________________
////Get mean and standard deviation from the eigen vectors
//pcl::getMeanStd(angle_to_avg_v, mean, std);
//double angle_to_avg = (float)(pcl::getAngle3D( ev_avg, ev_value)*180.0/M_PI); 

////ROS_INFO("q_avg = %f %f %f %f",q_avg[0], q_avg[1], q_avg[2], q_avg[3]);


//if( fabs(angle_to_avg - mean) < std)
//{

//}
//else
//{
//pcout->points.push_back(pc->points[k]);
//}


////Get the value for the current point
//btQuaternion q0(pc->points[k].normal[0], pc->points[k].normal[1], pc->points[k].normal[2], 0);
//q0.normalize();
//Eigen::Vector4f v; 
//v[0] = q0[0];
//v[1] = q0[1];
//v[2] = q0[2];
//v[3] = q0[3];

//float val = (float)pcl::getAngle3D(ev_avg, v);
//double f=0.3;


////  ______________________________________
////  |                                    |
////  | 			Draw markers			 |
////  |____________________________________|
//if(k%100==0 )
//{

//ROS_INFO("q_avg = %f %f %f %f",q_avg[0], q_avg[1], q_avg[2], q_avg[3]);
//ROS_INFO("Point %d has %ld neighbors",k,qv.size());
//for (size_t i = 0; i<qv.size(); i++)
//{
//ROS_INFO("Neighbor %ld Angle to Average Quat is %f (degrees)", i, angle_to_avg_v[i]);
//}
//ROS_INFO("Angle to Average Mean = %f std = %f",mean, std);

//draw_markers(k, &q_avg, &qv, pc, &pointIdxRadiusSearch);

//printf("olaolaola!\n");
//char ola[1024];
//scanf("%s",ola);  
//if (ola[0]=='q') exit(0);


//}




//////if( val > ( mean + f*std ) || val < (mean - f*std))
//////if( val > (50*M_PI/180.0))
////if( std*180.0/M_PI > 20)
////{

////}
////else
////{
////pcout->points.push_back(pc->points[k]);
////}

//}
//}

//ROS_INFO("pcin =%d points", (int) pc->points.size());
//ROS_INFO("pcout =%d points", (int) pcout->points.size());
//}


//int erase_old_markers(std::vector<visualization_msgs::Marker>* marker_vec, unsigned int from, unsigned int to, std::string namesp)
//{
//for (unsigned int j=from; j<to; j++) //erase old markers
//{
//visualization_msgs::Marker msg; //declare the msg 
//msg.header.frame_id = data.frames.global_name;
//msg.header.stamp = ros::Time::now();
//msg.ns = namesp.c_str();
//msg.id = j;
//msg.action = visualization_msgs::Marker::DELETE;
//marker_vec->push_back(msg);
//}	
//return 1;
//}


#endif
/**
 *@}
 */      
