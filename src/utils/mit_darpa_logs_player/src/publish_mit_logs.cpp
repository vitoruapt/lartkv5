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
 * @addtogroup mit_logs
 * @file
 * @brief Reads lcm logs and publishes in ROS messages format. Also save a ros
 * bag file.
 *@{
 */
#ifndef _PUBLISH_MIT_LOGS_
#define _PUBLISH_MIT_LOGS_

#include "publish_mit_logs.h"
#include "rosbag/bag.h"

void signal_handler(int sig)
{
	printf("%d %d %d\n",sig,SIGSEGV,SIGINT);

	if(sig==SIGSEGV)
	{	

		signal(SIGSEGV, SIG_DFL); 

		ROS_WARN("System segfaulted"); 

		ros::shutdown();

		exit(0);
	}
	else if(sig==SIGINT)
	{
		ROS_WARN("Ctrl-c pressed"); 

		ros::shutdown();

		exit(0);
	}
}


void copy_pixels_to_image_msg_data(unsigned char *in, sensor_msgs::Image *image, int size)          {
	//for(int i=0; i<size; i+=3)
	////for (int l=0; l<960; l++)
	////for(int c=0; c<1280; c++)
	//{
	////printf("t=%d i=%d\n",t,i);
	////char a  = in[i+2];    
	////int t = c + l*1280;   
	//image->data[i] = in[i+2];   
	//image->data[i+1] = in[i+1]; 
	//image->data[i+2] = in[i];   
	////t+=3;
	//}

	for(int i=0; i<240*376*3; i+=3)
		//for (int l=0; l<960; l++)
		//for(int c=0; c<1280; c++)
	{
		//printf("t=%d i=%d\n",t,i);
		//char a  = in[i+2];    
		//int t = c + l*1280;   
		//image->data[i] = 255;;   
		//image->data[i+1] = 0; 
		//image->data[i+2] = 0;   
		image->data[i] = in[i+2];   
		image->data[i+1] = in[i+1]; 
		image->data[i+2] = in[i];   

		//t+=3;
	}

}

void set_fixed_fields_image_msg(sensor_msgs::Image* msg, int height, int width, char* encoding, int is_bigendian, char* frame_id)
{
	msg->height   = height; //set the height.
	msg->width    = width; //set the width
	msg->encoding = sensor_msgs::image_encodings::RGB8; //Set the encoding
	msg->is_bigendian = 0;
	msg->step = width*3;
	msg->data.resize(width*height*3);
	msg->header.frame_id = frame_id;
}

/// --------------------------------------------------------------------
/// Following tutorial on http://www.ros.org/wiki/navigation/Tutorials/RobotSetup/Sensors#Publishing_PointClouds_over_ROS
///Point_Cloud_2 msg is defined at http://www.ros.org/doc/api/sensor_msgs/html/msg/PointCloud2.html
/// --------------------------------------------------------------------

int main(int argc, char **argv)
{
	/// Initialize ROS coms
	ros::init(argc, argv, "point_cloud_publisher");

	ros::NodeHandle n;
	ros::Publisher cloud_pub = n.advertise<sensor_msgs::PointCloud2>("/cloud", 1);
	ros::Publisher filtered_cloud_pub = n.advertise<sensor_msgs::PointCloud2>("/filtered_cloud", 1);
	ros::Publisher pose_pub = n.advertise<geometry_msgs::PoseStamped >("/vehicle_pose", 1);

	rosbag::Bag bag("test.bag", rosbag::bagmode::Write);


	image_transport::ImageTransport it(n);
	image_transport::Publisher pub_image_rfc = it.advertise("/cam_roof_fc", 1);
	image_transport::Publisher pub_image_rfl = it.advertise("/cam_roof_fl", 1);
	image_transport::Publisher pub_image_rfr = it.advertise("/cam_roof_fr", 1);
	image_transport::Publisher pub_image_rrc = it.advertise("/cam_roof_rc", 1);
	image_transport::Publisher pub_image_rfc_6mm = it.advertise("/cam_roof_fc_6mm", 1);

	geometry_msgs::PoseStamped pose_msg;

	sensor_msgs::Image image_rfc;
	set_fixed_fields_image_msg(&image_rfc, 240, 376,(char*)"RGB8", 0, (char*) "/tf_cam_roof_fc");
	sensor_msgs::Image image_rfl;
	set_fixed_fields_image_msg(&image_rfl, 240, 376,(char*)"RGB8", 0, (char*) "/tf_cam_roof_fl");
	sensor_msgs::Image image_rfr;
	set_fixed_fields_image_msg(&image_rfr, 240, 376,(char*)"RGB8", 0, (char*) "/tf_cam_roof_fr");
	sensor_msgs::Image image_rrc;
	set_fixed_fields_image_msg(&image_rrc, 240, 376,(char*)"RGB8", 0, (char*) "/tf_cam_roof_rc");
	sensor_msgs::Image image_rfc_6mm;
	set_fixed_fields_image_msg(&image_rrc, 240, 376,(char*)"RGB8", 0, (char*) "/tf_cam_roof_fc_6mm");

	tf::TransformBroadcaster tf_br; 
	tf::Transform tr;


	tf::TransformBroadcaster tf_br1; 
	tf::Transform tr1;
	tr1.setOrigin( tf::Vector3(2.0, 0.10, 1.87));
	tf::Quaternion q1;
	q1.setRPY( M_PI/180.*-92.732548686299864, M_PI/180.* 1.5247112920413703, M_PI/180.* -88.947474213001513);
	tr1.setRotation(q1); 


	tf::TransformBroadcaster tf_br2; 
	tf::Transform tr2;
	tr2.setOrigin( tf::Vector3(1.70, 0.66, 1.83));
	tf::Quaternion q2;
	q2.setRPY( M_PI/180.*-104.85977729830938, M_PI/180.* 1.7669342713597069, M_PI/180.* -24.369926193961831);
	tr2.setRotation(q2); 

	tf::TransformBroadcaster tf_br3; 
	tf::Transform tr3;
	tr3.setOrigin( tf::Vector3(1.60, -0.66, 1.83));
	tf::Quaternion q3;
	q3.setRPY( M_PI/180.*-104.67989704943309, M_PI/180.* -1.6346433362446215, M_PI/180.* -154.50843893511401);
	tr3.setRotation(q3); 

	tf::TransformBroadcaster tf_br4; 
	tf::Transform tr4;
	tr4.setOrigin( tf::Vector3(-0.72, 0.58, 1.87));
	tf::Quaternion q4;
	q4.setRPY( M_PI/180.*-87.556890981409339, M_PI/180.* 0.20153392228877995, M_PI/180.* 90.018251146632338);
	tr4.setRotation(q4); 

	tf::TransformBroadcaster tf_br5; 
	tf::Transform tr5;
	tr5.setOrigin( tf::Vector3(2.0, 0.10, 1.87));
	tf::Quaternion q5;
	q5.setRPY( M_PI/180.*-92.994985165680859, M_PI/180.* 1.8089691922136495, M_PI/180.* -89.480172060729586);
	tr5.setRotation(q5); 

	visualization_msgs::Marker car_marker;
	ros::Publisher car_marker_pub = n.advertise<visualization_msgs::Marker>("/car_resource", 1);
	car_marker.id = 1;  
	car_marker.header.stamp = ros::Time::now(); 
	car_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
	car_marker.action = visualization_msgs::Marker::ADD;
	car_marker.ns = "ns";
	car_marker.mesh_resource = "package://wrapper_collada/models/audi_q7_blender.dae";
	car_marker.mesh_use_embedded_materials = 1; 
	car_marker.header.frame_id = "/world";
	car_marker.scale.x = 1;
	car_marker.scale.y = 1;
	car_marker.scale.z = 1;
	car_marker.color.r = 0.8;
	car_marker.color.g = 0.4;
	car_marker.color.b = 0.0;
	car_marker.color.a = 0.9;

	ros::Rate r(0.1);

	//pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
	pcl::PointCloud<pcl::PointXYZ> cloud;
	sensor_msgs::PointCloud2::Ptr cloud_msg(new sensor_msgs::PointCloud2 ());
	sensor_msgs::PointCloud2::Ptr cloud_msg_filtered(new sensor_msgs::PointCloud2 ());
	cloud.header.frame_id = "/world";
	cloud.height = 1;
	cloud.is_dense=0;

	if(argc < 4) 
	{
		fprintf(stderr, 
				"usage: example4-velodyne <logfile> <start_time> <end time>\n"
				"\n"
				"start_time and end_time are given in seconds from the\n"
				"start of the log file\n");
		return 1;
	}

	lcm_eventlog_t *log = lcm_eventlog_create(argv[1], "r");
	if(!log) 
	{
		fprintf(stderr, "error opening log file\n");
		return 1;
	}

	double start_time = strtod(argv[2], NULL);
	double end_time = strtod(argv[3], NULL);

	Config *config = config_parse_default();
	if(!config) {
		fprintf(stderr, "couldn't find config file\n");
		return 1;
	}

	// load the velodyne sensor calibration
	velodyne_calib_t *vcalib = velodyne_calib_create();



	// read the first timestamp of the log file
	lcm_eventlog_event_t *event = lcm_eventlog_read_next_event(log);
	int64_t first_timestamp = event->timestamp;
	lcmtypes_pose_t last_pose;


	//int64_t seek_utime = first_timestamp + (int64_t)((start_time-1)* 1000000);
	//lcm_eventlog_seek_to_timestamp(log, seek_utime);


	//lcm_eventlog_free_event(event);
	//event = lcm_eventlog_read_next_event(log);
	//first_timestamp = event->timestamp;

	//Start the main log reading/publication cycle
	//for (double ctime = start_time; ctime < end_time && n.ok(); ctime+=4)
	//{

		cloud.points.erase(cloud.points.begin(), cloud.points.end());
		//ROS_INFO("Time %3.2f", ctime);
		//
	printf("start_time=%f\n",start_time);	
	printf("end_time=%f\n",end_time);	
	ros::Time time_start = ros::Time::now();

		// compute the desired start and end timestamps
		int64_t start_utime = first_timestamp + (int64_t)(start_time*1000000);
		int64_t end_utime = first_timestamp   + (int64_t)(end_time  *1000000);

		double start_velodyne_acquisition=-1;

		memset(&last_pose, 0, sizeof(last_pose));

		while(n.ok())
		{
			// release the last event
			lcm_eventlog_free_event(event);


			// read an event
			event = lcm_eventlog_read_next_event(log);

			if(!event)
				break;
			else
			{
				printf("Event %s at time %3.2f\n",event->channel,((double)(event->timestamp-first_timestamp))/1000000.); 
				//if(!strncmp(event->channel, "CAM_THUMB", 9)) 
				//printf("Event %s at time %3.2f\n",event->channel,((double)(event->timestamp-first_timestamp))/1000000.); 

			}


			// always keep track of the current pose
			if(!strcmp(event->channel, "POSE")) {
				if(last_pose.utime) 
					lcmtypes_pose_t_decode_cleanup(&last_pose);

				lcmtypes_pose_t_decode(event->data, 0, event->datalen, &last_pose);
			}

			// ignore other messages until the desired start time
			if(event->timestamp < start_utime) {
				continue;
			}

			// quit if we're done
			if(event->timestamp >= end_utime) {
				break;
			}
			
			ros::Time event_time = time_start + ros::Duration(((double)(event->timestamp-first_timestamp))/1000000.);

			printf("ROS Event %s at time %3.2f\n",event->channel,event_time.toSec()); 

			if(!strcmp(event->channel, "POSE"))
		   	{
				geometry_msgs::TransformStamped msg;
				std::vector< geometry_msgs::TransformStamped > vec_msg;
				tf::tfMessage tfmsg; 

				printf("Processing Event %s at time %3.2f\n",event->channel,((double)(event->timestamp-first_timestamp))/1000000.); 
				pose_msg.header.frame_id = "/world";
				pose_msg.header.stamp = ros::Time::now();
				pose_msg.pose.position.x = last_pose.pos[0];
				pose_msg.pose.position.y = last_pose.pos[1];
				pose_msg.pose.position.z = last_pose.pos[2];

				pose_msg.pose.orientation.x  = last_pose.orientation[1];
				pose_msg.pose.orientation.y  = last_pose.orientation[2];
				pose_msg.pose.orientation.z  = last_pose.orientation[3];
				pose_msg.pose.orientation.w  = last_pose.orientation[0];

				tr.setOrigin( tf::Vector3(pose_msg.pose.position.x,
										  pose_msg.pose.position.y,
										  pose_msg.pose.position.z));
				tr.setRotation( tf::Quaternion( pose_msg.pose.orientation.x,
						   						pose_msg.pose.orientation.y, 
												pose_msg.pose.orientation.z,
												pose_msg.pose.orientation.w
											  ));

				pose_pub.publish(pose_msg);

				//the car resource
				car_marker.pose.position.x = pose_msg.pose.position.x;	
				car_marker.pose.position.y = pose_msg.pose.position.y;	
				car_marker.pose.position.z = pose_msg.pose.position.z;	
				car_marker.pose.orientation.x = pose_msg.pose.orientation.x;	
				car_marker.pose.orientation.y = pose_msg.pose.orientation.y;	
				car_marker.pose.orientation.z = pose_msg.pose.orientation.z;	
				car_marker.pose.orientation.w = pose_msg.pose.orientation.w;	
				car_marker.header.stamp = event_time;
				car_marker_pub.publish(car_marker);

				bag.write("/car_resource", event_time,car_marker); 

				tf_br.sendTransform(tf::StampedTransform(tr, ros::Time::now(), "/world","/tf_vehicle"));
				tf::transformStampedTFToMsg(tf::StampedTransform(tr, ros::Time::now(), "/world","/tf_vehicle"),msg);
				vec_msg.erase(vec_msg.begin(),vec_msg.end());
				vec_msg.push_back(msg);
				//tfmsg.set_transforms_vec(vec_msg);
				//tfmsg.set_transforms_size(1);
				//bag.write("/tf", event_time,tfmsg); 

				tf_br1.sendTransform(tf::StampedTransform(tr1, ros::Time::now(), "/tf_vehicle","/tf_cam_roof_fc"));
				tf::transformStampedTFToMsg(tf::StampedTransform(tr1, ros::Time::now(), "/tf_vehicle","/tf_cam_roof_fc"),msg);
				//vec_msg.erase(vec_msg.begin(),vec_msg.end());
//				vec_msg.push_back(msg);
				tfmsg.transforms.push_back(msg);
				//tfmsg.set_transforms_vec(vec_msg);
				//tfmsg.set_transforms_size(1);
				//bag.write("/tf", event_time,tfmsg); 


				tf_br2.sendTransform(tf::StampedTransform(tr2, ros::Time::now(), "/tf_vehicle","/tf_cam_roof_fl"));
				tf::transformStampedTFToMsg(tf::StampedTransform(tr2, ros::Time::now(), "/tf_vehicle","/tf_cam_roof_fl"),msg);
				//vec_msg.erase(vec_msg.begin(),vec_msg.end());
//				vec_msg.push_back(msg);
				tfmsg.transforms.push_back(msg);
				//tfmsg.set_transforms_vec(vec_msg);
				//tfmsg.set_transforms_size(1);
				//bag.write("/tf", event_time,tfmsg); 


				tf_br3.sendTransform(tf::StampedTransform(tr3, ros::Time::now(), "/tf_vehicle","/tf_cam_roof_fr"));
				tf::transformStampedTFToMsg(tf::StampedTransform(tr3, ros::Time::now(), "/tf_vehicle","/tf_cam_roof_fr"),msg);
				//vec_msg.erase(vec_msg.begin(),vec_msg.end());
//				vec_msg.push_back(msg);
				tfmsg.transforms.push_back(msg);
				//tfmsg.set_transforms_vec(vec_msg);
				//tfmsg.set_transforms_size(1);
				//bag.write("/tf", event_time,tfmsg); 


				tf_br4.sendTransform(tf::StampedTransform(tr4, ros::Time::now(), "/tf_vehicle","/tf_cam_roof_rc"));
				tf::transformStampedTFToMsg(tf::StampedTransform(tr4, ros::Time::now(), "/tf_vehicle","/tf_cam_roof_rc"),msg);
				//vec_msg.erase(vec_msg.begin(),vec_msg.end());
//				vec_msg.push_back(msg);
				tfmsg.transforms.push_back(msg);
				//tfmsg.set_transforms_vec(vec_msg);
				//tfmsg.set_transforms_size(1);
				//bag.write("/tf", event_time,tfmsg); 


				tf_br5.sendTransform(tf::StampedTransform(tr5, ros::Time::now(), "/tf_vehicle","/tf_cam_roof_fc_6mm"));
				tf::transformStampedTFToMsg(tf::StampedTransform(tr5, ros::Time::now(), "/tf_vehicle","/tf_cam_roof_fc_6mm"),msg);
				//vec_msg.erase(vec_msg.begin(),vec_msg.end());
//				vec_msg.push_back(msg);
				tfmsg.transforms.push_back(msg);
//				tfmsg.set_transforms_vec(vec_msg);
//				tfmsg.set_transforms_size(6);
				bag.write("/tf", event_time,tfmsg); 

			}


			if(!strcmp(event->channel, "VELODYNE")) 
			{


				if(start_velodyne_acquisition ==-1)
				{
					start_velodyne_acquisition = (double)(first_timestamp + event->timestamp)/1000000.;
				//int64_t start_utime = first_timestamp + (int64_t)(start_time*1000000);
				}

				if (((double)(first_timestamp+event->timestamp)/1000000. - start_velodyne_acquisition)>1)
				{

					printf("Processing Event %s at time %3.2f\n",event->channel,((double)(event->timestamp-first_timestamp))/1000000.); 
					//Publish raw velodyne pc
// 					cloud.header.stamp = ros::Time::now();
                    ROS_ERROR("Cloud timestamp not set, problem during migration lar3 to lar4");
					pcl::toROSMsg(cloud, *cloud_msg);
					//ROS_INFO("PUBLISHING cloud_msg with %d points", cloud_msg->width);
					//cloud_pub.publish(cloud_msg);
				
                    ROS_ERROR("Voxel grid REMOVED, not working in ros::hydro, please remake, problem during migration lar3 to lar4");
                    
// 					pcl::VoxelGrid<sensor_msgs::PointCloud2> sor; //Create the filtering object
// 					sor.setInputCloud(cloud_msg);
// 					sor.setLeafSize(0.2, 0.2, 0.01);
					//sor.setLeafSize(0.3, 0.3, 0.3);
					//sor.setLeafSize(0.01, 0.01, 0.01);
					//sor.setLeafSize(0.05, 0.05, 0.05);
// 					sor.filter(*cloud_msg_filtered);
					filtered_cloud_pub.publish(cloud_msg_filtered);
					bag.write("/filtered_cloud", event_time, cloud_msg_filtered);

					cloud.points.erase(cloud.points.begin(), cloud.points.end());

					start_velodyne_acquisition = (double)(first_timestamp + event->timestamp)/1000000.;
				}

				// parse the LCM packet into a velodyne data packet.
				lcmtypes_velodyne_t vel;
				lcmtypes_velodyne_t_decode(event->data, 0, event->datalen, 
						&vel);

				// compute the velodyne-to-local transformation matrix
				// 
				// This is an approximation because we're using the last recorded
				// pose.  A more accurate projection might be to project the
				// vehicle's pose forward based on its last measured velocity.
				double velodyne_to_local[16];
				config_util_sensor_to_local_with_pose (config, "VELODYNE", 
						velodyne_to_local, &last_pose);

				// parse the velodyne data packet
				velodyne_decoder_t vdecoder;
				velodyne_decoder_init(vcalib, &vdecoder, vel.data, vel.datalen);

				// project each sample in the velodyne data packet into the local
				// frame
				velodyne_sample_t vsample;

				while (!velodyne_decoder_next(vcalib, &vdecoder, &vsample))
				{
					if (vsample.range < 3 || vsample.range>50)
					{
						continue;
					}

					double sensor_xyz[4] = {vsample.xyz[0], vsample.xyz[1], vsample.xyz[2], 1 };
					double local_xyz[4];

					matrix_vector_multiply_4x4_4d(velodyne_to_local, sensor_xyz, local_xyz);

					pcl::PointXYZ Pt;
					Pt.x = local_xyz[0];
					Pt.y = local_xyz[1];
					Pt.z = local_xyz[2];

					//Pt.x = sensor_xyz[0];
					//Pt.y = sensor_xyz[1];
					//Pt.z = sensor_xyz[2];
					cloud.push_back(Pt);
				}

				cloud.width = cloud.size();



				lcmtypes_velodyne_t_decode_cleanup(&vel);
			}

			if(!strcmp(event->channel, "CAM_THUMB_RFC")) 
			{

				printf("Processing Event %s at time %3.2f\n",event->channel,((double)(event->timestamp-first_timestamp))/1000000.); 

				ROS_INFO("EVENT CAM_THUMB_RFC");
				lcmtypes_image_t img;
				lcmtypes_image_t_decode(event->data, 0, event->datalen, &img);

				//Save the image to file
				char fname[80];
				sprintf(fname, "cam.jpg");
				FILE *fp = fopen(fname, "wb");
				int status = fwrite(img.image, img.size, 1, fp);
				if(status != 1) {
					perror("fwrite");
					return 1;
				}
				fclose(fp);
				//printf("%s\n", fname);

				//reload and copy to sensor_msgs::Image
				cv_bridge::CvImage out_msg;
				out_msg.header.stamp = ros::Time::now();// Same timestamp and tf frame as input image
				out_msg.header.frame_id = "/tf_cam_roof_fc";
				out_msg.encoding = sensor_msgs::image_encodings::TYPE_8UC3; // Or whatever
				out_msg.image = cv::imread(fname,1); // Your cv::Mat
				out_msg.toImageMsg(image_rfc);

				pub_image_rfc.publish(image_rfc);
				bag.write("/cam_roof_fc", event_time, image_rfc);

				//copy_pixels_to_image_msg_data(img.image, &image_rfc, img.size); 
				lcmtypes_image_t_decode_cleanup(&img);
			}

			if(!strcmp(event->channel, "CAM_THUMB_RFL")) 
			{
				printf("Processing Event %s at time %3.2f\n",event->channel,((double)(event->timestamp-first_timestamp))/1000000.); 
				lcmtypes_image_t img;
				lcmtypes_image_t_decode(event->data, 0, event->datalen, &img);

				//Save the image to file
				char fname[80];
				sprintf(fname, "cam.jpg");
				FILE *fp = fopen(fname, "wb");
				int status = fwrite(img.image, img.size, 1, fp);
				if(status != 1) {
					perror("fwrite");
					return 1;
				}
				fclose(fp);

				//reload and copy to sensor_msgs::Image
				cv_bridge::CvImage out_msg;
				out_msg.header.stamp = ros::Time::now();// Same timestamp and tf frame as input image
				out_msg.header.frame_id = "/tf_cam_roof_fl";
				out_msg.encoding = sensor_msgs::image_encodings::TYPE_8UC3; // Or whatever
				out_msg.image = cv::imread(fname,1); // Your cv::Mat

				out_msg.toImageMsg(image_rfl);

				pub_image_rfl.publish(image_rfl);
				bag.write("/cam_roof_fl", event_time, image_rfl);

				//copy_pixels_to_image_msg_data(img.image, &image_rfc, img.size); 
				lcmtypes_image_t_decode_cleanup(&img);
			}

			if(!strcmp(event->channel, "CAM_THUMB_RFR")) 
			{
				printf("Processing Event %s at time %3.2f\n",event->channel,((double)(event->timestamp-first_timestamp))/1000000.); 
				lcmtypes_image_t img;
				lcmtypes_image_t_decode(event->data, 0, event->datalen, &img);

				//Save the image to file
				char fname[80];
				sprintf(fname, "cam.jpg");
				FILE *fp = fopen(fname, "wb");
				int status = fwrite(img.image, img.size, 1, fp);
				if(status != 1) {
					perror("fwrite");
					return 1;
				}
				fclose(fp);
				//printf("%s\n", fname);

				//reload and copy to sensor_msgs::Image
				cv_bridge::CvImage out_msg;
				out_msg.header.stamp = ros::Time::now();// Same timestamp and tf frame as input image
				out_msg.header.frame_id = "/tf_cam_roof_fr";
				out_msg.encoding = sensor_msgs::image_encodings::TYPE_8UC3; // Or whatever
				out_msg.image = cv::imread(fname,1); // Your cv::Mat
				out_msg.toImageMsg(image_rfr);

				pub_image_rfr.publish(image_rfr);
				bag.write("/cam_roof_fr", event_time, image_rfr);

				//copy_pixels_to_image_msg_data(img.image, &image_rfc, img.size); 
				lcmtypes_image_t_decode_cleanup(&img);

			}

			if(!strcmp(event->channel, "CAM_THUMB_RRC")) 
			{
				printf("Processing Event %s at time %3.2f\n",event->channel,((double)(event->timestamp-first_timestamp))/1000000.); 
				lcmtypes_image_t img;
				lcmtypes_image_t_decode(event->data, 0, event->datalen, &img);

				//Save the image to file
				char fname[80];
				sprintf(fname, "cam.jpg");
				FILE *fp = fopen(fname, "wb");
				int status = fwrite(img.image, img.size, 1, fp);
				if(status != 1) {
					perror("fwrite");
					return 1;
				}
				fclose(fp);
				//printf("%s\n", fname);

				//reload and copy to sensor_msgs::Image
				cv_bridge::CvImage out_msg;
				out_msg.header.stamp = ros::Time::now();// Same timestamp and tf frame as input image
				out_msg.header.frame_id = "/tf_cam_roof_rc";
				out_msg.encoding = sensor_msgs::image_encodings::TYPE_8UC3; // Or whatever
				out_msg.image = cv::imread(fname,1); // Your cv::Mat
				out_msg.toImageMsg(image_rrc);

				pub_image_rrc.publish(image_rrc);
				bag.write("/cam_roof_rc", event_time, image_rrc);

				//copy_pixels_to_image_msg_data(img.image, &image_rfc, img.size); 
				lcmtypes_image_t_decode_cleanup(&img);
			}

			if(!strcmp(event->channel, "CAM_THUMB_RFC_6mm")) 
			{
				printf("Processing Event %s at time %3.2f\n",event->channel,((double)(event->timestamp-first_timestamp))/1000000.); 
				lcmtypes_image_t img;
				lcmtypes_image_t_decode(event->data, 0, event->datalen, &img);

				//Save the image to file
				char fname[80];
				sprintf(fname, "cam.jpg");
				FILE *fp = fopen(fname, "wb");
				int status = fwrite(img.image, img.size, 1, fp);
				if(status != 1) {
					perror("fwrite");
					return 1;
				}
				fclose(fp);
				//printf("%s\n", fname);

				//reload and copy to sensor_msgs::Image
				cv_bridge::CvImage out_msg;
				out_msg.header.stamp = ros::Time::now();// Same timestamp and tf frame as input image
				out_msg.header.frame_id = "/tf_cam_roof_fc_6mm";
				out_msg.encoding = sensor_msgs::image_encodings::TYPE_8UC3; // Or whatever
				out_msg.image = cv::imread(fname,1); // Your cv::Mat
				out_msg.toImageMsg(image_rfc_6mm);

				pub_image_rfc_6mm.publish(image_rfc_6mm);
				bag.write("/cam_roof_fc_6mm", event_time, image_rfc_6mm);

				//copy_pixels_to_image_msg_data(img.image, &image_rfc, img.size); 
				lcmtypes_image_t_decode_cleanup(&img);
			}



			ros::spinOnce();
		}



	//}
	bag.close();
	
	lcm_eventlog_free_event(event);
	lcm_eventlog_destroy(log);
	return 0;
}

#endif
/**
 *@}
 */


