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
 * @brief functions to read velodyne data
 *@{
 */
#ifndef _READ_VELODYNE_CPP_
#define _READ_VELODYNE_CPP_

#include "read_velodyne.h"

class c_polygon_representation
{
  public:

    /// Constructor. Allocate space for Ptr objecs
    c_polygon_representation()
    {
     coefficients =  pcl::ModelCoefficients::Ptr(new pcl::ModelCoefficients);
     indices = pcl::PointIndices::Ptr(new pcl::PointIndices); 
    
    }

    /// destructor. free space of Ptr objecs
    ~c_polygon_representation()
    {
     delete &coefficients;
     delete &indices;
    }


    /// Variables for polyplane segmentation and representation
    pcl::ModelCoefficients::Ptr coefficients;
    pcl::PointIndices::Ptr indices;
    pcl::PointCloud<PointT> cloud;



  private:



};

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

/// --------------------------------------------------------------------
/// Following tutorial on http://www.ros.org/wiki/navigation/Tutorials/RobotSetup/Sensors#Publishing_PointClouds_over_ROS
///Point_Cloud_2 msg is defined at http://www.ros.org/doc/api/sensor_msgs/html/msg/PointCloud2.html
/// --------------------------------------------------------------------

int main(int argc, char **argv)
{
	 if(argc < 4) {
        fprintf(stderr, 
                "usage: example4-velodyne <logfile> <start_time> <end time>\n"
                "\n"
                "start_time and end_time are given in seconds from the\n"
                "start of the log file\n");
        return 1;
    }

    lcm_eventlog_t *log = lcm_eventlog_create(argv[1], "r");
    if(!log) {
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

    // compute the desired start and end timestamps
    int64_t start_utime = first_timestamp + (int64_t)(start_time * 1000000);
    int64_t end_utime = first_timestamp + (int64_t)(end_time * 1000000);

    lcmtypes_pose_t last_pose;
    memset(&last_pose, 0, sizeof(last_pose));


  /// Set default values of variables, init pointers, etc
	t_flags.debug=false;

	/// Initialize ROS coms
  ros::init(argc, argv, "point_cloud_publisher");

  ros::NodeHandle n;
  ros::Publisher cloud_pub = n.advertise<sensor_msgs::PointCloud2>("cloud", 1);

  ros::Publisher pub[10];

  for (int i=0; i< _NUM_POLYGONS_; i ++)
  {
    char str[256];
    sprintf(str, "plg_%d",i);
    pub[i] = n.advertise<sensor_msgs::PointCloud2>(str, 1);
  }

  ///Marker publisher
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);


  ros::Rate r(0.5);

  //pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT> cloud;
  sensor_msgs::PointCloud2 cloud_msg;
  cloud.header.frame_id = "/sensor_frame";
  cloud.height = 1;
  cloud.is_dense=0;

  while(1)
  {
        // release the last event
        lcm_eventlog_free_event(event);

        // read an event
        event = lcm_eventlog_read_next_event(log);

        if(!event)
            break;

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

        if(!strcmp(event->channel, "VELODYNE")) {
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
                if (vsample.range < 0.01)
                {
                    continue;
                }

                double sensor_xyz[4] = {vsample.xyz[0], vsample.xyz[1], vsample.xyz[2], 1 };
                double local_xyz[4];

                matrix_vector_multiply_4x4_4d(velodyne_to_local, sensor_xyz, local_xyz);

                //printf("%f %f %f\n", local_xyz[0], local_xyz[1], local_xyz[2]);

   
                PointT Pt;
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
}
 
  while(n.ok())
  {

    visualization_msgs::Marker markers;
    markers.header.frame_id = "/sensor_frame";
    markers.header.stamp = ros::Time::now();
    markers.ns = "triangles";
    markers.action = visualization_msgs::Marker::ADD;
    markers.pose.orientation.w = 1.0;
    markers.type = visualization_msgs::Marker::TRIANGLE_LIST;

    markers.scale.x = 1;
    markers.scale.y = 1;
    markers.scale.z = 1;
    
    markers.color.r = 1;
    markers.color.g = 1;
    markers.color.b = 1;
    markers.color.a = 1;

    geometry_msgs::Point p;
    std_msgs::ColorRGBA color;
    p.x = 0;
    p.y = 0;
    p.z = 0;
    markers.points.push_back(p);
    color.r = 1.0f;
    color.g = 0.2f;
    color.b = 1.0f;
    color.a = 1.0;
    markers.colors.push_back(color);

    p.x = 10;
    p.y = 10;
    p.z = 0;
    markers.points.push_back(p);
    color.r = 1.0f;
    color.g = 0.2f;
    color.b = 0.5f;
    color.a = 1.0;
    markers.colors.push_back(color);

    p.x = 10;
    p.y = -10;
    p.z = 0;
    markers.points.push_back(p);
    color.r = 0.8f;
    color.g = 0.2f;
    color.b = 1.0f;
    color.a = 1.0;
    markers.colors.push_back(color);

    marker_pub.publish(markers);

    printf("num pts = %d\n",(int)markers.points.size());
    printf("num colors = %d\n",(int)markers.colors.size());

//     cloud.header.stamp = ros::Time::now();
    ROS_ERROR("Cloud timestamp not set, problem during migration lar3 to lar4");
    ROS_INFO("publishing cloud with %d points",(int)cloud.size());

    pcl::toROSMsg(cloud, cloud_msg);
    cloud_pub.publish(cloud_msg);

    c_polygon_representation *plg[10];

    /// Create Model coefficients and array of model point clouds
    //pcl::ModelCoefficients coefficients;
    //pcl::PointIndices::Ptr indices_model(new pcl::PointIndices);
    //pcl::PointCloud<PointT> cloud_model;
    
    ///Create the Pointcloud2 msg object
    sensor_msgs::PointCloud2 cloud_model_msg;


    /// Create the segmentation object
    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC); 
    seg.setDistanceThreshold (0.3);
    seg.setMaxIterations (1000);

    ///Create the extraction object
    pcl::ExtractIndices<PointT> extract;

    
    pcl::PointCloud<PointT> cloud_tmp;
    cloud_tmp = cloud;

    ///start iterative process
    for (int i=0; i<_NUM_POLYGONS_;i++)
    {

      plg[i] = new c_polygon_representation;
      /// now start to segment the point cloud
      seg.setInputCloud(cloud_tmp.makeShared());
      seg.segment(*plg[i]->indices, *plg[i]->coefficients);

      if (plg[i]->indices->indices.size () == 0)
      {
        ROS_ERROR("Could not estimate a planar model for the given dataset."); return (-1);
      }
      //std::cerr << "Model coefficients: " << coefficients[i].values[0] << " " << coefficients[i].values[1] << " " << coefficients[i].values[2] << " " coefficients[i].values[3] << std::endl;
      //std::cerr << "Model inliers: " << model[i].size () << std::endl;

      ///Extract the planar inliers from the input cloud
      extract.setInputCloud(cloud_tmp.makeShared());
      extract.setIndices(plg[i]->indices);
      extract.setNegative(false);
      extract.filter(plg[i]->cloud);
      extract.setNegative(true);
      extract.filter(cloud_tmp);
        }

    ///Now lets publish
    for (int i=0; i<_NUM_POLYGONS_;i++)
    {

      ///Fill some required stuff
      plg[i]->cloud.header.frame_id = "/sensor_frame";
      plg[i]->cloud.height = 1; //1 since its and unordered pc
      plg[i]->cloud.is_dense=0;
      plg[i]->cloud.width = plg[i]->cloud.size();
      ROS_INFO("publishing cloud_model[%d] with %d points",i,(int)plg[i]->cloud.size());
      pcl::toROSMsg(plg[i]->cloud, cloud_model_msg);
      pub[i].publish(cloud_model_msg);
    }










  //for (size_t i = 0; i < inliers.indices.size (); ++i)
  //{
    //Pt.x = cloud.points[inliers.indices[i]].x;
    //Pt.y = cloud.points[inliers.indices[i]].y;
    //Pt.z = cloud.points[inliers.indices[i]].z;

    ///// From http://docs.pointclouds.org/trunk/structpcl_1_1_point_x_y_z_r_g_b.html
    //// pack r/g/b into rgb
    //uint8_t r = 255, g = 0, b = 0;    // Example: Red color
    //uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
    //Pt.rgb = *reinterpret_cast<float*>(&rgb);

    //cloud_inliers.push_back(Pt);
  //}

    //cloud_inliers.header.stamp = ros::Time::now();
    //cloud_inliers.width = cloud_inliers.size();
    //ROS_INFO("publishing cloud_inliers with %d points",cloud_inliers.size());
    //pcl::toROSMsg(cloud_inliers, cloud_inliers_msg);
    //cloud_inliers_pub.publish(cloud_inliers_msg);

    //pcl::ExtractIndices<PointXYZ> extract;

    

    r.sleep();


}
     lcm_eventlog_free_event(event);

    lcm_eventlog_destroy(log);
   

	 return 0;
}

#endif
/**
*@}
*/

