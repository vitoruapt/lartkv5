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
///Ros includes
#include <ros/ros.h>
#include <pcl_ros/transforms.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>

/**
  \file pc_accumulation.h
  \brief A class to accumulation pointlouds
  \author Pedro Salvado
*/


/**
 * \class pc_accumulation
 * Class to accumulate point_clouds
 * \date March 2012
 * \author Pedro Salvado
 * 
 */
using namespace std;
class pc_accumulation
{
	public:

		tf::TransformListener *p_listener;
		ros::NodeHandle* p_n;
		double voxel_size;
		std::string _acc_frame_id;
		
		nav_msgs::Odometry odometry_;
		ros::Time time_msg;

		sensor_msgs::PointCloud2 msg_cloud;

		pcl::PointCloud<pcl::PointXYZRGB> pcl_pc_acc; //accumulation pc
		
		   /**
		   * \brief Function that receive a LaserScan and Accumulation Frame 
		   * It converts the laserScan to a pcl::PointXYZRGB and call's the pointcloud_accumulated (main function)
		   * \param[in]  sensor_msgs::LaserScan
		   * \param[in]  std::string
		   */
		int pointcloud_accumulated(sensor_msgs::LaserScan& scan_in,std::string acc_frame_id)
		{
			
		          laser_geometry::LaserProjection projector_;
		          pcl::PointCloud<pcl::PointXYZ> pc_tmp;
		          pcl::PointCloud<pcl::PointXYZRGB> pc_laser;
		          sensor_msgs::PointCloud2 poincloud_in;
		          //Transformacao de LaserScan para PointCloud
		          projector_.transformLaserScanToPointCloud(scan_in.header.frame_id,scan_in,poincloud_in,*p_listener);


                    pcl::PCLPointCloud2 pcl_pc;
                    pcl_conversions::toPCL(poincloud_in, pcl_pc);
                    pcl::fromPCLPointCloud2(pcl_pc, pc_tmp);

		          pcl::fromROSMsg(poincloud_in,pc_tmp);
// 		          pc_tmp.header.frame_id=scan_in.header.frame_id;

		          pc_laser.points.resize(pc_tmp.size());

		          for (size_t i = 0; i < pc_laser.points.size(); i++)
		          {

                            pc_laser.points[i].x = pc_tmp.points[i].x;
                            pc_laser.points[i].y = pc_tmp.points[i].y;
                            pc_laser.points[i].z = pc_tmp.points[i].z;
                            pc_laser.points[i].r = 0.0;
                            pc_laser.points[i].g = 0.0;
                            pc_laser.points[i].b = 0.0;
                   }
		           pc_laser.header.frame_id =  scan_in.header.frame_id;
                   
                   std::cout<<"Message stamp not set!! please correct"<<std::endl;
//                    pc_laser.header.stamp =  scan_in.header.stamp;
                   
		          return pointcloud_accumulated(pc_laser,acc_frame_id);
	            };

		   /**
		   * \brief Function that receive a PointCloud2 and Accumulation Frame 
		   * If PointCloud2 has no field "rgb" then it is responsible to add it
		   * \param[in]  sensor_msgs::PointCloud2
		   * \param[in]  std::string
		   */
		int pointcloud_accumulated(sensor_msgs::PointCloud2& pc_in,std::string acc_frame_id)
		{
			pcl::PointCloud<pcl::PointXYZRGB> pcl_pc; //temporary pcl
			for (size_t i=0; i <pc_in.fields.size();i++)
			{
			        if (pc_in.fields[i].name=="rgb")
			        {
			               ROS_ERROR("TEM RGB");
			               
				      pcl::fromROSMsg(pc_in,pcl_pc);
			        }
			        else
			        {
			               
				      pcl::PointCloud<pcl::PointXYZ> pcl_tmp; //temporary pcl
				      pcl::fromROSMsg(pc_in,pcl_tmp);
				      pcl_pc.points.resize(pcl_tmp.size());
			               for (size_t i = 0; i < pcl_pc.points.size(); i++)
			               {
				      pcl_pc.points[i].x = pcl_tmp.points[i].x;
				      pcl_pc.points[i].y = pcl_tmp.points[i].y;
				      pcl_pc.points[i].z = pcl_tmp.points[i].z;
				      pcl_pc.points[i].r = 0.0;
				      pcl_pc.points[i].g = 0.0;
				      pcl_pc.points[i].b = 0.0;
			               }

			        }
			}
			pcl_pc.header.frame_id=pc_in.header.frame_id;
			return pointcloud_accumulated(pcl_pc,acc_frame_id);
			
		};
		int pointcloud_accumulated(pcl::PointCloud<pcl::PointXYZRGB> pcl_pc,std::string acc_frame_id);
		

		    /**
		   * \brief Function the removes points from cloud 
		   * This function calculated the origin of the given accumulation frame_id
		   * \param[in]  pcl::PointCloud<pcl::PointXYZRGB>
		   * \param[in]  float
		   * \param[in]  std::string
		   */
		   int remove_points_from_pointcloud(pcl::PointCloud<pcl::PointXYZRGB> pcl_pc,float dist,std::string frame_id)
		   {
			tf::StampedTransform transform;
			
			try
			{
                 std::cout<<"Message stamp not set!! please correct"<<std::endl;
                p_listener->lookupTransform(frame_id,_acc_frame_id ,  ros::Time::now(), transform);
// 			        p_listener->lookupTransform(frame_id,_acc_frame_id ,  pcl_pc.header.stamp, transform);
			}
			catch (tf::TransformException ex)
			{
			        ROS_ERROR("%s",ex.what());
			        return 0;
			}
// 			ROS_INFO("get origin");
			double X0 = transform.getOrigin().x();
			double Y0 = transform.getOrigin().y();
			double Z0 = transform.getOrigin().z();

			return remove_points_from_pointcloud(pcl_pc,dist, X0, Y0, Z0);
		}

		int remove_points_from_pointcloud(pcl::PointCloud<pcl::PointXYZRGB> pcl_pc,float dist, float X0, float Y0, float Z0);

private:
		
};
