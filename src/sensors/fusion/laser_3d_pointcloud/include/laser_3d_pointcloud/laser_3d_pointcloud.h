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
#ifndef _LASER3D_POINTCLOUD_H_
#define _LASER3D_POINTCLOUD_H_

/**
  \brief Header file with class declaration to accumulate pointlouds of the laser3D data.
  \file laser3D_pointcloud.h
  \author Diogo Matos
  \date June 2013
*/

#include <stdio.h>
#include <ros/ros.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>
#include <map>
#include <tf/tf.h>

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/thread/mutex.hpp>



  // NOTE: invalid scan errors (will be present in LaserScan.msg in D-Turtle)
  const float LASER_SCAN_INVALID   = -1.0;
  const float LASER_SCAN_MIN_RANGE = -2.0;
  const float LASER_SCAN_MAX_RANGE = -3.0;

  namespace channel_option
  {
  //! Enumerated output channels options.
  /*!
   * An OR'd set of these options is passed as the final argument of
   * the projectLaser and transformLaserScanToPointCloud calls to
   * enable generation of the appropriate set of additional channels.
   */
    enum ChannelOption
      {
        None = 0x00,      //!< Enable no channels
        Intensity = 0x01, //!< Enable "intensities" channel
        Index     = 0x02, //!< Enable "index" channel
        Distance  = 0x04, //!< Enable "distances" channel
        Timestamp = 0x08, //!< Enable "stamps" channel
        Viewpoint = 0x10, //!< Enable "viewpoint" channel
        Default   = (Intensity | Index) //!< Enable "intensities" and "index" channels
      };
  }

/**
 * \class las3D_PointCloud
 * \brief Class to accumulate point clouds laser3D
 * \date June 2013
 * \author Diogo Matos
 * 
 */

class las3D_PointCloud
{
	public:
		
		/** \brief ROS tf listener */
		tf::TransformListener *p_listener;
		
		/** \brief PointCloud<pcl::PointXYZ> to accumulate the laser scans */
		pcl::PointCloud<pcl::PointXYZ> pc_accumulated;	
		
		/** \brief PointCloud size */
		size_t cloud_size;	
		
		/** \brief Vector with the external shaft angle mensage */
		std::vector<sensor_msgs::JointStatePtr> joints;
		
		/** \brief Number of laser scans received */
		int laser_count;
		
		/** \brief bool laser scan received */
		bool laserscan_arrived;
		
		/** \brief Vector with the laser scans received */
		std::vector<sensor_msgs::LaserScanPtr> scans;
		
		/** \brief Type of accumulation wanted to be used */
		int accumulation_mode;

		/** \brief Max of scans to be accumulated */
		int max_scans_accumulated;
		
		/** \brief Time stamp to the output cloud, when using rosbag or in real time accumulation */
		int pointcloud_stamp;
		
		las3D_PointCloud()
		{
			accumulation_mode=1;
			max_scans_accumulated=450;
			pointcloud_stamp=1;
			
			angle_min_=0;
			angle_max_=0;
			laser_count=0;
			laserscan_arrived=false;
		}
		
		~las3D_PointCloud();
		
		/**
		* \brief Function to accumulate PointCloud.
		* Receives the pointcloud with the transformed laser scan and
		* accumulates it on the class variable pc_accumulated
		* \param 	pc_in pointcloud to transform
		*/
		
		void accumulate_cloud(pcl::PointCloud<pcl::PointXYZ>* pc_in);
      
      
		/**
		*\brief Transform a sensor_msgs::LaserScan into a sensor_msgs::PointCloud2 in target frame using the external information of the external shaft angle.
		* Transform a single laser scan from a linear array into a 3D
		* point cloud, accounting for movement of the laser over the
		* course of the scan.  In order for this transform to be
		* meaningful at a single point in time, the target_frame must
		* be a fixed reference frame. 
		*
		* \param 	target_frame The frame of the resulting point cloud
		* \param 	scan_in The input laser scan
		* \param 	cloud_out The output point cloud
		* \param 	tf a tf::Transformer object to use to perform the
		*   transform
		* \param 	state_start The external shaft angle mensage at the start of the scan
		* \param	state_end	The external shaft angle mensage at the end of the scan
		* \param 	range_cutoff An additional range cutoff which can be
		*   applied which is more limiting than max_range in the scan.
		* \param channel_option An OR'd set of channels to include.
		*   Options include: channel_option::Default,
		*   channel_option::Intensity, channel_option::Index,
		*   channel_option::Distance, channel_option::Timestamp.
		*/
	  
		int las3D_transformLaserScanToPointCloud(const std::string &target_frame, 
                                           const sensor_msgs::LaserScan &scan_in, 
                                           sensor_msgs::PointCloud2 &cloud_out,
											const sensor_msgs::JointState &state_start,
											const sensor_msgs::JointState &state_end,
											tf::Transformer &tf,
                                           double range_cutoff = -1.0,
                                           int channel_options = channel_option::Default)
		{
			int error=0;
			error=las3D_transformLaserScanToPointCloud_(target_frame, scan_in, cloud_out, state_start,state_end, tf, range_cutoff, channel_options);
			return error;				
		}
		
		/**
		*\brief Transform a sensor_msgs::LaserScan into a sensor_msgs::PointCloud2 in target frame using the scan time stamps.
		* Transform a single laser scan from a linear array into a 3D
		* point cloud, accounting for movement of the laser over the
		* course of the scan.  In order for this transform to be
		* meaningful at a single point in time, the target_frame must
		* be a fixed reference frame. 
		*
		* \param 	target_frame The frame of the resulting point cloud
		* \param 	scan_in The input laser scan
		* \param 	cloud_out The output point cloud
		* \param 	tf a tf::Transformer object to use to perform the
		*   transform
		* \param 	range_cutoff An additional range cutoff which can be
		*   applied which is more limiting than max_range in the scan.
		* \param channel_option An OR'd set of channels to include.
		*   Options include: channel_option::Default,
		*   channel_option::Intensity, channel_option::Index,
		*   channel_option::Distance, channel_option::Timestamp.
		*/
		
		int las3D_transformLaserScanToPointCloud(const std::string &target_frame, 
                                           const sensor_msgs::LaserScan &scan_in, 
                                           sensor_msgs::PointCloud2 &cloud_out,
											tf::Transformer &tf,
                                           double range_cutoff = -1.0,
                                           int channel_options = channel_option::Default)
		{
			int error=0;
			error=las3D_transformLaserScanToPointCloud_(target_frame, scan_in, cloud_out, tf, range_cutoff, channel_options);
			return error;				
		}

    protected:

      //! Internal protected representation of getUnitVectors
      /*!
       * This function should not be used by external users, however,
       * it is left protected so that test code can evaluate it
       * appropriately.
       */
      const boost::numeric::ublas::matrix<double>& getUnitVectors_(double angle_min,
                                                                   double angle_max,
                                                                   double angle_increment,
                                                                   unsigned int length);

    private:

	  
		//! Internal hidden representation of las3D_projectLaser_
      void las3D_projectLaser_ (const sensor_msgs::LaserScan& scan_in, 
								sensor_msgs::PointCloud2 &cloud_out,
								double range_cutoff,
								int channel_options);

	  
	  //! Internal hidden representation of las3D_transformLaserScanToPointCloud_
      int las3D_transformLaserScanToPointCloud_ (const std::string &target_frame, 
                                            const sensor_msgs::LaserScan &scan_in,
                                            sensor_msgs::PointCloud2 &cloud_out, 
                                            const sensor_msgs::JointState &state_start,
											 const sensor_msgs::JointState &state_end,
											 tf::Transformer &tf,
                                            double range_cutoff,
                                            int channel_options);
	  
	  int las3D_transformLaserScanToPointCloud_ (const std::string &target_frame, 
																const sensor_msgs::LaserScan &scan_in,
																sensor_msgs::PointCloud2 &cloud_out, 
																tf::Transformer &tf,
																double range_cutoff,
																int channel_options);
	  

      //! Internal map of pointers to stored values
      std::map<std::string,boost::numeric::ublas::matrix<double>* > unit_vector_map_;
      float angle_min_;
      float angle_max_;
      Eigen::ArrayXXd co_sine_map_;
      boost::mutex guv_mutex_;		
	  
}; //end class

#endif
