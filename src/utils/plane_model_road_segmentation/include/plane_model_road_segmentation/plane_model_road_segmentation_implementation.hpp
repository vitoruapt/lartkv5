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

#ifndef _PLANE_MODEL_ROAD_SEGMENTATION_HPP_
#define _PLANE_MODEL_ROAD_SEGMENTATION_HPP_

/**
 * \file
 * \brief Function implementation for class plane_model_road< T >
 * \author Tiago Talhada
 **/

	  // PUBLIC MEMBER FUNCTIONS

template <class T> 
tf::Transform plane_model_road<T>::get_transform(T origin_projected, T pt_x_projected, T pt_y_projected)
{
  tf::Transform transform2;
  tf::Quaternion quaternion;
  transform2.setOrigin( tf::Vector3((float)origin_projected.x, (float)origin_projected.y, (float)origin_projected.z) );
  
  // Calculate RP angles  (R and P only)
  
  vector<double> RP=find_RP_angle(origin_projected, pt_x_projected, pt_y_projected);
  
  if (debug)
  {
    cout << "PMS_DEBUG: R= " << RP[0]<< " P= " << RP[1]<< " Y= "<< RP[3] << endl;
  }
  
  quaternion.setRPY(RP[0]-(pi/2),RP[1]-(pi/2),0.0); // Y does not matter
  transform2.setRotation(quaternion);  
  return transform2;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

template<class T>
bool plane_model_road<T>::find_road_frame(pcl::PointCloud<T> cloud_in)
{
  // STEP 1 create the model coeeficients
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  plane_model_road::outliers=plane_model_road::get_plane_coefficients(cloud_in, plane_model_road::iterations, plane_model_road::threshold ,coefficients);
  
  plane_model_road::coef=coefficients;
  
  //STEP 2 project origin to plane, point in x and another in y
  T  origin_projected=plane_model_road::project_point_to_plane(plane_model_road::point_origin, coefficients);
  T  pt_x_projected=plane_model_road::project_point_to_plane(plane_model_road::point_x, coefficients);
  T  pt_y_projected=plane_model_road::project_point_to_plane(plane_model_road::point_y, coefficients);
  
  plane_model_road::origin_projected=origin_projected;
  plane_model_road::pt_x_projected=pt_x_projected;
  plane_model_road::pt_y_projected=pt_y_projected;
  
  // STEP 3 Find the difference between the cloud frame_id to ground
  T point2 = plane_model_road::transform_pcl_point(cloud_in.header.frame_id,plane_model_road::reference_ground_frame,plane_model_road::point_origin);
  
  //STEP 4 Calculate the difference  z axis
  double distance=point2.z-origin_projected.z;
  distance=abs(distance); 
  if (debug)
    cout << "PMS_DEBUG: Distance to reference frame "<< distance << endl;
  
  bool succeed=plane_model_road::check_distance(distance,threshold);
  return succeed;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

template<class T>
bool plane_model_road<T>::find_road_frame(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  pcl::PointCloud<T> cloud_in;
//   pcl::fromROSMsg(*msg,cloud_in);
  pcl::PCLPointCloud2 pcl_pc;
  pcl_conversions::toPCL(*msg, pcl_pc);
  pcl::fromPCLPointCloud2(pcl_pc, cloud_in);
    
  return plane_model_road::find_road_frame(cloud_in);
};
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

template<class T>
tf::Transform plane_model_road<T>::find_and_publish_road_frame(pcl::PointCloud<T> cloud_in)
{
  plane_model_road::outliers=cloud_in;
  bool succeed=plane_model_road::find_road_frame(plane_model_road::outliers);
  static tf::Transform transform;
  transform=plane_model_road::get_transform(plane_model_road::origin_projected, plane_model_road::pt_x_projected, plane_model_road::pt_y_projected);
  if (succeed)
  {
    plane_model_road::broadcast_ptr->sendTransform(tf::StampedTransform(transform, ros::Time::now(),cloud_in.header.frame_id, plane_model_road::pub_frame_name));
  }
  return transform;
};
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

template<class T>
tf::Transform plane_model_road<T>::find_and_publish_road_frame(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  pcl::PointCloud<T> cloud_in;
  pcl::fromROSMsg(*msg,cloud_in);
  return plane_model_road::find_and_publish_road_frame(cloud_in);
};
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

template<class T>
pcl::PointCloud<T> plane_model_road<T>::get_plane_coefficients(pcl::PointCloud<T> cloud_in, int iterations, float threshold, pcl::ModelCoefficients::Ptr coefficients)
{
  typename pcl::PointCloud<T>::Ptr cloud_in_ptr  (new pcl::PointCloud<T>  (cloud_in));
  pcl::PointCloud<T> cloud_out;
  //   pcl::ModelCoefficients::Ptr coefficients_ptr (new pcl::ModelCoefficients (coefficients));
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  
  // Create the segmentation object
  pcl::SACSegmentation<T> seg;
  pcl::ExtractIndices<T> extract;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (iterations);
  seg.setDistanceThreshold (threshold);
  seg.setInputCloud (cloud_in_ptr);
  seg.segment (*inliers, *coefficients);
  extract.setInputCloud (cloud_in_ptr);
  extract.setIndices (inliers);
  extract.setNegative (true);
  extract.filter (cloud_out);
  
  cloud_in_ptr.reset();
  inliers.reset();
  return cloud_out;
};
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	// PRIVATE CLASS FUNCTIONS


template<class T>
T plane_model_road<T>::project_point_to_plane(const T ptin, const pcl::ModelCoefficients::Ptr coeff)
{
  pcl::PointCloud<T> pcin;
  pcin.points.push_back(ptin);
  
  typename pcl::PointCloud<T>::Ptr pcin_ptr  (new pcl::PointCloud<T> (pcin) );
  //     pcin_ptr=&pcin;
  
  pcl::PointCloud<T> pcout;
  //Create the projection object
  pcl::ProjectInliers<T> projection;
  projection.setModelType(pcl::SACMODEL_PLANE); //set model type
  projection.setInputCloud(pcin_ptr);
  projection.setModelCoefficients(coeff);
  projection.filter(pcout);
  T ptout = pcout.points[0];
  return ptout;
};
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

template<class T>
T plane_model_road<T>::transform_pcl_point(std::string frame1, std::string frame2, T pt_origin)
{
  tf::StampedTransform transform_ground;
  try
  {
    plane_model_road::listener_atc_ground_ptr->lookupTransform(frame1, frame2, ros::Time(0), transform_ground);
  }
  catch (tf::TransformException ex)
  { 
    ROS_ERROR("%s",ex.what());
  }
  
  pcl::PointCloud<T> pc_point;   // A point cloud with a single point
  pc_point.points.push_back(pt_origin);
  pcl_ros::transformPointCloud (pc_point,pc_point,transform_ground);
  return pc_point.points[0];
};
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

template<class T>
vector<double> plane_model_road<T>::find_RP_angle(T origin_projected, T pt_x_projected, T pt_y_projected)
{
  double R,P;
  
  // Vector definition
  
  vector<double> vect_x;
  vect_x.push_back(pt_x_projected.x-origin_projected.x);
  vect_x.push_back(pt_x_projected.y-origin_projected.y);
  vect_x.push_back(pt_x_projected.z-origin_projected.z);
  
  vector<double> vect_y;
  vect_y.push_back(pt_y_projected.x-origin_projected.x);
  vect_y.push_back(pt_y_projected.y-origin_projected.y);
  vect_y.push_back(pt_y_projected.z-origin_projected.z);
  
  vector<double> vect_z;
  vect_z.push_back(0);  vect_z.push_back(0);  vect_z.push_back(1);
  
  double norm_x=sqrt(pow(vect_x[0],2)+pow(vect_x[1],2)+pow(vect_x[2],2));
  double norm_y=sqrt(pow(vect_y[0],2)+pow(vect_y[1],2)+pow(vect_y[2],2));
  double norm_z=sqrt(pow(vect_z[0],2)+pow(vect_z[1],2)+pow(vect_z[2],2));
  
  R=acos((vect_y[0]*vect_z[0]+vect_y[1]*vect_z[1]+vect_y[2]*vect_z[2])/norm_y*norm_z);
  P=acos((vect_x[0]*vect_z[0]+vect_x[1]*vect_z[1]+vect_x[2]*vect_z[2])/norm_x*norm_z);
  
  vector<double> result;
  result.push_back(R); result.push_back(P);
  
  return result;
}

#endif
