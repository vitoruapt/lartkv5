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
 * @addtogroup polygon_representation 
 * @{
 * @file 
 * @brief This code provides a data structure to have a polygonal primitive
 * representation
 */
#ifndef _C_POLYGON_REPRESENTATION_CPP_
#define _C_POLYGON_REPRESENTATION_CPP_


#include "c_polygon_representation.h"

c_polygon_representation::c_polygon_representation(const char* name, ros::NodeHandle *node)
{
        allocate_space();
        set_names(name);
        rosnode = node;
        raw.publisher = rosnode->advertise<sensor_msgs::PointCloud2>(raw.advertising_name, 1);
        additional.publisher = rosnode->advertise<sensor_msgs::PointCloud2>(additional.advertising_name, 1);
        projected.publisher = rosnode->advertise<sensor_msgs::PointCloud2>(projected.advertising_name, 1);
        convex_hull.publisher = rosnode->advertise<sensor_msgs::PointCloud2>(convex_hull.advertising_name, 1);
        ROS_INFO("__________________________________");
        ROS_INFO("Starting a new polygon class named %s",polygon_name);
};

c_polygon_representation::~c_polygon_representation()
{
        delete &coefficients;
        delete &indices;
};

void c_polygon_representation::allocate_space(void)
{
        raw.cloud = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
        additional.cloud = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
        projected.cloud = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
        convex_hull.cloud = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
        coefficients =  pcl::ModelCoefficients::Ptr(new pcl::ModelCoefficients);
        indices = pcl::PointIndices::Ptr(new pcl::PointIndices); 
}

void c_polygon_representation::compute_plane_candidate_from_pc(
                pcl::PointCloud<PointT> *input_cloud,
                pcl::PointCloud<pcl::Normal> *input_normals, 
                double DistanceThreshold,
                double NormalDistanceWeight,
        int MaxIterations)
{
int num_original_pts = (int)input_cloud->size();

// Create the segmentation object
pcl::SACSegmentationFromNormals<PointT, pcl::Normal> segmentation; 

//Setup segmentation parameters
segmentation.setOptimizeCoefficients(true);
segmentation.setModelType(pcl::SACMODEL_NORMAL_PLANE);
segmentation.setMethodType (pcl::SAC_RANSAC); 
segmentation.setMaxIterations (MaxIterations);
segmentation.setNormalDistanceWeight (NormalDistanceWeight);
segmentation.setDistanceThreshold(DistanceThreshold);

//Compute  a plane candidate from the point cloud
segmentation.setInputCloud(input_cloud->makeShared());
segmentation.setInputNormals(input_normals->makeShared());
segmentation.segment(*indices, *coefficients);

if (indices->indices.size () == 0)
{
     ROS_ERROR("Could not estimate a planar model for the given dataset."); return;
}

indices_extraction(indices,
                   input_cloud,
                   input_cloud,
                   raw.cloud,
                   input_normals,
                   input_normals);


ROS_INFO("COMPUTING A PLANE CANDIDATE: candidate with %d points from an input pc with %d points",(int)raw.cloud->size(), num_original_pts);   

}

void c_polygon_representation::segment_plane_from_point_cloud(
                pcl::PointCloud<PointT> *input_cloud,
                double DistanceThreshold,
                int MaxIterations)
    {

        //Setup segmentation parameters
        pcl::SACSegmentation<PointT> segmentation; // Create the segmentation object

        segmentation.setOptimizeCoefficients(true);
        segmentation.setModelType(pcl::SACMODEL_PLANE);
        segmentation.setMethodType (pcl::SAC_RANSAC); 
        segmentation.setMaxIterations (MaxIterations);
        segmentation.setDistanceThreshold(DistanceThreshold);

        ROS_INFO("Indices before %d ...",(int)raw.cloud->size());   

        //Compute  a plane candidate from the point cloud
        segmentation.setInputCloud(input_cloud->makeShared());
        segmentation.segment(*indices, *coefficients);

        if (indices->indices.size () == 0)
        {
             ROS_ERROR("Could not estimate a planar model for the given dataset."); return;
        }
        


    }

#endif
/**
 *@}
 */      
