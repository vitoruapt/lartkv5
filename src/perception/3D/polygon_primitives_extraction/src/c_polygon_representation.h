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
 * @addtogroup c_polygon_representation 
 * @{
 * @file
 * @brief header for polygon representation. Defines how a polygon is stored in
 * memory
 */
#ifndef _C_POLYGON_REPRESENTATION_H_
#define _C_POLYGON_REPRESENTATION_H_

//####################################################################
//// Includes:
////####################################################################
//
#include <ros/ros.h>
//#include <pcl/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
//#include <pcl/point_types.h>
//#include <ros/ros.h>
// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <signal.h>


#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "config.h"
#include "config_util.h"
#include "eventlog.h"
#include "lcmtypes_velodyne_t.h"
#include "lcmtypes_pose_t.h"
#include "small_linalg.h"
#include "velodyne.h"

#include <visualization_msgs/Marker.h>

#include <cmath>

#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/project_inliers.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/segmentation/extract_polygonal_prism_data.h>

//#include "pcl_ros/transforms.h"
#include "pcl_ros/segmentation/extract_polygonal_prism_data.h"

typedef pcl::PointXYZ PointT;
#define _NUM_POLYGONS_ 5
#define _NUM_CLUSTERS_ 5



typedef struct tag_t_polygon_point_cloud
{
    pcl::PointCloud<PointT>::Ptr cloud;
    ros::Publisher publisher;
    char advertising_name[1024];
}t_polygon_point_cloud;


//Define the polygon class
class c_polygon_representation
{
  public:

    /// Constructor. Allocate space for Ptr objecs
    c_polygon_representation(const char* name, ros::NodeHandle *node);

    /// Destructor. free space of Ptr objecs
    ~c_polygon_representation();

    void publish_raw_cloud(void){publish_polygon_point_cloud(&raw);};
    void publish_projected_cloud(void){publish_polygon_point_cloud(&projected);};
    void publish_convex_hull_cloud(void){publish_polygon_point_cloud(&convex_hull);};
    void publish_additional_cloud(void){publish_polygon_point_cloud(&additional);};

    void publish_polygon_point_cloud(t_polygon_point_cloud *ppc)
    {
      ///Fill some required stuff
      ppc->cloud->header.frame_id = "/sensor_frame";
      ppc->cloud->height = 1; //1 since its and unordered pc
      ppc->cloud->is_dense=0;
      ppc->cloud->width = ppc->cloud->size();

      pcl::toROSMsg(*ppc->cloud, cloud_msg);
      ppc->publisher.publish(cloud_msg);
    }

    // ------------------------------------------------------------------------------
    //utility functions
    // ------------------------------------------------------------------------------

   int indices_extraction(pcl::PointIndices::Ptr ind,
                          pcl::PointCloud<PointT> *input_cloud,
                          pcl::PointCloud<PointT> *remove_cloud,
                          pcl::PointCloud<PointT>::Ptr copy_cloud,
                          pcl::PointCloud<pcl::Normal> *input_normals,
                          pcl::PointCloud<pcl::Normal> *remove_normals)
   {
        if (input_cloud!=NULL) //if an input cloud is given
        {
                pcl::ExtractIndices<PointT> extract; //Create the extraction object
                extract.setInputCloud(input_cloud->makeShared());
                extract.setIndices(ind);

                //Copy to copy_cloud if not NULL 
                if(copy_cloud!=NULL)
                {
                        extract.setNegative(false);
                        extract.filter(*copy_cloud);
                }

                //Remove from remove if not NULL
                if(remove_cloud!=NULL)
                {
                        extract.setNegative(true);
                        extract.filter(*remove_cloud);
                }
        }

        if (input_normals!=NULL)
        {
                pcl::ExtractIndices<pcl::Normal> extract_normals; //Create the normal extraction object
                extract_normals.setInputCloud(input_normals->makeShared());
                extract_normals.setIndices(ind);

                if (remove_normals!=NULL)
                {
                        extract_normals.setNegative(true);
                        extract_normals.filter(*remove_normals);
                }
        } 


        
   return 1;}

    //Computes a new polygon candidate using RANSAC from a given pc with normals, ::PointIndices and removes inliers from the input cloud and normals. 
    void compute_plane_candidate_from_pc(pcl::PointCloud<PointT> *input_cloud,
                                         pcl::PointCloud<pcl::Normal> *input_normals,
                                         double DistanceThreshold = 0.5,
                                         double NormalDistanceWeight = 0.5,
                                         int MaxIterations = 1000
                                         );

   void project_raw_cloud_to_plane(void)
   {
        project_cloud_to_plane(raw.cloud->makeShared());
   }

   void project_cloud_to_plane(pcl::PointCloud<PointT>::Ptr input_cloud)
   {
        // Project the model inliers 
        pcl::ProjectInliers<pcl::PointXYZ> projection;

        projection.setModelType(pcl::SACMODEL_NORMAL_PLANE);
        projection.setInputCloud(raw.cloud->makeShared());
        projection.setModelCoefficients(coefficients);
        projection.filter(*projected.cloud);

   } 

   void compute_convex_hull(void)
   {
        // Create a Convex Hull representation of the projected inliers
        pcl::ConvexHull<pcl::PointXYZ> chull;
        chull.setInputCloud(projected.cloud->makeShared());
        chull.reconstruct(*convex_hull.cloud);
   }


void segment_plane_from_point_cloud(
                pcl::PointCloud<PointT> *input_cloud,
                double DistanceThreshold,
                int MaxIterations);


   void compute_convex_hull_inliers(pcl::PointCloud<PointT> *input_cloud, pcl::PointCloud<pcl::Normal> *input_normals)
   {
        //Create variables
        pcl::ExtractPolygonalPrismData<PointT> epp; 
        pcl::PointCloud<PointT>::ConstPtr input_cloud_constptr; 
        input_cloud_constptr.reset (new pcl::PointCloud<PointT> (*input_cloud));
        pcl::PointCloud<PointT>::ConstPtr convex_hull_constptr;
        convex_hull_constptr.reset (new pcl::PointCloud<PointT> (*convex_hull.cloud));
        pcl::PointIndices::Ptr ind =  pcl::PointIndices::Ptr(new pcl::PointIndices);
        pcl::ExtractIndices<PointT> extract; //Create the extraction object

        //Start processing
        ROS_INFO("Extracting Convex Hull inliers");
        ROS_INFO("Input cloud has %d points",(int)input_cloud->size());
 

        //Set epp parameters
        epp.setInputCloud(input_cloud_constptr);
        epp.setInputPlanarHull(convex_hull_constptr);
        epp.setHeightLimits(-0.2, 0.2); 
        epp.setViewPoint(0,0,0); 

        //Segment
        epp.segment(*ind);


        indices_extraction(ind,
                          input_cloud,
                          input_cloud,
                          additional.cloud,
                          input_normals,
                          input_normals);


        //Extract the plane inliers from the input cloud. Generate raw.cloud
        //extract.setInputCloud(input_cloud->makeShared());
        //extract.setIndices (boost::make_shared<const pcl::PointIndices> (ind));
        //extract.setIndices (ind);
        //extract.setNegative(false);
        //extract.filter(*additional.cloud);

        //Remove the plane inliers from the input_cloud
        //extract.setNegative(true);
        //extract.filter(*input_cloud);

        //Remove plane inliers from the input_normals
        //pcl::ExtractIndices<pcl::Normal> extract_normals;
        //extract_normals.setNegative (true);
        //extract_normals.setInputCloud(input_normals->makeShared());
        ////extract_normals.setIndices(boost::make_shared<const pcl::PointIndices> (ind));
        //extract_normals.setIndices(ind);
        //extract_normals.filter(*input_normals);

        (*raw.cloud) += (*additional.cloud);

        //segment_plane_from_point_cloud(&raw.cloud, 3.0, 100);

        //Setup segmentation parameters
        pcl::SACSegmentation<PointT> segmentation; // Create the segmentation object

        segmentation.setOptimizeCoefficients(true);
        segmentation.setModelType(pcl::SACMODEL_PLANE);
        segmentation.setMethodType (pcl::SAC_RANSAC); 
        segmentation.setMaxIterations (100);
        segmentation.setDistanceThreshold(333);

        ROS_INFO("Indices before %d ...",(int)raw.cloud->size());   

        //Compute  a plane candidate from the point cloud
        segmentation.setInputCloud(input_cloud->makeShared());
        segmentation.segment(*indices, *coefficients);

        if (indices->indices.size () == 0)
        {
             ROS_ERROR("Could not estimate a planar model for the given dataset."); return;
        }

        ROS_INFO("Found %d inliers of polygon",(int)additional.cloud->size());   
        ROS_INFO("Removed %d points from input_cloud. Now  has %d points",(int)additional.cloud->size(),(int)input_cloud->size());   

       
        ROS_INFO("Extracting Convex Hull inliers done ...");
   }
        
//Build the condition 
//pcl::ConditionAnd<PointT>::Ptr range_cond (new pcl::ConditionAnd<PointT> ()); 
//range_cond->addComparison (pcl::FieldComparison<PointT>::Ptr (new pcl::FieldComparison<PointT>("z", pcl::ComparisonOps::LT, 2.0))); 
//range_cond->addComparison (pcl::FieldComparison<PointT>::Ptr (new pcl::FieldComparison<PointT>("z", pcl::ComparisonOps::GT, 0.0))); 

//Build the filter 
//pcl::ConditionalRemoval<PointT> range_filt; 
//range_filt.setCondition (range_cond); 
//range_filt.setKeepOrganized (false);



    // Variables for polyplane segmentation and representation
    pcl::ModelCoefficients::Ptr coefficients;
    pcl::PointIndices::Ptr indices;
    t_polygon_point_cloud raw, projected, convex_hull, additional;
    //pcl::PointCloud<PointT> cluster[_NUM_CLUSTERS_];

  private:


    //sprintf(str, "plg_%d",i);
    //pub[i] = n.advertise<sensor_msgs::PointCloud2>(str, 1);
    ros::NodeHandle *rosnode;
    char polygon_name[1024];
    sensor_msgs::PointCloud2 cloud_msg;

    
//void pcl::SampleConsensusModelPlane< PointT >::optimizeModelCoefficients 	( 	const std::vector< int > &  	inliers,
		//const Eigen::VectorXf &  	model_coefficients,
		//Eigen::VectorXf &  	optimized_coefficients	 
	//) 			[virtual]




    /// set names
    void set_names(const char* s)
    { 
      sprintf(polygon_name,"%s",s);
      sprintf(raw.advertising_name,"%s/raw",s);
      sprintf(additional.advertising_name,"%s/additional",s);
      sprintf(projected.advertising_name,"%s/projected",s);
      sprintf(convex_hull.advertising_name,"%s/convex_hull",s);
    }


    /** allocates space all for pointers*/
    void allocate_space(void);
    
      

};
#endif
 /**
 *@}
 */
/*Previous 3 lines appended automatically on Sun Feb  5 19:40:16 WET 2012 */
