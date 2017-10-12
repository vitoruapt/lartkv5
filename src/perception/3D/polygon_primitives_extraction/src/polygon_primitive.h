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
 * @defgroup polygon_primitive  polygon_primitive -Automatic Group Name-Please adjust. vs strikes again :-) 05-Feb-2012
 * @{
 * @file 
 * @brief A class c_polygon_primitive that contains information about a detected polygon primitive as well as the methods for detection 
 * @author Miguel Armando Riem de Oliveira
 * @version 0.0
 * @date 2011-12-15
 */
#ifndef _polygon_primitive_H_
#define _polygon_primitive_H_

/**
 */

#define PFLN {printf("DEBUG PRINT FILE %s LINE %d\n",__FILE__,__LINE__);}

//####################################################################
//// Includes:
////####################################################################

///Ros includes
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_broadcaster.h>      

///PCL includes
#include "pcl_ros/impl/transforms.hpp"
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
// #include <pcl-1.6/pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/extract_clusters.h>
//#include <pcl/segmentation/extract_labeled_clusters.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
//#include <pcl_ros/segmentation/extract_polygonal_prism_data.h>

///Opencv includes
#include <opencv2/core/core.hpp>

///polygon_primitive_msg include
#include <polygon_primitive_msg/polygon_primitive.h>

//The interval variables
/** @brief A pcl_polygon_3d is a definition of a polygon in 3d, composed of a set of points that when united produce the polygon. In reality the pcl_polygon_3d is just a pcl::PointCloud<PointT>, where PointT can be any pcl 3d point types, for example pcl::PointXYZ
 * @tparam PointT PointType can be for example a pcl::PointXYZ, or any 3d point type of pcv
 */
template <class PointT>
class polygon_3d:public pcl::PointCloud<PointT>{};


typedef struct {double x,y,z;}t_vector3d;
typedef struct 
{
	t_vector3d origin;
	t_vector3d arrow_x;
	t_vector3d arrow_y;
	t_vector3d arrow_z;
	tf::Transform transform; ///The transform that in direct mode (not inverse) bring the points in the local polygon frame coordinates (where z=0) to globl point coordinates
}t_reference_frame;

typedef struct
{
	struct
	{
		/**
		 * @brief the color of the polygon
		 */
		struct 
		{
			unsigned char r,g,b;
		}color; 

		char name[1024];
		bool ground_search;

	}misc;

	/**
	 * @brief Information about the several hulls used
	 */
	struct
	{
		/**
		 * @brief data from the polygon convex hull
		 */
		struct
		{
			polygon_3d<pcl::PointXYZ>::Ptr polygon, extended_polygon;
			double area; /// the area of the convex hull
			double solidity; ///the solidity of the convex hull
		}convex;

		/**
		 * @brief data from the polygon concave hull
		 */
		struct
		{
			polygon_3d<pcl::PointXYZ>::Ptr polygon, extended_polygon;
			double area; /// the area of the concave hull
			double solidity; ///the solidity of the concave hull
		}concave;
	}hulls;

	/**
	 * @brief The supporting plane coefficients
	 */
	struct
	{
		pcl::ModelCoefficients::Ptr current; /// current supporting plane coefficients
		pcl::ModelCoefficients::Ptr previous; /// previous (before expansion) supporting plane coefficients
	}planes;

	/**
	 * @brief the axis frames
	 */
	struct
	{
			std::string local_name;
			std::string global_name;
		t_reference_frame current; ///the current reference frame
		t_reference_frame previous; ///the previous (before expansion) frame
	}frames;

}t_polygon_primitive_data;


//Define the polygon class
/**
 * @brief The polygon_primitive class. Defines methods for polygon extraction and expansion from a point cloud. Stores information about the polygon like the number of supporting points, supporting plane, convex hull, etc 
 */
class c_polygon_primitive
{
	// ------------------------------------------------------------------------------
	//PUBLIC SECTION OF THE CLASS
	// ------------------------------------------------------------------------------
	public:


		/**
		 * @brief A structure containing all the point clouds needed to represent the polygon primitive
		 */
		struct {
			pcl::PointCloud<pcl::PointXYZ>::Ptr 
				all, ///A point cloud with all supporting points
				projected, ///A point cloud with all supporting points
				additional, ///The points that are to be added to the point cloud after an expand operation
				growed,///Dont know
				tmp; ///nedeed
		}pointclouds;

		ros::NodeHandle *rosnode;
		t_polygon_primitive_data data;

	private:
		tf::TransformBroadcaster br;
		sensor_msgs::PointCloud2 cloud_msg;
		unsigned int grow_number;    


		//--------------------------Now the functions --------------------
	public:

		//-------------------------------------------
		//--- DEFINED IN polygon_primitive.cpp
		//-------------------------------------------
		c_polygon_primitive(ros::NodeHandle *node, const char* name="unamed", unsigned char r=0, unsigned char g=0, unsigned char b=0);
		~c_polygon_primitive();

		//-------------------------------------------
		//--- DEFINED IN polygon_primitive_comunications.cpp
		//-------------------------------------------
		int export_to_polygon_primitive_msg(polygon_primitive_msg::polygon_primitive* msg);
		int import_from_polygon_primitive_msg(polygon_primitive_msg::polygon_primitive* msg);
		int create_vizualization_msgs(visualization_msgs::MarkerArray* marker_vec, unsigned int id_start=0);
		visualization_msgs::Marker create_visualization_marker_header(
		std::string frame_id, ros::Time stamp, std::string name,
		int action, int id, int type,
		double px, double py, double pz,
		double qx, double qy, double qz, double qw,
		double sx, double sy, double sz,
		double cr, double cg, double cb, double alpha
		);


		//-------------------------------------------
		//--- DEFINED IN polygon_primitive_auxiliary.cpp
		//-------------------------------------------
		int print_polygon_information(void);

		int compute_supporting_perpendicular_plane_ransac( pcl::PointCloud<pcl::PointXYZ> *pc_in,
				pcl::PointCloud<pcl::Normal> *n_in,
				double DistanceThreshold,
				double NormalDistanceWeight,
				int MaxIterations, 
				pcl::PointIndices::Ptr ind_out,
				pcl::ModelCoefficients::Ptr coeff_out
				);

		int compute_supporting_plane_ransac( pcl::PointCloud<pcl::PointXYZ> *input_cloud,
				pcl::PointCloud<pcl::Normal> *input_normals,
				double DistanceThreshold,
				double NormalDistanceWeight,
				int MaxIterations, 
				pcl::PointIndices::Ptr indices,
				pcl::ModelCoefficients::Ptr coefficients
				);

		int indices_extraction(pcl::PointIndices::Ptr ind,
				pcl::PointCloud<pcl::PointXYZ> *input_cloud,
				pcl::PointCloud<pcl::PointXYZ> *remove_cloud,
				pcl::PointCloud<pcl::PointXYZ>::Ptr copy_cloud,
				pcl::PointCloud<pcl::Normal> *input_normals,
				pcl::PointCloud<pcl::Normal> *remove_normals,
				int compute_normals=1);

		void project_pc_to_plane(pcl::PointCloud<pcl::PointXYZ>::Ptr pcin, pcl::ModelCoefficients::Ptr coeff, pcl::PointCloud<pcl::PointXYZ>::Ptr pcout);

		//-------------------------------------------
		//--- DEFINED IN polygon_primitive_operations.cpp
		//-------------------------------------------
		int polygon_create(pcl::PointCloud<pcl::PointXYZ> *input_cloud,
				pcl::PointCloud<pcl::Normal> *input_normals,
				double DistanceThreshold = 0.5,
				double NormalDistanceWeight = 0.5,
				int MaxIterations = 1000,
				int do_spatial_division=0
				);

		void polygon_split(pcl::PointCloud<pcl::PointXYZ>::Ptr pcin, pcl::PointCloud<pcl::PointXYZ>::Ptr pcdumpster, pcl::ModelCoefficients::Ptr coeff);


		int polygon_expansion(pcl::PointCloud<pcl::PointXYZ> *input_cloud,
				double longitudinal_offset=0.2,
				double perpendicular_offset=0.3,
				pcl::PointCloud<pcl::Normal>* input_normals=NULL); 

		//-------------------------------------------
		//--- DEFINED IN polygon_primitive_2dhulls.cpp
		//-------------------------------------------

		void compute_convex_hull(const pcl::PointCloud<pcl::PointXYZ>::Ptr pcin, const pcl::ModelCoefficients::Ptr coeff, pcl::PointCloud<pcl::PointXYZ>::Ptr pcout);

		void compute_convex_hull_cgal(const pcl::PointCloud<pcl::PointXYZ>::Ptr pcin, const pcl::ModelCoefficients::Ptr coeff, pcl::PointCloud<pcl::PointXYZ>::Ptr pcout, tf::Transform *tr, double &area, double &solidity, bool flg=false);

		void compute_convex_hull_cgal(const pcl::PointCloud<pcl::PointXYZ>::Ptr pcin, const pcl::ModelCoefficients::Ptr coeff, pcl::PointCloud<pcl::PointXYZ>::Ptr pcout, tf::Transform *tr);

		void compute_concave_hull(pcl::PointCloud<pcl::PointXYZ>::Ptr pcin, pcl::PointCloud<pcl::PointXYZ>::Ptr pcout, const pcl::ModelCoefficients::Ptr coeff);

		// ------------------------------------------------------------------------------
		//PRIVATE SECTION OF THE CLASS
		// ------------------------------------------------------------------------------
	private:

		//-------------------------------------------
		//--- DEFINED IN polygon_primitive_2dhulls.cpp
		//-------------------------------------------
		int _polygon_expansion(pcl::PointCloud<pcl::PointXYZ> *input_cloud,
				double longitudinal_offset,
				double perpendicular_offset,
				pcl::PointCloud<pcl::Normal>* input_normals);


		//-------------------------------------------
		//--- DEFINED IN polygon_primitive_auxiliary.cpp
		//-------------------------------------------

		void set_names(const char* s);

		// allocates space all for pointers
		void allocate_space(void);
		double compute_polygon_area(const polygon_3d<pcl::PointXYZ>::Ptr pcin);

		void normalize_vector(double *v);

		/**
		 * @brief Copies the plane model coefficients
		 *
		 * @param in Modelcoefficients in
		 * @param out Modelcoefficients out
		 */
		void copy(pcl::ModelCoefficients::Ptr in, pcl::ModelCoefficients::Ptr out)
		{
			out->values[0] = in->values[0];
			out->values[1] = in->values[1];
			out->values[2] = in->values[2];
			out->values[3] = in->values[3];
		}

		/**
		 * @brief Copies a t_reference_frame to another
		 *
		 * @param in the reference frame to copy
		 * @param out the reference frame where to copy to
		 */
		void copy(const t_reference_frame *in, t_reference_frame *out)
		{
			out->origin.x = in->origin.x; out->origin.y = in->origin.y;	out->origin.z = in->origin.z;	
			out->arrow_x.x = in->arrow_x.x;	out->arrow_x.y = in->arrow_x.y;	out->arrow_x.z = in->arrow_x.z;	
			out->arrow_y.x = in->arrow_y.x;	out->arrow_y.y = in->arrow_y.y;	out->arrow_y.z = in->arrow_y.z;	
			out->arrow_z.x = in->arrow_z.x;	out->arrow_z.y = in->arrow_z.y;	out->arrow_z.z = in->arrow_z.z;	
			out->transform = in->transform;
		}

		double fit_plane_to_two_pc_and_ratio(pcl::PointCloud<pcl::PointXYZ>::Ptr pcin1, pcl::PointCloud<pcl::PointXYZ>::Ptr pcin2, double ratio, pcl::ModelCoefficients::Ptr plane);

		void project_point_to_plane(const pcl::PointXYZ *ptin, const pcl::ModelCoefficients::Ptr coeff, pcl::PointXYZ *ptout);

		bool check_if_point_lies_on_plane(const pcl::ModelCoefficients::Ptr plane, const pcl::PointXYZ *point);

		int check_plane_normal_orientation(pcl::ModelCoefficients::Ptr plane, pcl::PointXYZ *ptin, double vx, double vy, double vz);

		void compute_arrow_points_from_transform(t_reference_frame *frame);

		void create_reference_frame_from_plane_and_two_points(
				pcl::ModelCoefficients::Ptr plane,
				pcl::PointXYZ *pt1,
				pcl::PointXYZ *pt2,
				t_reference_frame *frame
				);

		void set_reference_systems(void);

		void refine_plane_coefficients(pcl::PointCloud<pcl::PointXYZ>::Ptr pcin, pcl::ModelCoefficients::Ptr coeff);


		//-------------------------------------------
		//--- DEFINED IN polygon_primitive_planefitting.cpp
		//-------------------------------------------
		double fit_plane_to_pc(pcl::PointCloud<pcl::PointXYZ>::Ptr pcin, pcl::ModelCoefficients::Ptr plane);

		//-------------------------------------------
		//--- DEFINED IN polygon_primitive_comunications.cpp
		//-------------------------------------------
		void publish_local_tf(void);

		//-------------------------------------------
		//--- DEFINED IN polygon_primitive_offseting.cpp
		//-------------------------------------------
		void offset_polygon(double val, const pcl::PointCloud<pcl::PointXYZ>::Ptr pcin, pcl::PointCloud<pcl::PointXYZ>::Ptr pcout, tf::Transform *tr);
};
#endif
/**
 *@}
 */      

