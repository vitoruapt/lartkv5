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
#ifndef _NAVIGABILITY_MAP_H_
#define _NAVIGABILITY_MAP_H_

/**
  \brief Header file with class declaration to creat a two-dimensional grid. 
  The two-dimensional grid has cells that contain several properties of a pointcloud
  \file navigability_map.h
  \author Diogo Matos
  \date June 2013
*/

#include <stdio.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <boost/shared_ptr.hpp>
#include <iostream> 
#include <Eigen/Core>

#include <algorithm>
#include <stdlib.h>
#include <math.h>

//PCL
#include <pcl_ros/transforms.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>
#include <pcl/surface/gp3.h>
#include <pcl/filters/passthrough.h>
//
#include <vector>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <colormap/colormap.h>

//opencv
#include <cv.h>   		// open cv general include file
#include <highgui.h>	// open cv GUI include file
#include <iostream>		// standard C++ I/O

using namespace Eigen;
using namespace std;

/**
 * \class grid_node
 * Class with classifiers for the cells/node in a two-dimensional grid
 * \date June 2013
 * \author Diogo Matos
 * 
 */

class grid_node
{
	public:

		/** \brief Matrix with points (X,Y,Z) in a node */
		MatrixXd Matrix_Points;
		
		/** \brief Matrix with angles for each normal of a point in a node */
		MatrixXd Angles;
		
		/** \brief If a node has at least one point with normal */
		bool has_normal;
		
		/** \brief Mean value of the angle X of the normal in a node */
		double med_angle_X;
		
		/** \brief Mean value of the angle Y of the normal in a node */
		double med_angle_Y;
		
		/** \brief Mean value of the angle Z of the normal in a node */
		double med_angle_Z;
		
		/** \brief Number of points projected in a node (cell) */
		int num_points;
		
		/** \brief Z mean value in a node */
		double Zmed;
		
		/** \brief Standard Deviation in Z */
		double Standard_Deviation_Z;
		
		/** \brief Z confidence in a node */
		double Z_confidence;
				
		/** \brief Angle X confidence in a node */
		double angle_confidence_x;
		
		/** \brief Angle Y confidence in a node */
		double angle_confidence_y;
		
		/** \brief Angle Z confidence in a node */
		double angle_confidence_z;
		
		/** \brief true if a cell cointains Z data interpolated from the neighbours, false it contains data from the original PointCloud */  
		bool interpolate_z_data;
		
		/** \brief true if that cell cointains normal data interpolated from the neighbours, false it contains data from the original PointCloud */  
		bool interpolate_normal_data;
		
		/** \brief Vector that contais the Z difference between neighbours to get the accessibility */
		std::vector<double> z_difference_neighbours;
		
		/** \brief Vector that contais the Z confidence between neighbours to get the accessibility */
		std::vector<double> z_confidence_neighbours;
		
		/** \brief Vector that contais the Angle X difference between neighbours to get the accessibility */
		std::vector<double> angleX_difference_neighbours;
		
		/** \brief Vector that contais the Angle Y difference between neighbours to get the accessibility */
		std::vector<double> angleY_difference_neighbours;
		
		/** \brief Vector that contais the Angle Z difference between neighbours to get the accessibility */
		std::vector<double> angleZ_difference_neighbours;
		
		/** \brief Vector that contais the Angle X confidence between neighbours to get the accessibility */
		std::vector<double> angleX_confidence_neighbours;
		
		/** \brief Vector that contais the Angle Y confidence between neighbours to get the accessibility */
		std::vector<double> angleY_confidence_neighbours;
		
		/** \brief Vector that contais the Angle Z confidence between neighbours to get the accessibility */
		std::vector<double> angleZ_confidence_neighbours;
		
		/** \brief Vector that contais a boolean value if a neighbour contains all data to get the accessibility*/
		std::vector<bool> has_fulldata_direction; //N,NE,E,SE,S,SW,W,NW /only for cell fully data
		
		/** \brief Z accessibility */
		double z_accessibility;
		
		/** \brief Angle X accessibility */
		double angleX_accessibility;
		
		/** \brief Angle Y accessibility */
		double angleY_accessibility;
		
		/** \brief Angle Z accessibility */
		double angleZ_accessibility;
		
		/** \brief Total accessibility */
		double total_accessibility;
		

		grid_node()
		{			
			
			z_accessibility=0;
			
			angleX_accessibility=0;
			angleY_accessibility=0;
			angleZ_accessibility=0;
			
			total_accessibility=0;

			interpolate_z_data=false;
			interpolate_normal_data=false;
			has_normal=false;
			Z_confidence=0;
	
			angle_confidence_x=0;
			angle_confidence_y=0;
			angle_confidence_z=0;
			
			num_points=0;		
			Zmed=-1000;
		}
		
		~grid_node()
		{}
		
	private:
	
};

//stores a pointer to a dynamically allocated object
typedef boost::shared_ptr<grid_node> grid_nodePtr;

//stores a pointer to a dynamically allocated object
typedef Matrix<grid_nodePtr, Dynamic, Dynamic> Grid;



/**
 * \class Navigability_Map
 * Class for a accessibility map
 * \date April 2013
 * \author Diogo Matos
 * 
 */

class Navigability_Map
{
	public:

		/** \brief PointCloud that contains the normals */
		pcl::PointCloud<pcl::Normal>::Ptr normals;
		
		/** \brief Min distance to be consider in the X direction of the PointCloud */
		double Xmin_filter;
		
		/** \brief Max distance to be consider in the X direction of the PointCloud */
		double Xmax_filter;
		
		/** \brief Min distance to be consider in the Y direction of the PointCloud */
		double Ymin_filter;
		
		/** \brief Max distance to be consider in the Y direction of the PointCloud */
		double Ymax_filter;
		
		/** \brief Min distance to be consider in the Z direction of the PointCloud */
		double Zmax_filter;
		
		/** \brief Max distance to be consider in the Z direction of the PointCloud */
		double Zmin_filter;
		
		/** \brief The radius of the neighbours search to estimate the normals in the PointCloud*/ 
		double Radius_neighbors;
		
		/** \brief The number of neighbours search to estimate the normals in the PointCloud */ 
		int K_neighbors;
		
		/** \brief 1 - to use the radius search ; 0 - to use the number of neighbours search */ 
		int Use_Radius_Search;
		
		/** \brief Color to represent the maps */ 
		int colorrange;

		/** \brief The two-dimensional grid with the cells */ 
		Grid grid;
		
		/** \brief The X dimension of the cell */ 
		double Sx;
		
		/** \brief The Y dimension of the cell */
		double Sy;
		
		/** \brief Total rows of the grid */
		int total_row;
		
		/** \brief Total cols of the grid */
		int total_col;
		
		/** \brief Default confidence for a cell with one point */
		double default_confidence;
		
		/** \brief Limit of the max difference between a cell and a neighbour cell */
		double fator_confidence_neighbour_limit;
		
		/** \brief Center col of the grid */
		int CARaxis_col;
		
		/** \brief Threshold for the Z difference between cells*/
		double Zmax_heigh_difference;
		
		/** \brief Normalization constant for the Z confidence value */
		double Standard_Deviation_max;
		
		/** \brief Threshold for the Angle X difference between cells*/
		double angleX_max_difference;
		
		/** \brief Threshold for the Angle Y difference between cells*/
		double angleY_max_difference;
		
		/** \brief Threshold for the Angle Z difference between cells*/
		double angleZ_max_difference;

		/** \brief Normalization constant for the Angle X confidence value */
		double Standard_Deviation_anglex_max_confidence;
		
		/** \brief Normalization constant for the Angle Y confidence value */
		double Standard_Deviation_angley_max_confidence;
		
		/** \brief Normalization constant for the Angle Z confidence value */
		double Standard_Deviation_anglez_max_confidence;

		/** \brief Which accessibility map should be draw */
		int debug_accessibility;

		/// _______________________
		///						    |
		/// Ground truth variables |
		///________________________|
		
		std::vector<geometry_msgs::Point> polygon_points;
		std::vector<geometry_msgs::Point> obstacle_points;
		std::vector<cv::Point2f> xy_polygon;
		std::vector<cv::Point2f> obstacle_polygon;
		
		MatrixXd Matrix_Cell_inpolygon_rowcol;
		MatrixXd Matrix_obstacle_inpolygon_rowcol;
		
		Navigability_Map()
		{
			debug_accessibility=1;
			default_confidence=0.5;
			fator_confidence_neighbour_limit=1.5;
			
			Zmax_heigh_difference=0.1;
			Standard_Deviation_max=0.2;
			
			angleX_max_difference=0.1;
			angleY_max_difference=0.1;
			angleZ_max_difference=0.1;
			Standard_Deviation_anglex_max_confidence=0.2;
			Standard_Deviation_angley_max_confidence=0.2;
			Standard_Deviation_anglez_max_confidence=0.2;
			
			Radius_neighbors=0.4;
			K_neighbors=10;
			Use_Radius_Search=1;
			
			Xmin_filter=0;
			Xmax_filter=22.5;
			Ymin_filter=-22.5;
			Ymax_filter=22.5;
			Zmax_filter=2;
			Zmin_filter=-15;
			
			Sx=0.2;
			Sy=0.2;
			colorrange=64;
		}
		
		~Navigability_Map()
		{}
		
		/**
		* \brief Function that filters the PointCloud.
		* Filters the point cloud receiveid, the filtered PointCloud is set in pc_filter
		* \param 	cloud_in PointCloud to be filtered
		* \param 	pc_filter Output variable for the filtered PointCloud
		*/
		
		void Filter_PointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in,pcl::PointCloud<pcl::PointXYZ>::Ptr &pc_filter);
		
		/**
		* \brief Computes the normal estimation for a PointCloud.
		* \param cloud Input PointCloud
		*/
		void Normal_Estimation(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
		
		/**
		* \brief Function that inicialize the Grid with default data.
		*
		*/	
		void inicialize_grid(void);
		
		/**
		* \brief Function that sets the parameters to generate the grid.
		* Grid Parameter:
		* Total of row, cols and the center col
		* \param 	max_Xval X max range of the pointcloud
		* \param 	max_Yval Y max range of the pointcloud
		*/
		void setGrid_parameter(double max_Xval,double max_Yval);
	
		/**
		* \brief Function that sets the data to the grid based on the PointCloud information. 
		* Saves the x,y,z points and normals of the PointCloud to a cell in the grid.
		* \param cloud Input PointCloud
		*/
		
		void setGrid_data(pcl::PointCloud<pcl::PointXYZ>& cloud);
		
		
		/**
		* \brief Function that calculates mean values and confidence of the x,y,z points and normals of the PointCloud.
		*
		*/	
		void calcGrid_data(void);

		/**
		* \brief Function that interpolates data to empty cells
		*
		*/	
		void fill_data_cells(void);
		
		/**
		* \brief Function that gets the neighbours data from a cell
		*	
		* \param row_pos cell row
		* \param col_pos cell col
		* \param empty_neighbours 0 - if a cell as neighbours; 1- if a cells does not have neighbours
		* \param number_neighbours Number of neighbours with full data 
		* \return std::vector<grid_nodePtr> - Data of the neighbours
		*/ 
		
		std::vector<grid_nodePtr> cell_neighbours(int row_pos, int col_pos, int &empty_neighbours, int &number_neighbours);
		
		/**
		* \brief Function that sets the accessibility of the cells
		* 
		*/	
		void set_Cells_accessibility(void);
		
		/// _______________________
		///						    |
		/// Ground truth functions |
		///________________________|
		void polygon_groundtruth(void);
		void getCell_inpolygon(void);
		void dataCell_inpolygon(void);
		
		
		
	private:
	
};


#endif
