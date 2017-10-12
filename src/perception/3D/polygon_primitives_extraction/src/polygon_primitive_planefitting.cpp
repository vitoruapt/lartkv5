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
 * @addtogroup polygon_primitive 
 * @{
 * @file 
 * @brief Methods for plane fitting using ransac are implemented here.
 * @author Miguel Oliveira
 * @version v0
 * @date 2011-12-19
 */
#ifndef _polygon_primitive_planefitting_CPP_
#define _polygon_primitive_planefitting_CPP_


#include "polygon_primitive.h"

#include <CGAL/Cartesian.h>
#include <CGAL/linear_least_squares_fitting_3.h>
#include <list>

		/**
		 * @brief Fits a plane to a point cloud
		 *
		 * @param pcin The point cloud in
		 * @param plane The plane coefficients
		 *
		 * @return The quality of fitting, from 0-1
		 */
double c_polygon_primitive::fit_plane_to_pc(pcl::PointCloud<pcl::PointXYZ>::Ptr pcin, pcl::ModelCoefficients::Ptr plane)
{

	//STEP1. Create a list of pts xyz
	std::list<CGAL::Cartesian<double>::Point_3> points;

	for (int i=0; i<(int)pcin->points.size(); i++)
	{
		points.push_back(CGAL::Cartesian<double>::Point_3(pcin->points[i].x, pcin->points[i].y, pcin->points[i].z));
	}

	//STEP2. Fit the plane to the list of points 
	CGAL::Cartesian<double>::Plane_3 pl;
	double fit_quality = CGAL::linear_least_squares_fitting_3(points.begin(),points.end(),pl,CGAL::Dimension_tag<0>());

	//STEP3. Copy to the model coefficients
	plane->values[0] = pl.a();
	plane->values[1] = pl.b();
	plane->values[2] = pl.c();
	plane->values[3] = pl.d();

return fit_quality;}

/**
		 * @brief fits a plane to two weighted point clouds, given a ratio
		 *
		 * @param pcin1 first point cloud
		 * @param pcin2 Second point cloud
		 * @param ratio the ratio, similar to slerp, if ratio=1 then the weight is all on the pcin1, if ratio=0, only pcin2 counts
		 * @param plane The output plane coefficients
		 *
		 * @return the fitting quality from 0 to 1
		 */
double c_polygon_primitive::fit_plane_to_two_pc_and_ratio(pcl::PointCloud<pcl::PointXYZ>::Ptr pcin1,  pcl::PointCloud<pcl::PointXYZ>::Ptr pcin2, double ratio, pcl::ModelCoefficients::Ptr plane)
{

	//STEP1. Create a list of pts xyz
	std::list<CGAL::Cartesian<double>::Point_3> points;

	//find out many insertions of points according to ratio
	double weight_pcin1 = (1-ratio)/(double)pcin1->points.size();
	double weight_pcin2 = ratio/(double)pcin2->points.size();

	//int count_pcin1 = (int)(weight_pcin1*((int)pcin1->points.size() + (int)pcin2->points.size()));	
	//int count_pcin2 = (int)(weight_pcin2*((int)pcin1->points.size() + (int)pcin2->points.size()));	
	
	//ROS_INFO("weight_pcin1 %f count_pcin1 %d", weight_pcin1, count_pcin1);
	//ROS_INFO("weight_pcin2 %f count_pcin2 %d", weight_pcin2, count_pcin2);

	for (int i=0; i<(int)pcin1->points.size(); i++)
	{
		for (double j=0; j<weight_pcin1; j+=0.00001)
			points.push_back(CGAL::Cartesian<double>::Point_3(pcin1->points[i].x, pcin1->points[i].y, pcin1->points[i].z));
	}

	for (int i=0; i<(int)pcin2->points.size(); i++)
	{
		for (double j=0; j<weight_pcin2; j+=0.00001)
			points.push_back(CGAL::Cartesian<double>::Point_3(pcin2->points[i].x, pcin2->points[i].y, pcin2->points[i].z));
	}

	//STEP2. Fit the plane to the list of points 
	CGAL::Cartesian<double>::Plane_3 pl;
	double fit_quality = CGAL::linear_least_squares_fitting_3(points.begin(),points.end(),pl,CGAL::Dimension_tag<0>());

	//STEP3. Copy to the model coefficients
	plane->values[0] = pl.a();
	plane->values[1] = pl.b();
	plane->values[2] = pl.c();
	plane->values[3] = pl.d();

return fit_quality;}




#endif
 /**
 *@}
 */
/*Previous 3 lines appended automatically on Sun Feb  5 19:40:16 WET 2012 */
