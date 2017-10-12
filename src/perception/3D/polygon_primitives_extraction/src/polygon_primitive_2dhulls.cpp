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
 * @brief Methods for convex and concave hull computation
 * @author Miguel Oliveira
 * @version v0
 * @date 2011-12-19
 */
#ifndef _polygon_primitive_2dhulls_CPP_
#define _polygon_primitive_2dhulls_CPP_


#include "polygon_primitive.h"

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/convex_hull_2.h>
#include<CGAL/Polygon_2.h>

/**
		 * @brief Uses pcl library to compute the convex hull of a set of points in pcin, by projecting them first to the plane defined by coeff
		 *
		 * @param pcin the point cloud in
		 * @param coeff the plane coefficients
		 * @param pcout the point cloud out
		 */
void c_polygon_primitive::compute_convex_hull(const pcl::PointCloud<pcl::PointXYZ>::Ptr pcin, const pcl::ModelCoefficients::Ptr coeff, pcl::PointCloud<pcl::PointXYZ>::Ptr pcout)
{
	//To compute the convex hull the following steps are required:
	//1. project the points from the input point cloud pc_in to the plane model (coeff)
	//2. Compute the convex hull on these projected points and store to pcout

	pcl::PointCloud<pcl::PointXYZ>::Ptr pc_projected = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>); //create the projection cloud

	//STEP 1. the projection
	project_pc_to_plane(pcin, coeff, pc_projected);

	//STEP 2. Conputing the convex hull
	pcl::ConvexHull<pcl::PointXYZ> chull; //declare the object
	chull.setInputCloud(pc_projected); //set the input point cloud
	chull.reconstruct(*pcout); //compute the convex hull
	pcout->width = pcin->width; //set the same width
	pcout->height = pcin->height; //set the same height
	pcout->header.frame_id = pcin->header.frame_id; //set the same frame_id

	pc_projected.reset(); //reset the pc_projected
}

/**
		 * @brief Uses cgal to compute the convex hull of the points in pcin. First, points are projected to the plane defined by coeff. Outputs the area and solidity of the hull
		 *
		 * @param pcin the point cloud in
		 * @param coeff the plane coefficients
		 * @param pcout the point cloud out
		 * @param tr a transformation used to define the local reference system
		 * @param area the hull area
		 * @param solidity the hull solidity
		 */
void c_polygon_primitive::compute_convex_hull_cgal(const pcl::PointCloud<pcl::PointXYZ>::Ptr pcin, const pcl::ModelCoefficients::Ptr coeff, pcl::PointCloud<pcl::PointXYZ>::Ptr pcout, tf::Transform *tr)
{
	double area,solidity;
	compute_convex_hull_cgal(pcin, coeff, pcout, tr, area, solidity);
}


void c_polygon_primitive::compute_convex_hull_cgal(const pcl::PointCloud<pcl::PointXYZ>::Ptr pcin, const pcl::ModelCoefficients::Ptr coeff, pcl::PointCloud<pcl::PointXYZ>::Ptr pcout, tf::Transform *tr, double &area, double &solidity,bool flg)
{
	//To compute the convex hull the following steps are required:
	//1. project the points from the input point cloud pc_in to the plane model (coeff)
	//2. Compute the convex hull on these projected points and store to pcout

	pcl::PointCloud<pcl::PointXYZ>::Ptr pc_projected = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>); //create the projection cloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr pc_local = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr pc_ch_local = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

	//STEP 1. the projection
	project_pc_to_plane(pcin, coeff, pc_projected);

	if(flg)
	{
		pointclouds.projected = pc_projected;
	}

	//STEP2. transforming to local frame

	//the given transform goes from local to global. We want to change from global to local. Hence we use the inverse transform
	pcl_ros::transformPointCloud(*pc_projected, *pc_local,  data.frames.current.transform.inverse());


	//printf("Reference coeff has A=%f B=%f C=%f D=%f\n",data.planes.current->values[0],data.planes.current->values[1],data.planes.current->values[2],data.planes.current->values[3]);
	//printf("ALL has %d points\n",(int) pcin->points.size());
	//printf("Projected has %d points\n",(int) pc_projected->points.size());
	//printf("Local has %d points\n",(int) pc_local->points.size());
	//for (size_t i=0; i<data.hulls.convex.polygon->points.size(); i++)
	//{
		//printf("ALL pt[%d] x=%3.5f y=%3.5f z=%3.5f\n",(int)i, pcin->points[i].x, pcin->points[i].y, pcin->points[i].z); 
		//printf("PROJECTED pt[%d] x=%3.5f y=%3.5f z=%3.5f\n",(int)i, pc_projected->points[i].x, pc_projected->points[i].y, pc_projected->points[i].z); 
		//printf("LOCAL pt[%d] x=%3.5f y=%3.5f z=%3.5f\n",(int)i, pc_local->points[i].x, pc_local->points[i].y, pc_local->points[i].z); 

	//}


	//STEP3. Copy the local to a pts vector
	std::vector<CGAL::Exact_predicates_inexact_constructions_kernel::Point_2> pts; //The polygon

	//Setup the polygon with the points from local
	for (int i=0; i< (int) pc_local->size(); i++)
	{
		pts.push_back(CGAL::Exact_predicates_inexact_constructions_kernel::Point_2(pc_local->points[i].x, pc_local->points[i].y));
	}


	std::vector<CGAL::Exact_predicates_inexact_constructions_kernel::Point_2> pts_ch; //The ch polygon

	//STEP4. Conpute the convex hull
	area = compute_polygon_area(pc_ch_local);
	data.hulls.convex.solidity = (double)(pcin->points.size())/area;



	CGAL::convex_hull_2( pts.begin(), pts.end(),std::back_inserter(pts_ch));



	for (int t = 0; t<(int)pts_ch.size(); t++ )
	{
		pc_ch_local->points.push_back(pcl::PointXYZ(pts_ch[t].x(), pts_ch[t].y(),0.0));
	}
	
	//STEP4. Compute the area
    data.hulls.convex.area = compute_polygon_area(pc_ch_local);
	data.hulls.convex.solidity = (double)(pointclouds.all->points.size())/data.hulls.convex.area;



	//STEP 3. Transform pc local extended to pcout
	pcl_ros::transformPointCloud(*pc_ch_local, *pcout,  data.frames.current.transform);

	pc_projected.reset();
	pc_local.reset();
	pc_ch_local.reset();
}

/**
* @brief Computes the area of a polygon
*
* @param pcin
*
* @return 
*/
double c_polygon_primitive::compute_polygon_area(const pcl::PointCloud<pcl::PointXYZ>::Ptr pcin)
{
	//It is assumed that input_cloud is a polygon with no z coordinates. Must be one of the local polygons
	CGAL::Polygon_2<CGAL::Exact_predicates_inexact_constructions_kernel> star; //The polygon
	//CGAL::Exact_predicates_inexact_constructions_kernel::Point_2 

	//Setup the polygon with the points from pc.ch_l
	for (int i=0; i< (int) pcin->size(); i++)
	{
		star.push_back(CGAL::Exact_predicates_inexact_constructions_kernel::Point_2(
					pcin->points[i].x, 
					pcin->points[i].y));
	}

	return fabs(star.area());

}

/**
 * @brief Computes the concave_hull of a given point cloud. 
 *
 * @param pcin the input point cloud
 * @param pcout the output polygon_3d 
 */
void c_polygon_primitive::compute_concave_hull(pcl::PointCloud<pcl::PointXYZ>::Ptr pcin, pcl::PointCloud<pcl::PointXYZ>::Ptr pcout, const pcl::ModelCoefficients::Ptr coeff)
{
	//ROS_INFO("Computing Concave hull");
	pcl::PointCloud<pcl::PointXYZ>::Ptr pctmp (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr voronoi_centers (new pcl::PointCloud<pcl::PointXYZ>);

	pcl::PointCloud<pcl::PointXYZ>::Ptr pc_projected = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>); //create the projection cloud

	//STEP 1. the projection
	project_pc_to_plane(pcin, coeff, pc_projected);


	std::vector< pcl::Vertices > V;
	pcl::ConcaveHull<pcl::PointXYZ> chull;
	chull.setInputCloud(pc_projected);
	chull.setAlpha(6.9);
	//chull.setKeepInformation(false);
	//chull.setVoronoiCenters(voronoi_centers);

	chull.reconstruct(*pctmp, V);
	pctmp->width = pcin->width;
	pctmp->header.frame_id = pcin->header.frame_id;


	//Delete the pcout and obtain a new pc with ordered points
	//ROS_INFO("vertices has %d points", (int)V.size()); 
	//ROS_INFO("vertices2  has %d points", (int)V[0].vertices.size()); 

	//ROS_INFO("Voronoi has %d points ",(int)voronoi_centers->points.size());
	//ROS_INFO("Concave hull has %d points (pcin %d)",(int)pctmp->points.size(), (int)pcin->points.size());

	//Delete all points from pcout
	pcout->points.erase(pcout->points.begin(), pcout->points.end());

	int i=0;
	int v=0;
	for (std::vector<pcl::Vertices>::iterator vit=V.begin(); vit != V.end(); vit++)
	{
		i=0;
		for (std::vector<uint32_t>::iterator it=V[v].vertices.begin(); it != V[v].vertices.end(); it++)
		{
			pcl::PointXYZ p;
			p.x = pctmp->points[V[v].vertices[i]].x;
			p.y = pctmp->points[V[v].vertices[i]].y;
			p.z = pctmp->points[V[v].vertices[i]].z;

			pcout->points.push_back(p);
			i++;
		}  
		v++;
	}
	//ROS_INFO("pcout %d points", (int)pcout->points.size()); 

	pcout->header.frame_id = pctmp->header.frame_id;
	pcout->width = pcout->points.size();
	pcout->height = 1;
	//*pcout = *pctmp;

}
#endif
/**
 *@}
 */      
