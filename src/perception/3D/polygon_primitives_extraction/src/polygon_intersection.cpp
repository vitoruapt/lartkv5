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
 * @addtogroup polygon_intersection
 * @{
 * @file
 * @brief Provides functions for testing the intersection of polygons
 */

#ifndef _POLYGON_INTERSECTION_CPP_
#define _POLYGON_INTERSECTION_CPP_

#include "polygon_intersection.h"

int class_polygon_intersection::clear_all_polygons(void){polygons.erase(polygons.begin(), polygons.end());return 1;};


int class_polygon_intersection::add_polygon_to_list(const pcl::PointCloud<pcl::PointXYZ>::Ptr p_pc)
{
	pcl::PointCloud<pcl::PointXYZ> pc;
	pc=*p_pc;
	return add_polygon_to_list(&pc);
}

int class_polygon_intersection::add_polygon_to_list(pcl::PointCloud<pcl::PointXYZ>* pc)
{
	//declare a polygon
	CGALPolygon_2 polygon;

	//declare de point cloud in local coordinates
	pcl::PointCloud<pcl::PointXYZ> pc_local; 

	//transform the pc to local coordinates
	transform_global_to_local(pc, &pc_local);

	for (size_t i=0; i< pc_local.points.size(); i++) //Build the CGAL polygon
	{
		if (!isnan(pc_local.at(i).x))
		{
			polygon.push_back(CGALPoint_2(
						pc_local.at(i).x,
						pc_local.at(i).y));
			//printf("pc_local.at(%d).z = %3.2f\n", (int)i, pc_local.at(i).z);
		}
	}

	if (simplify_polygon(&polygon))
	{
		polygons.push_back(polygon); //Push back on the polygons vector
		return 1;
	}
	else
	{
		ROS_ERROR("Could not add polygon. It is not simple and could not simplify");
		return 0;
	}
}

int class_polygon_intersection::compute_polygon_intersection(pcl::PointCloud<pcl::PointXYZ>* pc_out)
{
	CGALPwh_list_2 polygon_out;
	//CGALPwh pol;
	CGALPwh_list_2::const_iterator  it;

	//CGAL::intersection (polygons[0], polygons[1], std::back_inserter(polygon_out));
	CGAL::intersection (polygons[0], polygons[1], std::back_inserter(polygon_out));

	for (size_t k=2; k<polygons.size(); k++)
	{

		////CGALPwh_list_2 polygon_tmp;
		////CGAL::intersection (polygons[k], polygons[k], std::back_inserter(polygon_tmp));
		it = polygon_out.begin();

		CGAL::intersection (*it, polygons[k], std::back_inserter(polygon_out));
	}

	int count=0;
	for (it = polygon_out.begin(); it != polygon_out.end(); ++it)
	{
		count++;
	}

	if(count!=1)
	{
		ROS_ERROR("Problem in polygon interception. Output was %d polygons",count);
		return 0;
	}
	else
	{
		it = polygon_out.begin();

		if (!(*it).is_unbounded())
		{
			//ROS_INFO("Polygon is bounded and has %d vertices", (*it).outer_boundary().size());
			pcl::PointCloud<pcl::PointXYZ> pc_out_local;

			CGALPolygon_2::Vertex_const_iterator vit;
			for (vit = (*it).outer_boundary().vertices_begin(); vit != (*it).outer_boundary().vertices_end(); ++vit)
			{
				pcl::PointXYZ pt;
				pt.x = (*vit).x();
				pt.y = (*vit).y();
				pt.z = 0;
				pc_out_local.push_back(pt);
			}

			//transform the pc to global coordinates
			transform_local_to_global(&pc_out_local, pc_out);
		}

	}
	return 1;}


int class_polygon_intersection::compute_polygon_union(std::vector<pcl::PointCloud<pcl::PointXYZ> > *pc_list)
{
	std::vector<CGALPolygon_with_holes_2> polygon_out_list;

	printf("polygon union: i will do union of %d polygons\n", (int)polygons.size());
	for (size_t i=0; i< polygons.size(); i++)
	{
		printf("polygon %d has %d vertices\n", (int)i, (int)polygons[i].size());
	}

	if (polygons.size()>0)
	{
		//insert the first polygon to the list
		CGALPolygon_with_holes_2 polygon_out;
		CGAL::join(polygons[0], polygons[0], polygon_out);
		polygon_out_list.push_back(polygon_out);

		for (size_t i=1; i<polygons.size(); i++) //go though all polygons
		{
			printf("Computing for plg %d\n", (int)i);
			bool was_merged=false;
			for (size_t j=0; j<polygon_out_list.size(); j++) //go through the list
			{

				if (CGAL::join(polygons[i], polygon_out_list[j], polygon_out_list[j])) //an union was possible
				{
					printf("plg %d was merged to list %d\n",(int)i,(int)j);
					was_merged=true;
					break;
				}
				else
				{
					printf("plg %d cannot be merged with plg list %d\n",(int)i,(int)j);
				}

			}

			if (was_merged==false)
			{
				CGALPolygon_with_holes_2 polygon_out;
				CGAL::join(polygons[i], polygons[i], polygon_out);
				polygon_out_list.push_back(polygon_out);
				printf("Adding plg %d to list. Now has %d size\n",(int)i, (int)polygon_out_list.size());
			}

		}

		///////////////////////////////////
		//now try to compact the list
		///////////////////////////////////
		std::vector<CGALPolygon_with_holes_2> polygon_out_list1;

		printf("Will try to compact list\n");

		//insert the first polygon to the list
		polygon_out_list1.push_back(polygon_out_list[0]);

		for (size_t i=1; i<polygon_out_list.size(); i++) //go though all polygon in polygonlist
 		{
			printf("Computing for plg %d\n", (int)i);
			bool was_merged=false;
			for (size_t j=0; j<polygon_out_list1.size(); j++) //go through the list
			{

				if (CGAL::join(polygon_out_list[i], polygon_out_list1[j], polygon_out_list1[j])) //an union was possible
				{
					printf("plg %d was merged to list %d\n",(int)i,(int)j);
					was_merged=true;
					break;
				}
				else
				{
					printf("plg %d cannot be merged with plg list %d\n",(int)i,(int)j);
				}

			}

			if (was_merged==false)
			{
				polygon_out_list1.push_back(polygon_out_list[i]);
				printf("Adding plg %d to list. Now has %d size\n",(int)i, (int)polygon_out_list1.size());
			}

		}		


		//printf("final list has %d polygons\n", (int)polygon_out_list.size());
		for (size_t i=0; i< polygon_out_list1.size(); i++)
		{
			//printf("polygon_out_list %d has %d vertices and %d holes\n", (int)i, (int)polygon_out_list[i].outer_boundary().size(), (int)polygon_out_list[i].number_of_holes());

			pcl::PointCloud<pcl::PointXYZ> pc_out_local;
			pcl::PointCloud<pcl::PointXYZ> pc_out;

			CGALPolygon_2::Vertex_const_iterator vit;

			for (vit = polygon_out_list1[i].outer_boundary().vertices_begin(); vit != polygon_out_list1[i].outer_boundary().vertices_end(); ++vit)
			{
				pcl::PointXYZ pt;
				pt.x = (*vit).x();
				pt.y = (*vit).y();
				pt.z = 0;
				pc_out_local.push_back(pt);
			}

			////transform the pc to global coordinates
			transform_local_to_global(&pc_out_local, &pc_out);
			pc_list->push_back(pc_out);
		}

	}
	else if (polygons.size()==0)
	{
		ROS_ERROR("No polygons given. No union can be performed");

	}


	return 1;
}

#endif
/**
 *@}
 */      
