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
 * @addtogroup polygon_boolean_operations
 *@{
 * @file polygon_boolean_operations.cpp
 * @brief Defines several methods for the boolean operations class
 */

#ifndef _polygon_boolean_operations_CPP_
#define _polygon_boolean_operations_CPP_

#include <bo_polygon2d/polygon_boolean_operations.h>


class_polygon_boolean_operations::class_polygon_boolean_operations(void){};
class_polygon_boolean_operations::~class_polygon_boolean_operations(void){};

int class_polygon_boolean_operations::insert(const pcl::PointCloud<pcl::PointXYZ>::Ptr p_pc)
{
	pcl::PointCloud<pcl::PointXYZ> pc;
	pc=*p_pc;
	return insert(&pc);
}

PBO_Polygon_2 class_polygon_boolean_operations::from_pcl_to_cgalpolygon(pcl::PointCloud<pcl::PointXYZ>* pc)
{
	PS_CGALPolygon_2 polygon;

	//declare de point cloud in local coordinates
	pcl::PointCloud<pcl::PointXYZ> pc_local; 

	//transform the pc to local coordinates
	transform_global_to_local(pc, &pc_local);

	for (size_t i=0; i< pc_local.points.size(); i++) //Build the CGAL polygon
	{
		if (!isnan(pc_local.at(i).x))
		{
			polygon.push_back(PS_CGALPoint_2(
						pc_local.at(i).x,
						pc_local.at(i).y));
		}
	}

	return polygon;
}

int class_polygon_boolean_operations::insert(pcl::PointCloud<pcl::PointXYZ>* pc)
{
	//declare a polygon
	PS_CGALPolygon_2 polygon = from_pcl_to_cgalpolygon(pc);

	if (simplify_polygon(&polygon))
	{
		S.insert(polygon); //Push back on the polygons vector
		return 1;
	}
	else
	{
		ROS_ERROR("Could not insert polygon. Input polygon is not simple and could not simplify");
		print_polygon(polygon);
		return 0;
	}
}

int class_polygon_boolean_operations::join(pcl::PointCloud<pcl::PointXYZ>* pc)
{
	//declare a polygon
	PS_CGALPolygon_2 polygon = from_pcl_to_cgalpolygon(pc);

	if (simplify_polygon(&polygon))
	{
		S.join(polygon); //Push back on the polygons vector
		return 1;
	}
	else
	{
		ROS_ERROR("Could not join polygon. Input polygon is not simple and could not simplify");
		print_polygon(polygon);
		return 0;
	}
}

int class_polygon_boolean_operations::intersection(pcl::PointCloud<pcl::PointXYZ>* pc)
{
	//declare a polygon
	PS_CGALPolygon_2 polygon = from_pcl_to_cgalpolygon(pc);

	if (simplify_polygon(&polygon))
	{
		S.intersection(polygon); //Push back on the polygons vector
		return 1;
	}
	else
	{
		ROS_ERROR("Could not intersect polygon. Input polygon is not simple and could not simplify");
		print_polygon(polygon);
		return 0;
	}
}

int class_polygon_boolean_operations::complement(void)
{
	S.complement();
	return 1;
}


int class_polygon_boolean_operations::get_largest_pcl(pcl::PointCloud<pcl::PointXYZ>* pc_out)
{
	std::list<PBO_Polygon_with_holes_2> res;
	std::list<PBO_Polygon_with_holes_2>::const_iterator it;

	std::cout << "The result contains " << S.number_of_polygons_with_holes() << " components:" << std::endl;

	S.polygons_with_holes (std::back_inserter (res));
			
	for (it = res.begin(); it != res.end(); ++it) 
	{
		pcl::PointCloud<pcl::PointXYZ> pc_out_local;

		PS_CGALPolygon_2::Vertex_const_iterator vit;
		for (vit = (*it).outer_boundary().vertices_begin(); vit != (*it).outer_boundary().vertices_end(); ++vit)
		{
			pcl::PointXYZ pt;
			pt.x = CGAL::to_double((*vit).x());
			pt.y = CGAL::to_double((*vit).y());
			pt.z = 0;
			pc_out_local.push_back(pt);
		}

		//transform the pc to global coordinates
		
		if (it==res.begin())
			transform_local_to_global(&pc_out_local, pc_out);
		else
		{
			if (pc_out_local.points.size() > pc_out->points.size())
				transform_local_to_global(&pc_out_local, pc_out);
		
		}

	}	



	return 1;
}



int class_polygon_boolean_operations::get_all_pcls(std::vector<pcl::PointCloud<pcl::PointXYZ> >* pc_out_vector)
{
	std::list<PBO_Polygon_with_holes_2> res;
	std::list<PBO_Polygon_with_holes_2>::const_iterator it;

	pc_out_vector->erase(pc_out_vector->begin(), pc_out_vector->end());

	std::cout << "The result contains " << S.number_of_polygons_with_holes() << " components:" << std::endl;

	S.polygons_with_holes (std::back_inserter (res));
			
	int k=0;
	for (it = res.begin(); it != res.end(); ++it) 
	{

		pcl::PointCloud<pcl::PointXYZ> pc_out_local;

		ROS_INFO("polygon %d is simple %d",k,(*it).outer_boundary().is_simple());

		
		PS_CGALPolygon_2::Vertex_const_iterator vit;
		for (vit = (*it).outer_boundary().vertices_begin(); vit != (*it).outer_boundary().vertices_end(); ++vit)
		{
			pcl::PointXYZ pt;
			pt.x = CGAL::to_double((*vit).x());
			pt.y = CGAL::to_double((*vit).y());
			pt.z = 0;
			pc_out_local.push_back(pt);
		}

		//transform the pc to global coordinates
		pcl::PointCloud<pcl::PointXYZ> pc_out;
		transform_local_to_global(&pc_out_local, &pc_out);

		pc_out_vector->push_back(pc_out);
	}
	return 1;
}

int class_polygon_boolean_operations::get_first_pcl(pcl::PointCloud<pcl::PointXYZ>* pc_out)
{
	std::list<PBO_Polygon_with_holes_2> res;
	std::list<PBO_Polygon_with_holes_2>::const_iterator it;

	std::cout << "The result contains " << S.number_of_polygons_with_holes() << " components:" << std::endl;

	S.polygons_with_holes (std::back_inserter (res));
			
	for (it = res.begin(); it != res.end(); ++it) 
	{
		pcl::PointCloud<pcl::PointXYZ> pc_out_local;

		PS_CGALPolygon_2::Vertex_const_iterator vit;
		for (vit = (*it).outer_boundary().vertices_begin(); vit != (*it).outer_boundary().vertices_end(); ++vit)
		{
			pcl::PointXYZ pt;
			pt.x = CGAL::to_double((*vit).x());
			pt.y = CGAL::to_double((*vit).y());
			pt.z = 0;
			pc_out_local.push_back(pt);
		}

		//transform the pc to global coordinates
		transform_local_to_global(&pc_out_local, pc_out);

		break;
	}	



	return 1;
}

		int class_polygon_boolean_operations::print(void)
		{
			// Print the result.
			std::list<PBO_Polygon_with_holes_2> res;
			std::list<PBO_Polygon_with_holes_2>::const_iterator it;

			std::cout << "The result contains " << S.number_of_polygons_with_holes() << " components:" << std::endl;

			S.polygons_with_holes (std::back_inserter (res));

			for (it = res.begin(); it != res.end(); ++it) 
			{
				std::cout << "--> ";
				print_polygon_with_holes (*it);
			}	

			return 1;
		}


#endif
/**
 *@}
 */      
