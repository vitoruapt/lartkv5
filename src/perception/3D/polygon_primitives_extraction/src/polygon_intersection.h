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
 * @brief  this is the header for the main code that performs geometric
 * polygonal primitives extraction
 * @author Miguel Armando Riem de Oliveira
 * @version 0.0
 * @date 2011-09-15
 */
#ifndef _POLYGON_INTERSECTION_H_
#define _POLYGON_INTERSECTION_H_

#define PFLN {printf("DEBUG PRINT FILE %s LINE %d\n",__FILE__,__LINE__);}

//####################################################################
//// Includes:
////####################################################################
#include <ros/ros.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <tf/tf.h>

//CGAL
#include <CGAL/basic.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Partition_traits_2.h>
#include <CGAL/Partition_is_valid_traits_2.h>
#include <CGAL/polygon_function_objects.h>
#include <CGAL/partition_2.h>
#include <CGAL/point_generators_2.h>
#include <CGAL/Polygon_set_2.h>

#include <CGAL/random_polygon_2.h>
//#include <cassert>
//#include <list>

#include <CGAL/Cartesian.h>
#include <CGAL/Boolean_set_operations_2.h>
//#include <list>

#include <bo_polygon2d/polygon_simplification.h>
#include <bo_polygon2d/transform_wrapper.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel CGALKernel;
typedef CGALKernel::Point_2                                   CGALPoint_2;
typedef CGAL::Polygon_2<CGALKernel>                           CGALPolygon_2;
typedef CGAL::Polygon_with_holes_2<CGALKernel>                CGALPolygon_with_holes_2;
typedef std::list<CGALPolygon_with_holes_2>                   CGALPwh_list_2;


class class_polygon_intersection: public class_polygon_simplification, public class_transform_wrapper
{
	public:
		class_polygon_intersection(void){};
		~class_polygon_intersection(void){};

		int clear_all_polygons(void);
		int add_polygon_to_list(pcl::PointCloud<pcl::PointXYZ>::Ptr p_pc);
		int add_polygon_to_list(pcl::PointCloud<pcl::PointXYZ>* pc);
		int compute_polygon_intersection(pcl::PointCloud<pcl::PointXYZ>* pc_out);
		int compute_polygon_union(std::vector<pcl::PointCloud<pcl::PointXYZ> > *pc_list);

	protected:
		std::vector<CGALPolygon_2> polygons;


};


#endif
/**
 *@}
 */      

