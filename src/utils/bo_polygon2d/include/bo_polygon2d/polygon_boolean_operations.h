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
 * @file 
 * @brief Defines the class bo_boolean operations
 * *@{
 */

#ifndef _polygon_boolean_operations_H_
#define _polygon_boolean_operations_H_

/**
 * @file polygon_boolean_operations.h
 * @brief 
 * @author Miguel Armando Riem de Oliveira
 * @version 0.0
 * @date 2011-09-15
 */

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

#include <CGAL/Cartesian.h>
#include <CGAL/Boolean_set_operations_2.h>

#include "polygon_simplification.h"
#include "transform_wrapper.h"

typedef CGAL::Exact_predicates_exact_constructions_kernel   PBO_Kernel;
typedef PBO_Kernel::Point_2                                   PBO_Point_2;
typedef CGAL::Polygon_2<PBO_Kernel>                           PBO_Polygon_2;
typedef CGAL::Polygon_with_holes_2<PBO_Kernel>                PBO_Polygon_with_holes_2;
typedef std::list<PBO_Polygon_with_holes_2>                   PBO_Pwh_list_2;
typedef CGAL::Polygon_set_2<PBO_Kernel>                       	  PBO_Polygon_set_2;

class class_polygon_boolean_operations: public class_polygon_simplification, public class_transform_wrapper
{
	public:
		class_polygon_boolean_operations(void);
		~class_polygon_boolean_operations(void);

		int insert(pcl::PointCloud<pcl::PointXYZ>::Ptr p_pc);
		int insert(pcl::PointCloud<pcl::PointXYZ>* pc);


		int intersection(pcl::PointCloud<pcl::PointXYZ>* pc);
		int join(pcl::PointCloud<pcl::PointXYZ>* pc);
		int complement(void);


		int get_largest_pcl(pcl::PointCloud<pcl::PointXYZ>* pc_out);
		int get_first_pcl(pcl::PointCloud<pcl::PointXYZ>* pc_out);
		int get_all_pcls(std::vector<pcl::PointCloud<pcl::PointXYZ> >* pc_out_vector);

		int print(void);

	protected:
		PBO_Polygon_set_2 S;
		PBO_Polygon_2 from_pcl_to_cgalpolygon(pcl::PointCloud<pcl::PointXYZ>* pc);


		template<class Kernel, class Container>
			void print_polygon (const CGAL::Polygon_2<Kernel, Container>& P)
			{
				typename CGAL::Polygon_2<Kernel, Container>::Vertex_const_iterator  vit;

				std::cout << "[ " << P.size() << " vertices:" << std::endl;
				int k=1;
				for (vit = P.vertices_begin(); vit != P.vertices_end(); ++vit)
				{
					std::cout << "x(" << k << ")=" << (*vit).x() << "; y(" << k << ")=" << (*vit).y()<< "; ";
					k++;
				}
				std::cout << std::endl;
			}

		template<class Kernel, class Container>
			void print_polygon_with_holes(const CGAL::Polygon_with_holes_2<Kernel, Container> & pwh)
			{
				if (! pwh.is_unbounded()) {
					std::cout << "{ is_simple=" << pwh.outer_boundary().is_simple() << "Outer boundary = "; 
					print_polygon (pwh.outer_boundary());
				}
				else
					std::cout << "{ Unbounded polygon." << std::endl;

				typename CGAL::Polygon_with_holes_2<Kernel,Container>::Hole_const_iterator hit;
				unsigned int k = 1;

				std::cout << "  " << pwh.number_of_holes() << " holes:" << std::endl;
				for (hit = pwh.holes_begin(); hit != pwh.holes_end(); ++hit, ++k) {
					std::cout << "is_simple=" << (*hit).is_simple() <<"    Hole #" << k << " = ";
					print_polygon (*hit);
				}
				std::cout << " }" << std::endl;
			}
};


#endif
/**
 *@}
 */      

