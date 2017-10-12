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
 * @addtogroup projection_mesh 
 * @file
 * @brief Header for projection mesh
 *@{
 */
#ifndef _projection_mesh_H_
#define _projection_mesh_H_

//####################################################################
// Includes:
//####################################################################

//System Includes
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/IO/Color.h>
#include <CGAL/Triangulation_2.h>
#include <CGAL/Triangulation_vertex_base_with_info_2.h>

#include <CGAL/Polygon_2_algorithms.h>
#include <CGAL/Delaunay_triangulation_2.h>

#include <CGAL/basic.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Partition_traits_2.h>
#include <CGAL/Partition_is_valid_traits_2.h>
#include <CGAL/polygon_function_objects.h>
#include <CGAL/partition_2.h>
#include <CGAL/point_generators_2.h>
#include <CGAL/Polygon_set_2.h>
#include <CGAL/Cartesian.h>
#include <CGAL/polygon_function_objects.h>
#include <CGAL/random_polygon_2.h>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl/ModelCoefficients.h>
#include <tf/tf.h>
#include <visualization_msgs/Marker.h>

#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Constrained_triangulation_plus_2.h>

#include <opencv2/imgproc/imgproc.hpp> 
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

//define the kernel
//typedef CGAL::Exact_predicates_exact_constructions_kernel K; 
typedef CGAL::Exact_predicates_inexact_constructions_kernel K; 

//Define an extension to the vertex
typedef struct {double z;float rgb;float weight;} t_vertex_extension;

//define the face from the class Enriched_face_base_2
template <class Gt, class Fb >
class face_extension : public Fb 
{
	public:
		typedef Gt Geom_traits;
		typedef typename Fb::Vertex_handle Vertex_handle;
		typedef typename Fb::Face_handle Face_handle;

		template < typename TDS2 >
			struct Rebind_TDS 
			{
				typedef typename Fb::template Rebind_TDS<TDS2>::Other Fb2;
				typedef face_extension<Gt,Fb2> Other; 
			};

		int status;
		int counter;
		bool visited;

		float weight;
  inline bool is_in_domain() const { return (status%2 == 1); };

  inline void set_in_domain(const bool b) { status = (b ? 1 : 0); };


		face_extension(): Fb(), status(-1) {};

		face_extension(Vertex_handle v0, 
				Vertex_handle v1, 
				Vertex_handle v2): Fb(v0,v1,v2), status(-1) {};

		face_extension(Vertex_handle v0, 
				Vertex_handle v1, 
				Vertex_handle v2,
				Face_handle n0, 
				Face_handle n1, 
				Face_handle n2): Fb(v0,v1,v2,n0,n1,n2), status(-1) {};

}; // end class Enriched_face_base_2


//Define the face as the extension class
//typedef face_extension<K, CGAL::Constrained_triangulation_face_base_2<K> > Fb; 
//typedef CGAL::Triangulation_vertex_base_with_info_2<t_vertex_extension, K> Vb;


//Define the triangulation data_structure
typedef CGAL::Triangulation_data_structure_2<CGAL::Triangulation_vertex_base_with_info_2<t_vertex_extension, K>, 
		face_extension<K, CGAL::Constrained_triangulation_face_base_2<K> > > TDS_projection_mesh;

//Define the Constrained Delaunay Triangulation plus we are using as projection triangulation PT
typedef CGAL::Constrained_triangulation_plus_2<CGAL::Constrained_Delaunay_triangulation_2<K, TDS_projection_mesh, CGAL::No_intersection_tag> > PT;
//typedef CGAL::Constrained_triangulation_plus_2<CGAL::Constrained_Delaunay_triangulation_2<K, TDS_projection_mesh, CGAL::Exact_intersections_tag> > PT;


//Now the actual class
class class_projection_mesh
{
	public:
		class_projection_mesh(void)
		{
		};
		~class_projection_mesh(void){};

		int draw_mesh_triangles(cv::Mat* image, cv::Scalar color, int thickness);
		int print_all_vertices(void);
		int clear_constraints(void);
		int clear_vertices(void);
		int add_vertex_to_mesh(double x, double y, double z, float rgb, float weight);
		int set_constraint_polygon(std::vector<pcl::PointXY>* p);
		int export_triangles(pcl::PointCloud<pcl::PointXYZRGB>* vertex_list, std::vector<float>* face_weights);
		int export_triangles_in_order(pcl::PointCloud<pcl::PointXYZRGB>* vertex_list, std::vector<float>* face_weights);
		int compute_face_weights(void);
		int initialize_visited(void)
		{
			for(PT::All_faces_iterator it = mesh.all_faces_begin(); it != mesh.all_faces_end(); ++it)
			{  
				it->visited=false;
			}
			return 1;
		}

		//Variables
		PT mesh;

		void discoverComponents(const PT& ct);
		void discoverComponent(const PT& ct, PT::Face_handle start, int index, std::list<PT::Edge>& border );
		int initialize_all_faces_status(void);
};



#endif
/**
 *@}
 */      
