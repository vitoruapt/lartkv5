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
 * @addtogroup constrained_delaunay_triangulation 
 * @{
 * @file
 * @brief Header for constrained delaunay triangulation
 */
#ifndef _CONSTRAINED_DELAUNAY_TRIANGULATION_H_
#define _CONSTRAINED_DELAUNAY_TRIANGULATION_H_

//####################################################################
// Includes:
//####################################################################

//System Includes
//
#include <boost/function.hpp>

#include <CGAL/basic.h>
#include <CGAL/assertions.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/IO/Color.h>
#include <CGAL/Triangulation_2.h>
#include <CGAL/Triangulation_vertex_base_with_info_2.h>

#include <CGAL/Triangulation_hierarchy_2.h>
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
#include <CGAL/circulator_bases.h>
#include "polygon_intersection.h"
#include <CGAL/basic.h>


//define the kernel
typedef CGAL::Exact_predicates_exact_constructions_kernel K_exact; 
//typedef CGAL::Simple_cartesian<CGAL::Gmpq> K_exact;

//define the vertexes
typedef struct {double z;float rgb; size_t index;} t_enriched_vertex_base_2;
typedef CGAL::Triangulation_vertex_base_with_info_2<t_enriched_vertex_base_2, K_exact> Vbb;
typedef CGAL::Triangulation_hierarchy_vertex_base_2<Vbb> Vb;

/* A vertex class with an additionnal handle */
//template < class Gt, class Vb = CGAL::Triangulation_vertex_base_2_with_info_2<t_enriched_vertex_base_2, Gt> >
//class My_vertex_base
//: public  Vb
//{
//typedef Vb                              Base;
//public:
//typedef typename Vb::Vertex_handle      Vertex_handle;
//typedef typename Vb::Face_handle        Face_handle;
//typedef typename Vb::Point              Point;

//template < typename TDS2 >
//struct Rebind_TDS {
//typedef typename Vb::template Rebind_TDS<TDS2>::Other    Vb2;
//typedef My_vertex_base<Gt,Vb2>                           Other;
//};

//private:
//Vertex_handle  va_;

//public:
//My_vertex_base() : Base() {}
//My_vertex_base(const Point & p) : Base(p) {}
//My_vertex_base(const Point & p, Face_handle f) : Base(f,p) {}
//My_vertex_base(Face_handle f) : Base(f) {}

//void set_associated_vertex(Vertex_handle va) { va_ = va;}
//Vertex_handle get_associated_vertex() {return va_ ; }
//};


//define the face from the class Enriched_face_base_2
template <class Gt, class Fb >
class Enriched_face_base_2 : public Fb 
{
	public:
		typedef Gt Geom_traits;
		typedef typename Fb::Vertex_handle Vertex_handle;
		typedef typename Fb::Face_handle Face_handle;

		template < typename TDS2 >
			struct Rebind_TDS 
			{
				typedef typename Fb::template Rebind_TDS<TDS2>::Other Fb2;
				typedef Enriched_face_base_2<Gt,Fb2> Other; 
			};

		int status;
		int counter;
		bool visited;

		float weight;
		int provenience;
		inline bool is_in_domain() const { return (status%2 == 1); };

		inline void set_in_domain(const bool b) { status = (b ? 1 : 0); };
		void initialize_weight_and_provenience(void){weight=-1; provenience=-1;visited=false;};

		Enriched_face_base_2(): Fb(), status(-1) {initialize_weight_and_provenience();};

		Enriched_face_base_2(Vertex_handle v0, 
				Vertex_handle v1, 
				Vertex_handle v2): Fb(v0,v1,v2), status(-1) {initialize_weight_and_provenience();};

		Enriched_face_base_2(Vertex_handle v0, 
				Vertex_handle v1, 
				Vertex_handle v2,
				Face_handle n0, 
				Face_handle n1, 
				Face_handle n2): Fb(v0,v1,v2,n0,n1,n2), status(-1) {initialize_weight_and_provenience();};

}; // end class Enriched_face_base_2

//Now define the Facebase
typedef Enriched_face_base_2<K_exact, CGAL::Constrained_triangulation_face_base_2<K_exact> > Fb; 

//Define the triangulation data_structure
typedef CGAL::Triangulation_data_structure_2<Vb,Fb>              TDS;

//Define the Constrained Delaunay Triangulation
//typedef CGAL::Exact_predicates_tag                               Itag;
typedef CGAL::No_intersection_tag                               Itag;


typedef CGAL::Constrained_Delaunay_triangulation_2<K_exact, TDS, Itag> CDT1;

typedef CGAL::Triangulation_hierarchy_2<CDT1> CDT;
//typedef CGAL::Constrained_Delaunay_triangulation_2<K, TDS, Itag> CDT1;
//typedef CGAL::Constrained_triangulation_plus_2<CDT1>       CDT;

//other stuff
typedef CDT::Vertex_handle Vertex_handle1;
typedef CDT::Vertex_iterator Vertex_iterator;
typedef CDT::Point Point_2;
typedef CDT::Finite_vertices_iterator Finite_vertices_iterator;
typedef CDT::Finite_faces_iterator Finite_faces_iterator;
typedef CDT::All_faces_iterator All_faces_iterator;

typedef CDT::Vertex_handle Vertex_handle2;
typedef CDT::Face_handle Face_handle;

typedef enum
{
	CAN_ADD=77,
	CANNOT_ADD,
	EXISTS,
	ERROR
}e_vertex_analysis;

typedef struct
{
	Point_2 v0,v1,v2;
}t_face;



//Now the actual class
class class_constrained_delaunay_triangulation
{
	public:
		class_constrained_delaunay_triangulation(void)
		{
			index_count=1;
			debug=1;
		};
		~class_constrained_delaunay_triangulation(void){};

		bool is_degenerate(double x0, double y0, double z0,
				double x1, double y1, double z1,
				double x2, double y2, double z2);

		CDT::Vertex_handle get_vertex_at(Point_2 p);
		int test_if_triangulation_is_valid(void);
		int remove_constraint(CDT::Face_handle fh, int li);
		bool equal(Point_2 p1, Point_2 p2);
		bool equal_e(CGAL::Point_2<K_exact> p1, CGAL::Point_2<K_exact> p2);
		bool overlaps(Point_2* v0, Point_2* v1, Point_2* v2, Point_2* p);
		int export_all_points_to_pc(void);
		int cleanup_isolated_vertices(void);
		int add_face_to_mesh(double x0, double y0, double z0, float rgb0, 
				double x1, double y1, double z1, float rgb1, 
				double x2, double y2, double z2, float rgb2,
				float face_weight, int provenience);

		int print_all_vertices(void);
		int clear_constraints(void);
		int clear_vertices(void);
		int set_transform(tf::Transform* st);
		int add_point_cloud(const pcl::PointCloud<pcl::PointXYZRGB>* pc, std::vector<float>* vweight,int provenience);
		int add_point_manager(double x, double y, double z, float rgb, float weight, int provenience);



		int get_seed_list_of_faces_for_tti(CDT::Face_handle fti, std::vector<CDT::Face_handle>* queue);
		int add_face_manager(double x0, double y0, double z0, float rgb0, 
				double x1, double y1, double z1, float rgb1, 
				double x2, double y2, double z2, float rgb2,
				float face_weight, int provenience);


		int add_vertex_to_mesh(double x, double y, double z, float rgb, float weight, int provenience);
		int export_points_to_pc(void);
		int compute_union(void);
		int project_triangulation_to_new_plane(pcl::ModelCoefficients::Ptr plane, tf::Transform tf);
		int cleanup_neighbours(Vertex_handle1 vh);

		//int add_four_points(void)
		//{
		//add_vertex_to_mesh(-9999, -9999, 0, 83.3, 0, -1);
		//add_vertex_to_mesh(-9999, 9999, 0, 83.3, 0, -1);
		//add_vertex_to_mesh(9999, -9999, 0, 83.3, 0, -1);
		//add_vertex_to_mesh(9999, 9999, 0, 83.3, 0, -1);
		//return 1;
		//}

		/////////////////////////////////////////////
		/////////////////////////////////////////////
		/////////////////////////////////////////////
		int set_constraint_polygon(void);
		int filter_isolated_vertexes(void);

		//Add vertex manager utils
		int get_point_face_distances(double x, double y, Face_handle fh, double* dv0, double* dv1, double* dv2);
		int get_point_face_proveniences(Face_handle fh, int* p0, int* p1, int* p2);
		bool do_face_vertices_have_equal_provenience(Face_handle fh, int* provenience=NULL);
		bool are_there_at_least_two_vertices_with_equal_provenience_not_mine(Face_handle fh, int my_provenience);
		bool at_least_two_vertices_have_my_provenience(Face_handle fh, int my_provenience);
		bool at_least_one_vertice_has_infinite_provenience(Face_handle fh);
		float get_face_average_weight(Face_handle fh);
		void remove_face_vertexes_without_this_provenience(Face_handle fh, int provenience);


		int remove_vertices_inside_triangle(double x0, double y0, double z0, float rgb0, 
				double x1, double y1, double z1, float rgb1, 
				double x2, double y2, double z2, float rgb2,
				float face_weight, int provenience);

		int add_vertex_manager(double x, double y, double z, float rgb, float weight, int provenience);
		bool check_if_face_should_be_inserted(double x0, double y0, double z0, float rgb0, 
				double x1, double y1, double z1, float rgb1, 
				double x2, double y2, double z2, float rgb2,
				float face_weight, int provenience);

		int remove_intersecting_constrained_edges(double x0, double y0, double z0, float rgb0, 
				double x1, double y1, double z1, float rgb1, 
				double x2, double y2, double z2, float rgb2,
				float face_weight, int provenience);


		CDT::Face face(double x0, double y0, double z0, float rgb0, 
				double x1, double y1, double z1, float rgb1, 
				double x2, double y2, double z2, float rgb2,
				float face_weight, int provenience);





		int iterate_intersecting_faces(CDT::Face_handle fti,  int predicate_number);
		//Define the predicates

		bool predicate_should_add_face(CDT::Face_handle fti, CDT::Face_handle fh)
		{
			//give a boost of 20% to weight of the face that is already there
			float boost = 1.2*fh->weight;
			if (boost>1) boost=1;

			if (fh->provenience==-1)
			{
				return true;
			}
			if (boost > fti->weight && fh->provenience != fti->provenience)
			{
				if(debug>0)ROS_INFO("Triangle to add overlaps faces with larger weight, not adding");
				return false;
			}	
			else
				return true;
		}

		/**
		 * @brief Check if the face fh has any vertex inside fti
		 *
		 * @param fti
		 * @param fh
		 *
		 * @return 
		 */
		bool predicate_remove_vertex(CDT::Face_handle fti, CDT::Face_handle fh);
		bool predicate_remove_intersecting_constraints(CDT::Face_handle fti, CDT::Face_handle fh);

		int add_face_to_mesh(CDT::Face_handle fti);
		float face_max_weight(Face_handle fh);

		int get_intersecting_faces_list(double x0, double y0, double z0, float rgb0, 
				double x1, double y1, double z1, float rgb1, 
				double x2, double y2, double z2, float rgb2,
				float face_weight, int provenience);

		CDT::Face_handle get_face_handle_from_t_face(t_face* f);

		//bool triangles_overlap(Point_2 t0_p0, Point_2 t0_p1, Point_2 t0_p2,
		//Point_2 t1_p0, Point_2 t1_p1, Point_2 t1_p2);

		//bool triangles_overlap(Point_2 it0_p0, Point_2 it0_p1, Point_2 it0_p2,
		//Point_2 it1_p0, Point_2 it1_p1, Point_2 it1_p2);

		bool triangles_overlap(Point_2 t0_p0, Point_2 t0_p1, Point_2 t0_p2,
				Point_2 t1_p0, Point_2 t1_p1, Point_2 t1_p2);


		int printf_face_info(CDT::Face_handle fh);

		unsigned int debug;
		std::vector<t_face> vector_faces; //the vector of faces that we should check
		pcl::PointCloud<pcl::PointXYZ> pc_faces_visited;
		pcl::PointCloud<pcl::PointXYZ> pc_faces_intersect;
		//Variables
		CDT dt;
		tf::Transform transform;
		pcl::PointCloud<pcl::PointXYZRGB> pc;
		pcl::PointCloud<pcl::PointXYZ> pc_constraints;
		std::vector<int> pc_proveniences;

		std::vector<size_t> pc_vertices_indices;
		pcl::PointCloud<pcl::PointXYZ> pc_vertices;

		class_polygon_intersection pi;
		std::vector<pcl::PointCloud<pcl::PointXYZ> > projection_union;

	private: 

		size_t index_count;

		bool segments_intersect_not_at_endpoints(CGAL::Segment_2<K_exact> seg1, CGAL::Segment_2<K_exact> seg2);



		void discoverComponents(const CDT & ct);
		void discoverComponent(const CDT & ct, Face_handle start, int index, std::list<CDT::Edge>& border );
		int initialize_all_faces_status(void)
		{
			for(All_faces_iterator it = dt.all_faces_begin(); it != dt.all_faces_end(); ++it)
			{  
				it->counter=-1;
				it->status=-1;
			}
			return 1;
		}

		int initialize_visited(void)
		{
			for(All_faces_iterator it = dt.all_faces_begin(); it != dt.all_faces_end(); ++it)
			{  
				it->visited=false;
			}
			return 1;
		}

};



#endif
/**
 *@}
 */      
