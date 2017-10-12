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
 * @defgroup polygon_primitive_with_texture  polygon_primitive_with_texture -Automatic Group Name-Please adjust. vs strikes again :-) 05-Feb-2012
 * @{
 * @file 
 * @brief A class c_polygon_primitive_with_texture that contains information about a detected polygon primitive as well as the methods the texturing
 ** @author Miguel Armando Riem de Oliveira
 * @version 0.0
 * @date 2011-12-15
 */
#ifndef _polygon_primitive_with_texture_H_
#define _polygon_primitive_with_texture_H_

#ifndef PFLN
#define PFLN {printf("DEBUG PRINT FILE %s LINE %d\n",__FILE__,__LINE__);}
#endif
//####################################################################
//// Includes:
////####################################################################

#include "polygon_primitive.h"
#include "camera_projection.h"
#include "constrained_delaunay_triangulation.h"
#include "textured_triangle.h"
#include <colormap/colormap.h>
#include <pthread.h>


class c_polygon_primitive_with_texture: public c_polygon_primitive
{
	// ------------------------------------------------------------------------------
	//PUBLIC SECTION OF THE CLASS
	// ------------------------------------------------------------------------------
	public:

		c_polygon_primitive_with_texture(ros::NodeHandle* n): c_polygon_primitive(n)
	{
		id_visited_faces=0; ns_visited_faces = "visited_faces";
		id_vertices_indices=0; ns_vertices_indices = "vertices_indices";
		id_edge0=0; ns_edge0 = "edge0";
		id_edge1=0; ns_edge1 = "edge1";
		id_edge2=0; ns_edge2 = "edge2";
		id_next_triangle=0; ns_next_triangle = "next_triangle";
		id_proveniences=0; ns_proveniences = "proveniences";
		id_constraints=0; ns_constraints = "constraints";
		id_local_mesh=0; ns_local_mesh = "local_mesh";
		id_camera_canvas=0; ns_camera_canvas = "camera_canvas";
		id_camera_position=0; ns_camera_position = "camera_position";
		id_projection_name=0; ns_projection_name = "projection_name";
		id_camera_intersection=0; ns_camera_intersection="camera_intersection";
		id_camera_intersection_vertices=0; ns_camera_intersection_vertices="camera_intersection_vertices";
		id_textured_triangles=0; ns_textured_triangles="textured_triangles";
		id_triangle_edges=0; ns_triangle_edges="triangle_edges";
		id_triangle_vertices=0; ns_triangle_vertices="triangle_vertices";
		id_projection_union=0; ns_projection_union="projection_union";
		id_projection_union_vertices=0; ns_projection_union_vertices="projection_union_vertices";
		std::string str="hot";
		colormap = (class_colormap*) new class_colormap(str,8, 1, false);
		mutex = PTHREAD_MUTEX_INITIALIZER;
	};


		int compute_mean_and_std(std::vector<float>* v, float* mean, float* std);

		size_t ask_for_number(void);
		int publish_to_rviz(ros::Publisher* p_marker_pub);
		int build_global_texture_set(ros::Publisher* p_marker_pub);
		int build_global_mesh(ros::Publisher* p_marker_pub);
		int compute_projection_union(void);
		int readapt_to_new_plane(polygon_primitive_msg::polygon_primitive* new_plg);
		int erase_old_markers(visualization_msgs::MarkerArray* marker_vec, unsigned int from, unsigned int to, std::string namesp);

		int add_camera_projection_to_triangle_mesh(int projection_index, ros::Publisher* p_marker_pub);

		int add_camera_projection_known_camera(cv::Mat* m, ros::Time t, tf::StampedTransform tf, std::string cam_name, std::string projection_name, cv::Scalar color);
		int add_camera_projection(cv::Mat* m, ros::Time t, double fx, double fy, double cx, double cy, tf::StampedTransform tf, double k1, double k2, double p1, double p2, double k3, double sd_cx, double sd_cy, double param, double scale, std::string projection_name, cv::Scalar color);

		int create_textures_vizualization_msg(visualization_msgs::MarkerArray* marker_vec, bool reset_id=false);
		int get_mesh_statistics(void);

		class_constrained_delaunay_triangulation dp;

		//a vector of camera projections. Can be many shots of one camera across time or multiple cameras
		std::vector<class_camera_projection> cp;

		pcl::PointCloud<pcl::PointXYZ> next_triangle;
		class_texture_set ts;
		class_colormap* colormap;
		pthread_mutex_t mutex;
		pthread_t thread;



		// ------------------------------------------------------------------------------
		//PRIVATE SECTION OF THE CLASS
		// ------------------------------------------------------------------------------
	private:

		//RVIZ stuff
		unsigned int id_visited_faces;
		std::string ns_visited_faces;
		unsigned int id_vertices_indices;
		std::string ns_vertices_indices;
		unsigned int id_edge0;
		std::string ns_edge0;
		unsigned int id_edge1;
		std::string ns_edge1;
		unsigned int id_edge2;
		std::string ns_edge2;
		unsigned int id_next_triangle;
		std::string ns_next_triangle;
		unsigned int id_proveniences;
		std::string ns_proveniences;
		unsigned int id_constraints;
		std::string ns_constraints;
		unsigned int id_local_mesh;
		std::string ns_local_mesh;
		unsigned int id_camera_canvas;
		std::string ns_camera_canvas;
		unsigned int id_camera_position;
		std::string ns_camera_position;
		unsigned int id_projection_name;
		std::string ns_projection_name;
		unsigned int id_camera_intersection;
		std::string ns_camera_intersection;
		unsigned int id_camera_intersection_vertices;
		std::string ns_camera_intersection_vertices;
		unsigned int id_textured_triangles;
		std::string ns_textured_triangles;
		unsigned int id_triangle_edges;
		std::string ns_triangle_edges;
		unsigned int id_triangle_vertices;
		std::string ns_triangle_vertices;
		unsigned int id_projection_union;
		std::string ns_projection_union;
		unsigned int id_projection_union_vertices;
		std::string ns_projection_union_vertices;

		unsigned int id_start;

};



#endif
/**
 *@}
 */      

