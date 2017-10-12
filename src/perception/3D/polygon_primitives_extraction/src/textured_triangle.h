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
 * @addtogroup textured_triangle 
 * @file
 * @brief header for textured triangle
 *@{
 */
#ifndef _TEXTURED_TRIANGLE_H_
#define _TEXTURED_TRIANGLE_H_

//####################################################################
// Includes:
//####################################################################

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K1;
//typedef CGAL::Triangulation_vertex_base_2<K1>                     Vb1;
//typedef CGAL::Constrained_triangulation_face_base_2<K1>           Fb1;
//typedef CGAL::Triangulation_data_structure_2<Vb1,Fb1>              TDS1;
//typedef CGAL::Exact_predicates_tag                               Itag1;
//typedef CGAL::Constrained_Delaunay_triangulation_2<K1, TDS1, Itag1> CDT2;




//System Includes
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl/ModelCoefficients.h>
#include <tf/tf.h>
#include <visualization_msgs/Marker.h>
#include <queue>

#define DONT_INTERSECT    0
#define DO_INTERSECT      1
#define COLLINEAR         2

#define SAME_SIGNS( a, b )  \
	(((long) ((unsigned long) a ^ (unsigned long) b)) >= 0 )



class class_textured_vertex
{
	public:
		class_textured_vertex(void){};
		class_textured_vertex(float vx, float vy, float vz, float vrgb);
		~class_textured_vertex(void){};

		class_textured_vertex operator=(class_textured_vertex vin)
		{
			class_textured_vertex v_out;
			v_out.x = vin.x;	
			v_out.y = vin.y;	
			v_out.z = vin.z;	
			v_out.rgb = vin.rgb;	
			return v_out;

		}

		float x,y,z,rgb;

};

class class_textured_triangle
{
	public:
		class_textured_triangle(void){};
		class_textured_triangle(class_textured_vertex v0, class_textured_vertex v1,class_textured_vertex v2, float weight=0, int provenience=-1);

		~class_textured_triangle(void){};

		class_textured_vertex v[3];	
		float weight;
		float provenience;

		bool overlaps(class_textured_triangle* t);
		void print_info(void)
		{
			ROS_INFO("v0 x=%f y=%f z=%f rgb=%f\n v1 x=%f y=%f z=%f rgb=%f\n v2 x=%f y=%f z=%f rgb=%f\n", v[0].x, v[0].y, v[0].z, v[0].rgb, v[1].x, v[1].y, v[1].z, v[1].rgb,v[2].x, v[2].y, v[2].z, v[2].rgb);	
		}

	private: 
		//  

		int test2( double px, double py, double m, double b ) {    
			if (py < m * px + b ) {
				return -1; // point is under line
			}else if ( py == m * px + b ){
				return 0; // point is on line
			} else {
				return 1; // point is over line
			}
		}

		int test1(double px, double py, double m,double b, double lx,double ly) {     
			return (test2(px,py, m,b) == test2(lx,ly,m,b));    
		} 

		

		int point_inside_triangle(double x0,double y0,double x1,double y1,double x2,double y2,double px, double py) 
		{
			int line1, line2, line3;    
			double m01 = (y1-y0)/(x1-x0);    
			double b01 = m01 * -x1 + y1;    
			double m02, m12, b02, b12;    
			m02 = (y2-y0)/(x2-x0);    
			m12 = (y2-y1)/(x2-x1);    
			b02 = m02 * -x2 + y2;    
			b12 = m12 * -x2 + y2;


			// vertical line checks

			if( x1 == x0 ) {    
				line1 = ( (px < x0) == (x2 < x0) );    
			} else {    
				line1 = test1( px, py, m01, b01,x2,y2);    
			}

			if( x1 == x2 ) {    
				line2 = ( (px < x2) == (x0 < x2) );    
			} else {    
				line2 = test1(px,py, m12, b12,x0,y0);    
			}

			if( x2 == x0 ) {    
				line3 = ( (px < x0 ) == (x1 < x0) );} else {    
					line3 = test1(px, py, m02,b02,x1,y1);    
				}

			if(line1==-1 && line2==-1 && line3==-1)
				return 1;
			else if(line1==1 && line2==1 && line3==1)
				return 1;
			else 
				return 0;


			//return line1 && line2 && line3;
		}


		/**
		 * @brief public domain function by Darel Rex Finley, 2006. Determines the intersection point of the line segment defined by points A and B with the line segment defined by points C and D. Returns YES if the intersection point was found, and stores that point in X,Y.  Returns NO if there is no determinable intersection point, in which case X,Y will be unmodified.
		 * @return 
		 */
		int lineSegmentIntersection(
				float Ax, float Ay,
				float Bx, float By,
				float Cx, float Cy,
				float Dx, float Dy,
				double *X, double *Y) {

			double  distAB, theCos, theSin, newX, ABpos ;

			//  Fail if either line segment is zero-length.
			if (((Ax==Bx) && (Ay==By)) || ((Cx==Dx) && (Cy==Dy))) return DONT_INTERSECT;

			//  Fail if the segments share an end-point.
			if (((Ax==Cx) && (Ay==Cy)) || ((Bx==Cx) && (By==Cy))
					||  ((Ax==Dx) && (Ay==Dy)) || ((Bx==Dx) && (By==Dy))) {
				return DONT_INTERSECT; }

			//  (1) Translate the system so that point A is on the origin.
			Bx-=Ax; By-=Ay;
			Cx-=Ax; Cy-=Ay;
			Dx-=Ax; Dy-=Ay;

			//  Discover the length of segment A-B.
			distAB=sqrt(Bx*Bx+By*By);

			//  (2) Rotate the system so that point B is on the positive X axis.
			theCos=Bx/distAB;
			theSin=By/distAB;
			newX=Cx*theCos+Cy*theSin;
			Cy  =Cy*theCos-Cx*theSin; Cx=newX;
			newX=Dx*theCos+Dy*theSin;
			Dy  =Dy*theCos-Dx*theSin; Dx=newX;

			//  Fail if segment C-D doesn't cross line A-B.
			if ((Cy<0. && Dy<0.) || (Cy>=0. && Dy>=0.)) return DONT_INTERSECT;

			//  (3) Discover the position of the intersection point along line A-B.
			ABpos=Dx+(Cx-Dx)*Dy/(Dy-Cy);

			//  Fail if segment C-D crosses line A-B outside of segment A-B.
			if (ABpos<0. || ABpos>distAB) return DONT_INTERSECT;

			//  (4) Apply the discovered position to line A-B in the original coordinate system.
			*X=Ax+ABpos*theCos;
			*Y=Ay+ABpos*theSin;

			//  Success.
			return DO_INTERSECT;
		}


};

typedef enum
{
	FORCE=55,
	CHECK
}t_add_method;


#define ADDED_TRIANGLE 1
#define DID_NOT_ADD_TRIANGLE 0

/**
 * @brief 
 */
class class_texture_set
{
	public:

		//methods
		class_texture_set(void)
		{
			next_key=0;
		};

		~class_texture_set(void){};

		int add_triangle(class_textured_vertex v0, class_textured_vertex v1, class_textured_vertex v2, int provenience, float weight, t_add_method method);
		int set_transform(tf::Transform* st);
		int export_to_pc(void);

		//Variables
		std::vector<boost::shared_ptr<class_textured_triangle> > set;
		pcl::PointCloud<pcl::PointXYZRGB> pc;
		std::vector<int> pc_proveniences;
		tf::Transform transform;
		size_t next_key;
};


#endif
/**
 *@}
 */      
