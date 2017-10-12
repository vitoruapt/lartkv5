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
 * @brief Here the polygon offset methods are implemented.
 * @author Miguel Oliveira
 * @version v0
 * @date 2011-12-19
 */
#ifndef _polygon_primitive_offsetpolygon_CPP_
#define _polygon_primitive_offsetpolygon_CPP_

#include "polygon_primitive.h"

#include<CGAL/basic.h>
#include<CGAL/Cartesian.h>
#include<CGAL/Polygon_2.h>
#include<CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include<CGAL/Straight_skeleton_builder_2.h>
#include<CGAL/Polygon_offset_builder_2.h>
#include<CGAL/compute_outer_frame_margin.h>
#include <CGAL/Cartesian.h>
#include <CGAL/Polygon_2.h>

// This is the recommended kernel
typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;

typedef Kernel::Point_2 Point_2;
typedef CGAL::Polygon_2<Kernel>    Contour;
typedef boost::shared_ptr<Contour> ContourPtr;
typedef std::vector<ContourPtr>    ContourSequence ;

typedef CGAL::Straight_skeleton_2<Kernel> Ss;

typedef Ss::Halfedge_iterator Halfedge_iterator;
typedef Ss::Halfedge_handle   Halfedge_handle;
typedef Ss::Vertex_handle     Vertex_handle;

typedef CGAL::Straight_skeleton_builder_traits_2<Kernel>      SsBuilderTraits;
typedef CGAL::Straight_skeleton_builder_2<SsBuilderTraits,Ss> SsBuilder;

typedef CGAL::Polygon_offset_builder_traits_2<Kernel>                  OffsetBuilderTraits;
typedef CGAL::Polygon_offset_builder_2<Ss,OffsetBuilderTraits,Contour> OffsetBuilder;


/**
 * @brief Offsets a given polygon. The offseting is produced along the XoY plane defined by the points in pcin.
 *
 * @param val the value of offset, in meters.
 * @param pcin the point cloud in
 * @param pcout the point cloud out, offseted.
 * @param tr the transformation that brings the points in pcin from the /world coordinates to the local coordinates, i.e., points in pcin are transformed by tr to pc_local, only them the offset is performed.
 */
void c_polygon_primitive::offset_polygon(double offset, const pcl::PointCloud<pcl::PointXYZ>::Ptr pcin, pcl::PointCloud<pcl::PointXYZ>::Ptr pcout, tf::Transform *tr)
	//void c_polygon_primitive::offset_polygon(double val)
{
	//Adapting demo from http://www.cgal.org/Manual/3.2/doc_html/cgal_manual/Straight_skeleton_2/Chapter_main.html#Section_16.3 

	pcl::PointCloud<pcl::PointXYZ>::Ptr pc_local = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr pc_local_extended = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

	//To offset a polygon the following steps are required:
	//1. transform the polygon pcin to local coordinates (i.e., where z=0)
	//2. Compute the offset in this local reference system
	//3. transform the offseted polygon to the global reference system

	//the given transform goes from local to global. We want to change from global to local. Hence we use the inverse transform
	pcl_ros::transformPointCloud(*pcin, *pc_local,  data.frames.current.transform.inverse());

	//we assume there are no values for z. The polygon is defined in the XY plane. Lets check if its true. If not send error and exit
	for (int i=0; i< (int) pc_local->size(); i++)
	{
		if (pc_local->points[i].z>0.01 || pc_local->points[i].z<-0.01)
		{
			//ROS_ERROR("offset polygon: %s has non zero z values in local frame. \n Are you sure that pcin is a projection to the plane?",data.misc.name);	
			//ROS_ERROR("pt%d x=%f y=%f z=%f",i,pc_local->points[i].x, pc_local->points[i].y, pc_local->points[i].z);	
			//exit(0);
		}
	}

	//now we must be sure that the polygon is oriented couter clockwise (CGAL only works for couterclockwise oriented polygons)
	Contour mypol;


	for (int i=0; i< (int) pc_local->size(); i++) //copy the pc to a Polygon_2 cgal data struct
	{
		mypol.push_back(Point_2(pc_local->points[i].x, pc_local->points[i].y));
	}




	if(!mypol.is_simple())
	{
		int keep_trying=1;

		while(keep_trying)
		{

			int tt=0;
			double prev_x=-999999, prev_y=-999999;
			//ROS_ERROR("offset polygon: polygon is not simple. Printing all vertexes");

			int tttt=0;
			for (Contour::Vertex_const_iterator j = mypol.vertices_begin(); j != mypol.vertices_end(); ++ j )
			{
				//ROS_ERROR("pt%d x=%f y=%f",tttt,j->x(), j->y());	
				tttt++;
			}


			Contour::Vertex_const_iterator begin = mypol.vertices_begin();
			Contour::Vertex_const_iterator end = --mypol.vertices_end();
			if ( begin->x() == end->x() && begin->y() == end->y())
			{
				//ROS_ERROR("Final and start point are equal. Erasing final point");
				mypol.erase(end);	
			}	


			for (Contour::Vertex_const_iterator j = mypol.vertices_begin(); j != mypol.vertices_end(); ++ j )
			{
				if(tt!=0)
				{
					if (prev_x==j->x() && prev_y==j->y())
					{
						//ROS_INFO("Erasing vertex %d",tt);
						mypol.erase(j);	
						break;
					}
					else
					{
						prev_x = j->x();	
						prev_y = j->y();	
					}

				}
				else
				{
					prev_x = j->x();	
					prev_y = j->y();	
				}


				//ROS_ERROR("pt%d x=%f y=%f",tt,j->x(), j->y());	
				tt++;
			}


			//ROS_ERROR("offset polygon: result of fix: %d", mypol.is_simple());

			int ttt=0;
			for (Contour::Vertex_const_iterator j = mypol.vertices_begin(); j != mypol.vertices_end(); ++ j )
			{
				//ROS_ERROR("pt%d x=%f y=%f",ttt,j->x(), j->y());	
				ttt++;
			}

			if (mypol.is_simple()) keep_trying=0;


		}
	}


	if(!mypol.is_counterclockwise_oriented()) //check if its counterclockwise. If not reverse
	{
		//ROS_INFO("Offset Polygon: ch  (%d points) is not counterclockwise oriented. Reversing orientation",(int)mypol.size());
		mypol.reverse_orientation();	
		//ROS_INFO("Result Polygon: ch  (%d points)",(int)mypol.size());
	}


	//Now we copy the polygon mypol (which was used just to check the orientation) to a std::vector
	std::vector<Point_2> star; //The polygon


	int t=0;
	for (Contour::Vertex_const_iterator j = mypol.vertices_begin(); j != mypol.vertices_end(); ++ j )
	{

		pc_local->points[t].x =  j->x();
		pc_local->points[t].y =  j->y();
		pc_local->points[t].z = 0; 
		t++;
	}




	//Setup the polygon with the points from pc.ch_l
	for (int i=0; i< (int) pc_local->size(); i++)
	{
		star.push_back(Point_2(pc_local->points[i].x, pc_local->points[i].y));
	}



	// We want an offset contour in the outside.
	// Since the package doesn't support that operation directly, we use the following trick:
	// (1) Place the polygon as a hole of a big outer frame.
	// (2) Construct the skeleton on the interior of that frame (with the polygon as a hole)
	// (3) Construc the offset contours
	// (4) Identify the offset contour that corresponds to the frame and remove it from the result

	// First we need to determine the proper separation between the polygon and the frame.
	// We use this helper function provided in the package.
	boost::optional<double> margin = CGAL::compute_outer_frame_margin(star.begin(),star.end(), offset);



	// Proceed only if the margin was computed (an extremely sharp corner might cause overflow) 
	if ( margin )
	{
		// Get the bbox of the polygon 
		CGAL::Bbox_2 bbox = CGAL::bbox_2(star.begin(),star.end());

		// Compute the boundaries of the frame 
		double fxmin = bbox.xmin() - *margin ;
		double fxmax = bbox.xmax() + *margin ;
		double fymin = bbox.ymin() - *margin ;
		double fymax = bbox.ymax() + *margin ;

		// Create the rectangular frame
		Point_2 frame[4]= { Point_2(fxmin,fymin)
			, Point_2(fxmax,fymin)
				, Point_2(fxmax,fymax)
				, Point_2(fxmin,fymax)
		} ;



		// Instantiate the skeleton builder
		SsBuilder ssb ;  


		// Enter the frame
		ssb.enter_contour(frame,frame+4);



		// Enter the polygon as a hole of the frame (NOTE: as it is a hole we insert it in the opposite orientation)
		ssb.enter_contour(star.rbegin(),star.rend());



		// Construct the skeleton
		boost::shared_ptr<Ss> ss = ssb.construct_skeleton();



		// Proceed only if the skeleton was correctly constructed.
		if ( ss )
		{
			// Instantiate the container of offset contours
			ContourSequence offset_contours ;


			// Instantiate the offset builder with the skeleton
			OffsetBuilder ob(*ss);


			// Obtain the offset contours
			ob.construct_offset_contours(offset, std::back_inserter(offset_contours));


			// Locate the offset contour that corresponds to the frame
			// That must be the outmost offset contour, which in turn must be the one
			// with the largetst unsigned area.
			ContourSequence::iterator f = offset_contours.end();
			double lLargestArea = 0.0 ;  

			for (ContourSequence::iterator i = offset_contours.begin(); i != offset_contours.end(); ++ i  )
			{

				double lArea = CGAL_NTS abs( (*i)->area() ) ; //Take abs() as  Polygon_2::area() is signed.
				if ( lArea > lLargestArea )
				{
					f = i ;
					lLargestArea = lArea ;
				}
			}


			// Remove the offset contour that corresponds to the frame.
			offset_contours.erase(f);


			// Print out the skeleton
			Halfedge_handle null_halfedge ;
			Vertex_handle   null_vertex ;



			// Dump the edges of the skeleton
			//for ( Halfedge_iterator i = ss->halfedges_begin(); i != ss->halfedges_end(); ++i )
			//{
			//std::string edge_type = (i->is_bisector())? "bisector" : "contour";
			//Vertex_handle s = i->opposite()->vertex(); 
			//Vertex_handle t = i->vertex();
			//std::cout << "(" << s->point() << ")->(" << t->point() << ") " << edge_type << std::endl;
			//}

			// Dump the generated offset polygons

			//std::cout << offset_contours.size() << " offset contours obtained\n" ;


			for (ContourSequence::const_iterator i = offset_contours.begin(); i != offset_contours.end(); ++ i )
			{
				// Each element in the offset_contours sequence is a shared pointer to a Polygon_2 instance.
				//std::cout << (*i)->size() << " vertices in offset contour\n" ;
				pc_local_extended->points.resize((*i)->size());
				pc_local_extended->height=1;
				pc_local_extended->width=(*i)->size();
				int t=0;

				for (Contour::Vertex_const_iterator j = (*i)->vertices_begin(); j != (*i)->vertices_end(); ++ j )
				{
					//std::cout << "(" << j->x() << "," << j->y() << ")" << std::endl ;
					pc_local_extended->points[t].x =  j->x();
					pc_local_extended->points[t].y =  j->y();
					pc_local_extended->points[t].z = 0; 
					t++;
				}

				break; //Use only first offset contour
			}
		}
	}



	//STEP 3. Transform pc local extended to pcout
	//tf::Transform tr_inv = (*tr).inverse(); //the given transform goes from local to global. We want to change from global to local. Hence we use the inverse transform
	pcl_ros::transformPointCloud(*pc_local_extended, *pcout,  data.frames.current.transform);


	//Delete temporary pc
	pc_local.reset();
	pc_local_extended.reset();	

} 





#endif
 /**
 *@}
 */
/*Previous 3 lines appended automatically on Sun Feb  5 19:40:16 WET 2012 */
