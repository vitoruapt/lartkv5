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
 * @brief These function provide texture mapping of triangles
 *@{
 */
#ifndef _TEXTURED_TRIANGLE_CPP_
#define _TEXTURED_TRIANGLE_CPP_

#include "textured_triangle.h"


//TEXTURED VERTEX methods

class_textured_vertex::class_textured_vertex(float vx, float vy, float vz, float vrgb)
{
	x=vx; y=vy; z=vz; rgb=vrgb;	
}

//TEXTURED TRIANGLE methods
class_textured_triangle::class_textured_triangle(class_textured_vertex v0, class_textured_vertex v1,class_textured_vertex v2, float w, int p)
{
	v[0].x = v0.x; 	v[0].y = v0.y;	v[0].z = v0.z;	v[0].rgb = v0.rgb;
	v[1].x = v1.x; 	v[1].y = v1.y;	v[1].z = v1.z;	v[1].rgb = v1.rgb;
	v[2].x = v2.x; 	v[2].y = v2.y;	v[2].z = v2.z;	v[2].rgb = v2.rgb;
	weight = w;
	provenience = p;
}


bool class_textured_triangle::overlaps(class_textured_triangle* t)
{

	CGAL::Triangle_2<K1> t1(CGAL::Point_2<K1>(v[0].x, v[0].y), CGAL::Point_2<K1>(v[1].x, v[1].y), CGAL::Point_2<K1>(v[2].x, v[2].y));
	CGAL::Triangle_2<K1> t2(CGAL::Point_2<K1>(t->v[0].x, t->v[0].y), CGAL::Point_2<K1>(t->v[1].x, t->v[1].y), CGAL::Point_2<K1>(t->v[2].x, t->v[2].y));

    CGAL::Object result = CGAL::intersection(t1, t2);
	if (const CGAL::Triangle_2<K1> *itriangle = CGAL::object_cast<CGAL::Triangle_2<K1> >(&result)) 
	{
		//if intersection is a triangle then the triangles overlap
		//ROS_WARN("intersection was a triangle");
		return 1;
	} 
	else if (const std::vector<CGAL::Point_2<K1> > *iptvector = CGAL::object_cast<std::vector<CGAL::Point_2<K1> > >(&result))
   	{
		//if intersection is vector of points, i.e., a polygon, then the triangles overlap
		//ROS_WARN("intersection was a pt vector");
		return 1;
	}
   	else
   	{
		//if intersection is any of the other cases, i.e., segment or point, then there is only overlap in the boundaries 
		return 0;
	}
//}

//Segment_2
//Triangle_2
//std::vector<Point_2>
	//CDT2 triang; 
	//for (int i=0; i<3; i++)
		//triang.insert(CDT2::Point(v[i].x, v[i].y));

	//CDT2::Locate_type lt[3];
	//int li;
	//CDT2::Face_handle fh = triang.locate(CDT2::Point(t->v[0].x, t->v[0].y), lt[0], li); //locate the point where we want to insert a new vertex


	//fh = triang.locate(CDT2::Point(t->v[1].x, t->v[1].y), lt[1], li); //locate the point where we want to insert a new vertex
	//fh = triang.locate(CDT2::Point(t->v[2].x, t->v[2].y), lt[2], li); //locate the point where we want to insert a new vertex


	//if (lt[0])

	//First see if any of the t vertices is inside the class triangle
	//for (int i=0; i<3; i++)
	//{
		////if ((v[0].x != t->v[i].x && v[0].y != t->v[i].y) &&
			////(v[1].x != t->v[i].x && v[1].y != t->v[i].y)
		//if (point_inside_triangle(v[0].x, v[0].y,  v[1].x, v[1].y,v[2].x, v[2].y, t->v[i].x, t->v[i].y))
			//return true;
	//}

	////then we see if any of the class triangle vertices are inside the input triangle t
	//for (int i=0; i<3; i++)
	//{
		//if (point_inside_triangle(t->v[0].x, t->v[0].y,  t->v[1].x, t->v[1].y,t->v[2].x, t->v[2].y, v[i].x, v[i].y))
			//return true;
	//}

	//double X,Y;
	//if (lineSegmentIntersection( t->v[0].x, t->v[0].y,  t->v[1].x, t->v[1].y, 
							 //v[0].x, v[0].y,  v[1].x, v[1].y,
							 //&X, &Y)==DO_INTERSECT)
			//return true;

	//if (lineSegmentIntersection( t->v[1].x, t->v[1].y,  t->v[2].x, t->v[2].y, 
							 //v[0].x, v[0].y,  v[1].x, v[1].y,
							 //&X, &Y)==DO_INTERSECT)
			//return true;

	//if (lineSegmentIntersection( t->v[2].x, t->v[2].y,  t->v[0].x, t->v[0].y, 
							 //v[0].x, v[0].y,  v[1].x, v[1].y,
							 //&X, &Y)==DO_INTERSECT)
			//return true;

	//if (lineSegmentIntersection( t->v[0].x, t->v[0].y,  t->v[1].x, t->v[1].y, 
							 //v[2].x, v[2].y,  v[0].x, v[0].y,
							 //&X, &Y)==DO_INTERSECT)
			//return true;

	//if (lineSegmentIntersection( t->v[1].x, t->v[1].y,  t->v[2].x, t->v[2].y, 
							 //v[2].x, v[2].y,  v[0].x, v[0].y,
							 //&X, &Y)==DO_INTERSECT)
			//return true;

	//if (lineSegmentIntersection( t->v[2].x, t->v[2].y,  t->v[0].x, t->v[0].y, 
							 //v[2].x, v[2].y,  v[0].x, v[0].y,
							 //&X, &Y)==DO_INTERSECT)
			//return true;

	//if (lineSegmentIntersection( t->v[0].x, t->v[0].y,  t->v[1].x, t->v[1].y, 
							 //v[1].x, v[1].y,  v[2].x, v[2].y,
							 //&X, &Y)==DO_INTERSECT)
			//return true;

	//if (lineSegmentIntersection( t->v[1].x, t->v[1].y,  t->v[2].x, t->v[2].y, 
							 //v[1].x, v[1].y,  v[2].x, v[2].y,
							 //&X, &Y)==DO_INTERSECT)
			//return true;

	//if (lineSegmentIntersection( t->v[2].x, t->v[2].y,  t->v[0].x, t->v[0].y, 
							 //v[1].x, v[1].y,  v[2].x, v[2].y,
							 //&X, &Y)==DO_INTERSECT)
			//return true;

	//return false;
}


//TEXTURE SET methods

int class_texture_set::add_triangle(class_textured_vertex v0, class_textured_vertex v1, class_textured_vertex v2, int provenience, float weight, t_add_method method)
{

	if (method==FORCE)
	{
		boost::shared_ptr<class_textured_triangle> t(new class_textured_triangle(v0,v1,v2, weight, provenience));
		set.push_back(t);
	}
	else if (method==CHECK)
	{
		boost::shared_ptr<class_textured_triangle> t(new class_textured_triangle(v0,v1,v2, weight, provenience));

		std::priority_queue<size_t> index_to_remove; //a list of triangles to remove on account of the insertion of the new one

		//ROS_INFO("ADDING new triangle, list size %d", (int)set.size());

		//check i the triangle to be inserted intersects any of the ones on the list
		for (size_t i=0; i<set.size(); ++i)
		{
			if (t->provenience != set[i]->provenience)
			if (set[i]->overlaps(&(*t)))
			{

				//ROS_DEBUG("new triangle overlaps with %i from list",(int)i);
				//printf("new:\n");
				//t.print_info();
				//printf("triangle %d:\n",(int)i);
				//it->print_info();
				//if they overlap must compare weight
				if (t->weight > set[i]->weight) //if new triangle has larger weight
				{
					//ROS_INFO("LARGER: new triangle W (%f); old triangle %d (W=%f)", t->weight,(int)i, set[i]->weight);
					index_to_remove.push(i);
				}
				else //if new triangle has less weight, dont add it and return
				{
					//ROS_INFO("SMALLER: new triangle W (%f); old triangle %d (W=%f)", t->weight,(int)i, set[i]->weight);
					t.reset();
					return DID_NOT_ADD_TRIANGLE;
				}
			}
		}

		//if the cycle ended and reached here, no return occurred which means the new triangle must be inserted and the ones in index_to_remove must be deleted

		//ROS_INFO("must remove %d triangles before adding new",(int) key_to_remove.size());
		//first lets delete. Must be carefull not to loose the indexes propper position, to delete from the end to the begining

		//ROS_INFO("List size before removing=%d", (int)set.size());
		while (!index_to_remove.empty())
		  {
			  ROS_INFO("Removing triangle %d", (int)index_to_remove.top());
			 set.erase(set.begin() + index_to_remove.top());	
			 index_to_remove.pop();
		  }


		//for (size_t i=0; i<index_to_remove.size(); ++i)

		//ROS_INFO("AAA List size after removing=%d", (int)set.size());
		//finnaly we add the new triangle
		set.push_back(t);
		//ROS_INFO("AAA List size after adding=%d", (int)set.size());
		return ADDED_TRIANGLE;
	}

	return -1;
}

int class_texture_set::set_transform(tf::Transform* st)
{
	transform.setRotation(st->getRotation());	
	transform.setOrigin(st->getOrigin());
	return 1;
}

int class_texture_set::export_to_pc(void)
{
	pc.points.erase(pc.points.begin(), pc.points.end());
	pc_proveniences.erase(pc_proveniences.begin(), pc_proveniences.end());

	for(size_t k=0; k<set.size(); k++)
	//std::map<size_t, boost::shared_ptr<class_textured_triangle> >::iterator it;
	//for (it=set.begin(); it!=set.end();++it)
	{

		//printf("INSIDE TRI%d\n v0 x=%f y=%f z=%f\n v1 x=%f y=%f z=%f\n v2 x=%f y=%f z=%f\n",(int)k, set[k].v[0].x, set[k].v[0].y,set[k].v[0].z,set[k].v[1].x,set[k].v[1].y,set[k].v[1].z,set[k].v[2].x,set[k].v[2].y,set[k].v[2].z);
		for (int u=0; u<3;u++)
		{
			pcl::PointCloud<pcl::PointXYZ> pc_local;
			pcl::PointXYZ pt;

			pt.x = set[k]->v[u].x; 
			pt.y = set[k]->v[u].y; 
			pt.z = set[k]->v[u].z; 
			pc_local.points.push_back(pt);

			pcl::PointCloud<pcl::PointXYZ> pc_global;
			pcl_ros::transformPointCloud(pc_local, pc_global, transform.inverse());     
			pcl::PointXYZRGB ptcolor;
			ptcolor.x = pc_global.points[0].x;
			ptcolor.y = pc_global.points[0].y;
			ptcolor.z = pc_global.points[0].z;
			ptcolor.rgb = set[k]->v[u].rgb;

			pc.points.push_back(ptcolor);
			pc_proveniences.push_back(set[k]->provenience);

			pc.height = 1;
			pc.width = pc.points.size();
			pc.is_dense = 0;

		}
	}

	ROS_INFO("Exported triangle set %d triangles from a list with set.size()=%d", (int)pc.points.size(),  (int)set.size());


	return 1;


}

#endif
/**
 *@}
 */      
