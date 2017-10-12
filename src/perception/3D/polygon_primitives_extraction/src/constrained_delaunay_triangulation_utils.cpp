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
 * @brief Some utils for the constrained delaunay triangulation code
 */
#ifndef _CONSTRAINED_DELAUNAY_TRIANGULATION_UTILS_CPP_
#define _CONSTRAINED_DELAUNAY_TRIANGULATION_UTILS_CPP_

#include "constrained_delaunay_triangulation.h"


/**
 * @brief Tests if two points are equal
 *
 * @param p1 Point 1
 * @param p2 Point 2
 *
 * @return true if p1=p2, false otherwise
 */
bool class_constrained_delaunay_triangulation::equal(Point_2 p1, Point_2 p2)
{
	if (p1.x()==p2.x() && p1.y()==p2.y())
		return true;	
	else
		return false;	
}

/**
 * @brief Tests if two points are equal
 *
 * @param p1 Point 1
 * @param p2 Point 2
 *
 * @return true if p1=p2, false otherwise
 */
bool class_constrained_delaunay_triangulation::equal_e(CGAL::Point_2<K_exact> p1, CGAL::Point_2<K_exact> p2)
{
	if (CGAL::to_double(p1.x())==CGAL::to_double(p2.x()) && CGAL::to_double(p1.y())==CGAL::to_double(p2.y()))
	{
		return true;	
	}
	else
	{
		return false;	
	}
}

/**
 * @brief Tests if the triangulation is valid or not. Usefull for indentifying broken triangulation data structures
 *
 * @return 1 on success 
 */
int class_constrained_delaunay_triangulation::test_if_triangulation_is_valid(void)
{	
	if(debug)
	{
		ROS_INFO("Checking if triangulation is valid ...");
		bool ret = dt.is_valid(true, 1);
		if (ret==true)
			ROS_INFO("Finished Checking: triangulation is valid");
		else
			ROS_ERROR("Finished Checking: triangulation is not valid");
	}
	return 1;
}

/**
 * @brief Tests whether a point p is inside or on the edges of triangle formed by vertices v0, v1 and v2
 *
 * @param v0 The first vertex
 * @param v1 The second vertex
 * @param v2 The third vertex
 * @param p the point to test
 *
 * @return true if point is inside of on the edges, false if point is outside of the triangle or on its vertices
 */
bool class_constrained_delaunay_triangulation::overlaps(Point_2* v0, Point_2* v1, Point_2* v2, Point_2* p)
{
	CDT triang;
	triang.insert(*v0);
	triang.insert(*v1);
	triang.insert(*v2);

	CDT::Locate_type lt;
	int li;
	triang.locate(*p, lt, li); //locate the point on the triangle

	if (lt==CDT::FACE || lt==CDT::EDGE)
		return true;
	else
		return false;	
}

/**
 * @brief Tests if two line segments intersect
 *
 * @param iseg1	first line segment
 * @param iseg2 second line segment
 *
 * @return true if segments intersect not at any of the endpoints. False otherwise
 */
bool class_constrained_delaunay_triangulation::segments_intersect_not_at_endpoints(CGAL::Segment_2<K_exact> iseg1, CGAL::Segment_2<K_exact> iseg2)
{


	if ((equal(iseg1.point(0), iseg2.point(0)) &&  equal(iseg1.point(1), iseg2.point(1))) ||
			(equal(iseg1.point(1), iseg2.point(0)) &&  equal(iseg1.point(0), iseg2.point(1)))
	   )
	{
		if(debug>1)	ROS_INFO("SEG_INTR: segments are the same");
		return false;
	}
	else
	{
		//Create segments using exact kernels
		CGAL::Point_2<K_exact> seg1_p0(iseg1.point(0).x(), iseg1.point(0).y());
		CGAL::Point_2<K_exact> seg1_p1(iseg1.point(1).x(), iseg1.point(1).y());
		CGAL::Segment_2<K_exact> seg1(seg1_p0, seg1_p1);

		CGAL::Point_2<K_exact> seg2_p0(iseg2.point(0).x(), iseg2.point(0).y());
		CGAL::Point_2<K_exact> seg2_p1(iseg2.point(1).x(), iseg2.point(1).y());
		CGAL::Segment_2<K_exact> seg2(seg2_p0, seg2_p1);


		CGAL::Object result = CGAL::intersection(seg1, seg2);
		if (const CGAL::Point_2<K_exact> *ip = CGAL::object_cast<CGAL::Point_2<K_exact> >(&result)) 
		{
			if (equal_e(seg1.point(0), seg2.point(0)) || equal_e(seg1.point(0), seg2.point(1)) || equal_e(seg1.point(1), seg2.point(1)) || equal_e(seg1.point(1), seg2.point(0))) 

			{
				return false;
				//if ((ip->x()!=seg1.point(0).x() || ip->y()!=seg1.point(0).y()) &&
				//(ip->x()!=seg1.point(1).x() || ip->y()!=seg1.point(1).y()) &&
				//(ip->x()!=seg2.point(0).x() || ip->y()!=seg2.point(0).y()) &&	
				//(ip->x()!=seg2.point(1).x() || ip->y()!=seg2.point(1).y()))
			}
			else
			{
				if(debug>1)ROS_INFO("SEG_INTR: point intersection and not at endpoints ip (%f, %f) seg1 p0 (%f %f) p1 (%f %f) seg2 p0(%f %f) p1(%f %f)", CGAL::to_double(ip->x()),CGAL::to_double( ip->y()),CGAL::to_double( seg1.point(0).x()),CGAL::to_double( seg1.point(0).y()),CGAL::to_double( seg1.point(1).x()), CGAL::to_double(seg1.point(1).y()), CGAL::to_double(seg2.point(0).x()),CGAL::to_double(seg2.point(0).y()),CGAL::to_double(seg2.point(1).x()),CGAL::to_double(seg2.point(1).y()));
				return true;
			}
		} 
		else if(const CGAL::Segment_2<K_exact> *ip = CGAL::object_cast<CGAL::Segment_2<K_exact> >(&result))
		{
			if(debug>1)		ROS_INFO("SEG_INTR: intersection on a segment");
			return true;
		}
		else
		{
			return false;
		}

	}

	return false;
}


/**
 * @brief Checks if a triangle with vertices xyz0, xyz1 and xyz2 is degenerate.
 *
 * @return true is triangle is degenerate, false otherwise
 */
bool class_constrained_delaunay_triangulation::is_degenerate(double x0, double y0, double z0,
		double x1, double y1, double z1,
		double x2, double y2, double z2)
{
	CGAL::Point_2<K_exact> t0_p0(x0, y0);
	CGAL::Point_2<K_exact> t0_p1(x1,y1);
	CGAL::Point_2<K_exact> t0_p2(x2, y2);

	CGAL::Triangle_2<K_exact>* t0;

	//make sure the triangles are built in ccw orientation
	if ((CGAL::Triangle_2<K_exact>(t0_p0, t0_p1, t0_p2)).orientation()>0)
		t0 = new CGAL::Triangle_2<K_exact>(t0_p0, t0_p1, t0_p2);
	else
		t0 = new CGAL::Triangle_2<K_exact>(t0_p0, t0_p2, t0_p1);

	if (t0->is_degenerate())
		return true;
	else
		return false;
}

/**
 * @brief Tests if triangle 1 ovelaps with triangle 1
 *
 * @param it0_p0 triangle 1, vertice 0
 * @param it0_p1 triangle 1, vertice 1
 * @param it0_p2 triangle 1, vertice 2
 * @param it1_p0 triangle 2, vertice 0
 * @param it1_p1 triangle 2, vertice 1
 * @param it1_p2 triangle 2, vertice 2
 *
 * @return true if triangles overlap not at their vertices, false otherwise
 */
bool class_constrained_delaunay_triangulation::triangles_overlap(Point_2 it0_p0, Point_2 it0_p1, Point_2 it0_p2,
		Point_2 it1_p0, Point_2 it1_p1, Point_2 it1_p2)
{

	CGAL::Point_2<K_exact> t0_p0(it0_p0.x(), it0_p0.y());
	CGAL::Point_2<K_exact> t0_p1(it0_p1.x(), it0_p1.y());
	CGAL::Point_2<K_exact> t0_p2(it0_p2.x(), it0_p2.y());

	CGAL::Point_2<K_exact> t1_p0(it1_p0.x(), it1_p0.y());
	CGAL::Point_2<K_exact> t1_p1(it1_p1.x(), it1_p1.y());
	CGAL::Point_2<K_exact> t1_p2(it1_p2.x(), it1_p2.y());

	CGAL::Triangle_2<K_exact>* t0;
	CGAL::Triangle_2<K_exact>* t1;


	//make sure the triangles are built in ccw orientation
	if ((CGAL::Triangle_2<K_exact>(t0_p0, t0_p1, t0_p2)).orientation()>0)
		t0 = new CGAL::Triangle_2<K_exact>(t0_p0, t0_p1, t0_p2);
	else
		t0 = new CGAL::Triangle_2<K_exact>(t0_p0, t0_p2, t0_p1);

	//make sure the triangles are built in ccw orientation
	if ((CGAL::Triangle_2<K_exact>(t1_p0, t1_p1, t1_p2)).orientation()>0)
		t1 = new CGAL::Triangle_2<K_exact>(t1_p0, t1_p1, t1_p2);
	else
		t1 = new CGAL::Triangle_2<K_exact>(t1_p0, t1_p2, t1_p1);

	//print the vertices of the created triangles
	//ROS_INFO("t0 p0 x=%f y=%f p1 x=%f y=%f p2 x=%f y=%f", CGAL::to_double(t0->vertex(0).x()), CGAL::to_double(t0->vertex(0).y()), CGAL::to_double(t0->vertex(1).x()), CGAL::to_double(t0->vertex(1).y()), CGAL::to_double(t0->vertex(2).x()), CGAL::to_double(t0->vertex(2).y()));
	//ROS_INFO("t1 p0 x=%f y=%f p1 x=%f y=%f p2 x=%f y=%f", CGAL::to_double(t1->vertex(0).x()), CGAL::to_double(t1->vertex(0).y()), CGAL::to_double(t1->vertex(1).x()), CGAL::to_double(t1->vertex(1).y()), CGAL::to_double(t1->vertex(2).x()), CGAL::to_double(t1->vertex(2).y()));

	//ROS_INFO("t0 is degenerate %d orientation %d",t0->is_degenerate(), t0->orientation());
	//ROS_INFO("t1 is degenerate %d orientation %d",t1->is_degenerate(), t1->orientation());
	CGAL::Object result;

	result = CGAL::intersection(*t0, *t1);

	delete t0;
	delete t1;
	if (const CGAL::Triangle_2<K_exact> *itriangle = CGAL::object_cast<CGAL::Triangle_2<K_exact> >(&result)) 
	{
		//if intersection is a triangle then the triangles overlap
		if(debug>1) ROS_WARN("intersection was a triangle");
		return true;
	} 
	else if (const std::vector<CGAL::Point_2<K_exact> > *iptvector = CGAL::object_cast<std::vector<CGAL::Point_2<K_exact> > >(&result))
	{
		//if intersection is vector of points, i.e., a polygon, then the triangles overlap
		if(debug>1) ROS_WARN("intersection was a pt vector");
		return true;
	}
	else
	{
		//if intersection is any of the other cases, i.e., segment or point, then there is only overlap in the boundaries 
		if(debug>1)	ROS_WARN("no_intr");
		return false;
	}

}

int class_constrained_delaunay_triangulation::printf_face_info(CDT::Face_handle fh)
{
	if (fh->is_valid())
	{
		ROS_INFO("Face (%ld,%ld,%ld)\nv0(index%ld x=%f y=%f)\nv1 (index%ld x=%f y=%f)\nv2 (index%ld x=%f y=%f)\nprovenience %d; weight %f", fh->vertex(0)->info().index, fh->vertex(2)->info().index, fh->vertex(2)->info().index,fh->vertex(0)->info().index, CGAL::to_double(fh->vertex(0)->point().x()), CGAL::to_double(fh->vertex(0)->point().y()), fh->vertex(1)->info().index, CGAL::to_double(fh->vertex(1)->point().x()), CGAL::to_double(fh->vertex(1)->point().y()), fh->vertex(2)->info().index, CGAL::to_double(fh->vertex(2)->point().x()), CGAL::to_double(fh->vertex(2)->point().y()), fh->provenience, fh->weight);	
	
	
	}
	else
	{
		ROS_WARN("Face handle is not valid");
	}

	return 1;
}

/**
 * @brief This method computes an initial list of faces to start the search from. It first sees if the average point of the tti is inside some face, if not it uses alternatives for all cases
 *
 * @param tti_v0 tti vertex 0
 * @param tti_v1 tti vertex 1
 * @param tti_v2 tti vertex 2
 * @param queue a pointer to the list of CDT::Face_hanle faces to start the search from
 *
 * @return 1 if success
 */
int class_constrained_delaunay_triangulation::get_seed_list_of_faces_for_tti(CDT::Face_handle fti, std::vector<CDT::Face_handle>* queue)
{
//static CDT::Face_handle fh_guess;
//static int start=1;

	queue->erase(queue->begin(), queue->end());

	
	CDT::Locate_type lt;
	
	int li;
	

	if(debug) printf_face_info(fti);

	Point_2 p((CGAL::to_double(fti->vertex(0)->point().x()) + CGAL::to_double(fti->vertex(1)->point().x()) + CGAL::to_double(fti->vertex(2)->point().x()))/3.,
				(CGAL::to_double(fti->vertex(0)->point().y()) + CGAL::to_double(fti->vertex(1)->point().y()) +CGAL::to_double(fti->vertex(2)->point().y()))/3.);
	
	CDT::Face_handle fh;
	
	//if (fh_guess->is_valid() && !start)
		//fh = dt.locate(p, lt, li, fh_guess); //locate the point on the mesh
	//else
		fh = dt.locate(p, lt, li); //locate the point on the mesh

	//start=0;
	//fh_guess = fh;	

	if (lt==CDT::OUTSIDE_AFFINE_HULL)
	{
		if(debug>1)ROS_INFO("seed point outside affine hull");
	}
	else if (lt==CDT::OUTSIDE_CONVEX_HULL)
	{
		queue->push_back(fh);			

	
		CDT::Face_handle fh_tti_v0 = dt.locate(fti->vertex(0)->point(), lt, li); 
		if(lt==CDT::FACE) queue->push_back(fh_tti_v0);			

	
		CDT::Face_handle fh_tti_v1 = dt.locate(fti->vertex(1)->point(), lt, li); 
		if(lt==CDT::FACE) queue->push_back(fh_tti_v1);			

	
		CDT::Face_handle fh_tti_v2 = dt.locate(fti->vertex(2)->point(), lt, li); 
		if(lt==CDT::FACE) queue->push_back(fh_tti_v2);			

	
		if(debug>1) ROS_INFO("seed point outside convex hull.Added faces on vertices. Queue size = %d",(int)queue->size());
	}
	if(lt==CDT::FACE)
	{
		queue->push_back(fh);			
		if(debug>1)	ROS_INFO("seed point on face (%ld %ld %ld)", fh->vertex(0)->info().index, fh->vertex(1)->info().index, fh->vertex(2)->info().index);
	}
	else if(lt==CDT::EDGE)
	{
	
		queue->push_back(fh);			
		queue->push_back(fh->neighbor(li));			
		if(debug>1) ROS_INFO("seed point in segment");
	}
	else if(lt==CDT::VERTEX)
	{
	
		CDT::Face_circulator fc = dt.incident_faces(fh->vertex(li)), done(fc); 
		if(fc!=0) 
		{ 
			do
			{ 
				queue->push_back(fc);			
			}
			while(++fc!=done); 
		} 

		if(debug>1)	ROS_INFO("seed point in vertex;");
	}
	else
	{
		if(debug)ROS_INFO("seed point not located no queue");
	}

	return 1;
}


/**
 * @brief Creates a CDT::Face from the all the input variables 
 *
 * @param x0 x of vertex 0
 * @param y0 y of vertex 0
 * @param z0 z of vertex 0
 * @param rgb0 rgb of vertex 0
 * @param x1 x of vertex 1
 * @param y1 y of vertex 1
 * @param z1 z of vertex 1
 * @param rgb1 rgb of vertex 1
 * @param x2 x of vertex 2
 * @param y2 y of vertex 2
 * @param z2 z of vertex 2
 * @param rgb2 rgb of vertex 2
 * @param face_weight the weight of the face
 * @param provenience the camera projection provenience
 *
 * @return the CDT::Face object
 */
CDT::Face class_constrained_delaunay_triangulation::face(double x0, double y0, double z0, float rgb0, 
		double x1, double y1, double z1, float rgb1, 
		double x2, double y2, double z2, float rgb2,
		float face_weight, int provenience)
{
	CDT::Vertex v0(Point_2(x0,y0));
	CDT::Vertex v1(Point_2(x1,y1));
	CDT::Vertex v2(Point_2(x2,y2));
	CDT::Vertex_handle vh0=&v0;
	CDT::Vertex_handle vh1=&v1;
	CDT::Vertex_handle vh2=&v2;

	CDT::Face f(vh0,vh1,vh2);
	f.weight = face_weight;
	f.provenience = provenience;
	return f;
}

int class_constrained_delaunay_triangulation::iterate_intersecting_faces(CDT::Face_handle fti,  int predicate_number)
{
	//find the initial list of faces to start searching from
	
	std::vector<CDT::Face_handle> queue;
	
	get_seed_list_of_faces_for_tti(fti, &queue);
	


	if(debug>1)ROS_INFO("Initial queue list has size %d" ,(int)queue.size());

	
	initialize_visited();
	

	for (size_t i=0; queue.size()>0; )
	{
		//if(debug>1)	ROS_INFO("new iterations Queue has %d faces i=%d",(int)queue.size(), (int)i);
		if (queue[i]->visited==false)
		{

	
			if(dt.is_infinite(queue[i]))//if is infinite do not test for intr but add all neightbors
			{
				//Propagation
				for (size_t k=0; k<3; ++k) 
				{
					CDT::Face_handle nfh = queue[i]->neighbor(k);	
					if (nfh->visited==false)
					{
						if(debug>1)	ROS_INFO("propagating to neighbour %ld",k);
						queue.push_back(nfh);			
					}
				}
			}
			else if (triangles_overlap(fti->vertex(0)->point(), fti->vertex(1)->point(), fti->vertex(2)->point(), queue[i]->vertex(0)->point(), queue[i]->vertex(1)->point(), queue[i]->vertex(2)->point()))
			{

				//NOTE the test for the predicate goes here
				if (predicate_number==0)
				{
					if(predicate_should_add_face(fti, queue[i])==false)
						return 0;
				}
				else if (predicate_number==1)
				{
					if(predicate_remove_vertex(fti, queue[i])==false)
						return 0;
				}
				else if (predicate_number==2)
				{
					if(predicate_remove_intersecting_constraints(fti, queue[i])==false)
						return 0;
				}

				//Propagation
				for (size_t k=0; k<3; ++k) 
				{
					CDT::Face_handle nfh = queue[i]->neighbor(k);	
					if (nfh->visited==false && !dt.is_infinite(nfh))
					{
						if(debug>1)	ROS_INFO("propagating to neighbour %ld",k);
						queue.push_back(nfh);			
					}
				}
			}
		}	

	
		queue[i]->visited=true;
		queue.erase(queue.begin());
	
	}

	//If return 1, all faces where visited and passed the predicate
	return 1;
}

/**
 * @brief 
 *
 * @param x0
 * @param y0
 * @param z0
 * @param rgb0
 * @param x1
 * @param y1
 * @param z1
 * @param rgb1
 * @param x2
 * @param y2
 * @param z2
 * @param rgb2
 * @param face_weight
 * @param provenience
 *
 * @return 
 */
int class_constrained_delaunay_triangulation::add_face_manager(double x0, double y0, double z0, float rgb0, 
		double x1, double y1, double z1, float rgb1, 
		double x2, double y2, double z2, float rgb2,
		float face_weight, int provenience)
{
	//Lets define some variables
	char str_seed_point[1024];

	//the triangle to insert will be called tti
	std::vector<Point_2> tti_vertex;
	tti_vertex.push_back(Point_2(x0,y0));
	tti_vertex.push_back(Point_2(x1,y1));
	tti_vertex.push_back(Point_2(x2,y2));


	vector_faces.erase(vector_faces.begin(),vector_faces.end());

	std::vector<CDT::Face_handle> queue;





	if(debug>0)
	{
		std::string str;
		char tmp[1024];
		str+="___get_intersecting_faces_list___\n";
		str+=str_seed_point;
		sprintf(tmp,"\nShould visit %d faces:\n", (int)vector_faces.size());
		str+=tmp;

		for (size_t i=0; i<vector_faces.size(); i++)
		{


			CDT::Face_handle fh = get_face_handle_from_t_face(&vector_faces[i]);
			if (fh==NULL)continue;


			sprintf(tmp,"Face(%ld,%ld,%ld), ", (get_face_handle_from_t_face(&vector_faces[i]))->vertex(0)->info().index, (get_face_handle_from_t_face(&vector_faces[i]))->vertex(1)->info().index, (get_face_handle_from_t_face(&vector_faces[i]))->vertex(2)->info().index);
			str+=tmp;
		}
		ROS_INFO("%s",str.c_str());
	}


	//Export visited faces to draw afterwards
	pc_faces_visited.erase(pc_faces_visited.begin(), pc_faces_visited.end());
	for (size_t i=0; i<vector_faces.size(); i++)
	{

		CDT::Face_handle fh = get_face_handle_from_t_face(&vector_faces[i]);
		if (fh==NULL)continue;


		pcl::PointXYZ pt;
		pt.x = (CGAL::to_double((get_face_handle_from_t_face(&vector_faces[i]))->vertex(0)->point().x()) +  CGAL::to_double((get_face_handle_from_t_face(&vector_faces[i]))->vertex(1)->point().x()) + CGAL::to_double((get_face_handle_from_t_face(&vector_faces[i]))->vertex(2)->point().x()))/3;
		pt.y = (CGAL::to_double((get_face_handle_from_t_face(&vector_faces[i]))->vertex(0)->point().y()) +  CGAL::to_double((get_face_handle_from_t_face(&vector_faces[i]))->vertex(1)->point().y()) + CGAL::to_double((get_face_handle_from_t_face(&vector_faces[i]))->vertex(2)->point().y()))/3;
		pt.z = (CGAL::to_double((get_face_handle_from_t_face(&vector_faces[i]))->vertex(0)->info().z) +  CGAL::to_double((get_face_handle_from_t_face(&vector_faces[i]))->vertex(1)->info().z) + CGAL::to_double((get_face_handle_from_t_face(&vector_faces[i]))->vertex(2)->info().z))/3;


		pc_faces_visited.points.push_back(pt);
	}
	pcl_ros::transformPointCloud(pc_faces_visited, pc_faces_visited, transform.inverse());     


	return 1;
}	

bool class_constrained_delaunay_triangulation::predicate_remove_vertex(CDT::Face_handle fti, CDT::Face_handle fh)
{
	for (size_t i=0; i<3; ++i)
	{
		if (overlaps(&fti->vertex(0)->point(), &fti->vertex(1)->point(), &fti->vertex(2)->point(), &fh->vertex(i)->point()))
		{
			test_if_triangulation_is_valid();

			if(dt.are_there_incident_constraints(fh->vertex(i)))
			{
				dt.remove_incident_constraints(fh->vertex(i));
				return 0;
			}

			test_if_triangulation_is_valid();
			dt.remove(fh->vertex(i));
			test_if_triangulation_is_valid();
			return 0;
		}
	}
	return 1;
}


bool class_constrained_delaunay_triangulation::predicate_remove_intersecting_constraints(CDT::Face_handle fti, CDT::Face_handle fh)
{

	if (debug) ROS_WARN("Testing intr with face(%ld,%ld,%ld)",fh->vertex(0)->info().index, fh->vertex(1)->info().index, fh->vertex(2)->info().index);


	for (int i=0; i<3; i++) //cycle all edges of fti triangle
	{
		int i1,i2;
		if (i==0) {i1=1; i2=2;}
		else if (i==1) {i1=0; i2=2;}
		else if (i==2) {i1=0; i2=1;}

		CGAL::Segment_2<K_exact> seg_fti(fti->vertex(i1)->point(),fti->vertex(i2)->point());

		for (int k=0; k<3; k++) //cycle all edges of fh triangle
		{
			int k1,k2;
			if (k==0) {k1=1; k2=2;}
			else if (k==1) {k1=0; k2=2;}
			else if (k==2) {k1=0; k2=1;}

			if (debug) ROS_INFO("testing intr with fh edge %ld - %ld",fh->vertex(k1)->info().index, fh->vertex(k2)->info().index);

			CGAL::Segment_2<K_exact> s(dt.segment(fh,k));

			if (segments_intersect_not_at_endpoints(seg_fti, s)) //test edge s with segment fti
			{
				if(debug)ROS_INFO("Found an intersection for seg%d%d and mesh edge mv%ld-mv%ld",i1,i2, fh->vertex(k1)->info().index, fh->vertex(k2)->info().index);
				if (dt.is_constrained(CDT::Edge(fh,k)))
				{

					dt.remove_constraint(fh,k);
					test_if_triangulation_is_valid();
					return 0;
				}
			}
		}
	}

	test_if_triangulation_is_valid();
	return 1;
}


#endif
/**
 *@}
 */      

