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
 *@{
 * @file
 * @brief This code perfomrs a constrained delaunay triangulation
 */
#ifndef _CONSTRAINED_DELAUNAY_TRIANGULATION_CPP_
#define _CONSTRAINED_DELAUNAY_TRIANGULATION_CPP_

#include "constrained_delaunay_triangulation.h"

int class_constrained_delaunay_triangulation::compute_union(void)
{
	//pi.compute_polygon_union(&projection_union;
	//printf("dp.projection_union.size()=%d\n",(int)projection_union.size());
	//for (size_t i=0; i<cp.size(); i++)
	//{

	//}

	return 1;

}


int class_constrained_delaunay_triangulation::clear_constraints(void)
{
	for (CDT::Vertex_iterator it=dt.vertices_begin(); it!=dt.vertices_end(); ++it)
	{
		dt.remove_incident_constraints(it); 
	}
	return 1;
}

int class_constrained_delaunay_triangulation::clear_vertices(void)
{
	dt.clear();
	index_count=1;
	return 1;
}

int class_constrained_delaunay_triangulation::set_constraint_polygon(void)
{
	//when the rest works, grab the code from previous versions
	return 1;
}

int class_constrained_delaunay_triangulation::set_transform(tf::Transform* st)
{
	transform.setRotation(st->getRotation());	
	transform.setOrigin(st->getOrigin());
	return 1;
}

int class_constrained_delaunay_triangulation::export_points_to_pc(void)
{
	//create a list of vertices and a list of indices
	pc_vertices_indices.erase(pc_vertices_indices.begin(), pc_vertices_indices.end());
	pc_vertices.erase(pc_vertices.points.begin(), pc_vertices.points.end());
	for (CDT::Finite_vertices_iterator vi=dt.finite_vertices_begin(); vi!=dt.finite_vertices_end(); ++vi)
	{
		pcl::PointXYZ pt;
		pt.x = CGAL::to_double(vi->point().x());
		pt.y = CGAL::to_double(vi->point().y());
		pt.z = CGAL::to_double(vi->info().z);
		pc_vertices.push_back(pt);	
		pc_vertices_indices.push_back(vi->info().index);
	}
	pcl_ros::transformPointCloud(pc_vertices, pc_vertices, transform.inverse());     

	//create a list of constraints
	pc_constraints.points.erase(pc_constraints.points.begin(), pc_constraints.points.end());
	pcl::PointCloud<pcl::PointXYZ> pc_local;
	pcl::PointXYZ pt;
	for(CDT::Edge_iterator ei = dt.finite_edges_begin(); ei != dt.finite_edges_end(); ++ei)
	{
		if (dt.is_constrained(*ei))	
		{
			CDT::Segment s= dt.segment(ei);	
			pt.x = CGAL::to_double((s.point(0)).x()); 
			pt.y = CGAL::to_double((s.point(0)).y());			
			pt.z = 0;
			pc_local.points.push_back(pt);

			pt.x = CGAL::to_double((s.point(1)).x()); 
			pt.y = CGAL::to_double((s.point(1)).y());			
			pt.z = 0;
			pc_local.points.push_back(pt);

		}
	}
	pcl_ros::transformPointCloud(pc_local, pc_constraints, transform.inverse());     
	//ROS_INFO("pc_constraints.size=%d mesh number of constraints %d", (int)pc_constraints.points.size(), (int)dt.number_of_constraints());

	//--------------------------------------------
	//Recompute the constraints 
	//--------------------------------------------

	//NOTE: must clear the constraint polygons. I think when the dt is cleared all constraints are also cleared
	//clear_constraints();

	//Use the new projection union to insert the new constraints
	//set_constraint_polygon();

	initialize_all_faces_status();
	discoverComponents(dt);

	pc.points.erase(pc.points.begin(), pc.points.end());
	pc_proveniences.erase(pc_proveniences.begin(), pc_proveniences.end());

	ros::Time t1=ros::Time::now();

	unsigned int num_triangles=0;
	for(Finite_faces_iterator fc1 = dt.finite_faces_begin(); fc1 != dt.finite_faces_end(); ++fc1)
	{

		//printf("Triangle is in domain =%d status=%d\n",fc1->is_in_domain(), fc1->status);
		if (!fc1->is_in_domain())
		{
			//printf("face %d is in domain = %d\n",num_triangles,  fc1->is_in_domain());
			//continue;
		}

		if (fc1->vertex(0)->info().rgb==-1)
		{
			//continue;
		}

		if (fc1->vertex(1)->info().rgb==-1)
		{
			//continue;
		}

		if (fc1->vertex(2)->info().rgb==-1)
		{
			//continue;
		}

		for (int u=0; u<3;u++)
		{
			pcl::PointCloud<pcl::PointXYZ> pc_local;
			pcl::PointXYZ pt;

			pt.x = CGAL::to_double(fc1->vertex(u)->point().x()); 
			pt.y = CGAL::to_double(fc1->vertex(u)->point().y()); 
			pt.z = CGAL::to_double(fc1->vertex(u)->info().z); 

			pc_local.points.push_back(pt);

			pcl::PointCloud<pcl::PointXYZ> pc_global;
			pcl_ros::transformPointCloud(pc_local, pc_global, transform.inverse());     
			pcl::PointXYZRGB ptcolor;
			ptcolor.x = pc_global.points[0].x;
			ptcolor.y = pc_global.points[0].y;
			ptcolor.z = pc_global.points[0].z;
			ptcolor.rgb = fc1->vertex(u)->info().rgb;

			pc.points.push_back(ptcolor);
			pc_proveniences.push_back(fc1->provenience);

			pc.height = 1;
			pc.width = pc.points.size();
			pc.is_dense = 0;

		}
		num_triangles++;
	}

	//ros::Duration d;
	//d = ros::Time::now()-t1;
	//ROS_INFO("Exporting mesh took %f seconds\nExported %d triangles",d.toSec(), (int)pc.points.size());

	return 1;
}


void class_constrained_delaunay_triangulation::discoverComponent(const CDT & ct,
		Face_handle start, 
		int index, 
		std::list<CDT::Edge>& border )
{
	if(start->counter != -1)
	{
		return;
	}
	std::list<Face_handle> queue;
	queue.push_back(start);

	while(! queue.empty())
	{
		Face_handle fh = queue.front();
		queue.pop_front();
		if(fh->counter == -1)
		{
			fh->counter = index;
			fh->set_in_domain(index%2 == 1);
			for(int i = 0; i < 3; i++)
			{
				CDT::Edge e(fh,i);
				Face_handle n = fh->neighbor(i);
				if(n->counter == -1)
				{
					if(ct.is_constrained(e))
					{
						border.push_back(e);
					}
					else 
					{
						queue.push_back(n);
					}
				}

			}
		}
	}
}

void class_constrained_delaunay_triangulation::discoverComponents(const CDT & ct)
{
	int index = 0;
	std::list<CDT::Edge> border;
	discoverComponent(ct, ct.infinite_face(), index++, border);
	while(! border.empty())
	{
		CDT::Edge e = border.front();
		border.pop_front();
		Face_handle n = e.first->neighbor(e.second);
		if(n->counter == -1)
		{
			discoverComponent(ct, n, e.first->counter+1, border);
		}
	}
} 


int class_constrained_delaunay_triangulation::project_triangulation_to_new_plane(pcl::ModelCoefficients::Ptr plane, tf::Transform tf)
{
	pcl::PointCloud<pcl::PointXYZ> pc_local;
	pcl::PointCloud<pcl::PointXYZ> pc_global;
	pcl::PointCloud<pcl::PointXYZ> pc_global_projected;
	pcl::PointCloud<pcl::PointXYZ> pc_local_projected;

	//build pc_local from the triangulation data structure
	for(Finite_vertices_iterator fv = dt.finite_vertices_begin(); fv != dt.finite_vertices_end(); ++fv)
	{
		pcl::PointXYZ p;
		p.x = CGAL::to_double(fv->point().x()); 
		p.y = CGAL::to_double(fv->point().y()); 
		p.z = CGAL::to_double(fv->info().z); 
		pc_local.points.push_back(p);
	}


	//transform the pc to local coordinates
	pcl_ros::transformPointCloud(pc_local,pc_global, transform.inverse()); 

	pcl::ProjectInliers<pcl::PointXYZ> projection;
	projection.setModelType(pcl::SACMODEL_PLANE); //set model type

	projection.setInputCloud(pc_global.makeShared());
	projection.setModelCoefficients(plane);
	projection.filter(pc_global_projected);

	//transform the projected pc to the new local coordinates
	pcl_ros::transformPointCloud(pc_global_projected, pc_local_projected, tf); 

	size_t count=0;
	//build pc_local from the triangulation data structure
	for(Finite_vertices_iterator fv = dt.finite_vertices_begin(); fv != dt.finite_vertices_end(); ++fv)
	{
		Point_2 p(pc_local_projected.points[count].x, pc_local_projected.points[count].y);      
		//dt.move(fv, p); 
		//dt.move_if_no_collision(); 
		//dt.remove(fv); 
		count++;
	}

	return 1;
}

CDT::Face_handle class_constrained_delaunay_triangulation::get_face_handle_from_t_face(t_face* f)
{
	CDT::Face_handle fh;

	CDT::Vertex_handle v0 = get_vertex_at(f->v0);
	CDT::Vertex_handle v1 = get_vertex_at(f->v1);
	CDT::Vertex_handle v2 = get_vertex_at(f->v2);

	if(v0==NULL || v1==NULL || v2==NULL) return NULL;

	if(dt.is_face(v0,v1,v2 , fh))
	{
		return fh;
	}
	else
	{
		ROS_ERROR("Something very wrong this is not a face");
		return NULL;
	}
}

CDT::Vertex_handle class_constrained_delaunay_triangulation::get_vertex_at(Point_2 p)
{
	CDT::Locate_type lt; int li; 

	CDT::Face_handle fh = dt.locate(p, lt, li); 
	if (lt==CDT::VERTEX) 
	{
		if(debug>2) ROS_INFO("Found vertex index=%ld at x=%f y=%f",fh->vertex(li)->info().index, CGAL::to_double(p.x()), CGAL::to_double(p.y()));
		return fh->vertex(li);
	}
	else
	{
		ROS_ERROR("There is no vertex at x=%f y=%f", CGAL::to_double(p.x()), CGAL::to_double(p.y()));
		return NULL;
	}
}	


int class_constrained_delaunay_triangulation::add_face_to_mesh(CDT::Face_handle fti)
{
	size_t index[3];
	for (int u=0; u<3;u++)//check what are the vertices to insert in the current triangulation
	{
		CDT::Locate_type lt; int li;
		CDT::Face_handle fh = dt.locate(fti->vertex(u)->point(), lt,li); 
		if (lt==CDT::VERTEX) 
		{	
			if(debug)printf("v%d is on vertex %ld\n",u, fh->vertex(li)->info().index);
			index[u] = fh->vertex(li)->info().index;
		}
		else if (lt==CDT::EDGE)
		{
			if(debug)	printf("v%d is on edge\n",u);
			index[u] = index_count++;
		}
		else if (lt==CDT::FACE)
		{
			if(debug)	printf("v%d is on face(%ld,%ld,%ld)\n",u, fh->vertex(0)->info().index, fh->vertex(1)->info().index, fh->vertex(2)->info().index);
			index[u] = index_count++;
		}
		else
		{
			if(debug)printf("v%d other\n",u);
			index[u] = index_count++;
		}
	}

	//insert the constraints
	for (int k=0; k<3; k++) //cycle all edges of fh triangle
	{
		int k1,k2;
		if (k==0) {k1=1; k2=2;}
		else if (k==1) {k1=0; k2=2;}
		else if (k==2) {k1=0; k2=1;}

		if(debug)ROS_INFO("inserting constraint v%d to v%d", k1,k2);
		dt.insert_constraint(fti->vertex(k1)->point(), fti->vertex(k2)->point());
	}

	CDT::Vertex_handle fh_v[3];
	for (int u=0; u<3;u++)//add the correct properties to the vertices
	{
		CDT::Locate_type lt; int li;
		CDT::Face_handle fh = dt.locate(fti->vertex(u)->point(), lt, li); 
		if (lt==CDT::VERTEX) 
		{	
			fh_v[u] = fh->vertex(li);
			fh_v[u]->info().index = index[u];
			fh_v[u]->info().rgb = fti->vertex(u)->info().rgb;
			fh_v[u]->info().z = fti->vertex(u)->info().z;
		}
		else
		{
			if(debug)ROS_ERROR("There is a error, should be a vertex and its not");
		}
	}


	//Check if face was created
	CDT::Face_handle fh;
	if(!dt.is_face(fh_v[0], fh_v[1],fh_v[2], fh))
	{
		ROS_ERROR("could not add explicit face");
	}
	else
	{
		fh->provenience = fti->provenience;
		fh->weight = fti->weight;	
	}

	test_if_triangulation_is_valid();
	return 1;
}


int class_constrained_delaunay_triangulation::add_face_to_mesh(double x0, double y0, double z0, float rgb0, 
		double x1, double y1, double z1, float rgb1, 
		double x2, double y2, double z2, float rgb2,
		float weight, int provenience)
{
	if(debug)ROS_WARN("Starting add_face_to_mesh");

	double x[3] = {x0, x1, x2};
	double y[3] = {y0, y1, y2};
	double z[3] = {z0, z1, z2};
	float rgb[3] = {rgb0,rgb1,rgb2};

	CDT::Locate_type lt[3];
	int li[3];
	CDT::Face_handle fh_v[3];
	size_t index[3];

	for (int u=0; u<3;u++)//check what are the vertices to insert in the current triangulation
	{
		fh_v[u] = dt.locate(Point_2(x[u], y[u]), lt[u], li[u]); 
		if (lt[u]==CDT::VERTEX) 
		{	
			if(debug)printf("v%d is on vertex %ld\n",u, fh_v[u]->vertex(li[u])->info().index);
			index[u] = fh_v[u]->vertex(li[u])->info().index;
		}
		else if (lt[u]==CDT::EDGE)
		{
			if(debug)	printf("v%d is on edge\n",u);
			index[u] = index_count++;
		}
		else if (lt[u]==CDT::FACE)
		{
			if(debug)	printf("v%d is on face(%ld,%ld,%ld)\n",u, fh_v[u]->vertex(0)->info().index, fh_v[u]->vertex(1)->info().index, fh_v[u]->vertex(2)->info().index);
			index[u] = index_count++;
		}
		else
		{
			if(debug)printf("v%d other\n",u);
			index[u] = index_count++;
		}
	}


	//insert the constraints
	//if(debug)ROS_INFO("inserting constraint v0 (%f; %f) to v1(%f; %f)", x[0], y[0], x[1], y[1]);
	//dt.insert_constraint(Point_2(x[0], y[0]), Point_2(x[1], y[1]));

	//if(debug)ROS_INFO("inserting constraint v1 (%f; %f) to v2(%f; %f)", x[1], y[1], x[2], y[2]);
	//dt.insert_constraint(Point_2(x[1], y[1]), Point_2(x[2], y[2]));

	//if(debug)ROS_INFO("inserting constraint v1 (%f; %f) to v2(%f; %f)", x[0], y[0], x[2], y[2]);
	//dt.insert_constraint(Point_2(x[0], y[0]), Point_2(x[2], y[2]));


	//Add vertices that are on faces, edges, etc
	for (int u=0; u<3;u++)
	{
		if (lt[u]!=CDT::VERTEX) 
		{	
			dt.insert(Point_2(x[u],y[u]));                  
			if(debug)ROS_INFO("Adding new vertex v%d",u);
		}
	}

	//relocate all vertices and set the info values straight
	Vertex_handle1 vh[3];
	for (int u=0; u<3;u++)
	{
		vh[u] = get_vertex_at(Point_2(x[u], y[u]));
		vh[u]->info().rgb = rgb[u];
		vh[u]->info().z = z[u];
		vh[u]->info().index = index[u];
	}

	if(debug)
	{
		for (int u=0; u<3;u++)
		{
			printf("v%d has index %ld\n",u, vh[u]->info().index);
		}
	}

	//insert the constraints
	if(debug)ROS_INFO("inserting constraint v0 (%f; %f) to v1(%f; %f)", CGAL::to_double(vh[0]->point().x()), CGAL::to_double(vh[0]->point().y()) , CGAL::to_double(vh[1]->point().x()) , CGAL::to_double(vh[1]->point().y()));
	dt.insert_constraint(get_vertex_at(Point_2(x[0], y[0])), get_vertex_at(Point_2(x[1], y[1])));

	if(debug)ROS_INFO("inserting constraint v1 (%f; %f) to v2(%f; %f)", x[1], y[1], x[2], y[2]);
	dt.insert_constraint(get_vertex_at(Point_2(x[1], y[1])), get_vertex_at(Point_2(x[2], y[2])));

	if(debug)ROS_INFO("inserting constraint v0 (%f; %f) to v2(%f; %f)", x[0], y[0], x[2], y[2]);
	dt.insert_constraint(get_vertex_at(Point_2(x[0], y[0])), get_vertex_at(Point_2(x[2], y[2])));


	//insert the vertices
	//Vertex_handle1 vh[3];
	//for (int i=0; i<3; i++)
	//{
	//if(debug)ROS_INFO("inserting vertex %d", i);
	//if (lt[i]!=CDT::VERTEX) //if v[i] is not already a vertex, then we must insert one
	//{
	//if(debug)ROS_INFO("Its not a vertex in the current triangulation");
	//vh[i] = dt.insert(Point_2(x[i],y[i]));                  
	//vh[i]->info().z = z[i];
	//vh[i]->info().rgb = rgb[i];
	//vh[i]->info().index = index_count++;
	//}
	//else //if v[i] is already a vertex we just update rgb and z?
	//{
	//if(debug)ROS_INFO("Its already a vertex in the current triangulation");
	//vh[i] = fh_v[i]->vertex(li[i]);
	//
	//fh_v[i]->vertex(li[i])->info().z = z[i];	
	//
	//fh_v[i]->vertex(li[i])->info().rgb = rgb[i];	
	//
	//}
	//}


	//Check if face was created
	CDT::Face_handle fh;
	if(!dt.is_face(get_vertex_at(Point_2(x[0], y[0])), get_vertex_at(Point_2(x[1], y[1])),get_vertex_at(Point_2(x[2], y[2])), fh))
	{
		ROS_ERROR("could not add explicit face");
	}
	else
	{
		fh->provenience = provenience;
		fh->weight = weight;	


		//for (int i=0;i<3;i++)
		//dt.remove_constraint(fh,i);

		//dt.insert_constraint();
		//for (int i=0;i<3;i++)
		//{
		////Find the neighbor edge of edge li
		//int k1,k2;
		//size_t id_k1, id_k2;
		//if (i==0) {k1=1; k2=2;}
		//else if (i==1) {k1=0; k2=2;}
		//else {k1=0; k2=1;}

		//id_k1 = fh->vertex(k1)->info().index;
		//id_k2 = fh->vertex(k2)->info().index;

		//CDT::Face_handle nf = fh->neighbor(i);
		//int nk1, nk2;
		//int u=-1;
		//for (u=0; u<3;u++)
		//{
		//if (u==0) {nk1=1; nk2=2;}
		//else if (u==1) {nk1=0; nk2=2;}
		//else {nk1=0; nk2=1;}

		//if ((nf->vertex(nk1)->info().index == id_k1 && nf->vertex(nk2)->info().index == id_k2) ||
		//(nf->vertex(nk1)->info().index == id_k2 && nf->vertex(nk2)->info().index == id_k1))
		//{
		//if (debug) ROS_INFO("Found matching edge on neighbor");
		//break;	
		//}
		//}

		//if (u==-1)
		//{
		//ROS_ERROR("Something wrong, could not find matching edge on neighbor");
		//}

		//fh->set_constraint(i,true);
		//nf->set_constraint(u,true);
		//}

	}

	test_if_triangulation_is_valid();
	return 1;
}







int class_constrained_delaunay_triangulation::remove_constraint(CDT::Face_handle fh, int li)
{


	//Get the vertexes for this constraint
	//CDT::Vertex_handle v1,v2;

	//if (li==0)
	//{
	//v1 = fh->vertex(1);
	//v2 = fh->vertex(2);
	//}
	//else if (li==1)
	//{
	//v1 = fh->vertex(0);
	//v2 = fh->vertex(2);
	//}
	//else if (li==2)
	//{
	//v1 = fh->vertex(0);
	//v2 = fh->vertex(1);
	//}
	//else
	//{
	//ROS_ERROR("remove_constraints error: li=%d value not possible", li);
	//return 0;
	//}


	//if(debug)ROS_INFO("first method says is_constrained=%d",dt.is_constrained(CDT::Edge(fh,li)));

	//if(fh->is_constrained(li) [>||  (fh->neighbor(li))->is_constrained(li)<])
	//{
	//if(debug)ROS_INFO("is_constrained=%d fh->is_valid()=%d n_vertices=%d n_faces=%d", fh->is_constrained(li), fh->is_valid(), (int)dt.number_of_vertices(), (int)dt.number_of_faces());
	//size_t bef = dt.number_of_constraints();
	//ROS_INFO("Edge li=%d is constrained, (%d total mesh constraints) removing constraint", li,(int)dt.number_of_constraints());


	//CDT::Context cont = dt.context( v1,v2);

	//CDT::Context_iterator cont_it = dt.contexts_begin(v1,v2);

	//int count=0;

	//for(; cont_it!=dt.contexts_end(v1,v2); ++cont_it)
	//{
	//count++;
	//}	
	//
	//ROS_INFO("There are %d contexts in this constraint", count);
	//

	//if (cont==NULL)
	//ROS_ERROR("Context is null");


	//ROS_INFO("number_of_enclosing_constraints=%d",(int)dt.number_of_enclosing_constraints ( fh->vertex(1), fh->vertex(2)));

	//dt.remove_constraint (v1,v2);

	//ROS_INFO("number_of_enclosing_constraints=%d",(int)dt.number_of_enclosing_constraints ( fh->vertex(1), fh->vertex(2)));

	//if(debug)ROS_INFO("RC BC test indices 0 - %d 1 - %d 2 - %d",(int)fh->vertex(0)->info().index, (int)fh->vertex(1)->info().index, (int)fh->vertex(2)->info().index );

	//Find the neighbor edge of edge li
	int k1,k2;
	size_t id_k1, id_k2;
	if (li==0) {k1=1; k2=2;}
	else if (li==1) {k1=0; k2=2;}
	else {k1=0; k2=1;}

	id_k1 = fh->vertex(k1)->info().index;
	id_k2 = fh->vertex(k2)->info().index;

	CDT::Face_handle nf = fh->neighbor(li);
	int nk1, nk2;
	int index_u=-1;
	for (int u=0; u<3;u++)
	{
		if (u==0) {nk1=1; nk2=2;}
		else if (u==1) {nk1=0; nk2=2;}
		else {nk1=0; nk2=1;}

		if ((nf->vertex(nk1)->info().index == id_k1 && nf->vertex(nk2)->info().index == id_k2) ||
				(nf->vertex(nk1)->info().index == id_k2 && nf->vertex(nk2)->info().index == id_k1))
		{
			index_u=u;
			break;	
		}
	}

	if(index_u==-1)
	{
		if (debug) ROS_INFO("Something wrong,  index_u =%d",index_u);
	}

	if(debug>1)ROS_INFO("Removing constraint from face(%ld,%ld,%ld) edge(%ld,%ld) and neighbor face(%ld,%ld,%ld) edge(%ld,%ld)",fh->vertex(0)->info().index, fh->vertex(1)->info().index,fh->vertex(2)->info().index,fh->vertex(k1)->info().index, fh->vertex(k2)->info().index, nf->vertex(0)->info().index, nf->vertex(1)->info().index,nf->vertex(2)->info().index,nf->vertex(nk1)->info().index, nf->vertex(nk2)->info().index);


	if(debug>1)ROS_INFO("fh has index %ld nf has index %ld",fh->vertex(li)->info().index, nf->vertex(index_u)->info().index);
	if(fh->is_constrained(li))
	{
		if(debug>1)ROS_INFO("fh is constrained");
		fh->set_constraint(li,false);
		if(debug>1)ROS_INFO("now fh is constrained=%d ",fh->is_constrained(li));
	}

	if(debug>1)ROS_INFO("fh has index %ld nf has index %ld",fh->vertex(li)->info().index, nf->vertex(index_u)->info().index);
	if(nf->is_constrained(index_u))
	{
		if(debug>1)ROS_INFO("nf is constrained");
		nf->set_constraint(index_u,false);
		if(debug>1)ROS_INFO("now nf is constrained=%d ",nf->is_constrained(index_u));
	}

	if(debug>1)ROS_INFO("fh has index %ld nf has index %ld",fh->vertex(li)->info().index, nf->vertex(index_u)->info().index);

	if(debug>1)ROS_INFO("fh->is_valid=%d",fh->is_valid(true));
	if(debug>1)ROS_INFO("nf->is_valid=%d",nf->is_valid(true));


	for (int u=0; u<3;u++)
	{
		if(debug>1)ROS_INFO("fh->vertex(%d)->is_valid=%d",u, fh->vertex(u)->is_valid(true));
		if(debug>1)ROS_INFO("nf->vertex(%d)->is_valid=%d",u, nf->vertex(u)->is_valid(true));
	}
	//if(debug)ROS_INFO("face fh edge v%d=%d - v%d=%d", k1,(int)fh->vertex(k1)->info().index, k2,(int)fh->vertex(k2)->info().index);
	//if(debug)ROS_INFO("face nf edge v%d=%d - v%d=%d",nk1, (int)nf->vertex(nk1)->info().index,nk2, (int)nf->vertex(nk2)->info().index);

	//dt.remove_constraint(fh,li);
	//if(debug)ROS_INFO("RC AC test indices 0 - %d 1 - %d 2 - %d",(int)fh->vertex(0)->info().index, (int)fh->vertex(1)->info().index, (int)fh->vertex(2)->info().index );



	//ROS_INFO("Now %d total mesh constraints.", (int)dt.number_of_constraints());

	//if(debug)ROS_INFO("AFTERREMOVE is_constrained=%d fh->is_valid()=%d n_vertices=%d n_faces=%d", fh->is_constrained(li), fh->is_valid(), (int)dt.number_of_vertices(), (int)dt.number_of_faces());


	test_if_triangulation_is_valid();
	//if (bef == dt.number_of_constraints())
	//{
	//ROS_ERROR("Could not remove constraint");
	////CDT::Face_handle nf = fh->neighbor(li);

	//////ROS_INFO("nf->is_constrained(0)=%d",nf->is_constrained(0));
	//////ROS_INFO("nf->is_constrained(1)=%d",nf->is_constrained(1));
	//////ROS_INFO("nf->is_constrained(2)=%d",nf->is_constrained(2));

	//////nf->set_constraint(li,false);

	////ROS_INFO("Now %d total mesh constraints", (int)dt.number_of_constraints());
	//}


	//ROS_INFO("dt is valid %d", dt.is_valid());
	//}
	//else
	//{
	//if(debug)ROS_INFO("Not Constrained");	

	//}

	//ROS_INFO("face edge is constrained %d", fh->is_constrained(li));
	//ROS_INFO("dt is valid %d", dt.is_valid());

	return 1;
}


int class_constrained_delaunay_triangulation::get_intersecting_faces_list(double x0, double y0, double z0, float rgb0, 
		double x1, double y1, double z1, float rgb1, 
		double x2, double y2, double z2, float rgb2,
		float face_weight, int provenience)
{
	char str_seed_point[1024];

	Point_2 p0(x0,y0);
	Point_2 p1(x1,y1);
	Point_2 p2(x2,y2);
	Point_2 p_seed((x0+x1+x2)/3., (y0+y1+y2)/3.);

	vector_faces.erase(vector_faces.begin(),vector_faces.end());

	std::vector<boost::shared_ptr<CDT::Face_handle> > queue;

	CDT::Locate_type lt;
	int li;
	CDT::Face_handle 

		fh = dt.locate(p_seed, lt, li); //locate the point on the mesh

	if (lt==CDT::OUTSIDE_AFFINE_HULL)
	{
		if(debug){sprintf(str_seed_point,"seed point outside affine hull");}
		if(debug>1)ROS_INFO("seed point outside affine hull");
	}
	else if (lt==CDT::OUTSIDE_CONVEX_HULL)
	{
		boost::shared_ptr<CDT::Face_handle> bp_fh(new CDT::Face_handle(fh));
		queue.push_back(bp_fh);			

		fh = dt.locate(p0, lt, li); //locate the point on the mesh
		if(lt==CDT::FACE) 
		{
			boost::shared_ptr<CDT::Face_handle> bp_fh(new CDT::Face_handle(fh));
			queue.push_back(bp_fh);			
		}

		fh = dt.locate(p1, lt, li); //locate the point on the mesh
		if(lt==CDT::FACE) 
		{
			boost::shared_ptr<CDT::Face_handle> bp_fh(new CDT::Face_handle(fh));
			queue.push_back(bp_fh);			
		}

		fh = dt.locate(p2, lt, li); //locate the point on the mesh
		if(lt==CDT::FACE) 
		{
			boost::shared_ptr<CDT::Face_handle> bp_fh(new CDT::Face_handle(fh));
			queue.push_back(bp_fh);			
		}


		if(debug){sprintf(str_seed_point,"seed point outside convex hull. Added faces on vertices. Queue size = %d",(int)queue.size());}
		if(debug>1) ROS_INFO("seed point outside convex hull.Added faces on vertices. Queue size = %d",(int)queue.size());
	}
	if(lt==CDT::FACE)
	{
		boost::shared_ptr<CDT::Face_handle> bp_fh(new CDT::Face_handle(fh));
		queue.push_back(bp_fh);			
		if(debug){sprintf(str_seed_point,"seed point on face (%ld %ld %ld)", fh->vertex(0)->info().index, fh->vertex(1)->info().index, fh->vertex(2)->info().index);}
		if(debug>1)	ROS_INFO("seed point on face (%ld %ld %ld)", fh->vertex(0)->info().index, fh->vertex(1)->info().index, fh->vertex(2)->info().index);
	}
	else if(lt==CDT::EDGE)
	{
		boost::shared_ptr<CDT::Face_handle> bp_fh(new CDT::Face_handle(fh));
		queue.push_back(bp_fh);			
		boost::shared_ptr<CDT::Face_handle> bp_nf(new CDT::Face_handle(fh->neighbor(li)));
		queue.push_back(bp_nf);			
		if(debug){sprintf(str_seed_point,"seed point in segment");}
		if(debug>1) ROS_INFO("seed point in segment");
	}
	else if(lt==CDT::VERTEX)
	{
		CDT::Face_circulator fc = dt.incident_faces(fh->vertex(li)), done(fc); 
		if(fc!=0) 
		{ 
			do
			{ 
				boost::shared_ptr<CDT::Face_handle> bp_fh(new CDT::Face_handle(fc));
				queue.push_back(bp_fh);			
			}
			while(++fc!=done); 
		} 
		//boost::shared_ptr<CDT::Face_handle> bp_fh(new CDT::Face_handle(fh));
		//queue.push_back(bp_fh);			
		//boost::shared_ptr<CDT::Face_handle> bp_nf(new CDT::Face_handle(fh->neighbor(li)));
		//queue.push_back(bp_nf);			

		if(debug){sprintf(str_seed_point,"seed point in vertex");}
		if(debug>1)	ROS_INFO("seed point in vertex;");
	}
	else
	{
		if(debug){sprintf(str_seed_point,"seed point not located no queue");}
		if(debug)ROS_INFO("seed point not located no queue");
	}




	if(debug>1)ROS_INFO("sterting queue search %d" ,(int)queue.size());
	initialize_visited();
	for (size_t i=0; i<queue.size(); i++)
	{
		bool reset=false;

		if(debug>1)	ROS_INFO("new iterations Queue has %d faces i=%d",(int)queue.size(), (int)i);
		if ((*queue[i])->visited==false)
		{

			//if(debug>1) ROS_INFO("queue[%d] face(%ld %ld %ld) was not visited is_infinite=%d",(int)i , fh->vertex(0)->info().index, fh->vertex(1)->info().index, fh->vertex(2)->info().index, dt.is_infinite(queue[i]));

			if(dt.is_infinite((*queue[i])))//if is infinite do not test for intr but add all neightbors
			{
				if(debug>1)ROS_INFO("queue[%d] is infinite. propagating to neighbours",(int)i);
				//add neighbours to queue list

				CDT::Face_handle nfh;

				nfh	= (*queue[i])->neighbor(0);	
				if (nfh->visited==false)
				{
					if(debug>1)				ROS_INFO("propagating to neighbour 0");
					boost::shared_ptr<CDT::Face_handle> bp_fh(new CDT::Face_handle(nfh));
					queue.push_back(bp_fh);			
				}

				nfh	= (*queue[i])->neighbor(1);	
				if (nfh->visited==false) 
				{
					if(debug>1)	ROS_INFO("propagating to neighbour 1");
					boost::shared_ptr<CDT::Face_handle> bp_fh(new CDT::Face_handle(nfh));
					queue.push_back(bp_fh);			
				}

				nfh	= (*queue[i])->neighbor(2);	
				if (nfh->visited==false) 
				{
					if(debug>1)				ROS_INFO("propagating to neighbour 2");
					boost::shared_ptr<CDT::Face_handle> bp_fh(new CDT::Face_handle(nfh));
					queue.push_back(bp_fh);			
				}
				(*queue[i])->visited=true;
				//vector_faces.push_back(queue[i]);	
				queue.erase(queue.begin()+i);

			}
			else if (triangles_overlap(p0,p1,p2, (*queue[i])->vertex(0)->point(), (*queue[i])->vertex(1)->point(), (*queue[i])->vertex(2)->point()))
			{
				if(debug>1)ROS_INFO("queue[%d] overlaps with triangle propagating to neighbours",(int)i);
				//add neighbours to queue list

				CDT::Face_handle nfh;

				nfh	= (*queue[i])->neighbor(0);	
				if (nfh->visited==false && !dt.is_infinite(nfh))
				{
					if(debug>1)				ROS_INFO("propagating to neighbour 0");
					boost::shared_ptr<CDT::Face_handle> bp_fh(new CDT::Face_handle(nfh));
					queue.push_back(bp_fh);			
				}

				nfh	= (*queue[i])->neighbor(1);	
				if (nfh->visited==false && !dt.is_infinite(nfh)) 
				{
					if(debug>1)	ROS_INFO("propagating to neighbour 1");
					boost::shared_ptr<CDT::Face_handle> bp_fh(new CDT::Face_handle(nfh));
					queue.push_back(bp_fh);			
				}

				nfh	= (*queue[i])->neighbor(2);	
				if (nfh->visited==false && !dt.is_infinite(nfh)) 
				{
					if(debug>1)				ROS_INFO("propagating to neighbour 2");
					boost::shared_ptr<CDT::Face_handle> bp_fh(new CDT::Face_handle(nfh));
					queue.push_back(bp_fh);			
				}
				(*queue[i])->visited=true;
				t_face face;
				face.v0 = (*queue[i])->vertex(0)->point();
				face.v1 = (*queue[i])->vertex(1)->point();
				face.v2 = (*queue[i])->vertex(2)->point();
				vector_faces.push_back(face);	
				queue.erase(queue.begin()+i);
			}
			else
			{

				if(debug>1)ROS_INFO("queue[%d] does not overlap with triangle",(int)i);
				(*queue[i])->visited=true;
				//vector_faces.push_back(queue[i]);	
				queue.erase(queue.begin()+i);
			}
		}	
		else
		{
			if(debug>1) ROS_INFO("queue[%d] was visited",(int)i);
			queue.erase(queue.begin()+i);
		}

		reset=true;
		if (reset)i=-1;
	}


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




bool class_constrained_delaunay_triangulation::check_if_face_should_be_inserted(double x0, double y0, double z0, float rgb0, 
		double x1, double y1, double z1, float rgb1, 
		double x2, double y2, double z2, float rgb2,
		float face_weight, int provenience)
{
	//if (dt.number_of_vertices()<2) //no need to do any checks if mesh has no triangles
	//{
	//return true;
	//}

	//// -----------------------------------------------------------------------------------
	////Check if any of the intersecting faces has larger weight than the one we are adding. If so, exit without adding
	//// -----------------------------------------------------------------------------------
	//if(debug>1)ROS_INFO("vector_faces has size =%d",(int)vector_faces.size());

	//for (size_t i=0; i<vector_faces.size(); ++i)
	//{
	//CDT::Face_handle tmp = get_face_handle_from_t_face(&vector_faces[i]);
	//if (tmp==NULL) continue;

	//if ((get_face_handle_from_t_face(&vector_faces[i]))->weight > face_weight && (get_face_handle_from_t_face(&vector_faces[i]))->provenience != provenience)
	//{
	//if(debug>0)ROS_INFO("Triangle to add overlaps faces with larger weight, not adding");
	//return false;
	//}	
	//}

	//if(debug>0)
	//{
	//ROS_INFO("___check_if_face_should_be_inserted___\nTriangle to insert overweights all %ld faces it intersects, should insert", vector_faces.size());
	//}

	return true;
}

int class_constrained_delaunay_triangulation::remove_vertices_inside_triangle(double x0, double y0, double z0, float rgb0, 
		double x1, double y1, double z1, float rgb1, 
		double x2, double y2, double z2, float rgb2,
		float face_weight, int provenience)
{

	//std::vector<boost::shared_ptr<CDT::Vertex_handle> > vector_vertices;
	//Point_2 p0(x0,y0);
	//Point_2 p1(x1,y1);
	//Point_2 p2(x2,y2);

	//// -----------------------------------------------------------------------------------
	////Check if any of the  visited faces' vertices is inside the triangle to be inserted
	//// -----------------------------------------------------------------------------------
	//for (size_t i=0; i<vector_faces.size(); ++i) //Get a list of vertices from the list of faces
	//for (int k=0; k<3; k++)
	//{
	//CDT::Face_handle tmp = get_face_handle_from_t_face(&vector_faces[i]);
	//if (tmp==NULL)continue;

	//boost::shared_ptr<CDT::Vertex_handle> bp_fh(new CDT::Vertex_handle((get_face_handle_from_t_face(&vector_faces[i]))->vertex(k)));
	//vector_vertices.push_back(bp_fh);			
	//}

	//std::vector<boost::shared_ptr<Point_2> > vector_vertices_to_remove; //Go through all vertices and see if they are inside the new triangle
	//for (size_t i=0; i<vector_vertices.size(); ++i)
	//{
	//Point_2 p((*vector_vertices[i])->point().x(),(*vector_vertices[i])->point().y());
	//if (overlaps(&p0, &p1, &p2, &p))
	//{
	//boost::shared_ptr<Point_2> bp_p(new Point_2( (*vector_vertices[i])->point().x(),(*vector_vertices[i])->point().y()));
	//vector_vertices_to_remove.push_back(bp_p);
	//}
	//}

	//// -----------------------------------------------------------------------------------
	////remove vertices that were found to be inside new triangle
	//// -----------------------------------------------------------------------------------

	//if(debug>1)ROS_INFO("There are %d vertices to remove before adding new triangle (mesh has %d vertices)", (int)vector_vertices_to_remove.size(), (int)dt.number_of_vertices()); 

	////for (size_t i=0; i<vector_vertices_to_remove.size(); ++i)
	////{
	////if(debug)ROS_INFO("Vertex index %ld", vector_vertices_to_remove[i]->info().index); 
	////}

	//for (size_t i=0; i<vector_vertices_to_remove.size(); ++i)
	//{

	////printf("i=%ld\n",i);
	//test_if_triangulation_is_valid();

	//CDT::Locate_type lt;
	//int li;

	////CDT::Vertex_handle tmp = (vector_vertices_to_remove[i]);
	////if (tmp==NULL)continue;

	//CDT::Face_handle fh = dt.locate((*vector_vertices_to_remove[i]), lt, li); //locate the point on the triangle


	//if (lt==CDT::VERTEX)
	//{

	//int incident_face_count=0;

	//CDT::Face_circulator fc = dt.incident_faces(fh->vertex(li)),  done(fc); //circulate all faces that connect to this vertice

	//if (fc != 0)
	//{
	//do 
	//{
	//if(debug>1)ROS_INFO("Circulating over face(%ld,%ld,%ld)", fc->vertex(0)->info().index, fc->vertex(1)->info().index, fc->vertex(2)->info().index);
	//int fc_li1=-1; int fc_li2=-1;

	//for (int u=0; u<3;u++)
	//{
	//int k1,k2;
	//if (u==0){k1=1;k2=2;}
	//else if (u==1){k1=0;k2=2;}
	//else {k1=0;k2=1;}

	//if (fc->vertex(k1)->info().index == fh->vertex(li)->info().index ||
	//fc->vertex(k2)->info().index == fh->vertex(li)->info().index)
	//{
	//if (fc_li1==-1){fc_li1 = u;}
	//else{fc_li2=u; break;}
	//}
	//}

	//if(debug>1)ROS_INFO("fc_li1 = %d, fc_li2=%d", fc_li1, fc_li2);


	//if (fc->is_constrained(fc_li1))
	//{

	//if(debug>1)ROS_INFO("fc_li1 is constrained");
	//fc->set_constraint(fc_li1,false);
	//}


	//if (fc->is_constrained(fc_li2))
	//{
	//if(debug>1)ROS_INFO("fc_li2 is constrained");
	//fc->set_constraint(fc_li2,false);
	//}


	//incident_face_count++;
	//}while(++fc != done);
	//}

	//dt.remove_incident_constraints(fh->vertex(li));
	//dt.remove(fh->vertex(li));	

	//}
	//else
	//{
	//if(debug>0)ROS_WARN("Should be a vertex and its not??");
	//}

	//}

	//if(debug>0)
	//{
	//ROS_INFO("___remove_vertices_inside_triangle___\nThere are %d vertices to remove before adding new triangle (mesh has %d vertices)", (int)vector_vertices_to_remove.size(), (int)dt.number_of_vertices()); 
	//}


	return 1;
}

int class_constrained_delaunay_triangulation::remove_intersecting_constrained_edges(double x0, double y0, double z0, float rgb0, 
		double x1, double y1, double z1, float rgb1, 
		double x2, double y2, double z2, float rgb2,
		float face_weight, int provenience)
{
	//Point_2 p0(x0,y0);
	//Point_2 p1(x1,y1);
	//Point_2 p2(x2,y2);
	//CGAL::Segment_2<K_exact> seg_01(p0,p1);
	//CGAL::Segment_2<K_exact> seg_12(p1,p2);
	//CGAL::Segment_2<K_exact> seg_02(p0,p2);



	//// -----------------------------------------------------------------------------------
	////Check all visited faces edges for intersections and if constrained remove constraints
	//// -----------------------------------------------------------------------------------
	//for (size_t i=0; i<vector_faces.size(); ++i)
	//{
	//CDT::Face_handle tmp;
	//tmp = get_face_handle_from_t_face(&vector_faces[i]);
	//if (tmp==NULL) continue;

	//if (debug) ROS_WARN("Testing intr with face(%ld,%ld,%ld)",(get_face_handle_from_t_face(&vector_faces[i]))->vertex(0)->info().index, (get_face_handle_from_t_face(&vector_faces[i]))->vertex(1)->info().index, (get_face_handle_from_t_face(&vector_faces[i]))->vertex(2)->info().index);
	//for (int k=0; k<3; k++) //cycle all edges of mesh triangle
	//{
	//CDT::Face_handle tmp;
	//tmp = get_face_handle_from_t_face(&vector_faces[i]);
	//if (tmp==NULL) continue;

	//int k1,k2;
	//if (k==0) {k1=1; k2=2;}
	//else if (k==1) {k1=0; k2=2;}
	//else if (k==2) {k1=0; k2=1;}

	//if (debug) ROS_INFO("testing intr with edge %d - %d",(int)(get_face_handle_from_t_face(&vector_faces[i]))->vertex(k1)->info().index, (int)(get_face_handle_from_t_face(&vector_faces[i]))->vertex(k2)->info().index );


	//CGAL::Segment_2<K_exact> s(dt.segment((get_face_handle_from_t_face(&vector_faces[i])),k));

	//if (segments_intersect_not_at_endpoints(seg_01, s)) //test edge s with segment 0-1
	//{
	//if(debug)ROS_INFO("Found an intersection for seg01 [v0(%f %f) - v1(%f %f)] and mesh edge mv%ld(%f %f)-mv%ld(%f %f)", CGAL::to_double(seg_01.point(0).x()), CGAL::to_double(seg_01.point(0).y()), CGAL::to_double(seg_01.point(1).x()), CGAL::to_double(seg_01.point(1).y()),(get_face_handle_from_t_face(&vector_faces[i]))->vertex(k1)->info().index, CGAL::to_double(s.point(0).x()), CGAL::to_double(s.point(0).y()), (get_face_handle_from_t_face(&vector_faces[i]))->vertex(k2)->info().index, CGAL::to_double(s.point(1).x()), CGAL::to_double(s.point(1).y()));

	//remove_constraint((get_face_handle_from_t_face(&vector_faces[i])),k);
	//}

	//tmp = get_face_handle_from_t_face(&vector_faces[i]);
	//if (tmp==NULL) continue;

	//if (segments_intersect_not_at_endpoints(seg_12, s)) //test edge s with segment 1-2
	//{
	//if(debug)ROS_INFO("Found an intersection for seg12 [v0(%f %f) - v1(%f %f)] and mesh edge mv%ld(%f %f)-mv%ld(%f %f)", CGAL::to_double(seg_12.point(0).x()), CGAL::to_double(seg_12.point(0).y()), CGAL::to_double(seg_12.point(1).x()), CGAL::to_double(seg_12.point(1).y()),(get_face_handle_from_t_face(&vector_faces[i]))->vertex(k1)->info().index, CGAL::to_double(s.point(0).x()), CGAL::to_double(s.point(0).y()), (get_face_handle_from_t_face(&vector_faces[i]))->vertex(k2)->info().index, CGAL::to_double(s.point(1).x()), CGAL::to_double(s.point(1).y()));

	//remove_constraint((get_face_handle_from_t_face(&vector_faces[i])),k);
	//}

	//tmp = get_face_handle_from_t_face(&vector_faces[i]);
	//if (tmp==NULL) continue;

	//if (segments_intersect_not_at_endpoints(seg_02, s)) //test edge s with segment 0-2
	//{
	//if(debug)ROS_INFO("Found an intersection for seg02 [v0(%f %f) - v1(%f %f)] and mesh edge mv%ld(%f %f)-mv%ld(%f %f)", CGAL::to_double(seg_02.point(0).x()), CGAL::to_double(seg_02.point(0).y()), CGAL::to_double(seg_02.point(1).x()), CGAL::to_double(seg_02.point(1).y()),(get_face_handle_from_t_face(&vector_faces[i]))->vertex(k1)->info().index, CGAL::to_double(s.point(0).x()), CGAL::to_double(s.point(0).y()), (get_face_handle_from_t_face(&vector_faces[i]))->vertex(k2)->info().index, CGAL::to_double(s.point(1).x()), CGAL::to_double(s.point(1).y()));

	//remove_constraint((get_face_handle_from_t_face(&vector_faces[i])),k);
	//}
	//}
	//}




	return 1;
}





#endif
/**
 *@}
 */      

