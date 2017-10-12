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
 *@addtogroup projection_mesh
 *@{
 * @file
 * @brief Code for computing the projection of local meshes to the global mesh
 */
#ifndef _projection_mesh_CPP_
#define _projection_mesh_CPP_

#include "projection_mesh.h"



int class_projection_mesh::clear_constraints(void)
{
	for (PT::Vertex_iterator it=mesh.vertices_begin(); it!=mesh.vertices_end(); ++it)
	{
		mesh.remove_incident_constraints(it); 
	}
	return 1;
}

int class_projection_mesh::clear_vertices(void)
{
	mesh.clear();
	return 1;
}

int class_projection_mesh::set_constraint_polygon(std::vector<pcl::PointXY>* p)
{
	PT::Locate_type lt;
	int li;
	std::vector<PT::Vertex_handle> vv;

	for (size_t i=0; i<p->size();i++)
	{
		pcl::PointXY pt = p->at(i);
		PT::Face_handle fh =  mesh.locate(PT::Point(pt.x,pt.y), lt, li);

		if (lt==PT::VERTEX)
		{
			vv.push_back(fh->vertex(li));
		}
		else
		{
			PT::Vertex_handle vh;
			vh = mesh.insert(PT::Point(pt.x,pt.y));                  
			vh->info().z = 0;
			vh->info().rgb = -1;
			vh->info().weight = 0;
			vv.push_back(vh);
		}
	}


	for (size_t i=1; i<vv.size();i++)
	{
		mesh.insert_constraint(vv[i-1], vv[i]);
	}
	mesh.insert_constraint(vv[vv.size()-1], vv[0]);

	return 1;
}

int class_projection_mesh::add_vertex_to_mesh(double x, double y, double z, float rgb, float weight)
{
	PT::Vertex_handle vh;
	vh = mesh.insert(PT::Point(x,y));                  
	vh->info().z = z;
	vh->info().rgb = rgb;
	vh->info().weight = weight;
	return 1;
}

int class_projection_mesh::initialize_all_faces_status(void)
{
	for(PT::All_faces_iterator it = mesh.all_faces_begin(); it != mesh.all_faces_end(); ++it)
	{  
		it->counter=-1;
		it->status=-1;
	}
	return 1;
}


void class_projection_mesh::discoverComponent(const PT& ct,
		PT::Face_handle start, 
		int index, 
		std::list<PT::Edge>& border )
{
	if(start->counter != -1)
	{
		return;
	}
	std::list<PT::Face_handle> queue;
	queue.push_back(start);

	while(! queue.empty())
	{
		PT::Face_handle fh = queue.front();
		queue.pop_front();
		if(fh->counter == -1)
		{
			fh->counter = index;
			fh->set_in_domain(index%2 == 1);
			for(int i = 0; i < 3; i++)
			{
				PT::Edge e(fh,i);
				PT::Face_handle n = fh->neighbor(i);
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

void class_projection_mesh::discoverComponents(const PT& ct)
{
	int index = 0;
	std::list<PT::Edge> border;
	discoverComponent(ct, ct.infinite_face(), index++, border);
	while(! border.empty())
	{
		PT::Edge e = border.front();
		border.pop_front();
		PT::Face_handle n = e.first->neighbor(e.second);
		if(n->counter == -1)
		{
			discoverComponent(ct, n, e.first->counter+1, border);
		}
	}
} 


int class_projection_mesh::draw_mesh_triangles(cv::Mat* image, cv::Scalar color, int thickness)
{
	for(PT::Finite_faces_iterator fc1 = mesh.finite_faces_begin(); fc1 != mesh.finite_faces_end(); ++fc1)
	{
		if (!fc1->is_in_domain())
		{
			//continue;
		}

		cv::line(*image, cv::Point2d( CGAL::to_double(fc1->vertex(0)->point().x()), CGAL::to_double(fc1->vertex(0)->point().y())), cv::Point2d( CGAL::to_double(fc1->vertex(1)->point().x()), CGAL::to_double(fc1->vertex(1)->point().y())), color, thickness);
		cv::line(*image, cv::Point2d( CGAL::to_double(fc1->vertex(1)->point().x()), CGAL::to_double(fc1->vertex(1)->point().y())), cv::Point2d( CGAL::to_double(fc1->vertex(2)->point().x()), CGAL::to_double(fc1->vertex(2)->point().y())), color, thickness);
		cv::line(*image, cv::Point2d( CGAL::to_double(fc1->vertex(0)->point().x()), CGAL::to_double(fc1->vertex(0)->point().y())), cv::Point2d( CGAL::to_double(fc1->vertex(2)->point().x()), CGAL::to_double(fc1->vertex(2)->point().y())), color, thickness);
	}

	return 1;
}


int class_projection_mesh::compute_face_weights(void)
{
	for(PT::Finite_faces_iterator fi = mesh.finite_faces_begin(); fi != mesh.finite_faces_end(); ++fi)
	{
		if (!fi->is_in_domain())
		{
			//continue;
		}

		fi->weight = (fi->vertex(0)->info().weight + fi->vertex(1)->info().weight + fi->vertex(2)->info().weight)/3.0;
	}


	return 1;
}

int class_projection_mesh::export_triangles_in_order(pcl::PointCloud<pcl::PointXYZRGB>* vertex_list, std::vector<float>* face_weights)
{
	//erase both lists
	vertex_list->points.erase(vertex_list->points.begin(), vertex_list->points.end());	
	vertex_list->erase(vertex_list->begin(), vertex_list->end());	

	pcl::PointXYZRGB pt;

	initialize_visited();

	std::vector<PT::Face_handle> queue;
	PT::Finite_faces_iterator fi= mesh.finite_faces_begin();
	queue.push_back(fi);


	for (size_t i=0; i<queue.size(); i++)
	{
		if (queue[i]->visited==false)
		{
			//propagate
			PT::Face_handle nfh;

			nfh	= queue[i]->neighbor(0);	
			if (nfh->visited==false && !mesh.is_infinite(nfh))
			{
				queue.push_back(nfh);			
			}

			nfh	= queue[i]->neighbor(1);	
			if (nfh->visited==false && !mesh.is_infinite(nfh))
			{
				queue.push_back(nfh);			
			}

			nfh	= queue[i]->neighbor(2);	
			if (nfh->visited==false && !mesh.is_infinite(nfh))
			{
				queue.push_back(nfh);			
			}

			queue[i]->visited=true;

			if (queue[i]->is_in_domain())
			{
				for (int u=0; u<3;u++)
				{
					pt.x = CGAL::to_double(queue[i]->vertex(u)->point().x()); 
					pt.y = CGAL::to_double(queue[i]->vertex(u)->point().y()); 
					pt.z = CGAL::to_double(queue[i]->vertex(u)->info().z); 
					pt.rgb = queue[i]->vertex(u)->info().rgb; 
					vertex_list->points.push_back(pt);
				}
				face_weights->push_back(queue[i]->weight);
			}
			queue.erase(queue.begin()+i);
			i=-1;
		}
	}

	return 1;
}


int class_projection_mesh::export_triangles(pcl::PointCloud<pcl::PointXYZRGB>* vertex_list, std::vector<float>* face_weights)
{
	//erase both lists
	vertex_list->points.erase(vertex_list->points.begin(), vertex_list->points.end());	
	vertex_list->erase(vertex_list->begin(), vertex_list->end());	

	pcl::PointXYZRGB pt;

	for(PT::Finite_faces_iterator fi = mesh.finite_faces_begin(); fi != mesh.finite_faces_end(); ++fi)
	{
		if (!fi->is_in_domain())
		{
			continue;
		}

		for (int u=0; u<3;u++)
		{
			pt.x = CGAL::to_double(fi->vertex(u)->point().x()); 
			pt.y = CGAL::to_double(fi->vertex(u)->point().y()); 
			pt.z = CGAL::to_double(fi->vertex(u)->info().z); 
			pt.rgb = fi->vertex(u)->info().rgb; 
			vertex_list->points.push_back(pt);
		}

		face_weights->push_back(fi->weight);
	}

	return 1;
}
#endif
/**
 *@}
 */      

