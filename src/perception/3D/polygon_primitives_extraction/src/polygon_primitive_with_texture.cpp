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
 * @addtogroup polygon_primitive_with_texture 
 * @{
 * @file 
 * @brief Holds the c_polygon_primitive basic methods
 * @author Miguel Oliveira
 * @version v0
 * @date 2011-12-19
 */
#ifndef _polygon_primitive_with_texture_CPP_
#define _polygon_primitive_with_texture_CPP_


#include "polygon_primitive_with_texture.h"

int c_polygon_primitive_with_texture::compute_projection_union(void)
{
	ROS_INFO("Computing the projection union");
	dp.projection_union.erase(dp.projection_union.begin(), dp.projection_union.end());

	if (cp.size()>0)
	{
		tf::Transform tf_inv = data.frames.current.transform.inverse();
		class_polygon_boolean_operations pbo;
		pbo.set_transform(&tf_inv);
		pbo.insert(&cp[0].vertex_intersection); //insert the first projection

		for (size_t i=1; i< cp.size(); i++) //union with all other projections
		{
			pbo.join(&cp[i].vertex_intersection);
		}

		//ROS_INFO("AFTER Computing the projection union polygon union description");
		//pbo.print();

		pbo.get_all_pcls(&dp.projection_union);
	}

	ROS_INFO("Computing the projection union FINISHED");

	return 1;
}

int c_polygon_primitive_with_texture::add_camera_projection_known_camera(cv::Mat* m, ros::Time t, tf::StampedTransform tf, std::string cam_name, std::string projection_name, cv::Scalar color)
{
	//the parameters to set according to the camera
	double fx=0,fy=0,cx=0,cy=0,k1=0,k2=0,p1=0,p2=0,k3=0,sd_cx=0, sd_cy=0, param=0, scale=0;

	if (cam_name=="cam_roof_fc")
	{
		fx = 479.867758/2.; fy=	479.867758/2.; cx= 360.036957/2.; cy= 267.139665/2.; 
		k1=0; k2=0; p1=0; p2=0; k3=0; 
		sd_cx= -0.020410; sd_cy=0.033517; param= 1.074213; scale= 0.5;
	}
	else if(cam_name=="cam_roof_fl")
	{
		fx = 479.567636/2; fy= 479.567636/2; cx= 359.563721/2; cy= 216.757192/2;
		k1= 0; k2= 0; p1= 0; p2= 0; k3= 0;
		sd_cx= -0.021519; sd_cy= -0.031968; param= 1.072291; scale= 0.5;
	}
	else if(cam_name=="cam_roof_fr")
	{
		fx= 482.093010/2; fy= 482.093010/2; cx= 384.128030/2; cy= 211.236693/2;
		k1= 0; k2= 0; p1= 0; p2= 0; k3=0; 
		sd_cx= 0.011822; sd_cy=-0.039534; param= 1.071786; scale= 0.5;
	}
	else if(cam_name=="cam_roof_rc")
	{
		fx= 473.1791/2; fy=473.1791/2; cx=333.0504/2; cy=237.5875/2;
		k1= 0; k2= 0; p1= 0; p2= 0; k3=0;
		sd_cx= -0.05616908; sd_cy=-0.005963998; param= 1.085173436; scale= 0.5;
	}
	else if(cam_name=="cam_roof_fc_6mm")
	{
		fx=1024.920603/2; fy=1024.920603/2; cx=373.788212/2; cy=233.819293/2;
		k1= 0; k2= 0; p1= 0; p2= 0; k3=0;
		sd_cx=-0.001310066976; sd_cy=-0.01458390567; param= 0.68462101929529; scale= 0.5;
	}
	else
	{
		ROS_ERROR("add_camera_projection_known_camera: camera name is unknown.");
		return 0;
	}

	return add_camera_projection(m, t, fx, fy, cx, cy, tf, k1, k2, p1, p2, k3, sd_cx, sd_cy, param, scale, projection_name, color);
}

int c_polygon_primitive_with_texture::add_camera_projection(cv::Mat* m, ros::Time t, double fx, double fy, double cx, double cy, tf::StampedTransform tf, double k1, double k2, double p1, double p2, double k3, double sd_cx, double sd_cy, double param, double scale, std::string projection_name, cv::Scalar color)
{
	//-----------------------------------------------
	//Start a new class camera projection
	//-----------------------------------------------
	class_camera_projection camera;

	PFLN
	//set some parameters
	camera.projection_name = projection_name;
	camera.set_width_height_num_channels(m->rows, m->cols,m->channels());
	camera.set_distortion_spherical(m->cols*2, m->rows*2, sd_cx, sd_cy, param, scale);
	camera.set_distortion(k1,k2,p1,p2,k3);
	camera.set_intrinsic(fx,fy,cx,cy);
	camera.set_extrinsic(&tf);
	tf::Transform tf_inv = data.frames.current.transform.inverse();
	camera.set_projection_plane(data.planes.current->values[0], data.planes.current->values[1], data.planes.current->values[2], data.planes.current->values[3]);

	camera.set_image(m, t, tf.frame_id_);

	camera.vertex_canvas.header.frame_id = "/world"; camera.vertex_canvas.header.stamp = t; camera.vertex_canvas.height = 1; camera.vertex_canvas.is_dense = 0;

	camera.set_vertex_chull(data.hulls.convex.polygon);	

	//-----------------------------------------------
	//Get the projection of the camera canvas polygon onto the polygonal plane
	//-----------------------------------------------
	if (!camera.project_pixel_to_vertex(&camera.pixels_canvas, &camera.vertex_canvas))
	{
		ROS_WARN("Projection %s: canvas polygon does not have projection on the plane of the polygon", camera.projection_name.c_str());	
		return -1;
	}
	camera.vertex_canvas.width = camera.vertex_canvas.points.size();


	//-----------------------------------------------
	//Obtain the intersection between the canvas polygon projection and the convex hull. The intersection is called vertex_intersection
	//-----------------------------------------------
	class_polygon_boolean_operations pbo;
	pbo.set_transform(&tf_inv);

	ROS_INFO("chull size = %ld",camera.vertex_chull.size());
	if (!pbo.insert(&camera.vertex_chull))
	{
		ROS_WARN("Could not insert convex hull");
		return -1;
	}

	ROS_INFO("vertex canvas size = %ld",camera.vertex_canvas.size());
	if (!pbo.intersection(&camera.vertex_canvas))
	{
		ROS_WARN("Could not intersect with canvas");
		return -1;
	}

	pbo.get_largest_pcl(&camera.vertex_intersection);

	if (camera.vertex_intersection.points.size()==0)
	{
		ROS_WARN("Vertex_intersection has zero points");
		return -1;
	}

	camera.vertex_intersection.header.frame_id = "/world"; camera.vertex_intersection.header.stamp = t;	camera.vertex_intersection.height = 1; camera.vertex_intersection.is_dense = 0;


	//-----------------------------------------------
	//Project the points in vertex_intersection to pixels
	//-----------------------------------------------
	camera.project_vertex_to_pixel(camera.vertex_intersection.makeShared(), &camera.pixels_intersection);
	camera.vertex_intersection.width = camera.vertex_intersection.points.size();


	//-----------------------------------------------
	//Compute the projectable pixels using the pixels_intersection polygon
	//-----------------------------------------------

	camera.compute_projectable_pixels();


	//-----------------------------------------------
	//Project the projectable pixels to the plane. Compute the weight for each pixel/vertex
	//-----------------------------------------------
	camera.project_pixel_with_color_to_vertex(&camera.pixels_projectable, &camera.vertex_projectable);
	camera.vertex_projectable.height=1;
	camera.vertex_projectable.header.frame_id="/world";
	camera.vertex_projectable.header.stamp=ros::Time::now();
	camera.vertex_projectable.is_dense=0;
	camera.vertex_projectable.width=camera.vertex_projectable.points.size();


	//float px = (tf.inverse().getOrigin()).x();
	//float py = (tf.inverse().getOrigin()).y();
	//float pz = (tf.inverse().getOrigin()).z();

	//float resolution_factor = cx/500;
	//for(size_t i=0; i<camera.vertex_projectable.points.size(); i++)
	//{

	////ROS_INFO("Origin x=%f y=%f z=%f", px, py,pz);
	////ROS_INFO("V[%d] x=%f y=%f z=%f", (int)i, camera.vertex_projectable.points[i].x, camera.vertex_projectable.points[i].y,camera.vertex_projectable.points[i].z);
	//float dist = sqrt(
	//(px-camera.vertex_projectable.points[i].x)*(px-camera.vertex_projectable.points[i].x)+
	//(py-camera.vertex_projectable.points[i].y)*(py-camera.vertex_projectable.points[i].y)+
	//(pz-camera.vertex_projectable.points[i].z)*(pz-camera.vertex_projectable.points[i].z)
	//);

	//if (dist>40) dist=40;
	//if (dist<1) dist=1;
	//float dist_factor = 1-(dist-1)/(40-1); //dist is now from 0-1
	////float dist_factor = 1.0/dist;
	//camera.vertex_projectable_weight.push_back(dist_factor*resolution_factor);
	////ROS_INFO("%d dist=%f dist_factor=%f", (int)i, dist, dist_factor);
	//}

	//-----------------------------------------------
	//Build a local mesh with all the pixels
	//-----------------------------------------------

	//for(size_t i=0; i<camera.pixels_projectable.points.size(); i++)
	//camera.add_vertex_to_mesh(camera.pixels_projectable.points[i].x, camera.pixels_projectable.points[i].y, camera.pixels_projectable.points[i].z, camera.pixels_projectable.points[i].rgb,  camera.vertex_projectable_weight[i]);


	//for(size_t i=0; i<camera.lines.size(); i++)
	//{
	//PT::Point pt1(camera.lines[i][0],camera.lines[i][1]);
	//PT::Point pt2(camera.lines[i][2],camera.lines[i][3]);
	//camera.mesh.insert_constraint(pt1,pt2);
	//}



	for (int l=0; l<camera.mask_projectable_pixels.rows; l++)
		for (int c=0; c<camera.mask_projectable_pixels.cols; c++)
		{
			if (camera.mask_projectable_pixels.at<unsigned char>(l,c)) //in this case should project
			{
				PT::Point pt(c,l);
				camera.mesh.insert(pt);
			}	
		}


	float px = (tf.inverse().getOrigin()).x();
	float py = (tf.inverse().getOrigin()).y();
	float pz = (tf.inverse().getOrigin()).z();

	//set the color of the mesh vertices
	for (PT::Vertex_iterator it = camera.mesh.vertices_begin(); it != camera.mesh.vertices_end(); ++it)
	{
		if (camera.mesh.is_infinite(it))
			continue;

		int c = (int)CGAL::to_double(it->point().x());
		int l = (int)CGAL::to_double(it->point().y());

		unsigned char r = camera.image_corrected.at<t_pixel>(l,c)[0];
		unsigned char g = camera.image_corrected.at<t_pixel>(l,c)[1];
		unsigned char b = camera.image_corrected.at<t_pixel>(l,c)[2];
		uint32_t rgb = ((uint32_t)b << 16 | (uint32_t)g << 8 | (uint32_t)r);
		it->info().rgb = *reinterpret_cast<float*>(&rgb);
		it->info().weight = camera.get_weight_for_pixel(l,c, 1, px,py,pz);
	}


	//run a filter to erase some unrequired vertices 
	std::vector<PT::Vertex_handle> vec;
	for (PT::Vertex_iterator vit = camera.mesh.vertices_begin(); vit != camera.mesh.vertices_end(); ++vit)
	{
		std::vector<float> vr; std::vector<float> vg; std::vector<float> vb;
		PT::Vertex_circulator vc = camera.mesh.incident_vertices(vit), done(vc); 
		if(vc!=0) 
		{ 
			do
			{ 
				// unpack rgb into r/g/b
				uint32_t rgb = *reinterpret_cast<int*>(&vc->info().rgb);
				uint8_t r = (rgb >> 16) & 0x0000ff;
				uint8_t g = (rgb >> 8)  & 0x0000ff;
				uint8_t b = (rgb)       & 0x0000ff;

				vr.push_back((float)r); vg.push_back((float)g); vb.push_back((float)b); 
			}
			while(++vc!=done); 

			float mean_r, mean_g, mean_b;
			float std_r, std_g, std_b;

			compute_mean_and_std(&vr, &mean_r, &std_r);
			compute_mean_and_std(&vg, &mean_g, &std_g);
			compute_mean_and_std(&vb, &mean_b, &std_b);

			// unpack rgb into r/g/b
			uint32_t rgb = *reinterpret_cast<int*>(&vit->info().rgb);
			uint8_t ur = (rgb >> 16) & 0x0000ff;
			uint8_t ug = (rgb >> 8)  & 0x0000ff;
			uint8_t ub = (rgb)       & 0x0000ff;

			float r= (float)ur; float g= (float)ug;	float b= (float)ub;

			if (std_r<5 && std_g<5 && std_b<5 &&
					fabs(mean_r-r)< 5 && 
					fabs(mean_g-g)< 5 && 
					fabs(mean_b-b)< 5)
			{
				vec.push_back(vit);	
			}

		}
	}

	//ROS_INFO("Plan to remove %ld vertices total now is %ld", vec.size(), camera.mesh.number_of_vertices());

	for (size_t i=0; i<vec.size(); i++)
	{
		camera.mesh.remove(vec[i]);
	}

	//ROS_INFO("Remove %ld vertices total now is %ld", vec.size(), camera.mesh.number_of_vertices());
	
	camera.set_constraint_polygon(&camera.pixels_intersection);

	px = (tf.inverse().getOrigin()).x();
	py = (tf.inverse().getOrigin()).y();
	pz = (tf.inverse().getOrigin()).z();

	//set the color of the mesh vertices
	for (PT::Vertex_iterator it = camera.mesh.vertices_begin(); it != camera.mesh.vertices_end(); ++it)
	{
		if (camera.mesh.is_infinite(it))
			continue;

		int c = (int)CGAL::to_double(it->point().x());
		int l = (int)CGAL::to_double(it->point().y());

		unsigned char r = camera.image_corrected.at<t_pixel>(l,c)[0];
		unsigned char g = camera.image_corrected.at<t_pixel>(l,c)[1];
		unsigned char b = camera.image_corrected.at<t_pixel>(l,c)[2];
		uint32_t rgb = ((uint32_t)b << 16 | (uint32_t)g << 8 | (uint32_t)r);
		it->info().rgb = *reinterpret_cast<float*>(&rgb);
		it->info().weight = camera.get_weight_for_pixel(l,c, 1, px,py,pz);
	}
	camera.compute_face_weights();


	camera.initialize_all_faces_status();
	camera.discoverComponents(camera.mesh);
	camera.export_triangles_in_order(&camera.pixel_list, &camera.face_weight);


	//-----------------------------------------------
	//Draw the gui image where the intersection and canvas polygons, the projectable pixels and triangles are drawn
	//-----------------------------------------------
	camera.draw_mesh_triangles(&camera.image_gui, cv::Scalar(0,255,0), 1);	
	camera.draw_pixels_vector_as_polyline(&camera.image_gui, &camera.pixels_canvas, cv::Scalar(0,0,255),3);
	camera.draw_pixels_vector_as_polyline(&camera.image_gui, &camera.pixels_intersection, cv::Scalar(255,0,0),2);
	//camera.draw_pixels_projectable(&camera.image_gui, &camera.pixels_projectable, color,1);

	//-----------------------------------------------
	// Finnally, add the camera to the list
	//-----------------------------------------------
	cp.push_back(camera);

	return cp.size()-1;
}


int c_polygon_primitive_with_texture::compute_mean_and_std(std::vector<float>* v, float* mean, float* std)
{
	float sum=0;
	for (size_t i=0; i<v->size(); i++)
	{
		sum += (*v)[i];
	}

	*mean = sum/(v->size());

	float std_sum=0;
	for (size_t i=0; i<v->size(); i++)
	{
		std_sum += ((*v)[i]-(*mean))*((*v)[i]-(*mean));
	}

	*std = sqrt(std_sum/(v->size()-1));
	return 1;
}

//int c_polygon_primitive_with_texture::add_camera_projection_to_triangle_mesh(int projection_index, ros::Publisher* p_marker_pub)
//{
//ROS_INFO("Adding camera projection %s mesh",cp[projection_index].projection_name.c_str());
//ROS_INFO("Before adding, triangle mesh has %d faces and %d vertices",(int)dp.dt.number_of_faces(),(int) dp.dt.number_of_vertices());

////--------------------------------------------
////give out a warning if this projection has already been executed
////--------------------------------------------
//if (cp[projection_index].is_mapped_to_mesh==true) 
//{
//ROS_WARN("Adding camera projection %d that has already been projected",projection_index);
//}


////--------------------------------------------
////update the pi and dp transformations, just in case they changed
////--------------------------------------------
//tf::Transform tf_inv = data.frames.current.transform.inverse();
//dp.set_transform(&tf_inv);

////--------------------------------------------
////Recompute the projection union
//// 	This is done by adding the polygons in the projection union, 
//// 	then adding this camera projection intersection polygon
//// 	Then erase the old projection_union and recomputing
////--------------------------------------------

//class_polygon_boolean_operations pbo;
//pbo.set_transform(&tf_inv);

////if (dp.projection_union.size()>0)
////{
////for (size_t k=0; k<dp.projection_union.size(); k++)
////{
////if (!pbo.insert(&dp.projection_union[k])) return 0;
////}
//////ROS_WARN("Projection union before join");
//////pbo.print();


////if (!pbo.join(&cp[projection_index].vertex_full_projection)) return 0;

//////ROS_WARN("Projection union after join");
//////pbo.print();

////}
////else
////{
////if (!pbo.insert(&cp[projection_index].vertex_intersection)) return 0;
////}

////ROS_WARN("Projection union");
////pbo.print();

////pbo.get_all_pcls(&dp.projection_union);





////dp.pi.clear_all_polygons(); //clear all the stored polygons

////for (size_t i=0; i<dp.projection_union.size(); i++) //add the previous projection union	
////{
////dp.pi.add_polygon_to_list(&dp.projection_union[i]);
////}

//////Add this camera projection intersection polygon
////dp.pi.add_polygon_to_list(&cp[projection_index].vertex_intersection);


////compute the new projection union
////dp.compute_union();

////ROS_INFO("Recomputed projection union. pu has %d polygons", (int)dp.projection_union.size());

////--------------------------------------------
////add the points in this camera projection
////--------------------------------------------
////ROS_WARN("Adding %d vertex projectable to mesh", (int)cp[projection_index].vertex_projectable.points.size());


//size_t num_added=0;
//size_t num_replacing=0;

////declare de point cloud in local coordinates
//pcl::PointCloud<pcl::PointXYZRGB> pc_local;

////transform the pc to local coordinates
//pcl_ros::transformPointCloud(cp[projection_index].vertex_projectable, pc_local, dp.transform); 

//int a=1;
//int start=100000;
//for (size_t i=0; i<pc_local.points.size();i++)
//{
//size_t r= dp.add_point_manager(pc_local.points[i].x,
//pc_local.points[i].y,
//pc_local.points[i].z,
//cp[projection_index].vertex_projectable.points[i].rgb,
//cp[projection_index].vertex_projectable_weight.at(i), projection_index);

//if (r==1)
//num_added++;
//else if(r==2)
//num_replacing++;

//if (i>(start+a)&& projection_index!=0 )
//{

//dp.export_points_to_pc();


////for (it=pmap.begin(); it!=pmap.end();++it)
////{
//std::vector<visualization_msgs::Marker> marker_vec;
//visualization_msgs::MarkerArray marker_array_msg;


////it->second.create_textures_vizualization_msg(&marker_vec,true);
////marker_array_msg.set_markers_vec(marker_vec);	
////
////p_markerarray_pub->publish(marker_array_msg);

//create_textures_vizualization_msg(&marker_vec, false);

//marker_array_msg.set_markers_vec(marker_vec);	

//p_marker_pub->publish(marker_array_msg);


////char str[90];
//printf ("Number of pixels to add to mesh ");
//int ret=scanf("%d",&a); ret=0; 
//start=i;
////}

//}

//}

////ROS_INFO("Adding a point cloud with %d pts to mesh\n %d pts where added\n%d pts replaced others", (int)pc->points.size(), (int)num_added, (int)num_replacing);
//printf("FINISHED triangulation. Triangles %d vertices %d\n",(int)dp.dt.number_of_faces(),(int) dp.dt.number_of_vertices() );



////dp.add_point_cloud(&cp[projection_index].vertex_projectable, &cp[projection_index].vertex_projectable_weight, projection_index);

//
//cp[projection_index].is_mapped_to_mesh=true; //set is_mapped flag


//ROS_INFO("After adding, triangle mesh has %d faces and %d vertices",(int)dp.dt.number_of_faces(),(int) dp.dt.number_of_vertices());


//return 1;
//}



int c_polygon_primitive_with_texture::readapt_to_new_plane(polygon_primitive_msg::polygon_primitive* new_plg)
{
	pcl::ModelCoefficients::Ptr coeff = pcl::ModelCoefficients::Ptr(new pcl::ModelCoefficients);

	coeff->values.resize(4);
	coeff->values[0] = new_plg->support_plane_A;
	coeff->values[1] = new_plg->support_plane_B;
	coeff->values[2] = new_plg->support_plane_C;
	coeff->values[3] = new_plg->support_plane_D;

	for (size_t u=0; u<cp.size(); u++)
	{
		pcl::ProjectInliers<pcl::PointXYZ> projection;
		projection.setModelType(pcl::SACMODEL_PLANE); //set model type

		projection.setInputCloud(cp[u].vertex_canvas.makeShared());
		projection.setModelCoefficients(coeff);
		projection.filter(cp[u].vertex_canvas);

		projection.setInputCloud(cp[u].vertex_chull.makeShared());
		projection.setModelCoefficients(coeff);
		projection.filter(cp[u].vertex_chull);

		projection.setInputCloud(cp[u].vertex_intersection.makeShared());
		projection.setModelCoefficients(coeff);
		projection.filter(cp[u].vertex_intersection);

		pcl::ProjectInliers<pcl::PointXYZRGB> projectionRGB;
		projectionRGB.setModelType(pcl::SACMODEL_PLANE); //set model type

		projectionRGB.setInputCloud(cp[u].vertex_projectable.makeShared());
		projectionRGB.setModelCoefficients(coeff);
		projectionRGB.filter(cp[u].vertex_projectable);
	}	

	for (size_t u=0; u<dp.projection_union.size(); u++)
	{
		pcl::ProjectInliers<pcl::PointXYZ> projection;
		projection.setModelType(pcl::SACMODEL_PLANE); //set model type	

		projection.setInputCloud(dp.projection_union[u].makeShared());
		projection.setModelCoefficients(coeff);
		projection.filter(dp.projection_union[u]);
	}

	//copy the data from primitive to it->second
	import_from_polygon_primitive_msg(new_plg);

	return 1;
}

size_t c_polygon_primitive_with_texture::ask_for_number(void)
{
	size_t myNumber = 0;
	std::string input = "";
	while (true)
	{
		std::getline(std::cin, input);
		// This code converts from string to number safely.
		std::stringstream myStream(input);
		if (myStream >> myNumber)
			break;
		std::cout << "Invalid number, please try again" << std::endl;
	}
	return myNumber;
}

int c_polygon_primitive_with_texture::publish_to_rviz(ros::Publisher* p_marker_pub)
{
	dp.export_points_to_pc();
	visualization_msgs::MarkerArray marker_vec;
	//visualization_msgs::MarkerArray marker_array_msg;
	create_textures_vizualization_msg(&marker_vec,true);
	//marker_array_msg.set_markers_vec(marker_vec);	
	p_marker_pub->publish(marker_vec);
	return 1;
}

int c_polygon_primitive_with_texture::build_global_mesh(ros::Publisher* p_marker_pub)
{
	int debug=0;
	//First, erase previous mesh as well as constraints
	//dp.clear_constraints();
	//dp.clear_vertices();

	tf::Transform tf_inv = data.frames.current.transform.inverse();
	dp.set_transform(&tf_inv);

	//go though all camera projections
	for (size_t i=0; i<cp.size(); i++)
	{
		if (cp[i].is_mapped_to_mesh==true) 
		{
			ROS_INFO("Camera projection %ld (%s) is already added to global mesh", i, cp[i].projection_name.c_str());	
			continue;
		}

		ros::Time t_overall = ros::Time::now();

		cp[i].set_projection_plane(data.planes.current->values[0], data.planes.current->values[1], data.planes.current->values[2], data.planes.current->values[3]);
		cp[i].project_pixel_with_color_to_vertex(&cp[i].pixel_list, &cp[i].vertex_list);
		cp[i].vertex_list.height=1;
		cp[i].vertex_list.header.frame_id="/world";
		cp[i].vertex_list.header.stamp=ros::Time::now();
		cp[i].vertex_list.is_dense=0;
		cp[i].vertex_list.width=cp[i].vertex_list.points.size();

		//get this projection triangle vertices list in local polygon coordinates
		pcl::PointCloud<pcl::PointXYZRGB> pl; //declare the point cloud in local coordinates
		pcl_ros::transformPointCloud(cp[i].vertex_list, pl, dp.transform); //transform the pc to local coordinates

		//round to 2 decimal place
		for (size_t g=0; g<pl.points.size(); ++g)
		{
			pl.points[g].x = floorf(pl.points[g].x * 1000 + 0.5) / 1000;
			pl.points[g].y = floorf(pl.points[g].y * 1000 + 0.5) / 1000;
			pl.points[g].z = floorf(pl.points[g].z * 1000 + 0.5) / 1000;
		}

		ROS_INFO("Adding camera projection %ld (%s) to global mesh %ld triangles", i, cp[i].projection_name.c_str(), pl.points.size()/3);	

		size_t process_until_index=0;
		size_t cnt_skip_search=0;
		size_t cnt_vertices_removed=0;
		size_t cnt_constraints_removed=0;
		size_t cnt_add_face=0;
		ros::Duration d_skip_search(0);
		ros::Duration d_vertices_removed(0);
		ros::Duration d_constraints_removed(0);
		ros::Duration d_add_face(0);
		for (size_t k=0, u=0; u<pl.points.size(); u+=3, k++) //go from triangle to triangle
		{
			ros::Duration d; ros::Time t; int cnt;

			if(debug)ROS_INFO("Adding camera projection %ld (%s) to global mesh (%ld of %ld triangles)", i, cp[i].projection_name.c_str(), k, pl.points.size()/3);	
			bool insert_triangle=true;

			//Set the next triangle to insert
			next_triangle.points.erase(next_triangle.points.begin(), next_triangle.points.end());
			pcl::PointXYZ pt;
			pt.x = cp[i].vertex_list.points[u].x; pt.y = cp[i].vertex_list.points[u].y; pt.z = cp[i].vertex_list.points[u].z; 
			next_triangle.points.push_back(pt);
			pt.x = cp[i].vertex_list.points[u+1].x; pt.y = cp[i].vertex_list.points[u+1].y; pt.z = cp[i].vertex_list.points[u+1].z;
			next_triangle.points.push_back(pt);


			pt.x = cp[i].vertex_list.points[u+1].x; pt.y = cp[i].vertex_list.points[u+1].y; pt.z = cp[i].vertex_list.points[u+1].z;
			next_triangle.points.push_back(pt);
			pt.x = cp[i].vertex_list.points[u+2].x; pt.y = cp[i].vertex_list.points[u+2].y; pt.z = cp[i].vertex_list.points[u+2].z;
			next_triangle.points.push_back(pt);


			pt.x = cp[i].vertex_list.points[u].x; pt.y = cp[i].vertex_list.points[u].y; pt.z = cp[i].vertex_list.points[u].z; 
			next_triangle.points.push_back(pt);
			pt.x = cp[i].vertex_list.points[u+2].x; pt.y = cp[i].vertex_list.points[u+2].y; pt.z = cp[i].vertex_list.points[u+2].z;
			next_triangle.points.push_back(pt);


			//Create a Face_handle fti for a tmp mesh
			CDT tmp_mesh; 
			CDT::Vertex_handle vh[3];

			for (size_t j=0; j<3; ++j)
			{
				vh[j] = tmp_mesh.insert(Point_2(pl.points[u+j].x, pl.points[u+j].y));
				vh[j]->info().rgb = cp[i].vertex_list.points[u+j].rgb;
				vh[j]->info().z = 0;
				vh[j]->info().index = 0;
			}

			CDT::Face_handle fti;
			if (tmp_mesh.is_face(vh[0],vh[1],vh[2], fti))
			{
				fti->weight = cp[i].face_weight[k];
				fti->provenience = i;
			}
			else
			{
				ROS_ERROR("Could not start fti face");	
				continue;
			}


			if (debug) dp.printf_face_info(fti);


			if ( (k==0 || k>(process_until_index)) && debug ) //ask for how many triangles to insert
			{
				publish_to_rviz(p_marker_pub);//send initial state to rviz
				ROS_INFO("How many triangles to insert? (k=%d out of %d)", (int)k, ((int)pl.points.size()/3));
				process_until_index	= k+ask_for_number()-1;
			}

			//set the debug flag accordingly
			if (k==process_until_index && debug) dp.debug=0;
			else dp.debug=0;


			//if triangle is degenerate abort the insertion
			if (dp.is_degenerate(pl.points[u].x, pl.points[u].y, pl.points[u].z,
						pl.points[u+1].x, pl.points[u+1].y, pl.points[u+1].z,
						pl.points[u+2].x, pl.points[u+2].y, pl.points[u+2].z))
			{
				ROS_WARN("Skiping a degenerate triangle");
				continue;
			}


			t = ros::Time::now(); cnt=0;
			if(!dp.iterate_intersecting_faces(fti, 0))//use predicate number 0 to assess if there are intersecting faces with more weitght	
			{
				if(debug)ROS_INFO("could not insert triangle %ld out of %d. check_if_face_should_be_inserted refused", k, (int)pl.points.size()/3);
				continue;
			}
			cnt_skip_search++; d_skip_search += ros::Time::now()-t; 


			if ((k==process_until_index) && debug ) //draw the the mesh (hopefully) with the constraints removed
			{
				d = ros::Time::now()-t;
				ROS_INFO("iterate intersecting faces in %f seconds.", d.toSec());
			}


			t = ros::Time::now(); cnt=0;
			while(!dp.iterate_intersecting_faces(fti, 1))//use predicate number 1 to remove vertices
			{
				cnt++;
			}
			cnt_vertices_removed++; d_vertices_removed += ros::Time::now()-t; 

			if ((k==process_until_index) && debug ) //draw the the mesh (hopefully) with the constraints removed
			{
				d = ros::Time::now()-t;
				publish_to_rviz(p_marker_pub);//send initial state to rviz
				ROS_INFO("Vertices inside triangle removed in %d iterations and %f seconds. press to continue", cnt, d.toSec());
				ask_for_number();
			}


			// ______________ REMOVE INTERSECTING CONSTRAINTS __________________
			t = ros::Time::now(); cnt=0;
			while(!dp.iterate_intersecting_faces(fti, 2))//use predicate number 2 to remove intersecting constraints
			{
				cnt++;
			}
			cnt_constraints_removed++; d_constraints_removed += ros::Time::now()-t; 

			if ((k==process_until_index) && debug )
			{
				d = ros::Time::now()-t;
				publish_to_rviz(p_marker_pub);//send initial state to rviz
				ROS_INFO("Intersecting constrained edges removed in %d iterations and %f seconds\nMesh prepared to receive new triangle %ld out of %d. \nDo you want to insert the triangle? [0,1]",cnt, d.toSec(), k, (int)pl.points.size()/3);
				if (ask_for_number()) insert_triangle=true;
				else insert_triangle=false;
			}

			// ______________ ADD FACE TO MESH __________________
			t = ros::Time::now(); cnt=0;
			if(insert_triangle)
			{
				dp.add_face_to_mesh(fti);
			}
			cnt_add_face++; d_add_face += ros::Time::now()-t; 

			if ((k==process_until_index) && debug )
			{
				d = ros::Time::now()-t;
				ROS_INFO("Triangle inserted in %f seconds", d.toSec());
			}
		}

		cp[i].is_mapped_to_mesh=true; //set is_mapped flag
		ros::Duration d_overall;
		d_overall = ros::Time::now()-t_overall;
		ROS_INFO("Finished adding camera projection %ld (%s) to global mesh %ld triangles in %f seconds\n%f secs per triangle overall\n%f secs (%f per pixel) skip_search\n%f secs (%f per pixel) vertices remove\n%f secs (%f per pixel) constraints remove\n%f secs (%f per pixel) add face", i, cp[i].projection_name.c_str(), pl.points.size()/3, d_overall.toSec(), d_overall.toSec()/(double)(pl.points.size()/3), d_skip_search.toSec(), d_skip_search.toSec()/cnt_skip_search , d_vertices_removed.toSec(), d_vertices_removed.toSec()/cnt_vertices_removed , d_constraints_removed.toSec(), d_constraints_removed.toSec()/cnt_constraints_removed , d_add_face.toSec(), d_add_face.toSec()/cnt_add_face);	
	}

	
	dp.export_points_to_pc(); //export points to triangles
	get_mesh_statistics();


	//dp.cleanup_isolated_vertices();

	//dp.clear_constraints();


	//Compute the projection union of the new mesh
	//compute_projection_union();
	//dp.set_constraint_polygon();


	//get_mesh_statistics();

	//export points to triangles
	//dp.export_points_to_pc();



	return 1;
}

int c_polygon_primitive_with_texture::get_mesh_statistics(void)
{


	std::vector<int> vec;


	for (size_t i=0; i<cp.size()+1; i++)
	{
		vec.push_back(0);	
	}


	for(CDT::Finite_faces_iterator ffi = dp.dt.finite_faces_begin(); ffi != dp.dt.finite_faces_end(); ++ffi)
	{

		int a=ffi->provenience;


		if (a>=-1 && a <(int)cp.size())
			vec.at(a+1) ++;
		else
			ROS_ERROR("There is a face with provenience %d", a);
	}



	char tmp[1024];
	sprintf(tmp, "Polygon %s mesh information:\n mesh has %d vertices and %d triangles\n Total to %d projections:\n", data.misc.name,(int) dp.dt.number_of_vertices(),(int) dp.dt.number_of_faces(),(int) vec.size()); 


	std::string st = tmp;;
	for (size_t i=1; i<vec.size(); i++)
	{

		char tmp[1024];
		sprintf(tmp, "%s: %d faces, %d in mesh\n", cp[i-1].projection_name.c_str(), (int)cp[i-1].mesh.number_of_faces(), vec[i]); 
		st += tmp;

	}


	sprintf(tmp, "Auto created: %d faces in mesh\n",  vec[0]); 
	st += tmp;


	ROS_INFO("%s",st.c_str());	
	return 1;
}

#endif
/**
 *@}
 */      
