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
 * @brief Implements the high lever polygon primitive operations such as create, expand, split, etc.
 * @author Miguel Oliveira
 * @version v0
 * @date 2011-12-19
 */
#ifndef _polygon_primitive_operations_CPP_
#define _polygon_primitive_operations_CPP_


#include "polygon_primitive.h"

/**
 * @brief Computes a polygon primitive candidate from an input point cloud and its normals
 *
 * @param input_cloud the point cloud in
 * @param input_normals the estimated normals of each point
 * @param DistanceThreshold the distance used in the ransac method
 * @param NormalDistanceWeight the distance used in the ransac method for the normals 
 * @param MaxIterations Maximum number of ransac iterations 
 * @param do_spatial_division if true, after ransac generates a candidate, an euclidian cluster is performed in order to select the biggest cluster
 *
 * @return 1 if polygon created, 0 if some error occurred
 */
int c_polygon_primitive::polygon_create(pcl::PointCloud<pcl::PointXYZ> *input_cloud,
		pcl::PointCloud<pcl::Normal> *input_normals, 
		double DistanceThreshold,
		double NormalDistanceWeight,
		int MaxIterations,
		int do_spatial_division)
{

	ros::Time start_tic = ros::Time::now();
	data.hulls.convex.area = 0;
	data.hulls.convex.solidity = 0;
	int polygon_created_correctly=true;
	//size_t num_start_points = input_cloud->size();

	
	//Set the name of the global reference system, the same as the one defined by the input cloud
	data.frames.global_name = input_cloud->header.frame_id; 
	set_reference_systems();

	
	//-------------------------------------
	//Define some internal variables
	//-------------------------------------
	//int num_original_pts = (int)input_cloud->size();
	
	
	//int num_candidate_plane_points=0;
	int j=0, max_cluster=0, count=0;
	int num_clusters=0;
	pcl::PointIndices::Ptr indices = pcl::PointIndices::Ptr(new pcl::PointIndices); 

	
	//-------------------------------------
	// Define the frame_id for all pc
	//-------------------------------------
	
	data.frames.global_name = input_cloud->header.frame_id; 
	pointclouds.all->header.frame_id = data.frames.global_name;
	data.hulls.convex.polygon->header.frame_id = data.frames.global_name;
	pointclouds.additional->header.frame_id = data.frames.global_name;

	
	//-------------------------------------
	// Find a supporting plane using ransac
	//-------------------------------------

	if (do_spatial_division==0)
	{

	
		//ROS_INFO("Computing a supporting plane ...");
		if (!compute_supporting_plane_ransac( input_cloud,
					input_normals,
					DistanceThreshold,
					NormalDistanceWeight,
					MaxIterations, 
					indices,
					data.planes.current
					))
		{
			ROS_WARN("polygon create: cannot create polygon"); return 0;
			polygon_created_correctly=false;
		}
		//ROS_INFO("Computing a supporting plane ...done");
		//
	
	}
	else
	{

	
		//ROS_INFO("Computing a PARALELL supporting plane ...");
		if (!compute_supporting_perpendicular_plane_ransac( input_cloud,
					input_normals,
					DistanceThreshold,
					NormalDistanceWeight,
					MaxIterations, 
					indices,
					data.planes.current
					))
		{
			ROS_WARN("PARALELL polygon create: cannot create polygon"); return 0;
			polygon_created_correctly=false;
		}
		//ROS_INFO("Computing a PARALELL supporting plane ...done");
	
	}

	//-------------------------------------
	// Extract plane supporting points from input point cloud
	//-------------------------------------
	
	indices_extraction(indices,
			input_cloud,
			input_cloud,
			pointclouds.all,
			input_normals,
			input_normals);

	
	//num_candidate_plane_points = (int)pointclouds.all->size();

	
	//-------------------------------------
	// PERFORM EUCLIDIAN CLUSTER OPTIMIZATION IF REQUIRED
	//-------------------------------------
	if (do_spatial_division==1 && polygon_created_correctly==true)
	{
		//polygon_split(pointclouds.all- input_cloud->makeShared(), data.planes.current);

		//Set the values of some control variables
		j=0; max_cluster=0; count=0; num_clusters=0;

		// Creating the KdTree object for the search method of the extraction 
		//pcl::KdTree<pcl::PointXYZ>::Ptr tree1 (new pcl::KdTreeFLANN<pcl::PointXYZ>);
		//pcl::EuclideanClusterExtraction<pcl::PointXYZ>::KdTreePtr tree1 (new pcl::KdTreeFLANN<pcl::PointXYZ>);
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree1 (new pcl::search::KdTree<pcl::PointXYZ>);
		//pcl::search::Search<pcl::PointXYZ>::Ptr tree1 (new pcl::search::Search<pcl::PointXYZ>);
		//KdTreePtr<pcl::PointXYZ> tree1 (new KdTree<pcl::PointXYZ>);
//		boost::shared_ptr::pcl::search::KdTree<pcl::PointXYZ> tree1 (new pcl::search::KdTree<pcl::PointXYZ>);
		//pcl::KdTree<pcl::PointXYZ>::Ptr tree1 (new pcl::KdTree<pcl::PointXYZ>);
//		pcl::KdTree<pcl::PointXYZ> tree1;
		pcl::PointCloud<pcl::PointXYZ> cluster;
		tree1->setInputCloud(pointclouds.all->makeShared());

		std::vector<pcl::PointIndices> cluster_indices;
		pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
		ec.setClusterTolerance(0.9); 
		ec.setMinClusterSize(1);
		ec.setMaxClusterSize((unsigned int)pointclouds.all->size());
		ec.setSearchMethod(tree1);
		ec.setInputCloud(pointclouds.all->makeShared());
		ec.extract(cluster_indices);

		
	
		//Find the largest cluster
		for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
		{
			int t=0;
			for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
			{
				t++;
			}
			//ROS_INFO("Cluster %d has %d points",j,t);
			if(t>count) //select a new max
			{
				max_cluster = j; 
				count = t;
			}
			j++;
		}


	
		j=0;
		pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud;
		tmp_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

	
		for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
		{
			if (j==max_cluster)
			{
				tmp_cloud->width=0;
				for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
				{
					tmp_cloud->width++;
					tmp_cloud->points.push_back(pointclouds.all->points[*pit]);
				}
			}
			else
			{
				//for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
				//{
				//input_cloud->width++;
				//input_cloud->points.push_back(pointclouds.all->points[*pit]);
				//}
			}
			j++;
		} 
		num_clusters=j;

	
		(*pointclouds.all) = (*tmp_cloud); //pointclouds.all-now has only largest cluster



	
		//refine_plane_coefficients(pointclouds.all- data.planes.current);
		//ROS_INFO("OLA plane A=%3.2f B=%3.2f C=%3.2f D=%3.2f ",data.planes.current->values[0],  data.planes.current->values[1],data.planes.current->values[2],data.planes.current->values[3]);   
		double fit_qual = fit_plane_to_pc(pointclouds.all, data.planes.current);
		fit_qual=fit_qual;

	
		//ROS_INFO("fit_qual=%f OLA2 plane A=%3.2f B=%3.2f C=%3.2f D=%3.2f ",fit_qual, data.planes.current->values[0],  data.planes.current->values[1],data.planes.current->values[2],data.planes.current->values[3]);   


	
		double fit_qual2 = fit_plane_to_two_pc_and_ratio(pointclouds.all, pointclouds.all, 0.5, data.planes.current);
		fit_qual2=fit_qual2;
		//ROS_INFO("fit_qual=%f OLA3 plane A=%3.2f B=%3.2f C=%3.2f D=%3.2f ",fit_qual2, data.planes.current->values[0],  data.planes.current->values[1],data.planes.current->values[2],data.planes.current->values[3]);   

	}


	
	if (polygon_created_correctly==true)
	{
		check_plane_normal_orientation(data.planes.current, &pointclouds.all->points[0], 100000, 100000, 100000);
		//-------------------------------------
		//obtain the convex hull of the supporting points
		//-------------------------------------
		//
		//
		//
		create_reference_frame_from_plane_and_two_points(data.planes.current,
				&pointclouds.all->points[0],
				&pointclouds.all->points[1],
				&data.frames.current);
		//compute_convex_hull(pointclouds.all-data.planes.current, data.hulls.convex.polygon);
		compute_convex_hull_cgal(pointclouds.all, data.planes.current,  data.hulls.convex.polygon, &data.frames.current.transform, data.hulls.convex.area, data.hulls.convex.solidity,false);
		pointclouds.projected->header.frame_id = "/world";
		//printf("CONVEX HULL POLYGON has %d points\n",(int) data.hulls.convex.polygon->points.size());
		//pcl::PointCloud<pcl::PointXYZ> pc_local; 
		//pcl_ros::transformPointCloud(*data.hulls.convex.polygon, pc_local,  reference_frame.transform.inverse());

		//printf("CONVEX HULL LOCAL POLYGON has %d points\n",(int) pc_local.points.size());
		//for (size_t i=0; i<data.hulls.convex.polygon->points.size(); i++)
		//{
		//printf("pt[%d] x=%3.2f y=%3.2f z=%3.2f  local x=%3.2f y=%3.2f z=%3.6f\n",(int)i, data.hulls.convex.polygon->points[i].x, data.hulls.convex.polygon->points[i].y, data.hulls.convex.polygon->points[i].z, pc_local.points[i].x, pc_local.points[i].y, pc_local.points[i].z);
		//}


		//if (do_spatial_division==1)
			//compute_concave_hull(pointclouds.all, data.hulls.concave.polygon, data.planes.current);

		//-------------------------------------
		////define a local reference system and compute the transform to it
		//-------------------------------------
		//create_reference_system_from_convex_hull(data.planes.current, data.hulls.convex.polygon,&reference_frame);


		offset_polygon(.1, data.hulls.convex.polygon, data.hulls.convex.extended_polygon, &data.frames.current.transform);

		//Copy to grow and old frame
		copy(&data.frames.current, &data.frames.previous);
	}


	
	ros::Duration time1 = ros::Time::now() - start_tic;
#if 0

	ROS_INFO("%s create in %3.2f seconds\n \
			Creation %s\n \
			Color rgb [%d,%d,%d]\n \
			Initial point cloud with %d points, after with %d points\n \
			Using Euclidean cluster extraction: %s\n \
			Divided into %d clusters. Largest %d with %d points\n \
			Support plane is A=%3.2f B=%3.2f C=%3.2f D=%3.2f \n \
			Number of support points %d \n \
			Convex Hull with %d points, Area of %3.2f and Solidity of %3.2f \n \
			Concave Hull with %d points, Area of %3.2f and Solidity of %3.2f \n \
			", data.misc.name, time1.toSec(),  (polygon_created_correctly==true)?"successfully":"failure. No ransac candidate output",data.misc.color.r, data.misc.color.g, data.misc.color.b, (int)num_start_points, (int)input_cloud->size(), (do_spatial_division==true)?"yes":"no", num_clusters, max_cluster, count, data.planes.current->values[0],  data.planes.current->values[1],data.planes.current->values[2],data.planes.current->values[3],(int)pointclouds.all->size(), (int)data.hulls.convex.polygon->points.size(),data.hulls.convex.area, data.hulls.convex.solidity,(int) data.hulls.concave.polygon->points.size(),0.,0.);


#endif


	indices.reset(); //free the indices object

	if (polygon_created_correctly==true)
		return 1;
	else
		return 0;
}

/**
 * @brief Grows polygon by adding points from the input pc that are close to the plane and inside the convex hull. First perfoms a longitudinal expansion of the convex hull, and then selects points which are in the prisma defined by the perpeddicular -/+ offset of the extended convex hull.
 *
 * @param input_cloud the point cloud in, from which the polygon can expand
 * @param longitudinal_offset the offset used to expand the convex hull
 * @param perpendicular_offset the offset used to compute the prisma
 *
 * @return 1 if expansion occurred, 0 if not
 */
int c_polygon_primitive::polygon_expansion(pcl::PointCloud<pcl::PointXYZ> *input_cloud,
		double longitudinal_offset,
		double perpendicular_offset,
		pcl::PointCloud<pcl::Normal>* input_normals) 
{
	ros::Time start_tic = ros::Time::now();
	int num_support_points_before_expansion = (int)pointclouds.all->points.size();
	bool significative_expansion=false;
	int old_convex_hull_num_pts = (int)data.hulls.convex.polygon->points.size();
	double old_convex_hull_area = data.hulls.convex.area;
	double old_convex_hull_solidity = data.hulls.convex.solidity;

	//Erase the points from the polygon expanded pc
	pointclouds.growed->points.erase(pointclouds.growed->points.begin(),
			pointclouds.growed->points.end()); 	

	int num_expansions=0;
	int flag_keep_expanding=1;
	// --------------------------------	
	//Expansion Cycle
	// --------------------------------	
	ros::Time t = ros::Time::now();
	while (flag_keep_expanding)
	{
		flag_keep_expanding = _polygon_expansion(input_cloud, longitudinal_offset, perpendicular_offset, input_normals);
		num_expansions++;	
	}

	if ((int)pointclouds.growed->points.size()==0)//no expansion occurred
	{
		significative_expansion=false;
	}
	else //An expansion occurred
	{
		significative_expansion=true;
		// ----------------------------------------------
		//Now the new reference system must be calculated
		// ----------------------------------------------
		//First, we backup the reference frame and plane to the old
		copy(&data.frames.current, &data.frames.previous); 
		copy(data.planes.current, data.planes.previous); 

		//Compute the ratio for interpolation of the new reference system, as division of the number of points already supported by the polygon and the new growed points
		double ratio = (double)pointclouds.growed->points.size() /
			((double)pointclouds.growed->points.size() +(double)pointclouds.all->points.size());

		double fit_qual3 = fit_plane_to_two_pc_and_ratio(pointclouds.all, pointclouds.growed, ratio, data.planes.current);
		fit_qual3=fit_qual3;

		project_pc_to_plane(pointclouds.all, data.planes.current, pointclouds.projected);

		create_reference_frame_from_plane_and_two_points(data.planes.current,
				&pointclouds.all->points[0],
				&pointclouds.all->points[1],
				&data.frames.current);



		//and finally, obtain a new ch by projecting the convex hull to the new plane
		compute_convex_hull_cgal(data.hulls.convex.polygon,data.planes.current,  data.hulls.convex.polygon, &data.frames.current.transform);

		//for now add to the all pc the growed pc
		pointclouds.growed->header.frame_id = pointclouds.all->header.frame_id;  
		(*pointclouds.all) += (*pointclouds.growed);

		//if (strcmp(data.misc.name,"p0"))
			//compute_concave_hull(pointclouds.all, data.hulls.concave.polygon, data.planes.current);

	}

	ros::Duration time1 = ros::Time::now() - start_tic;

#if 1

	ROS_INFO("%s Expansion in %3.2f seconds\n \
			Expansion point cloud with %d points, after with %d points\n \
			Expansion params: longitudinal offset=%3.2f perpendicular offset=%3.2f \n \
			Polygon expanded %d times to a total of %d points \n \
			Number of support points at T-1 %d, at T %d \n \
			Support plane was recomputed: %s \n \
			Support plane at T-1 A=%3.2f B=%3.2f C=%3.2f D=%3.2f \n \
			Support plane at T A=%3.2f B=%3.2f C=%3.2f D=%3.2f \n \
			Convex Hull at T-1 with %d points, Area of %3.2f and Solidity of %3.2f \n \
			Convex Hull at T with %d points, Area of %3.2f and Solidity of %3.2f \n \
			Concave Hull with %d points, Area of %3.2f and Solidity of %3.2f \n \
			", data.misc.name, time1.toSec(),  (int)num_support_points_before_expansion, (int)pointclouds.all->points.size(), longitudinal_offset, perpendicular_offset, num_expansions, (int)pointclouds.growed->points.size(), num_support_points_before_expansion, (int)pointclouds.all->size(), significative_expansion?"yes":"no", data.planes.previous->values[0],  data.planes.previous->values[1],data.planes.previous->values[2],data.planes.previous->values[3],data.planes.current->values[0],  data.planes.current->values[1],data.planes.current->values[2],data.planes.current->values[3],old_convex_hull_num_pts, old_convex_hull_area, old_convex_hull_solidity, (int)data.hulls.convex.polygon->points.size(),data.hulls.convex.area, data.hulls.convex.solidity,(int) data.hulls.concave.polygon->points.size(),0.,0.);


#endif
	return 1;}

	/**
	 * @brief Internal function to perform single step polygon expansion
	 *
	 * @param input_cloud the input cloud to from which the poligon primitive may expand
	 * @param longitudinal_offset the longitudinal offset threshold, i.e., in the local XoY plane, how much should the convex hull polygon expand
	 * @param perpendicular_offset the perpendicular offset threshold, i.e., in the local Z axis, how much should the polygon expansion zone be extruded. The functio will use the negative and positive value of perpendicular offset tow expan in +Z and -Z.
	 *
	 * @return 1 if an expansion occurred, 0 if no expansion occurred.
	 */
int c_polygon_primitive::_polygon_expansion(pcl::PointCloud<pcl::PointXYZ> *input_cloud,
		double longitudinal_offset,
		double perpendicular_offset,
		pcl::PointCloud<pcl::Normal>* input_normals) 
{
	//-------------------------------------
	//offset the ch polygon
	//-------------------------------------
	offset_polygon(longitudinal_offset, data.hulls.convex.polygon, data.hulls.convex.extended_polygon, &data.frames.current.transform);

	//Create variables
	pcl::ExtractPolygonalPrismData<pcl::PointXYZ> epp; 
	pcl::PointCloud<pcl::PointXYZ>::ConstPtr input_cloud_constptr; 
	input_cloud_constptr.reset (new pcl::PointCloud<pcl::PointXYZ> (*input_cloud));
	pcl::PointCloud<pcl::PointXYZ>::ConstPtr convex_hull_constptr;
	convex_hull_constptr.reset (new pcl::PointCloud<pcl::PointXYZ> (*data.hulls.convex.extended_polygon));
	pcl::PointIndices::Ptr ind =  pcl::PointIndices::Ptr(new pcl::PointIndices);
	pcl::ExtractIndices<pcl::PointXYZ> extract; //Create the extraction object
	pcl::PointIndices::Ptr indices;
	indices.reset();
	indices = pcl::PointIndices::Ptr(new pcl::PointIndices); 

	//Set epp parameters
	epp.setInputCloud(input_cloud_constptr);
	epp.setInputPlanarHull(convex_hull_constptr);
	epp.setHeightLimits(-perpendicular_offset, perpendicular_offset); 
	epp.setViewPoint(10000,10000,10000); 

	//Segment
	epp.segment(*ind);

	indices_extraction(ind,
			input_cloud,
			input_cloud,
			pointclouds.additional,
			input_normals,
			input_normals);


	//add to growed point cloud


	pointclouds.growed->header.frame_id = data.frames.global_name;
	pointclouds.all->header.frame_id = data.frames.global_name;  
	(*pointclouds.growed) += (*pointclouds.additional);


	if ((int)pointclouds.additional->size() == 0) //very little expansion
	{ 
		indices.reset(); //free the indices object
		return 0; //no expansion occurred
	}

	//-------------------------------------
	//obtain the projection of the new expanded points on the plane
	//-------------------------------------

	//erase all from tmp cloud
	pointclouds.tmp->header.frame_id = data.frames.global_name;
	data.hulls.convex.polygon->header.frame_id = data.frames.global_name;
	pointclouds.growed->header.frame_id = data.frames.global_name;

	(*pointclouds.tmp) = (*data.hulls.convex.polygon);

	(*pointclouds.tmp) += (*pointclouds.growed);


	//-------------------------------------
	//obtain the convex hull of the old ch plus the additional projected points
	//-------------------------------------
	compute_convex_hull_cgal(pointclouds.tmp,data.planes.current,  data.hulls.convex.polygon, &data.frames.current.transform);

	return 1; //grow operation expanded the polygon
}

void c_polygon_primitive::polygon_split(pcl::PointCloud<pcl::PointXYZ>::Ptr pcin, pcl::PointCloud<pcl::PointXYZ>::Ptr pcdumpster, pcl::ModelCoefficients::Ptr coeff)
{
	//Set the values of some control variables
	int j=0, max_cluster=0, count=0, num_clusters=0;

	// Creating the KdTree object for the search method of the extraction 
	//pcl::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ>);
	//pcl::EuclideanClusterExtraction<pcl::PointXYZ>::KdTreePtr tree (new pcl::KdTreeFLANN<pcl::PointXYZ>);

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ> cluster;
	tree->setInputCloud(pcin->makeShared());

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance(1.0); 
	ec.setMinClusterSize(1);
	ec.setMaxClusterSize((unsigned int)pcin->size());
	ec.setSearchMethod(tree);
	ec.setInputCloud(pcin->makeShared());
	ec.extract(cluster_indices);

	//Find the largest cluster
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
	{
		int t=0;
		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
		{
			t++;
		}
		//ROS_INFO("Cluster %d has %d points",j,t);
		if(t>count) //select a new max
		{
			max_cluster = j; 
			count = t;
		}
		j++;
	}


	j=0;
	pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
	{
		if (j==max_cluster)
		{
			tmp_cloud->width=0;
			for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
			{
				tmp_cloud->width++;
				tmp_cloud->points.push_back(pcin->points[*pit]);
			}
		}
		else
		{
			for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
			{
				pcdumpster->width++;
				pcdumpster->points.push_back(pcin->points[*pit]);
			}
		}
		j++;
	} 
	num_clusters=j;

	(*pcin) = (*tmp_cloud); //pcin now has only largest cluster. The actual split operation
	tmp_cloud.reset(); //free tmp_cloud

	refine_plane_coefficients(pcin, coeff); //now recompute plane coefficients

	ROS_INFO("Performing Euclidian cluster extraction optimization:");
	ROS_INFO("Largest cluster is %d with %d points (%d clusters)",max_cluster,count, num_clusters);
	ROS_INFO("New support plane A=%3.2f B=%3.2f C=%3.2f D=%3.2f supported by %d points",data.planes.current->values[0],  data.planes.current->values[1],data.planes.current->values[2],data.planes.current->values[3],(int)pointclouds.all->size());   

}


#endif
/**
 *@}
 */
/*Previous 3 lines appended automatically on Sun Feb  5 19:40:16 WET 2012 */
