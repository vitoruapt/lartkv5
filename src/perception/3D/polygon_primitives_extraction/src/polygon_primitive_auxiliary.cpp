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
 * @brief Auxiliary methods go here. Usually miscelaneous methods.
 * @author Miguel Oliveira
 * @version v0
 * @date 2011-12-19
 */
#ifndef _polygon_primitive_auxiliary_CPP_
#define _polygon_primitive_auxiliary_CPP_

#include "polygon_primitive.h"


int c_polygon_primitive::compute_supporting_perpendicular_plane_ransac( pcl::PointCloud<pcl::PointXYZ> *pc_in,
		pcl::PointCloud<pcl::Normal> *n_in,
		double DistanceThreshold,
		double NormalDistanceWeight,
		int MaxIterations, 
		pcl::PointIndices::Ptr ind_out,
		pcl::ModelCoefficients::Ptr coeff_out
		)
{
	// Create the segmentation objects
	pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg_normals; 
	//Setup segmentation parameters
	seg_normals.setOptimizeCoefficients(true);
	seg_normals.setModelType(pcl::SACMODEL_NORMAL_PARALLEL_PLANE);
	//seg_normals.setModelType(pcl::SACMODEL_NORMAL_PERPENDICULAR_PLANE);
	seg_normals.setMethodType (pcl::SAC_RANSAC); 
	seg_normals.setMaxIterations(MaxIterations+0);
	seg_normals.setNormalDistanceWeight(NormalDistanceWeight);
	seg_normals.setDistanceThreshold(DistanceThreshold);
	seg_normals.setInputCloud(pc_in->makeShared());
	seg_normals.setInputNormals(n_in->makeShared());
	Eigen::Vector3f v;                                                                
	v[0] = 0; v[1] = 0; v[2] = 1;
	seg_normals.setAxis(v);
	seg_normals.setEpsAngle(M_PI*30.0 / 180.0);
	//seg_normals.setDistanceFromOrigin(0);
	//seg_normals.setEpsDist(1000);

	seg_normals.segment(*ind_out, *coeff_out);
	//pcl::SampleConsensusModelNormalParallelPlane<pcl::PointXYZ, pcl::Normal> model(pc_in->makeShared(), indices); 

	//model.setInputCloud(pc_in->makeShared());
	//model.setInputNormals(n_in->makeShared());
	//model.setNormalDistanceWeight(NormalDistanceWeight);

	//Eigen::Vector3f v;                                                                
	//v[0] = 0; v[1] = 0; v[2] = 1;
	//model.setAxis(v);

	//model.setEpsAngle(M_PI*15/180.0);



	//model.setDistanceFromOrigin(0);
	//model.setEpsDist(1000);


	//model.setIndices(indices);

	//Eigen::VectorXf mc; 
	//std::vector<int> indices_out;
	//printf("computing\n");

	////model.computeModelCoefficients(indices, mc);
	//printf("computing done\n");

	//SampleConsensusModelNormalParallelPlane
	//Setup segmentation parameters
	//seg_normals.setOptimizeCoefficients(true);
	//seg_normals.setModelType(pcl::SACMODEL_NORMAL_PARALLEL_PLANE);
	//seg_normals.setMethodType (pcl::SAC_RANSAC); 
	//seg_normals.setMaxIterations(MaxIterations);
	//seg_normals.setNormalDistanceWeight(NormalDistanceWeight);
	//seg_normals.setDistanceThreshold(DistanceThreshold);
	//seg_normals.setDistanceThreshold(DistanceThreshold);

	////Compute  a plane candidate from the point cloud
	//seg_normals.setInputCloud(pc_in->makeShared());
	//seg_normals.setInputNormals(n_in->makeShared());
	//seg_normals.segment(*ind_out, *coeff_out);


	//ROS_INFO("NORMAL PARALLEL PLANE computed supporting planee A=%3.2f B=%3.2f C=%3.2f D=%3.2f supported by %d points",mc[0],  mc[1],mc[2], mc[3],(int)pc_in->size());   

	if (ind_out->indices.size() < 10)
	{
		//ROS_WARN("NORMAL PARALLEL PLANE Could not estimate a planar model for the given dataset. Indices size = %ld", ind_out->indices.size());
		return 0;
	}



	return 1;}


	int c_polygon_primitive::compute_supporting_plane_ransac( pcl::PointCloud<pcl::PointXYZ> *pc_in,
			pcl::PointCloud<pcl::Normal> *n_in,
			double DistanceThreshold,
			double NormalDistanceWeight,
			int MaxIterations, 
			pcl::PointIndices::Ptr ind_out,
			pcl::ModelCoefficients::Ptr coeff_out
			)
{
	// Create the segmentation objects
	pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg_normals; 
	//Setup segmentation parameters
	seg_normals.setOptimizeCoefficients(true);
	seg_normals.setModelType(pcl::SACMODEL_NORMAL_PLANE);
	seg_normals.setMethodType (pcl::SAC_RANSAC); 
	seg_normals.setMaxIterations(MaxIterations);
	seg_normals.setNormalDistanceWeight(NormalDistanceWeight);
	seg_normals.setDistanceThreshold(DistanceThreshold);

	//Compute  a plane candidate from the point cloud
	seg_normals.setInputCloud(pc_in->makeShared());
	seg_normals.setInputNormals(n_in->makeShared());
	seg_normals.segment(*ind_out, *coeff_out);


	//ROS_INFO("computed supporting planee A=%3.2f B=%3.2f C=%3.2f D=%3.2f supported by %d points",coeff_out->values[0],  coeff_out->values[1],coeff_out->values[2],coeff_out->values[3],(int)pc_in->size());   

	if (ind_out->indices.size () < 10)
	{
		ROS_WARN("Could not estimate a planar model for the given dataset.");
		return 0;
	}



	return 1;}

	/**
	 * @brief Extracts points from input pc and copies them to copy_cloud. Does the same with normals
	 *
	 * @param ind
	 * @param input_cloud
	 * @param remove_cloud
	 * @param copy_cloud
	 * @param input_normals
	 * @param remove_normals
	 * @param compute_normals
	 *
	 * @return 
	 */
int c_polygon_primitive::indices_extraction(pcl::PointIndices::Ptr ind,
		pcl::PointCloud<pcl::PointXYZ> *input_cloud,
		pcl::PointCloud<pcl::PointXYZ> *remove_cloud,
		pcl::PointCloud<pcl::PointXYZ>::Ptr copy_cloud,
		pcl::PointCloud<pcl::Normal> *input_normals,
		pcl::PointCloud<pcl::Normal> *remove_normals,
		int compute_normals)
{

	if (input_cloud!=NULL) //if an input cloud is given
	{
		pcl::ExtractIndices<pcl::PointXYZ> extract; //Create the extraction object
		extract.setInputCloud(input_cloud->makeShared());
		extract.setIndices(ind);

		//Copy to copy_cloud if not NULL 
		if(copy_cloud!=NULL)
		{
			extract.setNegative(false);
			extract.filter(*copy_cloud);
		}

		//Remove from remove if not NULL
		if(remove_cloud!=NULL)
		{
			extract.setNegative(true);
			extract.filter(*remove_cloud);
		}
	}

	if (compute_normals==1)
		if (input_normals!=NULL)
		{
			pcl::ExtractIndices<pcl::Normal> extract_normals; //Create the normal extraction object
			extract_normals.setInputCloud(input_normals->makeShared());
			extract_normals.setIndices(ind);

			if (remove_normals!=NULL)
			{
				extract_normals.setNegative(true);
				extract_normals.filter(*remove_normals);
			}
		} 

	return 1;}

	/**
	 * @brief Creates an arbitrary reference frame from a plane equation and two inliers. The Z axis is given by the vector normal to the plane, the X axis is defined by the vector to goes from pt1 to pt2. Y is defined by the external product Z*X.
	 *
	 * @param plane the plane that defines the Z axis
	 * @param pt1 The first point
	 * @param pt2 The second point
	 * @param The output reference frame
	 */
	void c_polygon_primitive::create_reference_frame_from_plane_and_two_points(
			pcl::ModelCoefficients::Ptr plane,
			pcl::PointXYZ *pt1,
			pcl::PointXYZ *pt2,
			t_reference_frame *frame
			)
{
	//STEP1. Project the two points to the plane
	pcl::PointXYZ pt1_projected, pt2_projected;
	project_point_to_plane(pt1, plane, &pt1_projected);
	project_point_to_plane(pt2, plane, &pt2_projected);

	//STEP2. Check if the points are cohincident. If so cannot compute
	if (pt1_projected.x == pt2_projected.x && 
			pt1_projected.y == pt2_projected.y && 
			pt1_projected.z == pt2_projected.z)	
	{
		ROS_ERROR("Cannot create reference frame. pt1 and pt2 after projection are cohincident.");
	}

	//STEP3. find the nsa vectors. a is given by the normal vector to the plane
	double a[3] = {plane->values[0],plane->values[1],plane->values[2]};

	//STEP4. Find the nsa vectors. the n vector will be the vector from pt1_projected to pt2_projected 
	double n[3]={pt2_projected.x - pt1_projected.x, pt2_projected.y - pt1_projected.y, pt2_projected.z - pt1_projected.z};

	//STEP5. Find the nsa vectors. The s vector is given by the external product a*n
	double s[3]={a[1]*n[2] - a[2]*n[1],	a[2]*n[0] - a[0]*n[2], a[0]*n[1] - a[1]*n[0]};

	//STEP6. Normalize all vectors
	normalize_vector(n); normalize_vector(s); normalize_vector(a);

	//STEP7. Compute the transform. The orientation given by nsa and the origin by pt1
	frame->transform = tf::Transform(tf::Matrix3x3(n[0], s[0] , a[0],
				n[1], s[1] , a[1],  
				n[2], s[2] , a[2]),  
			tf::Vector3(pt1_projected.x,
				pt1_projected.y,
				pt1_projected.z));

	compute_arrow_points_from_transform(frame);
}

/**
 * @brief Projects all the points in a point cloud ptin to the plane defined by coeff, and writes the result to ptout
 *
 * @param ptin the input point cloud
 * @param coeff the projection plane
 * @param ptout the output point cloud
 */
void c_polygon_primitive::project_point_to_plane(const pcl::PointXYZ *ptin, const pcl::ModelCoefficients::Ptr coeff, pcl::PointXYZ *ptout)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcin = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
	pcin->points.push_back(*ptin);

	pcl::PointCloud<pcl::PointXYZ>::Ptr pcout = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
	//
	/// from http://www.9math.com/book/projection-point-plane	
	//float a=coeff->values[0];
	//float b=coeff->values[1];
	//float c=coeff->values[2];
	//float d=coeff->values[3];

	//float u=ptin->x;
	//float v=ptin->y;
	//float w=ptin->z;

	//float t0 = (a*u + b*v + c*w + d)/(a*a+b*b+c*c);
	//PFLN

	//ptout->x = u - a*t0;
	//ptout->y = v - b*t0;
	//ptout->z = w - c*t0;


	//float val = (u-a*t0)*a + (v-b*t0)*b + (w-c*t0)*c +d;
	//printf("new val=%f\n",val);
	//printf("coeff A=%f B=%f C=%f D=%f\n", coeff->values[0], coeff->values[1],coeff->values[2],coeff->values[3]);

	//printf("coeff sqrt(A2+B2+C2)=%f\n", sqrt(coeff->values[0]*coeff->values[0] + coeff->values[1]*coeff->values[1] +coeff->values[2]*coeff->values[2]));
	//PFLN
	//Create the projection object
	pcl::ProjectInliers<pcl::PointXYZ> projection;
	projection.setModelType(pcl::SACMODEL_PLANE); //set model type
	//projection.setModelType(pcl::SACMODEL_NORMAL_PARALLEL_PLANE); //set model type

	projection.setInputCloud(pcin);
	projection.setModelCoefficients(coeff);
	projection.filter(*pcout);

	*ptout = pcout->points[0];

	if(!check_if_point_lies_on_plane(coeff, ptout))
	{
		//PFLN
		ROS_ERROR("Error in projection of point. \nPoint x=%f y=%f z=%f does not lie on plane A=%f B=%f C=%f D=%f",ptout->x, ptout->y, ptout->z, coeff->values[0], coeff->values[1], coeff->values[2], coeff->values[3]);
		exit(0);
	}

	pcin.reset();
	pcout.reset();
}

/**
 * @brief Bollean test to assess if a point lies on a plane
 *
 * @param plane The definition of the plane
 * @param point the point
 *
 * @return true if yes, false if the point does not lie on the plane
 */
bool c_polygon_primitive::check_if_point_lies_on_plane(const pcl::ModelCoefficients::Ptr plane, const pcl::PointXYZ *p)
{
	double val = plane->values[0]*p->x + plane->values[1]*p->y + plane->values[2]*p->z + plane->values[3];	


	if (val> -0.001 && val < 0.001) //Dont know why this happens. Floating point precision?
		return true;
	else
	{
		//ROS_INFO("val = %f",val);
		return false;
	}
}

/**
 * @brief Checks the the normal vector of a plane at point ptin is propperly oriented according to the viewpoint vx, vy, vz
 *
 * @param plane The plane from where the normal vector is extracted
 * @param ptin The point where the test is made
 * @param vx The x coordinate of the viewpoint
 * @param vy The y coordinate of the viewpoint
 * @param vz The z coordinate of the viewpoint
 *
 * @return 1 if ok, 0 if error 
 */
int c_polygon_primitive::check_plane_normal_orientation(pcl::ModelCoefficients::Ptr plane, pcl::PointXYZ *point, double vx, double vy, double vz)
{	
	//Receive a plane equation in Hessian form Ax+By+Cz+D=0
	//And a point xyz
	//And an arbitrary viewpoint vx vy vz
	//Must check if the Normal is oriented towards the viewpoint. If not reorient it

	//STEP1. Backup the plane coefficients
	pcl::ModelCoefficients::Ptr plane_old; //create a backup struct
	plane_old = pcl::ModelCoefficients::Ptr(new pcl::ModelCoefficients);
	plane_old->values.resize(4); //allocate mem
	copy(plane, plane_old); //copy to the backup

	//STEP2. Obtain the projection of point onto the plane

	pcl::PointXYZ point_projected;
	project_point_to_plane(point, plane, &point_projected);


	//STEP3. Flip normal if necessary
	pcl::flipNormalTowardsViewpoint (point_projected, vx, vy, vz, plane->values[0], plane->values[1], plane->values[2]);

	//STEP4. If there was a flip must recompute parameter D
	if (plane_old->values[0]!=plane->values[0] ||
			plane_old->values[1]!=plane->values[1] ||
			plane_old->values[2]!=plane->values[2])
	{
		//ROS_WARN("Normal was fliped!");

		//Rcompute the new plane->values[3] using D = -Ax-By-Cz 
		plane->values[3] = -plane->values[0]*point_projected.x 
			-plane->values[1]*point_projected.y 
			-plane->values[2]*point_projected.z; 

		//ROS_INFO("Before flip A=%f B=%f C=%f D=%f Non-projected point x=%f y=%f z=%f lies on plane %d", plane->values[0], plane->values[1], plane->values[2], plane->values[3], point->x, point->y, point->z, check_if_point_lies_on_plane(plane, point));	
		//ROS_INFO("Before flip A=%f B=%f C=%f D=%f Projected point x=%f y=%f z=%f lies on plane %d", plane->values[0], plane->values[1], plane->values[2], plane->values[3], point_projected.x, point_projected.y, point_projected.z, check_if_point_lies_on_plane(plane, &point_projected));	
	}
	else
	{
		//ROS_INFO("Normal was NOT fliped!");
	}

	plane_old.reset();
	return 1;}


	/**
	 * @brief Computes the 3D coordinates of the arrows of a given reference frame
	 *
	 * @param frame the reference frame
	 */
void c_polygon_primitive::compute_arrow_points_from_transform(t_reference_frame *frame)
{
	pcl::PointCloud<pcl::PointXYZ> pc_in;
	pcl::PointCloud<pcl::PointXYZ> pc_out;

	pcl::PointXYZ p;
	double arrow_size = 1.5;

	p.x = arrow_size ; p.y=0; p.z=0; 
	pc_in.points.push_back(p);
	p.x = 0; p.y=arrow_size; p.z=0; 
	pc_in.points.push_back(p);
	p.x = 0; p.y=0; p.z=arrow_size; 
	pc_in.points.push_back(p);
	p.x = 0; p.y=0; p.z=0; 
	pc_in.points.push_back(p);

	//transform_pc(pc_in, pc_out, &frame->transform);
	pcl_ros::transformPointCloud(pc_in, pc_out,  frame->transform);

	frame->arrow_x.x = pc_out.points[0].x;
	frame->arrow_x.y = pc_out.points[0].y;
	frame->arrow_x.z = pc_out.points[0].z;

	frame->arrow_y.x = pc_out.points[1].x;
	frame->arrow_y.y = pc_out.points[1].y;
	frame->arrow_y.z = pc_out.points[1].z;

	frame->arrow_z.x = pc_out.points[2].x;
	frame->arrow_z.y = pc_out.points[2].y;
	frame->arrow_z.z = pc_out.points[2].z;

	frame->origin.x = pc_out.points[3].x;
	frame->origin.y = pc_out.points[3].y;
	frame->origin.z = pc_out.points[3].z;

}









/**
 * @brief recomputes the plane coefficients of the plane that best fits in the minimum square distance sense all the points in pcin
 *
 * @param pcin the input point cloud
 * @param coeff the output refined plane coefficients
 */

void c_polygon_primitive::refine_plane_coefficients(pcl::PointCloud<pcl::PointXYZ>::Ptr pcin, pcl::ModelCoefficients::Ptr coeff)
{
	//Create the indices object
	pcl::PointIndices::Ptr indices = pcl::PointIndices::Ptr(new pcl::PointIndices); 
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;

	//Setup segmentation parameters
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC); 
	seg.setMaxIterations (50);
	seg.setDistanceThreshold(99999); //with this huge value all points are considered
	seg.setInputCloud(pcin->makeShared());

	//Compute  a plane candidate from the point cloud
	seg.segment(*indices, *coeff);

	indices.reset(); //free the indices object
}

/**
 * @brief Projects all the points in pcin to a plane (along the normal to that plane) defined by coeff and stores the projected points in pcout
 *
 * @param pcin the point cloud in
 * @param coeff the plane coefficients
 * @param pcout the point cloud out
 */
void c_polygon_primitive::project_pc_to_plane(pcl::PointCloud<pcl::PointXYZ>::Ptr pcin, pcl::ModelCoefficients::Ptr coeff, pcl::PointCloud<pcl::PointXYZ>::Ptr pcout)
{
	//Create the projection object
	pcl::ProjectInliers<pcl::PointXYZ> projection;

	projection.setModelType(pcl::SACMODEL_NORMAL_PLANE); //set model type
	projection.setInputCloud(pcin);
	projection.setModelCoefficients(coeff);
	projection.filter(*pcout);
}

/**
 * @brief Sets the polygon name
 *
 * @param s the string with the name
 */
void c_polygon_primitive::set_names(const char* s)
{ 
	sprintf(data.misc.name,"%s",s);
}

/**
 * @brief Sets the reference systems
 */
void c_polygon_primitive::set_reference_systems(void)
{
	pointclouds.all->header.frame_id = data.frames.global_name;
	pointclouds.additional->header.frame_id = data.frames.global_name; 
	pointclouds.projected->header.frame_id = data.frames.global_name; 
	data.hulls.convex.polygon->header.frame_id = data.frames.global_name; 
	data.hulls.convex.extended_polygon->header.frame_id = data.frames.global_name; 
	data.hulls.concave.polygon->header.frame_id = data.frames.global_name; 
	data.hulls.concave.extended_polygon->header.frame_id = data.frames.global_name; 
	pointclouds.growed->header.frame_id = data.frames.global_name; 
	pointclouds.tmp->header.frame_id = data.frames.global_name; 
}

void c_polygon_primitive::allocate_space(void)
{
	pointclouds.all= pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
	pointclouds.projected= pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
	pointclouds.additional = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
	data.hulls.convex.polygon = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
	data.hulls.convex.extended_polygon = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
	data.hulls.concave.polygon = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
	data.hulls.concave.extended_polygon = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
	pointclouds.growed = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
	pointclouds.tmp = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

	data.planes.current = pcl::ModelCoefficients::Ptr(new pcl::ModelCoefficients);
	data.planes.current->values.resize(4);
	data.planes.previous = pcl::ModelCoefficients::Ptr(new pcl::ModelCoefficients);
	data.planes.previous->values.resize(4);
}

/**
 * @brief Normalizes a vector
 *
 * @param v the vector to normalize v[0]=x , v[1]=y, v[2]=z
 */
void c_polygon_primitive::normalize_vector(double *v)
{

	double n = sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);

	v[0] = v[0]/n;
	v[1] = v[1]/n;
	v[2] = v[2]/n;

}

int c_polygon_primitive::print_polygon_information(void)
{
	ROS_INFO("%s INFORMATION\n \
			Color rgb [%d,%d,%d]\n \
			Support plane is A=%3.2f B=%3.2f C=%3.2f D=%3.2f \n \
			Number of support points %d \n \
			Convex Hull with %d points, Area of %3.2f and Solidity of %3.2f \n \
			Concave Hull with %d points, Area of %3.2f and Solidity of %3.2f \n \
			", data.misc.name, data.misc.color.r, data.misc.color.g, data.misc.color.b,  data.planes.current->values[0],  data.planes.current->values[1],data.planes.current->values[2],data.planes.current->values[3],(int)pointclouds.all->size(), (int)data.hulls.convex.polygon->points.size(),data.hulls.convex.area, data.hulls.convex.solidity,(int) data.hulls.concave.polygon->points.size(),0.,0.);

	//ROS_INFO("Transform T-1 Origin = [%3.4f, %3.4f, %3.4f]",
			//(data.frames.previous.transform.getOrigin()).x(),
			//(data.frames.previous.transform.getOrigin()).y(),
			//(data.frames.previous.transform.getOrigin()).z()
			//);

	//ROS_INFO("Transform T-1 Quat = [%3.4f, %3.4f, %3.4f, %3.4f]",
			//(data.frames.previous.transform.getRotation()).x(),
			//(data.frames.previous.transform.getRotation()).y(),
			//(data.frames.previous.transform.getRotation()).z(),
			//(data.frames.previous.transform.getRotation()).w()
			//);

	//ROS_INFO("Transform T Origin = [%3.4f, %3.4f, %3.4f]",
			//(data.frames.current.transform.getOrigin()).x(),
			//(data.frames.current.transform.getOrigin()).y(),
			//(data.frames.current.transform.getOrigin()).z()
			//);

	//ROS_INFO("Transform T Quat = [%3.4f, %3.4f, %3.4f, %3.4f]",
			//(data.frames.current.transform.getRotation()).x(),
			//(data.frames.current.transform.getRotation()).y(),
			//(data.frames.current.transform.getRotation()).z(),
			//(data.frames.current.transform.getRotation()).w()
			//);

	return 1;
}


#endif
/**
 *@}
 */      
