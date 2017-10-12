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
 * @addtogroup pinhole_projection
 * @file
 * @brief Performs pinhole model projection. Is also capable of doing spherical
 * model undistortion as used in MIT dataset.
 *@{
 */

#ifndef _CLASS_PINHOLE_PROJECTION_CPP_
#define _CLASS_PINHOLE_PROJECTION_CPP_

#include "class_pinhole_projection.h"




/** 
 * @brief Sets the values of the projection plane. Values of the plane equation ax + by + cz + d =0
 * @return _RETURN_SUCCESS_
 */
int class_pinhole_projection::set_projection_plane(double a, double b, double c, double d)
{
	projection_plane.a = a;
	projection_plane.b = b;
	projection_plane.c = c;
	projection_plane.d = d;
	return 1;}

/**
 * @brief General printing function of a CvMat
 * @param mat the CvMat matrix to print
 * @param name optional name of the matrix to print
 */
int class_pinhole_projection::print_CvMat(cv::Mat *mat, const char* name)
{

	if (name==NULL)
	{
		printf("cvMat has %d cols and %d lines\n", mat->cols, mat->rows);
	}
	else
	{
		printf("%s has %d cols and %d lines\n",name, mat->cols, mat->rows);
	}

	for (int l=0; l<mat->rows; l++)
	{
		printf("[");
		for (int c=0; c<mat->cols; c++)
		{
			printf("%3.2f ", mat->at<double>(l,c));
		}
		printf("]\n");
	}

	return 1;}

int class_pinhole_projection::project_vertex_to_pixel(const pcl::PointCloud<pcl::PointXYZ>::Ptr v, std::vector<pcl::PointXY>* p)
{
	
	cv::Mat vertexes(4, (int)v->points.size(), CV_64FC1);
	cv::Mat pixels(3, (int)v->points.size(), CV_64FC1);

	
	for (size_t i=0; i < v->points.size(); i++) //set the vertexes homogeneous matrix
	{
		vertexes.at<double>(0,i) = v->points.at(i).x;
		vertexes.at<double>(1,i) = v->points.at(i).y;
		vertexes.at<double>(2,i) = v->points.at(i).z;
		vertexes.at<double>(3,i) = 1;
	}

	
	cv::Mat transformation;
	transformation.create(4,4,CV_64F);
	transformation = intrinsic*extrinsic;

	
	//compute the pixel coordinates
	pixels = transformation*vertexes;

	
	//erase all elements of p
	p->erase(p->begin(), p->end());


	
	for (int i=0; i < pixels.cols; i++) //set output vector
	{
		pcl::PointXY pt;
		pt.x = (pixels.at<double>(0,i) / pixels.at<double>(2,i));
		pt.y = (pixels.at<double>(1,i) / pixels.at<double>(2,i));
		p->push_back(pt);

		//printf("mat 0 = %f, mat 1 = %f mat2 = %f\n",pixels.at<double>(0,i), pixels.at<double>(1,i), pixels.at<double>(2,i));
		//printf("i=%d X=%3.2f Y=%3.2f Z=%3.2f --> pix x=%f y=%f\n",
				//i,
				//v->points[i].x,
				//v->points[i].y,
				//v->points[i].z,
				//p->at(i).x,
				//p->at(i).y	
			  //);



	}

	
	//printf("p->size()=%d\n",(int) p->size());
	//for (int i=1; i<(int)p->size(); i++)
	//{
	//printf("i=%d X=%3.2f Y=%3.2f Z=%3.2f --> pix x=%f y=%f\n",
	//i,
	//v->points[i].x,
	//v->points[i].y,
	//v->points[i].z,
	//p->at(i).x,
	//p->at(i).y	
	//);
	//}


	return 1;}


int class_pinhole_projection::project_pixel_with_color_to_vertex(const pcl::PointCloud<pcl::PointXYZRGB>* p, pcl::PointCloud<pcl::PointXYZRGB>* v)
{
	cv::Mat transformation;
	transformation.create(4,4,CV_64F);
	transformation = intrinsic*extrinsic;

	cv::Mat* M = &transformation;
	int valid_projections=0;

	//set variables to the transformation matrix
	double p11 = M->at<double>(0,0); double p12 = M->at<double>(0,1); double p13 = M->at<double>(0,2); double p14 = M->at<double>(0,3);
	double p21 = M->at<double>(1,0); double p22 = M->at<double>(1,1); double p23 = M->at<double>(1,2); double p24 = M->at<double>(1,3);
	double p31 = M->at<double>(2,0); double p32 = M->at<double>(2,1); double p33 = M->at<double>(2,2); double p34 = M->at<double>(2,3);

	//set references to the plane parameters
	double &A = projection_plane.a; double &B = projection_plane.b; double &C = projection_plane.c; double &D = projection_plane.d;

	//compute constants
	double K1 = (C*p12*p34+p13*D*p32-p12*p33*D-p32*C*p14-p13*B*p34+p33*B*p14);
	double K2 = (p32*C*p24-p34*C*p22-p33*B*p24-p32*p23*D+p33*D*p22+p34*p23*B);
	double K3 = (-p13*D*p22+C*p14*p22-C*p12*p24+p12*p23*D-p14*p23*B+p13*B*p24);
	double K4 = (C*p31*p12+p11*p33*B+p13*A*p32-C*p11*p32-p33*A*p12-p31*p13*B);
	double K5 = (C*p21*p32+p31*p23*B+p33*A*p22-C*p31*p22-p23*A*p32-p21*p33*B);
	double K6 = (-C*p21*p12-p11*p23*B+C*p11*p22+p23*A*p12+p21*p13*B-p13*A*p22);
	double K7 = (p11*p34*C-p13*A*p34-p11*p33*D+p33*A*p14+p31*p13*D-p31*C*p14);
	double K8 = (-p33*A*p24-p31*p23*D+p23*A*p34+p21*p33*D-C*p21*p34+C*p31*p24);
	double K9 = (C*p21*p14+p13*A*p24-p23*A*p14-p21*p13*D+p11*p23*D-C*p11*p24);
	double K10 = (C*p31*p12+p11*p33*B+p13*A*p32-C*p11*p32-p33*A*p12-p31*p13*B);
	double K11 = (C*p21*p32+p31*p23*B+p33*A*p22-C*p31*p22-p23*A*p32-p21*p33*B);
	double K12 = (-C*p21*p12-p11*p23*B+C*p11*p22+p23*A*p12+p21*p13*B-p13*A*p22);

	//empty all points from cloud
	v->points.erase(v->points.begin(), v->points.end());

	//Create the cv::Mat for valid projection assesment
	cv::Mat XYZ_plane(4, 1, CV_64FC1);
	XYZ_plane.at<double>(3,0) = 1; //homogeneous coordinates
	cv::Mat XYZ_image(4, 1, CV_64FC1);

	//Begin projection of pixels
	for (int i=0; i<(int)p->points.size();i++)
	{
		pcl::PointXYZRGB pix = p->at(i);	
		pcl::PointXYZRGB pt;
	    pt.x = -(K1*pix.y + K2*pix.x + K3) / (K4*pix.y + K5*pix.x + K6);
		pt.y = (K7*pix.y + K8*pix.x + K9) / (K10*pix.y + K11*pix.x + K12);
		pt.z = -(A*pt.x + B*pt.y + D)/C;
	
		//Check is projection is valid
		XYZ_plane.at<double>(0,0) = pt.x;
		XYZ_plane.at<double>(1,0) = pt.y;
		XYZ_plane.at<double>(2,0) = pt.z;
		XYZ_image = extrinsic*XYZ_plane;

		if (XYZ_image.at<double>(2,0)>0) //if its valid project
		{
			valid_projections++;
			pt.rgb = pix.rgb;
			v->points.push_back(pt);
		}
		else
		{
		    pt.x = NAN;
			pt.y = NAN;
			pt.z = NAN;
			pt.rgb = NAN;
			v->points.push_back(pt);
		}
	}

	if (0) //Debug
	{
		ROS_INFO("Projecting %d pixels. Valid projections = %d", (int)p->size(), valid_projections);
	}
return valid_projections;};

int class_pinhole_projection::project_pixel_to_vertex(const std::vector<pcl::PointXY>* p, pcl::PointCloud<pcl::PointXYZ>* v)
{
	cv::Mat transformation;
	transformation.create(4,4,CV_64F);
	transformation = intrinsic*extrinsic;

	cv::Mat* M = &transformation;
	int valid_projections=0;

	//set variables to the transformation matrix
	double p11 = M->at<double>(0,0); double p12 = M->at<double>(0,1); double p13 = M->at<double>(0,2); double p14 = M->at<double>(0,3);
	double p21 = M->at<double>(1,0); double p22 = M->at<double>(1,1); double p23 = M->at<double>(1,2); double p24 = M->at<double>(1,3);
	double p31 = M->at<double>(2,0); double p32 = M->at<double>(2,1); double p33 = M->at<double>(2,2); double p34 = M->at<double>(2,3);

	//set references to the plane parameters
	double &A = projection_plane.a; double &B = projection_plane.b; double &C = projection_plane.c; double &D = projection_plane.d;

	//compute constants
	double K1 = (C*p12*p34+p13*D*p32-p12*p33*D-p32*C*p14-p13*B*p34+p33*B*p14);
	double K2 = (p32*C*p24-p34*C*p22-p33*B*p24-p32*p23*D+p33*D*p22+p34*p23*B);
	double K3 = (-p13*D*p22+C*p14*p22-C*p12*p24+p12*p23*D-p14*p23*B+p13*B*p24);
	double K4 = (C*p31*p12+p11*p33*B+p13*A*p32-C*p11*p32-p33*A*p12-p31*p13*B);
	double K5 = (C*p21*p32+p31*p23*B+p33*A*p22-C*p31*p22-p23*A*p32-p21*p33*B);
	double K6 = (-C*p21*p12-p11*p23*B+C*p11*p22+p23*A*p12+p21*p13*B-p13*A*p22);
	double K7 = (p11*p34*C-p13*A*p34-p11*p33*D+p33*A*p14+p31*p13*D-p31*C*p14);
	double K8 = (-p33*A*p24-p31*p23*D+p23*A*p34+p21*p33*D-C*p21*p34+C*p31*p24);
	double K9 = (C*p21*p14+p13*A*p24-p23*A*p14-p21*p13*D+p11*p23*D-C*p11*p24);
	double K10 = (C*p31*p12+p11*p33*B+p13*A*p32-C*p11*p32-p33*A*p12-p31*p13*B);
	double K11 = (C*p21*p32+p31*p23*B+p33*A*p22-C*p31*p22-p23*A*p32-p21*p33*B);
	double K12 = (-C*p21*p12-p11*p23*B+C*p11*p22+p23*A*p12+p21*p13*B-p13*A*p22);

	//empty all points from cloud
	v->points.erase(v->points.begin(), v->points.end());

	//Create the cv::Mat for valid projection assesment
	cv::Mat XYZ_plane(4, 1, CV_64FC1);
	XYZ_plane.at<double>(3,0) = 1; //homogeneous coordinates
	cv::Mat XYZ_image(4, 1, CV_64FC1);

	//Begin projection of pixels
	for (int i=0; i<(int)p->size();i++)
	{
		pcl::PointXY pix = p->at(i);	
		pcl::PointXYZ pt;
	    pt.x = -(K1*pix.y + K2*pix.x + K3) / (K4*pix.y + K5*pix.x + K6);
		pt.y = (K7*pix.y + K8*pix.x + K9) / (K10*pix.y + K11*pix.x + K12);
		pt.z = -(A*pt.x + B*pt.y + D)/C;
	
		//Check is projection is valid
		XYZ_plane.at<double>(0,0) = pt.x;
		XYZ_plane.at<double>(1,0) = pt.y;
		XYZ_plane.at<double>(2,0) = pt.z;
		XYZ_image = extrinsic*XYZ_plane;

		if (XYZ_image.at<double>(2,0)>0) //if its valid project
		{
			valid_projections++;
			v->points.push_back(pt);
		}
		else
		{
		    pt.x = NAN;
			pt.y = NAN;
			pt.z = NAN;
			v->points.push_back(pt);
		}
	}

	if (0) //Debug
	{
		ROS_INFO("Projecting %d pixels. Valid projections = %d", (int)p->size(), valid_projections);
	}

return valid_projections;}

#endif
/**
 *@}
 */      
