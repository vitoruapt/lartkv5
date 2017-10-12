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
#include <ros/ros.h>							//Includes all the headers necessary to use the most common public pieces of the ROS system.
#include <image_transport/image_transport.h>	//Use image_transport for publishing and subscribing to images in ROS
#include <cv_bridge/cv_bridge.h>				//Use cv_bridge to convert between ROS and OpenCV Image formats
#include <sensor_msgs/image_encodings.h>		//Include some useful constants for image encoding. Refer to: http://www.ros.org/doc/api/sensor_msgs/html/namespacesensor__msgs_1_1image__encodings.html for more info.
#include <opencv2/imgproc/imgproc.hpp>			//Include headers for OpenCV Image processing
#include <opencv2/highgui/highgui.hpp>			//Include headers for OpenCV GUI handling
#include <vector>
#include <cmath> 
#include <iostream>
#include <signal.h>
#include <Eigen/Dense>

#include "rosbag/bag.h"
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#include <string>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
// #include <crpcut.hpp>
   
using namespace Eigen;
using namespace std;
using namespace cv;
int main(void)
{
	rosbag::Bag bag;
    bag.open("test.bag", rosbag::bagmode::Read);

    std::vector<std::string> topics;
    topics.push_back(std::string("chatter"));
    topics.push_back(std::string("numbers"));

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    BOOST_FOREACH(rosbag::MessageInstance const m, view)
    {
        std_msgs::String::ConstPtr s = m.instantiate<std_msgs::String>();
//         if (s != NULL)
//             ASSERT_EQ(s->data, std::string("foo"));

        std_msgs::Int32::ConstPtr i = m.instantiate<std_msgs::Int32>();
//         if (i != NULL)
//             ASSERT_EQ(i->data, 42);
    }

    bag.close();
	
	
// 	Mat image= imread("/home/morais/Pictures/download.jpg");
// 	Mat out;
	
	
	/*
	watershed(image,out);
	imshow("result",out);
	waitKey(0);
	return 0;
	
	
	cvtColor(image,image,CV_RGB2GRAY,CV_8U);
	Mat clone = image(Range(50,image.rows),Range(0,image.cols));
	
	Mat clone_2 = clone.clone();
	
	imshow("Clone",clone);
	imshow("image clone_2",clone);*/
// 	cvtColor(image,image,CV_RGB2GRAY,CV_8U);
// // 	int threshold=100;
// // 	int colour=255;
// 	threshold(image,out, 100,255,CV_THRESH_BINARY_INV);
// 	imshow("imagem original",image);
// 	imshow("imagem ivertida",out);
// 	waitKey(0);

// 	MatrixXd m = MatrixXd::Random(3,3);
// 	cout<<"inicial n"<<endl<<m<<endl;
// // 	MatrixXd m_f;
// 	
// // 	m_f = m ;
// 	MatrixXi::Index min_i, min_j;
// 	double minOfM = m.minCoeff(&min_i,&min_j);
// 	cv::Mat img(240,360,CV_8U,cv::Scalar(0));
// 	int lineType = 8;
// 	Point rook_points[1][6];
// 	rook_points[0][0] = Point( 0, 120 );
// 	rook_points[0][1] = Point( 0, 120 );
// 	rook_points[0][2] = Point( 1, 239 );
// 	rook_points[0][3] = Point( 359, 239 );
// 	rook_points[0][4] = Point( 360, 158 );
// 	rook_points[0][5] = Point( 218, 121 );
// 	rook_points[0][6] = Point( 3*w/4.0, w/8.0 );
// 	rook_points[0][7] = Point( 26*w/40.0, w/8.0 );
// 	rook_points[0][8] = Point( 26*w/40.0, w/4.0 );
	
// 	[0, 120]
// 	[0, 120]
// 	[1, 239]
// // // 	[359, 239]
// 	[360, 158]
// 	[218, 121]
// 	[0, 120]
/*	
	
	const Point* ppt[1] = { rook_points[0] };
	int npt[] = { 6 };
	
	fillPoly( img,
			  ppt,
		   npt,
		   1,
		   Scalar( 255, 255, 255 ),
			  lineType );
	
	imshow("test",img);
	waitKey(0);*/
	
// 	Match_dis(i) = minOfM;
// 	rowInd = min_i;
// 	colInd = min_j;
	
// 	R.col(j1).swap(mat1.col(j2));
	
	
// 	cout<<"mininmo ="<<minOfM<<" em "<<min_i<<" "<<min_j<<endl;
// 	return 1;
}
