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
#include <opencv2/imgproc/imgproc.hpp>			//Include headers for OpenCV Image processing
#include <opencv2/highgui/highgui.hpp>			//Include headers for OpenCV GUI handling
#include <iostream> 
#include <stdio.h> 
#include <ros/ros.h>	
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
#include <signal.h>

using namespace std;
using namespace cv;
/**
 *  intersept Ctrt+C handler
 */
void sighandler(int sig)
{
	ROS_ERROR("Signal %d caught...",sig);
	cout<<"Shuting down road_recognition"<<endl;
	exit(0);
}

void Callback(const sensor_msgs::CameraInfoPtr& cam_info)
{
	cout<<"---> Camera configurations"<<endl;
	cout<<"imge size "<<endl<<"\t"<<cam_info->height<<" x "<<cam_info->width<<endl;
	cout<<"distortion_model:"<<endl<<"\t"<<cam_info->distortion_model<<endl;
	
// 	cout<<"\tbinning_x = "<<cam_info->binning_x<<endl;
// 	cout<<"\tbinning_y = "<<cam_info->binning_y<<endl;
	
	cout<<"focal lengths"<<endl<<"\t fx ="<<cam_info->K[0]<<"  fy = "<<cam_info->K[4]<<endl;
	cout<<"principal point"<<endl<<"\t cx ="<<cam_info->K[2]<<"  cy = "<<cam_info->K[5]<<endl;
	
	
	cout<<"---> Camera configurations Updated"<<endl;
	exit(0);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "demo");
	
	/* Obter as definições das cameras do Atlas e guardar para ficheiro */
	ros::NodeHandle nh;
	signal(SIGINT, &sighandler);
	
	/* Create an ImageTransport instance, initializing it with our NodeHandle. */
// 	image_transport::ImageTransport it(nh);
	ros::Subscriber sub = nh.subscribe("/snr/scam/wide/left/camera_info", 1, Callback);
	ros::spin();
	ROS_INFO("Caltech::main.cpp::No error.");
// 	/* carregar a imagem */
// 	Mat image,eq_image,gray_image;	
// 	image = imread("src/src.png", CV_LOAD_IMAGE_COLOR);
// 	cvtColor(image,gray_image,CV_RGB2GRAY);
// 	
// 	equalizeHist( gray_image, eq_image );
// 	
// 	
// 	imshow( "Original img", image );
// 	imshow( "Eq img", eq_image );
// 	imshow( "gray_image",gray_image);
// 	waitKey(0); 
	
// 	return 0;
}
 
 
