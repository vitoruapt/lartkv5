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
/** \brief Matlab - Chroma based road tracking
 *  \file road_recognition.cpp
 *  \author Ricardo Morais
 *  \date Março 2013
 */

#include <ros/ros.h>							//Includes all the headers necessary to use the most common public pieces of the ROS system.
#include <image_transport/image_transport.h>	//Use image_transport for publishing and subscribing to images in ROS
#include <cv_bridge/cv_bridge.h>				//Use cv_bridge to convert between ROS and OpenCV Image formats
#include <sensor_msgs/image_encodings.h>		//Include some useful constants for image encoding. Refer to: http://www.ros.org/doc/api/sensor_msgs/html/namespacesensor__msgs_1_1image__encodings.html for more info.
#include <opencv2/imgproc/imgproc.hpp>			//Include headers for OpenCV Image processing
#include <opencv2/highgui/highgui.hpp>			//Include headers for OpenCV GUI handling

//Store all constants for image encodings in the enc namespace to be used later.
namespace enc = sensor_msgs::image_encodings;

//Declare a string with the name of the window that we will create using OpenCV where processed images will be displayed.
static const char WINDOW1[] = "Image Processed";
static const char WINDOW0[] = "Image Raw";

//Use method of ImageTransport to create image publisher
image_transport::Publisher pub;
// image_transport::Publisher pub_raw;

/**
*This function is called everytime a new image is published
*/
void imageCallback(const sensor_msgs::ImageConstPtr& original_image)
{
	int x=0 , y=0;
	//Convert from the ROS image message to a CvImage suitable for working with OpenCV for processing
	cv_bridge::CvImagePtr cv_ptr;
	cv_bridge::CvImagePtr cv_ptr_clone;
	
	try
	{
// 		Always copy, returning a mutable CvImage
// 		OpenCV expects color images to use BGR channel order.
		
		cv_ptr = cv_bridge::toCvCopy(original_image, enc::BGR8);
		cv_ptr_clone = cv_bridge::toCvCopy(original_image, enc::BGR8);
		cv::Mat cv_clone=cv_ptr->image.clone();
		
	}
	catch (cv_bridge::Exception& e)
	{
// 		if there is an error during conversion, display it
		ROS_ERROR("tutorialROSOpenCV::main.cpp::cv_bridge exception: %s", e.what());
		return;
	}
	
	
	/*Process the image */
	/* para todas as linhas */
	
	for(int i = 0; i<cv_ptr->image.rows; i++)
	{
		/* Para todas as colunas */
		for(int j = 0; j<cv_ptr->image.cols; j++)
		{
			if (cv_ptr->image.at<cv::Vec3b>(i,j)[0]>100 && cv_ptr->image.at<cv::Vec3b>(i,j)[1]>120 && cv_ptr->image.at<cv::Vec3b>(i,j)[2]>110 && cv_ptr->image.at<cv::Vec3b>(i,j)[0]<255 && cv_ptr->image.at<cv::Vec3b>(i,j)[1]<255 && cv_ptr->image.at<cv::Vec3b>(i,j)[2]<255)
			{
				if (cv_ptr->image.at<cv::Vec3b>(i,j)[0]>120 && cv_ptr->image.at<cv::Vec3b>(i,j)[1]>190 && cv_ptr->image.at<cv::Vec3b>(i,j)[2]>120 && cv_ptr->image.at<cv::Vec3b>(i,j)[0]<160 && cv_ptr->image.at<cv::Vec3b>(i,j)[1]<255 && cv_ptr->image.at<cv::Vec3b>(i,j)[2]<150)
				{
					cv_ptr_clone->image.at<cv::Vec3b>(i,j)[0] = 255;
					cv_ptr_clone->image.at<cv::Vec3b>(i,j)[1] = 255;
					cv_ptr_clone->image.at<cv::Vec3b>(i,j)[2] = 255;
				}
				else
				{
					cv_ptr_clone->image.at<cv::Vec3b>(i,j)[0] = 0;
					cv_ptr_clone->image.at<cv::Vec3b>(i,j)[1] = 0;
					cv_ptr_clone->image.at<cv::Vec3b>(i,j)[2] = 0;
				}
			}
			else
			{
				cv_ptr_clone->image.at<cv::Vec3b>(i,j)[0] = 255;
				cv_ptr_clone->image.at<cv::Vec3b>(i,j)[1] = 255;
				cv_ptr_clone->image.at<cv::Vec3b>(i,j)[2] = 255;
			}
		}
	}
	
	/* Criar um vector com as varias camadas da imagem */
	std::vector<cv::Mat> channels;
	
	/* separar a imagem em camadas */
	cv::split(cv_ptr_clone->image, channels);
	
	/* Separar a img nos varios canais */
	cv::Mat imageB = channels[0];
	cv::Mat imageG = channels[1];
	cv::Mat imageR = channels[2];
	
	/* Iniciar o check = 0 */
	int check = 0;

// 	ROS_INFO("valor das colunas %d",imageR.at<uchar>(100 , 100));
// 	ROS_INFO("O valor das condicoes do if \n %d \n %d",cv_ptr->image.at<uchar>(cv_ptr->image.rows , cv_ptr->image.cols/2) , cv_ptr->image.at<uchar>(cv_ptr->image.rows - 2, cv_ptr->image.cols / 2));
// 	ROS_INFO("O valor das condicoes do if da camada red \n %d\n %d",imageR.at<uchar>(imageR.rows , imageR.cols/2),imageR.at<uchar>(imageR.rows - 2, imageR.cols / 2));
// 	ROS_INFO("Numero de colunas da img %s e linhas %s",cv_ptr->image.cols,cv_ptr->image.rows);
	
// 	if (imageR.at<uchar>(imageR.rows , imageR.cols/2) == 0 &&  imageR.at<uchar>(imageR.rows - 2, imageR.cols / 2) == 0)
	if ( (imageR.at<uchar>(imageR.rows-1 , imageR.cols/2) <= 2) && (imageR.at<uchar>(imageR.rows - 2, imageR.cols / 2) <= 2) )
	{
// 		ROS_INFO("Entrou dentro do if");
		
		for (int i=1 ; i<imageR.rows-10 ; i++)
		{
			if ( imageR.at<uchar>(i , 1) && imageR.at<uchar>(i+1 , 1) && imageR.at<uchar>(i+4 , 1) && imageR.at<uchar>(i+5 , 1)==255 && imageR.at<uchar>(i+10 , 1)==255)
			{
				x=i;
			}
			
			if ( imageR.at<uchar>(i , cv_ptr->image.cols) && imageR.at<uchar>(i+2 , cv_ptr->image.cols) && imageR.at<uchar>(i+4 , cv_ptr->image.cols)==255 && imageR.at<uchar>(i+10 , cv_ptr->image.cols)==255)
			{
				y=i;
			}
		}
// 		ROS_INFO("Saiu do for ");
// 		ROS_INFO("x=%d - y=%d",x,y);
		if(x>=160 || y>=160)
		{
			if(x==y || x==y+2)
			{
// 				cv::imshow("SIGNAL", arrow3);
				ROS_ERROR("Right");
				check=1;
			}
			else if (x<y)
			{
// 				cv::imshow("SIGNAL", arrow4);
				ROS_ERROR("Left");
				check=1;
			}
			else if (x>y)
			{
// 				cv::imshow("SIGNAL", arrow2);
				ROS_ERROR("Right");
				check=1;
			}
		}
		
		if (check==0)
		{
// 			cv::imshow("SIGNAL", arrow5);
			ROS_ERROR("UP");
		}
	}
	else
	{
// 		ROS_ERROR("Cross");
	}

// 	cv_bridge::CvImage cv_ptr_clone(cv_ptr->header,cv_ptr->encoding,copy);
	
	/* Criar uma janela para publicar as imagens */
	cv::imshow(WINDOW1, cv_ptr_clone->image);
	cv::imshow(WINDOW0, cv_ptr->image);
	cv::waitKey(3);
	
	//Convert the CvImage to a ROS image message and publish it on the "camera/image_processed" topic.
// 	pub.publish(cv_ptr->toImageMsg());
	pub.publish(cv_ptr_clone->toImageMsg());
}

/**
* Always running ( check if there is a new message )
*/
int main(int argc, char **argv)
{
	ros::init(argc, argv, "image_processor");

	ros::NodeHandle nh;
	//Create an ImageTransport instance, initializing it with our NodeHandle.
	image_transport::ImageTransport it(nh);
	//OpenCV HighGUI call to create a display window on start-up.
	cv::namedWindow(WINDOW0, CV_WINDOW_AUTOSIZE);
	cv::namedWindow(WINDOW1, CV_WINDOW_AUTOSIZE);

	image_transport::Subscriber sub = it.subscribe("image", 1, imageCallback);
	//OpenCV HighGUI call to destroy a display window on shut-down.
	cv::destroyWindow(WINDOW0);
	cv::destroyWindow(WINDOW1);
	
// 	pub_raw = it.advertise("image_raw", 1);
	pub = it.advertise("image_processed", 1);
	
	ros::spin();
	ROS_INFO("tutorialROSOpenCV::main.cpp::No error.");
}
