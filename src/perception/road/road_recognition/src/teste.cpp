/*------------------------------------------------------------------------------------------*\
   This file contains material supporting chapter 7 of the cookbook:  
   Computer Vision Programming using the OpenCV Library. 
   by Robert Laganiere, Packt Publishing, 2011.

   This program is free software; permission is hereby granted to use, copy, modify, 
   and distribute this source code, or portions thereof, for any purpose, without fee, 
   subject to the restriction that the copyright notice may not be removed 
   or altered from any source or altered source distribution. 
   The software is released on an as-is basis and without any warranties of any kind. 
   In particular, the software is not guaranteed to be fault-tolerant or free from failure. 
   The author disclaims all warranties with regard to this software, any use, 
   and any consequent failure, is purely the responsibility of the user.
 
   Copyright (C) 2010-2011 Robert Laganiere, www.laganiere.name
\*------------------------------------------------------------------------------------------*/

#include <iostream>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <ros/ros.h>							//Includes all the headers necessary to use the most common public pieces of the ROS system.
#include <image_transport/image_transport.h>	//Use image_transport for publishing and subscribing to images in ROS
#include <cv_bridge/cv_bridge.h>				//Use cv_bridge to convert between ROS and OpenCV Image formats
#include <sensor_msgs/image_encodings.h>		//Include some useful constants for image encoding. Refer to: http://www.ros.org/doc/api/sensor_msgs/html/namespacesensor__msgs_1_1image__encodings.html for more info.

#include "linefinder.h"
#include "edgedetector.h"

#define PI 3.1415926
namespace enc = sensor_msgs::image_encodings;

//Declare a string with the name of the window that we will create using OpenCV where processed images will be displayed.
static const char WINDOW1[] = "Image Processed";
static const char WINDOW0[] = "Image Raw";

//Use method of ImageTransport to create image publisher
image_transport::Publisher pub;

 int imageCallback(const sensor_msgs::ImageConstPtr& original_image)
{
	int x=0 , y=0;
	//Convert from the ROS image message to a CvImage suitable for working with OpenCV for processing
	cv_bridge::CvImagePtr cv_ptr;
	cv::Mat im_gray;	/* imagem recebida em niveis de cinza*/
	cv::Mat YCbCr;
	cv::Mat subImg;
	cv::Mat cv_clone;
	
	try
	{
// 		Always copy, returning a mutable CvImage
// 		OpenCV expects color images to use BGR channel order.
		cv_ptr = cv_bridge::toCvCopy(original_image, enc::BGR8);
		cv_clone = cv_ptr->image.clone();
		
	}
	catch (cv_bridge::Exception& e)
	{
// 		if there is an error during conversion, display it
		ROS_ERROR("tutorialROSOpenCV::main.cpp::cv_bridge exception: %s", e.what());
		return 0;
	}
	
	// Read input image
	cv::Mat image;
	cv_clone.copyTo(image);
	if (!image.data)
		return 0; 

    // Display the image
// 	cv::namedWindow("Original Image");
// 	cv::imshow("Original Image",image);
// 	cv::waitKey(0);

	// Compute Sobel
	EdgeDetector ed;
	ed.computeSobel(image);

    // Display the Sobel orientation
	cv::namedWindow("Sobel (orientation)");
	cv::imshow("Sobel (orientation)",ed.getSobelOrientationImage());
	cv::waitKey(3);
	cv::imwrite("ori.bmp",ed.getSobelOrientationImage());

    // Display the Sobel low threshold
	cv::namedWindow("Sobel (low threshold)");
	cv::imshow("Sobel (low threshold)",ed.getBinaryMap(125));
	cv::waitKey(3);
    // Display the Sobel high threshold
	cv::namedWindow("Sobel (high threshold)");
	cv::imshow("Sobel (high threshold)",ed.getBinaryMap(350));
	cv::waitKey(3);
	// Apply Canny algorithm
	cv::Mat contours;
	cv::Canny(image,contours,125,350);
	cv::Mat contoursInv;
	cv::threshold(contours,contoursInv,128,255,cv::THRESH_BINARY_INV);

    // Display the image of contours
	cv::namedWindow("Canny Contours");
	cv::imshow("Canny Contours",contoursInv);
	cv::waitKey(3);
		
	// Hough tranform for line detection
	std::vector<cv::Vec2f> lines;
	cv::HoughLines(contours,lines,1,(75*PI)/180,60);

	// Draw the lines
	cv::Mat result(contours.rows,contours.cols,CV_8U,cv::Scalar(255));
	image.copyTo(result);

	std::cout << "Lines detected: " << lines.size() << std::endl;

	std::vector<cv::Vec2f>::const_iterator it= lines.begin();
	while (it!=lines.end()) {

		float rho= (*it)[0];   // first element is distance rho
		float theta= (*it)[1]; // second element is angle theta
		
		if (theta < PI/4. || theta > 3.*PI/4.) { // ~vertical line
		
			// point of intersection of the line with first row
			cv::Point pt1(rho/cos(theta),0);        
			// point of intersection of the line with last row
			cv::Point pt2((rho-result.rows*sin(theta))/cos(theta),result.rows);
			// draw a white line
			cv::line( result, pt1, pt2, cv::Scalar(255), 1); 

		} else { // ~horizontal line

			// point of intersection of the line with first column
			cv::Point pt1(0,rho/sin(theta));        
			// point of intersection of the line with last column
			cv::Point pt2(result.cols,(rho-result.cols*cos(theta))/sin(theta));
			// draw a white line
			cv::line( result, pt1, pt2, cv::Scalar(255), 1); 
		}

		std::cout << "line: (" << rho << "," << theta << ")\n"; 

		++it;
	}
	
    // Display the detected line image
	cv::namedWindow("Detected Lines with Hough");
	cv::imshow("Detected Lines with Hough",result);
	cv::waitKey(3);
	// Create LineFinder instance
	LineFinder ld;

	// Set probabilistic Hough parameters
	ld.setLineLengthAndGap(15,40);
	ld.setMinVote(50);

	// Detect lines
	std::vector<cv::Vec4i> li= ld.findLines(contours);
	ld.drawDetectedLines(image);
	cv::namedWindow("Detected Lines with HoughP");
	cv::imshow("Detected Lines with HoughP",image);
// 	cv::waitKey(0);
	
	std::vector<cv::Vec4i>::const_iterator it2= li.begin();
	while (it2!=li.end()) {

		std::cout << "(" << (*it2)[0] << ","<< (*it2)[1]<< ")-(" 
			     << (*it2)[2]<< "," << (*it2)[3] << ")" <<std::endl;

		++it2;
	}

	// Display one line
// 	image= cv::imread("../road.jpg",0);
	int n=0;
	cv::line(image, cv::Point(li[n][0],li[n][1]),cv::Point(li[n][2],li[n][3]),cv::Scalar(255),5);
	cv::namedWindow("One line of the Image");
	cv::imshow("One line of the Image",image);
	
	// Extract the contour pixels of the first detected line
	cv::Mat oneline(image.size(),CV_8U,cv::Scalar(0));
	cv::line(oneline, cv::Point(li[n][0],li[n][1]),cv::Point(li[n][2],li[n][3]),cv::Scalar(255),5);
	cv::bitwise_and(contours,oneline,oneline);
	cv::line(oneline, cv::Point(li[n+1][0],li[n+1][1]),cv::Point(li[n+1][2],li[n+1][3]),cv::Scalar(255),5);
	cv::bitwise_and(contours,oneline,oneline);
// 	cv::bitwise_and(contours,secline,secline);
	cv::Mat onelineInv;
// 	cv::threshold(oneline,onelineInv,128,255,cv::THRESH_BINARY_INV);
	cv::adaptiveThreshold(oneline, onelineInv, 128, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY_INV, 3, 5);
	cv::namedWindow("One line");
	cv::imshow("One line",onelineInv);

	std::vector<cv::Point> points;

	// Iterate over the pixels to obtain all point positions
	for( int y = 0; y < oneline.rows; y++ ) {
    
		uchar* rowPtr = oneline.ptr<uchar>(y);
    
		for( int x = 0; x < oneline.cols; x++ ) {

		    // if on a contour
			if (rowPtr[x]) {

				points.push_back(cv::Point(x,y));
			}
		}
    }
	
	// find the best fitting line
	cv::Vec4f line;
	cv::fitLine(cv::Mat(points),line,CV_DIST_L2,0,0.01,0.01);
	
	std::cout << "line: (" << line[0] << "," << line[1] << ")(" << line[2] << "," << line[3] << ")\n"; 

	int x0= line[2];
	int y0= line[3];
	int x1= x0-200*line[0];
	int y1= y0-200*line[1];
// 	image= cv::imread("../road.jpg",0);
	cv::line(image,cv::Point(x0,y0),cv::Point(x1,y1),cv::Scalar(0),3);
	cv::namedWindow("Estimated line");
	cv::imshow("Estimated line",image);

	// eliminate inconsistent lines
	ld.removeLinesOfInconsistentOrientations(ed.getOrientation(),0.4,0.1);

   // Display the detected line image
// 	image= cv::imread("../road.jpg",0);
	ld.drawDetectedLines(image);
	cv::namedWindow("Detected Lines (2)");
	cv::imshow("Detected Lines (2)",image);

	// Create a Hough accumulator
	cv::Mat acc(200,180,CV_8U,cv::Scalar(0));

	// Choose a point
	x=50, y=30;

	// loop over all angles
	for (int i=0; i<180; i++) {

		double theta= i*PI/180.;

		// find corresponding rho value 
		double rho= x*cos(theta)+y*sin(theta);
		int j= static_cast<int>(rho+100.5);

		std::cout << i << "," << j << std::endl;

		// increment accumulator
		acc.at<uchar>(j,i)++;
	}

// 	cv::imwrite("hough1.bmp",acc*100);

	// Choose a second point
	x=30, y=10;

	// loop over all angles
	for (int i=0; i<180; i++) {

		double theta= i*PI/180.;
		double rho= x*cos(theta)+y*sin(theta);
		int j= static_cast<int>(rho+100.5);

		acc.at<uchar>(j,i)++;
	}

	cv::namedWindow("Hough Accumulator");
	cv::imshow("Hough Accumulator",acc*100);
	ROS_ERROR("-------ERROR -----");
// // 	cv::imwrite("hough2.bmp",acc*100);
// // ROS_ERROR("-------ERROR -----");
// 	// Detect circles
// // 	image= cv::imread("../chariot.jpg",0);
// 	cv::GaussianBlur(image,image,cv::Size(5,5),1.5);
// 	std::vector<cv::Vec3f> circles;
// // 	ROS_ERROR("-------ERROR -----");
// 	cv::HoughCircles(image, circles, CV_HOUGH_GRADIENT, 
// 		2,   // accumulator resolution (size of the image / 2) 
// 		50,  // minimum distance between two circles
// 		200, // Canny high threshold 
// 		100, // minimum number of votes 
// 		25, 100); // min and max radius
// ROS_ERROR("-------ERROR -----");
// 	std::cout << "Circles: " << circles.size() << std::endl;
// 	ROS_ERROR("-------ERROR -----");
// 	// Draw the circles
// // 	image= cv::imread("../chariot.jpg",0);
// 	std::vector<cv::Vec3f>::const_iterator itc= circles.begin();
// 	ROS_ERROR("-------ERROR -----");
// 	while (itc!=circles.end()) {
// 		
// 	  cv::circle(image, 
// 		  cv::Point((*itc)[0], (*itc)[1]), // circle centre
// 		  (*itc)[2], // circle radius
// 		  cv::Scalar(255), // color 
// 		  2); // thickness
// 		
// 	  ++itc;	
// 	}
// 
// 	
// 	cv::namedWindow("Detected Circles");
// 	cv::imshow("Detected Circles",image);

	cv::waitKey(3);
	return 0;
}


/**
* Always running ( check if there is a new message )
*/
int main(int argc, char **argv)
{
	ros::init(argc, argv, "test");

	ros::NodeHandle nh;
	/* Create an ImageTransport instance, initializing it with our NodeHandle. */
	image_transport::ImageTransport it(nh);
	/* OpenCV HighGUI call to create a display window on start-up. */
	cv::namedWindow(WINDOW0, CV_WINDOW_AUTOSIZE);
	cv::namedWindow(WINDOW1, CV_WINDOW_AUTOSIZE);

	image_transport::Subscriber sub = it.subscribe("image", 1, imageCallback);
	/* OpenCV HighGUI call to destroy a display window on shut-down. */
	cv::destroyWindow(WINDOW0);
	cv::destroyWindow(WINDOW1);
	
	pub = it.advertise("image_processed", 1);
	
	ros::spin();
	ROS_INFO("tutorialROSOpenCV::main.cpp::No error.");
}