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

/*! 
 *	@file 	test1.cpp
 * 	@brief	=========== Find arrow ==========
 *  
 *	@author		César Sousa, cesarsousa@ua.pt
 *	@date 6-3-2014
 * 	@version V0.0
 *	@internal
 * 
 * 		Revision	---
 * 		Compiler	gcc
 * 		Company		DEM - Universidade de Aveiro
 * 		Copyright	Copyright (c) 2014, César Sousa
 * 
 * 		Info:		
 * 	
 *	command to make doxyfile: "make doxyfile" than "make doc"
 * 
 */

 ///******************************
/// Notes :
///******************************

			
/************ proximo passo, ficar so com os quadrados q não têm nada à volta
* numa distancia da largura e cumprimento respectivamente deles
* ***************/
/** hughLines para encontrar linhas**/

/*
 * 
 * http://opencv-srf.blogspot.pt/2011/09/object-detection-tracking-using-contours.html
 * */
 
 //colocar "assertes" no código! nas PRÈ condiçoes
 
 //colocar funcoes todas com a mesma nomenclatura! (primeira letra peq_segndas palavras.. grde!)
 
 ///VERIFICAR SE o que esta a detetar é preto! e se é solido!!!!! (cor constante)
 
 ///http://docs.opencv.org/doc/tutorials/imgproc/shapedescriptors/hull/hull.html

//colocar um parametro na funcao principal para dizer qual a camara que quero

///******************************
/// Includes
///******************************

#ifndef _ARROW_DETECTION_H_
#define _ARROW_DETECTION_H_

#include <stdlib.h>
#include <iostream>
#include <stdio.h>
#include <string>
#include <math.h>
#include <vector>
#include <map>

//ROS
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <sstream>
#include <unistd.h>
#include <iostream>
#include <cmath>  

#include <sys/types.h>
#include <unistd.h>
#include <stdlib.h>

#include <geometry_msgs/Point.h>


//Visp
#include <visp/vp1394CMUGrabber.h>
#include <visp/vp1394TwoGrabber.h>
#include <visp/vpDisplayGDI.h>
#include <visp/vpDisplayX.h>
#include <visp/vpDot2.h>
#include <visp/vpPixelMeterConversion.h>
#include <visp/vpPose.h>

//OpenCV
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>


#include <arrow_detection/stabelized_func_arrow_detection.h>

///******************************
/// Const variables
///******************************

const unsigned int EROSION_SIZE=1;
const unsigned int EROSION_SIZE_MIN=1;
const unsigned int THRESH = 100;
const unsigned int MAX_THRESH = 255;//Trackbar
const unsigned int LINE_THICKNESS = 2;

/// For validate_lengths function
const unsigned short VAL_MAX_LENGTH= 500;
const unsigned short VAL_MIN_LENGTH = 100;
const float VAL_MIN_RELATION_LENGTH = 1.3; 
const float VAL_MAX_RELATION_LENGTH = 3.1;

/// Variables for validate_contour_Area_Length function  
const double BBCA_MIN = 1.3;//1.3; 
const double BBCA_MAX = 1.9;//1.7;
const double BBCL_MIN = 7;//12;
const double BBCL_MAX = 35;//30;
const double CACL_MIN = 4;//6;
const double CACL_MAX = 22;//17;
const double AREA_MIN = 500;//17;

/// Variable for validate_convexHull    
const unsigned short CONVEX_HULL_CORNERS = 5;

/// Variable for validate_corners_arrow
const unsigned short CONVEX_ARROW_CORNERS = 7;


// my files
//#include "rotate_image.cpp"

//to avoid having to prepend 'cv::' to all openCV related stuff (although I put some of them)
	using namespace cv;		
//to avoid having to prepend 'std::' to all standard library related stuff (although I put some of them)
	using namespace std;
	
///******************************
/// Function headers
///******************************
	
//void thresh_callback(int, void* );
/// FIND THE BOUDING BOX	
void boundig_box(Mat & src_tresh, Mat & dst);
	
/// FIND THE SMALLEST BOUNDING BOX
//vector<RotatedRect> boundig_box_small( Mat & src_tresh, Mat & threshold_output,vector<vector<cv::Point> > contours);		
vector<RotatedRect> boundig_box_small( Mat & src_tresh, Mat & threshold_output,vector<vector<cv::Point> > contours,
                                       cv::Point& src_centre,bool& havepoint);
	
/// Morphological operation OPENING
void Opening(Mat & src, Mat & dest,unsigned int erosion_size);
  
/// Morphological operation Closing
void closing(Mat & src, Mat & dest,unsigned int erosion_size);

/// See if the contour have neighbors! (arrow dont have!)	
//Point process_bbox_pnts_neighbors(vector<RotatedRect> pontos_retangulos);
/// Find Counturs
vector<vector<cv::Point> >  find_contours(Mat & src, Mat & dest,const unsigned int & line_thinckness,
		   const unsigned int & erosion_size_min);

///Main function to processec image and return the position of object
///Find arrow
	cv::Point find_arrow(void);
///Canny
  void ProcessMyImage(Mat & src, Mat & dest);
	
/// Boundig box BW
  void ProcessEdges(Mat & src, Mat & dest);
  
  void multiplymatrixbool(Mat & src, Mat & matbol);
  
  void descritor( Mat & img_object, Mat & img_scene);
  
static bool validate_lengths(cv::Point2f rect_points[],
							const unsigned short max_length= VAL_MAX_LENGTH,
							const unsigned short min_length = VAL_MIN_LENGTH,
							const float min_relation_length = VAL_MIN_RELATION_LENGTH , 
							const float max_relation_length = VAL_MAX_RELATION_LENGTH
							);
#endif
