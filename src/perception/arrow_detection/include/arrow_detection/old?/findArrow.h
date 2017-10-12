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
 *	@file 	finArrow.h
 * 	@brief	find and return the center position of an arrow
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

#ifndef FIND_ARROW_H
#define FIND_ARROW_H

#include <opencv2/features2d/features2d.hpp>//opencv
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <stdlib.h>//boost/cpp
#include <iostream>
#include <stdio.h>
#include <string>
#include <math.h>
#include <vector>
#include <map>

// my files
#include "rotateImage.h"

/* assert */ // http://www.cplusplus.com/reference/cassert/assert/
#include <assert.h>     

using namespace cv;     
using namespace std;

class findArrow
{
public:
    
//global vars
    //static unsigned int src_centre_x;
    //static unsigned int src_centre_y;
    //static cv::Point src_centre;
    static bool found_new_point;

// Const variables

    static const unsigned int EROSION_SIZE;
    static const unsigned int EROSION_SIZE_MIN;
    static const unsigned int THRESH;
    static const unsigned int MAX_THRESH;//Trackbar
    static const unsigned int LINE_THICKNESS;
  
    // For validate_lengths function
    static const unsigned short VAL_MAX_LENGTH;
    static const unsigned short VAL_MIN_LENGTH;
    static const float VAL_MIN_RELATION_LENGTH;
    static const float VAL_MAX_RELATION_LENGTH;
  
    // Variables for validate_contour_Area_Length function  
    static const double BBCA_MIN;
    static const double BBCA_MAX;
    static const double BBCL_MIN;
    static const double BBCL_MAX;
    static const double CACL_MIN;
    static const double CACL_MAX;
    static const double AREA_MIN;
    
    // Variable for validate_convexHull    
    static const unsigned short CONVEX_HULL_CORNERS;
    
    // Variable for validate_corners_arrow
    static const unsigned short CONVEX_ARROW_CORNERS;

//functions
    /// FIND THE BOUDING BOX    
    static void boundig_box(Mat & src_tresh, Mat & dst);
        
    /// FIND THE SMALLEST BOUNDING BOX
    static vector<RotatedRect> boundig_box_small( Mat & src_tresh, Mat & threshold_output,vector<vector<Point> > contours,
                                cv::Point & src_centre);

    /// Morphological operation OPENING
    static void Opening(Mat & src, Mat & dest,unsigned int erosion_size);
      
    /// Morphological operation Closing
    static void closing(Mat & src, Mat & dest,unsigned int erosion_size);

    /// Find Counturs
    static vector<vector<Point> >  find_contours(Mat & src, Mat & dest,const unsigned int & line_thinckness);

    ///Main function to processec image and return the position of object
    ///Find arrow
    //static int find_arrow(Mat img_scene);
    static int find_arrow(cv::Mat frame,cv::Point & center_point);
    
    static void ProcessMyImage(Mat & src, Mat & dest);
        
    /// Boundig box BW
    static void ProcessEdges(Mat & src, Mat & dest);
      
    static void multiplymatrixbool(Mat & src, Mat & matbol);
    
    /// Find discriptors (Match points)  
    static void descritor( Mat & img_object, Mat & img_scene);
      
    static bool validate_lengths(Point2f rect_points[],
                                const unsigned short max_length= VAL_MAX_LENGTH,
                                const unsigned short min_length = VAL_MIN_LENGTH,
                                const float min_relation_length = VAL_MIN_RELATION_LENGTH , 
                                const float max_relation_length = VAL_MAX_RELATION_LENGTH
                                );

    static Point calculate_center_contour(Point2f contour[],unsigned int size_contour);

    static bool validate_contour_Area_Length(vector<Point> contours,vector<double> bbox_points,
        const double bbca_min = BBCA_MIN,
        const double bbca_max = BBCA_MAX,const double bbcl_min = BBCL_MIN, const double bbcl_max = BBCL_MAX,
        const double cacl_min = CACL_MIN,const double cacl_max = CACL_MAX);
    /// SORTE POINTS helper validate_lengths 
    static vector<double> sort_points(Point2f rect_points[]);
    
    static bool validate_corners_arrow(vector<Point> contours,
                    const unsigned short convex_arrow_corners = CONVEX_ARROW_CORNERS);
    //check number of corners of convexhull 
    static bool validate_convexHull(vector<Point> contours,
                    const unsigned short convex_hull_corners = CONVEX_HULL_CORNERS);


protected:
    static void setLabel(cv::Mat& im, const std::string label, std::vector<cv::Point>& contour);

    static float innerAngle(float px1, float py1, float px2, float py2, float cx1, float cy1);

    static double angle(cv::Point pt1, cv::Point pt2, cv::Point pt0);

};
#endif
