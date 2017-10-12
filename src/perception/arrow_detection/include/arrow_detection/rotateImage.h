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
 *	@file 	rotateImage.h
 * 	@brief	helper function to rotate an image.
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

#ifndef ROTATE_IMAGE_H
#define ROTATE_IMAGE_H


#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <stdlib.h>
#include <iostream>
#include <stdio.h>
#include <string>
#include <math.h>
#include <vector>
#include <map>

using namespace cv;
using namespace std;

struct Equ {
    double m;
    double c;
    bool isVertical;
    bool isHorizontal;
};

class rotateImage
{
public:
  
    static void rotate_image(Mat & src, Mat & dst,const double degree);
  

protected:

    static double round(double number);

    static double degreeToRadian(double degree);

    static double radianToDegree(double radian);
    
    static Size rotationNewCanvasSize(double degree,double angle,double h);

    static double solveEquationY(Equ e,double x);

    static double solveEquationX(Equ e, double y);

    static map<string,int> rotationExtraMargins(Size &original, Size &newSize);

    static map<string,Point> getCorners(Size &original,map<string,int> &margins);
    
    static map<string,Point> getProjectedCorners(Size &s,double h,double degree,double angle);

    static Point getCentreBetweenPoints(Point &a, Point &b) ;

    static map<string,Point> getCentreBetweenOriginalsAndProjections(map<string,Point> &originals,map<string,Point> &projections);

    static Equ getLinearEquation(Point &a,Point &b);

    static Equ getPerpendicular(Equ e,Point p);
    
    static map<string,Equ> getLinearEquationBetweenOriginalsAndProjections(map<string,Point> &originals,map<string,Point> &projections);

    static map<string,Equ> getPerpendicularLinearEquation(map<string,Point> &originals,map<string,Point> &projections,map<string,Point> &centre);

    static Point getColisionPoint(Equ e1,Equ e2);

};


#endif // ROTATE_IMAGE_H

