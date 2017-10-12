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

#ifndef _PEDDETECT_H_
#define _PEDDETECT_H_

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cvwimage.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <cv.h>
#include <ros/package.h>
#include <boost/filesystem.hpp>
#include <boost/thread.hpp>
#include <iostream>
#include <iterator>
#include <algorithm>
#include <vector>
#include <math.h>
#include "opencv/ml.h"
#include <fstream>
#include <cstdlib>


#define PI 3.14159265359
#define CHANNELNR 10
#define STEP 4
#define NRFEATURE 15000
#define NRFEATURE2 NRFEATURE/CHANNELNR
#define DWHEIGHT 128
#define DWWIDTH 64
#define SEED 1234
#define MINAREA 8
#define MAXAREAWIDTH 64
#define MAXAREAHEIGHT 48
#define THRESHOLD 7
#define NOCTUP 0
#define SCALEPOCT 8

using namespace boost::filesystem;
using namespace std;
using namespace cv;
namespace enc = sensor_msgs::image_encodings;

typedef struct
{
  int nFtrs;
  int width;
  int height;
  int x;
  int y;
} FtrParams2;

typedef struct
{
  int x;
  int y;
  Size Scale;
} PedRoi;





typedef vector<PedRoi> PedRoiVec;
typedef vector<path> PVector;
typedef vector<Mat> MatVector;
typedef Vec<float, CHANNELNR> vec10d;
typedef vector<vec10d> d10Vector;
typedef vector<FtrParams2> FtrVecParams2;
typedef vector<float> DVector;



void GetFileList(string folder_path, PVector & dest_vect);

Mat GradientMagnitude(Mat src);
Mat GradientMagnitude(Mat xsobel, Mat ysobel);


MatVector OrientedGradientsDiagram(Mat GradMag, Mat xsobel, Mat ysobel);

MatVector LUVcolourchannels(Mat Img);


void GetChnFtrsOverImagePyramid(Mat Image , CvRect & region , vector<float> & features, int nOctUp, Size minSize, int nPerOct, FtrVecParams2 Params, PedRoiVec & PedRect, CvBoost & boost_classifier);
void GetChnFtrsOverImagePyramid(Mat Image , CvRect & region , vector<float> & features, vector<DVector> & WindowFtrs, int nOctUp, Size minSize, int nPerOct, FtrVecParams2 Params);


void GetChnFtrsOverImage(Mat IntegralChannels , CvRect & region  ,vector<float> & features, FtrVecParams2 Params, PedRoiVec & PedRect,CvBoost & boost_classifier);
void GetChnFtrsOverImage(Mat IntegralChannels , CvRect & region  ,vector<float> & features,vector<DVector> & WindowFtrs, FtrVecParams2 Params);

void GetChnFtrsOverWindow(Mat IntegralChannels , vector<float> & features ,FtrVecParams2 Params, CvRect region,PedRoiVec & PedRect,CvBoost & boost_classifier);
void GetChnFtrsOverWindow(Mat IntegralChannels , vector<float> & features,vector<DVector> & WindowFtrs ,FtrVecParams2 Params, CvRect region);

void GetIntegralSum(Mat IntegralChannels, vector<float> & features, FtrParams2 Params,CvRect region);

void ComputeChannels(Mat Image, Mat & MergedChannels);

void GetRandParams(int seed, int NrFtrs, FtrVecParams2 & RandParams, Rect region);

void GetChnFtrsOverWindow(Mat IntegralChannels , vector<float> & features, FtrVecParams2 Params, CvRect region,PedRoiVec & PedRect);
void GetChnFtrsOverWindow(Mat IntegralChannels , vector<float> & features,vector<DVector> & WindowFtrs, FtrVecParams2 Params);


void GetIntegralSum(Mat IntegralChannels, vector<float> & features, FtrParams2 Params,CvRect region);

void PostProcess(Mat Img, vector<Rect> PedRect, FtrVecParams2 randparams, CvBoost & boost_classifier, PedRoiVec & PedRect_Post);

#endif