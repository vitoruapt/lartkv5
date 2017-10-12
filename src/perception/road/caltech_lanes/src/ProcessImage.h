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
/** \brief Class para fazer o processamento da imagem 
 *  \file ProcessImage.h
 *  \author Ricardo Morais
 *  \date Maio 2013
 * 
 * \param filename the input file name
 * \param cameraInfo the camera calibration info
 * \param lanesConf the lane detection settings
 * \param stoplinesConf the stop line detection settings
 * \param options the command line arguments
 */

#include "main.hh"
#include "cmdline.h"
#include "LaneDetector.hh"
#include "main.hh"
#include "cmdline.h"
#include "LaneDetector.hh"
#include <stdio.h>
#include <fstream>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <ctime>

// includes do opencv antigo
#include <cv.h>
#include <highgui.h>
// #include <opencv2/imgproc/imgproc.hpp>
// #include <opencv2/highgui/highgui.hpp>
// Useful message macro
#define MSG(fmt, ...) \
(fprintf(stdout, "%s:%d msg   " fmt "\n", __FILE__, __LINE__, ##__VA_ARGS__) ? 0 : 0)

// Useful error macro
#define ERROR(fmt, ...) \
(fprintf(stderr, "%s:%d error " fmt "\n", __FILE__, __LINE__, ##__VA_ARGS__) ? -1 : -1)
// int DEBUG_LINES = 1;

using namespace LaneDetector;
using namespace std;
using namespace cv;

class Procecess
{
	private:
		// parse the command line paramters
		gengetopt_args_info options;
		// read the camera configurations
		CameraInfo cameraInfo;
		// read the configurations
		LaneDetectorConf lanesConf, stoplinesConf;
		
		
	public:
	// Load the configuration files to global variables
	int Load_config(int argc, char** argv)
	{
		// parse the command line paramters
// 		gengetopt_args_info options_local;
		if (cmdline_parser (argc, argv,  &options) < 0)
			return -1;
		
		// read the camera configurations
		mcvInitCameraInfo(options.camera_conf_arg,&cameraInfo);
		MSG("Loaded camera file");
			
		// read the configurations
		if (!options.no_lanes_flag)
		{
			mcvInitLaneDetectorConf(options.lanes_conf_arg,&lanesConf);
			MSG("Loaded lanes config file");
		}
		
		if (!options.no_stoplines_flag)
		{
			mcvInitLaneDetectorConf(options.stoplines_conf_arg, &stoplinesConf);
			MSG("Loaded stop lines config file");
		}
		
		// set debug to true
		if (options.debug_flag)
			DEBUG_LINES = 1;

		
		return 1;
	}
			
			
	/**
		* This function processes an input image and detects lanes/stoplines
		* based on the passed in command line arguments
		* \param outputFile the output file stream to write output lanes to
		* \param index the image index (used for saving output files)
		* \param elapsedTime if NOT NULL, it is accumulated with clock ticks for
		*        the detection operation
		*/
	void ProcessImage(Mat &raw_mat, ofstream* outputFile,int index, clock_t *elapsedTime)
	{

		
		Mat eq_img = raw_mat;
		cvtColor(eq_img, eq_img, CV_BGR2GRAY);
		
		if ( DEBUG_LINES == 1)
			imshow("grey_img",eq_img);
		
// // // // // // Fazer o pré-processamento da imagem em C++
		/* Equalização de histograma */
		equalizeHist( eq_img , eq_img );
		if ( DEBUG_LINES == 1)
			imshow("eq_img",eq_img);
		
		// Since MORPH_X : 2,3,4,5 and 6
		int operation = 2;
			
		Mat element = getStructuringElement( 0, Size( 2*0 + 1, 2*0+1 ), Point( 0, 0 ) );
// 		morphologyEx(eq_img,eq_img,operation,element);
		
		
		/* TopHat */
		if ( DEBUG_LINES == 1)
			imshow("top_hat",eq_img);

		
		// convert cv::Mat to cvMat
		Mat mat = eq_img;
		CvMat clrImage = mat;
		CvMat cv_raw_mat = raw_mat;
		// convert to mat and get first channel
		CvMat *channelImage;
// 		cout<<"------> Debug at in "<<__FILE__<<" at line "<<__LINE__<<endl;
		
		// convert to single channel
// 		CvMat* tchannelImage = cvCreateMat(raw_mat.rows, raw_mat.cols, INT_MAT_TYPE);
// 		cvSplit(raw_mat, tchannelImage, NULL, NULL, NULL);
// 		cout<<"------> Debug at in "<<__FILE__<<" at line "<<__LINE__<<endl;
		
		// convert to float
		channelImage = cvCreateMat(raw_mat.rows, raw_mat.cols, FLOAT_MAT_TYPE);
		cvConvertScale(&clrImage, channelImage, 1./255);
// 		cout<<"------> Debug at in "<<__FILE__<<" at line "<<__LINE__<<endl;
		
		// destroy
// 		cvReleaseMat(&tchannelImage);

// 		cout<<"Enter - ProcessImage"<<endl;
		// detect lanes
		vector<FLOAT> lineScores, splineScores;
		vector<Line> lanes;
		vector<Spline> splines;
		clock_t startTime = clock();
		
		mcvGetLanes(channelImage, &cv_raw_mat, &lanes, &lineScores, &splines, &splineScores,&cameraInfo, &lanesConf, NULL);
// 		cout<<"-> Saiu do mcvGetLanes"<<endl;
		
		clock_t endTime = clock();
		MSG("Found %d lanes in %f msec", splines.size(), static_cast<double>(endTime - startTime) / CLOCKS_PER_SEC * 1000.);
		
		
		// update elapsed time
		if (elapsedTime)
			(*elapsedTime) += endTime - startTime;
		
// 		cout<<"------> Debug at in "<<__FILE__<<" at line "<<__LINE__<<endl;
		// save results?
		if (options.save_lanes_flag && outputFile && outputFile->is_open())
		{
			(*outputFile) << "frame#" << setw(8) << setfill('0') << index <<
			" has " << splines.size() << " splines" << endl;
			for (uint i=0; i<splines.size(); ++i)
			{
				(*outputFile) << "\tspline#" << i+1 << " has " <<splines[i].degree+1 << " points and score " <<	splineScores[i] << endl;
				for (int j=0; j<=splines[i].degree; ++j)
					(*outputFile) << "\t\t" <<splines[i].points[j].x << ", " <<splines[i].points[j].y << endl;
			}
		}
// 		cout<<"------> Debug at in "<<__FILE__<<" at line "<<__LINE__<<endl;
		
		// show or save
		if (options.show_flag || options.save_images_flag)
		{
// 			cout<<"------> Debug at in "<<__FILE__<<" at line "<<__LINE__<<endl;
			// show detected lanes
			CvMat *imDisplay = cvCloneMat(&cv_raw_mat);
// 			cout<<"------> Debug at in "<<__FILE__<<" at line "<<__LINE__<<endl;
			// convert to BGR
			//     cvCvtColor(raw_mat, imDisplay, CV_RGB2BGR);
			if (lanesConf.ransacLine && !lanesConf.ransacSpline)
				for(uint i=0; i<lanes.size(); i++)
					mcvDrawLine(imDisplay, lanes[i], CV_RGB(0,125,0), 3);
// 				cout<<"------> Debug at in "<<__FILE__<<" at line "<<__LINE__<<endl;
				// print lanes
			if (lanesConf.ransacSpline)
			{
				for(uint i=0; i<splines.size(); i++)
				{
					if (splines[i].color == LINE_COLOR_YELLOW)
					{
						mcvDrawSpline(imDisplay, splines[i], CV_RGB(255,255,0), 3);
						
					
					}
					else
					{
						mcvDrawSpline(imDisplay, splines[i], CV_RGB(0,255,0), 3);
						
// 						cout<<"splines.degree--> "<<splines[i].degree<<endl;
						double x_max=0,y_max=0;
						double x_min = cameraInfo.imageWidth;
						double y_min = cameraInfo.imageHeight;
						
						for (uint n=0 ; n<4 ; n++)
						{
							if (DEBUG_LINES)
							{
								cout<<"spline->"<<i<<endl;
								cout<<"splines.points.x--> "<<splines[i].points[n].x<<endl;
								cout<<"splines.points.y--> "<<splines[i].points[n].y<<endl;
							}
							
							/* Escolher os pontos extremos, para desenhar o poligono que reprensenta a estrada */
							if ( (splines[i].points[n].x < x_min) )
							{
								x_min = splines[i].points[n].x;
								y_min = splines[i].points[n].y;
							}
							
							if ( (splines[i].points[n].x > x_max) )
							{
								x_max = splines[n].points[i].x;
								y_max = splines[n].points[i].y;
							}
							
							/* display dos pontos extremos */
// 							if (DEBUG_LINES)
// 							{
								if (n==3)
								{
									cout<<"Image size: "<<cameraInfo.imageHeight<<"x"<<cameraInfo.imageWidth<<endl;
									cout<<"min point x = ("<<x_min<<" , "<<y_min<<" )"<<endl;
									cout<<"max point x = ("<<x_max<<" , "<<y_max<<" )"<<endl;
									cout<<"i=="<<i<<endl;
									/////////////////////////////////////////
									/* Creat a poligom image with the road	*/
									/////////////////////////////////////////	
// 									int npts = 6;
// 									int npt[] = { 6 };
// 									
// 									Point Pts_poly[1][npts];
// 									Pts_poly[0][0] = cv::Point( TwoLanes1(0,0) , TwoLanes1(1,0) );
// 									Pts_poly[0][1] = cv::Point( TwoLanes1(2,0) , TwoLanes1(3,0) );
// 									Pts_poly[0][2] = cv::Point( Templ(0) , Templ(1) );
// 									Pts_poly[0][3] = cv::Point( Tempr(0) , Tempr(1) );
// 									Pts_poly[0][4] = cv::Point( TwoLanes1(2,1),TwoLanes1(3,1) );
// 									Pts_poly[0][5] = cv::Point( TwoLanes1(0,1),TwoLanes1(1,1) );
// 									
// 									const Point *ppt[1]={ Pts_poly[0] };
									
// 									fillPoly( cv_clone, ppt, npt, 1, Scalar( 200, 200, 200 ),8);
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
									if ( i==1 ) /* Só desenhar o poligono caso existam mais de uma linha encontrada */
									{
										int npts = 4;
										int npt[] = { 4 };
										
										Point Pts_poly[1][npts];
										
										for (uint pt = 0; pt<npts ; pt++)
											Pts_poly[0][0] = Point( splines[0].points[0].x , splines[0].points[0].y ) ;
// 																						
										Mat disp = cv::Mat(imDisplay, true); // to copy the data
											
										const Point *ppt[1]={ Pts_poly[0] };
										fillPoly( disp, ppt, npt, 1, Scalar( 200, 200, 200 ),8);
									}
								}
// 							}
						}
						
					}
					// print numbers?
					if (options.show_lane_numbers_flag)
					{
						char str[256];
						sprintf(str, "%d", i);
						mcvDrawText(imDisplay, str,cvPointFrom32f(splines[i].points[splines[i].degree]),1, CV_RGB(0, 0, 255));
					}
				}
			}
			// show?
			if (options.show_flag)
			{
				// set the wait value
				int wait = options.step_flag ? 0 : options.wait_arg;
				// show image with detected lanes
				SHOW_IMAGE(imDisplay, "Detected Lanes", wait);
			}
			
			// clear
			cvReleaseMat(&imDisplay);
		}
		
// 		
		
		
		
// 		cout<<"------> Debug at in "<<__FILE__<<" at line "<<__LINE__<<endl;
// 		cvReleaseMat(&raw_mat);
// 		cvReleaseMat(&channelImage);
// 		cout<<"------> Debug at in "<<__FILE__<<" at line "<<__LINE__<<endl;
	}
	
	
	/**
	 * This function reads lines from the input file into a vector of strings
	 *
	 * \param filename the input file name
	 * \param lines the output vector of lines
	 */
	bool ReadLines(const char* filename, vector<string> *lines)
	{
		// make sure it's not NULL
		if (!lines)
			return false;
		// resize
			lines->clear();
			
			ifstream  file;
			file.open(filename, ifstream::in);
			char buf[5000];
			// read lines and process
			while (file.getline(buf, 5000))
			{
				string str(buf);
				lines->push_back(str);
			}
			// close
			file.close();
			return true;
	}

};	//end class0xd6c474
