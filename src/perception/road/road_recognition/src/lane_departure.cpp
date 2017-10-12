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
/** \brief Matlab - lane departure warning system
 *  \file lane_departure.cpp
 *  \author Ricardo Morais
 *  \date Março 2013
 */

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

#define PI 3.1415926
#include "linefinder.h"
#include "edgedetector.h"
#include "lines.h"
#include "road_representation.h"
#include "watershedSegmentation.h"

//Store all constants for image encodings in the enc namespace to be used later.
namespace enc = sensor_msgs::image_encodings;
using namespace Eigen;

//Declare a string with the name of the window that we will create using OpenCV where processed images will be displayed.
static const char WINDOW1[] = "Image Processed";
static const char WINDOW0[] = "Image Raw";

int CountUpperThresh;

void printPts(Point *p,uint size);

//Use method of ImageTransport to create image publisher
image_transport::Publisher pub;

// * Global variables *//
static VectorXi Count_ref(20);			/* Count of each stored line */
static Eigen::MatrixXd Rep_ref(2, 20);
int OutMsg,frame,frame_lost;
Scalar stddev_old;						/* Desvio padrão da imagem anterior */

/**
* Chooses the best lines finded
*/
int  videolanematching( int& MaxLaneNum , int &ExpLaneNum , VectorXi &Enable , MatrixXd &Line , int& TrackThreshold, int &CountUpperThresh)
{
	ROS_INFO("Enter -- videolanematching");
	
	/* Calculate the distances between the lines found in the current frame and those in the repository. */
	MatrixXd List(MaxLaneNum,ExpLaneNum);
	List.setOnes();
	
// 	cout<<"List"<<endl<<List<<endl;
// 	cout<<"Count_ref"<<endl<<Count_ref<<endl;
// 	cout<<"Line"<<endl<<Line<<endl;

	for (int i = 0 ; i<MaxLaneNum ; i++)
	{
		for (int j = 0 ; j<ExpLaneNum ; j++)
		{
			if ( (Count_ref(i)>0) && (Enable(j)==1) )
			{
				List(i,j) = abs(Line(0,j) - Rep_ref(0,i)) + abs(Line(1,j) - Rep_ref(1,i)) * 200.;
			}
		}
	}
	
	/* Find the best matches between the current lines and those in the repository */
	VectorXd Match_dis(MaxLaneNum);
	Match_dis.setOnes();

	Eigen::MatrixXi Match_list(2,MaxLaneNum);
	Match_list.setZero();

	int rowInd=-1;
	int colInd=-1;
	
	for (int i = 0 ; i<ExpLaneNum ; i++)
	{
		if (i>0) 
		{	/* Reset the row and column where the minimum element lies on. */
			List.row(rowInd).setOnes();
			List.col(colInd).setOnes();
		}
		/* 	In the 1st iteration, find minimum element (corresponds to
			best matching targets) in the distance matrix. Then, use the
			updated distance matrix where the minimun elements and their
			corresponding rows and columns have been reset. */
		
		MatrixXi::Index min_i, min_j;
		double minOfM = List.minCoeff(&min_i,&min_j);
		
		Match_dis(i) = minOfM;
		rowInd = min_i;
		colInd = min_j;
		
		Match_list(0,i) = min_i;
		Match_list(1,i) = min_j;
// 		cout<<"Match_list"<<endl<<Match_list<<endl;
// 		cout<<"Match_dis"<<endl<<Match_dis<<endl;
	}
	
	/* Update reference target list.
		If a line in the repository matches with an input line, replace
		it with the input one and increase the count number by one;
		otherwise, reduce the count number by one. The count number is
		then saturated. */
	VectorXi Ones(Count_ref.size());
	Ones.setOnes();
	
	Count_ref = Count_ref - Ones;
	
	for (int i = 0 ; i<ExpLaneNum ; i++)
	{
		if (Match_dis(i)>TrackThreshold)
		{
			/* Insert in an unused space in the reference target list */
			int NewInd = 0;
			do{
				NewInd++;
			} while ( Count_ref(NewInd) > 0 );
			
			Rep_ref.col(NewInd) = Line.col( Match_list(1,i) );
			Count_ref( NewInd ) = Count_ref( NewInd ) + 2;
		}
		else
		{
			/* Update the reference list */
			Rep_ref.col( Match_list(0,i) ) = Line.col( Match_list(1,i) );
			Count_ref(Match_list(0,i)) = Count_ref(Match_list(0,i)) + 2;
		}
	}
		
	for (int i = 0;i<Count_ref.size(); i++)
	{
		if (Count_ref(i) < 0)
			Count_ref(i) = 0;
		
		if (Count_ref(i) > CountUpperThresh)
			Count_ref(i) = CountUpperThresh;
	}
	
	ROS_INFO("Exit -- videolanematching");
	return 1;
}
	

/**
* Detectar se a linha e amarela/branca e se e traço continuo ou tracejado 
*/
VectorXi videodetectcolorandtype(MatrixXi twoLines_0b , cv::Mat image ,MatrixXi li)
{
	ROS_INFO(" Enter -- videodetectcolorandtype");
	/* Declaração de todas as variaveis locais a usar */
	int INVALID_COLOR = 0;
	int WHITE_COLOR = 1;
	int YELLOW_COLOR = 2;
	
	int INVALID_MARKING = 0;
	int BROKEN_MARKING = 1;
	int SOLID_MARKING = 2; 
	
	int INVALID_COLOR_OR_TYPE = 1;
	int YELLOW_BROKEN = 2;
	int YELLOW_SOLID = 3;
	int WHITE_BROKEN = 4;
	int WHITE_SOLID = 5;
	
	VectorXi colorAndTypeIdx(2);
	colorAndTypeIdx.setZero();
	
	VectorXi lineColorIdx(2);
	lineColorIdx.setZero();
	
	VectorXi solidBrokenIdx(2);
	solidBrokenIdx.setZero();
	
	int HALF_OFFSET = 10;
	
	int rH = image.rows;
	int cW = image.cols;
	
	VectorXi leftC( rH );
	VectorXi rightC( rH );
	VectorXi Rs( rH );	
	
	Rs.setZero();
	leftC.setZero();
	rightC.setZero();
	
	bool line_within_image;
	
	int whiteCount;
	int yelowCount;
	int grayCount;
	int SumOfGotAlLeastOneWhitePixelInTheRow;
	int SumOfGotAlLeastOneYellowPixelInTheRow;
	
	bool gotAlLeastOneWhitePixelInTheRow;
	bool gotAlLeastOneYellowPixelInTheRow;
	
	int yellowVsTotal;
	int whiteVsTotal;
	
	int linearPixelRatio;
	int pointNotLine = 0;
	int len;
	
	/* Converter para YCbCr e criar um vector com as varias camadas da imagem */
	cvtColor(image , image ,CV_BGR2YCrCb,CV_8U);
	vector<cv::Mat> channels;
	
	/* separar a imagem em camadas */
	split(image, channels);
	
	MatrixXi twoLines_1b( twoLines_0b.rows() , twoLines_0b.cols() );
	MatrixXi ones( twoLines_0b.rows() , twoLines_0b.cols() );
	ones.setOnes();
	
	twoLines_1b.row(0).swap(twoLines_0b.row(1));
	twoLines_1b.row(1).swap(twoLines_0b.row(0));
	twoLines_1b.row(2).swap(twoLines_0b.row(3));
	twoLines_1b.row(3).swap(twoLines_0b.row(2));
	
	twoLines_1b = twoLines_1b + ones;
	
// 	cout<<"twoLines_1b"<<endl<<twoLines_1b<<endl;
	// check all lines
	for(uint lineIdx=0 ; lineIdx<2 ; lineIdx++)
	{
		VectorXi r1c1r2c2(4);
		r1c1r2c2 = twoLines_1b.col(lineIdx);
// 		cout<<"r1c1r2c2"<<endl<<r1c1r2c2<<endl;
		
		for (uint i=0; i<4 ; i++)
		{
			if (r1c1r2c2(i) > rH)
				r1c1r2c2(i) = rH;
			
			if (r1c1r2c2(i) > cW)
				r1c1r2c2(i) = cW;
		}
		
		int r1 = r1c1r2c2(0);
		int c1 = r1c1r2c2(1);
		int r2 = r1c1r2c2(2);
		int c2 = r1c1r2c2(3);
		
		
		
		/* make sure r1 is the min (r1,r2) */
		if (r1>r2)
		{
			int tmp;
			tmp=r2;
			r2 = r1;
			r1 = tmp;
			
			tmp = c2;
			c2 = c1;
			c1 = tmp;
		}
		
		if ( (r1==r2) && (c1==c2) )
		{
			pointNotLine = 1;
		}
		else
		{
			pointNotLine = 0;
		}
			
		/* find if line is within image: (r1,c1)  and (r2,c2) must be within image */
		if ( (r1>0 && r1<=rH) && (c1>0 && c1<=cW) && (r2>0 && r2<=rH) && (c2>0 && c2<=cW) && pointNotLine == 0 )
		{
			line_within_image = true;
		}
		else
		{
			line_within_image = false;
		}

		
		if (line_within_image == true)
		{
			len = r2-r1+1;
// 			cout<<"-> len : "<<len<<endl;
			int i = 0;
			for (int p=r1; p<r2 ; p++)
			{
				Rs(i) = p;
				i++;
			}
			double quotient = (c2 - c1)/(len - 1);

			for (int i=0; i<len ; i++)
			{
				leftC(i) = (c1 - HALF_OFFSET) + (i-1)*quotient;
				rightC(i) = leftC(i) + 2*HALF_OFFSET;
				
				if (leftC(i)<1)
				{
					leftC(i) = 1;
					
					if (rightC(i) < 1)
					{
						rightC(i) = 1;
					}
				}
				
				if (rightC(1) > cW)
				{
					rightC(i) = cW;
				}
			}
			
			whiteCount = 0;
			yelowCount = 0;
			grayCount = 0;
			
			SumOfGotAlLeastOneWhitePixelInTheRow = 0;
			SumOfGotAlLeastOneYellowPixelInTheRow = 0;
			
			for (int i = 0; i < len ; i++)
			{
				gotAlLeastOneWhitePixelInTheRow = false;
				gotAlLeastOneYellowPixelInTheRow = false;
				
				for (int cv = leftC(i); cv<rightC(i) ; cv++)
				{
					if ( (int)channels[0].at<uchar>(Rs(i) , cv) >= 175)
					{
						whiteCount++;
						gotAlLeastOneWhitePixelInTheRow = true;
					}
					else if ( ((int)channels[2].at<uchar>(Rs(i),cv) >= 90) && ((int)channels[2].at<uchar>(Rs(i),cv) <= 127) )
					{
						yelowCount++;
						gotAlLeastOneYellowPixelInTheRow  = true;
					}
					else
					{
						grayCount++;
					}
				}
				
				if (gotAlLeastOneWhitePixelInTheRow == true)
					SumOfGotAlLeastOneWhitePixelInTheRow ++;
			
				if (gotAlLeastOneYellowPixelInTheRow == true)
					SumOfGotAlLeastOneYellowPixelInTheRow ++;
			}

				yellowVsTotal = yelowCount/(grayCount + yelowCount + whiteCount);
				whiteVsTotal = whiteCount/(grayCount + yelowCount + whiteCount);

			
			if (yellowVsTotal > whiteVsTotal)
			{
				lineColorIdx(lineIdx) = YELLOW_COLOR;
				linearPixelRatio = SumOfGotAlLeastOneYellowPixelInTheRow/len;
			}
			else
			{
				lineColorIdx(lineIdx) = WHITE_COLOR;
				linearPixelRatio = SumOfGotAlLeastOneWhitePixelInTheRow/len;
			}
			
// 			cout<<"LinearPixelRatio :"<<linearPixelRatio<<endl;
			
			if (linearPixelRatio > 0.8)
			{
				solidBrokenIdx(lineIdx) = SOLID_MARKING;
			}
			else
			{
				solidBrokenIdx(lineIdx) = BROKEN_MARKING;
			}
			
			if ( (lineColorIdx(lineIdx) == YELLOW_COLOR)  && (solidBrokenIdx(lineIdx) == BROKEN_MARKING) )
			{
				colorAndTypeIdx(lineIdx) = YELLOW_BROKEN;
			}
			else if ( (lineColorIdx(lineIdx) == YELLOW_COLOR) && (solidBrokenIdx(lineIdx) == SOLID_MARKING) )
			{
				colorAndTypeIdx(lineIdx) = YELLOW_SOLID;
			}
			else if ( (lineColorIdx(lineIdx) == WHITE_COLOR) && (solidBrokenIdx(lineIdx) == BROKEN_MARKING) )
			{
				colorAndTypeIdx(lineIdx) = WHITE_BROKEN; 
			}
			else if ( (lineColorIdx(lineIdx) == WHITE_COLOR) && (solidBrokenIdx(lineIdx) == SOLID_MARKING) )
			{
				colorAndTypeIdx(lineIdx) = WHITE_SOLID;
			}
		}
		else
		{
			lineColorIdx(lineIdx) = INVALID_COLOR;
			solidBrokenIdx(lineIdx) = INVALID_MARKING;
			colorAndTypeIdx(lineIdx) = INVALID_COLOR_OR_TYPE;
		}
	}
	
	ROS_INFO(" Exit -- videodetectcolorandtype --");
	return colorAndTypeIdx;
}
		
	
/**
 * Detect whether there is a lane departure
 */
int videodeparturewarning(MatrixXi Pts, cv::Mat Imlow ,MatrixXi &TwoLanes, int &TwoValidLanes,int &NumNormalDriving , int &MaxLaneNum)
{
	ROS_INFO("Enter -- videodeparturewarning");
	
	VectorXi Left_pts(4);
	Left_pts.setZero();
	
	VectorXi Right_pts(4);
	Right_pts.setZero();
	
	VectorXi TmpLeftPts(4);
	TmpLeftPts.setZero();
	
	VectorXi TmpRightPts(4);
	TmpRightPts.setZero();
	
	int RawMsg;	
	int Left_dis =  111111111;
	int Right_dis =  111111111;
	int Dis_inf = Imlow.cols;
	int ColNum = -1;
	int Halfwidth = Dis_inf * 0.5;
	int centerDis = -1;	
	
	MatrixXi Pts_local;
	MatrixXi Pts_tmp(Pts.rows(),Pts.cols());
	Pts_tmp.col(0).swap(Pts.col(1));
	Pts_tmp.col(1).swap(Pts.col(0));
	Pts_tmp.col(2).swap(Pts.col(3));
	Pts_tmp.col(3).swap(Pts.col(2));
	
// 	Pts_local = Pts_tmp.adjoint();
	Pts_local = Pts_tmp.transpose();
// 	cout<<"Pts_local"<<endl<<Pts_local<<endl;
	for (int i=0 ; i<MaxLaneNum ; i++)
	 {
		/* Pick the column corresponding to the point closer to the top */
		if ( Pts_local(0,i) >= Pts_local(2,i) )
		{
			ColNum = Pts_local(1,i);
		}
		else
		{
			ColNum = Pts_local(3,i);
		}
		
		if (Count_ref(i) >= 5)
		{
			centerDis = abs(Halfwidth - ColNum);
		}
		else
		{
			centerDis = Dis_inf;
		}
			
		/* distancia do centro da imagem */
		if ((Halfwidth - ColNum) >= 0)  /* Left lane */
		{
			if (centerDis < Left_dis)
			{
				Left_dis = centerDis;
				Left_pts = Pts_local.col(i);
			}
		}
		else							/* Right lane */
		{
			if (centerDis < Right_dis)
			{
				Right_dis = centerDis;
				Right_pts = Pts_local.col(i);
			}
		}
	 }
	 
	 /* Departure detection */
	if (Left_dis <= Dis_inf)
	{
		TmpLeftPts = Left_pts;
	}
	else
	{
		TmpLeftPts.setZero();
	}
	
	if (Right_dis <= Dis_inf)
	{
		TmpRightPts = Right_pts;
	}
	else
	{
		TmpRightPts.setZero();
	}

	TwoLanes << TmpLeftPts,TmpRightPts;
	/* Check whether both lanes are valid */
	int Check1 = 0;
	int Check2 = 0;
	
	if ( (TwoLanes.row(0)!=TwoLanes.row(2)) | (TwoLanes.row(1)!=TwoLanes.row(3)) )
	{
		Check1 = 1;
	}
	
	if ((abs(TwoLanes.row(0).sum() - TwoLanes.row(2).sum()) + abs(TwoLanes.row(1).sum() - TwoLanes.row(3).sum())) >=10 )
	{
		Check2 = 1;
	}
	if ( (Left_dis <= Dis_inf) && (Right_dis <= Dis_inf) && (TwoLanes(0,0)>=0) && (TwoLanes(0,1)>=0) & (Check1==1) & (Check2==1) )
	{
		TwoValidLanes = 1;
	}
	
	int Diswarn = Dis_inf * 0.4; /* Distance threshold for departure warning */
	
// 	cout<<"Left_dis="<<Left_dis<<endl;
// 	cout<<"Right_dis="<<Right_dis<<endl;
// 	cout<<"Diswarn="<<Diswarn<<endl;
	
	
	if (Left_dis <= Diswarn && Left_dis <= Right_dis)
	{
		RawMsg = 2;
		ROS_ERROR("WARNING -- Left lane departure");
	}
	else if (Right_dis <= Diswarn && Left_dis > Right_dis)
	{
		RawMsg = 0;
		ROS_ERROR("WARNING -- Right lane departure");
	}
	else
	{
		RawMsg = 1;
	}
	
	/* The following code combines left-right departure to left departure and
	right-left departure to right departure. It utilizes the fact that there
	must be at least 4 frames of normal driving between a left departure
	warning and a right departure warning.*/
	if (RawMsg == 1)
		NumNormalDriving++;
	
	if (RawMsg == 1 || NumNormalDriving >= 4)
		OutMsg = RawMsg;
	
	if (RawMsg != 1)
		NumNormalDriving = 0;
	
	MatrixXi TwoLanes_tmp(TwoLanes.rows(), TwoLanes.cols());
	TwoLanes_tmp = TwoLanes;
	TwoLanes.row(0).swap(TwoLanes_tmp.row(1));
	TwoLanes.row(1).swap(TwoLanes_tmp.row(0));
	TwoLanes.row(2).swap(TwoLanes_tmp.row(3));
	TwoLanes.row(3).swap(TwoLanes_tmp.row(2));
// 	cout<<"TwoLanes:"<<endl<<TwoLanes<<endl;
	ROS_INFO("Exit -- videodeparturewarning");
	return 1;
}

/**
 *  Detect the limit to cut the image
 */
int framedivider(Mat image)
{
	ROS_INFO("ENTER -- frmedivider");
	int rows;
	// Create watershed segmentation object

	WatershedSegmenter segmenter;
	
	// Create markers
	Mat markers( image.rows , image.cols , CV_8U , Scalar(0) );
	int height = 20;
	
	// Creat the markers to the image
	Point pt1 = Point(0 , 0);
	Point pt2 = Point(image.cols-1 , height);
	Point pt3 = Point(0, image.rows-1-height);
	Point pt4 = Point(image.cols-1, image.rows-1);
	
	rectangle(markers, pt1, pt2, cvScalar(255) );
	rectangle(markers, pt3, pt4, cvScalar(128) );
	
	cout<<"->  Error"<<endl;
	imshow("markers",markers);
	
	// Set markers and process
	segmenter.setMarkers(markers);
// 	cvtColor(image,image,CV_RGB2GRAY,CV_8U);
	segmenter.process(image);
	
	// Display segmentation result
	namedWindow("Segmentation");
	imshow("Segmentation",segmenter.getSegmentation());

	// Display watersheds
	namedWindow("Watersheds");
	imshow("Watersheds",segmenter.getWatersheds());
	
	std::vector<int> y;
	y.clear();
	
	Mat Watersheds = segmenter.getWatersheds();
	/* Encontrar todos os pixeis pretos e guardar a sua coordenada segundo Y*/
	for (int col=0 ; col<image.cols ; col++)
	{
		for (int row=0 ; row<image.rows ; row++)
		{
			if ( (int)Watersheds.at<uchar>(row , col) <= 5 )
			{
				y.push_back(row);
// 				break;
			}
		}
	}
	
	/* Calcular a média das coordenada segundo y da fronteira entre as regiões segmentadas */
	cout<<"size y ="<<y.size()<<endl;
	cout<<"mean(y) ="<<mean(y)<<endl;
	return (int)mean(y)[0];
	
// 	int min = std::min(y.front(), y.back());
// 	return min;
}

/**
*This function is called everytime a new image is published
*/
 int imageCallback(const sensor_msgs::ImageConstPtr& original_image)
{
// 	int x=0 , y=0;
	//Convert from the ROS image message to a CvImage suitable for working with OpenCV for processing
	cv_bridge::CvImagePtr cv_ptr;
	cv::Mat im_gray;	/* imagem recebida em niveis de cinza*/
	cv::Mat YCbCr;
	Mat subImg;
	
	cv::Mat original_subImg;
	cv::Mat cv_clone;
	Mat cv_clone_gray;
	Mat top_img;
	Mat bgr_image;
	
	try
	{
// 		Always copy, returning a mutable CvImage
// 		OpenCV expects color images to use BGR channel order.
		cv_ptr = cv_bridge::toCvCopy(original_image, enc::BGR8);
		cv_clone = cv_ptr->image.clone();
	}
	catch (cv_bridge::Exception& e)
	{
		/* if there is an error during conversion, display it */
		ROS_ERROR("tutorialROSOpenCV::main.cpp::cv_bridge exception: %s", e.what());
		return 0;
	}
	
	//// Variáveis a usar ////////////////////////////////////////////////////////////////////////////////
	int NumRows = 120;									/* Number of rows in the image region to process */
	int MaxLaneNum = 20; 								/* Maximum number of lanes to store in the tracking repository */
	int ExpLaneNum = 2;  								/* Maximum number of lanes to find in the current frame. */
	
	int TrackThreshold = 75; 							/* Maximum allowable change of lane distance */
	int frameFound = 5;
	int frameLost = 20;
	
	cv::Point anchor;
	VectorXi colorAndTypeIdx;
	
	MatrixXi TwoLanes1(4,2);
	
	VectorXd Theta;
	VectorXd Rho;
	//// Process the images ////////////////////////////////////////////////////////////////////////////////
	VectorXi size(3);
	size << NumRows - 20 , NumRows + 20, NumRows;
	
	NumRows = framedivider(cv_clone);
	cout<<"NumRows ="<<NumRows<<endl;
/* Select the lower portion of the input */
	Rect ROI_1(0, NumRows,cv_clone.cols,cv_clone.rows-NumRows);	/* Create my roy */
	/* Select the upper portion of the input */
	Mat upperImg = cv_clone(Range(0,NumRows),Range(0,cv_clone.cols)); /* Create my roy */
	
	subImg = cv_clone(Range(NumRows,cv_clone.rows),Range(0,cv_clone.cols));
	original_subImg = cv_clone(ROI_1);
	bgr_image = cv_clone(ROI_1); 
	
	cv::cvtColor(subImg,subImg,CV_RGB2GRAY,CV_8U);
	cv::cvtColor(original_subImg,original_subImg,CV_RGB2GRAY,CV_8U);
	cvtColor(upperImg,upperImg,CV_RGB2GRAY,CV_8U);
	//////////////////////////////////////////////////////////////////
	
	cv::Mat kernel(1,3,CV_32F,cv::Scalar(0));
	// assigns kernel values
	kernel.at<float>(0,0)= -1;
	kernel.at<float>(0,1)= 0;
	kernel.at<float>(0,2)= 1;
	cv::Mat Edge;
	
	//filter the image
	cv::filter2D(subImg,Edge,subImg.depth(),kernel);
	cv::threshold(Edge,Edge,100,255,cv::THRESH_OTSU);
	// Display Edge
	cv::namedWindow("Edge");
	cv::imshow("Edge",Edge);
	
	//////////////////////////
	/*		HoughLines P	*/
	/////////////////////////	
	// Create LineFinder instance
	LineFinder ld;
	
	// Set probabilistic Hough parameters
	ld.setLineLengthAndGap(10,170);
	ld.setMinVote(30);
	
	// Detect lines
	cv::Mat image(Edge.rows,Edge.cols,CV_8U,cv::Scalar(0));

	
	vector<cv::Vec4i> li= ld.findLines(Edge);
// 							waitKey(0);
	
	if (li.size() == 0)
	{
		
		ROS_ERROR("Nenhuma linha encontrada");
		frame_lost++;
		return 0;
	}
	else if (li.size() ==1 )
	{
		ROS_ERROR("So uma linha encontrada");
		frame_lost++;
		return 0;
	}
	
	/* Extrair as melhores linhas */
	MatrixXi Pts;
	
	Lines lines;
	lines.end_points=li;
	lines.calculatePolarCoodinates();
	
	MatrixXd Line(2,li.size());
	Line.setZero();
	VectorXi Enable(ExpLaneNum);
	Enable.setOnes();
	
	
	/* Obter os extremos das linhas encontradas */
	for (uint i=0 ; i<li.size() ; i++)
	{
		Line(0,i) = lines.rhos[i];
		Line(1,i) = lines.thetas[i];
	}
	
	ld.drawDetectedLines(image);
// 	ld.drawDetectedLines(imagea);
// 	ld.drawDetectedLines(imageb);
	
	namedWindow("Original");
	imshow("Original",cv_ptr->image);
	
	cv::namedWindow("Detected Lines with HoughP");
	cv::imshow("Detected Lines with HoughP",image);
	
// 	cv::namedWindow("Detected Lines with HoughP imga");
// 	cv::imshow("Detected Lines with HoughP imga",imagea);
// 	
// 	cv::namedWindow("Detected Lines with HoughP imgb");
// 	cv::imshow("Detected Lines with HoughP imgb",imageb);
// 	waitKey(0);
	
	MatrixXi TwoLanes(4,2);
	TwoLanes.setZero();
	int CountUpperThresh = frameFound+frameLost;
	
// 	for (uint ff=0; ff<li.size() ; ff++)
// 		cout<<"llliiiii"<<li[ff]<<endl;
// 	
// 	cout<<"MaxLaneNum: "<<MaxLaneNum<<endl;
// 	cout<<"ExpLaneNum: "<<ExpLaneNum<<endl;
// 	cout<<"Enable: "<<Enable<<endl;
// 	cout<<"Line: "<<Line<<endl;
// 	cout<<"TrackThreshold: "<<TrackThreshold<<endl;
// 	cout<<"CountUpperThresh: "<<CountUpperThresh<<endl;
// 	cout<<"Rep_ref"<<endl<<Rep_ref<<endl;
// 	cout<<"Count_ref"<<endl<<Count_ref<<endl;
	
	
	
	/* Escolher quais as melhores linhas */
	videolanematching(MaxLaneNum, ExpLaneNum, Enable, Line,TrackThreshold, CountUpperThresh);
// 	cout<<"Rep_ref"<<endl<<Rep_ref<<endl;
// 	cout<<"Count_ref"<<endl<<Count_ref<<endl;
	
// 	return 0;
	
	int TwoValidLanes = 0;
	int NumNormalDriving = 0;
	
	/* Converter as coordenadas polares para sartesianas */
	Pts = lines.extremePoints( subImg.rows , subImg.cols );
	
// 	cout<<"Pts"<<endl<<Pts<<endl;
// 	cout<<"Twolanes"<<endl<<TwoLanes<<endl;
// 	cout<<"TwoValidLanes"<<endl<<TwoValidLanes<<endl;
// 	cout<<"NumNormalDriving"<<endl<<NumNormalDriving<<endl;
// 	cout<<"MaxLaneNum"<<endl<<MaxLaneNum<<endl;
// 	cout<<"OutMsg :"<<OutMsg<<endl;
// 	
// 	cout<<"Antes -----------------------------"<<endl;
	/* Detectar se existe mudança de faixa de rodagem */
	videodeparturewarning(Pts , subImg ,TwoLanes,TwoValidLanes,NumNormalDriving,MaxLaneNum);
	
// 	cout<<"TwoValidLanes :"<<TwoValidLanes<<endl;
// 	cout<<"NumNormalDriving :"<<NumNormalDriving<<endl;
// 	cout<<"TwoLanes"<<endl<<TwoLanes<<endl;
// 	cout<<"OutMsg :"<<OutMsg<<endl;	
	
	//////////////////////////////////////////
	/* Detectar o tipo de linha e a cor desta */
	colorAndTypeIdx = videodetectcolorandtype(TwoLanes , bgr_image , Pts);
// 	cout<<"colorAndTypeIdx :"<<colorAndTypeIdx<<endl;
// 	waitKey(0);
// 	exit(0);
	
	/* Meaning of ColorAndTypeIdx:
    INVALID_COLOR_OR_TYPE = int8(0);
    YELLOW_BROKEN = int8(1); YELLOW_SOLID = int8(2);
    WHITE_BROKEN = int8(3);  WHITE_SOLID = int8(4). */
	
	if (TwoValidLanes == 1)
	{
		ostringstream str_right;
		ostringstream str_left;
		ostringstream str_warning;
		cv::Mat poly_img(cv_clone.rows,cv_clone.cols,CV_8U,cv::Scalar(0));
		
		/* Right Lane*/
		switch ( colorAndTypeIdx(0) )
		{
			case 0:
				str_right << "INVALID_COLOR_OR_TYPE";
				break;
				
			case 1:
				str_right << "YELLOW_BROKEN";
				break;
				
			case 2:
				str_right << "YELLOW_SOLID";
				break;
				
			case 3:
				str_right << "WHITE_BROKEN";
				break;
				
			case 4:
				str_right <<"WHITE_SOLID";
				break;
		}
		
// 		putText(cv_clone,str_right.str() , cvPoint(10,100), 
// 				FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(255,255,255), 1, CV_AA);
	
		/* Left Lane */
		switch ( colorAndTypeIdx(1) )
		{
			case 0:
				str_left << "INVALID_COLOR_OR_TYPE";
				break;
				
			case 1:
				str_left << "YELLOW_BROKEN";
				break;
				
			case 2:
				str_left << "YELLOW_SOLID";
				break;
				
			case 3:
				str_left << "WHITE_BROKEN";
				break;
				
			case 4:
				str_left << "WHITE_SOLID";
				break;
		}
		
		/* Escrever texto na imagem */
// 		putText(cv_clone, str_left.str() , cvPoint(200,100), 
// 				FONT_HERSHEY_COMPLEX_SMALL, 0.7, cvScalar(255,255,255), 1, CV_AA);	

		switch (OutMsg)
		{
			case 0:
				str_warning << "Right lane departure";
				break;
				
			case 2:
				str_warning << "Left lane departure";
				break;
				
			case 1:
				str_warning << "";
			break;
							
		}
		/* Escrever texto na imagem */
// 		putText(cv_clone, str_warning.str() , cvPoint(100,50),	
// 				FONT_HERSHEY_COMPLEX_SMALL, 0.7, cvScalar(255,0,255), 1, CV_AA);	
		
		VectorXi Tempr(2);
		VectorXi Templ(2);
		Tempr.setZero();
		Templ.setZero();
		
		/* Ofsset para compensar a imagem cortada */
		MatrixXi offset(4,2);
		int off = cv_clone.rows - subImg.rows; 
		offset<<0, 0,
		off, off,
		0  ,   0, 
		off, off;

		TwoLanes1 = TwoLanes + offset;
		
		/* Criar os pontos extremos casos estes existam */
		if ( TwoLanes(3,0) >= 230 )
		{
			Templ(0) = TwoLanes1(2, 0);
			Templ(1) = TwoLanes1(3, 0);
		}
		else
		{
			Templ(0) = 0;
			Templ(1) = cv_clone.rows;//-subImg.rows;
		}
		
		if (TwoLanes(3,1) >= 230)
		{
			Tempr(0) = TwoLanes1(2, 1);
			Tempr(1) = TwoLanes1(3, 1);
		}
		else
		{
			Tempr(0) = 359;
			Tempr(1) = cv_clone.rows;//-subImg.rows;
		}
		/////////////////////////////////////////
		/* Creat a poligom image with the road	*/
		/////////////////////////////////////////	
		int npts = 6;
		int npt[] = { 6 };
		
		Point Pts_poly[1][npts];
		Pts_poly[0][0] = cv::Point( TwoLanes1(0,0) , TwoLanes1(1,0) );
		Pts_poly[0][1] = cv::Point( TwoLanes1(2,0) , TwoLanes1(3,0) );
		Pts_poly[0][2] = cv::Point( Templ(0) , Templ(1) );
		Pts_poly[0][3] = cv::Point( Tempr(0) , Tempr(1) );
		Pts_poly[0][4] = cv::Point( TwoLanes1(2,1),TwoLanes1(3,1) );
		Pts_poly[0][5] = cv::Point( TwoLanes1(0,1),TwoLanes1(1,1) );
		
		const Point *ppt[1]={ Pts_poly[0] };
		
		fillPoly( cv_clone, ppt, npt, 1, Scalar( 200, 200, 200 ),8);
		
		cv::namedWindow("Polygon road - 2");
		cv::imshow("Polygon road - 2",cv_clone);
	}

	frame++;
	int total = frame + frame_lost;
	double percentage = (frame_lost * 100) / total;
	cout<<"Numero de frames perdidos = "<<frame_lost<<endl<<"Numero de frames analisados = "<<total<<endl;
	cout<<"Percentagem de frames perdidos = "<<percentage<<" %"<<endl;

	cv::waitKey(10);
	return 1;
	
}

/**
 * função para imprimir os pontos do poligno
 */
void printPts(Point *p,uint size)
{
// 	cout<<"Poligon Points"<<endl;
	for(uint i=0;i<size;i++)
		cout<<p[i]<<endl;
}

/**
 *  intersept Ctrt+C handler
 */
void sighandler(int sig)
{
    ROS_ERROR("Signal %d caught...",sig);
	cout<<"Shuting down road_recognition"<<endl;
	exit(0);
}

/**
* Always running ( check if there is a new message )
*/
int main(int argc, char **argv)
{
	ros::init(argc, argv, "lane_processor");
	
	/* Por as variáveis globais a zero */
	Rep_ref.setZero();
	Count_ref.setZero();
	OutMsg = -1;
	frame = 0;
	frame_lost = 0;
	
	ros::NodeHandle nh;
	/* Create an ImageTransport instance, initializing it with our NodeHandle. */
	image_transport::ImageTransport it(nh);
	/* OpenCV HighGUI call to create a display window on start-up. */
	cv::namedWindow(WINDOW0, CV_WINDOW_AUTOSIZE);
	cv::namedWindow(WINDOW1, CV_WINDOW_AUTOSIZE);
	
	signal(SIGINT, &sighandler);

	image_transport::Subscriber sub = it.subscribe("snr/cam/0/image_color", 1, imageCallback);
	
	/* OpenCV HighGUI call to destroy a display window on shut-down. */
	cv::destroyWindow(WINDOW0);
	cv::destroyWindow(WINDOW1);
	
	pub = it.advertise("image_processed", 1);
	
	ros::spin();
	ROS_INFO("tutorialROSOpenCV::main.cpp::No error.");
	
}
