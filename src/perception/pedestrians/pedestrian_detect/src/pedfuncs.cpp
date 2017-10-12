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
#include "peddetect.h"


Mat GradientMagnitude(Mat src)

{
   Mat xsobel, ysobel, xsobel2, ysobel2, xysobel, GradMag, GrayImg;
   
   cvtColor(src,GrayImg, CV_RGB2GRAY,0);
  
   Sobel(GrayImg, xsobel, CV_32F, 1, 0, 3, 1, 0, BORDER_DEFAULT);
   Sobel(GrayImg, ysobel, CV_32F, 0, 1, 3, 1, 0, BORDER_DEFAULT);
   
   multiply(xsobel, xsobel, xsobel2, 1, -1 );
   multiply(ysobel, ysobel, ysobel2, 1, -1 );
   
   xysobel=xsobel2+ysobel2;
   sqrt(xysobel,GradMag);
   
   return GradMag;
  
}

Mat GradientMagnitude(Mat xsobel, Mat ysobel)

{
   Mat xsobel2, ysobel2, xysobel, GradMag, GrayImg;
   
   multiply(xsobel, xsobel, xsobel2, 1, -1 );
   multiply(ysobel, ysobel, ysobel2, 1, -1 );
   
   xysobel=xsobel2+ysobel2;
   sqrt(xysobel,GradMag);
   
   return GradMag;
  
}


MatVector OrientedGradientsDiagram(Mat GradMag, Mat xsobel, Mat ysobel)
{
  Mat xsobeldiv=xsobel + 0.00001;
  Mat div;
  float angl;
  
  MatVector BinVector;
   
  divide(ysobel,xsobeldiv,div);

   
   Mat Bins[6];
    
    for (int n=0; n<6; n++)
    {
      Bins[n].create(GradMag.rows,GradMag.cols,CV_32FC1);
      Bins[n] = Mat::zeros(GradMag.rows,GradMag.cols,CV_32FC1);
    }
 for( int r = 0; r < div.rows; r++ )
    {
      for( int c = 0; c < div.cols; c++ )
      {
	
	angl = ((atan (div.at<float>(r,c))) * (180 / PI)) + 90;
	if (angl<=30)
	{
	  Bins[0].at<float>(r,c) = GradMag.at<float>(r,c);
	  
	}
	
	else if (angl<=60)
	{
	  Bins[1].at<float>(r,c) = GradMag.at<float>(r,c);
	  
	}
	else if (angl<=90)
	{
	  Bins[2].at<float>(r,c) = GradMag.at<float>(r,c);
	  
	}
	else if (angl<=120)
	{
	  Bins[3].at<float>(r,c) = GradMag.at<float>(r,c);
	  
	}
	
	else if (angl<=150)
	{
	  Bins[4].at<float>(r,c) = GradMag.at<float>(r,c);
	  
	}
	else
	{
	  Bins[5].at<float>(r,c) = GradMag.at<float>(r,c);
	  
	}
      }
    
    }
  
  for(int i=0; i<6; i++)
  {
    BinVector.push_back(Bins[i]);
    
  }
  
  return BinVector;
  
}

MatVector LUVcolourchannels(Mat Img)
{
  Mat LUV(Img.rows, Img.cols, CV_32FC3);
  MatVector LUVchannels; 
  
  cvtColor(Img,LUV, CV_RGB2Luv,CV_32F);
  
  LUV.assignTo(LUV, CV_32F );
  
  split(LUV,LUVchannels);
  
  
  return LUVchannels;
  
}


 void GetFileList(string folder_path, PVector & vect)
{
  
  path p = folder_path;
  
  try
  {
    if (exists(p))    // does p actually exist?
    {

      if (is_directory(p))      // is p a directory?
      {
	
        copy(directory_iterator(p), directory_iterator(), back_inserter(vect)); 
	
      }
      else
      cout << p << " is not a folder"<<endl;

    }
   
  }
  

  catch (const filesystem_error& ex)
  {
    cout << ex.what() << '\n';
  }
  
  
}


void ComputeChannels(Mat Image, Mat & MergedChannels)
{
  
  Mat xsobel, ysobel, GradMag, GrayImg;
  MatVector BinVect, LUVchannels;
  
  cvtColor (Image, GrayImg, CV_RGB2GRAY, 0);
  
  Sobel (GrayImg, xsobel, CV_32FC1, 1, 0, 3, 1, 0, BORDER_DEFAULT);
  Sobel (GrayImg, ysobel, CV_32FC1, 0, 1, 3, 1, 0, BORDER_DEFAULT);
  
  GradMag = GradientMagnitude (xsobel, ysobel);
  
  BinVect = OrientedGradientsDiagram (GradMag, xsobel, ysobel);
  
  LUVchannels = LUVcolourchannels (Image);
  
  
  Mat ToBeMergedChannels[] =
      { GradMag, BinVect[0], BinVect[1], BinVect[2], BinVect[3], BinVect[4],
      BinVect[5], LUVchannels[0], LUVchannels[1], LUVchannels[2]
    };
    
    

    merge (ToBeMergedChannels, CHANNELNR, MergedChannels);

}


void GetRandParams(int seed, int NrFtrs, FtrVecParams2 & randparams, Rect region)
{
  
  cv::RNG rng(seed);
  
//   vector <int> record;
//   ifstream infile("/home/pedrobatista/workingcopy/lar3/perception/pedestrians/PedestrianDetect/PredictorImportance.txt");
// 
//   while (infile)
//   {
//     string s;
//     if (!getline( infile, s )) break;
// 
//     istringstream ss( s );
// 
//     while (ss)
//     {
//       string s;
//       if (!getline( ss, s, ',' )) break;
//       record.push_back( atoi(s.c_str() ));
//     }
//     
//   }
//   
//   if (!infile.eof())
//   {
//     cerr << "Fooey!\n";
//   }
//   

  
  FtrParams2 Params;
  Params.nFtrs=NRFEATURE2;
  
  for (int n=0; n<NrFtrs; n++)
  {
//     Params.channelidx=rng.uniform(0,CHANNELNR);
    
//     if(std::find(record.begin(), record.end(), n) != record.end())
//       
//     {
//     
    Params.width=rng.uniform(MINAREA,MAXAREAWIDTH+1); 
    Params.height=rng.uniform(MINAREA,MAXAREAHEIGHT+1);
    
    Params.x=rng.uniform(0,region.width-Params.width+1);
    Params.y=rng.uniform(0,region.height-Params.height+1);
    
    randparams.push_back(Params);
//     }
    
  }
  

  
}

void GetChnFtrsOverImagePyramid(Mat Image , CvRect & region , vector<float> & features,int nOctUp, Size minSize, int nPerOct, FtrVecParams2 Params,PedRoiVec & PedRect, CvBoost & boost_classifier)
{
  
  
  Size currSize=Image.size();
  float scaledown = pow(2,(-1.0/nPerOct)), scaleup = pow(2,1.0/nPerOct);
  
  Mat currScaledImg=Image , IntegralChns;
  
//   cout<<"currsize w: "<<currSize.width<<" minSize w: "<<minSize.width<<" currsize height: "<<currSize.height<< " minsize height: "<<minSize.height<<endl;
  
  while (currSize.width>=minSize.width && currSize.height>=minSize.height)
  {
    
    Mat MergedChannels (currScaledImg.rows, currScaledImg.cols, CV_32FC (CHANNELNR));
    
    ComputeChannels(currScaledImg,MergedChannels);
   
    integral (MergedChannels, IntegralChns, CV_32FC (CHANNELNR));
    GetChnFtrsOverImage (IntegralChns, region, features, Params,PedRect,boost_classifier);
    
    currSize.width *=scaledown; currSize.height *=scaledown; 
    resize(Image, currScaledImg, currSize, 0, 0, INTER_LINEAR);
    
  
  } 
  
  if (nOctUp !=0)
  {
    
    currSize = Image.size();
    Size maxSize;
    
    maxSize.width = currSize.width+currSize.width*0.5*nOctUp;
    maxSize.height = currSize.height+currSize.height*0.5*nOctUp;
    currScaledImg=Image;
    
    while (currSize.width<maxSize.width && currSize.height<maxSize.height)
  {
    currSize.width *=scaleup; currSize.height *=scaleup; 
    resize(Image, currScaledImg, currSize, 0, 0, INTER_LINEAR); 
    
    Mat MergedChannels (currScaledImg.rows, currScaledImg.cols, CV_32FC (CHANNELNR));
    
    ComputeChannels(currScaledImg,MergedChannels);
   
    integral (MergedChannels, IntegralChns, CV_32FC (CHANNELNR));
   
    GetChnFtrsOverImage (IntegralChns, region, features, Params,PedRect,boost_classifier);
  
  }
    
  }
  
  
}

void GetChnFtrsOverImagePyramid(Mat Image , CvRect & region , vector<float> & features,vector<DVector> &  WindowFtrs,int nOctUp, Size minSize, int nPerOct, FtrVecParams2 Params)
{
  
  
  Size currSize=Image.size();
  float scaledown = pow(2,(-1.0/nPerOct)), scaleup = pow(2,1.0/nPerOct);
  
  Mat currScaledImg=Image , IntegralChns;
  
//   cout<<"currsize w: "<<currSize.width<<" minSize w: "<<minSize.width<<" currsize height: "<<currSize.height<< " minsize height: "<<minSize.height<<endl;
  
  while (currSize.width>=minSize.width && currSize.height>=minSize.height)
  {
    
    Mat MergedChannels (currScaledImg.rows, currScaledImg.cols, CV_32FC (CHANNELNR));
    
    ComputeChannels(currScaledImg,MergedChannels);
   
    integral (MergedChannels, IntegralChns, CV_32FC (CHANNELNR));
    GetChnFtrsOverImage (IntegralChns, region, features,WindowFtrs, Params);
    
    currSize.width *=scaledown; currSize.height *=scaledown; 
    resize(Image, currScaledImg, currSize, 0, 0, INTER_LINEAR);
    
  
  } 
  
  if (nOctUp !=0)
  {
    
    currSize = Image.size();
    Size maxSize;
    
    maxSize.width = currSize.width+currSize.width*0.5*nOctUp;
    maxSize.height = currSize.height+currSize.height*0.5*nOctUp;
    currScaledImg=Image;
    
    while (currSize.width<maxSize.width && currSize.height<maxSize.height)
  {
    currSize.width *=scaleup; currSize.height *=scaleup; 
    resize(Image, currScaledImg, currSize, 0, 0, INTER_LINEAR); 
    
    Mat MergedChannels (currScaledImg.rows, currScaledImg.cols, CV_32FC (CHANNELNR));
    
    ComputeChannels(currScaledImg,MergedChannels);
   
    integral (MergedChannels, IntegralChns, CV_32FC (CHANNELNR));
   
    GetChnFtrsOverImage (IntegralChns, region, features, WindowFtrs ,Params);
  
  }
    
  }
  
  
}


void GetChnFtrsOverImage(Mat IntegralChannels , CvRect & region ,vector<float> & features , FtrVecParams2 Params,PedRoiVec & PedRect,CvBoost & boost_classifier)
{

  int Rows=0, Cols=0;

  for (Rows=0; Rows<IntegralChannels.rows-region.height-1; Rows+=STEP)
  {
    
    for (Cols=0; Cols<IntegralChannels.cols-region.width-1; Cols+=STEP)
    {
      
      
      region.x=Cols ; region.y=Rows;
      GetChnFtrsOverWindow(IntegralChannels ,features,Params,region,PedRect,boost_classifier);
      
      
    }

    
  }

  
    
    
    for (Cols=0 ; Cols<IntegralChannels.cols-region.width-1; Cols+=STEP)
    {
      
    region.x=Cols; region.y=IntegralChannels.rows-region.height-1;
    GetChnFtrsOverWindow(IntegralChannels ,features ,Params,region,PedRect,boost_classifier);
    
    }

  
  

    for (Rows=0 ; Rows<IntegralChannels.rows-region.height-1; Rows+=STEP)
    {
      
    region.y=Rows; region.x=IntegralChannels.cols-region.width-1;
    GetChnFtrsOverWindow(IntegralChannels ,features,Params,region,PedRect,boost_classifier);
    
    
    
    }
    
    region.y=IntegralChannels.rows-region.height-1; region.x=IntegralChannels.cols-region.width-1;
    
    GetChnFtrsOverWindow(IntegralChannels ,features,Params,region,PedRect,boost_classifier);
    
    
  
}

void GetChnFtrsOverImage(Mat IntegralChannels , CvRect & region ,vector<float> & features ,vector<DVector> & WindowFtrs , FtrVecParams2 Params)
{

  int Rows=0, Cols=0;

  for (Rows=0; Rows<IntegralChannels.rows-region.height-1; Rows+=STEP)
  {
    
    for (Cols=0; Cols<IntegralChannels.cols-region.width-1; Cols+=STEP)
    {
      
      
      region.x=Cols ; region.y=Rows;
      GetChnFtrsOverWindow(IntegralChannels ,features,WindowFtrs,Params,region);
      
      
    }

    
  }

  
    
    
    for (Cols=0 ; Cols<IntegralChannels.cols-region.width-1; Cols+=STEP)
    {
      
    region.x=Cols; region.y=IntegralChannels.rows-region.height-1;
    GetChnFtrsOverWindow(IntegralChannels ,features,WindowFtrs ,Params,region);
    
    }

  
  

    for (Rows=0 ; Rows<IntegralChannels.rows-region.height-1; Rows+=STEP)
    {
      
    region.y=Rows; region.x=IntegralChannels.cols-region.width-1;
    GetChnFtrsOverWindow(IntegralChannels ,features,WindowFtrs,Params,region);
    
    
    }
    
    region.y=IntegralChannels.rows-region.height-1; region.x=IntegralChannels.cols-region.width-1;

    GetChnFtrsOverWindow(IntegralChannels ,features,WindowFtrs,Params,region);

    
  
}

void GetIntegralSum(Mat IntegralChannels, vector<float> & features,FtrParams2 Params, CvRect region)
{
  
  vec10d tl, tr, bl, br , sum;
  int sR=Params.y+region.y, sC=Params.x+region.x, eR=sR+Params.height, eC=sC+Params.width;
  
  tl= IntegralChannels.at<vec10d>(sR,sC);
  tr= IntegralChannels.at<vec10d>(sR,eC);
  bl= IntegralChannels.at<vec10d>(eR,sC);
  br= IntegralChannels.at<vec10d>(eR,eC);
  
  
  sum=br-bl-tr+tl;
  
  for (int n=0;n<CHANNELNR;n++)
  features.push_back(sum[n]);  
  
}



void GetChnFtrsOverWindow(Mat IntegralChannels , vector<float> & features ,FtrVecParams2 Params, CvRect region , PedRoiVec & PedRect,CvBoost & boost_classifier)
{
  

  Mat Test;
  Test=Mat::zeros(1,NRFEATURE,CV_32FC1);
  PedRoi ROI;
//   vector<float> aux;
  
  
  for (int n=0; n< Params[0].nFtrs ; n++)
  {
   
  GetIntegralSum(IntegralChannels,features ,Params[n],region);
  

  }
  
    
    
  for(int n=0; n<NRFEATURE; n++)
  {
   Test.at<float>(0,n)=features[features.size()-NRFEATURE+n];
//    aux.push_back(features[features.size()-NRFEATURE+n]);
   
  }
//    CvMat TestMat = Test;
//    CvMat * weak_responses = cvCreateMat(1,Test.cols,CV_32F);

  float x = boost_classifier.predict(Test,Mat(),Range::all(),false,true);
//    float x = boost_classifier.predict(&TestMat,0,weak_responses,CV_WHOLE_SEQ,0,1);
   
//    Mat WR(weak_responses,false);
   
//    cout<<x<<endl;
   
  if (x<=THRESHOLD) 
  {
    ROI.x=region.x;
    ROI.y=region.y;
    ROI.Scale=IntegralChannels.size();
    PedRect.push_back(ROI);
 
    // UNCOMMENT BELLOW TO ENABLE BOOTSTRAPPING --> BE SURE TO CHANGE THE FILE WHERE YOU WANT DATA TO BE SAVED //
    
//     fstream outfile;
//     
//     outfile.open ("/home/pedrobatista/workingcopy/lar3/perception/pedestrians/PedestrianDetect/train_15000_boot3.csv", fstream::in | fstream::out | fstream::app);
//     
//     outfile<<"2,";
//     
//     for (int i=0;i<NRFEATURE;i++)
//     {
//       if (i != NRFEATURE-1)
//       {
//       outfile<<Test.at<float>(0,i)<<",";
//       }
//       else
//       {
// 	outfile<<Test.at<float>(0,i);
//       }
//     }
//     
//     outfile<<endl;
    }
    
  
}

void GetChnFtrsOverWindow(Mat IntegralChannels , vector<float> & features,vector<DVector> & WindowFtrs ,FtrVecParams2 Params, CvRect region)
{
  vector<float> aux;
 
  for (int n=0; n< Params[0].nFtrs ; n++)
  {
   
  GetIntegralSum(IntegralChannels,features ,Params[n],region);
    
  }
  
  for(int n=0; n<NRFEATURE; n++)
  {
   aux.push_back(features[features.size()-NRFEATURE+n]);
  }
  
  WindowFtrs.push_back(aux);
  

  
  
}

void PostProcess(Mat Img, vector<Rect> PedRect, FtrVecParams2 randparams, CvBoost & boost_classifier, PedRoiVec & PedRect_Post)
{
  // MAKE POST PROCESSING 
  
  
  
}

