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

int main (int argc, char **argv)
{

/* STEP 2. Opening the file */
//1. Declare a structure to keep the data
  CvMLData cvml;
//2. Read the file
  cvml.read_csv ("/home/pedrobatista/workingcopy/lar3/perception/pedestrians/PedestrianDetect/train_15000_boot3.csv");
//3. Indicate which column is the response
  
  cvml.set_response_idx (0);
//   CvTrainTestSplit cvtts (8000, true);
//   cvml.set_train_test_split (&cvtts);
  
  const CvMat* Resp = cvml.get_responses();
  const CvMat* Values = cvml.get_values();
  
  Mat RespM(Resp, false);
  Mat ValM(Values,false);
  
//   RespM.assignTo(RespM, CV_32F );
//   ValM.assignTo(ValM, CV_32F );

  
  
  Mat trainData = ValM.colRange(1, 15001);
  cout<<trainData.size()<<endl;
//   cout<<trainData.at<float>(0,0)<<endl;
  
//   Mat varIdx(1,NRFEATURE,CV_32F);
//   
//   for (int n=0; n<varIdx.cols;n++)
//     varIdx.at<float>(1,n)=n;
//   
//   Mat sampleIdx(1,trainData.rows,CV_32F);
//   
//   for (int i=0; i<sampleIdx.cols;i++)
//     sampleIdx.at<float>(1,i)=i;
//   
//   Mat varType(1,trainData.rows,CV_32F);
//     
// 
//   for (int y=0; y<sampleIdx.cols;y++)
//       varType.at<float>(1,y)=CV_VAR_CATEGORICAL;
    
    
  
  
   CvBoost boost;
  
    boost.train(	trainData, 
			  CV_ROW_SAMPLE, 
			  RespM, 
			  Mat(), 
			  Mat(), 
			  Mat(), 
			  Mat(), 
			  CvBoostParams(CvBoost::REAL,2000, 0.95, 2, false, 0), 
			  false);
    
    
//     float elements[10];
//     CvSeq * weak = boost.get_weak_predictors();
//     
//     CvtSeqToArray(weak,elements,CV_WHOLE_SEQ);
    
    
    
    //     cout<<weak->total<<endl;
    
// 	CvPoint pt0;
	
//     for(int i = 0; i < weak->total; i++)
//     {
//     pt0 = *CV_GET_SEQ_ELEM( CvPoint, weak, i );
//     cout<<pt0.x<<endl;
//     }
    
    
    
    boost.save ("./trained_boost_10Kf_2000w_13Ks_m8_M64_boot1.xml", "boost");
  
  
  return 0;
  
}
