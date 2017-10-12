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
  cvml.read_csv ("/home/pedrobatista/workingcopy/lar3/perception/pedestrians/PedestrianDetect/train_seed1234_rand.csv");
//3. Indicate which column is the response
  cvml.set_response_idx (0);
    
/* STEP 3. Splitting the samples */
//1. Select 40 for the training
  CvTrainTestSplit cvtts (8000, true);
//2. Assign the division to the data
  cvml.set_train_test_split (&cvtts);

  printf ("Training ... ");
/* STEP 4. The training */
//1. Declare the classifier
  CvBoost boost;
//2. Train it with 100 features
  boost.train (&cvml, CvBoostParams (CvBoost::REAL,1500, 0.95, 2, false, 0),
	       false);
		
/* STEP 5. Calculating the testing and training error */
// 1. Declare a couple of vectors to save the predictions of each sample
  std::vector<float> train_responses, test_responses;
// 2. Calculate the training error
  float fl1 = boost.calc_error (&cvml, CV_TRAIN_ERROR, &train_responses);
// 3. Calculate the test error
  float fl2 = boost.calc_error (&cvml, CV_TEST_ERROR, &test_responses);
  
  cout<<"Error train: "<<fl1<<endl;
  
  cout<<"Error test: "<<fl2<<endl;

/* STEP 6. Save your classifier */
// Save the trained classifier
//   boost.save ("./trained_boost_8000samples-1000ftrs.xml", "boost");

  return 0;
}
