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
  ros::init (argc, argv, "image_publisher");
  PVector vect;

  ros::NodeHandle nh;
  image_transport::ImageTransport it (nh);
  image_transport::Publisher pub = it.advertise ("Image_In", 1);

  cv_bridge::CvImagePtr cv_ptr (new cv_bridge::CvImage);

//   string folder_path = ros::package::getPath("PedDetect");
  
  /////////////////INRIA///////////////////
  
//   string folder_path= "/home/pedrobatista/Desktop/INRIAPerson/train_64x128_H96/pos"; // pos train windows
//   string folder_path= "/home/pedrobatista/Desktop/INRIAPerson/train_64x128_H96/neg"; // -< neg big train images
//   
//   string folder_path= "/home/pedrobatista/Desktop/INRIAPerson/test_64x128_H96/neg"; // < neg big test images 
//   string folder_path= "/home/pedrobatista/Desktop/INRIAPerson/test_64x128_H96/pos"; // -< post test windows

  /////////////////INRIA///////////////////
  
  /////////////////Atlas///////////////////
  string folder_path="/home/pedrobatista/Desktop/AtlasDataset/BootData";
  
  
  vect.clear();
  
  
  GetFileList(folder_path, vect);
  
  sensor_msgs::ImagePtr msg;
  
    
  ros::Rate loop_rate (0.04);

  int nfiles = vect.size(); int n=0;

//   cout<<"nfiles: "<<nfiles<<endl;
  
  sleep(1);
 
  while (nh.ok () && n<nfiles )
    {
      string ImgPath=vect[n].string();
      
      if (strstr(vect[n].string().c_str(),".jpg") || strstr(vect[n].string().c_str(),".png"))
      {
	
	cv_ptr->image = cv::imread(ImgPath, CV_LOAD_IMAGE_COLOR);
	
	msg = cv_ptr->toImageMsg ();
	msg->header.frame_id = "Image_in";
	msg->encoding = "rgb8";
	msg->header.stamp = ros::Time::now ();
	
	pub.publish (msg);
	n++;
	cout<<"nfiles: "<<n<<endl; 
      }
      
      ros::spinOnce ();
      
      loop_rate.sleep ();
    }
  
  
  
}
  
