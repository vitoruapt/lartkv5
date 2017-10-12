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

  cv_bridge::CvImagePtr cv_ptr (new cv_bridge::CvImage);

  string folder_path = ros::package::getPath("PedestrianDetect");
  
  vect.clear();
  
  
//   GetFileList(folder_path, vect);
  
//   cout<<"Numero de ficheiros"<<vect.size()<<endl;
//   
//   for(uint n=0; n<vect.size();n++)
//     
//   {
//     cout<<"directorio: "<<vect[n].string()<<endl;
//   }
  
  
//   string ImgPath=folder_path+"/640.png";
  string ImgPath="/home/pedrobatista/workingcopy/lar3/perception/pedestrians/PedestrianDetect/human3.png";
  
//   cout<<"path: "<<path<<endl;
  
  cv_ptr->image = cv::imread(ImgPath, CV_LOAD_IMAGE_COLOR);
  
  image_transport::Publisher pub = it.advertise ("Image_In", 1);

  sensor_msgs::ImagePtr msg = cv_ptr->toImageMsg ();

  msg->encoding = "rgb8";
  msg->header.frame_id = "Image_in";


//   ROS_INFO("msg = %s", msg->encoding.c_str() );

  ros::Rate loop_rate (1);
  
  while (nh.ok ())
    {
      msg->header.stamp = ros::Time::now ();
      pub.publish (msg); 
      ros::spinOnce ();
      loop_rate.sleep ();
    }


}
