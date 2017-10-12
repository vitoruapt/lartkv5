/**************************************************************************************************
 Software License Agreement (BSD License)

 Copyright (c) 2011-2014, LAR toolkit developers - University of Aveiro - http://lars.mec.ua.pt
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

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <camera_info_manager/camera_info_manager.h>


int main(int argc, char** argv)
{
  ros::init(argc, argv, "usb_cam_reader");
  ros::NodeHandle nh("~");
  image_transport::ImageTransport it(nh);
  image_transport::CameraPublisher img_pub = it.advertiseCamera("image_raw", 1);
  
//   Define the camera info url names
  std::string camera_info_url;
  std::string camera_name;
  
  nh.param("camera_name", camera_name, std::string("my_camera"));
  nh.param("camera_info_url", camera_info_url, std::string(""));
  
  int cam_id;
  nh.param<int>("cam_id",cam_id,0);
  int img_width;
  nh.param<int>("width",img_width,0);
  int img_height;
  nh.param<int>("height",img_height,0);
  
  camera_info_manager::CameraInfoManager cinfo_manager_(nh, camera_name, camera_info_url);
  
//   Creating the camara info manager
//   camera_info_manager::CameraInfoManagerPtr cinfo_ = camera_info_manager::CameraInfoManagerPtr(new camera_info_manager::CameraInfoManager(nh, camera_name_, camera_info_url_));
  
//   Capture video stream
  cv::VideoCapture captura(cam_id);
//   Set the resolution to 720p, otherwise deafult is [640 480]
  
  if ( !captura.isOpened())
  {
    std::cout << "Impossible to open device" << std::endl;
    return -1;
  }
  bool resolution = false;
  sensor_msgs::CameraInfoPtr cam_info(new sensor_msgs::CameraInfo(cinfo_manager_.getCameraInfo()));
//   std::cout << "width: " << cam_info->width << std::endl;
  
//   Checks if we are calibrating the camera for a given resolution
  if ( img_width != 0 )
  {
      captura.set(CV_CAP_PROP_FRAME_WIDTH, img_width);
      captura.set(CV_CAP_PROP_FRAME_HEIGHT, img_height);
  }else{
//       If not, checks if there is already a calibration resolution and gets it
      if ( cam_info->width != 0) 
      {
          captura.set(CV_CAP_PROP_FRAME_WIDTH, cam_info->width);
          captura.set(CV_CAP_PROP_FRAME_HEIGHT, cam_info->height);
      }
  }
      
      
//   Rate = 60 -> double the camera frame rate
  ros::Rate loop_rate(60);
  while (ros::ok()) 
  {
    
    cv::Mat frame;
    captura.read(frame);
    if ( !resolution )
        std::cout << frame.size() << std::endl;
    resolution = true;
    
    if(img_pub.getNumSubscribers() >  0)
    {
      cv_bridge::CvImage my_image;
      
      my_image.header.stamp = ros::Time::now();
      my_image.header.frame_id = "my_image";
      my_image.encoding = "bgr8";
      my_image.image = frame;
      
      
      cam_info->header.frame_id = my_image.header.frame_id;
      cam_info->header.stamp = my_image.header.stamp;
      
//       sensor_msgs::Image send_image;
//       my_image.toImageMsg(send_image);
      img_pub.publish(  my_image.toImageMsg(), cam_info);
      
    }
    
//     sensor_msgs::ImagePtr msg = sensor_msgs::CvBridge::cvToImgMsg(image.Ipl(), "bgr8");
        
    ros::spinOnce();
    loop_rate.sleep();
  }
}