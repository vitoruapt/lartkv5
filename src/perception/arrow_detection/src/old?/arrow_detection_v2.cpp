
#include <ros/ros.h>

#include <visp/vpImage.h>
#include <visp/vpDisplayX.h>
#include <visp_ros/vpROSGrabber.h>

using namespace std;

int main(int argc,char**argv) 
{
    vpImage<unsigned char> I; 
    
    vpROSGrabber g; 
    g.setImageTopic("/camera/image_raw");
    g.open(I); 
    
    vpDisplayX d(I);
    
    ros::spinOnce();//necessary for catking include ros libraries  
    
    while(ros::ok())
    {
        g.acquire(I);
        vpDisplay::display(I);
        vpDisplay::flush(I);
        
        if (vpDisplay::getClick(I, false))
            break;
        
    }
    
    return 0;
}


//#include <visp/vpDisplayX.h>
//#include <visp/vpImage.h>
// #include <ros/ros.h>
// 
// #include <visp_ros/vpROSGrabber.h>
// #include <iostream>
// using namespace std;
// 
// int main(int argc,char**argv)
// {
//     ros::init( argc, argv, "arrow_detection_v2" );
//     
//     cout<<"i'm alive"<<endl;
//     vpROSGrabber g;
//     
//     return 0;
    
  //try {
    ////vpImage<unsigned char> I; // Create a gray level image container
    //vpImage<vpRGBa> I; // Create a color image container
//     vpROSGrabber g; // Create a grabber based on ROS

    //g.setCameraInfoTopic("/camera/camera_info");
    //g.setImageTopic("/camera/image_raw");
    //g.setRectify(true);
    //g.open(I);
    //std::cout << "Image size: " << I.getWidth() << " " << I.getHeight() << std::endl;

//#ifdef VISP_HAVE_X11
    ////vpDisplayX d(I);
//#else
    //std::cout << "No image viewer is available..." << std::endl;
//#endif

    //while(1) {
      //g.acquire(I);
  ////    vpDisplay::display(I);
    ////  vpDisplay::flush(I);
      ////if (vpDisplay::getClick(I, false))
       //// break;
    //}
  //}
  //catch(vpException e) {
    //std::cout << "Catch an exception: " << e << std::endl;
  //}
// }

//#include <visp/vpImage.h> 
////#include <visp/vpROSGrabber.h>
//#include <visp_ros/vpROSGrabber.h>
//#include <visp_ros/vpROSRobot.h>

////#include <visp/vpPixelMeterConversion.h>
////#include <visp/vpDisplayGDI.h>
////#include <visp/vpDisplayX.h>
////#include <visp/vpConfig.h>
////#include <visp/vpRGBa.h>


//int main() 
//{
  //vpImage<unsigned char> I; 
  ////vpROSRobot robot; 
  ////robot.setCmdVelTopic("/myrobot/cmd_vel");
  ////robot.init();

  //vpROSGrabber g; 
  //g.setImageTopic("/camera/image_raw");
  //g.open(I); 

  //while(1) {
    //g.acquire(I); 
    ////vpDisplay::display(I);
    ////vpDisplay::flush(I);
    //// Visual servoing code 
    ////robot.setVelocity(vpRobot::CAMERA_FRAME, v);
  //}
//}

///**************************************************************************************************
 //Software License Agreement (BSD License)

 //Copyright (c) 2011-2014, LAR toolkit developers - University of Aveiro - http://lars.mec.ua.pt
 //All rights reserved.

 //Redistribution and use in source and binary forms, with or without modification, are permitted
 //provided that the following conditions are met:

  //*Redistributions of source code must retain the above copyright notice, this list of
   //conditions and the following disclaimer.
  //*Redistributions in binary form must reproduce the above copyright notice, this list of
   //conditions and the following disclaimer in the documentation and/or other materials provided
   //with the distribution.
  //*Neither the name of the University of Aveiro nor the names of its contributors may be used to
   //endorse or promote products derived from this software without specific prior written permission.
 
 //THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 //IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 //FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 //CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 //DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 //DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 //IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 //OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//***************************************************************************************************/

//#include <iostream>
//#include <string>

//#include <ros/ros.h>

//#include <datamatrix_detection/DatamatrixMsg.h>

//#include <image_transport/image_transport.h>
//#include <cv_bridge/cv_bridge.h>
//#include <sensor_msgs/image_encodings.h>

//#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/highgui/highgui.hpp>

//#include <dmtx.h>


////visp includes
//#include <visp/vpImage.h> 
//#include <visp/vpROSGrabber.h>
//#include <visp/vpROSRobot.h>

//#include <visp/vpDisplayX.h>
//#include <visp/vpMbEdgeKltTracker.h>
//#include <visp/vpMbKltTracker.h>
//#include <visp/vpMbEdgeTracker.h>
//#include <visp/vpTime.h>

//#include <visp/vpCameraParameters.h>
//#include <sensor_msgs/CameraInfo.h>

//#include <visp_bridge/camera.h>
//#include <visp_bridge/image.h>
//#include <visp_bridge/3dpose.h>

////#include "libauto_tracker/tracking.h"

//#include "resource_retriever/retriever.h"

//#include "std_msgs/Int8.h"


//class ImageConverter
//{
  //ros::NodeHandle nh_;
////   ros::Publisher datamatrix_pub;
  
  //image_transport::ImageTransport it_;
  //image_transport::Subscriber image_sub_;
  
////   std::vector<std::string> leituras;
  
//public:
  //ImageConverter()
    //: it_(nh_)
  //{
    //// Subscribe to input video feed and publish output video feed
    //image_sub_ = it_.subscribe("/camera/image_raw", 1,
      //&ImageConverter::imageCallback, this);
////     datamatrix_pub = nh_.advertise<datamatrix_detection::DatamatrixMsg>("datamatrix_detection/datamatrix_msg",1);
  //}

  //~ImageConverter()
  //{

  //}
  //void imageCallback(const sensor_msgs::ImageConstPtr& msg)
  //{
    //vpImage<unsigned char> I; //cesar
    //I = visp_bridge::toVispImage (*msg);//cesar
    
    //cv_bridge::CvImagePtr cv_ptr;
    //try
    //{
      //cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    //}
    //catch (cv_bridge::Exception& e)
    //{
      //ROS_ERROR("cv_bridge exception: %s", e.what());
      //return;
    //}
    //*
    //DatamatrixDecode(cv_ptr->image,cv_ptr->header);*/
    

    //// Update GUI Window
    //cv::imshow("Image", cv_ptr->image);
    //cv::waitKey(3);
    
  //}
  
      
//};

//int main(int argc, char** argv)
//{
  //ros::init(argc, argv, "datamatrix_detection");
  //ImageConverter ic;
  //ros::spin();
  //return 0;
//}
