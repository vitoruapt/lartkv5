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

#include <iostream>
#include <string>

#include <ros/ros.h>

#include <datamatrix_detection/DatamatrixMsg.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <dmtx.h>

class ImageConverter
{
  ros::NodeHandle nh_;
  ros::Publisher datamatrix_pub;
  
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  
  std::vector<double> elapsed_time;
  
//   std::vector<std::string> leituras;
  
public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscribe to input video feed and publish output video feed
    image_sub_ = it_.subscribe("usb_cam_reader/image_rect_color", 1,
      &ImageConverter::imageCallback, this);
    datamatrix_pub = nh_.advertise<datamatrix_detection::DatamatrixMsg>("datamatrix_detection/datamatrix_msg",1);
  }

  ~ImageConverter()
  {
      
      double sum_time=0, max_time = 0.0, min_time = 1000;
        
        for( int i = 0 ; i<elapsed_time.size();i++)
        {
            sum_time = sum_time + elapsed_time[i];
            if (elapsed_time[i] < min_time)
                min_time = elapsed_time[i];
            if (elapsed_time[i] > max_time)
                max_time = elapsed_time[i];
        }
        
        std::cout << "Detection: " << std::endl;
        std::cout << "max time: " << max_time*1000 << " min time: " << min_time*1000 << " mean time: " << sum_time*1000/elapsed_time.size() << std::endl;
      
//     uint total = leituras.size();
//     std::cout << std::endl;
//     std::cout << "Foram lidas: " << total << " DataMatrix" << std::endl;
//     int error = 0;
//     uint j;
//     for ( j=0; j < leituras.size(); j++)
//     {
//       if (leituras[j] != "111222") error++;
//       
//     }
//     
//     std::cout << "Das quais: " << error << " foram lidas erradamente" << std::endl;
//     std::cout << "Success rate: " << ((total - error)/static_cast<double>(total))*100. << " %" << std::endl;
//     std::cout << std::endl;
  }

  void imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
//     ros::Time start = ros::Time::now();
    DatamatrixDecode(cv_ptr->image,cv_ptr->header);
//     elapsed_time.push_back( fabs((ros::Time::now() - start).toSec()) );

    // Update GUI Window
//     cv::imshow("Image", cv_ptr->image);
//     cv::waitKey(3);
    
  }
  
  void DatamatrixDecode(cv::Mat& cv_ori_img, std_msgs::Header image_header)
  {
    ros::Time start = ros::Time::now();
    
//     static int i = 1;
    DmtxImage      *img;
    DmtxDecode     *dec;
    DmtxRegion     *reg;
    DmtxMessage    *msg;
//     int height = cv_img.rows-1;
    
//     Rotate image
    
    cv::Mat rot_img;
    cv::Mat rot_mat = getRotationMatrix2D(cv::Point(cv_ori_img.cols/2, cv_ori_img.rows/2), 180, 1);
    cv::warpAffine(cv_ori_img, rot_img, rot_mat, cv_ori_img.size());
    
    cv::Mat cv_img = rot_img.rowRange(rot_img.rows/4, 3 * rot_img.rows/4);
    int height = cv_img.rows-1 + rot_img.rows/4;
    
//     cv::Mat cv_img = cv_ori_img.rowRange(cv_ori_img.rows/4, 3 * cv_ori_img.rows/4);
//     int height = cv_img.rows-1 + cv_ori_img.rows/4;
    
//     ImageFilter
//     cv::Mat imgHSV;
//     cv::cvtColor(cv_img, imgHSV, cv::COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
//     
//     cv::Mat imgThresholded;
//     cv::inRange(imgHSV, cv::Scalar(0, 0, 0), cv::Scalar(179, 255, 100), imgThresholded); //Threshold the image
//     
//     cv_img = imgThresholded.clone();
    
    
    datamatrix_detection::DatamatrixMsg datamatrix_msg;
    datamatrix_detection::DatamatrixData datamatrix_data;
    datamatrix_msg.header = image_header;
    
    img = dmtxImageCreate(cv_img.data, cv_img.cols, cv_img.rows, DmtxPack24bppRGB);
    assert(img != NULL);
    
    dec = dmtxDecodeCreate(img, 1);
    assert(dec != NULL);
    
    //    For multiple matrix reading =========================================================
    int delay = 80;     // Delay in miliseconds
    DmtxTime timeout = dmtxTimeAdd(dmtxTimeNow(),delay);   //    adds 50 miliseconds to the timeout time [now + 50ms]
    reg = dmtxRegionFindNext(dec, &timeout);
    while(reg != NULL) {
      msg = dmtxDecodeMatrixRegion(dec, reg, DmtxUndefined);
      if(msg != NULL) {
          datamatrix_data.code = std::string((char*)(msg->output));
          
          datamatrix_data.left.x = reg->leftLoc.X;
          datamatrix_data.left.y = height - reg->leftLoc.Y;
          datamatrix_data.left.z = 0;
          datamatrix_data.right.x = reg->rightLoc.X;
          datamatrix_data.right.y = height - reg->rightLoc.Y;
          datamatrix_data.right.z = 0;
          datamatrix_data.top.x = reg->topLoc.X;
          datamatrix_data.top.y = height - reg->topLoc.Y;
          datamatrix_data.top.z = 0;
          datamatrix_data.bottom.x = reg->bottomLoc.X;
          datamatrix_data.bottom.y = height - reg->bottomLoc.Y;
          datamatrix_data.bottom.z = 0;
          
          datamatrix_msg.decoded_matrices.push_back(datamatrix_data);
          
          dmtxMessageDestroy(&msg);
      }
      
      timeout = dmtxTimeAdd(dmtxTimeNow(),delay); //  It searches for matrixes in the image for delay miliseconds each one
      reg = dmtxRegionFindNext(dec, &timeout);

    }
    
//       datamatrix_detection::DatamatrixMsg datamatrix_msg Publish
    datamatrix_pub.publish(datamatrix_msg);
    
    dmtxDecodeDestroy(&dec);
    dmtxImageDestroy(&img);
    dmtxRegionDestroy(&reg);
    
    if ( msg !=NULL)
        elapsed_time.push_back( fabs((ros::Time::now() - start).toSec()) );
//     cv::imshow("ROImage",cv_img);
  }
      
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "datamatrix_detection");
  ImageConverter ic;
  ros::spin();
  return 0;
}