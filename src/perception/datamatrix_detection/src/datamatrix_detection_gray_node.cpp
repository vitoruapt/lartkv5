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

#include <dmtx.h>

// static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
//   image_transport::Publisher image_pub_;
    std::vector<std::string> leituras;
  
public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/usb_cam/image_raw", 1, 
      &ImageConverter::imageCallback, this);
//     image_pub_ = it_.advertise("/image_converter/output_video", 1);

  }

  ~ImageConverter()
  {
//     cv::destroyWindow(OPENCV_WINDOW);
    uint total = leituras.size();
    std::cout << std::endl;
    std::cout << std::endl;
    std::cout << "Foram lidas: " << total << " DataMatrix" << std::endl;
    int error = 0;
    uint j;
    for ( j=0; j < leituras.size(); j++)
    {
      if (leituras[j] != "111222") error++;
//       std::cout << "leitura " << leituras[j] << std::endl;
    }
    
    std::cout << "Das quais: " << error << " foram lidas erradamente" << std::endl;
    std::cout << "Success rate: " << ((total - error)/static_cast<double>(total))*100. << " %" << std::endl;
    std::cout << std::endl;
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
    DatamatrixDecode(cv_ptr->image);
    
    // Draw an example circle on the video stream
//     if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
//       cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

    // Update GUI Window
    cv::imshow("Imagem", cv_ptr->image);
    cv::waitKey(3);
    
    // Output modified video stream
//     image_pub_.publish(cv_ptr->toImageMsg());
  }
  void DatamatrixDecode(cv::Mat& cv_img)
  {
    static int i = 1;
    DmtxImage      *img;
    DmtxDecode     *dec;
    DmtxRegion     *reg;
    DmtxMessage    *msg;
    
    cv::Mat gray_img;
    cvtColor(cv_img, gray_img, CV_BGR2GRAY, 0);
    
        
//     É mais rápido ler em grayscale com DmtxPack8bppK
//     Porém é pior na leitura à distância
    img = dmtxImageCreate(gray_img.data, gray_img.cols, gray_img.rows, DmtxPack8bppK);
    assert(img != NULL);

    dec = dmtxDecodeCreate(img, 1);
    assert(dec != NULL);
    
    //    For single matrix reading =========================================================
    DmtxTime timeout = dmtxTimeAdd(dmtxTimeNow(),50);
    reg = dmtxRegionFindNext(dec, &timeout);
    if(reg != NULL) {
      msg = dmtxDecodeMatrixRegion(dec, reg, DmtxUndefined);
      if(msg != NULL) {
	
// 	fputs("output: \"", stdout);
	
//         fwrite(msg->output, sizeof(unsigned char), msg->outputIdx, stdout);
	std::cout << "output " << i++ << ": " << msg->output << std::endl;
	leituras.push_back(std::string((char*)(msg->output)));
// 	cout << "Código completo: " << msg->codeSize << endl;
//         fputs("\"\n", stdout);
        dmtxMessageDestroy(&msg);
      }
      
      int height = cv_img.rows-1;
      cv::circle(cv_img, cv::Point (reg->leftLoc.X, height - reg->leftLoc.Y), 10, CV_RGB(255,0,0));
      cv::circle(cv_img, cv::Point (reg->rightLoc.X, height - reg->rightLoc.Y), 10, CV_RGB(255,0,0));
      cv::circle(cv_img, cv::Point (reg->topLoc.X, height - reg->topLoc.Y), 10, CV_RGB(255,0,0));
      cv::circle(cv_img, cv::Point (reg->bottomLoc.X, height - reg->bottomLoc.Y), 10, CV_RGB(255,0,0));
      
      dmtxRegionDestroy(&reg);
    }
    
//     cv::imshow("Imagem", gray_img);
//     cv::waitKey(3);
    
    dmtxDecodeDestroy(&dec);
    dmtxImageDestroy(&img);
  }
  
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "read_gray_datamatrix");
  ImageConverter ic;
  ros::spin();
  return 0;
}