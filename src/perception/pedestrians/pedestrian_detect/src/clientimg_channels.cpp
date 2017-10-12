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

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

  Mat xsobel, ysobel, GrayImg, xsobeldiv, div, integralim, GradMag,
    IntegralChns;

  MatVector BinVect, LUVchannels;

  d10Vector features;

  int ImgCount;
  cv_bridge::CvImagePtr cv_ptr;

public:

ImageConverter ():
  it_ (nh_)
  {

    ImgCount = 0;
    image_pub_ = it_.advertise ("Image_Out", 1);
    image_sub_ =
      it_.subscribe ("Image_In", 1, &ImageConverter::imageCb, this);

  }

  ~ImageConverter ()
  {

  }

  void imageCb (const sensor_msgs::ImageConstPtr & msg)
  {




    try
    {

      cv_ptr = cv_bridge::toCvCopy (msg, "rgb8");

      cv_ptr->image.convertTo (cv_ptr->image, CV_8UC1);

    }
    catch (cv_bridge::Exception & e)
    {
      ROS_ERROR ("cv_bridge exception: %s", e.what ());
      return;
    }
    Mat Img = cv_ptr->image;

    if (Img.rows == 0)
      return;
    
    //     ROS_INFO ("Image nr %d\n", ImgCount++);

    Mat MergedChannels (Img.rows, Img.cols, CV_64FC (CHANNELNR));

    ///////USE OPENCV TO PROCESS IMAGE////////


    cvtColor (Img, Img, CV_BGR2RGB, 0);

    cvtColor (Img, GrayImg, CV_RGB2GRAY, 0);

    features.clear ();

    Sobel (GrayImg, xsobel, CV_32FC1, 1, 0, 3, 1, 0, BORDER_DEFAULT);
    Sobel (GrayImg, ysobel, CV_32FC1, 0, 1, 3, 1, 0, BORDER_DEFAULT);

    GradMag = GradientMagnitude (xsobel, ysobel);

    BinVect = OrientedGradientsDiagram (GradMag, xsobel, ysobel);

    LUVchannels = LUVcolourchannels (Img);



    cvtColor (Img, GrayImg, CV_RGB2GRAY, 0);
    /////////////////////////////////////////


    cv_ptr->image = BinVect[5];


    sensor_msgs::ImagePtr msg_out = cv_ptr->toImageMsg ();
    msg_out->encoding = "32FC1";
    msg_out->header.frame_id = "Image_Out";
    msg_out->header.stamp = ros::Time::now ();


    image_pub_.publish (msg_out);


  }

};

int main (int argc, char **argv)
{
  ros::init (argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin ();
  return 0;
}
