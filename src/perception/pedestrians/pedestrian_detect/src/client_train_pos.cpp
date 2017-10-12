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
  int ImgCount;
  cv_bridge::CvImagePtr cv_ptr;
  DVector features;
  CvRect region; 
  Size minSize;
  FtrVecParams2 randparams;
  vector<DVector> WindowFtrs;
  
public:

ImageConverter ():
  it_ (nh_)
  {
    
    ImgCount = 1;
    image_pub_ = it_.advertise ("Image_Out", 1);
    image_sub_ = it_.subscribe ("Image_In", 1, &ImageConverter::imageCb,this);
    region.x = 0; region.y = 0; region.width = 64; region.height = 128;
    minSize.width=region.width; minSize.height=region.height;
    GetRandParams(SEED, NRFEATURE2, randparams, region);
    
//     for (uint n = 0; n<randparams.size();n++) 
//     {
//       cout<<randparams[n].width<<" x "<<randparams[n].height<<" x: "<<randparams[n].x<<" y: "<<randparams[n].y<<endl;
//     }
    
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
    
        
	
      Size win; win.width=64; win.height=128;
      resize(Img, Img, win , 0, 0, INTER_LINEAR);

    ///////USE OPENCV TO PROCESS IMAGE////////
      

    cvtColor (Img, Img, CV_BGR2RGB, 0);

    features.clear ();
    WindowFtrs.clear();
    

    GetChnFtrsOverImagePyramid(Img , region , features, WindowFtrs ,0, minSize, 8 , randparams);

//     cout<<features.size()<<endl;
    
    fstream outfile;
    
    outfile.open ("/home/pedrobatista/workingcopy/lar3/perception/pedestrians/PedestrianDetect/train_10000.csv", fstream::in | fstream::out | fstream::app);
    
    outfile<<"1,";
    
    for (int i=0;i<NRFEATURE;i++)
    {
      if (i != NRFEATURE-1)
      outfile<<features[i]<<",";
      else
	outfile<<features[i];
    }
    
    outfile<<endl;
    
    outfile.close();
    
    ROS_INFO ("Image nr %d\n", ImgCount++);

    /////////////////////////////////////////


    cv_ptr->image = Img;



    sensor_msgs::ImagePtr msg_out = cv_ptr->toImageMsg ();
    msg_out->encoding = "rgb8";
    msg_out->header.frame_id = "Image_Out";
    msg_out->header.stamp = ros::Time::now ();


    image_pub_.publish (msg_out);



  }

};

int main (int argc, char **argv)
{
  fstream outfile("/home/pedrobatista/workingcopy/lar3/perception/pedestrians/PedestrianDetect/train_10000.csv");
  ros::init (argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin ();
  return 0;
}
