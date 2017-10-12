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
#define _MAIN_C_
#include "peddetect.h" 
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <opencv/cvwimage.h>
#include <opencv/highgui.h>
  

class PedestrianDetect
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
  long long int Pedcount, nPedcount;
  PedRoiVec PedRect;
  PedRoiVec PedRect_Post;
  CvBoost boost_classifier;
  vector<Rect> rectangles;
  
  
public:

  PedestrianDetect ():
  nh_("~"),
  it_ (nh_),
  Pedcount(0),
  nPedcount(0)
  {

    ImgCount = 0;
    image_pub_ = it_.advertise ("Image_Out", 1);
//     cout<<"subscribede to :"<<ros::names::remap("image_in")<<endl;
    image_sub_ = it_.subscribe (ros::names::remap("image_in"), 1, &PedestrianDetect::imageCb,this);
//     image_sub_ = it_.subscribe ("Image_In", 1, &PedestrianDetect::imageCb,this);
//     image_sub_ = it_.subscribe ("image_input", 1, &PedestrianDetect::imageCb, this);
//     it_.subscribe ("Image_In", 1, &PedestrianDetect::imageCb, this);
    
    string classifier_file;
    nh_.param<string>("classifier",classifier_file,string("not_found"));
    
    region.x = 0; region.y = 0; region.width = DWWIDTH; region.height = DWHEIGHT;
    minSize.width=DWWIDTH; minSize.height=DWHEIGHT;
    
    GetRandParams(SEED,NRFEATURE2, randparams, region);
    boost_classifier.load(classifier_file.c_str());
//     boost_classifier.load("/home/pedrobatista/workingcopy/lar3/perception/pedestrians/PedestrianDetect/trained_boost_15Kf_2000w_19Ks_m8_M64_boot3.xml");

  }

  ~PedestrianDetect ()
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
    
//     imshow("/Image_Out",Img);
//     if( waitKey (30) >= 0)
    
    if (Img.rows == 0)
      return;
    
//       if (Img.rows<162)
//       {
//         Size win; win.width=DWWIDTH; win.height=DWHEIGHT;
//         resize(Img, Img, win , 0, 0, INTER_LINEAR);
//       }
//       else
//       {
//         Size win; win.width=640; win.height=480;
//         resize(Img, Img, win , 0, 0, INTER_LINEAR);
//       }
      
    double secs =ros::Time::now().toSec(); // inicia a contagem do tempo
    
    ///////USE OPENCV TO PROCESS IMAGE////////
    cvtColor (Img, Img, CV_BGR2RGB, 0);
    
    features.clear ();
    PedRect.clear();
    rectangles.clear();
    PedRect_Post.clear();
    GetChnFtrsOverImagePyramid(Img , region , features ,NOCTUP, minSize, SCALEPOCT , randparams,PedRect, boost_classifier);
    
    cout << "a imagem contem peoes?" << PedRect.size() << endl;
    if (PedRect.size()!=0)
    {
	cout << "sim" << endl;
    }
    else {cout << "nao" << endl;}
        
//     for (uint n=0; n<PedRect.size(); n++) // processa o quantos pedestres encontrou e arranja um rectangulo para cada um
//     {
//       
//       double FR = (double)Img.rows / (double)PedRect[n].Scale.height, FC=(double)Img.cols / (double)PedRect[n].Scale.width;
//       cv::Rect BB(Point((double)PedRect[n].x*FC,(double)PedRect[n].y*FR),Point((double)PedRect[n].x*FC+(double)DWWIDTH*FC,(double)PedRect[n].y*FR+(double)DWHEIGHT*FR));
//       
//       rectangles.push_back(BB);
// 
//     }
   
    
    
//     if (rectangles.size()!=0) groupRectangles(rectangles, 1, 0.2);
    
    
//     for (uint n=0; n<rectangles.size();n++)
//     {
//       rectangle(Img,rectangles[n],Scalar(0,255,0), 2, 8, 0);
// //       cout<<"width: "<<rectangles[n].width<<" height: "<<rectangles[n].height<<" x: "<<rectangles[n].x<<" y: "<<rectangles[n].y<<endl;
//       
//     }
// //  
// //     PostProcess(Img, rectangles, randparams, boost_classifier, PedRect_Post);
//    
//     Pedcount=PedRect.size();
//     
//     nPedcount=features.size()/NRFEATURE-PedRect.size();
// 
//     cout<<"Ped: "<<Pedcount<<" nPed: "<<nPedcount<<endl;
//     
//     ROS_INFO ("Image nr %d\n", ImgCount++);
// //     cout<<endl<<features.size()<<endl;
// //     cout<<endl<<Img.rows<<" x "<<Img.cols<<endl;
//     
//     //////////////////////////////////////////
//     
// //     cvtColor (Img, Img, CV_RGB2BGR, 0);
//     
//     cout << "iniciacao da escrita" << endl;
//     std::ostringstream oss;
//     oss<<ImgCount;
// //     imwrite( "/home/rui/workingcopies/tutorials/src/pedestrian_detection"+oss.str()+".png", Img );
// //     imwrite( "/home/rui/Documents/2013_PedroSilva/PedestrianDetect/Results/RuiExamples/Frame"+oss.str()+".png", Img );
//     
//     
// //     cout << "image gravada" << endl;
//     cv_ptr->image = Img;
// 
//     sensor_msgs::ImagePtr msg_out = cv_ptr->toImageMsg ();
//     msg_out->encoding = "rgb8";
//     msg_out->header.frame_id = "Image_Out";
//     msg_out->header.stamp = ros::Time::now ();
//     
// 
//     image_pub_.publish (msg_out);

//     cvtColor(Img, Img, CV_BGR2RGB);
    
    cout << "duracao: " << ros::Time::now().toSec() -secs << endl; // fornece o tempo total que o programa demora a correr uma imagem
    
//     imshow("processed image",Img);
//     waitKey(30);

  }

};

int main (int argc, char **argv)
{
//   fstream outfile("/home/pedrobatista/workingcopy/lar3/perception/pedestrians/PedestrianDetect/train_seed1234_MIN8_MAX64.csv");
    ros::init (argc, argv, "image_converter");
    PedestrianDetect ic;
        
    ros::spin ();
    return 0;
  
}













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
//  #define _MAIN_C_
//  #include "peddetect.h" 
// 
// class PedestrianDetect
// {
//   ros::NodeHandle nh_;
//   image_transport::ImageTransport it_;
//   image_transport::Subscriber image_sub_;
//   image_transport::Publisher image_pub_;
//   int ImgCount;
//   cv_bridge::CvImagePtr cv_ptr;
//   DVector features;
//   CvRect region; 
//   Size minSize;
//   FtrVecParams2 randparams;
//   long long int Pedcount, nPedcount;
//   PedRoiVec PedRect;
//   PedRoiVec PedRect_Post;
//   CvBoost boost_classifier;
//   vector<Rect> rectangles;
//   
//   
// public:
// 
//   PedestrianDetect ():
//   nh_("~"),
//   it_ (nh_)
//   {
// 
//     ImgCount = 0;
//     image_pub_ = it_.advertise ("Image_Out", 1);
//     image_sub_ = it_.subscribe ("image_input", 1, &PedestrianDetect::imageCb, this);
// //     it_.subscribe ("Image_In", 1, &PedestrianDetect::imageCb, this);
//     
//     string classifier_file;
//     nh_.param<string>("classifier",classifier_file,string("not_found"));
//     
//     region.x = 0; region.y = 0; region.width = DWWIDTH; region.height = DWHEIGHT;
//     minSize.width=DWWIDTH; minSize.height=DWHEIGHT;
//     
//     GetRandParams(SEED,NRFEATURE2, randparams, region);
//     boost_classifier.load(classifier_file.c_str());
// //     boost_classifier.load("/home/pedrobatista/workingcopy/lar3/perception/pedestrians/PedestrianDetect/trained_boost_15Kf_2000w_19Ks_m8_M64_boot3.xml");
// 
//     Pedcount=0;
//     nPedcount=0;
// 
//   }
// 
//   ~PedestrianDetect ()
//   {
//    
// 
// 
//   }
// 
//   void imageCb (const sensor_msgs::ImageConstPtr & msg)
//   {
//     
//     try
//     {
// 
//       cv_ptr = cv_bridge::toCvCopy (msg, "rgb8");
// 
//       cv_ptr->image.convertTo (cv_ptr->image, CV_8UC1);
// 
//     }
//     catch (cv_bridge::Exception & e)
//     {
//       ROS_ERROR ("cv_bridge exception: %s", e.what ());
//       return;
//     }
//     Mat Img = cv_ptr->image;
// 
//     if (Img.rows == 0)
//       return;
//     
//       if (Img.rows<162)
//       {
//       Size win; win.width=DWWIDTH; win.height=DWHEIGHT;
//       resize(Img, Img, win , 0, 0, INTER_LINEAR);
//       }
//       else
//       {
//     Size win; win.width=640; win.height=480;
//     resize(Img, Img, win , 0, 0, INTER_LINEAR);
//       }
//       
//     
//     
//     ///////USE OPENCV TO PROCESS IMAGE////////
//       
//     cvtColor (Img, Img, CV_BGR2RGB, 0);
// 
//     features.clear ();
//     PedRect.clear();
//     rectangles.clear();
//     PedRect_Post.clear();
// 
//    
//     GetChnFtrsOverImagePyramid(Img , region , features ,NOCTUP, minSize, SCALEPOCT , randparams,PedRect, boost_classifier);
//     
//     
//     for (uint n=0; n<PedRect.size(); n++)
//     {
//       
//       double FR = (double)Img.rows / (double)PedRect[n].Scale.height, FC=(double)Img.cols / (double)PedRect[n].Scale.width;
//       cv::Rect BB(Point((double)PedRect[n].x*FC,(double)PedRect[n].y*FR),Point((double)PedRect[n].x*FC+(double)DWWIDTH*FC,(double)PedRect[n].y*FR+(double)DWHEIGHT*FR));
//       
//       rectangles.push_back(BB);
// 
//     }
//    
//     
//     if (rectangles.size()!=0) groupRectangles(rectangles, 1, 0.2);
//     
//     for (uint n=0; n<rectangles.size();n++)
//     {
//       rectangle(Img,rectangles[n],Scalar(0,255,0), 2, 8, 0);
// //       cout<<"width: "<<rectangles[n].width<<" height: "<<rectangles[n].height<<" x: "<<rectangles[n].x<<" y: "<<rectangles[n].y<<endl;
//       
//     }
// //  
// //     PostProcess(Img, rectangles, randparams, boost_classifier, PedRect_Post);
//     
// // //     Pedcount+=rectangles.size();
// //     
// // //     cout<<"Ped: "<<Pedcount<<"   "<<PedRect.size()<<endl;
//     
//     Pedcount+=PedRect.size();
//     nPedcount+=features.size()/NRFEATURE-PedRect.size();
//     cout<<"Ped: "<<Pedcount<<" nPed: "<<nPedcount<<endl;
// 
//     ROS_INFO ("Image nr %d\n", ImgCount++);
// //     cout<<endl<<features.size()<<endl;
// //     cout<<endl<<Img.rows<<" x "<<Img.cols<<endl;
//     
//     //////////////////////////////////////////
//     
// //     cvtColor (Img, Img, CV_RGB2BGR, 0);
//     
//     string classifier_result_file;
//     nh_.param<string>("classifier_result",classifier_result_file,string("not_found"));
//     
//     std::ostringstream oss;
//     oss<<ImgCount;
//     imwrite(classifier_result_file.c_str()+oss.str()+".png", Img );
// 
//     cv_ptr->image = Img;
// 
// 
// 
//     sensor_msgs::ImagePtr msg_out = cv_ptr->toImageMsg ();
//     msg_out->encoding = "rgb8";
//     msg_out->header.frame_id = "Image_Out";
//     msg_out->header.stamp = ros::Time::now ();
// 
// 
//     image_pub_.publish (msg_out);
// 
// 
// 
//   }
// 
// };
// 
// int main (int argc, char **argv)
// {
// //   fstream outfile("/home/pedrobatista/workingcopy/lar3/perception/pedestrians/PedestrianDetect/train_seed1234_MIN8_MAX64.csv");
//   ros::init (argc, argv, "image_converter");
//   PedestrianDetect ic;
//   ros::spin ();
//   return 0;
//   
// }
