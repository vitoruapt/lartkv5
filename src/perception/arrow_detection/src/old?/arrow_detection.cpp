 /**************************************************************************************************
  * Software License Agreement (BSD License)
  * 
  * Copyright (c) 2011-2014, LAR toolkit developers - University of Aveiro - http://lars.mec.ua.pt
  * All rights reserved.
  * 
  * Redistribution and use in source and binary forms, with or without modification, are permitted
  * provided that the following conditions are met:
  * 
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
     //   ros::Publisher datamatrix_pub;
     
     image_transport::ImageTransport it_;
     image_transport::Subscriber image_sub_;
     
     //   std::vector<std::string> leituras;
     
 public:
     ImageConverter()
     : it_(nh_)
     {
         // Subscribe to input video feed and publish output video feed
         image_sub_ = it_.subscribe("/camera/image_raw", 1,
         &ImageConverter::imageCallback, this);
         //     datamatrix_pub = nh_.advertise<datamatrix_detection::DatamatrixMsg>("datamatrix_detection/datamatrix_msg",1);
     }
     
     ~ImageConverter()
     {
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
         /*
          *   DatamatrixDecode(cv_ptr->image,cv_ptr->header);*/
         
         
         // Update GUI Window
         cv::imshow("Image", cv_ptr->image);
         cv::waitKey(3);
         
     }
     
     
 };
 
 int main(int argc, char** argv)
 {
     ros::init(argc, argv, "datamatrix_detection");
     ImageConverter ic;
     ros::spin();
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

//c++
// #include <stdio.h>
// #include <iostream>

////Visp
//#include <visp/vpPixelMeterConversion.h>
//#include <visp/vp1394CMUGrabber.h>
//#include <visp/vp1394TwoGrabber.h>
//#include <visp/vpDisplayGDI.h>
//#include <visp/vpDisplayX.h>
//#include <visp/vpDot2.h>
//#include <visp/vpPose.h>
//#include <visp/vpConfig.h>
//#include <visp/vpImage.h>
//#include <visp/vpRGBa.h>

////convert images //cesar
//#include <visp/vpImageIo.h>
//#include <visp/vpImageConvert.h>

////my files
//#include <arrow_detection/findArrow.h>

////ros
//#include <ros/ros.h>
//#include <std_msgs/String.h>

//#include <sstream>
//#include <unistd.h>
//#include <iostream>
//#include <cmath>  

//#include <sys/types.h>
//#include <unistd.h>
//#include <stdlib.h>

//#include <geometry_msgs/Point.h>


//void wait_lum_set(vpImage<unsigned char> &I_grey,
//#if defined(VISP_HAVE_DC1394_2)
    //vp1394TwoGrabber &g//THIS, is the one that is called!!! 
//#elif defined(VISP_HAVE_CMU1394)
    //vp1394CMUGrabber g
    
//#endif
//)
//{
    //std::cout << "**PLEASE SET THE CAMERA LUMINIOSITY!! ***" << std::endl;
    ////std::cout << "press 'Esc' to exit" << std::endl;
    ////while (true)
    ////{
        //////if (cv::waitKey(20) == 27)
        ////if(vpDisplay::getClick(I_grey))
        ////{
            ////break;
        ////}
        ////g.acquire(I_grey);//g.acquire(I);
        ////vpDisplay::display(I_grey);//vpDisplay::display(I);
        ////vpDisplay::flush(I_grey);
    ////}
    ////std::cout << "Luminiosity setted!" << std::endl;
//}

///*** Main***
// int main(int argc, char** argv)
// {
    
    ////nome do nodo
    //ros::init( argc, argv, "find_arrow_node" );
    
    //ros::NodeHandle n("~");
    
    ////where and what will be published
    //ros::Publisher pub_state= n.advertise< geometry_msgs::Point  >( "/find_arrow_state", 1000 );
    
    //ros::Rate r(20); // 50 hz
    
    //cv::VideoCapture cap(0);// open the first avialable video camera (no. 0?)

    //const int min_servo_pos=600;
    
    //namedWindow("frame", CV_WINDOW_AUTOSIZE);    //create a window called "MyVideo"
    
    //double dWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH);   //get the width of frames of the video
    //double dHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT); //get the height of frames of the video
    ////width 640 height 480
    
    //#if (defined(VISP_HAVE_DC1394_2) || defined(VISP_HAVE_CMU1394))
    //vpImage<unsigned char> I; // Create a gray level image container

//#if defined(VISP_HAVE_DC1394_2)
    //vp1394TwoGrabber g(false);
//#elif defined(VISP_HAVE_CMU1394)
    //vp1394CMUGrabber g;
//#endif
    //g.open(I);
    //g.acquire(I);

//#if defined(VISP_HAVE_X11)
    //vpDisplayX d(I, 0, 0, "Camera view");
//#elif defined(VISP_HAVE_GDI)
    //vpDisplayGDI d(I, 0, 0, "Camera view");
//#else
    //std::cout << "No image viewer is available..." << std::endl;
//#endif
    //vpDisplay::display(I);
    //vpDisplay::flush(I);
//std::cout << "teste1" << std::endl;
    //vpDot2 blob;
    //blob.setGraphics(true);
    //blob.setGraphicsThickness(2);
    ////blob.initTracking(I);
    //cv::Point src_centre;   bool init = true;   cv::Mat I_mat;
    //std::cout << "teste2" << std::endl;
    
    //wait_lum_set(I,g);//set luminiosity
  //#endif  
    
    ////send to initial position
    //geometry_msgs::Point ponto;
    //int const start_pos_x = 1950;
    //int const start_pos_y = 1470;
    ////set start position
    //ponto.x = start_pos_x; ponto.y = start_pos_y;
   
    //int const xcenter = 235;
    //int const ycenter = 320;
    //std::cout << "before loop..." << std::endl;
    
    //while(ros::ok())
    //{  
        //try
        //{
            //if (init)
            //{
                //ponto.x = 1; ponto.y = 0; ponto.z = 0;
                //pub_state.publish( ponto );
                //std::cout << "arrow lost!" << std::endl;
                //vpImageConvert::convert(I,I_mat);
                //vpTime::wait(10);
                
                //findArrow::find_arrow(I_mat,src_centre);
                //if (findArrow::found_new_point == true)
                //{
                    //blob.initTracking(I,vpImagePoint(src_centre.y, src_centre.x));//cesar
                    //init = false;
                //}
                ////update image
                //g.acquire(I);
                //vpDisplay::display(I);
                //vpDisplay::flush(I);
            //}   
            //else 
            //{
                //std::cout << "arrow FOUND!" << std::endl;
                //vpImagePoint center(blob.getCog());
                //std::cout << "i: " << (int) center.get_i() << " j:" << (int) center.get_j() << std::endl;
                
            //////ROS
                ////ponto.x = 2000 - ((int) center.get_i());
                ////verificar posiçoes maxias antes de enviar a posição para os servos
                ////ponto.x = start_pos_x - ((int) center.get_i()) + xcenter;
                ////std::cout << "((int) center.get_i()) + xcenter" << ((int) center.get_i()) + xcenter << std::endl;
                ////ponto.y = start_pos_y + (((int) center.get_j()) - ycenter);
                ////std::cout << "(((int) center.get_j()) - ycenter);" << (((int) center.get_j()) - ycenter) << std::endl;
                ////ponto.z = 3; // VELOCiTy
                
                //int dif_x = (((int) center.get_i()) - xcenter);
                //int dif_y = (((int) center.get_j()) - ycenter);
                //std::cout << "dif_x" << dif_x << "dif_y" << dif_y << std::endl;
                
                //if(dif_x > 0 && dif_y > 0)
                //{
                    //ponto.x = 1;
                //}
                //if(dif_x > 0 && dif_y < 0)
                //{
                    //ponto.x = 2;
                //}
                //if(dif_x < 0 && dif_y > 0)
                //{
                    //ponto.x = 3;
                //}
                //if(dif_x < 0 && dif_y < 0)
                //{
                    //ponto.x = 4;
                //}
                
                ////MOD
                //dif_y < 0 ? dif_y = -dif_y : dif_x = dif_x;     
                //dif_x < 0 ? dif_x = -dif_x : dif_x = dif_x;
                
                
                //ponto.y = ((int) dif_x) / 15;                
                //ponto.z = ((int) dif_y) / 15;                
                
                //std::cout << "x: " << ponto.x << "y:" << ponto.y << "vel" << ponto.z << std::endl;
                ////if ((dif_x + dif_y) < 50)
                ////{
                    //pub_state.publish( ponto );
                    
                ////}
              
                //g.acquire(I); // Acquire an image
                //vpDisplay::display(I);
                //blob.track(I);
                //vpDisplay::flush(I);
                //if (vpDisplay::getClick(I, false))
                //break;
            //}
        //}
        //catch(vpException e) 
        //{
            //std::cout << "Catch an exception: " << e << std::endl;
            //init=true;
        //}
        //ros::spinOnce ( );
        //r.sleep(); //rosrate (50Hz)
    //}
    
    //std::cout << " Exit successful" << std::endl;
    //return 0;

    
// }
