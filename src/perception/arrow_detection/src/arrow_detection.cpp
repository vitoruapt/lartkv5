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
 
 /*! 
 *      @file    arrow_detection.cpp
 *      @brief  usinf find arrow class, and image_raw_topic_ throw the point of the center of the arrow for the track arrow pakge
 *  
 *      @author         César Sousa, cesarsousa@ua.pt
 *      @date 6-3-2014
 *      @version V0.0
 *      @internal
 * 
 *              Revision        ---
 *              Compiler        gcc
 *              Company         DEM - Universidade de Aveiro
 *              Copyright       Copyright (c) 2014, César Sousa
 * 
 *              Info:           
 *      
 *      command to make doxyfile: "make doxyfile" than "make doc"
 * 
 */

#include <arrow_detection/arrow_detection.h>
 
class ImageConverter
{
     ros::NodeHandle nh_;
     
     ros::Publisher pub_state;
     
     image_transport::ImageTransport it_;
     image_transport::Subscriber image_sub_;
     
     
 public:
     ImageConverter()
     : it_(nh_)
     {
std::cout << "initialiing ImageConverter constructor from 'arrow_detection' " << std::endl;
        
        // Subscribe to input video feed and publish output video feed
         image_sub_ = it_.subscribe("/camera/image_rect_color", 1, &ImageConverter::imageCallback, this);
         //image_sub_ = it_.subscribe("/camera/image_raw", 1, &ImageConverter::imageCallback, this);
         pub_state= nh_.advertise< geometry_msgs::Point  >( "/find_arrow_position", 1000 );
     }
     
     ~ImageConverter()
     {

     }
     
     void imageCallback(const sensor_msgs::ImageConstPtr& msg)
     {

        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            //pick the last available frame
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
         
        //// Update GUI Window
        //cv::imshow("arrow_detection:: grabbed image", cv_ptr->image);
        //cv::waitKey(3);
         
        cv::Point src_centre;
        
        cv::Mat grey_image_orig;
        //convert to grey scale
        cvtColor(cv_ptr->image, grey_image_orig, CV_BGR2GRAY); 
        findArrow::find_arrow(grey_image_orig,src_centre);
        //found new point of the arrow's position?  if yes, then send it to track_arrow
        if (findArrow::found_new_point == true)
        {
//std::cout << "x: " << src_centre.x <<  " y: " << src_centre.y << std::endl;
            geometry_msgs::Point center_point;
            center_point.x = src_centre.x; //copy points form cv::point to geometry_msgs::Point
            center_point.y = src_centre.y;
            center_point.z = 0;
            pub_state.publish(center_point);
         
            findArrow::found_new_point = false;
        }
    }
 };
 
 int main(int argc, char** argv)
 {
     ros::init(argc, argv, "arrow_detection");
     ImageConverter ic;
     ros::spin();
     return 0;
 }
 
 
