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
#include <rosbag/bag.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/LaserScan.h>

#include <std_msgs/String.h>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/lexical_cast.hpp>
// #include <boost/thread.hpp>

using namespace std;

class MyRosBag
{
    rosbag::Bag bag;
    std::string image_topic;
    std::string info_topic;
    std::string laser_topic;
    
    ros::NodeHandle nh;
    ros::Subscriber image_sub;
    ros::Subscriber info_sub;
    ros::Subscriber laser_sub;
    
    bool record;
    public:
        MyRosBag(std::string path_, std::string image_topic_, std::string info_topic_, std::string laser_topic_)
        :bag(),
        image_topic(image_topic_),
        info_topic(info_topic_),
        laser_topic(laser_topic_)
        {
            global = ros::Time::now();
            start = ros::Time::now();
            ref = ros::Time::now();
            
            boost::posix_time::ptime t(boost::posix_time::second_clock::local_time());  //  Gets the current date and time
            boost::gregorian::date d = t.date();                                        //  Gets the date part
            std::string date = to_iso_extended_string(d);                               //  Converts the date to a string
            
            boost::posix_time::time_duration time = t.time_of_day();                    //  Gets the time part
            
            
//             std::string hours = boost::lexical_cast<std::string>(time.hours());         //  Gets the current hours
//             std::string minutes = boost::lexical_cast<std::string>(time.minutes());     //  Gets the current minutes
//             std::string seconds = boost::lexical_cast<std::string>(time.seconds());     //  Gets the current seconds
            
            boost::format hours("%02f");    //  Format the output string
            hours % time.hours();                                //  Assign alpha to the output string
            
            boost::format minutes("%02f");    //  Format the output string
            minutes % time.minutes();  
            
            boost::format seconds("%02f");    //  Format the output string
            seconds % time.seconds();  
            
            std::string path = path_ + "_" + date + "-" + hours.str() + "-" + minutes.str() + "-" + seconds.str() + ".bag";   //  Sets the path where to save the bag file
            bag.open(path, rosbag::bagmode::Write);                                     //  Creates the bag file in write mode
            record = false;
        }
        
        ~MyRosBag()
        {
            std::cout<<__FUNCTION__<<std::endl;
            bag.close();                                                                //  Closes the previously created bag file
            std::cout<<__FUNCTION__<<std::endl;
        }
        
        void SetupSubs()
        {
//             topic_sub = nh.subscribe(topic,1000,&MyRosBag::WriteBagCallback, this);
            image_sub = nh.subscribe(image_topic,1,&MyRosBag::compressedImageHandler, this);
            info_sub = nh.subscribe(info_topic,1,&MyRosBag::CameraInfoHandler, this);
            laser_sub = nh.subscribe(laser_topic,1,&MyRosBag::LaserScanHandler, this);
        }
        
        
        void toogle()
        {
            if(record)
            {
//                 std::cout<<"Paused"<<std::endl;
                ref = ref + (ros::Time::now() - start);
                record = false;
                
            }else
            {
//                 std::cout<<"Start"<<std::endl;
                start = ros::Time::now();
                record = true;
            }
            
        }        
        
    private:
        
        void writeImage(const sensor_msgs::ImagePtr& msg)
        {
            ros::Time warped_time = ref + (ros::Time::now() - start);
//             msg->header.stamp = warped_time;

//             cout<<"Ref time: "<<ref-global<<endl;
//             cout<<"Start time: "<<start-global<<endl;
//             cout<<"Writing time: "<<warped_time-global<<endl<<endl;
                
            bag.write(image_topic, warped_time, msg);
        }
        
        void writeImage(const sensor_msgs::CompressedImagePtr& msg)
        {
            ros::Time warped_time = ref + (ros::Time::now() - start);
//             msg->header.stamp = warped_time;

//             cout<<"Ref time: "<<ref-global<<endl;
//             cout<<"Start time: "<<start-global<<endl;
//             cout<<"Writing time: "<<warped_time-global<<endl<<endl;
                
            bag.write(image_topic, warped_time, msg);
        }
        
        
        void compressedImageHandler(const sensor_msgs::CompressedImagePtr msg)
        {
            if (record)
            {
                writeImage(msg);
//                 threads.create_thread(boost::bind(&MyRosBag::writeImage,this,msg));
            

            }else
            {
//                 cout<<"not writing"<<endl;
            }
        }
        
        void CameraInfoHandler(const sensor_msgs::CameraInfo& cam_info_msg)
        {
            if ( record )
            {
                ros::Time warped_time = ref + (ros::Time::now() - start);
    //             cam_info_msg.header.stamp = warped_time;
                
//                 cout<<"Ref time: "<<ref-global<<endl;
//                 cout<<"Start time: "<<start-global<<endl;
//                 cout<<"Writing time: "<<warped_time-global<<endl<<endl;
                
                bag.write(info_topic, warped_time, cam_info_msg);
            }
//             info_topic.shutdown();
        }

        void LaserScanHandler(const sensor_msgs::LaserScan& laser_msg)
        {
            if ( record )
            {
                ros::Time warped_time = ref + (ros::Time::now() - start);
    //             cam_info_msg.header.stamp = warped_time;
                
//                 cout<<"Ref time: "<<ref-global<<endl;
//                 cout<<"Start time: "<<start-global<<endl;
//                 cout<<"Writing time: "<<warped_time-global<<endl<<endl;
                
                bag.write(laser_topic, warped_time, laser_msg);
            }
        }
        
//         boost::thread_group threads;
        
        ros::Time global;
        ros::Time ref;
        ros::Time start;
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "bag");
    
    ros::NodeHandle nh("~");
  
    std::string path;
    nh.param<std::string>("bag_path",path,"");
    
    std::string image_topic;
    nh.param<std::string>("image_topic",image_topic," ");
    
    std::string info_topic;
    nh.param<std::string>("info_topic",info_topic," ");
    
    std::string laser_topic;
    nh.param<std::string>("laser_topic",laser_topic," ");
    
    MyRosBag my_bag(path, image_topic, info_topic, laser_topic);
    my_bag.SetupSubs();
    
//     cv::namedWindow("Display window", cv::WINDOW_AUTOSIZE);
    cv::Mat image(320, 240, CV_8UC3, cv::Scalar(0,0,0));
    cv::Mat image_clone = image.clone();
    cv::putText(image_clone, std::string("Paused"), cv::Point (image.cols/2 - 50 , image.rows/2 - 10), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0,255,0), 2, 8);
    
    bool record = false;
    ros::Rate r(500);
    ros::Time begin;
    while(ros::ok()){
        char key;
//         std::cout << key << std::endl;
        if ((ros::Time::now() - begin).toSec() >= 2 & record)
        {
            my_bag.toogle();
            image_clone = image.clone();
            cv::putText(image_clone, std::string("Paused"), cv::Point (image.cols/2 - 50 , image.rows/2 - 10), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0,255,0), 2, 8);
            record = false;
        }
        else 
            key = cv::waitKey(2);
        if (key == 'p' )
        {
            my_bag.toogle();
            image_clone = image.clone();
            if (record)
            {
                cv::putText(image_clone, std::string("Paused"), cv::Point (image.cols/2 - 50 , image.rows/2 - 10), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0,255,0), 2, 8);
                record = false;
            }else
            {
                cv::putText(image_clone, std::string("Recording"), cv::Point (image.cols/2 - 60 , image.rows/2 - 10), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0,255,0), 2, 8);
                record = true;
                begin = ros::Time::now();
            }
            
        }else if(key=='q')
            break;
        cv::imshow("Display window", image_clone);
        ros::spinOnce();
        r.sleep();
    }
    
    
    return 0;
}