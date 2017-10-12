#include <ros/ros.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

#include "mtt/TargetList.h"

#include <image_transport/image_transport.h>

#include <cv_bridge/cv_bridge.h>

#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud.h"
#include <sensor_msgs/image_encodings.h>

#include <pcl_conversions/pcl_conversions.h>

using namespace std;
using namespace cv;
namespace enc = sensor_msgs::image_encodings;

class ImageConverter
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    ros::Subscriber mtt_laser_sub_;
    
    public:
        
        double distance_to_target_from_center_bumper;
        double largura;
        int laser_done;
        Point3f laser_left_point;
        Point3f laser_right_point;
        
        std::string imagePoints1FileName;
//         std::string imagePoints2FileName;
//         std::string imagePoints3FileName;

        std::string objectPoints1FileName;
//         std::string objectPoints2FileName;
//         std::string objectPoints3FileName;
        
        ImageConverter()
        : it_(nh_)
        {
            largura=0;
            laser_done = 0;
            
            ros::NodeHandle nh_local("~");
            
            image_sub_ = it_.subscribe("/xb3/right/new_info/image_rect_color", 1, &ImageConverter::imageCb, this);
//             image_sub_ = it_.subscribe("image", 1, &ImageConverter::imageCb, this);
//             image_sub_ = it_.subscribe("/usb_cam_reader/image_raw", 1, &ImageConverter::imageCb, this);
            mtt_laser_sub_ = nh_.subscribe("/new_targets", 1, &ImageConverter::mttLaserGather, this);
    
            nh_local.param<std::string>("imagePoints1FileName",imagePoints1FileName,"");
//             nh_local.param<std::string>("imagePoints2FileName",imagePoints2FileName,"");
//             nh_local.param<std::string>("imagePoints3FileName",imagePoints3FileName,"");
            
            nh_local.param<std::string>("objectPoints1FileName",objectPoints1FileName,"");
//             nh_local.param<std::string>("objectPoints2FileName",objectPoints2FileName,"");
//             nh_local.param<std::string>("objectPoints3FileName",objectPoints3FileName,"");
            
        }

        ~ImageConverter()
        {

        }
        
        void imageCb(const sensor_msgs::ImageConstPtr& cam_msg)
        {
            cv_bridge::CvImagePtr cv_ptr;
            try
            {
                cv_ptr = cv_bridge::toCvCopy(cam_msg, enc::BGR8);
            }
            catch (cv_bridge::Exception& e)
            {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            }
//             imshow("input4", cv_ptr->image);
        // Convert code to ros, to subscrive image and laser data
            
//             Mat input = imread("/home/asus/workingcopies/lar4/src/perception/pedestrians/multimodal_pedestrian_detect/images/frame0009.jpg", CV_LOAD_IMAGE_COLOR);
            
            Mat input = cv_ptr->image;
            Mat redOnly;
            
            if (laser_done == 2)
            {
//                 resize(input, input, Size(800,600));
                
    //             for(int j=1; j<210; j++)
    //                 for(int i=1; i<800 ; i++)
    //                     input.at<double>(j,i) = 0;
                
                //Grayscale matrix
                Mat grayscaleMat(input.size(), CV_8UC1);
                
//                 imshow("drawing1", grayscaleMat);
                
                cvtColor(input, grayscaleMat, CV_BGR2GRAY);
                threshold(grayscaleMat, grayscaleMat, 170, 255, THRESH_BINARY);
                
//                 imshow("drawing1", grayscaleMat);
    //             inRange(input, Scalar(75, 90, 0), Scalar(120,170,15), redOnly); // placa verde plastica
//                 inRange(input, Scalar(180, 180, 180), Scalar(255,255,255), redOnly); // placa branca 
                
                //Find the contours. Use the contourOutput Mat so the original image doesn't get overwritten
                vector<vector<Point> > contours;
//                 Mat contourOutput = redOnly.clone();
                Mat contourOutput = grayscaleMat.clone();
                findContours( contourOutput, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE );
                
                vector<vector<Point> > contours_poly( contours.size() );
                
                Mat drawing = Mat::zeros( grayscaleMat.size(), CV_8UC3 );
                
                int max_size_triangle=0;
                for(size_t i = 0; i < contours.size(); i++)
                {
                    approxPolyDP(Mat(contours[i]), contours_poly[i], 5, true);
    //                 cout << "number of triangles: " << contours_poly.size() << endl;

    //                 cout << "size_t: " << i << endl;
//                     cout << "size_poly: " << contours_poly[i].size() << endl;
                    drawContours(drawing, contours_poly, i, Scalar(0, 0, 255),CV_FILLED,8); // fill RED 
                    if (contours_poly[i].size() == 4) //from all object, only triangles are choosen
                    {
//                         cout << "arclength: " << arcLength(Mat(contours[i]),true) << endl;
//                         cout << "size_poly: " << contours_poly[i].size() << endl;
//                         cout << "size: " << contours[i].size() << endl;
    //                     drawContours(drawing, contours_poly, i, Scalar(0, 0, 255),CV_FILLED,8); // fill RED 
                        if(i > 0)
                            if( arcLength(Mat(contours[i]),true) > arcLength(Mat(contours[max_size_triangle]),true)) //choose bigger triangle
                            {
                                max_size_triangle = i;
                            }
                    }
                }
                
                double min_width;
                double max_width;
                double left_height;
                double right_height;
                
                Point left_upper_point;
                Point right_upper_point;
                Point left_lower_point;
                Point right_lower_point;
                
                if (contours_poly[max_size_triangle][1].y < (contours_poly[max_size_triangle][0].y + 10) ) 
                {
                    left_upper_point.x = contours_poly[max_size_triangle][1].x;
                    left_upper_point.y = contours_poly[max_size_triangle][1].y;
                    
                    right_upper_point.x = contours_poly[max_size_triangle][0].x;
                    right_upper_point.y = contours_poly[max_size_triangle][0].y;
                    
                    left_lower_point.x = contours_poly[max_size_triangle][2].x;
                    left_lower_point.y = contours_poly[max_size_triangle][2].y;
                    
                    right_lower_point.x = contours_poly[max_size_triangle][3].x;
                    right_lower_point.y = contours_poly[max_size_triangle][3].y;
                    
                }else{
                    left_upper_point.x = contours_poly[max_size_triangle][0].x;
                    left_upper_point.y = contours_poly[max_size_triangle][0].y;
                    
                    right_upper_point.x = contours_poly[max_size_triangle][3].x;
                    right_upper_point.y = contours_poly[max_size_triangle][3].y;
                    
                    left_lower_point.x = contours_poly[max_size_triangle][1].x;
                    left_lower_point.y = contours_poly[max_size_triangle][1].y;
                    
                    right_lower_point.x = contours_poly[max_size_triangle][2].x;
                    right_lower_point.y = contours_poly[max_size_triangle][2].y;
                }
                
                min_width = right_upper_point.x - left_upper_point.x;
                max_width = right_lower_point.x - left_lower_point.x;
                left_height = left_lower_point.y - left_upper_point.y;
                right_height = right_lower_point.y - right_upper_point.y;
                
                cout << "triangle pixel size: " << max_size_triangle << "  " << contours_poly[max_size_triangle] << "  " << arcLength(Mat(contours[max_size_triangle]),true) << endl
                    << "max width: " << max_width << endl
                    << "min width: " << min_width << endl
                    << "left_height: " << left_height << endl
                    << "right_height: " << right_height << endl;
                
                drawContours(drawing, contours_poly, max_size_triangle, Scalar(255, 255, 255),CV_FILLED,8); // fill WHITE  
                
//                 cout << "here" << endl;
                
                double object_width_percent_laser_given = largura; // inside value is the percentage, return by the laser
                
                int laser_left_line_int = object_width_percent_laser_given * left_height;
                double laser_left_line = object_width_percent_laser_given * left_height;
                
                int laser_right_line_int = object_width_percent_laser_given * right_height;
                double laser_right_line = object_width_percent_laser_given * right_height;
                
    //             cout << "laser_line, double: " << laser_line << " int: " << laser_line_int << endl;
                
                if(laser_left_line - laser_left_line_int >=0.5 )
                    laser_left_line_int++;
                
                if(laser_right_line - laser_right_line_int >=0.5 )
                    laser_right_line_int++;
                
                int laser_left_line_position = left_upper_point.y + laser_left_line_int;
                int laser_right_line_position = right_upper_point.y + laser_right_line_int;
                
                //Grayscale matrix
                Mat grayscaleMat_2(drawing.size(), CV_8UC1);

                //Convert BGR to Gray
                cvtColor(drawing, grayscaleMat_2, CV_BGR2GRAY);
                
                threshold(grayscaleMat_2, grayscaleMat_2, 245, 255, THRESH_BINARY);
//                 cout << "here2" << endl;
                int k=0;
                Point left_point;
                Point right_point;
                
                for(int col=(left_lower_point.x - 10); col<(right_lower_point.x + 10); col++) 
                    if(grayscaleMat_2.at<uchar>(laser_left_line_position,col) != 0) 
                        if (k==0)
                        {
                            left_point.y=laser_left_line_position;
                            left_point.x=col - 2;
                            break;
                        }
                 
                for(int col=(right_lower_point.x + 10); col>(left_lower_point.x - 10); col--) 
                    if(grayscaleMat_2.at<uchar>(laser_left_line_position,col) != 0) 
                        if (k==0)
                        {
                            right_point.y=laser_left_line_position;
                            right_point.x=col + 2;
                            break;
                        }
                
                        
//                 cout << "laser_line, double: " << laser_line << " int: " << laser_line_int << endl;
                cout << "points, left_point: " << left_point << " right_point: " << right_point << endl;

                line(input, left_point, right_point, Scalar(0, 0, 255), 2, 8,0);
                
    //             // gravar pontos da imagem automaticamente
                
    //                 Grava os pontos da imagem onde o feixe laser está a tocar
                fstream fs;
                fs.open (imagePoints1FileName, fstream::in | fstream::out | fstream::app);

                
                fs << left_point.x << "  " << left_point.y << "\n";  
                fs << right_point.x << "  " << right_point.y << "\n";
                
                fs.close();

                cout << "please change the object position, and then run the program again" << endl;
                
                imshow("input_incial", input);
                
                imshow("drawing", drawing);
                imshow("drawing1", grayscaleMat_2);
                waitKey(0);
                
                image_sub_.shutdown();
            }
            
        }
    
        void  mttLaserGather(const mtt::TargetList& laser_msg)
        {               
            double distance_to_target_from_center_bumper_;
            double theta_;
            double largura_;
            
            cout << "laser_size: " << laser_msg.Targets.size() << endl;
            
            for(int i=0; i<laser_msg.Targets.size();i++)
            {
                distance_to_target_from_center_bumper_ = sqrt(laser_msg.Targets[i].pose.position.x * laser_msg.Targets[i].pose.position.x + 
                                    laser_msg.Targets[i].pose.position.y * laser_msg.Targets[i].pose.position.y);
                
                theta_ = acos(laser_msg.Targets[i].pose.position.x / distance_to_target_from_center_bumper);
                
                largura_ = sqrt(((laser_msg.Targets[i].finalpose.x - laser_msg.Targets[i].initialpose.x) * (laser_msg.Targets[i].finalpose.x - laser_msg.Targets[i].initialpose.x)) + ((laser_msg.Targets[i].finalpose.y - laser_msg.Targets[i].initialpose.y) * (laser_msg.Targets[i].finalpose.y - laser_msg.Targets[i].initialpose.y)));
                
                if(laser_msg.Targets[i].pose.position.x > 0 && laser_msg.Targets[i].pose.position.y > -10 && laser_msg.Targets[i].pose.position.y < 10 /*&& largura_ > 0.10 && largura_ < 1.50*/) {
//                 if(laser_msg.Targets[i].id == 1)
//                 {
                    distance_to_target_from_center_bumper = distance_to_target_from_center_bumper_;
                    
                    cout << "largura"<< largura << endl;
                    
                    if (largura_ > largura)
                    {
                        largura = largura_;
                    
                        laser_left_point = Point3f(laser_msg.Targets[i].initialpose.x, laser_msg.Targets[i].initialpose.y, laser_msg.Targets[i].initialpose.z);
                        
                        laser_right_point = Point3f(laser_msg.Targets[i].finalpose.x, laser_msg.Targets[i].finalpose.y, laser_msg.Targets[i].finalpose.z);
                    }
                }
            }
            
            laser_done = laser_done + 1;
            
            cout << "laser_done, width: " << largura << endl;
                
            if (laser_done==2)
            {
    //              Gravar pontos dos objectos automaticamente
                fstream fs;
                fs.open (objectPoints1FileName, fstream::in | fstream::out | fstream::app);

                fs << laser_left_point.x << "   " << laser_left_point.y << "   " << 0 << "\n";
                fs << laser_right_point.x << "   " << laser_right_point.y << "   " << 0 << "\n";
                
                fs.close();            
            
                mtt_laser_sub_.shutdown(); // the subscriber only runs one time
            }
        }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "calibrate_save_camera_laser_points_node");
  
  ImageConverter ic;
  
  ros::spin();
  return 0;
}