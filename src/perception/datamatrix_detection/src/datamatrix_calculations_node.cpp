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
#include <fstream>
#include <string>

#include <ros/ros.h>

#include <datamatrix_detection/DatamatrixMsg.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/LaserScan.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <boost/format.hpp>
#include <boost/shared_ptr.hpp>

#include <cmath>

// 

// class my_super_vector 
// {
//     public:
//         std::vector<double> data;
//         std::vector<double> timestamp;
//     
//         void push_back(double val)
//         {
//             data.push_back(val);
//         }
//         
//         double operator[](int i)
//         {
//             return data[i];
//         }
//         
//         double meanPos(int size)
//         {
// //             for
//         }
//         
// };
// 
// my_super_vector vec;
// 
// vec.push_back(4);
// cout<<"vopc_: "<< vec.timestamp[0]<<endl

class GlobalPosition: public cv::Point2d
{
    public:
        
        GlobalPosition()
        {
            phi = 0;
            x = 0;
            y = 0;
        }
        
        GlobalPosition(double x_,double y_,double phi_):
        Point_(x_,y_)
        {
            phi = phi_;
        }
        
        GlobalPosition operator=(const cv::Point2d& p)
        {
            return GlobalPosition(p.x,p.y,0);
        }
        
        typedef boost::shared_ptr<GlobalPosition> Ptr;
        
        double phi;
};


GlobalPosition operator+(const GlobalPosition& p1,const GlobalPosition& p2)
{
    cv::Point2d po = static_cast<cv::Point2d>(p1)+static_cast<cv::Point2d>(p2);
 
    return GlobalPosition(po.x,po.y,p1.phi+p2.phi);
}

GlobalPosition::Ptr operator+(const GlobalPosition::Ptr& p1,const GlobalPosition::Ptr& p2)
{
    return boost::make_shared<GlobalPosition>(*p1 + *p2);
}

GlobalPosition operator/(const GlobalPosition& p1, double i)
{
    return GlobalPosition(p1.x / i,p1.y / i ,p1.phi / i);
}

GlobalPosition::Ptr operator/(const GlobalPosition::Ptr& p1, double i)
{
    return boost::make_shared<GlobalPosition>(*p1/i);
}

GlobalPosition operator*(const GlobalPosition& p1, double i)
{
    return GlobalPosition(p1.x * i,p1.y * i ,p1.phi * i);
}

GlobalPosition::Ptr operator*(const GlobalPosition::Ptr& p1, double i)
{
    return boost::make_shared<GlobalPosition>(*p1 * i);
}


class LaserPosition
{
//     Received variables
//     std::vector<cv::Point2d> matrix_coordinates;
    std::vector<double> angle;
    sensor_msgs::LaserScan laser_data;
    
    
    public:
    LaserPosition (std::vector<double> angle_, sensor_msgs::LaserScan laser_data_)
    {
//         matrix_coordinates = matrix_coord_;
        angle = angle_;
        laser_data = laser_data_;
        
//         std::vector<double> distance_ = GetLaserDistance( angle_, laser_data_);
//         GetLaserPosition(matrix_coord_, angle_, distance_);
    }
    
    typedef boost::shared_ptr<LaserPosition> Ptr;
    
    
    std::vector<double> GetLaserDistance()
    {
        std::vector<double> distances_;
        for ( uint i = 0; i < angle.size(); i++)
        {
            int j = round(((angle[i] - 2)*M_PI/180 - laser_data.angle_min)/laser_data.angle_increment);
            if ( laser_data.ranges[j] < laser_data.range_max && laser_data.ranges[j] > laser_data.range_min )
                distances_.push_back(laser_data.ranges[j]);
            else
                distances_.push_back(std::numeric_limits<double>::quiet_NaN());
        }
        return distances_;
    }
    
//     GlobalPosition::Ptr GetLaserPosition(std::vector<double>& distance)
//     {
//         GlobalPosition::Ptr laser_position_ptr;
//         if ( distance.size() >= 2 )
//         {
// //         std::vector<GlobalPosition::Ptr> my_intermediate_positions;
//             int i = 0;
//             GlobalPosition::Ptr sum(new GlobalPosition);
//             for ( uint j = 0; j < distance.size() - 1; j++)
//             {
//                 for ( uint k = j + 1; k < distance.size(); k++ )
//                 {
//                     GlobalPosition::Ptr my_intermediate_position_ptr = CalculateCircleIntersection( j, k, distance );
//                     if (!std::isnan(my_intermediate_position_ptr->x) && !std::isnan(my_intermediate_position_ptr->y) && !std::isnan(my_intermediate_position_ptr->phi))
//                     {
//     //                     my_intermediate_positions.push_back(my_intermediate_position_ptr);
//                         i++;
//                         sum = sum + my_intermediate_position_ptr;
//                     }
//                 }
//             }
//             return sum / i;
//         }
//         else
//         {
//             double return_nan = std::numeric_limits<double>::quiet_NaN();
//             return GlobalPosition::Ptr (new GlobalPosition(return_nan,return_nan,return_nan));
//         }
// //         return sum / my_intermediate_positions.size();
//     }
    
//     private:
//     GlobalPosition::Ptr CalculateCircleIntersection( uint first, uint second, std::vector<double>& distance)
//     {
//         
//         double a, dx, dy, d, h, rx, ry;
//         double x2, y2;
// 
// //         My matrices coordinates
//         double x0 = matrix_coordinates[first].x;
//         double y0 = matrix_coordinates[first].y;
//         double r0 = distance[first]*100;
//         
//         double x1 = matrix_coordinates[second].x;
//         double y1 = matrix_coordinates[second].y;
//         double r1 = distance[second]*100;
//         
//         /* dx and dy are the vertical and horizontal distances between
//         * the circle centers.
//         */
//         dx = x1 - x0;
//         dy = y1 - y0;
// 
//         /* Determine the straight-line distance between the centers. */
//         //d = sqrt((dy*dy) + (dx*dx));
//         d = hypot(dx,dy); // Suggested by Keith Briggs
//         
// //         /* Check for solvability. */
//         if (d > (r0 + r1))
//         {
//             /* no solution. circles do not intersect. */
//             double return_nan = std::numeric_limits<double>::quiet_NaN();
//             return GlobalPosition::Ptr (new GlobalPosition(return_nan,return_nan,return_nan));
//         }
//         if (d < fabs(r0 - r1))
//         {
//             /* no solution. one circle is contained in the other */
//             double return_nan = std::numeric_limits<double>::quiet_NaN();
//             return GlobalPosition::Ptr (new GlobalPosition(return_nan,return_nan,return_nan));
//         }
//         /* 'point 2' is the point where the line through the circle
//         * intersection points crosses the line between the circle
//         * centers.
//         */
// 
//         /* Determine the distance from point 0 to point 2. */
//         a = ((r0*r0) - (r1*r1) + (d*d)) / (2.0 * d) ;
// 
//         /* Determine the coordinates of point 2. */
//         x2 = x0 + (dx * a/d);
//         y2 = y0 + (dy * a/d);
// 
//         /* Determine the distance from point 2 to either of the
//         * intersection points.
//         */
//         h = sqrt((r0*r0) - (a*a));
// 
//         /* Now determine the offsets of the intersection points from
//         * point 2.
//         */
//         rx = -dy * (h/d);
//         ry = dx * (h/d);
// 
//         /* Determine the absolute intersection points.*/
// //         Coordinates are in centimeters -----------------------------------------!!!
//         cv::Point2d int_point1(x2 + rx, y2 + ry);
//         cv::Point2d int_point2(x2 - rx, y2 - ry);
//         
// //         std::cout << "x,y: " << int_point1.x << ", " << int_point1.y << std::endl;
// //         std::cout << "x,y: " << int_point2.x << ", " << int_point2.y << std::endl;
//         
// //         Determine the correct intersection point based on the angle of view to a known matrix
// //         In order to determine to which side is one matrix relatively to the other we calculate the vector product between the vectors from the intersection point to each matrix position that will give the signal of the angle
//         
//         cv::Point vetor_i1_m1 = matrix_coordinates[first] - int_point1;
//         cv::Point vetor_i1_m2 = matrix_coordinates[second] - int_point1;
//         int angle_direction_i1 = vetor_i1_m1.cross(vetor_i1_m2)/abs(vetor_i1_m1.cross(vetor_i1_m2));
//         
// //         std::cout << "angle 1: " << angle_direction_i1 << std::endl; 
//         
//         cv::Point vetor_i2_m1 = matrix_coordinates[first] - int_point2;
//         cv::Point vetor_i2_m2 = matrix_coordinates[second] - int_point2;
//         int angle_direction_i2 = vetor_i2_m1.cross(vetor_i2_m2)/abs(vetor_i2_m1.cross(vetor_i2_m2));
//         
// //         std::cout << "angle 2: " << angle_direction_i2 << std::endl;
//         
//         int real_angle_direction = (angle[second] - angle[first])/(abs(angle[second] - angle[first]));
//         
// //         std::cout << "real angle: " << real_angle_direction << std::endl;
//         
//         GlobalPosition::Ptr my_position_ptr(new GlobalPosition);
//         if ( real_angle_direction == angle_direction_i1 )
//         {
//             my_position_ptr->x = int_point1.x;
//             my_position_ptr->y = int_point1.y;
//         }else if ( real_angle_direction == angle_direction_i2 )
//         {
//             my_position_ptr->x = int_point2.x;
//             my_position_ptr->y = int_point2.y;
//         }
//         
// //         In order to get the global orientation we need to calculate the angle from our pos to 1 of the matrices and then subtract the measured angle
// //         Angle from our position to the first matrix
//         double dx_ = matrix_coordinates[first].x - my_position_ptr->x;
//         double dy_ = matrix_coordinates[first].y - my_position_ptr->y;
//         
//         double int_mat1_angle = atan2( dy_,dx_ );
//         
//         my_position_ptr->phi = int_mat1_angle - (angle[first] * M_PI/180 );
//         
// //         std::cout << "x,y,phi: " << my_position_ptr->x << ", " <<my_position_ptr->y << ", " << my_position_ptr->phi << std::endl;
//         return my_position_ptr;
//         
// //         std::cout << "angulo: " << my_position_ptr.phi * 180 / M_PI << std::endl;
//         
//     }
    
};


class DataMatrixPosition
{
  ros::NodeHandle nh_;
  ros::Subscriber datamatrix_sub;
  ros::Subscriber camera_info_sub;
  
//   Image transport handler and subscriber
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;  
  
//   Images Vector
  std::list<cv_bridge::CvImagePtr> frame_list;
//   Incoming message types
  datamatrix_detection::DatamatrixMsg datamatrix_msg;
  sensor_msgs::CameraInfo cam_info;
  
  
//   Global variables
  cv::Mat plant;
  cv::Mat plant_img;
  cv::Point image_center;
  double plant_width;             //  [cm] -> same as the markers position
  double matrix_real_height;      //   [19.2 cm]
  double focal_length;
  
  std::vector<double> distance;
  std::vector<double> alpha;
  std::vector<cv::Point2d> matrix_coordinates;
//   std::vector<double> beta;
  
  typedef std::list<std::pair<double,GlobalPosition::Ptr> > DataList;
  
  std::vector<GlobalPosition::Ptr> my_positions;
  DataList position_data;
  
//   LASER STUFF and data comparison
  ros::Subscriber laser_scan_sub;
  std::list<sensor_msgs::LaserScan> laser_list;
  std::vector<double> laser_distances;
  
  std::vector<GlobalPosition::Ptr> laser_pos_vector;
  
  std::vector<std::vector<double>> total_camera_distances;
  std::vector<std::vector<double>> total_laser_distances;
  
  std::vector<double> elapsed_time;
  
//   Static situation variables
  std::vector<GlobalPosition::Ptr> same_pos_mean;
  std::vector<GlobalPosition::Ptr> same_pos_mean_laser;
  
//   measurement variables
  ros::Time start_time;
  bool time;
  
    public:    
    DataMatrixPosition(const std::string& path, const double& mat_height, const double& width)
//         :nh_(nh),
        :it_(nh_)
    {
        time = false;
        plant = cv::imread(path, CV_LOAD_IMAGE_COLOR);
        matrix_real_height = mat_height;    // [m] -> same as the distance
        plant_width = width;  //  [cm] -> same as the markers position
        
        SetupMessaging();
        
    }
    
    ~DataMatrixPosition()
    {
        GlobalPosition::Ptr mean_position_ptr = meanPosition(same_pos_mean_laser);
        laser_pos_vector.push_back(mean_position_ptr);
        
        GlobalPosition::Ptr c_mean_position_ptr = meanPosition(same_pos_mean);
        my_positions.push_back(c_mean_position_ptr);
        
        ros::Time end_time = datamatrix_msg.header.stamp;
        int total_pos = 0;
        
        std::string nome("/home/luis/Documentos/Dissertação/Dados/planta.png");
        cv::imwrite(nome, plant);
        
        std::ofstream position_error;
        position_error.open ("/home/luis/Documentos/Dissertação/Dados/position_error.txt", std::ios::out);
        while( laser_pos_vector.size() > 0 )
        {
            
//             double position_distance = sqrt(pow((my_positions.back()->x - laser_pos_vector.back()->x),2) + pow((my_positions.back()->y - laser_pos_vector.back()->y),2));
            position_error << sqrt(pow((my_positions.back()->x - laser_pos_vector.back()->x),2) + pow((my_positions.back()->y - laser_pos_vector.back()->y),2)) << ", ";
            my_positions.pop_back();
            laser_pos_vector.pop_back();
            total_pos++;
        }
        position_error.close();
        
        std::ofstream distance_error;
        distance_error.open ("/home/luis/Documentos/Dissertação/Dados/distance_error.txt", std::ios::out);
        while( total_laser_distances.size() > 0)
        {
            while(total_laser_distances.back().size() > 0 )
            {
                distance_error << (total_laser_distances.back().back() - total_camera_distances.back().back())*100 << ", ";
                total_laser_distances.back().pop_back();
                total_camera_distances.back().pop_back();
            }
            total_laser_distances.pop_back();
            total_camera_distances.pop_back();
        }
        distance_error.close();
        
        double sum_time=0, max_time = 0, min_time = 100;
        
        for( int i = 0 ; i<elapsed_time.size();i++)
        {
            sum_time = sum_time + elapsed_time[i];
            if (elapsed_time[i] < min_time)
                min_time = elapsed_time[i];
            if (elapsed_time[i] > max_time)
                max_time = elapsed_time[i];
        }
        
        
        std::cout << "max time: " << max_time*1000 << " min time: " << min_time*1000 << " mean time: " << sum_time*1000/elapsed_time.size() << std::endl;
        std::cout << "pos: " << total_pos << std::endl;
        std::cout << "detection per second: " << total_pos / (end_time-start_time).toSec() << std::endl;
    }
    
    void SetupMessaging()
    {
        // Subscribe to input video feed
        image_sub_ = it_.subscribe("usb_cam_reader/image_rect_color", 1,&DataMatrixPosition::imageCallback, this);
        
        camera_info_sub = nh_.subscribe("usb_cam_reader/camera_info", 1, &DataMatrixPosition::CameraInfoCallback, this);
        datamatrix_sub = nh_.subscribe("datamatrix_detection/datamatrix_msg", 1,&DataMatrixPosition::DataMatrixReceiveCallback, this);
        
        
        laser_scan_sub = nh_.subscribe("laser_1/scan",1,&DataMatrixPosition::LaserScanCallback,this);
    }
    
//     Laser scan callback
    void LaserScanCallback(const sensor_msgs::LaserScan& laser_msg)
    {
        laser_list.push_back(laser_msg);
        if ( laser_list.size() > 100 ) laser_list.pop_front();
    }
    
    
    void CameraInfoCallback(const sensor_msgs::CameraInfo& cam_info_msg)
    {
    //       Runs once!!!!
        cam_info = cam_info_msg;
        std::cout << cam_info_msg << std::endl;
            
    //           Center of the image
        image_center.x = cam_info.width/2;
        image_center.y = cam_info.height/2;
        focal_length = (cam_info.P[0] + cam_info.P[5])/2;
        
    //     Shuts the subscriber so the callback is only called once
        camera_info_sub.shutdown();
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
        cv::Mat rot_img;
        cv::Mat rot_mat = getRotationMatrix2D(cv::Point(cv_ptr->image.cols/2, cv_ptr->image.rows/2), 180, 1);
        cv::warpAffine(cv_ptr->image, rot_img, rot_mat, cv_ptr->image.size());
        cv_ptr->image = rot_img;
        
        frame_list.push_back(cv_ptr);
    //     Necessary for memory management, otherwise the computer runs out of memory when not detecting datamatrix
        if (frame_list.size() > 200 ) frame_list.pop_front();
        
    }
    
    void DataMatrixReceiveCallback(const datamatrix_detection::DatamatrixMsg& msg_data)
    {
        ros::Time start = ros::Time::now();
        
        datamatrix_msg = msg_data;
        static int i = 0;
//         std::cout << "header: " << msg_data.header << std::endl;
        for ( std::list<cv_bridge::CvImagePtr>::iterator it = frame_list.begin() ; it != frame_list.end(); it++)
        {
            cv_bridge::CvImagePtr img_ptr = *it;
            if (datamatrix_msg.header.stamp == img_ptr->header.stamp)
            {
                GetMatrixData();
//                 ImageDrawing(img_ptr->image);
                GetGlobalPositioning();
                GetLaserPositioning();
//                 std::cout<<__FUNCTION__<<std::endl;
                GlobalPositioningView();
//                 std::cout<<__FUNCTION__<<std::endl << std::endl;
                
    //               std::cout << "output " << i++ << ": " << datamatrix_msg.code << std::endl;
//                 cv::imshow("Imagem1",img_ptr->image);
//                 cv::imshow("Planta",plant_img);
//                 cv::waitKey(2);
                frame_list.erase(frame_list.begin(),it);
                
//                 distance.clear();
//                 alpha.clear();
//                 matrix_coordinates.clear();
//                 datamatrix_msg.decoded_matrices.clear();
                break;
            }
        }


    distance.clear();
    alpha.clear();
    matrix_coordinates.clear();
    datamatrix_msg.decoded_matrices.clear();
    
    laser_distances.clear();
    
    elapsed_time.push_back( (ros::Time::now() - start).toSec() );
    
    }
    
    
    void GetLaserPositioning()
    {
        if ( datamatrix_msg.decoded_matrices.size() >= 2 && alpha.size() >= 2 && laser_list.size() > 0)
        {
            double prev_elapsed_time = 1000;
            sensor_msgs::LaserScan laser_correct_data;
            
            double min = 1e6;
            
            std::list<sensor_msgs::LaserScan>::iterator it_min;
            
            for(auto it = laser_list.begin();it!=laser_list.end();it++)
            {
                double diff = fabs((datamatrix_msg.header.stamp - it->header.stamp).toSec());
                if(diff<min)
                {
                    min = diff;
                    it_min = it;
                }
            }
            
            //                 fazer e usar a classe;
            if (!time)
            {
                start_time = datamatrix_msg.header.stamp;
                time = true;
            }
//                 std::cout << laser_correct_data.angle_min << std::endl;
                
            LaserPosition laser_pos(alpha, *it_min);
            laser_distances = laser_pos.GetLaserDistance();
            
            total_laser_distances.push_back(laser_distances);
            total_camera_distances.push_back(distance);
//                     for (int u = 0; u < laser_distances.size(); u++)
//                         std::cout << "laser distance: " << laser_distances[u] << std::endl;
//                     std::cout << std::endl;
            static ros::Time prev_laser_time;
            GlobalPosition::Ptr my_laser_position_ptr = GetCorrectPosition(laser_distances, same_pos_mean_laser, prev_laser_time);
            prev_laser_time = datamatrix_msg.header.stamp;
            
            laser_pos_vector.push_back(my_laser_position_ptr);
            laser_list.erase(laser_list.begin(),it_min);
            
            
            
//                     std::cout<<__FUNCTION__<<std::endl;
            std::cout << "camera pos: " << my_positions.back()->x << ", " << my_positions.back()->y << ", " << my_positions.back()->phi << std::endl;
            std::cout << "laser pos: " << my_laser_position_ptr->x << ", " << my_laser_position_ptr->y << ", " << my_laser_position_ptr->phi << std::endl << std::endl;
//                     std::cout << "distances: " << laser_distances.size() << std::endl;
//                     std::cout<<__FUNCTION__<<std::endl << std::endl;

        }
    }

    
    GlobalPosition::Ptr GetCorrectPosition(std::vector<double>& distance_, std::vector<GlobalPosition::Ptr>& mean_vector_, ros::Time prev_time)
    {
        GlobalPosition::Ptr current_pos_ptr = GetPosition(distance_ );
        return current_pos_ptr;
        
//         static ros::Time prev_time;
        if ( (datamatrix_msg.header.stamp - prev_time).toSec() < 1 )
        {
            if (!std::isnan(current_pos_ptr->x))
                mean_vector_.push_back(current_pos_ptr);
        //                 std::cout << "passou" << std::endl;
        }
        else
        {
            GlobalPosition::Ptr mean_position_ptr = meanPosition(mean_vector_);
            mean_vector_.clear();
            if ( !std::isnan(current_pos_ptr->x) )
            {
                mean_vector_.push_back(current_pos_ptr);
                return mean_position_ptr;
            }
        }
//         prev_time = datamatrix_msg.header.stamp;
        
        
        double return_nan = std::numeric_limits<double>::quiet_NaN();
        return GlobalPosition::Ptr (new GlobalPosition(return_nan,return_nan,return_nan));
    }
    
    
    
    
    
    
    void GetMatrixData()
    {
        
        for (auto it = datamatrix_msg.decoded_matrices.begin(); it!=datamatrix_msg.decoded_matrices.end();)
        {
//             Gets the matrix coordinates
            if ( it->code.size() % 2 == 0 )
            {
                std::string string_x = it->code.substr(0, it->code.size()/2);
                std::string string_y = it->code.substr(it->code.size()/2);
                
                cv::Point2d matrix_coord(atoi(string_x.c_str()), atoi(string_y.c_str()) - 50 );
                matrix_coordinates.push_back(matrix_coord);
                
                GetMatrixGeometry(it);
                GetMatrixDistance(it);
                GetMatrixAngle(it);
//                 CalculateMatrixGeometry(i);
                
                it++;
            }
            else
            {
                it = datamatrix_msg.decoded_matrices.erase(it);
                
            }
        }
    }
    
    void GetMatrixGeometry(std::vector<datamatrix_detection::DatamatrixData>::iterator it)
    {
      it->matrix_center.x = (it->right.x + it->top.x)/2;
      it->matrix_center.y = (it->right.y + it->top.y)/2;
      it->matrix_center.z = 0;
      
      cv::Point bottom_mean_point;
      if ( it->right.x - it->matrix_center.x < it->right.x - it->bottom.x )
      {
          bottom_mean_point.x = (it->right.x + it->bottom.x)/2;
          bottom_mean_point.y = (it->right.y + it->bottom.y)/2;
      }else{
        bottom_mean_point.x = (it->right.x + it->left.x)/2;
        bottom_mean_point.y = (it->right.y + it->left.y)/2;
      }
      cv::Point left_mean_point;
      left_mean_point.x = (it->top.x + it->bottom.x)/2;
      left_mean_point.y = (it->top.y + it->bottom.y)/2;
      
//       Calculates the matrix height
      it->matrix_height = sqrt((bottom_mean_point.x - it->matrix_center.x)*(bottom_mean_point.x - it->matrix_center.x) + (bottom_mean_point.y - it->matrix_center.y)*(bottom_mean_point.y - it->matrix_center.y)) * 2;
      
//       Calculates the matrix width
      it->matrix_width = sqrt((left_mean_point.x - it->matrix_center.x)*(left_mean_point.x - it->matrix_center.x) + (left_mean_point.y - it->matrix_center.y)*(left_mean_point.y - it->matrix_center.y)) * 2;
    }
    
    void GetMatrixDistance(std::vector<datamatrix_detection::DatamatrixData>::iterator it)
    {
    //     Gets the distance to the center of the matrix
        if ( cam_info.P[0] != 0 )
        {
            distance.push_back(matrix_real_height * focal_length/it->matrix_height);
//             std::cout << "camera distance: " << distance.back() << std::endl;
//             total_camera_distances.push_back(distance);
        }
    }

    void GetMatrixAngle(std::vector<datamatrix_detection::DatamatrixData>::iterator it)
    {
    //     Gets the angle to the center of the matrix
        if ( cam_info.P[0] != 0)
        {
            double center_dist_pix = image_center.x - it->matrix_center.x;
            alpha.push_back(( center_dist_pix/focal_length) * 180/M_PI);
        }
    }

    void ImageDrawing(cv::Mat& cv_img)
    {
        if ( alpha.size() == datamatrix_msg.decoded_matrices.size() )
        {
            for ( uint i = 0 ; i < datamatrix_msg.decoded_matrices.size(); i++)
            {
        //         drawing the 4 direct message points
    //             cv::circle(cv_img, cv::Point (datamatrix_msg.decoded_matrices[i].left.x, datamatrix_msg.decoded_matrices[i].left.y), 10, CV_RGB(255,0,0));
                cv::circle(cv_img, cv::Point (datamatrix_msg.decoded_matrices[i].right.x, datamatrix_msg.decoded_matrices[i].right.y), 10, CV_RGB(255,0,0));
                cv::circle(cv_img, cv::Point (datamatrix_msg.decoded_matrices[i].top.x, datamatrix_msg.decoded_matrices[i].top.y), 10, CV_RGB(255,0,0));
                cv::circle(cv_img, cv::Point (datamatrix_msg.decoded_matrices[i].bottom.x, datamatrix_msg.decoded_matrices[i].bottom.y), 10, CV_RGB(255,0,0));    
                cv::circle(cv_img, cv::Point (datamatrix_msg.decoded_matrices[i].matrix_center.x, datamatrix_msg.decoded_matrices[i].matrix_center.y), 10, CV_RGB(255,0,0));
                
                cv::Point top_right;
                
            //     Estimate the top_right point
                double left_angle;
                double top_left_distance = sqrt((datamatrix_msg.decoded_matrices[i].top.x - datamatrix_msg.decoded_matrices[i].left.x)*(datamatrix_msg.decoded_matrices[i].top.x - datamatrix_msg.decoded_matrices[i].left.x) + (datamatrix_msg.decoded_matrices[i].top.y - datamatrix_msg.decoded_matrices[i].left.y)*(datamatrix_msg.decoded_matrices[i].top.y - datamatrix_msg.decoded_matrices[i].left.y));
                
            //     if( top_left_distance > 20)
                    left_angle = atan2(datamatrix_msg.decoded_matrices[i].top.y - datamatrix_msg.decoded_matrices[i].left.y, datamatrix_msg.decoded_matrices[i].top.x - datamatrix_msg.decoded_matrices[i].left.x);
            //     else
            //         left_angle = atan2(datamatrix_msg.top.y - datamatrix_msg.bottom.y,datamatrix_msg.top.x - datamatrix_msg.bottom.x);
                
                top_right.x = datamatrix_msg.decoded_matrices[i].right.x + round(datamatrix_msg.decoded_matrices[i].matrix_height*cos(left_angle));
                top_right.y = datamatrix_msg.decoded_matrices[i].right.y + round(datamatrix_msg.decoded_matrices[i].matrix_height*sin(left_angle));
                
                cv::circle(cv_img, image_center, 10, CV_RGB(0,255,0));
                cv::circle(cv_img, top_right, 10, CV_RGB(0,255,0));
                
            //     Chosing the which point to connect the line
                if ( datamatrix_msg.decoded_matrices[i].top.y > datamatrix_msg.decoded_matrices[i].right.y && top_left_distance > 30)
                {
                    cv::line(cv_img, cv::Point (datamatrix_msg.decoded_matrices[i].top.x, datamatrix_msg.decoded_matrices[i].top.y), cv::Point (datamatrix_msg.decoded_matrices[i].left.x, datamatrix_msg.decoded_matrices[i].left.y), cv::Scalar(0,255,0));
                    cv::line(cv_img, cv::Point (datamatrix_msg.decoded_matrices[i].left.x, datamatrix_msg.decoded_matrices[i].left.y), cv::Point (datamatrix_msg.decoded_matrices[i].right.x, datamatrix_msg.decoded_matrices[i].right.y), cv::Scalar(0,255,0));        
                }else{
                    cv::line(cv_img, cv::Point (datamatrix_msg.decoded_matrices[i].top.x, datamatrix_msg.decoded_matrices[i].top.y), cv::Point (datamatrix_msg.decoded_matrices[i].bottom.x, datamatrix_msg.decoded_matrices[i].bottom.y), cv::Scalar(0,255,0));
                    cv::line(cv_img, cv::Point (datamatrix_msg.decoded_matrices[i].bottom.x, datamatrix_msg.decoded_matrices[i].bottom.y), cv::Point (datamatrix_msg.decoded_matrices[i].right.x, datamatrix_msg.decoded_matrices[i].right.y), cv::Scalar(0,255,0));
                }
                cv::line(cv_img, cv::Point (datamatrix_msg.decoded_matrices[i].right.x, datamatrix_msg.decoded_matrices[i].right.y), top_right, cv::Scalar(0,255,0));
                cv::line(cv_img, top_right, cv::Point (datamatrix_msg.decoded_matrices[i].top.x, datamatrix_msg.decoded_matrices[i].top.y), cv::Scalar(0,255,0));
                cv::line(cv_img, cv::Point (1,cv_img.rows/2), cv::Point (cv_img.cols,cv_img.rows/2), cv::Scalar(255,0,0));
                
            //     Matrix code display on image
                std::string leitura = "Leitura: " + datamatrix_msg.decoded_matrices[i].code;
                cv::putText(cv_img, leitura, cv::Point (datamatrix_msg.decoded_matrices[i].top.x, datamatrix_msg.decoded_matrices[i].top.y - 15), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0,255,0), 2, 8);
                
            //     Matrix distance display
                boost::format dist_str("Distancia: %1$.3f [m]");    //  Format the output string
                dist_str % distance[i];                                //  Assign distance to the output string
                cv::putText(cv_img, dist_str.str(), cv::Point (datamatrix_msg.decoded_matrices[i].right.x + 15, datamatrix_msg.decoded_matrices[i].right.y), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0,255,0), 2, 8);
                
            //     Matrix angle display
                boost::format ang_str("Angulo: %1$.3f [deg]");    //  Format the output string
                ang_str % alpha[i];                                //  Assign alpha to the output string
                cv::putText(cv_img, ang_str.str(), cv::Point (datamatrix_msg.decoded_matrices[i].bottom.x, datamatrix_msg.decoded_matrices[i].bottom.y + 20), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0,255,0), 2, 8);
            }
        }
        cv::imshow("Imagem1",cv_img);
    }
    

    void GetGlobalPositioning()
    {
        if ( datamatrix_msg.decoded_matrices.size() >= 2 && alpha.size() >= 2)
        {
            static ros::Time prev_cam_time;
            GlobalPosition::Ptr my_position_ptr = GetCorrectPosition(distance, same_pos_mean, prev_cam_time);
            prev_cam_time = datamatrix_msg.header.stamp;
//             GlobalPosition::Ptr my_estimated_position_ptr = GetPosition(distance);
//             
//             static ros::Time prev_time;
//             if ( (datamatrix_msg.header.stamp - prev_time).toSec() < 1 )
//             {
//                 if (!std::isnan(my_estimated_position_ptr->x))
//                     same_pos_mean.push_back(my_estimated_position_ptr);
//             }
//             else
//             {
//                 same_pos_mean.clear();
//                 if (!std::isnan(my_estimated_position_ptr->x))
//                     same_pos_mean.push_back(my_estimated_position_ptr);
//             }
//             prev_time = datamatrix_msg.header.stamp;
//             GlobalPosition::Ptr my_position_ptr = meanPosition(same_pos_mean);
            
            
//             Doing the mean of the previous x positions
//             static ros::Time prev_time;
//             if ( prev_time.sec != 0 && my_estimated_position_ptr->x >= 0)
//                 position_data.push_front(std::make_pair((datamatrix_msg.header.stamp - prev_time).toSec(), my_estimated_position_ptr));
//             if ( position_data.size() > 10 )
//                 position_data.pop_back();
//             prev_time = datamatrix_msg.header.stamp;
// //             Simple moving average
//             GlobalPosition::Ptr my_position_ptr = meanPosition(position_data);
// //             Exponential Moving Average
//             GlobalPosition::Ptr my_position_ptr = EMAPosition(position_data);
// //             weighted moving average
//             GlobalPosition::Ptr my_position_ptr = WMAPosition(position_data);
            
            
//             GlobalPosition::Ptr my_position_ptr = my_estimated_position_ptr;
//             std::cout << "x,y,phi " << my_position_ptr->x << ", " << my_position_ptr->y << ", " << my_position_ptr->phi << std::endl;
            my_positions.push_back(my_position_ptr);
        }
    }
    
    void GlobalPositioningView()
    {
        plant_img = plant.clone();
            
        double plant_cm_pix_ratio = plant_img.cols/plant_width;
        
        if ( datamatrix_msg.decoded_matrices.size() >= 2 && alpha.size() >= 2)
        {
            GlobalPosition::Ptr my_position = my_positions.back();
//             cv::circle(plant_img, cv::Point( plant_img.cols - (my_position->x * plant_cm_pix_ratio), my_position->y * plant_cm_pix_ratio), 3, CV_RGB(255,0,0));
            cv::circle(plant_img, cv::Point( plant_img.rows*2.3 - (my_position->x * plant_cm_pix_ratio), plant_img.rows*0.7 - (my_position->y * plant_cm_pix_ratio)), 3, CV_RGB(255,0,0));
            
            if ( laser_pos_vector.size() >= 1)
            {
                GlobalPosition::Ptr laser_position = laser_pos_vector.back();
//                 cv::circle(plant_img, cv::Point( plant_img.cols - (laser_position->x * plant_cm_pix_ratio), laser_position->y * plant_cm_pix_ratio), 3, CV_RGB(0,255,0));
                cv::circle(plant_img, cv::Point( plant_img.rows*2.3 - (laser_position->x * plant_cm_pix_ratio), plant_img.rows*0.7 - (laser_position->y * plant_cm_pix_ratio)), 3, CV_RGB(0,255,0));
            }            
            plant = plant_img.clone();
            
//             int arrow_size = 500;
//             cv::Point arrow_end(plant_img.cols - (my_position->x + arrow_size * cos(my_position->phi)) * plant_cm_pix_ratio, (my_position->y + arrow_size * sin(my_position->phi)) * plant_cm_pix_ratio);
//             cv::line(plant_img, cv::Point( plant_img.cols - (my_position->x * plant_cm_pix_ratio), my_position->y * plant_cm_pix_ratio), arrow_end, CV_RGB(0,255,255), 3, CV_AA);
//             if ( laser_pos_vector.size() >= 1)
//             {
//                 GlobalPosition::Ptr laser_position = laser_pos_vector.back();
//                 cv::Point arrow_end(plant_img.cols - (laser_position->x + arrow_size * cos(laser_position->phi)) * plant_cm_pix_ratio, (laser_position->y + arrow_size * sin(laser_position->phi)) * plant_cm_pix_ratio);
//                 cv::line(plant_img, cv::Point( plant_img.cols - (laser_position->x * plant_cm_pix_ratio), laser_position->y * plant_cm_pix_ratio), arrow_end, CV_RGB(255,0,255), 3, CV_AA);
//             }
        }
        
//         for ( uint i = 0 ; i < datamatrix_msg.decoded_matrices.size(); i++)
//         {                
//             cv::Point matrix_pos(plant_img.cols - (matrix_coordinates[i].x * plant_cm_pix_ratio), matrix_coordinates[i].y * plant_cm_pix_ratio);
//             cv::circle(plant_img, matrix_pos, distance[i] * 100 * plant_cm_pix_ratio, CV_RGB(255,0,0));
// //             laser_pos_vector.size() >= 1
// //             std::cout<<__FUNCTION__<<std::endl;
//             if ( laser_distances.size() >= 1)
//                 if ( !std::isnan(laser_distances[i]))
//                     cv::circle(plant_img, matrix_pos, laser_distances[i] * 100 * plant_cm_pix_ratio, CV_RGB(0,255,0));
// //             std::cout<<__FUNCTION__<<std::endl << std::endl;
//             
//             cv::circle(plant_img, matrix_pos, 5, CV_RGB(255,0,0));
//         }
//         
//         cv::imshow("Planta",plant_img);
        cv::waitKey(2);
    }
    
    GlobalPosition::Ptr meanPosition(const std::vector<GlobalPosition::Ptr>& data)
    {
        GlobalPosition::Ptr mean_pos(new GlobalPosition);
        
        for(auto it = data.begin();it!=data.end();it++)
            mean_pos = mean_pos + *it;
        
        return mean_pos / data.size();
    }
    
    GlobalPosition::Ptr EMAPosition(const DataList& data)
    {
//         cout<<__FUNCTION__<<endl;
//         The greater the alpha value is the most responsive the algorithm will be
        double denominator, alpha = 0.2;
        GlobalPosition::Ptr ema_pos (new GlobalPosition);
        int i = 0;
        for ( auto it = data.begin();it != data.end();it++)
        {
            double mult = pow((1-alpha),i++);
            ema_pos = ema_pos + it->second * mult;
            denominator = denominator + mult;
        }
        
//         std::cout << "x, y, phi: " << ema_pos->x/denominator << ", " << ema_pos->y/denominator << ", " << ema_pos->phi/denominator << std::endl;
        
//         cout<<"out of "<<__FUNCTION__<<endl;
        return ema_pos / denominator;
    }
    
    GlobalPosition::Ptr WMAPosition(const DataList& data)
    {
        GlobalPosition::Ptr wma_pos (new GlobalPosition);
        double denominator;
        for ( auto it = data.begin();it != data.end();it++)
        {
            wma_pos = wma_pos + it->second * it->first;
            denominator = denominator + it->first;
        }
//         std::cout << "deno: " << denominator << std::endl;
        return wma_pos / denominator;
    }
    
    GlobalPosition::Ptr GetPosition(std::vector<double>& distance_)
    {
        GlobalPosition::Ptr laser_position_ptr;
        if ( distance_.size() >= 2 )
        {
//         std::vector<GlobalPosition::Ptr> my_intermediate_positions;
            int i = 0;
            GlobalPosition::Ptr sum(new GlobalPosition);
            for ( uint j = 0; j < distance_.size() - 1; j++)
            {
                for ( uint k = j + 1; k < distance_.size(); k++ )
                {
                    double matrix_distance = sqrt(pow((matrix_coordinates[j].x - matrix_coordinates[k].x),2) + pow((matrix_coordinates[j].y - matrix_coordinates[k].y),2));
                    double matrix_angle = fabs(alpha[j] - alpha[k]);
                    if( matrix_distance > 100 && matrix_angle > 15)
                    {
                        GlobalPosition::Ptr my_intermediate_position_ptr = CalculateCircleIntersection( j, k, distance_ );
                        if (!std::isnan(my_intermediate_position_ptr->x) && !std::isnan(my_intermediate_position_ptr->y) && !std::isnan(my_intermediate_position_ptr->phi))
                        {
        //                     my_intermediate_positions.push_back(my_intermediate_position_ptr);
                            i++;
                            sum = sum + my_intermediate_position_ptr;
                        }
                    }
                }
            }
            return sum / i;
        }
        else
        {
            double return_nan = std::numeric_limits<double>::quiet_NaN();
            return GlobalPosition::Ptr (new GlobalPosition(return_nan,return_nan,return_nan));
        }
//         return sum / my_intermediate_positions.size();
    }
    
    
    
    
    GlobalPosition::Ptr CalculateCircleIntersection( uint first, uint second, std::vector<double>& distance_)
    {
        
        double a, dx, dy, d, h, rx, ry;
        double x2, y2;

//         My matrices coordinates
        double x0 = matrix_coordinates[first].x;
        double y0 = matrix_coordinates[first].y;
        double r0 = distance_[first]*100;
        
        double x1 = matrix_coordinates[second].x;
        double y1 = matrix_coordinates[second].y;
        double r1 = distance_[second]*100;
        
        /* dx and dy are the vertical and horizontal distances between
        * the circle centers.
        */
        dx = x1 - x0;
        dy = y1 - y0;

        /* Determine the straight-line distance between the centers. */
        //d = sqrt((dy*dy) + (dx*dx));
        d = hypot(dx,dy); // Suggested by Keith Briggs
        
//         /* Check for solvability. */
        if (d > (r0 + r1))
        {
            /* no solution. circles do not intersect. */
            double return_nan = std::numeric_limits<double>::quiet_NaN();
            return GlobalPosition::Ptr (new GlobalPosition(return_nan,return_nan,return_nan));
        }
        if (d < fabs(r0 - r1))
        {
            /* no solution. one circle is contained in the other */
            double return_nan = std::numeric_limits<double>::quiet_NaN();
            return GlobalPosition::Ptr (new GlobalPosition(return_nan,return_nan,return_nan));
        }
        /* 'point 2' is the point where the line through the circle
        * intersection points crosses the line between the circle
        * centers.
        */

        /* Determine the distance from point 0 to point 2. */
        a = ((r0*r0) - (r1*r1) + (d*d)) / (2.0 * d) ;

        /* Determine the coordinates of point 2. */
        x2 = x0 + (dx * a/d);
        y2 = y0 + (dy * a/d);

        /* Determine the distance from point 2 to either of the
        * intersection points.
        */
        h = sqrt((r0*r0) - (a*a));

        /* Now determine the offsets of the intersection points from
        * point 2.
        */
        rx = -dy * (h/d);
        ry = dx * (h/d);

        /* Determine the absolute intersection points.*/
//         Coordinates are in centimeters -----------------------------------------!!!
        cv::Point2d int_point1(x2 + rx, y2 + ry);
        cv::Point2d int_point2(x2 - rx, y2 - ry);
        
//         std::cout << "x,y: " << int_point1.x << ", " << int_point1.y << std::endl;
//         std::cout << "x,y: " << int_point2.x << ", " << int_point2.y << std::endl;
        
//         Determine the correct intersection point based on the angle of view to a known matrix
//         In order to determine to which side is one matrix relatively to the other we calculate the vector product between the vectors from the intersection point to each matrix position that will give the signal of the angle
        
        cv::Point vetor_i1_m1 = matrix_coordinates[first] - int_point1;
        cv::Point vetor_i1_m2 = matrix_coordinates[second] - int_point1;
        int angle_direction_i1 = vetor_i1_m1.cross(vetor_i1_m2)/abs(vetor_i1_m1.cross(vetor_i1_m2));
        
//         std::cout << "angle 1: " << angle_direction_i1 << std::endl; 
        
        cv::Point vetor_i2_m1 = matrix_coordinates[first] - int_point2;
        cv::Point vetor_i2_m2 = matrix_coordinates[second] - int_point2;
        int angle_direction_i2 = vetor_i2_m1.cross(vetor_i2_m2)/abs(vetor_i2_m1.cross(vetor_i2_m2));
        
//         std::cout << "angle 2: " << angle_direction_i2 << std::endl;
        
        int real_angle_direction = (alpha[second] - alpha[first])/(abs(alpha[second] - alpha[first]));
        
//         std::cout << "real angle: " << real_angle_direction << std::endl;
        
        GlobalPosition::Ptr my_position_ptr(new GlobalPosition);
        if ( real_angle_direction*-1 == angle_direction_i1 )
        {
            my_position_ptr->x = int_point1.x;
            my_position_ptr->y = int_point1.y;
        }else if ( real_angle_direction*-1 == angle_direction_i2 )
        {
            my_position_ptr->x = int_point2.x;
            my_position_ptr->y = int_point2.y;
        }
        
//         In order to get the global orientation we need to calculate the angle from our pos to 1 of the matrices and then subtract the measured angle
//         Angle from our position to the first matrix
        double dx_ = matrix_coordinates[first].x - my_position_ptr->x;
        double dy_ = matrix_coordinates[first].y - my_position_ptr->y;
        
        double int_mat1_angle = atan2( dy_,dx_ );
        
        my_position_ptr->phi = int_mat1_angle + (alpha[first] * M_PI/180 );
        
//         std::cout << "x,y,phi: " << my_position_ptr->x << ", " <<my_position_ptr->y << ", " << my_position_ptr->phi << std::endl;
        return my_position_ptr;
        
//         std::cout << "angulo: " << my_position_ptr.phi * 180 / M_PI << std::endl;
        
    }
    

};

int main(int argc, char** argv)
{
    
//     cv::Point2d p1(1,2.2);
//     double timestamp1 = 123545656;
//     
//     cv::Point2d p2(123,2.42);
//     double timestamp2 = 132565465;
//     
//     std::vector<std::pair<cv::Point2d,double> > data;
//     
//     std::pair<cv::Point2d,double> par;
//     
//     par.first = p1;
//     
//     data.push_back(std::make_pair(p1,timestamp1));
//     data.push_back(std::make_pair(p2,timestamp2));
//     
//     for(uint i=0;i<data.size();i++)
//     {
//         std::cout<<"data["<<i<<"] = "<<data[i].first;
//         std::cout<<", t = "<<data[i].second<<std::endl;
//         
//     }
//     
//     return 0;
    
//     cout<<"Init"<<endl;
//     
//     PointTheta p1(0.2,1.5,8);
//     PointTheta p2(0.3,2,4);
//     PointTheta p3 = p1 +p2;
//     
//     double c = p1.cross(p2);
//     double d = p1.dot(p2);
//     
//     
//     cout<<"P1: "<<p1<<", "<<p1.phi<<endl;
//     cout<<"P2: "<<p2<<", "<<p2.phi<<endl;
//     cout<<"P3: "<<p3<<", "<<p3.phi<<endl;
//     
//     cout<<"Cross: "<<p1.cross(p2)<<endl;
//     
//     return 0;
  ros::init(argc, argv, "datamatrix_calculations");
  ros::NodeHandle nh("~");
  
  std::string path;
  nh.param<std::string>("plant_path",path,"");
  
  double real_height;
  nh.param<double>("matrix_height",real_height,0);
  
  double plant_width;
  nh.param<double>("plant_width",plant_width,0);
  
  DataMatrixPosition datamatrix_pos(path, real_height, plant_width);
  ros::spin();
  return 0;
}