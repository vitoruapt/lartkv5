#include <ros/ros.h>

#include <camera_info_manager/camera_info_manager.h>

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <fstream>
#include <iostream>
#include <string>

#include "mtt/TargetList.h"

#include "cmath"
#include <iostream>
#include <vector>

#include <image_transport/image_transport.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv/cvwimage.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "sensor_msgs/LaserScan.h"
#include <sensor_msgs/image_encodings.h>

#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>

#include <fstream>
#include <string>

using namespace std;
using namespace cv;

ros::Subscriber cam_sub;
ros::Subscriber cam_info_sub;
std::string fileStorageYAML;

std::vector<cv::Point2f> Read2DPoints(const std::string& filename);
std::vector<cv::Point3f> Read3DPoints(const std::string& filename);

std::string imagePoints1FileName;
std::string objectPoints1FileName;

vector<double> k_matriz;

void cameraInfo(const sensor_msgs::CameraInfo& cam_info_msg)
{
    double k;
    for (int i=0; i<cam_info_msg.K.size(); i++)
    {
        k = cam_info_msg.K[i];
        k_matriz.push_back(k);
    }     

    cam_info_sub.shutdown(); // the subscriber only runs one time
}

void cameraCalibration(const sensor_msgs::ImageConstPtr& cam_msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    
    try
    {
        cv_ptr = cv_bridge::toCvCopy(cam_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // Read points
    std::vector<cv::Point2f> inputImage1Points = Read2DPoints(imagePoints1FileName);  
    
    std::vector<cv::Point3f> inputObject1Points = Read3DPoints(objectPoints1FileName);
    
    cv::Mat initialCameraMatrix = cvCreateMat(3,3 , CV_64FC1);
    for (int i = 0; i < 3; i++) 
    {
        for (int j = 0; j < 3; j++) 
        {
            initialCameraMatrix.at<double>(i, j) = k_matriz[3*i+j];
        }
    }
    
    cv::Mat distCoeffs = cvCreateMat(1,5 , CV_64FC1);
    
    distCoeffs.at<double>(0, 0) = 0.0;
    distCoeffs.at<double>(0, 1) = 0.0;
    distCoeffs.at<double>(0, 2) = 0.0;
    distCoeffs.at<double>(0, 3) = 0.0;
    distCoeffs.at<double>(0, 4) = 0.0;  
    
    cv::Mat cameraMatrix;
    cv::Mat rvecs;
    cv::Mat tvecs;
    
    cameraMatrix = initialCameraMatrix;
    
    cv::solvePnP(inputObject1Points, inputImage1Points, cameraMatrix, distCoeffs, rvecs, tvecs);
    
    cv::Mat rvecs_3x3;
    Rodrigues(rvecs, rvecs_3x3);

//     // Open yaml file
    FileStorage fs(fileStorageYAML, FileStorage::WRITE); // xb3

    Mat R = (Mat_<double>(3,3) << rvecs.at<double>(0,0), rvecs.at<double>(0,1), rvecs.at<double>(0,2), rvecs.at<double>(1,0), rvecs.at<double>(1,1), rvecs.at<double>(1,2), rvecs.at<double>(2,0), rvecs.at<double>(2,1), rvecs.at<double>(2,2));   
    
    Mat T = (Mat_<double>(1,3) << tvecs.at<double>(0,0), tvecs.at<double>(0,1), tvecs.at<double>(0,2));
    
    fs << "image_width" << cv_ptr->image.cols << "image_height" << cv_ptr->image.rows << "camera_name" << "right" <<  "rectification_matrix" << R << "translation_matrix" << T << "intrinsic_xb3_matrix" << "insert a rows: 3; cols: 3; dt: d; data: (camera proc image intrinsic matrix)";
    
    fs.release();
    cout << "calibration ended" << endl;
    
    cam_sub.shutdown(); // the subscriber only runs one time

}

int main( int argc, char* argv[])
{
    ros::init(argc, argv, "sensor_fusion");
    
    ros::NodeHandle nh("~"); 
    
    cam_sub = nh.subscribe("/xb3/right/new_info/image_rect_color", 1, cameraCalibration);
    cam_info_sub = nh.subscribe("/xb3/right/new_info/camera_info", 1, cameraInfo);
    
    nh.param<std::string>("fileStorageYAML",fileStorageYAML,"");
    
    nh.param<std::string>("imagePoints1FileName",imagePoints1FileName,"");
    
    nh.param<std::string>("objectPoints1FileName",objectPoints1FileName,"");

    // Output arguments
    std::cout << "fileStorageYAML: " << fileStorageYAML << std::endl;
    
    std::cout << "imagePointsFileName: " << imagePoints1FileName << std::endl;
    
    std::cout << "objectPointsFileName: " << objectPoints1FileName << std::endl;

    ros::spin();
    return 0;
}


std::vector<cv::Point2f> Read2DPoints(const std::string& filename)
{
    // Read points
    std::ifstream pointsstream(filename.c_str());

    if(pointsstream == NULL)
    {
        std::cout << "Cannot open file " << filename << std::endl;
        exit(-1);
    }

    // Read the point from the first image
    std::string line;
    std::vector<cv::Point2f> points;

    while(getline(pointsstream, line))
    {
        std::stringstream ss(line);
        float x,y;
        ss >> x >> y;
        points.push_back(cv::Point2f(x,y));
    }

    return points;
}


std::vector<cv::Point3f> Read3DPoints(const std::string& filename)
{
    // Read points
    std::ifstream pointsstream(filename.c_str());

    if(pointsstream == NULL)
    {
        std::cout << "Cannot open file " << filename << std::endl;
        exit(-1);
    }

    // Read the point from the first image
    std::string line;
    std::vector<cv::Point3f> points;

    while(getline(pointsstream, line))
    {
        std::stringstream ss(line);
        float x,y,z;
        ss >> x >> y >> z;
        points.push_back(cv::Point3f(x,y,z));
    }

    return points;
}
