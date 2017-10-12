#include <ros/ros.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

using namespace std;
using namespace cv;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_test");

    Mat input = imread("/home/asus/workingcopies/lar4/src/perception/pedestrians/multimodal_pedestrian_detect/images/outside_test/frame0002.jpg", CV_LOAD_IMAGE_COLOR);
//     
    Mat redOnly;
    
//     inRange(input, Scalar(75, 90, 0), Scalar(120,170,15), redOnly); // placa verde plastica
    inRange(input, Scalar(150, 150, 150), Scalar(255,255,255), redOnly); // placa branca
    
    //Find the contours. Use the contourOutput Mat so the original image doesn't get overwritten
    vector<std::vector<cv::Point> > contours;
    Mat contourOutput = redOnly.clone();
    findContours( contourOutput, contours, CV_RETR_TREE/*EXTERNAL*/, CV_CHAIN_APPROX_NONE );
    
    vector<vector<Point> > contours_poly( contours.size() );
    
    Mat drawing = Mat::zeros( redOnly.size(), CV_8UC3 );
    
    int max_size_triangle=0;
    for(size_t i = 0; i < contours.size(); i++)
    {
        approxPolyDP(Mat(contours[i]), contours_poly[i], 0.01*arcLength(Mat(contours[i]),true), true);
//         cout << "number of triangles: " << contours_poly.size() << endl;

//         cout << "size_t: " << i << endl;
//         drawContours(drawing, contours_poly, i, Scalar(0, 0, 255),CV_FILLED,8); // fill RED 
        if (contours_poly[i].size() == 4) //from all object, only triangles are choosen
        {
            cout << "arclength: " << arcLength(Mat(contours[i]),true) << endl;
            cout << "size_poly: " << contours_poly[i].size() << endl;
            cout << "size: " << contours[i].size() << endl;
//             drawContours(drawing, contours_poly, i, Scalar(0, 0, 255),CV_FILLED,8); // fill RED 
            if(i > 0)
                if( arcLength(Mat(contours[i]),true) > arcLength(Mat(contours[max_size_triangle]),true)) //choose bigger triangle
                {
                    max_size_triangle = i;
                }
        }
    }
    
    double max_width = contours_poly[max_size_triangle][3].x - contours_poly[max_size_triangle][0].x;
    double min_width = contours_poly[max_size_triangle][2].x - contours_poly[max_size_triangle][1].x;
    double height = contours_poly[max_size_triangle][1].y - contours_poly[max_size_triangle][0].y;
    
    cout << "triangle pixel size: " << max_size_triangle << "  " << contours_poly[max_size_triangle] << "  " << arcLength(Mat(contours[max_size_triangle]),true) << endl
         << "max width: " << max_width << endl
         << "min width: " << min_width << endl
         << "height: " << height << endl;
    
    drawContours(drawing, contours_poly, max_size_triangle, Scalar(255, 255, 255),CV_FILLED,8); // fill WHITE  
    

    double object_width_percent_laser_given = 1 - (0.02); // (inside value is the percentage, return by the laser)
    
    int laser_line_int = object_width_percent_laser_given * height;
    double laser_line = object_width_percent_laser_given * height;
    
//     cout << "laser_line, double: " << laser_line << " int: " << laser_line_int << endl;
    
    if(laser_line - laser_line_int >=0.5 )
           laser_line_int++;
    
    int laser_line_position = contours_poly[max_size_triangle][0].y + laser_line_int;
    
    //Grayscale matrix
    Mat grayscaleMat(drawing.size(), CV_8U);

    //Convert BGR to Gray
    cvtColor(drawing, grayscaleMat, CV_BGR2GRAY);
    
    int k=0;
    Point left_point;
    Point right_point;
    
    for(int col=contours_poly[max_size_triangle][0].x; col<contours_poly[max_size_triangle][3].x; col++) 
        if(grayscaleMat.at<uchar>(laser_line_position,col) != 0) 
            if (k==0)
            {
                left_point.y=laser_line_position;
                left_point.x=col;
                k++;
            }else{
                
                right_point.y=laser_line_position;
                right_point.x=col;
            }
            
    
    cout << "laser_line, double: " << laser_line << " int: " << laser_line_int << endl;
    cout << "points, left_point: " << left_point << " right_point: " << right_point << endl;
    
    line(input, left_point, right_point, Scalar(0, 0, 255), 1, 8,0);
    
    imshow("input", input);
//     imshow("redOnly", redOnly);
//     imshow("drawing", drawing);
//     imshow("drawing1", grayscaleMat);
    waitKey();

    // detect squares after filtering...
    ros::spin();
    return 0;
}