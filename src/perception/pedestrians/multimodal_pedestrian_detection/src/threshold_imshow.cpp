#include <ros/ros.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.h>

#include <sensor_msgs/image_encodings.h>

using namespace std;
using namespace cv;
namespace enc = sensor_msgs::image_encodings;

class ImageConverter
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    
    public:
        
        ImageConverter()
        : it_(nh_)
        {   
            ros::NodeHandle nh_local("~");
            
            image_sub_ = it_.subscribe("/xb3/right/new_info/image_rect_color", 1, &ImageConverter::imageCb, this);
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
        
            //Grayscale matrix
            Mat grayscaleMat(input.size(), CV_8UC1);
            
            cvtColor(input, grayscaleMat, CV_BGR2GRAY);
            threshold(grayscaleMat, grayscaleMat, 170, 255, THRESH_BINARY);
            
            imshow("drawing1", grayscaleMat);
            waitKey(2);
            
        }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "threshold_imshow_node");
  
  ImageConverter ic;
  
  cout << "estou aqui" << endl;
  
  ros::spin();
  return 0;
}