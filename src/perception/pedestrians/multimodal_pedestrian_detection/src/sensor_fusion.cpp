#include <ros/ros.h>

#include "peddetect.h"

#include <stdio.h>

#include "mtt/TargetList.h"

#include "cmath"
#include <iostream>
#include <vector>

#include <image_transport/image_transport.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv/cvwimage.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud.h"
#include <sensor_msgs/image_encodings.h>

#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>

#include <sensor_msgs/PointCloud2.h>
// #include <pcl/conversions.h>
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>

// #include <pcl/io/pcd_io.h>

// #include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
// #include <pcl/pcl_base.h>

using namespace std;
using namespace cv;
namespace enc = sensor_msgs::image_encodings;

// static const char WINDOW[] = "Image window";
// static const char WINDOW1[] = "Image window1";

int number_of_threads;

typedef enum
{
    PEDESTRIAN,
    NON_PEDESTRIAN,
    UNKNOWN
} TypeClassification;

class BoostClassifier
{
    public:
        typedef boost::shared_ptr<BoostClassifier> Ptr;
        
        bool load(const boost::filesystem::path& classifier_file)
        {
            region.x = 0;
            region.y = 0;
            
            region.width = DWWIDTH;
            region.height = DWHEIGHT;
            
            GetRandParams(SEED,NRFEATURE2, randparams, region);
            boost_classifier.load(classifier_file.string().c_str());
        }
        
        CvRect region;
        FtrVecParams2 classifier_randparams;
        CvBoost classifier_boost;
        FtrVecParams2 randparams;
        CvBoost boost_classifier;
};

class PedestrianClassifier
{
    public:
        PedestrianClassifier(const BoostClassifier::Ptr& classifier_base_):
        classifier_base(classifier_base_)
        {
            region.x = 0;
            region.y = 0;
            
            region.width = DWWIDTH;
            region.height = DWHEIGHT;
            
            minSize.width = DWWIDTH;
            minSize.height = DWHEIGHT;
        }
        
        TypeClassification classify(const Mat& image)
        {
//             region.width = rectwidth-1;
//             region.height = rectheight-1;
            
            GetChnFtrsOverImagePyramid(image, region, features ,NOCTUP, minSize, SCALEPOCT ,classifier_base->randparams,PedRect, classifier_base->boost_classifier);
            
            if(PedRect.size()>0)
            {
                return PEDESTRIAN;
            }
            else
            {
                return NON_PEDESTRIAN;
            }
        }
        
    private:
        
        BoostClassifier::Ptr classifier_base;
        
        CvRect region;  
        Size minSize;
        
        DVector features;
        PedRoiVec PedRect;
};
    
class Candidate
{
    public:
        
        typedef boost::shared_ptr<Candidate> Ptr;
        long int id;
        double distance;
        bool marked_for_deletion;
        bool classification_busy;
        
        Point3f world_upper_left;
        Point3f world_down_right;
        
        Point2f image_upper_right;
        Point2f image_down_left;
        
        boost::thread object_thread;
        TypeClassification classification;
        BoostClassifier::Ptr classifier_base;
        
//         Mat image;
        Mat object_image;
        
        Candidate(const BoostClassifier::Ptr& classifier_base_):
        classifier_base(classifier_base_)
        {
            
        }
        
        
        bool projectToImage(const Mat& rvecs_, const Mat& tvecs_, const Mat& camera_matrix, const Mat& dist_coeffs)
        {
            vector<Point3f> world_points;
            vector<Point2f> image_points;
            
            // Project Laser Points in Image
            world_points.push_back(world_upper_left);
            world_points.push_back(world_down_right);
            
            try
            {
                cv::projectPoints(world_points, rvecs_, tvecs_, camera_matrix, dist_coeffs, image_points);
            }
            catch(cv::Exception& e)
            {
                cout << "ERROR: " << e.what() << endl;
                return false;
            }
            
//             image_upper_right = Point2f(image_points[0].x + 45, image_points[0].y - 1200*tan(1./distance)); //usb_cam_reader
//             
//             image_down_left = Point2f(image_points[1].x - 55, image_points[1].y + 1100*tan(1./distance));// image_points[3];

            image_upper_right = Point2f(image_points[0].x, image_points[0].y);
            
            image_down_left = Point2f(image_points[1].x, image_points[1].y);// image_points[3];
            
            return true;
        }
        
        bool crop(const Mat& image)
        {
            int x = image_down_left.x;
            int y = image_upper_right.y;
            int width = image_upper_right.x - image_down_left.x;
            int height = image_down_left.y - image_upper_right.y;
            
            //boundary conditions
            // X - axis
            if(x+width < 0)
            {
                object_image = image(Rect(0,0,0,0));
                return false;
            }else
            {
                if(x > image.cols)
                {
                    object_image = image(Rect(0,0,0,0));
                    return false;
                }else
                {
                    if(x < 0)
                    {
                        x = 0;
                        width = width - sqrt(x^2);
                    }else
                    {
                        if(x+width > image.cols)
                        {
                            width=(image.cols-x);
                        }
                    }
                }
            }
            
            //Antigo
//                 if(x<0)
//                     x=0;
//                 if(x>image.cols)
//                     x=image.cols;
//                 if(x+width > image.cols) 
//                     width=(image.cols-x);
 
            // Y - axis
            if(y+height < 0)
            {
                object_image = image(Rect(0,0,0,0));
                return false;
            }else
            {
                if(y > image.rows)
                {
                    object_image = image(Rect(0,0,0,0));
                    return false;
                }else
                {
                    if(y < 0)
                    {
                        y = 0;
                        height = height - sqrt(y^2);
                    }else
                    {
                        if(y+height > image.rows)
                        {
                            height=(image.rows-y);
                        }
                    }
                }
            }
            
            // Antigo
//                 if(y<0)
//                     y=0;
//                 if(y>image.rows)
//                     y=image.rows;
//                 if(y+height > image.rows)
//                     height=(image.rows-y);
           
            try
            {
                object_image = image(Rect(x,y,width,height));
            }
            catch(cv::Exception& e)
            {
                object_image = image(Rect(0,0,0,0));
                cout << "ERROR: " << e.what() << endl;
            }
                
            return true;
        }
        
        bool draw(Mat& image)
        {            
            if (classification == PEDESTRIAN)
            {
                putText(image,"Pedestrian: " + to_string(id), Point2f(image_down_left.x,image_upper_right.y-5), FONT_HERSHEY_SIMPLEX, 0.5, cvScalar(0,255,0), 1, CV_AA); 
                rectangle(image, image_upper_right, image_down_left, Scalar(0,255,0), 2);
            }
            else
            {
                if(classification == NON_PEDESTRIAN)
                {
                    putText(image, "Object: " + to_string(id), Point2f(image_down_left.x,image_upper_right.y-5), FONT_HERSHEY_SIMPLEX, 0.5, cvScalar(0,0,0), 1, CV_AA);
                }
                else
                {
                    if(classification_busy == true)
                    {
                        putText(image, "Processing:  " + to_string(id), Point2f(image_down_left.x,image_upper_right.y-5), FONT_HERSHEY_SIMPLEX, 0.5, cvScalar(0,0,250), 1, CV_AA);
                    }
                    else
                    {
                        putText(image, "Not Processed "  + to_string(id), Point2f(image_down_left.x,image_upper_right.y-5), FONT_HERSHEY_SIMPLEX, 0.5, cvScalar(0,0,250), 1, CV_AA);
                    }
                }
                rectangle(image, image_upper_right, image_down_left, Scalar(0,0,255), 2);   
            }
                
            
        }
        
        void classify()
        {
            object_thread = boost::thread(boost::bind(&Candidate::doClassification,this));
            object_thread.join();
        }
    
        void doClassification()
        {
//             cout << "start_classification id: " << id << endl;
            classification_busy = true;
            double secs = ros::Time::now().toSec(); // inicia a contagem do tempo
            
            PedestrianClassifier classifier(classifier_base);
            
            if (object_image.cols <= 10) {
//                 cout << "Error reading file " << endl;
            }else
            {
                //Image resize (faster classification)
                Mat object_image_resize;
                if (object_image.rows > 250)
                {
//                          cout << "resize: " << cvSize((300/object_image.rows)*object_image.cols, 300) << endl;
                    int object_image_cols = round(object_image.cols*(250./object_image.rows));
                    
                    resize(object_image,object_image_resize,cvSize(object_image_cols, 250), 0, 0, CV_INTER_AREA);

    //                 object_image_resize *= (300/object_image.rows);
                }
                else
                {
                    object_image_resize = object_image;
                }
                                                    
                //Run the heady classification code from pedro
                classification = classifier.classify(object_image_resize);
            }
//             number_of_threads = number_of_threads - 1;
            
//             cout << "id:" << id << " duracao: " << ros::Time::now().toSec() -secs << endl; // fornece o tempo total que o programa demora a correr a imagem
//             cout << "classification done id: " << id << endl;
            classification_busy = false;
        }
                
        bool isBusy()
        {
            if (classification_busy)
            {
                return true;
            }
            else
            {
                return false;
            }
        }
        
        bool kill()
        {
//             cout<<"waiting for slaughter: "<<id<<endl;
//             number_of_threads = number_of_threads - 1;
            object_thread.interrupt();//
            
            bool success = object_thread.timed_join(boost::posix_time::microseconds(1.0));//
             
            if(success)
            {
//                 cout<<"thread is dead"<<endl;
                return true;
            }else
            {
//                 cout<<"thread is still alive"<<endl;
                return false;
            }
//             cout<<"thread is dead: "<<id<<endl;
//             return true;
        }

};

class ImageConverter
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    ros::Subscriber cam_info_sub_;
    ros::Subscriber mtt_laser_sub_;
    ros::Subscriber initial_laser_sub_;
    
    ros::Subscriber extrinsic_sub_;

    image_transport::Publisher image_pub_;
  
    public:
                
        double time_to_grab_frame;
        
        uint id;
        uint priority;
        double distance_to_target_from_center_bumper;
        double theta;
        int max_number_of_images_allowed;
        int number_of_images;

        cv::Mat cameraMatrix;
        cv::Mat distCoeffs;
        cv::Mat rvecs;
        cv::Mat tvecs;
                
        Mat final_image;
        vector<cv::Point3f> laser_points;
        vector<Candidate::Ptr> candidates;
        
        BoostClassifier::Ptr classifier_base;
  
        ImageConverter()
        : it_(nh_),
        classifier_base(new BoostClassifier)
        {
        
//             image_pub_ = it_.advertise("/out", 1);
//             image_sub_ = it_.subscribe("/usb_cam_reader/image_raw", 1, &ImageConverter::imageCb, this);
//             image_sub_ = it_.subscribe("/usb_cam_reader/image_rect_color", 1, &ImageConverter::imageCb, this);
//             
//             cam_info_sub_ = nh_.subscribe("/usb_cam_reader/camera_info", 1, &ImageConverter::cameraInfoCallback, this);
            
            image_sub_ = it_.subscribe("/xb3/right/new_info/image_rect_color", 1, &ImageConverter::imageCb, this);
            cam_info_sub_ = nh_.subscribe("/xb3/right/new_info/camera_info", 1, &ImageConverter::cameraInfoCallback, this);
            
            mtt_laser_sub_ = nh_.subscribe("/new_targets", 100, &ImageConverter::mttLaserGather, this);
            initial_laser_sub_ = nh_.subscribe("/scan_cloud", 1, &ImageConverter::initiallaserGather, this);
            
            string classifier_file;
//             nh_.param<string>("classifier",classifier_file,string("not_found"));
//             cout << "classifier_file" << classifier_file << endl;
            
            classifier_file = "/home/asus/workingcopies/lar4/src/perception/pedestrians/multimodal_pedestrian_detect/trained_boost_15Kf_2000w_19Ks_m8_M64_boot3.xml" ;
//             cout << "classifier_file1" << classifier_file << endl;
    //         cv::namedWindow(WINDOW);
    //         cv::namedWindow(WINDOW1);
            
            classifier_base->load(classifier_file);
            
            time_to_grab_frame= 0.0;
            
            max_number_of_images_allowed = 4;
            number_of_images = 1;
        }

        ~ImageConverter()
        {
    //         cv::destroyWindow(WINDOW);
    //         cv::destroyWindow(WINDOW1);
        }
        
        void cameraInfoCallback(const sensor_msgs::CameraInfo& cam_info_msg)
        {
            // Read Extrinsic Camera Parameters (D, R ,P) to extract cameraMatrix, distCoeffs, rvecs, tvecs
                        
//          -----Atlascar xb3---------------------------------------------
          FileStorage fs2("/home/asus/workingcopies/lar4/src/perception/pedestrians/multimodal_pedestrian_detect/calibration/bumblebeeXB3_1280_960_transformation_matrix_to_2Dlaser_18_09_2014.yaml", FileStorage::READ);  
//             FileStorage fs2("/home/asus/workingcopies/lar4/src/perception/pedestrians/multimodal_pedestrian_detect/calibration/bumblebeeXB3_1280_960_transformation_matrix_to_2Dlaser_semiAutomatic.yaml", FileStorage::READ); 

            distCoeffs = cvCreateMat(1,5 , CV_64FC1);
            for (int i = 0; i < 5; i++) 
                distCoeffs.at<double>(0,i) = 0;
    
            cameraMatrix = cvCreateMat(3,3, CV_64FC1);
            cv::Mat rotation_matriz= cvCreateMat(3,3 , CV_64FC1);
            
            cv::Mat trans_matriz = cvCreateMat(3,1, CV_64FC1);
            
            fs2["rectification_matrix"] >> rotation_matriz;
            fs2["translation_matrix"] >> trans_matriz;
            fs2["intrinsic_xb3_matrix"] >> cameraMatrix;
            
            rvecs= rotation_matriz;
            tvecs = trans_matriz;
//          ---------------------------------------------------------------

// //       -----usb_cam_reader -------------------------------------------   
//                  
//             cameraMatrix = cvCreateMat(3,3, CV_64FC1);
//             for (int i = 0; i < 3; i++)
//             {
//                 for (int j = 0; j < 3; j++)
//                 {
//                     if (i==0){ cameraMatrix.at<double>(i, j) = cam_info_msg.P[3*i+j];}
//                     if (i==1){ cameraMatrix.at<double>(i, j) = cam_info_msg.P[3*i+j+1];}
//                     if (i==2){ cameraMatrix.at<double>(i, j) = cam_info_msg.P[3*i+j+2];}
//                 }
//             }
//                         
//             distCoeffs = cvCreateMat(1,5 , CV_64FC1);
//             for (int i = 0; i < 5; i++) 
//                 distCoeffs.at<double>(0,i) = cam_info_msg.D[i];
//                                       
//             
//             cv::Mat rotation_matriz= cvCreateMat(3,1, CV_64FC1);
//             
//             for (int i = 0; i < 3; i++) 
//             {
//                 rotation_matriz.at<double>(i,0) = cam_info_msg.R[i];
//             }
//             
//             rvecs = rotation_matriz;
// 
//             cv::Mat trans_matriz = cvCreateMat(3,1, CV_64FC1);
// 
//             for (int i = 0; i < 3; i++) 
//             {
//                 trans_matriz.at<double>(i,0) = cam_info_msg.P[4*(i+1)-1];
//             }
// 
//             tvecs = trans_matriz;
//            --------------------------------------------------------------------------------------
           
//             cout << "camera_matrix: " << cameraMatrix 
//                 << "dist_coeffs: " << distCoeffs 
//                 << "rotation_matriz: " << rvecs 
//                 << "trans_matriz: " << tvecs << endl;
            cam_info_sub_.shutdown(); // the subscriber only runs one time
        }
        
        void imageCb(const sensor_msgs::ImageConstPtr& cam_msg)
        {
            cout << "grab_image: " <<  endl;
            
            double secs = ros::Time::now().toSec(); // inicia a contagem do tempo

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
//                    cout << "tvecs" << tvecs << endl;     
            if(tvecs.cols!=3)
            {
                cout<<"Incorrect size on T vecs matrix"<<endl;
                return;
            }
                        
            // Classificação dos objectos
            Mat cutted_image;
            cvtColor (cv_ptr->image, cutted_image, CV_BGR2RGB);
            
            // Crop candidate
            for(uint i=0;i<candidates.size();i++)
                if (candidates[i]->classification == UNKNOWN)
                    if (candidates[i]->classification_busy == false)
                        candidates[i]->crop(cutted_image);
            
            // Classify candidate  
            for(uint i=0;i<candidates.size();i++)
                if (candidates[i]->classification == UNKNOWN)
                    if (candidates[i]->classification_busy == false)
//                             candidates[i]->classify();
            
            int number_of_classified_ped = 0;
                    
//             int x = image_down_left.x;
//             int y = image_upper_right.y;
//             int width = image_upper_right.x - image_down_left.x;
//             int height = image_down_left.y - image_upper_right.y;
            
            
//             std::fstream fs;
            std::ostringstream oss;
            oss<< std::setfill('0') << std::setw(5) << cv_ptr->header.seq;
// //             fs.open ("/home/asus/workingcopies/lar4/src/perception/pedestrians/multimodal_pedestrian_detect/images/bag_1_sensor_fusion_1.3_txts/"+oss.str()+".txt", fstream::in | fstream::out | fstream::app);
//             fs.open ("/media/Novo volume/bags_tese_results/bag_2_sensor_fusion_4image_reset_txts/"+oss.str()+".txt", fstream::in | fstream::out | fstream::app);
//             
            for(uint i=0;i<candidates.size();i++)
            { 
//                 if (candidates[i]->classification == PEDESTRIAN || candidates[i]->classification == UNKNOWN)
//                 {
                    candidates[i]->draw(cv_ptr->image);
//                     if (candidates[i]->image_down_left.x >= 0 || candidates[i]->image_down_left.x <= 1280)
//                     {
//                         fs << candidates[i]->image_down_left.x << "  " << candidates[i]->image_upper_right.y << "  " << candidates[i]->image_upper_right.x - candidates[i]->image_down_left.x<< "  " << candidates[i]->image_down_left.y - candidates[i]->image_upper_right.y << "  " << ros::Time::now().toSec() -secs << "\n"; // xmin ymin width height
//                         number_of_classified_ped++;
//                     }
//                 }
            }
//             
//             if(number_of_classified_ped == 0)
//                 fs << 0 << "  " << 0 << "  " << 0 << "  " << 0 << "  " << ros::Time::now().toSec() -secs << "\n";
//             
//             fs.close();
            

//             imwrite( "/home/asus/workingcopies/lar4/src/perception/pedestrians/multimodal_pedestrian_detect/images/bag_1_sensor_fusion_1.3/"+oss.str()+".png", cv_ptr->image );
//             imwrite( "/media/Novo volume/bags_tese_results/teste_AtlasCar_outside/teste/"+oss.str()+".png", cv_ptr->image );
            
            cout << "time: " << ros::Time::now().toSec() -secs << endl;
//             Project Laser Points in Image
//             std::vector<cv::Point2f> projectedPoints;
//             
//             try
//             {
//                 cv::projectPoints(laser_points, rvecs, tvecs, cameraMatrix, distCoeffs, projectedPoints);
//             }
//             catch(cv::Exception& e)
//             {
//                 cout << "ERROR: " << e.what() << endl;
//             }
//                 
//                 
//             for(uint i=0;i<projectedPoints.size();i++)
//             {
//                 line(cv_ptr->image, projectedPoints[i],projectedPoints[i],Scalar(0,0,255),5);
//             }
            
            imshow("test",cv_ptr->image);
            waitKey(2); 
            
//             imwrite( "/media/Novo volume/bags_tese_results/bag_2_automatic_calibration/"+oss.str()+".png", cv_ptr->image );
            
//             time_to_grab_frame = ros::Time::now().toSec();
            
//             cout << "image_number: " << cv_ptr->header.seq << endl;
            
//             number_of_images++;
//             
//             if(number_of_images > max_number_of_images_allowed)
//                 number_of_images = 1;
//             
            cout << "let_go_image" << cv_ptr->header.seq << endl;
        }
    
        void  mttLaserGather(const mtt::TargetList& laser_msg)
        {
            double distance_to_target_from_center_bumper;
            double theta;
            double largura;
            bool id_found;
            
//             cout << "received laser: " << laser_msg.Targets.size() << endl;
            
            for(uint i=0;i<candidates.size();i++) // mark all candidates to slaughter
            {
                candidates[i]->marked_for_deletion = true;
            }
            
            for(int i=0; i<laser_msg.Targets.size();i++)
            {                
                id_found = false;
                
                distance_to_target_from_center_bumper = sqrt(laser_msg.Targets[i].pose.position.x * laser_msg.Targets[i].pose.position.x + 
                                    laser_msg.Targets[i].pose.position.y * laser_msg.Targets[i].pose.position.y);
                
                theta = acos(laser_msg.Targets[i].pose.position.x / distance_to_target_from_center_bumper);
                
                largura = sqrt(((laser_msg.Targets[i].finalpose.x - laser_msg.Targets[i].initialpose.x) * (laser_msg.Targets[i].finalpose.x - laser_msg.Targets[i].initialpose.x)) + ((laser_msg.Targets[i].finalpose.y - laser_msg.Targets[i].initialpose.y) * (laser_msg.Targets[i].finalpose.y - laser_msg.Targets[i].initialpose.y)));
                
                for(uint l=0;l<candidates.size();l++) // Search in current vector if laser target id already exists
                {
                    if (candidates[l]->id == laser_msg.Targets[i].id)
                    {                    
                        candidates[l]->distance = distance_to_target_from_center_bumper;
                        
                        candidates[l]->world_upper_left = cv::Point3f(laser_msg.Targets[i].initialpose.x, laser_msg.Targets[i].initialpose.y-0.4, laser_msg.Targets[i].initialpose.z+1.7);
                        
                        candidates[l]->world_down_right = cv::Point3f(laser_msg.Targets[i].finalpose.x, laser_msg.Targets[i].finalpose.y+0.4, laser_msg.Targets[i].finalpose.z-0.7); 
                        
                        candidates[l]->projectToImage(rvecs, tvecs, cameraMatrix,distCoeffs);
//                         cout << "linear velocity x: " << laser_msg.Targets[i].velocity.linear.x << endl;
                        if(sqrt(laser_msg.Targets[i].velocity.linear.x * laser_msg.Targets[i].velocity.linear.x) > 0.08 && sqrt(laser_msg.Targets[i].velocity.linear.y * laser_msg.Targets[i].velocity.linear.y) > 0.08)                            
                            if(candidates[l]->classification == NON_PEDESTRIAN)
                                candidates[l]->classification = UNKNOWN;
                            
                        if(largura < 1.3) 
                            candidates[l]->marked_for_deletion = false;
                        
                        if(laser_msg.Targets[i].initialpose.x < 0)
                            candidates[l]->marked_for_deletion = true;
                        
//                         if(number_of_images == max_number_of_images_allowed)
//                             candidates[l]->classification = UNKNOWN;
                                                
                        id_found = true;
//                             cout << "candidate: " << candidates[l]->classification << endl;
//                             cout << "updated candidate: " <<candidates[l]->id<< endl;
                        break;                            
                    }
                }
                
                if (laser_msg.Targets[i].initialpose.x > 0 && largura > 0.15 && largura < 0.8) 
                {//inicial conditions to be consider a possible pedestrian candidate
                    if (id_found == false)
                    {
                        Candidate::Ptr candidate = Candidate::Ptr(new Candidate(classifier_base));
                    
                        candidate->id = laser_msg.Targets[i].id;
                        
                        cout << "new candidate: " << candidate->id << endl;
                       
                        candidate->distance = distance_to_target_from_center_bumper;
                        
                        candidate->world_upper_left = cv::Point3f(laser_msg.Targets[i].initialpose.x, laser_msg.Targets[i].initialpose.y-0.4, laser_msg.Targets[i].initialpose.z+1.7);
                        
                        candidate->world_down_right = cv::Point3f(laser_msg.Targets[i].finalpose.x, laser_msg.Targets[i].finalpose.y+0.4, laser_msg.Targets[i].finalpose.z-0.7); 
                        
                        candidate->projectToImage(rvecs, tvecs, cameraMatrix,distCoeffs);
                        
                        candidate->classification = UNKNOWN;
                        
                        candidate->marked_for_deletion = false;
                        candidate->classification_busy = false;
                        
                        candidates.push_back(candidate);
                    }
                }
            }
            
            for(auto it = candidates.begin(); it!= candidates.end();) // Candidate cleaning process
            { 
                if((*it)->marked_for_deletion) 
                { 
                    if((*it)->isBusy())
                    {
                        if((*it)->kill())
                        {
                            cout << "removing candidate: " << (*it)->id << endl;
                            it = candidates.erase(it);
                        }
                    }else
                    {
                        cout << "removing candidate: " << (*it)->id << endl;
                        it = candidates.erase(it); 
                    }
                }
                else 
                    it++;
            }
            
            // Ordenar os candidatos pela distancia a que estao do Laser
            sort(candidates.begin(), candidates.end(),[&](const Candidate::Ptr c1, const Candidate::Ptr c2)
            { 
                return c1->distance < c2->distance; 
            });
        }

        void  initiallaserGather(const sensor_msgs::PointCloud2& laser_msg)
        {
            
            double distance_to_target_from_center_bumper;
            double theta;
    //         cout << "i'm here" << endl;
            pcl::PCLPointCloud2 pcl_pc;
            pcl_conversions::toPCL(laser_msg, pcl_pc);

            pcl::PointCloud<pcl::PointXYZ> cloud;

            pcl::fromPCLPointCloud2(pcl_pc, cloud);
            
            laser_points.clear();
            
            for(int i=0; i<cloud.points.size();i++)
            {
                distance_to_target_from_center_bumper = sqrt(cloud.points[i].x * cloud.points[i].x + 
                                    cloud.points[i].y * cloud.points[i].y);

                theta = acos(cloud.points[i].x / distance_to_target_from_center_bumper);
            
                if (theta < 1.5)
                {
                    laser_points.push_back(cv::Point3f(cloud.points[i].x,cloud.points[i].y,cloud.points[i].z));
                }
            }
        }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sensor_fusion");
  
  ImageConverter ic;
  
  ros::spin();
  return 0;
}
