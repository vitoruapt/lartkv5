#include <iostream>
#include <chrono>
#include <cstring>
#include <csignal>

#include <dc1394/log.h>
#include <dc1394/control.h>
#include <dc1394/conversions.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <boost/thread.hpp>

#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>

#include <sensor_msgs/image_encodings.h>

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

#define BAYER_TILE_MAPPING_REGISTER     (0x1040)
#define SENSOR_BOARD_INFO_REGISTER      (0x1f28)
#define IMAGE_DATA_FORMAT_REGISTER  (0x1048)

using namespace std;

class xb3CameraDriver
{
    public:
        dc1394_t* d;
        unsigned int nThisCam;
        dc1394camera_t* camera;
        dc1394color_filter_t bayerTile;
        
        unsigned int nBufferSize;
        unsigned char* pucRightRGB;
        unsigned char* pucLeftRGB;
        unsigned char* pucCenterRGB;
        unsigned char* pucGreenBuffer;
        unsigned char* pucRGBBuffer;
        unsigned char* pucDeInterlacedBuffer;
        unsigned char* pucGrabBuffer;
        
        const unsigned int hardware_width;
        const unsigned int hardware_height;
        const unsigned int nBytesPerPixel;
        
        unsigned int output_width;
        unsigned int output_height;
        
        //Load camera info from file
        string left_calibration_file;
        string center_calibration_file;
        string right_calibration_file;
            
        image_transport::CameraPublisher left_publisher;
        image_transport::CameraPublisher center_publisher;
        image_transport::CameraPublisher right_publisher;
        
        camera_info_manager::CameraInfoManager left_info_manager;
        camera_info_manager::CameraInfoManager center_info_manager;
        camera_info_manager::CameraInfoManager right_info_manager;
        
        sensor_msgs::CameraInfo left_camera_info;
        sensor_msgs::CameraInfo center_camera_info;
        sensor_msgs::CameraInfo right_camera_info;
        
        sensor_msgs::Image left;                                                                                                                                                                   
        sensor_msgs::Image center;
        sensor_msgs::Image right;
            
        cv::Mat opencv_left;
        cv::Mat opencv_center;
        cv::Mat opencv_right;
        
        cv::Mat opencv_output_left;
        cv::Mat opencv_output_center;
        cv::Mat opencv_output_right;
        
        boost::thread* buffer_processing_thread;
        boost::thread_group threads;

        ros::NodeHandle nh;
        image_transport::ImageTransport it;
        
        //Diagnostics class
        diagnostic_updater::Updater updater;
        
        //Maximum admissible frequency
        double status_max_frequency;
        
        //Minimum admissible frequency
        double status_min_frequency;
        
        diagnostic_updater::HeaderlessTopicDiagnostic frequency_diagnostics;
    
        xb3CameraDriver(ros::NodeHandle nh_):
        nh(nh_),
        it(nh_),
        hardware_width(1280),
        hardware_height(960),
        nBytesPerPixel(3),
        left_info_manager(ros::NodeHandle(nh_, "left"),"left"),
        center_info_manager(ros::NodeHandle(nh_, "center"),"center"),
        right_info_manager(ros::NodeHandle(nh_, "right"),"right"),
        status_max_frequency(18),
        status_min_frequency(14),
        frequency_diagnostics("Xb3",updater,diagnostic_updater::FrequencyStatusParam(&status_min_frequency,&status_max_frequency, 0.01, 10)),
        buffer_processing_thread(NULL)
        {
            updater.setHardwareID("Xb3");
            
            init();
            
            //Reduce shutter time
//             dc1394_feature_set_mode(camera,DC1394_FEATURE_SHUTTER,DC1394_FEATURE_MODE_MANUAL);
//             dc1394_feature_set_value(camera,DC1394_FEATURE_SHUTTER,100);
            
            nBufferSize = hardware_height*hardware_width*nBytesPerPixel;
            pucDeInterlacedBuffer = new unsigned char[nBufferSize];
            pucRGBBuffer = new unsigned char[3*nBufferSize];
            pucGreenBuffer = new unsigned char[nBufferSize];
            pucGrabBuffer = new unsigned char[3*nBufferSize];
            
            opencv_left.create(cvSize(hardware_width,hardware_height),CV_8UC3);
            opencv_center.create(cvSize(hardware_width,hardware_height),CV_8UC3);
            opencv_right.create(cvSize(hardware_width,hardware_height),CV_8UC3);
            
            //Create camera publishers
            left_publisher = it.advertiseCamera("left/image_raw", 100);
            center_publisher = it.advertiseCamera("center/image_raw", 100);
            right_publisher = it.advertiseCamera("right/image_raw", 100);
            
            //Load calibration paths
            nh.param("left_calibration_file",left_calibration_file,string("package://xb3/calibrations/left_full_resolution.yaml"));
            nh.param("center_calibration_file",center_calibration_file,string("package://xb3/calibrations/center_full_resolution.yaml"));
            nh.param("right_calibration_file",right_calibration_file,string("package://xb3/calibrations/right_full_resolution.yaml"));
            
            //Load camera calibrations
            left_info_manager.loadCameraInfo(left_calibration_file);
            center_info_manager.loadCameraInfo(center_calibration_file);
            right_info_manager.loadCameraInfo(right_calibration_file);
            
            //Create camera info messages
            left_camera_info = left_info_manager.getCameraInfo();
            center_camera_info = center_info_manager.getCameraInfo();
            right_camera_info = right_info_manager.getCameraInfo();
            
            //Fill the remaining parameters
            left_camera_info.header.frame_id = ros::names::remap("/xb3_optical_frame"); 
            left_camera_info.roi.do_rectify = true;
            
            center_camera_info.header.frame_id = ros::names::remap("/xb3_optical_frame");
            center_camera_info.roi.do_rectify = true;
            
            right_camera_info.header.frame_id = ros::names::remap("/xb3_optical_frame");
            right_camera_info.roi.do_rectify = true;
            
            output_height = left_camera_info.height;
            output_width = left_camera_info.width;
            
            opencv_output_left.create(cvSize(output_width,output_height),CV_8UC3);
            opencv_output_center.create(cvSize(output_width,output_height),CV_8UC3);
            opencv_output_right.create(cvSize(output_width,output_height),CV_8UC3);
            
            //Fill image static parameters
            left.height = left_camera_info.height; //set the height.
            left.width = left_camera_info.width; //set the width
            left.encoding = sensor_msgs::image_encodings::RGB8; //Set the encoding
            left.is_bigendian = 0;
            left.step = left.width*3;
            left.data.resize(left.width*left.height*3);
            left.header.frame_id = ros::names::remap( "/xb3_optical_frame");
            
            center.height = center_camera_info.height; //set the height.
            center.width = center_camera_info.width; //set the width
            center.encoding = sensor_msgs::image_encodings::RGB8; //Set the encoding
            center.is_bigendian = 0;
            center.step = center.width*3;
            center.data.resize(center.width*center.height*3);
            center.header.frame_id = ros::names::remap( "/xb3_optical_frame");
            
            right.height = right_camera_info.height; //set the height.
            right.width = right_camera_info.width; //set the width
            right.encoding = sensor_msgs::image_encodings::RGB8; //Set the encoding
            right.is_bigendian = 0;
            right.step = right.width*3;
            right.data.resize(right.width*right.height*3);
            right.header.frame_id = ros::names::remap( "/xb3_optical_frame");
        }
        
        ~xb3CameraDriver()
        {
            cout<<"[ xb3CameraDriver]: Stopping transmission"<<endl;
            buffer_processing_thread->join();
            clean_dc1394();
            cout<<"[ xb3CameraDriver]: Camera closed"<<endl;
        }
        
        void clean_dc1394(void)
        {
            dc1394_capture_stop(camera);
            dc1394_video_set_transmission(camera, DC1394_OFF);
            dc1394_camera_free(camera);
        }
        
        dc1394error_t init()
        {
            dc1394camera_list_t* list;
            dc1394error_t err;

            // Find cameras on the 1394 buses
            d = dc1394_new();

            // Enumerate cameras connected to the PC
            err = dc1394_camera_enumerate (d, &list);
            if (err != DC1394_SUCCESS )
            {
                ROS_ERROR("Unable to look for cameras\n\nPlease check \n"
                                "  - if the kernel modules `ieee1394',`raw1394' and `ohci1394' are loaded \n"
                                "  - if you have read/write access to /dev/raw1394\n\n");
                return err;
            }

            //  get the camera nodes and describe them as we find them
            if (list->num == 0)
            {
                ROS_ERROR("No cameras found!");
                clean_dc1394();
                return DC1394_FAILURE;
            }

            ROS_INFO("There were %d camera(s) found attached to your PC", list->num);

            // Identify cameras. Use the first stereo camera that is found
            for ( nThisCam = 0; nThisCam < list->num; nThisCam++ )
            {
                camera = dc1394_camera_new(d, list->ids[nThisCam].guid);

                if(!camera)
                {
                    ROS_ERROR("Failed to initialize camera with guid %lx", list->ids[nThisCam].guid);
                    continue;
                }

                ROS_INFO("Camera %d model = '%s'", nThisCam, camera->model );

                if(strncmp(camera->model, "Bumblebee XB3", strlen("Bumblebee XB3")) == 0)
                {
                    ROS_INFO("Found Bumblebee XB3 camera");
                    break;
                }

                dc1394_camera_free(camera);
            }

            // set the capture mode
            ROS_INFO("Setting stereo video capture mode");

            err = setStereoVideoCapture();
            if ( err != DC1394_SUCCESS )
            {
                ROS_ERROR("Could not set up video capture mode");
                clean_dc1394();
                return DC1394_FAILURE;
            }

            // have the camera start sending us data
            ROS_INFO("Start transmission");
            err = startTransmission();
            if ( err != DC1394_SUCCESS )
            {
                ROS_ERROR("Unable to start camera iso transmission" );
                clean_dc1394();
            }
            
            err = getBayerTile(camera,&bayerTile);
            if ( err != DC1394_SUCCESS )
            {
                ROS_ERROR("Could not query the Bayer Tile Register!" );
                return DC1394_FAILURE;
            }
      
            return DC1394_SUCCESS;
        }
        
        dc1394error_t getBayerTile( dc1394camera_t* camera,dc1394color_filter_t* bayerPattern)
        {
            uint32_t value;
            dc1394error_t err;

            // query register 0x1040
            // This register is an advanced PGR register called BAYER_TILE_MAPPING
            // For more information check the PGR IEEE-1394 Digital Camera Register Reference
            err = dc1394_get_control_register( camera, BAYER_TILE_MAPPING_REGISTER, &value );
            if ( err != DC1394_SUCCESS )
            {
                return err;
            }

            // Ascii R = 52 G = 47 B = 42 Y = 59
            switch( value )
            {
                default:
                case 0x59595959:  // YYYY
                // no bayer
                *bayerPattern = (dc1394color_filter_t) 0;
                break;
                case 0x52474742:  // RGGB
                *bayerPattern = DC1394_COLOR_FILTER_RGGB;
                break;
                case 0x47425247:  // GBRG
                *bayerPattern = DC1394_COLOR_FILTER_GBRG;
                break;
                case 0x47524247:  // GRBG
                *bayerPattern = DC1394_COLOR_FILTER_GRBG;
                break;
                case 0x42474752:  // BGGR
                *bayerPattern = DC1394_COLOR_FILTER_BGGR;
                break;
            }

            return err;
        }

        dc1394error_t setStereoVideoCapture()
        {
            dc1394error_t    err;

            // Bumblebee3 transmits stereo images in Format 7
            // load the factory defaults - this is auto-everything
            err = dc1394_memory_load(camera, 0 );
            if ( err != DC1394_SUCCESS ) 
            {
                ROS_ERROR("Can't load default memory channel" );
                return err;
            }

            // assume the XB is plugged into a 1394B network
            // XB3 can work with a 1394A bus but code changes will be required
            dc1394_video_set_operation_mode(camera, DC1394_OPERATION_MODE_1394B);
            
            dc1394_video_set_iso_speed(camera, DC1394_ISO_SPEED_800 );
            dc1394_video_set_mode(camera, DC1394_VIDEO_MODE_FORMAT7_3 );
            err = dc1394_format7_set_roi(camera,
                            DC1394_VIDEO_MODE_FORMAT7_3,
                            DC1394_COLOR_CODING_RGB8,
                                            // bytes per packet - sets frame rate
                            7680, //DC1394_USE_MAX_AVAIL
                            0, 
                            0,
                            hardware_width,
                            hardware_height);
            if ( err != DC1394_SUCCESS )
            {
                ROS_ERROR("Can't setup Bumblebee XB3 capture" );
                return err;
            }

            err = dc1394_capture_setup(camera, 1, DC1394_CAPTURE_FLAGS_DEFAULT );
            if ( err != DC1394_SUCCESS ) 
            {
                ROS_ERROR("Can't setup Bumblebee capture" );
                return err;
            }

            return DC1394_SUCCESS;
        }
        
        dc1394error_t startTransmission()
        {
            dc1394error_t err;
            
            // have the camera start sending us data
            err = dc1394_video_set_transmission(camera, DC1394_ON );
            if ( err != DC1394_SUCCESS )
            {
                ROS_ERROR("Unable to start camera iso transmission" );
                return err;
            }

            // printf( "Waiting for transmission... \n" );
            //  Sleep untill the camera has a transmission
            dc1394switch_t status = DC1394_OFF;

            for ( int i = 0; i <= 5; i++ )
            {
                usleep(50000);
                
                
                err = dc1394_video_get_transmission(camera, &status );
                if ( err != DC1394_SUCCESS ) 
                {
                    ROS_ERROR("Unable to get transmision status" );
                    return err;
                }
                
                if ( status != DC1394_OFF )
                    break;

                if( i == 5 ) 
                {
                    ROS_ERROR("Camera doesn't seem to want to turn on!");
                    return DC1394_FAILURE;
                }
            }
            
            return DC1394_SUCCESS;
        }
        
        void dc1394_deinterlace_rgb(unsigned char* src,unsigned char* dest, unsigned int width_, unsigned int height_)
        {
            register int i = (width_*height_)-1;
            register int r = ((width_*height_)/3)-1;
            register int g = ((width_*height_)*2/3)-1;
            register int b = (width_*height_)-1;

            while (i >= 0)
            {
                dest[r--] = src[i--];
                dest[g--] = src[i--];
                dest[b--] = src[i--];
            }
        }

        void getFrameAndPublish()
        {
            dc1394error_t err;
            dc1394video_frame_t* frame;

            err = dc1394_capture_dequeue(camera,DC1394_CAPTURE_POLICY_WAIT,&frame);
            if ( err != DC1394_SUCCESS )
            {
                ROS_ERROR("extractImagesColor - cannot dequeue image!" );
                return;
            }

            dc1394_capture_enqueue(camera,frame);
            
            memcpy(pucGrabBuffer,frame->image,nBufferSize);
            
//             if(buffer_processing_thread)
//                 buffer_processing_thread->join();
            
            processBuffer();
//             buffer_processing_thread = new boost::thread(boost::bind(&xb3CameraDriver::processBuffer,this)); 
//             publish();
         
            return;
        }
        
        void processBuffer()
        {
            dc1394_deinterlace_rgb(pucGrabBuffer,pucDeInterlacedBuffer,hardware_width,3*hardware_height);

            // extract color from the bayer tile image
            // note: this will alias colors on the top and bottom rows
            dc1394_bayer_decoding_8bit(pucDeInterlacedBuffer,pucRGBBuffer,hardware_width,3*hardware_height,bayerTile,DC1394_BAYER_METHOD_HQLINEAR);

            if(output_height == hardware_height && output_width == hardware_width)
            {
                threads.join_all();
                
                ros::Time timestamp = ros::Time::now();
                threads.create_thread(boost::bind(&xb3CameraDriver::processLeftImage,this,timestamp));
                threads.create_thread(boost::bind(&xb3CameraDriver::processCenterImage,this,timestamp));
                threads.create_thread(boost::bind(&xb3CameraDriver::processRightImage,this,timestamp));
            }else
            {
                ///TODO change this mode to multi thread
                copyToMat();
                resizeToOutputMat();
                fillRosImagesFromMat();
                publish();
            }
        }
        
        void processLeftImage(ros::Time timestamp)
        {
            //Copy to ros image
            unsigned char* ptr_left_buffers = pucRGBBuffer;
            memcpy(left.data.data(),ptr_left_buffers,left.data.size());
            
            left.header.stamp = timestamp;
            left_camera_info.header.stamp = timestamp;
            
            left_publisher.publish(left,left_camera_info,timestamp);
        }
        
        void processCenterImage(ros::Time timestamp)
        {
            //Copy to ros image
            unsigned char* ptr_center_buffers = pucRGBBuffer + 3 * hardware_width*hardware_height; 
            memcpy(center.data.data(),ptr_center_buffers,center.data.size());
            
            center.header.stamp = timestamp;
            center_camera_info.header.stamp = timestamp;
            
            center_publisher.publish(center,center_camera_info,timestamp);
        }
        
        void processRightImage(ros::Time timestamp)
        {
            //Copy to ros image
            unsigned char* ptr_right_buffers = pucRGBBuffer + 6 * hardware_width*hardware_height; 
            memcpy(right.data.data(),ptr_right_buffers,right.data.size());
            
            right.header.stamp = timestamp;
            right_camera_info.header.stamp = timestamp;
            
            right_publisher.publish(right,right_camera_info,timestamp);
        }
        
        void fillRosImagesFromMat()
        {
            unsigned char* ptr_left_buffers = opencv_output_left.ptr<unsigned char>(0); 
            unsigned char* ptr_center_buffers = opencv_output_center.ptr<unsigned char>(0); 
            unsigned char* ptr_right_buffers = opencv_output_right.ptr<unsigned char>(0); 

            memcpy(left.data.data(),ptr_left_buffers,left.data.size());
            memcpy(center.data.data(),ptr_center_buffers,center.data.size());
            memcpy(right.data.data(),ptr_right_buffers,right.data.size());
        }
        
        void resizeToOutputMat()
        {
            cv::resize(opencv_left,opencv_output_left,opencv_output_left.size(),0,0,CV_INTER_AREA);
            cv::resize(opencv_center,opencv_output_center,opencv_output_center.size(),0,0,CV_INTER_AREA);
            cv::resize(opencv_right,opencv_output_right,opencv_output_right.size(),0,0,CV_INTER_AREA);
        }
        
        void copyToMat()
        {
            //Get pointers to the three image buffers
            unsigned char* ptr_left_buffers = pucRGBBuffer; 
            unsigned char* ptr_center_buffer = pucRGBBuffer + 3 * hardware_width*hardware_height; 
            unsigned char* ptr_right_buffers = pucRGBBuffer + 6 * hardware_width*hardware_height; 

            //Get pointers to the three cv::Mat
            unsigned char* ptr_left_cvMat = opencv_left.ptr<unsigned char>(0);
            unsigned char* ptr_center_cvMat = opencv_center.ptr<unsigned char>(0);
            unsigned char* ptr_right_cvMat = opencv_right.ptr<unsigned char>(0);

            memcpy(ptr_left_cvMat,ptr_left_buffers,nBufferSize);
            memcpy(ptr_center_cvMat,ptr_center_buffer,nBufferSize);
            memcpy(ptr_right_cvMat,ptr_right_buffers,nBufferSize);
            
            return;
        }
        
        void fillRosImages()
        {
            unsigned char* ptr_left_buffers = pucRGBBuffer; 
            unsigned char* ptr_center_buffers = pucRGBBuffer + 3 * hardware_width*hardware_height; 
            unsigned char* ptr_right_buffers = pucRGBBuffer + 6 * hardware_width*hardware_height; 

            memcpy(left.data.data(),ptr_left_buffers,left.data.size());
            memcpy(center.data.data(),ptr_center_buffers,center.data.size());
            memcpy(right.data.data(),ptr_right_buffers,right.data.size());
        }
        
        void publish()
        {
            ros::Time timestamp = ros::Time::now();
            
            left.header.stamp = timestamp;
            center.header.stamp = timestamp;
            right.header.stamp = timestamp;
            
            left_camera_info.header.stamp = timestamp;
            center_camera_info.header.stamp = timestamp;
            right_camera_info.header.stamp = timestamp;
            
            left_publisher.publish(left,left_camera_info,timestamp);
            center_publisher.publish(center,center_camera_info,timestamp);
            right_publisher.publish(right,right_camera_info,timestamp);            
        }
};

int main(int argc,char** argv)
{
    ros::init(argc, argv, "xb3");
    ros::NodeHandle nh("~"); 
    
    //Start camera driver
    xb3CameraDriver xb3(nh);
    
//     ros::Rate loop_rate(15);
//     ros::Time start = ros::Time::now();
//     boost::posix_time::ptime start = boost::posix_time::microsec_clock::local_time();
//     boost::posix_time::ptime end;
    while(ros::ok())
    {
        xb3.getFrameAndPublish();
//         end = boost::posix_time::microsec_clock::local_time();
//         cout<<"Freq: "<<1e6/((end-start).total_microseconds())<<endl;
//         start = boost::posix_time::microsec_clock::local_time();
//         cout<<"frame rate: "<<1./((ros::Time::now()-start).toSec())<<endl;
//         start = ros::Time::now();
    }
    
    return 0;
}
    