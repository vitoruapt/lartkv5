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
/**
 * @addtogroup xb3
 * @file 
 * @brief the actual cpp code for the xb3 module
 *@{
 */
#ifndef _XB3_CPP_
#define _XB3_CPP_

#include "xb3.h"

/** \brief stop camera captur
 *  stops camera capture using dc1394
 *  \return (void)
 */
void clean_dc1394(void)
{
	dc1394_capture_stop(t_dc1394.camera);
	
	ROS_WARN("stopping camera capture\n");
	
	dc1394_video_set_transmission(t_dc1394.camera, DC1394_OFF );
	
	dc1394_camera_free( t_dc1394.camera );
}


/** \brief signal handler function
 *  This handles with SIGINT and SIGSEGV signals
 *  \param[in] sig signal
 *  \return (void)
 */
void signal_handler(int sig)
{
// 	printf("%d %d %d\n",sig,SIGSEGV,SIGINT);
	
	if(sig==SIGSEGV)
	{	
	
		signal(SIGSEGV, SIG_DFL); 
	
		ROS_WARN("System segfaulted"); 
	
		clean_dc1394();
		ros::shutdown();
	
// 		exit(0);
	}
	else if(sig==SIGINT)
	{
		ROS_WARN("Ctrl-c pressed"); 
	
		clean_dc1394();
		ros::shutdown();
	
// 		exit(0);
	}
}


/** \brief initialize camera
 *  initializes camera with dc1394
 * \param[in] d
 * \param[in] stereoCamera
 * \param[in] camera
 * \param[in] nThisCam
 *  \return Success 
 */
int init_camera(dc1394_t * d, PGRStereoCamera_t* stereoCamera, dc1394camera_t** camera, unsigned int nThisCam)	

{
    dc1394camera_list_t * list;
    dc1394error_t 	err;

	// Find cameras on the 1394 buses
	d = dc1394_new ();

    // Enumerate cameras connected to the PC
	err = dc1394_camera_enumerate (d, &list);

	if ( err != DC1394_SUCCESS )
	{
		fprintf( stderr, "Unable to look for cameras\n\n"
				"Please check \n"
						"  - if the kernel modules `ieee1394',`raw1394' and `ohci1394' are loaded \n"
						"  - if you have read/write access to /dev/raw1394\n\n");
		return 1;
	}

	//  get the camera nodes and describe them as we find them
	if (list->num == 0)
	{
		fprintf( stderr, "No cameras found!\n"); clean_dc1394();
	}

	printf( "There were %d camera(s) found attached to your PC\n", list->num  );

	// Identify cameras. Use the first stereo camera that is found
	for ( nThisCam = 0; nThisCam < list->num; nThisCam++ )
	{
		*camera = dc1394_camera_new(d, list->ids[nThisCam].guid);

		if(!*camera)
		{
			printf("Failed to initialize camera with guid %llx", list->ids[nThisCam].guid);
			continue;
		}

		printf( "Camera %d model = '%s'\n", nThisCam, (*camera)->model );

		if ( isStereoCamera(*camera))
		{

			printf( "Using this camera\n" );
			break;
		}

		dc1394_camera_free(*camera);
	}

	if ( nThisCam == list->num )
	{
		printf( "No stereo cameras were detected\n" );
		return 0;
	}

	
	// query information about this stereo camera
	err = queryStereoCamera( *camera, stereoCamera );
	if ( err != DC1394_SUCCESS )
	{
		fprintf( stderr, "Cannot query all information from camera\n" );
		clean_dc1394();
	}

	if ( stereoCamera->model != BUMBLEBEEXB3 )
	{
		fprintf( stderr, "Stereo camera found was not a BB XB3\n" );
		clean_dc1394();
	}

	// override the default nBytesPerPixel to be 3, because we really do
	// want to run at 3 camera mode
	stereoCamera->nBytesPerPixel = 3;
	stereoCamera->nRows = 960; //this should be untouched. This is the actual sensor resolution
	stereoCamera->nCols = 1280;//this should be untouched. This is the actual sensor resolution

	// set the capture mode
	printf( "Setting stereo video capture mode\n" );

	err = setStereoVideoCapture( stereoCamera );

	if ( err != DC1394_SUCCESS )
	{
		fprintf( stderr, "Could not set up video capture mode\n" );
		clean_dc1394();
	}

	// have the camera start sending us data
	printf( "Start transmission\n" );
	err = startTransmission( stereoCamera );
	if ( err != DC1394_SUCCESS )
	{
		fprintf( stderr, "Unable to start camera iso transmission\n" );
		clean_dc1394();
	}
return 1;}


/** \brief copy an unsigned char buffer to cv::Mat
 * \param[in] Buffer Buffer with the 3 images
 * \param[in] left left image
 * \param[in] center center image
 * \param[in] right right image
 * \return 1
 */
int copy_img_buffer_to_cvMat(unsigned char* Buffer, cv::Mat left, cv::Mat center, cv::Mat right)
{
		//Get pointers to the three image buffers
		unsigned char* ptr_left_buffers = Buffer; 
		unsigned char* ptr_center_buffer = Buffer + 3 * 1280*960; 
		unsigned char* ptr_right_buffers = Buffer + 6 * 1280*960; 

		//Get pointers to the three cv::Mat
		unsigned char* ptr_left_cvMat = left.ptr<unsigned char>(0);
		unsigned char* ptr_center_cvMat = center.ptr<unsigned char>(0);
		unsigned char* ptr_right_cvMat = right.ptr<unsigned char>(0);


		int count=0;
		for(int c=0; c<1280;c++)
			for (int l=0; l<960; l++)
			{
				//from RGB to BGR	
				ptr_left_cvMat[count] = ptr_left_buffers[count+2];
				ptr_left_cvMat[count+1] = ptr_left_buffers[count+1];
				ptr_left_cvMat[count+2] = ptr_left_buffers[count];

				ptr_center_cvMat[count] = ptr_center_buffer[count+2];
				ptr_center_cvMat[count+1] = ptr_center_buffer[count+1];
				ptr_center_cvMat[count+2] = ptr_center_buffer[count];

				ptr_right_cvMat[count] = ptr_right_buffers[count+2];
				ptr_right_cvMat[count+1] = ptr_right_buffers[count+1];
				ptr_right_cvMat[count+2] = ptr_right_buffers[count];

				count+=3;	
			}
return 1;}

/** \brief This function will set all the fixed parameters on image sensor messages.
 * \param[in] msg the image message
 * \param[in] height 
 * \param[in] width
 * \param[in] encoding
 * \param[in] is_bigendian
 * \param[in] frame_id
 * \return (void)
 */
void set_fixed_fields_image_msg(sensor_msgs::Image* msg, int height, int width, char* encoding, int is_bigendian, char* frame_id)
{
	msg->height   = height; //set the height.
	msg->width    = width; //set the width
	msg->encoding = sensor_msgs::image_encodings::RGB8; //Set the encoding
	msg->is_bigendian = 0;
	msg->step = width*3;
	msg->data.resize(width*height*3);
	msg->header.frame_id = frame_id;
	//ROS_INFO("Resized image msg to %d",width*height*3);
}

/** \brief This function will set all the fixed parameters on image camera info messages
 * \param[in] info
 * \param[in] height
 * \param[in] width
 * \param[in] frame_id
 * \return (void)
 */
void set_fixed_camera_info(sensor_msgs::CameraInfo *info, int height, int width, char* frame_id)
{
	info->height = height;
	info->width = width;
	info->header.frame_id = frame_id;		
	info->roi.do_rectify = true;
}

/** \brief Copy image pixel data to ros sensor image message
 * \param[in] in
 * \param[in] image
 * \param[in] size
 * \return (void)
 */
void copy_pixels_to_image_msg_data(unsigned char *in, sensor_msgs::Image *image, int size)
{
	//printf("image->width=%d height=%d\n",image->width, image->height);
	//int t=0;
	for(int i=0; i<size; i+=3)
	//for (int l=0; l<960; l++)
		//for(int c=0; c<1280; c++)
	{
		//printf("t=%d i=%d\n",t,i);
	//char a  = in[i+2];	
		//int t = c + l*1280;	
		image->data[i] = in[i+2];	
		image->data[i+1] = in[i+1];	
		image->data[i+2] = in[i];	
		//t+=3;
	}
}

/** \brief necessary function for dynamic_reconfigure (I guess :))
 * \param[in] config
 * \param[in] level
 * \return (void)
 */
void reconfig(xb3::xb3Config &config, uint32_t level)
{
	ROS_INFO("dynamic reconfigure1n"); 
}

using namespace std;

int main(int argc, char **argv)
{
	/// Set default values of variables, init pointers, etc
	t_flags.debug=false;
	t_buffers.pucLeftRGB=NULL;
	t_buffers.pucCenterRGB=NULL;
	t_buffers.pucRightRGB=NULL;

	/// Initialize ROS coms
	ros::init(argc, argv, "xb3", ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
	ros::NodeHandle nh;	
	
/// Setup the signals
	signal(SIGINT, signal_handler);
	signal(SIGSEGV, signal_handler);

	/// Get the parameters for image resolution
	
	int width;
	if (nh.hasParam("width"))
	{
		nh.getParam("width", width);
	}
	else
	{
		ROS_ERROR("Set properly an image width value. (param= 'width') ");
		return -1;
	}
	
	int height;
	if (nh.hasParam("height"))
	{
		nh.getParam("height", height);
	}
	else
	{
		ROS_ERROR("Set properly an image height value. (param= 'height') ");
		return -1;
	}
	
	/// Set the fixed fields of the info msg
	set_fixed_camera_info(&t_msgs.short_left_info, height,width, (char*)(ros::names::remap("/xb3_optical_frame")).c_str());
	set_fixed_camera_info(&t_msgs.short_right_info, height,width, (char*)(ros::names::remap("/xb3_optical_frame")).c_str());
	set_fixed_camera_info(&t_msgs.wide_left_info, height,width, (char*)(ros::names::remap("/xb3_optical_frame")).c_str());
	set_fixed_camera_info(&t_msgs.wide_right_info, height,width, (char*)(ros::names::remap("/xb3_optical_frame")).c_str());

	/// Set the fixed fields of the image msgs Check http://www.ros.org/doc/api/sensor_msgs/html/msg/Image.html for all fields
	set_fixed_fields_image_msg(&t_msgs.short_left, height,width, (char*)"RGB8", 0,  (char*)(ros::names::remap( "/xb3_optical_frame")).c_str());
	set_fixed_fields_image_msg(&t_msgs.short_right, height,width, (char*)"RGB8", 0,  (char*)(ros::names::remap( "/xb3_optical_frame")).c_str());
	set_fixed_fields_image_msg(&t_msgs.wide_left, height,width, (char*)"RGB8", 0,  (char*)(ros::names::remap( "/xb3_optical_frame")).c_str());
	set_fixed_fields_image_msg(&t_msgs.wide_right, height,width, (char*)"RGB8", 0,  (char*)(ros::names::remap("/xb3_optical_frame")).c_str());

	/// Create the image transport and advertise the image and info msgs
	image_transport::ImageTransport it(nh);
	t_msgs.short_left_pub = it.advertiseCamera("short/left/image_raw", 1);
	t_msgs.short_right_pub = it.advertiseCamera("short/right/image_raw", 1);
	t_msgs.wide_left_pub = it.advertiseCamera("wide/left/image_raw", 1);
	t_msgs.wide_right_pub = it.advertiseCamera("wide/right/image_raw", 1);

   
	t_msgs.short_left_info_manager = new camera_info_manager::CameraInfoManager(ros::NodeHandle(nh, "short/left"),"short/left","package://xb3/calibrations/short_left.yaml");
	t_msgs.short_right_info_manager = new camera_info_manager::CameraInfoManager(ros::NodeHandle(nh, "short/right"),"short/right", "package://xb3/calibrations/short_right.yaml");
	t_msgs.wide_left_info_manager = new camera_info_manager::CameraInfoManager(ros::NodeHandle(nh, "wide/left"),"wide/left","package://xb3/calibrations/wide_left.yaml");
	t_msgs.wide_right_info_manager = new camera_info_manager::CameraInfoManager(ros::NodeHandle(nh, "wide/right"),"wide/right","package://xb3/calibrations/wide_right.yaml");


//while(nh.ok())
//{
	//ros::spinOnce();
//}

    cout<<"TT"<<endl;
	/// Initialize camera using libdc1394. Searches for cameras on the bus and connects to a stereo camera.
	init_camera(t_dc1394.d, &t_dc1394.stereoCamera,&t_dc1394.camera, t_dc1394.nThisCam );
	PFLN
	ROS_INFO("Connected to XB3 camera");
        
    cout<<"A1"<<endl;
    dc1394_feature_set_mode(t_dc1394.camera,DC1394_FEATURE_SHUTTER,DC1394_FEATURE_MODE_MANUAL);
    dc1394_feature_set_value(t_dc1394.camera,DC1394_FEATURE_SHUTTER,100);
            
    dc1394featureset_t features;
    dc1394_feature_get_all(t_dc1394.camera,&features);
    
    FILE *fd = fopen("/home/jorge/Desktop/conf.txt","w");
    dc1394_feature_print_all(&features,fd);
    
    fclose(fd);
        
	/// Allocate all the buffers. size of buffer for all images at mono8
	t_buffers.nBufferSize = t_dc1394.stereoCamera.nRows * t_dc1394.stereoCamera.nCols * t_dc1394.stereoCamera.nBytesPerPixel;
	t_buffers.pucDeInterlacedBuffer = new unsigned char[ t_buffers.nBufferSize ];
	t_buffers.pucRGBBuffer 		= NULL;
	t_buffers.pucGreenBuffer 	= NULL;

	t_imgs.left.create(cvSize(1280,960),CV_8UC3);
	t_imgs.center.create(cvSize(1280,960),CV_8UC3);
	t_imgs.right.create(cvSize(1280,960),CV_8UC3);

	t_imgs.left_640_480.create(cvSize(width,height),CV_8UC3);
	t_imgs.center_640_480.create(cvSize(width,height),CV_8UC3);
	t_imgs.right_640_480.create(cvSize(width,height),CV_8UC3);

	//allocation color processing buffers
	if ( t_dc1394.stereoCamera.bColor )
	{
		t_buffers.pucRGBBuffer 		= new unsigned char[ 3 * t_buffers.nBufferSize ];
		t_buffers.pucGreenBuffer 		= new unsigned char[ t_buffers.nBufferSize ];
	}
	else
	{
		ROS_ERROR("This driver works only with color stereo cameras");
		signal_handler(SIGINT);
	}
	
	cout<<"Hf"<<endl;

	/// Create windows for showing the image if in debug
	if (t_flags.debug)
	{
		cvNamedWindow("left",CV_WINDOW_AUTOSIZE);
		cvNamedWindow("center",CV_WINDOW_AUTOSIZE);
		cvNamedWindow("right",CV_WINDOW_AUTOSIZE);
	}


		

	dynamic_reconfigure::Server<xb3::xb3Config> srv;
	dynamic_reconfigure::Server<xb3::xb3Config>::CallbackType f;
	f = boost::bind(&reconfig,  _1, _2);
	srv.setCallback(f);

	static tf::TransformBroadcaster br;
	tf::Transform transform;
	transform.setOrigin( tf::Vector3(0,-1, 0.0) );
	transform.setRotation( tf::Quaternion(0, 0, 0) );

    cout<<"tf broadcast"<<endl;

    while(1)
    {
        cout<<"get images"<<endl;
        // get the images from the capture buffer and do all required processing
        extractImagesColorXB3( &t_dc1394.stereoCamera, DC1394_BAYER_METHOD_HQLINEAR, t_buffers.pucDeInterlacedBuffer, t_buffers.pucRGBBuffer, t_buffers.pucGreenBuffer, &t_buffers.pucRightRGB,   &t_buffers.pucLeftRGB, &t_buffers.pucCenterRGB);
            
        cout<<"get current time"<<endl;
            //Get current time
        ros::Time stamp=ros::Time::now();

        copy_img_buffer_to_cvMat(t_buffers.pucRGBBuffer, t_imgs.left, t_imgs.center, t_imgs.right);

        
        if (width==640 && height==480)
        {
        cv::pyrDown(t_imgs.left, t_imgs.left_640_480);
        cv::pyrDown(t_imgs.right, t_imgs.right_640_480);
        cv::pyrDown(t_imgs.center, t_imgs.center_640_480);
        }
        else if (width==1280 && height==960)
        {
            t_imgs.left_640_480=t_imgs.left;
            t_imgs.right_640_480=t_imgs.right;
            t_imgs.center_640_480=t_imgs.center;
        }

        cout<<"copy and publish"<<endl;
        
        /// In ROS I think the reference is the left camera.
        //http://www.ros.org/wiki/stereo_image_proc
        
        //short_left is actually xb3 left camera
        copy_pixels_to_image_msg_data(t_imgs.left_640_480.ptr<unsigned char>(0), &t_msgs.short_left, height*width*3);

        //short_right is actually xb3 center camera
        copy_pixels_to_image_msg_data(t_imgs.center_640_480.ptr<unsigned char>(0), &t_msgs.short_right, height*width*3);
            
        //wide_left is actually xb3 left camera
        copy_pixels_to_image_msg_data(t_imgs.left_640_480.ptr<unsigned char>(0), &t_msgs.wide_left, height*width*3);

        //wide_right is actually xb3 right camera
        copy_pixels_to_image_msg_data(t_imgs.right_640_480.ptr<unsigned char>(0), &t_msgs.wide_right, height*width*3);


        t_msgs.short_left_info = t_msgs.short_left_info_manager->getCameraInfo();
        t_msgs.short_right_info = t_msgs.short_right_info_manager->getCameraInfo();

        t_msgs.wide_left_info = t_msgs.wide_left_info_manager->getCameraInfo();
        t_msgs.wide_right_info = t_msgs.wide_right_info_manager->getCameraInfo();
        
        t_msgs.short_left_info.header.frame_id =  ros::names::remap("/xb3_optical_frame");
        t_msgs.short_right_info.header.frame_id = ros::names::remap("/xb3_optical_frame");
        t_msgs.wide_left_info.header.frame_id = ros::names::remap("/xb3_optical_frame");
        t_msgs.wide_right_info.header.frame_id = ros::names::remap("/xb3_optical_frame");

        t_msgs.short_left_pub.publish(t_msgs.short_left, t_msgs.short_left_info, stamp);
        t_msgs.short_right_pub.publish(t_msgs.short_right, t_msgs.short_right_info, stamp);
        t_msgs.wide_left_pub.publish(t_msgs.wide_left, t_msgs.wide_left_info, stamp);
        t_msgs.wide_right_pub.publish(t_msgs.wide_right, t_msgs.wide_right_info, stamp);

        cout<<"published"<<endl;
        if (t_flags.debug) //copy to cvimage and display in highgui
        {
            cv::imshow("left",t_imgs.left_640_480);
            cv::imshow("center",t_imgs.center_640_480);
            cv::imshow("right",t_imgs.right_640_480);
        }

        //br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/world", "/xb3_optical_frame"));

        ros::spinOnce(); //spin ros
        
        if (t_flags.debug)
        {
            char k=cvWaitKey(40);
            if(k=='q') break;
        }
    }

	clean_dc1394();

    return 0;
}

#endif
/**
*@}
*/
