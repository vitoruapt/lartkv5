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
 * \file config_cam.cpp
 * \author Ricardo Morais <ricardo.morais@ua.pt>
 * \date May , 2013
 *
 * This module subscribes the camera_info and the tf and creates a output file with the camera configurations
 */

#include <iostream> 
#include <stdio.h> 
#include <ros/ros.h>	
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
#include <signal.h>
#include <fstream>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <ros/package.h>
#include <math.h>
using namespace std;

/**
 *  intersept Ctrt+C handler
 */
void sighandler(int sig)
{
	ROS_ERROR("Signal %d caught...",sig);
	cout<<"Shuting down road_recognition"<<endl;
	exit(0);
}

/**
 * Callback for the message received
 */
void Callback(const sensor_msgs::CameraInfoPtr& cam_info)
{
	tf::TransformListener lt;
	tf::StampedTransform trans_cam;
	
	string target_ref = "/atc/vehicle/center_bumper";
	string ref = cam_info->header.frame_id;
	
	/* Obter as transformações dos referenciais */
	try
	{
		lt.lookupTransform( ref.c_str() , target_ref.c_str() , ros::Time(0), trans_cam);
		ROS_INFO("Found");
	}
	catch (tf::TransformException ex)
	{
		ROS_INFO("Transformation not found:  entered catch for 1.5 seconds!");
		if(lt.waitForTransform( ref.c_str() , target_ref.c_str(),ros::Time(0), ros::Duration(1.5)))
		{
			try
			{
				lt.lookupTransform( ref.c_str() , target_ref.c_str() , ros::Time(0), trans_cam);
				ROS_INFO("Found");
			}
			catch (tf::TransformException ex)
			{
				ROS_ERROR("Transformation not found: tansforrmation not found after waiting 1.5 seconds\n.%s",ex.what());
			}
		}
		else
		{
			ROS_ERROR("Transformation not found: Could not find valid transform after waiting 1.5 seconds\n.%s",ex.what());
			exit(0);
		}
		
	};
	
	double R,P,Y;
	tf::Quaternion q;
	q = trans_cam.getRotation(); 
	double height = trans_cam.getOrigin().z() * 1000;
	/* convert from quaternion to RPY */
	tf::Matrix3x3(q).getRPY(R, P, Y);
	
	/* Show camera configurations*/
	cout<<"---> Camera configurations"<<endl;
	cout<<"imge size "<<endl<<"\t"<<cam_info->height<<" x "<<cam_info->width<<endl;
	cout<<"distortion_model:"<<endl<<"\t"<<cam_info->distortion_model<<endl;
	cout<<"focal lengthstrans_cam"<<endl<<"\t fx ="<<cam_info->K[0]<<"  fy = "<<cam_info->K[4]<<endl;
	cout<<"principal point"<<endl<<"\t cx ="<<cam_info->K[2]<<"  cy = "<<cam_info->K[5]<<endl;	
	cout<<"cameraHeight = "<<endl;
	cout<<"pitch ="<<P<<endl;
	cout<<"yaw = "<<Y<<endl;
	
	/* Converter de radianos para graus */
	P = (P + (M_PI/2)) * ( 180/M_PI );
	Y = (Y + (M_PI/2)) * ( 180/M_PI );
	
	/* Save camera configuration do file */
	/* Abrir o ficheiro */
	std::string file_path=ros::package::getPath("caltech_lanes")+"/config/CameraInfo.conf";
	cout<<"file_path:"<<file_path<<endl;
	ofstream myfile(file_path.c_str(),ios::trunc);
	if (myfile.is_open())
	{
		/* Escrever as definições da camera dectro do ficheiro */
		myfile<<"\n# focal length\n";
		myfile<<"focalLengthX "<<cam_info->K[0]<<"\n";
		myfile<<"focalLengthY "<<cam_info->K[4]<<"\n";
		myfile<<"\n# optical center \n";
		myfile<<"opticalCenterX "<<cam_info->K[2]<<"\n";
		myfile<<"opticalCenterY "<<cam_info->K[5]<<"\n";
		myfile<<"\n# height of the camera in mm\n";
		myfile<<"cameraHeight "<<height<<"\n";
		myfile<<"\n# pitch of the camera\n";
		myfile<<"pitch "<<P<<"\n";
		myfile<<"\n# yaw of the camera\n";
		myfile<<"yaw "<<Y<<"\n";
		myfile<<"\n# imag width and height\n";
		myfile<<"imageWidth "<<cam_info->width<<"\n";
		myfile<<"imageHeight "<<cam_info->height<<"\n";
		/* fechar o ficheiro */
		myfile.close();
		/* Configurações gravadas com sucesso */
		cout<<"---> Camera configurations Updated"<<endl;
		exit(0); /* terminar o programa */
	}
	else 
	{
		cout << "Unable to open file"<<endl;
		exit(0);
	}	
}

/**
 * this function updates the camera parameter whem is called
 */
int main(int argc, char **argv)
{
	ros::init(argc, argv, "camera_configurations");
	
	string messages;
	ros::NodeHandle nh("~");
	/* obter os parametros de entrada do launch file */
	nh.param("cam_info",messages,string("/snr/scam/wide/left/camera_info"));
// 	nh.param("ref",ref,string("/atc/camera/xb3/left") );
	/* parametros subscritos do launch file */
	cout<<"cam_info: "<<messages<<endl;
// 	cout<<"ref: "<<ref<<endl;
	/* Obter as definições das cameras do Atlas e guardar para ficheiro */
	signal(SIGINT, &sighandler);
	ROS_INFO("messages = %s",messages.c_str());
	ros::Subscriber sub = nh.subscribe(messages.c_str(), 1, Callback);
	ros::spin();
	return 0;
}
