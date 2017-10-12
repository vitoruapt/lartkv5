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
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Float64.h>
#include <math.h>
tf::TransformBroadcaster* p_broadcaster;

/** 
 * @brief Generates a frame higher than the car frame, to publish the point cloud to the mtt
 * @param int
 * @param char**
 * @return int
 */

int main(int argc, char** argv)
{
	ros::init(argc, argv, "tf_generator_node");
	ros::NodeHandle n;
	tf::TransformBroadcaster broadcaster;
	p_broadcaster=& broadcaster;
	ros::Rate r(10);


	tf::Transform transform1(tf::Matrix3x3(1,0,0, 0,1,0, 0,0,1),
							tf::Vector3(0, 0, 0.10));
	double theta=0;
	int inc=0;
	while(n.ok())
	{
		theta=inc*(2*M_PI)/1000;
			
		
	  	p_broadcaster->sendTransform(tf::StampedTransform(transform1, ros::Time::now(),"/vehicle_odometry", "/tracking_frame1")); 
		r.sleep();
		ros::spinOnce();

	}
}
