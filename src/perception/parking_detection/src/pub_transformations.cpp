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
 * @brief Transformation between the kinect axis frame and the vehicle frame
 * @author Joel
 * @date 5-May-2012
 */

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Float64.h>
#include <math.h>

/** 
 * @brief In function of the tilt angle, set the kinect position relative to the car frame
 * 
 * @return int
 */
int main(int argc, char** argv){
	ros::init(argc, argv, "pub_transformations");
	ros::NodeHandle n;
	tf::TransformBroadcaster broadcaster;
	ros::Rate r(10);
	
    float alpha=(38.5)*(3.1415/180);// 38.5 is obtained by multiplying the tillt angle by -1


//Had to put then following alternative for FUERTE/GROOVY. V. Santos, 27-Mai-2013
#if ROS_VERSION_MINIMUM(1, 8, 0)   //At least FUERTE
	tf::Transform transform(tf::Matrix3x3(0,-1,0, cos(alpha),0,sin(alpha), -sin(alpha),0,cos(alpha)), tf::Vector3(0.236, -0.05, 0.68));
#else    //earlier releases
	tf::Transform transform(btMatrix3x3(0,-1,0, cos(alpha),0,sin(alpha), -sin(alpha),0,cos(alpha)), btVector3(0.236, -0.05, 0.68));
#endif

	ros::spinOnce();
	while(n.ok())
	{
	  broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(),"/vehicle_odometry", "/openni_camera")); 

	  r.sleep();
	  ros::spinOnce();
	}
}
