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
 * \file
 * \brief Single planar laser scan generator
 * \author Rui Azevedo
 * \version v0
 * \date 2014-05-09
 */

#include <ros/ros.h>
#include <laser_geometry/laser_geometry.h>

class LaserscanToPointcloud
{
    public:

        LaserscanToPointcloud()
        {
        ros::NodeHandle nh_;

        scan_sub_ = nh_.subscribe("/laser_1/scan", 1, &LaserscanToPointcloud::scanCallback, this); /*/snr/las/2/scan*/
        point_cloud2_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/scan_cloud",1);
        }

        void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
        {
        sensor_msgs::PointCloud2 cloud2;
        projector_.projectLaser(*scan_in, cloud2, 60); //projectLaser(const sensor_msgs::LaserScan& scan_in, sensor_msgs::PointCloud2& cloud_out, double range_cutoff)   
        
                                            //         Parameters:
                                            //              scan_in     The input laser scan
                                            //              cloud_out   The output point cloud
                                            //              range_cutoff    An additional range cutoff which can be applied which is more limiting than max_range in the scan. 
        point_cloud2_pub_.publish(cloud2);
        }

    protected:
        
        ros::Subscriber scan_sub_;
        ros::Publisher point_cloud2_pub_;

        laser_geometry::LaserProjection projector_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "laserscan_to_pointcloud_node");

    LaserscanToPointcloud ls;

    ros::spin();
}