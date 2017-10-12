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
  \brief Nodelet that uses las3D_PointCloud class to generate PointCloud from 2D scan data and external shaft position.
 * This nodelet subscribes sensor_msgs::LaserScan, sensor_msgs::JointState, 
 * and projects the scan in to pointcloud, using the transform published by the pkg laser_rotate3D. 
 * The PointCloud is accumulated on the desired /ac_frame (see laser3D_pointcloud.lauch).
 \file main.cpp
 \author Diogo Matos
 \date June 2013
 **/

#include <laser_3d_pointcloud/laser_3d_pointcloud.h>

//GLOBAL VARIABLES
las3D_PointCloud *g_las3D;
int phis_count=0;
int firstscan=1;
ros::Time firstscan_stamp;
ros::Time phi1_stamp;
ros::Time phi2_stamp;
sensor_msgs::JointStatePtr first_state1(new sensor_msgs::JointState);
sensor_msgs::JointStatePtr first_state2(new sensor_msgs::JointState);

/**
 * \brief Callback from the JointState subscribed topic
 * \param[in] const sensor_msgs::JointState::ConstPtr&
 */

void state_cb (const sensor_msgs::JointState::ConstPtr& state_in)
{
	//only access corret joint state
	std::string joint_name = "/connect_laser_roof_rotating";
	if (joint_name.compare(state_in->name[0]) != 0)
		return;
	
	if (firstscan)
	{
		*first_state1=*first_state2;
		*first_state2=*state_in;
		
		phi1_stamp=phi2_stamp;
		phi2_stamp = state_in->header.stamp;
	
		phis_count++;
	}
			
	sensor_msgs::JointStatePtr new_state(new sensor_msgs::JointState);	
	*new_state=*state_in;
	
	if (!firstscan)//store the state joints in to vector
		g_las3D->joints.push_back(new_state); 	

}

/**
 * \brief Callback from the LaserScan subscribed topic
 * \param[in] const sensor_msgs::LaserScan::ConstPtr&
 */

void scan_cb(const sensor_msgs::LaserScan::ConstPtr& scan_in)
{	
	
	if (g_las3D->accumulation_mode==2 || g_las3D->accumulation_mode==3)
	{
		sensor_msgs::LaserScanPtr new_scan(new sensor_msgs::LaserScan);
		*new_scan=*scan_in;
		int error=0;
		sensor_msgs::PointCloud2 pcmsg_in;
		
		//transform the laser scan using the scan time stamps
		error=g_las3D->las3D_transformLaserScanToPointCloud(ros::names::remap("/ac_frame"), *new_scan,pcmsg_in,*g_las3D->p_listener); 
	
		if (error)
			return;
		
		pcl::PointCloud<pcl::PointXYZ> pc_in;
        pcl::PCLPointCloud2 pcl_pc;
        pcl_conversions::toPCL(pcmsg_in, pcl_pc);
        pcl::fromPCLPointCloud2(pcl_pc, pc_in);
    
		pcl::fromROSMsg(pcmsg_in,pc_in);
		
		//accumulate the laser scans
		g_las3D->accumulate_cloud(&pc_in);
		
		g_las3D->laser_count++;
		g_las3D->laserscan_arrived=true;	
		
	}	
	else if (g_las3D->accumulation_mode==1)
	{
		if (phis_count<2)
			return;
		
		if (firstscan)
		{
			firstscan_stamp = scan_in->header.stamp;
			
			if ( (firstscan_stamp>phi1_stamp && firstscan_stamp<phi2_stamp) && (phi1_stamp.toSec()>0))
			{
				firstscan=0;
				g_las3D->joints.push_back(first_state1); 
				g_las3D->joints.push_back(first_state2);
			}
			else
				return;
		}
		
		int error=0;
		
		sensor_msgs::LaserScanPtr new_scan(new sensor_msgs::LaserScan);
		*new_scan=*scan_in;
		
		g_las3D->scans.push_back(new_scan); //store the scans to a vector
		
		if(g_las3D->joints.size()<2) //not enough phis to process a scan
			return;
		
		while(g_las3D->joints.size()>=2 && g_las3D->scans.size()>0)
		{			
			int need_state=0;
			sensor_msgs::LaserScanPtr working_scan(new sensor_msgs::LaserScan);
			*working_scan=*g_las3D->scans[0]; //get the oldest scan	
			g_las3D->scans.erase(g_las3D->scans.begin()); 
			
			//state start
			sensor_msgs::JointStatePtr state_start(new sensor_msgs::JointState);	
			*state_start=*g_las3D->joints[0];						
			
			g_las3D->joints.erase(g_las3D->joints.begin());
			
			//state end
			sensor_msgs::JointStatePtr state_end(new sensor_msgs::JointState);	
			*state_end=*g_las3D->joints[0];		
			
			//if phi start is lost, get new scan
			while (working_scan->header.stamp<state_start->header.stamp)
			{			
				if (g_las3D->scans.size()>0)
					*working_scan=*g_las3D->scans[0]; 
				else	
				{
					need_state=1;
					break;
				}
				
				g_las3D->scans.erase(g_las3D->scans.begin()); 
							
			}
			//if scan is lost and has no relation with the latest phi, get the phi with relation with the scan
			while (working_scan->header.stamp>state_end->header.stamp)
			{
				g_las3D->joints.erase(g_las3D->joints.begin());
				
				if (g_las3D->joints.size()>0)
					*state_end=*g_las3D->joints[0];
				else	
				{
					need_state=1;
					break;
				}
			}
		
			if (need_state)
				return;
						
			sensor_msgs::PointCloud2 pcmsg_in;
			
			//transform the laser scan in to a point cloud using the external shaft information
			if (g_las3D->accumulation_mode==1) 
				error=g_las3D->las3D_transformLaserScanToPointCloud(ros::names::remap("/ac_frame"), *working_scan,pcmsg_in,*state_start,*state_end,*g_las3D->p_listener);
		
			if (error)
				return;
			
			pcl::PointCloud<pcl::PointXYZ> pc_in;
			pcl::fromROSMsg(pcmsg_in,pc_in);
			
			g_las3D->accumulate_cloud(&pc_in);

			g_las3D->laser_count++;
			g_las3D->laserscan_arrived=true;
			
// 			erase some old data
			if (g_las3D->laser_count>g_las3D->max_scans_accumulated)
				g_las3D->pc_accumulated.erase(g_las3D->pc_accumulated.begin(),g_las3D->pc_accumulated.begin()+g_las3D->cloud_size);
		}
	}
	
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "laser3D_pointcloud");
	ros::NodeHandle n("~");
	tf::TransformListener listener(n,ros::Duration(100));
	//Declaration of class
	las3D_PointCloud las3D;
	g_las3D=&las3D;
	
	g_las3D->p_listener=&listener;

	double output_freq;
	n.param("output_frequency", output_freq, 200.0);
	n.param("accumulation_mode", g_las3D->accumulation_mode, 1);
	n.param("max_scans_accumulated", g_las3D->max_scans_accumulated, 450);
	n.param("pointcloud_stamp", g_las3D->pointcloud_stamp,1);
	
	//Test the remapping of /ac_frame
	if (ros::names::remap("/ac_frame")=="/ac_frame")
	{
		ROS_ERROR("/ac_frame was not remapped. Aborting program.");	
		ros::shutdown();
	}

	//declare the subsriber
	ros::Subscriber sub_phiversion = n.subscribe ("/laserscan0", 100, scan_cb);
	ros::Subscriber sub_state = n.subscribe ("/joint_state", 100, state_cb);
	
	//declare the publisher
	ros::Publisher pub = n.advertise<sensor_msgs::PointCloud2>("/pc_out", 1);

	ros::Rate loop_rate(output_freq);

	while (n.ok())
	{
		ros::spinOnce();

		if (g_las3D->laserscan_arrived)
		{
			g_las3D->laserscan_arrived=false;			
			sensor_msgs::PointCloud2 pcmsg_out;
			pcl::toROSMsg(g_las3D->pc_accumulated, pcmsg_out);
			// set the time stamp of output pointcloud
			if (g_las3D->pointcloud_stamp)
				pcmsg_out.header.stamp = ros::Time::now();
			else
            {
                std::cout<<"The message stamp is missing!!, please solve!!"<<std::endl;
// 				pcmsg_out.header.stamp = g_las3D->pc_accumulated.header.stamp;
            }
#if _USE_DEBUG_
			ROS_INFO("Publishing pc_accumulated with %d points", (int)g_las3D->pc_accumulated.size());
#endif
			//publish the pointcloud
			pub.publish(pcmsg_out);

		}
		
		loop_rate.sleep();	
	}
	
}
