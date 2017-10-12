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
 * @file follow_pedestrian.cpp
 * @brief Planar scan generator
 * @author Miguel Oliveira
 * @version v0
 * @date 2012-02-29
 */

#include "follow_pedestrian.h"


class FollowPedestrian
{
    public:
        typedef enum
        {
            SEARCHING,TRACKING,TRACKING_NOT_SAFE,TARGET_LOST,INITIALISE   
        }enum_state;
        
        typedef enum 
        {
            SEARCHING_2_TRACKING=88,
            SEARCHING_2_TRACKING_NOT_SAFE,
            TRACKING_2_TARGET_LOST,
            TRACKING_2_TRACKING_NOT_SAFE,
            TRACKING_NOT_SAFE_2_TRACKING,
            TRACKING_NOT_SAFE_2_TARGET_LOST,
            NONE
        }t_transitions;
            
        class SearchArea
        {   
            public:
                double x;
                double y;
                double radius;
        };

        class SafetyZone
        {
            public:
                double xmin,xmax;
                double ymin,ymax;
        };

        class Targets
        {
            public:
                pcl::PointCloud<pcl::PointXYZ> position;
                pcl::PointCloud<pcl::PointXYZ> velocity;
                std::vector<int> id;
        };

        class Status
        {
            public:
                enum_state state;
                enum_state previous_state;
                int target_id;
                double current_angle,current_distance;
                double current_x,current_y,current_z;
                geometry_msgs::Quaternion current_q;

                bool target_found;          
                int last_id;

                bool CONTROL_CAR_SPEED;

                double direction;
                ros::Time tic_tracking_not_safe;
                ros::Time tic_searching;
                ros::Time tic_tracking;
                ros::Time tic_last_sound;
                ros::Time tic_last_itsnotsafe;
                bool reset_tracking;
        };
        
        void createMarkers(std::vector<visualization_msgs::Marker>& marker_vec)
        {
            visualization_msgs::Marker marker;
            geometry_msgs::Point p;
            static ros::Time trigger_blink=ros::Time::now();
            static bool flip=false;

            //Trigger zone cylinder
            marker.header.frame_id = "/atc/vehicle/ground";
            marker.header.stamp = ros::Time();
            marker.ns = "trigger_zone";
            marker.id = 0;
            marker.type = visualization_msgs::Marker::CYLINDER;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = search_area.x;
            marker.pose.position.y = search_area.y;
            marker.pose.position.z = 0;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 1;
            marker.scale.y = 1;
            marker.scale.z = 0.1;
            marker.color.a = 1.0;

            if ((ros::Time::now()-trigger_blink).toSec() > 1)
            {
                flip=!flip;
                trigger_blink = ros::Time::now();
            }

            if (flip)
            {
                marker.color.r = 0.5;
                marker.color.g = 0.5;
                marker.color.b = 0.0;
            }
            else
            {
                marker.color.r = 1;
                marker.color.g = 1;
                marker.color.b = 0.0;
            }
            marker_vec.push_back(marker);

            //Trigger zone Text
            marker.header.frame_id = "/atc/vehicle/ground";
            marker.header.stamp = ros::Time();
            marker.ns = "trigger_text";
            marker.id = 0;
            marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = search_area.x;
            marker.pose.position.y = search_area.y;
            marker.pose.position.z = 0.3;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = search_area.radius;
            marker.scale.y = search_area.radius;
            marker.scale.z = 0.3;
            marker.color.a = 1.0;
            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            if (status.state==SEARCHING)
                marker.text = "SEARCHING: Waiting for someone";
            else if (status.state==TRACKING)
            {
                char str[1024];
                sprintf(str,"TRACKING: Following target id %d",status.target_id);
                marker.text = str;
            }
            else if (status.state==TRACKING_NOT_SAFE)
            {
                char str[1024];
                sprintf(str,"TRACKING_NOT_SAFE: Waiting for clearance to follow target id %d",status.target_id);
                marker.text = str;
            }

            marker_vec.push_back(marker);

            //Safety zone lines
            marker.header.frame_id = "/atc/vehicle/ground";
            marker.header.stamp = ros::Time();
            marker.ns = "safety_zone";
            marker.id = 0;
            marker.type = visualization_msgs::Marker::LINE_STRIP;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = 0;
            marker.pose.position.y = 0;
            marker.pose.position.z = 0;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.2;
            marker.scale.y = 1;
            marker.scale.z = 0.2;
            marker.color.a = 1.0;

            if (!isSafeUsingLasers())//replace here by the danger flag
            {
                marker.color.r = 1;
                marker.color.g = 0;
                marker.color.b = 0;
            }
            else
            {
                marker.color.r = 0;
                marker.color.g = 1;
                marker.color.b = 0;
            }

            p.x = safety_zone.xmin; p.y = safety_zone.ymin; p.z = 0;
            marker.points.push_back(p);

            p.x = safety_zone.xmin; p.y = safety_zone.ymax; p.z = 0;
            marker.points.push_back(p);
            marker.points.push_back(p);

            p.x = safety_zone.xmax; p.y = safety_zone.ymax; p.z = 0;
            marker.points.push_back(p);
            marker.points.push_back(p);

            p.x = safety_zone.xmax; p.y = safety_zone.ymin; p.z = 0;
            marker.points.push_back(p);
            marker.points.push_back(p);

            p.x = safety_zone.xmin; p.y = safety_zone.ymin; p.z = 0;
            marker.points.push_back(p);

            marker_vec.push_back(marker);

            //send the obj model for the pedestrian marker
            marker.id = 1;  
            marker.header.stamp = ros::Time::now(); 
            marker.type = visualization_msgs::Marker::MESH_RESOURCE;    
            marker.action = visualization_msgs::Marker::ADD;    
            marker.ns = "pedestrian";
            marker.mesh_use_embedded_materials = 1; 
            marker.header.frame_id = "/atc/vehicle/ground";
            marker.scale.x = .4;
            marker.scale.y = .4;
            marker.scale.z = .4;
            marker.color.r = 0.2;
            marker.color.g = 0.3;
            marker.color.b = 0.4;
            marker.color.a = 1;
            marker.mesh_resource = "package://wrapper_collada/models/decisive_woman.obj";
            marker.header.stamp = ros::Time::now(); 

            if (status.state==TRACKING || status.state == TRACKING_NOT_SAFE)
            {
                marker.pose.position.x = status.current_x;
                marker.pose.position.y = status.current_y;
                marker.pose.position.z = status.current_z;
                marker.pose.orientation = status.current_q;
            }
            else
            {
                marker.pose.position.x = 0;
                marker.pose.position.y = 0;
                marker.pose.position.z = 3000;
            }

            marker_vec.push_back(marker);
        }
        
        bool isSafe(const SafetyZone zone, const Targets targets)
        {
            
            for(size_t i=0; i<targets.position.points.size(); ++i)
            {
                double x=targets.position[i].x;
                double y=targets.position[i].y;

                if(x<zone.xmax && x>zone.xmin && y<zone.ymax && y>zone.ymin)
                    return false;
            }

            return true;
        }
        
        FollowPedestrian(ros::NodeHandle nh_)
        :nh(nh_),
        transform_listener(nh,ros::Duration(10))
        {
            target_publisher = nh.advertise<sensor_msgs::PointCloud2>("/target", 1);
            
//             command_publisher = 
            
            marker_publisher = nh.advertise<visualization_msgs::MarkerArray>( "/follow_pedestrian_markers", 0 );
            
            targets_subscriber = nh.subscribe("/mtt/targets", 1, &FollowPedestrian::targetsHandler,this);
            
            search_area.x = 6.0;
            search_area.y = 0.0; 
            search_area.radius = 0.6;
    
            safety_zone.xmax = 4.0;
            safety_zone.xmin = 3.5;
            safety_zone.ymax = 0.5;
            safety_zone.ymin = -0.5;
            
            a_h = drand48();
            
            FLAG_LASER_SAFE_TO_DRIVE=false;
            
            status.state=INITIALISE;
            status.tic_last_sound = ros::Time::now();
        }

        void targetsHandler(const mtt::TargetList& targetlist)
        {
            pcl::PointCloud<pcl::PointXYZ> pc_local;
            pcl::PointCloud<pcl::PointXYZ> v_local;
            
            std::string frame_id;
            for(uint i=0;i<targetlist.Targets.size();i++)
            {
                pcl::PointXYZ p;
                
                p.x = targetlist.Targets[i].pose.position.x;
                p.y = targetlist.Targets[i].pose.position.y;
                p.z = targetlist.Targets[i].pose.position.z;
                
                pc_local.push_back(p);
                
                pcl::PointXYZ vel;
             
                vel.x = targetlist.Targets[i].velocity.linear.x;
                vel.y = targetlist.Targets[i].velocity.linear.y;
                vel.z = targetlist.Targets[i].velocity.linear.z;
                
                v_local.push_back(vel);
                
                frame_id=targetlist.Targets[i].header.frame_id;
            }
            
            tf::StampedTransform transform;
            try
            {
                transform_listener.lookupTransform(frame_id, "/atc/vehicle/rear_axis" , ros::Time(0), transform);
            }
            catch (tf::TransformException ex)
            {
                ROS_ERROR("Could find valid transform\n.%s",ex.what());
            }

            // Transform points
            pcl_ros::transformPointCloud(pc_local, targets.position, transform.inverse());
            targets.position.header.frame_id = "/atc/vehicle/rear_axis";

            //Copy velocity
            targets.velocity = v_local;

            targets.id.erase(targets.id.begin(), targets.id.end());
            for (size_t i=0; i<targetlist.Targets.size(); ++i)
            {
                targets.id.push_back(targetlist.Targets[i].id);
            }

            //check safety zone
            FLAG_LASER_SAFE_TO_DRIVE = isSafe(safety_zone,targets);
            
            #if _USE_DEBUG_
                //ROS_INFO("Received mtt_targets id size=%ld pc size = %ld", targets.id.size(), pc.points.size());
            #endif
        }
        
        void foveationcontrolHome(void)
        {
            pcl::PointCloud<pcl::PointXYZ> pc;
            pcl::PointXYZ pt;   
            pt.x = 10; pt.y = 0;    pt.z = 5;
            pc.points.push_back(pt);
            sensor_msgs::PointCloud2 pcmsg_out;
            pcl::toROSMsg(pc, pcmsg_out);
            pcmsg_out.header.stamp = ros::Time::now();
            pcmsg_out.header.frame_id = "/atc/vehicle/rear_axis";
            target_publisher.publish(pcmsg_out);
        }
        
        void foveationcontrolSweeping(void)
        {
            static ros::Time tic=ros::Time::now();
            //  static int state=0;
            double x,y,z;
            pcl::PointXYZ pt;   

            if((ros::Time::now()-tic).toSec()>4.)
            {
                tic=ros::Time::now();

                x = (((double)rand())/(double)RAND_MAX);
                pt.x = x*10+15;

                y = (((double)rand())/(double)RAND_MAX);
                pt.y = y*20-10;

                z = (((double)rand())/(double)RAND_MAX);
                pt.z = z*2;

                pcl::PointCloud<pcl::PointXYZ> pc;
                pc.points.push_back(pt);
                sensor_msgs::PointCloud2 pcmsg_out;
                pcl::toROSMsg(pc, pcmsg_out);
                pcmsg_out.header.stamp = ros::Time::now();
                pcmsg_out.header.frame_id = "/atc/vehicle/rear_axis";
                target_publisher.publish(pcmsg_out);

            }
        }
 
        void foveationcontrol(const Status& status)
        {
            pcl::PointCloud<pcl::PointXYZ> pc;
            pcl::PointXYZ pt;   
            pt.x = status.current_x; pt.y = status.current_y; pt.z = 0.8;
            pc.points.push_back(pt);
            sensor_msgs::PointCloud2 pcmsg_out;
            pcl::toROSMsg(pc, pcmsg_out);
            pcmsg_out.header.stamp = ros::Time::now();
            pcmsg_out.header.frame_id = "/atc/vehicle/rear_axis";
            target_publisher.publish(pcmsg_out);
        }
        
        void playSound(int sit, int mandatory, double num_h=5)
        {
            //always 0 - 5 hipothesys
            unsigned int b_h;   

            if (mandatory<0)
            {
                status.tic_last_sound=ros::Time::now();

            }else if ((ros::Time::now()-status.tic_last_sound).toSec() >mandatory)
            {
                status.tic_last_sound=ros::Time::now();
            }
            else
            {
                return;
            }

            b_h = round((double)(a_h*(double)num_h));
            char text[1024];
            sprintf(text,"sit%d_hip%d",sit,b_h);


            std::string path= ros::package::getPath("atlas_sound_player")+ "/Recordings/GoogleLady_ros/" + text + ".wav";
            sound_client.playWave(path);
            sleepok(0.1);

            printf("playing sound %s\n",text);
            //sleep(3);

        }

        void sleepok(int t)
        {
            if (nh.ok())
                sleep(t);
        }

        bool searchForNewTarget(const Targets& targets, Status& status,const SearchArea& area)
        {
            double d;

            for(size_t i=0; i<targets.id.size(); ++i)
            {
                d=sqrt( (targets.position[i].x - area.x)*(targets.position[i].x - area.x)+
                        (targets.position[i].y - area.y)*(targets.position[i].y - area.y));

                if(d > area.radius)
                    continue;


                ROS_INFO("Search area x=%f y=%f rad=%f \nFound target %d x=%f y=%f inside search zone %f",area.x, area.y, area.radius, targets.id[i],targets.position[i].x, targets.position[i].y, d);
                ///If the object meets all requeriments we put his information on the current target
                status.target_id = targets.id[i];
                status.current_x = targets.position[i].x;
                status.current_y = targets.position[i].y;
                status.current_z = targets.position[i].z;

                return true;
            }

            return false;
        }

        bool isSafeUsingLasers(void)
        {
            static ros::Time tic = ros::Time::now();

            if(!FLAG_LASER_SAFE_TO_DRIVE)
                tic = ros::Time::now();

            if((ros::Time::now() - tic).toSec() > 5.5)
            {
                return 1;
            }
            else
            {
                return 0;
            }
        }
        
        bool getNewTargetInformation(Targets& targets,Status& status)
        {
            for(size_t i=0; i<targets.id.size(); ++i)
            {
                if(status.target_id!= targets.id[i])
                    continue;

                status.target_id = targets.id[i];
                status.current_x = targets.position[i].x;
                status.current_y = targets.position[i].y;
                status.current_z = targets.position[i].z;

                status.current_q = tf::createQuaternionMsgFromYaw(atan2(targets.velocity[i].y, targets.velocity[i].x)); 

                return true;
            }

            return false;
        }
        
        void printStatus()
        {
            printf("*****************************\n");
            switch(status.state)
            {   
                case SEARCHING:
                    printf("** Searching for new target\n");
                    break;

                case TRACKING:
                    printf("** Tracking target %d\n",status.target_id);
                    break;

                case TARGET_LOST:
                    printf("** Target lost\n");
                    break;

                case INITIALISE:
                    printf("** Initialising\n");
                    break;
                case TRACKING_NOT_SAFE:
                    printf("** Tracking_not_safe\n");
                    break;
            }   
        }

        void stateMachine(void)
        {
            a_h = (((double)rand())/(double)RAND_MAX);

            printf("status.target_found=%d\n",status.target_found);
            printf("status.last_id=%d\n",status.last_id);

            /// Tracking state machine
            status.target_found=false;
            int should_move=false;
            double dist_to_target=1000;

            printStatus();
            /// ---------------------------------------------------------------------------
            /// STATE MACHINE
            /// ---------------------------------------------------------------------------
            switch(status.state)
            {
                /// ---------------------------------------------------------------------------
                case INITIALISE: /// STATE INITIALIZE
                    /// ---------------------------------------------------------------------------

                    ///--- TIMER UPDATES --- //

                    /// --- STATE ACTIONS ---//
                    //bring the ptu to home position
                    foveationcontrolHome(); 

                    status.direction = DIRECTION_IN_FRONT;

                    /// --- TRANSITIONS ---//
                    status.previous_state=INITIALISE; //unconditional transition to searching state
                    status.state=SEARCHING;
                    /// --- TRANSITION ACTIONS ---//

                    break;
                    
                    /// ---------------------------------------------------------------------------
                case SEARCHING: /// STATE SEARCHING
                    /// ---------------------------------------------------------------------------

                    ///--- TIMER UPDATES --- //
                    if (status.previous_state!=SEARCHING) //update start of SEARCHING state tic
                    {
                        status.tic_searching=ros::Time::now();
                    }

                    /// --- STATE ACTIONS ---//
                    //search for a target
                    
                    status.target_found = searchForNewTarget(targets,status,search_area);
                    printf("status.target_found=%d\n",status.target_found);
                    status.last_id=status.target_id;

                    playSound(4, 10);
                    foveationcontrolSweeping();

                    /// --- TRANSITIONS ---//
                    if(status.target_found && !isSafeUsingLasers()) //if a target but safe zone not clear we transit to TRACKING_NOT_SAFE
                    {
                        status.previous_state=SEARCHING;
                        status.state=TRACKING_NOT_SAFE;
                        /// --- TRANSITION ACTIONS ---//
                        playSound(2, 10);
                    }
                    else if(status.target_found) //if a target is found we transit to tracking
                    {
                        status.previous_state=SEARCHING;
                        status.state=TRACKING;
                        /// --- TRANSITION ACTIONS ---//
                        playSound(0, -1);
                    }
                    break;
                    /// ---------------------------------------------------------------------------
                case TRACKING: /// STATE TRACKING
                    /// ---------------------------------------------------------------------------
                    ///--- TIMER UPDATES --- //
                    if (status.previous_state!=TRACKING) //update start of SEARCHING state tic
                    {
                        status.tic_tracking=ros::Time::now();
                    }

                    /// --- STATE ACTIONS ---//
                    foveationcontrol(status);//do foveationcontrol
                    status.target_found=getNewTargetInformation(targets,status);

                    //setting up the high command message
                    playSound(1, 15);

                    //send pedestrian marker
                    

                    status.direction = atan2(status.current_y,status.current_x);
        //          set_high_command_msg_direction_speed(command_msg, status.direction, SPEED_MOVE_SLOW);
        //          set_high_command_msg_lights(command_msg, LIGHT_OFF, LIGHT_ON, LIGHT_OFF, LIGHT_OFF, LIGHT_ON);

                    /// --- TRANSITIONS ---//
                    if(status.reset_tracking==true)//user ordered a reset of tracking
                    {
                        status.previous_state=TRACKING;
                        status.state=SEARCHING;
                        status.reset_tracking = false;
                        /// --- TRANSITION ACTIONS ---//
                        playSound(5, -1);
                    }
                    else if(status.target_found==false || status.current_x<3.0)//target was lost
                    {
                        status.previous_state=TRACKING;
                        status.state=TARGET_LOST;
                        /// --- TRANSITION ACTIONS ---//
                        playSound(5, -1);

                    }else if (!isSafeUsingLasers())
                    {
                        status.previous_state=TRACKING;
                        status.state=TRACKING_NOT_SAFE;
                        /// --- TRANSITION ACTIONS ---//
                        if (dist_to_target<6)
                            playSound(6, -1);
                        else
                            playSound(2, -1);
                    }
                    break;
                    /// ---------------------------------------------------------------------------
                case TRACKING_NOT_SAFE: /// TRACKING_NOT_SAFE
                    /// ---------------------------------------------------------------------------
                    ///--- TIMER UPDATES --- //
                    if (status.previous_state!=TRACKING_NOT_SAFE) //update start of SEARCHING state tic
                    {
                        status.tic_tracking_not_safe=ros::Time::now();
                    }

                    /// --- STATE ACTIONS ---//
                    status.target_found = getNewTargetInformation(targets,status);
                    foveationcontrol(status);//do foveationcontrol

                    if (status.target_found)
                    {
                        dist_to_target = sqrt(pow(status.current_x-2.7,2) + pow(status.current_y,2));
                        printf("status.target_id=%d\n",status.target_id);
                        printf("dist_to_target=%f\n",dist_to_target);
                        if (dist_to_target<4)
                            playSound(6, 10);
                        else
                            playSound(2, 10);
                    }


                    //setting up the high command message
                    status.direction = atan2(status.current_y,status.current_x);
        //          set_high_command_msg_direction_speed(command_msg, status.direction, SPEED_STOP);
        //          set_high_command_msg_lights(command_msg, LIGHT_OFF, LIGHT_ON, LIGHT_ON, LIGHT_ON, LIGHT_ON);

                    /// --- TRANSITIONS ---//
                    if(status.reset_tracking==true)//user ordered a reset of tracking
                    {
                        status.previous_state=TRACKING_NOT_SAFE;
                        status.state=SEARCHING;
                        status.reset_tracking = false;
                        /// --- TRANSITION ACTIONS ---//

                    }
                    else if(status.target_found==false || status.current_x<3.0)//target was lost
                    {
                        status.previous_state=TRACKING_NOT_SAFE;
                        status.state=TARGET_LOST;
                        /// --- TRANSITION ACTIONS ---//
                        playSound(5, -1);

                    }
                    else if (isSafeUsingLasers())
                    {
                        int tmp = status.previous_state;
                        status.previous_state=TRACKING_NOT_SAFE;
                        status.state=TRACKING;
                        /// --- TRANSITION ACTIONS ---//
                        if (tmp!=SEARCHING)
                        {
                            playSound(3, -1);
                        }
                        else
                        {
                        }
                    }

                    break;
                    /// ---------------------------------------------------------------------------
                case TARGET_LOST: /// TARGET_LOST
                    /// ---------------------------------------------------------------------------
                    ///--- TIMER UPDATES --- //


                    /// --- STATE ACTIONS ---//         
                    foveationcontrolHome();

                    //setting up the high command message
        //          set_high_command_msg_direction_speed(command_msg, command_msg.steering_wheel, SPEED_STOP);
        //          set_high_command_msg_lights(command_msg, LIGHT_OFF, LIGHT_OFF, LIGHT_OFF, LIGHT_OFF, LIGHT_ON);


                    /// --- TRANSITIONS ---//           
                    status.previous_state=TARGET_LOST; 
                    status.state=SEARCHING; //unconditional transition to SEARCHING


                    break;
                    /// ---------------------------------------------------------------------------
                default: ///UNKNOWN STATE
                    /// ---------------------------------------------------------------------------

                    ROS_ERROR("WARNING DANGER unknown state in state machine abort demonstration");
                    break;
            }
        }

        bool loop()
        {
            ros::Rate loop_rate(10);
            
            while (nh.ok())
            {
                ros::spinOnce();

                //Run the state machine
                stateMachine();

                //   ___________________________________
                //   |                                 |
                //   |        DRAW_IN_RVIZ             |
                //   |_________________________________| 
                visualization_msgs::MarkerArray marker_msg;
                createMarkers(marker_msg.markers);
                marker_publisher.publish(marker_msg);

                loop_rate.sleep();  
            }
        }
        
        bool FLAG_LASER_SAFE_TO_DRIVE;
        double a_h;
        
        SearchArea search_area;
        SafetyZone safety_zone;
        Targets targets;
        Status status;
    
        ros::NodeHandle nh;
        tf::TransformListener transform_listener;
        sound_play::SoundClient sound_client;
        ros::Publisher target_publisher;
        ros::Publisher command_publisher;
        ros::Publisher marker_publisher;
        ros::Subscriber targets_subscriber;
};


int main(int argc, char **argv)
{
	ros::init(argc, argv, "planar_pc_generator_node");
	ros::NodeHandle n("~");
    
    FollowPedestrian follow_pedestrian(n);
    
    follow_pedestrian.loop();
}