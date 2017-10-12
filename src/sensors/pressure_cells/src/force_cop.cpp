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
 * @file force_cop.cpp
 * @author Emílio Estrelinha nº 38637 (emilio.estrelinha@ua.pt)
 * @brief Calculates the COP of both feet
 */


#include <ros/ros.h>
#include <std_msgs/Time.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>

#include <sys/types.h>
#include <unistd.h>
#include <stdlib.h>

#include <stdio.h>
#include <stdlib.h>

#include <pressure_cells/SenVal.h>
#include <pressure_cells/Cop.h>
#include <pressure_cells/arduino.h>


vector< geometry_msgs::PointStamped > p_origins_l(4), p_origins_r(4);
bool ok1, ok2;
ros::Publisher pub_cop1, pub_cop2;
pressure_cells::Cop COP1, COP2, COP;
double g_sen1[4], g_sen2[4];
ros::Publisher pub_viz;
ros::Publisher pub_cop;
bool g_viz=0, g_lev=0;
string tf_feet_l;
string tf_feet_r;


/**
* @fn void SenVal1CallBk ( const pressure_cells::SenVal & senval )
* Callback function fot subscriber of the 1st group of Load Cells
* @param a vector w/ the values of the Load Cells
* @return void
*/
void SenVal1CallBk ( const pressure_cells::SenVal & senval )
{
    g_sen1[0] = senval.sen1;
    g_sen1[1] = senval.sen2;
    g_sen1[2] = senval.sen3;
    g_sen1[3] = senval.sen4;
    
    COP1.copx=0;
    COP1.copy=0;
    
    for ( uint i = 0; i < 4; i++ )
    {
        COP1.copx += g_sen1[i] * p_origins_l[i].point.x;
        COP1.copy += g_sen1[i] * p_origins_l[i].point.y;
    }
    COP1.copx = COP1.copx / (g_sen1[0] + g_sen1[1] + g_sen1[2] + g_sen1[3]);
    
    COP1.copy= COP1.copy / (g_sen1[0] + g_sen1[1] + g_sen1[2] + g_sen1[3]);
    
    //// publish COP1
    COP1.header.stamp = ros::Time::now (  );
    pub_cop1.publish(COP1);
    //cout<<"COP1 X = "<<COP1.copx<<"\t COP1 Y = "<<COP1.copy<<endl;
    ok1 = 1;
    
    if(!g_viz) return;

    visualization_msgs::Marker Cop_l;

    // COP Left
    Cop_l.header.frame_id = tf_feet_l;
    Cop_l.header.stamp = ros::Time::now();
    Cop_l.ns = "Cop_l";
    Cop_l.id = 0;
    Cop_l.type = visualization_msgs::Marker::SPHERE;
    Cop_l.action = visualization_msgs::Marker::ADD;
    Cop_l.scale.x = 0.02;
    Cop_l.scale.y = 0.02;
    Cop_l.scale.z = 0.02;
    Cop_l.color.a = 1.0;
    Cop_l.color.r = 0.0;
    Cop_l.color.g = 0.0;
    Cop_l.color.b = 0.0;
    
    Cop_l.pose.position.x = COP1.copx;
    Cop_l.pose.position.y = COP1.copy;
    Cop_l.pose.position.z = 0;
    Cop_l.pose.orientation.x = 0.0;
    Cop_l.pose.orientation.y = 0.0;
    Cop_l.pose.orientation.z = 0.0;
    Cop_l.pose.orientation.w = 1.0;
    
    pub_viz.publish( Cop_l );
}

/**
* @fn void SenVal2CallBk ( const pressure_cells::SenVal & senval )
* Callback function fot subscriber of the 2nd group of Load Cells
* @param a vector w/ the values of the Load Cells
* @return void
*/
void SenVal2CallBk ( const pressure_cells::SenVal & senval )
{
    g_sen2[0] = senval.sen1;
    g_sen2[1] = senval.sen2;
    g_sen2[2] = senval.sen3;
    g_sen2[3] = senval.sen4;

    COP2.copx=0;
    COP2.copy=0;
    
    for ( uint i = 0; i < 4; i++ )
    {
        COP2.copx += g_sen2[i] * p_origins_r[i].point.x;
        COP2.copy += g_sen2[i] * p_origins_r[i].point.y;
    }
    //cout<<"COP.copx = "<<COP2.copx<<"\t COP.copy"<<COP2.copx<<endl;
    COP2.copx = COP2.copx / (g_sen2[0] + g_sen2[1] + g_sen2[2] + g_sen2[3]);
    
    COP2.copy = COP2.copy / (g_sen2[0] + g_sen2[1] + g_sen2[2] + g_sen2[3]);
    
    //// publish COP2
    COP2.header.stamp = ros::Time::now (  );
    pub_cop2.publish(COP2);
    ok2 = 1;
    
    if(!g_viz) return;

    visualization_msgs::Marker Cop_r;
    
    // COP Rigth
    Cop_r.header.frame_id = tf_feet_r;
    Cop_r.header.stamp = ros::Time::now();
    Cop_r.ns = "Cop_r";
    Cop_r.id = 0;
    Cop_r.type = visualization_msgs::Marker::SPHERE;
    Cop_r.action = visualization_msgs::Marker::ADD;
    Cop_r.scale.x = 0.02;
    Cop_r.scale.y = 0.02;
    Cop_r.scale.z = 0.02;
    Cop_r.color.a = 1.0;
    Cop_r.color.r = 0.0;
    Cop_r.color.g = 0.0;
    Cop_r.color.b = 0.0;

    Cop_r.pose.position.x = COP2.copx;
    Cop_r.pose.position.y = COP2.copy;
    Cop_r.pose.position.z = 0;
    Cop_r.pose.orientation.x = 0.0;
    Cop_r.pose.orientation.y = 0.0;
    Cop_r.pose.orientation.z = 0.0;
    Cop_r.pose.orientation.w = 1.0;

    pub_viz.publish( Cop_r );
}

/**
* @fn int main ( int argc, char **argv )
* Main 
*/
int main ( int argc, char **argv )
{
    
    //std::cout<<"CENAS 1"<<std::endl;
    uint i;
    double sen[8];

    //std::cout<<"CENAS 2"<<std::endl;
    p_origins_l[0].point.x = 0.044;
    p_origins_l[1].point.x = 0.044;
    p_origins_l[2].point.x = -0.046;
    p_origins_l[3].point.x = -0.046;
    p_origins_l[0].point.y = 0.022;
    p_origins_l[1].point.y = -0.022;
    p_origins_l[2].point.y = -0.023;
    p_origins_l[3].point.y = 0.023;
    
    p_origins_r[0].point.x = 0.044;
    p_origins_r[1].point.x = 0.044;
    p_origins_r[2].point.x = -0.046;
    p_origins_r[3].point.x = -0.046;
    p_origins_r[0].point.y = 0.022;
    p_origins_r[1].point.y = -0.022;
    p_origins_r[2].point.y = -0.023;
    p_origins_r[3].point.y = 0.023;

    
    ros::init ( argc, argv, "force_cop" );
    
    ros::NodeHandle n("~");

    tf::TransformListener lt;
    tf::StampedTransform trans_feet;
    
    // frame for left feet
    
    string tf_feet_l_default = "tf_default_feet_l";
    n.param( "tf_feet_l", tf_feet_l, tf_feet_l_default );
    // frame for right feet
    string tf_feet_r_default = "tf_default_feet_r";
    n.param( "tf_feet_r", tf_feet_r, tf_feet_r_default );
    
    // set marker publisher name
    string markers;
    string markers_default = "default_markers";
    n.param("markers", markers, markers_default );

    std::string check_markers;
    n.getParam("markers", check_markers);
    
    if (check_markers!=markers_default)
    {
        g_viz=1;
        
        ROS_INFO("Setting markers parameter to %s", check_markers.c_str());
    }

    // feet at same level
    string feet_level;
    string feet_level_default = "feet_level_default";
    n.param( "feet_level", feet_level, feet_level_default );
    
    std::string check_level;
    n.getParam("feet_level", check_level);
    
    if (check_level!=feet_level_default)
    {
        g_lev=1;
        ROS_INFO("Setting feet level parameter to %s", check_level.c_str());
    }
    
    pub_cop1 = n.advertise < pressure_cells::Cop > ( "/cop1", 1000 );
    pub_cop2 = n.advertise < pressure_cells::Cop > ( "/cop2", 1000 );
    
    ros::Subscriber sub_msg1 = n.subscribe ( "/msg1", 1000, SenVal1CallBk );
    ros::Subscriber sub_msg2 = n.subscribe ( "/msg2", 1000, SenVal2CallBk );
    
    ros::Rate loop_rate ( 2.5*1300 );

    if(g_lev) pub_cop = n.advertise < pressure_cells::Cop > ( "/cop", 1000 );
    if(g_viz) pub_viz = n.advertise<visualization_msgs::Marker>( markers, 0 );

    
    p_origins_l[0].header.frame_id=tf_feet_l;
    p_origins_l[1].header.frame_id=tf_feet_l;
    p_origins_l[2].header.frame_id=tf_feet_l;
    p_origins_l[3].header.frame_id=tf_feet_l;
    
    p_origins_r[0].header.frame_id=tf_feet_r;
    p_origins_r[1].header.frame_id=tf_feet_r;
    p_origins_r[2].header.frame_id=tf_feet_r;
    p_origins_r[3].header.frame_id=tf_feet_r;
    //std::cout<<"Frame points right -> "<<p_origins_r[3].header.frame_id<<std::endl;

    int pid = getpid(), rpid;

    boost::format fmt("sudo renice -10 %d");
    fmt % pid;
    
    rpid = std::system(fmt.str().c_str());
    
    while ( ros::ok (  ) )
    {
        
        if ( ( ok2 && ok1 ) == 1  && g_lev)
        {
            ok1 = 0;
            ok2 = 0;
            
            try
            {
                lt.lookupTransform( tf_feet_r, tf_feet_l, ros::Time(0), trans_feet);
                
            }
            catch (tf::TransformException ex)
            {
                ROS_INFO("Transformation not found:  entered catch for 1.5 seconds!");
                
                if(lt.waitForTransform(tf_feet_r, tf_feet_l,ros::Time(0), ros::Duration(1.5)))
                {
                    try
                    {
                        lt.lookupTransform( tf_feet_r, tf_feet_l, ros::Time(0), trans_feet);
                    }
                    catch (tf::TransformException ex)
                    {
                        ROS_ERROR("Joystick Transforms: tansforrmation not found after waiting 1.5 seconds\n.%s",ex.what());
                    }
                }
                else
                {
                    ROS_ERROR("Joystick Transforms: Could not find valid transform after waiting 1.5 seconds\n.%s",ex.what());
                }
            }

            
            vector<geometry_msgs::PointStamped> ptrans(4);
            
            for (uint n = 0; n < ptrans.size(); n++)
            {
                lt.transformPoint (tf_feet_r, p_origins_l[n], ptrans[n]);
            }
            
            sen[0] = g_sen1[0];//left
            sen[1] = g_sen1[1];
            sen[2] = g_sen1[2];
            sen[3] = g_sen1[3];
            sen[4] = g_sen2[0];//right
            sen[5] = g_sen2[1];
            sen[6] = g_sen2[2];
            sen[7] = g_sen2[3];

            COP.copx=0;
            COP.copy=0;
            
            for ( i = 0; i < 4; i++ )
            {
                COP.copx += sen[i] * ptrans[i].point.x;
                COP.copy += sen[i] * ptrans[i].point.y;
            }
            for ( ; i < 8; i++ )
            {
                COP.copx += sen[i] * p_origins_r[i-4].point.x;
                COP.copy += sen[i] * p_origins_r[i-4].point.y;
            }
            
            COP.copx = COP.copx / (sen[0] + sen[1] + sen[2] + sen[3] + sen[4] + sen[5] + sen[6] + sen[7]);
            
            COP.copy = COP.copy / (sen[0] + sen[1] + sen[2] + sen[3] + sen[4] + sen[5] + sen[6] + sen[7]);
            
            COP.header.stamp = ros::Time::now (  );
            pub_cop.publish ( COP );

            if( g_viz)
            {
                visualization_msgs::Marker Cop_;
    
                // COP Left&Right
                Cop_.header.frame_id = tf_feet_r;
                Cop_.header.stamp = ros::Time::now();
                Cop_.ns = "Cop_";
                Cop_.id = 0;
                Cop_.type = visualization_msgs::Marker::SPHERE;
                Cop_.action = visualization_msgs::Marker::ADD;
                Cop_.scale.x = 0.02;
                Cop_.scale.y = 0.02;
                Cop_.scale.z = 0.02;
                Cop_.color.a = 1.0;
                Cop_.color.r = 1.0;
                Cop_.color.g = 0.78;
                Cop_.color.b = 0.58;
                
                Cop_.pose.position.x = COP.copx;
                Cop_.pose.position.y = COP.copy;
                Cop_.pose.position.z = 0;
                Cop_.pose.orientation.x = 0.0;
                Cop_.pose.orientation.y = 0.0;
                Cop_.pose.orientation.z = 0.0;
                Cop_.pose.orientation.w = 1.0;
                
                pub_viz.publish( Cop_ );
            }
        }
        
        ros::spinOnce (  );
        
        loop_rate.sleep (  );
    }
    
    return 0;
}
