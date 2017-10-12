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
 * @file pressure_cells.cpp
 * @author Emílio Estrelinha nº 38637 (emilio.estrelinha@ua.pt)
 * @brief Communicates w/ the arduino and publishes a value for each Load Cell
 */

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>

#include <sys/types.h>
#include <unistd.h>
#include <stdlib.h>

#include <sstream>
#include <unistd.h>

#include <pressure_cells/arduino.h>
#include <pressure_cells/BufferedAsyncSerial.h>
#include <pressure_cells/SenVal.h>

using namespace ros;
using namespace std;

double ee_calibrate(double adc, int cell)
{
   
    if(cell==1)
    {
        //Calibrate cell I
        return (double)(0.026574084375916 * adc -0.834411121568734);
    }
    if(cell==2)
    {
        //Calibrate cell II
        return (double)(0.026233317081797 * adc  -4.311235918674921);
    }
    if(cell==3)
    {
        //Calibrate cell III
        return (double)(0.027158321888210 * adc  -1.748472910437862);
    }
    if(cell==4)
    {
        //Calibrate cell IV
        return (double)(0.026998009432107 * adc  -3.485178371297848);
    }
    if(cell==5)
    {
        //Calibrate cell V
        return (double)(0.061665094219161 * adc  -0.037584380861199);
    }
    if(cell==6)
    {
        //Calibrate cell VI
        return (double)(0.059355778053492 * adc  -0.971028934842100);
    }
    if(cell==7)
    {
        //Calibrate cell VII
        return (double)(0.061558158124062 * adc  -1.514361056192554);
    }
    if(cell==8)
    {
        //Calibrate cell VIII
        return (double)(0.062712489442863 * adc  -0.845013447028478);
    }

    return -1;
}

/**
* @fn int main ( int argc, char **argv )
* Main 
*/
int main ( int argc, char **argv )
{
    
    
    bool viz=0;
    
    init( argc, argv, "arduino" );
    
    NodeHandle n("~");
    
    ros::Publisher pub_viz;
    
    string device;
    string device_default = "default_device";
    n.param("/device", device, device_default );
    
    // set marker publisher name
    string tf_feet;
    string tf_feet_default = "tf_default_feet";
    n.param( "tf_feet", tf_feet, tf_feet_default );
    
    // frame for feet
    string markers;
    string markers_default = "default_markers";
    n.param("markers", markers, markers_default );
    

    std::string check_markers;
    n.getParam("markers", check_markers);
    
    if (check_markers!=markers_default)
    {
        ROS_INFO("Setting markers parameter to %s", check_markers.c_str());
        viz=1;
    }

    // Cell Calibration
    
    int cell1;
    int cell1_default = 1;
    n.param("cell1", cell1, cell1_default );
    
    int cell2;
    int cell2_default = 2;
    n.param("cell2", cell2, cell2_default );
    
    int cell3;
    int cell3_default = 3;
    n.param("cell3", cell3, cell3_default );
    
    int cell4;
    int cell4_default = 4;
    n.param("cell4", cell4, cell4_default );

    
    pub_viz = n.advertise<visualization_msgs::Marker>( markers, 0 );
    
    Publisher pub_sen_val = n.advertise< pressure_cells::SenVal >( "/values", 1000 );
    
    Rate loop_rate ( 1400 );
    
    pressure_cells::SenVal senval;

    BufferedAsyncSerial serial( device, 115200 );

    string data;
    double adc1, adc2, adc3, adc4;
    
    int pid = getpid(), rpid;

    boost::format fmt("sudo renice -10 %d");
    fmt % pid;
    
    rpid = std::system(fmt.str().c_str());
    
    while ( ok () )
    {
        try
        {
            // Always returns immediately. If the terminator \n has not yet
            // arrived, returns an empty string.
            data = serial.readStringUntil ( "\n" );
            
            //Incomplete message
            if(data.size()!=5) {loop_rate.sleep (  ); continue;}
            
            adc1 = (unsigned char)data[0] << 2 | (unsigned char)data[4] >> 6 ;
            adc2 = (unsigned char)data[1] << 2 | ((unsigned char)data[4] & 0x30) >> 4;
            adc3 = (unsigned char)data[2] << 2 | ((unsigned char)data[4] & 0x0C) >> 2;
            adc4 = (unsigned char)data[3] << 2 | ((unsigned char)data[4] & 0x03);

            senval.sen1=ee_calibrate(adc1, cell1); if(senval.sen1<=0) senval.sen1=0.001;
            senval.sen2=ee_calibrate(adc2, cell2); if(senval.sen2<=0) senval.sen2=0.001;
            senval.sen3=ee_calibrate(adc3, cell3); if(senval.sen3<=0) senval.sen3=0.001;
            senval.sen4=ee_calibrate(adc4, cell4); if(senval.sen4<=0) senval.sen4=0.001;
            
            senval.header.stamp = ros::Time::now (  );
            
            pub_sen_val.publish ( senval );
            
            if(viz)
            {
                geometry_msgs::Point f1_p1;
                geometry_msgs::Point f1_p2;
                geometry_msgs::Point f2_p1;
                geometry_msgs::Point f2_p2;
                geometry_msgs::Point f3_p1;
                geometry_msgs::Point f3_p2;
                geometry_msgs::Point f4_p1;
                geometry_msgs::Point f4_p2;
                
                visualization_msgs::Marker force1;
                visualization_msgs::Marker force2;
                visualization_msgs::Marker force3;
                visualization_msgs::Marker force4;

                // Load Cell 1
                force1.header.frame_id = tf_feet;
                force1.header.stamp = ros::Time();
                force1.ns = "force1";
                force1.id = 0;
                force1.type = visualization_msgs::Marker::ARROW;
                force1.action = visualization_msgs::Marker::ADD;
                force1.scale.x = 0.02;
                force1.scale.y = 0.02;
                force1.scale.z = 0;
                force1.color.a = 1.0;
                force1.color.r = 1.0;
                force1.color.g = 0.5;
                force1.color.b = 0.0;

                force1.points.clear();
                f1_p1.x = 0.044;
                f1_p1.y = 0.022;
                f1_p1.z = 0;
                force1.points.push_back(f1_p1);
                f1_p2.x = 0.044;
                f1_p2.y = 0.022;
                f1_p2.z = senval.sen1/25;
                force1.points.push_back(f1_p2);
                
                // Load Cell 2
                force2.header.frame_id = tf_feet;
                force2.header.stamp = ros::Time();
                force2.ns = "force2";
                force2.id = 0;
                force2.type = visualization_msgs::Marker::ARROW;
                force2.action = visualization_msgs::Marker::ADD;
                force2.scale.x = 0.02;
                force2.scale.y = 0.02;
                force2.scale.z = 0;
                force2.color.a = 1.0;
                force2.color.r = 0.02;
                force2.color.g = 0.8;
                force2.color.b = 0.02;

                force2.points.clear();
                f2_p1.x = 0.044;
                f2_p1.y = -0.022;
                f2_p1.z = 0;
                force2.points.push_back(f2_p1);
                f2_p2.x = 0.044;
                f2_p2.y = -0.022;
                f2_p2.z = senval.sen2/25;
                force2.points.push_back(f2_p2);
                
                // Load Cell 3
                force3.header.frame_id = tf_feet;
                force3.header.stamp = ros::Time();
                force3.ns = "force3";
                force3.id = 0;
                force3.type = visualization_msgs::Marker::ARROW;
                force3.action = visualization_msgs::Marker::ADD;
                force3.scale.x = 0.02;
                force3.scale.y = 0.02;
                force3.scale.z = 0;
                force3.color.a = 1.0;
                force3.color.r = 0.02;
                force3.color.g = 0.37;
                force3.color.b = 0.8;

                force3.points.clear();
                f3_p1.x = -0.046;
                f3_p1.y = -0.023;
                f3_p1.z = 0;
                force3.points.push_back(f3_p1);
                f3_p2.x = -0.046;
                f3_p2.y = -0.023;
                f3_p2.z = senval.sen3/25;
                force3.points.push_back(f3_p2);
                
                // Load Cell 4
                force4.header.frame_id = tf_feet;
                force4.header.stamp = ros::Time();
                force4.ns = "force4";
                force4.id = 0;
                force4.type = visualization_msgs::Marker::ARROW;
                force4.action = visualization_msgs::Marker::ADD;
                force4.scale.x = 0.02;
                force4.scale.y = 0.02;
                force4.scale.z = 0;
                force4.color.a = 1.0;
                force4.color.r = 0.37;
                force4.color.g = 0.02;
                force4.color.b = 0.8;

                force4.points.clear();
                f4_p1.x = -0.046;
                f4_p1.y = 0.023;
                f4_p1.z = 0;
                force4.points.push_back(f4_p1);
                f4_p2.x = -0.046;
                f4_p2.y = 0.023;
                f4_p2.z = senval.sen4/25;
                force4.points.push_back(f4_p2);
                
                pub_viz.publish( force1 );
                pub_viz.publish( force2 );
                pub_viz.publish( force3 );
                pub_viz.publish( force4 );

            }
            
        }
        catch ( boost::system::system_error& e )
        {
            cout << "Error: " << e.what (  ) << endl;
            return 1;
        }
        
        spinOnce (  );
        
        loop_rate.sleep (  );
    
    }
    
    return 0;
}
