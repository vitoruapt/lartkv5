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
 * @file sensors_receiver.cpp
 * @author Telmo Rafeiro n.º 45349 (rafeiro@ua.pt)
 * @brief Reads the network IMU's data and publish
 */

#include <ros/ros.h>
#include "std_msgs/String.h"
#include <sstream>
#include <string>

//Include of message topic
#include <imu_network/raw_imu_message.h>
#include <imu_network/sensors_values.h>
#include <imu_network/sensors_network.h>

//INCLUDE SERIAL LIB FROM LAR3/utils
#include <serialcom/SerialCom.h>
#include <algorithm>

using namespace std;

/**
* @fn int main ( int argc, char **argv )
* Main 
*/
int main (int argc ,char** argv)
{
    
    serialcom::SerialCom serial;
    
    ros::init(argc , argv , "sensors_raw_data");
    ros::NodeHandle n;
    ros::Publisher chatter_pub;
    chatter_pub = n.advertise<imu_network::sensors_network>("topic_raw_data", 1000);
    
    ros::Rate loop_rate(10);
    
    try
    {
        
        string number_buffer;
        string buffer_aux;
        string Ax,Ay,Az,Mx,My,Mz,Gx,Gy,Gz;
        string junk;
        int number,total_number;
        int numbered=0;
        string converter;
        
        serial.open("/dev/ttyACM0",115200);
        
        usleep(10000000);
        
		imu_network::sensors_network values;
        
        while (numbered == 0)
        {
	  serial.write("#n");
	  serial.readLine(&number_buffer,100);
	  if ((count(number_buffer.begin(),number_buffer.end(),'N')) && (count(number_buffer.begin(),number_buffer.end(),',')) && (number_buffer.length()==3))
	  {
	      
	      istringstream ssN(number_buffer);
	      getline(ssN,junk,'N');
	      getline(ssN,converter, ',');
	      total_number = (int) atoi(converter.c_str());
	      ROS_INFO("Comunicação iniciada com %d sensores",total_number);
	      numbered = 1;
	  }
	  
        }
// init of message variable for total_number sensors
        for(int i=0;i<total_number;i++)
        {
	  imu_network::sensors_values sensor;
	  values.sensors_val.push_back(sensor);
        }
        
// reading and sort the variables of raw imu data & sending it within the message
        while(ros::ok())
        {
	  
	  string data[total_number];
	  int ncsa[total_number];
	  int ncsm[total_number];
	  int ncsg[total_number];
	  int na[total_number];
	  int nm[total_number];
	  int ng[total_number];

	  string num[total_number];
	  
	  serial.flush();
	  serial.write("#f");
	  


	  for (int c = 0; c<total_number;c++)
	  {
	      try
	      {
		serial.readLine(&data[c],100);
	
	      }catch(serialcom::Exception ex)
	      
	      {
		cout<<ex.what()<<endl;
	      }
	      
	      ncsa[c]=count(data[c].begin(), data[c].end(), ',');
	      
	      na[c]=count(data[c].begin(), data[c].end(), 'A');
	      nm[c]=count(data[c].begin(), data[c].end(), 'M');
	      ng[c]=count(data[c].begin(), data[c].end(), 'G');

	      // testing the content inside buffer
	      if(ncsa[c]!=10)
	      {
		exit(0);
	      }
	      else
	      {
		
	      }
// verify the sensor number to match with the for cicle
	      istringstream ss(data[c]);
	      getline(ss,junk,'A');
	      num[c] = junk[1];
	      if(c == (int)atoi(num[c].c_str()))
	      {
// getting the raw imu data from buffer
		getline(ss,Ax, ',');
		getline(ss,Ay, ',');
		getline(ss,Az, ',');
		getline(ss,junk,'M');
    		getline(ss,Mx, ',');
    		getline(ss,My, ',');
    		getline(ss,Mz, ',');
		getline(ss,junk,'G');
		getline(ss,Gx, ',');
		getline(ss,Gy, ',');
		getline(ss,Gz, ',');
		
	      }
// storing the raw imu data into the message variables for total_number sensors
	      values.sensors_val[c].S_Ax= (double) atof(Ax.c_str());
	      values.sensors_val[c].S_Ay = (double) atof(Ay.c_str());
	      values.sensors_val[c].S_Az = (double) atof(Az.c_str());
	      
	      values.sensors_val[c].S_Mx = (double) atof(Mx.c_str());
	      values.sensors_val[c].S_My = (double) atof(My.c_str());
	      values.sensors_val[c].S_Mz = (double) atof(Mz.c_str());
	      
	      values.sensors_val[c].S_Gx =(double)atof(Gx.c_str());
	      values.sensors_val[c].S_Gy =(double)atof(Gy.c_str());
	      values.sensors_val[c].S_Gz =(double)atof(Gz.c_str());
	      
	      values.sensors_val[c].total_number = total_number;
	      values.sensors_val[c].header.stamp = ros::Time::now();
	      values.sensors_val[c].number = (int)atoi(num[c].c_str());
	      values.sensors_val[c].header.frame_id = "imu_raw";
	      
	  }
	  chatter_pub.publish(values);
	  loop_rate.sleep();
        }
//close COM after CTRL+C intercepted
        serial.close();
        
    }catch (std::exception& e)
    {
        std::cerr << "Exception: " << e.what() << "\n";
    }
    
    return 0;
    
}
