/**************************************************************************************************
 * Software License Agreement (BSD License)                                                           *
 * 
 * Copyright (c) 2011-2014, LAR toolkit developers - University of Aveiro - http://lars.mec.ua.pt
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification, are permitted
 * provided that the following conditions are met:
 * 
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
 * \brief Orientation_base 
 */

#include <iostream>
#include <pthread.h>

#include <math.h>   
#include <string>
#include <sstream>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <iostream>

#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <boost/thread.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>
#include <boost/thread/thread.hpp>

#include <ros/ros.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <serialcom/SerialCom.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <sensor_msgs/Range.h>
#include <ros/package.h>
#include <geometry_msgs/Vector3.h>

using namespace std;
using namespace boost;

/**
 \brief Plane
 
 This class is responsible for define the plane coeficients.
 */

class Plane
{
public:
    /**
     \brief Class constructor
     
     Initialize variables equal to zero, do not call any function.
     */
    Plane()
    {
        a=0;b=0;c=0;d=0;
    }
    
    double a,b,c,d;
};
/**
 \brief SharpMath
 
 This class is responsible for calculate the plane to obtain the orientation. Is also responsible to open and write the calibration parameters from yaml file.
 */
class SharpMath
{
public:
    /**
     \brief Class constructor
     
     Do nothing
     */
    SharpMath()
    {
    }
    /**
     \brief readParameters
     Function responsible for read the parameters from yaml file.
     * @param  parameters_file - file url.
     */
    void readParameters(string parameters_file)
    {
        cv::FileStorage parameters(parameters_file.c_str(),cv::FileStorage::READ);
        
        number_sensors = (int)parameters["number_sensors"];
        
//         cout << "sensores: " << number_sensors << endl;
        
        for(uint s = 0;s<number_sensors;s++)
        {
            Eigen::Vector2d pos;
            
            pos(0) = parameters["x"+to_string(s)];
            pos(1) = parameters["y"+to_string(s)];
            
            sensors_positions.push_back(pos);
            
            Eigen::VectorXd curve(4);
            
            curve(0) = parameters["p1_"+to_string(s)];
            curve(1) = parameters["p2_"+to_string(s)];
            curve(2) = parameters["p3_"+to_string(s)];
            curve(3) = parameters["p4_"+to_string(s)];
            
            sensors_curve_parameters.push_back(curve);
        }
        
        pitch_bias = parameters["pitch_bias"];
        roll_bias =  parameters["roll_bias"];
        
//         cout << "pitch: " << pitch_bias << " roll: " << roll_bias << endl;
//         cout << "sensor n: " << number_sensors << " x0 " << sensors_positions[0](0) << " y0 " << sensors_positions[0](1) << " x1 " << sensors_positions[1](0) << " y1 " << sensors_positions[1](1) << " x2 " << sensors_positions[2](0) << " y2 " << sensors_positions[2](1) << " x3 " << sensors_positions[3](0) << " y3 " << sensors_positions[3](1) << " x4 " << sensors_positions[4](0) << " y4 " << sensors_positions[4](1) << " x5 " << sensors_positions[5](0) << " y5 " << sensors_positions[5](1) << " x6 " << sensors_positions[6](0) << " y6 " << sensors_positions[6](1) << " x7 " << sensors_positions[7](0) << " y7 " << sensors_positions[7](1) << endl;
    }
    /**
     \brief getPlane
     Function responsible for calculate the plane acording to the given points.
     * @param  values - vector(double) with the sensors readings.
     * @return Plane with the plane coeficients.
     */ 
    Plane getPlane(vector<double> values)
    {
//         for(uint n=0;n<number_sensors;n++)
//             cout << "Valores: " << values[n] << endl;
        
        Eigen::VectorXd g(number_sensors);
        g = Eigen::VectorXd::Ones(number_sensors);
        
//         cout<<"g: "<<g<<endl;
        
        Eigen::MatrixXd F(number_sensors,3);
        
        for(uint i=0;i<number_sensors;i++)
        {
            F(i,0) = sensors_positions[i](0);
            F(i,1) = sensors_positions[i](1);
            F(i,2) = values[i];
        }
        
        Eigen::VectorXd r(3);
        
        r = ((F.transpose()*F).inverse() * F.transpose())*g;
        
        double zp = ((-r(0)/r(2))*sensors_positions[0](0)) - ((r(1)/r(2))*sensors_positions[0](1));
        
        double d = values[0] - zp;
        double a = r(0)*d;
        double b = r(1)*d;
        double c = r(2)*d;
        
//         cout << "F: " << F << endl;
//         cout << "r: " << r << endl;
        
        Plane temp;
        
        temp.a=a;
        temp.b=b;
        temp.c=c;
        temp.d=d;
        
        return temp;
    }
    /**
     \brief getPitch
     Function responsible for calculate the plane pitch.
     * @param  p - Plane with the plane coeficients.
     * @return double with the pitch angle.
     */
    double getPitch( Plane p)
    {
        double a, b, c, d;
        a=p.a;
        b=p.b;
        c=p.c;
        d=p.d;
        return ((180./M_PI)*(atan2(d,(d*c/a)))) - pitch_bias;        
    }
    /**
     \brief getRoll
     Function responsible for calculate the plane roll.
     * @param  p - Plane with the plane coeficients.
     * @return double with the roll angle.
     */
    double getRoll( Plane p)
    {
        double a, b, c, d;
        a=p.a;
        b=p.b;
        c=p.c;
        d=p.d;
        return ((180./M_PI)*(atan2(d,(d*c/b)))) - roll_bias;    
    }
    
    
private:
    
    uint number_sensors;
    vector<Eigen::Vector2d> sensors_positions;
    vector<Eigen::VectorXd> sensors_curve_parameters;
    
    double pitch_bias;
    double roll_bias;
    
};

/**
 \brief OptoelectricVehiclePose
 
 This class is responsible for subscribe to sensors message and calculate the calibration of the system.
 */
class OptoelectricVehiclePose
{
public:
    /**
     \brief Class constructor
     Init the serial port, open the txt file and the yaml file.
     */
    OptoelectricVehiclePose(const ros::NodeHandle& nh):
    nh_(nh)
    {
        // iniciar mensagens
        setupMessaging();
         
        string parameters_url;
        
        nh_.param("parameters",parameters_url,std::string("no parameters file"));
           
        cout << "url: " << parameters_url << endl;
        
        s.readParameters(parameters_url);
        // iniciar leitura dos dados yaml
    }
    // Inicia os publicadores e os subscrevedores
    /**
     \brief setupMessaging
     Function responsible for creat the publishers.
     */
    void setupMessaging()
    {
        /// SUBSCREVER
        
        new_data.resize(8,false);
        
        for(uint i=0;i<8;i++)
        {
            ros::Subscriber sub = nh_.subscribe<sensor_msgs::Range>("/optoelectic_vehicle_base/sharp_"+to_string(i), 1, boost::bind(&OptoelectricVehiclePose::sharpCallback,this,_1,i));
            
            sharp_subscriber.push_back(sub);
        }
        
        
        /// PUBLICAR
        
        pose_publisher = nh_.advertise<geometry_msgs::Vector3>("pose_orientation", 1000);
        car_mark_pub = nh_.advertise<visualization_msgs::Marker>("atlascar_ori",1000);
    }
    /**
     \brief sharpCallback
     Function responsible evaluate is the message is valid.
     * @param  msg - const sensor_msgs::Range::ConstPtr with the message.
     * @param i - 1 if the message is valid.
     */
    void sharpCallback(const sensor_msgs::Range::ConstPtr msg,int i)
    {
        //cout<<"Received sensor: "<<i<< " value: "<<msg->range<<endl;
        if(msg->range<1023)
        {
            sharp_values[i]=msg->range;
            new_data[i] = true;
        }
    }
    /**
     \brief carMarker
    Function responsible publish the car marker.
    * @param  pose - geometry_msgs::Vector3 with the pose of the vehicle.
    */
    void carMarker(geometry_msgs::Vector3 pose)
    {
        pose.x=pose.x*3.14/180;
        pose.y=pose.y*3.14/180;
        pose.z=pose.z*3.14/180;
        
        car_marker.header.frame_id = "base_link";
        
        car_marker.id = 8484;
        car_marker.ns = "atlas";
        
        car_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
        car_marker.action = visualization_msgs::Marker::ADD;
        
        
        car_marker.pose.position.x = 0;
        car_marker.pose.position.y = 0;
        car_marker.pose.position.z = 0.4; 
        // Escala do marcador
        car_marker.scale.x = 1;
        car_marker.scale.y = 1;
        car_marker.scale.z = 1;
        
        car_marker.pose.orientation.x = sin(pose.x/2)*cos(pose.y/2)*cos(pose.z/2) - cos(pose.x/2)*sin(pose.y/2)*sin(pose.z/2);
        car_marker.pose.orientation.y = cos(pose.x/2)*sin(pose.y/2)*cos(pose.z/2) + sin(pose.x/2)*cos(pose.y/2)*sin(pose.z/2);
        car_marker.pose.orientation.z = cos(pose.x/2)*cos(pose.y/2)*sin(pose.z/2) - sin(pose.x/2)*sin(pose.y/2)*cos(pose.z/2);
        car_marker.pose.orientation.w = cos(pose.x/2)*cos(pose.y/2)*cos(pose.z/2) + sin(pose.x/2)*sin(pose.y/2)*sin(pose.z/2);
        
        
        // Cor do marcador (255,99,71)
        car_marker.color.r = 0.0f;
        car_marker.color.g = 0.0f;
        car_marker.color.b = 0.0f;
        car_marker.color.a = 0.0f;
        
        car_marker.mesh_use_embedded_materials=1;
        
        car_marker.mesh_resource = "package://optoelectric/ship/FordEscort_ori.DAE";

        car_marker.lifetime = ros::Duration();
    } 
    
    /**
     \brief loop
     Function responsible to put the program running forever.
     */
    void loop()
    {
        ros::Rate r(100);
                
        while(ros::ok())
        {
            r.sleep();
            ros::spinOnce();
            
            bool process = true;
            
            for(bool v: new_data)
            {
                if(v==false)
                {
                    process = false;
                    break;
                }
            }
            
            if(process==false)
                continue;
            
            vector<double> values;
            for(uint i=0;i<8;i++)
            {
                double val = sharp_values[i];
                values.push_back(val);
//                 cout << "i: " << i << " sensor " << val << endl;
            }

            Plane plane = s.getPlane(values);
            
            pitch = s.getPitch(plane);
            roll = s.getRoll(plane);
            
            if (roll > 90)
                roll=roll-180;
            if (roll < -90)
                roll=roll+180;
            if (pitch > 90)
                pitch=pitch-180;
            if (pitch < -90)
                pitch=pitch+180;
            
            cout << "pitch: " << pitch << " roll: " << roll << endl;
            
            geometry_msgs::Vector3 posee;
            
            if(pitch < 45 && roll < 45)
            {
                posee.x=roll;
                posee.y=pitch;
                posee.z=0;
                
                
                
                pose_publisher.publish(posee);
                
                carMarker(posee);
                car_marker.header.stamp = ros::Time::now();
                car_mark_pub.publish(car_marker);
            }
        
                                    
            new_data.clear();
            new_data.resize(8,false);
            
        }
    }
    
private:
    
    visualization_msgs::Marker car_marker;
    ros::Publisher car_mark_pub;
    
    ros::NodeHandle nh_;
    
    double pitch;
    double roll;
    
    SharpMath s;
    
    ros::Publisher pose_publisher;
 
    vector<ros::Subscriber> sharp_subscriber;
    
    int sharp_values[8];
    vector<bool> new_data;

};

/**
 \brief main
 Init ros and stays in loop forever.
 */
int main(int argc, char* argv[])
{
    // Iniciar o ros
    ros::init(argc, argv, "preception_orientation_base");
    // Handle
    ros::NodeHandle nh("~");
    
    // inciar
    OptoelectricVehiclePose opto(nh);

    opto.loop();
    
    return 0;
}
