/**************************************************************************************************
 Software License Agreement (BSD License)                                                           *
 
 Copyright (c) 2011-2014, LAR toolkit developers - University of Aveiro - http://lars.mec.ua.pt
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
 \file
 \brief Orientation_base 
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

#include <serialcom/BufferedAsyncSerial.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <sensor_msgs/Range.h>
#include <ros/package.h>

using namespace std;
using namespace boost;
/**
 \brief Plane
 
 This class is responsible for define the plane coeficients.
 */

// Class to defini the plane coeficients
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
 
 This class is responsible for convert the analog read signal to the equivalent distance, calibration. Is also responsible to open and read the calibration parameters from yaml file.
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
        
        cout<<"number_sensors: "<<number_sensors<<endl;
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
        cout<<"Reed parameters: "<<sensors_curve_parameters.size()<<endl;
        pitch_bias = parameters["pitch_bias"];
        roll_bias =  parameters["roll_bias"];
    }
    
    /**
     \brief convertoV
     Function responsible for convert the analog signal to the equivalent voltage.
     * @param  A - double with the analog signal.
     * @return double with the voltage convertion.
     */
    double convertoV(double A)
    {
        return A*5/1023;    
    }
    
    /**
     \brief curveCalibration
     Function responsible for convert the voltage signal to distance. Diferent sensors have diferent calibration curves.
     * @param  sensor_id - uint with the number of the sensor.
     * @param  value - double with the volage value.
     * @return double with the distance.
     */
    double curveCalibration(uint sensor_id, double value)
    {
        Eigen::VectorXd curve_parameters = sensors_curve_parameters[sensor_id];
        double a = curve_parameters(0);
        double b = curve_parameters(1);
        double c = curve_parameters(2);
        double d = curve_parameters(3);
        
//         cout << "a: " << a << "b: " << b << "c: " << c << "d: " << d << "valor: " << a*exp(b*value)+c*exp(d*value) << endl;
        
        return a*exp(b*value)+c*exp(d*value);
    }

//     Plane getPlane(vector<double> values)
//     {
//         Eigen::VectorXd g(values.size());
//         g = Eigen::VectorXd::Ones(values.size());
//         
//         cout<<"g: "<<g<<endl;
//         
//         Eigen::MatrixXd F(values.size(),3);
//         
//         for(uint i=0;i<values.size();i++)
//         {
//             F(i,0) = sensors_positions[i](0);
//             F(i,1) = sensors_positions[i](1);
//             F(i,2) = values[i];
//         }
//         
//         Eigen::VectorXd r(3);
//         
//         r = ((F.transpose()*F).inverse() * F.transpose())*g;
//         
//         double zp = ((-r(0)/r(2))*sensors_positions[0](0)) - ((r(1)/r(2))*sensors_positions[0](1));
//         
//         double d = values[0] - zp;
//         double a = r(0)*d;
//         double b = r(1)*d;
//         double c = r(2)*d;
//         
//         Plane temp;
//         
//         temp.a=a;
//         temp.b=b;
//         temp.c=c;
//         temp.d=d;
//         
//         return temp;
//     }
    
//     double getPitch( Plane p)
//     {
//         double a, b, c, d;
//         a=p.a;
//         b=p.b;
//         c=p.c;
//         d=p.d;
//         return ((180./M_PI)*(atan2(d,(d*c/a)))) - pitch_bias;        
//     }
//     double getRoll( Plane p)
//     {
//         double a, b, c, d;
//         a=p.a;
//         b=p.b;
//         c=p.c;
//         d=p.d;
//         return ((180./M_PI)*(atan2(d,(d*c/b)))) - roll_bias;    
//     }
    
    
private:
    
    uint number_sensors;
    vector<Eigen::Vector2d> sensors_positions;
    vector<Eigen::VectorXd> sensors_curve_parameters;
    
    double pitch_bias;
    double roll_bias;
    
};
/**
 \brief OptoelectricVehiclePose
 
 This class is responsible for open the serial port to recive the data from arduino. Also open a txt file to save the read values. And publish a range menssage with the sensors distance.
 */

class OptoelectricVehiclePose
{
public:
    /**
     \brief Class constructor
     Init the serial port, open the txt file and the yaml file.
     */
    OptoelectricVehiclePose(const ros::NodeHandle& nh):
    nh_(nh),
    serial("/dev/ttyACM0",115200)
    {
        
        outfile1.open(("/home/carpinteiro/workingcopies/lar4/src/sensors/optoelectric/data/Sensors_bag.txt"));
        // iniciar mensagens
        setupMessaging();
                
        string parameters_url;
        
        nh_.param("parameters",parameters_url,std::string("no parameters file"));
        
        string package_tag = "package://";
        int pos = parameters_url.find(package_tag);
        if(pos != string::npos)
        {
            //String contains package, replace by package path
            int fpos = parameters_url.find("/",pos+package_tag.length());
            string package = parameters_url.substr(pos+package_tag.length(),fpos-(pos+package_tag.length()));
            
            string package_path = ros::package::getPath(package);
            parameters_url.replace(pos,fpos-pos,package_path);
        }

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
        
        /// PUBLICAR
        // advertive da ship_move_requests para posição desejada
        for(uint i=0;i<8;i++)
        {
            ros::Publisher pub = nh_.advertise<sensor_msgs::Range>("sharp_"+to_string(i), 1000);
            publishers.push_back(pub);
        }
    }
    // Inicia a leitura do serial port

    /**
     \brief checkMessageIntegrety
     Function responsible evaluate is the message is valid.
     * @param  msg - string with the message.
     * @return bool true is the message is valid.
     */
    bool checkMessageIntegrety(string msg)
    {
        bool is_valid = true;
        
        if(msg.length()==0)
            return false;
        
        if(msg.find("A0 ")==string::npos)
            is_valid = false;
        
        if(msg.find("A1 ")==string::npos)
            is_valid = false;
        
        if(msg.find("A2 ")==string::npos)
            is_valid = false;
        
        if(msg.find("A3 ")==string::npos)
            is_valid = false;
        
        if(msg.find("A4 ")==string::npos)
            is_valid = false;
        
        if(msg.find("A5 ")==string::npos)
            is_valid = false;
        
        if(msg.find("A6 ")==string::npos)
            is_valid = false;
        
        if(msg.find("A7 ")==string::npos)
            is_valid = false;
        
        return is_valid;
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
            ros::spinOnce();
            r.sleep();
            
            string line = serial.readStringUntil("\n");
            
            if(checkMessageIntegrety(line)==false)
            {
//                 cout<<"Error: Invalid message ("<<line.length()<<"): "<<line<<endl;
                continue;
            }
            
            cout<<endl;
            cout<<"line: "<<endl;
            cout<<line<<endl;
            cout<<"size: "<<line.length()<<endl;
            
            int readings[8];
            int values_reed = sscanf(line.c_str(),"%*s %d %*s %d %*s %d %*s %d %*s %d %*s %d %*s %d %*s %d",&readings[0],&readings[1],&readings[2],&readings[3],&readings[4],&readings[5],&readings[6],&readings[7]);
            
            cout << "leituras: " << readings[0] << " " <<readings[1] << " " <<readings[2] << " " <<readings[3] << " " <<readings[4] << " " <<readings[5] << " " <<readings[6] << " " <<readings[7] << " " << endl; 
            
            if(values_reed != 8)
            {
                cout<<"Failed to read all 8 values"<<endl;
                cout<<"Read: "<<values_reed<<endl;
                perror("Error");
            }
            
            vector<double> values;
            for(int i=0;i<values_reed;i++)
            {
                double voltage = s.convertoV(readings[i]);
                double value = s.curveCalibration(i,voltage);
                values.push_back(value);
            }
            // Publicar sensores
            
            for(uint i=0;i<values.size();i++)
            {
                sensor_msgs::Range msg;
                msg.range=values[i];
                msg.header.stamp = ros::Time::now();
                msg.header.frame_id = "sharp_"+to_string(i);
                cout << "msg: " << msg.range << endl;
                publishers[i].publish(msg);
            }
            
            outfile1<<ros::Time::now()<<" ";
            
            for(uint i=0;i<8;i++)
            {
                if(i<values.size())
                    outfile1<<" "<<values[i];
                else
                    outfile1<<" NaN";
                
            }
            
            outfile1<<"\n";
            
//             Plane plane = s.getPlane(values);
//             
//             double pitch = s.getPitch(plane);
//             double roll = s.getRoll(plane);


        }
        
    }
        
private:
    
    ros::NodeHandle nh_;
    
    double pitch;
    double roll;
    
    ofstream outfile1;
    
    SharpMath s;
    BufferedAsyncSerial serial;
    
    vector<ros::Publisher> publishers;

};

/**
 \brief main
 Init ros and stays in loop forever.
 */
int main(int argc, char* argv[])
{
    // Iniciar o ros
    ros::init(argc, argv, "orientation_base");
    // Handle
    ros::NodeHandle nh("~");
    
    // inciar
    OptoelectricVehiclePose opto(nh);
    
    opto.loop();
    
    return 0;
}
