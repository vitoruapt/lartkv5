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
 \brief Calibration the orientation_base 
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

#include <arpa/inet.h>
#include <stdbool.h>
#include <stdio.h>
#include <sys/socket.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <gtk/gtk.h>
#include <stdlib.h>
#include <sys/shm.h>
#include <errno.h>
#include <math.h>
#include <ctype.h>
#include <time.h>
#include <sys/time.h>
#include <fcntl.h>
#include "cv.h"
#include "highgui.h"
#include <cairo.h>

#include <ros/ros.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <sensor_msgs/Range.h>
#include <ros/package.h>
#include <geometry_msgs/Vector3.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>



using namespace std;



extern "C"
{

GtkBuilder *builderG;       /**< @brief variavel global que contem informação do gtkbuilder*/

double read_values[17];     /**< @brief variavel global que contem informação da interface*/

double last_values[16];     /**< @brief variavel global que contem informação da yaml*/
 
/**
 * @brief  Calback para detectar o fecho forçado da janela principal
 *
 * @param  widget - widget associado ao evento
 * @param  event - evento associado à calback
 * @param  user_data - user data
 */
void delete_handler(GtkWidget * widget, GdkEvent * event, gpointer user_data)
{
    g_print("You should not use the OS to force leave!\n");
    read_values[17]=0;
    gtk_main_quit();
}

/**
 * @brief  Calback que gere o evento de terminar o programa (interface e comunicação)
 *
 * Invoca a função terminate_comunication que comunica com o gestor de comunicações, essa função
 * econtra-se no ficheiro func_inter.c
 * 
 * @param  widget - widget associado ao evento
 * @param  event - evento associado à calback
 * @param  user_data - user data
 */

void bt_sair_on(GtkWidget * widget, GdkEventMotion * event, gpointer user_data)
{
    read_values[17]=0;
    gtk_main_quit();
}

/**
 * @brief  Calback que gere o evento de terminar o programa (interface e comunicação)
 *
 * Invoca a função terminate_comunication que comunica com o gestor de comunicações, essa função
 * econtra-se no ficheiro func_inter.c
 * 
 * @param  widget - widget associado ao evento
 * @param  event - evento associado à calback
 * @param  user_data - user data
 */

void bt_ler_on(GtkWidget * widget, GdkEventMotion * event, gpointer user_data)
{   
    char estadosensor[512];
    
    char estadoxx0[512];
    char estadoxx1[512];
    char estadoxx2[512];
    char estadoxx3[512];
    char estadoxx4[512];
    char estadoxx5[512];
    char estadoxx6[512];
    char estadoxx7[512];
    
    char estadoyy0[512];
    char estadoyy1[512];
    char estadoyy2[512];
    char estadoyy3[512];
    char estadoyy4[512];
    char estadoyy5[512];
    char estadoyy6[512];
    char estadoyy7[512];
    
    GtkWidget *senso= GTK_WIDGET(gtk_builder_get_object (builderG, "sensores"));
    
    GtkWidget *x_0= GTK_WIDGET(gtk_builder_get_object (builderG, "x0"));
    GtkWidget *y_0= GTK_WIDGET(gtk_builder_get_object (builderG, "y0"));
    
    GtkWidget *x_1= GTK_WIDGET(gtk_builder_get_object (builderG, "x1"));
    GtkWidget *y_1= GTK_WIDGET(gtk_builder_get_object (builderG, "y1"));
    
    GtkWidget *x_2= GTK_WIDGET(gtk_builder_get_object (builderG, "x2"));
    GtkWidget *y_2= GTK_WIDGET(gtk_builder_get_object (builderG, "y2"));
    
    GtkWidget *x_3= GTK_WIDGET(gtk_builder_get_object (builderG, "x3"));
    GtkWidget *y_3= GTK_WIDGET(gtk_builder_get_object (builderG, "y3"));
    
    GtkWidget *x_4= GTK_WIDGET(gtk_builder_get_object (builderG, "x4"));
    GtkWidget *y_4= GTK_WIDGET(gtk_builder_get_object (builderG, "y4"));
    
    GtkWidget *x_5= GTK_WIDGET(gtk_builder_get_object (builderG, "x5"));
    GtkWidget *y_5= GTK_WIDGET(gtk_builder_get_object (builderG, "y5"));
    
    GtkWidget *x_6= GTK_WIDGET(gtk_builder_get_object (builderG, "x6"));
    GtkWidget *y_6= GTK_WIDGET(gtk_builder_get_object (builderG, "y6"));
    
    GtkWidget *x_7= GTK_WIDGET(gtk_builder_get_object (builderG, "x7"));
    GtkWidget *y_7= GTK_WIDGET(gtk_builder_get_object (builderG, "y7"));
    
    sprintf(estadosensor,"%f",last_values[0]);
    
    sprintf(estadoxx0,"%f",last_values[1]);
    sprintf(estadoyy0,"%f",last_values[2]);
    
    sprintf(estadoxx1,"%f",last_values[3]);
    sprintf(estadoyy1,"%f",last_values[4]);
    
    sprintf(estadoxx2,"%f",last_values[5]);
    sprintf(estadoyy2,"%f",last_values[6]);
    
    sprintf(estadoxx3,"%f",last_values[7]);
    sprintf(estadoyy3,"%f",last_values[8]);
    
    sprintf(estadoxx4,"%f",last_values[9]);
    sprintf(estadoyy4,"%f",last_values[10]);
    
    sprintf(estadoxx5,"%f",last_values[11]);
    sprintf(estadoyy5,"%f",last_values[12]);
    
    sprintf(estadoxx6,"%f",last_values[13]);
    sprintf(estadoyy6,"%f",last_values[14]);
    
    sprintf(estadoxx7,"%f",last_values[15]);
    sprintf(estadoyy7,"%f",last_values[16]);
    
    gtk_entry_set_text((GtkEntry *)senso,estadosensor);
    
    gtk_entry_set_text((GtkEntry *)x_0,estadoxx0);
    gtk_entry_set_text((GtkEntry *)y_0,estadoyy0);
    
    gtk_entry_set_text((GtkEntry *)x_1,estadoxx1);
    gtk_entry_set_text((GtkEntry *)y_1,estadoyy1);
    
    gtk_entry_set_text((GtkEntry *)x_2,estadoxx2);
    gtk_entry_set_text((GtkEntry *)y_2,estadoyy2);
   
    gtk_entry_set_text((GtkEntry *)x_3,estadoxx3);
    gtk_entry_set_text((GtkEntry *)y_3,estadoyy3);
    
    gtk_entry_set_text((GtkEntry *)x_4,estadoxx4);
    gtk_entry_set_text((GtkEntry *)y_4,estadoyy4);
    
    gtk_entry_set_text((GtkEntry *)x_5,estadoxx5);
    gtk_entry_set_text((GtkEntry *)y_5,estadoyy5);
    
    gtk_entry_set_text((GtkEntry *)x_6,estadoxx6);
    gtk_entry_set_text((GtkEntry *)y_6,estadoyy6);
    
    gtk_entry_set_text((GtkEntry *)x_7,estadoxx7);
    gtk_entry_set_text((GtkEntry *)y_7,estadoyy7);
    
    
}

/**
 * @brief  Calback que gere o evento de terminar o programa (interface e comunicação)
 *
 * Invoca a função terminate_comunication que comunica com o gestor de comunicações, essa função
 * econtra-se no ficheiro func_inter.c
 * 
 * @param  widget - widget associado ao evento
 * @param  event - evento associado à calback
 * @param  user_data - user data
 */

void bt_calibrar_on(GtkWidget * widget, GdkEventMotion * event, gpointer user_data)
{
    double x0,x1,x2,x3,x4,x5,x6,x7,y0,y1,y2,y3,y4,y5,y6,y7,sensors;
    
    const gchar* sensor;
    
    const gchar* xx0;
    const gchar* xx1;
    const gchar* xx2;
    const gchar* xx3;
    const gchar* xx4;
    const gchar* xx5;
    const gchar* xx6;
    const gchar* xx7;
    
    const gchar* yy0;
    const gchar* yy1;
    const gchar* yy2;
    const gchar* yy3;
    const gchar* yy4;
    const gchar* yy5;
    const gchar* yy6;
    const gchar* yy7;
    
    GtkWidget *senso= GTK_WIDGET(gtk_builder_get_object (builderG, "sensores"));
    
    GtkWidget *x_0= GTK_WIDGET(gtk_builder_get_object (builderG, "x0"));
    GtkWidget *y_0= GTK_WIDGET(gtk_builder_get_object (builderG, "y0"));
    
    GtkWidget *x_1= GTK_WIDGET(gtk_builder_get_object (builderG, "x1"));
    GtkWidget *y_1= GTK_WIDGET(gtk_builder_get_object (builderG, "y1"));
    
    GtkWidget *x_2= GTK_WIDGET(gtk_builder_get_object (builderG, "x2"));
    GtkWidget *y_2= GTK_WIDGET(gtk_builder_get_object (builderG, "y2"));
    
    GtkWidget *x_3= GTK_WIDGET(gtk_builder_get_object (builderG, "x3"));
    GtkWidget *y_3= GTK_WIDGET(gtk_builder_get_object (builderG, "y3"));
    
    GtkWidget *x_4= GTK_WIDGET(gtk_builder_get_object (builderG, "x4"));
    GtkWidget *y_4= GTK_WIDGET(gtk_builder_get_object (builderG, "y4"));
    
    GtkWidget *x_5= GTK_WIDGET(gtk_builder_get_object (builderG, "x5"));
    GtkWidget *y_5= GTK_WIDGET(gtk_builder_get_object (builderG, "y5"));
    
    GtkWidget *x_6= GTK_WIDGET(gtk_builder_get_object (builderG, "x6"));
    GtkWidget *y_6= GTK_WIDGET(gtk_builder_get_object (builderG, "y6"));
    
    GtkWidget *x_7= GTK_WIDGET(gtk_builder_get_object (builderG, "x7"));
    GtkWidget *y_7= GTK_WIDGET(gtk_builder_get_object (builderG, "y7"));
    
    sensor=gtk_entry_get_text((GtkEntry *)senso);
    
    xx0=gtk_entry_get_text((GtkEntry *)x_0);
    yy0=gtk_entry_get_text((GtkEntry *)y_0);
    
    xx1=gtk_entry_get_text((GtkEntry *)x_1);
    yy1=gtk_entry_get_text((GtkEntry *)y_1);
    
    xx2=gtk_entry_get_text((GtkEntry *)x_2);
    yy2=gtk_entry_get_text((GtkEntry *)y_2);
    
    xx3=gtk_entry_get_text((GtkEntry *)x_3);
    yy3=gtk_entry_get_text((GtkEntry *)y_3);
    
    xx4=gtk_entry_get_text((GtkEntry *)x_4);
    yy4=gtk_entry_get_text((GtkEntry *)y_4);
    
    xx5=gtk_entry_get_text((GtkEntry *)x_5);
    yy5=gtk_entry_get_text((GtkEntry *)y_5);
    
    xx6=gtk_entry_get_text((GtkEntry *)x_6);
    yy6=gtk_entry_get_text((GtkEntry *)y_6);
    
    xx7=gtk_entry_get_text((GtkEntry *)x_7);
    yy7=gtk_entry_get_text((GtkEntry *)y_7);
    
    sensors=atof(sensor);
    
    x0=atof(xx0);
    y0=atof(yy0);
    
    x1=atof(xx1);
    y1=atof(yy1);
    
    x2=atof(xx2);
    y2=atof(yy2);
    
    x3=atof(xx3);
    y3=atof(yy3);
    
    x4=atof(xx4);
    y4=atof(yy4);
    
    x5=atof(xx5);
    y5=atof(yy5);
    
    x6=atof(xx6);
    y6=atof(yy6);
    
    x7=atof(xx7);
    y7=atof(yy7);
    

    read_values[0]=(sensors);
    
    read_values[1]=(x0);
    read_values[2]=(y0);
    
    read_values[3]=(x1);
    read_values[4]=(y1);
    
    read_values[5]=(x2);
    read_values[6]=(y2);
    
    read_values[7]=(x3);
    read_values[8]=(y3);
    
    read_values[9]=(x4);
    read_values[10]=(y4);
    
    read_values[11]=(x5);
    read_values[12]=(y5);
    
    read_values[13]=(x6);
    read_values[14]=(y6);
    
    read_values[15]=(x7);
    read_values[16]=(y7);
    
    read_values[17]=1;
    gtk_main_quit();
}
    
}

class Interface
{
public:
    /**
     \brief start
     Function responsible for init the interface.
     */
    void start()
    {
        int startup;
        
        gtk_init(0,0);
        builderG = gtk_builder_new ();
        /* load the interface from a file return positive number if no error or 0 if got an error*/
        startup=gtk_builder_add_from_file(builderG,"home/sergio/workincopies/lar4/src/sensors/optoelectric/src/Interface/calibrar.glade", NULL);
        if( ! startup ) // se for zero entra na função que dá erro.
        {
            g_print("%s File was not found. Aborting!\n", "/home/sergio/workingcopies/lar4/src/sensors/optoelectric/src/Interface/calibrar.glade");
        }
        //     cout << "conecting signals" << endl;
        /* connect the signals in the interface */
        gtk_builder_connect_signals (builderG, NULL);
        //     cout << "conecting signals out" << endl;
        /* get main window ID and connect special signals */
        /* This is necessary because names must be read from the user GUI file*/
        GtkWidget *m = GTK_WIDGET(gtk_builder_get_object (builderG, "Main"));
        /*Connect the desireg interface signals*/
        
        //     cout <<"GTK WIDGET: " << m << endl;
        
        if(m)
        {
            g_signal_connect(G_OBJECT(m), "delete-event", G_CALLBACK(delete_handler), NULL);
        }
        
        //     cout << "Ciclo on!" << endl;
        gtk_main();
    }
    
};
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
           
        for(uint s = 0;s<8;s++)
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
        
        
        last_values[0]=number_sensors;
        
        for(uint nn = 0;nn<number_sensors;nn++)
        {
            last_values[2*nn+1]=sensors_positions[nn](0);
            last_values[2*nn+2]=sensors_positions[nn](1);
        }
        
    }
    /**
     \brief getPlane
     Function responsible for calculate the plane acording to the given points.
     * @param  values - vector(double) with the sensors readings.
     * @return Plane with the plane coeficients.
     */
    Plane getPlane(vector<double> values)
    {
        Eigen::VectorXd g(number_sensors);
        g = Eigen::VectorXd::Ones(number_sensors);
        
        //cout<<"g: "<<g<<endl;
        
        Eigen::MatrixXd F(number_sensors,3);
        
//         cout << "size: " <<values.size()<< endl;
        
        for(uint i=0;i<number_sensors;i++)
        {
            F(i,0) = read_values[2*i+1];
            F(i,1) = read_values[2*i+2];
            F(i,2) = values[i];
        }
        
//         cout << "F: " << F << endl;
        
        Eigen::VectorXd r(3);
        
       
        
        r = ((F.transpose()*F).inverse() * F.transpose())*g;
        
//         cout << "r: " << r << endl;
        
        double zp = ((-r(0)/r(2))*sensors_positions[0](0)) - ((r(1)/r(2))*sensors_positions[0](1));
        
        double d = values[0] - zp;
        double a = r(0)*d;
        double b = r(1)*d;
        double c = r(2)*d;
        
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
        return ((180./M_PI)*(atan2(d,(d*c/a))));        
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
        return ((180./M_PI)*(atan2(d,(d*c/b))));    
    }
    
    /**
     \brief writeParameters
     Function responsible for write the parameters to yaml file.
     * @param  parameters_file - file url.
     */
    void writeParameters(string parameters_file)
    {
        double pitch_h=0;
        double roll_h=0;
        
        for (uint m=0; m<pitch_bias.size(); m++)
        {
            pitch_h = pitch_h + pitch_bias[m];
        }
        
        for (uint m=0; m<roll_bias.size(); m++)
        {
            roll_h = roll_h + roll_bias[m];
        }
        
        pitch_h = pitch_h / pitch_bias.size();
        roll_h = roll_h / roll_bias.size();
        
//         cout << "pitch: " << pitch_h << " roll: " << roll_h << endl;
        
        // Open yaml file
        
        cv::FileStorage parameters(parameters_file.c_str(),cv::FileStorage::WRITE);
        
        parameters << "number_sensors" << read_values[0] << "pitch_bias" << pitch_h << "roll_bias" << roll_h << "x0" << read_values[1] << "y0" << read_values[2] << "x1" << read_values[3] << "y1" << read_values[4] << "x2" << read_values[5] << "y2" << read_values[6] << "x3" << read_values[7] << "y3" << read_values[8] << "x4" << read_values[9] << "y4" << read_values[10] << "x5" << read_values[11] << "y5" << read_values[12] << "x6" << read_values[13] << "y6" << read_values[14] << "x7" << read_values[15] << "y7" << read_values[16] << "p1_0" << sensors_curve_parameters[0](0)<< "p2_0" << sensors_curve_parameters[0](1)<< "p3_0" << sensors_curve_parameters[0](2) << "p4_0" << sensors_curve_parameters[0](3)<< "p1_1" << sensors_curve_parameters[1](0)<< "p2_1" << sensors_curve_parameters[1](1) << "p3_1" << sensors_curve_parameters[1](2)<< "p4_1" << sensors_curve_parameters[1](3)<< "p1_2" << sensors_curve_parameters[2](0) << "p2_2" << sensors_curve_parameters[2](1)<< "p3_2" << sensors_curve_parameters[2](2)<< "p4_2" << 
sensors_curve_parameters[2](3) << "p1_3" << sensors_curve_parameters[3](0)<< "p2_3" << sensors_curve_parameters[3](1)<< "p3_3" << sensors_curve_parameters[3](2) << "p4_3" << sensors_curve_parameters[3](3)<< "p1_4" << sensors_curve_parameters[4](0)<< "p2_4" << sensors_curve_parameters[4](1) << "p3_4" << sensors_curve_parameters[4](2)<< "p4_4" << sensors_curve_parameters[4](3)<< "p1_5" << sensors_curve_parameters[5](0) << "p2_5" << sensors_curve_parameters[5](1)<< "p3_5" << sensors_curve_parameters[5](2)<< "p4_5" << sensors_curve_parameters[5](3) << "p1_6" << sensors_curve_parameters[6](0)<< "p2_6" << sensors_curve_parameters[0](1)<< "p3_6" << sensors_curve_parameters[6](2) << "p4_6" << sensors_curve_parameters[6](3)<< "p1_7" << sensors_curve_parameters[7](0)<< "p2_7" << sensors_curve_parameters[7](1) << "p3_7" << sensors_curve_parameters[7](2)<< "p4_7" << sensors_curve_parameters[7](3);
        
        parameters.release();
        
    }
    
    vector<double> pitch_bias;
    vector<double> roll_bias;
    
    
private:
    
    uint number_sensors;
    vector<Eigen::Vector2d> sensors_positions;
    vector<Eigen::VectorXd> sensors_curve_parameters;
    
};
/**
 \brief OptoelectricVehiclePose
 
 This class is responsible for subscribe to sensors message and calculate the calibration of the system.
 */
class OptoelectricVehiclePose
{
public:
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
        
         // iniciar leitura dos dados yaml
         s.readParameters(parameters_url);
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
        sharp_values[i]=msg->range;
        new_data[i] = true;
    }
    /**
     \brief loop
     Function responsible to put the program running forever.
     */
    void loop()
    {
        ros::Rate r(100);
        
        int count=0;
        
        while(ros::ok() && (count<51))
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
            }
            
            Plane plane = s.getPlane(values);
             
            double pitch = s.getPitch(plane);
            double roll = s.getRoll(plane);
            
            s.pitch_bias.push_back(pitch);
            s.roll_bias.push_back(roll);
            
            new_data.clear();
            new_data.resize(8,false);
            
            count ++;
            
        }
        
        s.writeParameters(parameters_url);
    }
    
private:
    
    ros::NodeHandle nh_;
        
    SharpMath s;
    
    vector<ros::Subscriber> sharp_subscriber;
    
    int sharp_values[8];
    vector<bool> new_data;
    
    string parameters_url;
};
/**
 \brief main
 Init ros and stays in loop forever.
 */
int main(int argc, char* argv[])
{
    Interface inter;
    
    // Iniciar o ros
    ros::init(argc, argv, "calibration_orientation");
    // Handle
    ros::NodeHandle nh("~");
    
    // inciar
    OptoelectricVehiclePose opto(nh);
    
    inter.start();

    if (read_values[17]==1)
    {
        opto.loop();
    }
    
    return 0;
}
