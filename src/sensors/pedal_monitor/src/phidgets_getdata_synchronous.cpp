/**************************************************************************************************
 Software License Agreement (BSD License)

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
\brief Read data from phidget interfacekit.

This script reads the data,from the pedals, of the phidget interfacekit.
*/

#include "phidgets_getdata_synchronous.h"

using namespace std;


#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>

using boost::property_tree::ptree;


/* Calibration and raw values */
double p1_calib_value_LC1, p2_calib_value_LC1, p3_calib_value_LC1;
double p1_calib_value_FSR1, p2_calib_value_FSR1, p3_calib_value_FSR1;
double p1_calib_value_IR1, p2_calib_value_IR1, p3_calib_value_IR1;

double p1_value_LC1, p2_value_LC1, p3_value_LC1;
double p1_value_FSR1, p2_value_FSR1, p3_value_FSR1;
double p1_value_IR1, p2_value_IR1, p3_value_IR1; 

// XML FILES
class Calibration
{
    public:
        
        double max;
        double min;
};

class Sensor
{
    public:
        
        typedef boost::shared_ptr<Sensor> Ptr;
        
        string name;
        string type;
        Calibration calibration;

};

class Pedal
{
    public:
        
        typedef boost::shared_ptr<Pedal> Ptr;
        
        vector<Sensor::Ptr> sensors;
        string name;
};

ostream& operator<<(ostream& o, Sensor::Ptr& sensor)
{
    o<<"Sensor: "<<sensor->name<<" ";
    o<<"Type: "<<sensor->type<<" ";
    o<<"Min: "<<sensor->calibration.min<<" ";
    o<<"Max: "<<sensor->calibration.max<<" ";
    return o;
}

const ptree& empty_ptree()
{
    static ptree t;
    return t;
}


/**
\brief Read data from the XML file
*/
vector<Pedal::Ptr> readPedalCalibration(string file_name,string& user_name, ptree& pt)
{
    vector<Pedal::Ptr> pedals;
    
    // Try to read the file, if can't files doesn't exist!
    try
    {
        read_xml(file_name, pt, boost::property_tree::xml_parser::trim_whitespace);
    }catch(boost::property_tree::xml_parser::xml_parser_error& error)
    {
        cout<<"Cannot read XML file:\n\t"<<error.what()<<endl;
        return pedals;
    }
    
    user_name = pt.get<std::string>("calibration.<xmlattr>.user_name");
    
    cout<<"Load calibration for user: "<<user_name<<endl;
    
    BOOST_FOREACH(ptree::value_type const& v, pt.get_child("calibration"))
    {    
        if(v.first == "pedal")
        {
            Pedal::Ptr pedal(new Pedal);
            
            pedal->name = v.second.get<string>("name");
            
            for(auto it=v.second.begin();it!=v.second.end();it++)
            {
                if(it->first == "sensor")
                {
                    Sensor::Ptr sensor(new Sensor);
                    
                    sensor->name = it->second.get<string>("name");
                    sensor->type = it->second.get<string>("type");
                    sensor->calibration.max = it->second.get<double>("max");
                    sensor->calibration.min = it->second.get<double>("min");
                    
                    pedal->sensors.push_back(sensor);
                }
            }
            
            pedals.push_back(pedal);
        }
    }
    
    return pedals;
}

/**
\brief Overwrite an existing XML file
*/
void writeExistingCalibrationFile(string file_name, ptree& pt,const vector<Pedal::Ptr>& pedals)
{
    boost::property_tree::ptree root_node;

    for(auto it = pt.begin(); it != pt.end();it++)
    {
//         cout<<it->first<<endl; // debug
        if(it->first == "calibration")
        {
            for(auto cc_it = it->second.begin();cc_it!=it->second.end();cc_it++)//cc_it - calibration child iterator
            {
//                 cout<<"  "<<cc_it->first<<endl; // debug
                if(cc_it->first=="pedal")
                {
                    //get the pedal name
                    string pedal_name = cc_it->second.get<string>("name");
                    
                    Pedal::Ptr pedal;
                    find_if(pedals.begin(),pedals.end(),[&](const Pedal::Ptr& p){if(p->name == pedal_name){pedal = p; return true;}});
                                        
                    for(auto pc_it = cc_it->second.begin(); pc_it!=cc_it->second.end();pc_it++)//pc_it - pedal child iterator
                    {
//                         cout<<"    "<<pc_it->first<<endl; // debug
                        if(pc_it->first == "sensor")
                        {
                            string sensor_name = pc_it->second.get<string>("name");
                            
                            Sensor::Ptr sensor;
                            find_if(pedal->sensors.begin(),pedal->sensors.end(),[&](const Sensor::Ptr& s){if(s->name == sensor_name){sensor = s; return true;}});
                            
                            for(auto sc_it = pc_it->second.begin(); sc_it!=pc_it->second.end();sc_it++)
                            {
                                if(sc_it->first=="type")
                                    sc_it->second.put_value(sensor->type);
                                
                                if(sc_it->first=="min")
                                    sc_it->second.put_value(sensor->calibration.min);
                                
                                if(sc_it->first=="max")
                                    sc_it->second.put_value(sensor->calibration.max);
                            }
                        }
                    }
                }
            }
        }
    }
    
    boost::property_tree::xml_writer_settings<char> settings('\t', 1);
    
    // Write the property tree to the XML file.
    write_xml(file_name, pt,std::locale(), settings);
}

/**
\brief Write the XML file for the first time
*/
void writeCalibrationFile(string file_name,const vector<Pedal::Ptr>& pedals, const string& user)
{
    ptree pt;
    
    boost::property_tree::ptree root_node;
   
    root_node.add("<xmlattr>.user_name",user);
    for(uint i=0;i<pedals.size();i++)
    {
        boost::property_tree::ptree pedal;
        
        pedal.add("name",pedals[i]->name);
        
        for(uint h=0;h<pedals[i]->sensors.size();h++)
        {
            boost::property_tree::ptree sensor;

            sensor.add("name", pedals[i]->sensors[h]->name);
            sensor.add("type", pedals[i]->sensors[h]->type);
            sensor.add("min", pedals[i]->sensors[h]->calibration.min);
            sensor.add("max", pedals[i]->sensors[h]->calibration.max);
            
            pedal.add_child("sensor",sensor);
        }
        
        
        root_node.add_child("pedal",pedal);
//         root_node.add_child("<xmlattr>.user_name",pedal);
    }
    
    pt.add_child("calibration",root_node);
    
    boost::property_tree::xml_writer_settings<char> settings('\t', 1);
    
    // Write the property tree to the XML file.
    write_xml(file_name, pt,std::locale(), settings);
}


/////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////


int CCONV CAttachHandler(CPhidgetHandle IFK, void *userptr)
{
    ((PhidgetClass *)userptr)->AttachHandler(IFK);
    return 0;
}
int CCONV CDetachHandler(CPhidgetHandle IFK, void *userptr)
{
    ((PhidgetClass *)userptr)->DetachHandler(IFK);
    return 0;
}
int CCONV CErrorHandler(CPhidgetHandle IFK, void *userptr, int ErrorCode, const char *unknown)
{
    ((PhidgetClass *)userptr)->ErrorHandler(IFK,ErrorCode,unknown); 
    return 0;
}

PhidgetClass::PhidgetClass(boost::asio::io_service& io_service,ros::NodeHandle& nh_):
ir_timer(io_service),
high_rate_timer(io_service),
nh(nh_)
{
    ir1_index = 3; //set the index of the infrared sensor 1
    ir2_index = 7; //set the index of the infrared sensor 2
    
    fsr1_index = 1; //set the index of the infrared sensor 1
    fsr2_index = 2; //set the index of the infrared sensor 2
    fsr3_index = 5; //set the index of the infrared sensor 3
    fsr4_index = 6; //set the index of the infrared sensor 4
    
    lc12_index = 0; //set the index of the infrared sensor 1
    lc34_index = 4; //set the index of the infrared sensor 2
    
    // define low rate period
    ir_period = 25; //milliseconds
           
    // now schedule the low rate timer.
    ir_timer.expires_from_now(boost::posix_time::milliseconds(ir_period));
    ir_timer.async_wait(boost::bind(&PhidgetClass::irHandler,this,boost::asio::placeholders::error));
    
    
    // define high rate period
    high_rate_period = 10; //milliseconds
        
    // now schedule the high rate timer.
    high_rate_timer.expires_from_now(boost::posix_time::milliseconds(high_rate_period));
    high_rate_timer.async_wait(boost::bind(&PhidgetClass::highRateHandler,this,boost::asio::placeholders::error));
    
  
    ir1_publisher = nh_.advertise<sensor_msgs::Range>("IR_1", 1);
    ir2_publisher = nh_.advertise<sensor_msgs::Range>("IR_2", 1);
    ForceVal_publisher = nh_.advertise<pressure_cells::SenVal>("ForceVal", 1);
    LC_marker_pub = nh_.advertise<visualization_msgs::Marker>( "LC_marker_data", 1); 
    FSR_marker_pub = nh_.advertise<visualization_msgs::Marker>( "FSR_marker_data", 1); 
    
    double LC1_data=0.01;
    double FSR1_data=0.01;
    
    LC_pub();
    FSR_pub();
    
    //Init message fields
    ir1.radiation_type = sensor_msgs::Range::INFRARED;
    ir1.field_of_view = 0.1745;
    ir1.min_range = 0; // meters
    ir1.max_range = 0.3; // meters
    ir2.radiation_type = sensor_msgs::Range::INFRARED;
    ir2.field_of_view = 0.1745;
    ir2.min_range = 0;
    ir2.max_range = 0.3;
    ir2.range = 0;

    ForceVal.sen1=0; // sen1 = LC1
    ForceVal.sen2=0; // sen2 = LC2
    ForceVal.sen3=0; // sen3 = FSR1
    ForceVal.sen4=0; // sen4 = FSR2
    
}

/**
\brief Markers declaration
*/
void PhidgetClass::LC_pub()
{
    LC_marker.header.frame_id = "base_link";
    LC_marker.header.stamp = ros::Time();
    
    LC_marker.ns = "LC_data";
    LC_marker.id = 0;
    LC_marker.type = visualization_msgs::Marker::CUBE;
    LC_marker.action = visualization_msgs::Marker::ADD;
    LC_marker.scale.x = 1;
    LC_marker.scale.y = 0.1;
    LC_marker.scale.z = LC1_data*4;
    LC_marker.pose.position.x = -3;
    LC_marker.pose.position.y = 0;
    LC_marker.pose.position.z = LC_marker.scale.z*0.5;
    LC_marker.pose.orientation.x = 0.0;
    LC_marker.pose.orientation.y = 0.0;
    LC_marker.pose.orientation.z = 0.0;
    LC_marker.pose.orientation.w = 0.0;
    LC_marker.color.a = 1.0;
    LC_marker.color.r = 0.0;
    LC_marker.color.g = 1.0;
    LC_marker.color.b = 0.0;
    
    LC_marker_pub.publish(LC_marker);
}

void PhidgetClass::FSR_pub()
{
    FSR_marker.header.frame_id = "base_link";
    FSR_marker.header.stamp = ros::Time();
    
    FSR_marker.ns = "FSR_data";
    FSR_marker.id = 0;
    FSR_marker.type = visualization_msgs::Marker::CUBE;
    FSR_marker.action = visualization_msgs::Marker::ADD;
    FSR_marker.scale.x = 1;
    FSR_marker.scale.y = 0.1;
    FSR_marker.scale.z = FSR1_data*4;
    FSR_marker.pose.position.x = -1.5;
    FSR_marker.pose.position.y = 0;
    FSR_marker.pose.position.z = FSR_marker.scale.z*0.5;
    FSR_marker.pose.orientation.x = 0.0;
    FSR_marker.pose.orientation.y = 0.0;
    FSR_marker.pose.orientation.z = 0.0;
    FSR_marker.pose.orientation.w = 0.0;
    FSR_marker.color.a = 1.0;
    FSR_marker.color.r = 0.0;
    FSR_marker.color.g = 0.0;
    FSR_marker.color.b = 1.0;
    
    FSR_marker_pub.publish(FSR_marker);
}


/**
\brief IR Handler - Low rate 40Hz
*/
void PhidgetClass::irHandler(boost::system::error_code const& cError)
{
    if (cError.value() == boost::asio::error::operation_aborted)
        return;

    if (cError && cError.value() != boost::asio::error::operation_aborted)
        return; // throw an exception?

//     cout << "Read IR sensor: "<< 1000./ir_period << "hz" << endl;
    
    readIRSensors();
    calibrateIRSensors();
    publishIRValues();

    // Schedule the timer again...
    ir_timer.expires_from_now(boost::posix_time::milliseconds(ir_period));
    ir_timer.async_wait(boost::bind(&PhidgetClass::irHandler, this, boost::asio::placeholders::error));
}
        

/**
\brief LC/FSR Handler - High rate 1000Hz
*/
void PhidgetClass::highRateHandler(boost::system::error_code const& cError)
{
    if (cError.value() == boost::asio::error::operation_aborted)
        return;

    if (cError && cError.value() != boost::asio::error::operation_aborted)
        return; // throw an exception?
    
//     cout << "High rate sensor: "<< 1000./high_rate_period << "hz" << endl;

    readFSRSensors();
    calibrateFSRSensors();
    publishFSRValues();
    
    readLCSensors();
    calibrateLCSensors();
    publishLCValues();
        
    // Schedule the timer again...
    high_rate_timer.expires_from_now(boost::posix_time::milliseconds(high_rate_period));
    high_rate_timer.async_wait(boost::bind(&PhidgetClass::highRateHandler, this, boost::asio::placeholders::error));
}


void PhidgetClass::start()
{
    CPhidgetInterfaceKit_create(&ifKit);
    CPhidget_set_OnAttach_Handler((CPhidgetHandle)ifKit, CAttachHandler, this);
    CPhidget_set_OnDetach_Handler((CPhidgetHandle)ifKit, CDetachHandler, this);
    CPhidget_set_OnError_Handler((CPhidgetHandle)ifKit, CErrorHandler, this);
    CPhidget_open((CPhidgetHandle)ifKit, -1);
}

int PhidgetClass::AttachHandler(CPhidgetHandle IFK)
{
    int serialNo;
    const char *name;

    CPhidget_getDeviceName(IFK, &name);
    CPhidget_getSerialNumber(IFK, &serialNo);

    printf("%s %10d attached!\n", name, serialNo);

    return 0;
}

int PhidgetClass::DetachHandler(CPhidgetHandle IFK)
{
    int serialNo;
    const char *name;

    CPhidget_getDeviceName (IFK, &name);
    CPhidget_getSerialNumber(IFK, &serialNo);

    printf("%s %10d detached!\n", name, serialNo);

    return 0;
}

int PhidgetClass::ErrorHandler(CPhidgetHandle IFK, int ErrorCode, const char *unknown)
{
    printf("Error handled. %d - %s", ErrorCode, unknown);

    return 0;
}


/**
\brief IR Sensor FUNCTIONS
*/
int PhidgetClass::readIRSensors()
{
    CPhidgetInterfaceKit_getSensorValue(ifKit,ir1_index,&ir1_value);
    CPhidgetInterfaceKit_getSensorValue(ifKit,ir2_index,&ir2_value);

//     cout<<"IR1: "<<ir1_value<<endl;
    // cout<<"sensor: "<<ir2_index<<" value: "<<ir2_value<<endl;
}


int PhidgetClass::calibrateIRSensors()
{
    // Pedal 1:
    if(ir1_value<80)
    {
        ir1_v = 0.3;     
    }
    else if(ir1_value>530)
    {
        ir1_v = 0.04;
    }
    else
    {
        ir1_v = 20.76/(ir1_value-11);
    }
            
    p1_value_IR1=(ir1_v-0.04)/(0.3-0.04); 
                   
//     p1_calib_value_IR1=(p1_value_IR1-ir1_min)/(ir1_max-ir1_min);
    
    
    // Pedal 2:
    if(ir2_value<80)
    {
        ir2_v = 0.3;     
    }
    else if(ir2_value>530)
    {
        ir2_v = 0.04;
    }
    else
    {
        ir2_v = 20.76/(ir2_value-11);
    }
                    
    p2_value_IR1=(ir2_v-0.04)/(0.3-0.04); 
    
//     p2_calib_value_IR1=(p2_value_IR1-ir2_min)/(ir2_max-ir2_min);
    
    
    ir1.range = p1_value_IR1;
    ir2.range = p2_value_IR1;
    
    p3_value_IR1=p1_value_IR1;
}   
        

void PhidgetClass::publishIRValues()
{
    //Publish readings in meters
    ir1.header.frame_id = "base_link";
    ir1.header.stamp = ros::Time::now();
    ir1_publisher.publish(ir1);
    
    ir2.header.frame_id = "base_link";
    ir2.header.stamp = ros::Time::now();
    ir2_publisher.publish(ir2);
}


/**
\brief FSR FUNCTIONS
*/
int PhidgetClass::readFSRSensors()
{
    CPhidgetInterfaceKit_getSensorValue(ifKit,fsr1_index,&fsr1_value);
    CPhidgetInterfaceKit_getSensorValue(ifKit,fsr2_index,&fsr2_value);
    CPhidgetInterfaceKit_getSensorValue(ifKit,fsr3_index,&fsr3_value);
    CPhidgetInterfaceKit_getSensorValue(ifKit,fsr4_index,&fsr4_value);
    
//    cout<<"FSR_1: "<<fsr1_value<<endl;
//    cout<<"sensor: "<<fsr2_index<<" value: "<<fsr2_value<<endl;
}


int PhidgetClass::calibrateFSRSensors()
{   
    int fsr_value1;
    fsr_value1=(fsr1_value+fsr2_value)/2;
    
    int fsr_value2;
    fsr_value2=(fsr3_value+fsr4_value)/2;
    
    
    //Calibration
        // Pedal 1:
        fsr1_v=fsr_value1/fsr1_max;
        fsr1_vmin=fsr1_min/fsr1_max;
        fsr1_vmax=fsr1_max/fsr1_max;
        
        FSR1_data=(fsr1_v-fsr1_vmin)/(fsr1_vmax-fsr1_vmin);

        if (FSR1_data<=fsr1_vmin)
        {
            FSR1_data=0.;
            
        }
        else if (FSR1_data>=fsr1_vmax)
        {
            FSR1_data=1.;
            
        }
        
        p1_calib_value_FSR1=FSR1_data; 
        p1_value_FSR1=0.;
        
                
        // Pedal 2:
        fsr2_v=fsr_value2/fsr2_max;
        fsr2_vmin=fsr2_min/fsr2_max;
        fsr2_vmax=fsr2_max/fsr2_max;
        
        FSR2_data=(fsr2_v-fsr2_vmin)/(fsr2_vmax-fsr2_vmin);

        if (FSR2_data<=fsr2_vmin)
        {
            FSR2_data=0.;
            
        }
        else if (FSR2_data>=fsr2_vmax)
        {
            FSR2_data=1.;
            
        }
        
        p2_calib_value_FSR1=FSR2_data; 
        p2_value_FSR1=0.;
        
                
        //Debug only
        p3_calib_value_FSR1=p1_calib_value_FSR1;
        p3_value_FSR1=0.;
        
        ForceVal.sen3=p1_calib_value_FSR1;
        ForceVal.sen4=p2_calib_value_FSR1;

}


void PhidgetClass::publishFSRValues()
{
    //Publish readings in meters
    ForceVal.header.frame_id = "base_link";
    ForceVal.header.stamp = ros::Time::now();
    ForceVal_publisher.publish(ForceVal);
    LC_pub();
}


/**
\brief LC FUNCTIONS
*/
int PhidgetClass::readLCSensors()
{
    CPhidgetInterfaceKit_getSensorValue(ifKit,lc12_index,&lc1_value);
    CPhidgetInterfaceKit_getSensorValue(ifKit,lc34_index,&lc2_value);

//     cout<<"LC_1: "<<lc1_value<<endl;
//     cout<<"sensor: "<<lc2_index<<" value: "<<lc2_value<<endl;
}


int PhidgetClass::calibrateLCSensors()
{    
        // Pedal 1:
        lc1_v=lc1_value/lc1_max;
        lc1_vmin=lc1_min/lc1_max;
        lc1_vmax=lc1_max/lc1_max;  
        
        LC1_data=(lc1_v-lc1_vmin)/(lc1_vmax-lc1_vmin);

        if (LC1_data<=lc1_vmin)
        {
            LC1_data=0.;
        }
        else if (LC1_data>=lc1_vmax)
        {
            LC1_data=1.;
        }
        
        p1_calib_value_LC1=LC1_data; 
        p1_value_LC1=0.;
        
                
        // Pedal 2: 
        lc2_v=lc2_value/lc2_max;
        lc2_vmin=lc2_min/lc2_max;
        lc2_vmax=lc2_max/lc2_max;  
        
        LC2_data=(lc2_v-lc2_vmin)/(lc2_vmax-lc2_vmin);

        if (LC2_data<=lc2_vmin)
        {
            LC2_data=0.;
        }
        else if (LC2_data>=lc2_vmax)
        {
            LC2_data=1.;
        }
        
        p2_calib_value_LC1=LC2_data; 
        p2_value_LC1=0.;
        
                
        //Debug only
        p3_calib_value_LC1=p1_calib_value_LC1;
        p3_value_LC1=0.;    
        
        ForceVal.sen1=LC1_data;
        ForceVal.sen2=LC2_data;
}


void PhidgetClass::publishLCValues()
{
    //Publish readings in meters
    ForceVal.header.frame_id = "base_link";
    ForceVal.header.stamp = ros::Time::now();
    ForceVal_publisher.publish(ForceVal);    
    FSR_pub();
}


void get_pedal_data(PhidgetClass& pClass, vector<Pedal::Ptr> pedals, int num_pedals)
{
    //Get data from pedals and save on pClass
    if(num_pedals==3 || num_pedals==2 || num_pedals==1)
    {
        pClass.lc1_max= pedals[0]->sensors[0]->calibration.max;
        pClass.lc1_min= pedals[0]->sensors[0]->calibration.min;
        pClass.fsr1_max= pedals[0]->sensors[1]->calibration.max;
        pClass.fsr1_min= pedals[0]->sensors[1]->calibration.min;
        
        if(num_pedals==3 || num_pedals==2)
        {
            pClass.lc2_max= pedals[1]->sensors[0]->calibration.max;
            pClass.lc2_min= pedals[1]->sensors[0]->calibration.min;
            pClass.fsr2_max= pedals[1]->sensors[1]->calibration.max;
            pClass.fsr2_min= pedals[1]->sensors[1]->calibration.min; 
        }
        if(num_pedals==3)
        {
            pClass.lc3_max= pedals[2]->sensors[0]->calibration.max;
            pClass.lc3_min= pedals[2]->sensors[0]->calibration.min;
            pClass.fsr3_max= pedals[2]->sensors[1]->calibration.max;
            pClass.fsr3_min= pedals[2]->sensors[1]->calibration.min;          
        } 
    }
}


/**
\brief MAIN function
*/
int main(int argc, char* argv[])
{     
    // Ros Init
    ros::init(argc, argv, "pedal_monitor_node");
    ros::NodeHandle node("~");
    
    boost::asio::io_service io_service;
    
    PhidgetClass pClass(io_service,node);
    pClass.start();

    // Get calibration data
    // File name and username
    string file_name;
    node.param("file_name",file_name,std::string("Not found"));

    string user;
    ptree pt;
    
    try
    {
        read_xml(file_name, pt, boost::property_tree::xml_parser::trim_whitespace);
    }catch(boost::property_tree::xml_parser::xml_parser_error& error)
    {
//         cout<<"Cannot read XML file:\n\t"<<error.what()<<endl;
        file_name = ros::package::getPath("pedal_monitor") + "/src/calibration_test.xml";
    }
    
    // Read existing file.xml
    vector<Pedal::Ptr> pedals = readPedalCalibration(file_name,user,pt);
    
//     cout<<"Calibration for user: "<<user<< endl;
    cout<<"Number of pedals: "<<pedals.size()<<endl;
    
    // Get data from pedals
    get_pedal_data(pClass,pedals,pedals.size());
  
     // Fix division when MAX = 0
    if(pClass.lc1_max<=0.)
    {
        pClass.lc1_max=0.01;
    }
    if(pClass.lc2_max<=0.)
    {
        pClass.lc2_max=0.01;
    }
    if(pClass.fsr1_max<=0.)
    {
        pClass.fsr1_max=0.01;
    }
    if(pClass.fsr2_max<=0.)
    {
        pClass.fsr2_max=0.01;
    }
    if(pClass.ir1_max<=0.)
    {
        pClass.ir1_max=0.01;
    }
    if(pClass.ir2_max<=0.)
    {
        pClass.ir2_max=0.01;
    }
       
    boost::thread thread(boost::bind(&boost::asio::io_service::run, &io_service));
    
    ros::spin();
    
    return 0;
}