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

#include "interface_phidgets_getdata_synchronous.h"

/* GTK variables */
GtkBuilder *builderG;
GtkWidget *txt_username;
GtkWidget *n11_pedal,*n21_pedal,*n31_pedal, *o11_pedal,*o21_pedal,*o31_pedal;
GtkWidget *n12_pedal,*n22_pedal,*n32_pedal, *o12_pedal,*o22_pedal,*o32_pedal;
GtkWidget *n13_pedal,*n23_pedal,*n33_pedal, *o13_pedal,*o23_pedal,*o33_pedal;

GtkWidget *calibration_menu, *calibration_new, *calibration_load;
GtkWidget *combo_box, *error_dialog_data_config, *start_config_dialog, *end_config_dialog;
GtkWidget *calibration_error, *continue_button;
GtkWidget *label11,*label21,*label31,*label41,*label51,*label61, *user_label,*pedal_label, *user_label1,*pedal_label1;
GtkWidget *label12,*label22,*label32,*label42,*label52,*label62;
GtkWidget *label13,*label23,*label33,*label43,*label53,*label63;
GtkWidget *instructions_button, *instructions_label, *about_dialog;
GtkWidget *pedal_box1,*pedal_box2,*pedal_box3,*pedal_box4,*pedal_box5,*pedal_box6;
GtkWidget *opendialog, *bt_dialog_cancel, *bt_dialog_open, *bt_dialog_save, *bt_dialog_cancel1, *savedialog; 
GtkFileFilter* fileFilter, *fileFilter1;


/* Variables definition */
char user_name[100];
char num_pedals[100];
char file_name[100];
bool calibrated;
bool save;
int pedal_step;

int config;
int pedal1;
int pedal2;
int pedal3;
int start;

typedef enum 
{
    CALIBRATION_START,
    CALIBRATION_ZERO,
    CALIBRATION_MIN,
    CALIBRATION_MID,
    CALIBRATION_MAX,
    CALIBRATION_END,
    CALIBRATION_FAIL
    
}calibration_step_type;

calibration_step_type calibration_step;
int count_down;
int count_down_id;
int timeout_id;

// Create a vector to check if IR is working properly
std::vector<double> IR1_data;
std::vector<double> IR2_data;
std::vector<double> IR3_data;

int IR_OK;
    
/* Sensor values */
int LC1_pedal1, FSR1_pedal1, IR1_pedal1;
int LC1_pedal2, FSR1_pedal2, IR1_pedal2;
int LC1_pedal3, FSR1_pedal3, IR1_pedal3;

double lc1_max, lc2_max, lc3_max;
double lc1_min, lc2_min, lc3_min;
double lc1_mid, lc2_mid, lc3_mid;
double lc1_zero, lc2_zero, lc3_zero;

double fsr1_max, fsr2_max, fsr3_max;
double fsr1_min, fsr2_min, fsr3_min;
double fsr1_mid, fsr2_mid, fsr3_mid;
double fsr1_zero, fsr2_zero, fsr3_zero;

double ir1_max, ir2_max, ir3_max;
double ir1_min, ir2_min, ir3_min;

/* Calibration and raw values */
double p1_calib_value_LC1, p2_calib_value_LC1, p3_calib_value_LC1;
double p1_calib_value_FSR1, p2_calib_value_FSR1, p3_calib_value_FSR1;
double p1_calib_value_IR1, p2_calib_value_IR1, p3_calib_value_IR1;

double p1_value_LC1, p2_value_LC1, p3_value_LC1;
double p1_value_FSR1, p2_value_FSR1, p3_value_FSR1;
double p1_value_IR1, p2_value_IR1, p3_value_IR1; 


using namespace std;
using boost::property_tree::ptree;

ofstream saved_data;
time_t time_now;
time_t timer;


class Calibration
{
    public:
        
        double zero;
        double min;       
        double mid;       
        double max;
        
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
        gtk_widget_hide(opendialog);
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

PhidgetClass::PhidgetClass(boost::asio::io_service& io_service):
ir_timer(io_service),
high_rate_timer(io_service)
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
    high_rate_period = 1; //milliseconds
        
    // now schedule the high rate timer.
    high_rate_timer.expires_from_now(boost::posix_time::milliseconds(high_rate_period));
    high_rate_timer.async_wait(boost::bind(&PhidgetClass::highRateHandler,this,boost::asio::placeholders::error));
 
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
    
    // Schedule the timer again...
    ir_timer.expires_from_now(boost::posix_time::milliseconds(ir_period));
    ir_timer.async_wait(boost::bind(&PhidgetClass::irHandler, this, boost::asio::placeholders::error));
}
        

/**
\brief LC/FSR Handler - High rate 100Hz
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
    readLCSensors();
    calibrateLCSensors();
  
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
    
//     cout << "IR 1: " << ir1_value << " || IR 2: " << ir2_value << endl;
}

int PhidgetClass::calibrateIRSensors()
{
    //Calibration :: Convert readings to meters (equation valid for sensorValue between 80 and 530)
    if(calibrated==FALSE)
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
            
            p3_value_IR1=p1_value_IR1;
        }

        else if (calibrated==TRUE)
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
                        
            
            if(p1_value_IR1<=ir1_min)
            {
                p1_value_IR1=ir1_min;
            }
            else if(p1_value_IR1>=ir1_max)
            {
                p1_value_IR1=ir1_max;
            }
            
            p1_calib_value_IR1=(p1_value_IR1-ir1_min)/(ir1_max-ir1_min);
            
            
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
            
            
            if(p2_value_IR1<=ir2_min)
            {
                p2_value_IR1=ir2_min;
            }
            else if(p2_value_IR1>=ir2_max)
            {
                p2_value_IR1=ir2_max;
            }
            
            p2_calib_value_IR1=(p2_value_IR1-ir2_min)/(ir2_max-ir2_min);
            
                        
            p3_value_IR1=p1_value_IR1;
            p3_calib_value_IR1=p1_calib_value_IR1;
        }   
        
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

//     cout << "FSR 1: " << fsr1_value << " || FSR 2: " << fsr2_value << "FSR 3: " << fsr3_value << " || FSR 4: " << fsr4_value << endl;
    
}

int PhidgetClass::calibrateFSRSensors()
{   
    int fsr_value1;
    fsr_value1=(fsr1_value+fsr2_value)/2;
    
    int fsr_value2;
    fsr_value2=(fsr3_value+fsr4_value)/2;
    
    
    //Calibration
    if(calibrated)
    { 
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
        
    }
    else
    {
        // Pedal 1:
        FSR1_data=fsr_value1/1023.;
        
        if (FSR1_data <= 0.)
        {
            FSR1_data=0.;
        }
        else if (FSR1_data >= 1.)
        {
            FSR1_data=1.;
        }
        
       
        p1_value_FSR1=FSR1_data; 
        p1_calib_value_FSR1=0.; 
         
        
        // Pedal 2:
        FSR2_data=fsr_value2/1023.;
        
        if (FSR2_data <= 0.)
        {
            FSR2_data=0.;
        }
        else if (FSR2_data >= 1.)
        {
            FSR2_data=1.;
        }
        
       
        p2_value_FSR1=FSR2_data; 
        p2_calib_value_FSR1=0.; 
        
        
        //Debug only
        p3_value_FSR1=p1_value_FSR1;
        p3_calib_value_FSR1=0.;        
    } 
}


/**
\brief LC FUNCTIONS
*/
int PhidgetClass::readLCSensors()
{
    CPhidgetInterfaceKit_getSensorValue(ifKit,lc12_index,&lc1_value);
    CPhidgetInterfaceKit_getSensorValue(ifKit,lc34_index,&lc2_value);
    
    
//     cout << "LC 1&2: " << lc1_value << " || LC 3&4: " << lc2_value << endl;
     
}

int PhidgetClass::calibrateLCSensors()
{    
    //Calibration
    if(calibrated)
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
    }
    else
    {
        // Pedal 1:
        LC1_data=lc1_value/1023.;
        
        if (LC1_data <= 0.)
        {
            LC1_data= 0.;
        }
        else if (LC1_data >=1.)
        {
            LC1_data = 1.;
        }
        
        p1_value_LC1=LC1_data; 
        p1_calib_value_LC1=0.; 

        
        // Pedal 2:
        LC2_data=lc2_value/1023.;
        
        if (LC2_data <= 0.)
        {
            LC2_data= 0.;
        }
        else if (LC2_data >=1.)
        {
            LC2_data = 1.;
        }
        
        p2_value_LC1=LC2_data; 
        p2_calib_value_LC1=0.; 
        
        //Debug only
        p3_value_LC1=p1_value_LC1;
        p3_calib_value_LC1=0.;        
    }
}


/**
\brief MAIN with interface GTK
*/
int main(int argc, char* argv[])
{    
    /* Reset calibration */
    calibrated=FALSE;
    save=FALSE;
    
    boost::asio::io_service io_service;
    
    PhidgetClass pClass(io_service);
    pClass.start();

    boost::thread thread(boost::bind(&boost::asio::io_service::run, &io_service));
      
    /* Load GTK */
    gtk_init(&argc, &argv);
    
    
    string interface_path = ros::package::getPath("pedal_monitor") + "/src/interface_pedais_remake.glade";
    
    /* Load the interface */
    builderG = gtk_builder_new ();
    gtk_builder_add_from_file (builderG,interface_path.c_str(), NULL);
    
    /* Connect the signals in the interface */
    gtk_builder_connect_signals (builderG, NULL);
    
    /* Calibration menu and connect special signals */
    calibration_menu = GTK_WIDGET(gtk_builder_get_object (builderG, "startup_menu"));
    g_signal_connect(G_OBJECT(calibration_menu), "delete-event", G_CALLBACK(delete_event), NULL);
    gtk_window_set_title(GTK_WINDOW(calibration_menu), "Pedal Calibration Menu");
    
    
    /* New Calibration and connect special signals */
    calibration_new = GTK_WIDGET(gtk_builder_get_object (builderG, "new_configg"));
    error_dialog_data_config = GTK_WIDGET(gtk_builder_get_object (builderG, "invalid_dataconfig"));
    start_config_dialog=GTK_WIDGET(gtk_builder_get_object (builderG, "start_config_dialog"));
    end_config_dialog=GTK_WIDGET(gtk_builder_get_object (builderG, "end_config_dialog"));

    g_signal_connect(G_OBJECT(calibration_new), "delete-event", G_CALLBACK(delete_event), NULL);
    gtk_window_set_title(GTK_WINDOW(calibration_new), "New Calibration Window");
    
    
    /* Load Calibration and connect special signals */
    calibration_load = GTK_WIDGET(gtk_builder_get_object (builderG, "load_configg"));
    g_signal_connect(G_OBJECT(calibration_load), "delete-event", G_CALLBACK(delete_event), NULL);
    gtk_window_set_title(GTK_WINDOW(calibration_load), "Pedal Monitorization Window");
    
    
    /* Username - Combo box nº pedals */
    txt_username = GTK_WIDGET(gtk_builder_get_object(builderG, "txt_username"));
    combo_box = GTK_WIDGET(gtk_builder_get_object(builderG, "num_pedals"));
        
    /* Pedals new calibration widgets */
    n11_pedal= GTK_WIDGET(gtk_builder_get_object(builderG, "progressbar1"));
    g_timeout_add(100,real_time_monitor,n11_pedal);
    n21_pedal= GTK_WIDGET(gtk_builder_get_object(builderG, "progressbar2"));
    g_timeout_add(100,real_time_monitor,n21_pedal);
    n31_pedal= GTK_WIDGET(gtk_builder_get_object(builderG, "progressbar3"));
    g_timeout_add(100,real_time_monitor,n31_pedal);
    n12_pedal= GTK_WIDGET(gtk_builder_get_object(builderG, "progressbar13"));
    g_timeout_add(100,real_time_monitor,n12_pedal);
    n22_pedal= GTK_WIDGET(gtk_builder_get_object(builderG, "progressbar15"));
    g_timeout_add(100,real_time_monitor,n22_pedal);
    n32_pedal= GTK_WIDGET(gtk_builder_get_object(builderG, "progressbar17"));
    g_timeout_add(100,real_time_monitor,n32_pedal);
    n13_pedal= GTK_WIDGET(gtk_builder_get_object(builderG, "progressbar14"));
    g_timeout_add(100,real_time_monitor,n13_pedal);
    n23_pedal= GTK_WIDGET(gtk_builder_get_object(builderG, "progressbar16"));
    g_timeout_add(100,real_time_monitor,n23_pedal);
    n33_pedal= GTK_WIDGET(gtk_builder_get_object(builderG, "progressbar18"));
    g_timeout_add(100,real_time_monitor,n33_pedal);
    
    
    /* Pedal load calibration widgets */
    o11_pedal= GTK_WIDGET(gtk_builder_get_object(builderG, "progressbar4"));
    g_timeout_add(100,real_time_monitor,o11_pedal);
    o21_pedal= GTK_WIDGET(gtk_builder_get_object(builderG, "progressbar5"));
    g_timeout_add(100,real_time_monitor,o21_pedal);
    o31_pedal= GTK_WIDGET(gtk_builder_get_object(builderG, "progressbar6"));
    g_timeout_add(100,real_time_monitor,o31_pedal);
    o12_pedal= GTK_WIDGET(gtk_builder_get_object(builderG, "progressbar7"));
    g_timeout_add(100,real_time_monitor,o12_pedal);
    o22_pedal= GTK_WIDGET(gtk_builder_get_object(builderG, "progressbar9"));
    g_timeout_add(100,real_time_monitor,o22_pedal);
    o32_pedal= GTK_WIDGET(gtk_builder_get_object(builderG, "progressbar11"));
    g_timeout_add(100,real_time_monitor,o33_pedal);
    o13_pedal= GTK_WIDGET(gtk_builder_get_object(builderG, "progressbar8"));
    g_timeout_add(100,real_time_monitor,o13_pedal);
    o23_pedal= GTK_WIDGET(gtk_builder_get_object(builderG, "progressbar10"));
    g_timeout_add(100,real_time_monitor,o23_pedal);
    o33_pedal= GTK_WIDGET(gtk_builder_get_object(builderG, "progressbar12"));
    g_timeout_add(100,real_time_monitor,o33_pedal);

    
    /* Labels - buttons */
    label11= GTK_WIDGET(gtk_builder_get_object(builderG, "sensor1_txt"));
    label21= GTK_WIDGET(gtk_builder_get_object(builderG, "sensor2_txt"));
    label31= GTK_WIDGET(gtk_builder_get_object(builderG, "sensor3_txt"));
    label41= GTK_WIDGET(gtk_builder_get_object(builderG, "sensor4_txt"));
    label51= GTK_WIDGET(gtk_builder_get_object(builderG, "sensor5_txt"));
    label61= GTK_WIDGET(gtk_builder_get_object(builderG, "sensor6_txt"));
    
    label12= GTK_WIDGET(gtk_builder_get_object(builderG, "sensor1_txt1"));
    label22= GTK_WIDGET(gtk_builder_get_object(builderG, "sensor2_txt1"));
    label32= GTK_WIDGET(gtk_builder_get_object(builderG, "sensor3_txt1"));
    label42= GTK_WIDGET(gtk_builder_get_object(builderG, "sensor4_txt1"));
    label52= GTK_WIDGET(gtk_builder_get_object(builderG, "sensor5_txt1"));
    label62= GTK_WIDGET(gtk_builder_get_object(builderG, "sensor6_txt1"));
    
    label13= GTK_WIDGET(gtk_builder_get_object(builderG, "sensor1_txt2"));
    label23= GTK_WIDGET(gtk_builder_get_object(builderG, "sensor2_txt2"));
    label33= GTK_WIDGET(gtk_builder_get_object(builderG, "sensor3_txt2"));
    label43= GTK_WIDGET(gtk_builder_get_object(builderG, "sensor4_txt2"));
    label53= GTK_WIDGET(gtk_builder_get_object(builderG, "sensor5_txt2"));
    label63= GTK_WIDGET(gtk_builder_get_object(builderG, "sensor6_txt2"));
    
    
    user_label= GTK_WIDGET(gtk_builder_get_object(builderG, "user_name_label"));
    pedal_label= GTK_WIDGET(gtk_builder_get_object(builderG, "num_pedals_label"));
    user_label1= GTK_WIDGET(gtk_builder_get_object(builderG, "user_name_label1"));
    pedal_label1= GTK_WIDGET(gtk_builder_get_object(builderG, "num_pedals_label1"));
    
    instructions_label= GTK_WIDGET(gtk_builder_get_object(builderG, "instructions_label"));
    instructions_button= GTK_WIDGET(gtk_builder_get_object(builderG, "save_button2"));
    about_dialog= GTK_WIDGET(gtk_builder_get_object(builderG, "about_dialog"));
    
    
    /* Open dialog */
    opendialog= GTK_WIDGET(gtk_builder_get_object(builderG, "opendialog"));
    fileFilter=gtk_file_filter_new();
    gtk_file_filter_add_pattern(fileFilter,"*.xml");
    gtk_file_chooser_set_filter(GTK_FILE_CHOOSER(opendialog),fileFilter);    
    bt_dialog_cancel= GTK_WIDGET(gtk_builder_get_object(builderG, "bt_dialog_cancel"));
    bt_dialog_open= GTK_WIDGET(gtk_builder_get_object(builderG, "bt_dialog_open"));
    
    /* Save dialog */
    savedialog= GTK_WIDGET(gtk_builder_get_object(builderG, "savedialog"));
    fileFilter1=gtk_file_filter_new();
    gtk_file_filter_add_pattern(fileFilter1,"*.xml");
    gtk_file_chooser_set_filter(GTK_FILE_CHOOSER(savedialog),fileFilter1); 
    bt_dialog_cancel1= GTK_WIDGET(gtk_builder_get_object(builderG, "bt_dialog_cancel1"));
    bt_dialog_save= GTK_WIDGET(gtk_builder_get_object(builderG, "bt_dialog_save"));
    

    /* IR error dialog */
    calibration_error=GTK_WIDGET(gtk_builder_get_object(builderG, "calibration_error"));
    continue_button=GTK_WIDGET(gtk_builder_get_object(builderG, "continue_button"));
    
    /* Pedal boxes */
    pedal_box1= GTK_WIDGET(gtk_builder_get_object(builderG, "pedal_box1"));
    pedal_box2= GTK_WIDGET(gtk_builder_get_object(builderG, "pedal_box2"));
    pedal_box3= GTK_WIDGET(gtk_builder_get_object(builderG, "pedal_box3"));
    pedal_box4= GTK_WIDGET(gtk_builder_get_object(builderG, "pedal_box4"));
    pedal_box5= GTK_WIDGET(gtk_builder_get_object(builderG, "pedal_box5"));
    pedal_box6= GTK_WIDGET(gtk_builder_get_object(builderG, "pedal_box6"));


    /* Start the event loop */
    gtk_main();
        
    return 0;
}


/* =================== GTK FUNCTIONS ==================== */
/**
 * @brief  Function to detect that you are leaving main window.
 *
 * @param  Widget and user data
 * @return void
 */
void delete_event(GtkWidget * window, GdkEvent * event, gpointer data)
{
    gtk_widget_hide(window);
}


/**
 * @brief  Function to open the new calibration dialog.
 *
 * @param  Widget and user data
 * @return void
 */
void callback_open_new(GtkWidget * widget, gpointer user_data)
{
  gtk_widget_show(start_config_dialog); 
}


/**
 * @brief  Function to start a new calibration.
 *
 * This function is called when a new calibration is started.
 * Depending on the number of pedals the function will show that exact number.
 * Reads the calibration values text file and checks if there is already a configuration with
 * that specific user_name and number of pedals, if it's true there will be an error, else it will start the new *configuration.
 * 
 * @param  Widget and user data
 * @return void
 */
void callback_new_config(GtkWidget * widget, gpointer user_data)
{ 
    /* Set default values of data */
    calibrated=FALSE;
    save=FALSE;
    pedal_step=1;
    IR1_data.push_back(1);
    IR2_data.push_back(1);
    IR3_data.push_back(1);
       
    /* Username and pedals */
    strcpy(user_name,gtk_entry_get_text((GtkEntry *)txt_username));
    strcpy(num_pedals,gtk_combo_box_get_active_text(GTK_COMBO_BOX(combo_box)));


    gtk_label_set_text(GTK_LABEL(user_label),user_name);
    gtk_label_set_text(GTK_LABEL(pedal_label),num_pedals);
    gtk_button_set_label(GTK_BUTTON(instructions_button),"Start"); 
    gtk_label_set_text(GTK_LABEL(instructions_label),"Please follow the instructions!\nPress start for new calibration");
    
    /* Change the number of pedals in the Interface */
    if (strcmp(num_pedals,"3")==0)
    {
        gtk_widget_show(pedal_box1);
        gtk_widget_show(pedal_box2);
        gtk_widget_show(pedal_box3);  
    }
    else if (strcmp(num_pedals,"2")==0)
    {
        gtk_widget_show(pedal_box1);
        gtk_widget_show(pedal_box2);
        gtk_widget_hide(pedal_box3);
    }
    else if (strcmp(num_pedals,"1")==0)
    {
        gtk_widget_show(pedal_box1);
        gtk_widget_hide(pedal_box3);
        gtk_widget_hide(pedal_box2);      
    }

    /* Load calibration_new window */
    gtk_widget_hide(calibration_menu);
    gtk_widget_hide(start_config_dialog);
    gtk_widget_show(calibration_new);
}


gboolean button_update(gpointer data)
{
    if (count_down>=1)
    {
        char count[10];
        sprintf(count,"Wait (%d)",count_down);
//         cout << count_down << endl;
        count_down--;
        gtk_button_set_label(GTK_BUTTON(instructions_button),count); 
        
        return true;
    }
    else
    {
        return false;
    }
}

gboolean calibration_control(gpointer data)
{
    switch(calibration_step)
    {
        case CALIBRATION_ZERO:
            
                IR1_data.push_back(1);
                IR2_data.push_back(1);
                IR3_data.push_back(1);
                gtk_label_set_text(GTK_LABEL(instructions_label),"Calibration started:\n Please release the pedal(s).");
                gtk_widget_set_sensitive(instructions_button,false);
                
                calibration_step = CALIBRATION_START;
                return true;
                
                
        case CALIBRATION_START:

                g_source_remove(count_down_id);
                count_down=5;
                count_down_id = g_timeout_add(1000,button_update,NULL);
                
                // ZERO VALUES
                IR1_data.push_back(p1_value_IR1);   
                IR2_data.push_back(p2_value_IR1);
                IR3_data.push_back(p3_value_IR1);
                lc1_zero = p1_value_LC1*1023.;
                fsr1_zero = p1_value_FSR1*1023.;
                lc2_zero = p2_value_LC1*1023.;
                fsr2_zero = p2_value_FSR1*1023.;
                lc3_zero = p3_value_LC1*1023.;
                fsr3_zero = p3_value_FSR1*1023.;

                if(pedal_step==1) // PEDAL 1
                {
                    gtk_label_set_text(GTK_LABEL(instructions_label),"Calibrating Pedal 1:\n Push the pedal gently.");
                    if(IR1_data[IR1_data.size()-2]<IR1_data[IR1_data.size()-1])
                    {
                        g_source_remove(count_down_id);
                        gtk_button_set_label(GTK_BUTTON(instructions_button),"Start"); 
                        gtk_label_set_text(GTK_LABEL(instructions_label),"Calibration error on Pedal 1!\n   Please follow the instructions\n   correctly or change IR sensor\n   orientation.");
                        gtk_widget_show(calibration_error);
                        gtk_widget_set_sensitive(instructions_button,true);
                        
                        calibration_step=CALIBRATION_FAIL;
                        return false;
                    } 
                }
                else if(pedal_step==2) // PEDAL 2
                {
                    gtk_label_set_text(GTK_LABEL(instructions_label),"Calibrating Pedal 2:\n Push the pedal gently.");
                    if(IR2_data[IR2_data.size()-2]<IR2_data[IR2_data.size()-1])
                    {
                        g_source_remove(count_down_id);
                        gtk_button_set_label(GTK_BUTTON(instructions_button),"Start"); 
                        gtk_label_set_text(GTK_LABEL(instructions_label),"Calibration error on Pedal 2!\n   Please follow the instructions\n   correctly or change IR sensor\n   orientation.");
                        gtk_widget_show(calibration_error);
                        gtk_widget_set_sensitive(instructions_button,true);
                        
                        calibration_step=CALIBRATION_FAIL;
                        return false;
                    } 
                }
                else if(pedal_step==3) // PEDAL 3
                {
                    gtk_label_set_text(GTK_LABEL(instructions_label),"Calibrating Pedal 3:\n Push the pedal gently.");
                    if(IR3_data[IR3_data.size()-2]<IR3_data[IR3_data.size()-1])
                    {
                        g_source_remove(count_down_id);
                        gtk_button_set_label(GTK_BUTTON(instructions_button),"Start"); 
                        gtk_label_set_text(GTK_LABEL(instructions_label),"Calibration error on Pedal 3!\n   Please follow the instructions\n   correctly or change IR sensor\n   orientation.");
                        gtk_widget_show(calibration_error);
                        gtk_widget_set_sensitive(instructions_button,true);
                        
                        calibration_step=CALIBRATION_FAIL;
                        return false;
                    } 
                }

                calibration_step = CALIBRATION_MID;
                return true;
            
//         case CALIBRATION_MIN:
//             
//             g_source_remove(count_down_id);
//             count_down=5;
//             count_down_id = g_timeout_add(1000,button_update,NULL);
//                 
//             if(pedal_step==1) // PEDAL 1
//             {
//                 gtk_label_set_text(GTK_LABEL(instructions_label),"Calibrating Pedal 1:\n Push the pedal till the mid.");
//                 
//                 // MIN VALUES
//                 IR1_data.push_back(p1_value_IR1);
//                 lc1_min = p1_value_LC1*1023.;
//                 fsr1_min = p1_value_FSR1*1023.;
//                 ir1_max = p1_value_IR1;
//                 
//                 if(IR1_data[IR1_data.size()-2]<IR1_data[IR1_data.size()-1])
//                 {
//                     g_source_remove(count_down_id);
//                     gtk_button_set_label(GTK_BUTTON(instructions_button),"Start"); 
//                     gtk_label_set_text(GTK_LABEL(instructions_label),"Calibration error on Pedal 1!\n   Please follow the instructions\n   correctly or change IR sensor\n   orientation.");
//                     gtk_widget_show(calibration_error);
//                     gtk_widget_set_sensitive(instructions_button,true);
//                     calibration_step=CALIBRATION_FAIL;
//                     
//                     return false;
//                 }               
//             }
//             else if(pedal_step==2) // PEDAL 2
//             {
//                 gtk_label_set_text(GTK_LABEL(instructions_label),"Calibrating Pedal 2:\n Push the pedal till the mid.");
//                 
//                 // MIN VALUES
//                 IR2_data.push_back(p2_value_IR1);
//                 lc2_min = p2_value_LC1*1023.;
//                 fsr2_min = p2_value_FSR1*1023.;
//                 ir2_max = p2_value_IR1;
//                 
//                 if(IR2_data[IR2_data.size()-2]<IR2_data[IR2_data.size()-1])
//                 {
//                     g_source_remove(count_down_id);
//                     gtk_button_set_label(GTK_BUTTON(instructions_button),"Start"); 
//                     gtk_label_set_text(GTK_LABEL(instructions_label),"Calibration error on Pedal 2!\n   Please follow the instructions\n   correctly or change IR sensor\n   orientation.");
//                     gtk_widget_show(calibration_error);
//                     gtk_widget_set_sensitive(instructions_button,true);
//                     calibration_step=CALIBRATION_FAIL;
//                     
//                     return false;
//                 }               
//             }
//             else if(pedal_step==3) // PEDAL 3
//             {
//                 gtk_label_set_text(GTK_LABEL(instructions_label),"Calibrating Pedal 3:\n Push the pedal till the mid.");
//                 
//                 // MIN VALUES
//                 IR3_data.push_back(p3_value_IR1);
//                 lc3_min = p3_value_LC1*1023.;
//                 fsr3_min = p3_value_FSR1*1023.;
//                 ir3_max = p3_value_IR1;
//                 
//                 if(IR3_data[IR3_data.size()-2]<IR3_data[IR3_data.size()-1])
//                 {
//                     g_source_remove(count_down_id);
//                     gtk_button_set_label(GTK_BUTTON(instructions_button),"Start"); 
//                     gtk_label_set_text(GTK_LABEL(instructions_label),"Calibration error on Pedal 3!\n   Please follow the instructions\n   correctly or change IR sensor\n   orientation.");
//                     gtk_widget_show(calibration_error);
//                     gtk_widget_set_sensitive(instructions_button,true);
//                     calibration_step=CALIBRATION_FAIL;
//                     
//                     return false;
//                 }               
//             }
//                 
//             calibration_step = CALIBRATION_MID;
//             return true;
            
            
        case CALIBRATION_MID:

            g_source_remove(count_down_id);
            count_down=5;
            count_down_id = g_timeout_add(1000,button_update,NULL); 
            
            if(pedal_step==1) // PEDAL 1
            {
                gtk_label_set_text(GTK_LABEL(instructions_label),"Calibrating Pedal 1:\n Push the pedal till the end.");

                // MID VALUES
                IR1_data.push_back(p1_value_IR1);
                lc1_min = p1_value_LC1*1023.;
                fsr1_min = p1_value_FSR1*1023.;
                ir1_max = p1_value_IR1;
                
                if(IR1_data[IR1_data.size()-2]<IR1_data[IR1_data.size()-1])
                {
                    g_source_remove(count_down_id);
                    gtk_button_set_label(GTK_BUTTON(instructions_button),"Start"); 
                    gtk_label_set_text(GTK_LABEL(instructions_label),"Calibration error on Pedal 1!\n   Please follow the instructions\n   correctly or change IR sensor\n   orientation.");
                    gtk_widget_show(calibration_error);
                    gtk_widget_set_sensitive(instructions_button,true);
                    calibration_step=CALIBRATION_FAIL;
                    
                    return false;
                }                
            }            
            else if(pedal_step==2) // PEDAL 2
            {
                gtk_label_set_text(GTK_LABEL(instructions_label),"Calibrating Pedal 2:\n Push the pedal till the end.");

                // MID VALUES
                IR2_data.push_back(p2_value_IR1);
                lc2_min = p2_value_LC1*1023.;
                fsr2_min = p2_value_FSR1*1023.;
                ir2_max = p2_value_IR1;
                
                if(IR2_data[IR2_data.size()-2]<IR2_data[IR2_data.size()-1])
                {
                    g_source_remove(count_down_id);
                    gtk_button_set_label(GTK_BUTTON(instructions_button),"Start"); 
                    gtk_label_set_text(GTK_LABEL(instructions_label),"Calibration error on Pedal 2!\n   Please follow the instructions\n   correctly or change IR sensor\n   orientation.");
                    gtk_widget_show(calibration_error);
                    gtk_widget_set_sensitive(instructions_button,true);
                    calibration_step=CALIBRATION_FAIL;
                    
                    return false;
                }                
            } 
            else if(pedal_step==3) // PEDAL 3
            {
                gtk_label_set_text(GTK_LABEL(instructions_label),"Calibrating Pedal 3:\n Push the pedal till the end.");

                // MID VALUES
                IR3_data.push_back(p3_value_IR1);
                lc3_min = p3_value_LC1*1023.;
                fsr3_min = p3_value_FSR1*1023.;
                ir3_max = p3_value_IR1;
                
                if(IR3_data[IR3_data.size()-2]<IR3_data[IR3_data.size()-1])
                { 
                    g_source_remove(count_down_id);
                    gtk_button_set_label(GTK_BUTTON(instructions_button),"Start"); 
                    gtk_label_set_text(GTK_LABEL(instructions_label),"Calibration error on Pedal 3!\n   Please follow the instructions\n   correctly or change IR sensor\n   orientation.");
                    gtk_widget_show(calibration_error);
                    gtk_widget_set_sensitive(instructions_button,true);
                    calibration_step=CALIBRATION_FAIL;
                    
                    return false;
                }                
            } 
            calibration_step = CALIBRATION_MAX;
            return true;
            
            
        case CALIBRATION_MAX:
            
            g_source_remove(count_down_id);
            count_down=5;
            count_down_id = g_timeout_add(1000,button_update,NULL);
            
            if(strcmp(num_pedals,"1")==0)
            {
                // MAX VALUES
                IR1_data.push_back(p1_value_IR1);
                lc1_max = p1_value_LC1*1023.;
                fsr1_max = p1_value_FSR1*1023.;
                ir1_min = p1_value_IR1;
                
                if(IR1_data[IR1_data.size()-2]<IR1_data[IR1_data.size()-1])
                {
                    g_source_remove(count_down_id);
                    gtk_button_set_label(GTK_BUTTON(instructions_button),"Start"); 
                    gtk_label_set_text(GTK_LABEL(instructions_label),"Calibration error on Pedal 1!\n   Please follow the instructions\n   correctly or change IR sensor\n   orientation.");
                    gtk_widget_show(calibration_error);
                    gtk_widget_set_sensitive(instructions_button,true);
                    
                    calibration_step=CALIBRATION_FAIL;
                    return false;
                }
            
                pedal_step=4;
                
            }
            else if(strcmp(num_pedals,"2")==0)
            {
                if(pedal_step==2)
                {
                    IR2_data.push_back(p2_value_IR1);
                    lc2_max = p2_value_LC1*1023.;
                    fsr2_max = p2_value_FSR1*1023.;
                    ir2_min = p2_value_IR1;
                
                        
                    if(IR2_data[IR2_data.size()-2]<IR2_data[IR2_data.size()-1])
                    {
                        g_source_remove(count_down_id);
                        gtk_button_set_label(GTK_BUTTON(instructions_button),"Start"); 
                        gtk_label_set_text(GTK_LABEL(instructions_label),"Calibration error on Pedal 2!\n   Please follow the instructions\n   correctly or change IR sensor\n   orientation.");
                        gtk_widget_show(calibration_error);
                        gtk_widget_set_sensitive(instructions_button,true);
                        
                        calibration_step=CALIBRATION_FAIL;
                        return false;
                    }
                
                    pedal_step=4;
                }   
                
                pedal_step++;
                if (pedal_step>4) pedal_step=4;
            }
            else if(strcmp(num_pedals,"3")==0)
            {    
                if(pedal_step==3)
                {
                    pedal_step=4;
                    
                    // MAX VALUES
                    IR3_data.push_back(p3_value_IR1);
                    lc3_max = p3_value_LC1*1023.;
                    fsr3_max = p3_value_FSR1*1023.;
                    ir3_min = p3_value_IR1;
                    
                    if(IR3_data[IR3_data.size()-2]<IR3_data[IR3_data.size()-1])
                    {
                        g_source_remove(count_down_id);
                        gtk_button_set_label(GTK_BUTTON(instructions_button),"Start"); 
                        gtk_label_set_text(GTK_LABEL(instructions_label),"Calibration error on Pedal 3!\n   Please follow the instructions\n   correctly or change IR sensor\n   orientation.");
                        gtk_widget_show(calibration_error);
                        gtk_widget_set_sensitive(instructions_button,true);
                        
                        calibration_step=CALIBRATION_FAIL;
                        return false;
                    }
                    
                    pedal_step=4;
                    
                }   
                
                pedal_step++;
                if (pedal_step>4) pedal_step=4;
            }
                 
            if(pedal_step==2)
            {
                // MAX VALUES
                IR1_data.push_back(p1_value_IR1);
                lc1_max = p1_value_LC1*1023.;
                fsr1_max = p1_value_FSR1*1023.;
                ir1_min = p1_value_IR1;
                
                if(IR1_data[IR1_data.size()-2]<IR1_data[IR1_data.size()-1])
                {
                    g_source_remove(count_down_id);
                    gtk_button_set_label(GTK_BUTTON(instructions_button),"Start"); 
                    gtk_label_set_text(GTK_LABEL(instructions_label),"Calibration error on Pedal 1!\n   Please follow the instructions\n   correctly or change IR sensor\n   orientation.");
                    gtk_widget_show(calibration_error);
                    gtk_widget_set_sensitive(instructions_button,true);
                    
                    calibration_step=CALIBRATION_FAIL;
                    return false;
                }
                
                IR2_data.push_back(1);
                gtk_label_set_text(GTK_LABEL(instructions_label),"Preparing pedal 2:\n Please release the pedal(s).");       
                calibration_step = CALIBRATION_START;
                return true;
            }
            else if(pedal_step==3)
            {   
                IR3_data.push_back(1);
                gtk_label_set_text(GTK_LABEL(instructions_label),"Preparing pedal 3:\n Please release the pedal(s).");       
                calibration_step = CALIBRATION_START;
                return true;           
            }
            else if(pedal_step==4)
            {
                gtk_label_set_text(GTK_LABEL(instructions_label),"Processing, it will only\n take a few seconds.");
                calibration_step = CALIBRATION_END;
            }

            return true;
            
            
        case CALIBRATION_FAIL:
          
            if(count_down_id>0) g_source_remove(count_down_id);
            IR1_data.push_back(1);
            IR2_data.push_back(1);
            IR3_data.push_back(1);
            gtk_button_set_label(GTK_BUTTON(instructions_button),"Start"); 
            gtk_label_set_text(GTK_LABEL(instructions_label),"Re-start calibration!\n Please release the pedal(s).");
            
            calibration_step = CALIBRATION_START;
            return false;
            
            
        case CALIBRATION_END:
            
            if(count_down_id>0) g_source_remove(count_down_id);
            gtk_label_set_text(GTK_LABEL(instructions_label),"Calibration sucessfull!");
            gtk_button_set_label(GTK_BUTTON(instructions_button),"Save");
            save=TRUE;
            gtk_widget_show(savedialog);
            gtk_widget_set_sensitive(instructions_button,true);
             
            calibration_step = CALIBRATION_ZERO;
            return false;
            
            
        default:
            
            gtk_widget_set_sensitive(instructions_button,true);
            
            calibration_step = CALIBRATION_ZERO;
            return false;
    }    
}

/**
 * @brief  Function with the instructions for the configuration.
 *
 * @param  Widget and user data
 * @return .txt file with all the configuration data
 */
void callback_set_data(GtkWidget * widget, gpointer user_data)
{
    config = 1;
    
    /* Username and pedals */
    strcpy(user_name,gtk_entry_get_text((GtkEntry *)txt_username));
    strcpy(num_pedals,gtk_combo_box_get_active_text(GTK_COMBO_BOX(combo_box)));
    
    if(save==FALSE)
    {
        // CALIBRATION_ZERO
        IR1_data.push_back(1);
        IR2_data.push_back(1);
        IR3_data.push_back(1);
        gtk_label_set_text(GTK_LABEL(instructions_label),"Calibration started:\n Please release the pedal(s).");
        count_down=5;
        count_down_id = g_timeout_add(1000,button_update,NULL);
        timeout_id = g_timeout_add(6000,calibration_control,NULL);
        calibration_step = CALIBRATION_START;
        gtk_widget_set_sensitive(widget,false);
    }
    else
    {
        gtk_widget_show(savedialog);
    }
    
    return;
     
}


void callback_opendialog_save(GtkWidget * widget, gpointer user_data)
{
        strcpy(file_name,gtk_file_chooser_get_filename (GTK_FILE_CHOOSER(savedialog)));
        save=TRUE;
         
      
        //Write the new file
        vector<Pedal::Ptr> pedals;                   
        
        //Pedal 1
        if(strcmp(num_pedals,"3")==0 || strcmp(num_pedals,"2")==0 || strcmp(num_pedals,"1")==0)
        {
            Pedal::Ptr pedal1(new Pedal);    
            pedal1->name = "clutch";

            Sensor::Ptr sensor11(new Sensor); 
            sensor11->name = "LC1";
            sensor11->type = "LC";
            sensor11->calibration.min = lc1_min;
            sensor11->calibration.max = lc1_max;       
            pedal1->sensors.push_back(sensor11);
            Sensor::Ptr sensor12(new Sensor);
            sensor12->name = "FSR1";
            sensor12->type = "FSR";
            sensor12->calibration.min = fsr1_min;
            sensor12->calibration.max = fsr1_max;
            pedal1->sensors.push_back(sensor12);
            Sensor::Ptr sensor13(new Sensor);
            sensor13->name = "IR1";
            sensor13->type = "IR";
            sensor13->calibration.min = ir1_min;
            sensor13->calibration.max = ir1_max;
            pedal1->sensors.push_back(sensor13);
            
            pedals.push_back(pedal1);
        }
        
        //Pedal 2       
        if(strcmp(num_pedals,"3")==0 || strcmp(num_pedals,"2")==0)
        {
            Pedal::Ptr pedal2(new Pedal);    
            pedal2->name = "brake";

            Sensor::Ptr sensor21(new Sensor); 
            sensor21->name = "LC2";
            sensor21->type = "LC";
            sensor21->calibration.min = lc2_min;
            sensor21->calibration.max = lc2_max;       
            pedal2->sensors.push_back(sensor21);  
            Sensor::Ptr sensor22(new Sensor);
            sensor22->name = "FSR2";
            sensor22->type = "FSR";
            sensor22->calibration.min = fsr2_min;
            sensor22->calibration.max = fsr2_max;
            pedal2->sensors.push_back(sensor22);
            Sensor::Ptr sensor23(new Sensor);
            sensor23->name = "IR2";
            sensor23->type = "IR";
            sensor23->calibration.min = ir2_min;
            sensor23->calibration.max = ir2_max;
            pedal2->sensors.push_back(sensor23);
            
            pedals.push_back(pedal2);
        }
        
        //Pedal 3  
        if(strcmp(num_pedals,"3")==0)
        {
            Pedal::Ptr pedal3(new Pedal);    
            pedal3->name = "throttle";

            Sensor::Ptr sensor31(new Sensor); 
            sensor31->name = "LC3";
            sensor31->type = "LC";
            sensor31->calibration.min = lc3_min;
            sensor31->calibration.max = lc3_max;       
            pedal3->sensors.push_back(sensor31);
            Sensor::Ptr sensor32(new Sensor);
            sensor32->name = "FSR3";
            sensor32->type = "FSR";
            sensor32->calibration.min = fsr3_min;
            sensor32->calibration.max = fsr3_max;
            pedal3->sensors.push_back(sensor32);
            Sensor::Ptr sensor33(new Sensor);
            sensor33->name = "IR3";
            sensor33->type = "IR";
            sensor33->calibration.min = ir3_min;
            sensor33->calibration.max = ir3_max;
            pedal3->sensors.push_back(sensor33);
            
            pedals.push_back(pedal3);
        }

        //Save calibration file
        if(strstr(file_name,".xml"))
        {
            writeCalibrationFile(file_name,pedals,user_name);
        }
        else
        {
            writeCalibrationFile((string)file_name + ".xml",pedals,user_name);       
        }

        gtk_widget_show(end_config_dialog);    
            gtk_widget_hide(savedialog);
}


/**
 * @brief  Function to load an existent calibration.
 *
 * This function is called to load a calibration.
 * Depending on the number of pedals the function will show that exact number.
 * Reads the calibration values text file and checks if there is already a configuration with
 * that specific user_name and number of pedals, if it's true it will load it, else there will be an error.
 * 
 * @param  Widget and user data
 * @return void
 */
void callback_load_config(GtkWidget * widget, gpointer user_data)
{ 
  gtk_widget_hide(end_config_dialog);
  gtk_widget_show(opendialog);
}


void callback_opendialog_cancel(GtkWidget * widget, gpointer user_data)
{
    gtk_widget_hide(opendialog);
    gtk_widget_hide(savedialog);
}


void callback_opendialog_open(GtkWidget * widget, gpointer user_data)
{
    calibrated=TRUE;
    time(&time_now);
    
//     cout << "FILE OPEN" << endl;
    saved_data.open (ros::package::getPath("pedal_monitor") + "/src/recording_data.txt", ofstream::out | ofstream::app);

    /* Get source path of the selected .XML file */
    strcpy(file_name,gtk_file_chooser_get_filename (GTK_FILE_CHOOSER(opendialog)));
//     cout << file_name << endl;
    
    /* Username and filename */
    string user;
    string filename=file_name;
    
    
    // Read existing file.xml
    ptree pt;
    vector<Pedal::Ptr> pedals = readPedalCalibration(filename,user,pt);
         
    /* Number of pedals */
//     cout<<"The calibration file for "<<user<<" has "<<pedals.size()<<" pedals."<<endl; 
    memcpy(user_name,user.c_str(),100);
    sprintf(num_pedals,"%ld",pedals.size());
    
    gtk_label_set_text(GTK_LABEL(user_label1),user_name);
    gtk_label_set_text(GTK_LABEL(pedal_label1),num_pedals);

   
    // Get data from pedals
    if (pedals.size()==1 || pedals.size()==2 || pedals.size()==3)
    {
        // Pedal 1
        lc1_max= pedals[0]->sensors[0]->calibration.max;
        lc1_min= pedals[0]->sensors[0]->calibration.min;
        fsr1_max= pedals[0]->sensors[1]->calibration.max;
        fsr1_min= pedals[0]->sensors[1]->calibration.min;
        ir1_max= pedals[0]->sensors[2]->calibration.max;
        ir1_min= pedals[0]->sensors[2]->calibration.min;
        
        if(pedals.size()==2 || pedals.size()==3)
        {
            // Pedal 2
            lc2_max= pedals[1]->sensors[0]->calibration.max;
            lc2_min= pedals[1]->sensors[0]->calibration.min;
            fsr2_max= pedals[1]->sensors[1]->calibration.max;
            fsr2_min= pedals[1]->sensors[1]->calibration.min;
            ir2_max= pedals[1]->sensors[2]->calibration.max;
            ir2_min= pedals[1]->sensors[2]->calibration.min;
        
            if(pedals.size()==3)
            {

                // Pedal 3
                lc3_max= pedals[2]->sensors[0]->calibration.max;
                lc3_min= pedals[2]->sensors[0]->calibration.min;
                fsr3_max= pedals[2]->sensors[1]->calibration.max;
                fsr3_min= pedals[2]->sensors[1]->calibration.min;
                ir3_max= pedals[2]->sensors[2]->calibration.max;
                ir3_min= pedals[2]->sensors[2]->calibration.min;
        
            }
        }
;
    }
    
    /* Change the number of pedals in the Interface according to the number of pedals on the calib file */
    if (pedals.size()==3)
    {
    gtk_widget_show(pedal_box4);
    gtk_widget_show(pedal_box5);
    gtk_widget_show(pedal_box6);   
    }
    else if (pedals.size()==2)
    {
    gtk_widget_show(pedal_box4);
    gtk_widget_show(pedal_box5);
    gtk_widget_hide(pedal_box6);   
    }
    else if (pedals.size()==1)
    {
    gtk_widget_show(pedal_box4);
    gtk_widget_hide(pedal_box5);
    gtk_widget_hide(pedal_box6);  
    }
           
    /* Load calibration_load window */
    gtk_widget_hide(calibration_new);
    gtk_widget_hide(calibration_menu);
    gtk_widget_hide(opendialog);
    gtk_widget_show(calibration_load); 
    
    // Fix division when MAX = 0
    if(lc1_max<=0.)
    {
        lc1_max=0.01;
    }
    if(lc2_max<=0.)
    {
        lc2_max=0.01;
    }
    if(lc3_max<=0.)
    {
        lc3_max=0.01;
    }
    if(fsr1_max<=0.)
    {
        fsr1_max=0.01;
    }
    if(fsr2_max<=0.)
    {
        fsr2_max=0.01;
    }
    if(fsr3_max<=0.)
    {
        fsr3_max=0.01;
    }
    if(ir1_max<=0.)
    {
    ir1_max=0.01;
    }
    if(ir2_max<=0.)
    {
        ir2_max=0.01;
    }
    if(ir3_max<=0.)
    {
        ir3_max=0.01;
    }
}


/**
 * @brief  Function to get the values in realtime for new config.
 * 
 * @param  Widget and user data
 * @return void
 */
gboolean real_time_monitor(gpointer progressbar)
{            
    if(calibrated==FALSE)
    {
        /* Set values on the progressbar */  
        if (progressbar==n11_pedal || progressbar==n12_pedal || progressbar==n13_pedal)
        {
            gtk_progress_bar_set_fraction(GTK_PROGRESS_BAR(n11_pedal),p1_value_LC1);
            gtk_progress_bar_set_fraction(GTK_PROGRESS_BAR(n12_pedal),p1_value_FSR1);
            gtk_progress_bar_set_fraction(GTK_PROGRESS_BAR(n13_pedal),p1_value_IR1);            
        }
        else if (progressbar==n21_pedal || progressbar==n22_pedal || progressbar==n23_pedal)
        {
            gtk_progress_bar_set_fraction(GTK_PROGRESS_BAR(n21_pedal),p2_value_LC1);    
            gtk_progress_bar_set_fraction(GTK_PROGRESS_BAR(n22_pedal),p2_value_FSR1);
            gtk_progress_bar_set_fraction(GTK_PROGRESS_BAR(n23_pedal),p2_value_IR1); 
        }
       
        else if (progressbar==n31_pedal || progressbar==n32_pedal || progressbar==n33_pedal)
        {
            gtk_progress_bar_set_fraction(GTK_PROGRESS_BAR(n31_pedal),p3_value_LC1);   
            gtk_progress_bar_set_fraction(GTK_PROGRESS_BAR(n32_pedal),p3_value_FSR1);
            gtk_progress_bar_set_fraction(GTK_PROGRESS_BAR(n33_pedal),p3_value_IR1);
        }
  
        
        char txt11[100];
        sprintf(txt11,"%d %%",int(p1_value_LC1*100));
        char txt21[100];
        sprintf(txt21,"%d %%", int(p2_value_LC1*100));
        char txt31[100];
        sprintf(txt31,"%d %%", int(p3_value_LC1*100));
        
        char txt12[100];
        sprintf(txt12,"%d %%", int(p1_value_FSR1*100));
        char txt22[100];
        sprintf(txt22,"%d %%", int(p2_value_FSR1*100));
        char txt32[100];
        sprintf(txt32,"%d %%", int(p3_value_FSR1*100));
        
        char txt13[100];
        sprintf(txt13,"%d %%", int(p1_value_IR1*100));
        char txt23[100];
        sprintf(txt23,"%d %%", int(p2_value_IR1*100));
        char txt33[100];
        sprintf(txt33,"%d %%", int(p3_value_IR1*100));
      
        
        gtk_label_set_text(GTK_LABEL(label41),txt11);
        gtk_label_set_text(GTK_LABEL(label51),txt21);
        gtk_label_set_text(GTK_LABEL(label61),txt31);
        
        gtk_label_set_text(GTK_LABEL(label42),txt12);
        gtk_label_set_text(GTK_LABEL(label52),txt22);
        gtk_label_set_text(GTK_LABEL(label62),txt32);
        
        gtk_label_set_text(GTK_LABEL(label43),txt13);
        gtk_label_set_text(GTK_LABEL(label53),txt23);
        gtk_label_set_text(GTK_LABEL(label63),txt33);
   
        fflush(stdout);
    }
    else if(calibrated)
    {
        /* Set values on the progressbar */  
        if (progressbar==o11_pedal || progressbar==o12_pedal || progressbar==o13_pedal)
        {
            gtk_progress_bar_set_fraction(GTK_PROGRESS_BAR(o11_pedal),p1_calib_value_LC1);
            gtk_progress_bar_set_fraction(GTK_PROGRESS_BAR(o12_pedal),p1_calib_value_FSR1);
            gtk_progress_bar_set_fraction(GTK_PROGRESS_BAR(o13_pedal),p1_calib_value_IR1);            
        }
        else if (progressbar==o21_pedal || progressbar==o22_pedal || progressbar==o23_pedal)
        {
            gtk_progress_bar_set_fraction(GTK_PROGRESS_BAR(o21_pedal),p2_calib_value_LC1);    
            gtk_progress_bar_set_fraction(GTK_PROGRESS_BAR(o22_pedal),p2_calib_value_FSR1);
            gtk_progress_bar_set_fraction(GTK_PROGRESS_BAR(o23_pedal),p2_calib_value_IR1); 
        }
       
        else if (progressbar==o31_pedal || progressbar==o32_pedal || progressbar==o33_pedal)
        {
            gtk_progress_bar_set_fraction(GTK_PROGRESS_BAR(o31_pedal),p3_calib_value_LC1);   
            gtk_progress_bar_set_fraction(GTK_PROGRESS_BAR(o32_pedal),p3_calib_value_FSR1);
            gtk_progress_bar_set_fraction(GTK_PROGRESS_BAR(o33_pedal),p3_calib_value_IR1);
        } 
        
      
        if (saved_data.is_open())
        {
            time(&timer);
            int elapsed_time = (timer-time_now);

            saved_data << elapsed_time << ":LC_pedal1:" << p1_calib_value_LC1 << ":FSR_pedal1:" << p1_calib_value_FSR1 << ":IR_pedal1:" << p1_calib_value_IR1 << ":LC_pedal2:" << p2_calib_value_LC1 << ":FSR_pedal2:" << p2_calib_value_FSR1 << ":IR_pedal2:" << p2_calib_value_IR1<< "\n";
        }
        
        char txt11[100];
        sprintf(txt11,"%d %%", int(p1_calib_value_LC1*100));
        char txt21[100];
        sprintf(txt21,"%d %%", int(p2_calib_value_LC1*100));
        char txt31[100];
        sprintf(txt31,"%d %%", int(p3_calib_value_LC1*100));
        
        char txt12[100];
        sprintf(txt12,"%d %%", int(p1_calib_value_FSR1*100));
        char txt22[100];
        sprintf(txt22,"%d %%", int(p2_calib_value_FSR1*100));
        char txt32[100];
        sprintf(txt32,"%d %%", int(p3_calib_value_FSR1*100));
        
        char txt13[100];
        sprintf(txt13,"%d %%", int(p1_calib_value_IR1*100));
        char txt23[100];
        sprintf(txt23,"%d %%", int(p2_calib_value_IR1*100));
        char txt33[100];
        sprintf(txt33,"%d %%", int(p3_calib_value_IR1*100));
        
        
        gtk_label_set_text(GTK_LABEL(label11),txt11);
        gtk_label_set_text(GTK_LABEL(label21),txt21);
        gtk_label_set_text(GTK_LABEL(label31),txt31);
        
        gtk_label_set_text(GTK_LABEL(label12),txt12);
        gtk_label_set_text(GTK_LABEL(label22),txt22);
        gtk_label_set_text(GTK_LABEL(label32),txt32);
        
        gtk_label_set_text(GTK_LABEL(label13),txt13);
        gtk_label_set_text(GTK_LABEL(label23),txt23);
        gtk_label_set_text(GTK_LABEL(label33),txt33);
   
        fflush(stdout);
    }
    
    return TRUE;   
}


/**
 * @brief  Function to open the about window.
 *
 * @param  Widget and user data
 * @return void
 */
void callback_about(GtkWidget * window, gpointer data)
{
  gtk_widget_show(about_dialog);
  
}


/**
 * @brief  Function to return to main window.
 * 
 * @param  Widget and user data
 * @return void
 */
void callback_back_function(GtkWidget * window, gpointer data)
{
    if (saved_data.is_open())
    {    
//         cout << "FILE CLOSED" << endl;
        saved_data.close();
        
    }
    if(timeout_id>0) g_source_remove(timeout_id);
    if(count_down_id>0) g_source_remove(count_down_id);
    gtk_widget_set_sensitive(instructions_button,true);
    gtk_widget_hide(calibration_load);
    gtk_widget_hide(calibration_new);
    gtk_widget_hide(start_config_dialog);
    gtk_widget_hide(end_config_dialog);
    gtk_widget_show(calibration_menu);
    gtk_widget_hide(opendialog);
}

void callback_IR_error(GtkWidget * widget, gpointer user_data)
{
    gtk_widget_hide(calibration_error);
    IR_OK=0;
}