// This file is part of the tcp/ip client library.
//
// Copyright (C) 2011 LAR - Laboratory for Automation and Robotics, ATLAS Project
//                    Department of Mechanical Engineering
//                    University of Aveiro
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2.1
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
// 02110-1301, USA.

/** @file device_mapper.cpp
* @brief Node that automatically creates parameters for devices paths given a set of rules,  presented in the \c urdf file.
*/

#include <urdf/model.h>
#include "ros/ros.h"
#include <ros/package.h>
#include "udev_control.h"
#include <libxml/encoding.h>
#include <libxml/xmlwriter.h>
#include <libxml/tree.h>
#include <libxml/parser.h>
#include <libxml/xpath.h>
#include <libxml/xpathInternals.h>
#include <libxml/xmlreader.h>


using namespace ros;
using namespace std;
using namespace boost;

///Vector of the connected devices udev classes
vector<shared_ptr<class_udev_control> > devices;

///Local node handler
NodeHandle*nh;

/**
@brief Device add callback

This function handles the addition of a device.
@param action not used
@param node path of the device
@param data id of the device
*/
void AddCallback(string action,string node,void*data)
{
	string*id=(string*)data;//device id
	nh->setParam("/device_mapper/"+*id,node);
	cout<<"Added device id: "<<*id<<" in node "<<node<<" action: "<<action<<endl;
}

/**
@brief Device remove callback

This function handles the removal of a device.
@param action not used
@param node path of the device
@param data id of the device
*/
void RemoveCallback(string action,string node,void*data)
{	
	string*id=(string*)data;//device id
	nh->setParam("/device_mapper/"+*id,"Not found");
	cout<<"Removed device id: "<<*id<<" in node "<<node<<" action: "<<action<<endl;
}

/**
@brief Get the name of a xml element

This function reads the name of the current xml element in the reader.
@param reader xmlTextReaderPtr pointing to the xml object
@param str variable that will store the name
*/
void GetName(xmlTextReaderPtr reader,string& str)
{
	xmlChar*name=xmlTextReaderName(reader);
	if(!name)return;
	str=(char*)name;
	xmlFree(name);
}

/**
@brief Get the value of a xml element

This function reads the value of the current xml element in the reader.
@param reader xmlTextReaderPtr pointing to the xml object
@param str variable that will store the value
*/
void GetValue(xmlTextReaderPtr reader,string& str)
{
	xmlChar*value=xmlTextReaderValue(reader);
	if(!value)return;
	str=(char*)value;
	xmlFree(value);
}

/**
@brief Get the value of an attribute in a xml element

This function reads the value of an attribute in the current xml element in the reader.
@param reader xmlTextReaderPtr pointing to the xml object
@param name name of the attribute to read
@param str variable that will store the value
*/
void GetAttribute(xmlTextReaderPtr reader,const xmlChar*name,string& str)
{
	xmlChar*attribute=xmlTextReaderGetAttribute(reader,name);
	if(!attribute)return;
	str=(char*)attribute;
	xmlFree(attribute);
}

/**
@brief Handler of xml nodes, heavy duty function

This node handler will handle all the nodes in the xml object
searching for specific tags that indicate the presence of the
desired \c SENSOR or \c DEVICE_INFORMATION info. When a new 
\c SENSOR if found a \c udev class is allocated and subsequent
pairs of rules are added to it.
@param reader xmlTextReaderPtr pointing to the xml object
*/
void NodeHandler(xmlTextReaderPtr reader)
{
	string name,value;
	string dev_name;
	GetName(reader,name);
	
	to_upper(name);
	
	if(name=="SENSOR" && xmlTextReaderNodeType(reader)==1)
	{
		GetAttribute(reader,BAD_CAST "name",dev_name);
		
		shared_ptr<class_udev_control> dev(new class_udev_control(dev_name));
		
		dev->RegistryAction("add",AddCallback,&dev->id);
		dev->RegistryAction("remove",RemoveCallback,&dev->id);
		
		devices.push_back(dev);
	}
	
	
	if(name=="DEVICE_INFORMATION" && xmlTextReaderNodeType(reader)==1)
	{
		//from here to the end of this element we must read all names and values and add them as pairs
		
		int ret = xmlTextReaderRead(reader);
		do
		{
			ret=xmlTextReaderRead(reader);
			GetName(reader,name);
			to_upper(name);//boost function
			
			if(xmlTextReaderNodeType(reader)==1)
			{	
				ret=xmlTextReaderRead(reader);//jump to the node that as the value of the past name
				
				GetValue(reader,value);
				
				shared_ptr<class_udev_control> dev = devices.back();
				
				if(name=="SUBSYSTEM")
					dev->AddSubsystem(value);
				else
					dev->AddProperty(name,value);
			}
			
			if(name=="DEVICE_INFORMATION" && xmlTextReaderNodeType(reader)==15)
				break;//this signals that we sould stop addin stuff to the last device

		}while(ret==1);//if the cicle exits by ret!=1 there was an error
	}
}

int main(int argc, char** argv)
{
	std::string urdf_file;
	
	//for now have a hard coded link to the atlascar urdf file, we could do this by the command line arguments
	std::string urdf_file_default = ros::package::getPath("atlascar_base") + "/urdf/atlascar_urdf_3dsmodels.xml";

	// Initialize ROS
	init(argc,argv,"device_mapper");
	
	nh=new NodeHandle("~");
	nh->param("robot_urdf_path",urdf_file,urdf_file_default);
	
	int ret;
	xmlTextReaderPtr reader = xmlNewTextReaderFilename(urdf_file.c_str());
	if(reader!=NULL)
	{
		do
		{
			ret=xmlTextReaderRead(reader);//this function returns 1 if ok, -1 on error and 0 if there is no more nodes to read
			NodeHandler(reader);
		}while(ret==1);
		
		xmlFreeTextReader(reader);
		if (ret != 0)//zero means the end of the file
		{
			ROS_ERROR("Failed to parse file");
			return -1;
		}
	}else
	{
		ROS_ERROR("Failed to parse file");
		return -1;
	}
	
	for(uint i=0;i<devices.size();i++)
	{
		devices[i]->SetUpMonitoring();
		devices[i]->EnumerateDevices();
		
		//more than one path can exist for this ruleset, the function devices[i].size() gives you the amount, and you can access
		//them via devices[i]->GetPath(c), to the the c path, this is zero based
		nh->setParam("/device_mapper/"+devices[i]->GetId(),devices[i]->GetPath());

		if (devices[i]->GetPath()=="Not found")
		{
			ROS_ERROR("Device Mapper Error: device %s was not found. Is the equipment connected?", (devices[i]->GetId()).c_str());
		}
	}
	
	Rate r(100);
	while(ok())
	{
		spinOnce();
		r.sleep();
		
		for(uint i=0;i<devices.size();i++)
			devices[i]->Monitoring();
	}
	
	return 0;
}
