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

/** @file udev_control.cpp
* @brief This file contains the source code of the class_udev_control class.
*/

#include "udev_control.h"

class_udev_control::class_udev_control(string id_)
{
	/* Create the udev object */
	udev = udev_new();
	if (!udev)
	{
		cout<<"Can't create udev!"<<endl;
		return;
	}
	
	id=id_;
	fd=-1;
}

class_udev_control::~class_udev_control()
{
	udev_unref(udev);
}

bool class_udev_control::RegistryAction(string action,void (*callback)(string action,string node,void*data),void*data)
{
	actions[action]=callback;
	user_data[action]=data;
	
	return true;
}
bool class_udev_control::AddProperty(string name,string value)
{
	to_upper(name);
	to_upper(value);
	
// 	cout<<id<<", Added property: "<<name<<" value: "<<value<<endl;
	
	properties[name]=value;
	return true;
}

bool class_udev_control::AddSubsystem(string subsystem)
{
// 	cout<<id<<", Added subsytem: "<<subsystem<<endl;
	sub_systems.push_back(subsystem);
	return true;
}

bool class_udev_control::SetUpMonitoring()
{
	/* This section sets up a monitor which will report events when
	devices attached to the system change.  Events include "add",
	"remove", "change", "online", and "offline".
	
	This section sets up and starts the monitoring. Events are
	polled for (and delivered) later in the file.
	
	It is important that the monitor be set up before the call to
	udev_enumerate_scan_devices() so that events (and devices) are
	not missed.  For example, if enumeration happened first, there
	would be no event generated for a device which was attached after
	enumeration but before monitoring began.
	
	Note that a filter is added so that we only get events for
	"sub_systems" devices. */
	
	mon = udev_monitor_new_from_netlink(udev, "udev");

	for(uint i=0;i<sub_systems.size();i++)
		udev_monitor_filter_add_match_subsystem_devtype(mon,sub_systems[i].c_str(),NULL);
										 
	udev_monitor_enable_receiving(mon);
	/* Get the file descriptor (fd) for the monitor.
	This fd will get passed to select() */
	fd = udev_monitor_get_fd(mon);
	return true;
}

string class_udev_control::GetProperty(struct udev_device *dev_,string name)
{
	string value;
        //error in the next line for gamepad on ubuntu 11.10
//         cout << "GAMEPAD2" << endl;
	value=udev_device_get_property_value(dev,name.c_str());
//         cout << "GAMEPAD2.1" << endl;
	return value;
}

bool class_udev_control::FilterProperties(struct udev_device *dev_)
{
	int num_properties_to_match=properties.size();
	int num_properties_match=0;
	
// 	cout<<"Num properties "<<num_properties_to_match<<endl;
	struct udev_list_entry * sysattr_list;
	
	sysattr_list=udev_device_get_properties_list_entry(dev_);
	
	while(sysattr_list!=NULL)//get all parameters
	{
		string name=udev_list_entry_get_name(sysattr_list);
		string value=udev_list_entry_get_value(sysattr_list);
		
		to_upper(name);
		to_upper(value);
		
// 		cout<<"Name "<<name<<" Value "<<value<<endl;
		
		properties_it=properties.find(name);
		if(properties_it!=properties.end())//found
			if(properties_it->second==value)
			{
// 				cout<<"Matched "<<name<< " " <<value<<endl;
				num_properties_match++;
			}
		sysattr_list=udev_list_entry_get_next(sysattr_list);
	}
	
// 	cout<<"Num matched "<<num_properties_match<<endl;
// 	if(num_properties_match==num_properties_to_match)
// 	{
// 		cout<<"Filtered sucessfully"<<endl;
// 	}
	
	
	return num_properties_match==num_properties_to_match;
}

bool class_udev_control::EnumerateDevices()
{
	/* Create a list of the devices in the 'sub_systems' subsystem. */
	enumerate = udev_enumerate_new(udev);
	
	for(uint i=0;i<sub_systems.size();i++)
		udev_enumerate_add_match_subsystem(enumerate,sub_systems[i].c_str());
	
	udev_enumerate_scan_devices(enumerate);	
	devices = udev_enumerate_get_list_entry(enumerate);

	/* For each item enumerated, print out its information.
	udev_list_entry_foreach is a macro which expands to
	a loop. The loop will be executed for each member in
	devices, setting dev_list_entry to a list entry
	which contains the device's path in /sys. */
	
	//The minor number is used to order the list of devices in tha case that there are more than one
	string last_minor="0";
	string minor;
	udev_list_entry_foreach(dev_list_entry, devices)
	{
		const char *path;
		string lpath;
		
		/* Get the filename of the /sys entry for the device
		and create a udev_device object (dev) representing it */
		path = udev_list_entry_get_name(dev_list_entry);
		dev = udev_device_new_from_syspath(udev, path);
		
		if(FilterProperties(dev))
		{
//                        cout << "GAMEPAD1" << endl;
                        //error in this line for gamepad
			minor=GetProperty(dev,"MINOR");
//                        cout << "GAMEPAD1.1" << endl;
			lpath=udev_device_get_devnode(dev);
			
			if(minor>last_minor)
				device_list.push_back(lpath);
			else
			{
				vector<string>::iterator it;
				it=device_list.begin();
				device_list.insert(it,lpath);
			}
			
			cout<<"Added device to local list, ID: "<<id<<" Path: "<<lpath<<endl;
			
			last_minor=minor;
		}
		
		
		
		udev_device_unref(dev);
	}	
	
	/* Free the enumerator object */
	udev_enumerate_unref(enumerate);
	
	return true;
}


bool class_udev_control::Monitoring()
{
	/* Begin polling for udev events. Events occur when devices
	attached to the system are added, removed, or change state. 
	udev_monitor_receive_device() will return a device
	object representing the device which changed and what type of
	change occured.
	
	The select() system call is used to ensure that the call to
	udev_monitor_receive_device() will not block.
	
	The monitor was set up earler in this file, and monitoring is
	already underway.
	*/
	/* Set up the call to select(). In this case, select() will
	only operate on a single file descriptor, the one
	associated with our udev_monitor. Note that the timeval
	object is set to 0, which will cause select() to not
	block. */
	fd_set fds;
	struct timeval tv;
	int ret;
	
	FD_ZERO(&fds);
	FD_SET(fd, &fds);
	tv.tv_sec = 0;
	tv.tv_usec = 0;
	
	if(fd<0)
	{
		cout<<"Monitoring was not configured, please run SetUpMonitoring() before calling Monitoring()"<<endl;
		return false;
	}
	
	ret = select(fd+1, &fds, NULL, NULL, &tv);
		
	/* Check if our file descriptor has received data. */
	if (ret > 0 && FD_ISSET(fd, &fds))
	{
		/* Make the call to receive the device.
		select() ensured that this will not block. */
		dev = udev_monitor_receive_device(mon);
		if (dev)
		{
			//dev is a udev_device
			
			if(FilterProperties(dev))//check if this is the correct device
			{
				actions_it=actions.find(udev_device_get_action(dev));
				if(actions_it!=actions.end())
					actions[udev_device_get_action(dev)](udev_device_get_action(dev),udev_device_get_devnode(dev),user_data[udev_device_get_action(dev)]);
			}
			
			udev_device_unref(dev);
		}
		else
			cout<<"No Device from receive_device(). An error occured."<<endl;
	}
	
	return true;
}
