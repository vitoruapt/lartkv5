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

/** @file udev_control.h
* @brief Class that simplifies the implementation of the udev library.
*/

#ifndef _UDEV_CONTROL_H_
#define _UDEV_CONTROL_H_

#include <libudev.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <unistd.h>
#include <string>
#include <vector>
#include <iostream>
#include <map>
#include <signal.h>
#include <string.h>
#include <boost/algorithm/string.hpp>

using namespace std;
using namespace boost;

/**
 * @brief This class simplifies the implementation of the udev library.
 *
 * Each instance can only handle one device with a variable size set of rules.\n
 * Rules are added with the \c AddSubsystem() and \c AddProperty() functions.\n
 * The paths to the devices that correspond to the rules are saved in the
 * \c device_list variable and are accessible via the \c GetPath() function.\n
*/
class class_udev_control
{
	private:
		///Auxiliary variable
		struct udev *udev;
		///Enumerator auxiliary variable 
		struct udev_enumerate *enumerate;
		///List entries
		struct udev_list_entry *devices, *dev_list_entry;
		///Device auxiliary variable
		struct udev_device *dev;
		///Udev monitor
		struct udev_monitor *mon;
		///File descriptor used in the monitoring
		int fd;
		///Subsystem vector, these will be used to prefilter the devices
		vector<string> sub_systems;
		///Map of all properties added
		map<string,string> properties;
		///Properties map iterator
		map<string,string>::iterator properties_it;
		///Map of pairs of actions strings and corresponding handler function
		map<string,void (*)(string,string,void*)> actions;
		///Map of pairs of actions strings and user data parameters
		map<string,void*> user_data;
		///Map iterator for the actions handler map
		map<string,void (*)(string,string,void*)>::iterator actions_it;
		
		/**
		* @brief Filter properties
		*
		* This function filters the devices using the properties added.
		*
		* @param dev_ device to filter
		* @return true if the devices passes all the filters or false if not
		*/
		bool FilterProperties(struct udev_device *dev_);
		
		/**
		* @brief Get device property
		*
		* This function reads a device property by its name.
		*
		* @param dev_ device to use
		* @param name name of the property
		* @return value of the property
		*/
		string GetProperty(struct udev_device *dev_,string name);
		
	public:
		/**
		* @brief Constructor
		*
		* The constructor initializes the udev main object.  And sets the class id.
		* @param id_ id of this class
		*/
		class_udev_control(string id_);
		
		/**
		@brief Destructor
		
		Unreference the udev main object.
		*/
		~class_udev_control();
		
		///Id of the class
		string id;
		
		///List of all devices that pass the filter
		vector<string> device_list;
		
		/**
		@brief Add subsystem to the properties
		
		This functions adds a subsystem to the filter pairs.
		@param subsystem value of the subsytem
		@return always true
		*/
		bool AddSubsystem(string subsystem);
		
		/**
		@brief Add a property pair to the list of properties
		
		This function adds a property name/value pair to the list of all 
		properties that will be used to filter the devices.
		@param name name of the property to add
		@param value value of the property
		@return always true
		*/
		bool AddProperty(string name,string value);
		
		/**
		@brief Registry and action handler
		
		This functions allows the user to set handler for specific actions on the devices \n
		(remove, add, modify, etc..).
		@param action name of the action
		@param callback handler function
		@param data data to be sent to the handler
		@return always true
		*/
		bool RegistryAction(string action,void (*callback)(string action,string node,void*data),void*data);
		
		/**
		@brief Configure monitoring
		
		Configure the monitoring, must be called before calling \c Monitoring().
		@return always true
		*/
		bool SetUpMonitoring();
		
		/**
		@brief Enumerate devices
		
		Enumerate all devices that pass the filter of properties and subsystems.\n
		This is the function that filters devices and makes them available in the list.\n
		Multiple devices with the same set of rules are ordered by \c minor number 
		(this is a property that all devices share).
		@return always true
		*/
		bool EnumerateDevices();
		
		/**
		@brief Monitor device
		
		This function monitors the device 
		@return always true
		*/
		bool Monitoring();
		
		/**
		@brief Get the size of the device list
		@return size of the device list
		*/
		uint size()
		{
			return device_list.size();
		}
		
		/**
		@brief Get id of the class
		@return id of the class
		*/
		string GetId()
		{
			return id;
		}
		
		/**
		@brief Get the path to the device
		
		This function returns the path 
		@return id of the class
		*/
		string GetPath(uint i=0)
		{
			if(i>=size())
				return "Not found";
			
			return device_list[i];
		}
		
};

#endif
