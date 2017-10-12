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
\file
\brief TopicQueuePriority class declaration
*/

#ifndef _TOPIC_PRIORITY_H_
#define _TOPIC_PRIORITY_H_

#include <ros/ros.h>

/**
\brief Topic priority sorting class

This class was developed to simplify the lifetime/priority scheme used in the command
messages, the use of this class guaranties that the command message being used fits the
parameters.
*/
template <class cmd_type>
class TopicQueuePriority
{
	///Map of all the messages received
	typename std::map<int,cmd_type> message_map;
	///Iterator for the messages
	typename std::map<int,cmd_type>::iterator it;
	
	public:
		/**
		\name Usage
		The correct use of this class is extremely simple, the used must only declare this class
		with a copy constructible and assignable class type, boost shared pointers are recommended.
		Function push_msg() add a new message to the queue while top_msg() returns a pointer to the 
		current command message that should be respected while doing the necessary maintenance() to the list.
		The template type cmd_type must be a pointer to a class with variables lifetime and priority.
		Lifetime is the number of seconds that this message will live after arrival and priority is a 
		positive integer value, with higher priority values overriding lower priority messages.
		@{*/
		
		/**
		@brief Add new command message
		
		Add or replace a command message of a given priority, it replaces 
		if a message with the same priority was already received.
		@param msg new message
		*/
		void push_msg(cmd_type msg)
		{
			msg->lifetime=ros::Time::now().toSec()+msg->lifetime;
			
			it=message_map.find(msg->priority);
			
			if(it==message_map.end())
				message_map[msg->priority]=msg;
			else
			{
				message_map.erase(it);
				message_map[msg->priority]=msg;
			}
		}
		
		/**
		@brief Return the current command message
		
		Returns the message with the most priority that is still alive
		@return message to be used
		*/
		cmd_type top_msg(void)
		{
			maintenance();
			
			int max_priority=-1;
			typename std::map<int,cmd_type>::iterator it_max;
			
			for(it=message_map.begin();it!=message_map.end();it++)
				if((*it).second->priority > max_priority)
				{
					max_priority=(*it).second->priority;
					it_max=it;
				}
			
			if(message_map.size())
				return (*it_max).second;
			
			cmd_type null_ptr;
			return null_ptr;
		}
		/**
		@}
		*/
	private:
		/**
		@brief Do maintenance on the message map 
		
		Remove messages that are past their lifetime.
		*/
		void maintenance(void)
		{
			for(it=message_map.begin();it!=message_map.end();it++)
				if(ros::Time::now().toSec() > (*it).second->lifetime)
					message_map.erase(it);
		}
};

#endif
