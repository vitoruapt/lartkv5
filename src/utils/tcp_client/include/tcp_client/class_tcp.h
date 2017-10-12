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

/** @file
* @brief Main header for tcp communications
*/

#ifndef _CLASS_TCP_H_
#define _CLASS_TCP_H_

//System Includes
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <iostream>

using namespace std;

/**
@brief This class implements the TCP/IP client communication.
*/
class tcp_client
{
	
	public:
		/**
		@brief Constructor of the class.
		
		The class constructor will setup the socket but will  <b><em>NOT</em></b> open it.
		
		@param ip Ip of the server
		@param port Port in which to communicate
		@param async_comm Activate the asynchronous communication.
		
		*/
		tcp_client(const char*ip,int port,bool async_comm=false);
		
		/** 
		@brief Destructor of the class.
		
		It disconnects the communication.
		*/
		~tcp_client();
		
		/**
		@brief Connect to server
		
		This function connects to the server in case of error sets the err_msg variable and the err code accordingly.
		@return 0 in case of success or -1 in error
		*/
		int Connect(void);
		
		/**
		@brief Disconnect from server
		
		This function disconnects to the server in case of error sets the err_msg variable and the err code accordingly.
		@return 0 in case of success or -1 in error
		*/
		int Disconnect(void);
		
		/**
		@brief Send data
		
		This function sends information to the server, in case of error sets the err_msg variable and the err code accordingly.
		@param data information to send, this is a byte array
		@param size size of the  information to send
		@return 0 in case of success or -1 in error
		*/
		int Send(char*data,int size);
		
		/**
		@brief Read data
		
		This function reads information from the memory buffer, if the peek variable is true the information is not 
		erased from the buffer.
		@param data where to store the data
		@param size maximum length to store
		@param peek if true data is not removed from the server
		@param number number of bytes received
		@param flags flags of the recv call
		@return 0 in case of success or -1 in error
		*/
		int Receive(char*data,int size,bool peek=false,int*number=NULL,int flags=0);
		
		/**
		@brief Print error message
		
		This function prints the last error of this class, this is only valid for the last function called, 
		if a function previous to that had a error it will not be printed.
		@param text additional text to print along with the error.
		@return 0 if theres no error to print or -1 in error
		*/
		int perr(const char*text);
		
		/**
		@brief Get socket value
		
		Get the current socket value.
		@return Socket value.
		*/
		int GetSocket();
		
		///Last error code.
		int err;
		
		///Last error message.
		char err_msg[1024];
		
		///Status of the connection.
		bool connected;
	private:
		
		///Socket identification
		int sock;
		
		///Configuration structure
		struct sockaddr_in server;
		
		///Return value for functions
		int ret;
		
		///Asynchronous comm
		bool async;
};

#endif

