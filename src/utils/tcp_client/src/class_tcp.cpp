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

/** 
 * \file
* @brief Source code for this class that does tcp communication
*/

#include <tcp_client/class_tcp.h>

tcp_client::tcp_client(const char*ip,int port,bool async_comm)
{
	err=0;//Clear the error variable
	
	connected=false;
	
	sock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
	if(sock<0)
	{
		sprintf(err_msg,"Cannot open socket");
		err=-1;
		return;
	}
	
	async=async_comm;
	
	/* Construct the server sockaddr_in structure */
	memset(&server, 0, sizeof(server));       /* Clear structure */
	server.sin_family = AF_INET;                  /* Internet/IP */
	server.sin_addr.s_addr = inet_addr(ip);  /* IP address */
	server.sin_port = htons(port);       /* server port */
}

tcp_client::~tcp_client()
{
	Disconnect();
}

int tcp_client::Connect(void)
{
	err=0;
	
	ret=connect(sock,(struct sockaddr *) &server,sizeof(server));
	if(ret<0)
	{
		sprintf(err_msg,"Failed to connect with server");
		err=-2;
		return -1;
	}
	
	if(async)
	{
		fcntl(sock,F_SETOWN,getpid());
		fcntl(sock,F_SETFL,O_ASYNC);
	}
	
	connected=true;
	
	return 0;
}

int tcp_client::Disconnect(void)
{
	err=0;
	
	ret=close(sock);
	if(ret<0)
	{
		sprintf(err_msg,"Failed to close connection");
		err=-3;
		return -1;
	}

	connected=false;

	return 0;
}

int tcp_client::Send(char*data,int size)
{
	if(!connected)
	{
		sprintf(err_msg,"Connection not established");
		err=-4;
		return -1;
	}
	
	err=0;
	errno=0;
	ret=send(sock,data,size,0);
	
	if(errno==EPIPE)
	{
		sprintf(err_msg,"Pipe broke");
		err=-4;
		return -1;
	}else if(ret<size)
	{
		sprintf(err_msg,"Mismatch in number of sent bytes");
		err=-4;
		return -1;
	}else if(errno!=0)
	{
		sprintf(err_msg,"Alarm! error not caught");
		perror("send");
		err=-3;
		return -1;
	}
	
	return 0;
}

int tcp_client::Receive(char*data,int size,bool peek,int*number,int flags)
{
	if(!connected)
	{
		sprintf(err_msg,"Connection not established");
		err=-4;
		return -1;
	}
	
	err=0;
	
	//Get the current time stamp
// 	timestamp=carmen_get_time();
	
	//If peek mode is selected we don't erase the information from the buffer
	if(peek)
	{
		ret=recv(sock,data,size, MSG_PEEK);
		if(ret<0)
		{
			sprintf(err_msg,"Failed to peek data from server");
			err=-5;
			return -1;
		}
		
		return 0;
	}
	
	//Normal read
	ret=recv(sock,data,size,flags);
	if(ret<0)
	{
		sprintf(err_msg,"Failed to receive bytes from server");
		err=-5;
		return -1;
	}else if(number!=NULL)
	{
		*number=ret;
	}
	
	return 0;
}

int tcp_client::perr(const char*text)
{
	if(err<0)
	{
		cout<<"Error!!   "<<text<<", "<<err_msg<<endl;
		return -1;
	}
	return 0;
}

int tcp_client::GetSocket()
{
	return sock;
}
