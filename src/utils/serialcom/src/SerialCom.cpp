/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2011, LAR-DEM University of Aveiro.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the LAR-DEM University of Aveiro nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* based on the work of:
* ISR University of Coimbra.
* Gonçalo Cabrita on 01/10/2010
*********************************************************************/

/**
\file
\brief implementation of the methods for RS232 serial communication
*/

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <termios.h>
#include <math.h>
#include <poll.h>
#include <signal.h>
#include <fcntl.h>
#include <iostream>
#include <fstream>

#include "serialcom/SerialCom.h"

//! Macro for throwing an exception with a message, passing args
#define SERIAL_EXCEPT(except, msg, ...) \
{ \
    char buf[1000]; \
    snprintf(buf, 1000, msg " (in serialcom::SerialCom::%s)" , ##__VA_ARGS__, __FUNCTION__); \
    throw except(buf); \
}

serialcom::SerialCom::SerialCom() : fd_(-1)
{
	stream_thread_ = NULL;
}

serialcom::SerialCom::~SerialCom()
{
	if(portOpen()) close();
}


tcflag_t serialcom::SerialCom:: selectbaud(int baudrate_)
{
 
  switch(baudrate_)
  {
    case(0):      return(B0);     //Hang up
    case(50):     return(B50);    //50 baud
    case(75): 	  return(B75);    //75 baud
    case(110):    return(B110);   //110 baud
    case(134):    return(B134);   //134.5 baud
    case(150):    return(B150);   //150 baud
    case(200):    return(B200);   //200 baud
    case(300):    return(B300);   //300 baud
    case(600):    return(B600);   //600 baud
    case(1200):   return(B1200);  //1200 baud
    case(1800):   return(B1800);  //1800 baud
    case(2400):   return(B2400);  //2400 baud
    case(4800):   return(B4800);  //4800 baud
    case(9600):   return(B9600);  //9600 baud
    case(19200):  return(B19200); //19200 baud
    case(38400):  return(B38400); //38400 baud
    case(57600):  return(B57600); 
    case(115200): return(B115200);
    case(230400): return(B230400);
    case(460800): return(B460800);
    case(500000): return(B500000);
    case(576000): return(B576000);
    default:
      return(B9600);

  }
};

bool serialcom::SerialCom:: config(int baudrate_, int databits_, int startbits_, int stopbits_, int parity_, int NLchar_)
{

  tcflag_t DATABITS;
  tcflag_t PARITY;
  tcflag_t confipar;
  tcflag_t MAPCRNL;
  tcflag_t STOPBITS;
  
  int attribset, attribget;
 
  switch (databits_)
  {
    case(5): DATABITS = CS5; break; // 5 bits
    case(6): DATABITS = CS6; break;// 6 bits
    case(7): DATABITS = CS7; break;// 7 bits
    case(8): DATABITS = CS8; break; // 8 bits
    default:
       DATABITS = CS8;
  };
  
  switch (parity_)
  {
    case(EVEN):
      PARITY = PARENB; //Parity enable.
      PARITY &= ~PARODD; //even.
      confipar = INPCK | ISTRIP;
      break;
      
    case(ODD):
      PARITY =  PARENB; //Parity enable.
      PARITY |= PARODD; //Odd parity, else even.
      confipar = INPCK | ISTRIP;
      break;
      
    default:
      PARITY = 0;
      confipar = IGNPAR; // no parity check
      
  };
  
  switch (stopbits_)
  {
     case(1):
         default:
            STOPBITS = 0; //one stopbit
            break;
      case(2):
            STOPBITS = CSTOPB; //two stopbits
            break;
  };  //end of switch stop bits

  if (NLchar_ == 1) //CANONICAL
  {
    MAPCRNL = ICRNL; // map the carriage return to a line feed
    newParams.c_lflag = (ICANON & (~(ECHO))); // control mode local flags for canonical; input is line oriented, do not echo
  }
  else // RAW
  {
    MAPCRNL = 0;
    newParams.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // control mode local flags for raw input
  };
    
    
  attribget = tcgetattr(fd_, &ComParams); // get present serial communication parameters

  newParams.c_iflag =  confipar | MAPCRNL; // input mode
  
//--------------------------------------------------
  newParams.c_oflag = 0; //output mode
//--------------------------------------------------

  newParams.c_cflag = selectbaud(baudrate_) |
				   DATABITS |
				   STOPBITS |
				     PARITY |
				     CLOCAL |
				     CREAD; //control mode
				     
  //newParams.c_lflag &= ~( ECHO | ECHOE);
 
  newParams.c_cc[VTIME] = 0; /* second timeout - 0 disabled */
  newParams.c_cc[VMIN] = 0;  /* read 0 byte - 0 disabled */

  tcflush(fd_, TCIFLUSH);
  
  attribset = tcsetattr(fd_, TCSANOW, &newParams ); // set serial communication parameters 
  
  if(attribset == 0)
  {
    baud_ = baudrate_;
    databits = databits_;
    startbits = startbits_;
    stopbits = stopbits_;
    parity = parity_;
    return true;
  }
  else
  {
    SERIAL_EXCEPT(serialcom::Exception,"Set serial communication parameters failed:");
    return false;
  }
  
};

bool serialcom::SerialCom:: changebaudrate(int baudrate_)
{
  int outset;
  int inset;
  int attribset;
  
  attribset = 0;
  
  outset = cfsetospeed(&newParams,(speed_t) selectbaud(baudrate_)); // output baud rate
  inset  = cfsetispeed(&newParams, (speed_t) selectbaud(baudrate_)); // input baud rate
  
  if((inset<0) || (outset<0))
  {
    SERIAL_EXCEPT(serialcom::Exception,"Setting baudrate failed");
  }
  
  tcflush(fd_, TCIFLUSH);
  attribset = tcsetattr(fd_,TCSANOW,&newParams); // set
  
  if(attribset<0)
  {
    SERIAL_EXCEPT(serialcom::Exception,"Set serial communication parameters failed:");
    return false;
  }
  
  return true;
};


void serialcom::SerialCom::open(const char * port_name, int baud_rate)
{
	tcflag_t MAPCRNL;
	
	if(portOpen()) close();
  
	// Make IO non blocking. This way there are no race conditions that
	// cause blocking when a badly behaving process does a read at the same
	// time as us. Will need to switch to blocking for writes or errors
	// occur just after a replug event.
	fd_ = ::open(port_name, O_RDWR | O_NONBLOCK | O_NOCTTY);

	if(fd_ == -1)
	{
		const char *extra_msg = "";
		switch(errno)
		{
			case EACCES:
			extra_msg = "You probably don't have premission to open the port for reading and writing.";
			break;

			case ENOENT:
			extra_msg = "The requested port does not exist. Was the port name misspelled?";
			break;
		}
		SERIAL_EXCEPT(serialcom::Exception, "Failed to open port: %s. %s (errno = %d). %s", port_name, strerror(errno), errno, extra_msg);
	}
	
	try
	{	
		struct flock fl;
		fl.l_type = F_WRLCK;
		fl.l_whence = SEEK_SET;
		fl.l_start = 0;
		fl.l_len = 0;
		fl.l_pid = getpid();

		if(fcntl(fd_, F_SETLK, &fl) != 0)
			SERIAL_EXCEPT(serialcom::Exception, "Device %s is already locked. Try 'lsof | grep %s' to find other processes that currently have the port open.", port_name, port_name);

		// Settings for USB?
		struct termios newtio;
		tcgetattr(fd_, &newtio);
		memset (&newtio.c_cc, 0, sizeof (newtio.c_cc));
		newtio.c_cflag = CS8 | CLOCAL | CREAD;
//		newtio.c_iflag = IGNPAR;  original
		
//rpascoal, because some PIC code is sending '\r'
		//TODO rpascoal may need to be changed preferebly by using the config ()
 		MAPCRNL = ICRNL;
		newtio.c_iflag = IGNPAR | MAPCRNL;
// ---------------
		
		newtio.c_oflag = 0;
//		newtio.c_lflag = 0;

// rpascoal line oriented input
		//TODO rpascoal may need to be changed preferebly by using the config ()
		newtio.c_lflag = ICANON; // so that we don't cut settences when reading lines
// -------------------------------------------------------
		
		cfsetspeed(&newtio, selectbaud(baud_rate));
		baud_ = baud_rate;

		// Activate new settings
		tcflush(fd_, TCIFLUSH);
		if(tcsetattr(fd_, TCSANOW, &newtio) < 0)
			SERIAL_EXCEPT(serialcom::Exception, "Unable to set serial port attributes. The port you specified (%s) may not be a serial port.", port_name); /// @todo tcsetattr returns true if at least one attribute was set. Hence, we might not have set everything on success.
		usleep (200000);
	}
	catch(serialcom::Exception& e)
	{
		// These exceptions mean something failed on open and we should close
		if(fd_ != -1) ::close(fd_);
		fd_ = -1;
		throw e;
	}
}

void serialcom::SerialCom::close()
{
	int retval = 0;
	
  	retval = ::close(fd_);

  	fd_ = -1;

  	if(retval != 0)
    		SERIAL_EXCEPT(serialcom::Exception, "Failed to close port properly -- error = %d: %s\n", errno, strerror(errno));
}

int serialcom::SerialCom::write(const char * data, int length)
{
	int len = length==-1 ? strlen(data) : length;

	// IO is currently non-blocking. This is what we want for the more serialcomon read case.
	int origflags = fcntl(fd_, F_GETFL, 0);
	fcntl(fd_, F_SETFL, origflags & ~O_NONBLOCK); // TODO: @todo can we make this all work in non-blocking?
	int retval = ::write(fd_, data, len);
	fcntl(fd_, F_SETFL, origflags | O_NONBLOCK);

	if(retval == len) return retval;
	else SERIAL_EXCEPT(serialcom::Exception, "write failed");
}

//created by jorge and procopio to be used with the xbee api lib
bool serialcom::SerialCom::writebyte(uint8_t byte)
{
	int written = 0;
	while (written != 1)
	{
		written = ::write(fd_, &byte, 1); //to make sure we get the std write
		if (written < 0)
		{
			return false;
		}

		usleep(1000);
	}
	return true;
}

//created by jorge and procopio to be used with the xbee api lib
int serialcom :: SerialCom:: readbyte(uint8_t *read_byte)
{
  //int read_result;
  //read_result = fgetc(datastream); // get a character from the stream
  char read_result;
  int ret;
  ret = ::read(fd_, &read_result, 1); //TODO rpascoal have to check that this is OK
  *read_byte = (uint8_t) read_result;
  
  //if(isindebugmode())
  //{
  //  printf("this was read %i\n",read_result);
  //};
  
  if (read_result == EOF) return false;
  return ret;
}


int serialcom::SerialCom::read(char * buffer, int max_length, int timeout)
{
	int ret;

	struct pollfd ufd[1];
	int retval;
	ufd[0].fd = fd_;
	ufd[0].events = POLLIN;

	if(timeout == 0) timeout = -1; // For compatibility with former behavior, 0 means no timeout. For poll, negative means no timeout.

	if((retval = poll(ufd, 1, timeout)) < 0) SERIAL_EXCEPT(serialcom::Exception, "poll failed -- error = %d: %s", errno, strerror(errno));

	if(retval == 0) SERIAL_EXCEPT(serialcom::TimeoutException, "timeout reached");
		
	if(ufd[0].revents & POLLERR) SERIAL_EXCEPT(serialcom::Exception, "error on socket, possibly unplugged");
		
    ret = ::read(fd_, buffer, max_length);
      		
	if(ret == -1 && errno != EAGAIN && errno != EWOULDBLOCK) SERIAL_EXCEPT(serialcom::Exception, "read failed");
	
	return ret;
}

int serialcom::SerialCom::readBytes(char * buffer, int length, int timeout)
{
	int ret;
	int current = 0;

	struct pollfd ufd[1];
	int retval;
	ufd[0].fd = fd_;
	ufd[0].events = POLLIN;

	if(timeout == 0) timeout = -1; // For compatibility with former behavior, 0 means no timeout. For poll, negative means no timeout.

	while(current < length)
	{
		if((retval = poll(ufd, 1, timeout)) < 0) SERIAL_EXCEPT(serialcom::Exception, "poll failed -- error = %d: %s", errno, strerror(errno));

		if(retval == 0) SERIAL_EXCEPT(serialcom::TimeoutException, "timeout reached");
		
		if(ufd[0].revents & POLLERR) SERIAL_EXCEPT(serialcom::Exception, "error on socket, possibly unplugged");
		
      	ret = ::read(fd_, &buffer[current], length-current);
      		
		if(ret == -1 && errno != EAGAIN && errno != EWOULDBLOCK) SERIAL_EXCEPT(serialcom::Exception, "read failed");

		current += ret;
	}
	return current;
}

int serialcom::SerialCom::readLine(char * buffer, int length, int timeout)
{
	int ret;
	int current = 0;

	struct pollfd ufd[1];
	int retval;
	ufd[0].fd = fd_;
	ufd[0].events = POLLIN;

	if(timeout == 0) timeout = -1; // For compatibility with former behavior, 0 means no timeout. For poll, negative means no timeout.

	while(current < length-1)
	{
		if(current > 0)
			if(buffer[current-1] == '\n')
				return current;

		if((retval = poll(ufd, 1, timeout)) < 0) SERIAL_EXCEPT(serialcom::Exception, "poll failed -- error = %d: %s", errno, strerror(errno));

		if(retval == 0) SERIAL_EXCEPT(serialcom::TimeoutException, "timeout reached");
		
		if(ufd[0].revents & POLLERR) SERIAL_EXCEPT(serialcom::Exception, "error on socket, possibly unplugged");
		
      	ret = ::read(fd_, &buffer[current], length-current);
      		
		if(ret == -1 && errno != EAGAIN && errno != EWOULDBLOCK) SERIAL_EXCEPT(serialcom::Exception, "read failed");

		current += ret;
	}
	SERIAL_EXCEPT(serialcom::Exception, "buffer filled without end of line being found");
}
using namespace std;
bool serialcom::SerialCom::readLine(std::string& buffer)
{
    int ret;
    struct pollfd ufd[1];
    int retval;
    ufd[0].fd = fd_;
    ufd[0].events = POLLIN;
    
    buffer.clear();
    
    int timeout=1;
    int c=0;
    
    while(1)
    {
        char ch;
        
        ret = readbyte((uint8_t*)&ch);
        
        if(ret == -1 && errno != EAGAIN && errno != EWOULDBLOCK)
        {
	  continue;
        }else if(ret==-1)
        {
// 	  perror("cannot read");
	  break;
        }
  
        if(ch=='\n')
	  break;
        
        buffer.append(1,ch);
    }
    
    return true;
}

bool serialcom::SerialCom::readLine(std::string * buffer, int timeout)
{
	int ret;

	struct pollfd ufd[1];
	int retval;
	ufd[0].fd = fd_;
	ufd[0].events = POLLIN;

	if(timeout == 0) timeout = -1; // For compatibility with former behavior, 0 means no timeout. For poll, negative means no timeout.

	buffer->clear();
	while(buffer->size() < buffer->max_size()/2)
	{
		// Look for the end char
		ret = buffer->find_first_of("\n");

		//printf("test rpascoal %d 4\n", (int)ret);
		if(ret == 0) // added because the code was getting stuck. This condition was not checked in cereal port.
		{
		 buffer->erase(0,1);
		 ret = buffer->find_first_of("\n");
		}
		
		if(ret > 0)
		{
			// If it is there clear everything after it and return
			buffer->erase(ret); //rpascoal
			//buffer->erase(ret+1, buffer->size()-ret-1); the original from cereal port was not clearing properly
	
			//printf("test rpascoal %s \n", buffer->c_str());	
			
			return true;
		}

		if((retval = poll(ufd, 1, timeout)) < 0) SERIAL_EXCEPT(serialcom::Exception, "poll failed -- error = %d: %s", errno, strerror(errno));

		if(retval == 0) SERIAL_EXCEPT(serialcom::TimeoutException, "timeout reached");
		
		if(ufd[0].revents & POLLERR) SERIAL_EXCEPT(serialcom::Exception, "error on socket, possibly unplugged");
		
      	char temp_buffer[128];
		
      	ret = ::read(fd_, temp_buffer, 128);

		//temp_buffer[126] = 't';
		//temp_buffer[127] = '\0';
		//printf("test rpascoal %s 2\n", temp_buffer);
      		
		if(ret == -1 && errno != EAGAIN && errno != EWOULDBLOCK) SERIAL_EXCEPT(serialcom::Exception, "read failed");
        
		// Append the new data to the buffer
      	buffer->append(temp_buffer, ret);

		//printf("test rpascoal %s 2\n", buffer->c_str());
				
		//sleep(1);		
	}
	
	SERIAL_EXCEPT(serialcom::Exception, "buffer filled without end of line being found");
}

bool serialcom::SerialCom::readBetween(std::string * buffer, char start, char end, int timeout)
{
	int ret;

	struct pollfd ufd[1];
	int retval;
	ufd[0].fd = fd_;
	ufd[0].events = POLLIN;

	if(timeout == 0) timeout = -1; // For compatibility with former behavior, 0 means no timeout. For poll, negative means no timeout.

	// Clear the buffer before we start
	buffer->clear();
	while(buffer->size() < buffer->max_size()/2)
	{
		if((retval = poll(ufd, 1, timeout)) < 0) SERIAL_EXCEPT(serialcom::Exception, "poll failed -- error = %d: %s", errno, strerror(errno));
		
		if(retval == 0) SERIAL_EXCEPT(serialcom::TimeoutException, "timeout reached");
		
		if(ufd[0].revents & POLLERR) SERIAL_EXCEPT(serialcom::Exception, "error on socket, possibly unplugged");
		
		char temp_buffer[128];
  		ret = ::read(fd_, temp_buffer, 128);
  	
  		if(ret == -1 && errno != EAGAIN && errno != EWOULDBLOCK) SERIAL_EXCEPT(serialcom::Exception, "read failed");
  	
  		// Append the new data to the buffer
  		buffer->append(temp_buffer, ret);
      	
      	// Look for the start char
      	ret = buffer->find_first_of(start);
      	// If it is not on the buffer, clear it
      	if(ret == -1) buffer->clear();
      	// If it is there, but not on the first position clear everything behind it
      	else if(ret > 0) buffer->erase(0, ret);
		
		// Look for the end char
		ret = buffer->find_first_of(end);
		if(ret > 0)
		{
			// If it is there clear everything after it and return
			buffer->erase(ret+1, buffer->size()-ret-1);
			return true;
		}
	}
	SERIAL_EXCEPT(serialcom::Exception, "buffer filled without reaching end of data stream");
}

int serialcom::SerialCom::flush()
{
	  int retval = tcflush(fd_, TCIOFLUSH);
	  if(retval != 0) SERIAL_EXCEPT(serialcom::Exception, "tcflush failed");
	  
	  return retval;
}

bool serialcom::SerialCom::startReadStream(boost::function<void(char*, int)> f)
{
	if(stream_thread_ != NULL) return false;

	stream_stopped_ = false;
	stream_paused_ = false;
	
	readCallback = f;
	
	stream_thread_ = new boost::thread(boost::bind(&serialcom::SerialCom::readThread, this));
	return true;
}

void serialcom::SerialCom::readThread()
{
	char data[MAX_LENGTH];
	int ret;

	struct pollfd ufd[1];
	ufd[0].fd = fd_;
	ufd[0].events = POLLIN;
	
	while(!stream_stopped_)
	{
		if(!stream_paused_)
		{
			if(poll(ufd, 1, 10) > 0)
			{
				if(!(ufd[0].revents & POLLERR))
				{
			  		ret = ::read(fd_, data, MAX_LENGTH);
			  		if(ret>0)
			  		{
			  			readCallback(data, ret);
			  		}
			  	}
			}
		}
	}
}

bool serialcom::SerialCom::startReadLineStream(boost::function<void(std::string*)> f)
{
	if(stream_thread_ != NULL) return false;

	stream_stopped_ = false;
	stream_paused_ = false;
	
	readLineCallback = f;
		
	stream_thread_ = new boost::thread(boost::bind(&serialcom::SerialCom::readLineThread, this));
	return true;
}

void serialcom::SerialCom::readLineThread()
{
	std::string data;
	bool error = false;

	while(!stream_stopped_)
	{
		
		if(!stream_paused_)
		{
			error = false;
			try{readLine(&data, 100); }
			catch(serialcom::Exception& e)
			{ 
				error = true;
			}
	
			//printf("rpascoal data %s \n", data.c_str());			

			if(!error && data.size()>0) {readLineCallback(&data);};
		}
	}
}

bool serialcom::SerialCom::startReadBetweenStream(boost::function<void(std::string*)> f, char start, char end)
{
	if(stream_thread_ != NULL) return false;

	stream_stopped_ = false;
	stream_paused_ = false;
	
	readBetweenCallback = f;
	
	stream_thread_ = new boost::thread(boost::bind(&serialcom::SerialCom::readBetweenThread, this, start, end));
	return true;
}

void serialcom::SerialCom::readBetweenThread(char start, char end)
{
	std::string data;
	bool error = false;

	while(!stream_stopped_)
	{
		if(!stream_paused_)
		{
			error = false;
			try{ readBetween(&data, start, end, 100); }
			catch(serialcom::Exception& e)
			{ 
				error = true;
			}
			
			if(!error && data.size()>0) readBetweenCallback(&data);
		}
	}
}

void serialcom::SerialCom::stopStream()
{
	stream_stopped_ = true;
	stream_thread_->join();
	
	delete stream_thread_;
	stream_thread_ = NULL;
}

void serialcom::SerialCom::pauseStream()
{
	stream_paused_ = true;
}

void serialcom::SerialCom::resumeStream()
{
	stream_paused_ = false;
}

// EOF
