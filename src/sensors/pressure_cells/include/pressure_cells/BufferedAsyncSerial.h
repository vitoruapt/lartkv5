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
 * @file  BufferedAsyncSerial.h
 * @author Terraneo Federico
 * @brief Class found to make buffers for Serial communications ()
 * 
 * Distributed under the Boost Software License, Version 1.0.
 * Created on January 6, 2011, 3:31 PM
 * 
 */
// ORIGINAL BELOW
/* 
 * File:   BufferedAsyncSerial.h
 * Author: Terraneo Federico
 * Distributed under the Boost Software License, Version 1.0.
 * Created on January 6, 2011, 3:31 PM
 */

#include <pressure_cells/AsyncSerial.h>

#ifndef BUFFEREDASYNCSERIAL_H
#define BUFFEREDASYNCSERIAL_H

/**
* \brief Asynchronous serial class w/ buffer
* Asynchronous serial class that sends data to buffer after reading it from the port.
*/
class BufferedAsyncSerial:public AsyncSerial {
 public:
   BufferedAsyncSerial (  );

    /**
    * Opens a serial device.
    * \param devname serial device name, example "/dev/ttyS0" or "COM1"
    * \param baud_rate serial baud rate
    * \param opt_parity serial parity, default none
    * \param opt_csize serial character size, default 8bit
    * \param opt_flow serial flow control, default none
    * \param opt_stop serial stop bits, default 1
    * \throws boost::system::system_error if cannot open the
    * serial device
    */
   BufferedAsyncSerial ( const std::string & devname, unsigned int baud_rate,
                         boost::asio::serial_port_base::parity opt_parity =
                         boost::asio::serial_port_base::parity ( boost::
                                                                 asio::serial_port_base::parity::
                                                                 none ),
                         boost::asio::serial_port_base::character_size opt_csize =
                         boost::asio::serial_port_base::character_size ( 8 ),
                         boost::asio::serial_port_base::flow_control opt_flow =
                         boost::asio::serial_port_base::flow_control ( boost::
                                                                       asio::serial_port_base::flow_control::
                                                                       none ),
                         boost::asio::serial_port_base::stop_bits opt_stop =
                         boost::asio::serial_port_base::stop_bits ( boost::
                                                                    asio::serial_port_base::stop_bits::
                                                                    one ) );

    /**
     * Read some data asynchronously. Returns immediately.
     * \param data array of char to be read through the serial device
     * \param size array size
     * \return numbr of character actually read 0<=return<=size
     */
   size_t read ( char *data, size_t size );

    /**
     * Read all available data asynchronously. Returns immediately.
     * \return the receive buffer. It iempty if no data is available
     */
     std::vector < char >read (  );

    /**
     * Read a string asynchronously. Returns immediately.
     * Can only be used if the user is sure that the serial device will not
     * send binary data. For binary data read, use read()
     * The returned string is empty if no data has arrived
     * \return a string with the received data.
     */
     std::string readString (  );

     /**
     * Read a line asynchronously. Returns immediately.
     * Can only be used if the user is sure that the serial device will not
     * send binary data. For binary data read, use read()
     * The returned string is empty if the line delimiter has not yet arrived.
     * \param delimiter line delimiter, default='\n'
     * \return a string with the received data. The delimiter is removed from
     * the string.
     */
     std::string readStringUntil ( const std::string delim = "\n" );

     virtual ~ BufferedAsyncSerial (  );

 private:

    /**
     * Read callback, stores data in the buffer
     */
   void readCallback ( const char *data, size_t len );

    /**
     * Finds a substring in a vector of char. Used to look for the delimiter.
     * \param v vector where to find the string
     * \param s string to find
     * \return the beginning of the place in the vector where the first
     * occurrence of the string is, or v.end() if the string was not found
     */
   static std::vector < char >::iterator findStringInVector ( std::vector < char >&v,
                                                              const std::string & s );

     std::vector < char >readQueue;
     boost::mutex readQueueMutex;
};

#endif //BUFFEREDASYNCSERIAL_H