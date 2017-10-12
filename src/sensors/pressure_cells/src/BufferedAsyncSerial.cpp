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
 * @file  BufferedAsyncSerial.cpp
 * @author Terraneo Federico
 * @brief Class found to make buffers for Serial communications ()
 * 
 * Distributed under the Boost Software License, Version 1.0.
 * Created on January 6, 2011, 3:31 PM
 * 
 */
// ORIGINAL BELOW
/* 
 * @File:   BufferedAsyncSerial.cpp
 * @Author: Terraneo Federico
 * @Brief:
 * Distributed under the Boost Software License, Version 1.0.
 * Created on January 6, 2011, 3:31 PM
 */

#include <pressure_cells/BufferedAsyncSerial.h>
#include <string>
#include <algorithm>
#include <iostream>
#include <boost/bind.hpp>

using namespace std;

using namespace boost;

//
//Class BufferedAsyncSerial
//

BufferedAsyncSerial::BufferedAsyncSerial (  ):AsyncSerial (  )
{
   setReadCallback ( boost::bind ( &BufferedAsyncSerial::readCallback, this, _1, _2 ) );
}

BufferedAsyncSerial::BufferedAsyncSerial ( const std::string & devname,
                                           unsigned int baud_rate,
                                           asio::serial_port_base::parity opt_parity,
                                           asio::
                                           serial_port_base::character_size opt_csize,
                                           asio::serial_port_base::flow_control opt_flow,
                                           asio::serial_port_base::stop_bits opt_stop ):
AsyncSerial ( devname, baud_rate, opt_parity, opt_csize, opt_flow, opt_stop )
{
   setReadCallback ( boost::bind ( &BufferedAsyncSerial::readCallback, this, _1, _2 ) );
}

size_t BufferedAsyncSerial::read ( char *data, size_t size )
{
   lock_guard < mutex > l ( readQueueMutex );
   size_t
      result = min ( size, readQueue.size (  ) );

   vector < char >::iterator
      it = readQueue.begin (  ) + result;

   copy ( readQueue.begin (  ), it, data );
   readQueue.erase ( readQueue.begin (  ), it );
   return result;
}

std::vector < char >
BufferedAsyncSerial::read (  )
{
   lock_guard < mutex > l ( readQueueMutex );
   vector < char >result;

   result.swap ( readQueue );
   return result;
}

std::string BufferedAsyncSerial::readString (  )
{
   lock_guard < mutex > l ( readQueueMutex );
   string result ( readQueue.begin (  ), readQueue.end (  ) );

   readQueue.clear (  );
   return result;
}

std::string BufferedAsyncSerial::readStringUntil ( const std::string delim )
{
   lock_guard < mutex > l ( readQueueMutex );
   vector < char >::iterator
      it = findStringInVector ( readQueue, delim );

   if ( it == readQueue.end (  ) )
      return "";
   string result ( readQueue.begin (  ), it );

   it += delim.size (  );       //Do remove the delimiter from the queue
   readQueue.erase ( readQueue.begin (  ), it );
   return result;
}

void
  BufferedAsyncSerial::readCallback ( const char *data, size_t len )
{
   lock_guard < mutex > l ( readQueueMutex );
   readQueue.insert ( readQueue.end (  ), data, data + len );
}

std::vector < char >::iterator
BufferedAsyncSerial::findStringInVector ( std::vector < char >&v, const std::string & s )
{
   if ( s.size (  ) == 0 )
      return v.end (  );

   vector < char >::iterator it = v.begin (  );

   for ( ;; )
     {
        vector < char >::iterator result = find ( it, v.end (  ), s[0] );

        if ( result == v.end (  ) )
           return v.end (  );   //If not found return

        for ( size_t i = 0; i < s.size (  ); i++ )
          {
             vector < char >::iterator temp = result + i;

             if ( temp == v.end (  ) )
                return v.end (  );
             if ( s[i] != *temp )
                goto mismatch;
          }
        //Found
        return result;

      mismatch:
        it = result + 1;
     }
}

BufferedAsyncSerial::~BufferedAsyncSerial (  )
{
   clearReadCallback (  );
}
