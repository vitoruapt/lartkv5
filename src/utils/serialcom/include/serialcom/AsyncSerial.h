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
 * @file AsyncSerial.h
 * @author Terraneo Federico
 * @brief Class found to work with asyncronous Serial communications ()
 * 
 * Distributed under the Boost Software License, Version 1.0.
 * Created on September 7, 2009, 10:46 AM
 * 
 */
// ORIGINAL BELOW
/*
 * File:   AsyncSerial.h
 * Author: Terraneo Federico
 * Distributed under the Boost Software License, Version 1.0.
 * Created on September 7, 2009, 10:46 AM
 */

#ifndef ASYNCSERIAL_H
#define ASYNCSERIAL_H

#include <vector>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/utility.hpp>
#include <boost/function.hpp>
#include <boost/shared_array.hpp>

namespace serialcom
{
    const unsigned int read_buffer_size=512;
}

#ifndef __APPLE__

/**
 * \brief Asyncronous serial class for internal usage
 * Used internally (pimpl)
 */
class AsyncSerialImpl:private boost::noncopyable
{
    public:
        AsyncSerialImpl():
        io (  ),
        port ( io ),
        backgroundThread (  ),
        open ( false ),
        error ( false )
        {
        }
        
        boost::asio::io_service io;///< Io service object

        boost::asio::serial_port port;   ///< Serial port object
        boost::thread backgroundThread;  ///< Thread that runs read/write operations
        bool open;                   ///< True if port open

        bool error;                  ///< Error flag

        mutable boost::mutex errorMutex; ///< Mutex for access to error

        /// Data are queued here before they go in writeBuffer
        std::vector <char> writeQueue;

        boost::shared_array <char> writeBuffer;              ///< Data being written

        size_t writeBufferSize;      ///< Size of writeBuffer

        boost::mutex writeQueueMutex;    ///< Mutex for access to writeQueue
        
        char readBuffer[serialcom::read_buffer_size];   ///< data being read

        /// Read complete callback
        boost::function < void ( const char *, size_t ) > callback;
};

/**
 * \brief Asyncronous serial class
 * Asyncronous serial class.
 * Intended to be a base class.
 */
class AsyncSerial:private boost::noncopyable
{
    public:

        AsyncSerial()   
        :pimpl ( new AsyncSerialImpl )
        {
        }
    
        /**
            * Constructor. Creates and opens a serial device.
            * \param devname serial device name, example "/dev/ttyS0" or "COM1"
            * \param baud_rate serial baud rate
            * \param opt_parity serial parity, default none
            * \param opt_csize serial character size, default 8bit
            * \param opt_flow serial flow control, default none
            * \param opt_stop serial stop bits, default 1
            * \throws boost::system::system_error if cannot open the
            * serial device
            */
        AsyncSerial ( const std::string & devname, unsigned int baud_rate,
                        boost::asio::serial_port_base::parity opt_parity =
                        boost::asio::serial_port_base::parity ( boost::asio::
                                                                serial_port_base::parity::none ),
                        boost::asio::serial_port_base::character_size opt_csize =
                        boost::asio::serial_port_base::character_size ( 8 ),
                        boost::asio::serial_port_base::flow_control opt_flow =
                        boost::asio::serial_port_base::flow_control ( boost::
                                                                    asio::serial_port_base::flow_control::
                                                                    none ),
                        boost::asio::serial_port_base::stop_bits opt_stop =
                        boost::asio::serial_port_base::stop_bits ( boost::asio::
                                                                    serial_port_base::stop_bits::
                                                                    one ) )
        :pimpl ( new AsyncSerialImpl )
        {
            open ( devname, baud_rate, opt_parity, opt_csize, opt_flow, opt_stop );
        }

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
        void open ( const std::string & devname, unsigned int baud_rate,
                    boost::asio::serial_port_base::parity opt_parity =
                    boost::asio::serial_port_base::parity ( boost::asio::serial_port_base::
                                                            parity::none ),
                    boost::asio::serial_port_base::character_size opt_csize =
                    boost::asio::serial_port_base::character_size ( 8 ),
                    boost::asio::serial_port_base::flow_control opt_flow =
                    boost::asio::serial_port_base::flow_control ( boost::asio::serial_port_base::flow_control::none ),
                    boost::asio::serial_port_base::stop_bits opt_stop =
                    boost::asio::serial_port_base::stop_bits ( boost::asio::serial_port_base::stop_bits::one ) )
        {
            if ( isOpen (  ) )
                close (  );

            setErrorStatus ( true );     //If an exception is thrown, error_ remains true
            pimpl->port.open ( devname );
            pimpl->port.set_option ( boost::asio::serial_port_base::baud_rate ( baud_rate ) );
            pimpl->port.set_option ( opt_parity );
            pimpl->port.set_option ( opt_csize );
            pimpl->port.set_option ( opt_flow );
            pimpl->port.set_option ( opt_stop );

            //This gives some work to the io_service before it is started
            pimpl->io.post ( boost::bind ( &AsyncSerial::doRead, this ) );

            boost::thread t ( boost::bind ( &boost::asio::io_service::run, &pimpl->io ) );

            pimpl->backgroundThread.swap ( t );
            setErrorStatus ( false );    //If we get here, no error
            pimpl->open = true;          //Port is now open
        }

        /**
        * \return true if serial device is open
        */
        bool isOpen (  ) const
        {
            return pimpl->open;
        }

        /**
        * \return true if error were found
        */
        bool errorStatus (  ) const
        {
            boost::lock_guard < boost::mutex > l ( pimpl->errorMutex );
            return pimpl->error;
        }

        /**
        * Close the serial device
        * \throws boost::system::system_error if any error
        */
        void close()
        {
            if ( !isOpen (  ) )
                return;

            pimpl->open = false;
            pimpl->io.post ( boost::bind ( &AsyncSerial::doClose, this ) );
            pimpl->backgroundThread.join (  );
            pimpl->io.reset (  );
            if ( errorStatus (  ) )
            {
                throw ( boost::system::system_error ( boost::system::error_code (  ),"Error while closing the device" ) );
            }
        }

        /**
        * Write data asynchronously. Returns immediately.
        * \param data array of char to be sent through the serial device
        * \param size array size
        */
        void write ( const char *data, size_t size )
        {
            {
                boost::lock_guard < boost::mutex > l ( pimpl->writeQueueMutex );
                pimpl->writeQueue.insert ( pimpl->writeQueue.end (  ), data, data + size );
            }
            
            pimpl->io.post ( boost::bind ( &AsyncSerial::doWrite, this ) );
        }

        /**
        * Write data asynchronously. Returns immediately.
        * \param data to be sent through the serial device
        */
        void write ( const std::vector < char >&data )
        {
            {
                boost::lock_guard < boost::mutex > l ( pimpl->writeQueueMutex );
                pimpl->writeQueue.insert ( pimpl->writeQueue.end (  ), data.begin (  ),data.end (  ) );
            }
            
            pimpl->io.post ( boost::bind ( &AsyncSerial::doWrite, this ) );
        }

        /**
        * Write a string asynchronously. Returns immediately.
        * Can be used to send ASCII data to the serial device.
        * To send binary data, use write()
        * \param s string to send
        */
        void writeString ( const std::string & s )
        {
            {
                boost::lock_guard < boost::mutex > l ( pimpl->writeQueueMutex );
                pimpl->writeQueue.insert ( pimpl->writeQueue.end (  ), s.begin (  ), s.end (  ) );
            }
            
            pimpl->io.post ( boost::bind ( &AsyncSerial::doWrite, this ) );
        }

        virtual ~AsyncSerial (  )
        {
            if ( isOpen (  ) )
            {
                try
                {
                    close (  );
                }
                catch ( ... )
                {
                    //Don't throw from a destructor
                }
            }
        }
        /**
        * Read buffer maximum size
        */
        static const int readBufferSize = 512;
    
    private:

        /**
        * Callback called to start an Asynchronous read operation.
        * This callback is called by the io_service in the spawned thread.
        */
        void doRead (  )
        {
            pimpl->port.async_read_some ( boost::asio::buffer ( pimpl->readBuffer, readBufferSize ),
                                 boost::bind ( &AsyncSerial::readEnd,
                                               this,
                                               boost::asio::placeholders::error,
                                               boost::asio::placeholders::bytes_transferred ) );
        }

        /**
        * Callback called at the end of the Asynchronous operation.
        * This callback is called by the io_service in the spawned thread.
        */
        void readEnd ( const boost::system::error_code & error, size_t bytes_transferred )
        {
            if ( error )
            {
#ifdef __APPLE__
                if ( error.value (  ) == 45 )
                {
                    //Bug on OS X, it might be necessary to repeat the setup
                    //http://osdir.com/ml/lib.boost.asio.user/2008-08/msg00004.html
                    doRead (  );
                    return;
                }
#endif //__APPLE__

                //error can be true even because the serial port was closed.
                //In this case it is not a real error, so ignore
                if ( isOpen (  ) )
                {
                    doClose (  );
                    setErrorStatus ( true );
                }
            } else
            {
                if ( pimpl->callback )
                pimpl->callback ( pimpl->readBuffer, bytes_transferred );
                doRead (  );
            }
        }
        
        /**
        * Callback called to start an Asynchronous write operation.
        * If it is already in progress, does nothing.
        * This callback is called by the io_service in the spawned thread.
        */
        void doWrite (  )
        {
            //If a write operation is already in progress, do nothing
            if ( pimpl->writeBuffer == 0 )
            {
                boost::lock_guard < boost::mutex > l ( pimpl->writeQueueMutex );
                pimpl->writeBufferSize = pimpl->writeQueue.size (  );
                pimpl->writeBuffer.reset ( new char[pimpl->writeQueue.size (  )] );

                copy ( pimpl->writeQueue.begin (  ), pimpl->writeQueue.end (  ),
                    pimpl->writeBuffer.get (  ) );
                pimpl->writeQueue.clear (  );
                async_write ( pimpl->port, boost::asio::buffer ( pimpl->writeBuffer.get (  ),
                                                        pimpl->writeBufferSize ),
                            boost::bind ( &AsyncSerial::writeEnd, this,
                                            boost::asio::placeholders::error ) );
            }
        }

        /**
        * Callback called at the end of an asynchronuous write operation,
        * if there is more data to write, restarts a new write operation.
        * This callback is called by the io_service in the spawned thread.
        */
        void writeEnd ( const boost::system::error_code & error )
        {
            if ( !error )
            {
                boost::lock_guard < boost::mutex > l ( pimpl->writeQueueMutex );
                if ( pimpl->writeQueue.empty (  ) )
                {
                    pimpl->writeBuffer.reset (  );
                    pimpl->writeBufferSize = 0;

                    return;
                }
                pimpl->writeBufferSize = pimpl->writeQueue.size (  );
                pimpl->writeBuffer.reset ( new char[pimpl->writeQueue.size (  )] );
                copy ( pimpl->writeQueue.begin (  ), pimpl->writeQueue.end (  ),
                    pimpl->writeBuffer.get (  ) );
                pimpl->writeQueue.clear (  );
                async_write ( pimpl->port, boost::asio::buffer ( pimpl->writeBuffer.get (  ),
                                                        pimpl->writeBufferSize ),
                            boost::bind ( &AsyncSerial::writeEnd, this,
                                            boost::asio::placeholders::error ) );
            } else
            {
                setErrorStatus ( true );
                doClose (  );
            }
        }

        /**
        * Callback to close serial port
        */
        void doClose (  )
        {
            boost::system::error_code ec;
            pimpl->port.cancel ( ec );
            if ( ec )
                setErrorStatus ( true );
            pimpl->port.close ( ec );
            if ( ec )
                setErrorStatus ( true );
        }

        boost::shared_ptr <AsyncSerialImpl > pimpl;

    protected:

        /**
        * To allow derived classes to report errors
        * \param e error status
        */
        void setErrorStatus ( bool e )
        {
            boost::lock_guard < boost::mutex > l ( pimpl->errorMutex );
            pimpl->error = e;
        }

        /**
        * To allow derived classes to set a read callback
        */
        void setReadCallback ( const boost::function < void ( const char *, size_t ) > &callback )
        {
            pimpl->callback = callback;
        }


        /**
        * To unregister the read callback in the derived class destructor so it
        * does not get called after the derived class destructor but before the
        * base class destructor
        */
        void clearReadCallback (  )
        {
            pimpl->callback.clear (  );
        }

};

#else //__APPLE__

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

class AsyncSerialImpl:private boost::noncopyable
{
    public:
        AsyncSerialImpl():
        backgroundThread (  ),
        open ( false ),
        error ( false )
        {
        }
        
        boost::thread backgroundThread;         ///< Thread that runs read operations

        bool open;                   ///< True if port open

        bool error;                  ///< Error flag

        mutable boost::mutex errorMutex; ///< Mutex for access to error

        int fd;                        ///< File descriptor for serial port

        char readBuffer[AsyncSerial::readBufferSize];   ///< data being read

        /// Read complete callback
        boost::function < void ( const char *, size_t ) >callback;
};

/**
 * \brief Asyncronous serial class
 * Asyncronous serial class.
 * Intended to be a base class.
 */
class AsyncSerial:private boost::noncopyable
{
    public:

        AsyncSerial()   
        :pimpl ( new AsyncSerialImpl )
        {
        }
    
        /**
            * Constructor. Creates and opens a serial device.
            * \param devname serial device name, example "/dev/ttyS0" or "COM1"
            * \param baud_rate serial baud rate
            * \param opt_parity serial parity, default none
            * \param opt_csize serial character size, default 8bit
            * \param opt_flow serial flow control, default none
            * \param opt_stop serial stop bits, default 1
            * \throws boost::system::system_error if cannot open the
            * serial device
            */
        AsyncSerial ( const std::string & devname, unsigned int baud_rate,
                        boost::asio::serial_port_base::parity opt_parity =
                        boost::asio::serial_port_base::parity ( boost::asio::
                                                                serial_port_base::parity::none ),
                        boost::asio::serial_port_base::character_size opt_csize =
                        boost::asio::serial_port_base::character_size ( 8 ),
                        boost::asio::serial_port_base::flow_control opt_flow =
                        boost::asio::serial_port_base::flow_control ( boost::
                                                                    asio::serial_port_base::flow_control::
                                                                    none ),
                        boost::asio::serial_port_base::stop_bits opt_stop =
                        boost::asio::serial_port_base::stop_bits ( boost::asio::
                                                                    serial_port_base::stop_bits::
                                                                    one ) )
        :pimpl ( new AsyncSerialImpl )
        {
            open ( devname, baud_rate, opt_parity, opt_csize, opt_flow, opt_stop );
        }


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
        void open ( const std::string & devname, unsigned int baud_rate,
                    boost::asio::serial_port_base::parity opt_parity =
                    boost::asio::serial_port_base::parity ( boost::asio::serial_port_base::
                                                            parity::none ),
                    boost::asio::serial_port_base::character_size opt_csize =
                    boost::asio::serial_port_base::character_size ( 8 ),
                    boost::asio::serial_port_base::flow_control opt_flow =
                    boost::asio::serial_port_base::flow_control ( boost::asio::
                                                                serial_port_base::flow_control::
                                                                none ),
                    boost::asio::serial_port_base::stop_bits opt_stop =
                    boost::asio::serial_port_base::stop_bits ( boost::asio::
                                                                serial_port_base::stop_bits::
                                                                one ) )
        {
            if ( isOpen (  ) )
                close (  );

            setErrorStatus ( true );     //If an exception is thrown, error remains true

            struct termios new_attributes;

            speed_t speed;

            int status;

            // Open port
            pimpl->fd =::open ( devname.c_str (  ), O_RDWR | O_NOCTTY | O_NONBLOCK );
            if ( pimpl->fd < 0 )
                throw ( boost::system::system_error ( boost::system::error_code (  ),"Failed to open port" ) );

            // Set Port parameters.
            status = tcgetattr ( pimpl->fd, &new_attributes );
            if ( status < 0 || !isatty ( pimpl->fd ) )
            {
                ::close ( pimpl->fd );
                throw ( boost::system::system_error ( boost::system::error_code (  ),"Device is not a tty" ) );
            }
            new_attributes.c_iflag = IGNBRK;
            new_attributes.c_oflag = 0;
            new_attributes.c_lflag = 0;
            new_attributes.c_cflag = ( CS8 | CREAD | CLOCAL );   //8 data bit,Enable receiver,Ignore modem
            /* In non canonical mode (Ctrl-C and other disabled, no echo,...) VMIN and VTIME work this way:
                if the function read() has'nt read at least VMIN chars it waits until has read at least VMIN
                chars (even if VTIME timeout expires); once it has read at least vmin chars, if subsequent
                chars do not arrive before VTIME expires, it returns error; if a char arrives, it resets the
                timeout, so the internal timer will again start from zero (for the nex char,if any) */
            new_attributes.c_cc[VMIN] = 1;   // Minimum number of characters to read before returning error
            new_attributes.c_cc[VTIME] = 1;  // Set timeouts in tenths of second

            // Set baud rate
            switch ( baud_rate )
            {
            case 50:
                speed = B50;
                break;
            case 75:
                speed = B75;
                break;
            case 110:
                speed = B110;
                break;
            case 134:
                speed = B134;
                break;
            case 150:
                speed = B150;
                break;
            case 200:
                speed = B200;
                break;
            case 300:
                speed = B300;
                break;
            case 600:
                speed = B600;
                break;
            case 1200:
                speed = B1200;
                break;
            case 1800:
                speed = B1800;
                break;
            case 2400:
                speed = B2400;
                break;
            case 4800:
                speed = B4800;
                break;
            case 9600:
                speed = B9600;
                break;
            case 19200:
                speed = B19200;
                break;
            case 38400:
                speed = B38400;
                break;
            case 57600:
                speed = B57600;
                break;
            case 115200:
                speed = B115200;
                break;
            case 230400:
                speed = B230400;
                break;
            default:
                {
                    ::close ( pimpl->fd );
                    throw ( boost::system::system_error ( boost::system::error_code (  ),"Unsupported baud rate" ) );
                }
            }

            cfsetospeed ( &new_attributes, speed );
            cfsetispeed ( &new_attributes, speed );

            //Make changes effective
            status = tcsetattr ( pimpl->fd, TCSANOW, &new_attributes );
            if ( status < 0 )
            {
                ::close ( pimpl->fd );
                throw ( boost::system::system_error ( boost::system::error_code (  ),"Can't set port attributes" ) );
            }
            //These 3 lines clear the O_NONBLOCK flag
            status = fcntl ( pimpl->fd, F_GETFL, 0 );
            if ( status != -1 )
                fcntl ( pimpl->fd, F_SETFL, status & ~O_NONBLOCK );

            setErrorStatus ( false );    //If we get here, no error
            pimpl->open = true;          //Port is now open

            boost::thread t ( bind ( &AsyncSerial::doRead, this ) );

            pimpl->backgroundThread.swap ( t );
        }

        /**
        * \return true if serial device is open
        */
        bool isOpen (  ) const
        {
            return pimpl->open;
        }

        /**
        * \return true if error were found
        */
        bool errorStatus (  ) const
        {
            boost::lock_guard < boost::mutex > l ( pimpl->errorMutex );
            return pimpl->error;
        }

        /**
        * Close the serial device
        * \throws boost::system::system_error if any error
        */
        void close()
        {
            if ( !isOpen (  ) )
                return;

            pimpl->open = false;

            ::close ( pimpl->fd );       //The thread waiting on I/O should return

            pimpl->backgroundThread.join (  );
            if ( errorStatus (  ) )
            {
                throw ( boost::system::system_error ( boost::system::error_code (  ),"Error while closing the device" ) );
            }
        }


        /**
        * Write data asynchronously. Returns immediately.
        * \param data array of char to be sent through the serial device
        * \param size array size
        */
        void write ( const char *data, size_t size )
        {
            if ( ::write ( pimpl->fd, data, size ) != size )
                setErrorStatus ( true );
        }
        
        /**
        * Write data asynchronously. Returns immediately.
        * \param data to be sent through the serial device
        */
        void write ( const std::vector < char >&data )
        {
            if ( ::write ( pimpl->fd, &data[0], data.size (  ) ) != data.size (  ) )
                setErrorStatus ( true );
        }

        /**
        * Write a string asynchronously. Returns immediately.
        * Can be used to send ASCII data to the serial device.
        * To send binary data, use write()
        * \param s string to send
        */
        void writeString ( const std::string & s )
        {
            if ( ::write ( pimpl->fd, &s[0], s.size (  ) ) != s.size (  ) )
                setErrorStatus ( true );
        }

        virtual ~AsyncSerial (  )
        {
            if ( isOpen (  ) )
            {
                try
                {
                    close (  );
                }
                catch ( ... )
                {
                    //Don't throw from a destructor
                }
            }
        }

        /**
        * Read buffer maximum size
        */
        static const int readBufferSize = 512;
    
    private:

        /**
        * Callback called to start an Asynchronous read operation.
        * This callback is called by the io_service in the spawned thread.
        */
        void doRead (  )
        {
            //Read loop in spawned thread
            for ( ;; )
            {
                int
                received =::read ( pimpl->fd, pimpl->readBuffer, readBufferSize );

                if ( received < 0 )
                {
                    if ( isOpen (  ) == false )
                        return;         //Thread interrupted because port closed
                    else
                    {
                        setErrorStatus ( true );
                        continue;
                    }
                }
                
                if ( pimpl->callback )
                    pimpl->callback ( pimpl->readBuffer, received );
            }
        }

        /**
        * Callback called at the end of the Asynchronous operation.
        * This callback is called by the io_service in the spawned thread.
        */
        void readEnd ( const boost::system::error_code & error, size_t bytes_transferred )
        {
            //Not used
        }
        
        /**
        * Callback called to start an Asynchronous write operation.
        * If it is already in progress, does nothing.
        * This callback is called by the io_service in the spawned thread.
        */
        void doWrite (  )
        {
            //Not used
        }

        /**
        * Callback called at the end of an asynchronuous write operation,
        * if there is more data to write, restarts a new write operation.
        * This callback is called by the io_service in the spawned thread.
        */
        void writeEnd ( const boost::system::error_code & error )
        {
            //Not used
        }

        /**
        * Callback to close serial port
        */
        void doClose (  )
        {
            //Not used
        }

        boost::shared_ptr <AsyncSerialImpl > pimpl;

    protected:

        /**
        * To allow derived classes to report errors
        * \param e error status
        */
        void setErrorStatus ( bool e )
        {
            boost::lock_guard < boost::mutex > l ( pimpl->errorMutex );
            pimpl->error = e;
        }

        /**
        * To allow derived classes to set a read callback
        */
        void setReadCallback ( const boost::function < void ( const char *, size_t ) > &callback )
        {
            pimpl->callback = callback;
        }


        /**
        * To unregister the read callback in the derived class destructor so it
        * does not get called after the derived class destructor but before the
        * base class destructor
        */
        void clearReadCallback (  )
        {
            pimpl->callback.clear (  );
        }

};

#endif //__APPLE__

/**
 * \brief Asynchronous serial class with read callback
 * Asynchronous serial class with read callback. User code can write data
 * from one thread, and read data will be reported through a callback called
 * from a separate thred.
 */
class CallbackAsyncSerial: public AsyncSerial
{
    public:
        CallbackAsyncSerial (  )
        :AsyncSerial (  )
        {

        }

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
        CallbackAsyncSerial ( const std::string & devname, unsigned int baud_rate,
                         boost::asio::serial_port_base::parity opt_parity =
                         boost::asio::serial_port_base::parity ( boost::asio::serial_port_base::parity::
                                                                 none ),
                         boost::asio::serial_port_base::character_size opt_csize =
                         boost::asio::serial_port_base::character_size ( 8 ),
                         boost::asio::serial_port_base::flow_control opt_flow =
                         boost::asio::serial_port_base::flow_control ( boost::asio::serial_port_base::flow_control::
                                                                       none ),
                         boost::asio::serial_port_base::stop_bits opt_stop =
                         boost::asio::serial_port_base::stop_bits ( boost::asio::serial_port_base::stop_bits::
                                                                    one ) )
        :AsyncSerial ( devname, baud_rate, opt_parity, opt_csize, opt_flow, opt_stop )
        {

        }

        /**
        * Set the read callback, the callback will be called from a thread
        * owned by the CallbackAsyncSerial class when data arrives from the
        * serial port.
        * \param callback the receive callback
        */
        void setCallback ( const boost::function < void ( const char *, size_t ) > &callback )
        {
            setReadCallback ( callback );
        }

        /**
        * Removes the callback. Any data received after this function call will
        * be lost.
        */
        void clearCallback (  )
        {
            clearReadCallback (  );
        }

        virtual ~ CallbackAsyncSerial (  )
        {
            clearReadCallback (  );
        }
};

#endif //ASYNCSERIAL_H
