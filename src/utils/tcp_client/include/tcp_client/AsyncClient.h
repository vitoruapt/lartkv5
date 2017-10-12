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
#ifndef _ASYNC_CLIENT_H_
#define _ASYNC_CLIENT_H_

/**
 * \file
 * \brief Asynchronous tcp/ip object declaration
*/

#include <iostream>
#include <istream>
#include <ostream>
#include <string>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/signal.hpp>
#include <boost/thread/thread.hpp>

using boost::asio::ip::tcp;

using namespace std;

/**
\brief Asynchronous tpc/ip communication object

This class preforms asynchronous communication with a tcp/ip socket object.
*/
class AsyncClient
{
	public:
		/**
		 * \brief Class constructor
		 * 
		 * The constructor initializes the server ip and port, the resolver and also the socket.
		 * It also calls the startConnection function which proceeds with establishing a connection to the server.
		 */
		AsyncClient(boost::asio::io_service& io_service,const std::string& server, const std::string& port,std::string delimiter=std::string("\n")):
		delimiter_(delimiter),
		server_ip_(server),
		server_port_(port),
		query_(server_ip_,server_port_),
		resolver_(io_service),
		socket_(io_service)
		{
			attempting_connection_=false;
			is_connected_=false;
			error_ = boost::asio::error::not_connected;
			
			startConnection();
		}
		
		/**
		 * \brief Class destructor
		 * 
		 * This function only closes the socket communication.
		 */
		~AsyncClient()
		{
			socket_.close();
		}
		
		/**
		 * \brief Starts a query on the ip and port and starts the asynchronous resolver.
		 * 
		 * This function stats the query of the ip and port and the asynchronous resolver. 
		 * The resolver will try to establish a connection to the endpoints returned by the query.
		 */
		void startConnection()
		{
			if(attempting_connection_)
				return;
			
			try
			{
				socket_.close();
			}catch(std::exception& e)
			{
// 				std::cout<<"Cannot close socket: "<<e.what()<<std::endl;
			}
			
			attempting_connection_=true;
			
			
			//Sets the established connection variable to false
			is_connected_=false;
			
			//A small delay to avoid unnecessary taxation of the thread
			boost::this_thread::sleep(boost::posix_time::milliseconds(50));
			
			//Stat the asynchronous resolver on the query object
			resolver_.async_resolve(query_,boost::bind(&AsyncClient::handleResolve, this,boost::asio::placeholders::error,boost::asio::placeholders::iterator));
		}
		
		/**
		 * \brief Get the connection status
		 * 
		 * \return true if the connection is established, false if not
		 */
		bool isConnected(void)
		{
			return is_connected_;
		}
		
		/**
		 * \brief Close the connection
		 */
		void close(void)
		{
			socket_.close();
			error_ = boost::asio::error::not_connected;
			is_connected_ = false;
		}
		
		/**
		 * \brief Function to reestablish the connection
		 */
		void reconnect(void)
		{
			//Attempt a new connection
			startConnection();	
		}
		
		/**
		 * \brief Send a message to the server
		 * \param message std::string containing the outgoing message
		 * 
		 * This function converts the std::string to a boost::asio::streambuf and stats a asynchronous write request with that buffer.
		 * If the connection is not established a boost::asio::error::not_connected error is thrown (it can be caught using a std::exception).
		 */
		void write(std::string message)
		{
			if(!is_connected_)
				throw boost::system::system_error(boost::asio::error::not_connected);
			
			std::ostream request_stream(&request_);
			request_stream << message;
			boost::asio::async_write(socket_,request_,boost::bind(&AsyncClient::writeRequestHandler, this,boost::asio::placeholders::error,boost::asio::placeholders::bytes_transferred));
		}
		
		///New data function handler
		boost::signal<void (std::string)> readHandler;
		///Successful writing handler
		boost::signal<void (void)> writeHandler;
		///Successful connection establish handler
		boost::signal<void (void)> connectHandler;
		///Current error code of the communication
		boost::system::error_code error_;
		
	private:
		/**
		 * \brief Write handler
		 * \param err status of the request
		 * \param bytes_transferred number of bytes transfered
		 * 
		 * This function is called upon asynchronous write of data to the socket.
		 * The user may preform a specific action after by setting the writeHandler.
		 * The user function will only be called on a successful write.
		 */
		void writeRequestHandler(const boost::system::error_code& err,std::size_t bytes_transferred)
		{
			if(err)
			{
				//Connection is not established
				is_connected_=false;
				//Copy the error to the global variable
				error_=err;
				
				attempting_connection_=false;
				
				//Attempt a new connection
				startConnection();
				
				return;
			}
			
			writeHandler();
		}
		
		/**
		 * \brief Query resolve handler
		 * \param err error status
		 * \param endpoint_iterator current iterator to the list of endpoints
		 * 
		 * This function is called when resolving the endpoints tying to establish a connection.
		 * If there is a error on the resolver a new connection is attempt.
		 * If there is no error a connection is attempt on the endpoint.
		 */
		void handleResolve(const boost::system::error_code& err,tcp::resolver::iterator endpoint_iterator)
		{
			//If there's a error start the process from the begin
			if(err)
			{
				//Connection is not established
				is_connected_=false;
				//Copy the error to the global variable
				error_=err;
				
				attempting_connection_=false;
				
				//Attempt a new connection
				startConnection();
				
				return;
			}
			
			// Attempt a connection to the first endpoint in the list. Each endpoint
			// will be tried until we successfully establish a connection.
			//Get the first endpoint of the list
			tcp::endpoint endpoint = *endpoint_iterator;
			
			//Attempt a asynchronous connection to the first endpoint, the function handleConnect will try to connect to the rest of the endpoints if the connection fails on the first
			socket_.async_connect(endpoint,boost::bind(&AsyncClient::handleConnect, this,boost::asio::placeholders::error, ++endpoint_iterator));
		}
		
		/**
		 * \brief Handle a connection
		 * \param err error status
		 * \param endpoint_iterator iterator to the endpoints list
		 * 
		 * This function is called after an attempt to connect to a endpoint.
		 * If the connection was successful (err==false) the function startRead is used to schedule a read operation.
		 * If the connection was unsuccessful, if the endpoint is not the last in the list a new connection is attempt on the next endpoint, if there are no more endpoints in the list we start a new connection from the beginning. 
		 */
		void handleConnect(const boost::system::error_code& err,tcp::resolver::iterator endpoint_iterator)
		{
			//If this function was called after the connection was established do nothing
			if(is_connected_)
				return;
			
			//If there is no error
			if (!err)
			{
				//Attempt to read data
				startRead();
				
				//Copy the error to the global variable
				error_=err;
				
				//Set the connection status to established
				is_connected_=true;
				
				attempting_connection_=false;
				
				//Call the user successful connection handler
				connectHandler();
			}
			else if (endpoint_iterator != tcp::resolver::iterator())
			{
				// The connection failed. Try the next endpoint in the list.
				
				//close the current socket
				socket_.close();
				
				//Get the next endpoint
				tcp::endpoint endpoint = *endpoint_iterator;
				//Attempt a connect on the next endpoint, the current function is set has a the handler for the connection attempt
				socket_.async_connect(endpoint,boost::bind(&AsyncClient::handleConnect,this,boost::asio::placeholders::error, ++endpoint_iterator));
			}
			else
			{
				//The list has no more endpoints
		
				//Set the connection status to not established
				is_connected_=false;
				
				attempting_connection_=false;
				
				//Copy the error to the global variable
				error_=err;
				
				if(error_ == boost::asio::error::operation_aborted || error_ == boost::asio::error::already_started)
					error_=boost::asio::error::host_unreachable;
					
				//Attempt a new connection from the beginning
				startConnection();
			}
		}
		
		/**
		 * \brief Start a read asynchronous operation
		 * 
		 * This function schedules a asynchronous read operation till a new line character appears.
		 * The function handleRead is scheduled to be called upon receiving the new data
		 */
		void startRead(void)
		{
			// Start an asynchronous operation to read a newline-delimited message.
			boost::asio::async_read_until(socket_,response_,delimiter_,boost::bind(&AsyncClient::handleRead,this,boost::asio::placeholders::error,boost::asio::placeholders::bytes_transferred));
		}
		
		/**
		 * \brief New data handler
		 * \param err error status
		 * \param bytes_transferred number of bytes received
		 * 
		 * This function is called upon a read attempt.
		 * If there was an error this function schedules a new connection attempt.
		 * On a successful read operation, the data is copied to a string and the user read handler is called.
		 * The function finalizes by scheduling a new read operation.
		 */
		void handleRead(const boost::system::error_code& err, std::size_t bytes_transferred)
		{
			//If there was some error on the operation
			if(err)
			{
				//Set the connection status variable to false
				is_connected_=false;
				
				//Copy the error to the global variable
				error_ = err;
				
				attempting_connection_=false;
				
				//Attempt a new connection from the beginning
				startConnection();
				
				return;
			}
			
			//Create a istream from the streambuf
			std::istream response_stream(&response_);
			//Create a string variable
			std::string line;
			//Copy from the istream to the string variable
			std::getline(response_stream, line);
			
			//Call the user read handler
			readHandler(line);
			
			//Schedule a new read operation
			startRead();
		}
		
		///String used as delimiter in the read until function, default is \\n
		std::string delimiter_;
		
		///Connection status variable
		bool is_connected_;
		
		///Ongoing reconnection
		bool attempting_connection_;

		///Ip of the server
		std::string server_ip_;
		///Port of the server
		std::string server_port_;
		
		///Start an asynchronous resolve to translate the server and service names into a list of endpoints.
		tcp::resolver::query query_;
		
		///Connection resolver, the resolver obtains the endpoints for the connection
		tcp::resolver resolver_;
		///Connection socket variable
		tcp::socket socket_;
		///Stream buffer to hold the communications requests
		boost::asio::streambuf request_;
		///Stream buffer to hold the communications responses
		boost::asio::streambuf response_;
};

#endif
