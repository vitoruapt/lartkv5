
/**
 * @file rs232_tcp.cpp
 * @author Telmo Rafeiro n.º 45349 (rafeiro@ua.pt)
 * @brief Communication with 1 IMU and Fanuc for its data aquisition.
 */

//TO LAUNCH THIS SCRIPT CORRETLY, must type "rosrun package fanuc_exp FILENAME S_MODE"
// S_MODE -> 1 to euler angles
// S_MODE -> 0 to raw values

#include <ros/ros.h>
#include "std_msgs/String.h"
#include <sstream>
#include <string>
//INCLUDE OF MESSAGE STRUCTURE
#include <trafeiro/fanuc_experiment.h>
//INCLUDE BOOST LIBS
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
//INCLUDE SERIAL LIB FROM LAR3/utils
#include <serialcom/SerialCom.h>
#include <algorithm>

using namespace std;

using boost::asio::ip::tcp;
/** 
 * @file
 * @brief Class for Fanuc data aquisition.
 */
class Position
{
	public:
		Position()
		{
			x=0;
			y=0;
			z=0;
			w=0;
			p=0;
			r=0;
			conf1=0;
			conf2=0;
			conf3=0;
			turn1=0;
			turn2=0;
			turn3=0;
		}
		
		double x,y,z;
		double w,p,r;
		int conf1,conf2,conf3;
		int turn1,turn2,turn3;
};

/** 
 * @file
 * @brief Class for Fanuc data aquisition.
 */
class CommunicationHandler
{
	public:
		CommunicationHandler(boost::asio::io_service& io_service,tcp::resolver::iterator endpoint_iterator)
		:io_service_(io_service),
		socket_(io_service)
		{
			tcp::endpoint endpoint = *endpoint_iterator;
			socket_.async_connect(endpoint,	boost::bind(&CommunicationHandler::handle_connect, this,boost::asio::placeholders::error, ++endpoint_iterator));
		}

		void getcrcpos(Position& pos)
		{
			string data;
		
			char msg[1000];
			strcpy(msg,"GETCRCPOS\n");
		
			write(msg,strlen(msg));
		
			boost::asio::streambuf response;
			istream response_stream(&response);
			
			boost::asio::read_until(socket_, response, "\n");
			getline(response_stream, data);//Current position

			boost::asio::read_until(socket_, response, "\n");
			getline(response_stream, data);// X , Y , Z
			
			//STRING SPLITING
			istringstream iss(data);
			for(int a=0;a<3;a++)
			{
				if (a==0) iss >> pos.x;
				if (a==1) iss >> pos.y;
				if (a==2) iss >> pos.z;
			}
			cout<<data<<endl;
			
			boost::asio::read_until(socket_, response, "\n");
			getline(response_stream, data);// W , P , R
			
			istringstream iss2(data);
			
			for(int a=0;a<3;a++)
			{
				if (a==0) iss2 >> pos.w;
				if (a==1) iss2 >> pos.p;
				if (a==2) iss2 >> pos.r;
			}
			
			boost::asio::read_until(socket_, response, "\n");
			getline(response_stream, data);// F , U , T
// 			cout<<data<<endl;
			
			boost::asio::read_until(socket_, response, "\n");
			getline(response_stream, data);// REDUNDÂNCIAS
// 			cout<<data<<endl;
		}

		void runtpp(char *tppname)
		{
			//PARA QUE ESTA FUNÇÃO FUNCIONE É PRECISO ENVIAR "1" a seguir ao nome da função
			string data;
			char msg[1000];

			strcpy(msg,"RUNTPP\n");
			strcat(msg,tppname);
			strcat(msg,"\n1\n");
			
			write(msg,strlen(msg));
			
			boost::asio::streambuf response;
			istream response_stream(&response);
			
			boost::asio::read_until(socket_, response, "\n");
			getline(response_stream, data);//On sucess it replies 1
			
// 			cout<<data<<endl;
		}
		
		void write(char* msg, int length)
		{
			do_write(msg,length);
		}

		void close()
		{
			do_close();
		}
		

	private:
		char data_[1000];
		
		void handle_connect(const boost::system::error_code& error,tcp::resolver::iterator endpoint_iterator)
		{
			if (!error)
			{
				
			}else if (endpoint_iterator != tcp::resolver::iterator())
			{
				socket_.close();
				tcp::endpoint endpoint = *endpoint_iterator;
				socket_.async_connect(endpoint,	boost::bind(&CommunicationHandler::handle_connect, this,boost::asio::placeholders::error, ++endpoint_iterator));
			}
		}

		void do_write(char*msg,int length)
		{
			boost::asio::write(socket_,boost::asio::buffer(msg,length));
		}

		void do_close()
		{
			socket_.close();
		}

	private:
		
		boost::asio::io_service& io_service_;
		tcp::socket socket_;
		
};

/**
* @fn int main ( int argc, char **argv )
* Main 
*/
int main (int argc ,char** argv)
{
	serialcom::SerialCom serial;
	
	
    ros::init(argc , argv , "fanux_exp");
    ros::NodeHandle n;
    ros::Publisher chatter_pub;
    chatter_pub = n.advertise<trafeiro::fanuc_experiment>("topic_exp", 1000);

	ros::Rate loop_rate(20);

	try
	{
		std::string buffer;
		std::string buffer_aux;
		std::string Ax,Ay,Az,Mx,My,Mz,Gx,Gy,Gz,Y,P,R;
		int MODE;
		std::string junk;
		Position p1;
		int findPV, findSV;

		serial.open("/dev/ttyACM0",57600);
		
		usleep(3000000);
	
		boost::asio::io_service io_service;

		tcp::resolver resolver(io_service);
		tcp::resolver::query query("192.168.0.230","4901");

		tcp::resolver::iterator iterator = resolver.resolve(query);

		CommunicationHandler c(io_service, iterator);
		
		trafeiro::fanuc_experiment experiment;

		char *tppname;
		
		if (argc>2)
		{
			if (strcmp(argv[2],"1")>=0)
			{
				MODE = 1;
			}
			if (strcmp(argv[2],"0")>=0)
			{
				serial.write("#osct");
				MODE = 0;
			}
			if ((strcmp(argv[2],"1")<0) && (strcmp(argv[2],"0")>0))
			{
				cout<<"SENSOR output MODE invalid!"<<endl;
				return 0;
			}
			tppname = argv[1];
		}
		else
		{
			cout<<"TPP NAME or SENSOR MODE MISSING ... TYPE fanuc_exp FILENAME SENSOR MODE"<<endl;
			return 0;
// 			//once again,
// 			//TO LAUNCH THIS SCRIPT CORRETLY, it is necessary type "rosrun package fanuc_exp FILENAME S_MODE
// 			// S_MODE -> 1 to euler angles
// 			// S_MODE -> 0 to raw values
		}
		c.runtpp(tppname);
		usleep(500000);

		// WHERE MAGIC HAPPENS!!!
		// ////////////////////////////////////////////////////
		// ////////////////////////////////////////////////////
		while(ros::ok())
		{

		if(MODE==1)
			{
				serial.write("#f");
				usleep(5000);
				serial.readLine(buffer);
				serial.readLine(buffer_aux);//The sensor sends a additional empty line
				if(buffer.size()>0)
				{
					std::basic_string <char> str2 = buffer.substr(5);

					findPV = str2.find(",");
					std::basic_string <char> inf3 = str2.substr(findPV+1);
					findSV = inf3.find(",");
					
					// WRITING SENSOR VALUES IN TOPIC VARIABLE
					experiment.S_Y = (double) atof(str2.substr(0,findPV).c_str());
					experiment.S_P = (double) atof(inf3.substr(0,findSV).c_str());
					experiment.S_R = (double) atof(inf3.substr(findSV+1).c_str());
				}
			}
			else
			{

				
				serial.flush();
				serial.write("#f");
				
				string sa;
				string sm;
				string sg;
				
					try
					{				
						serial.readLine(&sa,100);
						serial.readLine(&sm,100);
						serial.readLine(&sg,100);
					}catch(serialcom::Exception ex)
					{
						cout<<ex.what()<<endl;
					}
					
					int ncsa=count(sa.begin(), sa.end(), ',');
					int ncsm=count(sm.begin(), sm.end(), ',');
					int ncsg=count(sg.begin(), sg.end(), ',');
					int na=count(sa.begin(), sa.end(), 'A');
					int nm=count(sm.begin(), sm.end(), 'M');
					int ng=count(sg.begin(), sg.end(), 'G');

					if((ncsa!=3 || ncsm!=3 || ncsg!=3) && (na==1 && nm==1 && ng==1))
					{
						exit(0);
					}
					else
					{
						
					}
				
				istringstream ss(sa);
				getline(ss,junk,'=');
				getline(ss,Ax, ',');
				getline(ss,Ay, ',');
				getline(ss,Az, ',');
				experiment.S_Ax = (double) atof(Ax.c_str());
				experiment.S_Ay = (double) atof(Ay.c_str());
				experiment.S_Az = (double) atof(Az.c_str());

				
				istringstream ss2(sm);
				getline(ss2,junk,'=');
				getline(ss2,Mx, ',');
				getline(ss2,My, ',');
				getline(ss2,Mz, ',');
				experiment.S_Mx = (double) atof(Mx.c_str());
				experiment.S_My = (double) atof(My.c_str());
				experiment.S_Mz = (double) atof(Mz.c_str());

				
				istringstream ss3(sg);
				getline(ss3,junk,'=');
				getline(ss3,Gx, ',');
				getline(ss3,Gy, ',');
				getline(ss3,Gz, ',');
				experiment.S_Gx = (double) atof(Gx.c_str());
				experiment.S_Gy = (double) atof(Gy.c_str());
				experiment.S_Gz = (double) atof(Gz.c_str());
				

			}
// // 			//GETTING FANUC'S POSITION
			c.getcrcpos(p1);

// 			WRITING FANUC VALUES IN TOPIC VARIABLE
			experiment.F_x = p1.x;
			experiment.F_y = p1.y;
			experiment.F_z = p1.z;
			experiment.F_w = p1.w;
			experiment.F_r = p1.r;
			experiment.F_p = p1.p;
			
			chatter_pub.publish(experiment);
			
			loop_rate.sleep();

		}

		c.close();
		serial.close();
	}catch (std::exception& e)
	{
		std::cerr << "Exception: " << e.what() << "\n";
	}

	return 0;
    
}