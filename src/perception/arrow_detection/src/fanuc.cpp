/**************************************************************************************************
  * Software License Agreement (BSD License)
  *
  * Copyright (c) 2011-2014, LAR toolkit developers - University of Aveiro - http://lars.mec.ua.pt
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without modification, are permitted
  * provided that the following conditions are met:
  *
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

 /*!
 *      @file   fanuc.cpp
 *      @brief  connect computer to fanuc to extract comands and extract data positions of it!
 *
 *      @author         César Sousa, cesarsousa@ua.pt
 *      @date 6-3-2014
 *      @version V0.0
 *      @internal
 *
 *              Revision        ---
 *              Compiler        gcc
 *              Company         DEM - Universidade de Aveiro
 *              Copyright       Copyright (c) 2014, César Sousa
 *
 *              Info:
 *
 *      command to make doxyfile: "make doxyfile" than "make doc"
 *
 */

#include <arrow_detection/fanuc.h>

//struct that contain a specific position of fanuc! 
struct pos_orient
{
    double x,y,z;
    double w,p,r;
};

struct configurations
{
    int conf1,conf2,conf3;
    int turn1,turn2,turn3;
};

std::string IntToString (int a)
{
    std::ostringstream temp;
    temp<<a;
    return temp.str();
}

class pos_orient_conf
{
    public:
    pos_orient position_orientation;
    configurations config;
    
    //constructor (overload!)
    pos_orient_conf()
    {
        position_orientation.x = 0;
        position_orientation.y = 0;
        position_orientation.z = 0;
        position_orientation.p = 0;
        position_orientation.r = 0;
        position_orientation.w = 0;
        config.conf1 = 0;
        config.conf2 = 1;
        config.conf3 = 1;
        config.turn1 = 0;
        config.turn2 = 0;
        config.turn3 = 0;
    }

    pos_orient_conf(int x,int y, int z, int p, int r, int w)
    {
        position_orientation.x = x;
        position_orientation.y = y;
        position_orientation.z = z;
        position_orientation.p = p;
        position_orientation.r = r;
        position_orientation.w = w;
        config.conf1 = 0;
        config.conf2 = 1;
        config.conf3 = 1;
        config.turn1 = 0;
        config.turn2 = 0;
        config.turn3 = 0;
    }

    pos_orient_conf(int x,int y, int z, int p, int r, 
        int w,int conf1,int conf2,int conf3, int turn1,int turn2,int turn3 )
    {
        position_orientation.x = x;
        position_orientation.y = y;
        position_orientation.z = z;
        position_orientation.p = p;
        position_orientation.r = r;
        position_orientation.w = w;
        config.conf1 = conf1;
        config.conf2 = conf2;
        config.conf3 = conf3;
        config.turn1 = turn1;
        config.turn2 = turn2;
        config.turn3 = turn3;
    }
    
    //check if the imputed position is the same as the This object (excluding the configuraions)
    bool same_pos_orient(pos_orient potition,double gama)
    {
        if( fabs(potition.x-position_orientation.x)>gama)
            return false;
        
        if( fabs(potition.y-position_orientation.y)>gama)
            return false;
        
        if( fabs(potition.z-position_orientation.z)>gama)
            return false;
        
        if( fabs(potition.p-position_orientation.p)>gama)
            return false;
        
        if( fabs(potition.r-position_orientation.r)>gama)
            return false;
            
        if( fabs(potition.w-position_orientation.w)>gama)
            return false;
        
        return true;
    }
    
    //check if the imputed position is the same as the This object (including the configuraions)
    bool same_pos_orient_conf(pos_orient_conf potition,double gama_pos,double gama_conf)
    {        
        //position
        if( !same_pos_orient(potition.position_orientation,gama_pos))
            return false;
            
        //Configuration
        if( fabs(potition.config.conf1 - config.conf1)>gama_conf)
            return false;
            
        if( fabs(potition.config.conf2 - config.conf1)>gama_conf)
            return false;
        
        if( fabs(potition.config.conf3 - config.conf1)>gama_conf)
            return false;
        
        if( fabs(potition.config.turn1 - config.turn1)>gama_conf)
            return false;
        
        if( fabs(potition.config.turn2 - config.turn2)>gama_conf)
            return false;
        
        if( fabs(potition.config.turn3 - config.turn3)>gama_conf)
            return false;
        
        return true;
    }
    //Print all values
    void print()
    {
        using namespace std;
        //std::cout << std::endl;
        //std::cout << "position_orientation.x: " << position_orientation.x << std::endl;
        //std::cout << "position_orientation.y: " << position_orientation.y << std::endl;
        //std::cout << "position_orientation.z: " << position_orientation.z << std::endl;
        //std::cout << "position_orientation.p: " << position_orientation.p << std::endl;
        //std::cout << "position_orientation.r: " << position_orientation.r << std::endl;
        //std::cout << "position_orientation.w: " << position_orientation.w << std::endl;
        //std::cout << "config.conf1: " << config.conf1 << std::endl;
        //std::cout << "config.conf2: " << config.conf2 << std::endl;
        //std::cout << "config.conf3: " << config.conf3 << std::endl;
        //std::cout << "config.turn1: " << config.turn1 << std::endl;
        //std::cout << "config.turn2: " << config.turn2 << std::endl;
        //std::cout << "config.turn3: " << config.turn3 << std::endl;
        //std::cout << std::endl;
        
        ofstream outfile;
        
        outfile.open("positions.txt",std::ofstream::out | std::ofstream::app);
        
        outfile << ros::Time::now() << "," << position_orientation.x << "," << position_orientation.y << "," << position_orientation.z
        << "," << position_orientation.p << "," << position_orientation.r << "," << position_orientation.w << std::endl;
        
        outfile << config.conf1 << "," << config.conf2 << "," << config.conf3 << 
        "," << config.turn1 << "," << config.turn2 << "," << config.turn3 << std::endl;
        
        outfile.close();
    }
};

void help()
{
    std::cout  << std::endl;
    std::cout << "\nCommands:" << std::endl;
    std::cout << "Start: 'start' " << std::endl;
    std::cout  << std::endl;
}

void init_vars()
{
    start = false;
}

class CommunicationHandler
{
    public:
    CommunicationHandler(boost::asio::io_service& io_service, boost::asio::ip::tcp::resolver::iterator endpoint_iterator)
    :io_service_(io_service), socket_(io_service)
    {
std::cout << "CommunicationHandler beging constructor " << std::endl;
        boost::asio::ip::tcp::tcp::endpoint endpoint = *endpoint_iterator;
        socket_.async_connect(endpoint, boost::bind(&CommunicationHandler::handle_connect, 
                    this,boost::asio::placeholders::error, ++endpoint_iterator));

std::cout << "CommunicationHandler ending constructor "<< std::endl;
    }
    /** 
 *  @brief send fanuc to a specific position
 * \n 
 */
    bool go_to_position(pos_orient_conf set_position,int regist,std::string prog_name,
                            double gama_pos, double gama_conf)
    {
          
std::cerr << "go_to_position function"<< std::endl;
        
        //setreg(regist,set_position);
std::cerr << "sleep...."  << std::endl;
usleep(1000000); 
std::cerr << "waked up" << std::endl;  
        runtpp(prog_name);
        
        //this variable indicates if the prgram is just starting or not! 
        //      because the need to save the prev position
//         bool just_starting= true; 
        
        //because the problem of the Fanuc sending its actual position at the begining of the program, thus
        //      it is needed to wait some iterations
//         int n_iterations= 500, cont_iterations = 0;
        
        //will the the current position/configuration
        pos_orient_conf current_pos_conf, prev_pos_conf;
std::cerr << "go_to_position:: before starting the loop"<< std::endl;
        while(true)
        {
            
            current_pos_conf = get_end_efector_position();
            current_pos_conf.print();
            
            //200hz??- tempo de comunicação
            boost::this_thread::sleep(boost::posix_time::millisec(5));

//std::cout << ".";
        }
    }

//pos_orient_conf CommunicationHandler::get_end_efector_position()
pos_orient_conf get_end_efector_position()
    { 
//std::cout << " get_end_efector_position function" << std::endl;
        char test;
        pos_orient_conf result(0,0,0,0,0,0,0,0,0,0,0,0);
        std::string data;//will save the last received string

        std::string msg="GETCRCPOS\n";
        write(msg); //ask Fanuc wish is it actual position
//std::cout << "sending GETCRCPOS" << std::endl;

        boost::asio::streambuf response;
        std::istream response_stream(&response);

        boost::asio::read_until(socket_, response, "\n");
        getline(response_stream, data);//Current position

//std::cout << "receiving response" << data << std::endl;

        boost::asio::read_until(socket_, response, "\n");
        getline(response_stream, data);// X , Y , Z
        //STRING SPLITING
        std::istringstream iss(data);
//std::cout << "iss" << iss << std::endl;
        iss >> result.position_orientation.x;
        iss >> result.position_orientation.y;
        iss >> result.position_orientation.z;
//std::cout << "result.position_orientation.x: " << result.position_orientation.x << std::endl;
//std::cout << "result.position_orientation.y: " << result.position_orientation.y << std::endl;
//std::cout << "result.position_orientation.z: " << result.position_orientation.z << std::endl;

        boost::asio::read_until(socket_, response, "\n");
        getline(response_stream, data);// W , P , R
        //STRING SPLITING
        std::istringstream iss2(data);
        
        iss2 >> result.position_orientation.w;
        iss2 >> result.position_orientation.p;
        iss2 >> result.position_orientation.r;
//std::cout << "result.position_orientation.w: " << result.position_orientation.w << std::endl;
//std::cout << "result.position_orientation.p: " << result.position_orientation.p << std::endl;
//std::cout << "result.position_orientation.r: " << result.position_orientation.r << std::endl;
        
        boost::asio::read_until(socket_, response, "\n");
        getline(response_stream, data);// N , U , T
        //STRING SPLITING
        std::istringstream iss3(data);
        
        iss3 >> test;
//std::cout << "test1: " << test << std::endl;
        if(test=='N')
        result.config.conf1=0;
        else if (test=='F')
        result.config.conf1=1;

        iss3 >> test;
//std::cout << "test2: " << test << std::endl;
        if (test=='U')
        result.config.conf2=1;

        iss3 >> test;
//std::cout << "test3: " << test << std::endl;
        if (test=='T')
        result.config.conf3=1;

        boost::asio::read_until(socket_, response, "\n");
        getline(response_stream, data);// REDUNDÂNCIAS
//std::cout << "last data received"<< data << std::endl;

//std::cout << "get_end_efector_position::starting wait! "<< std::endl;
    //usleep(1000000); 
//std::cout << "get_end_efector_position:: ended"<< std::endl;
        
        return result;
    }

    private:

    boost::asio::io_service& io_service_;
    boost::asio::ip::tcp::socket socket_;

//?!?!?!?!?!?! don't understand this function! :/
    void handle_connect(const boost::system::error_code& error,boost::asio::ip::tcp::resolver::iterator endpoint_iterator)
    {
        if (!error)
        {
        }else if (endpoint_iterator != boost::asio::ip::tcp::resolver::iterator()){
            socket_.close();
            boost::asio::ip::tcp::endpoint endpoint = *endpoint_iterator;
            socket_.async_connect(endpoint, boost::bind(&CommunicationHandler::handle_connect, this,boost::asio::placeholders::error, ++endpoint_iterator));
        }
    }

    void setreg(int reg,pos_orient_conf position) //registar posição
    {
std::cout << "setreg function" << std::endl;
        //string where is the message
        std::ostringstream convert;        
        
        convert << "SETREG" << std::endl;
        
        convert << "1 "<<reg<< std::endl;
        //Position xyz
        convert << position.position_orientation.x 
                << " " << position.position_orientation.y 
                << " " << position.position_orientation.z 
                << std::endl;
        //Position pwr               
        convert << position.position_orientation.w 
                << " " << position.position_orientation.p 
                << " " << position.position_orientation.r 
                << std::endl;
        
        //confuguration 
        convert << IntToString(position.config.conf1)
                << " " << IntToString(position.config.conf2)
                << " " << IntToString(position.config.conf3)
                << std::endl;
        //turn! 
        convert << IntToString(position.config.conf1)
                << " " << IntToString(position.config.conf2)
                << " " << IntToString(position.config.conf3)
                << std::endl;

        std::string msg; msg=convert.str();
std::cerr << "sending this message:" << msg << std::endl;
std::cerr << "sleep...." << msg << std::endl;
        usleep(1000000); 
std::cerr << "waked up" << msg << std::endl;        
        CommunicationHandler::write(msg);

    }

    void runtpp(std::string& prog_name)
    {   
std::cout << "runtpp function" << std::endl;

        std::stringstream msg;  msg << "RUNTPP\n" << prog_name << "\n1\n";
        std::string smsg = msg.str();
std::cerr << "runtpp::sleep...."  << std::endl;
usleep(5000000); 
std::cerr << "waked up" << std::endl;
        CommunicationHandler::write(smsg);

        boost::asio::streambuf response;
        std::istream response_stream(&response);

        boost::asio::read_until(socket_, response, "\n");
        std::string data;
        getline(response_stream, data);
std::cout << "runtpp::data received: " << data << std::endl;
    } 

    void write(std::string& msg)
    {
//std::cout << "write::sending this message:" << msg << std::endl;
        do_write(msg.c_str(),msg.length());
    }

    void do_write(const char*msg,int length)
    {
//std::cout << "do_write::sending this message:" << msg << "size : " << length << std::endl;
        boost::asio::write(socket_,boost::asio::buffer(msg,length));
    }

    pos_orient posicao_garra_anterior;

};

void thread_fanuc_command()
{
    int regist = 69; // ?!?!?!?!?!?! check this value! 
    std::string prog_name = "CESAR7"; // ?!?!?!?!?!?! check this value!  
    double gama_pos = 1;    double gama_conf = 1;
std::cout << "before while" << std::endl;
    while(true)
    {
        //State machinec
      //  if(start)
       // {
std::cout << "Starting fanuc connection... " << std::endl;
            try
            {
                //Estabelecer a comunicação
                boost::asio::io_service io_service;
                boost::asio::ip::tcp::resolver resolver(io_service);
                boost::asio::ip::tcp::resolver::query query("192.168.0.230","4900");                    
                boost::asio::ip::tcp::resolver::iterator iterator = resolver.resolve(query);
                CommunicationHandler ch(io_service, iterator); //construtor
                
// put here the start position (default position)
                pos_orient_conf start_pos_conf(-420,-372,350,180,0,0,0,1,1,0,0,0);//69
                
                pos_orient_conf current_position;
                current_position = ch.get_end_efector_position();
                current_position.print(); // print all values
                
std::cout << "Starting go_to_position function... (wait first)" << std::endl;
                usleep(5000000); 
std::cout << "Starting go_to_position function" << std::endl;

                ch.go_to_position(start_pos_conf,regist,prog_name,gama_pos,gama_conf);

            }
            catch (std::exception& e)
            {
                std::cout << " !!catch an ERROR!! " <<std::endl;
                std::cerr << " Exception: " << e.what() << std::endl;
            }            
        //}
    }
std::cout << "thread_fanuc_command()" << std::endl;
}

void keyboard_interrupt(const std_msgs::String & str)
{
std::cout << "keyboard_interrupt: " << str << std::endl;    
    if(str.data == "start")
    {
std::cout << "keyboard_interrupt::Starting conection" << std::endl;  
        start = true;
    }else
    {
std::cout << "keyboard_interrupt::Unrecognized command " << std::endl;  
        help();
    }
}

int main(int argc, char** argv)
{    
    //initialize all vars
    init_vars();
    //show help to user
    help();
    
    std::cout << "Initiaizing 'fanuc' Node" << std::endl;

    ros::init (argc, argv, "fanuc");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/keyboard", 1,keyboard_interrupt);
    
    //ros::Publisher pub_state= nh_.advertise< geometry_msgs::Point  >( "/fanuc_state", 1000 );
    //CRIAR UM TOPICO QUE VAI GUARDAR OS VALORES? que quero.. 

    //thread 
    std::cout << "Initiaizing thread_fanuc_command Thread" << std::endl;
    boost::thread fanuc_command (thread_fanuc_command);
    
    
    //Spin    
    ros::spin();
    return 0;
}
//1-10-2014 I've chaged this file, adding a brief pause when geting the position of fanuc! to 100hz
