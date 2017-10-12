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
/*
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of Ricardo JS Pascoal nor the name of any other
 *    contributor may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY Ricardo JS Pascoal AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL Ricardo JS Pascoal OR ANY OTHER
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/*

*/

/**
*@file laser_rotate3D.H
*@brief brief Definition of the RotateLaser class which has methods to interface between the PIC and ROS
*@author Ricardo Pascoal  
*/

#ifndef _LASER_ROTATE3D_H_
#define _LASER_ROTATE3D_H_

#define deg2rad 0.017453293 //change this to a standard ros function
#define pulse2rad 0.003067962 // convert encoder pulses to radian
#define deg2pulse 5.689 //2048/360

using namespace serialcom;

/** 
		 * @brief class implements a gateway to communicate with the PIC to rotate the LMS200
		 * 
		 */

class RotateLaser: private SerialCom
{

	protected:
		ros::NodeHandle RotateLaser_nh; // the nodehandler associated with the rotation axis

	private:
		/* http://drdobbs.com/cpp/184401518?pgno=2
http://www.boost.org/doc/libs/1_37_0/doc/html/interprocess/synchronization_mechanisms.html
don't allow the callback to access the variable while it is being used just before sending a message
*/		 
		boost::mutex want2publish_mutex;


		// data to be transfered from the callback in the read thread to the publisher
		//std_msgs::String datatopublish; // change accordingly
		sensor_msgs::JointState joint_state;

		int previous_position_request;
		int previous_speed_request;

		// serial port specfic data
		int baud_rate_;
		std::string port_name_;

		int sizeofcommand;
		char command2send[8]; //may be position or speed

		// reference frame transformation
		tf::TransformBroadcaster TransfBroadcaster;
		tf::Transform Transform2Laser;



	public: 
		// ROS members
		//ros::ServiceServer set_speed_srv_, set_position_srv_, set_portandbaud_srv_; TODO use services
		ros::Publisher RotateLaser_pub;
		ros::Subscriber RotateLaser_sub;

		//--------------
		RotateLaser(); // opens the serial port to enable comm with PIC
		~RotateLaser(); // sends the laser to the home position, stop, and closes the serial port
		bool SetPosition(int desiredposition);
		bool SetSpeed(int desiredrpm);
		bool SendCommand(); // send what is in command2send to the PIC
		void SetJointState(const sensor_msgs::JointState::ConstPtr& set_joint_state);
		boost::function<void(std::string*)> BoostRotateLaserCallback;
		void ReadLinesSerialPort(boost::function<void(std::string*)> f);
		void RotateLaserCallback(std::string* readdata); // callback to be used in the readline thread, 
		//it will publish the information collected from the PIC
		void Publish(); // change this to make it adequate for the type of message

};

/** 
		 * @brief destructor
		 * 
		 */

RotateLaser:: ~RotateLaser()
{
	stopStream(); //stop the thread, the SerialCom destructor will close it
};

/** 
		 * @brief constructor
		 * 
		 */

// RotateLaser::RotateLaser() : RotateLaser_nh("~")  // initial values for members
RotateLaser::RotateLaser() : RotateLaser_nh("~")  // initial values for members
{
	//unlock the mutex
// 	want2publish_mutex.unlock();
	
	/*default port name
	  .param Assign value from parameter server, with default. */
	RotateLaser_nh.param ("port", port_name_, std::string("/default"));
	RotateLaser_nh.param ("baud", baud_rate_, 115200);
	std::cout<<"Param port: "<<port_name_<<std::endl;

	open(port_name_.c_str(),baud_rate_); //opens the serial port for reading and writing, inherited from SerialCom

	// set initial to home position and zero speed
	previous_position_request = 0;
	previous_speed_request = 0;
	SetPosition(previous_position_request); //send to home

	ROS_INFO("sent to home");

	joint_state.header.stamp = ros::Time::now();
// 	joint_state.set_name_size(1);
	joint_state.name.resize(1);
// 	joint_state.set_position_size(1);
	joint_state.position.resize(1);
// 	joint_state.set_velocity_size(1);
	joint_state.velocity.resize(1);
	
	joint_state.name[0] =ros::names::remap("shaft");
	joint_state.position[0] = -1;
	joint_state.velocity[0] = 0;

	#if ROS_VERSION_MINIMUM(1, 8, 0)
		Transform2Laser.setOrigin(tf::Vector3(0,0, 0.0)); //nota estava Transform2Laser.setOrigin( tf::Vector3(0,-1, 0.0) )
		Transform2Laser.setRotation(tf::Quaternion(0, 0, 0));
	#else
		Transform2Laser.setOrigin(btVector3(0,0, 0.0)); //nota estava Transform2Laser.setOrigin( tf::Vector3(0,-1, 0.0) )
		Transform2Laser.setRotation(btQuaternion(0, 0, 0));
	#endif
		
// 	Transform2Laser.setOrigin(tf::Vector3(0,0, 0.0)); //nota estava Transform2Laser.setOrigin( tf::Vector3(0,-1, 0.0) )
// 	Transform2Laser.setRotation(tf::Quaternion(0, 0, 0));

	printf("remap=%s\n", (ros::names::remap("state")).c_str());
	// publish the current angular position
	RotateLaser_pub = RotateLaser_nh.advertise<sensor_msgs::JointState>(ros::names::remap("state"),10); // this advertises the values
	//sensor_msgs::JointState std_msgs::String
	// subscribe a command
	RotateLaser_sub = RotateLaser_nh.subscribe<sensor_msgs::JointState>("cmd",1,&RotateLaser::SetJointState,this);
	//name, qeue, object member, object type

	//service to set the serial port and baud rate
	//	set_portandbaud_srv_ = RotateLaser_nh.advertiseService ("port", &RotateLaser::Set, this); TODO make this possible by changing the serial port class
	//service set speed
	//set_speed_srv_ = RotateLaser_nh.advertiseService ("speed", &RotateLaser::SetSpeed, this);
	//service set position
	//set_position_srv_ = RotateLaser_nh.advertiseService ("position", &RotateLaser::SetPosition, this);


};

/** 
		 * @brief SetJointState selects setting the angular position or rate for the external LM200 joint calls SetSpeed or SetPosition
		 * 
		 * @param set_joint_state - JointState type for set speed / position
		 * 
		 */

void RotateLaser::SetJointState(const sensor_msgs::JointState::ConstPtr& set_joint_state)
{

	/* it is not possible to request speed and position simultaneously
	   check which request has changed and make that one the active
	   In case both change, give priority to speed requests.
	   */

	if (set_joint_state->velocity.size()==0)
	{		
		ROS_WARN("LASER3D VELOCITY REQUEST 0");
		int position_request = (int)set_joint_state->position[0];
		int speed_request = (int)set_joint_state->velocity[0];
		//if(position_request != previous_position_request)
		{
			SetPosition((position_request*180./3.14));
			ROS_INFO("Setting a new position");
			previous_position_request = position_request;
		}
	}
	else
	{
		int position_request = (int)set_joint_state->position[0];
		int speed_request = (int)set_joint_state->velocity[0];
		SetSpeed(speed_request);
		ROS_INFO("Setting a new speed");
		previous_speed_request = speed_request;
		previous_position_request = position_request;
	}

};

/** 
		 * @brief SetPosition set the external angular position of the laser
		 * 
		 */
bool RotateLaser::SetPosition(int desiredposition)
{
	//send a command that sets the position of the laser
	//position is in degrees
	if ((desiredposition>359) | (desiredposition<0))
	{
		ROS_WARN("Demanded position is in integer degrees range {0,...,359}.");
		return false;
	}

	if(desiredposition==0)
	{
		command2send[0] = 'h'; // go to home
		command2send[1] = '\r';
		sizeofcommand = 2;
		// sendthecommand
		return SendCommand();
	};

	// common part of the command
	command2send[0] = 'v';
	command2send[1] = '1';
	command2send[2] = 'p'; //position

	desiredposition = round(desiredposition*deg2pulse); // PIC is expecting pulse count

	//itoa(desiredposition,command2send[3],10); //this is a nonstandard function
	sprintf(command2send+3,"%d",desiredposition);
	sizeofcommand = strlen(command2send);
	//ROS_INFO("position command received : %s", command2send);
	command2send[sizeofcommand] = '\r';
	sizeofcommand+=1;

	return SendCommand();
}


/** 
		 * @brief SetSpeed set the external angular velocity of the laser
		 * 
		 */
bool RotateLaser::SetSpeed(int desiredrpm)
{
	//send a command that sets the angular rate of the laser RPM.
	if ((desiredrpm>19) & (desiredrpm<100))
	{
		command2send[0] = 'v';
		command2send[1] = '1';
		command2send[2] = 's';
		command2send[3] = (desiredrpm - desiredrpm%10)/10 + 48;
		command2send[4] = desiredrpm%10 + 48; // 48 is the ascii value for 0
		command2send[5] = '\r';
		sizeofcommand = 6;
		return (SendCommand());
	}
	else
	{
		ROS_WARN("Demanded speed is in integer rpm range {20,...,99}.");
		return false;
	};

}

/** 
		 * @brief SendCommand send a command to the PIC controlling the stepper motor
		 * 
		 */
bool RotateLaser::SendCommand()
{

	if(write(command2send,sizeofcommand)<sizeofcommand)
	{
		ROS_WARN ("Error writing the command!");
		return false;
	}
	else
	{	
		return true;
	}
};

/** 
		 * @brief ReadLinesSerialPort thread to read from the serial port, inherited from SerialCom
		 * 
		 */
void RotateLaser::ReadLinesSerialPort(boost::function<void(std::string*)> f)
{
	startReadLineStream(f); //start a stream (thread) for reading from the serial port, inherited from SerialCom
};

/** 
		 * @brief Publish mutex protected publish
		 * 
		 */
void RotateLaser::Publish() //publish data safely
{	

	if(want2publish_mutex.try_lock())
	{
		// also lock in the callback for reading the serial port
		// it is safe to do whatever is needed with msg
		//ROS_INFO("safe publishing %s \n", "main thread");//readdata->c_str);
		if(joint_state.position[0]>=0) //it is initialized with -1
		{
			// publish the position as a change in a joint_state
			RotateLaser_pub.publish(joint_state);
			// publish the position as a change in reference frame

		#if ROS_VERSION_MINIMUM(1, 8, 0)
			Transform2Laser.setOrigin( tf::Vector3(0,0.0, 0.0) ); //nota estava Transform2Laser.setOrigin( tf::Vector3(0,-1, 0.0) )
		#else
			Transform2Laser.setOrigin(btVector3(0.0,0.0, 0.0)); //nota estava Transform2Laser.setOrigin( tf::Vector3(0,-1, 0.0) )
		#endif
			
// 			Transform2Laser.setOrigin( tf::Vector3(1,0, 0.0) ); //nota estava Transform2Laser.setOrigin( tf::Vector3(0,-1, 0.0) )
			
			Transform2Laser.setRotation( tf::createQuaternionFromRPY(joint_state.position[0], 0, 0) );
// 			TransfBroadcaster.sendTransform(tf::StampedTransform(Transform2Laser, joint_state.header.stamp, "/atc/laser/roof_rotating", "/atc/laser/roof_rotating/base"));
			TransfBroadcaster.sendTransform(tf::StampedTransform(Transform2Laser, joint_state.header.stamp, "/atc/laser/roof_rotating/base", "/atc/laser/roof_rotating"));
			
			want2publish_mutex.unlock();
// 			ROS_INFO("publishing %s\n","");
		}
		else
		{
			ROS_INFO("no data available to publish %s\n","");
			want2publish_mutex.unlock();
		}
	}
};


void RotateLaser::RotateLaserCallback(std::string* readdata)
	// this function is called back once the readline thread is invoked
{
	int thesize;
	int aux;
	char Pos[1024];
	char Vel[1024];
	thesize = readdata->size();
	
	// for now print on screen what has been read

	//ROS_INFO("In the rotatelaser callback");

	 // if it is not locked then
	// have to pass the information in order to safely allow publishing
	// copy the message to a new variable and unlock
	//	ROS_INFO("in callback string size %s \n", readdata->c_str());//readdata->c_str);

	joint_state.header.stamp = ros::Time::now();
	
// 		joint_state.position[0] = atof(readdata->c_str())*pulse2rad;
	//ROS_INFO("value read %f :", joint_state.position[0]);
	/*std::stringstream ss;
		ss << "collected in the callback";
		datatopublish.data = ss.str();*/
	
	//get the posistion in the string P posistion V velocity
	aux=sscanf(readdata->c_str(),"%*c %s %*c %s\n",Pos,Vel);
// 		ROS_INFO("value read: %s , atof: %f, converted: &f\n", Pos,atof(Pos),atof(Pos)*pulse2rad);
// 		std::cout<<"value read: "<<Pos<<" atof: "<<atof(Pos)<<" converted: "<<atof(Pos)*pulse2rad<<std::endl;
	
	want2publish_mutex.lock();
	
	joint_state.position[0] = atof(Pos)*pulse2rad;
	joint_state.velocity[0] = atof(Vel);
	want2publish_mutex.unlock();
	usleep(10);

	Publish(); //new change 2013 - 11 april
};


#endif //EOF
