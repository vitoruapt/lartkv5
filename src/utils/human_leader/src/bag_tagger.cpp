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
/*! \brief Tag good/bad leader in bag files
 *
 *  this program receives input from the keyboard (arrow keys)
 *  and when this happens, a message called /timetag is created
 *  and published. this message should be subscribed by a bag
 *  recorder (together with all the messages of interest) so the 
 *  result is a bag equal to the original + a tag.
 *  this tag signals when a good leader becames a bad leader and
 *  this is used by the extract_features code together with other
 *  measurements from each target.
 */

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <tf/transform_listener.h>

#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <cstdlib>

#define KEYCODE_R 0x43 
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71
#define KEYCODE_E 0x65


class Teleop
{
public:
  Teleop();
  void keyLoop();

private:

  ros::NodeHandle nodehandle;
  ros::Publisher tag_pub_;
  ros::Subscriber bag_sub_;
  std_msgs::Header header;
  void bagCallback(const tf::tfMessage& msg);  
}; 


Teleop::Teleop()
{
  tag_pub_ = nodehandle.advertise<std_msgs::Header>("/timetag", 1);
  bag_sub_ = nodehandle.subscribe("/tf", 1000, &Teleop::bagCallback, this);
}

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "keycapture");
  Teleop teleop_turtle;

  signal(SIGINT,quit);

  ros::Rate loop_rate(10);

  while (ros::ok())
  {      
    teleop_turtle.keyLoop();
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  return(0);
}


void Teleop::keyLoop()
{
  char c;
  bool dirty=false;

  // get the console in raw mode                                                              
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  
  // Setting a new line, then end of file                         
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("---------------------------");
  puts("Use arrow keys to generate tag!");

  // get the next event from the keyboard  
  if(read(kfd, &c, 1) < 0)
  {
    perror("read():");
    exit(-1);
  }

  ROS_DEBUG("value: 0x%02X\n", c);

  switch(c)
  {
    case KEYCODE_L:
      ROS_DEBUG("LEFT");
      ROS_INFO("left");
      dirty = true;
      break;
    case KEYCODE_R:
      ROS_DEBUG("RIGHT");
      ROS_INFO("right");
      dirty = true;
      break;
    case KEYCODE_U:
      ROS_DEBUG("UP");
      ROS_INFO("up");
      dirty = true;
      break;
    case KEYCODE_D:
      ROS_DEBUG("DOWN");
      ROS_INFO("down");
      dirty = true;
      break;
    case KEYCODE_Q:
      ROS_DEBUG("NEG PHI");
      ROS_INFO("neg phi");
      dirty = true;
      break;
    case KEYCODE_E:
      ROS_DEBUG("POS PHI");
      ROS_INFO("pos phi");
      dirty = true;
      break;
  }
  
  if(dirty ==true)
  {
    ROS_INFO("time:%f",header.stamp.toSec());
    tag_pub_.publish(header);    
    dirty=false;
  }
  
  return;
}

void Teleop::bagCallback(const tf::tfMessage& msg) 
{
  header.stamp = msg.transforms[0].header.stamp;    
  return;
}

