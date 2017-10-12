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
/* Auto-generated by genmsg_cpp for file /home/prometeu/ros/inria_wheelchair/social_filter/msg/humanPose.msg */
#ifndef SOCIAL_FILTER_MESSAGE_HUMANPOSE_H
#define SOCIAL_FILTER_MESSAGE_HUMANPOSE_H
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/time.h"

#include "ros/macros.h"

#include "ros/assert.h"

#include "std_msgs/Header.h"

namespace social_filter
{
template <class ContainerAllocator>
struct humanPose_ {
  typedef humanPose_<ContainerAllocator> Type;

  humanPose_()
  : header()
  , id(0)
  , x(0.0)
  , y(0.0)
  , theta(0.0)
  , linear_velocity(0.0)
  , angular_velocity(0.0)
  {
  }

  humanPose_(const ContainerAllocator& _alloc)
  : header(_alloc)
  , id(0)
  , x(0.0)
  , y(0.0)
  , theta(0.0)
  , linear_velocity(0.0)
  , angular_velocity(0.0)
  {
  }

  typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
   ::std_msgs::Header_<ContainerAllocator>  header;

  typedef int8_t _id_type;
  int8_t id;

  typedef float _x_type;
  float x;

  typedef float _y_type;
  float y;

  typedef float _theta_type;
  float theta;

  typedef float _linear_velocity_type;
  float linear_velocity;

  typedef float _angular_velocity_type;
  float angular_velocity;


private:
  static const char* __s_getDataType_() { return "social_filter/humanPose"; }
public:
  ROS_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROS_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "ceacb455de3799af175bff49a64446eb"; }
public:
  ROS_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROS_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "Header header\n\
\n\
int8 id\n\
float32 x\n\
float32 y\n\
float32 theta\n\
float32 linear_velocity\n\
float32 angular_velocity\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.secs: seconds (stamp_secs) since epoch\n\
# * stamp.nsecs: nanoseconds since stamp_secs\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
"; }
public:
  ROS_DEPRECATED static const std::string __s_getMessageDefinition() { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED const std::string __getMessageDefinition() const { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const
  {
    ros::serialization::OStream stream(write_ptr, 1000000000);
    ros::serialization::serialize(stream, header);
    ros::serialization::serialize(stream, id);
    ros::serialization::serialize(stream, x);
    ros::serialization::serialize(stream, y);
    ros::serialization::serialize(stream, theta);
    ros::serialization::serialize(stream, linear_velocity);
    ros::serialization::serialize(stream, angular_velocity);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    ros::serialization::IStream stream(read_ptr, 1000000000);
    ros::serialization::deserialize(stream, header);
    ros::serialization::deserialize(stream, id);
    ros::serialization::deserialize(stream, x);
    ros::serialization::deserialize(stream, y);
    ros::serialization::deserialize(stream, theta);
    ros::serialization::deserialize(stream, linear_velocity);
    ros::serialization::deserialize(stream, angular_velocity);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint32_t serializationLength() const
  {
    uint32_t size = 0;
    size += ros::serialization::serializationLength(header);
    size += ros::serialization::serializationLength(id);
    size += ros::serialization::serializationLength(x);
    size += ros::serialization::serializationLength(y);
    size += ros::serialization::serializationLength(theta);
    size += ros::serialization::serializationLength(linear_velocity);
    size += ros::serialization::serializationLength(angular_velocity);
    return size;
  }

  typedef boost::shared_ptr< ::social_filter::humanPose_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::social_filter::humanPose_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct humanPose
typedef  ::social_filter::humanPose_<std::allocator<void> > humanPose;

typedef boost::shared_ptr< ::social_filter::humanPose> humanPosePtr;
typedef boost::shared_ptr< ::social_filter::humanPose const> humanPoseConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::social_filter::humanPose_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::social_filter::humanPose_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace social_filter

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::social_filter::humanPose_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::social_filter::humanPose_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::social_filter::humanPose_<ContainerAllocator> > {
  static const char* value() 
  {
    return "ceacb455de3799af175bff49a64446eb";
  }

  static const char* value(const  ::social_filter::humanPose_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xceacb455de3799afULL;
  static const uint64_t static_value2 = 0x175bff49a64446ebULL;
};

template<class ContainerAllocator>
struct DataType< ::social_filter::humanPose_<ContainerAllocator> > {
  static const char* value() 
  {
    return "social_filter/humanPose";
  }

  static const char* value(const  ::social_filter::humanPose_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::social_filter::humanPose_<ContainerAllocator> > {
  static const char* value() 
  {
    return "Header header\n\
\n\
int8 id\n\
float32 x\n\
float32 y\n\
float32 theta\n\
float32 linear_velocity\n\
float32 angular_velocity\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.secs: seconds (stamp_secs) since epoch\n\
# * stamp.nsecs: nanoseconds since stamp_secs\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
";
  }

  static const char* value(const  ::social_filter::humanPose_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct HasHeader< ::social_filter::humanPose_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct HasHeader< const ::social_filter::humanPose_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::social_filter::humanPose_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.header);
    stream.next(m.id);
    stream.next(m.x);
    stream.next(m.y);
    stream.next(m.theta);
    stream.next(m.linear_velocity);
    stream.next(m.angular_velocity);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct humanPose_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::social_filter::humanPose_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::social_filter::humanPose_<ContainerAllocator> & v) 
  {
    s << indent << "header: ";
s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "id: ";
    Printer<int8_t>::stream(s, indent + "  ", v.id);
    s << indent << "x: ";
    Printer<float>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<float>::stream(s, indent + "  ", v.y);
    s << indent << "theta: ";
    Printer<float>::stream(s, indent + "  ", v.theta);
    s << indent << "linear_velocity: ";
    Printer<float>::stream(s, indent + "  ", v.linear_velocity);
    s << indent << "angular_velocity: ";
    Printer<float>::stream(s, indent + "  ", v.angular_velocity);
  }
};


} // namespace message_operations
} // namespace ros

#endif // SOCIAL_FILTER_MESSAGE_HUMANPOSE_H

