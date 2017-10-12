#ifndef ros_JoyFeedbackArray_h
#define ros_JoyFeedbackArray_h

#include "WProgram.h"
#include "ros.h"
#include "sensor_msgs/JoyFeedback.h"

namespace sensor_msgs
{

  class JoyFeedbackArray : public ros::Msg
  {
    public:
      unsigned char array_length;
      sensor_msgs::JoyFeedback st_array;
      sensor_msgs::JoyFeedback * array;

    virtual int serialize(unsigned char *outbuffer)
    {
      int offset = 0;
      *(outbuffer + offset++) = array_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( unsigned char i = 0; i < array_length; i++){
      offset += this->array[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      unsigned char array_lengthT = *(inbuffer + offset++);
      if(array_lengthT > array_length)
        this->array = (sensor_msgs::JoyFeedback*)realloc(this->array, array_lengthT * sizeof(sensor_msgs::JoyFeedback));
      offset += 3;
      array_length = array_lengthT;
      for( unsigned char i = 0; i < array_length; i++){
      offset += this->st_array.deserialize(inbuffer + offset);
        memcpy( &(this->array[i]), &(this->st_array), sizeof(sensor_msgs::JoyFeedback));
      }
     return offset;
    }

    const char * getType(){ return "sensor_msgs/JoyFeedbackArray"; };

  };

}
#endif