#ifndef ros_JoyFeedback_h
#define ros_JoyFeedback_h

#include "WProgram.h"
#include "ros.h"

namespace sensor_msgs
{

  class JoyFeedback : public ros::Msg
  {
    public:
      unsigned char type;
      unsigned char id;
      float intensity;
      enum { TYPE_LED = 0 };
      enum { TYPE_RUMBLE = 1 };
      enum { TYPE_BUZZER = 2 };

    virtual int serialize(unsigned char *outbuffer)
    {
      int offset = 0;
      union {
        unsigned char real;
        unsigned char base;
      } u_type;
      u_type.real = this->type;
      *(outbuffer + offset + 0) = (u_type.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->type);
      union {
        unsigned char real;
        unsigned char base;
      } u_id;
      u_id.real = this->id;
      *(outbuffer + offset + 0) = (u_id.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->id);
      union {
        float real;
        unsigned long base;
      } u_intensity;
      u_intensity.real = this->intensity;
      *(outbuffer + offset + 0) = (u_intensity.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_intensity.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_intensity.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_intensity.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->intensity);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        unsigned char real;
        unsigned char base;
      } u_type;
      u_type.base = 0;
      u_type.base |= ((typeof(u_type.base)) (*(inbuffer + offset + 0))) << (8 * 0);
      this->type = u_type.real;
      offset += sizeof(this->type);
      union {
        unsigned char real;
        unsigned char base;
      } u_id;
      u_id.base = 0;
      u_id.base |= ((typeof(u_id.base)) (*(inbuffer + offset + 0))) << (8 * 0);
      this->id = u_id.real;
      offset += sizeof(this->id);
      union {
        float real;
        unsigned long base;
      } u_intensity;
      u_intensity.base = 0;
      u_intensity.base |= ((typeof(u_intensity.base)) (*(inbuffer + offset + 0))) << (8 * 0);
      u_intensity.base |= ((typeof(u_intensity.base)) (*(inbuffer + offset + 1))) << (8 * 1);
      u_intensity.base |= ((typeof(u_intensity.base)) (*(inbuffer + offset + 2))) << (8 * 2);
      u_intensity.base |= ((typeof(u_intensity.base)) (*(inbuffer + offset + 3))) << (8 * 3);
      this->intensity = u_intensity.real;
      offset += sizeof(this->intensity);
     return offset;
    }

    const char * getType(){ return "sensor_msgs/JoyFeedback"; };

  };

}
#endif