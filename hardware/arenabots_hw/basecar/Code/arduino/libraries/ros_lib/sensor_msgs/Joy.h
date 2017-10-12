#ifndef ros_Joy_h
#define ros_Joy_h

#include "WProgram.h"
#include "ros.h"
#include "std_msgs/Header.h"

namespace sensor_msgs
{

  class Joy : public ros::Msg
  {
    public:
      std_msgs::Header header;
      unsigned char axes_length;
      float st_axes;
      float * axes;
      unsigned char buttons_length;
      long st_buttons;
      long * buttons;

    virtual int serialize(unsigned char *outbuffer)
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset++) = axes_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( unsigned char i = 0; i < axes_length; i++){
      union {
        float real;
        unsigned long base;
      } u_axesi;
      u_axesi.real = this->axes[i];
      *(outbuffer + offset + 0) = (u_axesi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_axesi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_axesi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_axesi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->axes[i]);
      }
      *(outbuffer + offset++) = buttons_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( unsigned char i = 0; i < buttons_length; i++){
      union {
        long real;
        unsigned long base;
      } u_buttonsi;
      u_buttonsi.real = this->buttons[i];
      *(outbuffer + offset + 0) = (u_buttonsi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_buttonsi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_buttonsi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_buttonsi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->buttons[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      unsigned char axes_lengthT = *(inbuffer + offset++);
      if(axes_lengthT > axes_length)
        this->axes = (float*)realloc(this->axes, axes_lengthT * sizeof(float));
      offset += 3;
      axes_length = axes_lengthT;
      for( unsigned char i = 0; i < axes_length; i++){
      union {
        float real;
        unsigned long base;
      } u_st_axes;
      u_st_axes.base = 0;
      u_st_axes.base |= ((typeof(u_st_axes.base)) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_axes.base |= ((typeof(u_st_axes.base)) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_axes.base |= ((typeof(u_st_axes.base)) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_axes.base |= ((typeof(u_st_axes.base)) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_axes = u_st_axes.real;
      offset += sizeof(this->st_axes);
        memcpy( &(this->axes[i]), &(this->st_axes), sizeof(float));
      }
      unsigned char buttons_lengthT = *(inbuffer + offset++);
      if(buttons_lengthT > buttons_length)
        this->buttons = (long*)realloc(this->buttons, buttons_lengthT * sizeof(long));
      offset += 3;
      buttons_length = buttons_lengthT;
      for( unsigned char i = 0; i < buttons_length; i++){
      union {
        long real;
        unsigned long base;
      } u_st_buttons;
      u_st_buttons.base = 0;
      u_st_buttons.base |= ((typeof(u_st_buttons.base)) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_buttons.base |= ((typeof(u_st_buttons.base)) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_buttons.base |= ((typeof(u_st_buttons.base)) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_buttons.base |= ((typeof(u_st_buttons.base)) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_buttons = u_st_buttons.real;
      offset += sizeof(this->st_buttons);
        memcpy( &(this->buttons[i]), &(this->st_buttons), sizeof(long));
      }
     return offset;
    }

    const char * getType(){ return "sensor_msgs/Joy"; };

  };

}
#endif