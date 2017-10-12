/*
 * File:   header.h
 * Author: dgameiro
 *
 * Created on 24 de Novembro de 2011, 17:17
 */

#ifndef HEADER_H
#define HEADER_H

#include <htc.h>
#include "../mylibs/i2c/i2c.h"
#include "../mylibs/adc/adc.h"
#include <math.h>


/*********************************
 *      ADC
 *********************************/
#define ADC_RES 1024    //10 bits adc
#define CH_THROTTLE 0
#define CH_PEDAL 1
#define CH_REF_PLC 3
#define VDD 5.0         //circuit supply voltage
//when the throtle is the minimun and maximum
//the values that will be imposed to the car EPS
//must be specified by the two defines. minimum value and maximum value
#define MIN_VOLT_CENTRALINA 2.5
#define MAX_VOLT_CENTRALINA 4.7

//value read from adc, value source will depend on man_auto state
unsigned char adc_new_pos;
unsigned char adc_old_pos;
unsigned char adc_pos_dif;
unsigned int adc_throttle;



/*********************************
 *      POTENTIOMETERS
 *********************************/
#define POT_RES 128     //7 bits potentiometer
#define I2C_POT_ADDRESS 0b01011110
#define I2C_WRITE_BIT 0b11111110
#define I2C_READ_BIT 0b11111111
#define I2C_POT_A0 RB1
#define I2C_POT_A1 RB5
#define I2C_POT_WP RB4

#define MSG_MEM_WP0 0x00
#define MSG_MEM_NWP1 0x30
#define MSG_MEM_WP2 0x60
#define MSG_MEM_WP3 0x70


float r_pot_adc;
unsigned char man_auto;
unsigned char pot_values[8];
unsigned char pot_servo_ctrl;
unsigned char pot_plc;
unsigned char pot_car;
unsigned char pot_ref_min_car;
unsigned char max_centralina_pot;


/*********************************
 *      SERVO CTRL
 *********************************/
#define SRV_CMD RA2
//values of pwm control are in s
#define MIN_SRV_PWM 1.0E-3        //1ms
#define MAX_SRV_PWM 2.0E-3        //2m
#define PERIOD_SRV_PWM 20.0E-3    //20ms


unsigned int pwm_period;
unsigned int duty_value;
char b_adc2timer;
float m_adc2timer;
unsigned int min_pwm_duty;
unsigned int max_pwm_duty;


/*********************************
 *      TIMER0
 *********************************/
// prescaler values
// 0 - 1:2  |   4 - 1:32
// 1 - 1:4  |   5 - 1:64
// 2 - 1:8  |   6 - 1:128
// 3 - 1:16 |   7 - 1:256
#define TMR0_PRE_SCALER 5
#define TMR0_VAL 1

float t_osc;
float t_osc_pres;
unsigned int counter_pwm;




/*********************************
 *      Functions
 *********************************/

void init_pic(void);
void init_dig_pot(unsigned char adress);
void send_pot_val(unsigned char adress, unsigned char* msg, unsigned char len);

#endif
