/*
 * File:   main.h
 * Author: dgameiro
 *
 * Created on 16 de Fevereiro de 2012, 10:37
 */

#ifndef main_H
#define main_H

#define _XTAL_FREQ 20

#include <p18cxxx.h>
#include <delays.h>
#include <adc.h>
#include <timers.h>
#include <i2c.h>
#include <portb.h>
#include <math.h>

#include "auxfunc.h"
//TODO: USER CODE

/*
 * place for DEFINES
 */

/*********************************
 *      ADC
 *********************************/
#define ADC_RES 1024.0    //10 bits adc
#define CH_THROTTLE ADC_CH0
#define CH_PEDAL ADC_CH1
#define CH_PLC ADC_CH3
#define VDD 5.0         //circuit supply voltage
//when the throtle is the minimun and maximum
//the values that will be imposed to the car EPS
//must be specified by the two defines. minimum value and maximum value
#define MIN_VOLT_CENTRALINA 2.5
#define MAX_VOLT_CENTRALINA 4.7

//value read from adc, value source will depend on man_auto state
unsigned int adc_new_pos;
unsigned int adc_old_pos;
unsigned int adc_pos_dif;
unsigned int adc_throttle;



/*********************************
 *      POTENTIOMETERS
 *********************************/
#define POT_RES 128.0     //7 bits potentiometer
#define I2C_POT_ADDRESS 0b01011001
#define I2C_WRITE_BIT 0b11111110
#define I2C_READ_BIT 0b11111111
#define I2C_POT_A0 PORTBbits.RB1
#define I2C_POT_A1 PORTBbits.RB5
#define I2C_POT_WP PORTBbits.RB4


#define MSG_MEM_TCON0 0x4c
#define MSG_MEM_TCON1 0xAd
#define MSG_MEM_STAT 0x5c
#define MSG_MEM_WP0 0x2c
#define MSG_MEM_WP1 0x3c
#define MSG_MEM_WP2 0x8c
#define MSG_MEM_WP3 0x9c

#define MSG_MEM_EE0 0b10111100  //0xbd
#define MSG_MEM_EE1 0xcc
#define MSG_MEM_EE2 0xdc
#define MSG_MEM_EE3 0xec
#define MSG_MEM_EE4 0xfc


#define CMD_READ 0b11111111
#define CMD_INC 0b11110111
#define CMD_DEC 0b11111011
#define CMD_WRITE 0b11110011

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
#define SRV_CMD PORTAbits.RA2
//values of pwm control are in s
#define MIN_SRV_PWM 1.0E-3        //1ms
#define MAX_SRV_PWM 2.0E-3        //2m
#define PERIOD_SRV_PWM 20.0E-3    //20ms


unsigned int pwm_period;
unsigned int duty_value;
int b_adc2timer;
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
#define TMR0_VAL 250

float t_osc;
float t_osc_pres;
unsigned int counter_pwm;


//functions
void high_isr(void); //hight interrupt service routine
void low_isr(void); //low interrupt service routine
void init_pic(void);//function just to setup microcontroller
void init_pot(unsigned char address);//function to initialize the digital potentiometer
void send_pot_values(unsigned char address, const unsigned char* msg);//used to sent new potentiometer config
#endif
