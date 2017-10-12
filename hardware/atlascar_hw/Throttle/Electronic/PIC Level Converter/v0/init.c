/*
 * File:   init.c
 *
 * Created on August 16, 2010, 12:09 PM
 */

#include "header.h"

void init_pic(void)
{
    TRISA = 0b00000111;
    TRISB = 0b00000001;
    TRISC = 0b11111111;

    #ifndef _XTAL_FREQ
        #error "missing _XTAL_FREQ"
    #endif

    //in soconds
    t_osc = 4.0/_XTAL_FREQ;
    t_osc_pres = t_osc * TMR0_PRE_SCALER;


    pwm_period = PERIOD_SRV_PWM*t_osc_pres;
    min_pwm_duty = (unsigned int) MIN_SRV_PWM * t_osc_pres;
    max_pwm_duty = (unsigned int) MAX_SRV_PWM * t_osc_pres;

    m_adc2timer = (max_pwm_duty - min_pwm_duty)/ADC_RES;
    b_adc2timer = min_pwm_duty;

    //starting value
    duty_value = min_pwm_duty;

    //for a crystal of 20Mhz a convertion time of FOSC/32 - Tad near 1,6us
    // no interrupts, value is right justified and use RA0, RA1 & RA3 as analog inputs
    // the remain A/D bits are defined as digital I/O
    setup_adc(ADC_FOSC_32 & ADC_INT_OFF & ADC_RIGHT_JUST & ADC_3ANA_0REF);



    I2C_POT_A0 = (0b000000010 & I2C_POT_ADDRESS)>>1;
    I2C_POT_A1 = (0b000000100 & I2C_POT_ADDRESS)>>2;
    
    //msg with memory address to set value to resistors
    pot_values[0] = MSG_MEM_WP0;    // Wiper0 - PLC signal
    pot_values[2] = MSG_MEM_WP2;    // Wiper2 - Servo ctrl
    pot_values[4] = MSG_MEM_WP3;    // Wiper3 - potentiometer for the car

    //not possible to write in the digital potetiometer EEPROM
    I2C_POT_WP = 0;

    r_pot_adc = ADC_RES/POT_RES;

    pot_ref_min_car = (MIN_VOLT_CENTRALINA*POT_RES)/VDD;

    counter_pwm = 0;    // wave period is 20ms
    man_auto = 0;       //  0 - system in manual, 1 - system receiving order from PLC


    //define TIMER0 to generate servo control pwm
    // pwm have a period of 20ms, and can be controlled between 1 to 2ms
    // 1ms -> 0º and 2ms -> 180º
    // using a prescaler rate of 64, an interrupt will happen at each 12,8us
    OPTION_REG = 0b01110100;
    TMR0 = TMR0_VAL;

    //activate GIE PEIE T0IE INTE 
    INTCON = 0b11110000;
}



/**
 * @fn void init_dig_pot(unsigned char adress)
 * @param adress - potentiometer MCP4441 address + the write bit
 */
void init_dig_pot(unsigned char adress)
{
    unsigned char len;
    unsigned char msg[14];

    SSPMode(MASTER_MODE);

    //initialize the i2c comunication
    i2c_Start();

    i2c_WriteTo(adress);

    //TCON0 register, R net 1 disabled | R net 1 enabled and shutdown not forced
    msg[0]= 0x41;
    msg[1]= 0x8f;

    //TCON1 register, R net 2 & 3 enabled and shutdown not forced
    msg[2]= 0xa1;
    msg[3]= 0xff;

    //Wiper 0 - volatile
    msg[4]= MSG_MEM_WP0;
    msg[5]= 0x0;

    //Wiper 1 - Non Volatile
    msg[6]= MSG_MEM_NWP1;
    msg[7]= pot_ref_min_car;

    //Wiper 2 - volatile
    msg[8]= MSG_MEM_WP2;
    msg[9]= 0x0;

    //Wiper 3 - volatile
    msg[10]= MSG_MEM_WP3;
    msg[11]= 0x0;


    len = 12;
    i2c_PutString(msg[0], len);

    i2c_Stop();
    
}