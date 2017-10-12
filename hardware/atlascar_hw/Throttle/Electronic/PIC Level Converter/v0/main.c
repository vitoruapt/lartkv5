/*
 * File:   main.c
 *
 * Created on August 16, 2010, 12:09 PM
 */

#define _XTAL_FREQ 20000000

#include "header.h"

__CONFIG(WDTE_OFF & LVP_OFF & FOSC_HS);

void interrupt int_call(void)
{

    //routine that happens depending the timer0 config at init()
    if(T0IE && T0IF)
    {
        T0IF = 0;
        T0IE = 0;

        counter_pwm++;

        if(counter_pwm > duty_value)
        {
            SRV_CMD = 0;
        }else if( counter_pwm > pwm_period)
        {
            SRV_CMD = 1;
            counter_pwm = 0;
        }

        T0IE = 1;
    }

    //routine to check if the system must wrk in manual or automatic mode
    if(INTE && INTF)
    {
        INTE = 0;
        INTF = 0;
        //change the way that the interrupt is done.
        INTEDG = !INTEDG;
        man_auto = !man_auto;//set the system in manual

        INTE = 1;
    }

}


int main(void) {

    init_pic();

    init_dig_pot(I2C_POT_ADDRESS & I2C_WRITE_BIT);

    /**
     * main cycle
     */


    while(1)
    {
        adc_throttle = read_adc(CH_THROTTLE);

        if(man_auto)
        {
            adc_new_pos = read_adc(CH_PEDAL);
        }else{
            adc_new_pos = read_adc(CH_REF_PLC);
        }

        //value from the throtle to be sent to PLC
        pot_plc = r_pot_adc *adc_throttle;
        if(pot_plc > POT_RES)
            pot_plc = POT_RES;

        pot_car = pot_plc;
        pot_servo_ctrl = pot_plc;

        pot_values[1] = pot_plc;
        pot_values[3] = pot_servo_ctrl;
        pot_values[5] = pot_car;

        send_pot_val(I2C_POT_ADDRESS & I2C_WRITE_BIT, &pot_car, 6);

        if( fabs(adc_new_pos - adc_old_pos) < adc_pos_dif)
        {
            duty_value = m_adc2timer * adc_new_pos + b_adc2timer;
        }

    };

    return 0;
}



void send_pot_val(unsigned char adress, unsigned char* msg, unsigned char len)
{
    i2c_Start();

    i2c_WriteTo(adress);

    i2c_PutString(msg, len);

    i2c_Stop();
}