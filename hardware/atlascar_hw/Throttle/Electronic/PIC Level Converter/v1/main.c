/*
 * File:   main.c
 *
 * Created on August 16, 2010, 12:09 PM
 */

#include "main.h"

#pragma config OSC = HS //select cristal
#pragma config OSCS = OFF // Oscillator switching
#pragma config PWRT = ON // Power on reset
#pragma config BOR = ON //brown-out reset
#pragma config BORV = 27 // brown out at 2.7V
#pragma config WDT = OFF // watch dog off
#pragma config STVR = OFF // stack overflow reset
#pragma config LVP = OFF // low voltage programming
#pragma config DEBUG = OFF // background debug

#pragma code high_vector=0x08
	void interrupt_at_high_vector (void)
	{
	 _asm GOTO high_isr _endasm
	}
#pragma code

#pragma code low_vector=0x18
        void interrupt_at_low_vector(void)
        {
            _asm GOTO low_isr _endasm
        }
#pragma code


/**
 * interrupt service routine for high interrupts
 */
#pragma interrupt high_isr
void high_isr(void)
{
    //routine that happens depending the timer0 config at init()
    if(INTCONbits.T0IE && INTCONbits.T0IF)
    {
        INTCONbits.T0IF = 0;
        INTCONbits.T0IE = 0;

        counter_pwm++;

        if(counter_pwm == duty_value)
        {
            SRV_CMD = 0;
        }
        if( counter_pwm == pwm_period)
        {
            SRV_CMD = 1;
            counter_pwm = 0;
        }

        WriteTimer0(TMR0_VAL);
        INTCONbits.T0IE = 1;
    }
}

/**
 * interrupt service routine for low interrupts
 */
#pragma interrupt low_isr
void low_isr(void)
{
    //routine to check if the system must wrk in manual or automatic mode
    if(INTCONbits.INT0IE && INTCONbits.INT0IF)
    {
        INTCONbits.INT0IE = 0;
        INTCONbits.INT0IF = 0;
        //change the way that the interrupt is done.
        INTCON2bits.INTEDG0 = !INTCON2bits.INTEDG0;
        man_auto = !man_auto;//set the system in manual

        INTCONbits.INT0IE = 1;
    }
}

/**
 * 
 */
void main(void) {

    init_pic();
    T0CONbits.TMR0ON = 0;
    
//    init_pot(I2C_POT_ADDRESS & I2C_WRITE_BIT);

    delay_us(2, _XTAL_FREQ);
    StartI2C();
    WriteI2C(0b111111111);
    StartI2C();
    StopI2C();
    delay_us(20, _XTAL_FREQ);

    while (1) {
/*
        adc_throttle = read_adc(CH_THROTTLE);

        if(man_auto)
        {
            adc_new_pos = read_adc(CH_PEDAL);
        }else{
            adc_new_pos = read_adc(CH_PLC);
        }

        pot_plc = r_pot_adc * adc_throttle;

        if(pot_plc > POT_RES)
            pot_plc = POT_RES;

        pot_car = pot_plc;
        pot_servo_ctrl = pot_plc;
*/
        /********************************************************
         * to get new servo position (duty-cycle) [1-2ms]
         *******************************************************/
        if( fabs(adc_new_pos - adc_old_pos) > adc_pos_dif)
        {
            duty_value = m_adc2timer * adc_new_pos + b_adc2timer;
            adc_old_pos = adc_new_pos;
        }


        send_pot_values(I2C_POT_ADDRESS, pot_values);

        //delay_us(5, _XTAL_FREQ);
    }
}


/**
 *  function to run a sequencial writting in the I2C bus
 *
 * @param msg - msg to be sent '\0' is consider the char to stop sending msg
 * @param address - the adress of the potentiometer, bit<0> must be 1 (changed during the sending script)
 * 
 */
void send_pot_values(unsigned char address, const unsigned char* msg)
{
    unsigned char a,a1,a2,a3;
    unsigned char pot0[4];
    unsigned char pot1[4];
    unsigned char pot2[2];
    unsigned char pot3[2];


    pot0[0]=pot0[1]=pot0[2]=0;
    pot1[0]=pot1[1]=0;
    pot2[0]=pot2[1]=0;
    pot3[0]=pot3[1]=0;

    //delay_ms(10, _XTAL_FREQ);
    //while(pot0[1] != )

    //RestartI2C();
    pot0[0]=I2C_POT_ADDRESS & I2C_WRITE_BIT;
    pot0[1]=MSG_MEM_STAT & CMD_READ;
    pot0[2]='\0';
    StartI2C();
    putsI2C(pot0);
    //a=WriteI2C(I2C_POT_ADDRESS & I2C_WRITE_BIT);
    //a1=WriteI2C(MSG_MEM_STAT & CMD_READ);//MSG_MEM_WP0
    RestartI2C();
    a2=WriteI2C(I2C_POT_ADDRESS & I2C_READ_BIT);
    //while (!DataRdyI2C());
    pot0[0]=ReadI2C();
    AckI2C();
    //while (!DataRdyI2C());
    pot0[1]=ReadI2C();
    //a3=getsI2C(pot0,2);
    /* AckI2C();
    RestartI2C();
    WriteI2C(I2C_POT_ADDRESS & I2C_WRITE_BIT);
    WriteI2C(MSG_MEM_EE0 & CMD_READ);//MSG_MEM_WP1
    RestartI2C();
    WriteI2C(I2C_POT_ADDRESS & I2C_READ_BIT);
    getsI2C(pot1,2);
    AckI2C();
    RestartI2C();
    WriteI2C(I2C_POT_ADDRESS & I2C_WRITE_BIT);
    WriteI2C(MSG_MEM_WP2 & CMD_READ);//MSG_MEM_WP2
    RestartI2C();
    WriteI2C(I2C_POT_ADDRESS & I2C_READ_BIT);
    getsI2C(pot2,2);
    AckI2C();
    RestartI2C();
    WriteI2C(I2C_POT_ADDRESS & I2C_WRITE_BIT);
    WriteI2C(MSG_MEM_WP3 & CMD_READ);//MSG_MEM_WP3
    RestartI2C();
    WriteI2C(I2C_POT_ADDRESS & I2C_READ_BIT);
    getsI2C(pot3,2);*/
    NotAckI2C();
    while ( SSPCON2bits.ACKEN );
    StopI2C();

    StartI2C();
    pot1[0]=I2C_POT_ADDRESS & I2C_WRITE_BIT;
    pot1[1]=MSG_MEM_EE0 & CMD_WRITE;
    pot1[2]=0xff;
    pot1[3]='\0';
    putsI2C(pot1);
    //WriteI2C(I2C_POT_ADDRESS & I2C_WRITE_BIT);
    //WriteI2C(MSG_MEM_EE0 & CMD_WRITE);//MSG_MEM_WP0
    //WriteI2C(0x0);
    //WriteI2C(0xff);
    //AckI2C();
    //while ( SSPCON2bits.ACKEN );
    StopI2C();

    //delay_us(10, _XTAL_FREQ);
    //I2C_POT_WP = 0;
}

/**
 * function to setup the microcontroller configurations. Define TRIS, ADC, and Timers
 */
void init_pic(void)
{
    TRISA = 0b00001011;
    TRISB = 0b00000001;
    TRISC = 0b11111111;

    #ifndef _XTAL_FREQ
        #error "missing _XTAL_FREQ"
    #endif

    //define TIMER0 to generate servo control pwm
    // pwm have a period of 20ms, and can be controlled between 1 to 2ms
    // 1ms -> 0º and 2ms -> 180º
    // using a prescaler rate of 64, an interrupt will happen at each 12,8us
    INTCON2bits.TMR0IP = 1; //sets that timer0 overflow interrupt is high level
    OpenTimer0(TIMER_INT_ON & T0_8BIT & T0_SOURCE_INT & T0_EDGE_RISE & T0_PS_1_64);
    WriteTimer0(TMR0_VAL);
    T0CONbits.TMR0ON = 0;

    //for a crystal of 20Mhz a convertion time of FOSC/32 - Tad near 1,6us
    // no interrupts, value is right justified and use RA0, RA1 & RA3 as analog inputs
    // the remain A/D bits are defined as digital I/O
    OpenADC(ADC_FOSC_RC & ADC_RIGHT_JUST & ADC_3ANA_0REF,\
            ADC_CH0 & ADC_INT_OFF);

    INTCON2bits.RBIP = 0;// set that RB0 interrupt is a low level
    OpenRB0INT(PORTB_CHANGE_INT_ON & RISING_EDGE_INT);

    /****************************
     * 
     * configure I2C
     *
     ***************************/
    OpenI2C(MASTER, SLEW_ON);
    SSPADD = 9;//number of clocks to be used

    //in seconds
    t_osc = 2.0E-7;//4.0/_XTAL_FREQ; //in seconds
    t_osc_pres = 1.28E-05;//t_osc * TMR0_PRE_SCALER; //in seconds

    pwm_period = 203;//number of times necessary to enter in the timer0 to achieve 50Hz
    min_pwm_duty = 10;//(unsigned int) (float)MIN_SRV_PWM / (float)t_osc_pres;
    max_pwm_duty = 20;//(unsigned int) (float)MAX_SRV_PWM / (float)t_osc_pres;

    m_adc2timer = 0.01953125;//(max_pwm_duty - min_pwm_duty)/(float)ADC_RES;
    b_adc2timer = min_pwm_duty;

    //starting value
    duty_value = min_pwm_duty;
    adc_pos_dif = 30;


    I2C_POT_A0 = 0;//(0b000000010 & I2C_POT_ADDRESS)>>1;
    I2C_POT_A1 = 0;//(0b000000100 & I2C_POT_ADDRESS)>>2;
    I2C_POT_WP = 1;

    //msg with memory address to set value to resistors
    pot_values[0] = MSG_MEM_WP0;    // Wiper0 - PLC signal
    pot_values[2] = MSG_MEM_WP2;    // Wiper2 - Servo ctrl
    pot_values[4] = MSG_MEM_WP3;    // Wiper3 - potentiometer for the car


    r_pot_adc = ADC_RES/POT_RES;

    pot_ref_min_car = 69;//(MIN_VOLT_CENTRALINA*POT_RES)/VDD;

    counter_pwm = 0;    // wave period is 20ms
    man_auto = 0;       //  0 - system in manual, 1 - system receiving order from PLC
    SRV_CMD = 0;

    RCONbits.IPEN = 1;
    INTCON1bits.GIEH = 1;
    INTCON1bits.GIEL = 1;

}


/**
 * Setup the digital potentiometer ic via i2c serial connection
 *
 * @param address - address of the digital potentiometer ic
 */
void init_pot(unsigned char address)//function to initialize the digital potentiometer
{
    unsigned char msg[14];


    //possible to write in the digital potetiometer EEPROM
    I2C_POT_WP = 1;

    //TCON0 register, R net 1 & 0 enabled and shutdown not forced
    msg[0]= MSG_MEM_TCON0 & CMD_WRITE;
    msg[1]= 0xff;//0x8f;

    //TCON1 register, R net 2 & 3 enabled and shutdown not forced
    msg[2]= MSG_MEM_TCON1 & CMD_WRITE;
    msg[3]= 0xff;

    //Wiper 0 - volatile
    msg[4]= MSG_MEM_WP0  & CMD_WRITE;
    msg[5]= 0x0;

    //Wiper 1 - Non Volatile
    msg[6]= MSG_MEM_WP1  & CMD_WRITE;
    msg[7]= 0x81;//pot_ref_min_car;

    //Wiper 2 - volatile
    msg[8]= MSG_MEM_WP2  & CMD_WRITE;
    msg[9]= 0x0;

    //Wiper 3 - volatile
    msg[10]= MSG_MEM_WP3  & CMD_WRITE;
    msg[11]= 0x0;

    msg[12] = '\0';

    StartI2C();
    WriteI2C(address);
    putsI2C(msg);
    StopI2C();

    delay_us(10, _XTAL_FREQ);
    
    I2C_POT_WP = 0;

}

