/*
 * File:   auxfunc.c
 * Author: dgameiro
 *
 * Created on 16 de Fevereiro de 2012, 10:37
 */

#include "auxfunc.h"
//#include "main.h"

/**
 * 
 * @param x - number of micro seconds to wait
 * @param cristal - cristal frequency in MHz
 */
void delay_us(const unsigned long x, const int cristal) {
    long counter;

    counter = x*(cristal/4);
    
    while(counter)
        counter--;

}

/**
 *
 * @param ch - the channel to convert from
 * @return - the value of the conversion
 */
unsigned int read_adc(const char ch)
{

    SetChanADC(ch);
    ConvertADC();
    while (BusyADC());
    return ReadADC();

}

/**
 *
 * @param x - number of micro seconds to wait
 * @param cristal - cristal frequency in MHz
 */
void delay_ms(const unsigned int x, const int cristal)
{
    delay_us(x*1000, cristal);
}
