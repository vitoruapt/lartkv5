/*
 * File:   auxfunc.h
 * Author: dgameiro
 *
 * Created on 16 de Fevereiro de 2012, 10:38
 */

#ifndef auxfunc_H
#define auxfunc_H

#include <adc.h>
//TODO: USER CODE

void delay_ms(const unsigned int x, const int cristal);
void delay_us(const unsigned long x, const int cristal);
unsigned int read_adc(const char ch);

#endif
