#ifndef _FUNC_H_
#define _FUNC_H_

/**
 *       @file  func.h
 *      @brief  
 *     @author  LAR
 *
 *   @internal
 *     Created  Janeiro 2012
 *    Revision  ---
 *    Compiler  C18
 *     Company  University of Aveiro
 *   Copyright  Copyright (c) 2012, LAR
 *
 * =====================================================================================
 */
 
#include <p18f2431.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

 
/*####################################################################*/
//Funções referentes ao ficheiro func.c

void PICinit(void);
void RS232(void);
void PWM_off(void);
void PWM_on(void);
void wait(unsigned int value);
void send(int count,char type);
void SendArray(char array[],int dim);
void SendPointer(rom near char *nline);
void SpeedRamp(int Pres,int OldPres);
int Array2Int(char cont[], int e);
short int HomePos(void);


#endif
