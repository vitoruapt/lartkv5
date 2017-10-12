#ifndef _VAR_H_
#define _VAR_H_

/**
 *       @file  var2.h
 *      @brief  PIC code laser 3D
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

 void high_isr(void);
 void low_isr(void);

char temp[12], str2[12]; // adjust this later for the maximum message size
unsigned char RX, i,j,w,t,ss,vv,pp,start;

int invalid_msg;
int Pos_stop,rs;
signed int deg1;
unsigned int T2Pres,NewSpeed,deg;
unsigned int speed,m,n,p,q;
unsigned short int cw;
unsigned short int portRB3, hp, pos_ref,first_time;
unsigned short int instr_p,instr_s,instr_v, instr_h,instr_m,instr_d;
unsigned long int Presc2,freq,k,vel,pic_vel,k1;
unsigned long int Speed,Pos_int;

/*####################################################################*/
//Definição das mensagens a enviar

// rom = program memory ; near ou far interessa para o compilador
// as strings devem ser armazenadas na memória do programa - rom, 
// pois as funções "stdio.h" assumem que será essa a sua localização

rom near char *initstr="\n \r PIC_initialization\n";
rom near char *resetstr="\n \r PIC_reset (MCLR)\n \r";
rom near char *home="\n \r Home\n \r";
rom near char *Msg_received="\n \r Msg_received\n \r";
rom near char *Msg_corrupted="\n \r Msg_corrupt\n \r";
rom near char *Invalid_speed="\n \r Invalid_requested_speed\n \r";
rom near char *Stopped="\n \r Laser_already_stopped\n \r";
rom near char *Invalid_rot="\n \r Laser_rotating_in_that_direction\n \r";
rom near char *Executed_msg="\n \r Message_executed\n \r";
rom near char *Not_responding="\n \r Motor_is_not_responding\n \r";
rom near char *Wrong_pos="\n \r Incorrect positions,transmission aborted\n \r";
rom near char *Invalid_pos="\n \r Invalid_requestedposition\n \r";
rom near char *Lost_pulse="\n \r MCU_skipped_pulses\n \r";
rom near char *Gain_pulse="\n \r MCU_overcounted\n \r";
rom near char *INT_int="\n \r Gerei_interrupt_no_INT1\n \r";

/*####################################################################*/

#endif
