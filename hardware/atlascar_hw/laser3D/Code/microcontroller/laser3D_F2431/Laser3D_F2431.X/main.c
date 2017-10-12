/**
 *       @file  main.c
 *      @brief  PIC code main
 *     @author  LAR
 *
 *   @internal
 *     Created  JAN 2012
 *    Revision  ---
 *    Compiler  Microchip c18
 *     Company  University of Aveiro
 *   Copyright  Copyright (c) 2012, LAR
 *
 * =====================================================================================
 */

#include <p18f2431.h>
//#include <p18f2331.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <delays.h>
#include "func.h"
#include "var.h"

#pragma config OSC=HS
#pragma config FCMEN = OFF
#pragma config WDTEN=OFF
#pragma config IESO = OFF
#pragma config LVP = OFF
#pragma config T1OSCMX = OFF
#pragma config STVREN = OFF
 			
/*####################################################################*/
/**
 * @fn void interrupt_at_high_vector (void)
 * @brief Função para definir interrupts de alta prioridade
 * 
 * Define os todos os interrupts do pic que sejam definidos
 * com 'high priority'. 
 *
 * @param não possui inputs
 * @return não possui outputs
 */	

#pragma code high_vector=0x08
	void interrupt_at_high_vector (void)		//interrupts de alta prioridade
	{
		_asm GOTO high_isr _endasm				//..."ir para" rotina high isr
	}
#pragma code

/*####################################################################*/
/**
 * @fn void interrupt_at_low_vector(void)
 * @brief Função para definir interrupts de baixa prioridade
 * 
 * Define os todos os interrupts do pic que sejam definidos
 * com 'low priority'. 
 *
 * @param não possui inputs
 * @return não possui outputs
 */
#pragma code low_vector=0x18
	void interrupt_at_low_vector(void) 			//interrupts de baixa prioridade
	{
		_asm GOTO low_isr _endasm				//"ir para" rotina low_isr 
	}
#pragma code


/*####################################################################*/
/**
 * @fn void high_isr (void)
 * @brief Função para interrupts de alta prioridade
 * 
 * Define a rotina do interrupt IC2QEIF (quando o encoder passa na home position).\n
 * Define a recepção de dados, RCIF (comentada pois não foi possível testá-la).
 *
 * @param não possui inputs
 * @return não possui outputs
 */

#pragma interrupt high_isr
	void high_isr (void)						//rotina para interrupts de baixa prioridade
	{
            // give highest priority to a message being received
           // unsigned char i; needs to be a global var
            unsigned char pos_h, pos_l, vel_h, vel_l;
            unsigned int pos, vel,vel1;
            unsigned long int pr2aux,vel2;


           /* if (PIR3bits.IC1IF) // capture interrupt flag
            {
                vel_h = VELRH;
                vel_l = VELRL;
                vel = VELRH*256 + VELRL;
                printf((rom far char*) "%uV\n", vel);

     //           Delay10KTCYx(50);
     //           PORTBbits.RB2=0;

               PIR3bits.IC1IF = 0;
            }*/

            if	(PIR1bits.RCIF==1) 	//data received through RS232
		{
			//disable the other interrupts
			PIE3bits.IC2QEIE=0;				//QEI Interrupt Flag Enable bit
			PIR3bits.IC2QEIF=0;	//clear the interrupt
                        INTCONbits.TMR0IE=0;  				// do not send any messages while reading and processing a new message
                        INTCONbits.TMR0IF=0;

                        
                        //
			i=0;
			while ((RCREG!='\r') && (i<9)) {				//"enquanto" não chegar o caracter indicativo de fim da mensagem...
				str2[i]=RCREG;					//os caracteres são armazenados no array str2
				while(PIR1bits.RCIF==0);  		//esperar pelo interrupt gerado pelo próximo caracter
				i++;
			}
                        if(i<8) // check according to max possible message size
                        {
                         //PIR1bits.RCIE = 0;
                         PIR1bits.RCIF = 0;
                         RCSTAbits.CREN=0; // disable continuous reception to make sure that the old message has been processed
			 RX=1;	 //complete message has arrived
			 str2[i]='\r'; //sinalização do fim do array
                        }
                       // renable of the interrupts is done in the main loop after the message has been interpreted
		}


            if (PIR3bits.IC2QEIF)
            {
                //interrupt gerado pelo CANAL 0 (codificador) INDEX: this is the physical home
		//Quando o encoder integrado no laser volta ao início (home position)
		//enviar a posição do encoder juntamente com uma letra 'H'
		//para se saber quando passa no home do laser
                if(!pos_ref)
                {
                    pos_ref=1;
                }

                if(hp)
                {
                    PWM_off();
                    SendPointer(home);
                    hp=0;
                }
               
                pos_h = POSCNTH;
                pos_l = POSCNTL;
                pos = (pos_h*256 +pos_l); // pay attention that position may be given in several precision modes
                                       // we are using the 4x and therefore 4 pulses for each enconder nominal pulse
                                       // that means it is counting all edges.
                //send(pos,'NewLine');
                //printf((rom far char*) "P%uP\0", pos);
                // ---------- led test ------
      //          Delay10KTCYx(50);
      //          PORTBbits.RB2=1;
                // ----------------------------
                //if(!INTCONbits.TMR0IE) INTCONbits.TMR0IE=1; // in case it wasn't reactivated in the receive

                PIR3bits.IC2QEIF=0;	//clear the interrupt
            }
	}
        
/*####################################################################*/
/**
 * @fn void low_isr (void)
 * @brief Função para interrupts de baixa prioridade
 * 
 * Define a rotina do interrupt TMR0IF (estouro do timer 0, TMR0).
 * Define a rotina do interrupt INT1 (provocado pelo sinal de sincronismo)
 * @param não possui inputs
 * @return não possui outputs
 */

#pragma interruptlow low_isr
	void low_isr (void)							//rotina para interrupts de baixa prioridade
	{    
                unsigned char pos_h, pos_l, vel_h, vel_l;
                unsigned int pos, vel,vel1;
                unsigned long int pr2aux,vel2;

                //interrupt no INT1

            /*
            if (INTCON3bits.INT1IF==1)
            {
                INTCON3bits.INT1IF=0;

                pos_h = POSCNTH;
                pos_l = POSCNTL;
                pos = (pos_h*256 + pos_l); // pay attention that position may be given in several precision modes
                                                   // we are using the 2x and therefore 2 pulses for each enconder nominal pulse



                if(QEICONbits.ERROR)
                {
                    QEICONbits.ERROR = 0;
                }
                else
                {
                    if(PIE3bits.IC2QEIE) PIE3bits.IC2QEIE = 0;
                    //printf((rom far char*) "P%uP\0", pos);
                    PIE3bits.IC2QEIE = 1;
                 }

                if(fabs(Pos_stop - pos)<3)
                {
                      PWM_off();
                      Pos_stop = 4000;
                }

                pr2aux=PR2;	//conversão de periodo em velocidade
                pr2aux*=853;
                vel2=5E6;
                vel2/=pr2aux;

                if (vel2>5000)
                {
                    vel1=0;
                }
                else
                {
                    vel1=vel2;
                }
                if (rs==1 && first_time==1)
                {
                    printf((rom far char*) "P %u V %u\n", pos,vel1);
                }

                //SendPointer(INT_int);
            }
            */


                
       		if (INTCONbits.TMR0IF)			//interrupt gerado pelo TIMER 0
                {
                        TMR0L=0x05;
                        TMR0H=0xFC;
                 
                        if (pos_ref)
                        {
                            pos_h = POSCNTH;
                            pos_l = POSCNTL;
                            pos = (pos_h*256 + pos_l); // pay attention that position may be given in several precision modes
                                                   // we are using the 2x and therefore 2 pulses for each enconder nominal pulse
                                                   
                            //send(pos,'NewLine');
                            if(QEICONbits.ERROR)
                            {
                                QEICONbits.ERROR = 0;
                            }
                            else
                            {
                                if(PIE3bits.IC2QEIE) PIE3bits.IC2QEIE = 0;
                               //printf((rom far char*) "P%uP\0", pos);
                               PIE3bits.IC2QEIE = 1;
                             }

                            if(fabs(Pos_stop - pos)<3)
                            {
                              PWM_off();
                              Pos_stop = 4000;
                            }
                            //vel_h = VELRH;
                            //vel_l = VELRL;
                            //vel = VELRH*256 + VELRL;
                            pr2aux=PR2;	//conversão de periodo em velocidade
                            pr2aux*=853;
                            vel2=5E6;
                            vel2/=pr2aux;

                            if (vel2>5000)
                            {
                                vel1=0;
                            }
                            else
                            {
                                vel1=vel2;
                            }
                            if (rs==1)
                            {
                                //printf((rom far char*) "P %u V %u\n", pos,vel1);
                                printf((rom far char*) "P %u V %u Z 0\n", pos,vel1);
                            }

                      

                          // ---------- led test ------
                            /*Delay10KTCYx(50);
                            PORTBbits.RB2=0;
                            Delay10KTCYx(50);
                            PORTBbits.RB2=1;*/
                        // ----------------------------
                
                        }
                        INTCONbits.TMR0IF=0;
             
		}
                 
                
	} 	


/*####################################################################*/
/**
 * @fn void main(void) 
 * @brief Função main do PIC.
 * 
 * Define as configurações de registos do pic e configura a 
 * comunicação RS232.\n
 * Activa o PWM do motor de passo.
 * Lê a mensagem vinda do computador (pedido de velocidade)
 * (comentada pois não foi possível testá-la).
 *
 * @param não possui inputs
 * @return não possui outputs
 */
void main(void) 
{								//rotina principal do programa
    
    PICinit();		//initialize some registers
    deg1=0;										//initialize some variables
    Pos_stop=4000;
    start = 0;
    pos_ref=0;
    vel=0;
    rs=1; //começar com tranmição de dados
    k=0;
    k1=0;
    ss=0;
    vv=0;
    pp=0;
    RX=0;
    t=0;
    i=0;
    first_time=0;
    
    invalid_msg=0;

    if (start==1)
    {								//se o PIC já foi iniciado...
    PICinit();
    SendPointer(resetstr);					//...então "fez" reset
    }
    else
    {					//se o PIC ainda não foi iniciado...
    start=1;
    SendPointer(initstr);					//...então foi iniciado
    }

    hp=HomePos();
    if (hp) first_time=1;

    // loop forever
    while(1)
    {
        Delay10KTCYx(50);
        PORTBbits.RB2=0;
        Delay10KTCYx(50);
        PORTBbits.RB2=1;

                /*
    		if(RX==1)  // a new message has been received
		{
                    j=0;
                    instr_h=0;
                    instr_v=0;
                    instr_s=0;
                    instr_p=0;
                    instr_m=0;
                    instr_d=0;
                    invalid_msg=0;

				///////////////////////////////////////////////////
				// 												 //
				//                 ANÁLISE MENSAGEM				 //
				//                                               //
				///////////////////////////////////////////////////

			while (str2[j]!='\r') //parse the message until the carriage return
				{
				if ((str2[j]=='s' || str2[j]=='S') && !instr_s) { 	//velocidade
					n=0;
					j++;
					instr_s=1;
					while (str2[j]>=48 && str2[j]<=57) {
						j++;
						n++;
					}
					j--;
					if (n>3 || n<=0) {
						invalid_msg=1;
						break;
					}
				}else if ((str2[j]=='m' || str2[j]=='M') && !instr_m) {	//movimento (desactivado)
					instr_m=1;
				}else if ((str2[j]=='b' || str2[j]=='B') && !instr_d) {	//movimento sentido anti-horário
					instr_d=1;
				}else if ((str2[j]=='f' || str2[j]=='F') && !instr_d) {	//movimento sentido horário
					instr_d=1;
				}else if ((str2[j]=='h' || str2[j]=='H') && !instr_h) {	//homeposition
					instr_h=1;
				}else if ((str2[j]=='v' || str2[j]=='V') && !instr_v) {	//verbose
					instr_v==1;
					j++;
					if (str2[j]!=49 && str2[j]!=48) {
						invalid_msg=1;
						break;
					}
				}else if ((str2[j]=='p' || str2[j]=='P') && instr_p==0){	//posição de paragem
					q=0;
					j++;
					instr_p=1;
					while (str2[j]>=48 && str2[j]<=57) {
						j++;
						q++;
					}
					j--;
					if (q>4 || q<=0) {
						invalid_msg=1;
						break;
					}
				}else {
					invalid_msg=1;
					break;
				}
				j++;
			}

				///////////////////////////////////////////////////
				// 												 //
				//    PROCESSAMENTO E EXECUÇÃO DE MENSAGEM		 //
				//                                               //
				///////////////////////////////////////////////////

			if (invalid_msg==1) {
				for (j=0;j<i+1;j++) {
					str2[j]=' ';	//limpar o array
				}
				SendPointer(Msg_corrupted);
			} else if (invalid_msg==0) {
				SendPointer(Msg_received);
				j=0;
				m=0;
				p=0;
				while(j<i) {
					if (str2[j]>=48 && str2[j]<=57) {
						if (ss==1) {
							temp[m]=str2[j];
							m++;
							if (ss==1 && m==n) {
								speed=Array2Int(temp,m);			//converter array em valor decimal
								if (speed<=150 && speed>24) {
									freq=speed;						//conversão da velocidade em período
									freq*=853;
									Presc2=5E6;
									Presc2/=freq;
									SpeedRamp(Presc2,PR2);			//executar rampa de aceleração
								} else {
									SendPointer(Invalid_speed);
								}
								for (w=0;w<m+1;w++) {
								temp[w]=' ';						//limpar o array
								}
								ss=0;
							}
						}else if (pp==1) {
							temp[p]=str2[j];
							p++;
							if (p==q) {
								Pos_int=Array2Int(temp,p);			//converter array em decimal
								//Pos_int*=5.7;						//converter posição angular em pulsos
								//Pos_int/=35;
								if (Pos_int<2048 && Pos_int>0) {
									SpeedRamp(245,PR2);

									Pos_stop=Pos_int;
								}else {
									SendPointer(Invalid_pos);
								}
								for (w=0;w<p+1;w++) {
									temp[p]=' ';
								}
								pp=0;
							}
						}else if (vv==1) {
							if (str2[j]==49) {
								rs=1;					//activar envio de posição e velocidade
							}else if (str2[j]==48) {
								rs=0;					//desactivar envio de posição e velocidade
							}
							vv=0;
						}
					}

					switch 	(str2[j])
					{
					case 's': case'S':

						ss=1;
						break;

					case 'v' : case'V':
					{
					//verbose
						vv=1;
						break;
					}
					case 'p': case 'P':
						//posição de paragem
						pp=1;
						break;

					case 'm': case 'M' :
						//movimento (desactivado)
						if (PORTAbits.RA0) {
							SpeedRamp(245,PR2);
							PWM_off();
						}
						else
						{
							SendPointer(Stopped);
						}
						break;

					case 'b' : case 'B':
						//movimento sentido anti-horário
						if (!PORTAbits.RA1 && PORTAbits.RA0) {
							Speed=PR2;
							SpeedRamp(245,PR2);
							PWM_off();
							PORTAbits.RA1=1;
							wait(1000000);
							SpeedRamp(Speed,PR2);
						}else if (!PORTAbits.RA1 && !PORTAbits.RA0) {
							PORTAbits.RA1=1;
						}else if (PORTAbits.RA1==1) {
							SendPointer(Invalid_rot);
						}
						break;

					case 'f': case 'F':
						//movimento sentido horário
						if (PORTAbits.RA1==1 && PORTAbits.RA0==1) {
							Speed=PR2;
							SpeedRamp(245,PR2);
							PWM_off();
							PORTAbits.RA1=0;
							wait(1000000);
							SpeedRamp(Speed,PR2);
						}else if (PORTAbits.RA1==1 && PORTAbits.RA0==0) {
							PORTAbits.RA1=0;
						}else if (PORTAbits.RA1==0) {
							SendPointer(Invalid_rot);
						}
						break;

					case 'h' : case 'H' :
						//homeposition
						SpeedRamp(245,PR2);
						hp=HomePos();
						k=0;
						break;

					default:
						break;
					}
					j++;
				}
				for (j=0;j<i+1;j++) {
					str2[j]=' ';	//clear the array
				}
			}

			RX=0;
			RCSTAbits.CREN=1; //Reactivate the continuous reception and prepare for new incomming messages
			PIE3bits.IC2QEIE=1;					//QEI Interrupt Flag Enable bit
			INTCONbits.TMR0IE=1;  				//send
		}
                */
    };
}

