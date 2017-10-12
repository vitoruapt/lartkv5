/**
 *       @file  func.c
 *      @brief  PIC code
 *     @author  LAR
 *
 *   @internal
 *     Created  JAN 2012
 *    Revision  ---
 *    Compiler  Microchip C18
 *     Company  University of Aveiro
 *   Copyright  Copyright (c) 2012, LAR
 *
 * =====================================================================================
 */

#include "func.h" 

/*####################################################################*/
/**
 * @brief pause
 * @param value to pause
 * @return void
 */
 
void wait(unsigned int value) 
{	//temporizador 

	unsigned long int i,k;
	for (i=0;i<400*value;i++) { 
	k=k;	// Obriga o pic a fazer k=k '400*value' vezes
	}
}
/*####################################################################*/
/**
 * @brief Fun??o para definir o PWM_off
 * @param n?o possui inputs
 * @return n?o possui outputs
 */

//Pic 18f2431 CHECK 
void PWM_off() {
		PR2=0; // The PWM period is specified by writing to the PR2 register, pag 150 data sheet
		// o microcontrolador possui dois modulos CCP1 e CCP2 (capture/compare/PWM)
		
		// DCxB0 e DCxB1 --> These bits are the two LSbs (bit 1 and bit 0) 
		// of the 10-bit PWM duty cycle. The eight MSbs of the duty cycle are found in CCPR1L.
 
		CCP1CONbits.CCP1M3=0x0;	//desactiva o gerador de pulsos // 0000 PWM disabled
		CCP1CONbits.CCP1M2=0x0;
		CCP1CONbits.CCP1M0=0x0; 
		CCP1CONbits.CCP1M1=0x0;
		wait(50000);			
		PORTAbits.RA0=0x0;		//desactiva o enable do L297

}

/*####################################################################*/
/**
 * @brief Fun??o para definir o PWM_on
 * @param n?o possui inputs
 * @return n?o possui outputs
 */

//Pic 18f2431 CHECK 
void PWM_on() {
		PR2=245; // define o periodo do PWM
		CCP1CONbits.CCP1M3=0x1;	//activa o gerador de pulsos // 11xx PWM enabled
		CCP1CONbits.CCP1M2=0x1;
		PORTAbits.RA0=1;		//activa o enable do L297
}

/*####################################################################*/
/**
 * @brief Função para enviar arrays
 * @param o array a enviar e a sua dimensão
 * @return não possui outputs
 */
 
//Pic 18f2431 CHECK 
void SendArray(char array[],int dim) {		//envio de arrays
		int s;
		for (s=0;s<dim;s++) {		
			while (PIR1bits.TXIF==0); // TXIF is set (==1) quando o TXREG transfer the data
			TXREG=array[s];
		}
}

/*####################################################################*/
/**
 * @brief Função para enviar ponteiros
 * @param variável char a enviar
 * @return não possui outputs
 */

//Pic 18f2431 CHECK 
void SendPointer(rom near char *nline) {	//envio de ponteiros

	while (*nline!=0) {				
		while(PIR1bits.TXIF==0);   // TXIF is set (==1) quando o TXREG transfer the data	
		TXREG=*nline++;			
	}
}

/*####################################################################*/
/**
 * @brief Função para configurar a comunicação RS232
 * @param não possui inputs
 * @return não possui outputs
 */

//Pic 18f2431 CHECK 
void RS232() {
	// this part has to be changed in accordance to the PIC, pay special attention to the baud rate generator
	//rs232 comunicação assincrona, envio de caracteres	
	
	// Activar TX e RX
	//TRISCbits.RC6=1;
	//TRISCbits.RC7=1;
	
	TXSTAbits.TXEN=1; 		//activar a escrita (Transmit enable)
	TXSTAbits.SYNC=0; 		//activar modo assincrono
	TXSTAbits.BRGH=1;		//selecção de modo baud rate elevado (high baud rate)
	TXSTAbits.TX9=0;		//negar nono bit, transmissão de 8 bits
	RCSTAbits.SPEN=1;		//activar porta série (serial port enable)
	RCSTAbits.CREN=1;		//activar recepção contínua
	RCSTAbits.RX9=0;		//negar nono bit, recepção de oito bits
		
	BAUDCONbits.BRG16=1; 	//BRGH=1 e BRG16=0 para 8 bits
	SPBRG=42;		//definição de baud rate de 115200 para uma Fosc de 20 MHz

        // Testes funcionaram com estes dois registos a 0.

	//PIE1bits.TXIE=1;		//activação do interrupt para transmissão de dados pela porta série
	//TXIE: EUSART Transmit Interrupt Enable bit
	//1 = Enables the EUSART transmit interrupt
	
	PIE1bits.RCIE=1;		//activa??o do interrupt para recep??o de dados pela porta s?rie
	//Peripheral Interrupt Enable register
	//RCIE: EUSART Receive Interrupt Enable bit
	//1 = Enables the EUSART receive interrupt

}

/*####################################################################*/
/**
 * @brief Função para configuração do PIC
 * @param não possui inputs
 * @return não possui outputs
 */
 
void PICinit() 
{
//definição de portas
	TRISB=0b11110011;
       	TRISA=0b11111110;
        TRISCbits.RC2=0;    // CCP1 pin an output - RC2 como saída

//definição interrupt RB5/KBI1
        /*
        INTCONbits.RBIE=1;              //enable interrupções RB
        INTCONbits.RBIF=0;             //inicializar flag a 0
        INTCON2bits.NOT_RBPU=1;
        INTCON2bits.RBIP=0;            //low priority interrupt
         */
//definição interrupt INT1
        /*
        TRISCbits.TRISC4=1;             //definir como input
        INTCON3bits.INT1E=1;
        INTCON3bits.INT1IF=0;
        INTCON3bits.INT1IP=0;           //low priority interrupt
        INTCON2bits.INTEDG1=0;          //falling edge
        */

//defini?o das portas como digitais ou anal?gicas
	ANSEL0bits.ANS0=0;
	ANSEL0bits.ANS1=0;
	ANSEL0bits.ANS2=0;
	ANSEL0bits.ANS3=0;	
	ANSEL0bits.ANS4=0;  // AN0,AN1,AN2,AN3 e AN4 como digitais (p?g 245)

        RCONbits.IPEN=1; //activação dos níveis de prioridade para interrupts


	RS232();
        //activação de interrupts

        //código de activação do timer comentado , envio de mensagem por
        //interrupt do pulso de sincronismo na entrada INT1
        
        TMR0L=0xB0;
        TMR0H=0xED;

	//IPR1bits.TX1IP=0 necess?rio definir prioridade para o interrupt de envio?? se sim qual??
	//IPR1bits.RCIP=1;		//alta prioridade para o interrupt RCIF //RPascoal was 0
	IPR3bits.IC2QEIP=1;		//definir alta prioridade para o interrupt QEI Interrupt Priority bit
	INTCON2bits.TMR0IP=0;	//definir baixa prioridade para interrupt de TIMER 0 //RPascoal was 1


        INTCONbits.TMR0IE=1;            //Enables the TMR0 overflow interrupt
	PIE3bits.IC2QEIE=1;		//activa o interrupt QEI Interrupt Flag Enable bit
        PIE3bits.IC1IE = 0; //interrupt capture
	//executa 1 interrupt quando receber 1 INDX pulse
	
//definição do timer0 que irá temporizar o envio das mensagens

	T0CONbits.T0CS=0;		//definir o timer0 com o clock interno
	T0CONbits.T016BIT=0;	//definir o contador para uma resolução de 16 bits
	T0CONbits.PSA=0;		//activar prescaler para o timer0
	T0CONbits.T0PS0=0;
	T0CONbits.T0PS1=0;
	T0CONbits.T0PS2=1;		//prescaler de timer 0 definido para 32 (<2:0>: 100)
        T0CONbits.TMR0ON=1;		//activação do timer0
        
	
//defini??o dos par?metros para para PWM
	CCP1CONbits.DC1B1=0;	//os bits DC1B0:DC1B1 s?o os 2 bits menos significativos (LSBs)
	CCP1CONbits.DC1B0=0;	//do registo que permite definir o duty cycle
	CCPR1L=4;				//oito bits mais significativos (MSBs)
	TRISCbits.TRISC2=0;		//definir o resgisto RC2/CCP1/P1A como sa?da
	T2CONbits.TMR2ON=1;		//activar o timer 2	
	T2CONbits.T2CKPS1=0x0;	//prescaler unit?rio
	T2CONbits.T2CKPS0=0x0;  // 00 prescaler == 1
	

//defini??o do m?dulo encoder do pic
//setup quadrature encoder

	/*
	QEICON = QEI_2XINDX | VELOCITY_DIV_4;
	QEICON &=0b01111111;      //clear Velocity mode bit to enable
	CAP1CON |= 0b01000000;   //enable Time Base reset (TMR5)
                            //TMR5 used for velocity measurement
	setup_timer_5 (T5_INTERNAL | T5_DIV_BY_4);
	*/
	
        QEICON = 0b10100101;
	// QEICONbits.VELM=0 --> Velocity mode enabled
	// QEICONbits.QERR=0 -->Position counter overflow or underflow
	// QEICONbits.UP/DOWN=1 --> Direction of Rotation Status Forward
	// QEICONbits.QEIM2=0
	// QEICONbits.QEIM1=0
	// QEICONbits.QEIM0=1 --> QEI enabled in 2x Update mode; INDX resets the position counter
	// QEICONbits.PDEC1=0
	// QEICONbits.PDEC0=1 --> Velocity Pulse Reduction Ratio 1:4


        // Have to choose the maximum count as function of the QEICON bits
        MAXCNTL = 0x00;
        MAXCNTH = 0x08;
	
	//DFLTCON necess?rio configurar o noise filter??
        DFLTCON=0b00111011;
	
	CAP1CON = 0b01001111;
        // see pg. 165 pdf
        // - not imp -
	 //CAP1CONbits.CAP1REN=1 --> Time Base Reset Enabled
	// - not imp -
        // - not imp -
        // CAP1M3
        // CAP1M2
        // CAP1M1
        // CAP1M0

        TMR5L=0xFF;							//13 ms
	TMR5H=0xFF;
	T5CON = 0b00110001;
	// T5CONbits.T5SEN=0 --> Timer5 is disabled during Sleep
	// T5CONbits.RESEN=0 --> not implemented on PIC18F2431 device and read as '0'
	// T5CONbits.T5MOD=1 --> Single-Shot mode is enabled; 0 continuous 
	// T5CONbits.T5PS1=1 
	// T5CONbits.T5PS0=0 --> Timer5 Input Clock Prescale Select bits 1:4
	// T5CONbits.T5SYNC=0 --> When TMR5CS = 0: This bit is ignored. Timer5 uses the internal clock when TMR5CS = 0.
	// T5CONbits.TMR5CS=0 --> Internal clock (TCY)
	// T5CONbits.TMR5ON=0 --> Timer5 is disabled will answer to cap
	
	//The Timer5 Special Event Trigger Reset input can act as a Timer5 wake-up and a start-up pulse. 
	//Timer5 must be in Single-Shot mode and disabled (TMR5ON = 0).

        
        INTCONbits.GIE=1;		//activa todos os interrupts (global interrupt enable)
	INTCONbits.PEIE=1;		//activa todos os interrupts periféricos (pheriferical interrupt enable)

}

/*####################################################################*/
/**
 * @brief Função para enviar um inteiro (int)
 * @param inteiro a enviar e um char a especificar 'espaço' ou 'nova_linha'
 * @return não possui outputs
 */
 	
void send(int count,char NL) {
	
	unsigned int rest,lap1,lap2;
	int v=0,u;
	static char str[];

	if (NL=='NewLine') {
		while(PIR1bits.TXIF==0);
		TXREG=0xa; // 0x0A ou 'newline' em ASCII
	}else if (NL=='SameLine') {
		while(PIR1bits.TXIF==0);
		TXREG=0x20; // 'space' em ASCII
	}

	lap1=count;
	lap2=count;		

	while (lap1>0) {		//contabiliza-se quantos algarismos tem o número
		v++;			
		lap1/=10;		// lap1 a dividir por 10
	}					
	for (u=v-1;u>0;u--) { 	//contabilizados os algarismos, inseri-los num array			
		rest=lap2%10;			
		rest+=0x30; // 0x30 ou '0' em ASCII
		str[u]=rest;		
		lap2/=10;			
	}						
	str[0]=lap2+0x30;			
	
	SendArray(str,v);		//enviar array por linha série		
}

/*####################################################################*/
/**
 * @brief Fun??o para definir a Home Position
 * @param n?o possui inputs
 * @return n?o possui outputs
 */
 
short int HomePos() 
{
		PWM_on();
		return 1;
}

/*####################################################################*/
/**
 * @brief Fun??o para transformar um array do tipo char num inteito (int)
 * @param array do tipo a char a transformar e a sua dimens?o
 * @return retorna um inteiro (int)
 */
 
int Array2Int(char cont[],int e) 
{		//convers?o de array em decimal
	unsigned int x,m=0,a=1,r=0;
	
	if (e>4) {			
		return 10000;	
	}			
	
	for(x=0;x<e-1;x++) {
		a*=10; // x ??
	}

	for(x=0;x<e;x++) {
		m=cont[x]-48;
		m*=a;
		r+=m;	
		a/=10;
	}
	return r;
}

/*####################################################################*/
/**
 * @brief Fun??o para definir a rampa de acelera??o
 * @param n?o possui inputs
 * @return n?o possui outputs
 */
 
void SpeedRamp(int Pres,int OldPres)
{	//rampa de velocidades
	unsigned int rp,fraq;
	
	if (PORTAbits.RA0==0) {
		PORTAbits.RA0=1;
	}
	if (OldPres==0) {
		OldPres=245;
		PWM_on();
	}
	rp=OldPres;
	
	if (Pres>OldPres) {					//Rampa de acelera??o
		while (rp<Pres+1) 
		{	
			if (rp>150) {
				wait(5);
			}else if (rp>50 && rp<=150){
				wait(10);
			}else if (rp<=50){
				wait(20);
			}
			rp++;
			PR2=rp;
		}
	}else if (Pres<OldPres){			//Rampa de desacelera??o
		while(rp>Pres-1) {
				if (rp>150) {
				wait(5);
			}else if (rp>50 && rp<=150){
				wait(10);
			}else if (rp<=50){
				wait(20);
			}
			rp--;
			PR2=rp;
		}
	}else if (Pres==OldPres) {			
		PR2=Pres;
	}
}

