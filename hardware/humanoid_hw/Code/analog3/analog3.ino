
#include <TimerOne.h> // Biblioteca para Timer1

// defeniçoes de variaveis para acesso aos registos do 
// microControlador para mudança de prescale clock
#define FASTADC 1
// defines for setting and clearing register bits
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

// declaraçao de variaveis
// numero dos pinos analogico
int SensorPin1 = A7;   
int SensorPin2 = A6;
int SensorPin3 = A5;
int SensorPin4 = A4; 

// valores de leitura analogica
int SensorValue1=0;
int SensorValue2=0;
int SensorValue3=0;
int SensorValue4=0;

// array com a menssagem completa
byte data[6];

// periodo do interrupt
unsigned long wait;

// bool que diz se existe uma mensagem lida e nao enviada
bool new_msg;

void setup() 
{
  // baud rate da comunicaçao serie
  Serial.begin(115200);
  
  // definiçao do prescale clock
  #if FASTADC
    // set prescale to 16
    sbi(ADCSRA,ADPS2) ;
    cbi(ADCSRA,ADPS1) ;
    cbi(ADCSRA,ADPS0) ;
  #endif
  
  //Definiçao de um timer interno
  wait=769; //time of the actualization in microseconds (769)
  Timer1.initialize(wait); // set a timer have 1300 Hz
  Timer1.attachInterrupt( timerInterr ); // attach the service routine here
  
  // inicializaçao da variavel bool de nova menssagem
  new_msg=0;
  // inicializaçao do bit de separaçao
  data[5]='\n';
}

// funçao que ao chamada pelo interrupt para leitura dos valores 
// analogicos
void timerInterr()
{
  // leitura analogica
  SensorValue1 = analogRead(SensorPin1);
  SensorValue2 = analogRead(SensorPin2);
  SensorValue3 = analogRead(SensorPin3);
  SensorValue4 = analogRead(SensorPin4);
  
  // set a bool => nova mensagem nao enviada
  new_msg=1;
}

// rotina/loop principal
void loop() 
{
  if(new_msg==1)
  {
      // shift dos 10-bits para 8-bits
      //data[0]=map(SensorValue1,45,525,0,255);
      //data[1]=map(SensorValue2,45,525,0,255);
      //data[2]=map(SensorValue3,45,525,0,255);
      //data[3]=map(SensorValue4,45,525,0,255);
      
      data[0]= SensorValue1 >> 2;
      data[1]= SensorValue2 >> 2;
      data[2]= SensorValue3 >> 2;
      data[3]= SensorValue4 >> 2;
      data[4]= (SensorValue1 & 0b00000011) << 6 | (SensorValue2 & 0b00000011) << 4 | (SensorValue3 & 0b00000011) << 2 | (SensorValue4 & 0b00000011);
      
      //data[0]=1;
      //data[1]=1;
      //data[2]=1;
      //data[3]=1;
      //data[4]=0b00000000;
      
      // int & 0b00001100
      //verificaçao que os valores nao sao iguais ao valor do 
      // bit de separaçao
      if( data[0]==data[5]) data[0]=11; 
      if( data[1]==data[5]) data[1]=11;
      if( data[2]==data[5]) data[2]=11;
      if( data[3]==data[5]) data[3]=11;
      if( data[4]==data[5]) data[4]=11; //!!!!!!!!??????????

      // envio do array por porta serie
      Serial.write(data,6);
      
      // reset do bool de nova mensagem
      new_msg=0;
  }

}


