/**
 * Copyright (c) 2009 Andrew Rapp. All rights reserved.
 *
 * This file is part of XBee-Arduino.
 *
 * XBee-Arduino is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * XBee-Arduino is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with XBee-Arduino.  If not, see <http://www.gnu.org/licenses/>.
 */
 
#include <XBee.h>

/*
This example is for Series 2 XBee
Receives a ZB RX packet and sets a PWM value based on packet data.
Error led is flashed if an unexpected packet is received
*/

XBee xbee = XBee();
//XBeeResponse response = XBeeResponse();
// create reusable response objects for responses we expect to handle 
ZBRxResponse rx = ZBRxResponse();
ModemStatusResponse msr = ModemStatusResponse();

int statusLed = 12;
int errorLed = 13;
int dataLed = 11;
int pwmOutput = 0;

void flashLed(int pin, int times, int wait) {
    
    for (int i = 0; i < times; i++) {
      digitalWrite(pin, HIGH);
      delay(wait);
      digitalWrite(pin, LOW);
      
      if (i + 1 < times) {
        delay(wait);
      }
    }
}

void setup() {
  pinMode(statusLed, OUTPUT);
  pinMode(errorLed, OUTPUT);
  pinMode(dataLed,  OUTPUT);
  
  // start serial
  xbee.begin(9600);
  
  flashLed(13, 2, 200);
  //flashLed(errorLed, 2, 255);
}

// continuously reads packets, looking for ZB Receive or Modem Status
void loop() {
    //Serial.println("loop");
    xbee.readPacket();
    
    if (xbee.getResponse().isAvailable()) {
      // got something
      
      if (xbee.getResponse().getApiId() == ZB_RX_RESPONSE) {
        // got a zb rx packet
        
        // now fill our zb rx class
        xbee.getResponse().getZBRxResponse(rx);
            
        if (rx.getOption() == ZB_PACKET_ACKNOWLEDGED) {
            // the sender got an ACK
            Serial.println("message received");
            flashLed(statusLed, 2, 20);
        } else {
            // we got it (obviously) but sender didn't get an ACK
            flashLed(errorLed, 2, 20);
        }
        
        if ((rx.getData(0) & 0x01)==0x01){
          Serial.println("negative direction");
          Serial.println(-1*rx.getData(1),DEC);
          Serial.println();
        }
        else {
          Serial.println("positive direction");
          Serial.println(rx.getData(1),DEC);
          Serial.println();          
        }          
        
        if ((rx.getData(0) & 0x02)==0x02){
          Serial.println("negative speed");
          Serial.println(-1*rx.getData(2),DEC);          
          Serial.println();
        }
        else {
          Serial.println("positive speed");
          Serial.println(rx.getData(2),DEC);
          Serial.println();
        }
        
//        Serial.println(rx.getData(0),BYTE);
//        Serial.println(rx.getData(1),BYTE);
//        Serial.println(rx.getData(2),BYTE);      
        
        // set dataLed PWM to value of the first byte in the data
        Serial.println(rx.getData(2),DEC);
        pwmOutput = rx.getData(2);
        //analogWrite(3, pwmOutput);
        flashLed(13, pwmOutput, 50);
        
      } else if (xbee.getResponse().getApiId() == MODEM_STATUS_RESPONSE) {
        xbee.getResponse().getModemStatusResponse(msr);
        // the local XBee sends this response on certain events, like association/dissociation
        
        if (msr.getStatus() == ASSOCIATED) {
          // yay this is great.  flash led
          flashLed(statusLed, 10, 100);
        } else if (msr.getStatus() == DISASSOCIATED) {
          // this is awful.. flash led to show our discontent
          flashLed(errorLed, 10, 10);
        } else {
          // another status
          flashLed(statusLed, 5, 10);
        }
      } else {
      	// not something we were expecting
        flashLed(errorLed, 5, 500);    
      }
    }
}
