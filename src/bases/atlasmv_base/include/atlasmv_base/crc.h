/**************************************************************************************************
 Software License Agreement (BSD License)

 Copyright (c) 2011-2013, LAR toolkit developers - University of Aveiro - http://lars.mec.ua.pt
 All rights reserved.

 Redistribution and use in source and binary forms, with or without modification, are permitted
 provided that the following conditions are met:

  *Redistributions of source code must retain the above copyright notice, this list of
   conditions and the following disclaimer.
  *Redistributions in binary form must reproduce the above copyright notice, this list of
   conditions and the following disclaimer in the documentation and/or other materials provided
   with the distribution.
  *Neither the name of the University of Aveiro nor the names of its contributors may be used to
   endorse or promote products derived from this software without specific prior written permission.
 
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***************************************************************************************************/
/**
* @defgroup crc Cyclic Redundancy Check
* @brief This library is intended to calculate the crc for a data string. This library was developed by Lammert Bies and readapt to LAR purposes.
* This lib contains source code for functions to calculate five commonly used CRC values: CRC-16, CRC-32, CRC-DNP, CRC-SICK, CRC-Kermit and CRC-CCITT.
*   To calculate a CRC, the following three steps must be followed:
*  1. Initialize the CRC value. For CRC-16, CRC-SICK CRC-Kermit and CRC-DNP       the initial value of the CRC is 0. For CRC-CCITT and CRC-MODBUS, the value 0xffff is used. CRC-32 starts with an initial value of 0xffffffffL.
*  2. For each byte of the data starting with the first byte, call the function update_crc_16(), update_crc_32(), update_crc_dnp(), update_crc_sick(), update_crc_kermit() or update_crc_ccitt() to recalculate the value of the CRC.
*  3. Only for CRC-32: When all bytes have been processed, take the one's complement of the obtained CRC value.
*  4. Only for CRC-DNP: After all input processing, the one's complement of the CRC is calcluated and the two bytes of the CRC are swapped.
*  5. Only for CRC-Kermit and CRC-SICK: After all input processing, the one's complement of the CRC is calcluated and the two bytes of the CRC are swapped.
*
* @ingroup utils
* @author dgameiro
* @version 1.0
* @date 27 April 2010
*@{
	*/
#ifndef _CRC_H_
#define _CRC_H_
/** @file
* @brief header for this library. Defines public funtions prototypes this library makes available to other modules.
*/

//####################################################################
// Includes:
//####################################################################

//System Includes
//all system includes go here
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


//####################################################################
// Typedefs:Declare here structs that have interesting params for ####
// other modules besides this one, but that will not be sent through #
// messages. A function to install this struct with param daemon #####
// should be declared ################################################
//####################################################################

//####################################################################
// Prototypes:	public functions can be declared here  ################
//####################################################################

#define CRC_VERSION     "1.16"
#define FALSE           0
#define TRUE            1

unsigned short update_crc_16(unsigned short crc, char c);
unsigned long update_crc_32(unsigned long  crc, char c);
unsigned short update_crc_ccitt(unsigned short crc, unsigned char c);
unsigned short update_crc_dnp(unsigned short crc, char c);
unsigned short update_crc_kermit(unsigned short crc, char c);
unsigned short update_crc_sick(unsigned short crc, char c, char prev_byte);

#endif
/**
*@}
*/
