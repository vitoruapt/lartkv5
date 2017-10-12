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
 * \file
 * \brief Des70-10 Communication library source code
 */

#include <atlasmv_base/des70_10.h>

/**
* @brief function to open serial port and set some params for servoamplifier
*  
* @param com_device - path to device
* @param port - device port identification (exit)
* @return this function will returns 0 if error occurs
*/
int InitDES_communication(std::string com_device, int* port)
{
	int result = true;
	
	//opens serial connection to DES servoamplifier
	
	struct termios newParams;
	int ret;
	
	memset(&newParams,0,sizeof(newParams));
	
	printf("Opening Comm for DES 70/10..."); fflush(stdout);
	*port = open(com_device.c_str(), O_RDWR | O_NONBLOCK ); 
	tcflush(*port, TCIFLUSH);
	
	if(*port == -1)
	{
		printf("Error\n");
		printf("Failed to open Port %s for device DES 70/10\n", com_device.c_str());
		result = false;
	}else{
		printf("Done\n");
		
		newParams.c_cflag = B38400 | CS8 | CLOCAL | CREAD;
		newParams.c_iflag = IGNPAR;
		newParams.c_oflag = 0;
		//newParams.c_lflag = ~ICANON;
		
		ret = tcsetattr(*port, TCSANOW, &newParams);
		
		if(ret < 0)
		{
			perror("Set serial communication parameters failed:");
			result = false;
		};
	}
	
	return result;
};

/** @fn int calc_crc_16(TYPE_msg_frame* msg)
* @brief function used to get the message crc for 16bits data, the crc is obtained using first the higher byte of the WORD
* 
* @param msg - TYPE_msg_frame must be specified previously, were is updated the crc field
* 
* @return - this function returns always 1 and crc field at msg structure
*/
int calc_crc_16(TYPE_msg_frame* msg)
{
	msg->size = (msg->data[0]+1)*2+1; //calculate the data size, based on msg->data[0]
	msg->crc = 0x00;//crc mus te initialized at 0x00
	msg->crc = update_crc_ccitt(msg->crc, msg->OpCode);
	
	msg->crc = update_crc_ccitt(msg->crc, msg->data[0]);
	for(int i=1; i< msg->size; i+=2)
	{
		//printf("#CRC# msg->data[%d]:0x%02x\n", i+1, msg->data[i+1]);
		msg->crc = update_crc_ccitt(msg->crc, msg->data[i+1]);
		//printf("#CRC# msg->data[%d]:0x%02x\n", i, msg->data[i]);
		msg->crc = update_crc_ccitt(msg->crc, msg->data[i]);
		
	};
	
	//printf("#CRC# msg->crc:0x%04x\n", msg->crc);
	msg->data[msg->size] = (msg->crc & 0xff);//
	msg->data[msg->size+1] = ((msg->crc >> 8) & 0xff);
	return 1;
}

/** @fn int write_msg_inbuf(const int port, unsigned char* data, int size)
* @brief send to serial port a stream of bytes
* 
* @param port - device port
* @param data - vector with streaming of data
* @param size - number of bytes to be sent
*
* @return this function will returns 0 if error occurs
*/
int write_msg_inbuf(const int port, unsigned char* data, int size)
{
	int result = true;
	
	//for(int i=0;i<size; i++)
	{
		int n = write(port, data, size);
		data++;
		if(n<0 || n!= size)
		{
			perror("\terror transmiting");
			result = false;
		};
	};
	
	return result;
};

/**
* @brief verify if there is some by in RX serial buffer
* 
* @param port - device port
* @param wait_time - maximum delay to receive the acknowledge from DES servoamplifier in seconds 
* @param msg - the return byte if the result is 1
* 
* @return the function returns: (0)-if error occurs, (-1)- waiting time exceeds limits, (1)-no error
*/
int check_msg_aceptance(const int port, float wait_time, unsigned char* msg)
{
	int n=0;
	int result = 1;
	unsigned char byteread[20];
	char finished = true; //byte have not arrived
	c_timer mid_timer;
	
	
	mid_timer.tic(0);//to count time to receive msg
	while(finished)
	{
		n = read(port, &byteread[0], 1);
		//sleep(0.01);
		if(n>0)
		{
			msg[0] = byteread[n-1];
			finished = false;
			
		}else if(n==-1)
		{
			result = 0;
			finished = false;
			*msg=20;
		}
		mid_timer.toc(0);
		if((mid_timer.get_toc(0)> wait_time))
		{
			result = -1;
			finished = false;
			*msg=20;
		}
	}
	return result;
};

/**
* @brief function just to send OpCode and verify if command was understood
* 
* @param port - device port
* @param tries - max number of tries to receive msg
* @param OpCode - unsigned char with DES command, according to DES command reference
* @param flg - (1) will printf special info
* 
* @return (-1)-'F' is received from DES,(0)-max num of tries is reached, (1)-'O' is received from DES
*/
int send_OpCode(const int port, unsigned char tries, unsigned char OpCode, char flg) 
{
	//prepare to send msg, first send OpCode and check if servoamplifier understand the given command
	char result = 0;
	unsigned char tmp[2];
	unsigned char num=0;
	int err;
	#define WAIT 0.05
	c_timer timer0;
	
	tmp[0] = 0;
	
	//printf("sending OpCode %f\n", lar_get_time());
	
	while(num < tries)
	{
		write_msg_inbuf(port, &OpCode, 1);
		
		err = check_msg_aceptance(port, WAIT, &tmp[0]);
		tcflush(port, TCIFLUSH);
		//printf("LIB_DES tries:%d err:%d tmp[0]:%d\n", num, err, tmp[0]);
		
		
		switch(err)
		{
			case 1:
			{
				if(tmp[0] == 'O')
				{
					result = 1;//command understood
					num = tries+20;
				}else if(tmp[0] == 'F')
				{
					result = -1;//command not understood
				}
				break;
			}
			case -1:
			{
				result=0;
				//printf("LIB_DES: err OpCode:%d\n", err);
			}
		};
		
		if(flg)
		{
			printf("### OpCode - 0x%02x###\n", OpCode);
			if(result == 1)
			{
				printf("OpCode understood\n");
			}else if(result == -1)
			{
				printf("OpCode not understood\n");
			}
			
		}
		num++;
	};
	
	//printf("finished send OpCode %f result:%d\n", lar_get_time(), result);
	return result;
};

/**
* @brief 
* 
* @param port - serial port conection
* @param msg - message frame to be sent according DES frame struct (TYPE_msg_frame)
* @param flg - (1) to print out debug info
* 
* @return (-1)-'F' is received from DES,(0)-max num of tries is reached, (1)-'O' is received from DES
*/
int send_data(const int port, TYPE_msg_frame* msg, char flg)
{
	int result = 0;
	unsigned char tmp[2];
	int err;
	#define WAIT 0.05
	
	tmp[0] = 'N';
	
	write_msg_inbuf(port, &msg->data[0], 1);
	write_msg_inbuf(port, &msg->data[1], msg->size-1);//send all data in the rigth order
	write_msg_inbuf(port, &msg->data[msg->size], 2);//send crc word
	tcflush(port, TCIFLUSH);
	
	err = check_msg_aceptance(port, WAIT, &tmp[0]);
	
	switch(err)
	{
		case 1:
		{
			if(tmp[0] == 'O')
			{
				result = 1;//command understood
				
			}else if(tmp[0] == 'F')
			{
				result = -1;//command not understood
				
			}
			break;
		}
		default:
		{
			result=0;
			//printf("LIB_DES: err sending Data:%d\n", err);
		}
	}
	
	if(flg)
	{
		printf("*\tdata[0]\n");
		for(int k=1;k<msg->size;k +=2)
			printf("*\tdata[%d,%d]:0x%02x,0x%02x\n", k, k+1, msg->data[k], msg->data[k+1]);
		
		printf("*\tdata[crc]:0x%04x\n", msg->crc);
		if(result == 1)
		{
			printf("msg sent without error\n");
		}else if(result == -1)
		{
			printf("msg sent but failed(crc or serial conection lost)\n");
		}
	};
	
	return result;
};

/**
* @brief function to read the buffer into a msg frame
* 
* @param port - device port
* @param msg - message frame type
* @param flg - (1) to print out debug info
* 
* @return () 
*/
int read_buffer(const int port, TYPE_msg_frame* msg, char flg)
{
	/** 
	* @brief this define the time that at maximum time is necessary to wait for a msg with a lenght of 110bytes with a baud of 38400 near 23ms
	*/
	#define MAX_WAIT_TIME 0.023
	int result = true;
	unsigned char byteread[20];
	int n;
	int count_bytes;
	char finished = false;
	unsigned char tmp;
	int crc_val=0x00;
	unsigned char non_opcode = 0xff;
	count_bytes = 0;
	c_timer timer0;
	
	timer0.tic(0);
	
	
	while(!finished)
	{
		timer0.toc(0);
		if (timer0.get_toc(0) > MAX_WAIT_TIME) 
		{
			result = false;
			finished = true;
			printf("LIB_DES exceed waiting time:%d\n", result);
		}
		n = read(port, &byteread[0] , 1);
		//cvWaitKey(5);
		if(n>0)
		{
			switch(count_bytes)
			{
				case 0://is the case of receiving the OpCode 0x00 - standard for an answer
				{
					if(byteread[n-1] == 0x00)
					{
						msg->OpCode = byteread[n-1];
						tmp = 'O';
						write_msg_inbuf(port, &tmp, 1);
					}else
					{
						non_opcode = byteread[n-1];
						tmp = 'F';
						write_msg_inbuf(port, &tmp, 1);
						finished = true;
						result = false;
					};
					break;
				};
				case 1://will receive the message len
				{
					msg->data[0] = byteread[n-1];
					msg->size = (msg->data[0]+1)*2+1;
					break;
				};
				default:
				{
					if(count_bytes < (msg->size+2))//msg->size+2 because crc obtained throw the last 2  bytes
					{
						msg->data[count_bytes-1] = byteread[n-1];
					}else
					{
						msg->data[count_bytes-1] = byteread[n-1];
						finished = true;
					}
				}
			}
			count_bytes++;
		}else if(n == -1)
		{
			result = false;
			finished = true;
			perror("error receiving msg");
		};
	}//end while(!finished)
				
				if (result)
				{
					crc_val = (msg->data[msg->size])+(msg->data[msg->size+1] << 8);
					calc_crc_16(msg);
					
					if(crc_val == msg->crc)
					{
						tmp = 'O';
						write_msg_inbuf(port, &tmp, 1);
					}else
					{
						tmp = 'F';
						write_msg_inbuf(port, &tmp, 1);
						result = false;
					};
				};
				
				if(flg)	
				{
					printf("#### Read Buffer ####\n");
					if(result)
					{
						printf("msg->OpCode:0x%02x\n", msg->OpCode);
						printf("msg->data[0]:0x%02x | msg->size:%d\n", msg->data[0], msg->size);
						for(int i = 1; i < msg->size; i += 2)
						{
							printf("msg->data[%d]:0x%02x msg->data[%d]:0x%02x\n", i, msg->data[i], i+1, msg->data[i+1]);
						};
						printf("msg->data[%d]:0x%02x + msg->data[%d]:0x%02x crc: 0x%04x\n", msg->size, msg->data[msg->size], msg->size+1, msg->data[msg->size+1], crc_val);
						printf("obtained msg->crc:0x%04x\n", msg->crc);
					}else
					{
						printf("OpCode not known[0x%02x] or obtained crc!=DES_crc [0x%04x!=0x%04x]\n", non_opcode, msg->crc, crc_val);
					}
				}
				
				return result;
};

/**
* @brief send by rs232 all the data frame inclusively crc value. CRC value is obtained were before send message
* 
* @param port - device port
* @param msg - TYPE_msg_frame frame
* @param flg - (1) if required print debug data
* 
* @return (0) if any error occurs during data transfer
*/
int send_msg_frame(const int port, TYPE_msg_frame* msg, char flg)
{
	int result = true;
	
	
	calc_crc_16(msg);
	
	
	int res = send_OpCode(port, 1, msg->OpCode, flg);
	
	if((res == -1) || (res == 0))
	{
		result = 0;
		//printf("problem sending OpCode not sending remaining data\n");
	}
	
	
	if(result)//msg understood send the remaining data
	{
		res = send_data(port, msg, flg);
		
		if((res == -1) || (res == 0))
		{
			result = false;
			//printf("problem sending remaining data\n");
		}
	};
	
	
	return result;
};

/**
* @brief execute the DES RS232 command 'ReadSysStatus' (OpCode=0x01). The system status is a 16bits value containing different flags.
* 
* @param port - Device Port for serial connection
* @param sys_status - sys_status is a variable inside data structure DES_Status_Variables
* @param flg - (1) to print debug info
* 
* @return this function will returns 0 if error occurs
*/
int DES_ST_read_sys_status(const int port, int* sys_status, char flg)
{
	char result = false;
	TYPE_msg_frame send_msg, received_msg;
	
	send_msg.OpCode = 0x01;//OpCode for ReadSysStatus
	send_msg.data[0] = 0x00;//len-1 where len is 1;
	send_msg.data[1] = 0x00;// dummy word
	send_msg.data[2] = 0x00;
	
	if(flg)
		printf("\n\n\tread system status\n");
	
	
	if(send_msg_frame(port, &send_msg, flg))
	{
		result = true;
		
	};
	
	if(result && (read_buffer(port, &received_msg, flg)))
	{
		
		*sys_status = (received_msg.data[1] + (received_msg.data[2]<<8));
	}else
	{
		
		result = false;
	};
	
	return result;
};


/**
* @brief execute the rs232 command ReadError (OpCode 0x02)
* 
* @param port - Device Port for serial connection
* @param errors - 16bits WORD containing all errors list
* @param flg - (1) to print debug info
* 
* @return this function will returns 0 if error occurs
*/
int DES_ST_read_error(const int port, int* errors, char flg)
{
	int result = false;
	TYPE_msg_frame send_msg, received_msg;
	
	send_msg.OpCode = 0x02;//OpCode for ReadError
	send_msg.data[0] = 0x00;//len-1 where len is 1;
	send_msg.data[1] = 0x00;// dummy word
	send_msg.data[2] = 0x00;
	
	if(flg)
		printf("\n\n\tread error\n");
	
	if(send_msg_frame(port, &send_msg, flg))
	{
		result = true;
	};
	
	if(result && (read_buffer(port, &received_msg, flg)))
	{
		*errors = (received_msg.data[received_msg.size-1] + (received_msg.data[received_msg.size-2]<<8));
	}else
	{
		result = false;
	}
	
	return result;
};


/**
* @brief Clear all DES system errors
* 
* @param port - device for serial port connection
* @param flg - (1) to print debug info
* 
* @return - this function returns 0 if error occurs
*/
int DES_ST_clear_errors(const int port, char flg)
{
	int result = false;
	
	TYPE_msg_frame send_msg;
	
	send_msg.OpCode = 0x03;//OpCode for Clear Errors
	send_msg.data[0] = 0x00;//len-1 where len is 1;
	send_msg.data[1] = 0x00;// dummy word
	send_msg.data[2] = 0x00;
	
	if(flg)
		printf("\n\n\tclear errors\n");
	
	if(send_msg_frame(port, &send_msg, flg))
	{
		result = true;
	};
	
	return result;
};

/**
* @brief RS232 command that reset the des system by restarting the software
* 
* @param port - serial device port
* @param flg - (1) to print debug info
* 
* @return  - this function returns 0 if error occurs
*/
int DES_ST_reset(const int port, char flg)
{
	int result = false;
	
	TYPE_msg_frame send_msg;
	
	send_msg.OpCode = 0x04;//OpCode for Reset Servoamplifier
	send_msg.data[0] = 0x00;//len-1 where len is 1;
	send_msg.data[1] = 0x00;// dummy word
	send_msg.data[2] = 0x00;
	
	if(flg)
		printf("\n\n\trestart servoamplifier with default values\n");
	
	if(send_msg_frame(port, &send_msg, flg))
	{
		result = true;
	};
	
	return result;
	
};

/**
* @brief Set the system to enabled or disabled state. The DES has to be configured for a software setting of enable. If the hardware enabled is activated this command has no effect.
* 
* @param port - serial conection device port
* @param newState - (0) disable servoamplifier (1) enable servoamplifier
* @param flg - (1) to print debug info
* 
* @return  - this function returns 0 if error occurs sending msg 
*/
int DES_ST_enable(const int port, int* newState, char flg)
{
	int result = false;
	
	TYPE_msg_frame send_msg;
	
	send_msg.OpCode = 0x05;//OpCode for Reset Servoamplifier
	send_msg.data[0] = 0x00;//len-1 where len is 1;
	send_msg.data[1] = (*newState & 0xff);// dummy word
	send_msg.data[2] = ((*newState >> 8) & 0xff);
	
	if(flg)
		printf("\n\n\tEnable servo %d\n", *newState);
	
	if(send_msg_frame(port, &send_msg, flg))
	{
		result = true;
	};
	
	return result;
};


/**
* @brief Read the requested temporary system parameter from DES-RAM. SPF is a System Parameter Functions
* 
* @param port - device port
* @param paramNb - Parameter Number according table DES System Parameters and DES Status Variables. check TYPE_DES_sysparam and TYPE_DES_status_var
* @param dataFormat - (0)answer will be a WORD (1) answer will be a LWORD
* @param response - pointer to a WORD
* @param flg - (1) to print debug info
* 
* @return (0) error occurs during sending (-1) when receiving msg
*/
int DES_SPF_read_temp_param(const int port, int paramNb, int dataFormat, int* response, char flg)
{
	int result = false;
	
	TYPE_msg_frame send_msg, received_msg;
	
	send_msg.OpCode = 0x14;//OpCode for Reset Servoamplifier
	send_msg.data[0] = 0x01;//len-1 where len is 1;
	send_msg.data[1] = (paramNb & 0xff);
	send_msg.data[2] = ((paramNb >> 8) & 0xff);
	send_msg.data[3] = (dataFormat & 0xff);
	send_msg.data[4] = ((dataFormat >> 8) & 0xff);
	
	if(flg)
		printf("\n\n\tread parameter: %d\n", paramNb);
	
	if(send_msg_frame(port, &send_msg, flg))
	{
		result = true;
	};
	
	if(result && (read_buffer(port, &received_msg, flg)))
	{
		*response = received_msg.data[1] + (received_msg.data[2]<<8);
	}else
	{
		result = false;
	}
	
	return result;
}

/** 
* @brief Write a new value to a temporary system parameter. Refer to the section about system parameters to find the desired system parameter numbers. SPF is a System Parameter Functions
* 
* @param port - serial device number
* @param paramNb - Parameter Number according table DES System Parameters and DES Status Variables. check TYPE_DES_sysparam and TYPE_DES_status_var
* @param dataFormat - (0)answer will be a WORD (1) answer will be a LWORD
* @param newValue - pointer to a WORD
* @param flg - (1) to print debug info
* 
* @return  (0) error occurs during sending 
*/
int DES_SPF_set_temp_param(const int port, int paramNb, int dataFormat, int* newValue, char flg)
{
	int result = false;
	
	TYPE_msg_frame send_msg;
	
	send_msg.OpCode = 0x15;//OpCode to Set DES-RAM temporary parameter
	send_msg.data[1] = (paramNb & 0xff);
	send_msg.data[2] = ((paramNb >> 8) & 0xff);
	send_msg.data[3] = (dataFormat & 0xff);
	send_msg.data[4] = ((dataFormat >> 8) & 0xff);
	
	if(dataFormat == 0)//required to send a WORD (16bits)
	{
		send_msg.data[0] = 0x02;// message with 3 WORDS (6 bytes)
		send_msg.data[5] = (*newValue & 0xff);
		send_msg.data[6] = ((*newValue >> 8) & 0xff);
		result = true;
	}else if(dataFormat == 1)//required to send a LWORD (32bits)
	{
		send_msg.data[0] = 0x03;// message with 4 WORDS (8 bytes)
		unsigned int low_word = (*newValue & 0xffff);
		unsigned int higher_word = (*newValue >> 16) & 0xffff;
		send_msg.data[5] = (higher_word & 0xff);
		send_msg.data[6] = ((higher_word >> 8) & 0xff);
		send_msg.data[7] = (low_word & 0xff);
		send_msg.data[8] = ((low_word >> 8) & 0xff);
		result = true;
	}
	
	if(flg)
		printf("\n\n\tSet parameter: %d\n", paramNb);
	
	if(result && send_msg_frame(port, &send_msg, flg))
	{
		result = true;
	}else
		result = false;
	
	return result;
};

/** 
* @brief Read all temporary system parameters. DES answer the structure described by TYPE_des_sysparam. SPF is a System Parameter Functions
* 
* @param port - serial port device
* @param sysparam - struct with all des system parameters
* @param flg - if is required to print data
* 
* @return (0) if error occurs when requesting data
*/
int DES_SPF_read_all_temp_param(const int port, TYPE_des_sysparam* sysparam, char flg)
{
	int result = false;
	
	TYPE_msg_frame send_msg, received_msg;
	
	send_msg.OpCode = 0x18;//OpCode requesting velocity
	send_msg.data[0] = 0x00;
	send_msg.data[1] = 0x00;
	send_msg.data[2] = 0x00;
	
	if(flg)
		printf("\n\n\tread all parameters\n");
	
	if(send_msg_frame(port, &send_msg, flg))
	{
		result = true;
	}
	
	if(result && (read_buffer(port, &received_msg, flg)))
	{
		unsigned char* data0;
		unsigned char* data1;
		unsigned char index0, index1;
		index0 = 1; index1 = 2;
		data0 = &received_msg.data[0];
		data1 = &received_msg.data[0];
		
		sysparam->baudrate = (data0[index0]) + ((data1[index1])<<8);
		index0 += 2; index1 += 2;
		
		sysparam->sys_config = (data0[index0]) + ((data1[index1])<<8);
		index0 += 2; index1 += 2;
		
		sysparam->cur_reg_gain_p = (data0[index0]) + ((data1[index1])<<8);
		index0 += 2; index1 += 2;
		
		sysparam->max_cur_output = (data0[index0]) + ((data1[index1])<<8);
		index0 += 2; index1 += 2;
		
		sysparam->speed_reg_gain_p = (data0[index0]) + ((data1[index1])<<8);
		index0 += 2; index1 += 2;
		
		sysparam->speed_reg_gain_i = (data0[index0]) + ((data1[index1])<<8);
		index0 += 2; index1 += 2;
		
		sysparam->internal_param1 = (data0[index0]) + ((data1[index1])<<8);
		index0 += 2; index1 += 2;
		
		sysparam->internal_param2 = (data0[index0]) + ((data1[index1])<<8);
		index0 += 2; index1 += 2;
		
		sysparam->internal_param3 = (data0[index0]) + ((data1[index1])<<8);
		index0 += 2; index1 += 2;
		
		sysparam->max_speed_error = (data0[index0]) + ((data1[index1])<<8);
		index0 += 2; index1 += 2;
		
		sysparam->setting_unit_gain = (data0[index0]) + ((data1[index1])<<8);
		index0 += 2; index1 += 2;
		
		sysparam->max_speed_error = (data0[index0]) + ((data1[index1])<<8);
		index0 += 2; index1 += 2;
		
		sysparam->setting_unit_gain = (data0[index0]) + ((data1[index1])<<8);
		index0 += 2; index1 += 2;
		
		sysparam->setting_unit_offset = (data0[index0]) + ((data1[index1])<<8);
		index0 += 2; index1 += 2;
		
		sysparam->setting_unit_delay = (data0[index0]) + ((data1[index1])<<8);
		index0 += 2; index1 += 2;
		
		sysparam->peak_current = (data0[index0]) + ((data1[index1])<<8);
		index0 += 2; index1 += 2;
		
		sysparam->max_cont_current = (data0[index0]) + ((data1[index1])<<8);
		index0 += 2; index1 += 2;
		
		sysparam->max_speed = (data0[index0]) + ((data1[index1])<<8);
		index0 += 2; index1 += 2;
		
		sysparam->acceleration = (data0[index0]) + ((data1[index1])<<8);
		index0 += 2; index1 += 2;
		
		sysparam->speed_constant = (data0[index0]) + ((data1[index1])<<8);
		index0 += 2; index1 += 2;
		
		sysparam->enc_resolution = (data0[index0]) + ((data1[index1])<<8);
		index0 += 2; index1 += 2;
		
		sysparam->pole_pair_number = (data0[index0]) + ((data1[index1])<<8);
		index0 += 2; index1 += 2;
		
		sysparam->internal_param4 = (data0[index0]) + ((data1[index1])<<8);
		index0 += 2; index1 += 2;
		
		sysparam->rpm2qc_factor = (data0[index0]) + ((data1[index1])<<8);
		index0 += 2; index1 += 2;
		
		sysparam->index_offset = (data0[index0]) + ((data1[index1])<<8);
		index0 += 2; index1 += 2;
		
		sysparam->pwm_period = (data0[index0]) + ((data1[index1])<<8);
		index0 += 2; index1 += 2;
		
		sysparam->max_duty_cycle = (data0[index0]) + ((data1[index1])<<8);
		index0 += 2; index1 += 2;
		
		sysparam->cur_det_ph_u_offset = (data0[index0]) + ((data1[index1])<<8);
		index0 += 2; index1 += 2;
		
		sysparam->cur_det_ph_v_offset = (data0[index0]) + ((data1[index1])<<8);
		index0 += 2; index1 += 2;
		
		sysparam->ad_conv_offset = (data0[index0]) + ((data1[index1])<<8);
		index0 += 2; index1 += 2;
		
		sysparam->can_module_id = (data0[index0]) + ((data1[index1])<<8);
		index0 += 2; index1 += 2;
		
		sysparam->can_service_id = (data0[index0]) + ((data1[index1])<<8);
		index0 += 2; index1 += 2;
		
		sysparam->can_rx_pdo_id = (data0[index0]) + ((data1[index1])<<8);
		index0 += 2; index1 += 2;
		
		sysparam->can_tx_pdo_id = (data0[index0]) + ((data1[index1])<<8);
		index0 += 2; index1 += 2;
		
		sysparam->can_bcr1 = (data0[index0]) + ((data1[index1])<<8);
		index0 += 2; index1 += 2;
		
		sysparam->can_bcr2 = (data0[index0]) + ((data1[index1])<<8);
		index0 += 2; index1 += 2;
		
		sysparam->can_op_mode = (data0[index0]) + ((data1[index1])<<8);
		index0 += 2; index1 += 2;
		
		sysparam->can_rx_sdo_id = (data0[index0]) + ((data1[index1])<<8);
		index0 += 2; index1 += 2;
		
		sysparam->can_tx_sdo_id = (data0[index0]) + ((data1[index1])<<8);
		index0 += 2; index1 += 2;
		
		sysparam->can_rtr0_id = (data0[index0]) + ((data1[index1])<<8);
		index0 += 2; index1 += 2;
		
		sysparam->can_rtr1_id = (data0[index0]) + ((data1[index1])<<8);
		index0 += 2; index1 += 2;
		
		sysparam->can_config = (data0[index0]) + ((data1[index1])<<8);
		index0 += 2; index1 += 2;
		
		sysparam->internal_param5 = (data0[index0]) + ((data1[index1])<<8);
		index0 += 2; index1 += 2;
		
		sysparam->error_proc = (data0[index0]) + ((data1[index1])<<8);
		index0 += 2; index1 += 2;
		
		sysparam->max_speed_curr = (data0[index0]) + ((data1[index1])<<8);
		index0 += 2; index1 += 2;
		
		sysparam->hall_angle_offs = (data0[index0]) + ((data1[index1])<<8);
		index0 += 2; index1 += 2;
		
		if(flg)
		{
			printf("baudrate:0x%04x\n", sysparam->baudrate);
			printf("sys_config:0x%04x\n", sysparam->sys_config);
			printf("cur_reg_gain_p:0x%04x\n", sysparam->cur_reg_gain_p);
			printf("max_cur_output:0x%04x\n", sysparam->max_cur_output);
			printf("speed_reg_gain_p:0x%04x\n", sysparam->speed_reg_gain_p);
			printf("speed_reg_gain_i:0x%04x\n", sysparam->speed_reg_gain_i);
			printf("internal_param1:0x%04x\n", sysparam->internal_param1);
			printf("internal_param2:0x%04x\n", sysparam->internal_param2);
			printf("internal_param3:0x%04x\n", sysparam->internal_param3);
			printf("max_speed_error:0x%04x\n", sysparam->max_speed_error);
			printf("setting_unit_gain:0x%04x\n", sysparam->setting_unit_gain);
			printf("max_speed_error:0x%04x\n", sysparam->max_speed_error);
			printf("setting_unit_gain:0x%04x\n", sysparam->setting_unit_gain);
			printf("setting_unit_offset:0x%04x\n", sysparam->setting_unit_offset);
			printf("setting_unit_delay:0x%04x\n", sysparam->setting_unit_delay);
			printf("peak_current:0x%04x\n", sysparam->peak_current);
			printf("max_cont_current:0x%04x\n", sysparam->max_cont_current);
			printf("max_speed:0x%04x\n", sysparam->max_speed);
			printf("acceleration:0x%04x\n", sysparam->acceleration);
			printf("speed_constant:0x%04x\n", sysparam->speed_constant);
			printf("enc_resolution:0x%04x\n", sysparam->enc_resolution);
			printf("pole_pair_number:0x%04x\n", sysparam->pole_pair_number);
			printf("internal_param4:0x%04x\n", sysparam->internal_param4);
			printf("rpm2qc_factor:0x%04x\n", sysparam->rpm2qc_factor);
			printf("index_offset:0x%04x\n", sysparam->index_offset);
			printf("pwm_period:0x%04x\n", sysparam->pwm_period);
			printf("max_duty_cycle:0x%04x\n", sysparam->max_duty_cycle);
			printf("cur_det_ph_u_offset:0x%04x\n", sysparam->cur_det_ph_u_offset);
			printf("cur_det_ph_v_offset:0x%04x\n", sysparam->cur_det_ph_v_offset);
			printf("ad_conv_offset:0x%04x\n", sysparam->ad_conv_offset);
			printf("can_module_id:0x%04x\n", sysparam->can_module_id);
			printf("can_service_id:0x%04x\n", sysparam->can_service_id);
			printf("can_rx_pdo_id:0x%04x\n", sysparam->can_rx_pdo_id);
			printf("can_tx_pdo_id:0x%04x\n", sysparam->can_tx_pdo_id);
			printf("can_bcr1:0x%04x\n", sysparam->can_bcr1);
			printf("can_bcr2:0x%04x\n", sysparam->can_bcr2);
			printf("can_op_mode:0x%04x\n", sysparam->can_op_mode);
			printf("can_rx_sdo_id:0x%04x\n", sysparam->can_rx_sdo_id);
			printf("can_tx_sdo_id:0x%04x\n", sysparam->can_tx_sdo_id);
			printf("can_rtr0_id:0x%04x\n", sysparam->can_rtr0_id);
			printf("can_rtr1_id:0x%04x\n", sysparam->can_rtr1_id);
			printf("can_config:0x%04x\n", sysparam->can_config);
			printf("internal_param5:0x%04x\n", sysparam->internal_param5);
			printf("error_proc:0x%04x\n", sysparam->error_proc);
			printf("max_speed_curr:0x%04x\n", sysparam->max_speed_curr);
			printf("hall_angle_offs:0x%04x\n", sysparam->hall_angle_offs);
		};
	}else
	{
		result = false;
	}
	
	return result;
};

/**
* @brief Set a new velocity of the motor. This function is only available in speed mode regulation mode. please garantee that SysConfig field inside DES_SysParam struct. SF is a setting function
* 
* @param port - serial port device
* @param newVelocity - new velocity in rpm {positive value rotor turns in counter clockwise, negative value rotor turns in clockwise}
* @param flg - (1) to print debug info
* 
* @return (0) error occurs during sending  
*/
int DES_SF_set_velocity(const int port, int newVelocity, char flg)
{
	int result = false;
	
	TYPE_msg_frame send_msg;
	
	send_msg.OpCode = 0x21;//opcode set new velocity
	send_msg.data[0] = 0x00;//len-1 where len is 1;
	send_msg.data[1] = (newVelocity & 0xff);
	send_msg.data[2] = ((newVelocity >> 8) & 0xff);
	
	
	if(flg)
		printf("\n\n\tset velocity: %d\n", newVelocity);
	
	if(send_msg_frame(port, &send_msg, flg))
	{
		result = true;
	};
	
	
	return result;
};

/**
* @brief Set a new current of the motor. This function is only available in current mode regulation mode. please garantee that SysConfig field inside DES_SysParam struct. SF is a setting function
* 
* @param port - serial port device
* @param newCurrent - new current in mA {positive value rotor turns counter clockwise, negative value rotor turns in clockwise}
* @param flg - (1) to print debug info
* 
* @return (0) error occurs during sending  
*/
int DES_SF_set_current(const int port, int newCurrent, char flg)
{
	int result = false;
	
	TYPE_msg_frame send_msg;
	
	send_msg.OpCode = 0x22;//opcode set new velocity
	send_msg.data[0] = 0x00;//len-1 where len is 1;
	send_msg.data[1] = (newCurrent & 0xff);
	send_msg.data[2] = ((newCurrent >> 8) & 0xff);
	
	if(flg)
		printf("\n\n\tset current: %d\n", newCurrent);
	
	if(send_msg_frame(port, &send_msg, flg))
	{
		result = true;
	};
	
	return result;
};

/** 
* @brief This command changes the stopping state. if the motor is already stopped it will be released. Only available in speed regulation mode. SF is a setting function
*
* @note WARNING: This is a toggle function, so is required to verify if the brake is disable or not. check TYPE_DES_status_var::sys_op_status.bits.bb13
* 
* @param port - serial port connection
* @param flg - (1) to print debug info
* 
* @return (0) error occurs during sending  
*/
int DES_SF_stop_motion(const int port, char flg)
{
	int result = false;
	
	TYPE_msg_frame send_msg;
	
	send_msg.OpCode = 0x23;//opcode set new velocity
	send_msg.data[0] = 0x00;//len-1 where len is 1;
	send_msg.data[1] = 0x00;
	send_msg.data[2] = 0x00;
	
	if(flg)
		printf("\n\n\tstop motion\n");
	
	if(send_msg_frame(port, &send_msg, flg))
	{
		result = true;
	};
	
	return result;
};


/**
* @brief Read the effective and requested velocity of the motor. This is a Monitor Function (MF)
* 
* @param port - serial port connection
* @param vel_type - (0)required mean value (1)realtime values
* @param velocity - velocity is the actual velocity depending on vel_type value.
* @param requested_vel - requested velocity to the servoamplifier
* @param flg - (1) to print debug info
* 
* @return - (0) if error occurs when requesting message
*/
int DES_MF_read_velocity_is_must(const int port, int vel_type, int* velocity, int* requested_vel, char flg)
{
	int result = false;
	
	TYPE_msg_frame send_msg, received_msg;
	
	send_msg.OpCode = 0x28;//OpCode requesting velocity
	send_msg.data[0] = 0x00;
	send_msg.data[1] = (vel_type & 0xff);
	send_msg.data[2] = ((vel_type >> 8) & 0xff);
	
	if(flg)
		printf("\n\n\trequesting velocity\n");
	
	if(send_msg_frame(port, &send_msg, flg))
	{
		result = true;
	}
	if(result && (read_buffer(port, &received_msg, flg)))
	{
		*velocity = received_msg.data[1/*received_msg.size-3*/] | ((char)received_msg.data[2/*received_msg.size-4*/]<<8);
		*requested_vel = received_msg.data[3/*received_msg.size-1*/] | (received_msg.data[4/*received_msg.size-2*/]<<8);
	}else
	{
		result = false;
		printf("LIB_DES: reading velocity success:%s\n", (result==false?"false":"true"));
	}
	return result;
}


/** 
* @brief Function that initializes DES servoamplifier with a start configuration. also enables the servoamplifier to be used by software and garantees that the electric brake is not active
* 
* @param port - serial port conection
* @param newSysParam - DES start configuration, although there is 45 start configuration variable is just defined SYS_CONFIG, ENC_RESOLUTION, MAX_SPEED, MAX_CONT_CURRENT, PEAK_CURRENT, SPEED_REG_GAIN_P and SPEED_REG_GAIN_I
* @param enable_des - 
* @param brake_des - 
* @param flg - if is required to print out some infomration
* 
* @return (-1) no errors, (0) brake not disabled or servoamplifier didn't not enabled, (1...) error according Standard Error Messages
*/
int InitDES(const int port, TYPE_des_sysparam* newSysParam, char enable_des, char brake_des, char flg)
{
	int result = 1;
	int des_err;
	TYPE_DES_status_var aDES_status_var;//DES system status
	TYPE_des_sysparam aDES_sysparam;//system params defined at DES
	int enable = 1;
	c_timer timer0;
	
	//printf("des_sys_config:%d\n", newSysParam->sys_config);
	//printf("des_enc_res:%d\n", newSysParam->enc_resolution);
	//printf("des_max_rpm:%d\n", newSysParam->max_speed);
	//printf("max_cont_current:%d\n", newSysParam->max_cont_current);
	//printf("des_peak_current:%d\n", newSysParam->peak_current);
	//printf("des_speed_gain_p:%d\n", newSysParam->speed_reg_gain_p);
	//printf("des_speed_gain_i:%d\n", newSysParam->speed_reg_gain_i);
	
	//configure DES servoamplifier with required config
	if(!DES_SPF_set_temp_param(port, SYS_CONFIG, 0, &newSysParam->sys_config, flg))
	{
		result=0;
	}
	
	if(result)
		result=DES_SPF_set_temp_param(port, ENC_RESOLUTION, 0, &newSysParam->enc_resolution, flg);
	
	if(result)
		result=DES_SPF_set_temp_param(port, MAX_SPEED, 0, &newSysParam->max_speed, flg);
	
	if(result)
		result=DES_SPF_set_temp_param(port, MAX_CONT_CURRENT, 0, &newSysParam->max_cont_current, flg);
	
	if(result)
		result=DES_SPF_set_temp_param(port, PEAK_CURRENT, 0, &newSysParam->peak_current, flg);
	
	if(result)
		result=DES_SPF_set_temp_param(port, SPEED_REG_GAIN_P, 0, &newSysParam->speed_reg_gain_p, flg);
	
	if(result)
		result=DES_SPF_set_temp_param(port, SPEED_REG_GAIN_I, 0, &newSysParam->speed_reg_gain_i, flg);
	
	
	char finished = false;
	if(result)
	{
		char f1,f2;
		f1=false;
		f1=false;
		//verify if system is in stop motion and if servoamplier is software disable and toggle them
		timer0.tic(0);
		while(!finished)
		{
			result = DES_ST_read_sys_status(port, &aDES_status_var.sys_op_status, flg);
			
			//printf("aDES_status_var.sys_op_status:%d\n", aDES_status_var.sys_op_status);
			if((aDES_status_var.sys_op_status & 0b0010000000000000) && brake_des)
			{
				DES_SF_stop_motion(port, flg);
			}else
				f1 = true;
			
			if(!(aDES_status_var.sys_op_status & 0b0000010000000000) && enable_des)
			{
				DES_ST_enable(port, &enable, flg);
			}else
				f2 = true;
			timer0.toc(0);
			if(timer0.get_toc(0)>1.5)
			{
				//was not possible to activate servoamplifier
				result = 0;
				finished = true;
			}else
			{
				usleep(10000);
			};
			finished = (f1 && f2);
		};
	};
	
	//check if there are errors
	if((aDES_status_var.sys_op_status & 0b0000001000000000) && result)
	{
		timer0.tic(1);
		finished = false;
		while(!finished)
		{
			DES_ST_read_error(port, &des_err, flg);
			result = (0b0010000111111111 & des_err); //result will depend on how many errors exists
			if(des_err & 0b1000000000000000)
			{
				DES_ST_clear_errors(port, flg);
				finished = true;
			}
		};
	}
	
	if(flg)
	{
		DES_SPF_read_temp_param(port, SYS_CONFIG, 0, &aDES_sysparam.sys_config, 1);
		printf("requested sys_config:0x%04x\n", newSysParam->sys_config);
		printf("specified at DES sys_config:0x%04x\n", aDES_sysparam.sys_config);
		
		if(des_err & 0b0000000000000001)
			printf("Error during offset detection\n");
		if(des_err & 0b0000000000000010)
			printf("Index processing error\n");
		if(des_err & 0b0000000000000100)
			printf("Index not received\n");
		if(des_err & 0b0000000000001000)
			printf("Error by initial position detection\n");
		if(des_err & 0b0000000000010000)
			printf("Over-Current Error\n");
		if(des_err & 0b0000000000100000)
			printf("Over-Voltage Error\n");
		if(des_err & 0b0000000001000000)
			printf("Over-Speed in current control mode\n");
		if(des_err & 0b0000000010000000)
			printf("Supply voltage too low for operation\n");
		if(des_err & 0b0000000100000000)
			printf("Angle Detection error\n");
		if(des_err & 0b0010000000000000)
			printf("Parameter out of range\n");
	};
	
	return result;
};


int stopDES(const int port, char flg)
{
	int result = false;
	c_timer timer0;
	TYPE_DES_status_var aDES_status_var;//DES system status
	char finished = false;
	int enable = 0;
	char f1 = false;
	char f2 = true;
	
	
	timer0.tic(0);
	
	while(!finished)
	{
		
		DES_SF_set_velocity(port, 0, flg);
		
		
		if(!DES_ST_read_sys_status(port, &aDES_status_var.sys_op_status, flg))
		{
			aDES_status_var.sys_op_status = 0b0010000000000000;
		}
		
		
		
		if(!(aDES_status_var.sys_op_status & 0b0010000000000000))
		{
			DES_SF_stop_motion(port, flg);
			f1 = true;
		}
		
		
		DES_ST_enable(port, &enable, flg);
		
		
		finished = (f1 && f2);
	}
	return result;
	
};


int DES_set_new_baud(int* port, int baud, char flg)
{
	int result = true;
	struct termios newParams;
	int ret;
	
	bzero(&newParams, sizeof(newParams));
	
	
	printf("Set new Baudrate for DES 70/10..."); fflush(stdout);
	
	DES_SPF_set_temp_param(*port, BAUDRATE, 0, &baud, flg);
	
	newParams.c_cflag = (B115200 | CS8 | CLOCAL | CREAD);
	ret = tcsetattr(*port, TCSANOW, &newParams);
	
	if(ret < 0)
	{
		perror("Set serial communication parameters failed:");
		result = false;
	};
	
	return result;
};
