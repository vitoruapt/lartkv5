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
 * \brief Des70-10 Communication library header
 */

#include <termios.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <termio.h>
#include <fcntl.h>
#include <string.h>
#include <stdio.h>
#include <atlasmv_base/crc.h>
#include <atlasmv_base/timer.h>

/** 
* @brief data struct with all DES System Parameters definition, all fields have a length of 16bits but the variable range can change normally [0, 32767]
*/
typedef struct{
	///ParNb 0: R/W; 0=9600; 1=14400; 2=19200;3=38400; 4=57600; 5=115200 baud range=[0, 1, 2, 3, 4, 5]
	int baudrate;
	
	///ParNb 1: R/W; System Configuration (0bxxxxxxxxxxxxxxxx)<br>b0: 0-speed/current setting by software | 1-speed/current defined by analog input "Set Value"<br>b1: 0-acceleration enabled | 1-acceleration disabled<br>b2: 0-depending of b3 | 1-current regulator<br>b3: 0-speed regulator | 1-reserved<br>b4: 0:speed monitor signal | 1-torque setting monitor signal<br>b5: do not change<br>b6: not used<br>b7: 0-stop motor by digital input 'STOP' | 1-stop motor by software (command 'stop_motion')<br>b8: 0-set max speed by potentiometer 'P1' | 1-set max speed by software (MAX_SPEED && max_speed)<br>b9: 0-set offset by potentiometer 'P2' | 1-set offset by software (OFFSET_OF_SETTING_UNIT && setting_unit_offset)<br>b10: 0-set max current by potentiometer 'P3' | 1-set max current by software (peak_current && max_cont_current)<br>b11: 0-Set the regulation <br>b12:<br>b13:<br>b14:<br>b15:<br>
	int sys_config;
	///ParNb 2: R/W; Current regulation P-gain
	int cur_reg_gain_p;
	///ParNb 3: R/W; Current regulation I-gain
	int cur_reg_gain_i;
	///ParNb 4: R/W; Max output of current regulator (just in service mode)
	int max_cur_output;
	///ParNb 5: R/W; Speed Regulator P-gain
	int speed_reg_gain_p;
	///ParNb 6: R/W; Speed Regulator I-gain
	int speed_reg_gain_i;
	///ParNb 7: R/W; do not change
	int internal_param1;
	///ParNb 8: R/W; do not change
	int internal_param2;
	///ParNb 9: R/W; do not change
	int internal_param3;
	///ParNb 10: R/W; Limitation of speed error for the input of the speed regulator (just in service mode)
	int max_speed_error;
	///ParNb 11: R/W; gain of setting unit
	int setting_unit_gain;
	///ParNb 12: R/W; offset of setting unit range=[-100, 100]
	int setting_unit_offset;
	///ParNb 13: R/W; Delay of setting unit (just in servide mode)
	int setting_unit_delay;
	///ParNb 14: R/W; Peak current in mA range=[1, 30000] mA
	int peak_current;
	///ParNb 15: R/W; Max continuous current range=[1, 10000] mA
	int max_cont_current;
	///ParNb 16: R/W; Thermal constant (just in service mode)
	int therm_const;
	///ParNb 17: R/W; Maximum Speed range=[0, 25000] rpm
	int max_speed;
	///ParNb 18: R/W; Acceleration rpm/ms²
	int acceleration;
	///ParNb 19: R/W; Speed Constant of motor (just in service mode)
	int speed_constant;
	///ParNb 20: R/W; Encoder resolution in counts/turn
	int enc_resolution;
	///ParNb 21: R/W; Number of pole pair range=[1, 64]
	int pole_pair_number;
	///ParNb 22: R/W; Internally used, do not change (just in service mode)
	int internal_param4;
	///ParNb 23: R/W; Conversion factor from rpm to qc/ms (just in service mode)
	int rpm2qc_factor;
	///ParNb 24: R/W; Angular Offset of Index Pulse range=[-32768, 32767](just in service mode)
	int index_offset;
	///ParNb 25: R; PWM period in clock
	int pwm_period;
	///ParNb 26: R/W; Max Duty Cycle (just in service mode)
	int max_duty_cycle;
	///ParNb 27: R/W; Offset of (phase U) current detection range=[0, 65535] (just in service mode)
	int cur_det_ph_u_offset;
	///ParNb 28: R/W; Offset of (phase V) current detection range=[0, 65535] (just in service mode)
	int cur_det_ph_v_offset;
	///ParNb 29: R/W; Offset of general AD converter range=[0, 65535] (just in service mode)
	int ad_conv_offset;
	///ParNb 30: R; CAN module ID range=[1, 127]
	int can_module_id;
	///ParNb 31: R; CAN service ID = CAN module ID range=[1, 127]
	int can_service_id;
	///ParNb 32: R; CAN Receive PDO ID range=[385, 1407]
	int can_rx_pdo_id;
	///ParNb 33: R; CAN Transmit PDO ID range=[385, 1407]
	int can_tx_pdo_id;
	///ParNb 34: R; CAN BCR1 range=[0, 65535]
	int can_bcr1;
	///ParNb 35: R; CAN BCR2 range=[0, 65535]
	int can_bcr2;
	///ParNb 36: R; CAN operation mode range=[0, 65535]
	int can_op_mode;
	///ParNb 37: R; CAN Receive SDO ID = 1536 + moduleID range=[1537, 1663] 
	int can_rx_sdo_id;
	///ParNb 38: R; CAN Transmit SDO ID = 1408 + moduleID range=[1409, 1535]
	int can_tx_sdo_id;
	///ParNb 39: R; Remote Transmission Request ID(channel0) range=[385, 1407]
	int can_rtr0_id;
	///ParNb 40: R; Remote Transmission Request ID(channel1) range=[385, 1407]
	int can_rtr1_id;
	///ParNb 41: R; CAN communication configuration register
	int can_config;
	///ParNb 42: R; Internally used, do not change
	int internal_param5;
	///ParNb 43: R/W; Error Reaction Procedure range=[0 - disable, 1-stop]
	int error_proc;
	///ParNb 44: R/W; Maximal Speed in regulation mode
	int max_speed_curr;
	///ParNb 45: R; Angular offset of hall sensor signals range=[-32768, 32767] (just in service mode)
	int hall_angle_offs;
}TYPE_des_sysparam;

/** 
* @brief enumerator connected with TYPE_des_sysparam and is used when comunication is required with DES servoamplifier
*/
enum ENUM_DES_SYS_PARAMS
{
	BAUDRATE = 0, 
	SYS_CONFIG = 1,
	CUR_REG_GAIN_P = 2,
	CUR_REG_GAIN_I = 3,
	MAX_CUR_OUTPUT = 4,
	SPEED_REG_GAIN_P = 5,
	SPEED_REG_GAIN_I = 6,
	INTERNAL_PARAM1 = 7,
	INTERNAL_PARAM2 = 8,
	INTERNAL_PARAM3 = 9,
	MAX_SPEED_ERROR = 10,
	SETTING_UNIT_GAIN = 11,
	SETTING_UNIT_OFFSET = 12,
	SETTING_UNIT_DELAY = 13,
	PEAK_CURRENT = 14,
	MAX_CONT_CURRENT = 15,
	THERM_CONST = 16,
	MAX_SPEED = 17,
	ACCELERATION = 18,
	SPEED_CONSTANT = 19,
	ENC_RESOLUTION = 20,
	POLE_PAIR_NUMBER = 21,
	INTERNAL_PARAM4 = 22,
	RPM_2QC_FACTOR = 23,
	INDEX_OFFSET = 24,
	PWM_PERIOD = 25,
	MAX_DUTY_CYCLE = 26,
	CUR_DET_PH_U_OFFSET = 27,
	CUR_DET_PH_V_OFFSET = 28,
	AD_CONV_OFFSET = 29,
	CAN_MODULE_ID = 30,
	CAN_SERVICE_ID = 31,
	CAN_RX_PDO_ID = 32,
	CAN_TX_PDO_ID = 33,
	CAN_BCR1 = 34,
	CAN_BCR2 = 35,
	CAN_OP_MODE = 36,
	CAN_RX_SDO_ID = 37,
	CAN_TX_SDO_ID = 38,
	CAN_RTR0_ID = 39,
	CAN_RTR1_ID = 40,
	CAN_CONFIG = 41,
	INTERNAL_PARAM5 = 42,
	ERROR_PROC = 43,
	MAX_SPEED_CURR = 44,
	HALL_ANGLE_OFFS = 45
};

/** 
* @brief DES data structure with all status variables, all variables are 16bits except Nb136 that is 32bits (double word)
*/
typedef struct{
	///Nb128: System Operation Status<br>b0: 0-encoder index not found yet | 1-encoder index found<br>b1: 0-hall sensor signal not found yet | 1-hall sensor signal found<br>b2: 0-rotor position not found yet | 1-rotor position found<br>b3: 0-not saving the system parameters in EEPROM | 1-saving the system parameters in EEPROM<br>b4: not used<br>b5: reserved<br>b6: reserved<br>b7: 0-Max Current Set to peak current | 1-Max current reduced to continous current<br>b8: 0-in the small current region | 1-in the large current region<br>b9: 0-no error | 1-error<br>b10: 0-software disabled | 1-software enabled<br>b11: 0-not debouncing the enable input | 1-debouncing the enable input<br>b12: 0-no offset in current circuit detected | 1-offsets in current circuit detected<br>b13: 0-not braking | 1-braking with maximum setting current<br>b14+15:<br>(0+0)-power stage is disabled<br>(0+1)-refresh the power stage<br>(1+0)-power stage is enabled<br>(1+1)-power stage is enabled<br>
	int sys_op_status;
	///Nb129: Actual mean current value in d-axis [mA]
	int actual_mean_cur_d;
	///Nb130: Actual mean current value in q-axis (torque) [mA]
	int actual_mean_cur_q;
	///Nb131: Current Setting Value [mA]
	int cur_setting_val;
	///Nb132: Relative rotor position in a revolution [qc]
	int rel_rot_pos_rev;
	///Nb133: Speed setting value [rpm]
	int speed_setting_val;
	///Nb134: Actual mean speed value [rpm]
	int actual_mean_speed_val;
	///Nb136: Absolute rotor position (32bits of size) [qc]
	int abs_rotor_pos;
	///Nb137: standard error
	int standard_error;
	///Nb 138: CAN error
	int can_error;
	///Nb139: Actual current value in q-axis (Torque not averaged) [mA]
	int actual_cur_q;
	///Nb140: Actual Speed Value (not averaged) [rpm]
	int actual_speed_val;
	///Nb141: Error History 1, 
	int error_hist1;
	///Nb142: Error History 2
	int error_hist2;
	///Nb143: Encoder Counter [qc]
	int enc_counter;
	///Nb144: Encoder Counter at last index [qc]
	int enc_counter_last_ind;
	///Nb145: Hall sensor pattern
	int hall_sens_pat;
}TYPE_DES_status_var;

/** 
* @brief enumerator connected with DES status var typedefinition, and is used when communicating with DES servo amplifier
*/
enum ENUM_DES_STATUS_VAR
{
	SYS_OP_STATUS = 128,
	ACTUAL_MEAN_CUR_D = 129,
	ACTUAL_MEAN_CUR_Q = 130,
	CUR_SETTING_VAL = 131,
	REL_ROT_POS_REV = 132,
	SPEED_SETTING_VAL = 133,
	ACTUAL_MEAN_SPEED_VAL = 134,
	ABS_ROTOR_POS = 135,
	STANDARD_ERROR = 136,
	CAN_ERROR = 137,
	ACTUAL_CUR_Q = 138,
	ACTUAL_SPEED_VAL = 139,
	ERROR_HIST1 = 140,
	ERROR_HIST2 = 141,
	ENC_COUNTER = 142,
	ENC_COUNTER_LAST_IND = 143,
	HALL_SENS_PAT = 144,
};

/** 
* @brief struct with message frame
*/
typedef struct{
	///instruction command, according DES command reference
	unsigned char OpCode;
	///array with all the bytes to be sent by serial port, except OpCode. data[size,size+1]=crc
	unsigned char data[520];
	///size - var with the number of bytes to be sent by serial port data[n]={data[0]=len-1|data[1]...data[n]}. So size=(data[0]+1)*2+1
	int size;
	///msg crc get according CCITT algorithm
	int crc;
}TYPE_msg_frame;



//####################################################################
// Prototypes:	for private functions can be declared #################
// in des70_10.c or preferably in des70_10_functions.c  ##
//####################################################################



//-------	des70_10_init.cpp		-------
int InitDES_communication(std::string com_device, int* port);

//-------	des70_10_functions.cpp	-------
int calc_crc_16(TYPE_msg_frame* msg);
int write_msg_inbuf(const int port, unsigned char* data, int size);
int check_msg_aceptance(const int port, float wait_time, unsigned char* msg);
int send_OpCode(const int port, unsigned char tries, unsigned char OpCode, char flg);
int send_data(const int port, TYPE_msg_frame* msg, char flg);
int read_buffer(const int port, TYPE_msg_frame* msg, char flg);
int send_msg_frame(const int port, TYPE_msg_frame* msg, char flg);
int DES_ST_read_sys_status(const int port, int* sys_status, char flg);
int DES_ST_read_error(const int port, int* errors, char flg);
int DES_ST_clear_errors(const int port, char flg);
int DES_ST_reset(const int port, char flg);
int DES_ST_enable(const int port, int* newState, char flg);
int DES_SPF_read_temp_param(const int port, int paramNb, int dataFormat, int* response, char flg);
int DES_SPF_set_temp_param(const int port, int paramNb, int dataFormat, int* newValue, char flg);
int DES_SPF_read_all_temp_param(const int port, TYPE_des_sysparam* sysparam, char flg);
int DES_SF_set_velocity(const int port, int newVelocity, char flg);
int DES_SF_set_current(const int port, int newCurrent, char flg);
int DES_SF_stop_motion(const int port, char flg);
int DES_MF_read_velocity_is_must(const int port, int vel_type, int* velocity, int* requested_vel, char flg);
int InitDES(const int port, TYPE_des_sysparam* newSysParam, char, char, char flg);
int stopDES(const int port, char flg);
int DES_set_new_baud(int* port, int baud, char flg);
