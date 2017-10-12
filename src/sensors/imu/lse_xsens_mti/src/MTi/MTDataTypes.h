/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, ISR University of Coimbra.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the ISR University of Coimbra nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Gonçalo Cabrita
* Notes: Original code in Cocoa from 07/08/2009, went C++ on 10/11/2010
*********************************************************************/

// MTComm message
// PRE BID MID LEN DATA CHECKSUM

//Extended MTComm message
// PRE BID MID LEN EXT_LEN DATA CHECKSUM

namespace Xsens
{
	typedef enum _MTMessageIdentifier {
	
		// WakeUp + State messages
		WakeUp = 62,
		WakeUpAck = 63,
		GoToConfig = 48,
		GoToConfigAck = 49,
		GoToMeasurement = 16,
		GoToMeasurementAck = 17,
		Reset = 64,
		ResetAck = 65,
	
		// Informational messages
		ReqDID = 0,
		DeviceID = 1,
		InitMT = 2,
		InitMTResults = 3,
		ReqProductCode = 28,
		ProductCode = 29,
		ReqFWRev = 18,
		FirmwareRev = 19,
		ReqDataLength = 10,
		DataLength = 11,
		Error = 66,
		ReqGPSStatus = 166,				// Only supported by MTi-G
		GPSStatus = 167,				// Only supported by MTi-G
	
		// Device-specific messages
		ReqBaurate = 24,
		ReqBaurateAck = 25,
		SetBaudrate = 24,
		SeqBaurateAck = 25,
		ReqErrorMode = 218,
		ReqErrorModeAck = 219,
		SetErrorMode = 218,
		SetErrorModeAck = 219,
		ReqLocationID = 132,
		ReqLocationIDAck = 133,
		SetLocationID = 132,
		SetLocationIDAck = 133,
		RestoreFactoryDef = 14,
		RestoreFactoryDefAck = 15,
		ReqTransmitDelay = 220,
		ReqTransmitDelayAck = 221,
		SetTransmitDelay = 220,
		SetTransmitDelayAck = 221,
	
		// Synchronization messages
		ReqSyncInSettings = 214,
		ReqSyncInSettingsAck = 215,
		SetSyncInSettings = 214,
		SetSyncInSettingsAck = 215,
		ReqSyncOutSettings = 216,
		ReqSyncOutSettingsAck = 217,
		SetSyncOutSettings = 216,
		SetSyncOutSettingsAck = 217,
	
		// Configuration messages
		ReqConfiguration = 12,
		Configuration = 13,
		ReqPeriod = 4,
		ReqPeriodAck = 5,
		SetPeriod = 4,
		SetPeriodAck = 5,
		ReqOutputSkipFactor = 212,
		ReqOutputSkipFactorAck = 213,
		SetOutputSkipFactor = 212,
		SetOutputSkipFactorAck = 213,
		ReqObjectAlignment = 224,
		ReqObjectAlignmentAck = 225,
		SetObjectAlignment = 224,
		SetObjectAlignmentAck = 225,
		ReqOutputMode = 208,
		ReqOutputModeAck = 209,
		SetOutputMode = 208,
		SetOutputModeAck = 209,
		ReqOutputSettings = 210,
		ReqOutputSettingsAck = 211,
		SetOutputSettings = 210,
		SetOutputSettingsAck = 211,
	
		// Data-related messages
		ReqData = 52,
		MTData = 50,
	
		// XKF Filter messages
		ReqHeading = 130,
		ReqHeadingAck = 131,
		SetHeading = 130,
		SetHeadingAck = 131,
		ResetOrientation = 164,
		ResetOrientationAck = 165,
		ReqUTCTime = 96,					// Only supported by MTi-G
		UTCTime = 97,						// Only supported by MTi-G
		ReqMagneticDeclination = 106,
		ReqMagneticDeclinationAck = 107,
		SetMagneticDeclination = 106,
		SetMagneticDeclinationAck = 107,
		ReqAvailableScenarios = 98,
		AvailableScenarios = 99,
		ReqCurrentScenario = 100,
		ReqCurrentScenarioAck = 101,
		SetCurrentScenario = 100,
		SetCurrentScenarioAck = 101,
		ReqGravityMagnitude = 102,
		ReqGravityMagnitudeAck = 103,
		SetGravityMagnitude = 102,
		SetGravityMagnitudeAck = 103,
		ReqProcessingFlags = 32,
		ReqProcessingFlagsAck = 33,
		SetProcessingFlags = 32,
		SetProcessingFlagsAck = 33,
		ReqLeverArmGps = 104,
		ReqLeverArmGpsAck = 105,
		SetLeverArmGps = 104,
		SetLeverArmGpsAck = 105,
		SetNoRotation = 34,
		SetNoRotationAck = 35,
		
	} MTMessageIdentifier;

	typedef enum _MTOrientationMode {
	
		Quaternion = 0,
		EulerAngles = 1,
		Matrix = 2,
	
	} MTOrientationMode;
}

// EOF


