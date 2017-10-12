/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, ISR University of Coimbra.
*  Copyright (c) 2011-2012, INRIA, CNRS, all rights reserved
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
*
* Author: Nicolas Vignard on 2 MAY 2012
* Notes: Add the ability to control the Mti-G
*********************************************************************/
#include "MTi.h"
#include <stdio.h>
#include <stdlib.h>

#include <ros/ros.h>
#include <netinet/in.h>

// Small workaround for the hexa to float problem...
float hexa2float(unsigned char * buffer)
{
    union
    {
        float value;
        unsigned char buffer[4];

    }floatUnion;

    floatUnion.buffer[0] = buffer[3];
    floatUnion.buffer[1] = buffer[2];
    floatUnion.buffer[2] = buffer[1];
    floatUnion.buffer[3] = buffer[0];

    return floatUnion.value;
}

void float2hexa(float valuetoSwap, std::vector<unsigned char>& buffer )
{
    union
    {
        float value;
        unsigned char buffer[4];

    }floatUnion;
    floatUnion.value = valuetoSwap;

    buffer.push_back(floatUnion.buffer[3]);
    buffer.push_back(floatUnion.buffer[2]);
    buffer.push_back(floatUnion.buffer[1]);
    buffer.push_back(floatUnion.buffer[0]);
}

int hexa2int(unsigned char * buffer)
{
    union
    {
        unsigned int value;
        unsigned char buffer[4];

    }floatUnion;

    floatUnion.buffer[0] = buffer[3];
    floatUnion.buffer[1] = buffer[2];
    floatUnion.buffer[2] = buffer[1];
    floatUnion.buffer[3] = buffer[0];

    return floatUnion.value;
}


/**
 * @brief
 * Constructor
 */
Xsens::MTi::MTi() : serial_port()
{
    numOfBytes = 4;

    // Serial Port Settings
    this->resetPackage();

    // Queue Settings
    queueIsWaiting = false;
    queueIsRunning = false;

    accX = accY = accZ = 0.0;
    gyrX = gyrY = gyrZ = 0.0;
    magX = magY = magZ = 0.0;
    q0 = q1 = q2 = q3 = 0.0;
    eroll = epitch = eyaw = 0.0;
    mTemperature = 0.0;
    ts = 0;


    resetGPSValues();
    mVelocityX = mVelocityY = mVelocityZ = 0.0;
    mStatus = 0;
    mHorizontalAccuracy = mVerticalAccuracy = 0;

    mInitialPosition.x = 0.0;
    mInitialPosition.y = 0.0;
    mInitialPosition.z = 0.0;

}

/**
 * @brief
 * Destructor
 */
Xsens::MTi::~MTi()
{
    if(serial_port.portOpen()) this->closePort();
}

/**
 * @brief
 * Check if the xsens model is a Mti-G
 *
 * @return bool
 */
bool Xsens::MTi::isMtiG()
{
    return (mDeviceID & 0xFFF00000) == 0x00500000;
}

/**
 * @brief
 * Check if the power‐up self test completed successfully
 * @return bool
 */
bool Xsens::MTi::isSelfTestCompleted()
{
    if(isMtiG())
    {
        return mStatus & SELF_TEST;
    }
    else
        return true;
}

/**
 * @brief
 * Set the configuration of the xsens:
 *  - outputMode
 *  - outputSettings
 *  - frameID of the IMU
 *
 *  - gps_arm (double, double, double) (MTI-G ONLY)
 *      - Default (0,0,0)
 *      - Specifies the x, y and z offsets of the gps antenna with respect to the imu
 * @param mode
 * @param settings
 * @param timeout
 * @return bool
 */
bool Xsens::MTi::setSettings(outputMode mode, outputSettings settings, Scenario scenario, const std::string& rosNamespace, const std::string& frameID, const Position& GPSLeverArm, int timeout)
{
    mRosNamespace =  rosNamespace.empty() == true ? "" : "/" + rosNamespace;
    mFrameID = mRosNamespace + frameID;
    this->outputModeData.clear();
    this->outputSettingsData.clear();

    unsigned char byte;
    unsigned short modeIn2Bytes = 0;

    // Go into config mode, set our preferences and go back to measuring
    addMessageToQueue(GoToConfig, NULL, GoToConfigAck);
    addMessageToQueue(ReqDID,NULL,DeviceID);
    waitForQueueToFinish(timeout);


    modeIn2Bytes =
            mode.temperatureData
            | mode.calibratedData<<1
                                   | mode.orientationData<<2
                                   |  mode.auxiliaryData<<3
                                   | mode.positionData<<4
                                   | mode.velocityData<<5
                                   | mode.statusData<<11
                                   | mode.rawGPSData<<12
                                   | mode.rawInertialData<<14;

    if (!isMtiG()) {
        // not an MTi-G, remove all GPS related stuff
        modeIn2Bytes &= 0x4007; //0x0FFF
        // CMT_OUTPUTMODE_POSITION = 0x001
        ROS_INFO("No GPS Position available : MTi only, %x", mDeviceID);
    }
    else
        ROS_INFO("MTi-G detected");


    byte = (modeIn2Bytes & 0xFF00) >> 8;
    outputModeData.push_back(byte);
    byte = (modeIn2Bytes & 0xFF);
    outputModeData.push_back(byte);


    unsigned int modeIn4Bytes = 0;
    modeIn4Bytes = settings.timeStamp
            | settings.orientationMode<<2
                                        | (!settings.enableAcceleration)<<4
                                        | (!settings.enableRateOfTurn)<<5
                                        | (!settings.enableMagnetometer)<<6
                                        | settings.velocityModeNED<<31;

    if (!isMtiG()) {
        // not an MTi-G, remove all GPS related stuff
        modeIn4Bytes &= 0x0000037F;
    }


    byte = (modeIn4Bytes & 0xFF000000) >> 24;
    outputSettingsData.push_back(byte);
    byte = (modeIn4Bytes & 0x00FF0000) >> 16;
    outputSettingsData.push_back(byte);
    byte = (modeIn4Bytes & 0x0000FF00) >> 8;
    outputSettingsData.push_back(byte);
    byte = (modeIn4Bytes & 0x000000FF);
    outputSettingsData.push_back(byte);


    this->addMessageToQueue(SetOutputMode, &outputModeData, SetOutputModeAck);
    this->addMessageToQueue(SetOutputSettings, &outputSettingsData, SetOutputSettingsAck);

    if(isMtiG())
    {
        modeIn2Bytes = scenario;
        byte = 0;
        byte = (modeIn2Bytes & 0xFF00) >> 8;
        mScenarioData.push_back(byte);
        byte = (modeIn2Bytes & 0x00FF);
        mScenarioData.push_back(byte);

        this->addMessageToQueue(SetCurrentScenario, &mScenarioData, SetCurrentScenarioAck);
        this->addMessageToQueue(ReqAvailableScenarios, NULL, AvailableScenarios);
        this->addMessageToQueue(ReqCurrentScenario, NULL, ReqCurrentScenarioAck);

        std::vector<unsigned char> GPSLeverArmVector;

        float2hexa(GPSLeverArm.x,GPSLeverArmVector);//X
        float2hexa(GPSLeverArm.y,GPSLeverArmVector);//Y
        float2hexa(GPSLeverArm.z,GPSLeverArmVector);//Z
        this->addMessageToQueue(SetLeverArmGps, &GPSLeverArmVector, SetLeverArmGpsAck);
        this->addMessageToQueue(ReqLeverArmGps, NULL, ReqLeverArmGpsAck);
        this->addMessageToQueue(ReqCurrentScenario, NULL, ReqCurrentScenarioAck);
    }


    this->addMessageToQueue(ReqOutputMode, NULL, ReqOutputModeAck);
    this->addMessageToQueue(ReqOutputSettings, NULL, ReqOutputSettingsAck);

    this->addMessageToQueue(GoToMeasurement, NULL, GoToMeasurementAck);
    bool result =  this->waitForQueueToFinish(timeout);


    /*  ROS_INFO("auxiliaryData: %d,calibratedData: %d,orientationData: %d,positionData: %d",output_mode.auxiliaryData,output_mode.calibratedData,output_mode.orientationData,output_mode.positionData);
    ROS_INFO("rawGPSData: %d,rawInertialData: %d,statusData: %d,temperatureData: %d,velocityData: %d",output_mode.rawGPSData,output_mode.rawInertialData,output_mode.statusData,output_mode.temperatureData,output_mode.velocityData);
    ROS_INFO("timeStamp: %d,orientationMode: %d",output_settings.timeStamp,output_settings.orientationMode);
    ROS_INFO("enableAcceleration: %d,enableMagnetometer: %d, enableRateOfTurn: %d",output_settings.enableAcceleration,output_settings.enableMagnetometer,output_settings.enableRateOfTurn);
    ROS_INFO("velocityModeNED: %d",output_settings.velocityModeNED);*/
    return result;
}

/**
 * @brief
 * Reset data received
 */
void Xsens::MTi::resetPackage()
{
    packageInTransit = false;
    packageLength = 0;
    packageIndex = 0;
    package.clear();
}

/**
 * @brief
 * Add a message to the queue in order to send it after
 * @param messageID XSens ID of the message
 * @param data Data corresponding to the Xsens ID
 * @param ack Xsens ID received when the message is sent (See Xsens Manual)
 */
void Xsens::MTi::addMessageToQueue(MTMessageIdentifier messageID, std::vector<unsigned char> * data, MTMessageIdentifier ack)
{
    this->queue.push_back(MTMessage(messageID, data, ack));
    if(queueIsRunning == false) this->manageQueue();
}

/**
 * @brief
 * Manage the queue in order to send messages
 */
void Xsens::MTi::manageQueue()
{
    queueIsRunning = true;
    if(queueIsWaiting == true)
    {
        this->queue.erase(queue.begin());
        queueIsWaiting = false;

        if(this->queue.size() == 0) queueIsRunning = false;
    }
    if(queueIsWaiting == false && queueIsRunning == true)
    {
        MTMessage message2send = this->queue[0];
        std::vector<unsigned char> data2send;
        this->makeMessage(message2send.getMessageID(), message2send.getData(), &data2send);
        queueAck = message2send.getMessageAck();
        queueIsWaiting = true;
        this->serialPortSendData(&data2send);
    }
}

/**
 * @brief
 * Wait all the messages queued to be sent
 * @param timeout
 * @return bool
 */
bool Xsens::MTi::waitForQueueToFinish(int timeout)
{
    for(int i=0 ; i<timeout ; i++)
    {
        usleep(1000);	// sleep for 1ms
        if(queueIsRunning == false) return true;
    }

    queue.clear();
    queueIsWaiting = false;
    queueIsRunning = false;

    this->resetPackage();

    return false;
}

/**
 * @brief
 * Open serial port to communicate with the Xsens
 * @param name  serial port name
 * @param baudrate
 * @return bool
 */
bool Xsens::MTi::openPort(char * name, int baudrate)
{
    try{ serial_port.open(name, baudrate); }
    catch(cereal::Exception& e)
    {
        return false;
    }
    return serial_port.startReadStream(boost::bind(&Xsens::MTi::serialPortReadData, this, _1, _2));
}

/**
 * @brief
 * Close the serial port
 * @return bool
 */
bool Xsens::MTi::closePort()
{
    try{ serial_port.close(); }
    catch(cereal::Exception& e)
    {
        return false;
    }
    return true;
}

/**
 * @brief
 * Ask to the xsens its Device ID
 */
void Xsens::MTi::getDeviceID()
{
    this->addMessageToQueue(GoToConfig, NULL, GoToConfigAck);
    this->addMessageToQueue(ReqDID,NULL,DeviceID);
    this->addMessageToQueue(GoToMeasurement, NULL, GoToMeasurementAck);
    waitForQueueToFinish(1000);
}

/**
 * @brief
 * Send data to the XSens
 * @param data
 * @return bool
 */
bool Xsens::MTi::serialPortSendData(std::vector<unsigned char> * data)
{
    /*printf("Sending data -");
    for(int i=0 ; i<data->size() ; i++) printf(" 0x%X", data->at(i));
    printf("\n");*/

    char buffer[data->size()];

    int i;
    std::vector<unsigned char>::iterator it;
    for(i=0, it=data->begin() ; it!=data->end() ; i++, it++) buffer[i] = (char)*it;

    try{ serial_port.write(buffer, data->size()); }
    catch(cereal::Exception& e)
    {
        return false;
    }
    return true;
}

/**
 * @brief
 * Read data on the serial port
 * @param data
 * @param length
 */
void Xsens::MTi::serialPortReadData(char * data, int length)
{	
    if(length > 0)
    {
        // Manage the received data...
        unsigned char buffer;
        for(int i=0 ; i<length ; i++)
        {
            buffer = (unsigned char)data[i];

            // PREAMBLE
            if(packageInTransit == false)
            {
                if(buffer == PRE)
                {
                    this->package.clear();
                    this->package.push_back(buffer);
                    packageInTransit = true;
                    packageIndex = 1;
                }

            } else {

                // CHECKSUM
                if( (packageIsExtended == true && packageIndex == 6+packageLength) || (packageIsExtended == false && packageIndex == 4+packageLength) )
                {
                    package.push_back(buffer);

                    unsigned char checksum = 0;
                    for(unsigned int i=1 ; i<this->package.size() ; i++)
                    {
                        buffer = this->package[i];
                        checksum += buffer;
                    }
                    // If message is ok manage it else reset the package
                    if(checksum == 0x00) this->manageIncomingData(&this->package, packageIsExtended);
                    else this->resetPackage();
                }
                // DATA
                if((packageIndex >= 6 && packageIndex < 6+packageLength) || (packageIsExtended == false && packageIndex >= 4 && packageIndex < 4+packageLength) )
                {
                    this->package.push_back(buffer);
                    packageIndex++;
                }
                // EXT_LEN
                if(packageIsExtended == true && packageIndex == 4)
                {
                    this->package.push_back(buffer);
                    packageIndex = 5;
                }
                if(packageIsExtended == true && packageIndex == 5)
                {
                    this->package.push_back(buffer);
                    packageIndex = 6;

                    union
                    {
                        unsigned int value;
                        unsigned char buffer[2];

                    } intUnion;
                    intUnion.buffer[0] = this->package[4];
                    intUnion.buffer[1] = this->package[5];
                    packageLength = intUnion.value;
                }
                // LEN
                if(packageIndex == 3)
                {
                    this->package.push_back(buffer);
                    packageIndex = 4;

                    if(buffer == 0xFF) packageIsExtended = true;
                    else
                    {
                        packageIsExtended = false;
                        packageLength = (int)buffer;
                    }
                }
                // MID
                if(packageIndex == 2)
                {
                    this->package.push_back(buffer);
                    packageIndex = 3;
                }
                // BID
                if(buffer == BID && packageIndex == 1)
                {
                    this->package.push_back(buffer);
                    packageIndex = 2;
                }
                if(packageIndex == 1 && buffer != BID)
                {
                    this->resetPackage();
                }
            }
        }
    }
}

/**
 * @brief
 * Manage the incoming data in order to retreive the ID and the corresponding data
 * @param incomingData Raw data
 * @param dataIsExtended Raw data can be larger as expected (Xsens protocol)
 */
void Xsens::MTi::manageIncomingData(std::vector<unsigned char> * incomingData, bool dataIsExtended)
{	
    /*printf("Getting data -");
    for(int i=0 ; i<incomingData->size() ; i++) printf(" 0x%X", incomingData->at(i));
    printf("\n");*/

    int dataIndex = 4;
    if(dataIsExtended) dataIndex = 6;

    // And now finnaly actualy manage the data
    std::vector<unsigned char> data;

    std::vector<unsigned char>::iterator it;
    for(it=incomingData->begin()+dataIndex ; it!=incomingData->end() ; it++)
    {
        data.push_back((unsigned char)*it);
    }

    unsigned char MID;
    MID = incomingData->at(2);

    this->resetPackage();

    if(queueIsWaiting == true && MID == queueAck) this->manageQueue();

    // Variables useful to manage the data
    unsigned char floatBuffer[numOfBytes];
    int index;

    // Switch case for managing the various MIDs that might arrive
    switch(MID)
    {
    case DeviceID:
        if(data.size()>0)
        {
            uint32_t* ID = (uint32_t*)data.data();
            mDeviceID = ntohl(ID[0]);
        }
        break;
    case GoToConfigAck:
        ConfigState = true;
        break;

    case GoToMeasurementAck:
        ConfigState = false;
        break;

    case ReqOutputModeAck:
        if(data.size()>0)
        {
            unsigned short mask;
            ushort* temp = (ushort*)data.data();
            unsigned short outputMode = ntohs(temp[0]);
            mask = 0x0001;
            output_mode.temperatureData = ((outputMode & mask) == mask);
            mask = mask << 1;
            output_mode.calibratedData = ((outputMode & mask) == mask);
            mask = mask << 1;
            output_mode.orientationData = ((outputMode & mask) == mask);
            mask = mask << 1;
            output_mode.auxiliaryData = ((outputMode & mask) == mask);
            mask = mask << 1;
            output_mode.positionData = ((outputMode & mask) == mask);
            mask = mask << 1;
            output_mode.velocityData = ((outputMode & mask) == mask);
            mask = 0x0800;
            output_mode.statusData = ((outputMode & mask) == mask);
            mask = mask << 1;
            output_mode.rawGPSData = ((outputMode & mask) == mask);
            mask = mask << 2;
            output_mode.rawInertialData = ((outputMode & mask) == mask);
        }
        break;

    case ReqOutputSettingsAck:
        if(data.size()>0)
        {
            unsigned int mask;
            uint32_t* temp = (uint32_t*)data.data();
            unsigned int outputSettings = ntohl(temp[0]);
            mask = 0x01;
            output_settings.timeStamp = ((outputSettings & mask) == mask);
            mask = 0x03;
            output_settings.orientationMode = (Xsens::MTOrientationMode)(outputSettings>>2 & mask);
            mask = 0x01;
            mask = mask << 4;
            output_settings.enableAcceleration = ((outputSettings & mask) == mask);
            mask = mask << 1;
            output_settings.enableRateOfTurn = ((outputSettings & mask) == mask);
            mask = mask << 1;
            output_settings.enableMagnetometer = ((outputSettings & mask) == mask);
            mask = 0x80000000;
            output_settings.velocityModeNED = ((outputSettings & mask) == mask);
        }
        break;
    case ReqCurrentScenarioAck:
        if(data.size()>1)
        {
            mScenario = (Scenario)(data.at(1));
        }
        break;
    case ReqLeverArmGpsAck:
        if(data.size()>0)
        {
            /*  uint32_t* lever_arm = (uint32_t*)data.data();
            int temp =  ntohl(lever_arm[0]);
            float x = *(float*)&temp;
            temp =  ntohl(lever_arm[1]);
            float y = *(float*)&temp;
            temp =  ntohl(lever_arm[2]);
            float z = *(float*)&temp;
            ROS_INFO("x: %f, y: %f, z: %f",x,y,z);*/
        }
        break;
    case AvailableScenarios:
        if(data.size()>0)
        { std::stringstream scenarios;
            int k = 0;
            for(int i = 0; i< 5; i++)
            {
                scenarios << (int)data.at(k) <<" -> ";
                for(int j=k+2;j<k+22;j++)
                    scenarios << data.at(j);
                k +=22;
            }
            ROS_INFO("Available scenarios: %s",scenarios.str().c_str());
        }
        break;
        // Read incoming data according to mode and settings data
    case MTData:
        index = 0;
        if(output_mode.rawGPSData == true)
        {
            if(GPSFix())
            {
                index = 7;

                for(int i=0 ; i<numOfBytes ; i++) floatBuffer[i] = data[index+i];
                mLatitude = hexa2int(floatBuffer)*pow(10,-7);
                index += numOfBytes;
                for(int i=0 ; i<numOfBytes ; i++) floatBuffer[i] = data[index+i];
                mLongitude = hexa2int(floatBuffer)*pow(10,-7);
                index += numOfBytes;
                for(int i=0 ; i<numOfBytes ; i++) floatBuffer[i] = data[index+i];
                mAltitude = hexa2int(floatBuffer)/1000;
                index += numOfBytes;
                for(int i=0 ; i<numOfBytes ; i++) floatBuffer[i] = data[index+i];
                mVelocityNorth = hexa2int(floatBuffer);
                index += numOfBytes;
                for(int i=0 ; i<numOfBytes ; i++) floatBuffer[i] = data[index+i];
                mVelocityEast = hexa2int(floatBuffer);///pow(2,20);
                index += numOfBytes;
                for(int i=0 ; i<numOfBytes ; i++) floatBuffer[i] = data[index+i];
                mVelocityDown = hexa2int(floatBuffer);///pow(2,20);
                index += numOfBytes;
                unsigned char * buffer = (unsigned char * ) data.data();
                unsigned int *temp = (unsigned int *)(&buffer[index]);
                mHorizontalAccuracy = ntohl(*temp)/1000;
                index += numOfBytes;
                temp = (unsigned int *)(&buffer[index]);
                mVerticalAccuracy = ntohl(*temp)/1000;

            }
            else
                resetGPSValues();

            index = GPS_PVT_DATA_OFFSET;
        }
        if(output_mode.temperatureData == true)
        {
            for(int i=0 ; i<numOfBytes ; i++) floatBuffer[i] = data[index+i];
            mTemperature = hexa2float(floatBuffer);
            index += numOfBytes;
        }
        if(output_mode.calibratedData == true)
        {
            if(isSelfTestCompleted())
            {
                for(int i=0 ; i<numOfBytes ; i++) floatBuffer[i] = data[index+i];
                accX = hexa2float(floatBuffer);
                index += numOfBytes;
                for(int i=0 ; i<numOfBytes ; i++) floatBuffer[i] = data[index+i];
                accY = hexa2float(floatBuffer);
                index += numOfBytes;
                for(int i=0 ; i<numOfBytes ; i++) floatBuffer[i] = data[index+i];
                accZ = hexa2float(floatBuffer);
                index += numOfBytes;
                for(int i=0 ; i<numOfBytes ; i++) floatBuffer[i] = data[index+i];
                gyrX = hexa2float(floatBuffer);
                index += numOfBytes;
                for(int i=0 ; i<numOfBytes ; i++) floatBuffer[i] = data[index+i];
                gyrY = hexa2float(floatBuffer);
                index += numOfBytes;
                for(int i=0 ; i<numOfBytes ; i++) floatBuffer[i] = data[index+i];
                gyrZ = hexa2float(floatBuffer);
                index += numOfBytes;
                for(int i=0 ; i<numOfBytes ; i++) floatBuffer[i] = data[index+i];
                magX = hexa2float(floatBuffer);
                index += numOfBytes;
                for(int i=0 ; i<numOfBytes ; i++) floatBuffer[i] = data[index+i];
                magY = hexa2float(floatBuffer);
                index += numOfBytes;
                for(int i=0 ; i<numOfBytes ; i++) floatBuffer[i] = data[index+i];
                magZ = hexa2float(floatBuffer);
                index += numOfBytes;
            }
            else
                index += 9*numOfBytes;
        }
        if(output_mode.orientationData == true)
        {

            if(output_settings.orientationMode == Quaternion)
            {
                if(isSelfTestCompleted())
                {
                    for(int i=0 ; i<numOfBytes ; i++) floatBuffer[i] = data[index+i];
                    q0 = hexa2float(floatBuffer);
                    index += numOfBytes;
                    for(int i=0 ; i<numOfBytes ; i++) floatBuffer[i] = data[index+i];
                    q1 = hexa2float(floatBuffer);
                    index += numOfBytes;
                    for(int i=0 ; i<numOfBytes ; i++) floatBuffer[i] = data[index+i];
                    q2 = hexa2float(floatBuffer);
                    index += numOfBytes;
                    for(int i=0 ; i<numOfBytes ; i++) floatBuffer[i] = data[index+i];
                    q3 = hexa2float(floatBuffer);
                    index += numOfBytes;
                }
                else
                    index += 4*numOfBytes;
            }
            if(output_settings.orientationMode == EulerAngles)
            {
                if(isSelfTestCompleted())
                {
                    for(int i=0 ; i<numOfBytes ; i++) floatBuffer[i] = data[index+i];
                    eroll = hexa2float(floatBuffer);
                    index += numOfBytes;
                    for(int i=0 ; i<numOfBytes ; i++) floatBuffer[i] = data[index+i];
                    epitch = hexa2float(floatBuffer);
                    index += numOfBytes;
                    for(int i=0 ; i<numOfBytes ; i++) floatBuffer[i] = data[index+i];
                    eyaw = hexa2float(floatBuffer);
                    index += numOfBytes;
                }
                else
                    index += 3*numOfBytes;
            }
            if(output_settings.orientationMode == Matrix)
            {
                index += 9*numOfBytes;
            }

        }
        if(output_mode.auxiliaryData == true)
        {
            index += numOfBytes;
        }
        if(output_mode.positionData == true)
        {

            for(int i=0 ; i<numOfBytes ; i++) floatBuffer[i] = data[index+i];
            mLatitude = hexa2float(floatBuffer);
            index += numOfBytes;
            for(int i=0 ; i<numOfBytes ; i++) floatBuffer[i] = data[index+i];
            mLongitude = hexa2float(floatBuffer);
            index += numOfBytes;
            for(int i=0 ; i<numOfBytes ; i++) floatBuffer[i] = data[index+i];
            mAltitude = hexa2float(floatBuffer);
            index += numOfBytes;
        }
        if(output_mode.velocityData == true)
        {
            for(int i=0 ; i<numOfBytes ; i++) floatBuffer[i] = data[index+i];
            mVelocityX = hexa2float(floatBuffer);
            index += numOfBytes;
            for(int i=0 ; i<numOfBytes ; i++) floatBuffer[i] = data[index+i];
            mVelocityY = hexa2float(floatBuffer);
            index += numOfBytes;
            for(int i=0 ; i<numOfBytes ; i++) floatBuffer[i] = data[index+i];
            mVelocityZ = hexa2float(floatBuffer);
            index += numOfBytes;
        }
        if(output_mode.statusData == true)
        {
            mStatus = data[index];
            index += ONE_BYTE;
        }
        if(output_settings.timeStamp == true)
        {
            index += 2*ONE_BYTE;
        }
        break;

    case ResetOrientationAck:
        // Reset ok.
        break;
    }
}

/**
 * @brief
 * Reset GPS values as altitude, longitude ...
 */
void Xsens::MTi::resetGPSValues()
{
    mAltitude = mLongitude = mLatitude = 0.0;
    mVelocityNorth = mVelocityEast = mVelocityDown = 0.0;
}

/**
 * @brief
 * Make the message with the right ID and to match with the xsens protocol
 * @param mid
 * @param data
 * @param message
 */
void Xsens::MTi::makeMessage(MTMessageIdentifier mid, std::vector<unsigned char> * data, std::vector<unsigned char> * message)
{
    int dataLength = 0;
    if(data!=NULL) dataLength = data->size();
    unsigned char byte;

    message->clear();
    // PREAMBLE
    message->push_back(PRE);
    // BID
    message->push_back(BID);
    // MID
    byte = (unsigned char)mid;
    message->push_back(byte);
    // LEN / EXT_LEN
    if(dataLength < 0xFF)
    {
        byte = (unsigned char)dataLength;
        message->push_back(byte);

    } else {

        byte = 0xFF;
        message->push_back(byte);

        union
        {
            int length;
            unsigned char buffer[2];
        }lengthUnion;

        lengthUnion.length = dataLength;
        message->push_back(lengthUnion.buffer[0]);
        message->push_back(lengthUnion.buffer[1]);
    }
    //  DATA
    if(data!=NULL)
    {
        if(data->size() > 0)
        {
            std::vector<unsigned char>::iterator it;
            for(it=data->begin() ; it!=data->end() ; it++) message->push_back(*it);
        }
    }
    // CHECKSUM
    unsigned char checksum = 0;
    // BID
    checksum += message->at(1);
    // MID
    checksum += message->at(2);
    // LEN
    if(dataLength < 0xFF)
    {
        checksum += message->at(3);

    } else {

        checksum += message->at(3);
        checksum += message->at(4);
        checksum += message->at(5);
    }
    // DATA
    for(int i=0 ; i<dataLength ; i++)
    {
        int dataIndex = 6;
        if(dataLength < 0xFF) dataIndex = 4;
        checksum += message->at(dataIndex+i);
    }
    int c = 0x100;
    byte = (unsigned char)(c-(int)checksum);
    message->push_back(byte);
}

/**
 * @brief
 * Fill ROS odometry message with the values come from the xsens
 * @param listener
 * @param odom_broadcaster
 * @param now
 * @return nav_msgs::Odometry
 */
nav_msgs::Odometry Xsens::MTi::fillOdometryMessage(const tf::TransformListener& listener, tf::TransformBroadcaster& odom_broadcaster, const ros::Time& now)
{
    nav_msgs::Odometry odom_msg;
    double northing, easting;
    std::string zone;
    if(!GPSFix())
        return odom_msg;



    gps_common::LLtoUTM(latitude(), longitude(), northing, easting, zone);

    Position current_position;
    if(output_settings.velocityModeNED)
    {
        current_position.x = northing;
        current_position.y = easting;
        current_position.z = -altitude();
    }
    else
    {
        current_position.x = northing;
        current_position.y = -easting;
        current_position.z = altitude();
    }

    if(current_position.z < 1.0)//First value are not quite right
        return odom_msg;

    geometry_msgs::QuaternionStamped qt;
    double quaternionW, quaternionX, quaternionY, quaternionZ;
    fillQuaternionWithOutputSettings(quaternionX,quaternionY,quaternionZ,quaternionW);

    if( listener.frameExists(mRosNamespace + BASE_LINK_FRAME_ID) && listener.frameExists(mFrameID) && quaternionW != 0 && quaternionX != 0 && quaternionY != 0 && quaternionZ != 0)
    {
        qt.header.frame_id = mFrameID;
        qt.header.stamp = now;

        qt.quaternion.x = quaternionX;
        qt.quaternion.y = quaternionY;
        qt.quaternion.z = quaternionZ;
        qt.quaternion.w = quaternionW;

        if((mInitialPosition.x == 0.0) && (mInitialPosition.y == 0.0) && (mInitialPosition.z == 0.0)  )
        {
            mInitialPosition.x = current_position.x;
            mInitialPosition.y = current_position.y;
            mInitialPosition.z = current_position.z;
            ROS_INFO("INITIAL x: %f, y: %f, z: %f", current_position.x,current_position.y,current_position.z);
        }

        current_position.x = current_position.x - mInitialPosition.x;
        current_position.y = current_position.y - mInitialPosition.y;
        current_position.z = current_position.z - mInitialPosition.z;

        tf::StampedTransform T_base_imu;
        try{
            listener.lookupTransform(mRosNamespace + BASE_LINK_FRAME_ID, mFrameID,ros::Time(0), T_base_imu);
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            return odom_msg;
        }

        tf::Transform T_odom_imu(tf::Quaternion(quaternionX,quaternionY,quaternionZ,quaternionW),tf::Vector3(current_position.x,current_position.y, current_position.z));
        tf::StampedTransform T_odom_base_st(T_odom_imu, now, mRosNamespace + ODOMETRY_FRAME_ID, mRosNamespace + BASE_LINK_FRAME_ID);
        T_odom_base_st *= T_base_imu.inverse();
        geometry_msgs::TransformStamped base_to_odom_msg;
        tf::transformStampedTFToMsg(T_odom_base_st, base_to_odom_msg);
        if(qt.quaternion.x != 0.0 && qt.quaternion.y != 0.0 && qt.quaternion.z != 0.0 && qt.quaternion.w != 0.0)
            odom_broadcaster.sendTransform(base_to_odom_msg);

        //next, we'll publish the odometry message over ROS
        odom_msg.header.stamp = now;
        odom_msg.header.frame_id = mRosNamespace + ODOMETRY_FRAME_ID;
        odom_msg.child_frame_id = mRosNamespace + BASE_LINK_FRAME_ID;

        //set the position
        odom_msg.pose.pose.position.x = base_to_odom_msg.transform.translation.x;
        odom_msg.pose.pose.position.y = base_to_odom_msg.transform.translation.y;
        odom_msg.pose.pose.position.z = base_to_odom_msg.transform.translation.z;
        odom_msg.pose.pose.orientation.w = base_to_odom_msg.transform.rotation.w;
        odom_msg.pose.pose.orientation.x = base_to_odom_msg.transform.rotation.x;
        odom_msg.pose.pose.orientation.y = base_to_odom_msg.transform.rotation.y;
        odom_msg.pose.pose.orientation.z = base_to_odom_msg.transform.rotation.z;


        //set the velocity
        tf::Transform orientation(tf::Quaternion(base_to_odom_msg.transform.rotation.x, base_to_odom_msg.transform.rotation.y, base_to_odom_msg.transform.rotation.z, base_to_odom_msg.transform.rotation.w));
        tf::Vector3 vel(velocity_x(), velocity_y(), velocity_z());

        vel = orientation.inverse() * vel;
        odom_msg.twist.twist.linear.x = vel.x();
        odom_msg.twist.twist.linear.y = vel.y();
        odom_msg.twist.twist.linear.z = vel.z();

        odom_msg.twist.twist.angular.x = gyroscope_x();
        odom_msg.twist.twist.angular.y = gyroscope_y();
        odom_msg.twist.twist.angular.z = gyroscope_z();

    }

    return odom_msg;
}

/**
 * @brief
 * Fill ROS IMU message with the values come from the xsens
 * @param now
 * @return sensor_msgs::Imu
 */
sensor_msgs::Imu Xsens::MTi::fillImuMessage(const ros::Time &now)
{
    sensor_msgs::Imu imu_msg;
    imu_msg.header.stamp = now;

    imu_msg.header.frame_id = mFrameID.c_str();

    fillQuaternionWithOutputSettings(imu_msg.orientation.x,imu_msg.orientation.y,imu_msg.orientation.z,imu_msg.orientation.w);


    imu_msg.angular_velocity.x = gyroscope_x();
    imu_msg.angular_velocity.y = gyroscope_y();
    imu_msg.angular_velocity.z = gyroscope_z();

    imu_msg.linear_acceleration.x = accelerometer_x();
    imu_msg.linear_acceleration.y = accelerometer_y();
    imu_msg.linear_acceleration.z = accelerometer_z();
    return imu_msg;
}

/**
 * @brief
 * Fill value according to the configuration of the xsens
 * @param x
 * @param y
 * @param z
 * @param w
 */
void Xsens::MTi::fillQuaternionWithOutputSettings(double& x, double& y, double& z, double& w )
{
    if(output_settings.orientationMode == EulerAngles)
    {
        tf::Quaternion quaternion = tf::createQuaternionFromRPY(roll(),pitch(),yaw());

        x = quaternion.x();
        y = quaternion.y();
        z = quaternion.z();
        w = quaternion.w();

    }
    else if(output_settings.orientationMode == Quaternion)
    {
        x = quaternion_x();
        y = quaternion_y();
        z = quaternion_z();
        w = quaternion_w();
    }
}

/**
 * @brief
 * Fill ROS NavSatFix message with the values come from the xsens
 * @param now
 * @return sensor_msgs::NavSatFix
 */
sensor_msgs::NavSatFix Xsens::MTi::fillNavFixMessage(const ros::Time& now)
{
    sensor_msgs::NavSatFix nav_fix_msg;
    sensor_msgs::NavSatStatus nav_status_msg;
    nav_fix_msg.header.stamp = now;
    nav_fix_msg.header.frame_id = mFrameID.c_str();

    nav_fix_msg.altitude = altitude();
    nav_fix_msg.latitude = latitude();
    nav_fix_msg.longitude = longitude();
    nav_fix_msg.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;

    if(GPSFix())
        nav_status_msg.status = sensor_msgs::NavSatStatus::STATUS_FIX;
    else
        nav_status_msg.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;

    nav_fix_msg.status = nav_status_msg;

    return nav_fix_msg;
}

// EOF

