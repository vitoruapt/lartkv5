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
#include <cereal_port/CerealPort.h>
#include "MTMessage.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <gps_common/conversions.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>

#include <string>
#include <vector>

#define PRE 0xFA
#define BID 0xFF

namespace Xsens
{
static const int GPS_PVT_DATA_OFFSET = 44;
static const unsigned char SELF_TEST = 0x01;  /**< Sensor OK ?*/
static const unsigned char XKF_VALID = 0x02; /**< Differential correction data available */
static const unsigned char GPS_FIX   = 0x04; /**< Satellite is detected and data used */
static const int ONE_BYTE = 1;
static const std::string IMU_FRAME_ID = "/base_imu";
static const std::string BASE_LINK_FRAME_ID = "/base_link";
static const std::string ODOMETRY_FRAME_ID = "/odom";


typedef enum _Scenario
{
    General = 1,
    Automotive = 2,
    Aerospace = 3,
    Human = 4,
    Human_large_accelerations = 5,
    Machine = 6,
    Machine_nomagfield = 7,
    Marine_MTIMTX = 8,
    General_nobaro = 9,
    Aerospace_nobaro = 10,
    Automotive_nobaro = 11,
    Marine_MTIG = 17,
}Scenario;

class MTi
{
public:
    MTi();
    ~MTi();

    typedef struct _outputMode
    {
        bool temperatureData;
        bool calibratedData;
        bool orientationData;
        bool auxiliaryData;
        bool positionData;
        bool velocityData;
        bool statusData;
        bool rawGPSData;
        bool rawInertialData;

    } outputMode;

    typedef struct _outputSettings
    {
        bool timeStamp;
        MTOrientationMode orientationMode;
        bool enableAcceleration;
        bool enableRateOfTurn;
        bool enableMagnetometer;
        bool velocityModeNED;

    } outputSettings;

    typedef struct _Position
    {
        float x;
        float y;
        float z;

    } Position;

    bool setSettings(outputMode mode, outputSettings settings, Scenario scenario, const std::string& rosNamespace, const std::string& frameID, const Position& GPSLeverArm, int timeout);

    bool openPort(char * name, int baudrate);
    bool closePort();

    void getDeviceID();

    void makeMessage(MTMessageIdentifier mid, std::vector<unsigned char> * data, std::vector<unsigned char> * message);
    void addMessageToQueue(MTMessageIdentifier messageID, std::vector<unsigned char> * data, MTMessageIdentifier ack);
    bool waitForQueueToFinish(int timeout);
    nav_msgs::Odometry fillOdometryMessage(const tf::TransformListener& listener, tf::TransformBroadcaster& odom_broadcaster, const ros::Time& now);
    sensor_msgs::Imu fillImuMessage(const ros::Time& now);
    sensor_msgs::NavSatFix fillNavFixMessage(const ros::Time& now);

    //void resetOrientation();

    float accelerometer_x() const { return accX; }
    float accelerometer_y() const { return accY; }
    float accelerometer_z() const { return accZ; }
    float gyroscope_x() const { return gyrX; }
    float gyroscope_y() const { return gyrY; }
    float gyroscope_z() const { return gyrZ; }
    float compass_x() const { return magX; }
    float compass_y() const { return magY; }
    float compass_z() const { return magZ; }
    float temperature() const { return mTemperature; }
    float quaternion_x() const {return q1; }
    float quaternion_y() const { return q2; }
    float quaternion_z() const { return q3; }
    float quaternion_w() const { return q0; }
    float roll() { return eroll; }
    float pitch() { return epitch; }
    float yaw() { return eyaw; }

    float altitude() const { return mAltitude; }
    float longitude() const { return mLongitude; }
    float latitude() const { return mLatitude; }

    float velocity_x() const { return mVelocityX;}
    float velocity_y() const { return mVelocityY; }
    float velocity_z() const { return mVelocityZ; }
    float velocityNorth() const { return mVelocityNorth;}
    float velocityEast() const { return mVelocityEast; }
    float velocityDown() const { return mVelocityDown; }
    bool GPSFix() const  {return (mStatus & GPS_FIX);}
    uint32_t horizontalAccuracy() const {return mHorizontalAccuracy;}
    uint32_t verticalAccuracy() const {return mVerticalAccuracy;}

private:
    // Serial Port variables
    cereal::CerealPort serial_port;

    // OutputMode
    std::vector<unsigned char> outputModeData;
    outputMode output_mode;

    // OutputSettings
    std::vector<unsigned char> outputSettingsData;
    outputSettings output_settings;

    //Scenario
    std::vector<unsigned char> mScenarioData;
    Scenario mScenario;

    std::string mFrameID;
    std::string mRosNamespace;


    // To manage incoming packages
    std::vector<unsigned char> package;
    int packageLength;
    bool packageInTransit;
    bool packageIsExtended;
    int packageIndex;

    bool ConfigState;

    float yawCompensation;

    int numOfBytes;
    uint32_t  mDeviceID;


    Position mInitialPosition;

    tf::TransformListener listener;
     geometry_msgs::PoseStamped source_pose;
     geometry_msgs::PoseStamped target_pose;

    // ******* MTi Data *******
    float accX, accY, accZ;
    float gyrX, gyrY, gyrZ;
    float magX, magY, magZ;
    float mTemperature;
    float q0, q1, q2, q3;
    float eroll, epitch, eyaw;
    unsigned int ts;

    float mAltitude, mLongitude, mLatitude;
    float mVelocityNorth, mVelocityEast, mVelocityDown;
    float mVelocityX, mVelocityY, mVelocityZ;
    unsigned char mStatus;
    uint32_t mHorizontalAccuracy, mVerticalAccuracy;
    // ************************

    // Message queue
    std::vector<MTMessage> queue;
    MTMessageIdentifier queueAck;
    bool queueIsRunning;
    bool queueIsWaiting;

    void resetPackage();
    void resetGPSValues();
    void manageQueue();

    bool serialPortSendData(std::vector<unsigned char> * data);
    void serialPortReadData(char * data, int length);
    void manageIncomingData(std::vector<unsigned char> * data, bool dataIsExtended);
    bool isMtiG();
    bool isSelfTestCompleted();
    void fillQuaternionWithOutputSettings(double& x, double& y, double& z, double& w );
};
}

// EOF

