/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef USE_RANGEFINDER_TF

#include "build/debug.h"
#include "build/build_config.h"

#include "io/serial.h"

#include "drivers/rangefinder/rangefinder.h"
#include "drivers/rangefinder/rangefinder_lidartf.h"
#include "drivers/rangefinder/flow_decode.h"
#include "drivers/rangefinder/flow_fusion.h"

#include "fc/runtime_config.h"

#include "flight/imu.h"

#define TF_DEVTYPE_NONE 0
#define TF_DEVTYPE_MINI 1
#define TF_DEVTYPE_02   2

static uint8_t tfDevtype = TF_DEVTYPE_NONE;

#define TF_FRAME_LENGTH    6             // Excluding sync bytes (0x59) x 2 and checksum
#define TF_FRAME_SYNC_BYTE 0x59
#define TF_TIMEOUT_MS      (100 * 2)

//
// Benewake TFmini frame format
// Byte Off Description
// 1    -   SYNC
// 2    -   SYNC
// 3    0   Measured distance (LSB)
// 4    1   Measured distance (MSB)
// 5    2   Signal strength (LSB)
// 6    3   Signal strength (MSB)
// 7    4   Integral time
// 8    5   Reserved
// 9    -   Checksum (Unsigned 8-bit sum of bytes 0~7)
//
// Credibility
// 1. If distance is 12m (1200cm), then OoR.
//
#define TF_MINI_FRAME_INTEGRAL_TIME 4

//
// Benewake TF02 frame format (From SJ-GU-TF02-01 Version: A01)
// Byte Off Description
// 1    -   SYNC
// 2    -   SYNC
// 3    0   Measured distance (LSB)
// 4    1   Measured distance (MSB)
// 5    2   Signal strength (LSB)
// 6    3   Signal strength (MSB)
// 7    4   SIG (Reliability in 1~8, less than 7 is unreliable)
// 8    5   TIME (Exposure time, 3 or 6)
// 9    -   Checksum (Unsigned 8-bit sum of bytes 0~7)
//
// Credibility
// 1. If SIG is less than 7, unreliable
// 2. If distance is 22m (2200cm), then OoR.
//
#define TF_02_FRAME_SIG 4

// Maximum ratings

#define TF_MINI_RANGE_MIN 1
#define TF_MINI_RANGE_MAX 1200

#define TF_02_RANGE_MIN 40
#define TF_02_RANGE_MAX 2200

#define TF_DETECTION_CONE_DECIDEGREES 900

static serialPort_t *tfSerialPort = NULL;

typedef enum {
    TF_FRAME_STATE_WAIT_START1,
    TF_FRAME_STATE_WAIT_START2,
    TF_FRAME_STATE_READING_PAYLOAD,
    TF_FRAME_STATE_WAIT_CKSUM,
} tfFrameState_e;

static tfFrameState_e tfFrameState;
static uint8_t tfFrame[TF_FRAME_LENGTH];
static uint8_t tfReceivePosition;

// TFmini
// Command for 100Hz sampling (10msec interval)
// At 100Hz scheduling, skew will cause 10msec delay at the most.
static uint8_t tfCmdTFmini[] = { 0x42, 0x57, 0x02, 0x00, 0x00, 0x00, 0x01, 0x06 };

// TF02
// Same as TFmini for now..
static uint8_t tfCmdTF02[] = { 0x42, 0x57, 0x02, 0x00, 0x00, 0x00, 0x01, 0x06 };

static int32_t lidarTFValue;
static uint16_t lidarTFerrors = 0;
//光流+激光二合一模块的输出值
static int16_t flow_x_integral = 0;
static int16_t flow_y_integral = 0;
static uint16_t ground_distance = 0;
static uint8_t valid = 0;
static uint8_t tof_confidence = 0;
static int16_t integration_timespan = 0;
uint8_t opti_flow_rc_state = 0;

//timeMs_t timeMs_Last = 0;

static void lidarTFSendCommand(void)
{
    switch (tfDevtype) {
    case TF_DEVTYPE_MINI:
        serialWriteBuf(tfSerialPort, tfCmdTFmini, sizeof(tfCmdTFmini));
        break;
    case TF_DEVTYPE_02:
        serialWriteBuf(tfSerialPort, tfCmdTF02, sizeof(tfCmdTF02));
        break;
    }
}

void lidarTFInit(rangefinderDev_t *dev)
{
    UNUSED(dev);

    tfFrameState = TF_FRAME_STATE_WAIT_START1;
    tfReceivePosition = 0;
}

void lidarTFUpdate(rangefinderDev_t *dev, timeUs_t currentTimeUs)
{
    UNUSED(dev);
    static timeUs_t lastTimeUs = 0;
    float dTime = (currentTimeUs - lastTimeUs)*1e-6f;
    // float fx = FlowGetX(dev);
    // float fy = FlowGetY(dev);
    if(opti_flow_rc_state == 1)
    {
        flow_fusion(dTime, flow_x_integral, flow_y_integral, ground_distance * getCosTiltAngle());
    }
    lastTimeUs = currentTimeUs;
}

// void lidarTFUpdate(rangefinderDev_t *dev)
// {
//     UNUSED(dev);
//     static timeMs_t lastFrameReceivedMs = 0;
//     const timeMs_t timeNowMs = millis();

//     if (tfSerialPort == NULL) {
//         return;
//     }

    // while (serialRxBytesWaiting(tfSerialPort)) {
    //     uint8_t c = serialRead(tfSerialPort);
    //     switch (tfFrameState) {
    //     case TF_FRAME_STATE_WAIT_START1:
    //         if (c == TF_FRAME_SYNC_BYTE) {
    //             tfFrameState = TF_FRAME_STATE_WAIT_START2;
    //         }
    //         break;

    //     case TF_FRAME_STATE_WAIT_START2:
    //         if (c == TF_FRAME_SYNC_BYTE) {
    //             tfFrameState = TF_FRAME_STATE_READING_PAYLOAD;
    //         } else {
    //             tfFrameState = TF_FRAME_STATE_WAIT_START1;
    //         }
    //         break;

    //     case TF_FRAME_STATE_READING_PAYLOAD:
    //         tfFrame[tfReceivePosition++] = c;
    //         if (tfReceivePosition == TF_FRAME_LENGTH) {
    //             tfFrameState = TF_FRAME_STATE_WAIT_CKSUM;
    //         }
    //         break;

    //     case TF_FRAME_STATE_WAIT_CKSUM:
    //         {
    //             uint8_t cksum = TF_FRAME_SYNC_BYTE + TF_FRAME_SYNC_BYTE;
    //             for (int i = 0 ; i < TF_FRAME_LENGTH ; i++) {
    //                 cksum += tfFrame[i];
    //             }

    //             if (c == cksum) {

    //                 uint16_t distance = tfFrame[0] | (tfFrame[1] << 8);
    //                 uint16_t strength = tfFrame[2] | (tfFrame[3] << 8);

    //                 DEBUG_SET(DEBUG_LIDAR_TF, 0, distance);
    //                 DEBUG_SET(DEBUG_LIDAR_TF, 1, strength);
    //                 DEBUG_SET(DEBUG_LIDAR_TF, 2, tfFrame[4]);
    //                 DEBUG_SET(DEBUG_LIDAR_TF, 3, tfFrame[5]);

    //                 switch (tfDevtype) {
    //                 case TF_DEVTYPE_MINI:
    //                     if (distance >= TF_MINI_RANGE_MIN && distance < TF_MINI_RANGE_MAX) {
    //                         lidarTFValue = distance;
    //                         if (tfFrame[TF_MINI_FRAME_INTEGRAL_TIME] == 7) {
    //                             // When integral time is long (7), measured distance tends to be longer by 12~13.
    //                             lidarTFValue -= 13;
    //                         }
    //                     } else {
    //                         lidarTFValue = -1;
    //                     }
    //                     break;

    //                 case TF_DEVTYPE_02:
    //                     if (distance >= TF_02_RANGE_MIN && distance < TF_02_RANGE_MAX && tfFrame[TF_02_FRAME_SIG] >= 7) {
    //                         lidarTFValue = distance;
    //                     } else {
    //                         lidarTFValue = -1;
    //                     }
    //                     break;
    //                 }
    //                 lastFrameReceivedMs = timeNowMs;
    //             } else {
    //                 // Checksum error. Simply discard the current frame.
    //                 ++lidarTFerrors;
    //                 //DEBUG_SET(DEBUG_LIDAR_TF, 3, lidarTFerrors);
    //             }
    //         }

    //         tfFrameState = TF_FRAME_STATE_WAIT_START1;
    //         tfReceivePosition = 0;

    //         break;
    //     }
    // }

//     // If valid frame hasn't been received for more than a timet, resend command.

//     if (timeNowMs - lastFrameReceivedMs > TF_TIMEOUT_MS) {
//         lidarTFSendCommand();
//     }
// }

// Return most recent device output in cm

int32_t lidarTFGetDistance(rangefinderDev_t *dev)
{
    UNUSED(dev);

    return lidarTFValue;
}

int16_t FlowGetX(rangefinderDev_t *dev)
{
    UNUSED(dev);
    return flow_x_integral;
}

int16_t FlowGetY(rangefinderDev_t *dev)
{
    UNUSED(dev);
    return flow_y_integral;
}

int16_t FlowGetIntegrationTimespan(rangefinderDev_t *dev)
{
    UNUSED(dev);
    return integration_timespan;
}

uint8_t FlowGetValid(rangefinderDev_t *dev)
{
    UNUSED(dev);
    return valid;
}

uint8_t GetTofConfidence(rangefinderDev_t *dev)
{
    UNUSED(dev);
    return tof_confidence;
}


//串口接收触发函数
static void OptiFlowReceive(uint16_t c, void* data){
    UNUSED(data);
    if(up_parse_char((uint8_t)c))
    {
        if(up_data.valid == 0xF5)
        {
            flow_x_integral = up_data.flow_x_integral;  //X像素点累计时间内的累加位移(除以10000乘以高度后为实际位移)
            flow_y_integral = up_data.flow_y_integral;  //y像素点累计时间内的累加位移
        }
        //integration_timespan = up_data.integration_timespan;
        ground_distance = up_data.ground_distance;
        valid = up_data.valid;  //光流数据是否可用 0x00为不可用，0xF5(245)为光流数据可用
        tof_confidence = up_data.tof_confidence; //测距置信度 0x64表示100%

        lidarTFValue = (int32_t)ground_distance;
        return;
    }

}

static bool lidarTFDetect(rangefinderDev_t *dev, uint8_t devtype)
{
    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_LIDAR_TF);

    if (!portConfig) {
        return false;
    }

    tfSerialPort = openSerialPort(portConfig->identifier, FUNCTION_LIDAR_TF, OptiFlowReceive, NULL, 115200, MODE_RX, SERIAL_STOPBITS_1);

    if (tfSerialPort == NULL) {
        return false;
    }

    tfDevtype = devtype;

    dev->delayMs = 10;
    dev->maxRangeCm = (devtype == TF_DEVTYPE_MINI) ? TF_MINI_RANGE_MAX : TF_02_RANGE_MAX;
    dev->detectionConeDeciDegrees = TF_DETECTION_CONE_DECIDEGREES;
    dev->detectionConeExtendedDeciDegrees = TF_DETECTION_CONE_DECIDEGREES;

    dev->init = &lidarTFInit;
    dev->update = &lidarTFUpdate;
    dev->read = &lidarTFGetDistance;


    return true;
}

bool lidarTFminiDetect(rangefinderDev_t *dev)
{
    return lidarTFDetect(dev, TF_DEVTYPE_MINI);
}

bool lidarTF02Detect(rangefinderDev_t *dev)
{
    return lidarTFDetect(dev, TF_DEVTYPE_02);
}

#endif
