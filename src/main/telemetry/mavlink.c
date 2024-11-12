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

/*
 * telemetry_mavlink.c
 *
 * Author: Konstantin Sharlaimov
 */
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#if defined(USE_TELEMETRY_MAVLINK)

#include "common/maths.h"
#include "common/axis.h"
#include "common/color.h"
#include "common/utils.h"

#include "config/feature.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/rx.h"

#include "drivers/accgyro/accgyro.h"
#include "drivers/sensor.h"
#include "drivers/time.h"

#include "config/config.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/failsafe.h"
#include "flight/position.h"

#include "io/serial.h"
#include "io/gimbal.h"
#include "io/gps.h"
#include "io/ledstrip.h"

#include "rx/rx.h"

#include "sensors/sensors.h"
#include "sensors/acceleration.h"
#include "sensors/gyro.h"
#include "sensors/barometer.h"
#include "sensors/boardalignment.h"
#include "sensors/battery.h"

#include "telemetry/telemetry.h"
#include "telemetry/mavlink.h"

/*以下是我添加的部分*/
#include "sensors/acceleration.h"
#include "sensors/gyro.h"
#include "common/axis.h"
#include "flight/imu.h"
#define GRAVITY_EARTH (9.80665f)
/*以上是我添加的部分*/


// mavlink library uses unnames unions that's causes GCC to complain if -Wpedantic is used
// until this is resolved in mavlink library - ignore -Wpedantic for mavlink code
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include "common/mavlink.h"
#pragma GCC diagnostic pop

#define TELEMETRY_MAVLINK_INITIAL_PORT_MODE MODE_TX
#define TELEMETRY_MAVLINK_MAXRATE 300
#define TELEMETRY_MAVLINK_DELAY ((1000 * 1000) / TELEMETRY_MAVLINK_MAXRATE)

extern uint16_t rssi; // FIXME dependency on mw.c

static serialPort_t *mavlinkPort = NULL;
static const serialPortConfig_t *portConfig;

static bool mavlinkTelemetryEnabled =  false;
static portSharing_e mavlinkPortSharing;

static MAV_AUTOPILOT autopilot_name = MAV_AUTOPILOT_GENERIC;
/*
// 串口接收回调，详细见serial.h第62行
// 参数 uint16_t c是串口接收到的数据，其实应该是8bit就够了，不知道为什么用16位的，而且后面也变成8位的
//      void* data是一个不关心的参数（按经验应该是数据的大小），所以后面用UNUSED修饰
static void mavlinkReceive(uint16_t c, void* data) {

    UNUSED(data);
    mavlink_message_t msg;
    mavlink_status_t status;
    // 下面这个函数用于判断当前我们指定的数据的接收状态，包含三个状态：1.数据帧接收完成，2.校验失败接收错误， 3.正在接收，还未结束
    // 只有接收完成，才会进入if的条件判断
    // 参数 MAVLINK_COMM_0是通道，默认0即可
    // (uint8_t)c是当前串口中断收到的数据，这里可见其实串口就是8位的
    // &msg是一个完整的mavlink数据包，定义如下：
    //  typedef struct __mavlink_message {
    // 	uint16_t checksum;      ///< sent at end of packet
    // 	uint8_t magic;          ///< protocol magic marker
    // 	uint8_t len;            ///< Length of payload
    // 	uint8_t incompat_flags; ///< flags that must be understood
    // 	uint8_t compat_flags;   ///< flags that can be ignored if not understood
    // 	uint8_t seq;            ///< Sequence of packet
    // 	uint8_t sysid;          ///< ID of message sender system/aircraft
    // 	uint8_t compid;         ///< ID of the message sender component
    // 	uint32_t msgid:24;      ///< ID of message in payload
    // 	uint64_t payload64[(MAVLINK_MAX_PAYLOAD_LEN+MAVLINK_NUM_CHECKSUM_BYTES+7)/8];
    // 	uint8_t ck[2];          ///< incoming checksum bytes
    // 	uint8_t signature[MAVLINK_SIGNATURE_BLOCK_LEN];
    // }) mavlink_message_t;


    // &status用于获取是三种状态的哪一种，只有是接收完成，即status为1是才会执行后续操作
    if (mavlink_parse_char(MAVLINK_COMM_0, (uint8_t)c, &msg, &status)) {
        switch(msg.msgid) {
            // receive heartbeat
            // case 0: {
            //     mavlink_heartbeat_t command;
            //     mavlink_msg_heartbeat_decode(&msg,&command);
            //     mav_custommode = command.custom_mode;
            //     mav_type = command.type;
            //     mav_autopilot = command.autopilot;
            //     mav_basemode = command.base_mode;
            //     mav_systemstatus = command.custom_mode;
            //     mav_version = command.mavlink_version;
            //     // mavlinkSendHeartbeat();
            //     // mavlinkSendHUD();
            //     // mavlinkSendAttitude();
            //     break;
            // }
            // setpoint command
            // case 81: {void &command);
            //     attitude_controller.altitude_thrust = -command.thrust * 100;
            //     attitude_controller.roll = command.roll;   //maybe need normalization but this should be done in the JeVois
            //     attitude_controller.pitch = -command.pitch;
            //     attitude_controller.yaw = command.yaw;

            case 82:{
                mavlink_set_attitude_target_t command;
                mavlink_msg_set_attitude_target_decode(&msg,&command);
                // get_offboard.q[0] = command.q[0];  //w
                // get_offboard.q[1] = command.q[1];  //x
                // get_offboard.q[2] = command.q[2];  //y
                // get_offboard.q[3] = command.q[3];  //z
                if(command.type_mask == 7) //attitude
                {
                    get_offboard.roll_angle = command.body_roll_rate;
                    get_offboard.pitch_angle =  -command.body_pitch_rate;
                    get_offboard.yaw_angle = -command.body_yaw_rate;
                }else
                {
                    get_offboard.roll_rate = command.body_roll_rate;
                    get_offboard.pitch_rate = -command.body_pitch_rate;
                    get_offboard.yaw_rate = -command.body_yaw_rate;
                }
                
                attitude_controller.sum++;
                // if(attitude_controller.sum == 180)
                // {
                //     attitude_controller.sum = 0;
                // }
                // if(get_offboard.thrust != command.thrust)
                // {
                //     attitude_controller.sum++;
                // }
                get_offboard.thrust = command.thrust;
                get_offboard.type_mask = command.type_mask;
                get_offboard.mavros_state = true;
                // cm4_receive = 1;
                break;
            }

            case 84: {
                mavlink_set_position_target_local_ned_t command;
                mavlink_msg_set_position_target_local_ned_decode(&msg,&command);
                attitude_controller.r_y = command.x;
                attitude_controller.r_x = command.y;
                attitude_controller.r_z = -command.z;
                attitude_controller.r_Yaw_OptiTrack = -command.yaw_rate * RAD_TO_DEGREES;
                attitude_controller.sum++;
                // if(attitude_controller.sum == 180)
                // {
                //     attitude_controller.sum = 0;
                // }
                attitude_controller.mavlink_state = true;
                break;
            }
            // case 102:{
            //     mavlink_vision_position_estimate_t commandvoid 
            //     attitude_controller.r_x = command.y;
            //     attitude_controller.r_z = -command.z;
            //     attitude_controller.sum2++;
            //     if(attitude_controller.sum2 == 180)
            //     {
            //         attitude_controller.sum2 = 0;
            //     }
            //     break;
            // }
            default:
                // attitude_controller.sum = 0;
                break;
        }
    }
}
*/

/* MAVLink datastream rates in Hz */
// 在这里设置mavlink的发送速率，其中姿态为EXTRA1
static const uint16_t mavRates[] = {
    [MAV_DATA_STREAM_EXTENDED_STATUS] = 2, //2Hz
    [MAV_DATA_STREAM_RC_CHANNELS] = 5, //5Hz
    [MAV_DATA_STREAM_POSITION] = 2, //2Hz
    [MAV_DATA_STREAM_EXTRA1] = 300, //自定义
    [MAV_DATA_STREAM_EXTRA2] = 10 //2Hz
};

#define MAXSTREAMS ARRAYLEN(mavRates)

static uint16_t mavTicks[MAXSTREAMS];
static mavlink_message_t mavMsg;
static uint8_t mavBuffer[MAVLINK_MAX_PACKET_LEN];
static uint32_t lastMavlinkMessage = 0;


// 检测当前streamNum对应的消息是否需要发送，发送函数返回1，否则返回0
// streamN由自动飞行算法设计者自己定义，官方的程序也给了一些他们的定义以供参考，见common.h中的MAV_DATA_STREAM变量
static int mavlinkStreamTrigger(enum MAV_DATA_STREAM streamNum)
{
    // 获取当前消息的发送速率，如果为0则不发送，直接退出函数
    uint16_t rate = (uint16_t) mavRates[streamNum]; 
    if (rate == 0) {
        return 0;
    }

    // mavlink的滴答计时器如果倒计时为0，则准备发送，首先进行一个速率限幅
    if (mavTicks[streamNum] == 0) {
        // we're triggering now, setup the next trigger point
        if (rate >= TELEMETRY_MAVLINK_MAXRATE) {
            rate = TELEMETRY_MAVLINK_MAXRATE + 1;
        }
        // 复位计时器从新开始计时，这里整数除法，所以如果TELEMETRY_MAVLINK_MAXRATE和rate恰好相等，得到1，会导致其实频率被除以2了
        mavTicks[streamNum] = (TELEMETRY_MAVLINK_MAXRATE / rate);
        return 1;
    }
    // 如果mavlink的滴答计时器如果倒计时不为0，说明还未到发送时机，计时器减一，同时返回0表示不发送
    // count down at TASK_RATE_HZ
    mavTicks[streamNum]--;
    return 0;
}


static void mavlinkSerialWrite(uint8_t * buf, uint16_t length)
{
    for (int i = 0; i < length; i++)
        serialWrite(mavlinkPort, buf[i]);
}

static int16_t headingOrScaledMilliAmpereHoursDrawn(void)
{
    if (isAmperageConfigured() && telemetryConfig()->mavlink_mah_as_heading_divisor > 0) {
        // In the Connex Prosight OSD, this goes between 0 and 999, so it will need to be scaled in that range.
        return getMAhDrawn() / telemetryConfig()->mavlink_mah_as_heading_divisor;
    }
    // heading Current heading in degrees, in compass units (0..360, 0=north)
    return DECIDEGREES_TO_DEGREES(attitude.values.yaw);
}


void freeMAVLinkTelemetryPort(void)
{
    closeSerialPort(mavlinkPort);
    mavlinkPort = NULL;
    mavlinkTelemetryEnabled = false;
}

void initMAVLinkTelemetry(void)
{
    portConfig = findSerialPortConfig(FUNCTION_TELEMETRY_MAVLINK);
    mavlinkPortSharing = determinePortSharing(portConfig, FUNCTION_TELEMETRY_MAVLINK);
}

void configureMAVLinkTelemetryPort(void)
{
    if (!portConfig) {
        return;
    }

    baudRate_e baudRateIndex = portConfig->telemetry_baudrateIndex;
    if (baudRateIndex == BAUD_AUTO) {
        // default rate for minimOSD
        baudRateIndex = BAUD_2000000;
    }

    mavlinkPort = openSerialPort(portConfig->identifier, FUNCTION_TELEMETRY_MAVLINK, NULL, NULL, baudRates[baudRateIndex], TELEMETRY_MAVLINK_INITIAL_PORT_MODE, telemetryConfig()->telemetry_inverted ? SERIAL_INVERTED : SERIAL_NOT_INVERTED);

    if (!mavlinkPort) {
        return;
    }

    mavlinkTelemetryEnabled = true;
}

void checkMAVLinkTelemetryState(void)
{
    if (portConfig && telemetryCheckRxPortShared(portConfig, rxRuntimeState.serialrxProvider)) {
        if (!mavlinkTelemetryEnabled && telemetrySharedPort != NULL) {
            mavlinkPort = telemetrySharedPort;
            mavlinkTelemetryEnabled = true;
        }
    } else {
        bool newTelemetryEnabledValue = telemetryDetermineEnabledState(mavlinkPortSharing);

        if (newTelemetryEnabledValue == mavlinkTelemetryEnabled) {
            return;
        }

        if (newTelemetryEnabledValue)
            configureMAVLinkTelemetryPort();
        else
            freeMAVLinkTelemetryPort();
    }
}

void mavlinkSendSystemStatus(void)
{
    uint16_t msgLength;

    uint32_t onboardControlAndSensors = 35843;

    /*
    onboard_control_sensors_present Bitmask
    fedcba9876543210
    1000110000000011    For all   = 35843
    0001000000000100    With Mag  = 4100
    0010000000001000    With Baro = 8200
    0100000000100000    With GPS  = 16416
    0000001111111111
    */

    if (sensors(SENSOR_MAG))  onboardControlAndSensors |=  4100;
    if (sensors(SENSOR_BARO)) onboardControlAndSensors |=  8200;
    if (sensors(SENSOR_GPS))  onboardControlAndSensors |= 16416;

    uint16_t batteryVoltage = 0;
    int16_t batteryAmperage = -1;
    int8_t batteryRemaining = 100;

    if (getBatteryState() < BATTERY_NOT_PRESENT) {
        batteryVoltage = isBatteryVoltageConfigured() ? getBatteryVoltage() * 10 : batteryVoltage;
        batteryAmperage = isAmperageConfigured() ? getAmperage() : batteryAmperage;
        batteryRemaining = isBatteryVoltageConfigured() ? calculateBatteryPercentageRemaining() : batteryRemaining;
    }

    mavlink_msg_sys_status_pack(0, 200, &mavMsg,
        // onboard_control_sensors_present Bitmask showing which onboard controllers and sensors are present.
        //Value of 0: not present. Value of 1: present. Indices: 0: 3D gyro, 1: 3D acc, 2: 3D mag, 3: absolute pressure,
        // 4: differential pressure, 5: GPS, 6: optical flow, 7: computer vision position, 8: laser based position,
        // 9: external ground-truth (Vicon or Leica). Controllers: 10: 3D angular rate control 11: attitude stabilization,
        // 12: yaw position, 13: z/altitude control, 14: x/y position control, 15: motor outputs / control
        onboardControlAndSensors,
        // onboard_control_sensors_enabled Bitmask showing which onboard controllers and sensors are enabled
        onboardControlAndSensors,
        // onboard_control_sensors_health Bitmask showing which onboard controllers and sensors are operational or have an error.
        onboardControlAndSensors & 1023,
        // load Maximum usage in percent of the mainloop time, (0%: 0, 100%: 1000) should be always below 1000
        0,
        // voltage_battery Battery voltage, in millivolts (1 = 1 millivolt)
        batteryVoltage,
        // current_battery Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the current
        batteryAmperage,
        // battery_remaining Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot estimate the remaining battery
        batteryRemaining,
        // drop_rate_comm Communication drops in percent, (0%: 0, 100%: 10'000), (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)
        0,
        // errors_comm Communication errors (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)
        0,
        // errors_count1 Autopilot-specific errors
        0,
        // errors_count2 Autopilot-specific errors
        0,
        // errors_count3 Autopilot-specific errors
        0,
        // errors_count4 Autopilot-specific errors
        0);
    msgLength = mavlink_msg_to_send_buffer(mavBuffer, &mavMsg);
    mavlinkSerialWrite(mavBuffer, msgLength);
}

void mavlinkSendRCChannelsAndRSSI(void)
{
    uint16_t msgLength;
    mavlink_msg_rc_channels_raw_pack(0, 200, &mavMsg,
        // time_boot_ms Timestamp (milliseconds since system boot)
        millis(),
        // port Servo output port (set of 8 outputs = 1 port). Most MAVs will just use one, but this allows to encode more than 8 servos.
        0,
        // chan1_raw RC channel 1 value, in microseconds
        (rxRuntimeState.channelCount >= 1) ? rcData[0] : 0,
        // chan2_raw RC channel 2 value, in microseconds
        (rxRuntimeState.channelCount >= 2) ? rcData[1] : 0,
        // chan3_raw RC channel 3 value, in microseconds
        (rxRuntimeState.channelCount >= 3) ? rcData[2] : 0,
        // chan4_raw RC channel 4 value, in microseconds
        (rxRuntimeState.channelCount >= 4) ? rcData[3] : 0,
        // chan5_raw RC channel 5 value, in microseconds
        (rxRuntimeState.channelCount >= 5) ? rcData[4] : 0,
        // chan6_raw RC channel 6 value, in microseconds
        (rxRuntimeState.channelCount >= 6) ? rcData[5] : 0,
        // chan7_raw RC channel 7 value, in microseconds
        (rxRuntimeState.channelCount >= 7) ? rcData[6] : 0,
        // chan8_raw RC channel 8 value, in microseconds
        (rxRuntimeState.channelCount >= 8) ? rcData[7] : 0,
        // rssi Receive signal strength indicator, 0: 0%, 254: 100%
        scaleRange(getRssi(), 0, RSSI_MAX_VALUE, 0, 254));
    msgLength = mavlink_msg_to_send_buffer(mavBuffer, &mavMsg);
    mavlinkSerialWrite(mavBuffer, msgLength);
}

#if defined(USE_GPS)
void mavlinkSendPosition(void)
{
    uint16_t msgLength;
    uint8_t gpsFixType = 0;

    if (!sensors(SENSOR_GPS))
        return;

    if (!STATE(GPS_FIX)) {
        gpsFixType = 1;
    }
    else {
        if (gpsSol.numSat < GPS_MIN_SAT_COUNT) {
            gpsFixType = 2;
        }
        else {
            gpsFixType = 3;
        }
    }

    mavlink_msg_gps_raw_int_pack(0, 200, &mavMsg,
        // time_usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
        micros(),
        // fix_type 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
        gpsFixType,
        // lat Latitude in 1E7 degrees
        gpsSol.llh.lat,
        // lon Longitude in 1E7 degrees
        gpsSol.llh.lon,
        // alt Altitude in 1E3 meters (millimeters) above MSL
        gpsSol.llh.altCm * 10,
        // eph GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: 65535
        65535,
        // epv GPS VDOP horizontal dilution of position in cm (m*100). If unknown, set to: 65535
        65535,
        // vel GPS ground speed (m/s * 100). If unknown, set to: 65535
        gpsSol.groundSpeed,
        // cog Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: 65535
        gpsSol.groundCourse * 10,
        // satellites_visible Number of satellites visible. If unknown, set to 255
        gpsSol.numSat);
    msgLength = mavlink_msg_to_send_buffer(mavBuffer, &mavMsg);
    mavlinkSerialWrite(mavBuffer, msgLength);

    // Global position
    mavlink_msg_global_position_int_pack(0, 200, &mavMsg,
        // time_usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
        micros(),
        // lat Latitude in 1E7 degrees
        gpsSol.llh.lat,
        // lon Longitude in 1E7 degrees
        gpsSol.llh.lon,
        // alt Altitude in 1E3 meters (millimeters) above MSL
        gpsSol.llh.altCm * 10,
        // relative_alt Altitude above ground in meters, expressed as * 1000 (millimeters)
        getEstimatedAltitudeCm() * 10,
        // Ground X Speed (Latitude), expressed as m/s * 100
        0,
        // Ground Y Speed (Longitude), expressed as m/s * 100
        0,
        // Ground Z Speed (Altitude), expressed as m/s * 100
        0,
        // heading Current heading in degrees, in compass units (0..360, 0=north)
        headingOrScaledMilliAmpereHoursDrawn()
    );
    msgLength = mavlink_msg_to_send_buffer(mavBuffer, &mavMsg);
    mavlinkSerialWrite(mavBuffer, msgLength);

    mavlink_msg_gps_global_origin_pack(0, 200, &mavMsg,
        // latitude Latitude (WGS84), expressed as * 1E7
        GPS_home[GPS_LATITUDE],
        // longitude Longitude (WGS84), expressed as * 1E7
        GPS_home[GPS_LONGITUDE],
        // altitude Altitude(WGS84), expressed as * 1000
        0);
    msgLength = mavlink_msg_to_send_buffer(mavBuffer, &mavMsg);
    mavlinkSerialWrite(mavBuffer, msgLength);
}
#endif

// 如果直接使用该函数发送imu数据，机载电脑端通过mavros读取的数据是有问题的，因为该格式与mavros包的解码方式不匹配
// 故参考了发送imu数据格式符合mavros包解码规则的px4固件，在px4源码中的头文件mavlink_msg_scaled_imu.h中可以查看px4的发送方式为：
// mavlink_scaled_imu_t packet;
//     packet.time_boot_ms = time_boot_ms;
//     packet.xacc = xacc;
//     packet.yacc = yacc;
//     packet.zacc = zacc;
//     packet.xgyro = xgyro;
//     packet.ygyro = ygyro;
//     packet.zgyro = zgyro;
//     packet.xmag = xmag;
//     packet.ymag = ymag;
//     packet.zmag = zmag;
//     packet.temperature = temperature;
//     _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SCALED_IMU, (const char *)&packet, MAVLINK_MSG_ID_SCALED_IMU_MIN_LEN, MAVLINK_MSG_ID_SCALED_IMU_LEN, MAVLINK_MSG_ID_SCALED_IMU_CRC);
void mavlinkSendAttitude(void)
{
    uint16_t msgLength;
    mavlink_msg_attitude_pack(0, 200, &mavMsg,
        // time_boot_ms Timestamp (milliseconds since system boot)
        // 时间戳
        millis(),
        // roll Roll angle (rad)
        DECIDEGREES_TO_RADIANS(attitude.values.roll),
        // pitch Pitch angle (rad)
        DECIDEGREES_TO_RADIANS(-attitude.values.pitch),
        // yaw Yaw angle (rad)
        DECIDEGREES_TO_RADIANS(attitude.values.yaw),
        //rollspeed Roll angular speed (rad/s)
        DEGREES_TO_RADIANS(gyro.gyroADCf[FD_ROLL]),
        // pitchspeed Pitch angular speed (rad/s)
        DEGREES_TO_RADIANS(gyro.gyroADCf[FD_PITCH]),
        // yawspeed Yaw angular speed (rad/s)
        DEGREES_TO_RADIANS(gyro.gyroADCf[FD_YAW]));
    msgLength = mavlink_msg_to_send_buffer(mavBuffer, &mavMsg);
    mavlinkSerialWrite(mavBuffer, msgLength);
}

void mavlinkSendImuRawData(void)
{
    uint16_t msgLength;
    mavlink_msg_raw_imu_pack(0, 200, &mavMsg,
        // time_boot_ms Timestamp (milliseconds since system boot)
            millis(),
            // 下面三个是线加速度，单位是 G/1000，也即'毫重力加速度'
            // 为什么/2048.0f*1000？请看README的2024.10.29日的笔记
            (int16_t)((float)(acc.accADC[X])/2048.0f*1000),
            (int16_t)((float)(-acc.accADC[Y])/2048.0f*1000),
            (int16_t)((float)(-acc.accADC[Z])/2048.0f*1000),
            // 下面三个单位是毫弧度/秒，即millirad/s
            DEGREES_TO_RADIANS(gyro.gyroADCf[FD_ROLL])*1000,
            DEGREES_TO_RADIANS(-gyro.gyroADCf[FD_PITCH])*1000,
            DEGREES_TO_RADIANS(-gyro.gyroADCf[FD_YAW])*1000,
            // MPU6500没有磁力计，所以为0
            0,
            0,
            0                                               
        );
        
    msgLength = mavlink_msg_to_send_buffer(mavBuffer, &mavMsg);
    mavlinkSerialWrite(mavBuffer, msgLength);
}


void mavlinkSendHUDAndHeartbeat(void)
{
    uint16_t msgLength;
    float mavAltitude = 0;
    float mavGroundSpeed = 0;
    float mavAirSpeed = 0;
    float mavClimbRate = 0;

#if defined(USE_GPS)
    // use ground speed if source available
    if (sensors(SENSOR_GPS)) {
        mavGroundSpeed = gpsSol.groundSpeed / 100.0f;
    }
#endif

    mavAltitude = getEstimatedAltitudeCm() / 100.0;

    mavlink_msg_vfr_hud_pack(0, 200, &mavMsg,
        // airspeed Current airspeed in m/s
        mavAirSpeed,
        // groundspeed Current ground speed in m/s
        mavGroundSpeed,
        // heading Current heading in degrees, in compass units (0..360, 0=north)
        headingOrScaledMilliAmpereHoursDrawn(),
        // throttle Current throttle setting in integer percent, 0 to 100
        scaleRange(constrain(rcData[THROTTLE], PWM_RANGE_MIN, PWM_RANGE_MAX), PWM_RANGE_MIN, PWM_RANGE_MAX, 0, 100),
        // alt Current altitude (MSL), in meters, if we have sonar or baro use them, otherwise use GPS (less accurate)
        mavAltitude,
        // climb Current climb rate in meters/second
        mavClimbRate);
    msgLength = mavlink_msg_to_send_buffer(mavBuffer, &mavMsg);
    mavlinkSerialWrite(mavBuffer, msgLength);


    uint8_t mavModes = MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
    if (ARMING_FLAG(ARMED))
        mavModes |= MAV_MODE_FLAG_SAFETY_ARMED;

    uint8_t mavSystemType;
    switch (mixerConfig()->mixerMode)
    {
        case MIXER_TRI:
            mavSystemType = MAV_TYPE_TRICOPTER;
            break;
        case MIXER_QUADP:
        case MIXER_QUADX:
        case MIXER_Y4:
        case MIXER_VTAIL4:
            mavSystemType = MAV_TYPE_QUADROTOR;
            break;
        case MIXER_Y6:
        case MIXER_HEX6:
        case MIXER_HEX6X:
            mavSystemType = MAV_TYPE_HEXAROTOR;
            break;
        case MIXER_OCTOX8:
        case MIXER_OCTOFLATP:
        case MIXER_OCTOFLATX:
            mavSystemType = MAV_TYPE_OCTOROTOR;
            break;
        case MIXER_FLYING_WING:
        case MIXER_AIRPLANE:
        case MIXER_CUSTOM_AIRPLANE:
            mavSystemType = MAV_TYPE_FIXED_WING;
            break;
        case MIXER_HELI_120_CCPM:
        case MIXER_HELI_90_DEG:
            mavSystemType = MAV_TYPE_HELICOPTER;
            break;
        default:
            mavSystemType = MAV_TYPE_GENERIC;
            break;
    }

    // Custom mode for compatibility with APM OSDs
    uint8_t mavCustomMode = 1;  // Acro by default

    if (FLIGHT_MODE(ANGLE_MODE) || FLIGHT_MODE(HORIZON_MODE)) {
        mavCustomMode = 0;      //Stabilize
        mavModes |= MAV_MODE_FLAG_STABILIZE_ENABLED;
    }

    uint8_t mavSystemState = 0;
    if (ARMING_FLAG(ARMED)) {
        if (failsafeIsActive()) {
            mavSystemState = MAV_STATE_CRITICAL;
        }
        else {
            mavSystemState = MAV_STATE_ACTIVE;
        }
    }
    else {
        mavSystemState = MAV_STATE_STANDBY;
    }

    mavlink_msg_heartbeat_pack(0, 200, &mavMsg,
        // type Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)
        mavSystemType,
        // autopilot Autopilot type / class. defined in MAV_AUTOPILOT ENUM
        // 假装是px4
        MAV_AUTOPILOT_ARDUPILOTMEGA,
        // base_mode System mode bitfield, see MAV_MODE_FLAGS ENUM in mavlink/include/mavlink_types.h
        mavModes,
        // custom_mode A bitfield for use for autopilot-specific flags.
        mavCustomMode,
        // system_status System status flag, see MAV_STATE ENUM
        mavSystemState);
    msgLength = mavlink_msg_to_send_buffer(mavBuffer, &mavMsg);
    mavlinkSerialWrite(mavBuffer, msgLength);
}

void mavlinkSendHeartbeat(void)
{
    uint16_t msgLength;
    uint8_t mavModes = MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
    if (ARMING_FLAG(ARMED))
        mavModes |= MAV_MODE_FLAG_SAFETY_ARMED;

    uint8_t mavSystemType;
    switch (mixerConfig()->mixerMode)
    {
        case MIXER_TRI:
            mavSystemType = MAV_TYPE_TRICOPTER;
            break;
        case MIXER_QUADP:
        case MIXER_QUADX:
        case MIXER_Y4:
        case MIXER_VTAIL4:
            mavSystemType = MAV_TYPE_QUADROTOR;
            break;
        case MIXER_Y6:
        case MIXER_HEX6:
        case MIXER_HEX6X:
            mavSystemType = MAV_TYPE_HEXAROTOR;
            break;
        case MIXER_OCTOX8:
        case MIXER_OCTOFLATP:
        case MIXER_OCTOFLATX:
            mavSystemType = MAV_TYPE_OCTOROTOR;
            break;
        case MIXER_FLYING_WING:
        case MIXER_AIRPLANE:
        case MIXER_CUSTOM_AIRPLANE:
            mavSystemType = MAV_TYPE_FIXED_WING;
            break;
        case MIXER_HELI_120_CCPM:
        case MIXER_HELI_90_DEG:
            mavSystemType = MAV_TYPE_HELICOPTER;
            break;
        default:
            mavSystemType = MAV_TYPE_GENERIC;
            break;
    }

    // Custom mode for compatibility with APM OSDs
    uint8_t mavCustomMode = 1;  // Acro by default

    if (FLIGHT_MODE(ANGLE_MODE) || FLIGHT_MODE(HORIZON_MODE)) {
        mavCustomMode = 0;      //Stabilize
        mavModes |= MAV_MODE_FLAG_STABILIZE_ENABLED;
    }

    uint8_t mavSystemState = 0;
    if (ARMING_FLAG(ARMED)) {
        if (failsafeIsActive()) {
            mavSystemState = MAV_STATE_CRITICAL;
        }
        else {
            mavSystemState = MAV_STATE_ACTIVE;
        }
    }
    else {
        mavSystemState = MAV_STATE_STANDBY;
    }
        mavlink_msg_heartbeat_pack(0, 1, &mavMsg,
        // type Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)
        mavSystemType,
        // autopilot Autopilot type / class. defined in MAV_AUTOPILOT ENUM
        // 假装是px4，具体见README.md的2024.10.30日笔记
        autopilot_name,
        // base_mode System mode bitfield, see MAV_MODE_FLAGS ENUM in mavlink/include/mavlink_types.h
        mavModes,
        // custom_mode A bitfield for use for autopilot-specific flags.
        mavCustomMode,
        // system_status System status flag, see MAV_STATE ENUM
        mavSystemState);
    msgLength = mavlink_msg_to_send_buffer(mavBuffer, &mavMsg);
    mavlinkSerialWrite(mavBuffer, msgLength);
}

void processMAVLinkTelemetry(void)
{
    // is executed @ TELEMETRY_MAVLINK_MAXRATE rate
    if (mavlinkStreamTrigger(MAV_DATA_STREAM_EXTENDED_STATUS)) {
        mavlinkSendSystemStatus();
    }

    if (mavlinkStreamTrigger(MAV_DATA_STREAM_RC_CHANNELS)) {
        mavlinkSendRCChannelsAndRSSI();
    }

#ifdef USE_GPS
    if (mavlinkStreamTrigger(MAV_DATA_STREAM_POSITION)) {
        mavlinkSendPosition();
    }
#endif
    // 借用一下GPS的计数器，以2Hz的频率发送心跳信息
    if(mavlinkStreamTrigger(MAV_DATA_STREAM_POSITION)) {
        if(FLIGHT_MODE(OFFBOARD_MODE)) {
            autopilot_name = MAV_AUTOPILOT_ARDUPILOTMEGA;
        }
        else {
            autopilot_name = MAV_AUTOPILOT_GENERIC;
        }
        mavlinkSendHeartbeat();
    }
    
    if (mavlinkStreamTrigger(MAV_DATA_STREAM_EXTRA1)) {
            mavlinkSendAttitude();
            mavlinkSendImuRawData();

    }

}

// 下面的函数最终在task.c中的TASK_TELEMETRY任务被调用
void handleMAVLinkTelemetry(void)
{
    if (!mavlinkTelemetryEnabled) {
        return;
    }

    if (!mavlinkPort) {
        return;
    }

    uint32_t now = micros();
    if ((now - lastMavlinkMessage) >= TELEMETRY_MAVLINK_DELAY) {
        processMAVLinkTelemetry();
        lastMavlinkMessage = now;
    }
}


#endif




