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

#include "build/debug.h"
#include "build/build_config.h"

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
#include "drivers/light_led.h"

#include "config/config.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"
#include "fc/rc.h"

#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/failsafe.h"
#include "flight/position.h"
#include "flight/alt_ctrl.h"
#include "flight/kalman_filter.h"

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
#include "sensors/rangefinder.h"

#include "telemetry/telemetry.h"
#include "telemetry/mavlink.h"

// mavlink library uses unnames unions that's causes GCC to complain if -Wpedantic is used
// until this is resolved in mavlink library - ignore -Wpedantic for mavlink code
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include "common/mavlink.h"
#pragma GCC diagnostic pop

#define TELEMETRY_MAVLINK_INITIAL_PORT_MODE MODE_RXTX
#define TELEMETRY_MAVLINK_MAXRATE 200
#define TELEMETRY_MAVLINK_DELAY ((1000 * 1000) / TELEMETRY_MAVLINK_MAXRATE) //1000*1000/200us=5ms

#define WIFI_AT         "AT\r\n"
#define WIFI_CWMODE     "AT+CWMODE=1\r\n"
#define WIFI_RST        "AT+RST\r\n"
#define WIFI_CWJAP      "AT+CWJAP=\"OptiTrack\",\"12345678\"\r\n"
// #define WIFI_CWJAP      "AT+CWJAP=\"NeSC\",\"nesc2022\"\r\n"
#define WIFI_CIPMUX     "AT+CIPMUX=0\r\n"
// #define WIFI_CIPSTART   "AT+CIPSTART=\"UDP\",\"192.168.31.142\",14555,9000,0\r\n"
#define WIFI_CIPSTART   "AT+CIPSTART=\"UDP\",\"192.168.50.24\",14555,9000,0\r\n"
//#define WIFI_CIPSTART   "AT+CIPSTART=\"UDP\",\"192.168.50.109\",14555,9000,0\r\n"
#define WIFI_CIPMODE    "AT+CIPMODE=1\r\n"
#define WIFI_CIPSEND    "AT+CIPSEND\r\n"


extern uint16_t rssi; // FIXME dependency on mw.c

static serialPort_t *mavlinkPort = NULL;
static const serialPortConfig_t *portConfig;

static bool mavlinkTelemetryEnabled =  false;
static portSharing_e mavlinkPortSharing;

/* MAVLink datastream rates in Hz */
static const uint8_t mavRates[] = {
    [MAV_DATA_STREAM_EXTENDED_STATUS] = 2, //2Hz
    [MAV_DATA_STREAM_RC_CHANNELS] = 40, //5Hz
    [MAV_DATA_STREAM_POSITION] = 1, //100Hz
    [MAV_DATA_STREAM_EXTRA1] = 40, //10Hz
    [MAV_DATA_STREAM_EXTRA2] = 100, //100Hz
    [MAV_DATA_STREAM_EXTRA3] = 5
};

#define MAXSTREAMS ARRAYLEN(mavRates)

static uint8_t mavTicks[MAXSTREAMS];
static mavlink_message_t mavMsg;
static uint8_t mavBuffer[MAVLINK_MAX_PACKET_LEN];
static uint32_t lastMavlinkMessage = 0;
static uint32_t mavlinkstate_position = 0;
static uint8_t cm4_receive = 0;

//串口接收触发函数
static void mavlinkReceive(uint16_t c, void* data) {

    UNUSED(data);
    mavlink_message_t msg;
    mavlink_status_t status;
    // uint8_t mav_type;
    // uint8_t mav_autopilot;
    // uint8_t mav_basemode;
    // uint32_t mav_custommode;
    // uint8_t mav_systemstatus;
    // uint8_t mav_version;

    if (mavlink_parse_char(MAVLINK_COMM_0, (uint8_t)c, &msg, &status)) {
        switch(msg.msgid) {
            // receive heartbeat
            case 0: {
                cm4_receive = 1;
                // mavlink_heartbeat_t command;
                // mavlink_msg_heartbeat_decode(&msg,&command);
                // mav_custommode = command.custom_mode;
                // mav_type = command.type;
                // mav_autopilot = command.autopilot;
                // mav_basemode = command.base_mode;
                // mav_systemstatus = command.custom_mode;
                // mav_version = command.mavlink_version;
                // mavlinkSendHeartbeat();
                // mavlinkSendHUD();
                // mavlinkSendAttitude();
                break;
            }
            // setpoint command
            // case 81: {
            //     mavlink_manual_setpoint_t command;
            //     mavlink_msg_manual_setpoint_decode(&msg,&command);
            //     attitude_controller.altitude_thrust = -command.thrust * 100;
            //     attitude_controller.roll = command.roll;   //maybe need normalization but this should be done in the JeVois
            //     attitude_controller.pitch = -command.pitch;
            //     attitude_controller.yaw = command.yaw;

            //     // DEBUG_SET(DEBUG_UART,1,command.time_boot_ms);	
            //     // DEBUG_SET(DEBUG_UART,3,uart_altitude);
            //     // DEBUG_SET(DEBUG_COMMAND,0,uart_altitude);
            //     // DEBUG_SET(DEBUG_COMMAND,1,uart_roll / 3.14 * 180);
            //     // DEBUG_SET(DEBUG_COMMAND,2,uart_pitch / 3.14 * 180);
            //     // DEBUG_SET(DEBUG_COMMAND,3,uart_yaw / 3.14 * 180);
            //     break;
            // }

            case 82:{
                mavlink_set_attitude_target_t command;
                mavlink_msg_set_attitude_target_decode(&msg,&command);
                get_offboard.q[0] = command.q[0];  //w
                get_offboard.q[1] = command.q[1];  //x
                get_offboard.q[2] = command.q[2];  //y
                get_offboard.q[3] = command.q[3];  //z
                get_offboard.roll_rate = command.body_roll_rate;
                get_offboard.pitch_rate = - command.body_pitch_rate;
                get_offboard.yaw_rate = command.body_yaw_rate;
                get_offboard.thrust = command.thrust;
                get_offboard.type_mask = command.type_mask;
                // attitude_controller.sum++;
                // if(attitude_controller.sum == 100)
                // {
                //     attitude_controller.sum = 0;
                // }
                break;
            }

            case 84: {
                mavlink_set_position_target_local_ned_t command;
                mavlink_msg_set_position_target_local_ned_decode(&msg,&command);
                attitude_y_controller.setpoint_input = command.afx;
                attitude_x_controller.setpoint_input = command.afy;
                attitude_controller.r_Yaw_OptiTrack = command.afz;
                // attitude_controller.sum1++;
                // if(attitude_controller.sum1 == 180)
                // {
                //     attitude_controller.sum1 = 0;
                // }
                // ledSet(1, state1); 
                // state1 = !state1;
                break;
            }
            case 102:{
                mavlink_vision_position_estimate_t command;
                mavlink_msg_vision_position_estimate_decode(&msg,&command);
                attitude_controller.r_y = command.x;
                attitude_controller.r_x = command.y;
                attitude_controller.r_z = -command.z;
                attitude_controller.sum2++;
                if(attitude_controller.sum2 == 180)
                {
                    attitude_controller.sum2 = 0;
                }
                // attitude_controller.r_Roll = command.roll;
                // attitude_controller.r_Pitch = command.pitch;
                // attitude_controller.r_Yaw = command.yaw;
                // attitude_controller.sum++;
                // kalman_filter1.Z_current->element[0] = -command.z;
                // kalman_filter1.optitrack_update = 1;
                // attitude_controller.sum = 0;
                // mavlinkSendHUD();
                // if(state1 == 1)
                // {
                // mavlinksendAltitude();
                //     state1 = 0;
                // }
                break;
            }
            // case 141:{
            //     mavlink_altitude_t command;
            //     mavlink_msg_altitude_decode(&msg,&command);
            //     attitude_controller.r_y = command.altitude_amsl;
            //     attitude_controller.r_x = command.altitude_local;
            //     attitude_controller.r_z = -command.altitude_monotonic;
            //     attitude_controller.r_Roll = command.altitude_relative;
            //     attitude_controller.r_Pitch = command.altitude_terrain;
            //     attitude_controller.r_Yaw = command.bottom_clearance;
            //     attitude_controller.sum++;
            //     break;
            // }
            default:
                cm4_receive = 0;
                // attitude_controller.sum = 0;
                break;
        }
    }
}


static int mavlinkStreamTrigger(enum MAV_DATA_STREAM streamNum)
{
    uint8_t rate = (uint8_t) mavRates[streamNum];
    if (rate == 0) {
        return 0;
    }

    if (mavTicks[streamNum] == 0) {
        // we're triggering now, setup the next trigger point
        if (rate > TELEMETRY_MAVLINK_MAXRATE) {
            rate = TELEMETRY_MAVLINK_MAXRATE;
        }

        mavTicks[streamNum] = (TELEMETRY_MAVLINK_MAXRATE / rate);
        return 1;
    }

    // count down at TASK_RATE_HZ
    mavTicks[streamNum]--;
    return 0;
}


static void mavlinkSerialWrite(uint8_t * buf, uint16_t length)
{
    for (int i = 0; i < length; i++)
        serialWrite(mavlinkPort, buf[i]);
}

// static int16_t headingOrScaledMilliAmpereHoursDrawn(void)
// {
//     if (isAmperageConfigured() && telemetryConfig()->mavlink_mah_as_heading_divisor > 0) {
//         // In the Connex Prosight OSD, this goes between 0 and 999, so it will need to be scaled in that range.
//         return getMAhDrawn() / telemetryConfig()->mavlink_mah_as_heading_divisor;
//     }
//     // heading Current heading in degrees, in compass units (0..360, 0=north)
//     return DECIDEGREES_TO_DEGREES(attitude.values.yaw);
// }


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
    else
    {
        baudRateIndex = BAUD_2000000;
    }

    mavlinkPort = openSerialPort(portConfig->identifier, FUNCTION_TELEMETRY_MAVLINK, mavlinkReceive, NULL, baudRates[baudRateIndex], TELEMETRY_MAVLINK_INITIAL_PORT_MODE, SERIAL_STOPBITS_1);

    if (!mavlinkPort) {
        return;
    }

    mavlinkTelemetryEnabled = true;
    // if(mavlinkstate_position < 1)
    // {
    //     WifiInitHardware_Esp8266();
    //     mavlinkstate_position++;
    // }
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

void mavlinkSendHeartbeat(void)  //ID 0
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
    if ( FLIGHT_MODE(RANGEFINDER_MODE))
        mavCustomMode = 2; // Alt Hold

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
        MAV_AUTOPILOT_GENERIC,
        // base_mode System mode bitfield, see MAV_MODE_FLAGS ENUM in mavlink/include/mavlink_types.h
        mavModes,
        // custom_mode A bitfield for use for autopilot-specific flags.
        mavCustomMode,
        // (uint8_t)attitude_controller.sum,
        // system_status System status flag, see MAV_STATE ENUM
        mavSystemState);
    msgLength = mavlink_msg_to_send_buffer(mavBuffer, &mavMsg);
    mavlinkSerialWrite(mavBuffer, msgLength);
}


void mavlinkSendAttitude(void) //ID 30
{
    uint16_t msgLength;
    mavlink_msg_attitude_pack(0, 200, &mavMsg,
        // time_boot_ms Timestamp (milliseconds since system boot)
        millis(),
        // roll Roll angle (rad)
        DECIDEGREES_TO_RADIANS(attitude.values.roll),
        // pitch Pitch angle (rad)
        DECIDEGREES_TO_RADIANS(-attitude.values.pitch),
        // yaw Yaw angle (rad)
        DECIDEGREES_TO_RADIANS(attitude.values.yaw),
        // rollspeed Roll angular speed (rad/s)
        DEGREES_TO_RADIANS(gyro.gyroADCf[FD_ROLL]),
        // pitchspeed Pitch angular speed (rad/s)
        DEGREES_TO_RADIANS(gyro.gyroADCf[FD_PITCH]),
        // yawspeed Yaw angular speed (rad/s)
        DEGREES_TO_RADIANS(gyro.gyroADCf[FD_YAW])
        // attitude_controller.r_x,  //roll
        // attitude_controller.r_y,  //pitch
        // attitude_controller.r_z, //yaw
        // // attitude_controller.r_Roll,
        // attitude_controller.r_Roll,  //rollspeed
        // // attitude_controller.r_Yaw 
        // attitude_controller.r_Pitch,  //pitchspeed
        // attitude_controller.r_Yaw  //yawspeed
        );
        
    msgLength = mavlink_msg_to_send_buffer(mavBuffer, &mavMsg);
    mavlinkSerialWrite(mavBuffer, msgLength);
}

void mavlinksendAltitude(void) //ID 141
{
    uint16_t msgLength;
    // float mavAltitude_Measure = 0;
    // float mavVel_Hat_current = 0;
    // float mavVel_Measure = 0;
    // float mavAltitude_Hat_current = 0;
    // float mavPID_vel_output = 0;
    // float mavPID_height_output = 0;
    // float mav_vel_throttle = 0;

    
    // mavVel_Measure = Get_Acc_bias_kalman(); //速度测量值  (airspeed)
    // mavVel_Hat_current = Get_Vel_Kalman(); //速度最优估计值 (groundspeed)
    // // mavAltitude_Measure = rangefinderGetLatestAltitude(); //高度测量值 (altitude)
    // mavAltitude_Measure = Get_z_measure();/home/nesc/single_ws/src/quarotor_fee
    // mavAltitude_Hat_current = Get_Alt_Kalman(); //高度最优估计值 (climb)
    // mavPID_height_output = Get_Height_PID_Output(); //获取外环pid结果
    // mavPID_vel_output = Get_Velocity_PID_Output(); //获取内环pid结果
    // mav_vel_throttle = Get_Velocity_throttle();

    mavlink_msg_altitude_pack(0, 200, &mavMsg,
    millis(),
    // attitude_send.ROLL/180*3.1415926,
    // attitude_send.PITCH/180*3.1415926,
    // attitude_send.YAW/180*3.1415926,
    // attitude_send.ROLL_rate,
    // -attitude_send.PITCH_rate,
    // -attitude_send.YAW_rate

    -Get_Height_PID_Output(0),  //altitude_monotonic
    Get_Height_PID_Output(1),  //altitude_amsl
    Get_Height_PID_Output(2),  //altitude_local
    Get_Velocity_PID_Output(0)*180.0/3.1415926f, //altitude_relative
    Get_Velocity_PID_Output(1)*180.0/3.1415926f,  //altitude_terrain
    Get_Velocity_PID_Output(2)  //bottom_clearance
    // attitude_controller.r_x,  //monotonic
    // attitude_controller.r_y,  //amsl
    // attitude_controller.r_z, //loacl
    // // attitude_controller.r_Roll,
    // attitude_controller.r_Roll,  //relative
    // // attitude_controller.r_Yaw 
    // attitude_controller.r_Pitch,  //terrain
    // attitude_controller.r_Yaw  //clearance
    );
    msgLength = mavlink_msg_to_send_buffer(mavBuffer, &mavMsg);
    mavlinkSerialWrite(mavBuffer, msgLength);
}

void mavlinkSendHUD(void) //ID 74
{
    uint16_t msgLength;
    // float mav_z_ierror = 0;
    // float mav_z_throttle = 0;
    // float mavAirSpeed = 0;
    // float mavClimbRate = 0;
    // float mav_Yaw = 0;


    // mav_z_ierror = Get_Height_PID_Error();
    // mav_z_throttle = Get_Velocity_throttle();
    // mavClimbRate = attitude_controller.r_y;
    // mav_Yaw = Get_vrpn_Yaw();
    //airspeed  groundspeed  heading throttle alt climb
    mavlink_msg_vfr_hud_pack(0, 200, &mavMsg,
        // airspeed Current airspeed in m/s
        // Get_Velocity_throttle(0),  //roll
        // Get_Velocity_throttle(1),  //pitch
        // Get_Velocity_LpFiter(0),
        // Get_Velocity_LpFiter(1),
        state_check.rc_receive,
        state_check.feedforward_apply,
        // attitude_controller.r_x_lowpassfilter,
        // attitude_controller.r_y_lowpassfilter,
        // attitude_controller.r_z, //yaw
        // attitude_controller.Error_x,
        // attitude_controller.Error_x_filter,
        // groundspeed Current ground speed in m/s
        //attitude_controller.r_Pitch,
        // heading Current heading in degrees, in compass units (0..360, 0=north)
        attitude_controller.sum2,
        //headingOrScaledMilliAmpereHoursDrawn(),
        // throttle Current throttle setting in integer percent, 0 to 100
        scaleRange(constrain(rcData[THROTTLE], PWM_RANGE_MIN, PWM_RANGE_MAX), PWM_RANGE_MIN, PWM_RANGE_MAX, 0, 100),
        // alt Current altitude (MSL), in meters, if we have sonar or baro use them, otherwise use GPS (less accurate)
        //attitude_controller.r_Yaw,
        attitude_controller.test_anglerate_setpoint[0],
        //Get_Velocity_LpFiter(2), //yaw
        // attitude_controller.Error_y
        //Get_Velocity_throttle(2)
        attitude_controller.test_anglerate_setpoint[1]
        //attitude_controller.sum1,
        //attitude_controller.sum
        );
    msgLength = mavlink_msg_to_send_buffer(mavBuffer, &mavMsg);
    mavlinkSerialWrite(mavBuffer, msgLength);
}


// void mavlinkLocalPositionNed(void) //ID 32
// {
//     uint16_t msgLength;
//     float r_x = Get_vrpn_x();
//     float r_y = Get_vrpn_y();
//     float r_z = Get_vrpn_z();

//     // float r_Roll = Get_vrpn_Roll();
//     // float r_Pitch = Get_vrpn_Pitch();
//     // float r_Yaw = Get_vrpn_Yaw();
//     mavlink_msg_local_position_ned_pack(0, 200, &mavMsg,
//     micros(),
//     Get_Alt_Kalman(),
//     Get_Vel_Kalman(),
//     Get_Acc_bias_kalman(),
//     0,
//     0,
//     0
//     );
//     msgLength = mavlink_msg_to_send_buffer(mavBuffer, &mavMsg);
//     mavlinkSerialWrite(mavBuffer, msgLength);
// }

void mavlinkLocalPositionNedCov(void)  //ID 64
{
    uint16_t msgLength;
    mavlink_msg_local_position_ned_cov_pack(0, 200, &mavMsg,
        micros(),
        0,
        Get_Position_LpFiter(1),
        Get_Position_LpFiter(0),
        -Get_Position_LpFiter(2),
        Get_Velocity_LpFiter(1),
        Get_Velocity_LpFiter(0),
        -Get_Velocity_LpFiter(2),
        attitude_controller.error_angle,
        attitude_controller.error_angle_output,
        attitude_controller.r_Yaw_OptiTrack,
        0
    );
    msgLength = mavlink_msg_to_send_buffer(mavBuffer, &mavMsg);
    mavlinkSerialWrite(mavBuffer, msgLength);
}
// void mavlinkLocalPositionNedSystemGlobalOffset(void) //89
// {
//     uint16_t msgLength;
//     mavlink_msg_local_position_ned_system_global_offset_pack(0, 200, &mavMsg,
//         millis(),
//         Get_Position_LpFiter(1),
//         Get_Position_LpFiter(0),
//         -Get_Position_LpFiter(2),
//         Get_Velocity_LpFiter(0),
//         Get_Velocity_LpFiter(1),
//         Get_Velocity_LpFiter(2)
//     );
//     msgLength = mavlink_msg_to_send_buffer(mavBuffer, &mavMsg);
//     mavlinkSerialWrite(mavBuffer, msgLength);
// }

void processMAVLinkTelemetry(void)
{

    // testdatanow = attitude_controller.r_Yaw;
    // if(testdatanow != testdatalast)
    // {
   
    // serialWrite(mavlinkPort,41);
        // mavlinksendAltitude();
    // serialPrint(mavlinkPort, "A");

    // if(state == 1)
    // {
    //     serialPrint(mavlinkPort, "B");
    //     state = 0;
    // }
    //    }
    //     attitude_controller.sum1++;

    // }
    // testdatalast = testdatanow;
    // is executed @ TELEMETRY_MAVLINK_MAXRATE rate
    // if (mavlinkStreamTrigger(MAV_DATA_STREAM_EXTENDED_STATUS)) {
    //     mavlinkSendSystemStatus();
    // }
    // if(mavlinkStreamTrigger(MAV_DATA_STREAM_POSITION)) {
    //     serialPrint(mavlinkPort,"\r\n");
    //     serialWrite(mavlinkPort,attitude_controller.sum);
    //     serialWrite(mavlinkPort,attitude_controller.sum1);
    //     serialWrite(mavlinkPort,attitude_controller.sum2);
    //     serialPrint(mavlinkPort,"\r\n");
    // mavlinkLocalPositionNed();
    // if(mavlinkStreamTrigger(MAV_DATA_STREAM_POSITION)) {
    if(mavlinkStreamTrigger(MAV_DATA_STREAM_POSITION)) {
        mavlinkSendHeartbeat();
   
    // mavlinkSendHUD();
    }

    //mavlinkSendHUD();
    mavlinksendAltitude();
    mavlinkSendHUD();
    mavlinkSendAttitude();
    mavlinkLocalPositionNedCov();
    // mavlinkLocalPositionNedSystemGlobalOffset();
    // mavlinkLocalPositionNed();
    // serialWrite(mavlinkPort,"success");
    // serialWrite(mavlinkPort,62);
    // serialPrint(mavlinkPort,"abcd");
    // mavlinkSendAttitude();
    

    // if (mavlinkStreamTrigger(MAV_DATA_STREAM_EXTRA1)) {
    // mavlinkSendHeartbeat();

    // }

}

void handleMAVLinkTelemetry(void)
{
    if (!mavlinkTelemetryEnabled) {
        return;
    }

    if (!mavlinkPort) {
        return;
    }

    uint32_t now = micros();
    if ((now - lastMavlinkMessage) >= TELEMETRY_MAVLINK_DELAY && cm4_receive == 1) {
        processMAVLinkTelemetry();
        lastMavlinkMessage = now;
    }
}

void WifiInitHardware_Esp8266(void)
{
    uint32_t nowtime;
    uint8_t c;
    #ifdef USE_WIFI_ESP8266
        nowtime = millis();
        serialPrint(mavlinkPort, WIFI_AT);
        delay(10);
        nowtime = millis();
        c = serialRead(mavlinkPort);
        while(c != 'O')
        {
            if(millis()-nowtime > 200){
                //serialPrint(mavlinkPort, "wait1\r\n");
                break;
            }
        };
        c = 0;

        serialPrint(mavlinkPort, WIFI_CWMODE);
        nowtime = millis();
        delay(1000);
        c = serialRead(mavlinkPort);
        while(c != 'O')
        { 
            if(millis()-nowtime > 500){
                //serialPrint(mavlinkPort, "wait2\r\n");     
                break;               
            }  
        };
        c = 0;

//         if(wifi_uart_baud == 0){
//             serialPrint(mavlinkPort, "AT+UART_DEF=230400,8,1,0,0\r\n");
//             nowtime = millis();
//             c = serialRead(mavlinkPort);
//             while(c != 'O')
//             { 
//                 if(millis()-nowtime > 500){
//                     serialPrint(mavlinkPort, "wait0\r\n");     
//                     break;               
//                 }  
//             };
//         }

        serialPrint(mavlinkPort, WIFI_RST);
        nowtime = millis();
        c = serialRead(mavlinkPort);
        while(c != 'O')
        {
            if(millis()-nowtime > 200){
                //serialPrint(mavlinkPort, "wait3\r\n");
                break;
            }
        };
        delay(1000);
        delay(1000);
        delay(1000);
        c = 0;

        //serialPrint(mavlinkPort, "AT+CWJAP=\"FAST_0530\",\"13525755559\"\r\n");
        //serialPrint(mavlinkPort, "AT+CWJAP=\"mi12\",\"11111111\"\r\n");
        //serialPrint(mavlinkPort, "AT+CWJAP=\"NeSC\",\"nesc2022\"\r\n");
        serialPrint(mavlinkPort, WIFI_CWJAP);
        nowtime = millis();
        delay(1000);
        delay(1000);
        delay(1000);
        delay(1000);
        delay(1000);
        delay(1000);
        delay(1000);
        delay(1000);
        delay(1000);
        delay(1000);

        serialPrint(mavlinkPort,WIFI_CIPMUX);
        nowtime = millis();
        c = serialRead(mavlinkPort);
        while(c != 'O')
        {
            if(millis()-nowtime > 2000){
                //serialPrint(mavlinkPort, "wait5\r\n");
                break;
            }
        };
        delay(200);
        c = 0;

        serialPrint(mavlinkPort,WIFI_CIPSTART);
        delay(1000);
        serialPrint(mavlinkPort,WIFI_CIPSTART);
        delay(1000);
        nowtime = millis();
        c = serialRead(mavlinkPort);
        while(c != 'O')
        {
            if(millis()-nowtime > 1000){
                //serialPrint(mavlinkPort, "wait6\r\n");
                break;
            }
        };
        delay(1000);
        c = 0;

        serialPrint(mavlinkPort,WIFI_CIPMODE);
        nowtime = millis();
        c = serialRead(mavlinkPort);
        while(c != 'O')
        {
            if(millis()-nowtime > 2000){
                serialPrint(mavlinkPort, "wait7\r\n");
                break;
            }
        };
        delay(200);
        c = 0;

        serialPrint(mavlinkPort,WIFI_CIPSEND);
        delay(200);

        nowtime = millis();
#endif
}
#endif
