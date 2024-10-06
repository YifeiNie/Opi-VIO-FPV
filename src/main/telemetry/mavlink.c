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
#include "drivers/rangefinder/rangefinder.h"
#include "drivers/rangefinder/flow_fusion.h"

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
#define TELEMETRY_MAVLINK_MAXRATE 180
#define TELEMETRY_MAVLINK_DELAY ((1000 * 1000) / TELEMETRY_MAVLINK_MAXRATE) //1000*1000/200us=5ms
#define GRAVITY_EARTH  (9.80665f)

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
static uint16_t rc_offboard_mode = 0;

float scale1;

/* MAVLink datastream rates in Hz */
static const uint8_t mavRates[] = {
    [MAV_DATA_STREAM_EXTENDED_STATUS] = 2, //2Hz
    [MAV_DATA_STREAM_RC_CHANNELS] = 40, //5Hz
    [MAV_DATA_STREAM_POSITION] = 60, //100Hz
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
//static uint8_t cm4_receive = 0;

//串口接收触发函数
static void mavlinkReceive(uint16_t c, void* data) {

    UNUSED(data);
    mavlink_message_t msg;
    mavlink_status_t status;

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
    else
    {
        baudRateIndex = BAUD_2000000;
    }

    mavlinkPort = openSerialPort(portConfig->identifier, FUNCTION_TELEMETRY_MAVLINK, mavlinkReceive, NULL, baudRates[baudRateIndex], TELEMETRY_MAVLINK_INITIAL_PORT_MODE, SERIAL_STOPBITS_1);

    if (!mavlinkPort) {
        return;
    }

    mavlinkTelemetryEnabled = true;
    if(mavlinkstate_position < 1)
    {
        //WifiInitHardware_Esp8266();
    if (acc.dev.acc_1G > 512 * 4) {
        scale1 = 8.0;
    } else if (acc.dev.acc_1G > 512 * 2) {
        scale1 = 4.0;
    } else if (acc.dev.acc_1G >= 512) {
        scale1 = 2.0;
    } else {
        scale1 = 1.0;
    }
        mavlinkstate_position++;
    }
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
        // system_status System status flag, see MAV_STATE ENUM
        mavSystemState);
    msgLength = mavlink_msg_to_send_buffer(mavBuffer, &mavMsg);
    mavlinkSerialWrite(mavBuffer, msgLength);
}



//  x = (acc.accADC[X]/ scale) * 0.001953125f;  //1/512u
//  y = (acc.accADC[Y]/ scale) * 0.001953125f;
//  z = (acc.accADC[Z]/ scale) * 0.001953125f;
//  uint64_t time_usec; /*< [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.*/
//  int16_t xacc; /*<  X acceleration (raw)*/
//  int16_t yacc; /*<  Y acceleration (raw)*/
//  int16_t zacc; /*<  Z acceleration (raw)*/
//  int16_t xgyro; /*<  Angular speed around X axis (raw)*/
//  int16_t ygyro; /*<  Angular speed around Y axis (raw)*/
//  int16_t zgyro; /*<  Angular speed around Z axis (raw)*/
//  int16_t xmag; /*<  X Magnetic field (raw)*/
//  int16_t ymag; /*<  Y Magnetic field (raw)*/
//  int16_t zmag; /*<  Z Magnetic field (raw)*/
//  uint8_t id; /*<  Id. Ids are numbered from 0 and map to IMUs numbered from 1 (e.g. IMU1 will have a message with id=0)*/
//  int16_t temperature; /*< [cdegC] Temperature, 0: IMU does not provide temper

void mavlinkSendImuRaw(void)
{
    uint16_t msgLength;
    mavlink_msg_raw_imu_pack(0, 200, &mavMsg,
        // time_boot_ms Timestamp (milliseconds since system boot)
            millis(),
            (int16_t)((float)(acc.accADC[X])/scale1*1.953125*GRAVITY_EARTH),
            (int16_t)((float)(acc.accADC[Y])/scale1*1.953125*GRAVITY_EARTH),
            (int16_t)((float)(acc.accADC[Z])/scale1*1.953125*GRAVITY_EARTH),
            (int16_t)(gyro.gyroADCf[FD_ROLL] * 10.0f),
            (int16_t)(gyro.gyroADCf[FD_PITCH] * 10.0f),
            (int16_t)(gyro.gyroADCf[FD_YAW] * 10.0f),
            0,
            0,
            0,
            18,
            0
        );
        
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
        //rollspeed Roll angular speed (rad/s)
        DEGREES_TO_RADIANS(gyro.gyroADCf[FD_ROLL]),
        // pitchspeed Pitch angular speed (rad/s)
        DEGREES_TO_RADIANS(gyro.gyroADCf[FD_PITCH]),
        // yawspeed Yaw angular speed (rad/s)
        DEGREES_TO_RADIANS(gyro.gyroADCf[FD_YAW])
        );
        
    msgLength = mavlink_msg_to_send_buffer(mavBuffer, &mavMsg);
    mavlinkSerialWrite(mavBuffer, msgLength);
}

void mavlinksendAltitude(void) //ID 141
{
    uint16_t msgLength;

    mavlink_msg_altitude_pack(0, 200, &mavMsg,
    millis(),
    Get_Opti_Vec_X(),  //altitude_monotonic(x)
    Get_Opti_Vec_Y(),  //altitude_amsl(y)
    Get_Opti_Vec_Z(),  //altitude_local(z)
    0, //altitude_relative
    0,  //altitude_terrain
    0  //bottom_clearance
    );
    msgLength = mavlink_msg_to_send_buffer(mavBuffer, &mavMsg);
    mavlinkSerialWrite(mavBuffer, msgLength);
}

void mavlinkSendHUD(void) //ID 74
{
    uint16_t msgLength;

    //airspeed groundspeed heading throttle alt climb
    mavlink_msg_vfr_hud_pack(0, 200, &mavMsg,
        0,
        0,
        // heading Current heading in degrees, in compass units (0..360, 0=north)
        // headingOrScaledMilliAmpereHoursDrawn(),
        rc_offboard_mode,
        // throttle Current throttle setting in integer percent, 0 to 100
        scaleRange(constrain(rcData[THROTTLE], PWM_RANGE_MIN, PWM_RANGE_MAX), PWM_RANGE_MIN, PWM_RANGE_MAX, 0, 100),
        // alt Current altitude (MSL), in meters, if we have sonar or baro use them, otherwise use GPS (less accurate)
        // attitude_controller.sum,
        //Timestamp_out
        0,
        attitude_controller.sum
        );
    msgLength = mavlink_msg_to_send_buffer(mavBuffer, &mavMsg);
    mavlinkSerialWrite(mavBuffer, msgLength);
}


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

void processMAVLinkTelemetry(void)
{

    if(mavlinkStreamTrigger(MAV_DATA_STREAM_POSITION)) {
        mavlinkSendHUD();
        mavlinksendAltitude();
        if(attitude_controller.sum >= 180)
        {
            attitude_controller.sum = 0;
        }
        if(FLIGHT_MODE(ANGLE_RATE_HOLD_MODE))
        {
            rc_offboard_mode = 1;
        }else
        {
            rc_offboard_mode = 0;
        }
    }

    mavlinkSendAttitude();
    mavlinkSendImuRaw();
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
    if ((now - lastMavlinkMessage) >= TELEMETRY_MAVLINK_DELAY) {
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
