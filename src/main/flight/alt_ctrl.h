#pragma once

#include "common/time.h"

#include "sensors/rangefinder.h"
#include "kalman_filter.h"

typedef struct pid
{
    float P;
    float I;
    float D;

    float Error1;   //Error[n-1]
    float Error2;   //Error[n-2]
    float iError;   //Error[n]
    float IiError;
    float DiError;

}pid_t;
typedef struct controller
{
    pid_t pid;

    float setpoint;
    float setpoint_input;
    float output;
    float output_min;
    float output_max;
    float throttle;

    float input_error_range;

    float dt;
    float process_dt;

}controller_t;


typedef struct attitude_ctrl
{
    float r_x;
    float r_y;
    float r_z;

    float r_x_last;
    float r_y_last;
    float r_z_last;

    float r_x_lowpassfilter;
    float r_y_lowpassfilter;
    float r_z_lowpassfilter;

    float r_x_lowpassfilter_last;
    float r_y_lowpassfilter_last;
    float r_z_lowpassfilter_last;

    float Error_x;
    float Error_y;
    float Error_z;

    float Error_x_filter;
    float Error_y_filter;
    float Error_z_filter;

    float Error_x_filter_last;
    float Error_y_filter_last;
    float Error_z_filter_last;

    float r_Roll;
    float r_Pitch;
    float r_Yaw;
    float r_Yaw_OptiTrack;

    float roll_rate;  //rad/s
    float pitch_rate;  //rad/s
    float yaw_rate;   //rad/s    

    float error_angle;    
    float error_angle_output;                                                                                              

    float altitude_thrust;  //0-1

    float error_angle_rate[3];
    float test_anglerate_setpoint[3];

    uint16_t sum;
    uint16_t sum1;
    uint16_t sum2;

    float usec;
    float pidupdate_dt;
    float filter_dt;
    int16_t flight_mode;

    bool mavlink_state;
}attitude_ctrl_t;

typedef struct attitude_send
{
    float ROLL;
    float PITCH;
    float YAW;

    float ROLL_rate;
    float PITCH_rate;
    float YAW_rate;

    float test_yaw;
}attitude_send_t;

typedef struct selectmode
{
    uint8_t angle_mode;
    uint8_t angularrate_mode;
}selectmode_t;

typedef struct get_offboard
{
    float q[4];  //w,x,y,z
    float roll_angle;
    float pitch_angle;
    float yaw_angle;
    float roll_rate;
    float pitch_rate;
    float yaw_rate;
    float thrust;
    uint8_t type_mask; //mode
    bool mavros_state;
}get_offboard_t;

typedef struct state_check
{
    float rc_receive;
    float feedforward_apply;
}state_check_t;

extern state_check_t state_check;
extern attitude_send_t attitude_send;
extern attitude_ctrl_t attitude_controller;
extern selectmode_t mode_seclct;
extern get_offboard_t get_offboard;

extern controller_t attitude_x_controller;
extern controller_t attitude_y_controller;
extern controller_t attitude_z_controller;
extern controller_t vel_x_controller;
extern controller_t vel_y_controller;
extern controller_t vel_z_controller;
extern controller_t attitude_yaw_controller;
extern controller_t height_controller; 
extern controller_t vel_controller; 

void state_check_init(state_check_t * state);
void attitude_controller_init(attitude_ctrl_t * ctrl);
void position_controller_init(controller_t * controller, int axis);
void vel_controller_init(controller_t * controller, int axis);
void mode_select_init(selectmode_t * mode_seclct);
void get_offboard_init(get_offboard_t * get_offboard);
void Controller_Init(void);

float pid_controller(float process_value, controller_t *controller, float I_limit);
void adjust_position(kalman_filter_t *filter);
void adjust_velocity(kalman_filter_t *filter);

void Lowpass_Filter(attitude_ctrl_t * ctrl, float alphax, float alphay, int n);

void Update_PID_Position(timeUs_t currentTimeUs);
void Update_PID_Velocity(timeUs_t currentTimeUs);
void Update_Lowpass_Filter(timeUs_t currentTimeUs);
void Updata_Angle_or_Anglerate(timeUs_t currentTimeUs);
void EulerAngles(attitude_ctrl_t * ctrl, get_offboard_t * offboard);

float Get_Height_PID_Output(int n);
float Get_Velocity_PID_Output(int n);
float Get_Velocity_throttle(int n);
float Get_Position_LpFiter(int n);
float Get_Velocity_LpFiter(int n);
float Get_offboard_thrust(void);
