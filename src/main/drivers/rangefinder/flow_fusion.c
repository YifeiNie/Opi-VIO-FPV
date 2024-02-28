#include "common/maths.h"
#include "sensors/gyro.h"
#include "sensors/acceleration.h"
#include "drivers/rangefinder/rangefinder.h"
#include "telemetry/mavlink.h"
#include "flight/imu.h"
#include <math.h>
#include <stdlib.h>
//#include <stdio.h>

#define GRAVITY_EARTH  (9.80665f)
#define LIMIT( x,min,max ) ( ((x) < (min)) ? (min) : ( ((x) > (max))? (max) : (x) ) )
#define safe_div(numerator,denominator,safe_value) ( (denominator == 0)? (safe_value) : ((numerator)/(denominator)) )

static t_fp_vector acc_buff;
float imu_raw_acc[3] = {0,0,0}; //x,y,z的机体坐标系加速度 m/s^2

float oldData_fx = 0;
float oldData_fy = 0;
float oldData_height = 0;
float out_vx = 0;
float out_vy = 0;
float out_vz = 0;
float last_out_vx = 0;
float last_out_vy = 0;
float UPflow_speed_x_test = 0;
float UPflow_speed_y_test = 0;
float UPflow_speed_z_test = 0;

float mavlink_vx,mavlink_vy,mavlink_vz;

float LPF_1_(float a,float nowData,float oldData) 
{
    return ((a) * (nowData)) + ((1.0 - (a)) * (oldData));
}	//一阶低通滤波,a为滤波系数（0-1）
float my_pow(double x)
{
    return powf(x,2);
}
float filter_1(float k,float in,float out)   //动态调整滤波截止频率的一阶滤波
{
    static float a = 0,b = 0; //误差滤波的平方
    float e_nr; //误差的系数

	LPF_1_(k,(in - out),a); //低通后的变化量
	b = my_pow(in - out);  //求一个数平方函数
	e_nr = LIMIT(safe_div(my_pow(a),((b) + my_pow(a)),0),0,1); //变化量的有效率，LIMIT 将该数限制在0-1之间，safe_div为安全除法
    out += (1.0 - e_nr) * (in - out);
    return out;
}

float filter_1_test(float hz,float dt,float in, float out)   //动态调整滤波截止频率的一阶滤波
{
    out += (1 / (1 + 1 / (hz * 3.14 * dt))) * (in - out);
    return out;
}

void flow_fusion(float dT,float fx,float fy,float flow_height) //输入为时间差，光流x原始值，光流y原始值，光流高度（单位m）
{
    static int8_t fun_run_state = 0;
    //输入的光流值，未经过任何处理的光流值
    float nowData_fx = fx / 10000.0; 
    float nowData_fy = fy / 10000.0;
    float nowData_height = flow_height / 1000.0;

    if(fun_run_state == 0)
    {
        oldData_height = nowData_height;
        fun_run_state = 1; 
    }

    //将光流输出值转为角速度
    float UPflow_rad_x = nowData_fx / 0.02;  //转换成rad/s
    float UPflow_rad_y = nowData_fy / 0.02;

    float flow_x = 0.2,flow_y = 0.4; //限幅设置 
    //旋转补偿融合，转化为光流实际速度
    float UPflow_speed_y = nowData_height * (UPflow_rad_x - LIMIT(((gyro.gyroADCf[FD_PITCH])/57.295779f),-flow_x,flow_x)); //旋转补偿
    float UPflow_speed_x = nowData_height * (UPflow_rad_y - LIMIT(((gyro.gyroADCf[FD_ROLL])/57.295779f),-flow_y,flow_y));

    //剔除光流异常值
    // if(fabs(UPflow_speed_x - last_out_vx) > 3.0)
    // {
    //     UPflow_speed_x = last_out_vx;
    // }

    // if(fabs(UPflow_speed_y - last_out_vy) > 3.0)
    // {
    //     UPflow_speed_y = last_out_vy;
    // }

    imu_raw_acc[0] = (acc.accADC[X]/scale1/1000.0f)*1.953125*GRAVITY_EARTH;  //将加速度的结果转换成m/s^2
    imu_raw_acc[1] = (acc.accADC[Y]/scale1/1000.0f)*1.953125*GRAVITY_EARTH;
    imu_raw_acc[2] = (acc.accADC[Z]/scale1/1000.0f)*1.953125*GRAVITY_EARTH;

    acc_buff.V.X = imu_raw_acc[0];
    acc_buff.V.Y = imu_raw_acc[1];
    acc_buff.V.Z = imu_raw_acc[2];
    
    imuTransformVectorBodyToEarth(&acc_buff); //机体系转向世界坐标系
    
    
    if(fabs((double)acc_buff.V.X) <= 10)
    {
        out_vx = out_vx + acc_buff.V.X*dT; //计算加速度计的积分得到世界坐标系的速度
    }
    else
    {
        out_vx = out_vx + acc_buff.V.X*dT*0.5; 
    }

    if(fabs((double)acc_buff.V.Y) <= 10)
    {
        out_vy = out_vy + acc_buff.V.Y*dT;
    }
    else
    {
        out_vy = out_vy + acc_buff.V.Y*dT*0.5;
    }

    out_vx = filter_1(0.9,UPflow_speed_x,out_vx);  //动态设置滤波系数,将光流值和加速度计得到的速度值进行融合
    out_vy = filter_1(0.9,UPflow_speed_y,out_vy);  //参数 滤波系数，光流值，加速度计得到的速度值

    // UPflow_speed_x_test = filter_1_test(5, dT, UPflow_speed_x, out_vx);
    // UPflow_speed_y_test = filter_1_test(5, dT, UPflow_speed_y, out_vy);
    // UPflow_speed_z_test = (nowData_height - oldData_height) / dT;

    // out_vx = LPF_1_(0.85, out_vx, last_out_vx);
    // out_vy = LPF_1_(0.85, out_vy, last_out_vy);
    out_vz = LPF_1_(0.9, nowData_height, oldData_height);
    last_out_vx = out_vx;
    last_out_vy = out_vy;
    oldData_height = out_vz;

    mavlink_vx = out_vx;
    mavlink_vy = out_vy;
    mavlink_vz = out_vz;
}

float Get_Opti_Vec_X(void)
{
    return mavlink_vx;
}
float Get_Opti_Vec_Y(void)
{
    return mavlink_vy;
}
float Get_Opti_Vec_Z(void)
{
    return mavlink_vz;
}

float Get_Flow_Vec_X(void)
{
    return UPflow_speed_x_test;
}

float Get_Flow_Vec_Y(void)
{
    return UPflow_speed_y_test;
}

float Get_Flow_Vec_Z(void)
{
    return UPflow_speed_z_test;
}