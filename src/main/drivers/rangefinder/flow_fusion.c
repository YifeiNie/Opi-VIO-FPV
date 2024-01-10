#include "common/maths.h"
#include "sensors/gyro.h"
#include "sensors/acceleration.h"
#include "drivers/rangefinder/rangefinder.h"
#include "telemetry/mavlink.h"
#include "flight/imu.h"
#include <math.h>

#define GRAVITY_EARTH  (9.80665f)
#define LIMIT( x,min,max ) ( ((x) < (min)) ? (min) : ( ((x) > (max))? (max) : (x) ) )
#define safe_div(numerator,denominator,safe_value) ( (denominator == 0)? (safe_value) : ((numerator)/(denominator)) )

static t_fp_vector opti_flow_buff;
float imu_raw_acc[3] = {0,0,0}; //x,y,z的机体坐标系加速度 m/s^2

float oldData_fx = 0;
float oldData_fy = 0;
float oldData_height = 0;
float out_fx = 0;
float out_fy = 0;
float out_fz = 0;

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
    out += e_nr * (in - out);
    return out;
}

void flow_fusion(float dT,float fx,float fy,float flow_height) //输入为时间差，光流x原始值，光流y原始值，光流高度（单位m）
{
    float nowData_fx = fx; //输入的光流值，未经过任何处理的光流值
    float nowData_fy = fy;
    float nowData_height = flow_height;

    nowData_fx = LPF_1_(0.5, nowData_fx, oldData_fx);
    nowData_fy = LPF_1_(0.5, nowData_fy, oldData_fy);
    nowData_height = LPF_1_(0.9, nowData_height, oldData_height);

    float UPflow_speed_x = nowData_fx / dT;  //转换成rad/s
    float UPflow_speed_y = nowData_fy / dT;
    float UPflow_speed_z = (nowData_height - oldData_height) / dT;

    oldData_fx = nowData_fx;
    oldData_fy = nowData_fy;
    oldData_height = nowData_height;
    /* --------------------利用陀螺仪对光流进行补偿，保证在原地旋转时，光流输出几乎为 0 -----------------*/
    /* 1.105和1.101系数需要自己一个一个试，用来抵消低通滤波带来的幅值减小 */
    float flow_x = 1.0,flow_y = 1.0; //限幅设置 
    UPflow_speed_x = nowData_height * (UPflow_speed_x - 1.105 * LIMIT(((gyro.gyroADCf[FD_PITCH])/57.295779f),-flow_x,flow_x)); //旋转补偿
    UPflow_speed_y = nowData_height * (UPflow_speed_y + 1.101 * LIMIT(((gyro.gyroADCf[FD_ROLL])/57.295779f),-flow_y,flow_y));
    
    imu_raw_acc[0] = acc.accADC[X]/scale1/1000.0f*1.953125*GRAVITY_EARTH;  //将加速度的结果转换成m/s^2
    imu_raw_acc[1] = acc.accADC[Y]/scale1/1000.0f*1.953125*GRAVITY_EARTH;
    imu_raw_acc[2] = acc.accADC[Z]/scale1/1000.0f*1.953125*GRAVITY_EARTH;
    
    opti_flow_buff.V.X = imu_raw_acc[0];
    opti_flow_buff.V.Y = imu_raw_acc[1];
    opti_flow_buff.V.Z = imu_raw_acc[2];
    
    imuTransformVectorBodyToEarth(&opti_flow_buff); //机体系转向世界坐标系
    
    out_fx = out_fx + opti_flow_buff.V.X*dT; //计算加速度计的积分得到世界坐标系的速度
    out_fy = out_fy + opti_flow_buff.V.Y*dT;
    out_fz = out_fz + opti_flow_buff.V.Z*dT;
    
    out_fx = filter_1(0.5,UPflow_speed_x,out_fx);  //动态设置滤波系数,将光流值和加速度计得到的速度值进行融合
    out_fy = filter_1(0.5,UPflow_speed_y,out_fy);  //参数 滤波系数，光流值，加速度计得到的速度值
    out_fz = filter_1(0.5,UPflow_speed_z,out_fz); 

    mavlink_vx = out_fx;
    mavlink_vy = out_fy;
    mavlink_vz = out_fz;
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