#include "common/maths.h"
#include "sensors/gyro.h"
#include "sensors/acceleration.h"
#include "rangefinder/rangefinder.h"
#include "telemetry/mavlink.h"

#define GRAVITY_EARTH  (9.80665f)
#define LIMIT( x,min,max ) ( ((x) < (min)) ? (min) : ( ((x) > (max))? (max) : (x) ) )
#define LPF_1_(a,nowData,oldData) (a * nowData + (1.0f - a) * oldData)	//一阶低通滤波,a为滤波系数（0-1）
#define safe_div(numerator,denominator,safe_value) ( (denominator == 0)? (safe_value) : ((numerator)/(denominator)) )

static t_fp_vector_def opti_flow_buff;
float imu_raw_acc[3] = {0,0,0}; //x,y,z的机体坐标系加速度 m/s^2

float oldData_fx = 0;
float oldData_fy = 0;
float oldData_height = 0;
float out_fx = 0;
float out_fy = 0;
float out_fz = 0;

void flow_fusion(float dT,float fx,float fy,float flow_height)
{
    float nowData_fx = fx; //输入的光流值，未经过任何处理的光流值
    float nowData_fy = fx;
    float nowData_height = flow_height;

    nowData_fx = LPF_1_(0.5, nowData_fx, oldData_fx);
    nowData_fy = LPF_1_(0.5, nowData_fy, oldData_fy);
    nowData_height = LPF_1_(0.9, nowData_height, oldData_height);

    float UPflow_speed_x = nowData_fx / dT;
    float UPflow_speed_y = nowData_fy / dT;
    float UPflow_speed_z = (nowData_height - oldData_height) / dT;

    /* --------------------利用陀螺仪对光流进行补偿，保证在原地旋转时，光流输出几乎为 0 -----------------*/
    /* 1.105和1.101系数需要自己一个一个试，用来抵消低通滤波带来的幅值减小 */
    float flow_x = 1.0,flow_y = 1.0; //限幅设置 
    UPflow_speed_x = nowData_height * (UPflow_speed_x - 1.105 * LIMIT(((gyro.gyroADCf[FD_ROLL])/57.295779f),-flow_x,flow_x));
    UPflow_speed_y = nowData_height * (UPflow_speed_y + 1.101 * LIMIT(((gyro.gyroADCf[FD_PITCH])/57.295779f),-flow_x,flow_x));

    imu_raw_acc[0] = acc.accADC[X]/scale1*1.953125*GRAVITY_EARTH;
    imu_raw_acc[1] = acc.accADC[Y]/scale1*1.953125*GRAVITY_EARTH;
    imu_raw_acc[2] = acc.accADC[Z]/scale1*1.953125*GRAVITY_EARTH;

    opti_flow_buff.X = imu_raw_acc[0];
    opti_flow_buff.Y = imu_raw_acc[1];
    opti_flow_buff.Z = imu_raw_acc[2];

    imuTransformVectorBodyToEarth(opti_flow_buff);
    
    out_fx = out_fx + opti_flow_buff.X*dT; //计算加速度计的积分得到世界坐标系的速度
    out_fy = out_fy + opti_flow_buff.Y*dT;
    out_fz = out_fz + opti_flow_buff.Z*dT;
    
}



acc.accADC[X]
imu_raw_acc[0] = acc.accADC[X]/scale1*1.953125*GRAVITY_EARTH; //得到机体加速度
gyro.gyroADCf[FD_ROLL]