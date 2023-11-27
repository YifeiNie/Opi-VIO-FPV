#include <stdint.h>

#pragma pack(1)

//UPIXELS协议
typedef struct optical_flow_distance_data
{
    int16_t		flow_x_integral; //2
    int16_t		flow_y_integral; //2
    uint16_t   	integration_timespan; //2
    uint16_t   	ground_distance; //2
    uint8_t    	valid; //1
    uint8_t    	tof_confidence; //1
}UpixelsOpticalFlowDistance;

//UPIXELS纯光流协议
typedef struct optical_flow_data
{
    int16_t		flow_x_integral; //2
    int16_t		flow_y_integral; //2
    uint16_t   	integration_timespan; //2
    uint16_t   	ground_distance; //2
    uint8_t    	valid; //1
    uint8_t    	version; //1
}UpixelsOpticalFlow;

//MAVLINK_PX4协议的光流部分
typedef struct optiocal_flow_data_px4
{
    uint64_t	time_us;   //8
    uint32_t   	integration_time; //4
    float   	integrated_x; //4
    float       integrated_y; //4
    float       integrated_xgyro; //4
    float       integrated_ygyro; //4
    float       integrated_zgyro; //4
    uint32_t    time_delta_distance; //4
    float       distance; //4
    uint16_t    temperature; //2
    uint8_t		sensor_id; //1
    uint8_t     quality; //1
}PX4OpticalFlow;

//MAVLINK_PX4协议的距离部分
typedef struct distance_data_px4
{
    uint32_t    time_us; //4
    uint16_t    min_distance; //2
    uint16_t    max_distance; //2
    uint16_t    current_distance; //2
    uint8_t     type; //1
    uint8_t     id; //1
    uint8_t     orientation; //1
    uint8_t     covariance; //1
}PX4Distance;

//MAVLINK_APM协议的光流部分
typedef struct optical_flow_data_apm
{
    uint64_t    time_us; //8
    float       flow_comp_x; //4
    float       flow_comp_y; //4
    float       ground_distance; //4
    int16_t     flow_x; //2
    int16_t     flow_y; //2
    uint8_t     sensor_id; //1
    uint8_t     quality; //1
}APMOpticalFlow;

//MAVLINK_APM协议的距离部分
typedef struct distance_data_apm
{
    uint32_t    time_ms; //4
    uint16_t    min_distance; //2
    uint16_t    max_distance; //2
    uint16_t    current_distance; //2
    uint8_t     type; //1
    uint8_t     id; //1
    uint8_t     orientation; //1
    uint8_t     covariance; //1
}APMDistance;

//MSP协议的距离部分
typedef struct distance_msp
{
    uint8_t     quality; //1
    uint32_t    distance; //4
}MSPDistance;

//MSP协议的光流部分
typedef struct optical_flow_data_msp
{
    uint8_t     flow_quality; //1
    int32_t     motionX; //4
    int32_t     motionY; //4
}MSPOpticalFlow;
#pragma pack()

typedef enum {
    UPIXELS=0,
    UPIXELS_NO_TOF,
	MSP,
	MSP_NO_TOF,
	MAVLINK_APM,
	MAVLINK_PX4,
	MAVLINK_PX4_NO_TOF
}PROTOCOL;

extern UpixelsOpticalFlowDistance up_data;
extern UpixelsOpticalFlow up_flow_data;
extern PX4OpticalFlow px4_flow_data;
extern PX4Distance px4_dis_data;
extern APMOpticalFlow apm_flow_data;
extern APMDistance apm_dis_data;
extern MSPDistance msp_dis_data;
extern MSPOpticalFlow msp_flow_data;

#ifdef __cplusplus
extern "C" {
#endif

int16_t up_parse_char(uint8_t ch); //319和306GS使用的UPIXELS协议
int16_t upnotof_parse_char(uint8_t ch); //302GS使用的UPIXELS_NO_TOF纯光流协议
int16_t msp_parse_char(uint8_t ch); //319和306GS使用的MSP协议
int16_t mspnotof_parse_char(uint8_t ch); //302GS使用的MSP_NO_TOF纯光流协议
int16_t apm_parse_char(uint8_t ch); //319和306GS使用的MAVLINK_APM协议
int16_t px4_parse_char(uint8_t ch); //319和306GS使用的MAVLINK_PX4协议
int16_t px4notof_parse_char(uint8_t ch); //302GS使用的MAVLINK_PX4_NO_TOF纯光流协议

#ifdef __cplusplus
}
#endif