#include <stdint.h>
#include <string.h>
#include "flow_decode.h"
#include "flow_check.h"

UpixelsOpticalFlowDistance up_data;
UpixelsOpticalFlow up_flow_data;
PX4OpticalFlow px4_flow_data;
PX4Distance px4_dis_data;
APMOpticalFlow apm_flow_data;
APMDistance apm_dis_data;
MSPDistance msp_dis_data;
MSPOpticalFlow msp_flow_data;

//319和306GS使用的UPIXELS协议
int16_t up_parse_char(uint8_t ch)
{
	int16_t ret = 0;
    static int s  = 0, p = 0;
    static char Xor_r = 0x00, Xor_c = 0x00;

    switch(s) {
        case 0:
            if(ch == 0xFE)
                s = 1;
        break;
        case 1:
            if(ch == 0x0A) {
                s = 2;
                p = 0;
                Xor_c = 0x00;
            } else {
                s = 0;
            }
        break;
        case 2:
            ((char *)&up_data)[p++] = ch;
            Xor_c ^= ch;
            if(p == 10){
                s = 3;
                p = 0;
            }
        break;
        case 3:
            s = 4;
            Xor_r = ch;
        break;
        case 4:
            if(ch == 0x55) {
                if(Xor_r == Xor_c)
                    ret = 1;
            }
            s = 0;
        break;
        default:
        break;
    }
	return ret;
}

//302GS使用的UPIXELS_NO_TOF纯光流协议
int16_t upnotof_parse_char(uint8_t ch)
{
	int16_t ret = -1;
    static int s  = 0, p = 0;
    static char Xor_r = 0x00, Xor_c = 0x00;

    switch(s) {
        case 0:
            if(ch == 0xFE)
                s = 1;
        break;
        case 1:
            if(ch == 0x0A) {
                s = 2;
                p = 0;
                Xor_c = 0x00;
            } else {
                s = 0;
            }
        break;
        case 2:
            ((char *)&up_flow_data)[p++] = ch;
            Xor_c ^= ch;
            if(p == 10) {
                s = 3;
                p = 0;
            }
        break;
        case 3:
            s = 4;
            Xor_r = ch;
        break;
        case 4:
            if(ch == 0x55) {
                if(Xor_r == Xor_c)
                    ret = 0;
            }
            s = 0;
        break;
        default:
        break;
    }
	return ret;
}

//319和306GS使用的MSP协议
int16_t msp_parse_char(uint8_t ch)
{
	int16_t ret = -1;
    static int s  = 0, p = 0;
    static uint8_t crc1[10] = {0x00, 0x01, 0x1F, 0x05, 0x00, 0, 0, 0, 0, 0};
    static uint8_t crc2[14] = {0x00, 0x02, 0x1F, 0x09, 0x00, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    
	switch(s) {
        case 0:
            if(ch == 0x24) //帧头
                s = 1;
            else
                s = 0;
        break;
        case 1:
            if(ch == 0x58) //帧头
                s = 2;
            else
                s = 0;
        break;
        case 2:
            if(ch == 0x3C) //requeset or response
                s = 3;
            else
                s = 0;
        break;
        case 3:
            if(ch == 0x00) //flag
                s = 4;
            else
                s = 0;
        break;
        case 4:
            if(ch == 0x01) //消息ID
                s = 5;
            else
                s = 0;
        break;
        case 5:
            if(ch == 0x1F) //消息ID
                s = 6;
            else
                s = 0;
        break;
        case 6:
            if(ch == 0x05) //负载长度
                s = 7;
            else
                s = 0;
        break;
        case 7:
            if(ch == 0x00) { //负载长度
                s = 8;
                p = 0;
            } else
                s = 0;
        break;
        case 8:
            ((uint8_t*)&msp_dis_data)[p++] = ch;
            if(p == 5){
                s = 9;
                p = 0;
            }
        break;
        case 9:
            memcpy(crc1+5, (uint8_t*)&msp_dis_data, 5);
            if(crc8(crc1, 10) == ch)
                s = 10;
            else
                s = 0;
        break;
        case 10:
            if(ch == 0x24) //帧头
                s = 11;
            else
                s = 0;
        break;
        case 11:
            if(ch == 0x58) //帧头
                s = 12;
            else
                s = 0;
        break;
        case 12:
            if(ch == 0x3C) //requeset or response
                s = 13;
            else
                s = 0;
        break;
        case 13:
            if(ch == 0x00) //flag
                s = 14;
            else
                s = 0;
        break;
        case 14:
            if(ch == 0x02) //消息ID
                s = 15;
            else
                s = 0;
        break;
        case 15:
            if(ch == 0x1F) //消息ID
                s = 16;
            else
                s = 0;
        break;
        case 16:
            if(ch == 0x09) //负载长度
                s = 17;
            else
                s = 0;
        break;
        case 17:
            if(ch == 0x00) { //负载长度
                s = 18;
                p = 0;
            } else
                s = 0;
        break;
        case 18:
            ((uint8_t*)&msp_flow_data)[p++] = ch;
            if(p == 9) {
                s = 19;
                p = 0;
            }
        break;
        case 19:
            memcpy(crc2+5, (uint8_t*)&msp_flow_data, 9);
            if(crc8(crc2, 14) == ch) {
                ret = 0;
            }
            s = 0;
        break;
        default:
        break;
    }
	return ret;
}

//302GS使用的MSP_NO_TOF纯光流协议
int16_t mspnotof_parse_char(uint8_t ch)
{
	int16_t ret = 1;
    static int s  = 0, p = 0;
	static uint8_t crc1[10] = { 0x00, 0x01, 0x1F, 0x05, 0x00, 0, 0, 0, 0, 0 };
	static uint8_t crc2[14] = { 0x00, 0x02, 0x1F, 0x09, 0x00, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    
    switch(s) {
        case 0:
            if(ch == 0x24) //帧头
                s = 1;
            else
                s = 0;
        break;
        case 1:
            if(ch == 0x58) //帧头
                s = 2;
            else
                s = 0;
        break;
        case 2:
            if(ch == 0x3C) //requeset or response
                s = 3;
            else
                s = 0;
        break;
        case 3:
            if(ch == 0x00) //flag
                s = 4;
            else
                s = 0;
        break;
        case 4:
            if(ch == 0x02) //消息ID
                s = 5;
            else
                s = 0;
        break;
        case 5:
            if(ch == 0x1F) //消息ID
                s = 6;
            else
                s = 0;
        break;
        case 6:
            if(ch == 0x09) //负载长度
                s = 7;
            else
                s = 0;
        break;
        case 7:
            if(ch == 0x00) { //负载长度
                s = 8;
                p = 0;
            } else
                s = 0;
        break;
        case 8:
            ((uint8_t*)&msp_flow_data)[p++] = ch;
            if(p == 9){
                s = 9;
                p = 0;
            }
        break;
        case 9:
            memcpy(crc2+5, (uint8_t*)&msp_flow_data, 9);
            if(crc8(crc2, 14) == ch) {
                ret = 0;
            }
            s = 0;
        break;
        default:
        break;
    }
    return ret;
}

//319和306GS使用的MAVLINK_APM协议
int16_t apm_parse_char(uint8_t ch)
{
	int16_t ret = -1;
    static int s  = 0, p = 0;
	static uint16_t px4_crc1, px4_crc2; 
	static uint8_t crc1[32] = { 0x1A, 0, 0x00, 0x9E, 0x64, 0, 0, 0, 0, 0,
                           0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                           0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                           0, 0};
	static uint8_t crc2[20] = { 0x0E, 0, 0x00, 0x9E, 0x84, 0, 0, 0, 0, 0,
                           0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    switch(s) {
        case 0:
            if(ch == 0xFE) //帧头
                s = 1;
            else
                s = 0;
        break;
        case 1:
            if(ch == 0x1A) //负载长度
                s = 2;
            else
                s = 0;
        break;
        case 2:
            if(ch >= 0x00 && ch <=0xFF) { //包序列号
                s = 3;
                crc1[1] = ch;
            } else
                s = 0;
        break;
        case 3:
            if(ch == 0x00) //系统ID
                s = 4;
            else
                s = 0;
        break;
        case 4:
            if(ch == 0x9E) //组件ID
                s = 5;
            else
                s = 0;
        break;
        case 5:
            if(ch == 0x64) { //消息ID
                s = 6;
                p = 0;
            } else
                s = 0;
        break;
        case 6:
            ((uint8_t*)&apm_flow_data)[p++] = ch;
            if(p == 26){
                s = 7;
                p = 0;
            }
        break;
        case 7: //crc
        {
            memcpy(crc1+5, (uint8_t*)&apm_flow_data, 26);
            px4_crc1 = hexin_crc16_compute(crc1, 0x64, 32, 0xFFFF);
            if(ch == (px4_crc1&0x00FF))
                s = 8;
            else
                s = 0;
        }
        break;
		case 8: //crc
        {
            if(ch == (px4_crc1&0xFF00)>>8)
                s = 9;
            else
                s = 0;
        }
        break;
        case 9:
            if(ch == 0xFE) //帧头
                s = 10;
            else
                s = 0;
        break;
        case 10:
            if(ch == 0x0E) //负载长度
                s = 11;
            else
                s = 0;
        break;
        case 11:
            if(ch >= 0x00 && ch <=0xFF) { //包序列号
                s = 12;
                crc2[1] = ch;
            } else
                s = 0;
        break;
        case 12:
            if(ch == 0x00) //系统ID
                s = 13;
            else
                s = 0;
        break;
        case 13:
            if(ch == 0x9E) //组件ID
                s = 14;
            else
                s = 0;
        break;
        case 14:
            if(ch == 0x84) { //消息ID
                s = 15;
                p = 0;
            } else
                s = 0;
        break;
        case 15:
            ((uint8_t*)&apm_dis_data)[p++] = ch;
            if(p == 14){
                s = 16;
                p = 0;
            }
        break;
        case 16: //crc
        {
            memcpy(crc2+5, (uint8_t*)&apm_dis_data, 14);
            px4_crc2 = hexin_crc16_compute(crc2, 0x84, 20, 0xFFFF);
            if(ch == (px4_crc2&0x00FF))
				s = 17;
            else
				s = 0;
        }
        break;
		case 17: //crc
        {
            if(ch == (px4_crc2&0xFF00)>>8)
				ret = 0;
            s = 0;
        }
        break;
        default:
        break;
    }
	return ret;
}

//319和306GS使用的MAVLINK_PX4协议
int16_t px4_parse_char(uint8_t ch)
{
	int16_t ret = -1;
    static int s  = 0, p = 0;
	static uint16_t px4_crc1, px4_crc2;
	static uint8_t crc1[50] = { 0x2C, 0, 0x00, 0x9E, 0x6A, 0, 0, 0, 0, 0,
                           0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                           0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                           0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                           0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	static uint8_t crc2[20] = { 0x0E, 0, 0x00, 0x9E, 0x84, 0, 0, 0, 0, 0,
                           0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    switch(s) {
        case 0:
            if(ch == 0xFE) //帧头
                s = 1;
            else
                s = 0;
        break;
        case 1:
            if(ch == 0x2C) //负载长度
                s = 2;
            else
                s = 0;
        break;
        case 2:
            if(ch >= 0x00 && ch <=0xFF) { //包序列号
                s = 3;
                crc1[1] = ch;
            } else
                s = 0;
        break;
        case 3:
            if(ch == 0x00) //系统ID
                s = 4;
            else
                s = 0;
        break;
        case 4:
            if(ch == 0x9E) //组件ID
                s = 5;
            else
                s = 0;
        break;
        case 5:
            if(ch == 0x6A) { //消息ID
                s = 6;
                p = 0;
            } else
                s = 0;
        break;
        case 6:
            ((uint8_t*)&px4_flow_data)[p++] = ch;
            if(p == 44){
                s = 7;
                p = 0;
            }
        break;
        case 7: //crc
        {
            memcpy(crc1+5, (uint8_t*)&px4_flow_data, 44);
            px4_crc1 = hexin_crc16_compute(crc1, 0x6A, 50, 0xFFFF);
            if(ch == (px4_crc1&0x00FF))
                s = 8;
            else
                s = 0;
        }
        break;
		case 8: //crc
        {
            if(ch == (px4_crc1&0xFF00)>>8 )
                s = 9;
            else
                s = 0;
        }
        break;
        case 9:
            if(ch == 0xFE) //帧头
                s = 10;
            else
                s = 0;
        break;
        case 10:
            if(ch == 0x0E) //负载长度
                s = 11;
            else
                s = 0;
        break;
        case 11:
            if(ch >= 0x00 && ch <=0xFF) { //包序列号
                s = 12;
                crc2[1] = ch;
            } else
                s = 0;
        break;
        case 12:
            if(ch == 0x00) //系统ID
                s = 13;
            else
                s = 0;
        break;
        case 13:
            if(ch == 0x9E) //组件ID
                s = 14;
            else
                s = 0;
        break;
        case 14:
            if(ch == 0x84) { //消息ID
                s = 15;
                p = 0;
            } else
                s = 0;
        break;
        case 15:
            ((uint8_t*)&px4_dis_data)[p++] = ch;
            if(p == 14){
                s = 16;
                p = 0;
            }
        break;
        case 16: //crc
        {
            memcpy(crc2+5, (uint8_t*)&px4_dis_data, 14);
            px4_crc2 = hexin_crc16_compute(crc2, 0x84, 20, 0xFFFF);
			if (ch == (px4_crc2 & 0x00FF))
				s = 17;
            else
				s = 0;
        }
        break;
		case 17: //crc
        {
            if(ch == (px4_crc2&0xFF00)>>8)
				ret = 0;
            s = 0;
        }
        break;
        default:
        break;
    }
    return ret;
}

//302GS使用的MAVLINK_PX4纯光流协议
int16_t px4notof_parse_char(uint8_t ch)
{
	int16_t ret = -1;
	static int s  = 0, p = 0;
	static uint16_t px4_crc1;
	static uint8_t crc1[50] = { 0x2C, 0, 0x00, 0x9E, 0x6A, 0, 0, 0, 0, 0,
                           0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                           0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                           0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                           0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    switch(s) {
        case 0:
            if(ch == 0xFE) //帧头
                s = 1;
            else
                s = 0;
        break;
        case 1:
            if(ch == 0x2C) //负载长度
                s = 2;
            else
                s = 0;
        break;
        case 2:
            if(ch >= 0x00 && ch <=0xFF) { //包序列号
                s = 3;
                crc1[1] = ch;
            } else
                s = 0;
        break;
        case 3:
            if(ch == 0x00) //系统ID
                s = 4;
            else
                s = 0;
        break;
        case 4:
            if(ch == 0x9E) //组件ID
                s = 5;
            else
                s = 0;
        break;
        case 5:
            if(ch == 0x6A) { //消息ID
                s = 6;
                p = 0;
            } else
                s = 0;
        break;
        case 6:
            ((uint8_t*)&px4_flow_data)[p++] = ch;
            if(p == 44){
                s = 7;
                p = 0;
            }
        break;
        case 7: //crc
        {
            memcpy(crc1+5, (uint8_t*)&px4_flow_data, 44);
            px4_crc1 = hexin_crc16_compute(crc1, 0x6A, 50, 0xFFFF);
            if(ch == (px4_crc1&0x00FF))
				s = 8;
			else 
				s = 0;
        }
        break;
		case 8: //crc
        {
            if(ch == (px4_crc1&0xFF00)>>8) {
				ret = 0;
            }
            s = 0;
        }		
		break;
        default:
        break;
    }
	return ret;
}

