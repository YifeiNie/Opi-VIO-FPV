#ifndef __FLOW_CHECK_H
#define __FLOW_CHECK_H

#include <stdint.h>

#ifndef TRUE
#define	TRUE	1
#endif
#ifndef FALSE
#define FALSE	0
#endif
#define	MAX_TABLE_ARRAY                     256
#define HEXIN_CRC16_WIDTH                   16
#define CRC16_POLYNOMIAL_1021               0x1021
#define HEXIN_REFIN_IS_TRUE(x)              ( x->refin  == TRUE )
#define HEXIN_REFOUT_IS_TRUE(x)             ( x->refout == TRUE )
#define HEXIN_GRADUAL_CALCULATE_IS_TRUE(x)	( x->is_gradual == 2 )

typedef struct _hexin_crc16_st _hexin_crc16;

#ifdef __cplusplus
extern "C" {
#endif

	uint8_t upxor(uint8_t* buf, int len); //UPIXELS协议校验
	uint8_t crc8(uint8_t* buf, int len); //MSP协议校验
	uint16_t hexin_crc16_compute(uint8_t *pSrc, int message_id, uint32_t len, uint16_t init); //MAVLINK校验
	
#ifdef __cplusplus
}
#endif

#endif
