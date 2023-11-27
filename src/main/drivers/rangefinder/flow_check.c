#include <stdint.h>
#include "flow_check.h"

//mavlink
static const uint8_t crc_extra_array[256]={
	50, 124, 137, 0, 237, 217, 104, 119, 0, 0, 
	0, 89, 0, 0, 0, 0, 0, 0, 0, 0,
	214, 159, 220, 168, 24, 23, 170, 144, 67, 115,
	39, 246, 185, 104, 237, 244, 222, 212, 9, 254,
	230, 28, 28, 132, 221, 232, 11, 153, 41, 39,
	78, 0, 0, 0, 15, 3, 0, 0, 0, 0,
	0, 153, 183, 51, 59, 118, 148, 21, 0, 243,
	124, 0, 0, 38, 20, 158, 152, 143, 0, 0,
	0, 106, 49, 22, 143, 140, 5, 150, 0, 231,
	183, 63, 54, 0, 0, 0, 0, 0, 0, 0,
	175, 102, 158, 208, 56, 93, 138, 108, 32, 185,
	84, 34, 174, 124, 237, 4, 76, 128, 56, 116,
	134, 237, 203, 250, 87, 203, 220, 25, 226, 46,
	29, 223, 85, 6, 229, 203, 1, 195, 109, 168,
	181, 148, 72, 0, 0, 0, 103, 154, 178, 200,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 90, 104, 85, 95, 83, 0, 0, 8, 204, 
	49, 170, 44, 83, 46, 0
};

struct _hexin_crc16_st {
	unsigned int    is_initial;
	unsigned int    is_gradual;
	unsigned short  width;
	unsigned short  poly;
	unsigned short  init;
	unsigned int    refin;
	unsigned int    refout;
	unsigned short  xorout;
	unsigned short  result;
	unsigned short  table[MAX_TABLE_ARRAY];
};

static uint8_t __hexin_reverse8(uint8_t data) {
	uint32_t  i = 0;
	uint8_t t = 0;
	for (i = 0; i < 8; i++) {
		t |= ((data >> i) & 0x01) << (7 - i);
	}
	return t;
}

static uint16_t hexin_reverse16(uint16_t data) {
	uint32_t i = 0;
	uint16_t t = 0;
	for (i = 0; i<16; i++) {
		t |= ((data >> i) & 0x0001) << (15 - i);
	}
	return t;
}

static uint32_t hexin_crc16_compute_init_table(_hexin_crc16 *param) {
	uint32_t i = 0, j = 0;
	uint16_t crc = 0, c = 0;

	for (i = 0; i<MAX_TABLE_ARRAY; i++) {
		crc = 0;
		c = ((uint16_t)i) << 8;
		for (j = 0; j<8; j++) {
			if ((crc ^ c) & 0x8000) 
				crc = (crc << 1) ^ param->poly;
			else                      
				crc = crc << 1;
			c = c << 1;
		}
		param->table[i] = crc;
	}

	return TRUE;
}

static uint16_t hexin_crc16_compute_char(uint16_t crc16, uint8_t c, _hexin_crc16* param) {
	uint16_t crc = crc16;
	crc = (crc << 8) ^ param->table[((crc >> 8) ^ (0x00FF & (uint16_t)c)) & 0xFF];
	return crc;
}

uint16_t hexin_crc16_compute(uint8_t *pSrc, int message_id, uint32_t len, uint16_t init) {
	uint32_t i = 0;
	uint16_t crc = init;              /* Fixed Issues #4  */
	_hexin_crc16 param_st = {
		.is_initial = FALSE,
		.width = HEXIN_CRC16_WIDTH,
		.poly = CRC16_POLYNOMIAL_1021,
		.init = 0xFFFF,
		.refin = TRUE,
		.refout = TRUE,
		.xorout = 0x0000,
		.result = 0
	};
	_hexin_crc16* param = &param_st;
	pSrc[len-1] = crc_extra_array[message_id];
	
	if (param->is_initial == FALSE) {
		param->is_initial = hexin_crc16_compute_init_table(param);
	}

	/* Fixed Issues #4 #8 */
	if (HEXIN_REFOUT_IS_TRUE(param) && (HEXIN_GRADUAL_CALCULATE_IS_TRUE(param))) {
		crc = hexin_reverse16(init);
	}

	/* Fixed Issues #8 */
	if (HEXIN_REFIN_IS_TRUE(param)) {
		for (i = 0; i<len; i++) {
			crc = hexin_crc16_compute_char(crc, __hexin_reverse8(pSrc[i]), param);
		}
	}
	else {
		for (i = 0; i<len; i++) {
			crc = hexin_crc16_compute_char(crc, pSrc[i], param);
		}
	}

	/* Fixed Issues #8 */
	if (HEXIN_REFOUT_IS_TRUE(param)) {
		crc = hexin_reverse16(crc);
	}

	return (crc ^ param->xorout);
}



//msp
static uint8_t crc8_dvb_s2(uint8_t crc, unsigned char a) {
    int ii = 0;
    crc ^= a;
    for (ii=0; ii < 8; ++ii) {
		if (crc & 0x80) {
			crc = (crc<<1) ^ 0xD5;
		} else {
			crc = crc<<1;
		}
	}
	return crc;
}

uint8_t crc8(uint8_t* buf, int len) { 
    int i = 0;
    uint8_t ck = 0; //initialise CRC
    for(i=0; i<len; i++) {
		ck = crc8_dvb_s2(ck, buf[i]); //loop over summable data
	}
	return ck;
}



//upixels
uint8_t upxor(uint8_t* buf, int len) {
    int i = 0;
    uint8_t ck = 0;
    for(i=0; i<len; i++) {
		ck ^= buf[i];
	}
	return ck;
}
