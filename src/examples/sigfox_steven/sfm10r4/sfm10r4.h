/*
 * sfm10r4.h
 *
 *  Created on: 2020-09-21
 *      Author: PandaWang
 */

#ifndef COMPILER_ON_PX4
#include "../../../config/config.h"
#endif

#if defined(USE_SFM10R4)

#ifndef _SFM10R4_H_
#define _SFM10R4_H_

enum{
	NONE = 0,
	IT,
	DMA
};

#define SFM10R4_WRITE_SIZE	33	// AT$SF=112233445566778899AABBCC,1
#define SFM10R4_READ_SIZE	64

typedef struct{
	uint8_t read[SFM10R4_READ_SIZE];
	uint16_t read_size;
	uint8_t write[SFM10R4_WRITE_SIZE];
	uint16_t write_size;
}SFM10R4_DRIVER_DATA;

typedef struct{
	char at_version[37];
	char sdk_version[20];
	char compile_time[30];
	char bin_version[10];
	uint8_t tx_power;
	SFM10R4_DRIVER_DATA driver;
}SFM10R4_DATA;

typedef struct{
	uint8_t (*init)(SFM10R4_DATA *data);
	uint8_t (*deInit)(SFM10R4_DATA *data);
	uint8_t (*getStatus)(void);
	void (*write)(SFM10R4_DATA *data, uint8_t mode);
	void (*readFlush)(SFM10R4_DATA *data, uint8_t mode);
}SFM10R4_FUNC;

extern SFM10R4_FUNC sfm10r4_func;

typedef struct{
	SFM10R4_DATA data;
	SFM10R4_FUNC *func;
}SFM10R4;

#endif /* _SFM10R4_H_ */

#endif
