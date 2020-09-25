/*
 * sigfox.h
 *
 *  Created on: 2020-09-23
 *      Author: PandaWang
 */

#define COMPILER_ON_PX4
#define USE_SIGFOX
#define USE_SFM10R4

#ifndef COMPILER_ON_PX4
#include "../../config/config.h"
#endif


#if defined(USE_SIGFOX)

#ifndef _SIGFOX_H_
#define _SIGFOX_H_

enum{
	SIGFOX_ERROR = 0,
	SIGFOX_OK,
	SIGFOX_BUSY
};

typedef struct{
	char ip[16];
	char gateway[16];
	char netmask[16];
	char mac[18];
}SIGFOX_STATION;

typedef struct{
	SIGFOX_STATION station;
}SIGFOX_PARAMETER;

typedef struct{
	uint8_t sigfox_mode;
	uint8_t state;
	SIGFOX_PARAMETER parameter;
}SIGFOX_DATA;

typedef struct{
	uint8_t (*init)(SIGFOX_DATA *data);
	uint8_t (*deInit)(SIGFOX_DATA *data);
	void (*writeTestStartup)(SIGFOX_DATA *data);
	uint8_t (*readTestStartup)(SIGFOX_DATA *data);
	void (*writeRestartsTheModule)(SIGFOX_DATA *data);
	uint8_t (*readRestartsTheModule)(SIGFOX_DATA *data);
	void (*writeSleepTheModule)(SIGFOX_DATA *data);
	uint8_t (*readSleepTheModule)(SIGFOX_DATA *data);
	void (*writeDeepSleepTheModule)(SIGFOX_DATA *data);
	uint8_t (*readDeepSleepTheModule)(SIGFOX_DATA *data);
	void (*writeIsNeedResetChannel)(SIGFOX_DATA *data);
	uint8_t (*readIsNeedResetChannel)(SIGFOX_DATA *data, uint8_t *isNeedRstChannel);
	void (*writeResetChannel)(SIGFOX_DATA *data);
	uint8_t (*readResetChannel)(SIGFOX_DATA *data);
	void (*writeSendMessage)(int32_t lat_in, int32_t lon_in, int32_t alt_in, uint8_t msgFormat);
	uint8_t (*readSendMessage)(SIGFOX_DATA *data);

#if 0
	void (*writeATCommandsEchoing)(SIGFOX_DATA *data);
	uint8_t (*readATCommandsEchoing)(SIGFOX_DATA *data);
	void (*writeMaximunValueOfRFTXPower)(SIGFOX_DATA *data);
	uint8_t (*readMaximunValueOfRFTXPower)(SIGFOX_DATA *data);
	void (*writeSIGFOXModeDef)(SIGFOX_DATA *data);
	uint8_t (*readSIGFOXModeDef)(SIGFOX_DATA *data);
	void (*writeAutoConnectsToTheAP)(SIGFOX_DATA *data);
	uint8_t (*readAutoConnectsToTheAP)(SIGFOX_DATA *data);
	void (*writeMACAddressofTheStationDef)(SIGFOX_DATA *data);
	uint8_t (*readMACAddressofTheStationDef)(SIGFOX_DATA *data);
	void (*writeDefaultIPofTheStationDef)(SIGFOX_DATA *data);
	uint8_t (*readDefaultIPofTheStationDef)(SIGFOX_DATA *data);
	void (*writeEstablishConnectionToAP)(SIGFOX_DATA *data);
	uint8_t (*readEstablishConnectionToAP)(SIGFOX_DATA *data);
	void (*writeEstablishTCPConnection)(SIGFOX_DATA *data);
	uint8_t (*readEstablishTCPConnection)(SIGFOX_DATA *data);
	void (*writeTransmissionMode)(SIGFOX_DATA *data);
	uint8_t (*readTransmissionMode)(SIGFOX_DATA *data);
	void (*writeSendDataMode)(SIGFOX_DATA *data);
	uint8_t (*readSendDataMode)(SIGFOX_DATA *data);
	void (*writeData)(void *data, uint16_t size);
	uint8_t (*readData)(void *data, uint16_t *size);
	void* (*readRawData)(void);
	void (*flush)(SIGFOX_DATA *data);
#endif
}SIGFOX_FUNC;

extern SIGFOX_FUNC sigfox_func;

typedef struct{
	SIGFOX_DATA data;
	SIGFOX_FUNC *func;
}SIGFOX;

#endif /* _SIGFOX_H_ */

#endif /* USE_SIGFOX */
