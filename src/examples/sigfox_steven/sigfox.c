/*
 * sigfox.c
 *
 *  Created on: 2020-09-23
 *      Author: PandaWang
 */

#include "sigfox.h"

#ifdef USE_SFM10R4

#include "./sfm10r4/sfm10r4.h"
#include "./at_command_sigfox/at_command_sigfox.h"
#include "debug.h"
#include "string.h"
#ifdef COMPILER_ON_PX4
#include <stdio.h>
#endif

SFM10R4 sfm10r4 = {
		.func = &sfm10r4_func
};

AT_COMMAND_SIGFOX at_command_sigfox = {
		.func = &at_command_sigfox_func
};

#endif

#if defined(_SIGFOX_H_)


#define DEFAULT_STATION_MAC "18:fe:35:98:d3:7b"
#define DEFAULT_STATION_IP "192.168.4.2"
#define DEFAULT_STATION_GATEWAY "192.168.4.1"
#define DEFAULT_STATION_NETMASK "255.255.255.0"

#define DEFAULT_AP_IP "192.168.4.1"
#define DEFAULT_AP_GATEWAY "192.168.4.1"
#define DEFAULT_AP_NETMASK "255.255.255.0"
#define DEFAULT_AP_PORT 1000
#define DEFAULT_AP_LOCAL_PORT 1001

#define DEFAULT_AP_SSID "Steven good good"
#define DEFAULT_AP_PWD "830430810113"

#ifdef COMPILER_ON_PX4
__EXPORT int sigfox_steven_main(int argc, char *argv[]);
int sigfox_steven_main(int argc, char *argv[]) 
{ 
    printf("sigfox_main Hello Sky!\n"); 
    return OK; 
} 
#endif

uint8_t sigfoxInitial(SIGFOX_DATA *data){
	uint8_t flag = 0;
#if defined(_SFM10R4_H_) && defined(_AT_COMMAND_SIGFOX_H_)
	flag = sfm10r4.func->init(&sfm10r4.data);
	if(flag){
		at_command_sigfox.data.read = (char *)sfm10r4.data.driver.read;
		at_command_sigfox.data.read_size = &sfm10r4.data.driver.read_size;
		at_command_sigfox.data.write = (char *)sfm10r4.data.driver.write;
		at_command_sigfox.data.write_size = &sfm10r4.data.driver.write_size;
		sfm10r4.func->readFlush(&sfm10r4.data,DMA);
	}
#endif
	return flag;
}

uint8_t sigfoxDeInitial(SIGFOX_DATA *data){
	uint8_t flag = 0;
#if defined(_SFM10R4_H_)
	flag = sfm10r4.func->deInit(&sfm10r4.data);
#endif
	return flag;
}

void sigfoxWriteTestStartup(SIGFOX_DATA *data){
	at_command_sigfox.func->writeTestStartup(&at_command_sigfox.data);
	sfm10r4.func->write(&sfm10r4.data,IT);
}

uint8_t sigfoxReadTestStartup(SIGFOX_DATA *data){
	uint8_t flag = 0;
#if defined(_AT_COMMAND_SIGFOX_H_)
	flag = at_command_sigfox.func->readTestStartup(&at_command_sigfox.data);
#endif
	return flag;
}

void sigfoxWriteRestartsTheModule(SIGFOX_DATA *data){
	at_command_sigfox.func->writeRestartsTheModule(&at_command_sigfox.data);
	sfm10r4.func->write(&sfm10r4.data,IT);
}

uint8_t sigfoxReadRestartsTheModule(SIGFOX_DATA *data){
	uint8_t flag = 0;
#if defined(_AT_COMMAND_SIGFOX_H_)
	flag = at_command_sigfox.func->readRestartsTheModule(&at_command_sigfox.data);
#endif
	return flag;
}

void sigfoxWriteSleepTheModule(SIGFOX_DATA *data){
	at_command_sigfox.func->writeSleepTheModule(&at_command_sigfox.data);
	sfm10r4.func->write(&sfm10r4.data,IT);
}

uint8_t sigfoxReadSleepTheModule(SIGFOX_DATA *data){
	uint8_t flag = 0;
#if defined(_AT_COMMAND_SIGFOX_H_)
	flag = at_command_sigfox.func->readSleepTheModule(&at_command_sigfox.data);
#endif
	return flag;
}

void sigfoxWriteDeepSleepTheModule(SIGFOX_DATA *data){
	at_command_sigfox.func->writeDeepSleepTheModule(&at_command_sigfox.data);
	sfm10r4.func->write(&sfm10r4.data,IT);
}

uint8_t sigfoxReadDeepSleepTheModule(SIGFOX_DATA *data){
	uint8_t flag = 0;
#if defined(_AT_COMMAND_SIGFOX_H_)
	flag = at_command_sigfox.func->readDeepSleepTheModule(&at_command_sigfox.data);
#endif
	return flag;
}

void sigfoxWriteIsNeedResetChannel(SIGFOX_DATA *data){
	at_command_sigfox.func->writeIsNeedResetChannel(&at_command_sigfox.data);
	sfm10r4.func->write(&sfm10r4.data,IT);
}

uint8_t sigfoxReadIsNeedResetChannel(SIGFOX_DATA *data, uint8_t *isNeedRstChannel){
	uint8_t flag = 0;
#if defined(_AT_COMMAND_SIGFOX_H_)
	flag = at_command_sigfox.func->readIsNeedResetChannel(&at_command_sigfox.data, isNeedRstChannel);
#endif
	return flag;
}

void sigfoxWriteResetChannel(SIGFOX_DATA *data){
	at_command_sigfox.func->writeResetChannel(&at_command_sigfox.data);
	sfm10r4.func->write(&sfm10r4.data,IT);
}

uint8_t sigfoxReadResetChannel(SIGFOX_DATA *data){
	uint8_t flag = 0;
#if defined(_AT_COMMAND_SIGFOX_H_)
	flag = at_command_sigfox.func->readResetChannel(&at_command_sigfox.data);
#endif
	return flag;
}

void sigfoxWriteSendMessage(int32_t lat_in, int32_t lon_in, int32_t alt_in, uint8_t msgFormat){
	at_command_sigfox.func->writeSendMessage(&at_command_sigfox.data, lat_in, lon_in, alt_in, msgFormat, false);
	sfm10r4.func->write(&sfm10r4.data,IT);
}

uint8_t sigfoxReadSendMessage(SIGFOX_DATA *data){
	uint8_t flag = 0;
#if defined(_AT_COMMAND_SIGFOX_H_)
	flag = at_command_sigfox.func->readSendMessage(&at_command_sigfox.data);
#endif
	return flag;
}


#if 0
void sigfoxWriteATCommandsEchoing(SIGFOX_DATA *data){
	at_command_sigfox.func->writeATCommandsEchoing(&at_command_sigfox.data, 1);
	sfm10r4.func->write(&sfm10r4.data,IT);
}

uint8_t sigfoxReadATCommandsEchoing(SIGFOX_DATA *data){
	uint8_t flag = 0;
#if defined(_AT_COMMAND_SIGFOX_H_)
	uint8_t mode;
	flag = at_command_sigfox.func->readATCommandsEchoing(&at_command_sigfox.data, &mode);
#endif
	return flag;
}


void sigfoxWriteMaximunValueOfRFTXPower(SIGFOX_DATA *data){
	at_command_sigfox.func->writeMaximunValueOfRFTXPower(&at_command_sigfox.data, 82);
	sfm10r4.func->write(&sfm10r4.data,IT);
}

uint8_t sigfoxReadMaximunValueOfRFTXPower(SIGFOX_DATA *data){
	uint8_t flag = 0;
#if defined(_AT_COMMAND_SIGFOX_H_)
	flag = at_command_sigfox.func->readMaximunValueOfRFTXPower(&at_command_sigfox.data, &sfm10r4.data.tx_power);
#endif
	return flag;
}


void sigfoxWriteSIGFOXModeDef(SIGFOX_DATA *data){
	at_command_sigfox.func->writeSIGFOXModeDef(&at_command_sigfox.data, STATION_MODE);
	sfm10r4.func->write(&sfm10r4.data,IT);
}

uint8_t sigfoxReadSIGFOXModeDef(SIGFOX_DATA *data){
	uint8_t flag = 0;
#if defined(_AT_COMMAND_SIGFOX_H_)
	flag = at_command_sigfox.func->readSIGFOXModeDef(&at_command_sigfox.data, &data->sigfox_mode);
#endif
	return flag;
}


void sigfoxWriteAutoConnectsToTheAP(SIGFOX_DATA *data){
	at_command_sigfox.func->writeAutoConnectsToTheAP(&at_command_sigfox.data, 0);
	sfm10r4.func->write(&sfm10r4.data,IT);
}

uint8_t sigfoxReadAutoConnectsToTheAP(SIGFOX_DATA *data){
	uint8_t flag = 0;
#if defined(_AT_COMMAND_SIGFOX_H_)
	uint8_t en;
	flag = at_command_sigfox.func->readAutoConnectsToTheAP(&at_command_sigfox.data, &en);
#endif
	return flag;
}

void sigfoxWriteMACAddressofTheStationDef(SIGFOX_DATA *data){
	memcpy(data->parameter.station.mac, DEFAULT_STATION_MAC, sizeof(DEFAULT_STATION_MAC));
	at_command_sigfox.func->writeMACAddressofTheStationDef(&at_command_sigfox.data, data->parameter.station.mac);
	sfm10r4.func->write(&sfm10r4.data,IT);
}

uint8_t sigfoxReadMACAddressofTheStationDef(SIGFOX_DATA *data){
	uint8_t flag = 0;
#if defined(_AT_COMMAND_SIGFOX_H_)
	flag = at_command_sigfox.func->readMACAddressofTheStationDef(&at_command_sigfox.data, data->parameter.station.mac);
#endif
	return flag;
}

void sigfoxWriteDefaultIPofTheStationDef(SIGFOX_DATA *data){
	memcpy(data->parameter.station.ip, DEFAULT_STATION_IP, sizeof(DEFAULT_STATION_IP));
	memcpy(data->parameter.station.gateway, DEFAULT_STATION_GATEWAY, sizeof(DEFAULT_STATION_GATEWAY));
	memcpy(data->parameter.station.netmask, DEFAULT_STATION_NETMASK, sizeof(DEFAULT_STATION_NETMASK));
	at_command_sigfox.func->writeDefaultIPofTheStationDef(&at_command_sigfox.data, data->parameter.station.ip, data->parameter.station.gateway, data->parameter.station.netmask);
	sfm10r4.func->write(&sfm10r4.data,IT);
}

uint8_t sigfoxReadDefaultIPofTheStationDef(SIGFOX_DATA *data){
	uint8_t flag = 0;
#if defined(_AT_COMMAND_SIGFOX_H_)
	flag = at_command_sigfox.func->readDefaultIPofTheStationDef(&at_command_sigfox.data, data->parameter.station.ip, data->parameter.station.gateway, data->parameter.station.netmask);
#endif
	return flag;
}

void sigfoxWriteEstablishConnectionToAP(SIGFOX_DATA *data){

#if defined(_SFM10R4_H_) && defined(_AT_COMMAND_SIGFOX_H_)
	at_command_sigfox.func->writeConnectToAnAPDef(&at_command_sigfox.data, DEFAULT_AP_SSID, DEFAULT_AP_PWD);
	sfm10r4.func->write(&sfm10r4.data,IT);
#endif

}

uint8_t sigfoxReadEstablishConnectionToAP(SIGFOX_DATA *data){
	uint8_t flag = 0;
#if defined(_AT_COMMAND_SIGFOX_H_)
	flag = at_command_sigfox.func->readConnectToAnAPDef(&at_command_sigfox.data, data->parameter.ap.ssid, data->parameter.ap.pwd);
#endif
	return flag;
}

void sigfoxWriteEstablishTCPConnection(SIGFOX_DATA *data){

#if defined(_SFM10R4_H_) && defined(_AT_COMMAND_SIGFOX_H_)
	at_command_sigfox.func->writeEstablishesTCPConnection(&at_command_sigfox.data, DEFAULT_AP_IP, DEFAULT_AP_PORT, 0);
	sfm10r4.func->write(&sfm10r4.data,IT);
#endif

}

uint8_t sigfoxReadEstablishTCPConnection(SIGFOX_DATA *data){
	uint8_t flag = 0;
#if defined(_AT_COMMAND_SIGFOX_H_)
	uint16_t buf = 0;
	flag = at_command_sigfox.func->readEstablishesTCPConnection(&at_command_sigfox.data, data->parameter.ap.ip, &data->parameter.ap.port, &buf);
#endif

	return flag;
}


void sigfoxWriteTransmissionMode(SIGFOX_DATA *data){

#if defined(_SFM10R4_H_) && defined(_AT_COMMAND_SIGFOX_H_)
	at_command_sigfox.func->writeTransmissionMode(&at_command_sigfox.data, 1);
	sfm10r4.func->write(&sfm10r4.data,IT);
#endif

}

uint8_t sigfoxReadTransmissionMode(SIGFOX_DATA *data){
	uint8_t flag = 0;
#if defined(_AT_COMMAND_SIGFOX_H_)
	flag = at_command_sigfox.func->readTransmissionMode(&at_command_sigfox.data);
#endif

	return flag;
}


void sigfoxWriteSendDataMode(SIGFOX_DATA *data){
#if defined(_SFM10R4_H_) && defined(_AT_COMMAND_SIGFOX_H_)
	at_command_sigfox.func->writeTCPSingleConnectionUARTPassthroughSendDataMode(&at_command_sigfox.data);
	sfm10r4.func->write(&sfm10r4.data,IT);
#endif
}

uint8_t sigfoxReadSendDataMode(SIGFOX_DATA *data){
	uint8_t flag = 0;
#if defined(_SFM10R4_H_) && defined(_AT_COMMAND_SIGFOX_H_)
	flag = at_command_sigfox.func->readTCPSingleConnectionUARTPassthroughSendDataMode(&at_command_sigfox.data);
#endif

	return flag;
}

void sigfoxWriteData(void *data, uint16_t size){
	#if defined(_SFM10R4_H_)
		sfm10r4.data.driver.write_size = size;
		memcpy(sfm10r4.data.driver.write, data, sfm10r4.data.driver.write_size);
		sfm10r4.func->write(&sfm10r4.data,IT);
	#endif
}


void* sigfoxReadRawData(void){
	void *data = 0;
	#if defined(_SFM10R4_H_)
		data = sfm10r4.data.driver.read;
	#endif
	return data;
}

uint8_t sigfoxReadData(void *data, uint16_t *size){
	uint8_t flag = 0;
#if defined(_SFM10R4_H_) && defined(_AT_COMMAND_SIGFOX_H_)
	uint16_t receive_size = strlen((char *)sfm10r4.data.driver.read);
	if(receive_size){
		*size = receive_size;
		memcpy(data, sfm10r4.data.driver.read, *size);
		sfm10r4.func->readFlush(&sfm10r4.data,DMA);
		flag = 1;
	}

#endif
	return flag;
}

void sigfoxFlush(SIGFOX_DATA *data){
	if(sfm10r4.func->getStatus()){
		sfm10r4.func->readFlush(&sfm10r4.data,DMA);
	}
}
#endif
SIGFOX_FUNC sigfox_func = {
		.init = sigfoxInitial,
		.deInit = sigfoxDeInitial,
		.writeTestStartup = sigfoxWriteTestStartup,
		.readTestStartup = sigfoxReadTestStartup,
		.writeRestartsTheModule = sigfoxWriteRestartsTheModule,
		.readRestartsTheModule = sigfoxReadRestartsTheModule,
		.writeSleepTheModule = sigfoxWriteSleepTheModule,
		.readSleepTheModule = sigfoxReadSleepTheModule,
		.writeDeepSleepTheModule = sigfoxWriteDeepSleepTheModule,
		.readDeepSleepTheModule = sigfoxReadDeepSleepTheModule,
		.writeIsNeedResetChannel = sigfoxWriteIsNeedResetChannel,
		.readIsNeedResetChannel = sigfoxReadIsNeedResetChannel,
		.writeResetChannel = sigfoxWriteResetChannel,
		.readResetChannel = sigfoxReadResetChannel,
		.writeSendMessage = sigfoxWriteSendMessage,
		.readSendMessage = sigfoxReadSendMessage,

#if 0
		.writeATCommandsEchoing = sigfoxWriteATCommandsEchoing,
		.readATCommandsEchoing = sigfoxReadATCommandsEchoing,
		.writeMaximunValueOfRFTXPower = sigfoxWriteMaximunValueOfRFTXPower,
		.readMaximunValueOfRFTXPower = sigfoxReadMaximunValueOfRFTXPower,
		.writeSIGFOXModeDef = sigfoxWriteSIGFOXModeDef,
		.readSIGFOXModeDef = sigfoxReadSIGFOXModeDef,
		.writeAutoConnectsToTheAP = sigfoxWriteAutoConnectsToTheAP,
		.readAutoConnectsToTheAP = sigfoxReadAutoConnectsToTheAP,
		.writeMACAddressofTheStationDef = sigfoxWriteMACAddressofTheStationDef,
		.readMACAddressofTheStationDef = sigfoxReadMACAddressofTheStationDef,
		.writeDefaultIPofTheStationDef = sigfoxWriteDefaultIPofTheStationDef,
		.readDefaultIPofTheStationDef = sigfoxReadDefaultIPofTheStationDef,
		.writeEstablishConnectionToAP = sigfoxWriteEstablishConnectionToAP,
		.readEstablishConnectionToAP = sigfoxReadEstablishConnectionToAP,
		.writeEstablishTCPConnection = sigfoxWriteEstablishTCPConnection,
		.readEstablishTCPConnection = sigfoxReadEstablishTCPConnection,
		.writeTransmissionMode = sigfoxWriteTransmissionMode,
		.readTransmissionMode = sigfoxReadTransmissionMode,
		.writeSendDataMode = sigfoxWriteSendDataMode,
		.readSendDataMode = sigfoxReadSendDataMode,
		.writeData = sigfoxWriteData,
		.readRawData = sigfoxReadRawData,
		.readData = sigfoxReadData,
		.flush = sigfoxFlush,
#endif
};

#endif	/* _SIGFOX_H_ */
