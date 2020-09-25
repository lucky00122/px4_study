/*
 * at_command_sigfox.h
 *
 *  Created on: 2020-09-24
 *      Author: PandaWang
 */

#ifndef _AT_COMMAND_SIGFOX_H_
#define _AT_COMMAND_SIGFOX_H_

#ifndef COMPILER_ON_PX4
#include "../config/config.h"
#endif

#define AT_SF_MSG_MAX_SIZE	24	// AT$SF=112233445566778899AABBCC,1

typedef enum _MSG_FORMAT{
	MSG_FORMAT_READABLE	= 0x00,	// readable format. (0XX.XXXXAXXX.XXXXAXXXX -> [Latitude]A[Longitude]A[Altitude])
	MSG_FORMAT_BYTES	= 0x01	// bytes format. (XXXXXXYYYYYYZZZZ -> [Latitude 3 bytes]A[Longitude 3 bytes]A[Altitude 2 bytes])
}MSG_FORMAT, *PMSG_FORMAT;

enum{
	AT_ERROR = 0,
	AT_OK,
	AT_BUSY
};

enum{
	STATION_MODE = 1,
	AP_MODE = 2,
	AP_AND_STATION_MODE = 3,
};

typedef struct{
	char *write;
	uint16_t *write_size;
	char *read;
	uint16_t *read_size;
}AT_COMMAND_SIGFOX_DATA;

typedef struct{
	void (*writeTestStartup)(AT_COMMAND_SIGFOX_DATA *at);
	uint8_t (*readTestStartup)(AT_COMMAND_SIGFOX_DATA *at);
	void (*writeRestartsTheModule)(AT_COMMAND_SIGFOX_DATA *at);
	uint8_t (*readRestartsTheModule)(AT_COMMAND_SIGFOX_DATA *at);
	void (*writeSleepTheModule)(AT_COMMAND_SIGFOX_DATA *at);
	uint8_t (*readSleepTheModule)(AT_COMMAND_SIGFOX_DATA *at);
	void (*writeDeepSleepTheModule)(AT_COMMAND_SIGFOX_DATA *at);
	uint8_t (*readDeepSleepTheModule)(AT_COMMAND_SIGFOX_DATA *at);
	void (*writeIsNeedResetChannel)(AT_COMMAND_SIGFOX_DATA *at);
	uint8_t (*readIsNeedResetChannel)(AT_COMMAND_SIGFOX_DATA *at, uint8_t *isNeedRstChannel);
	void (*writeResetChannel)(AT_COMMAND_SIGFOX_DATA *at);
	uint8_t (*readResetChannel)(AT_COMMAND_SIGFOX_DATA *at);
	void (*writeSendMessage)(AT_COMMAND_SIGFOX_DATA *at, int32_t lat_in, int32_t lon_in, int32_t alt_in, uint8_t msgFormat, uint8_t isDownlink);
	uint8_t (*readSendMessage)(AT_COMMAND_SIGFOX_DATA *at);
    
#if 0
	void (*writeCheckVersionInformation)(AT_COMMAND_SIGFOX_DATA *at);
	uint8_t (*readCheckVersionInformation)(AT_COMMAND_SIGFOX_DATA *at, char *at_version, char * sdk_version, char *compile_time, char *bin_version);

	
	void (*writeATCommandsEchoing)(AT_COMMAND_SIGFOX_DATA *at, uint8_t mode);
	uint8_t (*readATCommandsEchoing)(AT_COMMAND_SIGFOX_DATA *at, uint8_t *mode);
	void (*writeMaximunValueOfRFTXPower)(AT_COMMAND_SIGFOX_DATA *at, uint8_t value);
	uint8_t (*readMaximunValueOfRFTXPower)(AT_COMMAND_SIGFOX_DATA *at, uint8_t *value);
	void (*writeWIFIModeDef)(AT_COMMAND_SIGFOX_DATA *at, uint8_t value);
	uint8_t (*readWIFIModeDef)(AT_COMMAND_SIGFOX_DATA *at, uint8_t *value);
	void (*writeConnectToAnAPDef)(AT_COMMAND_SIGFOX_DATA *at, char *ssid, char* pwd);
	uint8_t (*readConnectToAnAPDef)(AT_COMMAND_SIGFOX_DATA *at, char *ssid, char* pwd);
	void (*writeDisconnectToAnAP)(AT_COMMAND_SIGFOX_DATA *at);
	uint8_t (*readDisconnectToAnAP)(AT_COMMAND_SIGFOX_DATA *at);
	void (*writeConfiguresTheSoftAPDef)(AT_COMMAND_SIGFOX_DATA *at, char *ssid, char *pwd, uint8_t chl, uint8_t ecn, uint8_t max_conn, uint8_t ssid_hidden);
	uint8_t (*readConfiguresTheSoftAPDef)(AT_COMMAND_SIGFOX_DATA *at, char *ssid, char *pwd, uint8_t *chl, uint8_t *ecn, uint8_t *max_conn, uint8_t *ssid_hidden);
	void (*writeCheckConnectedStationsIP)(AT_COMMAND_SIGFOX_DATA *at);
	uint8_t (*readCheckConnectedStationsIP)(AT_COMMAND_SIGFOX_DATA *at, char *ip, char *mac);
	void (*writeDHCPEnableDef)(AT_COMMAND_SIGFOX_DATA *at, uint8_t mode, uint8_t en);
	uint8_t (*readDHCPEnableDef)(AT_COMMAND_SIGFOX_DATA *at, uint8_t *mode, uint8_t *en);
	void (*writeAutoConnectsToTheAP)(AT_COMMAND_SIGFOX_DATA *at, uint8_t en);
	uint8_t (*readAutoConnectsToTheAP)(AT_COMMAND_SIGFOX_DATA *at, uint8_t *en);
	void (*writeMACAddressofTheStationDef)(AT_COMMAND_SIGFOX_DATA *at, char *mac);
	uint8_t (*readMACAddressofTheStationDef)(AT_COMMAND_SIGFOX_DATA *at, char *mac);
	void (*writeMACAddressofTheAPDef)(AT_COMMAND_SIGFOX_DATA *at, char *mac);
	uint8_t (*readMACAddressofTheAPDef)(AT_COMMAND_SIGFOX_DATA *at, char *mac);
	void (*writeDefaultIPofTheStationDef)(AT_COMMAND_SIGFOX_DATA *at, char *ip, char *gateway, char* netmask);
	uint8_t (*readDefaultIPofTheStationDef)(AT_COMMAND_SIGFOX_DATA *at, char *ip, char *gateway, char* netmask);
	void (*writeDefaultIPofTheAPDef)(AT_COMMAND_SIGFOX_DATA *at, char *ip, char *gateway, char* netmask);
	uint8_t (*readDefaultIPofTheAPDef)(AT_COMMAND_SIGFOX_DATA *at, char *ip, char *gateway, char* netmask);
	void (*writeEstablishesTCPConnection)(AT_COMMAND_SIGFOX_DATA *at, char *ip, uint16_t port, uint16_t tcp_keep_alive);
	uint8_t (*readEstablishesTCPConnection)(AT_COMMAND_SIGFOX_DATA *at, char *ip, uint16_t *port, uint16_t *tcp_keep_alive);
	void (*writeEstablishesUDPConnection)(AT_COMMAND_SIGFOX_DATA *at, char *ip, uint16_t port, uint16_t udp_local_port, uint8_t udp_mode);
	void (*writeCloseTCPOrUDPOrSSLConnection)(AT_COMMAND_SIGFOX_DATA *at);
	uint8_t (*readCloseTCPOrUDPOrSSLConnection)(AT_COMMAND_SIGFOX_DATA *at);
	void (*writeEnableOrDisableMultipleConnections)(AT_COMMAND_SIGFOX_DATA *at, uint8_t enable);
	uint8_t (*readEnableOrDisableMultipleConnections)(AT_COMMAND_SIGFOX_DATA *at);
	void (*writeDeletesOrCreatesTCPServer)(AT_COMMAND_SIGFOX_DATA *at, uint8_t enable, uint16_t port);
	uint8_t (*readDeletesOrCreatesTCPServer)(AT_COMMAND_SIGFOX_DATA *at, uint8_t *enable, uint16_t *port);
	void (*writeTransmissionMode)(AT_COMMAND_SIGFOX_DATA *at, uint8_t mode);
	uint8_t (*readTransmissionMode)(AT_COMMAND_SIGFOX_DATA *at);
	void (*writeTCPSingleConnectionUARTPassthroughSendDataMode)(AT_COMMAND_SIGFOX_DATA *at);
	uint8_t (*readTCPSingleConnectionUARTPassthroughSendDataMode)(AT_COMMAND_SIGFOX_DATA *at);
	uint8_t (*readWIFIConnectedOK)(AT_COMMAND_SIGFOX_DATA *at);
#endif
}AT_COMMAND_SIGFOX_FUNC;

extern AT_COMMAND_SIGFOX_FUNC at_command_sigfox_func;

typedef struct{
	AT_COMMAND_SIGFOX_DATA data;
	AT_COMMAND_SIGFOX_FUNC *func;
}AT_COMMAND_SIGFOX;

#endif /* _AT_COMMAND_SIGFOX_H_ */
