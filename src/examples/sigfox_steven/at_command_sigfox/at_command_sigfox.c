/*
 * at_command_sigfox.c
 *
 *  Created on: 2020-09-24
 *      Author: PandaWang
 */

#include "at_command_sigfox.h"
#include "string.h"
#include "stdio.h"
#include "stdlib.h"
#include "debug.h"

#define AT					"AT"
#define RESP_END			"\n"
#define RST					"$P=0"              // Reset Sigfox module
#define SLEEP				"$P=1"              // Sleep (Send any character to wake up)
#define DEEP_SLEEP			"$P=2"              // Deep Sleep (Toggle GPIO9 or RESET_N to wake up)
#define GET_CHANNEL_INFO	"$GI?"              // Get channel information
#define RST_CHANNEL	        "$RC"               // Reset channel
#define SEND_MSG			"$SF="              // Send Sigfox message

#define DOWNLINK            ",1"
#define NO_DOWNLINK         ",0"

#define GMR					"+GMR"
#define GSLP				"+GSLP"
#define ATE					"ATE"
#define RESTORE				"+RESTORE"
#define UART_CUR			"+UART_CUR"
#define UART_DEF			"+UART_DEF"
#define SLEEP				"+SLEEP"
#define WAKEUPGPIO			"+WAKEUPGPIO"
#define RFPOWER				"+RFPOWER"
#define RFVDD				"+RFVDD"
#define SYSRAM				"+SYSRAM"
#define SYSADC				"+SYSADC"
#define SYSIOSETCFG			"+SYSIOSETCFG"
#define SYSIOGETCFG			"+SYSIOGETCFG"
#define SYSGPIODIR			"+SYSGPIODIR"
#define SYSGPIOWRITE		"+SYSGPIOWRITE"
#define SYSGPIOREAD			"+SYSGPIOREAD"
#define SYSMSG_CUR			"+SYSMSG_CUR"
#define SYSMSG_DEF			"+SYSMSG_DEF"


#define CWMODE_CUR			"+CWMODE_CUR"
#define CWMODE_DEF			"+CWMODE_DEF"
#define CWJAP_CUR			"+CWJAP_CUR"
#define CWJAP_DEF			"+CWJAP_DEF"
#define CWLAPOPT			"+CWLAPOPT"
#define CWLAP				"+CWLAP"
#define CWQAP				"+CWQAP"
#define CWSAP_CUR			"+CWSAP_CUR"
#define CWSAP_DEF			"+CWSAP_DEF"
#define CWLIF				"+CWLIF"
#define CWDHCP_CUR			"+CWDHCP_CUR"
#define CWDHCP_DEF			"+CWDHCP_DEF"
#define CWDHCPS_CUR			"+CWDHCPS_CUR"
#define CWDHCPS_DEF			"+CWDHCPS_DEF"
#define CWAUTOCONN			"+CWAUTOCONN"
#define CIPSTAMAC_CUR		"+CIPSTAMAC_CUR"
#define CIPSTAMAC_DEF		"+CIPSTAMAC_DEF"
#define CIPAPMAC_CUR		"+CIPAPMAC_CUR"
#define CIPAPMAC_DEF		"+CIPAPMAC_DEF"
#define CIPSTA_CUR			"+CIPSTA_CUR"
#define CIPSTA_DEF			"+CIPSTA_DEF"
#define CIPAP_CUR			"+CIPAP_CUR"
#define CIPAP_DEF			"+CIPAP_DEF"
#define CWSTARTSMART		"+CWSTARTSMART"
#define CWSTOPSMART			"+CWSTOPSMART"
#define CWSTARTDISCOVER		"+CWSTARTDISCOVER"
#define CWSTOPDISCOVER		"+CWSTOPDISCOVER"
#define WPS					"+WPS"
#define MDNS				"+MDNS"
#define CWHOSTNAME			"+CWHOSTNAME"
#define CWCOUNTRY_CUR		"+CWCOUNTRY_CUR"
#define CWCOUNTRY_DEF		"+CWCOUNTRY_DEF"


#define CIPSTATUS			"+CIPSTATUS"
#define CIPDOMAIN			"+CIPDOMAIN"
#define CIPSTART			"+CIPSTART"
#define CIPSSLSIZE			"+CIPSSLSIZE"
#define CIPSSLCONF			"+CIPSSLCONF"
#define CIPSEND				"+CIPSEND"
#define CIPSENDEX			"+CIPSENDEX"
#define CIPSENDBUF			"+CIPSENDBUF"
#define CIPBUFRESET			"+CIPBUFRESET"
#define CIPBUFSTATUS		"+CIPBUFSTATUS"
#define CIPCHECKSEQ			"+CIPCHECKSEQ"
#define CIPCLOSE			"+CIPCLOSE"
#define CIFSR				"+CIFSR"
#define CIPMUX				"+CIPMUX"
#define CIPSERVER			"+CIPSERVER"
#define CIPSERVERMAXCONN	"+CIPSERVERMAXCONN"
#define CIPMODE				"+CIPMODE"
#define SAVETRANSLINK		"+SAVETRANSLINK"
#define CIPSTO				"+CIPSTO"
#define PING				"+PING"
#define CIUPDATE			"+CIUPDATE"
#define CIPDINFO			"+CIPDINFO"
#define IPD					"+IPD"
#define CIPRECVMODE			"+CIPRECVMODE"
#define CIPRECVDATA			"+CIPRECVDATA"
#define CIPRECVLEN			"+CIPRECVLEN"
#define CIPSNTPCFG			"+CIPSNTPCFG"
#define CIPSNTPTIME			"+CIPSNTPTIME"
#define CIPDNS_CUR			"+CIPDNS_CUR"
#define CIPDNS_DEF			"+CIPDNS_DEF"

#define OK					    "OK"
#define READY				    "ready"
#define ERROR				    "ERROR"
#define WAIT_DOWNLINK_TIMEOUT   "ERR_SFX_ERR_SEND_FRAME_WAIT_TIMEOUT"
#define WIFI_CONNECTED		"WIFI CONNECTED"
#define WIFI_GOT_IP			"WIFI GOT IP"
#define WIFI_DISCONNECTED	"WIFI DISCONNECTED"
#define BUSY_S				"busy s..."
#define BUSY_P				"busy p..."
#define CONNECT				"CONNECT"
#define CLOSED				"CLOSED"
#define LINK_CONN			"+LINK_CONN"
#define ALREADY_CONNECTED	"ALREADY CONNECTED"
#define STA_CONNECTED		"+STA_CONNECTED:"
#define DIST_STA_IP			"+DIST_STA_IP:"
#define STA_DISCONNECTED	"+STA_DISCONNECTED:"

#define DEFAULT_AP			"Aeroprobing wifi 0"
#define DEFAULT_AP_PASSWORD	"7878787878"

void writeTestStartup(AT_COMMAND_SIGFOX_DATA *at){
	*at->write_size = sizeof(AT);         // Include the null-character
	memcpy(at->write,AT,*at->write_size);
}

uint8_t readTestStartup(AT_COMMAND_SIGFOX_DATA *at){
	uint8_t flag = AT_BUSY;
    char *str = at->read;
	char *ok = strstr(str,OK RESP_END);
	char *error = strstr(str,ERROR);
	if(ok != 0){
		flag = AT_OK;
	}
	else if(error != 0){
		flag = AT_ERROR;
	}
	else{
		flag = AT_BUSY;
	}
	return flag;
}

void writeRestartsTheModule(AT_COMMAND_SIGFOX_DATA *at){
	*at->write_size = sizeof(AT RST);         // Include the null-character
	memcpy(at->write,AT RST,*at->write_size);
}

uint8_t readRestartsTheModule(AT_COMMAND_SIGFOX_DATA *at){
	uint8_t flag = AT_BUSY;
    char *str = at->read;
	char *ok = strstr(str,OK RESP_END);
	char *error = strstr(str,ERROR);
	if(ok != 0){
		flag = AT_OK;
	}
	else if(error != 0){
		flag = AT_ERROR;
	}
	else{
		flag = AT_BUSY;
	}
	return flag;
}

void writeSleepTheModule(AT_COMMAND_SIGFOX_DATA *at){
	*at->write_size = sizeof(AT SLEEP);                 // Include the null-character
	memcpy(at->write,AT SLEEP,*at->write_size);
}

uint8_t readSleepTheModule(AT_COMMAND_SIGFOX_DATA *at){
	uint8_t flag = AT_BUSY;
    char *str = at->read;
	char *ok = strstr(str,OK RESP_END);
	char *error = strstr(str,ERROR);
	if(ok != 0){
		flag = AT_OK;
	}
	else if(error != 0){
		flag = AT_ERROR;
	}
	else{
		flag = AT_BUSY;
	}
	return flag;
}

void writeDeepSleepTheModule(AT_COMMAND_SIGFOX_DATA *at){
	*at->write_size = sizeof(AT DEEP_SLEEP);         // Include the null-character
	memcpy(at->write,AT DEEP_SLEEP,*at->write_size);
}

uint8_t readDeepSleepTheModule(AT_COMMAND_SIGFOX_DATA *at){
	uint8_t flag = AT_BUSY;
    char *str = at->read;
	char *ok = strstr(str,OK RESP_END);
	char *error = strstr(str,ERROR);
	if(ok != 0){
		flag = AT_OK;
	}
	else if(error != 0){
		flag = AT_ERROR;
	}
	else{
		flag = AT_BUSY;
	}
	return flag;
}

void writeIsNeedResetChannel(AT_COMMAND_SIGFOX_DATA *at){
	*at->write_size = sizeof(AT GET_CHANNEL_INFO);         // Include the null-character
	memcpy(at->write,AT GET_CHANNEL_INFO,*at->write_size);
}

uint8_t readIsNeedResetChannel(AT_COMMAND_SIGFOX_DATA *at, uint8_t *isNeedRstChannel){
	uint8_t flag = AT_BUSY;
    char *str = at->read;
	char *ok = strstr(str,",");         // return X,Y
	char *error = strstr(str,ERROR);
    char * pch = NULL;
    int prevMacroCh = 0;
    int nextMicroCh = 0;
	if(ok != 0){
		flag = AT_OK;
        pch = strtok(str,",");
        if(pch != NULL){
            prevMacroCh = atoi(pch);
            pch = strtok (NULL, ",");
            if(pch != NULL){
                nextMicroCh = atoi(pch);
            }
        }
        if(prevMacroCh==0 || nextMicroCh<3){
            *isNeedRstChannel = true;
        }
        else{
            *isNeedRstChannel = false;
        }
	}
	else if(error != 0){
		flag = AT_ERROR;
	}
	else{
		flag = AT_BUSY;
	}
	return flag;
}

void writeResetChannel(AT_COMMAND_SIGFOX_DATA *at){
	*at->write_size = sizeof(AT RST_CHANNEL);         // Include the null-character
	memcpy(at->write,AT RST_CHANNEL,*at->write_size);
}

uint8_t readResetChannel(AT_COMMAND_SIGFOX_DATA *at){
	uint8_t flag = AT_BUSY;
    char *str = at->read;
	char *ok = strstr(str,OK RESP_END);
	char *error = strstr(str,ERROR);
	if(ok != 0){
		flag = AT_OK;
	}
	else if(error != 0){
		flag = AT_ERROR;
	}
	else{
		flag = AT_BUSY;
	}
	return flag;
}

void writeSendMessage(AT_COMMAND_SIGFOX_DATA *at, int32_t lat_in, int32_t lon_in, int32_t alt_in, uint8_t msgFormat, uint8_t isDownlink){
	char buf[sizeof(AT SEND_MSG) + AT_SF_MSG_MAX_SIZE + 2] = {0};
    char buf1[AT_SF_MSG_MAX_SIZE];
    int32_t lat = 0;
    int32_t lon = 0;
    int32_t alt = 0;
    
	lat = lat_in / 1000;	// remove the last 3 number to reduce sigfox message bytes (0XX.XXXX)
	lon = lon_in / 1000;	// remove the last 3 number to reduce sigfox message bytes (XXX.XXXX)
	alt = alt_in / 1000;	// remove the last 3 number to reduce sigfox message bytes (m)

	if(msgFormat == MSG_FORMAT_BYTES){
		sprintf(buf, AT SEND_MSG "%06x%06x%04x", lat, lon, alt);
	}
	else if(msgFormat == MSG_FORMAT_READABLE){   // ex: A0235894B1521234A0200E
        strcat(buf, AT SEND_MSG);
        if(lat>=0){
		    sprintf(buf1, "A%07d", lat);          // Positive Value
        }
        else{
		    sprintf(buf1, "B%07d", abs(lat));     // Negative Value
        }
        strcat(buf, buf1);
        if(lon>=0){
		    sprintf(buf1, "A%07d", lon);          // Positive Value
        }
        else{
		    sprintf(buf1, "B%07d", abs(lon));     // Negative Value
        }
        strcat(buf, buf1);
        if(alt>=0){
		    sprintf(buf1, "A%04d", alt);          // Positive Value
        }
        else{
		    sprintf(buf1, "B%04d", abs(alt));     // Negative Value
        }
        strcat(buf, buf1);
        strcat(buf, "E");
	}

    // Request to downlink frame?
    if(isDownlink){
        strcat(buf, DOWNLINK);
    }
    else{
        strcat(buf, NO_DOWNLINK);
    }
        
	*at->write_size = strlen(buf)+1;                // Include the null-character
	memcpy(at->write,buf,*at->write_size);
}

uint8_t readSendMessage(AT_COMMAND_SIGFOX_DATA *at){
	uint8_t flag = AT_BUSY;
    char *str = at->read;
	char *ok = strstr(str,OK RESP_END);
	char *error = strstr(str,ERROR);
    char *timeout = strstr(str,WAIT_DOWNLINK_TIMEOUT);
	if(ok != 0){
		flag = AT_OK;
	}
	else if(error != 0 || timeout != 0){
		flag = AT_ERROR;
	}
	else{
		flag = AT_BUSY;
	}
	return flag;
}


#if 0
void writeCheckVersionInformation(AT_COMMAND_SIGFOX_DATA *at){
	*at->write_size = strlen(AT GMR END);
	memcpy(at->write,AT GMR END,*at->write_size);
}

uint8_t readCheckVersionInformation(AT_COMMAND_SIGFOX_DATA *at, char *at_version, char * sdk_version, char *compile_time, char *bin_version){
	uint8_t flag = AT_BUSY;

	char *str = strstr(at->read,GMR);
	if(str != 0){
		char *ok = strstr(str, END OK);
		char *error = strstr(str, END ERROR);
		if(ok != 0){
			char *at_version_ptr = strstr(str, "AT version:");
			char *sdk_version_ptr = strstr(at_version_ptr, "SDK version:");
			char *compile_time_ptr = strstr(sdk_version_ptr, "compile time:");
			char *bin_version_ptr = strstr(compile_time_ptr, "Bin version(Wroom 02):");
			if(at_version_ptr != 0){
				memcpy(at_version,strstr(at_version_ptr, ":") + 1,strstr(at_version_ptr, "\r\n") - strstr(at_version_ptr, ":") - 1);
				if(sdk_version_ptr != 0){
					memcpy(sdk_version,strstr(sdk_version_ptr, ":") + 1,strstr(sdk_version_ptr, "\r\n") - strstr(sdk_version_ptr, ":") - 1);
					if(compile_time_ptr != 0){
						memcpy(compile_time,strstr(compile_time_ptr, ":") + 1,strstr(compile_time_ptr, "\r\n") - strstr(compile_time_ptr, ":") - 1);
						if(bin_version_ptr != 0){
							memcpy(bin_version,strstr(bin_version_ptr, ":") + 1,strstr(bin_version_ptr, "\r\n") - strstr(bin_version_ptr, ":") - 1);
							flag = AT_OK;
						}
					}
				}
			}
		}
		else if(error != 0){
			flag = AT_ERROR;
		}
		else{
			flag = AT_BUSY;
		}
	}

	return flag;
}

void writeATCommandsEchoing(AT_COMMAND_SIGFOX_DATA *at, uint8_t mode){
	*at->write_size = strlen(ATE "1" END);
	if(mode){
		memcpy(at->write,ATE "1" END,*at->write_size);
	}
	else{
		memcpy(at->write,ATE "0" END,*at->write_size);
	}
}

uint8_t readATCommandsEchoing(AT_COMMAND_SIGFOX_DATA *at, uint8_t *mode){
	uint8_t flag = AT_BUSY;
	char *str = strstr(at->read,ATE);
	if(str != 0){
		char *ok = strstr(str,END END OK);
		char *error = strstr(str,END END ERROR);
		if(ok != 0 && (ok - str) == strlen(ATE "1")){
			*mode = atoi(ok-1);
			flag = AT_OK;
		}
		else if(error != 0 && (error - str) == strlen(ATE "1")){
			flag = AT_ERROR;
		}
		else{
			flag = AT_BUSY;
		}
	}
	return flag;
}

void writeMaximunValueOfRFTXPower(AT_COMMAND_SIGFOX_DATA *at, uint8_t value){
	if(value > 82){
		value = 82;
	}
	char num[3];
	sprintf(num,"%d",value);
	char buf[sizeof(AT RFPOWER "=") - 1 + sizeof(num) - 1 + sizeof(END) - 1] = {0};
    strcat(buf, AT RFPOWER "=");
    strcat(buf, num);
    strcat(buf, END);
	*at->write_size = strlen(buf);
	memcpy(at->write,buf,*at->write_size);
}

uint8_t readMaximunValueOfRFTXPower(AT_COMMAND_SIGFOX_DATA *at, uint8_t *value){
	uint8_t flag = AT_BUSY;
	char *str = strstr(at->read,AT RFPOWER "=");
	if(str != 0){
		char *ok = strstr(str,END END OK);
		char *error = strstr(str,END END ERROR);
		if(ok != 0 && (ok - str) <= strlen(AT RFPOWER "=82")){
			char *ok = strstr(str, "=");
			*value = atoi(ok + 1);
			flag = AT_OK;
		}
		else if(error != 0 && (error - str) <= strlen(AT RFPOWER "=82")){
			flag = AT_ERROR;
		}
		else{
			flag = AT_BUSY;
		}
	}
	return flag;
}

void writeWIFIModeDef(AT_COMMAND_SIGFOX_DATA *at, uint8_t value){
	if(value > 3){
		value = 3;
	}
	else if(value < 1){
		value = 1;
	}
	char num[2];
	sprintf(num,"%d",value);
	char buf[sizeof(AT CWMODE_DEF "=") - 1 + sizeof(END) - 1 + sizeof(num) - 1] = {0};
    strcat(buf, AT CWMODE_DEF "=");
    strcat(buf, num);
    strcat(buf, END);
	*at->write_size = strlen(buf);
	memcpy(at->write,buf,*at->write_size);
}

uint8_t readWIFIModeDef(AT_COMMAND_SIGFOX_DATA *at, uint8_t *mode){
	uint8_t flag = AT_BUSY;
	char *str = strstr(at->read,CWMODE_DEF);
	if(str != 0){
		char *ok = strstr(str,END END OK);
		char *error = strstr(str,END END ERROR);
		if((ok != 0) && (ok - str) == strlen(CWMODE_DEF "=1")){
			*mode = atoi(ok-1);
			flag = AT_OK;
		}
		else if(error != 0 && (error - str) == strlen(CWMODE_DEF "=1")){
			flag = AT_ERROR;
		}
		else{
			flag = AT_BUSY;
		}
	}
	return flag;
}

void writeConnectToAnAPDef(AT_COMMAND_SIGFOX_DATA *at, char *ssid, char* pwd){
	char buf[sizeof(AT CWJAP_DEF "=") + 100 + sizeof(END) - 1] = {0};
    char double_quotes[2] = {0x22,'\0'};
	strcat(buf, AT CWJAP_DEF "=");
	strcat(buf, double_quotes);
	strcat(buf, ssid);
	strcat(buf, double_quotes);
	strcat(buf, ",");
	strcat(buf, double_quotes);
	strcat(buf, pwd);
	strcat(buf, double_quotes);
    strcat(buf, END);
	*at->write_size = strlen(buf);
	memcpy(at->write,buf,*at->write_size);
}

uint8_t readConnectToAnAPDef(AT_COMMAND_SIGFOX_DATA *at, char *ssid, char* pwd){
	uint8_t flag = AT_BUSY;
	char *str = strstr(at->read,CWJAP_DEF);
	if(str != 0){
		char *ok = strstr(str,END END OK);
		char *wif_connected = strstr(str, WIFI_CONNECTED);
		char *wif_got_ip = strstr(str, WIFI_GOT_IP);
		char *error = strstr(str,END END ERROR);
		if((ok != 0) && wif_connected != 0 && wif_got_ip != 0){
			char *ssid_ptr = strstr(str,"=");
			char *pwd_ptr = strstr(ssid_ptr,",");
			memcpy(ssid, ssid_ptr + 2,pwd_ptr - ssid_ptr - 3);
			memcpy(pwd, pwd_ptr + 2,ok - pwd_ptr - 3);
			flag = AT_OK;
		}
		else if(error != 0){
			flag = AT_ERROR;
		}
		else{
			flag = AT_BUSY;
		}
	}
	return flag;
}

void writeDisconnectToAnAP(AT_COMMAND_SIGFOX_DATA *at){
	*at->write_size = strlen(AT CWQAP END);
	memcpy(at->write,AT CWQAP END,*at->write_size);
}


uint8_t readDisconnectToAnAP(AT_COMMAND_SIGFOX_DATA *at){
	uint8_t flag = AT_BUSY;
	char *str = strstr(at->read,CWQAP);
	if(str != 0){
		char *ok = strstr(str,END END OK);
		char *error = strstr(str,END END ERROR);
		if(ok != 0 && (ok - str) == strlen(CWQAP)){
			flag = AT_OK;
		}
		else if(error != 0 && (error - str) == strlen(CWQAP)){
			flag = AT_ERROR;
		}
		else{
			flag = AT_BUSY;
		}
	}
	return flag;
}

void writeConfiguresTheSoftAPDef(AT_COMMAND_SIGFOX_DATA *at, char *ssid, char *pwd, uint8_t chl, uint8_t ecn, uint8_t max_conn, uint8_t ssid_hidden){
	char buf[sizeof(AT CWSAP_DEF "=") + 108 + sizeof(END) - 1] = {0};
    char double_quotes[2] = {0x22,'\0'};
	char chl_buf[2] = {0},max_conn_buf[2] = {0},ecn_buf[2] = {0};
	if(max_conn > 8){
		max_conn = 8;
	}
	else if(max_conn < 1){
		max_conn = 1;
	}

	if(chl > 14){
		chl = 14;
	}
	else if(chl < 1){
		chl = 1;
	}

	if(ecn > 4 || ecn == 1){
		ecn = 4;
	}

	sprintf(max_conn_buf,"%d",max_conn);
	sprintf(chl_buf,"%d",chl);
	sprintf(ecn_buf,"%d",ecn);
	strcat(buf, AT CWSAP_DEF "=");
	strcat(buf, double_quotes);
	strcat(buf, ssid);
	strcat(buf, double_quotes);
	strcat(buf, ",");
	strcat(buf, double_quotes);
	strcat(buf, pwd);
	strcat(buf, double_quotes);
	strcat(buf, ",");
	strcat(buf, chl_buf);
	strcat(buf, ",");
	strcat(buf, ecn_buf);
	strcat(buf, ",");
	strcat(buf, max_conn_buf);
	strcat(buf, ",");
	if(ssid_hidden){
		strcat(buf, "1");
	}
	else{
		strcat(buf, "0");
	}
    strcat(buf, END);
	*at->write_size = strlen(buf);
	memcpy(at->write,buf,*at->write_size);
}

uint8_t readConfiguresTheSoftAPDef(AT_COMMAND_SIGFOX_DATA *at, char *ssid, char *pwd, uint8_t *chl, uint8_t *ecn, uint8_t *max_conn, uint8_t *ssid_hidden){
	uint8_t flag = AT_BUSY;
	char *str = strstr(at->read,CWSAP_DEF);
	if(str != 0){
		char *ok = strstr(str,END END OK);
		char *error = strstr(str,END END ERROR);
		if(ok != 0 && *(ok - 2) == ',' && *(ok - 4) == ','){
			char *ssid_ptr = strstr(str,"=");
			char *pwd_ptr = strstr(ssid_ptr,",");
			char *chl_ptr = strstr(pwd_ptr + 1,",");
			char *ecn_ptr = strstr(chl_ptr + 1,",");
			char *max_conn_ptr = strstr(ecn_ptr + 1,",");
			char *ssid_hidden_ptr = strstr(max_conn_ptr + 1,",");
			memcpy(ssid, ssid_ptr + 2,pwd_ptr - ssid_ptr - 3);
			memcpy(pwd, pwd_ptr + 2,chl_ptr - pwd_ptr - 3);
			*chl = atoi(chl_ptr + 1);
			*ecn = atoi(ecn_ptr + 1);
			*max_conn = atoi(max_conn_ptr + 1);
			*ssid_hidden = atoi(ssid_hidden_ptr + 1);
			flag = AT_OK;
		}
		else if(error != 0 && *(error - 2) == ',' && *(error - 4) == ','){
			flag = AT_ERROR;
		}
		else{
			flag = AT_BUSY;
		}
	}
	return flag;
}

void writeCheckConnectedStationsIP(AT_COMMAND_SIGFOX_DATA *at){
	*at->write_size = strlen(AT CWLIF END);
	memcpy(at->write,AT CWLIF END,*at->write_size);
}

uint8_t readCheckConnectedStationsIP(AT_COMMAND_SIGFOX_DATA *at, char *ip, char *mac){
	uint8_t flag = AT_BUSY;
	char *str = strstr(at->read,CWLIF);
	if(str != 0){
		char *ok = strstr(str,END END OK);
		char *error = strstr(str,END END ERROR);
		if(ok != 0 && *(ok - 3) == ':' && *(ok - 6) == ':'){
			char *ip_ptr = strstr(str,"F\r\n") + 3;
			char *mac_ptr = strstr(ip_ptr,",") + 1;
			memcpy(ip, ip_ptr,mac_ptr - ip_ptr - 1);
			memcpy(mac, mac_ptr,ok - mac_ptr);
			flag = AT_OK;
		}
		else if(error != 0 && *(error - 3) == ':' && *(error - 6) == ':'){
			flag = AT_ERROR;
		}
		else{
			flag = AT_BUSY;
		}
	}
	return flag;
}

void writeDHCPEnableDef(AT_COMMAND_SIGFOX_DATA *at, uint8_t mode, uint8_t en){
	char buf[sizeof(AT CWDHCP_DEF "=") - 1 + 3 + sizeof(END) - 1] = {0};
	char mode_buf[2] = {0};
	if(mode > 2){
		mode = 2;
	}
	sprintf(mode_buf,"%d",mode);
	strcat(buf, AT CWDHCP_DEF "=");
	strcat(buf, mode_buf);
	strcat(buf, ",");
	if(en){
		strcat(buf, "1");
	}
	else{
		strcat(buf, "0");
	}
    strcat(buf, END);
	*at->write_size = strlen(buf);
	memcpy(at->write,buf,*at->write_size);
}

uint8_t readDHCPEnableDef(AT_COMMAND_SIGFOX_DATA *at, uint8_t *mode, uint8_t *en){
	uint8_t flag = AT_BUSY;
	char *str = strstr(at->read,CWDHCP_DEF);
	if(str != 0){
		char *ok = strstr(str,END END OK);
		char *error = strstr(str,END END OK);
		if(ok != 0 && *(ok - 2) == ','){
			char *mode_ptr = strstr(str,"=") + 1;
			char *en_ptr = strstr(mode_ptr,",") + 1;
			*mode = atoi(mode_ptr);
			*en = atoi(en_ptr);
			flag = AT_OK;
		}
		else if(error != 0 && *(error - 2) == ','){
			flag = AT_ERROR;
		}
		else{
			flag = AT_BUSY;
		}
	}
	return flag;
}

void writeAutoConnectsToTheAP(AT_COMMAND_SIGFOX_DATA *at, uint8_t en){
	*at->write_size = strlen(AT CWAUTOCONN "=1" END);
	if(en){
		memcpy(at->write,AT CWAUTOCONN "=1" END,*at->write_size);
	}
	else{
		memcpy(at->write,AT CWAUTOCONN "=0" END,*at->write_size);
	}
}


uint8_t readAutoConnectsToTheAP(AT_COMMAND_SIGFOX_DATA *at, uint8_t *en){
	uint8_t flag = AT_BUSY;
	char *str = strstr(at->read,CWAUTOCONN);
	if(str != 0){
		char *ok = strstr(str,END END OK);
		char *error = strstr(str,END END OK);
		if(ok != 0 && (ok - str) == strlen(CWAUTOCONN "=1")){
			*en = atoi(ok-1);
			flag = AT_OK;
		}
		else if(error != 0 && (error - str) == strlen(CWAUTOCONN "=1")){
			flag = AT_ERROR;
		}
		else{
			flag = AT_BUSY;
		}
	}
	return flag;
}

void writeMACAddressofTheStationDef(AT_COMMAND_SIGFOX_DATA *at, char *mac){
	char buf[sizeof(AT CIPSTAMAC_DEF "=") - 1 + 19 + sizeof(END) - 1] = {0};
    char double_quotes[2] = {0x22,'\0'};
	strcat(buf, AT CIPSTAMAC_DEF "=");
	strcat(buf, double_quotes);
	strcat(buf, mac);
	strcat(buf, double_quotes);
    strcat(buf, END);
	*at->write_size = strlen(buf);
	memcpy(at->write,buf,*at->write_size);
}


uint8_t readMACAddressofTheStationDef(AT_COMMAND_SIGFOX_DATA *at, char *mac){
	uint8_t flag = AT_BUSY;
	char *str = strstr(at->read,CIPSTAMAC_DEF);
	if(str != 0){
		char *ok = strstr(str,END END OK);
		char *error = strstr(str,END END OK);
		if(ok != 0 && *(ok - 4) == ':' && *(ok - 7) == ':'){
			char *mac_ptr = strstr(str,"=") + 2;
			memcpy(mac, mac_ptr,ok - mac_ptr - 1);
			flag = AT_OK;
		}
		else if(error != 0 && *(error - 4) == ':' && *(error - 7) == ':'){
			flag = AT_ERROR;
		}
		else{
			flag = AT_BUSY;
		}
	}
	return flag;
}


void writeMACAddressofTheAPDef(AT_COMMAND_SIGFOX_DATA *at, char *mac){
	char buf[sizeof(AT CIPAPMAC_DEF "=") - 1 + 19 + sizeof(END) - 1] = {0};
    char double_quotes[2] = {0x22,'\0'};
	strcat(buf, AT CIPAPMAC_DEF "=");
	strcat(buf, double_quotes);
	strcat(buf, mac);
	strcat(buf, double_quotes);
    strcat(buf, END);
	*at->write_size = strlen(buf);
	memcpy(at->write,buf,*at->write_size);
}


uint8_t readMACAddressofTheAPDef(AT_COMMAND_SIGFOX_DATA *at, char *mac){
	uint8_t flag = AT_BUSY;
	char *str = strstr(at->read,CIPAPMAC_DEF);
	if(str != 0){
		char *ok = strstr(str,END END OK);
		char *error = strstr(str,END END OK);
		if(ok != 0 && *(ok - 4) == ':' && *(ok - 7) == ':'){
			char *mac_ptr = strstr(str,"=") + 2;
			memcpy(mac, mac_ptr,ok - mac_ptr - 1);
			flag = AT_OK;
		}
		else if(error != 0 && *(error - 4) == ':' && *(error - 7) == ':'){
			flag = AT_ERROR;
		}
		else{
			flag = AT_BUSY;
		}
	}
	return flag;
}

void writeDefaultIPofTheStationDef(AT_COMMAND_SIGFOX_DATA *at, char *ip, char *gateway, char* netmask){
	char buf[sizeof(AT CIPSTA_DEF "=") - 1 + 53 + sizeof(END) - 1] = {0};
    char double_quotes[2] = {0x22,'\0'};
	strcat(buf, AT CIPSTA_DEF "=");
	strcat(buf, double_quotes);
	strcat(buf, ip);
	strcat(buf, double_quotes);
	strcat(buf, ",");
	strcat(buf, double_quotes);
	strcat(buf, gateway);
	strcat(buf, double_quotes);
	strcat(buf, ",");
	strcat(buf, double_quotes);
	strcat(buf, netmask);
	strcat(buf, double_quotes);
    strcat(buf, END);
	*at->write_size = strlen(buf);
	memcpy(at->write,buf,*at->write_size);
}


uint8_t readDefaultIPofTheStationDef(AT_COMMAND_SIGFOX_DATA *at, char *ip, char *gateway, char* netmask){
	uint8_t flag = AT_BUSY;
	char *str = strstr(at->read,CIPSTA_DEF);
	if(str != 0){
		char *ok = strstr(str,END END OK);
		char *error = strstr(str,END END OK);
		if(ok != 0 && (ok - str) <= (strlen(CIPSTA_DEF "=") + 53)){
			char *ip_ptr = strstr(str,"=") + 2;
			char *gateway_ptr = strstr(ip_ptr,",") + 2;
			char *netmask_ptr = strstr(gateway_ptr,",") + 2;
			memcpy(ip, ip_ptr,gateway_ptr - ip_ptr - 3);
			memcpy(gateway, gateway_ptr,netmask_ptr - gateway_ptr - 3);
			memcpy(netmask, netmask_ptr,ok - netmask_ptr - 1);
			flag = AT_OK;
		}
		else if(error != 0 && (error - str) <= (strlen(CIPSTA_DEF "=") + 53)){
			flag = AT_ERROR;
		}
		else{
			flag = AT_BUSY;
		}
	}
	return flag;
}

void writeDefaultIPofTheAPDef(AT_COMMAND_SIGFOX_DATA *at, char *ip, char *gateway, char* netmask){
	char buf[sizeof(AT CIPAP_DEF "=") - 1 + 53 + sizeof(END) - 1] = {0};
    char double_quotes[2] = {0x22,'\0'};
	strcat(buf, AT CIPAP_DEF "=");
	strcat(buf, double_quotes);
	strcat(buf, ip);
	strcat(buf, double_quotes);
	strcat(buf, ",");
	strcat(buf, double_quotes);
	strcat(buf, gateway);
	strcat(buf, double_quotes);
	strcat(buf, ",");
	strcat(buf, double_quotes);
	strcat(buf, netmask);
	strcat(buf, double_quotes);
    strcat(buf, END);
	*at->write_size = strlen(buf);
	memcpy(at->write,buf,*at->write_size);
}


uint8_t readDefaultIPofTheAPDef(AT_COMMAND_SIGFOX_DATA *at, char *ip, char *gateway, char* netmask){
	uint8_t flag = AT_BUSY;
	char *str = strstr(at->read,CIPAP_DEF);
	if(str != 0){
		char *ok = strstr(str,END END OK);
		char *error = strstr(str,END END OK);
		if(ok != 0 && (ok - str) <= (strlen(CIPAP_DEF "=") + 53)){
			char *ip_ptr = strstr(str,"=") + 2;
			char *gateway_ptr = strstr(ip_ptr,",") + 2;
			char *netmask_ptr = strstr(gateway_ptr,",") + 2;
			memcpy(ip, ip_ptr,gateway_ptr - ip_ptr - 3);
			memcpy(gateway, gateway_ptr,netmask_ptr - gateway_ptr - 3);
			memcpy(netmask, netmask_ptr,ok - netmask_ptr - 1);
			flag = AT_OK;
		}
		else if(error != 0 && (error - str) <= (strlen(CIPAP_DEF "=") + 53)){
			flag = AT_ERROR;
		}
		else{
			flag = AT_BUSY;
		}
	}
	return flag;
}

void writeEstablishesTCPConnection(AT_COMMAND_SIGFOX_DATA *at, char *ip, uint16_t port, uint16_t tcp_keep_alive){
	char buf[sizeof(AT CIPSTART "=") - 1 + 33 + sizeof(END) - 1] = {0};
	char port_buf[5] = {0};
    char double_quotes[2] = {0x22,'\0'};
    char tcp_keep_alive_buf[5] = {0};
	if(port > 9999){
		port = 9999;
	}
	if(tcp_keep_alive > 7200){
		tcp_keep_alive = 7200;
	}
	sprintf(port_buf,"%d",port);
	sprintf(tcp_keep_alive_buf,"%d",tcp_keep_alive);
	strcat(buf, AT CIPSTART "=");
	strcat(buf, double_quotes);
	strcat(buf, "TCP");
	strcat(buf, double_quotes);
	strcat(buf, ",");
	strcat(buf, double_quotes);
	strcat(buf, ip);
	strcat(buf, double_quotes);
	strcat(buf, ",");
	strcat(buf, port_buf);
	strcat(buf, ",");
	strcat(buf, tcp_keep_alive_buf);
    strcat(buf, END);
	*at->write_size = strlen(buf);
	memcpy(at->write,buf,*at->write_size);
}

uint8_t readEstablishesTCPConnection(AT_COMMAND_SIGFOX_DATA *at, char *ip, uint16_t *port, uint16_t *tcp_keep_alive){
	uint8_t flag = AT_BUSY;
	char *str = strstr(at->read,CIPSTART);
	if(str != 0){
		char *ok = strstr(str,END END OK);
		char *link_conn = strstr(str,LINK_CONN);
		char *already_connected = strstr(str, ALREADY_CONNECTED);
		char *error = strstr(str,END END OK);
		if((ok != 0 && link_conn != 0) || already_connected != 0){
			char *ip_ptr = strstr(str,",") + 2;
			char *port_ptr = strstr(ip_ptr,",") + 1;
			char *tcp_keep_alive_ptr = strstr(port_ptr,",") + 1;
			memcpy(ip, ip_ptr,port_ptr - ip_ptr - 2);
			*port = atoi(port_ptr);
			*tcp_keep_alive = atoi(tcp_keep_alive_ptr);
			flag = AT_OK;
		}
		else if(error != 0 && already_connected == 0){
			flag = AT_ERROR;
		}
		else{
			flag = AT_BUSY;
		}
	}
	return flag;
}

void writeEstablishesUDPConnection(AT_COMMAND_SIGFOX_DATA *at, char *ip, uint16_t port, uint16_t udp_local_port, uint8_t udp_mode){
	char buf[sizeof(AT CIPSTART "=") - 1 + 36 + sizeof(END) - 1] = {0};
	char port_buf[5] = {0};
	char udp_local_port_buf[5] = {0};
    char double_quotes[2] = {0x22,'\0'};
	if(port > 9999){
		port = 9999;
	}
	if(udp_local_port > 9999){
		udp_local_port = 9999;
	}
	sprintf(port_buf,"%d",port);
	sprintf(udp_local_port_buf,"%d",udp_local_port);
	strcat(buf, AT CIPSTART "=");
	strcat(buf, double_quotes);
	strcat(buf, "UDP");
	strcat(buf, double_quotes);
	strcat(buf, ",");
	strcat(buf, double_quotes);
	strcat(buf, ip);
	strcat(buf, double_quotes);
	strcat(buf, ",");
	strcat(buf, port_buf);
	strcat(buf, ",");
	strcat(buf, udp_local_port_buf);
	strcat(buf, ",");
	switch(udp_mode){
	default:
	case 0:
		strcat(buf, "0");
		break;
	case 1:
		strcat(buf, "1");
		break;
	case 2:
		strcat(buf, "2");
		break;
	}
    strcat(buf, END);
	*at->write_size = strlen(buf);
	memcpy(at->write,buf,*at->write_size);
}

void writeCloseTCPOrUDPOrSSLConnection(AT_COMMAND_SIGFOX_DATA *at){
	*at->write_size = strlen(AT CIPCLOSE END);
	memcpy(at->write,AT CIPCLOSE END,*at->write_size);
}

uint8_t readCloseTCPOrUDPOrSSLConnection(AT_COMMAND_SIGFOX_DATA *at){
	uint8_t flag = AT_BUSY;
	char *str = strstr(at->read,CIPCLOSE);
	if(str != 0){
		char *ok = strstr(str,END END OK);
		char *error = strstr(str,END END ERROR);
		if(ok != 0 && (ok - str) == strlen(CIPCLOSE)){
			flag = AT_OK;
		}
		else if(error != 0 && (error - str) == strlen(CIPCLOSE)){
			flag = AT_ERROR;
		}
		else{
			flag = AT_BUSY;
		}
	}
	return flag;
}

void writeEnableOrDisableMultipleConnections(AT_COMMAND_SIGFOX_DATA *at, uint8_t enable){
	*at->write_size = strlen(AT CIPMUX "=1" END);
	if(enable){
		memcpy(at->write,AT CIPMUX "=1" END,*at->write_size);
	}
	else{
		memcpy(at->write,AT CIPMUX "=0" END,*at->write_size);
	}
}

uint8_t readEnableOrDisableMultipleConnections(AT_COMMAND_SIGFOX_DATA *at){
	uint8_t flag = AT_BUSY;
	char *str = strstr(at->read,CIPMUX);
	if(str != 0){
		char *ok = strstr(str,END END OK);
		char *error = strstr(str,END END ERROR);
		if(ok != 0 && (ok - str) == strlen(CIPMUX "=1")){
			flag = AT_OK;
		}
		else if(error != 0 && (error - str) == strlen(CIPMUX "=1")){
			flag = AT_ERROR;
		}
		else{
			flag = AT_BUSY;
		}
	}
	return flag;
}

void writeDeletesOrCreatesTCPServer(AT_COMMAND_SIGFOX_DATA *at, uint8_t enable, uint16_t port){
	char buf[sizeof(AT CIPSERVER "=") - 1 + 6 + sizeof(END) - 1] = {0};
	char port_buf[5] = {0};
	sprintf(port_buf,"%d",port);
	if(enable){
		strcat(buf, AT CIPSERVER "=1,");
	}
	else{
		strcat(buf, AT CIPSERVER "=0,");
	}
    strcat(buf, port_buf);
    strcat(buf, END);
	*at->write_size = strlen(buf);
	memcpy(at->write,buf,*at->write_size);
}

uint8_t readDeletesOrCreatesTCPServer(AT_COMMAND_SIGFOX_DATA *at, uint8_t *enable, uint16_t *port){
	uint8_t flag = AT_BUSY;
	char *str = strstr(at->read,CIPSERVER);
	if(str != 0){
		char *ok = strstr(str,END END OK);
		char *error = strstr(str,END END ERROR);
		if(ok != 0 && (ok - str) <= strlen(CIPSERVER "=1") + 5){
			char *enable_ptr = strstr(str,"=")+1;
			char *port_ptr = strstr(enable_ptr,",")+1;
			*enable = atoi(enable_ptr);
			*port = atoi(port_ptr);
			flag = AT_OK;
		}
		else if(error != 0 && (error - str) <= strlen(CIPSERVER "=1") + 5){
			flag = AT_ERROR;
		}
		else{
			flag = AT_BUSY;
		}
	}
	return flag;
}

void writeTransmissionMode(AT_COMMAND_SIGFOX_DATA *at, uint8_t mode){
	*at->write_size = strlen(AT CIPMODE "=1" END);
	if(mode){
		memcpy(at->write,AT CIPMODE "=1" END,*at->write_size);
	}
	else{
		memcpy(at->write,AT CIPMODE "=0" END,*at->write_size);
	}
}

uint8_t readTransmissionMode(AT_COMMAND_SIGFOX_DATA *at){
	uint8_t flag = AT_BUSY;
	char *str = strstr(at->read,CIPMODE);
	if(str != 0){
		char *ok = strstr(str,END END OK);
		char *error = strstr(str,END END ERROR);
		if(ok != 0 && (ok - str) == strlen(CIPMODE "=1")){
			flag = AT_OK;
		}
		else if(error != 0 && (error - str) == strlen(CIPMODE "=1")){
			flag = AT_ERROR;
		}
		else{
			flag = AT_BUSY;
		}
	}
	return flag;
}

void writeTCPSingleConnectionUARTPassthroughSendDataMode(AT_COMMAND_SIGFOX_DATA *at){
	*at->write_size = strlen(AT CIPSEND END);
	memcpy(at->write,AT CIPSEND END,*at->write_size);
}

uint8_t readTCPSingleConnectionUARTPassthroughSendDataMode(AT_COMMAND_SIGFOX_DATA *at){
	uint8_t flag = AT_BUSY;
	char *str = strstr(at->read,CIPSEND);
	if(str != 0){
		char *ok = strstr(str,END END OK);
		char *error = strstr(str,END END ERROR);
		if(ok != 0 && (ok - str) == strlen(CIPSEND)){
			flag = AT_OK;
		}
		else if(error != 0 && (error - str) == strlen(CIPSEND)){
			flag = AT_ERROR;
		}
		else{
			flag = AT_BUSY;
		}
	}
	return flag;
}

uint8_t readWIFIConnectedOK(AT_COMMAND_SIGFOX_DATA *at){
	uint8_t flag = AT_BUSY;
	char *str = strstr(at->read,WIFI_CONNECTED);
	if(str != 0){
		flag = AT_OK;
	}
	return flag;
}
#endif

AT_COMMAND_SIGFOX_FUNC at_command_sigfox_func = {
		.writeTestStartup = writeTestStartup,
		.readTestStartup = readTestStartup,
		.writeRestartsTheModule = writeRestartsTheModule,
		.readRestartsTheModule = readRestartsTheModule,
		.writeSleepTheModule = writeSleepTheModule,
		.readSleepTheModule = readSleepTheModule,
		.writeDeepSleepTheModule = writeDeepSleepTheModule,
		.readDeepSleepTheModule = readDeepSleepTheModule,
		.writeIsNeedResetChannel = writeIsNeedResetChannel,
		.readIsNeedResetChannel = readIsNeedResetChannel,
		.writeResetChannel = writeResetChannel,
		.readResetChannel = readResetChannel,
		.writeSendMessage = writeSendMessage,
		.readSendMessage = readSendMessage,

#if 0
		.writeCheckVersionInformation = writeCheckVersionInformation,
		.readCheckVersionInformation = readCheckVersionInformation,
		.writeATCommandsEchoing = writeATCommandsEchoing,
		.readATCommandsEchoing = readATCommandsEchoing,
		.writeMaximunValueOfRFTXPower = writeMaximunValueOfRFTXPower,
		.readMaximunValueOfRFTXPower = readMaximunValueOfRFTXPower,
		.writeWIFIModeDef = writeWIFIModeDef,
		.readWIFIModeDef = readWIFIModeDef,
		.writeConnectToAnAPDef = writeConnectToAnAPDef,
		.readConnectToAnAPDef = readConnectToAnAPDef,
		.writeDisconnectToAnAP = writeDisconnectToAnAP,
		.readDisconnectToAnAP = readDisconnectToAnAP,
		.writeConfiguresTheSoftAPDef = writeConfiguresTheSoftAPDef,
		.readConfiguresTheSoftAPDef = readConfiguresTheSoftAPDef,
		.writeCheckConnectedStationsIP = writeCheckConnectedStationsIP,
		.readCheckConnectedStationsIP = readCheckConnectedStationsIP,
		.writeDHCPEnableDef = writeDHCPEnableDef,
		.readDHCPEnableDef = readDHCPEnableDef,
		.writeAutoConnectsToTheAP = writeAutoConnectsToTheAP,
		.readAutoConnectsToTheAP = readAutoConnectsToTheAP,
		.writeMACAddressofTheStationDef = writeMACAddressofTheStationDef,
		.readMACAddressofTheStationDef = readMACAddressofTheStationDef,
		.writeMACAddressofTheAPDef = writeMACAddressofTheAPDef,
		.readMACAddressofTheAPDef = readMACAddressofTheAPDef,
		.writeDefaultIPofTheStationDef = writeDefaultIPofTheStationDef,
		.readDefaultIPofTheStationDef = readDefaultIPofTheStationDef,
		.writeDefaultIPofTheAPDef = writeDefaultIPofTheAPDef,
		.readDefaultIPofTheAPDef = readDefaultIPofTheAPDef,
		.writeEstablishesTCPConnection = writeEstablishesTCPConnection,
		.readEstablishesTCPConnection = readEstablishesTCPConnection,
		.writeEstablishesUDPConnection = writeEstablishesUDPConnection,
		.writeCloseTCPOrUDPOrSSLConnection = writeCloseTCPOrUDPOrSSLConnection,
		.readCloseTCPOrUDPOrSSLConnection = readCloseTCPOrUDPOrSSLConnection,
		.writeEnableOrDisableMultipleConnections = writeEnableOrDisableMultipleConnections,
		.readEnableOrDisableMultipleConnections = readEnableOrDisableMultipleConnections,
		.writeDeletesOrCreatesTCPServer = writeDeletesOrCreatesTCPServer,
		.readDeletesOrCreatesTCPServer = readDeletesOrCreatesTCPServer,
		.writeTransmissionMode = writeTransmissionMode,
		.readTransmissionMode = readTransmissionMode,
		.writeTCPSingleConnectionUARTPassthroughSendDataMode = writeTCPSingleConnectionUARTPassthroughSendDataMode,
		.readTCPSingleConnectionUARTPassthroughSendDataMode = readTCPSingleConnectionUARTPassthroughSendDataMode,
		.readWIFIConnectedOK = readWIFIConnectedOK,
#endif
};
