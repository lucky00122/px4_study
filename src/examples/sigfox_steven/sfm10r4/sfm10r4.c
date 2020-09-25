/*
 * sfm10r4.c
 *
 *  Created on: 2020-09-21
 *      Author: PandaWang
 */
#include "sfm10r4.h"

#if defined(_SFM10R4_H_)

#include "../../../config/config.h"
#include "../../../driver/driver.h"
#include "debug.h"
#include "string.h"
#include "stdlib.h"


#if IS_USE_UART(SFM10R4_DRIVER)
static UART_DRIVER *uart = 0;
#endif

uint8_t sfm10r4Initial(SFM10R4_DATA *data){
	uint8_t flag = 0;
	#if IS_USE_UART(SFM10R4_DRIVER)
		uart = system_driver.port.uart.port[SFM10R4_DRIVER - USE_USART1];	// USE_USART1?
		if(uart != 0){

			uart->data.write = data->driver.write;
			uart->data.read = data->driver.read;
			uart->func->readDMA(&uart->data, sizeof(data->driver.read));
			flag = 1;
		}
	#endif
	return flag;
}

uint8_t sfm10r4DeInit(SFM10R4_DATA *data){

	#if IS_USE_UART(SFM10R4_DRIVER)
	uart = 0;
	#endif
	return 1;
}

uint8_t sfm10r4GetStatus(){
	uint8_t flag = 0;
	#if IS_USE_UART(SFM10R4_DRIVER)
		if(uart->data.port->gState == HAL_UART_STATE_READY){
			flag = 1;
		}
		else{
			flag = 0;
		}
	#endif
	return flag;
}

void sfm10r4ReadFlush(SFM10R4_DATA *data, uint8_t mode){
	#if IS_USE_UART(SFM10R4_DRIVER)
		uart->func->abortReadIT(&uart->data);
		memset(uart->data.read,0,sizeof(data->driver.read));
		switch(mode){
			case NONE:
				uart->func->read(&uart->data, sizeof(data->driver.read), 10);
				break;
			case IT:
				uart->func->readIT(&uart->data, sizeof(data->driver.read));
				break;
			case DMA:
				uart->func->readDMA(&uart->data, sizeof(data->driver.read));
				break;
		}
	#endif
}

void sfm10r4Write(SFM10R4_DATA *data, uint8_t mode){
	#if IS_USE_UART(SFM10R4_DRIVER)
		switch(mode){
			case NONE:
				uart->func->write(&uart->data, data->driver.write_size, 10);
				break;
			case IT:
				uart->func->writeIT(&uart->data, data->driver.write_size);
				break;
			case DMA:
				uart->func->writeDMA(&uart->data, data->driver.write_size);
				break;
		}
	#endif
}


SFM10R4_FUNC sfm10r4_func = {
		.init = sfm10r4Initial,
		.deInit = sfm10r4DeInit,
		.getStatus = sfm10r4GetStatus,
		.write = sfm10r4Write,
		.readFlush = sfm10r4ReadFlush
};

#endif
