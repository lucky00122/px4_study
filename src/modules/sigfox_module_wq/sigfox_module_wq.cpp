/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "sigfox_module_wq.hpp"

#include <drivers/drv_hrt.h>

#include <fcntl.h>

using namespace time_literals;

sigfox_module_wq::sigfox_module_wq() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::sigfox_module_wq)
{
}

sigfox_module_wq::~sigfox_module_wq()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
	m_sigfoxPort_wq.InitSerialPort( FALSE );
}

bool sigfox_module_wq::init()
{
	ScheduleOnInterval(60_s); // 1000 us interval, 1000 Hz rate
	
	return true;
}

void sigfox_module_wq::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);


	// DO WORK
	

#if 0
	// Example
	// grab latest accelerometer data
	_sensor_accel_sub.update();
	const sensor_accel_s &accel = _sensor_accel_sub.get();
	
#if 0 // test code
	//printf("gx:%f gy:%f gz:%f\n", (double)accel.x, (double)accel.y, (double)accel.z);
#endif


	// Example
	// publish some data
	orb_test_s data{};
	data.timestamp = hrt_absolute_time();
	data.val = accel.device_id;
	_orb_test_pub.publish(data);

#if 0 // test code
	_orb_test_sub.update();
	const orb_test_s &test = _orb_test_sub.get();

	printf("id:%d, testid:%d\n", accel.device_id, test.val);
#endif
#else
	char pcCmdIn[64];
	time_t timep; 
	struct tm *pcurTime;
#ifdef TEST_FOR_RECEIVE_RESPONSE
    char pcResp[64];
    uint8_t nData = 0;
    int16_t wSizeRead = 0;
    int i = 0;
#endif
		
	time( &timep );
		
	pcurTime = gmtime( &timep );

	sprintf( pcCmdIn, "AT$SF=%04d%02d%02d%02d%02d%02d,1", 
			( pcurTime->tm_year + 1900 ), ( pcurTime->tm_mon + 1 ), pcurTime->tm_mday, 
			pcurTime->tm_hour, pcurTime->tm_min, pcurTime->tm_sec );

	printf( "%s\n", pcCmdIn );
	
    // Open UART Port
    if( !m_sigfoxPort_wq.InitSerialPort( TRUE ) )
    {
        printf("ERROR: OpenPort fail.\n" );
    }
	else
	{
    	printf("\nEnter Sigfox Module, Sigfox message will be sent per minute\n\n");
	}
	
	if(!m_sigfoxPort_wq.WriteData( ( uint8_t* )pcCmdIn, ( strlen( pcCmdIn ) + 1 ) ))
	{
        printf("ERROR: WriteData fail. (errno:%d %s)\n", errno, strerror( errno ) );

		return;
	}

#ifdef TEST_FOR_RECEIVE_RESPONSE
    printf( "\nWaiting for Sigfox response...\n\n" );

	// Wait and read the response from Sigfox module
	while( !should_exit() && pcResp[i-1] != '\n' )
	{
	    wSizeRead = m_sigfoxPort_wq.ReadData( &nData, 1 );

	    if( wSizeRead > 0 )
	    {
	        pcResp[i++] = nData;
	    }
	}

	if( !should_exit() )
	{
	   	pcResp[i-1] = 0x00;

	   	printf( "%s\n", pcResp );

	   	i = 0;
	   	memset( pcResp, 0, sizeof(pcResp) );
	}
#else
	px4_usleep(100_ms);
#endif

	m_sigfoxPort_wq.InitSerialPort( FALSE );
	
    printf("\nGoodbye Sigfox Module...\n\n");

#endif

	perf_end(_loop_perf);
}

int sigfox_module_wq::task_spawn(int argc, char *argv[])
{
	sigfox_module_wq *instance = new sigfox_module_wq();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int sigfox_module_wq::print_status()
{
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	return 0;
}

int sigfox_module_wq::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int sigfox_module_wq::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
sigfox_module_wq

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("sigfox_module_wq", "sigfox");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int sigfox_module_wq_main(int argc, char *argv[])
{
	return sigfox_module_wq::main(argc, argv);
}
