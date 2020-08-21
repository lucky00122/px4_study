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

#include "sigfox_sender.hpp"

#include <px4_platform_common/getopt.h>
#include <drivers/drv_hrt.h>

#include <fcntl.h>

using namespace time_literals;

sigfox_sender::sigfox_sender( void )
	: ModuleParams(nullptr)
{
}

int sigfox_sender::print_status()
{
	PX4_INFO("Running");
	// TODO: print additional runtime information about the state of the module
	
	return 0;
}

int sigfox_sender::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int sigfox_sender::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd(	"sigfox_sender",
				      				SCHED_DEFAULT,
				      				SCHED_PRIORITY_DEFAULT,
				      				2048,
				      				(px4_main_t)&run_trampoline,
				      				(char *const *)argv);

	if (_task_id < 0) 
	{
		_task_id = -1;
		
		return -errno;
	}

	return 0;
}

sigfox_sender *sigfox_sender::instantiate(int argc, char *argv[])
{
	//int example_param = 0;
	//bool example_flag = false;
	bool error_flag = false;
	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	// parse CLI arguments
	while ((ch = px4_getopt(argc, argv, "p:f", &myoptind, &myoptarg)) != EOF) 
	{
		switch (ch) 
		{
		case 'p':
			//example_param = (int)strtol(myoptarg, nullptr, 10);
			break;

		case 'f':
			//example_flag = true;
			break;

		case '?':
			error_flag = true;
			break;

		default:
			PX4_WARN("unrecognized flag");
			error_flag = true;
			break;
		}
	}

	if (error_flag) 
	{
		return nullptr;
	}

	sigfox_sender *instance = new sigfox_sender();

	if (instance == nullptr) 
	{
		PX4_ERR("alloc failed");
	}

	return instance;
}

void sigfox_sender::run()
{
#if 0
	// Example
	// grab latest accelerometer data
	_sensor_accel_sub.update();
	const sensor_accel_s &accel = _sensor_accel_sub.get();
	
#if 0 // test code
	//PX4_INFO( "gx:%f gy:%f gz:%f", (double)accel.x, (double)accel.y, (double)accel.z);
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

	PX4_INFO( "id:%d, testid:%d", accel.device_id, test.val);
#endif
#else
	char pcCmdIn[64];
	sigfox_data_s sRecvSigfoxCtl = {0};
	sigfox_data_s sSendSigfoxCtl = {0};
    int sigfox_data_sub_fd = 0;
    orb_advert_t sigfox_data_pub_fd = NULL;
	vehicle_gps_position_s sGpsData = {0};
#ifdef TEST_FOR_RECEIVE_RESPONSE
    char pcResp[64];
    uint8_t nData = 0;
    int16_t wSizeRead = 0;
    int i = 0;
#endif

	// Open UART Port
	while( !should_exit() )
	{
	    if( !m_sigfoxPort.InitSerialPort( TRUE ) )
	    {
	        PX4_ERR( "InitSerialPort fail" );

			px4_usleep(5_s);
	    }
		else
		{
	    	PX4_INFO( "Starting Sigfox on %s", UART_DEVICE_NAME );

			break;
		}
	}
	
    /* advertise sigfox_data topic */
    sigfox_data_pub_fd = orb_advertise( ORB_ID( sigfox_data ), &sSendSigfoxCtl );

    /* subscribe sigfox_data topic */
	sigfox_data_sub_fd = orb_subscribe( ORB_ID( sigfox_data ));
    orb_set_interval( sigfox_data_sub_fd, 200 );	// 200ms
    px4_pollfd_struct_t fds[] = 
    {
    	{ .fd = sigfox_data_sub_fd,	.events = POLLIN },
    };

	while( !should_exit() )
	{	
		// Polling Sigfox sending start signal from another module
		int poll_ret = px4_poll( fds, 1, 1000 );	// Wait 1 file descriptor's data updated for 1 seconds.

		if( poll_ret == 0 )		// Time out
        {
            //PX4_INFO( "No data time-out" );

		#if 0// Update the sigfox starting signal for test
			sSendSigfoxCtl.bstartsend = TRUE;
			
            orb_publish( ORB_ID( sigfox_data ), sigfox_data_pub_fd, &sSendSigfoxCtl );
				
			px4_usleep(10_s);
		#endif
			continue;
        }
        else if( poll_ret < 0 )		// Error
        {
            PX4_ERR( "ERROR return value from px4_poll(): %d", poll_ret );

			return;
        }
        else
        {
		    if( fds[0].revents & POLLIN )	// data updated
		    {
		    	orb_copy( ORB_ID( sigfox_data ), sigfox_data_sub_fd, &sRecvSigfoxCtl );

	 	    	//PX4_INFO( "timestamp:%d\tbstartsend:%d", sRecvSigfoxCtl.timestamp, sRecvSigfoxCtl.bstartsend );
		    }
			else
			{
            	PX4_INFO( "Another event occurs: 0x%x", fds[0].revents );
				
				continue;
			}
        }

		// Check if sigfox should start to send
		if( !sRecvSigfoxCtl.bstartsend )
		{
			PX4_INFO( "No sigfox msg to send" );
			
			continue;
		}

		// Start to send sigfox
	#if 0
		time_t timep; 
		struct tm *pcurTime;
	
		time( &timep );
			
		pcurTime = gmtime( &timep );

		sprintf( pcCmdIn, "AT$SF=%04d%02d%02d%02d%02d%02d,1", 
				( pcurTime->tm_year + 1900 ), ( pcurTime->tm_mon + 1 ), pcurTime->tm_mday, 
				pcurTime->tm_hour, pcurTime->tm_min, pcurTime->tm_sec );
	#else
		m_vehicle_gps_position_sub.update();
		sGpsData = m_vehicle_gps_position_sub.get();
	
		sprintf( pcCmdIn, "AT$SF=%010dAA%010d,1", sGpsData.lat, sGpsData.lon );
	#endif
		
	    PX4_INFO( "Send Sigfox: %s", pcCmdIn);
		
		if(!m_sigfoxPort.WriteData( ( uint8_t* )pcCmdIn, ( strlen( pcCmdIn ) + 1 ) ))
		{
	        PX4_ERR( "WriteData fail. (errno:%d %s)", errno, strerror( errno ) );

			return;
		}

#ifdef TEST_FOR_RECEIVE_RESPONSE
	    PX4_INFO( "Waiting for Sigfox response..." );

		// Wait and read the response from Sigfox module
		while( !should_exit() && pcResp[i-1] != '\n' )
		{
		    wSizeRead = m_sigfoxPort.ReadData( &nData, 1 );

		    if( wSizeRead > 0 )
		    {
		        pcResp[i++] = nData;
		    }
		}

		if( !should_exit() )
		{
		   	pcResp[i-1] = 0x00;

		   	PX4_INFO( "%s", pcResp );

		   	i = 0;
		   	memset( pcResp, 0, sizeof(pcResp) );
		}
		
		// Update the sigfox starting signal to FASLE
		sSendSigfoxCtl.bstartsend = FALSE;
			
        orb_publish( ORB_ID( sigfox_data ), sigfox_data_pub_fd, &sSendSigfoxCtl );
#else
		px4_usleep(100_ms);
#endif
	}
#endif

	m_sigfoxPort.InitSerialPort( FALSE );

    PX4_INFO( "Goodbye Sigfox Sender ..." );
}


int sigfox_sender::print_usage(const char *reason)
{
	if (reason) 
	{
		PX4_WARN( "%s\n", reason );
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
sigfox_sender

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("sigfox_sender", "sigfox");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int sigfox_sender_main(int argc, char *argv[])
{
	return sigfox_sender::main(argc, argv);
}
