/****************************************************************************
 *
 *   Copyright (c) 2012-2019 PX4 Development Team. All rights reserved.
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
 **********************************************************************/

 /**
 * @file testapp_subscribe.c
 * Minimal application example for PX4 autopilot
 *
 * @author Example User <mail@example.com>
 */

#include <px4_platform_common/posix.h>
#include <px4_log.h>
#include <uORB/topics/sensor_combined.h>

__EXPORT int testapp_subscribe_main(int argc, char *argv[]);

int testapp_subscribe_main(int argc, char *argv[])
{
    PX4_INFO("testapp_subscribe Hello Sky!");

    // sensor_sub_fd is a topic handle
    int sensor_sub_fd = orb_subscribe(ORB_ID(sensor_combined));
    
    /* limit the update rate to 5 Hz */
    orb_set_interval( sensor_sub_fd, 200 );

	/* one could wait for multiple topics with this technique, just using one here */
    px4_pollfd_struct_t fds[] = 
    {
    	{ .fd = sensor_sub_fd,	.events = POLLIN },
    };

    while( 1 )
    {
    	/* wait for sensor update of 1 file descriptor for 1000 ms (1 second) */
	    //int poll_ret = px4_poll(fds, 1, 1000);
	    px4_poll(fds, 1, 1000);

	    if( fds[0].revents & POLLIN )
	    {
	    	struct sensor_combined_s raw;

	    	/* copy sensors raw data into local buffer */
	    	orb_copy( ORB_ID( sensor_combined ), sensor_sub_fd, &raw );

	    	PX4_INFO("Accelerometer:\t%.4f\t%.4f\t%.4f",
                    (double)raw.accelerometer_m_s2[0],
                    (double)raw.accelerometer_m_s2[1],
                    (double)raw.accelerometer_m_s2[2]);
	    }
	}

    return OK;
}