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
 * @file testapp_publish.c
 * Minimal application example for PX4 autopilot
 *
 * @author Example User <mail@example.com>
 */

#include <string.h>
#include <px4_platform_common/posix.h>
#include <px4_log.h>
//#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>

__EXPORT int testapp_publish_main(int argc, char *argv[]);

struct vehicle_attitude_s att;
int error_counter = 0;

int testapp_publish_main(int argc, char *argv[])
{
    PX4_INFO("testapp_publish Hello Sky!");

    // sensor_sub_fd is a topic handle
    int sensor_sub_fd = orb_subscribe(ORB_ID(sensor_combined));

    /* limit the update rate to 5 Hz */
    orb_set_interval( sensor_sub_fd, 200 );

    /* advertise attitude topic */
    memset(&att, 0, sizeof(att));
    orb_advert_t att_pub_fd = orb_advertise( ORB_ID( vehicle_attitude ), &att );

	/* one could wait for multiple topics with this technique, just using one here */
    px4_pollfd_struct_t fds[] = 
    {
    	{ .fd = sensor_sub_fd,	.events = POLLIN },
    };

    for( int i = 0; i < 5; i++ )
    {
    	/* wait for sensor update of 1 file descriptor for 1000 ms (1 second) */
	    int poll_ret = px4_poll(fds, 1, 1000);

        /* handle the poll result */
        if( poll_ret == 0 )
        {
            /* this means none of our providers is giving us data */
            PX4_ERR( "No data time-out" );
        }
        else if( poll_ret < 0 )
        {
            /* this is seriously bad - should be an emergency */
            if( error_counter < 10 || error_counter % 50 == 0 ) 
            {
                /* use a counter to prevent flooding (and slowing us down) */
                PX4_ERR( "ERROR return value from poll(): %d", poll_ret );
            }

            error_counter++;
        }
        else
        {
            if( fds[0].revents & POLLIN )
            {
                struct sensor_combined_s raw;

                /* copy sensors raw data into local buffer */
                orb_copy( ORB_ID( sensor_combined ), sensor_sub_fd, &raw );

                PX4_INFO("Accelerometer:\t%8.4f\t%8.4f\t%8.4f",
                        (double)raw.accelerometer_m_s2[0],
                        (double)raw.accelerometer_m_s2[1],
                        (double)raw.accelerometer_m_s2[2]);

                /* set att and publish this information for other apps
                   the following does not have any meaning, it's just an example
                */
                att.q[0] = raw.accelerometer_m_s2[0];
                att.q[1] = raw.accelerometer_m_s2[1];
                att.q[2] = raw.accelerometer_m_s2[2];

                orb_publish( ORB_ID( vehicle_attitude ), att_pub_fd, &att );
            }       

            /* there could be more file descriptors here, in the form like:
             * if (fds[1..n].revents & POLLIN) {}
             */     
        }
	}

    PX4_INFO("exiting");

    return OK;
}