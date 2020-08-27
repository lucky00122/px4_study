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
 * @file sigfox.c
 * Minimal application example for PX4 autopilot
 *
 * @author Example User <mail@example.com>
 */

#include <string.h>
#include <px4_platform_common/posix.h>
#include <px4_log.h>
#include <uORB/topics/sigfox_data.h>
//#include <px4_time.h>
#include <uORB/uORB.h>
//#include <uORB/topics/sensor_combined.h>
//#include <uORB/topics/vehicle_attitude.h>
#ifdef __PX4_NUTTX
//#include <nuttx/fs/fs.h>
#endif
//#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <termios.h>

#define SIGFOX_UART_DEVICE_NAME "/dev/ttyS3"    // UART4/TELEM4
#define SIGFOX_UART_BAUDRATE    9600
#define SIGFOX_UART_TEST_STR    "AT"
#define CLI_ARGC_MAX            8

__EXPORT int sigfox_main(int argc, char *argv[]);

bool OpenPort( const char* pcPortName );
bool ClosePort( void );
bool SetupUart( int iBaudRate,
                int iDataBits,
                int iStopBits,
                bool bParity,
                bool bHardwareControl );
int16_t ReadData(   uint8_t* punData,
                    uint16_t uwSize );
bool WriteData( const char* punData,
                uint16_t uwSize );

//struct vehicle_attitude_s att;
//int error_counter = 0;

int sigfox_fd = -1;


int sigfox_main(int argc, char *argv[])
{
    char pcCmdIn[64];
    char* psCmdArgv[CLI_ARGC_MAX];
    int iCmdArgc = 0;
    char pcResp[64];
    //char* pcRet = NULL;
    uint8_t nData = 0;
    int16_t wSizeRead = 0;
    int i = 0;
    int iCmdIn = 0;
    int iCmdIdx = 0;
    struct sigfox_data_s sSigfoxData = {0};

    // Open UART Port
    if( !OpenPort( SIGFOX_UART_DEVICE_NAME ) )
    {
        printf("ERROR: OpenPort fail.\n" );

        return ERROR;
    }
    
    // Setup UART
    if( !SetupUart( SIGFOX_UART_BAUDRATE, 8, 1, false, false ) )
    {
        printf("ERROR: SetupUart fail.\n" );

        return ERROR;
    }

    // Subscribe and Advertise sigfox_data Topic
    orb_advert_t sigfox_data_pub_fd = orb_advertise( ORB_ID( sigfox_data ), &sSigfoxData );
    int sigfox_data_sub_fd = orb_subscribe( ORB_ID( sigfox_data ) );
    orb_set_interval( sigfox_data_sub_fd, 200 );	// 200ms
    px4_pollfd_struct_t fds[] = 
    {
    	{ .fd = sigfox_data_sub_fd,	.events = POLLIN },
    };

    printf("\nEnter Sigfox Command Mode, Please Enter AT Command...\n\n");

    while(1)
    {
        printf( "SIGFOX$ " );

        fflush( stdout );

        iCmdIdx = 0;

        while(1)
        {
            iCmdIn = getchar();
                
            // CLI Interface Editing
            if( iCmdIn == EOF )
            {
                printf("ERROR: stdin EOF.\n" );
                
                return ERROR;
            }
            else if( iCmdIn == '\r' || iCmdIn == 0x0A )   // command end
            {
                pcCmdIn[iCmdIdx] = 0x00;
                
                printf( "\r\n" );
                
                //printf( "%s\n", pcCmdIn );

                break;
            }
            else if( iCmdIn == 0x7F )   // backspace
            {
                if( iCmdIdx > 0 )
                {
                    iCmdIdx--;
                
                    printf( "\b \b" );
                }
            }
            else
            {
                putchar( iCmdIn );
                
                pcCmdIn[iCmdIdx++] = (char)iCmdIn;
            }
            
            fflush( stdout );
        }

        // Check the special input.
        if( pcCmdIn[0] == 0x00 )        // only press "ENTER"
        {
            continue;
        }
        else if( !strcmp( pcCmdIn, "q" ) )    // exit sigfox cli
        {
            break;
        }
        else
        {
            iCmdArgc = 0;
            
            psCmdArgv[iCmdArgc] = strtok( pcCmdIn, " " );
              
            while( psCmdArgv[iCmdArgc] != NULL && iCmdArgc < ( CLI_ARGC_MAX - 1 ) )
            {
                iCmdArgc++;
                
                psCmdArgv[iCmdArgc] = strtok( NULL, " " );
            }

            if( iCmdArgc >= CLI_ARGC_MAX )
            {
                printf( "[ERROR] Too many command\n" );
                    
                continue;
            }

            if( !strcmp( psCmdArgv[0], "send" ) )    // send sigfox message through sigfox_sender module
            {
    	    	orb_copy( ORB_ID( sigfox_data ), sigfox_data_sub_fd, &sSigfoxData );

                if( sSigfoxData.bstartsend )
                {
                    printf( "[WARN] Sigfox module is busy\n" );
                }
                else
                {
                    // Set Sigfox start sending signal
                    
                    //memset( &sSigfoxData, 0, sizeof( sSigfoxData ) );

                    sSigfoxData.bstartsend = TRUE;

                    if( psCmdArgv[1] != NULL )
                    {
                        sSigfoxData.unformat = (uint8_t)strtol( psCmdArgv[1], NULL, 10 );
                    }
                    else
                    {
                        sSigfoxData.unformat = 0;
                    }
                    
                    orb_publish( ORB_ID( sigfox_data ), sigfox_data_pub_fd, &sSigfoxData );
                
                    printf( "Notify sigfox_sender module to send and wait for response...\n" );


                    while(1)
                    {
                		// Polling Sigfox sending start signal to be reset
                		int poll_ret = px4_poll( fds, 1, 1000 );	// Wait 1 file descriptor's data updated for 1 seconds.

                		if( poll_ret == 0 )		// Time out
                        {
                            //printf( "No data time-out\n" );
                        }
                        else if( poll_ret < 0 )		// Error
                        {
                            printf( "[ERROR] return value from px4_poll(): %d\n", poll_ret );

                			break;
                        }
                        else
                        {
                		    if( fds[0].revents & POLLIN )	// data updated
                		    {
                                memset( &sSigfoxData, 0, sizeof( sSigfoxData ) );
                            
                		    	orb_copy( ORB_ID( sigfox_data ), sigfox_data_sub_fd, &sSigfoxData );

                                if( !sSigfoxData.bstartsend )
                                {
                	 	    	    printf( "Sigfox sent complete. msg: %s\n", sSigfoxData.clastsentmsg );
                                    
                			        break;

                                }
                            }
                			else
                			{
                            	printf( "Another event occurs: 0x%x\n", fds[0].revents );
                			}
                        }
                    }
                }
                
                continue;
            }
        }

        // Send the command to Sigfox module
        WriteData( pcCmdIn, strlen(pcCmdIn)+1 );

        printf( "\nWaiting for Sigfox response...\n\n" );

        // Wait and read the response from Sigfox module
        while( pcResp[i-1] != '\n' )
        {
            wSizeRead = ReadData( &nData, 1 );

            if( wSizeRead > 0 )
            {
                pcResp[i++] = nData;
            }
            else
            {
                //printf("ERROR: ReadData fail.\n" );
                px4_usleep(100000);  // sleep for not blocking the process
            }
        }

        pcResp[i-1] = 0x00;

        printf( "%s\n\n", pcResp );

        i = 0;
        memset( pcResp, 0, sizeof(pcResp) );
    }

    ClosePort();


    printf("\nGoodbye Sigfox Command Mode...\n\n");

#if 0
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
            printf( "No data time-out\n" );
        }
        else if( poll_ret < 0 )
        {
            /* this is seriously bad - should be an emergency */
            if( error_counter < 10 || error_counter % 50 == 0 ) 
            {
                /* use a counter to prevent flooding (and slowing us down) */
                printf( "ERROR return value from poll(): %d\n", poll_ret );
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

                printf("Accelerometer:\t%8.4f\t%8.4f\t%8.4f\n",
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

    printf("exiting\n");
#endif

    return OK;
}

bool OpenPort( const char* pcPortName )
{
    //printf("OPEN PORT\n");

    sigfox_fd = open( pcPortName, O_RDWR | O_NOCTTY | O_NDELAY );

    if ( sigfox_fd == -1 )
    {
        printf( "ERROR: Could not open port. (errno:%d %s)\n", errno, strerror( errno ) );
        return false;
    }

    // Check file descriptor
    if( !isatty( sigfox_fd ) )
    {
        printf( "ERROR: File descriptor %d is NOT a serial port\n", sigfox_fd );
        ClosePort();
        return false;
    }

    //fcntl( sigfox_fd, F_SETFL, 0 );

    return true;
}

bool ClosePort( void )
{
    //printf("CLOSE PORT\n");

    int32_t iResult = close( sigfox_fd );

    if ( iResult < 0 )
    {
        printf("ERROR: Could not close port. (errno:%d %s)\n", errno, strerror( errno ) );
        return false;
    }

    return true;
}

bool SetupUart( int iBaudRate,
                int iDataBits,
                int iStopBits,
                bool bParity,
                bool bHardwareControl )
{
    // Read file descritor configuration
    struct termios  config;

    if( tcgetattr( sigfox_fd, &config ) < 0 )
    {
        printf("ERROR: Could not read configuration. (errno:%d %s)\n", errno, strerror( errno ) );
        return false;
    }

    // Input flags - Turn off input processing
    // convert break to null byte, no CR to NL translation,
    // no NL to CR translation, don't mark parity errors or breaks
    // no input parity check, don't strip high bit off,
    // no XON/XOFF software flow control
    config.c_iflag &= ~( IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON );

    // Output flags - Turn off output processing
    // no CR to NL translation, no NL to CR-NL translation,
    // no NL to CR translation, no column 0 CR suppression,
    // no Ctrl-D suppression, no fill characters, no case mapping,
    // no local output processing
    #if 0
    config.c_oflag &= ~( OCRNL | ONLCR | ONLRET | ONOCR | OFILL | OPOST );

    #ifdef OLCUC
        config.c_oflag &= ~OLCUC;
    #endif

    #ifdef ONOEOT
        config.c_oflag &= ~ONOEOT;
    #endif
    #else
    config.c_oflag = 0;
    #endif

    // local modes
    // No line processing:
    // echo off, echo newline off, canonical mode off,
    // extended input processing off, signal chars off
    config.c_lflag &= ~( ECHO | ECHONL | ICANON | IEXTEN | ISIG );

    // control modes
    // one stop bit, no parity checking, disable flow control
    config.c_cflag &= ~( CSTOPB | PARENB | CRTSCTS );
    // force 8 bit input
    config.c_cflag |= CS8;

    // One input byte is enough to return from read()
    // Inter-character timer off
    config.c_cc[VMIN]  = 1;
    config.c_cc[VTIME] = 10; // was 0

    // Get the current options for the port
    ////struct termios options;
    ////tcgetattr(sigfox_fd, &options);

    // Apply baudrate
    switch( iBaudRate )
    {
        case 1200:
            if( cfsetispeed( &config, B1200 ) < 0 || cfsetospeed( &config, B1200 ) < 0 )
            {
                printf( "ERROR: Could not set desired baud rate of %d Baud\n", iBaudRate );
                return false;
            }
            break;
        case 1800:
            cfsetispeed( &config, B1800 );
            cfsetospeed( &config, B1800 );
            break;
        case 9600:
            if( cfsetispeed( &config, B9600 ) < 0 || cfsetospeed( &config, B9600 ) < 0 )
            {
                printf( "ERROR: Could not set desired baud rate of %d Baud\n", iBaudRate );
                return false;
            }
            break;
        case 19200:
            cfsetispeed( &config, B19200 );
            cfsetospeed( &config, B19200 );
            break;
        case 38400:
            if( cfsetispeed( &config, B38400 ) < 0 || cfsetospeed( &config, B38400 ) < 0 )
            {
                printf( "ERROR: Could not set desired baud rate of %d Baud\n", iBaudRate );
                return false;
            }
            break;
        case 57600:
            if( cfsetispeed( &config, B57600 ) < 0 || cfsetospeed( &config, B57600 ) < 0 )
            {
                printf( "ERROR: Could not set desired baud rate of %d Baud\n", iBaudRate );
                return false;
            }
            break;
        case 115200:
            if( cfsetispeed( &config, B115200 ) < 0 || cfsetospeed( &config, B115200 ) < 0 )
            {
                printf( "ERROR: Could not set desired baud rate of %d Baud\n", iBaudRate );
                return false;
            }
            break;

        // These two non-standard (by the 70'ties ) rates are fully supported on
        // current Debian and Mac OS versions (tested since 2010).
        case 460800:
            if( cfsetispeed( &config, B460800 ) < 0 || cfsetospeed( &config, B460800 ) < 0 )
            {
                printf( "ERROR: Could not set desired baud rate of %d Baud\n", iBaudRate );
                return false;
            }
            break;
        case 921600:
            if( cfsetispeed( &config, B921600 ) < 0 || cfsetospeed( &config, B921600 ) < 0 )
            {
                printf( "ERROR: Could not set desired baud rate of %d Baud\n", iBaudRate );
                return false;
            }
            break;
        default:
            printf( "ERROR: Could not set desired baud rate of %d Baud\n", iBaudRate );
            return false;
    }

    // Finally, apply the configuration
    if( tcsetattr( sigfox_fd, TCSANOW, &config ) < 0 )
    {
        printf("ERROR: Could not set configuration. (errno:%d %s)\n", errno, strerror( errno ) );
        return false;
    }

    // Done!
    return true;
}

int16_t ReadData(   uint8_t* punData,
                    uint16_t uwSize )
{
    int16_t wSizeRead = 0;

    // Lock
    //pthread_mutex_lock( &m_sReadMutex );

    wSizeRead = read( sigfox_fd, punData, uwSize );

    if( wSizeRead == -1 )
    {
        //printf("ERROR: Read fail. (errno:%d %s)\n", errno, strerror( errno ) );
    }

    // Unlock
    //pthread_mutex_unlock( &m_sReadMutex );

    return wSizeRead;
}

bool WriteData( const char* punData,
                uint16_t uwSize )
{
    int16_t wBytesWritten = 0;

    // Lock
    //pthread_mutex_lock( &m_sWriteMutex );

    // Write packet via serial link
    wBytesWritten = write( sigfox_fd, punData, uwSize );
    //char TMP = 0x30;
    //write( sigfox_fd, &TMP, 1 );

    if( wBytesWritten == -1 )
    {
        printf("ERROR: Write fail. (errno:%d %s)\n", errno, strerror( errno ) );
    }
    else
    {
        // Wait until all data has been written
        tcdrain( sigfox_fd );

        //printf( "write %d bytes.\n", wBytesWritten );

        //printf( "%s\n", punData );
    }

    // Unlock
    //pthread_mutex_unlock( &m_sWriteMutex );


    return ( wBytesWritten == -1 )? false: true;
}