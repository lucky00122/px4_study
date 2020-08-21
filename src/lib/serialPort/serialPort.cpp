#include <px4_platform_common/log.h>

#include "serialPort.h"
#include <string.h>


// ====================================
// Public Functions
// ====================================
/*
 * Function name: SerialPort
 * Purpose: Constructor of SerialPort class
 *
 * Return: N/A
 * Param: N/A
 */
serialPort::serialPort( const char *pcUartName ,
							int iBaudRate ):
	m_fdPort( -1 ),
	m_iBaudRate( iBaudRate )
{
	if( pcUartName )
	{
		strcpy( m_pcUartName, pcUartName );
	}
	else
	{
		memset( m_pcUartName, 0, sizeof( m_pcUartName ) );
	}
}

/*
 * Function name: ~SerialPort
 * Purpose: Constructor of SerialPort class
 *
 * Return: N/A
 * Param: N/A
 */
serialPort::~serialPort()
{
	InitSerialPort( FALSE );
}

/*
 * Function name: InitSerialPort
 * Purpose: Initialize the serial port
 *
 * Return: N/A
 * Param:
 *		bool bInit:
 *			TRUE: Initialize
 *			FALSE: Denitialize
 */
bool serialPort::InitSerialPort( bool bInit )
{
	bool bRet = TRUE;

	if( bInit )
	{
		// Start mutex
		if ( pthread_mutex_init( &m_sReadMutex,
								 reinterpret_cast< const pthread_mutexattr_t * >( NULL ) ) )
		{
			PX4_ERR( "SerialPort Read Mutex init failed" );
		}

		if ( pthread_mutex_init( &m_sWriteMutex,
								 reinterpret_cast< const pthread_mutexattr_t * >( NULL ) ) )
		{
			PX4_ERR( "SerialPort Write Mutex init failed" );
		}

		bRet = OpenSerialPort( m_pcUartName,
							   m_iBaudRate );
	}
	else
	{
		bRet = CloseSerialPort();

		// destroy mutex
		pthread_mutex_destroy( &m_sReadMutex );
		pthread_mutex_destroy( &m_sWriteMutex );
	}

	return bRet;
}

int16_t serialPort::ReadData( uint8_t* punData,
							  uint16_t uwSize )
{
	int16_t wSizeRead = 0;

	// Lock
	pthread_mutex_lock( &m_sReadMutex );

	wSizeRead = read( m_fdPort, punData, uwSize );

	if( wSizeRead == -1 )
	{
		//APPRINT_ERR( "Read fail -> errno %d:%s\n", errno, strerror(errno) );	// errno = EPERM, ...
	}

	// Unlock
	pthread_mutex_unlock( &m_sReadMutex );

	return wSizeRead;
}

bool serialPort::WriteData( uint8_t* punData,
							  uint16_t uwSize )
{
	int16_t wBytesWritten = 0;
	// Lock
	pthread_mutex_lock( &m_sWriteMutex );

	// Write packet via serial link
	wBytesWritten = write( m_fdPort, punData, uwSize );
	//char TMP = 0x30;
	//write( m_fdPort, &TMP, 1 );

	// Wait until all data has been written
	tcdrain( m_fdPort );

	// Unlock
	pthread_mutex_unlock( &m_sWriteMutex );

	return ( wBytesWritten == -1 )? FALSE: TRUE;
}

bool serialPort::WaitWritingCompletion( void )
{	
	if( m_fdPort == -1 )
	{
		return FALSE;
	}

	if( fsync( m_fdPort ) == -1 )
	{
        PX4_ERR( "WaitWritingCompletion fail. (errno:%d %s)", errno, strerror( errno ) );
		
		return FALSE;
	}
		
	return TRUE;
}


// ====================================
// Private Functions
// ====================================
bool serialPort::OpenSerialPort( const char* pcUartName,
								   int iBaudRate )
{
	// --------------------------------------------------------------------------
	//   OPEN PORT
	// --------------------------------------------------------------------------
	//APPRINT( "OPEN PORT\n" );

	if ( !OpenPort( pcUartName ) )
	{
		PX4_ERR( "Open port NG (UART NAME: %s) -> errno %d:%s",
					 pcUartName, errno, strerror(errno) );	// errno = EPERM, ...
		return FALSE;
	}

	// --------------------------------------------------------------------------
	//   SETUP PORT
	// --------------------------------------------------------------------------
	if ( !SetupPort( iBaudRate,
					 8,
					 1,
					 false,
					 false ) )
	{
        PX4_ERR( "SetupPort fail" );

		return FALSE;
	}

	// --------------------------------------------------------------------------
	//   CONNECTED!
	// --------------------------------------------------------------------------
	//PX4_INFO( "Connected to %s with %d baud, 8 data bits, no parity, 1 stop bit (8N1)", pcUartName, iBaudRate);

	return TRUE;
}


bool serialPort::CloseSerialPort( void )
{
	int iResult = 0;

	if( m_fdPort == -1 )
	{
		return FALSE;
	}
	
	iResult = close( m_fdPort );

	m_fdPort = -1;

	if ( iResult )
	{
		PX4_ERR( "Error on port close (%i)", iResult );
		return FALSE;
	}

	return TRUE;
}

bool serialPort::OpenPort( const char* pcPortName )
{
	if( m_fdPort != -1 )
	{
        //PX4_WARN("%s port is already opened. fd:%d\n", pcPortName, m_fdPort );
		
		return TRUE;
	}

	// Open serial port
	// O_RDWR - Read and write
	// O_NOCTTY - Ignore special chars like CTRL-C
	m_fdPort = open( pcPortName, O_RDWR | O_NOCTTY | O_NDELAY );

	if ( m_fdPort == -1 )
	{
        PX4_ERR( "Could not open port. (errno:%d %s)", errno, strerror( errno ) );
		
		return FALSE;
	}

	// Check file descriptor
	if( !isatty( m_fdPort ) )
	{
		PX4_ERR( "file descriptor %d is NOT a serial port", m_fdPort );

		CloseSerialPort();
		
		return FALSE;
	}

	//fcntl( m_fdPort, F_SETFL, 0 );

	return TRUE;
}

bool serialPort::SetupPort( int iBaudRate,
							   int iDataBits,
							   int iStopBits,
							   bool bParity,
							   bool bHardwareControl )
{
	// Read file descritor configuration
	struct termios  config;
	
	if( tcgetattr( m_fdPort, &config ) < 0 )
	{
		PX4_ERR( "could not read configuration of m_fdPort %d", m_fdPort );
		return FALSE;
	}

	// Input flags - Turn off input processing
	// convert break to null byte, no CR to NL translation,
	// no NL to CR translation, don't mark parity errors or breaks
	// no input parity check, don't strip high bit off,
	// no XON/XOFF software flow control
	config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL |
						INLCR | PARMRK | INPCK | ISTRIP | IXON);

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
	////tcgetattr(m_fdPort, &options);

	// Apply baudrate
	switch( iBaudRate )
	{
		case 1200:
			if ( cfsetispeed( &config, B1200 ) < 0 || cfsetospeed( &config, B1200 ) < 0 )
			{
				PX4_ERR( "Could not set desired baud rate of %d Baud", iBaudRate );
				
				return FALSE;
			}
			break;
		case 1800:
			cfsetispeed( &config, B1800 );
			cfsetospeed( &config, B1800 );
			break;
		case 9600:
            if( cfsetispeed( &config, B9600 ) < 0 || cfsetospeed( &config, B9600 ) < 0 )
            {
                PX4_ERR( "Could not set desired baud rate of %d Baud", iBaudRate );
                return false;
            }
			break;
		case 19200:
			cfsetispeed( &config, B19200 );
			cfsetospeed( &config, B19200 );
			break;
		case 38400:
			if ( cfsetispeed( &config, B38400 ) < 0 || cfsetospeed( &config, B38400 ) < 0 )
			{
				PX4_ERR( "Could not set desired baud rate of %d Baud", iBaudRate );
				
				return FALSE;
			}
			break;
		case 57600:
			if ( cfsetispeed( &config, B57600 ) < 0 || cfsetospeed( &config, B57600 ) < 0 )
			{
				PX4_ERR( "Could not set desired baud rate of %d Baud", iBaudRate );
				
				return FALSE;
			}
			break;
		case 115200:
			if ( cfsetispeed( &config, B115200 ) < 0 || cfsetospeed( &config, B115200 ) < 0 )
			{
				PX4_ERR( "Could not set desired baud rate of %d Baud", iBaudRate );
				
				return FALSE;
			}
			break;

		// These two non-standard (by the 70'ties ) rates are fully supported on
		// current Debian and Mac OS versions (tested since 2010).
		case 460800:
			if ( cfsetispeed( &config, B460800 ) < 0 || cfsetospeed( &config, B460800 ) < 0 )
			{
				PX4_ERR( "Could not set desired baud rate of %d Baud", iBaudRate);
				
				return FALSE;
			}
			break;
		case 921600:
			if ( cfsetispeed( &config, B921600 ) < 0 || cfsetospeed( &config, B921600 ) < 0 )
			{
				PX4_ERR( "Could not set desired baud rate of %d Baud", iBaudRate );
				
				return FALSE;
			}
			break;
		default:
			PX4_ERR( "Desired baud rate %d could not be set, aborting.", iBaudRate );
			
			return FALSE;
	}

	// Finally, apply the configuration
	if( tcsetattr( m_fdPort, TCSANOW, &config ) < 0 )
	{
        PX4_ERR( "Could not set configuration. (errno:%d %s)", errno, strerror( errno ) );
		
		return FALSE;
	}

	// Done!
	return TRUE;
}





