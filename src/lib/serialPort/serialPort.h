#ifndef SERIALPORT_H_
#define SERIALPORT_H_

#include <cstdlib>
#include <stdio.h>   // Standard input/output definitions
#include <unistd.h>  // UNIX standard function definitions
#include <fcntl.h>   // File control definitions
#include <termios.h> // POSIX terminal control definitions
#include <pthread.h> // This uses POSIX Threads

// ====================================
// Definitions
// ====================================
#ifdef ESP8266_UART
	#define SERIALPORT_UART_NAME		"/dev/ttyACM0"
#else
	#ifdef __APPLE__
		#define SERIALPORT_UART_NAME		"/dev/tty.usbserial-DN00BWPM"
	#else
		#define SERIALPORT_UART_NAME		"/dev/ttyUSB0"
	#endif
#endif

#ifdef ESP8266_UART
	#define SERIALPORT_UART_BAUDRATE	115200
#else
	#define SERIALPORT_UART_BAUDRATE	57600
#endif

// The following two non-standard baudrates should have been defined by the system
// If not, just fallback to number
#ifndef B460800
#define B460800 460800
#endif

#ifndef B921600
#define B921600 921600
#endif


// ====================================
// Class Declaration
// ====================================
class serialPort
{
public:
    serialPort( const char* pcUartName = SERIALPORT_UART_NAME,
                int iBaudRate = SERIALPORT_UART_BAUDRATE );
	~serialPort();

	bool InitSerialPort( bool bInit );
	int16_t ReadData( uint8_t* punData,
                       uint16_t uwCount );
	bool WriteData( uint8_t* punData,
					uint16_t uwSize );

    bool WaitWritingCompletion( void );

private:
	bool OpenSerialPort( const char* pcUartName,
						 int iBaudRate );
	bool CloseSerialPort( void );
	bool OpenPort( const char* pcPortName );
	bool SetupPort( int iBaudRate,
					int iDataBits,
					int iStopBits,
					bool bParity,
					bool bHardwareControl );

	int m_fdPort;
	char m_pcUartName[32];
	int m_iBaudRate;
	pthread_mutex_t m_sReadMutex;
	pthread_mutex_t m_sWriteMutex;
};

#endif // SERIALPORT_H_


