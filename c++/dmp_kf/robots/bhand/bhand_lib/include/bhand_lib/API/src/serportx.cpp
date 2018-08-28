#include "ctb-0.14/serportx.h"

bool wxSerialPort_x::IsStandardRate( int rate )
{
    const int rates[] = {
	   150, 300, 600, 1200, 2400, 4800, 9600, 19200, 38400, 57600,
	   115200, 230400, 460800, 921600
    };

    for( unsigned int i = 0; i < ( sizeof( rates ) / sizeof( int ) ); i++ ) {

	   if( rate == rates[ i ] ) {

		  return true;

	   }
    }
    return false;
}
