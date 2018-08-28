#ifndef BSERIALPORT_H
#ifndef LINUX

#include "Windows.h"

enum BHBaud {
	wxBAUD_600 = 600,
	wxBAUD_1200 = 1200,
	wxBAUD_2400 = 2400,
	wxBAUD_4800 = 4800,
	wxBAUD_9600 = 9600,
	wxBAUD_19200 = 19200,
	wxBAUD_38400 = 38400
	};

#define wxTIMEOUT_INFINITY 0xffffffff

class BSerialPort
{
public:
	BSerialPort();
	~BSerialPort();

	int Open(const char *devname);
	int Close();
	int SetBaudrate(BHBaud baud);

	int Read(char *buf, int len); // never blocks, returns 0 or the number of bytes read
	int Readv(char *buf, int len, unsigned int timeout_in_ms);

	//int Write(const char *buf, int len);
	int Writev(const char *buf, int len, unsigned int timeout_in_ms);

protected:
	HANDLE hSerial;
};

#endif // LINUX
#endif // BSERIALPORT_H
