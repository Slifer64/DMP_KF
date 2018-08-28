#include "BSerialPort.h"

#include <stdio.h>

#ifndef LINUX


BSerialPort::BSerialPort() :
	hSerial(NULL)
{
}

BSerialPort::~BSerialPort()
{
	Close();
}

int BSerialPort::Open(const char *devname)
{
	printf("BSerialPort::Open(%s)\n", devname);

	hSerial = CreateFile(devname, // //"COM1"
		GENERIC_READ | GENERIC_WRITE,
		0,
		0,
		OPEN_EXISTING,
		FILE_ATTRIBUTE_NORMAL,
		0);
	if (hSerial==INVALID_HANDLE_VALUE)
	{
		if (GetLastError()==ERROR_FILE_NOT_FOUND)
		{
			//serial port does not exist. Inform user.
			printf("Serial port does not exist\n");
			return -1;
		}
		//some other error occurred. Inform user.
		printf("Open serial port error\n");
		return -1;
	}

	COMMTIMEOUTS timeouts = {0};
	timeouts.ReadIntervalTimeout = MAXDWORD;
	timeouts.ReadTotalTimeoutConstant = 0;
	timeouts.ReadTotalTimeoutMultiplier = 0;
	timeouts.WriteTotalTimeoutConstant = 0;
	timeouts.WriteTotalTimeoutMultiplier = 0;

	if (!SetCommTimeouts(hSerial, &timeouts))
	{
		//error occurred. Inform user
		printf("SetCommTimeouts Error\n");
		return -1;
	}

	// return -1 if an error occurred or 0 on success
	return 0;
}

int BSerialPort::Close()
{
	printf("BSerialPort::Close()\n");
	if (hSerial != NULL)
	{
		CloseHandle(hSerial);
		hSerial = NULL;
	}
	else
		return -1;

	// return -1 if an error occurred or 0 on success
	return 0;
}

int BSerialPort::SetBaudrate(BHBaud baud)
{
	printf("BSerialPort::SetBaudrate(%d)\n", baud);

	DCB dcbSerialParams = {0};
	dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
	if (!GetCommState(hSerial, &dcbSerialParams))
	{
		// error getting state
		printf("SetBaudrate GetCommState Error\n");
		return -1;
	}
	switch (baud)
	{
		case wxBAUD_600:   { dcbSerialParams.BaudRate = CBR_600;   break; }
		case wxBAUD_1200:  { dcbSerialParams.BaudRate = CBR_1200;  break; }
		case wxBAUD_2400:  { dcbSerialParams.BaudRate = CBR_2400;  break; }
		case wxBAUD_4800:  { dcbSerialParams.BaudRate = CBR_4800;  break; }
		case wxBAUD_9600:  { dcbSerialParams.BaudRate = CBR_9600;  break; }
		case wxBAUD_19200: { dcbSerialParams.BaudRate = CBR_19200; break; }
		case wxBAUD_38400: { dcbSerialParams.BaudRate = CBR_38400; break; }
		default:
		{
		}
	}
	dcbSerialParams.ByteSize=8;
	dcbSerialParams.StopBits=ONESTOPBIT;
	dcbSerialParams.Parity=NOPARITY;
	if (!SetCommState(hSerial, &dcbSerialParams))
	{
		//error setting serial port state
		printf("SetBaudrate SetCommState Error\n");
		return -1;
	}

	// return -1 if an error occurred or 0 on success
	return 0;
}

int BSerialPort::Read(char *buf, int len)
{
	//printf("BSerialPort::Read(%d)\n", len);
	//printf("R ");

	DWORD dwBytesRead = 0;
	if (!ReadFile(hSerial, buf, len, &dwBytesRead, NULL))
	{
		//error occurred. Report to user.
		//printf("Error");
		return 0;
	}
	return dwBytesRead; // never blocks, returns 0 or the number of bytes read
}

int BSerialPort::Readv(char *buf, int len, unsigned int timeout_in_ms)
{
	//printf("BSerialPort::Readv(%d)\n", len);
	//printf("Rv ");

	int n = 0;

	unsigned int startTicks = GetTickCount();

	while (1)
	{
		DWORD dwBytesRead = 0;

		{

			if (!ReadFile(hSerial, buf, len - n, &dwBytesRead, NULL))
			{
				//error occurred. Report to user.
				printf("ReadFile error %d ", (unsigned int)dwBytesRead);
				//Sleep(1);
			}
			else
			{
				n += dwBytesRead;
				buf += dwBytesRead;

				if (n == len)
					return n;
				//else
				//	printf("polling %d of %d ", n, len);
				//SleepEx(0);
				Sleep(1); // works sortof




				/*if(!SetCommMask(hSerial, EV_RXCHAR))
				{
					//Handle Error Condition
				}*/


				/*DWORD dwEventMask;
				if (WaitCommEvent(hSerial, &dwEventMask, NULL))
				{
				}

				unsigned int dt = GetTickCount() - startTicks;
				if (dt >= timeout_in_ms)
				{
					printf("Readv long delay %d of %d dt = %d\n", n, len, dt);
				}*/

			}
		}

		unsigned int dt = GetTickCount() - startTicks;
		if (timeout_in_ms != wxTIMEOUT_INFINITY &&  dt >= timeout_in_ms)
		{
			printf("Readv timeout %d of %d\n", n, len);
			return n;
		}
	}
}

int BSerialPort::Writev(const char *buf, int len, unsigned int timeout_in_ms)
{
	//printf("BSerialPort::Writev(%d)\n", len);
	//printf("Wv ");

	int n = 0;

	unsigned int startTicks = GetTickCount();

	while (1)
	{
		DWORD dwBytesWritten = 0;

		if (!WriteFile(hSerial, buf, len - n, &dwBytesWritten, NULL))
		{
			//error occurred. Report to user.
			printf("WriteFile error %d", (unsigned int)dwBytesWritten);

		}
		else
		{
			n += dwBytesWritten;
			buf += dwBytesWritten;

			if (n == len)
				return n;
		}

		unsigned int dt = GetTickCount() - startTicks;
		if (timeout_in_ms != wxTIMEOUT_INFINITY &&  dt >= timeout_in_ms)
			return n;

	}
}

#endif // LINUX
