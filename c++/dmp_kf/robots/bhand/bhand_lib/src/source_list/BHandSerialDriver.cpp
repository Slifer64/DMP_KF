///////////////////////////////////////////////////////////////////////////////
//                   (C) Barrett Technology Inc. 2009-2010                   //
///////////////////////////////////////////////////////////////////////////////

#include "BHandSerialDriver.h"
#include "BHandSupervisoryRealTime.h"
#include "BHandHardware.h"

#include "bhand_motors.h"
#include "bhand_misc.h"

#include "BHand.h"

// CTB include for serial port
#include "ctb-0.14/ctb.h"

#include <math.h>


// Create a table of for platform dependent comport names for serial port
//    (e.g. "COM1" for windows or "/dev/ttyS0" for linux) from POCO
const static char comPortAsString[9][50] =
	{ wxCOM1, wxCOM2, wxCOM3, wxCOM4, wxCOM5, wxCOM6, wxCOM7, wxCOM8, wxCOM9 };

const static char usbPortAsString[10][50] =
	{ "/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyUSB2", "/dev/ttyUSB3", "/dev/ttyUSB4",
	  "/dev/ttyUSB5", "/dev/ttyUSB6", "/dev/ttyUSB7", "/dev/ttyUSB8", "/dev/ttyUSB9"};

int BHandSerialDriver::defBaud = 9600;

// Global Variable for access to pointers of BHand instances
BHand* _BHandArray[BH_MAXPORT] = { NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL };



void BHandSerialDriver::SetWaitCallbackFunc(BHCallback waitCallbackFunc)
{
	// Do nothing
}



///////////////////////////////////////////////////////////////////////////
// CAN BHandSerial Constructor/Deconstructor
///////////////////////////////////////////////////////////////////////////

BHandSerialDriver::BHandSerialDriver(BHand *bhand, BHandSupervisoryRealtime *moduleSuperReal, unsigned int comport) :
	BHandDriver(bhand, moduleSuperReal, BH_SERIAL_COMMUNICATION),
	dev(NULL) //, /*comPort(comport),*/ // usb(false)
{
	comBaud = defBaud; // assign the default baud rate
	comPort = comport;
/*	if (comPort <= 0)
	{
		comPort = -comPort;
		usb = true;
	}*/
}

BHandSerialDriver::~BHandSerialDriver()
{
}




///////////////////////////////////////////////////////////////////////////
// BarrettHand Driver Methods
///////////////////////////////////////////////////////////////////////////

int BHandSerialDriver::Initialize()
{
	if (m_open)
		return BHERR_OPENCOMMPORT;

	// Open Serial interface with the BarrettHand using comPort passed in constructor
	int result;
	if ((result = ComInitialize(comPort)))
		return result;

	// Driver is up and running
	m_open = true;

	//printf("BHandSerialDriver::Initialize() m_open = true\n");

	return 0;
}

void BHandSerialDriver::Close()
{
	ComClose();
	m_open = false;
}


///////////////////////////////////////////////////////////////////////////////
// Supervisory Module Methods
///////////////////////////////////////////////////////////////////////////////

int BHandSerialDriver::HandInit(BHMotors bhMotors)
{
	// prepare output char buffer
	char motor[5];
	toMotorChar(bhMotors, motor);
	sprintf(outbuf, "%sHI\r", motor);
	nout = strlen(outbuf);

	return HandleSupervisoryCall();
}
int BHandSerialDriver::HandReset(BHMotors bhMotors, bool *responseReceived)
{
	//printf("    sending RESET - clear errrors command \n");
	nout = 1;
	outbuf[0] = '\r';

	// exchange characters
	int result = HandleSupervisoryCall();

	//printf("    sending RESET - clear errrors command %d\n", result);

	if (result != 0)
		return result;

	//printf("    sending RESET command\n");

	// prepare output char buffer, reset baud rate
	sprintf(outbuf, "RESET\r");
	nout = strlen(outbuf);
	requestBaud = defBaud;

	// exchange characters
	result = HandleSupervisoryCall();

	//printf("    sending RESET command result = %d, *responseReceived = %d inbuf = %s\n", result, *responseReceived, inbuf);

	// Check to see if hand displays part of the banner after a RESET command
	if (responseReceived)
		*responseReceived = searchstrn(inbuf, "BarrettHand Firmware\n", nin);

	return result;
}

char * BHandSerialDriver::HandCommand(const char *send, int *errorCode)
{
	// assign input, check size, add \r
	if ((nout = strlen(send) + 1) >= BH_MAXCHAR)
	{
		*errorCode = BHERR_LONGSTRING;
		return NULL;
	}
	strcpy(outbuf, send);
	strcat(outbuf, "\r");

	// exchange characters
	*errorCode = HandleSupervisoryCall();

	// Return pointer to serial input buffer
	if (*errorCode)
		inbuf[0] = 0;

	return inbuf;
}

int BHandSerialDriver::HandClose(BHMotors bhMotors)
{
	// prepare output char buffer
	char motor[5];
	toMotorChar(bhMotors, motor);
	sprintf(outbuf, "%sC\r", motor);
	nout = strlen(outbuf);

	// exchange characters
	return HandleSupervisoryCall();
}
int BHandSerialDriver::HandOpen(BHMotors bhMotors)
{
	// prepare output char buffer
	char motor[5];
	toMotorChar(bhMotors, motor);
	sprintf(outbuf, "%sO\r", motor);
	nout = strlen(outbuf);

	// exchange characters
	return HandleSupervisoryCall();
}

int BHandSerialDriver::HandGoToDefault(BHMotors bhMotors, bool valueIncluded, int defaultPosition)
{
	// prepare output char buffer
	char motor[5];
	toMotorChar(bhMotors, motor);
	if (valueIncluded)
		sprintf(outbuf, "%sM %d\r", motor, defaultPosition);
	else
		sprintf(outbuf, "%sM\r", motor);
	nout = strlen(outbuf);

	// exchange characters
	return HandleSupervisoryCall();
}

int BHandSerialDriver::HandGoToDifferentPositionsHand(const int *encoderPositions, unsigned int numEncoderPositions)
{
	if (numEncoderPositions != 4)
		return BHERR_OUTOFRANGE;

	// set all default positions
	int result;
	for (unsigned int i = 0; i < numEncoderPositions; i++)
		if ((result = HandSet(1 << i, "DP", encoderPositions[i])))
			return result;

	return HandGoToDefault(15);
}

int BHandSerialDriver::HandGoToHome(BHMotors bhMotors)
{
	// prepare output char buffer
	char motor[5];
	toMotorChar(bhMotors, motor);

	int result;

	if (ContainsAnyFingers(motor))
	{
		sprintf(outbuf, "GHOME\r");
		nout = strlen(outbuf);

		// exchange characters
		result = HandleSupervisoryCall();

		if (result || !ContainsSpread(motor))
			return result;
	}

	// GoToHome command contains spread
	// prepare output char buffer
	sprintf(outbuf, "SHOME\r");
	nout = strlen(outbuf);

	// exchange characters
	return HandleSupervisoryCall();
}

int BHandSerialDriver::HandGoToPosition(BHMotors bhMotors, unsigned int encoderPositionTickCount)
{
	// prepare output char buffer
	char motor[5];
	toMotorChar(bhMotors, motor);

	// prepare output char buffer
	sprintf(outbuf, "%sM %d\r", motor, encoderPositionTickCount);
	nout = strlen(outbuf);

	// exchange characters
	return HandleSupervisoryCall();
}

int BHandSerialDriver::HandStepClose(BHMotors bhMotors, bool valueIncluded, int stepAmount)
{
	// prepare output char buffer
	char motor[5];
	toMotorChar(bhMotors, motor);
	if (valueIncluded)
		sprintf(outbuf, "%sIC %d\r", motor, stepAmount);
	else
		sprintf(outbuf, "%sIC\r", motor);
	nout = strlen(outbuf);

	// exchange characters
	return HandleSupervisoryCall();
}

int BHandSerialDriver::HandStepOpen(BHMotors bhMotors, bool valueIncluded, int stepAmount)
{
	// prepare output char buffer
	char motor[5];
	toMotorChar(bhMotors, motor);
	if (valueIncluded)
		sprintf(outbuf, "%sIO %d\r", motor, stepAmount);
	else
		sprintf(outbuf, "%sIO\r", motor);
	nout = strlen(outbuf);

	// exchange characters
	return HandleSupervisoryCall();
}

int BHandSerialDriver::HandStopMotor(BHMotors bhMotors)
{
	// prepare output char buffer
	char motor[5];
	toMotorChar(bhMotors, motor);
	sprintf(outbuf, "%sT\r", motor);
	nout = strlen(outbuf);

	// exchange characters
	return HandleSupervisoryCall();
}

int BHandSerialDriver::HandTorqueClose(BHMotors bhMotors)
{
	// prepare output char buffer
	char motor[5];
	toMotorChar(bhMotors, motor);
	sprintf(outbuf, "%sTC\r", motor);
	nout = strlen(outbuf);

	// exchange characters
	return HandleSupervisoryCall();
}

int BHandSerialDriver::HandTorqueOpen(BHMotors bhMotors)
{
	// prepare output char buffer
	char motor[5];
	toMotorChar(bhMotors, motor);
	sprintf(outbuf, "%sTO\r", motor);
	nout = strlen(outbuf);

	// exchange characters
	return HandleSupervisoryCall();
}

int BHandSerialDriver::HandGet(BHMotors bhMotors, const char *property, int *propertyResult, int *nresults)
{
	// prepare output char buffer
	char motor[5];
	toMotorChar(bhMotors, motor);
	sprintf(outbuf, "%sFGET %s\r", motor, property);
	nout = strlen(outbuf);

	// exchange characters
	int ret = HandleSupervisoryCall();
	if (ret)
		return ret;

	// read parameters - as many as present
	int sz = strlen(Response()), pos = 0, cnt = 0;

	// look for an error from hand
	if (sz >= 3 && Response()[0] == 'E' && Response()[1] == 'R' && Response()[2] == 'R')
		return BHERR_BADRESPONSE;

	while (pos < sz)
	{
		// skip white space
		while (isspacechar(Response()[pos]) && pos < sz)
			pos++;

		// read field
		if (pos < sz)
			sscanf(Response()+pos, "%d", propertyResult+cnt++);

		// skip digits
		while ((isdigit(Response()[pos]) || Response()[pos] == '-') && pos < sz)
			pos++;
	}

	*nresults = cnt;

	return 0;
}

int BHandSerialDriver::HandSet(BHMotors bhMotors, const char *property, int value)
{
	// prepare output char buffer
	char motor[5];
	toMotorChar(bhMotors, motor);
	sprintf(outbuf, "%sFSET %s %d\r", motor, property, value);
	nout = strlen(outbuf);

	// exchange characters
	return HandleSupervisoryCall();
}

int BHandSerialDriver::HandPGet(const char *property, int *propertyResult)
{
	// TODO: Handle multiple get property?

	// prepare output char buffer
	sprintf(outbuf, "PGET %s\r", property);
	nout = strlen(outbuf);

	// exchange characters
	int ret = HandleSupervisoryCall();
	if (ret)
		return ret;

	// read parameters - as many as present
	int sz = strlen(Response()), pos = 0, cnt = 0;

	// look for an error from hand
	if (sz >= 3 && Response()[0] == 'E' && Response()[1] == 'R' && Response()[2] == 'R')
		return BHERR_BADRESPONSE;

	while (pos < sz)
	{
		// skip white space
		while (isspacechar(Response()[pos]) && pos < sz)
			pos++;

		// read field
		if (pos < sz)
			sscanf(Response() + pos, "%d", propertyResult + cnt++);

		// skip digits
		while ((isdigit(Response()[pos]) || Response()[pos] == '-') && pos < sz)
			pos++;
	}

	return 0;
}

int BHandSerialDriver::HandPSet(const char *property, int value)
{
	// prepare output char buffer
	sprintf(outbuf, "PSET %s %d\r", property, value);
	nout = strlen(outbuf);

	// exchange characters
	return HandleSupervisoryCall();
}

int BHandSerialDriver::HandDefault(BHMotors bhMotors)
{
	// prepare output char buffer
	char motor[5];
	toMotorChar(bhMotors, motor);
	sprintf(outbuf, "%sFDEF\r", motor);
	nout = strlen(outbuf);

	// exchange characters
	return HandleSupervisoryCall();
}

int BHandSerialDriver::HandLoad(BHMotors bhMotors)
{
	// prepare output char buffer
	char motor[5];
	toMotorChar(bhMotors, motor);
	sprintf(outbuf, "%sFLOAD\r", motor);
	nout = strlen(outbuf);

	// exchange characters
	return HandleSupervisoryCall();
}

int BHandSerialDriver::HandSave(BHMotors bhMotors)
{
	// prepare output char buffer
	char motor[5];
	toMotorChar(bhMotors, motor);
	sprintf(outbuf, "%sFSAVE\r", motor);
	nout = strlen(outbuf);

	// exchange characters
	return HandleSupervisoryCall();
}

int BHandSerialDriver::HandTemperature(BHMotors bhMotors, int *temperature, unsigned int numTemperatures)
{
	// prepare output char buffer
	char motor[5];
	toMotorChar(bhMotors, motor);
	sprintf(outbuf, "PGET TEMP\r");
	nout = strlen(outbuf);

	// exchange characters
	int ret = HandleSupervisoryCall();

	sscanf(inbuf, "%d", temperature);
	*temperature = *temperature / 10;

	return ret;
}

int BHandSerialDriver::HandDelay(unsigned int msec)
{
	// prepare output char buffer
	sprintf(outbuf, "DELAY %u\r", msec);
	nout = strlen(outbuf);

	// exchange characters
	return HandleSupervisoryCall();
}

int BHandSerialDriver::HandBaud(unsigned int newbaud)
{
	// prepare output char buffer, set new baudrate
	sprintf(outbuf, "PSET BAUD %u\r", newbaud / 100);
	nout = strlen(outbuf);
	requestBaud = newbaud;

	// exchange characters
	return HandleSupervisoryCall();
}

int BHandSerialDriver::HandleSupervisoryCall()
{
	char dummy[BH_MAXCHAR];     // used for clearing echo characters
	int result;

	// NULL terminate the output buffer
	outbuf[nout] = 0;

	// check for delay
	sscanf(outbuf, "%s", dummy);
	if (!strcmp(dummy, "DELAY"))
	{
		// read delay value
		unsigned int msec;
		sscanf(outbuf+6, "%u", &msec);
		if (msec < 1)
			msec = 1;
		else if (msec > 100000)
			msec = 100000;

		DELAY(msec);

		inbuf[0] = 0;

		return 0;
	}

	// clear buffers - eliminate echo chars
	if (!ComClear(true))
		return BHERR_CLEARBUFFER;

	// send data
	if (ComWrite(outbuf, nout))
		return BHERR_WRITECOM;

	// clear echo chracters - except the last one (CR or *)
	if (nout > 1)
	{
		if (ComRead(dummy, nout - 1))
			return BHERR_READCOM;
	}

	// prepare to receive response and wait for correct ending
	nin = 0;
	inbuf[0] = 0;

	// set new baud rate if necessary
	if (requestBaud)
	{
		if ((result = ComSetBaudrate(requestBaud)) != 0)
			return result;


		if (!strncmp(outbuf, "PSET BAUD", 9))
		{
			DELAY(1000);

			if (!ComClear())
			{
				//printf("\nCOM CLEAR ERROR\n");
				return BHERR_CLEARBUFFER;
			}
			if (ComWrite("\r", 1))
				return BHERR_WRITECOM;
		}

		requestBaud = 0;
	}

	// Check that LOOP command is acknowledged with the '*' character
	if (!strcmp("LOOP\r", outbuf + MAX(0, nout - 5)))
	{
		if (ComRead(inbuf, 1))
			return BHERR_READCOM;

		if (inbuf[0] != '*')
		{
			inbuf[0] = 0;
			nin = 0;
		}
		else
			return 0;
	}


	return Interactive();
}

int BHandSerialDriver::Interactive()
{
	int result;

_INTERACTIVE:

	if (ComRead(inbuf + nin, 1))
		return BHERR_READCOM;

	// remove leading white space
	if (nin == 0 && isspacechar(inbuf[0]))
		nin--;

	// terminate string
	inbuf[++(nin)] = 0;

	// ending not yet correct - read more chars
	if (nin < BH_MAXCHAR - 1 && strcmp("=> ", (inbuf + MAX(nin - 3, 0))))
		goto _INTERACTIVE;


	// check for buffer overflow
	if (nin >= BH_MAXCHAR - 1)
		return BHERR_BADRESPONSE;

	// remove end marker and trailing white space
	nin -= 3;
	inbuf[nin] = 0;
	while (nin > 0 && isspacechar(inbuf[nin - 1]))
		inbuf[--nin] = 0;

	// check for ERR X - return as positive
	if (!strncmp(inbuf, "ERR", 3))
		sscanf(inbuf + 4, "%d", &result);
	else
		result = 0;

	return result;
}

int BHandSerialDriver::ExecuteRealtimeCall()
{
	//static int count = 0;

	// Send control block
	if (ComWrite(outbuf, nout))
		return BHERR_WRITECOM;

	// Receive feedback block
	nin = dev->Readv(&inbuf[0], nin, 10000);

	if (nin < 0)
	{
		//printf("\nNOT RECEIVING feedback block\n");
		return BHERR_READCOM;
	}

	if (nin > 0 && inbuf[0] == '*')
	{
		// ack character received
		// copy feedback block to rtIn buffer
		memcpy(rtIn, &inbuf[1], nin - 1);
		return 0;
	}
	else
	{
		// Check for errors from hand

		// A hand error has occurred
		// Wait for a second to receive anymore characters
		int numRead = dev->Readv(&inbuf[nin], 1024, 1000); // There should only be a few remaining characters some of the times

		// Return BHERR_READCOM if there is an error reading from the serial port
		if (numRead < 0)
		{
			return BHERR_READCOM;
		}
		nin += numRead; // increment number of characters in input buffer

		// Parse string (look for "\n\rERR <error code>" followed by "\n\r=> ")
		inbuf[nin] = '\0'; // NULL terminate the buffer so that it becomes a string

		// Return error code
		char *errCode = strstr(inbuf, "\n\rERR ");
		if (errCode != NULL && strstr(inbuf, "\n\r=> ") != NULL)
		{
			return atoi(errCode + 6);
		}
		else
		{
			return BHERR_READCOM;
		}
	}
}




///////////////////////////////////////////////////////////////////////////
// Serial Communication Methods
///////////////////////////////////////////////////////////////////////////

int BHandSerialDriver::ComInitialize(int comport)
{
	// comport Com port number (1 for "COM1" in Windows or "/dev/ttyS0" in Linux)
	if (comport > 0)
	{
		// Range checking
		if (comport < 1 || comport > BH_MAXPORT)
			return BHERR_PORTOUTOFRANGE;

		// Check for previous instance of BHand attached to this port
		//if (_BHandArray[comport - 1] != NULL) // TODO: put this back in
		//	return BHERR_BHANDEXISTS;
	}
	else if (comport < -BH_MAXPORT)
		return BHERR_PORTOUTOFRANGE;

	// Open serial port communication with the Barrett Hand
	if (ComOpen(comport, defBaud))
	{
		//printf("ComInitialize bad \n");
		ComClose(); // new
		return BHERR_OPENCOMMPORT;
	}

	//printf("ComInitialize ok \n");

	// Save instance in port array
	if (comPort > 0)
		_BHandArray[comPort - 1] = m_bhand;

	requestBaud = 0;

	return 0;
}

int BHandSerialDriver::ComOpen(int comport, int baudrate)
{
	//printf("ComOpen(%d, %d)\n", comport, baudrate);
	// Range checking
#ifdef LINUX
/*	if (comport <= 0)
	{
		usb = true;
		//comport = -comport;
	}
	else
		usb = false;*/
#endif
	//if ((!usb && comport < 1) || comport > BH_MAXPORT)
	if (comport < -BH_MAXPORT || comport > BH_MAXPORT)
		return BHERR_PORTOUTOFRANGE;

	// Check that comport is not already open
	if (m_open)
		return BHERR_OPENCOMMPORT;

	// Save comport - 1
	//if (!usb)
	//	comPort = comport - 1;
	comPort = comport;

	// Create and open the serial port
//#ifdef LINUX
	dev = new wxSerialPort();
//#else
//	dev = new BSerialPort();
//#endif

	// Set baud rate and number of start/stop bits before opening?

	// Open the serial port
	if ((comPort > 0 && dev->Open(comPortAsString[comPort - 1]) < 0) ||
	    (comPort <= 0 && dev->Open(usbPortAsString[-comPort]) < 0))
	{
		//printf("ComOpen(%d) failed to open %s successfully\n", comport, ((comPort <= 0) ? usbPortAsString[comPort-1] : comPortAsString[-comPort]));
		ComClose();
		return BHERR_OPENCOMMPORT;
	}

	//printf("ComOpen(%d) opened %s successfully\n", comport, ((comPort <= 0) ? usbPortAsString[-comPort] : comPortAsString[comPort - 1]));

	// Set default baud rate
	if (ComSetBaudrate(baudrate))
	{
		ComClose();
		return BHERR_SETCOMMSTATE;
	}

	int result;
	if ((result = ComSetTimeouts(50, 15000, 50, 5000)) > 0)
		return result;

	//printf("opened serial port is true 1\n");

	// Clear receive buffer
	if (!ComClear(true))
	{
		ComClose();
		return BHERR_CLEARBUFFER;
	}

	//printf("opened serial port is true 2\n");

	// Com port opened successfully and ready to use
	m_open = true;

	return 0;
}

bool BHandSerialDriver::ComIsOpen()
{
	return dev != NULL && m_open;
}

void BHandSerialDriver::ComClose()
{
	if (dev != NULL)
	{
		if (comPort > 0)
			_BHandArray[comPort - 1] = NULL;
		dev->Close();
		delete dev;
		dev = NULL;
	}

	m_open = false;
}


int BHandSerialDriver::ComSetBaudrate(int baudrate)
{
	if (dev == NULL)
		return BHERR_SETCOMMSTATE;

	switch (baudrate)
	{
		case 600:    { dev->SetBaudrate(wxBAUD_600); comBaud = 600; break; }
		case 1200:   { dev->SetBaudrate(wxBAUD_1200); comBaud = 1200; break; }
		case 2400:   { dev->SetBaudrate(wxBAUD_2400); comBaud = 2400; break; }
		case 4800:   { dev->SetBaudrate(wxBAUD_4800); comBaud = 4800; break; }
		case 9600:   { dev->SetBaudrate(wxBAUD_9600); comBaud = 9600; break; }
		case 19200:  { dev->SetBaudrate(wxBAUD_19200); comBaud = 19200; break; }
		case 38400:  { dev->SetBaudrate(wxBAUD_38400); comBaud = 38400; break; }
		default:     { return BHERR_SETCOMMSTATE; }
	}

	return 0;
}

int BHandSerialDriver::ComSetTimeouts(unsigned int readMultiplier, unsigned int readConstant, unsigned int writeMultiplier, unsigned int writeConstant)
{
	readTotalTimeoutMultiplier = readMultiplier;
	readTotalTimeoutConstant = readConstant;
	writeTotalTimeoutMultiplier = writeMultiplier;
	writeTotalTimeoutConstant = writeConstant;

	return 0;
}


bool BHandSerialDriver::ComClear(bool rxOnly)
{
	// No problem if com port is not open yet
	if (!ComIsOpen())
		return true;

	if (rxOnly)
	{
		int result;
		char temp[256];

		// Read from the receive buffer to clear it until no bytes are left to read
		while ((result = dev->Read(temp, 256)) > 0) ;

		// Return false if there was a problem with reading from the serial port
		return (result == 0) ? true : false;
	}
	else
	{
		// Both the RX and TX buffers need to be cleared
		// More expensive than just clearing RX buffer and we need to actually
		// reopen the serial port
		dev->Close();

		// Open the com port again and set the baud rate again

//	if ((!usb && dev->Open(comPortAsString[comPort]) < 0) ||
//	    ( usb && dev->Open(usbPortAsString[comPort]) < 0 ))

		bool notOpen =
			(comPort > 0 && dev->Open(comPortAsString[comPort - 1]) < 0) ||
			(comPort <= 0 && dev->Open(usbPortAsString[-comPort]) < 0 );

		//if (dev->Open(comPortAsString[comPort]) < 0 || ComSetBaudrate(comBaud))
		if (notOpen || ComSetBaudrate(comBaud))
			return false;
	}

	return true;
}


int BHandSerialDriver::ComRead(char* rxBuf, int rxNumBytes)
{
	if (!ComIsOpen())
	{
		return BHERR_READCOM;
	}

	// Calculate total read timeout
	unsigned int readTotalTimeout = readTotalTimeoutMultiplier * rxNumBytes + readTotalTimeoutConstant;
	if (readTotalTimeout == 0)
		readTotalTimeout = wxTIMEOUT_INFINITY;

	int numRead = dev->Readv(rxBuf, rxNumBytes, readTotalTimeout);

	// Return BHERR_READCOM if number of characters read is not the number requested, otherwise return 0 on success
	return (rxNumBytes != numRead) ? BHERR_READCOM : 0;
}

int BHandSerialDriver::ComWrite(const char* txBuf, int txNumBytes)
{
	if (!ComIsOpen())
		return BHERR_WRITECOM;

	// Calculate total write timeout
	unsigned int writeTotalTimeout = writeTotalTimeoutMultiplier * txNumBytes + writeTotalTimeoutConstant;
	if (writeTotalTimeout == 0)
		writeTotalTimeout = wxTIMEOUT_INFINITY;

	int numWritten = dev->Writev((char *)txBuf, txNumBytes, writeTotalTimeout);

	//printf("numWritten = %d of %d\n", numWritten, txNumBytes);

	// Return BHERR_WRITECOM if number of characters written is not
	// the number requested, otherwise return 0 on success
	return (txNumBytes != numWritten) ? BHERR_WRITECOM : 0;
}

const char * BHandSerialDriver::Response()
{
	// return inbuf address
	return (const char*)inbuf;
}

const char * BHandSerialDriver::Buffer()
{
	// return outbuf address
	return (const char*)outbuf;
}




///////////////////////////////////////////////////////////////////////////
// RealTime mode commands
///////////////////////////////////////////////////////////////////////////

int BHandSerialDriver::RTStart(const char *motor, BHMotorProtection motorProtection)
{
	// set active flags
	rtFlags[0][0] = (strrchr(motor,'1') || strrchr(motor,'G'));
	rtFlags[1][0] = (strrchr(motor,'2') || strrchr(motor,'G'));
	rtFlags[2][0] = (strrchr(motor,'3') || strrchr(motor,'G'));
	rtFlags[3][0] = (strrchr(motor,'4') || strrchr(motor,'S'));

	// get flags from hand
	for (int m = 0; m < 4; m++)
	{
		char motor[] = "1";
		motor[0] += m;
		if (m_bhand->Get(motor, "LCV LCPG LCT LFV LFS LFAP LFDP LFBP LFAIN", &rtFlags[m][1]))
			return BHERR_NOTINITIALIZED;

		// ensure that only 1 mode is enabled at the same time
		if ((rtFlags[m][1] || rtFlags[m][2]) && rtFlags[m][3])
			return BHERR_NOTINITIALIZED;
	}

	if (PGet("LFT", &rtGlobalFlags[0]))
		return BHERR_NOTINITIALIZED;


	// compute control and feedback positions
	int myFeedbackPos = 0;
	int myControlPos = 0;
	for (int m = 0; m < 4; m++)
	{
		// rtControl[][] is used as an index into the rtout[] (serial xmit bytes) array.
		// For example: rtControl[3][1] will tell you which element of rtout[] corresponds to motor 3's Prop Gain
		// Note that rtControl[3][1] by itself does not indicate whether motor 3's Prop Gain should be included
		// in rtout[], you need to check rtFlags[3][2] for that info. See the Get() commands above.
		rtControl[m][0] = myControlPos; if (rtFlags[m][0]*rtFlags[m][1]) myControlPos++;  /* Velocity */
		rtControl[m][1] = myControlPos; if (rtFlags[m][0]*rtFlags[m][2]) myControlPos++;  /* Proportional Gain */
		rtControl[m][2] = myControlPos; if (rtFlags[m][0]*rtFlags[m][3]) myControlPos+=2; /* Torque */ // EH updated size to match firmware (7-29-10)

		// rtFeedback[][] is used as an index into the rtin[] (serial recv bytes) array.
		rtFeedback[m][0] = myFeedbackPos; if (rtFlags[m][0]*rtFlags[m][4]) myFeedbackPos++;  /* Velocity */
		rtFeedback[m][1] = myFeedbackPos; if (rtFlags[m][0]*rtFlags[m][5]) myFeedbackPos++;  /* Strain */
		rtFeedback[m][2] = myFeedbackPos; if (rtFlags[m][0]*rtFlags[m][6]) myFeedbackPos+=2; /* Position */
		rtFeedback[m][3] = myFeedbackPos; if (rtFlags[m][0]*rtFlags[m][7]) myFeedbackPos++;  /* Delta Position */
		rtFeedback[m][4] = myFeedbackPos; if (rtFlags[m][0]*rtFlags[m][8]) myFeedbackPos+=2; /* Breakaway Position */
		rtFeedback[m][5] = myFeedbackPos; if (rtFlags[m][0]*rtFlags[m][9]) myFeedbackPos++;  /* Analog INput */

		// Initialize references to zero
		RTSetVelocity('1'+m, 0);
		RTSetGain('1'+m, 0);
		RTSetTorque('1'+m, 0);
	}

	rtGlobalFeedback[0] = myFeedbackPos; if (rtGlobalFlags[0]) myFeedbackPos+=1; /* Temperature */

	// Clear RealTime feedback buffer
	memset(rtIn, 0, 4*8+1);

	// assign totals
	nSend = myControlPos;
	nReceive = myFeedbackPos;

	// prepare output char buffer
	sprintf(outbuf, "%sLOOP\r", motor);
	nout = strlen(outbuf);

	// ask thread to send command
	return m_moduleSuperReal->ComRequest(BHREQ_SUPERVISE);
}


int BHandSerialDriver::RTUpdate(bool control, bool feedback)
{
	// prepare output buffer
	if (control)
	{
		outbuf[0] = (feedback ? 'C' : 'c');
		memcpy(outbuf + 1, rtOut, nSend);
		nout = nSend + 1;
	}
	else
	{
		outbuf[0] = (feedback ? 'A' : 'a');
		nout = 1;
	}

	// ask thread to send command
	nin = (feedback ? 1 + nReceive : 1);

	return m_moduleSuperReal->ComRequest(BHREQ_REALTIME);
}

int BHandSerialDriver::RTAbort()
{
	// prepare output char buffer
	outbuf[0] = 3;
	outbuf[1] = 0;
	nout = 1;

	// ask thread to send command
	return m_moduleSuperReal->ComRequest(BHREQ_SUPERVISE);
}

int BHandSerialDriver::RTSetFlags(const char *motor, bool LCV, int LCVC, bool LCPG,
                         bool LFV, int LFVC, bool LFS, bool LFAP, bool LFDP, int LFDPC)
{
	// prepare output char buffer
	sprintf(outbuf, "%sFSET LCV %d LCVC %d LCPG %d LFV %d LFVC %d LFS %d LFAP %d LFDP %d LFDPC %d\r",
		motor, LCV, LCVC, LCPG, LFV, LFVC, LFS, LFAP, LFDP, LFDPC);
	nout = strlen(outbuf);

	// ask thread to send command
	return m_moduleSuperReal->ComRequest(BHREQ_SUPERVISE);
}

int BHandSerialDriver::RTSetFlags(const char *motor, bool LCV, int LCVC, bool LCPG, bool LCT,
                         bool LFV, int LFVC, bool LFS, bool LFAP, bool LFDP, int LFDPC,
                         bool LFBP, bool LFAIN, bool LFDPD, bool LFT)
{
	int err;

	// prepare output char buffer
	sprintf(outbuf, "%sFSET LCV %d LCVC %d LCPG %d LCT %d LFV %d LFVC %d LFS %d LFAP %d LFDP %d LFDPC %d LFBP %d LFAIN %d\r",
		motor, LCV, LCVC, LCPG, LCT, LFV, LFVC, LFS, LFAP, LFDP, LFDPC, LFBP, LFAIN);
	nout = strlen( outbuf );

	// ask thread to send command
	err = m_moduleSuperReal->ComRequest(BHREQ_SUPERVISE);

	// prepare output char buffer
	sprintf(outbuf, "PSET LFT %d LFDPD %d\r", LFT, LFDPD);
	nout = strlen(outbuf);

	// ask thread to send command
	err |= m_moduleSuperReal->ComRequest(BHREQ_SUPERVISE);

	return err;
}

int BHandSerialDriver::RTUpdate(const char *motor, const char *property, int *values)
{
	// This method is not going to be implemented for the BH8-262 hand
	return BHERR_NOTCOMPLETED;
}

//bool BHandSerialDriver::RTMotorIncluded(unsigned int motorIndex)
//{
//	return rtFlags[motorIndex][0];
//}

int BHandSerialDriver::RTSetVelocity(const char motor, int velocity)
{
	// convert '1'-'4' to 0-3
	int m = motor - '1';
	if (m < 0)
		m = 0;
	else if (m > 3)
		m = 3;

	// active check
	if (!rtFlags[m][0])
		return BHERR_MOTORINACTIVE;

	// range check
	if (velocity > 127)
		velocity = 127;
	else if (velocity < -127)
		velocity = -127;

	// assign
	rtOut[rtControl[m][0]] = velocity;
	return 0;
}

int BHandSerialDriver::RTSetGain(const char motor, int gain)
{
	// convert '1'-'4' to 0-3
	int m = motor - '1';
	if (m < 0)
		m = 0;
	else if (m > 3)
		m = 3;

	// active check
	if (!rtFlags[m][0])
		return BHERR_MOTORINACTIVE;

	// range check
	if (gain > 255)
		gain = 255;
	else if (gain < 0)
		gain = 0;

	// assign
	rtOut[rtControl[m][1]] = (unsigned char)(gain & 0xFF); /* BCZ 11/12/99 WAS: rtOut[rtControl[m][1]] = (char)gain; */
	return 0;
}

int BHandSerialDriver::RTSetTorque(const char motor, int torque)
{
	// convert '1'-'4' to 0-3
	int m = motor - '1';
	if (m < 0)
		m = 0;
	else if (m > 3)
		m = 3;

	// active check
	if (!rtFlags[m][0])
		return BHERR_MOTORINACTIVE;

	// range check
	if (torque > 32767)
		torque = 32767;
	else if (torque < -32768)
		torque = -32768;

	// assign
	rtOut[rtControl[m][2]] = (char)((torque >> 8) & 0x00FF);
	rtOut[rtControl[m][2]+1] = (char)((torque) & 0x00FF);
	return 0;
}

int BHandSerialDriver::RTSetPosition(const char motor, int position)
{
	// BH8-262 hand does not have RealTime Position mode
	return 0;
}

char BHandSerialDriver::RTGetVelocity(const char motor)
{
	// convert '1'-'4' to 0-3
	int m = motor - '1';
	if (m < 0)
		m = 0;
	else if (m > 3)
		m = 3;

	// get result
	if (!rtFlags[m][0] || !rtFlags[m][4])
		return 0;
	else
		return rtIn[rtFeedback[m][0]];
}

unsigned char BHandSerialDriver::RTGetStrain(const char motor)
{
	// convert '1'-'4' to 0-3
	int m = motor - '1';
	if (m < 0)
		m = 0;
	else if (m > 3)
		m = 3;

	// get result
	if (!rtFlags[m][0] || !rtFlags[m][5])
		return 0;
	else
		return (unsigned char)rtIn[rtFeedback[m][1]];
}

int BHandSerialDriver::RTGetPosition(const char motor)
{
	// convert '1'-'4' to 0-3
	int m = motor - '1';
	if (m < 0)
		m = 0;
	else if (m > 3)
		m = 3;

	// get result
	if (!rtFlags[m][0] || !rtFlags[m][6])
		return 0;
	else //BCZ 10/15/99, Added (unsigned char) to the following line
		return ( (((int)rtIn[rtFeedback[m][2]])<<8) + (int)(unsigned char)rtIn[rtFeedback[m][2]+1] );
}

char BHandSerialDriver::RTGetDeltaPos(const char motor)
{
	// convert '1'-'4' to 0-3
	int m = motor - '1';
	if (m < 0)
		m = 0;
	else if (m > 3)
		m = 3;

	// get result
	if (!rtFlags[m][0] || !rtFlags[m][7])
		return 0;
	else
		return rtIn[rtFeedback[m][3]];
}

int BHandSerialDriver::RTGetBreakawayPosition(const char motor) // merged version untested
{
	// convert '1'-'4' to 0-3
	int m = motor - '1';
	if (m < 0)
		m = 0;
	else if (m > 3)
		m = 3;

	// get result
	if (!rtFlags[m][0] || !rtFlags[m][8])
		return 0;
	else
		return ( (((int)rtIn[rtFeedback[m][4]])<<8) + (int)(unsigned char)rtIn[rtFeedback[m][4]+1] );
}

int BHandSerialDriver::RTGetTemp()
{
	// get result
	if (!rtGlobalFlags[0])
		return 0;
	else
		return rtIn[rtGlobalFeedback[0]];
}

unsigned char BHandSerialDriver::RTGetAIN(const char motor)
{
	// Get AIN[1-4] reading
	// convert '1'-'4' to 0-3
	int m = motor - '1';
	if (m < 0)
		m = 0;
	else if (m > 3)
		m = 3;

	// get result
	if (!rtFlags[m][0] || !rtFlags[m][9])
		return 0;
	else
		return (unsigned char)rtIn[rtFeedback[m][5]];
}

void BHandSerialDriver::RTGetPPS(const char motor, int *pps, int ppsElements)
{
	// Create artificial data for pressure profile sensors
	/*for (int i = 0; i < 24; i++)
		pps[i] = (23 - i)  * 4095 / 24;*/

	static float x = 0.3f;
	static float y = 0.6f;
	static int dirX = 1;
	static int dirY = 1;

	unsigned int GRIDX = (motor <= '3') ? 3 : 7;
	unsigned int GRIDY = (motor <= '3') ? 8 : 4;

	int i = 0;
	for (unsigned int r = 0; r < GRIDY; r++)
	{
		for (unsigned int c = 0; c < GRIDX; c++)
		{
			if (i >= ppsElements)
				continue;
			float x0 = (c + 0.5f) / GRIDX;
			float y0 = (r + 0.5f) / GRIDY;
			float dist = sqrtf((x-x0)*(x-x0) + (y-y0)*(y-y0));
			pps[i] = (int)(4095.0f * (1 - dist /1.42f));

			if (pps[i] > 4095)
				pps[i] = 4095;
			else if (pps[i] < 0)
				pps[i] = 0;

			if (dirX > 0)
			{
				x += 0.0003f;
				if (x > 1)
					dirX = -1;
			}
			else
			{
				x -= 0.0003f;
				if (x < 0)
					dirX = 1;
			}

			if (dirY > 0)
			{
				y += 0.0003f;
				if (y > 1)
					dirY = -1;
			}
			else
			{
				y -= 0.0003f;
				if (y < 0)
					dirY = 1;
			}

			if (motor > '3' && (r == 0 || r == (GRIDY - 1)) && (c == 0 || c == (GRIDX - 1)))
			{
				// Ignore these elements
			}
			else
				i++;

		}
	}
}



// Return 1 if there are bytes available, 0 otherwise
/*int BHand::bytesAvailable(int port)
{
	char ch;

	if (dev->Read(&ch, 1) == 1)
	{
		dev->PutBack(ch);
		return 1;
	}
	else
		return 0;

}
int BHand::readNullUntilTimeout(int port, int timeOutSecs)
{
//DLG: added a helper function that times out if no input is read
//DLG: only used once, but is was kinda useful, so decided to include it as a helper function

	for (int n = 0; n < 1000*timeOutSecs; n++)
	{
		if (bytesAvailable(port) > 0)
			return 0;

		DELAY(1);
	}
	return -1;
}*/
