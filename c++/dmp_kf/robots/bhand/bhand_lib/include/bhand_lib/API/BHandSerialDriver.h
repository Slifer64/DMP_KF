///////////////////////////////////////////////////////////////////////////////
//                   (C) Barrett Technology Inc. 2009-2010                   //
///////////////////////////////////////////////////////////////////////////////

#ifndef BHAND_SERIAL_H
#define BHAND_SERIAL_H

#include "BHandDriver.h"

// char buffer size, max comport
#define BH_MAXCHAR 5000
#define BH_MAXPORT 9


class wxSerialPort;

class BHandSerialDriver : public BHandDriver
{
public:

	///////////////////////////////////////////////////////////////////////////
	// CAN BHandDriver Constructor/Deconstructor
	///////////////////////////////////////////////////////////////////////////

	BHandSerialDriver(BHand *bhand, BHandSupervisoryRealtime *moduleSuperReal, unsigned int comport);
	virtual ~BHandSerialDriver();




	///////////////////////////////////////////////////////////////////////////
	// BarrettHand Driver Methods
	///////////////////////////////////////////////////////////////////////////

	// Methods for initializing and closing the BarrettHand serial driver
	int Initialize();
	void Close();

	void SetWaitCallbackFunc(BHCallback waitCallbackFunc);




	///////////////////////////////////////////////////////////////////////////
	// Serial Communication Methods
	///////////////////////////////////////////////////////////////////////////

	// Main serial communication methods
	int		ComInitialize(int comport);
	int		ComOpen(int comport, int baudrate = 9600);
	bool	ComIsOpen();
	void	ComClose();

	int		ComSetBaudrate(int baud);
	int		ComSetTimeouts(unsigned int readMultiplier, unsigned int readConstant,
			               unsigned int writeMultiplier, unsigned int writeConstant);

	bool	ComClear(bool rxOnly = false);
	int		ComRead(char *rxBuf, int rxNumBytes);
	int		ComWrite(const char *txBuf, int txNumBytes);

	// Other serial communication methods
	const char * Response();
	const char * Buffer();




	///////////////////////////////////////////////////////////////////////////
	// RealTime Module Methods
	///////////////////////////////////////////////////////////////////////////

	// Control Over Execution
	int		RTStart(const char *motor, BHMotorProtection motorProtection = BHMotorTSTOPProtect);
	int		RTUpdate(bool control = true, bool feedback = true);
	int		RTAbort();

	// Parameter Get/Set
	int		RTSetFlags(const char *motor, bool LCV, int LCVC, bool LCPG,
	       	           bool LFV, int LFVC, bool LFS, bool LFAP, bool LFDP, int LFDPC);
	int		RTSetFlags(const char *motor, bool LCV, int LCVC, bool LCPG, bool LCT,
	   				   bool LFV, int LFVC, bool LFS, bool LFAP, bool LFDP, int LFDPC,
	   				   bool LFBP, bool LFAIN, bool LFDPD, bool LFT);

	int		RTUpdate(const char *motor, const char *property, int *values);

	// Control
	int		RTSetVelocity(const char motor, int velocity);
	int		RTSetGain(const char motor, int gain);
	int		RTSetTorque(const char motor, int torque);
	int		RTSetPosition(const char motor, int position);

	// Feedback
	char	RTGetVelocity(const char motor);
	unsigned char RTGetStrain(const char motor);
	int		RTGetPosition(const char motor);
	char	RTGetDeltaPos(const char motor);
	int		RTGetBreakawayPosition(const char motor);
	int		RTGetTemp();
	unsigned char RTGetAIN(const char motor);
	void	RTGetPPS(const char motor, int *pps, int ppsElements);




	///////////////////////////////////////////////////////////////////////////
	// Other Methods
	///////////////////////////////////////////////////////////////////////////

	static void SetDefaultBaud(unsigned int baud) { defBaud = baud; } // 9600, 19200, or 38400


private:

	///////////////////////////////////////////////////////////////////////////////
	// Supervisory Module Methods
	///////////////////////////////////////////////////////////////////////////////

	// Init, Calibration, Reset, etc.
	int HandInit(BHMotors bhMotors);
	int HandReset(BHMotors bhMotors, bool *responseReceived);

	// Motor Movement
	int HandClose(BHMotors bhMotors);
	int HandOpen(BHMotors bhMotors);

	int HandGoToDefault(BHMotors bhMotors, bool valueIncluded = false, int defaultPosition = 0);
	int HandGoToDifferentPositionsHand(const int *encoderPositions, unsigned int numEncoderPositions);
	int HandGoToHome(BHMotors bhMotors);
	int HandGoToPosition(BHMotors bhMotors, unsigned int encoderPositionTickCount);

	int HandStepClose(BHMotors bhMotors, bool valueIncluded = false, int stepAmount = 0);
	int HandStepOpen(BHMotors bhMotors, bool valueIncluded = false, int stepAmount = 0);

	int HandTorqueClose(BHMotors bhMotors);
	int HandTorqueOpen(BHMotors bhMotors);

	// Parameter Get/Set
	int HandGet(BHMotors bhMotors, const char *property, int *propertyResult, int *nresults = 0);
	int HandSet(BHMotors bhMotors, const char *property, int value);

	int HandPGet(const char *property, int *propertyResult);
	int HandPSet(const char *property, int value);

	int HandDefault(BHMotors bhMotors);
	int HandLoad(BHMotors bhMotors);
	int HandSave(BHMotors bhMotors);

	int HandTemperature(BHMotors bhMotors, int *temperature, unsigned int numTemperatures);

	// Misc.
	int HandDelay(unsigned int msec);

	int HandStopMotor(BHMotors bhMotors);

	char * HandCommand(const char *send, int *errorCode);

	int HandBaud(unsigned int newbaud);


	// Methods for Supervisory and RealTime control of the BarrettHand
	int ExecuteRealtimeCall();
	int HandleSupervisoryCall();

	//int runSupervisoryCommand(BHandSupervisoryCommand *command);

	int		rtFlags[4][10];
	int		rtGlobalFlags[1];
	int		nSend;
	int		nReceive;
	char	rtIn[4*8+1];

	// Pointer to a serial port interface (from the CTB serial library)
	wxSerialPort *dev;

	// Serial port state
	static int defBaud;
	int		comPort;
	int		comBaud;

	// Read/write timeouts
	unsigned int readTotalTimeoutMultiplier;
	unsigned int readTotalTimeoutConstant;
	unsigned int writeTotalTimeoutMultiplier;
	unsigned int writeTotalTimeoutConstant;

	// Temporary variable to help set a new baud rate
	unsigned int requestBaud;

	// Serial input/output buffers and number of bytes in input/output buffers
	char	inbuf[BH_MAXCHAR];
	int		nin;
	char	outbuf[BH_MAXCHAR];
	int		nout;

	// Real time mode state
	char	rtOut[4*4+1];
	int		rtControl[4][3];
	int		rtFeedback[4][6];
	int		rtGlobalFeedback[1];


	///////////////////////////////////////////////////////////////////////////
	// Private Methods
	///////////////////////////////////////////////////////////////////////////

	int Interactive();

};


#endif
