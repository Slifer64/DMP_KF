///////////////////////////////////////////////////////////////////////////////
//                   (C) Barrett Technology Inc. 2009-2010                   //
///////////////////////////////////////////////////////////////////////////////

#ifndef BHAND_DRIVER_H
#define BHAND_DRIVER_H

/** \file BHandDriver.h

    \brief Contains the interface for the BarrettHand.

    Drivers that implement the interface must provide an implementation of each
    abstract function.  The driver implementation does the actual communication
    with the hand.  Users don't need to be concerned how this is done since the
    API provides higher level Supervisory and RealTime methods that hide
    the device driver implementation.  Users should only be calling public
    methods found in BHand.

*/

#include "BHandCommands.h"

#include <stdio.h>


// Define constants in Linux to be same priority constants as Windows uses for a NORMAL_PRIORITY_CLASS process
#ifdef LINUX
	#define THREAD_PRIORITY_TIME_CRITICAL 15
	#define THREAD_PRIORITY_HIGHEST 10
	#define THREAD_PRIORITY_ABOVE_NORMAL 9
	#define THREAD_PRIORITY_NORMAL 8
	#define THREAD_PRIORITY_BELOW_NORMAL 7
	#define THREAD_PRIORITY_LOWEST 6
	#define THREAD_PRIORITY_IDLE 1
#endif


//! Type of Communication
/*! The type of communication used with the BarrettHand */
enum BHCommunication {
	BH_SERIAL_COMMUNICATION = 0,
	BH_CAN_COMMUNICATION
	};


class BHand;
class BHandSupervisoryRealtime;
class BHandSupervisoryCommand;
class BHandSupervisoryResult;


//! BarrettHand API Callback Type
/*! This type is used by the API to call user defined functions outside the BarrettHand API */
typedef	void (*BHCallback)(class BHand *);


class BHandDriver
{
public:

	///////////////////////////////////////////////////////////////////////////
	// BHandDriver Constructor/Deconstructor
	///////////////////////////////////////////////////////////////////////////

	BHandDriver(BHand *bhand, BHandSupervisoryRealtime *moduleSuperReal, BHCommunication comm) :
		m_bhand(bhand), m_moduleSuperReal(moduleSuperReal), m_Comm(comm), m_open(false), m_activeSupervisoryCommand(NULL), m_supervisoryCommandResult(NULL) {}
	virtual ~BHandDriver()
	{
		// Check for last command result
		if (m_supervisoryCommandResult != NULL)
		{
			// The result from the last supervisory command that was executed should be deleted
			delete m_supervisoryCommandResult;
			m_supervisoryCommandResult = NULL;

		}
	}


	///////////////////////////////////////////////////////////////////////////
	// BHandDriver Misc. Public Methods
	///////////////////////////////////////////////////////////////////////////

	BHCommunication getComm() { return m_Comm; }

	void setActiveSupervisoryCommand(BHandSupervisoryCommand *activeSupervisoryCommand) { m_activeSupervisoryCommand = activeSupervisoryCommand; }


	///////////////////////////////////////////////////////////////////////////
	// BarrettHand Driver Pure Virtual Methods
	///////////////////////////////////////////////////////////////////////////

	// Abstract methods for initializing and closing the BarrettHand driver
	virtual int Initialize() = 0;
	virtual void Close() = 0;
	bool IsOpen() { return m_open; } // Useful to determine if the driver interface is open or not

	// Abstract methods for Supervisory and RealTime control of the BarrettHand
	int ExecuteSupervisoryCall();
	virtual int ExecuteRealtimeCall() = 0;

	virtual void SetWaitCallbackFunc(BHCallback waitCallbackFunc) = 0;

	BHandSupervisoryResult *GetResult() { return m_supervisoryCommandResult; }


	///////////////////////////////////////////////////////////////////////////////
	// Supervisory Module Pure Virtual Methods
	///////////////////////////////////////////////////////////////////////////////

	// Methods (Init, Calibration, Reset, etc.)
	int InitHand(const char *motor);
	int Reset(bool *responseReceived = 0);

	// Methods (Motor Movement)
	int Close(const char *motor);
	int Open(const char *motor);

	int GoToDefault(const char *motor);
	int GoToDifferentPositions(const int *encoderPositons, unsigned int numEncoderPositions);
	int GoToHome(const char *motor = "");
	int GoToPosition(const char *motor, int value);

	int StepClose(const char *motor, bool valueIncluded = false, int stepAmount = 0);
	int StepOpen(const char *motor, bool valueIncluded = false, int stepAmount = 0);

	int StopMotor(const char *motor);

	int TorqueClose(const char *motor);
	int TorqueOpen(const char *motor);

	// Methods (Parameter Get/Set)
	int Get(const char *motor, const char *parameter, int *result);
	int Set(const char *motor, const char *parameter, int value);

	int PGet(const char *parameter, int *result);
	int PSet(const char *parameter, int value);

	int Default(const char *motor);
	int Load(const char *motor);
	int Save(const char *motor);

	int Temperature(int *temperature);

	// Methods (Misc.)
	int Delay(unsigned int msec);

	int Command(const char *send);
	int Command(const char *send, char *receive);

	int Baud(unsigned int baud);


	///////////////////////////////////////////////////////////////////////////
	// RealTime Module Pure Virtual Methods
	///////////////////////////////////////////////////////////////////////////

	// Abstract methods (Control Over Execution)
	virtual int		RTStart(const char *motor, BHMotorProtection motorProtection = BHMotorTSTOPProtect) = 0;
	virtual int		RTUpdate(bool control = true, bool feedback = true) = 0;
	virtual int		RTAbort() = 0;

	// Abstract methods (Parameter Get/Set)
	virtual int		RTSetFlags(const char *motor, bool LCV, int LCVC, bool LCPG,
	            	           bool LFV, int LFVC, bool LFS, bool LFAP, bool LFDP, int LFDPC) = 0;
	virtual int		RTSetFlags(const char *motor, bool LCV, int LCVC, bool LCPG, bool LCT,
	           				   bool LFV, int LFVC, bool LFS, bool LFAP, bool LFDP, int LFDPC,
	           				   bool LFBP, bool LFAIN, bool LFDPD, bool LFT) = 0;

	virtual int		RTUpdate(const char *motor, const char *property, int *values) = 0;

	// Abstract methods (Control)
	virtual int		RTSetVelocity(const char motor, int velocity) = 0;
	virtual int		RTSetGain(const char motor, int gain) = 0;
	virtual int		RTSetTorque(const char motor, int torque) = 0;
	virtual int		RTSetPosition(const char motor, int position) = 0;

	// Abstract methods (Feedback)
	virtual char	RTGetVelocity(const char motor) = 0;
	virtual unsigned char RTGetStrain(const char motor) = 0;
	virtual int		RTGetPosition(const char motor) = 0;
	virtual char	RTGetDeltaPos(const char motor) = 0;
	virtual int		RTGetBreakawayPosition(const char motor) = 0;
	virtual int		RTGetTemp() = 0;
	virtual unsigned char RTGetAIN(const char motor) = 0;
	virtual void	RTGetPPS(const char motor, int *pps, int ppsElements) = 0;


protected:

	///////////////////////////////////////////////////////////////////////////////
	// Methods in the PucksInHand Supervisory Module
	///////////////////////////////////////////////////////////////////////////////

	// Init, Calibration, Reset, etc.
	virtual int HandInit(BHMotors bhMotors) = 0;
	virtual int HandReset(BHMotors bhMotors, bool *responseReceived) = 0;

	// Motor Movement
	virtual int HandClose(BHMotors bhMotors) = 0;
	virtual int HandOpen(BHMotors bhMotors) = 0;

	virtual int HandGoToDefault(BHMotors bhMotors, bool valueIncluded = false, int defaultPosition = 0) = 0;
	virtual int HandGoToDifferentPositionsHand(const int *encoderPositions, unsigned int numEncoderPositions) = 0;
	virtual int HandGoToHome(BHMotors bhMotors) = 0;
	virtual int HandGoToPosition(BHMotors bhMotors, unsigned int encoderPositionTickCount) = 0;

	virtual int HandStepClose(BHMotors bhMotors, bool valueIncluded = false, int stepAmount = 0) = 0;
	virtual int HandStepOpen(BHMotors bhMotors, bool valueIncluded = false, int stepAmount = 0) = 0;

	virtual int HandTorqueClose(BHMotors bhMotors) = 0;
	virtual int HandTorqueOpen(BHMotors bhMotors) = 0;

	//virtual int HandStopMotor(BHMotors bhMotors) = 0;

	// Parameter Get/Set
	virtual int HandGet(BHMotors bhMotors, const char *property, int *propertyResult, int *nresults = 0) = 0;
	virtual int HandSet(BHMotors bhMotors, const char *property, int value) = 0;

	virtual int HandPGet(const char *property, int *propertyResult) = 0;
	virtual int HandPSet(const char *property, int value) = 0;

	virtual int HandDefault(BHMotors bhMotors) = 0;
	virtual int HandLoad(BHMotors bhMotors) = 0;
	virtual int HandSave(BHMotors bhMotors) = 0;

	virtual int HandTemperature(BHMotors bhMotors, int *temperature, unsigned int numTemperatures) = 0;

	// Misc.
	virtual int HandDelay(unsigned int milliSeconds) = 0;

	virtual int HandStopMotor(BHMotors bhMotors) = 0;

	virtual char * HandCommand(const char *send, int *errorCode) = 0;

	// Serial support
	virtual int HandBaud(unsigned int newBaud) = 0;

	///////////////////////////////////////////////////////////////////////////////
	// More general methods
	///////////////////////////////////////////////////////////////////////////////

	virtual int HandleSupervisoryCall() = 0;

	int runSupervisoryCommand(BHandSupervisoryCommand *command);

	BHand *m_bhand;
	BHandSupervisoryRealtime *m_moduleSuperReal;

	BHCommunication m_Comm;

	bool m_open;

	BHandSupervisoryCommand *m_activeSupervisoryCommand;
	BHandSupervisoryResult *m_supervisoryCommandResult;

};

#endif
