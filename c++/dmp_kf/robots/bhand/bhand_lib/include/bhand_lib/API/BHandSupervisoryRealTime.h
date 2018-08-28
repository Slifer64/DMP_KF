///////////////////////////////////////////////////////////////////////////////
//                   (C) Barrett Technology Inc. 2009-2010                   //
///////////////////////////////////////////////////////////////////////////////

#ifndef BHAND_SUPERVISORY_REALTIME_H
#define BHAND_SUPERVISORY_REALTIME_H

/** \file BHandSupervisoryRealTime.h

    \brief Contains the declaration of a class for the BarrettHand Supervisory and RealTime modules.

    The BarrettHand API provides high-level control functions for the Barrett
    Hand that operates under in a supervisory mode or under a real time mode.
    This module needs to be initialized with a call to #InitSoftware or #Init.

    If the API has been compiled to support serial communication then low-level
    access of the serial port methods that are used internally are exposed.
    These serial port methods may be used to support building a custom software
    interface to the BarrettHand.  Barrett uses these methods to provide a
    bootloader for uploading new firmware outside the API.

    Supervisory mode is the mode that the hand starts in after initialization.
    There are several useful commands such as open and close commands that
    will open and close fingers on the BarrettHand.

    The Supervisory and RealTime modules contained here use the POCO C++
    Foundation library for cross-platform multithreading.

    RealTime mode allows control loops to be formed so that control and
    feedback data may be exchanged with the hand.  This mode is used
    to control the hand directly.

*/


#include "BHandDriver.h"
#include "BHandHardware.h"
#include "BHandCommands.h"

#include "bhand_motors.h"

// Required for POCO to not remove extra Windows code that a users code may need
#define POCO_NO_UNWINDOWS

// POCO library includes
#include "Poco/Thread.h"
#include "Poco/Runnable.h"
#include "Poco/Mutex.h"
#include "Poco/Event.h"

using Poco::Thread;
using Poco::Runnable;
using Poco::Event;


// TODO: Comment and document this define
//#define BHAND_SUPERVISORY_PRIVATE_VARIABLES


//! Communication Requests
/*! Requests to communications thread */
enum BHRequest {
	BHREQ_EXIT = 1,
	BHREQ_REALTIME,
	BHREQ_SUPERVISE,
	BHREQ_CLEAR
	};

//! Synchronization Modes
/*! Controls supervisory blocking/non-blocking behavior of supervisory commands */
enum BHSyncMode {
	BHMODE_SYNC = 1,
	BHMODE_ASYNCNOW,
	BHMODE_ASYNCWAIT,
	BHMODE_RETURN
	};

//! API Error Codes
/*! Negative value error codes used within the BarrettHand API */
enum BHAPIErrorCode {
	BHERR_BHANDEXISTS = -1,
	BHERR_OPENCOMMPORT = -2,
	BHERR_GETCOMMSTATE = -3,
	BHERR_SETCOMMSTATE = -4,
	BHERR_SETCOMMTIMEOUT = -5,
	BHERR_THREADSTART = -6,
	BHERR_THREADPRIORITY = -7,
	BHERR_WRITECOM = -8,
	BHERR_READCOM = -9,
	BHERR_BADRESPONSE = -10,
	BHERR_CLEARBUFFER = -11,
	BHERR_TIMEOUT = -12,
	BHERR_NOTCOMPLETED = -13,
	BHERR_PENDING = -14,
	BHERR_NOTINITIALIZED = -15,
	BHERR_BADPARAMETER = -16,
	BHERR_LONGSTRING = -17,
	BHERR_OUTOFRANGE = -18,
	BHERR_MOTORINACTIVE = -19,
	BHERR_PORTOUTOFRANGE = -20
	};


/** This class provides methods from Supervisory and RealTime modules.

This class provides an device independent way to access the Supervisory and
RealTime modules in the BarrettHand.  It must be initialized before most
methods may be used.  In addition there may be a serial communication module
included if the API was compiled to support serial communication.
*/
class BHandSupervisoryRealtime : public Runnable
{
public:

	///////////////////////////////////////////////////////////////////////////
	// Public Methods (SupervisoryRealtime Constructor/Deconstructor)
	///////////////////////////////////////////////////////////////////////////

	BHandSupervisoryRealtime(BHand *bhand, int priority = THREAD_PRIORITY_TIME_CRITICAL);
	~BHandSupervisoryRealtime();


	///////////////////////////////////////////////////////////////////////////
	// Public Methods (SupervisoryRealtime Initialization)
	///////////////////////////////////////////////////////////////////////////

	int InitSoftware(int port, int priority = THREAD_PRIORITY_TIME_CRITICAL);
	int Init(int port, int priority, BHCommunication comm, bool async = false);
	bool IsInitialized() { return driverIsOpen(); }


	///////////////////////////////////////////////////////////////////////////////
	// Methods in Supervisory Module (Control Over Execution)
	///////////////////////////////////////////////////////////////////////////////

	int		ComRequest(int requestNumber);
	int		ComWaitForCompletion(unsigned int timeout);
	bool	ComIsPending();

	int		ComGetError();
	BHandSupervisoryResult * GetResult(); // TODO: check that this is a good format


	///////////////////////////////////////////////////////////////////////////
	// Methods in Serial Communication Module
	///////////////////////////////////////////////////////////////////////////

#ifdef BH8_262_HARDWARE

	int		ComInitialize(int comport, int priority = THREAD_PRIORITY_TIME_CRITICAL);
	int		ComOpen(int comport, int baudrate = 9600);
	bool	ComIsOpen();
	void	ComClose();

	int		ComSetBaudrate(int baud);

	int		ComSetTimeouts(unsigned int readInterval,
			               unsigned int readMultiplier, unsigned int readConstant,
			               unsigned int writeMultiplier, unsigned int writeConstant);
	int		ComSetTimeouts(unsigned int readMultiplier, unsigned int readConstant,
			               unsigned int writeMultiplier, unsigned int writeConstant);

	bool	ComClear(bool rxOnly = false);
	int		ComRead(char *rxBuf, int rxNumBytes);
	int		ComWrite(const char *txBuf, int txNumBytes);

#endif

	void SetWaitCallbackFunc(BHCallback waitCallbackFunc);

	///////////////////////////////////////////////////////////////////////////////
	// More methods in Supervisory Module (Init, Calibration, Reset, etc.)
	///////////////////////////////////////////////////////////////////////////////

	int		InitHand(const char *motor);
	int		Reset(bool *responseReceived = 0);

	///////////////////////////////////////////////////////////////////////////
	// More methods in Supervisory Module (Motor Movement)
	///////////////////////////////////////////////////////////////////////////

	int		Close(const char *motor);
	int		Open(const char *motor);

	int		GoToDefault(const char *motor);
	int		GoToDifferentPositions(int value1, int value2, int value3, int value4);
	int		GoToHome(const char *motor = "");
	int		GoToPosition(const char *motor, int value);

	int		StepClose(const char *motor);
	int		StepClose(const char *motor, int stepAmount);
	int		StepOpen(const char *motor);
	int		StepOpen(const char *motor, int stepAmount);

	int		TorqueClose(const char *motor);
	int		TorqueOpen(const char *motor);

	///////////////////////////////////////////////////////////////////////////
	// More methods in Supervisory Module (Property Get/Set)
	///////////////////////////////////////////////////////////////////////////

	int		Get(const char *motor, const char *propertyName, int *result);
	int		Set(const char *motor, const char *propertyName, int value);

	int		PGet(const char *propertyName, int *result);
	int		PSet(const char *propertyName, int value);

	int		Default(const char *motor);
	int		Load(const char *motor);
	int		Save(const char *motor);

	int		Temperature(int *result);

	///////////////////////////////////////////////////////////////////////////
	// More methods in Supervisory Module (Misc.)
	///////////////////////////////////////////////////////////////////////////

	int		Command(const char *send, char *receive = 0);

	int		Delay(unsigned int msec);

	int		StopMotor(const char *motor);

#ifdef BH8_262_HARDWARE
	// Non-Supervisory Accessor Methods
	const char * Response();
	const char * Buffer();
#endif

	///////////////////////////////////////////////////////////////////////////
	// More methods in Supervisory Module (Legacy support)
	///////////////////////////////////////////////////////////////////////////

	int		Baud(unsigned int baud);


	///////////////////////////////////////////////////////////////////////////
	// Public Methods (Control Over Execution of Supervisory Mode Commands)
	///////////////////////////////////////////////////////////////////////////

	void setSyncMode(BHSyncMode syncMode) { this->syncMode = syncMode; }
	BHSyncMode getSyncMode() { return (BHSyncMode)syncMode; }

	void setRequestTimeout(unsigned int timeout) { requestTimeout = timeout; }
	unsigned int getRequestTimeout() { return requestTimeout; }

	void setpCallback(BHCallback funcPtr) { pCallback = funcPtr; }
	BHCallback getpCallback() { return pCallback; }


	///////////////////////////////////////////////////////////////////////////
	// Methods in RealTime Module (Control Over Execution)
	///////////////////////////////////////////////////////////////////////////

	int		RTStart(const char *motor, BHMotorProtection motorProtection = BHMotorTSTOPProtect);
	int		RTUpdate(bool control = true, bool feedback = true);
	int		RTAbort();

	///////////////////////////////////////////////////////////////////////////
	// More methods in RealTime Module (Parameter Get/Set)
	///////////////////////////////////////////////////////////////////////////

	int		RTSetFlags(const char *motor, bool LCV, int LCVC, bool LCPG,
	    	           bool LFV, int LFVC, bool LFS, bool LFAP, bool LFDP, int LFDPC);
	int		RTSetFlags(const char *motor, bool LCV, int LCVC, bool LCPG, bool LCT,
					   bool LFV, int LFVC, bool LFS, bool LFAP, bool LFDP, int LFDPC,
					   bool LFBP, bool LFAIN, bool LFDPD, bool LFT);

	int		RTUpdate(const char *motor, const char *property, int *values);

	///////////////////////////////////////////////////////////////////////////
	// More methods in RealTime Module (Control)
	///////////////////////////////////////////////////////////////////////////

	int		RTSetVelocity(const char motor, int velocity);
	int		RTSetGain(const char motor, int gain);
	int		RTSetTorque(const char motor, int torque);
	int		RTSetPosition(const char motor, int position);

	///////////////////////////////////////////////////////////////////////////
	// More methods in RealTime Module (Feedback)
	///////////////////////////////////////////////////////////////////////////

	char	RTGetVelocity(const char motor);
	unsigned char RTGetStrain(const char motor);
	int		RTGetPosition(const char motor);
	char	RTGetDeltaPos(const char motor);
	int		RTGetBreakawayPosition(const char motor);
	int		RTGetTemp();
	unsigned char RTGetAIN(const char motor);
	void	RTGetPPS(const char motor, int *pps, int ppsElements);




#ifdef BHAND_SUPERVISORY_PRIVATE_VARIABLES
protected:
#endif

	///////////////////////////////////////////////////////////////////////////
	// Variables in Supervisory Module (Control Over Execution)
	///////////////////////////////////////////////////////////////////////////

	/**
	 This variable determines whether/how the user program waits for the
	 low-level thread to complete the request before it continues.

	 Values:
	 - BHMODE_SYNC:        User program waits for completion of low-level thread.
	 - BHMODE_ASYNCNOW:    Try to send request now (error if another request is
	    being processed), do not wait for completion.
	 - BHMODE_ASYNCWAIT:   Wait for completion of previous request, then send
	    request and return immediately
	 - BHMODE_RETURN:      Send only requests for parameters (Temperature or Get
	    commands), disregard all other requests, run the callback
        function and return.

	 Default: BHMODE_SYNCH

	 Notes:
	 Setting the variable to asynchronous mode allows you to continue program
	 execution while the request is still being processed.
	 \par Do not use asynchronous mode when a result is to be returned.
	 */
	int		syncMode;

	/**
	 This variable specifies the timeout interval (in milliseconds) used in synchronous mode.

	 Values:	position integers

	 Default:	INFINITE

	 Notes:     INFINITE specifies that the user program does not resume until the low-level thread is finished processing the present request
	 */
	unsigned int requestTimeout;

	/**
	This variable, if different from NULL, is a pointer to a function that
	will be called right before the low-level thread signals the user program
	that processing has finished.

	Values: 	any function

	Default:	NULL

	Notes:  	The function will be executed with high priority so it should
	not be computationally intensive.  The callback function types is:
	typedef void (*BHCallback)(class BHand*).  Following is an example:

	  \code
	  void RealTimeCallbackFunction(BHand *nopotr)
	  {
	    // Insert code to be executed here
	  }
	  \endcode

	  In your main() function use the following assignment:

	  \code
	  pCallBack = RealTimeCallbackFunction;
	  \endcode


	*/
	BHCallback pCallback;

protected:

	BHand * getBHand() { return m_bhand; }

private:

	///////////////////////////////////////////////////////////////////////////
	// Private method (required to be implemented to be Runnable)
	///////////////////////////////////////////////////////////////////////////

	void run();


	///////////////////////////////////////////////////////////////////////////
	// Private methods (Misc.)
	///////////////////////////////////////////////////////////////////////////

	bool driverIsOpen();
	bool serialDriverIsOpen();

	void createThread(int priority);


	///////////////////////////////////////////////////////////////////////////
	// Methods used to validate calls to commands before they are made
	///////////////////////////////////////////////////////////////////////////

	int validate(BHMotors motors, const char *propertyName, int value);
	int validate(const char *propertyName, int value);


	///////////////////////////////////////////////////////////////////////////
	// Private member variables
	///////////////////////////////////////////////////////////////////////////

	bool m_initialized;

	BHand *m_bhand;
	BHandDriver *m_driver;

	Thread	m_thread;

	Event m_requestPending;
	Event m_requestComplete;

	int		m_request;
	int		m_comErr;

};

#endif
