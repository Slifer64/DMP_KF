/** \file BHandSupervisoryRealTime.cpp

    \brief Contains the implementation of a class for the BarrettHand Supervisory and RealTime modules.

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

#include "BHandSupervisoryRealTime.h"

#include "BHand.h"

#ifdef BH8_280_HARDWARE
	#include "BHandCANDriver.h" // Include headers for puck communication with the BarrettHand
#endif
#ifdef BH8_262_HARDWARE
	#include "BHandSerialDriver.h" // Include header for serial communication with the BarrettHand
	#include "BHandBH8_262.h"
#endif

#include "bhand_parse.h"

///////////////////////////////////////////////////////////////////////////
// Public Methods (SupervisoryRealtime Constructor/Deconstructor)
///////////////////////////////////////////////////////////////////////////

BHandSupervisoryRealtime::BHandSupervisoryRealtime(BHand *bhand, int priority) :
	syncMode(BHMODE_SYNC), requestTimeout(INFINITE), pCallback(NULL),
	m_initialized(false), m_bhand(bhand), m_driver(NULL),
	m_requestPending(),
	m_requestComplete(false),
	m_request(0), m_comErr(0)
{
	createThread(priority);
}

BHandSupervisoryRealtime::~BHandSupervisoryRealtime()
{
	while (ComIsPending());
	// Ask thread to stop
	syncMode = BHMODE_ASYNCWAIT;
	ComRequest(BHREQ_EXIT);

	// Wait for thread to complete execution for up to 1000 milliseconds
	m_thread.tryJoin(1000);

	// Close communication interface
	if (m_driver != NULL)
	{
		m_driver->Close();
		delete m_driver;
	}
}


///////////////////////////////////////////////////////////////////////////
// Public Methods (SupervisoryRealtime Initialization)
///////////////////////////////////////////////////////////////////////////



/** Initialize this BHand instance.

  The InitSoftware method will initialize communication with the BarrettHand.
  The BarrettHand API needs to have the Supervisory and RealTime modules
  initialized as well as having the means to communicate with the hand prior to
  using Supervisory or RealTime methods.

  This method will initialize the Supervisory and RealTime modules given that
  the parameters are set correctly and serial communication with the hand is
  established.  If this method successfully establishes communication with the
  hand then a low-level thread is started for executing high-level BarrettHand
  Supervisory commands.  Both reset and init hand commands will also be issued.

  \param port The comport to use for communication (1 for "COM1" in Windows or "/dev/ttyS0" in Linux)
  \param priority The thread priority for the communication thread
  \retval int Returns 0 on success and a BHERR error code on failure
*/
int	BHandSupervisoryRealtime::InitSoftware(int port, int priority)
{
	// Set hardware description before initialization
	int hwIndex = BHandHardware::getBHandHardwareIndex("BH8-262");
	if (hwIndex < 0)
	{
		//printf("\n\nThe API has not been compiled to include target hand.\n");
		return BHERR_NOTINITIALIZED;
	}
	m_bhand->setHardwareDesc(hwIndex);


	// BarrettHand API is compiled with BH8-262 support
	// so initialize for serial communication
	return Init(port, priority, BH_SERIAL_COMMUNICATION);
}

/** Initialize this BHand instance.

  This method will need to be called to initialize the hand for CAN
  or serial communication.  Before Init is called, setHardwareDesc should be
  called to set the hardware description so that the API knows what hand it
  will be communicating with.  This method will attempt to reset the hand and
  will report an error if there is a problem.  Users may run initialize as a
  non-blocking call by passing true to the async parameter.  This is for
  advanced applications that need to not block if there is a timeout or
  problem attempting to reset the hand.  Note that all supervisory commands
  will be run asynchronously until the mode is switched.

  \param port The comport to use for communication (1 for "COM1" in Windows or "/dev/ttyS0" in Linux)
  \param priority The thread priority for the communication thread
  \param async Will set the blocking/non-blocking behavior of Init, default value is false
  \retval int Returns 0 on success and a BHERR error code on failure
*/
int BHandSupervisoryRealtime::Init(int port, int priority, BHCommunication comm, bool async)
{
	// Check to make sure software has not already been initialized
	if (m_initialized)
		return BHERR_BHANDEXISTS;

	///////////////////////////////////////////////////////////////////////////
	// Initialize BarrettHand the Device Driver (CAN or serial)
	///////////////////////////////////////////////////////////////////////////
	//printf("Init 1( port = %d\n", port);

	if (m_driver != NULL)
	{
		// Cleanup driver created previously
		m_driver->Close();
		delete m_driver;
		m_driver = NULL;
	}

	switch (comm)
	{
#ifdef BH8_280_HARDWARE
		case BH_CAN_COMMUNICATION:
		{
			m_driver = new BHandCANDriver(m_bhand, this);
			break;
		}
#endif
#ifdef BH8_262_HARDWARE
		case BH_SERIAL_COMMUNICATION:
		{
			m_driver = new BHandSerialDriver(m_bhand, this, port);
			break;
		}
#endif
		default:
		{
			// Return if communication with the BarrettHand is not supported
			return BHERR_NOTINITIALIZED;
		}
	}

	//printf("Init 2( port = %d\n", port);

	///////////////////////////////////////////////////////////////////////////
	// Set device driver used for the Supervisory and RealTime module
	///////////////////////////////////////////////////////////////////////////

	m_bhand->setDeviceDriver(m_driver);

	///////////////////////////////////////////////////////////////////////////
	// Initialize device driver
	///////////////////////////////////////////////////////////////////////////

	int result;
	if ((result = m_driver->Initialize()))
	{
		m_driver->Close();
		m_comErr = result;
		return result;                    // Driver did not initialize
	}

	//printf("Init before set timeouts\n");

#ifdef BH8_262_HARDWARE
	// Replace timeouts with more reasonable ones to achieve a faster
	// timeout if there is no response from the hand after sending a Reset command
	if (comm == BH_SERIAL_COMMUNICATION && (result = ComSetTimeouts(50, 2000, 50, 2000)) > 0)
	{
		m_driver->Close();
		m_comErr = result;
		return result;
	}
#endif

	//printf("Init after set timeouts\n");

	// Set asynchronous mode if requested (more advanced)
	if (async)
		setSyncMode(BHMODE_ASYNCWAIT);

	///////////////////////////////////////////////////////////////////////////
	// Send Reset command and store if a response is received
	///////////////////////////////////////////////////////////////////////////

	//printf("Init wait for response\n");

	bool receivedResponse;
	result = Reset(&receivedResponse);

#ifdef BH8_262_HARDWARE
	// Restore default timeouts (set in ComInitialize)
	if (!async && comm == BH_SERIAL_COMMUNICATION && (result = ComSetTimeouts(50, 15000, 50, 5000)) > 0)
	{
		m_driver->Close();
		m_comErr = result;
		return result;
	}
#endif

	// Set initialized flag
	m_initialized = receivedResponse;
	/*if (async)
	{
		m_initialized = true;
		printf("Init m_initialized = %d receivedResponse = %d\n", m_initialized, receivedResponse);
		return 0;
	}
	else
	{
		m_initialized = receivedResponse;
		printf("Init m_initialized = %d receivedResponse = %d\n", m_initialized, receivedResponse);
	}*/

	//printf("Init received\n");

	// Return an error if there was no response received after sending a reset
	return receivedResponse ? 0 : BHERR_READCOM;
}


///////////////////////////////////////////////////////////////////////////
// Private methods (Misc.)
///////////////////////////////////////////////////////////////////////////

bool BHandSupervisoryRealtime::driverIsOpen() { return m_driver != NULL && m_driver->IsOpen(); }
bool BHandSupervisoryRealtime::serialDriverIsOpen() { return driverIsOpen() && m_driver->getComm() == BH_SERIAL_COMMUNICATION; }

void BHandSupervisoryRealtime::createThread(int priority)
{
	// Initialize variables for thread
	m_requestComplete.set();
	m_request = 0;

	// Create thread for asynchronous communication
	m_thread.setName("BHand Async");
	switch (priority)
	{
		case THREAD_PRIORITY_TIME_CRITICAL: { m_thread.setPriority(Poco::Thread::PRIO_HIGHEST); break; }
		case THREAD_PRIORITY_HIGHEST:       { m_thread.setPriority(Poco::Thread::PRIO_HIGHEST); break; }
		case THREAD_PRIORITY_ABOVE_NORMAL:  { m_thread.setPriority(Poco::Thread::PRIO_HIGH);    break; }
		case THREAD_PRIORITY_NORMAL:        { m_thread.setPriority(Poco::Thread::PRIO_NORMAL);  break; }
		case THREAD_PRIORITY_BELOW_NORMAL:  { m_thread.setPriority(Poco::Thread::PRIO_LOW);     break; }
		case THREAD_PRIORITY_LOWEST:        { m_thread.setPriority(Poco::Thread::PRIO_LOWEST);  break; }
		case THREAD_PRIORITY_IDLE:          { m_thread.setPriority(Poco::Thread::PRIO_LOWEST);  break; }
		default:                            { m_thread.setPriority(Poco::Thread::PRIO_HIGHEST); break; }
	}

	m_thread.start(*this);

	// Note that POCO does not have any way that would allow for detecting if
	// the thread was not created or if setting the thread priority failed.
	// So BHERR_THREADSTART and BHERR_THREADPRIORITY are never returned anymore.
}




///////////////////////////////////////////////////////////////////////////
// Methods used to validate calls to commands before they are made
///////////////////////////////////////////////////////////////////////////

int BHandSupervisoryRealtime::validate(BHMotors motors, const char *propertyName, int value)
{
	// Check if there are motor properties to validate
	if (!motors)
		return 0;

	//printf("validate(%d, %s, %d)", motors, propertyName, value);

	// Get property that may be used to validate input
	BHandHardware *hw = m_bhand->getHardwareDesc();

	BHandProperty *p = hw->getBHandProperty(propertyName);

	// Make sure that this property may be a motor property
	if (p->getNumExtendedAttributes() != 4)
		return BHERR_OUTOFRANGE;

	for (unsigned int i = 0; i < 4; i++) // handle more motors
		if ((motors >> i) & 1)
		{
			int minValue = p->getMinValue(i);
			int maxValue = p->getMaxValue(i);

			//printf("validate %s check range %d minValue = %d maxValue = %d\n", propertyName, i, minValue, maxValue);

			// Validate value is in property range
			if (value < minValue || value > maxValue)
				return BHERR_OUTOFRANGE;
		}

	return 0;
}

int BHandSupervisoryRealtime::validate(const char *propertyName, int value)
{
	// Get property that may be used to validate input
	BHandHardware *hw = m_bhand->getHardwareDesc();
	BHandProperty *p = hw->getBHandProperty(propertyName);

	// Make sure that this property may be a global property
	if (p->getNumExtendedAttributes() != 1)
		return BHERR_OUTOFRANGE;

	int minValue = p->getMinValue();
	int maxValue = p->getMaxValue();

	// Validate value is in property range
	if (value < minValue || value > maxValue)
		return BHERR_OUTOFRANGE;

	return 0;
}




///////////////////////////////////////////////////////////////////////////
// Public Methods (Control Over Execution)
///////////////////////////////////////////////////////////////////////////


/** Sends a request to the low-level communication thread.

  The request must be BHREQ_EXIT, BHREQ_REALTIME, BHREQ_SUPERVISE, or
  BHREQ_CLEAR.

  \param requestNumber One of the request constants listed above
  \retval int Returns 0 on success or an error code on failure
*/
int BHandSupervisoryRealtime::ComRequest(int requestNumber)
{
	// Wait for the request handler thread to be ready to accept a request
	if (syncMode == BHMODE_ASYNCNOW)
	{
		// in immediate async mode, so throw an error if the request handler is busy
		if (!m_requestComplete.tryWait(0))
			return BHERR_NOTCOMPLETED;
	}
	else
	{
		if (requestTimeout < INFINITE)
			m_requestComplete.wait(requestTimeout);
		else
		{
			m_requestComplete.wait();
		}
	}

	// return immediately
	if (syncMode == BHMODE_RETURN)
	{
		// call callback anyway
		m_comErr = 0;
		if (pCallback)
			(*(pCallback))(getBHand());

		return 0;
	}

	// clear request complete event
	m_requestComplete.reset();

	// send request
	m_request = requestNumber;
	m_requestPending.set();

	// wait for completion in sync mode
	if (syncMode == BHMODE_SYNC)
		return ComWaitForCompletion(requestTimeout);
	else
		return BHERR_PENDING;
}


/** Waits for low-level communication thread to complete processing.

  Blocks until the communication thread completes processing in the given
  amount of time.  It will return 0, a positive hand error code, or a negative
  timeout error code.

  \param timeout Number of milliseconds to wait for completion (may be INFINITE)
  \retval int Returns 0, a positive hand error code, or BHERR_TIMEOUT if there is a timeout
*/
int BHandSupervisoryRealtime::ComWaitForCompletion(unsigned int timeout)
{
	if (timeout < INFINITE)
	{
		bool wait = m_requestComplete.tryWait(timeout);
		if (wait)
			return m_comErr;
		else
			return BHERR_TIMEOUT;
	}
	else
		m_requestComplete.wait();

	return m_comErr;
}


/** Checks for a low-level pending communication request

\retval bool Returns true if a communication request is pending and false otherwise
*/
bool BHandSupervisoryRealtime::ComIsPending()
{
	return !m_requestComplete.tryWait(0);
}

/** Checks the low-level thread for communication errors

Communication errors are set to zero before communication begins with the
low-level thread and is set after completion to the value that would be
returned by the Supervisory command if run synchronously.  Use this to
get the last error, which is the only method available to access the error
code in asynchronous mode.  Call this after ComIsPending() returns false
or the application receives a callback on a method that the supervisory
command has executed.

\retval int Returns the most recent communication error
 */
int BHandSupervisoryRealtime::ComGetError()
{
	return m_comErr;
}

/** Obtain command results after the supervisory command has finished running

The last command run should be a supervisory command that returns a result.
Ensure that the command has finished running.  Use ComIsPending() or received
the finished command event through a callback method.  Usually, a cast is
needed to receive useful information from the result returned as appropriate.

\retval BHandSupervisoryResult Pointer to most recent result
 */
BHandSupervisoryResult * BHandSupervisoryRealtime::GetResult()
{
	return m_driver->GetResult();
}


// A macro to end thread processing and invoke callback function
#define THREADEND(err)             \
{                                  \
	m_comErr = (err);              \
	if (pCallback)                 \
		(*(pCallback))(m_bhand);   \
	break;                         \
}


///////////////////////////////////////////////////////////////////////////
// Private method (required to be implemented to be Runnable)
///////////////////////////////////////////////////////////////////////////

/** The low-level communication thread for the BarrettHand.

  This thread waits for low-level communication requests and may make calls
  that block until they are finished.  Blocking calls may take quite awhile to
  complete so this thread waits for them and then handles making calls to the
  BarrettHand driver one at a time.  Requests are signalled using ComRequest.

  \internal
*/
void BHandSupervisoryRealtime::run()
{
	int result;

	// Loop forever - user must ask thread to exit
	while (1)
	{
		// Signal that the request is complete
		m_requestComplete.set();

		// Now wait for a request
		m_requestPending.wait(); // event is auto reset

		m_comErr = 0;

		// Check to see if the driver ready to process Supervisory or RealTime requests
		if (!driverIsOpen() && (m_request == BHREQ_REALTIME || m_request == BHREQ_SUPERVISE))
			continue;

		// handle all requests
		switch (m_request)
		{
		case BHREQ_EXIT:
			{
				// Set request complete
				m_requestComplete.set();
				//m_comPending = false;

				return;
			}
		case BHREQ_REALTIME:
			{
				// loop mode command
				result = m_driver->ExecuteRealtimeCall();

				THREADEND(result);
			}

		case BHREQ_SUPERVISE:
			{
				// Supervisory mode command
				result = m_driver->ExecuteSupervisoryCall();

				// Note that 280 commands still exist to return results in asynchonous mode
				// This will need to get deleted within this class later

				THREADEND(result);
			}

		case BHREQ_CLEAR:
			{
#ifdef BH8_262_HARDWARE
				// Clear com port buffers (will have no effect if not using the serial driver)
				if (!ComClear())
					THREADEND(BHERR_CLEARBUFFER)
#endif
				THREADEND(0)
			}
		default:
			{
			}
		}
	}
}




///////////////////////////////////////////////////////////////////////////////
// Methods in Serial Communication Module
///////////////////////////////////////////////////////////////////////////////

#ifdef BH8_262_HARDWARE

/** Initializes serial communication.

  This method is used internally by InitSoftware.

  \param comport Com port number (1 for "COM1" in Windows or "/dev/ttyS0" in Linux)
  \param priority The thread priority parameter is not used anymore
  \retval int Returns 0 on success or an error code on failure
*/
int BHandSupervisoryRealtime::ComInitialize(int comport, int priority)
{
	//if (m_driver != NULL)
		return (m_driver != NULL && m_driver->getComm() == BH_SERIAL_COMMUNICATION) ?
			((BHandSerialDriver *)m_driver)->ComInitialize(comport) : BHERR_OPENCOMMPORT;
	/*else
	{
		printf("BHandSupervisoryRealtime::ComInitialize(%d ... new case for driver being NULL\n", comport);
		// Open serial port communication with hand
		m_driver = new BHandSerialDriver(m_bhand, this, comport);
		m_bhand->setDeviceDriver(m_driver);


		//printf("m_driver->getComm() == BH_SERIAL_COMMUNICATION = %d", m_driver->getComm() == BH_SERIAL_COMMUNICATION);
		return (m_driver != NULL && m_driver->getComm() == BH_SERIAL_COMMUNICATION) ?
			((BHandSerialDriver *)m_driver)->ComInitialize(comport) : BHERR_OPENCOMMPORT;
	}*/
}


/** Opens serial communication port.

  This is provided to support serial communication with the BarrettHand. A user
  may choose to write their own application that uses the serial port using
  only the serial communication provided in a BHand instance.

  \param comport Com port number (1 for "COM1" in Windows or "/dev/ttyS0" in Linux)
  \param baudrate The desired baud rate
  \retval int Returns 0 on success or an error code on failure
*/
int BHandSupervisoryRealtime::ComOpen(int comport, int baudrate)
{
	// Check to make sure software has not already been initialized
	if (m_initialized || m_driver != NULL)
	{
		printf("BHandSupervisoryRealtime: m_initialized || m_driver != NULL\n");
		if (m_driver != NULL && m_driver->getComm() == BH_SERIAL_COMMUNICATION)
		{
			printf("For the first time - using serial driver that already exists.\n");
			return ((BHandSerialDriver *)m_driver)->ComOpen(comport, baudrate);
		}
		else
			return BHERR_BHANDEXISTS;
	}


	printf("BHandSupervisoryRealtime::ComOpen\n");
	if (m_driver != NULL)
		printf("m_driver != NULL\n");
	else
		printf("m_driver = NULL\n");

	// Open serial port communication with hand
	m_driver = new BHandSerialDriver(m_bhand, this, comport);
	m_bhand->setDeviceDriver(m_driver);


	//printf("m_driver->getComm() == BH_SERIAL_COMMUNICATION = %d", m_driver->getComm() == BH_SERIAL_COMMUNICATION);
	return (m_driver != NULL && m_driver->getComm() == BH_SERIAL_COMMUNICATION) ?
		((BHandSerialDriver *)m_driver)->ComOpen(comport, baudrate) : BHERR_OPENCOMMPORT;
}


/** Close serial port communication.

  Will close the serial port if it is opened.

*/
void BHandSupervisoryRealtime::ComClose()
{
	if (serialDriverIsOpen())
	{
		((BHandSerialDriver *)m_driver)->ComClose();

		if (m_driver != NULL)
		{
			delete m_driver;
			m_driver = NULL;
		}
		m_initialized = false;
	}
}

/** Useful for determining if the com port is opened.

  \retval bool Returns true if opened and false if not opened
*/
bool BHandSupervisoryRealtime::ComIsOpen()
{
	return serialDriverIsOpen() ? ((BHandSerialDriver *)m_driver)->ComIsOpen() : false;
}


/** Clears the serial port input and output buffers.

  Will always clear the input buffer and will also clear the output buffer by
  default.  Clearing the output buffer takes longer since the serial port needs
  to be reopened.

  \param rxOnly Clear only the receive buffer
  \retval bool Returns true on success or false if there is a problem.
*/
bool BHandSupervisoryRealtime::ComClear(bool rxOnly)
{
	return serialDriverIsOpen() ? ((BHandSerialDriver *)m_driver)->ComClear(rxOnly) : false;
}


/** Reads bytes from serial port input buffer.

  If the serial port is opened then this method will transfer rxNumBytes from
  the input buffer to the given buffer.  It will block forever until the
  number of requested bytes have been received.

  \param rxBuf Pointer to a receive buffer that bytes will be transferred to
  \param rxNumBytes Number of bytes to receive from the input buffer
  \retval int Returns 0 on success and BHERR_READCOM if there is a problem.
*/
int BHandSupervisoryRealtime::ComRead(char* rxBuf, int rxNumBytes)
{
	return serialDriverIsOpen() ? ((BHandSerialDriver *)m_driver)->ComRead(rxBuf, rxNumBytes) : BHERR_OPENCOMMPORT;
}


/** Writes bytes to the serial port output buffer.

  If the serial port is opened then this method will transfer txNumBytes to
  the output buffer from the given buffer.  It will block forever until the
  number of requested bytes have been transmitted.

  \param txBuf Pointer to a transmit buffer that bytes will be transferred from
  \param txNumBytes Number of bytes to transfer to the output buffer
  \retval int Returns 0 on success and BHERR_WRITECOM if there is a problem.
*/
int BHandSupervisoryRealtime::ComWrite(const char* txBuf, int txNumBytes)
{
	return serialDriverIsOpen() ? ((BHandSerialDriver *)m_driver)->ComWrite(txBuf, txNumBytes) : BHERR_OPENCOMMPORT;
}


/** Registers a callback function to be called periodically by the BHandCANDriver

  The method will be called periodically while movement commands are running
  and with the Supervisory Delay command.  This method can be helpful for
  polling property values with the RTUpdate get property method.  This method
  is okay to execute during one of the supervisory events described above.

  \param waitCallbackFunc Method signature must be: void waitFunc(BHand *bh)
 */
void BHandSupervisoryRealtime::SetWaitCallbackFunc(BHCallback waitCallbackFunc)
{
	if (driverIsOpen())
		m_driver->SetWaitCallbackFunc(waitCallbackFunc);
}


/** Sets timeout parameters for serial communication.

  Both timeout constants are in milliseconds.  Each read and write call to the
  serial functions ComRead and ComWrite will end after the calculated timeout
  period.  Timeout periods for reads and writes are computed as the sum of the
  read/write constant and read/write multiplier multiplied by the number of
  bytes to transmit or receive.  A computed timeout value of zero disables the
  respective timeout, essentially making the timeout infinite.

  Example:
  \code
  // Set timeout parameters
  // read interval of 999 is ignored
  // read multiplier of 50
  // read constant of 15000
  // write multiplier of 50
  // write constant of 5000
  err = bh.ComSetTimeouts(999, 50, 15000, 50, 5000);
  \endcode

  It is recommended to remain at the default values unless shorter or longer
  timeouts are required in your application.  Even at the slowest possible baud
  rate of 600 baud, the communications will not timeout under normal
  circumstances.

  \param readInterval Ignored
  \param readMultiplier Average time per character
  \param readConstant Constant for the entire transaction
  \param writeMultiplier Average time per character
  \param writeConstant Constant for the entire transaction
  \retval int Returns 0 on success and BHERR_SETCOMMTIMEOUT on failure
*/
int BHandSupervisoryRealtime::ComSetTimeouts(unsigned int readInterval,
                                             unsigned int readMultiplier, unsigned int readConstant,
                                             unsigned int writeMultiplier, unsigned int writeConstant)
{
	return ComSetTimeouts(readMultiplier, readConstant, writeMultiplier, writeConstant);
}


/** Sets timeout parameters for serial communication.

  Both timeout constants are in milliseconds.  Each read and write call to the
  serial functions ComRead and ComWrite will end after the calculated timeout
  period.  Timeout periods for reads and writes are computed as the sum of the
  read/write constant and read/write multiplier multiplied by the number of
  bytes to transmit or receive.  A computed timeout value of zero disables the
  respective timeout, essentially making the timeout infinite.

  Example:
  \code
  // Set timeout parameters
  // read multiplier of 50
  // read constant of 15000
  // write multiplier of 50
  // write constant of 5000
  err = bh.ComSetTimeouts(50, 15000, 50, 5000);
  \endcode

  It is recommended to remain at the default values unless shorter or longer
  timeouts are required in your application.  Even at the slowest possible baud
  rate of 600 baud, the communications will not timeout under normal
  circumstances.

  \param readMultiplier Average time per character
  \param readConstant Constant for the entire transaction
  \param writeMultiplier Average time per character
  \param writeConstant Constant for the entire transaction
  \retval int Returns 0 on success and BHERR_SETCOMMTIMEOUT on failure
*/
int BHandSupervisoryRealtime::ComSetTimeouts(unsigned int readMultiplier, unsigned int readConstant, unsigned int writeMultiplier, unsigned int writeConstant)
{
	return serialDriverIsOpen() ? ((BHandSerialDriver *)m_driver)->ComSetTimeouts(readMultiplier, readConstant, writeMultiplier, writeConstant) : BHERR_OPENCOMMPORT;
}


/** Sets the baud rate for serial communication.

  This method may also be called after ComOpen to set the baud rate.  The
  BarrettHand will only work with the standard baud rates up to 38400.  More
  baud rates are possible but are not supported in the BarrettHand.  Users
  should also look at the Baud command if interested in changing the baud rate
  used by the hand.

  \param baudrate The requested baud rate (possible values are 600, 1200, 2400, 4800, 9600, 19200, 38400)
  \retval int Returns 0 on success and BHERR_SETCOMMSTATE on failure
*/
int BHandSupervisoryRealtime::ComSetBaudrate(int baudrate)
{
	return serialDriverIsOpen() ? ((BHandSerialDriver *)m_driver)->ComSetBaudrate(baudrate) : BHERR_OPENCOMMPORT;
}

#endif




///////////////////////////////////////////////////////////////////////////////
// Methods in Supervisory Module
///////////////////////////////////////////////////////////////////////////////

/** Sends the "Hand Initialize" command to the hand.

  The purpose of this command is to determine encoder and motor alignment for
  commutation.  It moves all fingers and spread to open positions.

  Example:
  \code
  // Initializes all finger motors
  char motor[2] = "G";
  err = bh.InitHand(motor);
  \endcode

  InitHand() needs to be called after the hand has been reset. This command
  must be run before any other motor commands, once the hand is turned on.

  \param motor Specifies which motors to initialize
  \retval int Returns 0 on success and a BHERR error code on failure
*/
int	BHandSupervisoryRealtime::InitHand(const char *motor)
{
	return driverIsOpen() ? m_driver->InitHand(motor) : BHERR_OPENCOMMPORT;
}


/** Sends the "Reset" command to the hand.

  Resets the firmware loop in the BarrettHand and sets the baud rate to the default
  baud rate of 9600 bps unless it is set differently with the
  BHandSerialDriver::SetDefaultBaud method.

  Example:
  \code
  // resets the hand
  err = bh.Reset();
  \endcode

  After resetting the BarrettHand you will need to call InitHand() before
  issuing any motion commands.

  \retval int Returns 0 on success and a BHERR error code on failure
*/
int	BHandSupervisoryRealtime::Reset(bool *responseReceived)
{
	return driverIsOpen() ? m_driver->Reset(responseReceived) : BHERR_OPENCOMMPORT;
}


/** Sends the "Close" supervisory command to the hand.

  Commands the selected motor(s) to move finger(s) in the close direction with
  a velocity ramp-down to target limit, CT.

  Example:
  \code
  // closes grasp
  char motor[4] = "123";
  err = bh.Close(motor);
  \endcode

  Finger(s) close until the joint stop(s) are reached, the close target is
  reached, or an obstacle is encountered.

  \param motor Specifies which motors will be closed
  \retval int Returns 0 on success and a BHERR error code on failure
*/
int	BHandSupervisoryRealtime::Close(const char *motor)
{
	return driverIsOpen() ? m_driver->Close(motor) : BHERR_OPENCOMMPORT;
}


/** Sends the "Open" supervisory command to the hand.

  Commands the selected motor(s) to move finger(s) in the open direction with
  a velocity ramp-down at target limit, OT.

  Example:
  \code
  // Opens the spread
  char motor[2] = "S";
  err = bh.Open(motor);
  \endcode

  Finger(s) open until the open target is reached or an obstacle is encountered.
  The motor argument passed to the function needs to be a pointer to a string.

  \param motor Specifies which motors will be opened
  \retval int Returns 0 on success and a BHERR error code on failure
*/
int	BHandSupervisoryRealtime::Open(const char *motor)
{
	return driverIsOpen() ? m_driver->Open(motor) : BHERR_OPENCOMMPORT;
}


/** Sends the "Goto Default" supervisory command to the hand.

  Moves all motors to default positions defined by the default property DP.

  Example:
  \code
  // move grasp to default positions
  char result;
  err = bh.GoToDefault(motor);
  \endcode

  The motor argument passed to the function needs to be a pointer to a string.

  \param motor Specifies which motors will be moved
  \retval int Returns 0 on success and a BHERR error code on failure
*/
int	BHandSupervisoryRealtime::GoToDefault(const char* motor)
{
	return driverIsOpen() ? m_driver->GoToDefault(motor) : BHERR_OPENCOMMPORT;
}


/** Sends the "GoTo Different Position" supervisory command to the hand.

  Moves all motors to specified encoder positions.

  Example:
  \code
  // moves finger F1 to 2000, finger F2 to 3000, finger F3 to 4000, and spread
  to 1000
  err = bh.GoToDifferentPositions(2000, 3000, 4000, 1000);
  \endcode

  Encoder positions will be validated before setting "DP" property and
  going to the desired encoder positions.

  \param value1 Specifies the encoder position for motor 1
  \param value2 Specifies the encoder position for motor 2
  \param value3 Specifies the encoder position for motor 3
  \param value4 Specifies the encoder position for motor 4
  \retval int Returns 0 on success and a BHERR error code on failure
*/
int	BHandSupervisoryRealtime::GoToDifferentPositions(int value1, int value2, int value3, int value4)
{
	if (!driverIsOpen()) return BHERR_OPENCOMMPORT;

	if (validate(1, "DP", value1)) return BHERR_OUTOFRANGE;
	if (validate(2, "DP", value2)) return BHERR_OUTOFRANGE;
	if (validate(4, "DP", value3)) return BHERR_OUTOFRANGE;
	if (validate(8, "DP", value4)) return BHERR_OUTOFRANGE;

	// Fill desired encoder positions in array
	int encoderPositions[4];
	encoderPositions[0] = value1;
	encoderPositions[1] = value2;
	encoderPositions[2] = value3;
	encoderPositions[3] = value4;

	return m_driver->GoToDifferentPositions(encoderPositions, 4);
}


/** Sends the "GoTo Home" supervisory command to the hand.

  Moves the specified motors to position 0. If any fingers are sent home then
  all fingers will be sent home.  Spread is sent home last and only if it is
  commanded to return to the home posiiton.

  Example:
  \code
  // moves all motors to the home position
  err = bh.GoToHome();
  \endcode

  \retval int Returns 0 on success and a BHERR error code on failure
*/
int	BHandSupervisoryRealtime::GoToHome(const char *motor)
{
	return driverIsOpen() ? m_driver->GoToHome(motor) : BHERR_OPENCOMMPORT;
}


/** Sends the "GoTo Position" supervisory command to the hand.

  Moves motors to specified encoder position.

  Example:
  \code
  // moves finger F3 to position 10000
  char motor[2] = "3";
  err = bh.GoToPosition(motor, 10000);
  \endcode

  Encoder position will be validated to be in the range of the "DP"
  property for each included motor.

  \param motor Specifies which motors will be moved to the encoder position
  \param value Specifies the encoder position to move to
  \retval int Returns 0 on success and a BHERR error code on failure
*/
int	BHandSupervisoryRealtime::GoToPosition(const char *motor, int value)
{
	if (!driverIsOpen())
		return BHERR_OPENCOMMPORT;

	BHMotors bhMotors = toBHMotors(motor);
	if (validate(bhMotors, "DP", value))
		return BHERR_OUTOFRANGE;

	return m_driver->GoToPosition(motor, value);
}


/** Sends the "Step Close" supervisory command to the hand.

  Incrementally closes the specified motors. The property DS contains the
  default increment size that will be used.

  Example:
  \code
  // step close finger F2 1500 encoder counts
  char motor[2] = "2";
  err = bh.StepClose(motor, 1500);
  \endcode

  Step size will be validated to be in the range of the "DS"
  property for each included motor.

  \param motor Specifies which motors will be closed
  \param stepAmount Specifies the step size amount in encoder ticks
  \retval int Returns 0 on success and a BHERR error code on failure
*/
int	BHandSupervisoryRealtime::StepClose(const char *motor, int stepAmount)
{
	if (!driverIsOpen())
		return BHERR_OPENCOMMPORT;

	BHMotors bhMotors = toBHMotors(motor);
	if (validate(bhMotors, "DS", stepAmount))
		return BHERR_OUTOFRANGE;

	return m_driver->StepClose(motor, true, stepAmount);
}

/** Sends the "Step Close" supervisory command to the hand.

  Incrementally closes the specified motors. The property DS contains the
  default increment size that will be used.

  Example:
  \code
  // step close finger F2 1500 encoder counts
  char motor[2] = "2";
  err = bh.StepClose(motor, 1500);
  \endcode

  \param motor Specifies which motors will be closed
  \retval int Returns 0 on success and a BHERR error code on failure
*/
int	BHandSupervisoryRealtime::StepClose(const char *motor)
{
	if (!driverIsOpen())
		return BHERR_OPENCOMMPORT;

	return m_driver->StepClose(motor);
}


/** Sends the "Step Open" supervisory command to the hand.

  Incrementally opens the specified motors. The property DS contains the
  default increment size that will be used.

  Example:
  \code
  // step open the grasp 2000 encoder counts
  char motor[2] = "G";
  err = bh.StepOpen(motor, 2000);
  \endcode

  Step size will be validated to be in the range of the "DS"
  property for each included motor.

  \param motor Specifies which motors will be opened
  \param stepAmount Specifies the step size amount in encoder ticks
  \retval int Returns 0 on success and a BHERR error code on failure
*/
int	BHandSupervisoryRealtime::StepOpen(const char *motor, int stepAmount)
{
	if (!driverIsOpen())
		return BHERR_OPENCOMMPORT;

	BHMotors bhMotors = toBHMotors(motor);
	if (validate(bhMotors, "DS", stepAmount))
		return BHERR_OUTOFRANGE;

	return m_driver->StepOpen(motor, true, stepAmount);
}


/** Sends the "Step Open" supervisory command to the hand.

  Incrementally opens the specified motors. The DS property contains the
  default increment size that will be used.

  Example:
  \code
  // step open finger F2 by DP encoder counts
  char motor[2] = "2";
  err = bh.StepOpen(motor);
  \endcode

  \param motor Specifies which motors will be opened
  \retval int Returns 0 on success and a BHERR error code on failure
*/
int	BHandSupervisoryRealtime::StepOpen(const char *motor)
{
	if (!driverIsOpen())
		return BHERR_OPENCOMMPORT;

	return m_driver->StepOpen(motor);
}

/** Sends the "Torque Close" supervisory command to the hand.

  Commands velocity of selected motor(s) in the direction that closes the
  finger(s) with control of motor torque at stall.

  Example:
  \code
  // closes grasp with torque control
  char motor[4] = "123";
  err = bh.TorqueClose(motor);
  \endcode

  \param motor Specifies which motors will be closed with torque control
  \retval int Returns 0 on success and a BHERR error code on failure
*/
int	BHandSupervisoryRealtime::TorqueClose(const char *motor)
{
	return driverIsOpen() ? m_driver->TorqueClose(motor) : BHERR_OPENCOMMPORT;
}


/** Sends the "Torque Open" supervisory command to the hand.

  Commands velocity of selected motor(s) in the direction that opens the
  finger(s) with control of motor torque at stall.

  Example:
  \code
  // opens grasp with torque control
  char motor[4] = "123";
  err = bh.TorqueOpen(motor);
  \endcode

  \param motor Specifies which motors will be opened with torque control
  \retval int Returns 0 on success and a BHERR error code on failure
*/
int	BHandSupervisoryRealtime::TorqueOpen(const char *motor)
{
	return driverIsOpen() ? m_driver->TorqueOpen(motor) : BHERR_OPENCOMMPORT;
}


///////////////////////////////////////////////////////////////////////////
// More methods in Supervisory Module (Property Get/Set)
///////////////////////////////////////////////////////////////////////////


/** Sends the "Get" command to the hand.

  Gets motor properties.

  Example:
  \code
  // gets the maximum close velocity for finger F1 and stores it in result
  char motor[2] = "1";
  char property[4] = "MCV";
  int result;
  err = bh.Get(motor, property, &result);
  \endcode

  Refer to the BH8-Series User Manual or the BHControl GUI for a list of motor
  properties and their functions. Verify that the size of the result variable
  can hold all values returned. For example, if you request the values for
  motors F1, F2, and F3, make sure you pass a pointer to an array with at least
  3 valid locations.

  \param motor Specifies which motor's property to get
  \param propertyName Specifies which motor property you want to get
  \param result Specifies a pointer to where the result(s) will be stored
  \retval int Returns 0 on success and a BHERR error code on failure
*/
int	BHandSupervisoryRealtime::Get(const char *motor, const char *propertyName, int *result)
{
	if (!driverIsOpen())
		return BHERR_OPENCOMMPORT;

	// Get value(s) for properties
	int r;
	if ((r = m_driver->Get(motor, propertyName, result)))
		return r;

	// Validate values before returning?

	return 0;
}


/** Sends the "Set" command to the hand.

  Sets motor properties.

  Example:
  \code
  // set finger F1 maximum close velocity to 20
  char motor[2] = "1";
  char property[4] = "MCV";
  err = bh.Set(motor, property, 20);
  \endcode

  Refer to the BH8-Series User Manual or the BHControl GUI for a list of motor
  properties and their functions.  Set validates desired property value first.

  \param motor Specifies which motor's properties to set
  \param propertyName Specifies which motor property will be set
  \param value Specifies the desired value of the property
  \retval int Returns 0 on success and a BHERR error code on failure
*/
int	BHandSupervisoryRealtime::Set(const char *motor, const char *propertyName, int value)
{
	if (!driverIsOpen())
		return BHERR_OPENCOMMPORT;

	//if (validate(toBHMotors(motor), propertyName, value))
	//	return BHERR_OUTOFRANGE;

	return m_driver->Set(motor, propertyName, value);
}


/** Sends the "PGet" command to the hand.

  Gets the value of a global property.

  Example:
  \code
  // get over temperature fault value
  char property[6] = "OTEMP";
  int result;
  err = bh.PGet(property, &result);
  \endcode

  Refer to the BH8-Series User Manual or the BHControl GUI for a list of motor
  properties and their functions.

  \param propertyName Specifies which global property to get
  \param result Specifies a pointer to where the result will be stored
  \retval int Returns 0 on success and a BHERR error code on failure
*/
int	BHandSupervisoryRealtime::PGet(const char *propertyName, int *result)
{
	if (!driverIsOpen())
		return BHERR_OPENCOMMPORT;

	// Get value for property
	int r;
	if ((r = m_driver->PGet(propertyName, result)))
		return r;

	return 0;
}


/** Sends the "PSet" command to the hand.

  Sets the value of a global property.

  Example:
  \code
  // set over temperature fault to 585 (58.5 degrees Celsius)
  char property[6] = "OTEMP";
  err = bh.PSet(property, 585);
  \endcode

  Refer to the BH8-Series User Manual or the BHControl GUI for a list of motor
  properties and their functions.  PSet validates desired property value first.

  \param propertyName Specifies which global property will be set
  \param value Specifies the desired value of the property
  \retval int Returns 0 on success and a BHERR error code on failure
*/
int	BHandSupervisoryRealtime::PSet(const char *propertyName, int value)
{
	if (!driverIsOpen())
		return BHERR_OPENCOMMPORT;

	if (validate(propertyName, value))
		return BHERR_OUTOFRANGE;

	return m_driver->PSet(propertyName, value);
}


/** Sends the "Default" command to the hand.

  Loads factory default motor properties from EEPROM into active property list.

  Example:
  \code
  // loads factory default properties for the grasp
  char motor[2] = "G";
  err = bh.Default(motor);
  \endcode

  This command only changes the active properties, to write the properties to
  EEPROM use Save(). The motor argument passed to the function needs to be a
  pointer to a string.

  \param motor Specifies which motor's default properties to load
  \retval int Returns 0 on success and a BHERR error code on failure
*/
int	BHandSupervisoryRealtime::Default(const char* motor)
{
	return driverIsOpen() ? m_driver->Default(motor) : BHERR_OPENCOMMPORT;
}


/** Sends the "Load" command to the hand.

  Loads the saved motor properties from EEPROM into active property list.

  Example:
  \code
  // loads previously saved properties for the grasp
  char motor[2] = "G";
  err = bh.Load(motor);
  \endcode

  The motor argument passed to the function needs to be a pointer to a string.
  All of the settable firmware properties will be loaded into RAM.

  \param motor Specifies which motor's properties to load
  \retval int Returns 0 on success and a BHERR error code on failure
*/
int	BHandSupervisoryRealtime::Load(const char* motor)
{
	return driverIsOpen() ? m_driver->Load(motor) : BHERR_OPENCOMMPORT;
}


/** Sends the "Save" command to the hand.

  Saves present motor properties from the active properties list to EEPROM.
  These values can be loaded later. Storing the values in EEPROM allows you to
  reset the BarrettHand and retain preferred motor properties.

  Example:
  \code
  // saves the grasp motor properties
  char motor[2] = "G";
  err = bh.Save(motor);
  \endcode

  The properties can be recalled into the active properties list by using the
  function Load(). The motor argument passed to the function needs to be a
  pointer to a string. However, this command should not be performed more than
  5,000 times or the Hand electronics may need repair.

  \param motor Specifies which motor's properties to save
  \retval int Returns 0 on success and a BHERR error code on failure
*/
int	BHandSupervisoryRealtime::Save(const char *motor)
{
	return driverIsOpen() ? m_driver->Save(motor) : BHERR_OPENCOMMPORT;
}


/** Sends the "Temperature" request command to the hand.

  Returns temperature from the BarrettHand.  For the 262 hand, this is CPU
  temperature and for the 280 hand it is the maximum temperature read from any
  of the Pucks.

  Example:
  \code
  // stores the temperature in result
  int result;
  err = bh.Temperature(&result);
  \endcode

  \param result A pointer to where the temperature value will be stored (in degrees Celsius)
  \retval int Returns 0 on success and a BHERR error code on failure
*/
int	BHandSupervisoryRealtime::Temperature(int *result)
{
	return driverIsOpen() ? m_driver->Temperature(result) : BHERR_OPENCOMMPORT;
}


///////////////////////////////////////////////////////////////////////////
// More methods in Supervisory Module (Misc.)
///////////////////////////////////////////////////////////////////////////

void allocate(COMMAND_RESULT *cResult)
{
	if (cResult)
	{
		// Allocatate space for string in command result
		cResult->result = new char[MAX_COMMAND_RESULT_LENGTH];
		cResult->result[0] = 0;
	}
}


/** Sends a command to the hand and is able to receive a response.

  Send an ASCII character string to the BarrettHand. The hand is expected to
  respond in Supervisory mode. If the receive buffer is supplied, the function
  will copy the hand response into the buffer. If not and you are using serial,
  you can obtain the hand response using the command Response().

  Example:
  \code
  // gets maximum close velocity of motor F3 and stores
  // the resultant string value in receive
  char command[10] = "3FGET MCV";
  char receive[10];
  err = bh.Command(command, receive);
  \endcode

  This function can be used to implement a simple terminal control. The command
  argument passed to the function needs to be a pointer to a string. The
  receive buffer for the response must be allocated by the user and be large
  enough to hold the response for the sent command.

  \param send String to send to the BarrettHand (any variation of letters and numbers)
  \param receive Pointer to a buffer where the response will be stored
  \retval int Returns 0 on success and a BHERR error code on failure
*/
int	BHandSupervisoryRealtime::Command(const char *send, char *receive)
{
	if (!driverIsOpen())
		return BHERR_OPENCOMMPORT;

#ifdef BH8_262_HARDWARE
	BHandHardware *hw = m_bhand->getHardwareDesc();

	// If 262 then send commands directly to the hand and ignore validating command parameters
	if (strcmp(hw->getModelNumber(), BHAND_BH8_262_MODEL_NUM) == 0)
		return m_driver->Command(send, receive);
#endif

	// Hand must be a 280
	//printf("Command(%s)\n", send);

	if (receive != NULL)
		*receive = 0; // NULL terminate receive buffer

	// Cannot send a command if it is too long
	if (strlen(send) + 1 >= MAX_CMD_LEN)
	{
		//printf("Command very long...\n");
		// Then send the command directly
		return m_driver->Command(send, receive);
	}

	//printf("Command NULL terminated receive buffer\n");

	// Local storage for command and command result
	COMMAND command;
	COMMAND_RESULT command_result;

	// Initialize variables for result
	COMMAND_RESULT *cResult = &command_result;
	cResult->result = NULL;

	// Copy send string into command buffer
	strcpy(command.command, send);

	//printf("Command parseInput(COMMAND(%s))\n", command.command);

	// Parse command (with code from btclient\src\btdiag that needs to be merged with Puck code)
	parseInput(&command, cResult);

	// Copy the results of parsing command to local variables
	int            cmd = command_result.command;       // parsed command
	BHMotors  bhMotors = command_result.bhMotors;      // optional parsed motors included
	//int   puckProperty = command_result.p;             // optional parsed property
	bool valueIncluded = command_result.valueIncluded; // optional parsed value included for commands such as step open/close
	int          value = command_result.value;         // optional parsed value

	//if (cResult->result != NULL)
	//	printf("Command parseInput: %s\n", cResult->result);
	//else
	//	printf("Command parseInput:\n");

	//printf("        cmd           = %d\n", cmd);
	//printf("        bhMotors      = %d\n", bhMotors);
	//printf("        puckProperty  = %d\n", puckProperty);
	//printf("        valueIncluded = %d\n", valueIncluded);
	//printf("        value         = %d\n", value);

	// Create for motor string for commands
	char motors[10];
	toMotorChar(bhMotors, motors);

	//printf("Command after toMotorChar(%d) returned %s\n", bhMotors, motors);

	// Call methods in hand driver
	switch (cmd)
	{
		// Init, Calibration, Reset, etc.
		case CMD_HI:    {             return InitHand(motors); }
		case CMD_RESET: { bool reset; return Reset(&reset); }

		// Movement commands
		case CMD_C:    { return Close(motors); }
		case CMD_O:    { return Open(motors);  }

		case CMD_M:    { return valueIncluded ? GoToPosition(motors, value) : GoToDefault(motors); }
		case CMD_HOME: { return GoToHome(motors); }

		case CMD_IC:   { return valueIncluded ? StepClose(motors, value) : StepClose(motors); }
		case CMD_IO:   { return valueIncluded ? StepOpen(motors, value)  : StepOpen(motors);  }

		case CMD_TC:   { return TorqueClose(motors); break; }
		case CMD_TO:   { return TorqueOpen(motors);  break; }

		// Motor and global property Get/Set commands
		case CMD_SET: { return Set(motors, command_result.propertyName, value); } // FSET, PSET
		case CMD_GET:
			{
				// Check if this is a global property
				unsigned int nResults = countMotors(bhMotors); //p->getNumExtendedAttributes();
				//printf("Command Get nResults = %d\n", nResults);

				// Allocate space for result(s)
				int *results = new int[nResults];

				//printf("  In command calling Get(%s, %s, results)\n", motors, propName);
				//printf("Command Get multiple\n");
				Get(motors, command_result.propertyName, results);

				// If return results immediately then create string containing results
				char *resultString = receive;
				char propValueString[64];
				for (unsigned int i = 0; i < nResults; i++)
				{
					sprintf(propValueString, "%d ", results[i]);
					strcat(resultString, propValueString);
				}

				// Remove space character at the end of string
				if (strlen(resultString) > 0)
					resultString[strlen(resultString) - 1] = 0;

				//printf("Command Get results concatenated: %s\n", resultString);

				// Free space allocated for result(s)
				delete[] results;

				return 0;

			} // FGET, PGET

		case CMD_LOAD: { return Load(motors);    } // FLOAD, PLOAD
		case CMD_SAVE: { return Save(motors);    } // FSAVE, PSAVE
		case CMD_DEF:  { return Default(motors); } // FDEF, PDEF

		// FLIST, FLISTV
		// PLIST, and PLISTV

		case CMD_T:    { return StopMotor(motors); }

		// RealTime commands
		case CMD_LOOP: { return RTStart(motors); }

		default:
			{
				// Drop this command - don't try to handle it
				return BHERR_NOTCOMPLETED;
			}
/*
		// Administrative Commands
		case CMD_HELP: { break; } // TODO: set the appropriate help string for "pucks in hand"
		case CMD_ERR:  { break; } // TODO: Handle this command
		case CMD_VERS: { break; } // TODO: Handle this command

		// Advanced Commands
		// A?, FLISTA, FLISTAV, PLISTA, PLISTAV
		*/
	}
}


/** Sends the "Delay" command to the hand.

  Insert a delay into sequence of commands.

  Example:
  \code
  // Inserts a delay of 3 seconds
  unsigned int time = 3000;
  err = bh.Delay(time);
  \endcode

  \param msec The desired delay in units of milliseconds
  \retval int Returns 0 on success and a BHERR error code on failure
*/
int BHandSupervisoryRealtime::Delay(unsigned int msec)
{
	if (!driverIsOpen())
		return BHERR_OPENCOMMPORT;

	// Just in case the user cast a negative number to an int
	if (((int)msec) <= 0)
		return BHERR_OUTOFRANGE;

	return m_driver->Delay(msec);
}


/** Sends the "Stop Motor" command to the hand.

  Stops actuating motors specified.

  Example:
  \code
  // stops actuating the spread motor
  char motor[2] = "S";
  err = bh.StopMotor(motor);
  \endcode

  Use StopMotor() command, when possible, to reduce the amount of heat
  generated by the motor.

  \param motor Specifies which motors will be terminated
  \retval int Returns 0 on success and a BHERR error code on failure
*/
int	BHandSupervisoryRealtime::StopMotor(const char *motor)
{
	return driverIsOpen() ? m_driver->StopMotor(motor) : BHERR_OPENCOMMPORT;
}


#ifdef BH8_262_HARDWARE

/** Gets a pointer to the receive buffer for the hand.

  Provides read access to the receive buffer. The buffer will contain the
  response for the last command sent to the BarrettHand.

  \retval const char * Returns a pointer to the receive buffer
*/
const char* BHandSupervisoryRealtime::Response()
{
	return serialDriverIsOpen() ? ((BHandSerialDriver *)m_driver)->Response() : NULL;
}


/** Gets a pointer to the transmit buffer for the hand.

  Provides read access to the transmit buffer.

  \retval const char * Returns a pointer to the transmit buffer
*/
const char* BHandSupervisoryRealtime::Buffer()
{
	return serialDriverIsOpen() ? ((BHandSerialDriver *)m_driver)->Response() : NULL;
}

#endif


/** Sends "Baud" command to the Barrett Hand.

  Changes the baud rate of both the associated hand and the COM port on the
  host PC to the new value. The possible values are the standard baud rates
  up to 38400.

  Example:
  \code
  // sets hand and serial port to 9600 baud
  unsigned int baudrate = 9600;
  err = bh.Baud(baudrate);
  \endcode

  The baud rate of the hand is reset to the default baud rate by issuing the
  Reset() command.  Baud rate can be saved between resets by executing the
  "PSAVE" command after the Baud command.  Examples and demos work only with
  the hand starting up at 9600 bps so saving higher baud rates should only
  be done by experienced users.

  \param newbaud The desired baud rate should be stored in this variable
  \retval int Returns 0 on success and a BHERR error code on failure
*/
int	BHandSupervisoryRealtime::Baud(unsigned int newbaud)
{
	return driverIsOpen() ? m_driver->Baud(newbaud) : BHERR_OPENCOMMPORT;
}




///////////////////////////////////////////////////////////////////////////////
// RealTime mode commands
///////////////////////////////////////////////////////////////////////////////


/** Sends "Start" RealTime Mode Command to the Barrett Hand.

  Call this function after desired loop mode properties have been set and you
  are ready to enter RealTime control.  Only one active motor control mode may
  be active or RTStart may return an error (e.g. don't try to control motor
  torques and velocities at the same time).

  Example:
  \code
  // Enter motor F2 into RealTime control
  char motor[2] = "2";
  err = bh.RTStart(motor);
  \endcode

  The motor argument passed to the function needs to be a pointer to a string.
  Motor protection is an optional argument that is used to set the desired
  motor protection level for the 280 hand only.  The default value is
  BHMotorTSTOPProtect.  This will protect motors from overheating and damage if
  a motor comes to a stop after TSTOP milliseconds.  The motor mode will be
  returned to idle mode to protect the motor so RTAbort and RTStart will need
  to be called again to regain control of the stopped motor(s).  Another option
  available is to set the motorProtection argument to
  BHMotorTorqueLimitProtect.  This will limit the maximum torque to motors to a
  safe amount during RTUpdate calls.  It is important that RTUpdate is called
  frequently or a stalled motor may not have its torque limited.  If a user
  program is not calling RTUpdate then motors may overheat and be damaged.

  \param motor Determines which motors will be controlled in RealTime
  \param motorProtection See above for detailed description
  \retval int Returns 0 on success and a BHERR error code on failure
*/
int	BHandSupervisoryRealtime::RTStart(const char *motor, BHMotorProtection motorProtection)
{
	return driverIsOpen() ? m_driver->RTStart(motor, motorProtection) : BHERR_OPENCOMMPORT;
}


/** Sends "Update" RealTime Mode Command to the Barrett Hand.

  This command is used to trigger the sending and receiving of data between the
  host PC and the Hand.

  Example:
  \code
  // Set Velocity to 30 and read position for motor 2, stop when position > 3000
  char motor[2] = "2";
  char parameter[2] = "P";
  int pos[1];

  err = bh.RTSetFlags(motor, TRUE, 1, FALSE, FALSE, 1,
        FALSE, TRUE, FALSE, 1, TRUE, FALSE);
  err = bh.Get(motor, parameter, pos);
  err = bh.RTStart(motor);

  while (pos[1] < 3000)
  {
     err = bh.RTSetVelocity(`2`, 30);
     err = bh.RTUpdate(TRUE, TRUE);
     pos = bh.RTGetPosition(`2`);
  }
  \endcode

  \param control Indicates if control data should be sent
  \param feedback Indicates if feedback data should be received
  \retval int Returns 0 on success and a BHERR error code on failure
*/
int BHandSupervisoryRealtime::RTUpdate(bool control, bool feedback)
{
	return driverIsOpen() ? m_driver->RTUpdate(control, feedback) : BHERR_OPENCOMMPORT;
}


/** Sends "Abort" RealTime Mode Command to the Barrett Hand.

  Ends RealTime mode and returns to Supervisory mode.

  Example:
  \code
  // Ends RealTime control
  err = bh.RTAbort();
  \endcode

  \retval int Returns 0 on success and a BHERR error code on failure
*/
int	BHandSupervisoryRealtime::RTAbort()
{
	return driverIsOpen() ? m_driver->RTAbort() : BHERR_OPENCOMMPORT;
}

/** This method will "Update" Puck properties while in RealTime Mode.

  This command will efficiently request property values from the given
  Pucks and wait until the properties have been read and stored in the array
  passed to this method.  It is provided to be able to read properties in
  between calls to RTUpdate(bool, bool).

  Example:
  \code
  // Set Velocity to 30 and read position for motor F2, stop when position > 3000
  char motors[2] = "G";
  char prop[] = "JP";
  int jointPositions[4]; // must be of the number of motors in the hand

  ...

  while (1)
  {
     ...
     err = bh.RTUpdate(TRUE, TRUE);
	 err = bh.RTUpdate(motors, prop, jointPositions);
  }
  \endcode

  The size of the array passed into this method must be equal to the number of
  motors in the hand.  Ensure that the array is large enough to hold values for
  each motor of the hand even if not all motor property values are read.

  \param motor Determines which motor properties to retrieve in RealTime mode
  \param property The name of the Puck 2 property
  \param values Pointer to an array that will contain retrieved results
  \retval int Returns 0 on success and a BHERR error code on failure
*/
int BHandSupervisoryRealtime::RTUpdate(const char *motor, const char *property, int *values)
{
	return driverIsOpen() ? m_driver->RTUpdate(motor, property, values) : BHERR_OPENCOMMPORT;
}

/** Sends "Set Flags" RealTime Mode Command to the Barrett Hand.

  Sets the nine of the parameters relevant for RealTime mode, for the specified
  motors. See BH8-Series User Manual for a list of relevant RealTime Parameters
  and their functions.

  Example:
  \code
  // Prepares flags and variables to send control velocity
  // and receive absolute position to/from motor F2
  bool control_velocity_flag = TRUE,
       control_propgain_flag = FALSE,
       feedback_velocity_flag = FALSE,
       feedback_strain_flag = FALSE,
       feedback_position_flag = TRUE,
       feedback_deltapos_flag = FALSE;

  int control_velocity_coefficient = 1;
  int feedback_velocity_coefficient = 1;
  int feedback_delta_position_coefficient = 1;

  char motor[2] = "2";

  err = bh.RTSetFlags(motor,
                      control_velocity_flag, control_velocity_coefficient, control_propgain_flag,
                      feedback_velocity_flag, feedback_velocity_coefficient, feedback_strain_flag,
                      feedback_position_flag, feedback_deltapos_flag, feedback_delta_position_coefficient);
  \endcode

  This function is provided for convenience, the same effect can be achieved
  with multiple calls to the Set() function. However, RTSetFlags can only
  define some of the parameters that can be defined with Set().
  The motor argument passed to the function needs to be a pointer to a string.

  \param motor Determines which motor parameters will be set
  \param LCV Loop Control Velocity Flag
  \param LCVC Loop Control Velocity Coefficient
  \param LCPG Loop Control Proportional Gain Flag
  \param LFV Loop Feedback Velocity Flag
  \param LFVC Loop Feedback Velocity Coefficient
  \param LFS Loop Feedback Stain Flag
  \param LFAP Loop Feedback Absolute Position Flag
  \param LFDP Loop Feedback Delta Position Flag
  \param LFDPC Loop Feedback Delta Position Coefficient
  \retval int Returns 0 on success and a BHERR error code on failure
*/
int	BHandSupervisoryRealtime::RTSetFlags(const char *motor, bool LCV, int LCVC, bool LCPG,
                      bool LFV, int LFVC, bool LFS, bool LFAP, bool LFDP, int LFDPC)
{
	/*
	if (!driverIsOpen)
		return BHERR_OPENCOMMPORT;

	// Validate input parameters
	if (validate("LCVC", LCVC)) return BHERR_OUTOFRANGE;
	if (validate("LFVC", LFVC)) return BHERR_OUTOFRANGE;
	if (validate("LFDPC", LFDPC)) return BHERR_OUTOFRANGE;

	// Call method provided by driver
	return m_driver->RTSetFlags(motor, LCV, LCVC, LCPG,
		LFV, LFVC, LFS, LFAP, LFDP, LFDPC);
	*/

	return driverIsOpen() ? m_driver->RTSetFlags(motor, LCV, LCVC, LCPG,
		LFV, LFVC, LFS, LFAP, LFDP, LFDPC) : BHERR_OPENCOMMPORT;
}


/** Sends "Set Flags" RealTime Mode Command to the Barrett Hand.

  Sets the fourteen of the parameters relevant for RealTime mode, for the
  specified motors. See BH8-Series User Manual for a list of relevant RealTime
  Parameters and their functions.

  Example:
  \code
  // Prepares flags and variables to send control velocity
  // and receive absolute position and temperature to/from motor F2
  bool control_velocity_flag = TRUE,
       control_propgain_flag = FALSE,
       control_torque_flag = FALSE,
       feedback_velocity_flag = FALSE,
       feedback_strain_flag = FALSE,
       feedback_position_flag = TRUE,
       feedback_deltapos_flag = FALSE,
       feedback_breakaway_position_flag = FALSE,
       feedback_analog_input_flag = FALSE,
       feedback_delta_position_discard_flag = FALSE,
       feedback_temperature = TRUE;

  int control_velocity_coefficient = 1;
  int feedback_velocity_coefficient = 1;
  int feedback_delta_position_coefficient = 1;

  char motor[2] = "2";

  err = bh.RTSetFlags(motor,
                      control_velocity_flag, control_velocity_coefficient, control_propgain_flag, control_torque_flag,
                      feedback_velocity_flag, feedback_velocity_coefficient, feedback_strain_flag,
                      feedback_position_flag, feedback_deltapos_flag, feedback_delta_position_coefficient,
                      feedback_breakaway_position_flag, feedback_analog_input_flag, feedback_delta_position_discard_flag, feedback_temperature);
  \endcode

  This function is provided for convenience, the same effect can be achieved
  with multiple calls to the Set() function. However, RTSetFlags can only
  define some of the parameters that can be defined with Set().
  The motor argument passed to the function needs to be a pointer to a string.

  \param motor Determines which motor parameters will be set
  \param LCV Loop Control Velocity Flag
  \param LCVC Loop Control Velocity Coefficient
  \param LCPG Loop Control Proportional Gain Flag
  \param LCT Loop Control Torque Flag
  \param LFV Loop Feedback Velocity Flag
  \param LFVC Loop Feedback Velocity Coefficient
  \param LFS Loop Feedback Stain Flag
  \param LFAP Loop Feedback Absolute Position Flag
  \param LFDP Loop Feedback Delta Position Flag
  \param LFDPC Loop Feedback Delta Position Coefficient
  \param LFBP Loop Feedback Breakaway Position Flag
  \param LFAIN Loop Feedback Analog Input Flag
  \param LFDPD Loop Feedback Delta Position Discard Flag
  \param LFT Loop Feedback Temperature Flag
  \retval int Returns 0 on success and a BHERR error code on failure
*/
int	BHandSupervisoryRealtime::RTSetFlags(const char *motor, bool LCV, int LCVC, bool LCPG, bool LCT,
                      bool LFV, int LFVC, bool LFS, bool LFAP, bool LFDP, int LFDPC,
                      bool LFBP, bool LFAIN, bool LFDPD, bool LFT)
{
	return driverIsOpen() ? m_driver->RTSetFlags(motor, LCV, LCVC, LCPG, LCT,
		LFV, LFVC, LFS, LFAP, LFDP, LFDPC, LFBP, LFAIN, LFDPD, LFT) : BHERR_OPENCOMMPORT;
}


/** Gets RealTime velocity feedback for the desired motor.

  Example:
  \code
  // Get velocity feedback of motor F1
  char velocity = bh.RTGetVelocity(`1`);
  \endcode

  BH8-262 velocity feedback is divided by the loop feedback velocity
  coefficient (LFVC) before it is sent by the hand.  This value can be set with
  the RTSetFlags method.  The returned velocity units are encoder ticks per 10
  milliseconds and will be scaled accordingly with a LFVC greater than 1.

  BH8-280 velocity feedback is in encoder ticks per millisecond.  Velocity
  feedback with this method will be clipped to limit the range to be from -127
  to 127.

  The loop feedback velocity (LFV) flag must be set to receive velocity
  feedback.  Only one motor velocity can be retrieved at a time.

  \param motor Determines which motor velocity will be retrieved
  \retval int Returns the velocity for the specified motor
*/
char BHandSupervisoryRealtime::RTGetVelocity(const char motor)
{
	return driverIsOpen() ? m_driver->RTGetVelocity(motor) : 0;
}


/** Gets RealTime strain gauge feedback for the desired motor.

  Example:
  \code
  // Gets strain gauge value for motor 3
  unsigned char strain = bh.RTGetStrain(`3`);
  \endcode

  This method may only return 8-bit values so it will scale strain readings
  to be from 0 to 255.  Better resolution can be achieved for BH8-280 users
  by calling RTUpdate with the "SG" property name passed as a parameter.

  The loop feedback strain (LFS) flag must be set to receive strain gauge
  feedback if using just the RTUpdate(bool, bool) method.  Only one motor
  strain gauge reading can be retrieved at a time.

  \param motor Determines which finger strain gauge values will be retrieved
  \retval unsigned char Returns the strain gauge value for the specified motor
*/
unsigned char BHandSupervisoryRealtime::RTGetStrain(const char motor)
{
	return driverIsOpen() ? m_driver->RTGetStrain(motor) : 0;
}


/** Gets RealTime absolute position feedback for the desired motor.

  Example:
  \code
  // Gets absolute position for motor 1
  unsigned char position = bh.RTGetPosition(`1`);
  \endcode

  The loop feedback absolute position (LFAP) flag must be set to receive
  absolute position feedback.  The hand also needs to be intialized with the HI
  command.  Only one motor position can be retrieved at a time.

  \param motor Determines which motor's absolute position will be retrieved
  \retval int Returns the absolute position of the motor
*/
int BHandSupervisoryRealtime::RTGetPosition(const char motor)
{
	return driverIsOpen() ? m_driver->RTGetPosition(motor) : 0;
}


/** Gets RealTime temperature feedback from the Barrett Hand.

  Example:
  \code
  // Get temperature
  int handtemperature = bh.RTGetTemp();
  \endcode

  The global loop feedback temperature (LFT) flag must be set to receive
  temperature feedback.  BH8-262 temperature is returned as stated in the user
  manual.  BH8-280 temperature will return the maximum of the present TEMP
  readings from the Pucks.

  \retval int Returns the temperature value
*/
int BHandSupervisoryRealtime::RTGetTemp()
{
	return driverIsOpen() ? m_driver->RTGetTemp() : 0;
}


/** Gets RealTime analog input feedback from the Barrett Hand.

  Example:
  \code
  // Get analog input value for finger F1
  int analog1 = bh.RTGetAIN(`1`);
  \endcode

  The loop feedback analog input (LFAIN) flag must be set to receive analog
  input feedback.  This is will only work on the BH8-262 hand.

  \param motor Determines which motor's analog value will be retrieved
  \retval unsigned char Returns the analog input value for the specified motor
*/
unsigned char BHandSupervisoryRealtime::RTGetAIN(const char motor)
{
	return driverIsOpen() ? m_driver->RTGetAIN(motor) : 0;
}


/** Gets RealTime delta position value feedback for the specified motor.

  Example:
  \code
  // Get delta position value for finger F1
  int deltaposition1 = bh.RTGetDeltaPos(`1`);
  \endcode

  The loop feedback delta position (LFDP) flag must be set to receive delta
  position feedback.  Delta position is the change in position from the last
  reported position and is limited to one signed byte. The present position is
  read and compared to the last reported position. The difference is divided by
  the RealTime variable LFDPC, clipped to a single signed byte, and then sent
  to the host.  The value sent to the host should then be multiplied by LFDPC
  and added to the last reported position.

  Example (with LFDPC set to 2): What will delta position feedback look like if
  last reported position was 1500 and the position jumps to 2000? The first
  feedback block will include the delta position value 127. This value should
  be multiplied by LFDPC on the host machine resulting in 254. The hand will
  internally update the reported position to 1754. The next feedback block will
  include the delta position 123, which should be multiplied by LFDPC resulting
  in 246. The reported position will be updated to 2000. Subsequent feedback
  blocks will include the delta position value 0 (until the next position
  change).

  Delta position feedback is only implemented on the BH8-262 hand in order to
  increase the servo rate.  Only one motor delta position can be retrieved at
  a time.

  \param motor Determines which motor's delta position value will be retrieved
  \retval char Returns the delta position value for the specified motor
*/
char BHandSupervisoryRealtime::RTGetDeltaPos(const char motor)
{
	return driverIsOpen() ? m_driver->RTGetDeltaPos(motor) : 0;
}

/** Gets RealTime breakaway position feedback from the Barrett Hand.

  Example:
  // Get breakaway position of finger 1
  int bp = bh.RTGetBreakawayPosition(`1`);
  \endcode

  The loop feedback breakaway position flag must be set to receive the
  breakaway positions.  Open, toque open, and hand initialize commands will
  reset the breakaway detected (BD) flag and clear the breakaway position (BP).
  Initialization hit count (IHIT) is used to get a consistent origin for finger
  motors and thus a consistent breakaway force.  Initialization Offset affects
  the force required to cause breakaway.  Properties that affect the breakaway
  position are the breakaway detection acceleration threshold (BDAT).
  Breakaway stop (BS) flag is used to stop a finger motor as soon as breakaway
  is detected.  The user is refered to the BH8-262 user manual for information
  concerning breakaway.  Only one motor breakaway position can be retrieved at
  a time.

*/
int BHandSupervisoryRealtime::RTGetBreakawayPosition(const char motor)
{
	return driverIsOpen() ? m_driver->RTGetBreakawayPosition(motor) : 0;
}

void BHandSupervisoryRealtime::RTGetPPS(const char motor, int *pps, int ppsElements)
{
	if (driverIsOpen())
		m_driver->RTGetPPS(motor, pps, ppsElements);
}


/** Sets RealTime control velocity reference for the desired motor.

  Example:
  \code
  // Set control velocity references for motors F1, F2, and F3 to 50
  err = bh.RTSetVelocity(`1`, 50);
  err = bh.RTSetVelocity(`2`, 50);
  err = bh.RTSetVelocity(`3`, 50);
  \endcode

  The loop control velocity (LCV) flag must be set to send velocity references
  to the hand.

  BH8-280 Puck motor controllers implement velocity mode on top of a position
  controller updated at a kilohertz.  Velocity references in units of encoder
  ticks/ms are added to the present commanded position each servo cycle.  KP,
  KI, and KD affect both position and velocity PID gains.

  BH8-262 motor controllers responds by applying motor torque according to the
  following equation in the HCTL-1100 datasheet:

     MCn = (K/4) * Yn

  Where K depends on the loop control proportional gain (LCPG) flag.  If LCPG
  is set then K is set with RTSetGain, otherwise K is equal to the value of the
  "FPG" property.  Yn is the velocity error and equals:

     Loop Control Velocity Reference * Loop Control Velocity Coefficient - Actual Velocity

  In proportional velocity control mode, the HCTL-1100 tries to match the
  desired motor control reference velocities by applying motor torques
  proportional to the velocity error.  The proportional gain "K" may be set at
  runtime.  The actual velocity is in units of encoder ticks per 10 milliseconds.

  The motor will not actually be sent this control velocity reference until
  RTUpdate() is called. Only one motor velocity reference can be set at a time.

  \param motor Determines which motor velocity reference will be set
  \param velocity Desired control velocity for the specified motor
  \retval int Returns 0 on success and BHERR_MOTORINACTIVE if motor is inactive
*/
int BHandSupervisoryRealtime::RTSetVelocity(const char motor, int velocity)
{
	return driverIsOpen() ? m_driver->RTSetVelocity(motor, velocity) : 0;
}


/** Sets RealTime control torque for the desired motor.

  Example:
  \code
  // Set control torque parameters of motors F1, F2, and F3 to 50
  err = bh.RTSetTorque(`1`, 50);
  err = bh.RTSetTorque(`2`, 50);
  err = bh.RTSetTorque(`3`, 50);
  \endcode

  The loop control torque (LCT) flag needs to be set to send 16-bit torque
  references to the hand.

  BH8-262 motor torque references will be used in position mode to apply motor
  torques.  The desired motor torque will be added to the present motor encoder
  position and be submitted as the commanded position.

  The BH8-280 will control motor currents directly by using the desired
  torque reference.

  The motor will not actually be set to this control torque until RTUpdate()
  is called. Only one motor control torque can be set at a time.

  \param motor Determines which motor torque will be set
  \param torque Desired control velocity for the specified motor
  \retval int Returns 0 on success and BHERR_MOTORINACTIVE if motor is inactive
*/
int BHandSupervisoryRealtime::RTSetTorque(const char motor, int torque)
{
	return driverIsOpen() ? m_driver->RTSetTorque(motor, torque) : 0;
}


/** Sets RealTime control proportional gain for the desired motor.

  Example:
  \code
  // Set gain parameters of motors 1 and 2 to 150
  err = bh.RTSetGain(`1`, 150);
  err = bh.RTSetGain(`2`, 150);
  \endcode

  This method should only be used for the BH8-262 hand.

  The loop control proportional gain (LCPG) flags must be set to send
  proportional gains to the hand.  The gains for the motors will not actually
  be set until RTUpdate() is called.  In RealTime control, the motors are
  controlled using a proportional velocity mode. The proportional gain affects
  the motor command according to the Velocity Control equations.  See
  RTSetVelocity method for more information.

  \param motor Determines which motor gain will be set
  \param gain Desired proportional gain for the specified motor
  \retval int Returns 0 on success and BHERR_MOTORINACTIVE if motor is inactive
*/
int BHandSupervisoryRealtime::RTSetGain(const char motor, int gain)
{
	return driverIsOpen() ? m_driver->RTSetGain(motor, gain) : 0;
}


/** Sets RealTime position reference for the desired motor.

  Example:
  \code
  // Set positions of F3 and spread to be in the center position
  err = bh.RTSetPosition(`3`, -100000);
  err = bh.RTSetPosition(`S`, -18000);
  \endcode

  RealTime position control is only possible with the 280 hand.

  The loop control position (LCP) flag needs to be set to send desired position
  references to the hand.  The new reference positions will not actually be set
  until RTUpdate() is called.  Only one motor control position reference can be
  set at a time.

  \param motor Determines which motor position will be set
  \param position Desired position for the specified motor
  \retval int Returns 0 on success and BHERR_MOTORINACTIVE if motor is inactive
 */
int BHandSupervisoryRealtime::RTSetPosition(const char motor, int position)
{
	return driverIsOpen() ? m_driver->RTSetPosition(motor, position) : 0;
}

