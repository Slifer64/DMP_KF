///////////////////////////////////////////////////////////////////////////////
//                                                                           //
//                   (C) Barrett Technology Inc. 1998-2010                   //
//                                                                           //
//    BHand Implementation version 4.4.3                                     //
//                                                                           //
///////////////////////////////////////////////////////////////////////////////

/** \file BHand.cpp

    \brief Contains the implementation of a class for controlling a BarrettHand.

	The BarrettHand C++ API provides declarations of different classes that
	provides everything necessary for controlling a BarrettHand with your
	programs.  An instance of BHand must be created and the Init method
	contained within BHandSupervisoryRealTime must be called.

    See #BHandSupervisoryRealTime for more information.
*/

#include "BHand.h"

#include "BHandCANDriver.h"
#ifdef BH8_262_HARDWARE
#include "BHandSerialDriver.h"
#endif

///////////////////////////////////////////////////////////////////////////////
//    Text Descriptions for BarrettHand API Error Messages                   //
//    Note: API error codes are negative                                     //
//          Error codes from the hand are positive                           //
///////////////////////////////////////////////////////////////////////////////

const int BHAND_NUM_ERROR_MESSAGES = 22;

/** BarrettHand API Error Messages
	Brief error messages for negative value BarrettHand API error codes
 */
const static char _ErrorMessage[BHAND_NUM_ERROR_MESSAGES][80] = {
	"No error",
	"An instance of class BHand has already been initialized",
	"Open comm port failed",
	"Get comm port state failed",
	"Set comm port state failed",
	"Set comm timeouts failed",
	"Unable to start hand i/o thread",
	"Unable to set hand i/o thread priority",
	"Error writing to comm port (or write timeout)",
	"Error reading from comm port (or read timeout)",
	"Hand sent incorrect sequence of characters - possible loss of data",
	"Clear comm buffer failed",
	"Request to hand i/o thread timed out",
	"Processing of previous request is not completed",
	"Request pending - in asynch mode",
	"No instances of class BHand have been initialized",
	"Unrecognized parameter name in call to Set",
	"String too long",
	"Parameter value out of range (0-30000)",
	"Specified motor was not activated in RTStart",
	"Hand error - check hand manual",
	"Invalid error number"
};


#ifndef LINUX
unsigned int BHand::m_handCount = 0;
#endif

///////////////////////////////////////////////////////////////////////////
// Public Methods (BarrettHand Constructor/Deconstructor)
///////////////////////////////////////////////////////////////////////////

/** Constructor for the BarrettHand object.

  There should be one BHand instance created for each BarrettHand being
  controlled.  This constructor initializes the BHand instance as required
  before any communication with the hardware takes place.
*/
BHand::BHand() : BHandSupervisoryRealtime(this),
                 m_driver(NULL), m_hardwareDesc(NULL)
{
	// Create Hardware Descriptions for all BarrettHand models known at compile time
	bhandCreateHardwareDesc();

	// Note: setHardwareDesc should be called to initialize the hardware
	// description for the hand.  It is called right away so that a GUI can
	// display a property list before connecting to a hand.

	// If the hand API is compiled for both hands, the user must set the hardware
	// description by hand to be a 262 hand if using a 262 hand
/*	unsigned int bHWIndex;
#ifdef BH8_280_HARDWARE
	bHWIndex = BHandHardware::getBHandHardwareIndex("BH8-280");
#else
	bHWIndex = BHandHardware::getBHandHardwareIndex("BH8-262");
#endif

	setHardwareDesc(bHWIndex);*/

	// Assume that there is a BarrettHand Hardware Description available
	// and hardcode it to be the first one found
	setHardwareDesc(0);

#ifndef LINUX
	if (++m_handCount == 1)
	{
		// Set resolution to the minimum supported by the system
		TIMECAPS tc;
		timeGetDevCaps(&tc, sizeof(TIMECAPS));
		m_timerRes = MIN(MAX(tc.wPeriodMin, 0), tc.wPeriodMax);
		timeBeginPeriod(m_timerRes);
	}
#endif
}


/** Deconstructor for the BarrettHand object.

  Stops the thread for serial communication and closes the serial
  communications port for a BHand instance.
*/
BHand::~BHand()
{
#ifndef LINUX
	if (--m_handCount == 0)
	{
		// reset timer resolution
		timeEndPeriod(m_timerRes);
	}
#endif

	// Release BarrettHand Hardware Descriptions
	bhandDestroyHardwareDesc();
}


///////////////////////////////////////////////////////////////////////////////
// Public Methods for BarrettHand Device Drivers
///////////////////////////////////////////////////////////////////////////////

/** Sets the device driver for this instance of the BarrettHand.

  \internal

  \param driver Device driver used for communication with the BarrettHand
  \retval int Error code that is returned will be zero if successful
*/
int BHand::setDeviceDriver(BHandDriver *driver)
{
	if (m_driver == NULL && driver != NULL)
	{
		m_driver = driver;
		return 0;
	}
	else
	{
		return -1; // non-zero
	}
}


/** Gets the device driver for this instance of the BarrettHand.

  \param driver Device driver used for communication with the BarrettHand
  \retval int Pointer to instance of the Device Driver use by this instance
*/
BHandDriver * BHand::getDeviceDriver()
{
	return m_driver;
}

///////////////////////////////////////////////////////////////////////////
// Public Methods for Access to Descriptions of BarrettHand Hardware
///////////////////////////////////////////////////////////////////////////

/** Gets the number of available BarrettHand model numbers.

  \retval "unsigned int" Number of available BarrettHand model numbers
*/
unsigned int BHand::getHardwareNumModels()
{
	return bhandGetHardwareNumModels();
}

/** Gets a Hardware Description of the given BarrettHand.

  \param hwIndex Device driver used for communication with the BarrettHand
  \retval BHandHardware Pointer to the BarrettHand Hardware Description
*/
BHandHardware * BHand::getHardwareDesc(unsigned int hwIndex)
{
	return bhandGetHardwareDesc(hwIndex);
}

/** Sets a Hardware Description for the BarrettHand.

  \param hwIndex Hardware Description for the BarrettHand
  \retval int Error code that is returned will be zero if successful
*/
int BHand::setHardwareDesc(unsigned int hwIndex)
{
	if (hwIndex < getHardwareNumModels())
		m_hardwareDesc = getHardwareDesc(hwIndex);
	else
		return -1; // failed to set BarrettHand Hardware Description

	return 0;
}

/** Gets a Hardware Description of the BarrettHand.

  \retval BHandHardware Pointer to the BarrettHand Hardware Description
*/
BHandHardware * BHand::getHardwareDesc()
{
	return m_hardwareDesc;
}


///////////////////////////////////////////////////////////////////////////
// Public Methods (Misc.)
///////////////////////////////////////////////////////////////////////////

/** Sets the default serial communication baud rate used by all new BHand instances

  Although users may desire setting higher baud rates for better RealTime
  performance, most users will not need to do this.  If the global BAUD
  property is permanently set to a value other than 96 (96 => 9600 bps) then
  SetDefaultBaud must be called to communicate with the hand at the correct
  baud rate.

  Beware of permanently changing the BAUD property.  The GUI is capable of
  being configured to connect to the hand with baud rates other than 9600 bps.
  The examples and demos will NOT work unless hand BAUD property is 9600 baud
  at startup.  So it is better to connect to the hand and then increase
  the baud rate using the Baud method.  Note that not all hands are capable of
  higher baud rates.

  \param baud Only pass in values of 9600, 19200, or 38400

 */
void BHand::SetDefaultBaud(unsigned int baud)
{
#ifdef BH8_262_HARDWARE
	BHandSerialDriver::SetDefaultBaud(baud);
#endif
}


/** Get an error message for a BHand error code.

  Gives a brief error message that describes a BHand error code.

  \param err An error code
  \retval const char* Pointer to a character array describing error code.
*/
const char* BHand::ErrorMessage(int err)
{
	return (err > 0) ? _ErrorMessage[20] : _ErrorMessage[(err > -20) ? -err : 21];
}
