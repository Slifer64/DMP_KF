/* ========================================================================== *
 *  File ............... BHand.h                                              *
 *  Creation Date ...... Dec 1998                                             *
 *  Last Updated ....... Oct 2010                                             *
 *  Revision ........... 4.4.3                                                *
 *  Authors ............ Emanuel Todorov                                      *
 *                       Brian Zenowich                                       *
 *                       Edward Hannigan                                      *
 *                                                                            *
 *  ************************************************************************  *
 *                                                                            *
 *  Copyright (C) 1998-2010 Barrett Technology, Inc. <support@barrett.com>    *
 *                          625 Mount Auburn St                               *
 *                          Cambridge, MA 02138, USA                          *
 *                                                                            *
 *  All rights reserved.                                                      *
 *                                                                            *
 *  Redistribution and use in source and binary forms, with or without        *
 *  modification, are permitted provided that the following conditions        *
 *  are met:                                                                  *
 *                                                                            *
 *  1. Redistributions of source code must retain the above copyright         *
 *     notice, this list of conditions and the following disclaimer.          *
 *  2. Redistributions in binary form must reproduce the above copyright      *
 *     notice, this list of conditions and the following disclaimer in the    *
 *     documentation and/or other materials provided with the distribution.   *
 *                                                                            *
 *  THIS SOFTWARE IS PROVIDED BY BARRETT TECHNOLOGY, INC AND CONTRIBUTORS     *
 *  ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT       *
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS         *
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL BARRETT       *
 *  TECHNOLOGY, INC OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,       *
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,      *
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS     *
 *  OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND    *
 *  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR     *
 *  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE    *
 *  USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.  *
 *                                                                            *
 *  The views and conclusions contained in the software and documentation     *
 *  are those of the authors and should not be interpreted as representing    *
 *  official policies, either expressed or implied, of Barrett Technology.    *
 *                                                                            *
 * ==========================================================================*/


/** \file BHand.h

    \brief Contains the declaration of a class for controlling a BarrettHand.

	The BarrettHand C++ API provides declarations of different classes that
	provides everything necessary for controlling a BarrettHand with your
	programs.  An instance of BHand must be created and the Init method
	contained within BHandSupervisoryRealTime must be called.

    See #BHandSupervisoryRealTime for more information.
*/


#ifndef BHAND_H
#define BHAND_H


#include "BHandSupervisoryRealTime.h"
#include "BHandHardware.h"


/* There should be one instance of this class created for each BarrettHand.

BHand contains methods for serial communication with the Barrett Hand,
supports a supervise mode to execute high-level commands on the hand, and a
real time mode that lets users write their own control loops.  In Real Time
mode the desired control variables are sent to the hand and desired feedback
values are received from the hand.

Commands for supervise mode include initialize hand, open, close, etc. See
Supervisory.cpp for an example program how Supervise mode works.

Real time mode is entered for specified motors by calling #RTStart. A user may
set control and feedback parameters sent during a control loop update with
a call to #RTSetFlags. To perform a control loop update call #RTUpdate and to
go back to supervise mode call #RTAbort. See RealTime.cpp for an example
program to see how Real Time mode works.
*/
/** There should be one instance of this class created for each BarrettHand.

This class allows access to different BarrettHand Hardware Descriptions.  It
also contains the device driver for communication with the hand and inherits
BHandSupervisoryRealtime.  This inheritance means that all high-level
supervisory and RealTime control/feedback methods will be callable.
 */
class BHand : public BHandSupervisoryRealtime
{
public:

	///////////////////////////////////////////////////////////////////////////
	// Public Methods BarrettHand (Constructor/Deconstructor)
	///////////////////////////////////////////////////////////////////////////

	BHand();
	virtual ~BHand();


	///////////////////////////////////////////////////////////////////////////
	// Public Methods for BarrettHand Device Drivers
	///////////////////////////////////////////////////////////////////////////

	int setDeviceDriver(BHandDriver *driver);
	BHandDriver * getDeviceDriver();


	///////////////////////////////////////////////////////////////////////////
	// Public Methods for Access to Descriptions of BarrettHand Hardware
	///////////////////////////////////////////////////////////////////////////

	static unsigned int getHardwareNumModels();
	static BHandHardware * getHardwareDesc(unsigned int hwIndex);

	int setHardwareDesc(unsigned int hwIndex);
	BHandHardware * getHardwareDesc();


	///////////////////////////////////////////////////////////////////////////
	// Public Methods (Misc.)
	///////////////////////////////////////////////////////////////////////////

	static void SetDefaultBaud(unsigned int baud);

	static const char* ErrorMessage(int err);

private:

	BHandDriver *m_driver;
	BHandHardware *m_hardwareDesc;

#ifndef LINUX
	// For Timer
	static unsigned int m_handCount;
	unsigned int m_timerRes;
#endif
};


// pointer to hand corresponding to each port-1 (1 : BH_MAXPORT)
extern BHand* _BHandArray[];


#endif
