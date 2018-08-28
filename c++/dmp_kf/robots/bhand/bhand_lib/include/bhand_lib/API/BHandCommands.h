///////////////////////////////////////////////////////////////////////////////
//                   (C) Barrett Technology Inc. 2009-2010                   //
///////////////////////////////////////////////////////////////////////////////

#ifndef BHAND_COMMANDS_H
#define BHAND_COMMANDS_H

#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>

#include "bhand_motors.h"
#include "bhand_misc.h"
#include "PucksInHand.h"

/** \file BHandCommands.h

    \brief Contains the declaration and implementation for intermediate level representations of Supervisory commands and results.

    Barrett uses these classes internally for running supervisory commands.  It
	is possible to access the results of supervisory commands, which is very
	useful when running commands in non-blocking (asynchronous) mode.  This is
	an advanced feature of the BarrettHand API.
 */

//! Supervisory Command Constants
/*! Identifies the command in the base class (passed in the constructor) */
enum BHSupervisoryCommand
{
	BHSupervisoryInit = 0,
	BHSupervisoryReset,
	BHSupervisoryClose,
	BHSupervisoryOpen,
	BHSupervisoryTorqueClose,
	BHSupervisoryTorqueOpen,
	BHSupervisoryStepClose,
	BHSupervisoryStepOpen,
	BHSupervisoryGoToDefault,
	BHSupervisoryGoToDifferentPositions,
	BHSupervisoryGoToHome,
	BHSupervisoryGoToPosition,
	BHSupervisoryGet,
	BHSupervisorySet,
	BHSupervisoryPGet,
	BHSupervisoryPSet,
	BHSupervisoryDefault,
	BHSupervisoryLoad,
	BHSupervisorySave,
	BHSupervisoryTemperature,
	BHSupervisoryCmd,
	BHSupervisoryDelay,
	BHSupervisoryStopMotor,
	BHSupervisoryBaud
};

//! Supervisory Command Result Constants
/*! Identifies the command result in the base class (passed in the constructor) */
enum BHSupervisoryResult
{
	BHSupervisoryResultReset = 0,
	BHSupervisoryResultGet,
	BHSupervisoryResultPGet,
	BHSupervisoryResultTemperature,
	BHSupervisoryResultCmd
};


///////////////////////////////////////////////////////////////////////////////
//                                                                           //
//   Declarations for Supervisory BarrettHand commands and results           //
//                                                                           //
///////////////////////////////////////////////////////////////////////////////


class BHandSupervisoryCommand
{
public:

	BHandSupervisoryCommand(BHSupervisoryCommand command) : m_supervisoryCommand(command) {}
	virtual ~BHandSupervisoryCommand() {}

	BHSupervisoryCommand getCommand() { return m_supervisoryCommand; }

private:
	BHSupervisoryCommand m_supervisoryCommand;
};


class BHandSupervisoryResult
{
public:

	BHandSupervisoryResult(BHSupervisoryResult result) : m_supervisoryResult(result) {}
	virtual ~BHandSupervisoryResult() {}

private:
	BHSupervisoryResult m_supervisoryResult;
};


///////////////////////////////////////////////////////////////////////////////
//                                                                           //
//   Implementations for Supervisory BarrettHand commands and results        //
//                                                                           //
///////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////
// Commands in Supervisory Module (Init, Calibration, Reset, etc.)           //
///////////////////////////////////////////////////////////////////////////////


class BHandCommandInit : public BHandSupervisoryCommand
{
public:
	BHandCommandInit(BHMotors motors) :
		BHandSupervisoryCommand(BHSupervisoryInit),
		Motors(motors) {}
	virtual ~BHandCommandInit() {}

	BHMotors Motors;
};

///////////////////////////////////////////////////////////////////////////////

class BHandCommandReset : public BHandSupervisoryCommand
{
public:
	BHandCommandReset() : BHandSupervisoryCommand(BHSupervisoryReset), Motors(0xffffffff) {}
	virtual ~BHandCommandReset() {}

	BHMotors Motors;
};

/** This class is used for returning the response received status for a Reset command.

BHandResultReset contains a boolean flag used to determine if any response
has been received after issuing the reset command.
*/
class BHandResultReset : public BHandSupervisoryResult
{
public:
	BHandResultReset(bool responseReceived) : BHandSupervisoryResult(BHSupervisoryResultReset), m_responseReceived(responseReceived) {}
	virtual ~BHandResultReset() {}

	bool responseReceived() { return m_responseReceived; }

private:
	bool m_responseReceived;
};


///////////////////////////////////////////////////////////////////////////////
// More commands in Supervisory Module (Motor Movement)                      //
///////////////////////////////////////////////////////////////////////////////

class BHandCommandClose : public BHandSupervisoryCommand
{
public:
	BHandCommandClose(BHMotors motors) : BHandSupervisoryCommand(BHSupervisoryClose), Motors(motors) {}
	virtual ~BHandCommandClose() {}

	BHMotors Motors;
};

///////////////////////////////////////////////////////////////////////////////

class BHandCommandOpen : public BHandSupervisoryCommand
{
public:
	BHandCommandOpen(BHMotors motors) : BHandSupervisoryCommand(BHSupervisoryOpen), Motors(motors) {}
	virtual ~BHandCommandOpen() {}

	BHMotors Motors;
};

///////////////////////////////////////////////////////////////////////////////

class BHandCommandTorqueClose : public BHandSupervisoryCommand
{
public:
	BHandCommandTorqueClose(BHMotors motors) : BHandSupervisoryCommand(BHSupervisoryTorqueClose), Motors(motors) {}
	virtual ~BHandCommandTorqueClose() {}

	BHMotors Motors;
};

///////////////////////////////////////////////////////////////////////////////

class BHandCommandTorqueOpen : public BHandSupervisoryCommand
{
public:
	BHandCommandTorqueOpen(BHMotors motors) : BHandSupervisoryCommand(BHSupervisoryTorqueOpen), Motors(motors) {}
	virtual ~BHandCommandTorqueOpen() {}

	BHMotors Motors;
};

///////////////////////////////////////////////////////////////////////////////

class BHandCommandStepClose : public BHandSupervisoryCommand
{
public:
	BHandCommandStepClose(BHMotors motors, bool valueIncluded, int stepAmount) :
		BHandSupervisoryCommand(BHSupervisoryStepClose),
		Motors(motors), ValueIncluded(valueIncluded), StepAmount(stepAmount) {}
	virtual ~BHandCommandStepClose() {}

	BHMotors Motors;
	bool ValueIncluded;
	int StepAmount;
};

///////////////////////////////////////////////////////////////////////////////

class BHandCommandStepOpen : public BHandSupervisoryCommand
{
public:
	BHandCommandStepOpen(BHMotors motors, bool valueIncluded, int stepAmount) :
		BHandSupervisoryCommand(BHSupervisoryStepOpen),
		Motors(motors), ValueIncluded(valueIncluded), StepAmount(stepAmount) {}
	virtual ~BHandCommandStepOpen() {}

	BHMotors Motors;
	bool ValueIncluded;
	int StepAmount;
};

///////////////////////////////////////////////////////////////////////////////

class BHandCommandGoToDefault : public BHandSupervisoryCommand
{
public:
	BHandCommandGoToDefault(BHMotors motors, bool valueIncluded = false, int defaultPosition = 0) :
		BHandSupervisoryCommand(BHSupervisoryGoToDefault),
		Motors(motors), ValueIncluded(valueIncluded), DefaultPosition(defaultPosition) {}
	virtual ~BHandCommandGoToDefault() {}

	BHMotors Motors;
	bool ValueIncluded;
	int DefaultPosition;
};

///////////////////////////////////////////////////////////////////////////////

class BHandCommandGoToDifferentPositions : public BHandSupervisoryCommand
{
public:

	BHandCommandGoToDifferentPositions(const int *encoderPositions, unsigned int numEncoderPositions)
		:	BHandSupervisoryCommand(BHSupervisoryGoToDifferentPositions)
	{
		m_encoderPositions = new int[numEncoderPositions];

		for (unsigned int i = 0; i < numEncoderPositions; i++)
			m_encoderPositions[i] = encoderPositions[i];

		m_numEncoderPositions = numEncoderPositions;
	}

	virtual ~BHandCommandGoToDifferentPositions() { delete[] m_encoderPositions; }

	const int * getEncoderPositions() { return m_encoderPositions; }
	unsigned int getNumEncoderPositions() { return m_numEncoderPositions; }

private:
	int *m_encoderPositions;
	unsigned int m_numEncoderPositions;
};

///////////////////////////////////////////////////////////////////////////////

class BHandCommandGoToHome : public BHandSupervisoryCommand
{
public:
	BHandCommandGoToHome(BHMotors motors) : BHandSupervisoryCommand(BHSupervisoryGoToHome), Motors(motors) {}
	virtual ~BHandCommandGoToHome() {}

	BHMotors Motors;
};

///////////////////////////////////////////////////////////////////////////////

class BHandCommandGoToPosition : public BHandSupervisoryCommand
{
public:
	BHandCommandGoToPosition(BHMotors motors, unsigned int encoderPositionTickCount) : BHandSupervisoryCommand(BHSupervisoryGoToPosition), Motors(motors), EncoderPositionTickCount(encoderPositionTickCount) {}
	virtual ~BHandCommandGoToPosition() {}

	BHMotors Motors;
	unsigned int EncoderPositionTickCount;
};




///////////////////////////////////////////////////////////////////////////////
// More commands in Supervisory Module (Parameter Get/Set)                   //
///////////////////////////////////////////////////////////////////////////////

class BHandCommandSet : public BHandSupervisoryCommand
{
public:
	BHandCommandSet(BHMotors motors, const char *property, int value) : BHandSupervisoryCommand(BHSupervisorySet), Motors(motors), Value(value)
	{
		m_property = new char[strlen(property) + 1];
		strcpy(m_property, property);
	}
	virtual ~BHandCommandSet() { delete[] m_property; }

	const char * getProperty() { return m_property; }

	BHMotors Motors;
	int Value;

private:
	char *m_property;
};

///////////////////////////////////////////////////////////////////////////////

class BHandCommandPSet : public BHandSupervisoryCommand
{
public:
	BHandCommandPSet(const char *property, int value) : BHandSupervisoryCommand(BHSupervisoryPSet), Value(value)
	{
		m_property = new char[strlen(property) + 1];
		strcpy(m_property, property);
	}
	virtual ~BHandCommandPSet() {}

	const char * getProperty() { return m_property; }

	int Value;

private:
	char *m_property;
};

///////////////////////////////////////////////////////////////////////////////

class BHandCommandGet : public BHandSupervisoryCommand
{
public:
	BHandCommandGet(BHMotors motors, const char *property) : BHandSupervisoryCommand(BHSupervisoryGet), Motors(motors)
	{
		m_property = new char[strlen(property) + 1];
		strcpy(m_property, property);
	}
	virtual ~BHandCommandGet() { delete[] m_property; }

	const char * getProperty() { return m_property; }

	BHMotors Motors;

private:
	char *m_property;
};

///////////////////////////////////////////////////////////////////////////////

/** This class is used for returning the get response for a Get command.

BHandResultGet contains a one or more integer responses received after issuing
the get command.
*/
class BHandResultGet : public BHandSupervisoryResult
{
public:
	BHandResultGet(BHMotors motors, const int *results)
		: BHandSupervisoryResult(BHSupervisoryResultGet)
		{
			// Get number of results
			m_numResults = countMotors(motors);

			// Allocate space for results
			m_results = new int[m_numResults];
			for (unsigned i = 0; i < m_numResults; i++)
				m_results[i] = results[i];
		}
	BHandResultGet(BHMotors motors, const int *results, int numResults) // new for handling multiple "get" property commands on the 262
		: BHandSupervisoryResult(BHSupervisoryResultGet)
		{
			// Get number of results
			m_numResults = numResults;

			// Allocate space for results
			m_results = new int[m_numResults];
			for (unsigned i = 0; i < m_numResults; i++)
				m_results[i] = results[i];
		}
	virtual ~BHandResultGet()
	{
		delete[] m_results;
	}

	int getResult(unsigned int i) { return m_results[i]; }
	unsigned int getNumResults() { return m_numResults; }

private:
	int *m_results;
	unsigned int m_numResults;
};


///////////////////////////////////////////////////////////////////////////////

class BHandCommandPGet : public BHandSupervisoryCommand
{
public:
	BHandCommandPGet(const char *property) : BHandSupervisoryCommand(BHSupervisoryPGet)
	{
		m_property = new char[strlen(property) + 1];
		strcpy(m_property, property);
	}
	virtual ~BHandCommandPGet() { delete[] m_property; }

	const char * getProperty() { return m_property; }

private:
	char *m_property;
};

///////////////////////////////////////////////////////////////////////////////

/** This class is used for returning the pget global response for a PGet command.

BHandResultPGet contains a one integer response received after issuing the pget
command.
*/
class BHandResultPGet : public BHandSupervisoryResult
{
public:
	BHandResultPGet(int result) : BHandSupervisoryResult(BHSupervisoryResultPGet), m_result(result) {}
	virtual ~BHandResultPGet() {}

	int getResult() { return m_result; }

private:
	int m_result;
};


///////////////////////////////////////////////////////////////////////////////

class BHandCommandSave : public BHandSupervisoryCommand
{
public:
	BHandCommandSave(BHMotors motors) : BHandSupervisoryCommand(BHSupervisorySave), Motors(motors) {}
	virtual ~BHandCommandSave() {}

	BHMotors Motors;
};

///////////////////////////////////////////////////////////////////////////////

class BHandCommandLoad : public BHandSupervisoryCommand
{
public:
	BHandCommandLoad(BHMotors motors) : BHandSupervisoryCommand(BHSupervisoryLoad), Motors(motors) {}
	virtual ~BHandCommandLoad() {}

	BHMotors Motors;
};

///////////////////////////////////////////////////////////////////////////////

class BHandCommandDefault : public BHandSupervisoryCommand
{
public:
	BHandCommandDefault(BHMotors motors) : BHandSupervisoryCommand(BHSupervisoryDefault), Motors(motors) {}
	virtual ~BHandCommandDefault() {}

	BHMotors Motors;
};

///////////////////////////////////////////////////////////////////////////////

class BHandCommandTemperature : public BHandSupervisoryCommand
{
public:
	BHandCommandTemperature(BHMotors motors = 15) : BHandSupervisoryCommand(BHSupervisoryTemperature), Motors(motors) {}
	virtual ~BHandCommandTemperature() {}

	BHMotors Motors;
};

/** This class is used for returning the temperature response for a Temperature command.

BHandResultTemperature contains a one integer response received after issuing the
temperature command.
*/
class BHandResultTemperature : public BHandSupervisoryResult
{
public:
	BHandResultTemperature(int temperature) : BHandSupervisoryResult(BHSupervisoryResultTemperature)
	{
		m_temperatures = new int[1];
		m_temperatures[0] = temperature;
		m_numTemperatures = 1;
	}
	BHandResultTemperature(const int *temperatures, unsigned int numTemperatures) : BHandSupervisoryResult(BHSupervisoryResultTemperature)
	{
		m_temperatures = new int[numTemperatures];
		for (unsigned int i = 0; i < numTemperatures; i++)
			m_temperatures[i] = temperatures[i];
		m_numTemperatures = numTemperatures;
	}
	virtual ~BHandResultTemperature()
	{
		delete[] m_temperatures;
	}

	int getTemperature()
	{
		if (m_numTemperatures > 1)
		{
			int maxTemperature = 0;
			for (unsigned int i = 0; i < m_numTemperatures; i++)
				maxTemperature = MAX(maxTemperature, m_temperatures[i]);
			return maxTemperature;
		}
		else
			return m_temperatures[0];
	}

private:
	int *m_temperatures;
	unsigned int m_numTemperatures;
};


///////////////////////////////////////////////////////////////////////////////
// More commands in Supervisory Module (Misc.)                               //
///////////////////////////////////////////////////////////////////////////////

class BHandCommandDelay : public BHandSupervisoryCommand
{
public:
	BHandCommandDelay(unsigned int msec) : BHandSupervisoryCommand(BHSupervisoryDelay), MilliSeconds(msec) {}
	virtual ~BHandCommandDelay() {}

	unsigned int MilliSeconds;
};

///////////////////////////////////////////////////////////////////////////////

class BHandCommandStopMotor : public BHandSupervisoryCommand
{
public:
	BHandCommandStopMotor(BHMotors motors) : BHandSupervisoryCommand(BHSupervisoryStopMotor), Motors(motors) {}
	virtual ~BHandCommandStopMotor() {}

	BHMotors Motors;
};

///////////////////////////////////////////////////////////////////////////////

class BHandCommandBaud : public BHandSupervisoryCommand
{
public:
	BHandCommandBaud(unsigned int newBaud) : BHandSupervisoryCommand(BHSupervisoryBaud), NewBaud(newBaud) {}
	virtual ~BHandCommandBaud() {}

	unsigned int NewBaud;
};

///////////////////////////////////////////////////////////////////////////////

class BHandCommandCommand : public BHandSupervisoryCommand
{
public:
	BHandCommandCommand(const char * send) :
		BHandSupervisoryCommand(BHSupervisoryCmd)
	{
		m_send = new char[strlen(send) + 1];
		strcpy(m_send, send);
	}
	virtual ~BHandCommandCommand() { delete[] m_send; }

	const char * getSend() { return m_send; }

private:
	char *m_send;
};

/** This class is used for returning the general response for a Command command.

BHandResultCommand contains an accessor method to the response received after
issuing the command command.
*/
class BHandResultCommand : public BHandSupervisoryResult
{
public:
	BHandResultCommand(const char * receive) :
		BHandSupervisoryResult(BHSupervisoryResultCmd)
	{
		if (receive != NULL)
		{
			m_receive = new char[strlen(receive) + 1];
			strcpy(m_receive, receive);
		}
		else
		{
			m_receive = new char[1];
			m_receive[0] = 0;
		}
	}
	virtual ~BHandResultCommand() { delete[] m_receive; }

	const char * getReceive() { return m_receive; }

private:
	char *m_receive;
};

///////////////////////////////////////////////////////////////////////////////

#endif
