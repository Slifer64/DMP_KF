///////////////////////////////////////////////////////////////////////////////
//                   (C) Barrett Technology Inc. 2009-2010                   //
///////////////////////////////////////////////////////////////////////////////

#include "BHandDriver.h"

#include "BHandSupervisoryRealTime.h"


int BHandDriver::runSupervisoryCommand(BHandSupervisoryCommand *command)
{
	// Check that there is not already a supervisory mode command that is running

	int result;
	// Ensure that previous supervisory command runs completely or return an error
	if (m_moduleSuperReal->getSyncMode() == BHMODE_ASYNCNOW)
	{
		// in immediate async mode, so throw an error if the request handler is busy
		if ((result = m_moduleSuperReal->ComWaitForCompletion(0)))
			return BHERR_NOTCOMPLETED;
	}
	else
	{
		// wait forever until previous command has finished
		m_moduleSuperReal->ComWaitForCompletion(INFINITE);
	}

	setActiveSupervisoryCommand(command);

	// Check for BHMODE_RETURN mode (run commands that return values synchronously)
	bool immediate = false;
	if (command != NULL && (
		command->getCommand() == BHSupervisoryReset ||
		command->getCommand() == BHSupervisoryGet ||
		command->getCommand() == BHSupervisoryPGet ||
		command->getCommand() == BHSupervisoryTemperature ||
		command->getCommand() == BHSupervisoryCmd))
		immediate = (m_moduleSuperReal->getSyncMode() == BHMODE_RETURN);
	if (immediate)
		m_moduleSuperReal->setSyncMode(BHMODE_SYNC);
	result = m_moduleSuperReal->ComRequest(BHREQ_SUPERVISE);
	if (immediate)
		m_moduleSuperReal->setSyncMode(BHMODE_RETURN);

	return result;
}

///////////////////////////////////////////////////////////////////////////////
// Methods in the PucksInHand Supervisory Module
///////////////////////////////////////////////////////////////////////////////

int BHandDriver::InitHand(const char *motor)
{
	return runSupervisoryCommand(new BHandCommandInit(toBHMotors(motor)));
}

int BHandDriver::Reset(bool *responseReceived)
{
	// Set result to a default value immediately so that it will be set
	// predictably in the case that there is a nonzero error-code returned
	// or the command is executed asynchronously
	if (responseReceived)
		*responseReceived = false;

	//printf("run reset command\n");

	int r;
	if ((r = runSupervisoryCommand(new BHandCommandReset())))
	{
		//printf("run reset command done %d\n", r);
		return r;
	}

	//printf("run reset command done %d\n", r);

	if (!m_moduleSuperReal->ComIsPending() && responseReceived)
		// Synchronous mode, command response received
		// Set "Reset" command result now that it has been received
		*responseReceived = ((BHandResultReset *)m_supervisoryCommandResult)->responseReceived();

	return 0;
}

int BHandDriver::Close(const char *motor)
{
	return runSupervisoryCommand(new BHandCommandClose(toBHMotors(motor)));
}

int BHandDriver::Open(const char *motor)
{
	return runSupervisoryCommand(new BHandCommandOpen(toBHMotors(motor)));
}

int BHandDriver::GoToDefault(const char *motor)
{
	return runSupervisoryCommand(new BHandCommandGoToDefault(toBHMotors(motor)));
}

int BHandDriver::GoToDifferentPositions(const int *encoderPositions, unsigned int numEncoderPositions)
{
	return runSupervisoryCommand(new BHandCommandGoToDifferentPositions(encoderPositions, numEncoderPositions));
}

int BHandDriver::GoToHome(const char *motor)
{
	return runSupervisoryCommand(new BHandCommandGoToHome(toBHMotors(motor)));
}

int BHandDriver::GoToPosition(const char *motor, int encoderPositionTickCount)
{
	return runSupervisoryCommand(new BHandCommandGoToPosition(toBHMotors(motor), encoderPositionTickCount));
}

int BHandDriver::StepClose(const char *motor, bool valueIncluded, int stepAmount)
{
	return runSupervisoryCommand(new BHandCommandStepClose(toBHMotors(motor), valueIncluded, stepAmount));
}

int BHandDriver::StepOpen(const char *motor, bool valueIncluded, int stepAmount)
{
	return runSupervisoryCommand(new BHandCommandStepOpen(toBHMotors(motor), valueIncluded, stepAmount));
}

int BHandDriver::StopMotor(const char *motor)
{
	return runSupervisoryCommand(new BHandCommandStopMotor(toBHMotors(motor)));
}

int BHandDriver::TorqueClose(const char *motor)
{
	return runSupervisoryCommand(new BHandCommandTorqueClose(toBHMotors(motor)));
}

int BHandDriver::TorqueOpen(const char *motor)
{
	return runSupervisoryCommand(new BHandCommandTorqueOpen(toBHMotors(motor)));
}


int BHandDriver::Get(const char *motor, const char *parameter, int *result)
{
	// Set result to a default value immediately so that it will be set
	// predictably in the case that there is a nonzero error-code returned
	// or the command is executed asynchronously
	BHMotors bhMotors = toBHMotors(motor);
	for (unsigned int i = 0; i < countMotors(bhMotors); i++)
		result[i] = 20 + i * 2;

	int r;
	if ((r = runSupervisoryCommand(new BHandCommandGet(bhMotors, parameter))))
		return r;

	if (!m_moduleSuperReal->ComIsPending())
	{
		// Synchronous mode, command response received
		BHandResultGet *getResult = (BHandResultGet *)m_supervisoryCommandResult;

		// Set "Get" command result(s) now that they have been received
		for (unsigned int i = 0; i < getResult->getNumResults(); i++)
			result[i] = getResult->getResult(i);
	}

	return 0;
}

int BHandDriver::Set(const char *motor, const char *parameter, int value)
{
	return runSupervisoryCommand(new BHandCommandSet(toBHMotors(motor), parameter, value));
}

int BHandDriver::PGet(const char *parameter, int *result)
{
	// Set result to a default value immediately so that it will be set
	// predictably in the case that there is a nonzero error-code returned
	// or the command is executed asynchronously
	*result = 0;


	int r;
	if ((r = runSupervisoryCommand(new BHandCommandPGet(parameter))))
		return r;

	if (!m_moduleSuperReal->ComIsPending())
	{
		// Synchronous mode, command response received
		// Set "PGet" command result now it has been received
		*result = ((BHandResultPGet *)m_supervisoryCommandResult)->getResult();
	}

	return 0;
}

int BHandDriver::PSet(const char *parameter, int value)
{
	return runSupervisoryCommand(new BHandCommandPSet(parameter, value));
}

int BHandDriver::Default(const char *motor)
{
	return runSupervisoryCommand(new BHandCommandDefault(toBHMotors(motor)));
}

int BHandDriver::Load(const char *motor)
{
	return runSupervisoryCommand(new BHandCommandLoad(toBHMotors(motor)));
}

int BHandDriver::Save(const char *motor)
{
	return runSupervisoryCommand(new BHandCommandSave(toBHMotors(motor)));
}

int BHandDriver::Temperature(int *temperature)
{
	// Set result to a default value immediately so that it will be set
	// predictably in the case that there is a nonzero error-code returned
	// or the command is executed asynchronously
	*temperature = 0;

	int r;
	if ((r = runSupervisoryCommand(new BHandCommandTemperature())))
		return r;

	if (!m_moduleSuperReal->ComIsPending())
		// Synchronous mode, command response received
		// Set "Temperature" command result now that it has been received
		*temperature = ((BHandResultTemperature *)m_supervisoryCommandResult)->getTemperature();

	return 0;

}

int BHandDriver::Delay(unsigned int msec)
{
	return runSupervisoryCommand(new BHandCommandDelay(msec));
}

int BHandDriver::Command(const char *send)
{
	return runSupervisoryCommand(new BHandCommandCommand(send));
}

int BHandDriver::Command(const char *send, char *receive)
{
	// Set result to a default value immediately so that it will be set
	// predictably in the case that there is a nonzero error-code returned
	// or the command is executed asynchronously

	if (receive)
		*receive = 0;

	int r;
	if ((r = runSupervisoryCommand(new BHandCommandCommand(send))))
		return r;

	if (receive && !m_moduleSuperReal->ComIsPending())
		// Synchronous mode, command response received
		// Set "Command" command result now that it has been received
	{
		if (m_supervisoryCommandResult != NULL)
			strcpy(receive, ((BHandResultCommand *)m_supervisoryCommandResult)->getReceive());
		else
			*receive = 0;
	}

	return 0;
}

int BHandDriver::Baud(unsigned int newbaud)
{
	return runSupervisoryCommand(new BHandCommandBaud(newbaud));
}


int BHandDriver::ExecuteSupervisoryCall()
{
	BHandSupervisoryCommand *command = m_activeSupervisoryCommand;
	if (command == NULL) // TODO: check if using the 262 driver at run time
		return HandleSupervisoryCall(); // call HandleSupervisory method instead (only needed for 262 hand)

	// Check for last command result
	if (m_supervisoryCommandResult != NULL)
	{
		// The result from the last supervisory command that was executed should be deleted
		delete m_supervisoryCommandResult;
		m_supervisoryCommandResult = NULL;
	}

	int r = -1;

	switch (command->getCommand())
	{
		case BHSupervisoryInit:                   {             r = HandInit(((BHandCommandInit *)command)->Motors);                                                                     break; }
		case BHSupervisoryReset:                  {
			bool reset;
			r = HandReset(((BHandCommandReset *)command)->Motors, &reset);
			//if (!r)
				m_supervisoryCommandResult = new BHandResultReset(reset);
			break;
												  }

		case BHSupervisoryClose:                  { r = HandClose(((BHandCommandClose *)command)->Motors);             break; }
		case BHSupervisoryOpen:                   { r = HandOpen(((BHandCommandOpen *)command)->Motors);               break; }

		case BHSupervisoryTorqueClose:            { r = HandTorqueClose(((BHandCommandTorqueClose *)command)->Motors); break; }
		case BHSupervisoryTorqueOpen:             { r = HandTorqueOpen(((BHandCommandTorqueOpen *)command)->Motors);   break; }

		case BHSupervisoryStepClose:              { r = HandStepClose(((BHandCommandStepClose *)command)->Motors, ((BHandCommandStepClose *)command)->ValueIncluded, ((BHandCommandStepClose *)command)->StepAmount); break; }
		case BHSupervisoryStepOpen:               { r = HandStepOpen(((BHandCommandStepOpen *)command)->Motors, ((BHandCommandStepOpen *)command)->ValueIncluded, ((BHandCommandStepOpen *)command)->StepAmount);     break; }

		case BHSupervisoryGoToDefault:            { r = HandGoToDefault(((BHandCommandGoToDefault *)command)->Motors, ((BHandCommandGoToDefault *)command)->ValueIncluded, ((BHandCommandGoToDefault *)command)->DefaultPosition);     break; }
		case BHSupervisoryGoToDifferentPositions: { r = HandGoToDifferentPositionsHand(((BHandCommandGoToDifferentPositions *)command)->getEncoderPositions(), ((BHandCommandGoToDifferentPositions *)command)->getNumEncoderPositions()); break; }
		case BHSupervisoryGoToHome:               { r = HandGoToHome(((BHandCommandGoToHome *)command)->Motors);                                                                                                                       break; }
		case BHSupervisoryGoToPosition:           { r = HandGoToPosition(((BHandCommandGoToPosition *)command)->Motors, ((BHandCommandGoToPosition *)command)->EncoderPositionTickCount);                                              break; }

		case BHSupervisoryGet:                    {
			int nResults = 4;
			if (getComm() == BH_SERIAL_COMMUNICATION && countMotors(((BHandCommandGet *)command)->Motors) == 1)
				// Count number of properties
				nResults = countWords(((BHandCommandGet *)command)->getProperty()); // maximum number of properties possibles

			int *results = new int[nResults];

			int n = -1;
			r = HandGet(((BHandCommandGet *)command)->Motors, ((BHandCommandGet *)command)->getProperty(), results, &n);
			//if (!r)
			//m_supervisoryCommandResult = new BHandResultGet(((BHandCommandGet *)command)->Motors, results, n >= 0 ? n : nResults); // works with 262

			if (n == -1)
				m_supervisoryCommandResult = new BHandResultGet(((BHandCommandGet *)command)->Motors, results); // testing 280 Get
			else
				m_supervisoryCommandResult = new BHandResultGet(((BHandCommandGet *)command)->Motors, results, n >= 0 ? n : nResults); // works with 262

			delete[] results;
			break;
												  }
/*		case BHSupervisoryGet:                    { // TODO: merge 280 version
			int results[4];
			HandGet(((BHandCommandGet *)command)->Motors, ((BHandCommandGet *)command)->getProperty(), results);

			m_supervisoryCommandResult = new BHandResultGet(((BHandCommandGet *)command)->Motors, results); // orginal
			//m_supervisoryCommandResult = new BHandResultGet(((BHandCommandGet *)command)->Motors, results, countMotors(((BHandCommandGet *)command)->Motors)); // new
			break; }*/

		case BHSupervisorySet:                    { r = HandSet(((BHandCommandSet *)command)->Motors, ((BHandCommandSet *)command)->getProperty(), ((BHandCommandSet *)command)->Value);                                                                                                     break; }

		case BHSupervisoryPGet:                   {
			int result;
			r = HandPGet(((BHandCommandPGet *)command)->getProperty(), &result);
			//if (!r)
				m_supervisoryCommandResult = new BHandResultPGet(result);
			break;
												  }
		case BHSupervisoryPSet:                   { r = HandPSet(((BHandCommandPSet *)command)->getProperty(), ((BHandCommandPSet *)command)->Value);                                          break; }

		case BHSupervisorySave:                   { r = HandSave(((BHandCommandSave *)command)->Motors);       break; }
		case BHSupervisoryLoad:                   { r = HandLoad(((BHandCommandLoad *)command)->Motors);       break; }
		case BHSupervisoryDefault:                { r = HandDefault(((BHandCommandDefault *)command)->Motors); break; }

		case BHSupervisoryTemperature:            {
			if (getComm() == BH_SERIAL_COMMUNICATION)
			{
				int temperature;
				r = HandTemperature(((BHandCommandTemperature *)command)->Motors, &temperature, 1);
				//if (!r)
					m_supervisoryCommandResult = new BHandResultTemperature(temperature);
			}
			else
			{
				int temperatures[4];
				r = HandTemperature(((BHandCommandTemperature *)command)->Motors, temperatures, 4);
				//if (!r)
					m_supervisoryCommandResult = new BHandResultTemperature(temperatures, 4);
			}
			break;
												  }

		case BHSupervisoryBaud:                   { r = HandBaud(((BHandCommandBaud *)command)->NewBaud);          break; }
		case BHSupervisoryDelay:                  { r = HandDelay(((BHandCommandDelay *)command)->MilliSeconds);   break; }
		case BHSupervisoryStopMotor:              { r = HandStopMotor(((BHandCommandStopMotor *)command)->Motors); break; }

		case BHSupervisoryCmd:
			{
				char *receive = HandCommand(((BHandCommandCommand *)command)->getSend(), &r);

				if (!r)
					m_supervisoryCommandResult = new BHandResultCommand(receive);

				break;
			}
		default:
			{
				return BHERR_NOTCOMPLETED;
			}
	}

	// Deallocate memory for the supervisory command and set the pointer to NULL
	delete m_activeSupervisoryCommand;
	m_activeSupervisoryCommand = NULL;

	return r;
}
