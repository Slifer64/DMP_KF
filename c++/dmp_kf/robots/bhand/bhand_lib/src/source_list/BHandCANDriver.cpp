///////////////////////////////////////////////////////////////////////////////
//                   (C) Barrett Technology Inc. 2009-2010                   //
///////////////////////////////////////////////////////////////////////////////

#include "BHandCANDriver.h"

#include "BHandSupervisoryRealTime.h"
#include "BHandCommands.h"


#include "bhand_misc.h"
#include "bhand_parse.h"
#include "bhand_motors.h"

#include "puck2.h"

#include <cstring>

#include <math.h>


#ifndef MAX
	#define MAX(x, y) ((x > y) ? x : y)
#endif


enum BHRealTimeMotorProperties {
	REALTIME_LCV = 0,
	REALTIME_LCT,
	REALTIME_LCP,
	REALTIME_LFV,
	REALTIME_LFS,
	REALTIME_LFT,
	REALTIME_LFAP,
	REALTIME_LFPPS
	};

const struct textKey propBHRealTimeMotor[] =
	{
		{ "LFPPS", REALTIME_LFPPS }, // RealTime Loop Feedback Pressure Profile Sensors
		{ "LFAP", REALTIME_LFAP }, // RealTime Loop Feedback Actual Position
		{ "LCP", REALTIME_LCP }, // RealTime Loop Control Position
		{ "LCT", REALTIME_LCT }, // RealTime Loop Control Torque
		{ "LCV", REALTIME_LCV }, // RealTime Loop Control Velocity
		{ "LFS", REALTIME_LFS }, // RealTime Loop Feedback Strain
		{ "LFT", REALTIME_LFT }, // RealTime Loop Feedback Temperature
		{ "LFV", REALTIME_LFV },  // RealTime Loop Feedback Velocity

		{ "", NONE }
	};


///////////////////////////////////////////////////////////////////////////
// RealTime Global and Motor Parameters
///////////////////////////////////////////////////////////////////////////

static const RealTimeGlobalParameters defGlobalParams = { 0, false, 0, BHMotorTSTOPProtect, 0 };
static const RealTimeMotorParameters defMotorParams =
	{ false, false, false, false, false, false, false, false, 0, 0, 0, 0, 0, 0, 0, { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, 0.0f };

//#define PPS_DEMO
#ifdef PPS_DEMO
	// Demo PPS sensors
	const int PPS_FILTER_SIZE = 32;
	int filterIndexPPS = 0;
	int FeedbackPresentPPS[4][24][PPS_FILTER_SIZE];
	int TaredPPS[4][24];
#endif


#define NUM_PUCKS_IN_HAND 4

#define PUCK2_ID_FINGER1 11
#define PUCK2_ID_FINGER2 12
#define PUCK2_ID_FINGER3 13
#define PUCK2_ID_SPREAD  14


///////////////////////////////////////////////////////////////////////////
// CAN BHandDriver Constructor/Deconstructor
///////////////////////////////////////////////////////////////////////////

BHandCANDriver::BHandCANDriver(BHand *bhand, BHandSupervisoryRealtime *moduleSuperReal)
	: BHandDriver(bhand, moduleSuperReal, BH_CAN_COMMUNICATION), pucksInHand(PUCK2_ID_PC),
	m_realtimeMode(false),
	realtimeGlobalParameters(defGlobalParams)
{
	realtimeMotorParameters = new RealTimeMotorParameters[NUM_PUCKS_IN_HAND];
	for (int i = 0; i < NUM_PUCKS_IN_HAND; i++)
		realtimeMotorParameters[i] = defMotorParams;
	pucks = new BPuck2* [NUM_PUCKS_IN_HAND];
}


BHandCANDriver::~BHandCANDriver()
{
	delete[] pucks;
	delete[] realtimeMotorParameters;
}


///////////////////////////////////////////////////////////////////////////
// BarrettHand Driver Methods
///////////////////////////////////////////////////////////////////////////

int BHandCANDriver::Initialize()
{
	if (m_open)
		return BHERR_OPENCOMMPORT;

	// Add pucks to be controlled to BPuck2Manager
	if (pucksInHand.addPuck(PUCK2_ID_FINGER1) || pucksInHand.addPuck(PUCK2_ID_FINGER2) ||
	    pucksInHand.addPuck(PUCK2_ID_FINGER3) || pucksInHand.addPuck(PUCK2_ID_SPREAD))
		return BHERR_OPENCOMMPORT;

	// Initializes the PCAN driver
	if (pucksInHand.init())
		return BHERR_OPENCOMMPORT;

	// Mark certain Pucks as not being "awake" to deal with hardware not
	// being fully functional during testing and development of drivers

	// This doesn't help anymore, see HandReset and/or awaken()

	//pucksInHand.putToSleep(PUCK2_ID_FINGER1);
	//pucksInHand.putToSleep(PUCK2_ID_FINGER2);
	//pucksInHand.putToSleep(PUCK2_ID_FINGER3);
	//pucksInHand.putToSleep(PUCK2_ID_SPREAD);

	// Get pointers to instances of the BPuck, which will be used to Get/Set properties
	pucks[0] = pucksInHand.getPuck(PUCK2_ID_FINGER1);
	pucks[1] = pucksInHand.getPuck(PUCK2_ID_FINGER2);
	pucks[2] = pucksInHand.getPuck(PUCK2_ID_FINGER3);
	pucks[3] = pucksInHand.getPuck(PUCK2_ID_SPREAD);

	// Setup required to run Supervisory and RealTime PucksInHand methods
	puckTwos.bhand = m_bhand;
	puckTwos.puckWithMotor = pucks;
	puckTwos.pucksWithMotors = NUM_PUCKS_IN_HAND;
	puckTwos.puckManager = &pucksInHand;

	waitCallbackFunc = NULL;

	// Driver is up and running
	m_open = true;

	return 0;
}

void BHandCANDriver::Close()
{
	pucksInHand.close();
	m_open = false;
}







void BHandCANDriver::waitForStop(BHMotors bhMotors)
{
	// Save TSTOP value for each motor (must be nonzero for waitForStop to return)
	int *tstop = new int[puckTwos.pucksWithMotors];
	bool *restoreTSTOP = new bool[puckTwos.pucksWithMotors];
	for (unsigned int i = 0; i < puckTwos.pucksWithMotors; i++)
	{
		restoreTSTOP[i] = false;
		if ((bhMotors >> i) & 1)
		{
			HandGet(1 << i, "TSTOP", &tstop[i]);
			if (tstop[i] == 0)
			{
				HandSet(1 << i, "TSTOP", BHAND_DEFAULT_TSTOP);
				restoreTSTOP[i] = true;
				//printf("waitForStop setting TSTOP[%d] to 200, saving %d\n", i, tstop[i]);
			}
		}
	}

	//printf("waitForStop(%d)\n", bhMotors);

	//BHiResTimer tmr;
	//tmr.Start();

	// Loop - while
	while (1)
	{
		int numMotors = 0;
		int numDone = 0;

		if (waitCallbackFunc != NULL)
		{
			waitCallbackFunc(puckTwos.bhand);
		}
		else
		{
			DELAY(50);

			// Testing
			/*int tor[4]; RTUpdate("GS", "T", tor);

			//DELAY(1);
			int pos[4]; RTUpdate("GS", "P", pos);
			printf("time/torque %d  %d %d %d %d %d %d %d %d\n", (unsigned int)tmr.GetElapsedInMilliSecs(),
				tor[0], tor[1], tor[2], tor[3],
				pos[0], pos[1], pos[2], pos[3]
				);*/
		}

		// Check if each motor is moving - mark as moving if it is
		// New method to check motor modes (faster)
		unsigned int mode[4];
		BPuck2Manager *puckManager = &pucksInHand;
		puckManager->startGetPropertyBatch();
		for (unsigned int i = 0; i < puckTwos.pucksWithMotors; i++)
			if ((bhMotors >> i) & 1)
				puckManager->addToGetPropertyBatch(pucks[i]->getID(), &(mode[i]));
		int err = puckManager->getBatchPropertyUInt32(PROP_MODE);

		if (err == 0)
		{
			for (unsigned int i = 0; i < puckTwos.pucksWithMotors; i++)
				if ((bhMotors >> i) & 1)
				{
					numMotors++;

					if (mode[i] == 0 || mode[i] == 3) // Idle mode or PID mode
					{
						numDone++;
						//printf("waitForStop() numDone = %d\n", numDone);
					}
					else
					{
						//printf("\n");
						//if (i == 3)
						//printf("Mode(%d) = %d, error = %d\n", i, result, error);
					}
					//printf("Mode(%d) = %d, error = %d\n", i, result, error);
				}
		}

		if (numDone == numMotors)
		{
			break;
		}
	}

	for (unsigned int i = 0; i < puckTwos.pucksWithMotors; i++)
	{
		if (restoreTSTOP[i])
		{
			HandSet(1 << i, "TSTOP", tstop[i]);
			//printf("waitForStop restore tstop[%d] to %d\n", i, tstop[i]);
		}
	}

	delete[] restoreTSTOP;
	delete[] tstop;
}








int BHandCANDriver::HandInit(BHMotors bhMotors)
{
	// Allow for a small delay after waking up the pucks
	int temp[4];
	HandGet(15, "MODE", temp);

	// Testing
	//HandSet(7, "KP", 500);  // original
	//HandSet(7, "KD", 5000); // original
	//HandSet(7, "KP", 100);
	//HandSet(7, "KD", 1000);

	//HandSet(7, "KP", 50); // latest gains on puck now
	//HandSet(7, "KD", 500);
	//int kp[4]; HandGet(7, "KP", kp); printf("KP (%d, %d, %d)\n", kp[0], kp[1], kp[2]);
	//int ki[4]; HandGet(7, "KI", ki); printf("KI (%d, %d, %d)\n", ki[0], ki[1], ki[2]);
	//int kd[4]; HandGet(7, "KD", kd); printf("KD (%d, %d, %d)\n", kd[0], kd[1], kd[2]);

	//BHiResTimer tmr;
	//tmr.Start();

	// Send HI commands to the fingers first
	if (bhMotors & 1) pucks[0]->setPropertyUInt8(PROP_CMD, CMD_HI);
	if (bhMotors & 2) pucks[1]->setPropertyUInt8(PROP_CMD, CMD_HI);
	if (bhMotors & 4) pucks[2]->setPropertyUInt8(PROP_CMD, CMD_HI);

	if (bhMotors & 7)
	{
		for (int i = 0; i < 60; i++)
		{
			int mode[3];
			//HandGet(7, "MODE", mode);
			RTUpdate("G", "MODE", mode);

			// Check for idle mode
			bool idle = true;
			if (bhMotors & 1 && mode[0] != 0) idle = false;
			if (bhMotors & 2 && mode[1] != 0) idle = false;
			if (bhMotors & 4 && mode[2] != 0) idle = false;

			if (idle)
				break;

			DELAY(100);
		}

		/*for (int i = 0; i < 600; i++) // testing
		{
			int mode[3];
			//HandGet(7, "MODE", mode);
			RTUpdate("G", "MODE", mode);
			DELAY(2); // Testing

			//printf("mode[0] = %d\n", mode[0]);

			// Check for idle mode
			bool idle = true;
			if (bhMotors & 1 && mode[0] != 0) idle = false;
			if (bhMotors & 2 && mode[1] != 0) idle = false;
			if (bhMotors & 4 && mode[2] != 0) idle = false;

			// Testing
			int tor[3]; RTUpdate("G", "T", tor);

			DELAY(2);
			int pos[3]; RTUpdate("G", "P", pos);
			printf("time/torque %d %d %d %d %d %d %d %d %d\n", (unsigned int)tmr.GetElapsedInMilliSecs(),
				tor[0], tor[1], tor[2], tor[3],
				pos[0], pos[1], pos[2], pos[3]
				);

			if (idle)
				break;

			DELAY(2); // Testing
		}*/

	}

	// Send HI commands to the spread last
	if (bhMotors & 8)
	{
		pucks[3]->setPropertyUInt8(PROP_CMD, CMD_HI);

		for (int i = 0; i < 40; i++)
		{
			int mode;
			HandGet(8, "MODE", &mode);

			if (mode == 0 || mode == 3)
				break;

			DELAY(100);
		}


		/*for (int i = 0; i < 400; i++) // testing
		{
			int mode;
			HandGet(8, "MODE", &mode);

			// Testing
			//int tor[1]; RTUpdate("S", "T", tor);
			int tor; HandGet(8, "T", &tor);
			int pos; HandGet(8, "P", &pos);
			printf("spread torque %d pos %d\n", tor, pos);

			if (mode == 0 || mode == 3)
				break;

			DELAY(10); // testing
		}*/

	}

	return 0;
}

int BHandCANDriver::HandReset(BHMotors bhMotors, bool *responseReceived)
{
	for (unsigned int i = 0; i < puckTwos.pucksWithMotors; i++)
		if ((bhMotors >> i) & 1)
			pucks[i]->setPropertyUInt8(PROP_CMD, CMD_RESET);

	DELAY(1000);

	int awaken = pucksInHand.awaken();
	//printf("awaken = %d\n", awaken);
	if (awaken)
	{
		*responseReceived = false;
		//printf("pucks[0]->awake() = %d\n", pucks[0]->awake());
		//printf("pucks[1]->awake() = %d\n", pucks[1]->awake());
		//printf("pucks[2]->awake() = %d\n", pucks[2]->awake());
		//printf("pucks[3]->awake() = %d\n", pucks[3]->awake());
		//if (!pucks[0]->awake() && !pucks[1]->awake() && !pucks[2]->awake() && !pucks[3]->awake()) // older
		if (!pucks[0]->awake() || !pucks[1]->awake() || !pucks[2]->awake() || !pucks[3]->awake()) // newer - all pucks should be awake after a reset
		{
			//printf("pucksInHandReset - no Pucks awake\n");
			return -9;
		}
	}

	//printf("pucksInHandReset - Awakened Pucks\n");

	// Set some default property values
	//HandSet(0x07, "MT", BHAND_FINGER_MAX_PEAK_TORQUE); // 3300 is the default on Puck 2, bump it up so open/close work after an HI
	//HandSet(0x08, "MT", BHAND_SPREAD_MAX_PEAK_TORQUE); // 3300 is the default on Puck 2, bump it up so open/close work after an HI
	HandSet(0x0f, "DP", 0);      // assume this should be done on a Puck reset but there is no harm in doing it again
	HandSet(0x0f, "TSTOP", BHAND_DEFAULT_TSTOP); // so many milliseconds are needed to switch from idle to position mode

	// At least one puck received a response
	*responseReceived = true;
	//printf("pucksInHandReset - reset done\n");

	return (*responseReceived) ? 0 : -9;
}

int BHandCANDriver::HandClose(BHMotors bhMotors)
{
	for (unsigned int i = 0; i < puckTwos.pucksWithMotors; i++)
		if ((bhMotors >> i) & 1)
			pucks[i]->setPropertyUInt8(PROP_CMD, CMD_C);

	waitForStop(bhMotors);

	return 0;
}

int BHandCANDriver::HandOpen(BHMotors bhMotors)
{
	for (unsigned int i = 0; i < puckTwos.pucksWithMotors; i++)
		if ((bhMotors >> i) & 1)
			pucks[i]->setPropertyUInt8(PROP_CMD, CMD_O);

	waitForStop(bhMotors);
	return 0;
}

int BHandCANDriver::HandGoToDefault(BHMotors bhMotors, bool valueIncluded, int defaultPosition)
{
	// Should this set the DP property or just cause the motors to move to the default position?
	if (valueIncluded)
	{
		// set DP property for motors
		HandSet(bhMotors, "DP", defaultPosition);
		//printf("pucksInHandGoToDefault(%d, %d, %d)\n", bhMotors, valueIncluded, defaultPosition);
	}
	//else
		//printf("pucksInHandGoToDefault(%d)\n", bhMotors);

	for (unsigned int i = 0; i < puckTwos.pucksWithMotors; i++)
		if ((bhMotors >> i) & 1)
			pucks[i]->setPropertyUInt8(PROP_CMD, CMD_M);
			//pucks[i]->setPropertyUInt8(PROP_MODE, 5); // works by setting property E

	waitForStop(bhMotors);

	return 0;
}

int BHandCANDriver::HandGoToDifferentPositionsHand(const int *encoderPositions, unsigned int numEncoderPositions)
{
	if (numEncoderPositions > puckTwos.pucksWithMotors)
		return -1; // problem

	for (unsigned int i = 0; i < numEncoderPositions; i++)
		pucks[i]->setPropertyUInt32(PROP_M, (unsigned int)encoderPositions[i]);
	waitForStop(15);

	return 0;
}

int BHandCANDriver::HandGoToHome(BHMotors bhMotors)
{
	if (bhMotors & 1) pucks[0]->setPropertyUInt8(PROP_CMD, CMD_HOME);
	if (bhMotors & 2) pucks[1]->setPropertyUInt8(PROP_CMD, CMD_HOME);
	if (bhMotors & 4) pucks[2]->setPropertyUInt8(PROP_CMD, CMD_HOME);

	waitForStop(bhMotors & 7);

	if (bhMotors & 8) pucks[3]->setPropertyUInt8(PROP_CMD, CMD_HOME);

	waitForStop(bhMotors & 8);

	return 0;
}

int BHandCANDriver::HandGoToPosition(BHMotors bhMotors, unsigned int encoderPositionTickCount)
{
	//printf("pucksInHandGoToPosition(%d, %d)\n", bhMotors, encoderPositionTickCount);

	if (bhMotors & 1) pucks[0]->setPropertyUInt32(PROP_M, encoderPositionTickCount);
	if (bhMotors & 2) pucks[1]->setPropertyUInt32(PROP_M, encoderPositionTickCount);
	if (bhMotors & 4) pucks[2]->setPropertyUInt32(PROP_M, encoderPositionTickCount);
	if (bhMotors & 8) pucks[3]->setPropertyUInt32(PROP_M, encoderPositionTickCount);

	waitForStop(bhMotors);

	return 0;
}

int BHandCANDriver::HandStepClose(BHMotors bhMotors, bool valueIncluded, int stepAmount)
{
	//printf("pucksInHandStepClose(%d, %d, %d)\n", bhMotors, valueIncluded, stepAmount);

	if (valueIncluded)
		// set DS stepsize property for motors
		HandSet(bhMotors, "DS", stepAmount);

	for (unsigned int i = 0; i < puckTwos.pucksWithMotors; i++)
		if ((bhMotors >> i) & 1)
			pucks[i]->setPropertyUInt8(PROP_CMD, CMD_IC);

	waitForStop(bhMotors);

	return 0;
}

int BHandCANDriver::HandStepOpen(BHMotors bhMotors, bool valueIncluded, int stepAmount)
{
	//printf("pucksInHandStepOpen(%d, %d, %d)\n", bhMotors, valueIncluded, stepAmount);

	if (valueIncluded)
		// set DS stepsize property for motors
		HandSet(bhMotors, "DS", stepAmount);

	for (unsigned int i = 0; i < puckTwos.pucksWithMotors; i++)
		if ((bhMotors >> i) & 1)
			pucks[i]->setPropertyUInt8(PROP_CMD, CMD_IO);

	waitForStop(bhMotors);

	return 0;
}

int BHandCANDriver::HandStopMotor(BHMotors bhMotors)
{
	//printf("pucksInHandStopMotor(%d)\n", bhMotors);

	for (unsigned int i = 0; i < puckTwos.pucksWithMotors; i++)
		if ((bhMotors >> i) & 1)
			pucks[i]->setPropertyUInt8(PROP_CMD, CMD_T);
	DELAY(1); // Wait a millisecond for the propeties to be set

	return 0;
}

int BHandCANDriver::HandTorqueClose(BHMotors bhMotors)
{
	for (unsigned int i = 0; i < puckTwos.pucksWithMotors; i++)
		if ((bhMotors >> i) & 1)
			pucks[i]->setPropertyUInt8(PROP_CMD, CMD_TC);

	waitForStop(bhMotors);

	return 0;
}

int BHandCANDriver::HandTorqueOpen(BHMotors bhMotors)
{
	for (unsigned int i = 0; i < puckTwos.pucksWithMotors; i++)
		if ((bhMotors >> i) & 1)
			pucks[i]->setPropertyUInt8(PROP_CMD, CMD_TO);

	waitForStop(bhMotors);

	return 0;
}

int BHandCANDriver::HandGet(BHMotors bhMotors, const char *property, int *propertyResult, int *nresults)
{
	//printf("HandGet(%d, %s, ...)\n", bhMotors, property);

	if (strcmp(property, "MCV") == 0 || strcmp(property, "MOV") == 0)
		return HandGet(bhMotors, "MV", propertyResult); // Redirect Get

	// Set default property value
	unsigned int mCount = countMotors(bhMotors);
	for (unsigned m = 0; m < mCount; m++)
		propertyResult[m] = 0;

	// Get puck property index
	int puckProperty = getPropertyValue(property);
	if (puckProperty == NONE)
	{
		pucksInHandRTGet(bhMotors, property, propertyResult); // Ignore return value
		return -1; // property index not found or was found in above call
	}

	//printf("HandGet(%d, %s, ...) past pucksInHandRTGet\n", bhMotors, property);

	// Print some of variables that method was called with and motor count

	unsigned int pCount = 0;

	// Get each motor property
	for (unsigned int i = 0; i < puckTwos.pucksWithMotors; i++)
		if ((bhMotors >> i) & 1)
		{
			pucks[i]->getPropertyUInt32(puckProperty, (unsigned int *)&propertyResult[pCount++]);

			//printf("pucksInHandGet(%d, %s, %d) pCount = %d\n", bhMotors, property, propertyResult[pCount-1], pCount);

			if (pCount == mCount)
			{
				//printf("HandGet(%d, %s, *%d) \n", bhMotors, property, *propertyResult);
				return 0;
			}
		}

	//printf("HandGet(%d, %s, *%d) \n", bhMotors, property, *propertyResult);

	return 0;
}


int BHandCANDriver::HandSet(BHMotors bhMotors, const char *property, int value)
{
	if (strcmp(property, "MCV") == 0 || strcmp(property, "MOV") == 0)
		HandSet(bhMotors, "MV", value);
		// double write to written property also

	//printf("pucksInHandSet(%d, %s, %d)\n", bhMotors, property, value);

	// Get puck property index
	int puckProperty = getPropertyValue(property);
	if (puckProperty == NONE)
	{
		pucksInHandRTSet(bhMotors, property, value); // Ignore return value
		return -1; // property index not found or was found in above call
	}

	// Set each motor property
	for (unsigned int i = 0; i < puckTwos.pucksWithMotors; i++)
		if ((bhMotors >> i) & 1)
		{
			//printf("pucks[%d]->setPropertyUInt32(%d, %d);\n", i, puckProperty, value);
			pucks[i]->setPropertyUInt32(puckProperty, value);
		}

	return 0;
}

int BHandCANDriver::HandPGet(const char *property, int *propertyResult)
{
	// Get puck property index
	if (strcmp(property, "LFT") == 0)
	{
		pucksInHandRTGet(1, property, propertyResult); // Handle a special case
		return 0;
	}

	// TODO: Get global properties from Pucks?
	*propertyResult = -9;

	return 0;
}

int BHandCANDriver::HandPSet(const char *property, int value)
{
	//printf("pucksInHandSuperPSet(%d, %d)\n", pset->Property, pset->Value);

	// Get puck property index
	int puckProperty = getPropertyValue(property);
	if (puckProperty == NONE)
	{
		pucksInHandRTSet(0xf, property, value); // Ignore return value
		return -1; // property index not found
	}

	// Global properties just set all motor properties
	// Set global parameter on each puck
	for (unsigned int i = 0; i < puckTwos.pucksWithMotors; i++)
		pucks[i]->setPropertyUInt32(puckProperty, value);

	return 0;
}

int BHandCANDriver::HandDefault(BHMotors bhMotors)
{
	// For printing out default property values to the console
	/*for (unsigned int i = 0; i < 100; i++)
	{
		if (i == PROP_ACCEL || i == PROP_CT || i == PROP_OT)
		{
			int p1; pucks[0]->getPropertyUInt32(i, (unsigned int *)&p1);
			int p2; pucks[1]->getPropertyUInt32(i, (unsigned int *)&p2);
			int p3; pucks[2]->getPropertyUInt32(i, (unsigned int *)&p3);
			int p4; pucks[3]->getPropertyUInt32(i, (unsigned int *)&p4);
			printf("property i = %d: %10d, %10d, %10d, %10d\n", i, p1, p2, p3, p4);
		}
		//DELAY(4000);
	}*/


	/*
		Properties not written:
		ROLE
		SN
		ID
		X0-X7

		MOFST
		IOFST
		UPSECS - write locked
		OD - write locked
		MDS
		MPE

		TENST
		TENSO

		JP
		JOFST
	 */
	// Set defaults
	int p, v;

	int jidx[] = { 8, 9, 10, 11 };
	int pidx[] = { 3, 4, 1, 2 };

	for (unsigned int i = 0; i < puckTwos.pucksWithMotors; i++)
		if ((bhMotors >> i) & 1)
		{
			//if (i != 0)
			//	continue;

			// Write default properties
			for (int n = 0; n < pucksInHandPropertyNumDefault(); n++)
			{
				pucksInHandPropertyDefault(n, &p, &v);
				pucks[i]->setPropertyUInt32(p, v); DELAY(1);
			}
			if (i == 3)
			{
				// Spread on BH8-280
				for (int n = 0; n < pucksInHandPropertyNumSpreadDefault(); n++)
				{
					pucksInHandPropertySpreadDefault(n, &p, &v);
					pucks[i]->setPropertyUInt32(p, v); DELAY(1);
				}
            }

			pucks[i]->setPropertyUInt32(PROP_JIDX, jidx[i]); DELAY(1);
			pucks[i]->setPropertyUInt32(PROP_PIDX, pidx[i]); DELAY(1);

			// Set each property
			// SAVE -1 (saves all properties)
			pucks[i]->setPropertyUInt32(PROP_SAVE, (unsigned int)-1);

			printf("Puck %d saved default parameters\n", i);

			// This was never used
			//pucks[i]->setPropertyUInt8(PROP_CMD, CMD_DEF);
		}

	DELAY(100); // Wait a 100 milliseconds for the propeties to be set

	return 0;
}

int BHandCANDriver::HandLoad(BHMotors bhMotors)
{
	for (unsigned int i = 0; i < puckTwos.pucksWithMotors; i++)
		if ((bhMotors >> i) & 1)
			pucks[i]->setPropertyUInt8(PROP_CMD, CMD_LOAD);
	DELAY(1); // Wait a millisecond for the propeties to be loaded

	return 0;
}

int BHandCANDriver::HandSave(BHMotors bhMotors)
{
	for (unsigned int i = 0; i < puckTwos.pucksWithMotors; i++)
		if ((bhMotors >> i) & 1)
			pucks[i]->setPropertyUInt8(PROP_CMD, CMD_SAVE);
	DELAY(100); // Wait a 100 milliseconds for the propeties to be saved

	return 0;
}

int BHandCANDriver::HandTemperature(BHMotors bhMotors, int *temperature, unsigned int numTemperatures)
{
	int r = 0;
	for (unsigned int i = 0; i < numTemperatures; i++)
		if ((bhMotors >> i) & 1)
			if ((r = pucks[i]->getPropertyUInt32(PROP_TEMP, (unsigned int *)&temperature[i])))
				return r; // new return value

	return 0;
}

int BHandCANDriver::HandDelay(unsigned int msec)
{
	BHiResTimer tmr;
	tmr.Start();
	while ((unsigned int)tmr.GetElapsedInMilliSecs() < msec)
	{
		if (waitCallbackFunc != NULL)
			waitCallbackFunc(puckTwos.bhand);
		DELAY(1);
	}

	return 0;
}

char * BHandCANDriver::HandCommand(const char *send, int *errorCode)
{
	//printf("BHandCANDriver::HandCommand(%s, ...)", send);
	*errorCode = 0;
	return NULL; // don't handle misc. supervisory command here (see BHandSupervisoryRealTime)
}

int BHandCANDriver::HandBaud(unsigned int newbaud)
{
	// do nothing with the new baud
	return 0;
}




int BHandCANDriver::HandleSupervisoryCall()
{
	// not suppose to happen, do nothing
	return 0;
}
int BHandCANDriver::ExecuteRealtimeCall()
{
	//return 0;
	//return pucksInHandRTUpdate(m_control, m_feedback);

	BPuck2Manager *puckManager = puckTwos.puckManager;

	///////////////////////////////////////////////////////////////////////////
	// Control
	///////////////////////////////////////////////////////////////////////////

	if (m_control)
	{
		for (unsigned int n = 0; n < puckTwos.pucksWithMotors; n++)
		{
			if (((realtimeGlobalParameters.MotorsInRealTime >> n) & 1) == 0)
				continue;

			if (realtimeMotorParameters[n].ControlVelocity)
				pucks[n]->setPropertyUInt32(PROP_V, realtimeMotorParameters[n].ControlPresentVelocity); // Velocity mode
			if (realtimeMotorParameters[n].ControlTorque)
				pucks[n]->setPropertyUInt32(PROP_T, realtimeMotorParameters[n].ControlPresentTorque); // Torque mode
			if (realtimeMotorParameters[n].ControlPosition)
				pucks[n]->setPropertyUInt32(PROP_P, realtimeMotorParameters[n].ControlPresentPosition); // Position mode
		}
	}

	///////////////////////////////////////////////////////////////////////////
	// Feedback
	///////////////////////////////////////////////////////////////////////////

	bool someFeedback = false;

	if (m_feedback)
	{
		// Batch Get Property (Position)
		int positions[4] = { 0x7fffff, 0x7fffff, 0x7fffff, 0x7fffff}; // 4 motors
		puckManager->startGetPropertyBatch();
		for (unsigned int n = 0; n < puckTwos.pucksWithMotors; n++)
			if (realtimeMotorParameters[n].FeedbackPosition)
				pucks[n]->addToGetPropertyBatch((unsigned int *)&positions[n]);
		puckManager->getBatchPropertyUInt32(PROP_P);
		for (unsigned int n = 0; n < puckTwos.pucksWithMotors; n++)
		{
			if (realtimeMotorParameters[n].FeedbackPosition)
			{
				// pucks[n]->getPropertyUInt32(PROP_P, (unsigned int *)&(realtimeMotorParameters[n].FeedbackPresentPosition)); // original
				if (positions[n] != 0x7fffff) // ignore such a position
				{
					bool negative = ((positions[n] & 0x800000) == 0x800000);
					positions[n] = negative ?
						(positions[n] | 0xff000000) :
						(positions[n] &   0xffffff);
					realtimeMotorParameters[n].FeedbackPresentPosition = positions[n];
					someFeedback = true;
				}
			}
		}

		// Calculate present velocity from actual position over some time interval
		for (unsigned int n = 0; n < puckTwos.pucksWithMotors; n++)
		{
			if (realtimeMotorParameters[n].FeedbackVelocity)
			{
				double elapsedMilliSeconds = realtimeMotorParameters[n].tmr.GetElapsedInMilliSecs();
				if (elapsedMilliSeconds >= 1)
				{
					int newPosition = realtimeMotorParameters[n].FeedbackPresentPosition;
					int lastPosition = realtimeMotorParameters[n].velLastPositionEncoderTicks;
					realtimeMotorParameters[n].velLastPositionEncoderTicks = newPosition;

					realtimeMotorParameters[n].FeedbackPresentVelocity = (int)((double)(newPosition - lastPosition) / elapsedMilliSeconds);

					realtimeMotorParameters[n].tmr.Start();

					//if (n == 3)
					//	printf("reading feedback vel[%d] = %d\n", n, realtimeMotorParameters[n].FeedbackPresentVelocity);
				}
				someFeedback = true;
			}
		}

		// Batch Get Property (Strain)
		int strain[4] = { 4096, 4096, 4096, 4096}; // 4 motors
		puckManager->startGetPropertyBatch();
		for (unsigned int n = 0; n < puckTwos.pucksWithMotors; n++)
			if (realtimeMotorParameters[n].FeedbackStrain)
				pucks[n]->addToGetPropertyBatch((unsigned int *)&strain[n]);
		puckManager->getBatchPropertyUInt32(PROP_SG);
		for (unsigned int n = 0; n < puckTwos.pucksWithMotors; n++)
		{
			if (realtimeMotorParameters[n].FeedbackStrain &&
				strain[n] >= 0 && strain[n] < 4096)
			{
				// pucks[n]->getPropertyUInt32(PROP_SG, (unsigned int *)&(realtimeMotorParameters[n].FeedbackPresentStrain)); // original
				realtimeMotorParameters[n].FeedbackPresentStrain = strain[n];
				someFeedback = true;
			}
		}

		// Include the maximum of the present temperature readings
		if (realtimeGlobalParameters.FeedbackTemperature)
		{
			int temps[4] = { 0, 0, 0, 0}; // 4 motors
			puckManager->startGetPropertyBatch();
			for (unsigned int n = 0; n < puckTwos.pucksWithMotors; n++)
				pucks[n]->addToGetPropertyBatch((unsigned int *)&temps[n]);
			puckManager->getBatchPropertyUInt32(PROP_TEMP);
			realtimeGlobalParameters.Temperature = 0;
			for (unsigned int n = 0; n < puckTwos.pucksWithMotors; n++)
				realtimeGlobalParameters.Temperature = MAX(realtimeGlobalParameters.Temperature, temps[n]);
		}

		// Check on whether updating PPS is required
		int pps = 0;
		for (unsigned int n = 0; n < puckTwos.pucksWithMotors; n++)
			pps |= realtimeMotorParameters[n].FeedbackPPS ? (1 << n) : 0;
		if (pps)
		{
			RTUpdatePPS(pps);
			someFeedback = true;
		}
	}



	// Will there be able to be some auto reset of desired motor mode?

	//for (unsigned int n = 0; n < puckTwos.pucksWithMotors; n++)
	//{
	//	// Check to be sure that motor mode is correctly set
	//	if (n == realtimeGlobalParameters.MotorExamineMode)
	//	{
	//		unsigned int mode;
	//		pucks[n]->getPropertyUInt32(PROP_MODE, &mode);
	//		if ((realtimeMotorParameters[n].ControlTorque && mode == 2) ||
	//			(realtimeMotorParameters[n].ControlPosition && mode == 3) ||
	//			(realtimeMotorParameters[n].ControlVelocity && mode == 4))
	//			realtimeMotorParameters[n].Stopped = false;
	//		else
	//		{
	//			if (!realtimeMotorParameters[n].Stopped)
	//			{
	//				printf("Motor has stopped %d\n", n);
	//				// Did motor stop because control signal wasn't strong enough?
	//				//if (realtimeMotorParameters[n].ControlTorque)
	//				//	StoppedControl = realtimeMotorParameters[n].ControlPresentTorque;
	//				//else if (realtimeMotorParameters[n].ControlPosition)
	//				//	StoppedControl = realtimeMotorParameters[n].ControlPresentPosition;
	//				//else if (realtimeMotorParameters[n].ControlVelocity)
	//				//	StoppedControl = realtimeMotorParameters[n].ControlPresentVelocity;
	//			}
	//			realtimeMotorParameters[n].Stopped = true;
	//		}
	//	}
	//}

	// Cycle motor mode to examine
	//if (realtimeGlobalParameters.MotorExamineMode++ > puckTwos.pucksWithMotors)
	//	realtimeGlobalParameters.MotorExamineMode = 0;


	if (realtimeGlobalParameters.motorProtect & BHMotorTorqueLimitProtect)
	{
		RTHandProtectMotors();
		someFeedback = true;
	}

	// Insert a manditory delay if there is no feedback (otherwise transmit buffer can grow extremely large...)
	if (!someFeedback)
	{
		DELAY(1);
	}

	return 0;

}




///////////////////////////////////////////////////////////////////////////
// RealTime mode commands
///////////////////////////////////////////////////////////////////////////

int BHandCANDriver::RTStart(const char *motor, BHMotorProtection motorProtection)
{
	if (m_realtimeMode)
		return -1; // Already in RealTime mode

	// Start RealTime mode and determine which motors are in RealTime mode
	BHMotors bhMotors = toBHMotors(motor);
	realtimeGlobalParameters.MotorsInRealTime = bhMotors;

	// Check that there is exactly 1 motor mode set for each active motor
	for (unsigned int n = 0; n < puckTwos.pucksWithMotors; n++)
	{
		if ((bhMotors >> n) & 1)
		{
			int nModes = 0;
			if (realtimeMotorParameters[n].ControlVelocity) nModes++;
			if (realtimeMotorParameters[n].ControlTorque) nModes++;
			if (realtimeMotorParameters[n].ControlPosition) nModes++;

			if (nModes != 1)
			{
				printf("pucksInHandRTStart - nModes != 1\n");
				return -1; // motor mode has not been set correctly
			}
		}
	}

	// Clear TSTOP only if it is requested to be disabled
	realtimeGlobalParameters.motorProtect = motorProtection;
	int r;
	for (unsigned int n = 0; n < puckTwos.pucksWithMotors; n++)
	{
		if ((bhMotors >> n) & 1)
		{
			if (realtimeGlobalParameters.motorProtect & BHMotorTSTOPProtect)
			{
				// Set default value for TSTOP for motor protection
				r = pucks[n]->setPropertyUInt32(PROP_TSTOP, BHAND_DEFAULT_TSTOP);
				if (r) return r; // double default TSTOP so that position mode has a chance
			}
			else if (realtimeGlobalParameters.motorProtect & BHMotorTorqueLimitProtect)
			{
				// Disable TSTOP safety and rely on software torque limiting for motor protection
				r = pucks[n]->setPropertyUInt32(PROP_TSTOP, 0); if (r) return r;
			}
			else
			{
				// Set default value for TSTOP for motor protection
				r = pucks[n]->setPropertyUInt32(PROP_TSTOP, BHAND_DEFAULT_TSTOP);
				if (r) return r;
			}
		}
	}

	// Set RealTime motor modes
	for (unsigned int n = 0; n < puckTwos.pucksWithMotors; n++)
	{
		realtimeMotorParameters[n].Stopped = true;
		if ((bhMotors >> n) & 1)
		{
			int numTries = 0;
			while (realtimeMotorParameters[n].Stopped)
			{
				if (numTries++ == 2)
					return -1; // the Puck has not responded to the control request
				if (realtimeMotorParameters[n].ControlTorque)
				{
					realtimeMotorParameters[n].ControlPresentTorque = 0;
					r = pucks[n]->setPropertyUInt32(PROP_T, 0); if (r) return r;
					r = pucks[n]->setPropertyUInt32(PROP_MODE, 2); if (r) return r;
				}
				if (realtimeMotorParameters[n].ControlPosition)
				{
					r = pucks[n]->getPropertyUInt32(PROP_P, (unsigned int *)&(realtimeMotorParameters[n].ControlPresentPosition)); if (r) return r;
					r = pucks[n]->setPropertyUInt32(PROP_MODE, 3); if (r) return r;
				}
				if (realtimeMotorParameters[n].ControlVelocity)
				{
					// Switch to Velocity mode (set reference velocity to 0 first)
					realtimeMotorParameters[n].ControlPresentVelocity = 0;
					r = pucks[n]->setPropertyUInt32(PROP_V, 0); if (r) return r;
					r = pucks[n]->setPropertyUInt32(PROP_MODE, 4); if (r) return r;
				}

				// Check to be sure that motor mode is correctly set
				unsigned int mode;
				r = pucks[n]->getPropertyUInt32(PROP_MODE, &mode); if (r) return r;
				if ((realtimeMotorParameters[n].ControlTorque && mode == 2) ||
					(realtimeMotorParameters[n].ControlPosition && mode == 3) ||
					(realtimeMotorParameters[n].ControlVelocity && mode == 4))
					realtimeMotorParameters[n].Stopped = false;

				if (realtimeMotorParameters[n].Stopped)
					printf("Inner Mode not set, trying again %d\n", n);
			}
		}
	}


	// Initialization for velocity calculation
	for (unsigned int n = 0; n < puckTwos.pucksWithMotors; n++)
	{
		if ((bhMotors >> n) & 1)
		{
			if (realtimeMotorParameters[n].FeedbackVelocity)
			{
				realtimeMotorParameters[n].FeedbackPosition = true; // Position feedback is required to compute velocity
				realtimeMotorParameters[n].FeedbackPresentVelocity = 0; // Fill with 0 until enough time passes to compute velocity

				r = pucks[n]->getPropertyUInt32(PROP_P, (unsigned int *)&(realtimeMotorParameters[n].velLastPositionEncoderTicks)); if (r) return r;
				realtimeMotorParameters[n].tmr.Start();
			}
		}
	}

	updateTmr.Start();

	// Set BarrettHand Supervisory/RealTime module state to be RealTime mode
	m_realtimeMode = true;

	return 0;
}

int BHandCANDriver::RTSetFlags(const char *motor, bool LCV, int LCVC, bool LCPG,
                         bool LFV, int LFVC, bool LFS, bool LFAP, bool LFDP, int LFDPC)
{
	BHMotors bhmotors = toBHMotors(motor);

	if (pucksInHandRTSet(bhmotors, "LCV", LCV)) return -1;   // Set    LCV flags
	                                                         // Ignore LCVC
	                                                         // Ignore LCPG
	if (pucksInHandRTSet(bhmotors, "LFV", LFV))  return -1;  // Set    LFV flags
	                                                         // Ignore LFVC
	if (pucksInHandRTSet(bhmotors, "LFS", LFS)) return -1;   // Set    LFS flags
	if (pucksInHandRTSet(bhmotors, "LFAP", LFAP)) return -1; // Set    LFAP flags
	                                                         // Ignore LFDP
	                                                         // Ignore LFDPC

	return 0;
}

int BHandCANDriver::RTSetFlags(const char *motor, bool LCV, int LCVC, bool LCPG, bool LCT,
                         bool LFV, int LFVC, bool LFS, bool LFAP, bool LFDP, int LFDPC,
                         bool LFBP, bool LFAIN, bool LFDPD, bool LFT)
{
	BHMotors bhmotors = toBHMotors(motor);

	if (pucksInHandRTSet(bhmotors, "LCV", LCV)) return -1;   // Set    LCV flags
	                                                         // Ignore LCVC
	                                                         // Ignore LCPG
	if (pucksInHandRTSet(bhmotors, "LCT", LCT)) return -1;   // Set    LCT flags

	if (pucksInHandRTSet(bhmotors, "LFV", LFV))  return -1;  // Set    LFV flags
	                                                         // Ignore LFVC
	if (pucksInHandRTSet(bhmotors, "LFS", LFS)) return -1;   // Set    LFS flags
	if (pucksInHandRTSet(bhmotors, "LFAP", LFAP)) return -1; // Set    LFAP flags
	                                                         // Ignore LFDP
	                                                         // Ignore LFDPC
	                                                         // Ignore LFBP, LFAIN, LFDPD, and LFT for now (handle LFT?)

	return 0;
}


void BHandCANDriver::RTHandProtectMotors()
{
	BPuck2Manager *puckManager = puckTwos.puckManager;

	// Get elapsed time between calls
	updateTmr.Stop();
	double elapsed = updateTmr.GetDurationInMilliSecs();
	updateTmr.Start();

	// Batch Get Property (Torque)
	int torque[4] = { BHAND_FINGER_MAX_PEAK_TORQUE, BHAND_FINGER_MAX_PEAK_TORQUE, BHAND_FINGER_MAX_PEAK_TORQUE, BHAND_SPREAD_MAX_PEAK_TORQUE}; // 4 motors
	puckManager->startGetPropertyBatch();
	for (unsigned int n = 0; n < puckTwos.pucksWithMotors; n++)
		pucks[n]->addToGetPropertyBatch((unsigned int *)&torque[n]);
	puckManager->getBatchPropertyUInt32(PROP_T);

	for (unsigned int n = 0; n < puckTwos.pucksWithMotors; n++)
	{
		//pucks[n]->getPropertyUInt32(PROP_T, (unsigned int *)&torque[n]);
		if (torque[n] > BHAND_FINGER_MAX_PEAK_TORQUE)
			torque[n] = BHAND_FINGER_MAX_PEAK_TORQUE; // ensures that torque is in valid range
		else if (torque[n] < -BHAND_FINGER_MAX_PEAK_TORQUE)
			torque[n] = -BHAND_FINGER_MAX_PEAK_TORQUE; // ensures that torque is in valid range

		// Exponentially filter torque^2 (instantaneous power dissipation proportional to I^2R)
		const float EXP = 0.99997f; // Not recommended to increase
		float e = pow(EXP, (float)elapsed);
		realtimeMotorParameters[n].texp = (float)(realtimeMotorParameters[n].texp * e + (torque[n]*torque[n]) * (1 - e));

		if (n < 3)
			// fingers
			pucks[n]->setPropertyUInt32(PROP_MT,
				(realtimeMotorParameters[n].texp < BHAND_FINGER_MAX_CONT_TORQUE * BHAND_FINGER_MAX_CONT_TORQUE) ?
					BHAND_FINGER_MAX_PEAK_TORQUE : BHAND_FINGER_MAX_CONT_TORQUE);
		else
			// spread
			pucks[n]->setPropertyUInt32(PROP_MT,
				(realtimeMotorParameters[n].texp < BHAND_SPREAD_MAX_CONT_TORQUE * BHAND_SPREAD_MAX_CONT_TORQUE) ?
					BHAND_SPREAD_MAX_PEAK_TORQUE : BHAND_SPREAD_MAX_CONT_TORQUE);
	}

	//if (lcount++ == 40)
	//{
	//	printf("%3.1f torque = { %d %d %d %d } { %3.0f %3.0f %3.0f %3.0f }\n", elapsed, torque[0], torque[1], torque[2], torque[3],
	//		realtimeMotorParameters[0].texp, realtimeMotorParameters[1].texp, realtimeMotorParameters[2].texp, realtimeMotorParameters[3].texp);
	//	lcount = 0;
	//}
}



int BHandCANDriver::RTUpdate(const char *motor, const char *property, int *values)
{
	BHMotors bhMotors = toBHMotors(motor);

	// Get puck property index
	int puckProperty = getPropertyValue(property);
	if (puckProperty == NONE)
		return -1;

	BPuck2Manager *puckManager = puckTwos.puckManager;

	// Batch Get Property
	puckManager->startGetPropertyBatch();
	for (unsigned int n = 0; n < puckTwos.pucksWithMotors; n++)
		if ((bhMotors >> n) & 1)
			pucks[n]->addToGetPropertyBatch((unsigned int *)&values[n]);

	// Block until properties are read
	return puckManager->getBatchPropertyUInt32(puckProperty);
}

int BHandCANDriver::RTUpdate(bool control, bool feedback)
{
	m_control = control;
	m_feedback = feedback;
	return m_moduleSuperReal->ComRequest(BHREQ_REALTIME);
}

int BHandCANDriver::RTUpdatePPS(BHMotors bhMotors)
{
	BPuck2Manager *puckManager = puckTwos.puckManager;

	unsigned int numMotors = 0;

	// Send set TACT property messages to pucks
	for (unsigned int n = 0; n < puckTwos.pucksWithMotors; n++)
		if ((bhMotors >> n) & 1)
		{
			if (!realtimeMotorParameters[n].ControlPosition) // PPS data not returned in position mode, so don't request it
			{
				pucks[n]->setPropertyUInt32(PROP_TACT, 2);
				numMotors++;
			}
		}

	for (unsigned int n = 0; n < numMotors * 5; n++)
	{
		int puckID;
		unsigned char data[8];
		int dataLength;

		// Read an asynchronous message
		BHiResTimer tmr;
		tmr.Start();
		int err = 1;
		while (err)
		{
			err = puckManager->readMsgAsync(&puckID, data, &dataLength);
			if (tmr.GetElapsedInMilliSecs() > 1000)
				return -1; // just in case a message is missed by the Puck
		}

		// Check that there is 64-bits of data
		if (dataLength != 8)
			return -1;

		// Parse feedback
		for (unsigned int m = 0; m < 4; m++)
		{
			if (pucks[m]->getID() != (unsigned int)puckID)
				continue;

			// Get sensor group ID
			int i = (data[0] >> 4) * 5;

#ifdef PPS_DEMO
			const int scalePressure = 100;

			// Store PPS sensor readings
			if (i < MAX_PPS_ELEMENTS)
			{
				FeedbackPresentPPS[m][i++][filterIndexPPS] = scalePressure * (((int)data[0]&0x000F)<<8 | ((int)data[1]&0x00FF));
				if (i < MAX_PPS_ELEMENTS)
				{
					FeedbackPresentPPS[m][i++][filterIndexPPS] = scalePressure * (((int)data[2]&0x00FF)<<4 | ((int)data[3]&0x00F0)>>4);
					if (i < MAX_PPS_ELEMENTS)
					{
						FeedbackPresentPPS[m][i++][filterIndexPPS] = scalePressure * (((int)data[3]&0x000F)<<8 | ((int)data[4]&0x00FF));
						if (i < MAX_PPS_ELEMENTS)
						{
							FeedbackPresentPPS[m][i++][filterIndexPPS] = scalePressure * (((int)data[5]&0x00FF)<<4 | ((int)data[6]&0x00F0)>>4);
							if (i < MAX_PPS_ELEMENTS)
							{
								FeedbackPresentPPS[m][i++][filterIndexPPS] = scalePressure * (((int)data[6]&0x000F)<<8 | ((int)data[7]&0x00FF));
							}
						}
					}
				}
			} // Done storing PPS sensor readings
#else
			// Store PPS sensor readings
			if (i < MAX_PPS_ELEMENTS)
			{
				realtimeMotorParameters[m].FeedbackPresentPPS[i++] = ((int)data[0]&0x000F)<<8 | ((int)data[1]&0x00FF);
				if (i < MAX_PPS_ELEMENTS)
				{
					realtimeMotorParameters[m].FeedbackPresentPPS[i++] = ((int)data[2]&0x00FF)<<4 | ((int)data[3]&0x00F0)>>4;
					if (i < MAX_PPS_ELEMENTS)
					{
						realtimeMotorParameters[m].FeedbackPresentPPS[i++] = ((int)data[3]&0x000F)<<8 | ((int)data[4]&0x00FF);
						if (i < MAX_PPS_ELEMENTS)
						{
							realtimeMotorParameters[m].FeedbackPresentPPS[i++] = ((int)data[5]&0x00FF)<<4 | ((int)data[6]&0x00F0)>>4;
							if (i < MAX_PPS_ELEMENTS)
							{
								realtimeMotorParameters[m].FeedbackPresentPPS[i++] = ((int)data[6]&0x000F)<<8 | ((int)data[7]&0x00FF);
							}
						}
					}
				}
			} // Done storing PPS sensor readings
#endif

		}

	}

#ifdef PPS_DEMO
	// Demo: View 1 % of the full PPS range in the Visual
	// Instructions: Uncomment filter/tare variables, set scalePressure to 100, and uncomment this block of code
	static int ncount = 0;

	for (unsigned int m = 0; m < 4; m++)
	{
		for (int i = 0; i < MAX_PPS_ELEMENTS; i++)
		{
			realtimeMotorParameters[m].FeedbackPresentPPS[i] = 0;
			for (int n = 0; n < PPS_FILTER_SIZE; n++)
			{
				realtimeMotorParameters[m].FeedbackPresentPPS[i] += FeedbackPresentPPS[m][i][n];
			}
			realtimeMotorParameters[m].FeedbackPresentPPS[i] /= PPS_FILTER_SIZE;
		}

		if (ncount++ == 300 * 3)
		{
			for (int i = 0; i < MAX_PPS_ELEMENTS; i++)
			{
				TaredPPS[m][i] = realtimeMotorParameters[m].FeedbackPresentPPS[i];
				ncount = 0;
			}
		}


		for (int i = 0; i < MAX_PPS_ELEMENTS; i++)
		{
			realtimeMotorParameters[m].FeedbackPresentPPS[i] -= TaredPPS[m][i];
		}


		if (m == 3)
		{
			printf("\n");
			for (int i = 0; i < 15; i++)
			{
				printf("%4d ", realtimeMotorParameters[m].FeedbackPresentPPS[i]);
			}
			printf("\r");
		}
	}

	if (++filterIndexPPS == PPS_FILTER_SIZE)
		filterIndexPPS = 0;
#endif

	return 0;
}

int BHandCANDriver::RTAbort()
{
	int result;
	for (unsigned int n = 0; n < puckTwos.pucksWithMotors; n++)
	{
		if (((realtimeGlobalParameters.MotorsInRealTime >> n) & 1) == 0)
			continue;

		// Restore TSTOP
		result = pucks[n]->setPropertyUInt32(PROP_TSTOP, BHAND_DEFAULT_TSTOP);
		result = pucks[n]->setPropertyUInt32(PROP_MODE, 0); // Idle mode
		result = pucks[n]->setPropertyUInt32(PROP_MT, (n < 3) ? BHAND_FINGER_MAX_PEAK_TORQUE : BHAND_SPREAD_MAX_PEAK_TORQUE); // restore max torque
	}

	m_realtimeMode = false;
	return 0;
}

int BHandCANDriver::pucksInHandRTSet(BHMotors bhMotors, const char *property, int value)
{
	//printf("pucksInHandRTSet(%d, %s, %d)\n", bhMotors, property, value);

	if (m_realtimeMode)
		return -1; // Already in RealTime mode

	// Get RealTime motor property index
	int rtMotorProperty = getPropertyValue(property, propBHRealTimeMotor);
	if (rtMotorProperty == NONE)
		return -1;

	//printf("pucksInHandRTSet() rtMotorProperty = %d\n", rtMotorProperty);

	// Set each RealTime motor property
	for (unsigned int i = 0; i < puckTwos.pucksWithMotors; i++)
	{
		if ((bhMotors >> i) & 1)
		{
			switch (rtMotorProperty)
			{
				case REALTIME_LCV:   { realtimeMotorParameters[i].ControlVelocity  = (value ? 1 : 0); break; }
				case REALTIME_LCT:   { realtimeMotorParameters[i].ControlTorque    = (value ? 1 : 0); break; }
				case REALTIME_LCP:   { realtimeMotorParameters[i].ControlPosition  = (value ? 1 : 0); break; }
				case REALTIME_LFV:   { realtimeMotorParameters[i].FeedbackVelocity = (value ? 1 : 0); break; }
				case REALTIME_LFS:   { realtimeMotorParameters[i].FeedbackStrain   = (value ? 1 : 0); break; }
				case REALTIME_LFAP:  { realtimeMotorParameters[i].FeedbackPosition = (value ? 1 : 0); break; }
				case REALTIME_LFPPS: { realtimeMotorParameters[i].FeedbackPPS      = (value ? 1 : 0); break; }
				case REALTIME_LFT:   { realtimeGlobalParameters.FeedbackTemperature = (value ? 1 : 0); break; }
			}
		}
	}
	//printf("pucksInHandRTSet(%d, %s, %d) done\n", bhMotors, property, value);
	return 0;
}

int BHandCANDriver::pucksInHandRTGet(BHMotors bhMotors, const char *property, int *value)
{
	//printf("pucksInHandRTGet(%d, %s, ...)\n", bhMotors, property, value);

	if (m_realtimeMode)
		return -1; // Already in RealTime mode

	// Get RealTime motor property index
	int rtMotorProperty = getPropertyValue(property, propBHRealTimeMotor);
	if (rtMotorProperty == NONE)
		return -1;

	//printf("pucksInHandRTGet() rtMotorProperty = %d\n", rtMotorProperty);

	if (rtMotorProperty == REALTIME_LFT)
	{
		value[0] = realtimeGlobalParameters.FeedbackTemperature ? 1 : 0;
		return 0;
	}

	// Get each RealTime motor property
	unsigned int pCount = 0;
	for (unsigned int i = 0; i < puckTwos.pucksWithMotors; i++)
	{
		if ((bhMotors >> i) & 1)
		{
			switch (rtMotorProperty)
			{
				case REALTIME_LCV:   { value[pCount] = realtimeMotorParameters[i].ControlVelocity  ? 1 : 0; break; }
				case REALTIME_LCT:   { value[pCount] = realtimeMotorParameters[i].ControlTorque    ? 1 : 0; break; }
				case REALTIME_LCP:   { value[pCount] = realtimeMotorParameters[i].ControlPosition  ? 1 : 0; break; }
				case REALTIME_LFV:   { value[pCount] = realtimeMotorParameters[i].FeedbackVelocity ? 1 : 0; break; }
				case REALTIME_LFS:   { value[pCount] = realtimeMotorParameters[i].FeedbackStrain   ? 1 : 0; break; }
				case REALTIME_LFAP:  { value[pCount] = realtimeMotorParameters[i].FeedbackPosition ? 1 : 0; break; }
				case REALTIME_LFPPS: { value[pCount] = realtimeMotorParameters[i].FeedbackPPS      ? 1 : 0; break; }
			}
			pCount++;
		}
	}
	return 0;
}


// Control
int BHandCANDriver::RTSetVelocity(const char motor, int velocity)
{
	int n = motor - '1';

	if (n >= 0 && (unsigned int)n < puckTwos.pucksWithMotors)
	{
		realtimeMotorParameters[n].ControlPresentVelocity = velocity;
		return 0;
	}
	else
		return -1;
}
int BHandCANDriver::RTSetGain(const char motor, int gain)
{
	return 0;
}
int BHandCANDriver::RTSetTorque(const char motor, int torque)
{
	int n = motor - '1';

	if (n >= 0 && (unsigned int)n < puckTwos.pucksWithMotors)
	{
		realtimeMotorParameters[n].ControlPresentTorque = torque;
		return 0;
	}
	else
		return -1;
}
int BHandCANDriver::RTSetPosition(const char motor, int position)
{
	int n = motor - '1';

	if (n >= 0 && (unsigned int)n < puckTwos.pucksWithMotors)
	{
		realtimeMotorParameters[n].ControlPresentPosition = position;
		return 0;
	}
	else
		return -1;
}

// Feedback
char BHandCANDriver::RTGetVelocity(const char motor)
{
	int n = motor - '1';
	if (n >= 0 && (unsigned int)n < puckTwos.pucksWithMotors)
	{
		// Clamp velocity feedback to 8-bit value
		if (realtimeMotorParameters[n].FeedbackPresentVelocity > 127)
			return 127;
		else if (realtimeMotorParameters[n].FeedbackPresentVelocity < -127)
			return -127;

		return realtimeMotorParameters[n].FeedbackPresentVelocity;
	}
	return 0;
}
unsigned char BHandCANDriver::RTGetStrain(const char motor)
{
	int n = motor - '1';
	return (n >= 0 && (unsigned int)n < puckTwos.pucksWithMotors) ? (realtimeMotorParameters[n].FeedbackPresentStrain >> 4) : 0;
}
int BHandCANDriver::RTGetPosition(const char motor)
{
	int n = motor - '1';
	return (n >= 0 && (unsigned int)n < puckTwos.pucksWithMotors) ? realtimeMotorParameters[n].FeedbackPresentPosition : 0;
}
char BHandCANDriver::RTGetDeltaPos(const char motor)
{
	return 0;
}
int BHandCANDriver::RTGetBreakawayPosition(const char motor)
{
	return 0;
}
int BHandCANDriver::RTGetTemp()
{
	return realtimeGlobalParameters.Temperature;
}
unsigned char BHandCANDriver::RTGetAIN(const char motor)
{
	return 0;
}
void BHandCANDriver::RTGetPPS(const char motor, int *pps, int ppsElements)
{
	int n = motor - '1';
	if (n >= 0 && (unsigned int)n < puckTwos.pucksWithMotors)
	{
		for (int i = 0; i < ppsElements; i++)
			pps[i] = realtimeMotorParameters[n].FeedbackPresentPPS[i];
	}
}

// Helper
void BHandCANDriver::SetWaitCallbackFunc(BHCallback waitCallbackFunc) { this->waitCallbackFunc = waitCallbackFunc; }
