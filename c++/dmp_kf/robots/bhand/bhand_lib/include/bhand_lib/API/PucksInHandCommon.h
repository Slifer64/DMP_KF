#ifndef PUCKSINHAND_COMMON_H
#define PUCKSINHAND_COMMON_H

#include "BPuck2Manager.h"
#include "BPuck2.h"

#include "bhand_motors.h"
#include "bhand_misc.h"

#include "puck2.h"


#define MAX_PPS_ELEMENTS 24

enum BHMotorProtection
{
	BHMotorTSTOPProtect = 1,
	BHMotorTorqueLimitProtect = 2
};

// Forward Declaration of BHand class
class BHand;

typedef struct REALTIME_GLOBAL_PARAMETERS
{
	BHMotors MotorsInRealTime;

	bool FeedbackTemperature;

	int Temperature;

	BHMotorProtection motorProtect;

	// For motors that switched mode
	unsigned int MotorExamineMode;

} RealTimeGlobalParameters;


typedef struct REALTIME_MOTOR_PARAMETERS
{
	// Control over motor mode
	bool ControlVelocity;
	bool ControlTorque;
	bool ControlPosition;

	bool FeedbackVelocity;
	bool FeedbackStrain;
	bool FeedbackPosition;
	bool FeedbackPPS;

	// For motors that switched mode
	bool Stopped;
	int StoppedControl;

	// New parameters
	int ControlPresentVelocity;
	int ControlPresentTorque;
	int ControlPresentPosition;

	int FeedbackPresentPosition;
	int FeedbackPresentVelocity;
	int FeedbackPresentStrain;

	int FeedbackPresentPPS[MAX_PPS_ELEMENTS];

	// Current Limiting
	float texp;

	// For velocity calculation
	BHiResTimer tmr;
	int velLastPositionEncoderTicks;
	double velLastPositionElapsedTime;

} RealTimeMotorParameters;


///////////////////////////////////////////////////////////////////////////////
// The structure that holds state for the 280 BarrettHand
///////////////////////////////////////////////////////////////////////////////

typedef struct
{
	BHand *bhand; // Will point to the BHand instance

	BPuck2Manager *puckManager;
	BPuck2 **puckWithMotor;       // pucks with motors
	unsigned int pucksWithMotors; // number of pucks with motors
} BHAND_PUCK2;

// Increasing these constants may overheat and damage motors
#define BHAND_FINGER_MAX_PEAK_TORQUE 3300
#define BHAND_SPREAD_MAX_PEAK_TORQUE 3300

#define BHAND_FINGER_MAX_CONT_TORQUE 1100
#define BHAND_SPREAD_MAX_CONT_TORQUE 1400

#define BHAND_DEFAULT_TSTOP 250

#endif
