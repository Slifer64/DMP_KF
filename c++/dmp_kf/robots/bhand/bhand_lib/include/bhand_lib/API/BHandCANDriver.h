///////////////////////////////////////////////////////////////////////////////
//                   (C) Barrett Technology Inc. 2009-2010                   //
///////////////////////////////////////////////////////////////////////////////

#ifndef BHAND_CAN_DRIVER_H
#define BHAND_CAN_DRIVER_H


#include "BHandDriver.h"

#include "PucksInHand.h"
#include "PucksInHandCommon.h"


class BHandCANDriver : public BHandDriver
{
public:

	///////////////////////////////////////////////////////////////////////////
	// CAN BHandDriver Constructor/Deconstructor
	///////////////////////////////////////////////////////////////////////////

	BHandCANDriver(BHand *bhand, BHandSupervisoryRealtime *moduleSuperReal);
	virtual ~BHandCANDriver();


	void SetWaitCallbackFunc(BHCallback waitCallbackFunc);


	///////////////////////////////////////////////////////////////////////////
	// BarrettHand Driver Methods
	///////////////////////////////////////////////////////////////////////////

	// Methods for initializing and closing the BarrettHand CAN driver
	int Initialize();
	void Close();


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


private:

	void waitForStop(BHMotors bhMotors = 0x0f);

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


	// Support methods for Supervisory and RealTime control of the BarrettHand
	//int ExecuteSupervisoryCall();
	int ExecuteRealtimeCall();
	int HandleSupervisoryCall();

	// RealTime
	void RTHandProtectMotors();
	int pucksInHandRTGet(BHMotors bhMotors, const char *property, int *value);
	int pucksInHandRTSet(BHMotors bhMotors, const char *property, int value);

	int RTUpdatePPS(BHMotors bhMotors);

	///////////////////////////////////////////////////////////////////////////
	// Pointers for Puck2s and Puck2Manager instances for low-level access to the hardware
	///////////////////////////////////////////////////////////////////////////

	BPuck2Manager pucksInHand;
	BPuck2 **pucks;

	BHAND_PUCK2 puckTwos;
	BHCallback waitCallbackFunc;  // method called often with longer running supervisory commands

	bool m_control;
	bool m_feedback;

	bool m_realtimeMode;
	BHiResTimer updateTmr; // software motor overheating protection

	RealTimeGlobalParameters realtimeGlobalParameters;
	RealTimeMotorParameters *realtimeMotorParameters;
};

#endif // BHAND_CAN_DRIVER_H
