#include "bhand_lib/BhandHWInterface.h"

#include <iostream>

BhandHWInterface::BhandHWInterface(const std::string &handType, bool rt_control_enable)
{
	initialize(handType, rt_control_enable);
}

BhandHWInterface::~BhandHWInterface()
{

}

// Initializes the Barrett Hand.
void BhandHWInterface::initialize(const std::string &handType, bool rt_control_enable)
{
	int err; // Return value (error) of all BHand calls
	std::string buf; // buffer for the error message

	// Set hardware description before initialization
	// int hwIndex = BHandHardware::getBHandHardwareIndex("BH8-262");
	// int hwIndex = BHandHardware::getBHandHardwareIndex("BH8-280");
	this->hand_type = handType;
	int hwIndex = BHandHardware::getBHandHardwareIndex(hand_type.c_str());
	if (hwIndex < 0)
  {
		throw std::runtime_error("\n\nThe API has not been compiled to include target hand.\n");
	}

	setHardwareDesc(hwIndex);
	//bool use280Config = (strcmp(getHardwareDesc()->getModelNumber(), "BH8-280") == 0);
	//printf("\nuse280Config = %d\n", use280Config);
	if ((err = handInitWithMenu(((BHand *)this)))){
		//this->printErrorMessage(err);
		// return;
		printErrorMessage(buf, err);
		throw std::runtime_error(buf);
	}
	//if (err = InitSoftware(com_port, THREAD_PRIORITY_TIME_CRITICAL))
	//	Error();

	//printf("Initialization...");
	if ((err = InitHand("123S"))){
		//this->printErrorMessage(err);
		//return;
		printErrorMessage(buf, err);
		throw std::runtime_error(buf);
	}
	//printf(" Done\n");

	initialized = true;

	if (rt_control_enable) enableRTControl();
}

int BhandHWInterface::command(const char *send, char *receive)
{
	return Command(send, receive);
}

// Stops the Barrett Hand and shuts down the motor.
void BhandHWInterface::terminate()
{
	RTAbort();
	StopMotor("123S");
	initialized = RT_control_enabled = false;
}

void BhandHWInterface::printErrorMessage(std::string &buf, int err_id) const
{
	const int buff_size = 100;
	char temp_buff[buff_size];
	snprintf(temp_buff, buff_size, "ERROR: %d\n%s\n", err_id, ErrorMessage(err_id));

	buf = temp_buff;
}

void BhandHWInterface::printErrorMessage(int err_id) const
{
	std::string buf;
	printErrorMessage(buf, err_id);
	std::cerr << buf;
}

void BhandHWInterface::enableRTControl()
{
	int err; // Return value (error) of all BHand calls

	char motor[] = "123S";

	control_velocity_flag                = true;   // LCV   Loop Control Velocity Flag
	control_propgain_flag                = false;  // LCPG  Loop Control Proportional Gain Flag
	control_torque_flag                  = false;  // LCT   Loop Control Torque Flag
	feedback_velocity_flag               = true;   // LFV   Loop Feedback Velocity Flag
	feedback_strain_flag                 = true;   // LFS   Loop Feedback Stain Flag
	feedback_position_flag               = true;   // LFAP  Loop Feedback Absolute Position Flag
	feedback_deltapos_flag               = false;  // LFDP  Loop Feedback Delta Position Flag
	feedback_breakaway_position_flag     = false;  // LFBP  Loop Feedback Breakaway Position Flag
	feedback_analog_input_flag           = false;  // LFAIN Loop Feedback Analog Input Flag
	feedback_delta_position_discard_flag = false;  // LFDPD Loop Feedback Delta Position Discard Flag
	feedback_temperature                 = true;   // LFT   Loop Feedback Temperature Flag

	control_position_flag                = false;

	control_velocity_coefficient        = 3;  // LCVC  Loop Control Velocity Coefficient
	feedback_velocity_coefficient       = 1;  // LFVC  Loop Feedback Velocity Coefficient
	feedback_delta_position_coefficient = 1;  // LFDPC Loop Feedback Delta Position Coefficient

	if ((err = RTSetFlags(motor,
                control_velocity_flag, control_velocity_coefficient, control_propgain_flag,
                control_torque_flag, feedback_velocity_flag, feedback_velocity_coefficient,
                feedback_strain_flag, feedback_position_flag, feedback_deltapos_flag,
                feedback_delta_position_coefficient, feedback_breakaway_position_flag,
                feedback_analog_input_flag, feedback_delta_position_discard_flag,
                feedback_temperature ))){
					std::string buf;
					printErrorMessage(buf, err);
					throw std::runtime_error(buf);
					// this->printErrorMessage(err);
	}

	//RTStart( "123S" , BHMotorTSTOPProtect);
	RTStart( "123S" , BHMotorTorqueLimitProtect);
	RTUpdate();

	RT_control_enabled = true;
}

bool BhandHWInterface::isInitialized() const
{
	return initialized;
}

bool BhandHWInterface::isRTControlEnabled() const
{
	return RT_control_enabled;
}

void BhandHWInterface::set_param(const char *motor, const char *propertyName, int value)
{
	int err = Set(motor, propertyName, value);

	if (err){
		if (err == -1){
		  printErrorMessage(err);
		  return;
		}
		std::string buf;
		printErrorMessage(buf, err);
		throw std::runtime_error(buf);
	}
}

void BhandHWInterface::get_param(const char *motor, const char *propertyName, int *result)
{
	int err = Get(motor, propertyName, result);

	if (err){
		if (err == -1){
		  printErrorMessage(err);
		  return;
		}
		std::string buf;
		printErrorMessage(buf, err);
		throw std::runtime_error(buf);
	}
}
