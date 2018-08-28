////////////////////
//  Barrett Hand  //
////////////////////

#include "bhand_lib/BarrettHand.h"

BarrettHand::BarrettHand()
{
  ctrl_cycle = 0.005; // control cycle 5 ms
  std::string hand_type ="BH8-280";
  bool enable_rt_control = true;

	set_max_min_pos();

  bh.reset(new BhandHWInterface(hand_type, enable_rt_control));
  urdf_model.reset(new BhandUrdfModel);
  freedrive_model.reset(new BhandFreedriveModel);

}

BarrettHand::~BarrettHand()
{
	bh->terminate();
}

bool BarrettHand::isOk() const
{
  return is_ok;
}

void BarrettHand::enable()
{
  is_ok = true;
}

std::string BarrettHand::getErrMsg() const
{
  return err_msg;
}

BarrettHand::Mode BarrettHand::getMode() const
{
  return mode;
}

void BarrettHand::setMode(const BarrettHand::Mode &m)
{
  mode = m;
}

// Updates the Barrett Hand.
// Must be called at every control cycle.
void BarrettHand::update()
{
	if (bh->isRTControlEnabled()) bh->RTUpdate();
}

double BarrettHand::getCtrlCycle() const
{
  return ctrl_cycle;
}

// Gets RealTime absolute position feedback for the desired motor.
// The loop feedback absolute position (LFAP) flag must be set to receive absolute position feedback.
double BarrettHand::getFingerPosition(int i)
{
  // Check if the feedback position flag is set
  // if (!feedback_position_flag)
  //   throw std::runtime_error("Error: BarrettHand: getFingerPosition: The loop feedback absolute position (LFAP) flag must be set to receive absolute position feedback.");

  if (i < 0)
  {
    // In case of negative value the convention is that we want the position of
    // every finger.
    return bh->RTGetPosition('G');
  }
  else if (i == 0)
  {
    // In case of zero ID the convention is that we want the position of the
    // spread joint.
    return bh->RTGetPosition('4');
  }

  // In any other case we check if the ID is one of the fingers and return this
  // value
  check_finger_index(i);
  return ticks2rad(i, bh->RTGetPosition(i + '0'));
}

// Gets RealTime velocity feedback for the desired motor.
// The loop feedback velocity (LFV) flag must be set to receive velocity feedback.
double BarrettHand::getFingerVelocity(int i)
{
	check_finger_index(i);

	// if (!feedback_velocity_flag)
	// 	throw std::runtime_error("Error: BarrettHand: getFingerVelocity: The loop feedback velocity (LFV) flag must be set to receive velocity feedback.");

	return bh->RTGetVelocity(i + '0');
}

// Gets RealTime strain gauge feedback for the desired motor.
// The loop feedback strain (LFS) flag must be set to receive strain gauge feedback.
double BarrettHand::getFingerTorque(int i)
{
  // Check if the feedback position flag is set

  // if (!feedback_strain_flag)
  //   throw std::runtime_error("Error: BarrettHand: getFingerTorque: The loop feedback strain (LFS) flag must be set to receive strain gauge feedback.");

  if (i == 0)
  {
    // In case of zero ID the convention is that we want the position of the
    // spread joint.
    return bh->RTGetStrain('S');
  }

  // In any other case we check if the ID is one of the fingers and return this
  // value
  check_finger_index(i);
  return getNewtonMetersFromSgValue(bh->RTGetStrain(i + '0'));
}

// Sets RealTime control velocity reference for the desired motor.
void BarrettHand::setFingerVelocity(int vel, int i)
{
  int err;

  // Check if the feedback position flag is set
  // if (!control_velocity_flag)
	// 	throw std::runtime_error("Error: BarrettHand: setFingerVelocity: The loop control velocity (LCV) flag must be set to send velocity references to the hand.");

  if (i == 0)
  {
    // In case of zero ID the convention is that we send velocity to the
    // spread joint.
    err = bh->RTSetVelocity('4', vel);
  }
  else
  {
    // In any other case we check if the ID is one of the fingers so command the
    // velocity accordingly
    check_finger_index(i);
    err = bh->RTSetVelocity(i + '0', vel);
  }

  if (err) errorHandler(err);
}

// Sets RealTime position reference for the desired motor.
// RealTime position control is only possible with the 280 hand.
// The loop control position (LCP) flag needs to be set to send desired position references to the hand.
void BarrettHand::setFingerPosition(double pos, int i)
{
  int err;
  // Check if the feedback position flag is set
  // if (!control_position_flag)
	// 	throw std::runtime_error("Error: BarrettHand: setFingerPosition: The loop control position (LCP) flag must be set to send position references to the hand.");

  if (i == 0)
  {
    // In case of zero ID the convention is that we send velocity to the
    // spread joint.
    err = bh->RTSetPosition('4', pos);
  }
  else
  {
    // In any other case we check if the ID is one of the fingers so command the
    // velocity accordingly
    check_finger_index(i);
    err = bh->RTSetPosition(i + '0', pos);
  }
  if (err) errorHandler(err);
}

void BarrettHand::set_max_min_pos()
{
  joint_limits_ticks.push_back(std::pair<int, int>(0, 35939)); // spread
  joint_limits_ticks.push_back(std::pair<int, int>(6, 195100)); // finger 1
  joint_limits_ticks.push_back(std::pair<int, int>(3, 195250)); // finger 2
  joint_limits_ticks.push_back(std::pair<int, int>(12, 194900)); // finger 3

  joint_limits.push_back(std::pair<double, double>(0.0, 0.0));
  joint_limits.push_back(std::pair<double, double>(0.0, 2.44)); // finger 1
  joint_limits.push_back(std::pair<double, double>(0.0, 2.44)); // finger 2
  joint_limits.push_back(std::pair<double, double>(0.0, 2.44)); // finger 3
}

// Sets RealTime control proportional gain for the desired motor.
void BarrettHand::setFingerGain(int gain, int i)
{
	check_finger_index(i);

	int err = bh->RTSetGain(i + '0', gain);
  if (err) errorHandler(err);
}

// Sets RealTime control torque for the desired motor.
// The loop control torque (LCT) flag needs to be set to send 16-bit torque references to the hand.
void BarrettHand::setFingerTorque(int torque, int i)
{
	check_finger_index(i);

	// if (!control_torque_flag)
	// 	throw std::runtime_error("Error: BarrettHand: setFingerTorque: The loop control torque (LCT) flag needs to be set to send 16-bit torque references to the hand.");

	int err = bh->RTSetTorque(i + '0', torque);
  if (err) errorHandler(err);
}

void BarrettHand::errorHandler(int err)
{
  if (err)
  {
    is_ok = false;
    bh->printErrorMessage(err_msg, err);
  }

}

double BarrettHand::ticks2rad(int i, int ticks) const
{
  return joint_limits[i].first + (ticks - joint_limits_ticks[i].first)*(joint_limits[i].second - joint_limits[i].first)/(double)(joint_limits_ticks[i].second - joint_limits_ticks[i].first);
}

int BarrettHand::rads2ticks(int i, double rads) const
{
  return 0.5 + joint_limits_ticks[i].first + (rads - joint_limits[i].first)*(joint_limits_ticks[i].second - joint_limits_ticks[i].first)/(joint_limits[i].second - joint_limits[i].first);
}

double BarrettHand::getNewtonMetersFromSgValue(int sg_value) const
{
  double max_tip_torque = 40 * 0.06;  // 5kg of max tip force with 6cm distal link size
  double min_tip_torque = -40 * 0.06;  // -5kg of min tip force with 6cm distal link size
  double min_sg = 0;
  double max_sg = 255;

  double sg_perc = (static_cast<double>(sg_value) + min_sg) / (max_sg - min_sg);
  return sg_perc * (max_tip_torque - min_tip_torque) + min_tip_torque ;
}

// Checks whether the referenced finger's value is between 1 and 3
void BarrettHand::check_finger_index(int i) const
{
	if (i<0 || i>3){
		std::ostringstream err_msg;
		err_msg << "ERROR in \"BarrettHand: valid finger indexes are \"0,1,2,3\".\n Given finger index (" << i << ") is invalid.";
		throw std::invalid_argument(err_msg.str());
	}
}
