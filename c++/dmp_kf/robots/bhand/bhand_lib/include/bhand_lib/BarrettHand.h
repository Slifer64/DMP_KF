////////////////////
//  Barrett Hand  //
////////////////////

#ifndef BARRETT_HAND_64_H
#define BARRETT_HAND_64_H

#include <iostream>
#include <cstdlib>
#include <cmath>
#include <cstring>
#include <memory>
#include <deque>
#include <stdexcept>
#include <sstream>

#include "BhandHWInterface.h"
#include "BhandUrdfModel.h"
#include "BhandFreedriveModel.h"

class BarrettHand
{
public:
  enum Mode
  {
    IDLE,
    JOINT_POS_CONTROL,
    JOINT_VEL_CONTROL,
    FREEDRIVE,
    PROTECTIVE_STOP
  };

  BarrettHand();
  ~BarrettHand();

  bool isOk() const;
  void enable();
  std::string getErrMsg() const;

  BarrettHand::Mode getMode() const;
  void setMode(const BarrettHand::Mode &m);

  void update();

  double getCtrlCycle() const;

  double getFingerPosition(int i); // const;
  double getFingerVelocity(int i); // const;
  double getFingerTorque(int i); // const;

  void setFingerVelocity(int vel, int i);
  void setFingerPosition(double pos, int i);

private:

	void set_max_min_pos();
  void setFingerTorque(int torque, int i);
  void setFingerGain(int gain, int i);
  void errorHandler(int err);
  void check_finger_index(int i) const;
  double ticks2rad(int i, int ticks) const;
  int rads2ticks(int i, double rad) const;
  double getNewtonMetersFromSgValue(int sg_value) const;

	std::shared_ptr<BhandHWInterface>  bh; // Handles all hand communication
  std::shared_ptr<BhandUrdfModel> urdf_model; // contains the urdf model and kdl chain solvers
  std::shared_ptr<BhandFreedriveModel> freedrive_model; // facilitates the freedrive mode

  Mode mode;

  bool is_ok;
  std::string err_msg;

  double ctrl_cycle;

  std::vector<std::pair<double, double>> joint_limits;
  std::vector<std::pair<int, int>> joint_limits_ticks;


};


#endif
