/**
 * Copyright (C) 2017 AUTH-ARL
 */

#ifndef ROBOT_CONTROLLER_64_H
#define ROBOT_CONTROLLER_64_H

#include <cstdlib>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <string>
#include <memory> // for std::shared_ptr
#include <thread>

#include <ros/ros.h>
#include <ros/package.h>

#include <lwr_robot/lwr_robot.h>
#include <autharl_core/robot/controller.h>


namespace as64_
{

class RobotController : public arl::robot::Controller
{
public:
	struct Params
	{
		double stiff_transl; ///< impedance translation stiffness
		double stiff_rot; ///< impedance rotation stiffness
		double damp_transl; ///< impedance translation damping
		double damp_rot; ///< impedance rotation damping
		double zeta;
		std::string control_mode_name; ///< control method for the robot
		bool USE_IN_GRAV_COMP; ///< if true the robot operates in gravity compensation.

		Params();
	};

	struct RefInput
	{
		arma::vec q; ///< joints reference
		arma::vec dq; ///< joint velocities reference

		arma::vec p; ///< Cartesian position reference
		arma::vec v; ///< Cartesian velocity reference

		arma::vec Q; ///< orientation reference
		arma::vec v_rot; ///< angular velocity reference

		arma::vec u; ///< extra input reference

		RefInput();
	};

	RobotController();

	void init(std::shared_ptr<arl::robot::Robot> robot, const std::string &params_file);
	void init(std::shared_ptr<arl::robot::Robot> robot, const RobotController::Params &params);

	void setParams(const RobotController::Params &params);
	void setParams(const std::string &params_file);
	void printParams(std::ostream &out = std::cout);

	void setRefInput(const RobotController::RefInput &refIn);

	void setControlMode(const std::string &control_mode_name);
	void setControlMode(arl::robot::Mode ctrl_mode);

	void measure();
	void update();
	void command();
	bool run();
	bool stop();

	void gotoJointPosition(const arma::vec &q_ref, double time);

private:

	void reset();
	void setZeta(double zeta);
	void setImpedanceParams();
	void CartImpedanceControl();
	void JointImpedanceControl();
	void JointPositionControl();

	double Ts;

	bool STOP_ROBOT; ///< if true, decelerates the robots until it stops

	int N_JOINTS; ///< number robot's joints

	arl::robot::Mode control_mode; ///< TORQUE_CONTROL or IMPEDANCE_CONTROL

	double time; ///< the time elapsed since the start of the program
	ros::NodeHandle nh_;

	arma::vec q, qdot, pose, p, pdot, Q, R, omega, V, Fee;
	arma::vec q_ref, qdot_ref, pose_ref, p_ref, pdot_ref, Q_ref, R_ref, omega_ref, V_ref, Fee_ref;

	arma::mat J, Jt, Jr, K_imp, D_imp;

	arma::vec u, e_p, e_o, Qdiff, friction;

	// //constraints
	// double rate_limit, v_err;

	Params params;
	RefInput refIn;
};

} // namespace as64_

#endif // ROBOT_CONTROLLER_64_H
