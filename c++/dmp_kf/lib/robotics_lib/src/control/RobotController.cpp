/**
 * Copyright (C) 201N_JOINTS AUTH-ARL
*/

#include <robotics_lib/control/RobotController.h>
#include <io_lib/io_utils.h>
#include <math_lib/math.h>
#include <math_lib/quaternions.h>
#include <time_lib/time.h>
#include <yaml-cpp/yaml.h>

namespace as64_
{
RobotController::Params::Params()
{
	stiff_transl = stiff_rot = damp_transl = damp_rot = zeta = 0;
	USE_IN_GRAV_COMP = true;
}

RobotController::RefInput::RefInput()
{
	//q.zeros(N_JOINTS);
	//dq.zeros(N_JOINTS);
	//u = zeros(?);

	p.zeros(4);
	v.zeros(3);

	Q.zeros(4); Q(0)=1;
	v_rot.zeros(3);
}

RobotController::RobotController()
{
	nh_ = ros::NodeHandle("~");
}

void RobotController::init(std::shared_ptr<arl::robot::Robot> robot, const std::string &params_file)
{
	this->robot = robot;
	Ts = robot->cycle;
	N_JOINTS = robot->model->getNrOfJoints(0);

	this->reset();

	// get the parameters from the yaml file
	this->setParams(params_file);
	//this->printParameters(std::cout);

	this->setControlMode(this->control_mode);
}

void RobotController::init(std::shared_ptr<arl::robot::Robot> robot, const RobotController::Params &params)
{
	this->robot = robot;
	Ts = robot->cycle;
	N_JOINTS = robot->model->getNrOfJoints(0);

	this->reset();

	this->setParams(params);

	this->setControlMode(this->control_mode);
}

void RobotController::setParams(const std::string &params_file)
{
	YAML::Node config = YAML::LoadFile(params_file);

	try { this->params.stiff_transl = config["stiff_transl"].as<double>(); }
	catch(std::exception &e) { this->params.stiff_transl = 0; }

	try { this->params.stiff_rot = config["stiff_rot"].as<double>(); }
	catch(std::exception &e) { this->params.stiff_rot = 0; }

	try { this->params.damp_transl = config["damp_transl"].as<double>(); }
	catch(std::exception &e) { this->params.damp_transl = 0; }

	try { this->params.damp_rot = config["damp_rot"].as<double>(); }
	catch(std::exception &e) { this->params.damp_rot = 0; }

	double zeta;
	try { zeta = config["zeta"].as<double>(); }
	catch(std::exception &e) { zeta = 0; }
	this->setZeta(zeta);

	try { this->params.USE_IN_GRAV_COMP = config["USE_IN_GRAV_COMP"].as<bool>(); }
	catch(std::exception &e) { this->params.USE_IN_GRAV_COMP = true; }

	//get control method for contact
	try { this->params.control_mode_name = config["control_method"].as<std::string>(); }
	catch(std::exception &e) { this->params.control_mode_name = "Cartesian"; }

	this->setImpedanceParams();
}

void RobotController::setParams(const RobotController::Params &params)
{
	this->params = params;
	this->setZeta(params.zeta);
	this->setImpedanceParams();
}

void RobotController::printParams(std::ostream &out)
{
	out << "stiff_transl = " << this->params.stiff_transl << std::endl;
	out << "damp_transl = " << this->params.damp_transl << std::endl;
	out << "stiff_rot = " << this->params.stiff_rot << std::endl;
	out << "damp_rot = " << this->params.damp_rot << std::endl;
	out << "zeta = " << this->params.zeta << std::endl;
	out << "control_method = " << this->params.control_mode_name << std::endl;
}

void RobotController::setControlMode(const std::string &control_mode_name)
{
	this->params.control_mode_name = control_mode_name;
	arl::robot::Mode ctrl_mode;

	if(!this->params.control_mode_name.compare("Cart_Imp")) //compare() returns 0 if equal.
	{
		ctrl_mode = arl::robot::IMPEDANCE_CONTROL;
	}
	else if (!this->params.control_mode_name.compare("Joint_Imp"))
	{
		ctrl_mode = arl::robot::TORQUE_CONTROL;
	}
	else if (!this->params.control_mode_name.compare("Joint_Pos"))
	{
		ctrl_mode = arl::robot::POSITION_CONTROL;
	}
	else
	{
		std::cout << "Method: " << this->params.control_mode_name << " is unknown.\n"
		          << "Candidates are: \"Cart_Imp\", \"Joint_Imp\", \"Joint_Pos\"" << std::endl;
		std::cout << "Using \"Joint_Imp\" instead." << std::endl;

		ctrl_mode = arl::robot::TORQUE_CONTROL;
		this->params.control_mode_name = "Joint_Imp"; //default method
	}

	this->setControlMode(ctrl_mode);
}

void RobotController::setControlMode(arl::robot::Mode ctrl_mode)
{
	this->control_mode = ctrl_mode;

	if (this->control_mode == robot->mode) return; // already in the desired control mode

	this->stop(); // stop the robot gracefully in case it has non zero velocity

	arma::vec temp_k(6), temp_c(6);

	// print the control mode
	switch (this->control_mode)
	{
		case arl::robot::IMPEDANCE_CONTROL:
			temp_k << params.stiff_transl << params.stiff_transl << params.stiff_transl << params.stiff_rot << params.stiff_rot << params.stiff_rot; //stiffness
			temp_c.fill(params.zeta);
			robot->setCartStiffness(temp_k);
			robot->setCartDamping(temp_c);
			std::cout << "Using Cartesian Impedance control" << std::endl;
			break;
		case arl::robot::TORQUE_CONTROL:
			std::cout << "Using Joint Impedance control" << std::endl;
			break;
		case arl::robot::POSITION_CONTROL:
			std::cout << "Using Joint Position control" << std::endl;
			break;
	}

	// set the control mode
	this->robot->setMode(this->control_mode);

}

void RobotController::setZeta(double zeta)
{
	this->params.zeta = zeta;
	if (this->params.zeta<0.0) this->params.zeta=0.0;
	else if (this->params.zeta>1.0) this->params.zeta=1.0;
}

void RobotController::reset()
{
	time = 0.0;

	q.zeros(N_JOINTS);
	qdot.zeros(N_JOINTS);
	pose.zeros(3,4);
	p.zeros(3);
	pdot.zeros(3);
	Q.zeros(4), Q(0)=1;
	R.eye(3,3);
	omega.zeros(3);
	V.zeros(6);
	Fee.zeros(6);

	q_ref.zeros(N_JOINTS);
	qdot_ref.zeros(N_JOINTS);
	pose_ref.zeros(3,4);
	p_ref.zeros(3);
	pdot_ref.zeros(3);
	Q_ref.zeros(4); Q_ref(0)=1;
	R_ref.eye(3,3);
	omega_ref.zeros(3);
	V_ref.zeros(6);
	Fee_ref.zeros(6);

	J.zeros(6, N_JOINTS);
	Jt.zeros(3, N_JOINTS);
	Jr.zeros(3, N_JOINTS);
	K_imp.zeros(6, 6);
	D_imp.zeros(6, 6);

	u.zeros(N_JOINTS);
	e_p.zeros(3);
	e_o.zeros(3);
	Qdiff.zeros(4); Qdiff(0)=1;
	friction.zeros(N_JOINTS);

	this->setImpedanceParams();
}

void RobotController::setImpedanceParams()
{
	//Impedance parameters
	arma::vec temp_k, temp_c;
	temp_k << params.stiff_transl << params.stiff_transl << params.stiff_transl << params.stiff_rot << params.stiff_rot << params.stiff_rot; //stiffness
	K_imp = diagmat(temp_k);
	temp_c << params.damp_transl << params.damp_transl << params.damp_transl << params.damp_rot << params.damp_rot << params.damp_rot; //damping
	D_imp = diagmat(temp_c);
}

void RobotController::setRefInput(const RobotController::RefInput &refIn)
{
	if (this->STOP_ROBOT) return;

	this->refIn = refIn;
}

void RobotController::gotoJointPosition(const arma::vec &q_ref, double time)
{
	if (this->robot->mode != arl::robot::POSITION_CONTROL)
	{
			this->setControlMode(arl::robot::POSITION_CONTROL);
	}

	if (time < 0) time = 5;
	this->robot->setJointTrajectory(q_ref, time);
}

void RobotController::measure()
{
	time += this->Ts;

	//read end-effector pose
	robot->getTaskPose(pose);

	robot->getTwist(V); //read Cartesian velocity (J*q_dot) [Does not work well in Cartesian impedance]
	if(!this->params.control_mode_name.compare("Cartesian")) //because there is a bug in V for this mode
	{
		V.subvec(0,2) = (pose.col(3) - p)/this->Ts; //so calculate it here [only for position so far]
	}

	//read Jacobian
	robot->getJacobian(J);
	Jt = J.submat(0, 0, 2, 6); //update translat.  Jacobian
	Jr = J.submat(3, 0, 5, 6); //update rotational Jacobian

	robot->getJointPosition(q); //read joint position
	robot->getJointVelocity(qdot); //read joint velocity

	//read end-effector forces
	robot->getExternalWrench(Fee);

	// update current position & orientation
	p = pose.col(3);
	R = pose.submat(0, 0, 2, 2);

	//update current orientation
	Q = as64_::math_::rotm2quat(R);
	// update orientation error
	arma::vec te=(Q.t()*Q_ref); if (te(0)<0.0) Q_ref=-Q_ref; //avoid discontinuity
	Qdiff = as64_::math_::quatDiff(Q, Q_ref);
	e_o = -2.0*Qdiff.rows(1, 3);
}

void RobotController::JointImpedanceControl()
{

	if (this->STOP_ROBOT || this->params.USE_IN_GRAV_COMP)
	{
		u.zeros(N_JOINTS);
	}
	else
	{
		//Ott's (3.18 page 38, Ott) cartesian impedance controller without inertia reshaping
		u = -Jt.t()*( K_imp.submat(0,0,2,2)*(p-p_ref) + D_imp.submat(0,0,2,2)*(V.subvec(0,2)-pdot_ref))
			+Jr.t()*( K_imp.submat(3,3,5,5)*e_o - D_imp.submat(3,3,5,5)*V.subvec(3,5) ); //the last term is friction compensation
	}

	robot->setJointTorque(u);
}

void RobotController::CartImpedanceControl()
{
	// if (!path->loop_closed) { //loop is still open
	// 	pose_ref.submat(0,3,2,3)  = p; //mirror position values to avoid CP limit
	// 	p_ref = p; //mirror p_ref for the first control loop when we close the task cycle
	//
	// 	//Extra damping in free-space when K_imp=0
	// 	u.subvec(0,2) = D_imp.submat(0,0,2,2)*V.subvec(0,2);
	// }
	// else {
	//
	// 	pose_ref.submat(0,3,2,3)  = p_ref;
	// 	robot->setCartStiffness(diagvec(K_imp));
	//
	// 	u.subvec(0,2) = D_imp.submat(0,0,2,2)*V.subvec(0,2) - getConstraints(); //enable constraint Fee
	// }
	// robot->setTaskPose(pose_ref);
	//
	// // It seems that setWrench() applies XY in the wrong way. This is a crappy way to fix it temporarily
	// double tmp = u(0);
	// u(0) = u(1);
	// u(1) = tmp;
	//
	// robot->setWrench(u.subvec(0,5)); //apply extra damping and constraint Fee
}

void RobotController::JointPositionControl()
{
	if (this->STOP_ROBOT)
	{
		double a = 0.98;
		qdot = -a*qdot;
		// q += Ts*qdot;
		// q_ref = q;
		q_ref += Ts*qdot;
	}

	this->robot->setJointPosition(q_ref);

}

void RobotController::update()
{}

void RobotController::command()
{
	//Select which method to use. They should have the same result. "Joint" is preferred because it is thoroughly tested.
	if(this->control_mode == arl::robot::IMPEDANCE_CONTROL)
	{
		CartImpedanceControl();
	}
	else if (this->control_mode == arl::robot::TORQUE_CONTROL)
	{
		JointImpedanceControl();
	}
	else if (this->control_mode == arl::robot::POSITION_CONTROL)
	{
		JointPositionControl();
	}
	else
	{
		throw std::runtime_error("RobotController::command(): Unknown control mode...");
	}

}

bool RobotController::run()
{
	// a mutex in robot should be locked to ensure no other controller is running
	// on this robot
	if (ros::ok() && robot->isOk())
	{
		measure();
		update();
		command();
		robot->waitNextCycle();
		ros::spinOnce();
		return true;
	}
	else
	{
		return false;
	}

}

bool RobotController::stop()
{
	std::cout << "[RobotController::stop()]: Stopping controller...\n";

	this->STOP_ROBOT = true;

	double vel_tol_stop = 1e-3;
	while (arma::norm(V) > vel_tol_stop)
	{
		this->run();
	}

	this->STOP_ROBOT = false; // reset flag

	std::cout << "[RobotController::stop()]: Controller stopped...\n";
}

} // namespace as64_
