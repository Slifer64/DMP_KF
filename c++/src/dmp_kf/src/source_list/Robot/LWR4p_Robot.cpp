#include <dmp_kf/Robot/LWR4p_Robot.h>

LWR4p_Robot::LWR4p_Robot():N_JOINTS(7)
{
  ros::Rate *loop_rate;
  bool use_sim = 0;

  ros::NodeHandle("~").getParam("use_sim", use_sim);

  // Create generic robot model
  std::shared_ptr<arl::robot::Model> model;
  // Initialize generic robot model with kuka-lwr model
  model.reset(new lwr::robot::Model());
  // Create generic robot
  if (use_sim == 0) {
    // Initialize generic robot with the kuka-lwr model
    robot.reset(new lwr::robot::Robot(model, "Kuka Robot"));
    ROS_INFO_STREAM("Robot created successfully.");
  } else {
    // Initialize generic robot with the kuka-lwr model
    robot.reset(new arl::robot::RobotSim(model, 0.001));
    loop_rate = new ros::Rate(1/robot->cycle);
    ROS_INFO_STREAM("Simulation robot created successfully.");
  }
}

LWR4p_Robot::~LWR4p_Robot()
{

}

bool LWR4p_Robot::isOk()
{
  return (robot->isOk() || robot->mode==arl::robot::Mode::STOPPED);
}

void LWR4p_Robot::init()
{
  vel_cmd = arma::vec().zeros(6);

  readParams();

  setSafetyStatus(Robot::SafetyStatus::OK);

  this->update();
}

double LWR4p_Robot::getControlCycle() const
{
  return robot->cycle;
}

void LWR4p_Robot::update()
{
  robot->waitNextCycle();
}

void LWR4p_Robot::command()
{
  arma::mat J;
  arma::vec dq;

  switch (this->getMode())
  {
    case Robot::Mode::VELOCITY_CONTROL:
      robot->getJacobian(J);
      dq = arma::pinv(J)*vel_cmd;
      robot->setJointVelocity(dq);
      break;
    case Robot::Mode::FREEDRIVE_MODE:
      robot->setJointTorque(arma::vec().zeros(N_JOINTS));
      break;
    case Robot::Mode::IDLE_MODE:
      // do nothing for kuka
      break;
  }
}

void LWR4p_Robot::setMode(const Robot::Mode &mode)
{
  if (mode == this->getMode()) return;

  switch (mode){
    case VELOCITY_CONTROL:
      robot->setMode(arl::robot::Mode::VELOCITY_CONTROL);
      PRINT_INFO_MSG("Robot set in VELOCITY_CONTROL MODE.\n");
      break;
    case FREEDRIVE_MODE:
      robot->setMode(arl::robot::Mode::TORQUE_CONTROL);
      PRINT_INFO_MSG("Robot set in FREEDRIVE MODE.\n");
      break;
    case IDLE_MODE:
      // for (int i=0;i<10;i++)
      // {
      //   robot->setJointVelocity(arma::vec().zeros(7));
      //   update();
      // }
      // robot->setMode(arl::robot::Mode::POSITION_CONTROL);
      robot->setMode(arl::robot::Mode::STOPPED);
      PRINT_INFO_MSG("Robot set in IDLE MODE.\n");
      break;
  }
  this->mode = mode;
}

void LWR4p_Robot::stop()
{
  robot->setMode(arl::robot::Mode::STOPPED);
}

void LWR4p_Robot::setTaskVelocity(const arma::vec &vel)
{
  if (this->getMode() != Robot::Mode::VELOCITY_CONTROL)
  {
    throw std::runtime_error("[ERROR] LWR4p_Robot::setTaskVelocity: Current mode is \"" + getModeName() + "\".\n");
  }

  vel_cmd = vel;
}

void LWR4p_Robot::setJointTrajectory(const arma::vec &qT, double duration)
{
  robot->setJointTrajectory(qT, duration);
}

arma::vec LWR4p_Robot::getJointPosition() const
{
  arma::vec joint_pos(N_JOINTS);
  robot->getJointPosition(joint_pos);
  return joint_pos;
}

arma::mat LWR4p_Robot::getTaskPose() const
{
  arma::mat task_pose;
  robot->getTaskPose(task_pose);
  task_pose = arma::join_vert(task_pose, arma::rowvec({0,0,0,1}));
  return task_pose;
}

arma::vec LWR4p_Robot::getTaskPosition() const
{
  arma::vec task_pos(3);
  robot->getTaskPosition(task_pos);
  return task_pos;
}

arma::vec LWR4p_Robot::getTaskOrientation() const
{
  arma::vec task_orient(4);
  arma::mat R;
  robot->getTaskOrientation(R);
  task_orient = rotm2quat(R);
  return task_orient;
}

arma::vec LWR4p_Robot::getTaskWrench() const
{
  arma::vec Fext;

  robot->getExternalWrench(Fext);
  Fext = -Fext;

  arma::vec sign_Fext = arma::sign(Fext);
  arma::vec Fext2 = Fext - sign_Fext%Fext_dead_zone;
  Fext2 = 0.5*(arma::sign(Fext2)+sign_Fext)%arma::abs(Fext2);

  return Fext2;
}
