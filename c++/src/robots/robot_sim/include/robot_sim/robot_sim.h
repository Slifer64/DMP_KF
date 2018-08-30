#ifndef SIMULATED_ROBOT_H
#define SIMULATED_ROBOT_H

#include <iostream>
#include <cstdlib>
#include <fstream>
#include <vector>
#include <memory>
#include <string>
#include <thread>
#include <mutex>
#include <condition_variable>

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/WrenchStamped.h>

#include <armadillo>
#include <robot_sim/utils.h>

namespace as64_
{


class RobotSim
{
public:
  enum Mode
  {
    IDLE,
    FREEDRIVE,
    JOINT_POS_CONTROL,
    JOINT_VEL_CONTROL,
    CART_VEL_CONTROL,
    PROTECTIVE_STOP
  };

  RobotSim();
  ~RobotSim();

  bool isOk() const;
  void enable();
  std::string getErrMsg() const;

  void update();

  void setMode(const RobotSim::Mode &m);
  RobotSim::Mode getMode() const;

  double getCtrlCycle() const;
  int getNumJoints() const;

  void setJointsPosition(const arma::vec &j_pos);
  void setJointsVelocity(const arma::vec &j_vel);
  void setTaskVelocity(const arma::vec &task_vel);
  void setJointsTrajectory(const arma::vec &j_targ, double duration);

  arma::vec getJointsPosition() const;
  arma::vec getJointsVelocity() const;
  arma::mat getTaskPose() const;
  arma::vec getTaskPosition() const;
  arma::vec getTaskOrientation() const;
  arma::mat getJacobian() const;
  arma::vec getJointTorques() const;
  arma::vec getExternalForce() const;

  void printRobotInfo() const;

private:

  std::vector<std::string> mode_name;

  Mode mode;

  int N_JOINTS;

  const double SINGULARITY_THRES;

  std::mutex robot_state_mtx;

  std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver;
  std::shared_ptr<KDL::ChainJntToJacSolver> jac_solver;
  KDL::Chain chain;

  std::string pub_topic;
  ros::NodeHandle node;
  ros::Publisher jState_pub; ///< joint state publisher
  ros::Subscriber jState_sub; ///< joint state subscriber
  sensor_msgs::JointState joint_state_msg;

  std::vector<std::string> joint_names;
  std::vector<double> joint_pos_lower_lim;
  std::vector<double> joint_pos_upper_lim;
  std::vector<double> joint_vel_lim;
  std::vector<double> effort_lim;

  std::vector<std::string> link_names;

  std::string base_link;
  std::string tool_link;

  double ctrl_cycle;
  unsigned long update_time;
  Timer timer;
  arma::vec joint_pos;
  arma::vec joint_prev_pos;
  arma::vec joint_vel;
  arma::vec joint_torques;
  arma::mat pose;
  arma::mat Jacob;
  arma::vec Fext;

  bool check_limits;
  bool  check_singularity;

  bool read_wrench_from_topic;
  std::string wrench_topic;
  ros::Subscriber wrench_sub;

  std::string err_msg;
  bool checkJointPosLimits(const arma::vec &j_pos);
  bool checkJointVelLimits(const arma::vec &dj_pos);
  bool checkSingularity();

  void print_err_msg(const std::string &msg);
  void print_info_msg(const std::string &msg);
  void print_warn_msg(const std::string &msg);

  void stop();
  void protectiveStop();

  std::string getModeName() const;

  void setJointsPositionHelper(const arma::vec &j_pos);
  void setJointsVelocityHelper(const arma::vec &j_vel);
  void setTaskVelocityHelper(const arma::vec &task_vel);

  void jStateSubCallback(const sensor_msgs::JointState::ConstPtr& jState);

  void readWrenchCallback(const geometry_msgs::WrenchStamped::ConstPtr& wrench_ptr);
};

}; // namespace as64_

#endif
