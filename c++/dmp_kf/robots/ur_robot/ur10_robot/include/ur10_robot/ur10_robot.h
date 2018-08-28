#ifndef UR10_ROBOT_H
#define UR10_ROBOT_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <chrono>
#include <functional>
#include <thread>
#include <mutex>
#include <memory>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/JointState.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

#include <armadillo>

namespace as64_
{

namespace ur10_
{

enum Mode
{
  STOPPED,          /**< When the robot is stopped and does not accept commands */
  POSITION_CONTROL, /**< For sending position commands */
  VELOCITY_CONTROL, /**< For sending velocity commands */
  FORCE_MODE,        /**< When the robot is in force mode */
  FREEDRIVE_MODE,    /**< When the robot is in freedrive mode */
};

class Robot
{

  struct RobotState
  {
    uint64_t timestamp_sec;
    uint64_t timestamp_nsec;

    arma::vec q, dq;
    arma::mat pose;
    arma::mat Jrobot;
    arma::vec pos, Q;
    arma::vec v_lin, v_rot;
    arma::vec wrench;
    arma::vec jTorques;

    RobotState()
    {
      q.resize(6);
      dq.resize(6);
      pose.resize(4,4);
      pos.resize(3);
      Q.resize(4);
      v_lin.resize(3);
      v_rot.resize(3);
      wrench.resize(6);
      jTorques.resize(6);
    }
  };

  struct LoggedData
  {
    arma::rowvec Time;
    arma::mat q_data, dq_data;
    arma::mat pos_data, Q_data;
    arma::mat V_data;
    arma::mat wrench_data;
    arma::mat jTorques_data;
  };

public:
  Robot();
  ~Robot();

  void position_control_mode();
  void velocity_control_mode();

  void freedrive_mode();
  void end_freedrive_mode();

  void teach_mode();
  void end_teach_mode();

  void force_mode(const arma::vec &task_frame, const arma::vec &selection_vector,
                  const arma::vec &wrench, int type, const arma::vec &limits);
  void end_force_mode();
  void force_mode_set_damping(double damping);

  void movej(const arma::vec &q, double a=1.4, double v=1.05, double t=0, double r=0) const;
  void movel(const arma::vec &p, double a=1.2, double v=0.25, double t=0, double r=0) const;

  void speedj(arma::vec dq, double a=1.4, double t=-1) const;
  void speedl(arma::vec dp, double a=1.2, double t=-1) const;

  void stopj(double a) const;
  void stopl(double a) const;
  //
  // void position_deviation_warning(enabled, threshold=0.8)  const;

  void set_gravity(const arma::vec &g) const;
  void set_payload(double m, const arma::vec &CoG) const;
  void set_payload_cog(const arma::vec &CoG) const;
  void set_payload_mass(double m) const;
  void set_tcp(const arma::vec &pose) const;

  void sleep(double t) const;
  void powerdown() const;

  void waitNextCycle();

  double getControlCycle() const { return this->cycle; }
  double getTime() const { return (rSt.timestamp_sec-time_offset + rSt.timestamp_nsec*1e-9); }
  arma::vec getJointPosition() const { return rSt.q; }
  arma::mat getTaskPose() const { return rSt.pose; }
  arma::vec getTaskPosition() const { return rSt.pos; }
  arma::vec getTaskOrientation() const { return rSt.Q; }

  arma::vec getJointVelocity() const { return rSt.dq; }
  arma::vec getTaskVelocity() const { return arma::join_vert(rSt.v_lin, rSt.v_rot); }

  arma::vec getTaskWrench() const;
  arma::vec getJointTorque() const { return rSt.jTorques; }
  arma::mat getJacobian() const { return rSt.Jrobot; }

  void setMode(const ur10_::Mode &mode);
  ur10_::Mode getMode() const { return this->mode; }

  void setJointTrajectory(const arma::vec &qT, double duration);
  void setJointPosition(const arma::vec &qd);
  void setJointVelocity(const arma::vec &dqd);
  void setTaskPose(const arma::mat &pose);
  void setTaskVelocity(const arma::vec &Twist);

  void stop() {}


  arma::mat forwardKinematic(const arma::vec &q) const;
  arma::vec inverseKinematic(const arma::mat &T) const;


  // void setJointVelocity(const arma::vec &input);
  // void setJointTorque(const arma::vec &input);
  // setWrench
  // setTaskPose

  bool isOk() const { return true; }

  void printRobotState(std::ostream &out=std::cout) const;
  void launch_printRobotStateThread(double freq=50, std::ostream &out=std::cout);
  void stop_printRobotStateThread();

  void load_URScript(const std::string &path_to_URScript);
  void execute_URScript() const;

  void getRobotState(RobotState &robotState) const;

  void startLogging();
  void stopLogging();
  void saveLoggedData(const std::string filename, bool binary=true, int precision=7);

  std::string ur_script;

  double cycle;

private:

  arma::wall_clock timer;
  bool timer_start;

  Mode mode;

  bool logging_on;

  uint64_t time_offset;


  std::mutex robotState_mtx;
  std::thread printRobotState_thread;
  bool printRobotStateThread_running;

  RobotState rSt;
  LoggedData log_data;

  ros::NodeHandle n;

  ros::AsyncSpinner spinner;

  std::string command_ur10_topic;
  std::string read_wrench_topic;
  std::string read_toolVel_topic;
  std::string read_jointState_topic;
  std::string base_frame;
  std::string tool_frame;

  ros::Publisher pub2ur10;
  ros::Subscriber wrench_sub;
  ros::Subscriber toolVel_sub;
  ros::Subscriber jointState_sub;

  tf2_ros::Buffer tfBuffer;
  std::shared_ptr<tf2_ros::TransformListener> tfListener;
  geometry_msgs::TransformStamped transformStamped;

  std::string print_vector(const arma::vec &v) const
  {
    std::ostringstream out;
    out << "[" << v(0);
    for (int i=1;i<v.size();i++) out << "," << v(i);
    out << "]";

    return out.str();
  }

  void urScript_command(const std::string &cmd) const
  {
    std_msgs::String cmd_str;
    cmd_str.data = cmd;
    pub2ur10.publish(cmd_str);
  }

  // Callbacks
  void readWrenchCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg);
  void readToolVelCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);
  void readJointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
  void readTaskPoseCallback();

  void printRobotStateThreadFun(double freq=50, std::ostream &out=std::cout);

  void parseConfigFile();

  void set_BaseLink0_and_Link6Ee_transforms();

  void command_mode(const std::string &mode) const;

  void logDataStep();

  arma::mat T_base_link0, T_link0_base;
  arma::mat T_link6_ee, T_ee_link6;
};

} // namespace ur10_

} // namespace as64_

#endif
