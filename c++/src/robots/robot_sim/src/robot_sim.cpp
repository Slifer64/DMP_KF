#include <robot_sim/robot_sim.h>

#include <stdexcept>
#include <sstream>
#include <chrono>
#include <map>
#include <stack>

#include <ros/package.h>
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>

// #define RESET   "\033[0m"
// #define BLACK   "\033[30m"  /* Black */
// #define RED     "\033[31m"  /* Red */
// #define GREEN   "\033[32m"  /* Green */
// #define YELLOW  "\033[33m"  /* Yellow */
// #define BLUE    "\033[34m"  /* Blue */
// #define MAGENTA "\033[35m"  /* Magenta */
// #define CYAN    "\033[36m"  /* Cyan */
// #define WHITE   "\033[37m"  /* White */
// #define BOLD    "\033[1m"   /* Bold */

namespace as64_
{

RobotSim::RobotSim():SINGULARITY_THRES(8e-3)
{
  mode_name.resize(6);
  mode_name[0] = "IDLE";
  mode_name[1] = "FREEDRIVE";
  mode_name[2] = "JOINT_POS_CONTROL";
  mode_name[3] = "JOINT_VEL_CONTROL";
  mode_name[4] = "CART_VEL_CONTROL";
  mode_name[5] = "PROTECTIVE_STOP";

  mode = RobotSim::Mode::IDLE;

  if (!node.getParam("/robot_sim/base_frame",base_link))
  {
    throw std::runtime_error("Failed to load parameter \"/robot_sim/base_frame\" ...\n");
  }

  if (!node.getParam("/robot_sim/tool_frame",tool_link))
  {
    throw std::runtime_error("Failed to load parameter \"/robot_sim/tool_frame\" ...\n");
  }

  if (!node.getParam("/robot_sim/check_limits",check_limits))
  {
    check_limits = false;
  }

  if (!node.getParam("/robot_sim/check_singularity",check_singularity))
  {
    check_singularity = false;
  }

  if (!node.getParam("/robot_sim/ctrl_cycle",ctrl_cycle))
  {
    ctrl_cycle = 0.01;
  }

  if (node.getParam("/robot_sim/wrench_topic",wrench_topic))
  {
    read_wrench_from_topic = true;
  }

  std::string robot_description_name;
  if (!node.getParam("/robot_sim/robot_description_name",robot_description_name))
  {
    throw std::runtime_error("Failed to load parameter \"/robot_sim/robot_description_name\" ...\n");
  }


  urdf::Model urdf_model;
  std::string urdf_file_path = ros::package::getPath("robot_sim") + "/urdf/lwr4p_robot.urdf";

  if (!urdf_model.initParam(robot_description_name.c_str()))
  // if (!urdf_model.initFile(urdf_file_path.c_str()))
  {
    throw std::ios_base::failure("Couldn't load urdf model from \"" + robot_description_name + "\"...\n");
  }

  std::vector<boost::shared_ptr<urdf::Joint>> _joints;
  std::stack<boost::shared_ptr<const urdf::Link>> link_stack;
  link_stack.push(urdf_model.getRoot());

  while (!link_stack.empty())
  {
    auto link = link_stack.top();
    link_stack.pop();

    link_names.push_back(link->name);

    for (int i=0;i<link->child_links.size();i++) link_stack.push(link->child_links[i]);
    for (int i=0;i<link->child_joints.size();i++) _joints.push_back(link->child_joints[i]);
  }

  bool found_base_link = false;
  bool found_tool_link = false;
  for (int i=0;i<link_names.size();i++)
  {
    if (base_link.compare(link_names[i]) == 0) found_base_link = true;
    else if (tool_link.compare(link_names[i]) == 0) found_tool_link = true;
  }

  if (!found_base_link)
    throw std::runtime_error("Couldn't find specified base link \"" + base_link + "\" in the robot urdf model...\n");

  if (!found_tool_link)
    throw std::runtime_error("Couldn't find specified tool link \"" + tool_link + "\" in the robot urdf model...\n");

  // char *joint_type[] = {"UNKNOWN", "REVOLUTE", "CONTINUOUS", "PRISMATIC", "FLOATING", "PLANAR", "FIXED"};

  for (int i=0;i<_joints.size();i++)
  {
    auto joint = _joints[i];
    auto jtype = joint->type;

    if (jtype==urdf::Joint::FIXED || jtype==urdf::Joint::FLOATING) continue;

    if (joint->mimic) continue;

    // std::cout << "===> joint " << i+1 << ": name = " << joint->name << ", type: " << joint_type[jtype] << "\n";
    //
    // boost::shared_ptr<urdf::JointMimic> j_mimic = joint->mimic;
    // if (j_mimic)
    // {
    //   std::cout << "mimic joint: " << j_mimic->joint_name << "\n";
    //   std::cout << "offset = " << j_mimic->offset << "\n";
    //   std::cout << "multiplier = " << j_mimic->multiplier << "\n";
    // }

    joint_names.push_back(joint->name);

    if (jtype==urdf::Joint::CONTINUOUS)
    {
      joint_pos_lower_lim.push_back(-M_PI);
      joint_pos_upper_lim.push_back(M_PI);
    }
    else
    {
      joint_pos_lower_lim.push_back(joint->limits->lower);
      joint_pos_upper_lim.push_back(joint->limits->upper);
    }

    effort_lim.push_back(joint->limits->effort);
    joint_vel_lim.push_back(joint->limits->velocity);
  }

  KDL::Tree tree;
  kdl_parser::treeFromUrdfModel(urdf_model, tree);

  if (!tree.getChain(base_link, tool_link, chain))
  {
    throw std::runtime_error("Failed to create kdl chain from " + base_link + " to " + tool_link + " ...\n");
  }
  else
  {
    fk_solver.reset(new KDL::ChainFkSolverPos_recursive(chain));
    jac_solver.reset(new KDL::ChainJntToJacSolver(chain));
  }

  N_JOINTS = joint_names.size();
  joint_pos.zeros(N_JOINTS);
  joint_prev_pos.zeros(N_JOINTS);
  joint_vel.zeros(N_JOINTS);
  joint_torques.zeros(N_JOINTS);
  pose.resize(4,4);
  Jacob.resize(6,N_JOINTS);
  Fext.zeros(6);

  pub_topic = "/as64_joint_states";
  jState_pub = node.advertise<sensor_msgs::JointState>(pub_topic, 1);

  jState_sub = node.subscribe("/joint_states", 1, &RobotSim::jStateSubCallback, this);

  wrench_sub = node.subscribe(wrench_topic.c_str(), 1, &RobotSim::readWrenchCallback, this);

  update_time = (int)(getCtrlCycle()*1e9);
  timer.start();
}

RobotSim::~RobotSim() {}

void RobotSim::printRobotInfo() const
{
  for (int i=0;i<joint_names.size();i++)
  {
    std::cout << "joint" << i+1 << ": " << joint_names[i] << "\n";
    std::cout << "pos_limits: [" << joint_pos_lower_lim[i] << ", " << joint_pos_upper_lim[i] << "] rad\n";
    std::cout << "vel limit: " << joint_vel_lim[i] << " rad/s\n";
  }
}

bool RobotSim::isOk() const
{
  return getMode()!=RobotSim::Mode::PROTECTIVE_STOP;
}

void RobotSim::enable()
{
  setMode(RobotSim::Mode::IDLE);
}

std::string RobotSim::getErrMsg() const
{
  return err_msg;
}

void RobotSim::update()
{
  std::unique_lock<std::mutex> lck(robot_state_mtx);

  ros::spinOnce();

  joint_vel = (joint_pos - joint_prev_pos) / ctrl_cycle;

  KDL::JntArray jnt(N_JOINTS);
  std::vector<double> q(N_JOINTS);
  std::vector<double> dq(N_JOINTS);
  for (int i=0;i<N_JOINTS;i++)
  {
    jnt(i) = joint_pos(i);
    q[i] = joint_pos(i);
    dq[i] = joint_vel(i);
  }

  KDL::Jacobian J(N_JOINTS);
  jac_solver->JntToJac(jnt, J);
  for (int i=0;i<Jacob.n_rows;i++)
  {
    for (int j=0;j<Jacob.n_cols;j++) Jacob(i,j) = J(i,j);
  }

  KDL::Frame fk;
  fk_solver->JntToCart(jnt, fk);
  for (int i=0;i<3;i++)
  {
    for (int j=0;j<4;j++) pose(i,j) = fk(i,j);
  }
  pose.row(3) = arma::rowvec({0,0,0,1});

  joint_state_msg.name = joint_names;
  joint_state_msg.position = q;
  joint_state_msg.velocity = dq;
  joint_state_msg.effort = std::vector<double>(N_JOINTS, 0);
  jState_pub.publish(joint_state_msg);

  unsigned long elaps_time = timer.elapsedNanoSec();
  if (elaps_time < update_time)
  {
    std::this_thread::sleep_for(std::chrono::nanoseconds(update_time-elaps_time));
  }
  timer.start();
}

double RobotSim::getCtrlCycle() const
{
  return ctrl_cycle;
}

void RobotSim::setMode(const RobotSim::Mode &m)
{
  if (getMode() == m) return;

  if (m == RobotSim::Mode::PROTECTIVE_STOP) protectiveStop();
  else
  {
    // if (getMode()!=RobotSim::Mode::IDLE && getMode()!=RobotSim::Mode::PROTECTIVE_STOP)
    mode = m;
    stop();
    if (getMode() == RobotSim::Mode::PROTECTIVE_STOP) return;
    // mode = m;
    print_info_msg("Mode changed to \"" + getModeName() + "\"\n");
  }
}

RobotSim::Mode RobotSim::getMode() const
{
  return mode;
}

std::string RobotSim::getModeName() const
{
  return mode_name[getMode()];
}

void RobotSim::stop()
{
  update();
  arma::vec q_current = getJointsPosition();
  setJointsPositionHelper(q_current);
  joint_prev_pos = joint_pos;
  update();
  // mode = RobotSim::Mode::IDLE;
  // print_info_msg("Mode changed to \"" + getModeName() + "\"\n");
}

void RobotSim::protectiveStop()
{
  joint_prev_pos = joint_pos;
  update();
  mode = RobotSim::Mode::PROTECTIVE_STOP;
  print_warn_msg("Mode changed to \"" + getModeName() + "\"\n");
}

int RobotSim::getNumJoints() const
{
  return N_JOINTS;
}

void RobotSim::setJointsPosition(const arma::vec &j_pos)
{
  if (getMode() != RobotSim::Mode::JOINT_POS_CONTROL)
  {
    print_warn_msg("Cannot set joints position. Current mode is \"" + getModeName() + "\"\n");
    return;
  }

  setJointsPositionHelper(j_pos);
}

void RobotSim::setJointsPositionHelper(const arma::vec &j_pos)
{
  arma::vec dj_pos = (joint_pos - j_pos) / ctrl_cycle;

  if (check_limits)
  {
    if (!checkJointPosLimits(j_pos)) return;
    if (!checkJointVelLimits(dj_pos)) return;
  }

  if (check_singularity)
  {
    if (!checkSingularity()) return;
  }

  joint_prev_pos = joint_pos;
  joint_pos = j_pos;
}

void RobotSim::setJointsVelocity(const arma::vec &j_vel)
{
  if (getMode() != RobotSim::Mode::JOINT_VEL_CONTROL)
  {
    print_warn_msg("Cannot set joints velocity. Current mode is \"" + getModeName() + "\"\n");
    return;
  }

  setJointsVelocityHelper(j_vel);
}

void RobotSim::setJointsVelocityHelper(const arma::vec &j_vel)
{
  setJointsPositionHelper(joint_pos + j_vel*ctrl_cycle);
}

arma::vec RobotSim::getJointsPosition() const
{
  // std::unique_lock<std::mutex> lck(robot_state_mtx);
  return joint_pos;
}

arma::vec RobotSim::getJointsVelocity() const
{
  // std::unique_lock<std::mutex> lck(robot_state_mtx);
  return joint_vel;
}

void RobotSim::setTaskVelocity(const arma::vec &task_vel)
{
  if (getMode() != RobotSim::Mode::CART_VEL_CONTROL)
  {
    print_warn_msg("Cannot set task velocity. Current mode is \"" + getModeName() + "\"\n");
    return;
  }
  setTaskVelocityHelper(task_vel);
}

void RobotSim::setTaskVelocityHelper(const arma::vec &task_vel)
{
  setJointsVelocityHelper(arma::pinv(Jacob)*task_vel);
}

void RobotSim::setJointsTrajectory(const arma::vec &j_targ, double duration)
{
  if (getMode() != RobotSim::Mode::JOINT_VEL_CONTROL)
  {
    setMode(RobotSim::Mode::JOINT_VEL_CONTROL);
  }

  arma::vec dq = (j_targ-joint_pos)/duration;

  while (isOk())
  {
    update();
    if (arma::norm(joint_pos-j_targ) < 1e-3) break;
    setJointsVelocity(dq);
  }
}

arma::mat RobotSim::getTaskPose() const
{
  // std::unique_lock<std::mutex> lck(robot_state_mtx);
  return pose;
}

arma::vec RobotSim::getTaskPosition() const
{
  // std::unique_lock<std::mutex> lck(robot_state_mtx);
  return pose.submat(0,3,2,3);
}

arma::vec RobotSim::getTaskOrientation() const
{
  // std::unique_lock<std::mutex> lck(robot_state_mtx);
  arma::mat R = pose.submat(0,0,3,3);
  arma::vec quat = rotm2quat(R);
  return quat;
}

arma::mat RobotSim::getJacobian() const
{
  // std::unique_lock<std::mutex> lck(robot_state_mtx);
  return Jacob;
}

arma::vec RobotSim::getJointTorques() const
{
  // std::unique_lock<std::mutex> lck(robot_state_mtx);
  return joint_torques;
}

arma::vec RobotSim::getExternalForce() const
{
  // std::unique_lock<std::mutex> lck(robot_state_mtx);
  return Fext;
}

bool RobotSim::checkJointPosLimits(const arma::vec &j_pos)
{
  for (int i=0;i<N_JOINTS;i++)
  {
    if (j_pos(i)>joint_pos_upper_lim[i] || j_pos(i)<joint_pos_lower_lim[i])
    {
      std::ostringstream out;
      out << joint_names[i] << ": position limit reached: " << j_pos(i) << " rad\n";
      err_msg = out.str();
      print_err_msg(err_msg);
      setMode(RobotSim::Mode::PROTECTIVE_STOP);
      return false;
    }
  }

  return true;
}

bool RobotSim::checkJointVelLimits(const arma::vec &dj_pos)
{
  for (int i=0;i<N_JOINTS;i++)
  {
    if (std::fabs(dj_pos(i))>joint_vel_lim[i])
    {
      std::ostringstream out;
      out << joint_names[i] << ": velocity limit reached: " << dj_pos(i) << " rad/s\n";
      err_msg = out.str();
      print_err_msg(err_msg);
      setMode(RobotSim::Mode::PROTECTIVE_STOP);
      return false;
    }
  }

  return true;
}

bool RobotSim::checkSingularity()
{
  if (getMode()==RobotSim::Mode::IDLE || getMode()==RobotSim::Mode::FREEDRIVE) return true;

  bool singularity_reached = false;

  arma::vec eigval; // = arma::eig_gen(Jacob);
  arma::svd(eigval, Jacob);

  if (arma::min(arma::abs(eigval)) < SINGULARITY_THRES) singularity_reached = true;

  if (singularity_reached)
  {
    err_msg = "Singularity reached!\n";
    print_err_msg(err_msg);
    setMode(RobotSim::Mode::PROTECTIVE_STOP);
    return false;
  }

  return true;
}

void RobotSim::print_err_msg(const std::string &msg)
{
  std::cerr << "\033[1m\033[31m" << "[RobotSim ERROR]: " << msg << "\033[0m";
}

void RobotSim::print_info_msg(const std::string &msg)
{
  std::cout << "\033[1m\033[34m" << "[RobotSim INFO]: " << msg << "\033[0m";
}

void RobotSim::print_warn_msg(const std::string &msg)
{
  std::cout << "\033[1m\033[33m" << "[RobotSim WARNING]: " << msg << "\033[0m";
}

void RobotSim::jStateSubCallback(const sensor_msgs::JointState::ConstPtr& j_state)
{
  if (getMode() != RobotSim::Mode::FREEDRIVE) return;

  std::map<std::string, double> j_map;

  for (int i=0;i<j_state->name.size();i++)
    j_map.insert( std::pair<std::string,double>(j_state->name[i], j_state->position[i]) );

  for (int i=0;i<joint_names.size();i++)
    joint_pos[i] = j_map[joint_names[i]];
  // joint_vel = j_state->velocity;
  // joint_torques = j_state->effort;
}

void RobotSim::readWrenchCallback(const geometry_msgs::WrenchStamped::ConstPtr& wrench_ptr)
{
  if (!read_wrench_from_topic) return;

  Fext(0) = wrench_ptr->wrench.force.x;
  Fext(1) = wrench_ptr->wrench.force.y;
  Fext(2) = wrench_ptr->wrench.force.z;

  Fext(3) = wrench_ptr->wrench.torque.x;
  Fext(4) = wrench_ptr->wrench.torque.y;
  Fext(5) = wrench_ptr->wrench.torque.z;
}

}; // namespace as64_
