/*******************************************************************************
 * Copyright (c) 2016-2017 Automation and Robotics Lab, AUTh
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 ******************************************************************************/

#ifndef AUTHARL_CORE_ROBOT_ROBOT_H
#define AUTHARL_CORE_ROBOT_ROBOT_H

#include <autharl_core/robot/model.h>
#include <autharl_core/robot/state.h>
#include <memory>
#include <vector>
#include <string>
#include <armadillo>

namespace arl
{
namespace robot
{
/**
 * @brief Different robot modes
 */
enum Mode {UNDEFINED = -1000, /**< For internal use */
           STOPPED = -1, /**< When the robot is stopped and does not accept commands */
           POSITION_CONTROL  = 0, /**< For sending position commands */
           VELOCITY_CONTROL  = 1, /**< For sending velocity commands */
           TORQUE_CONTROL    = 2, /**< For sending torque commands */
           IMPEDANCE_CONTROL = 3, /**< For operating in Impedance control */
           JOINT_TRAJECTORY  = 4 /**< Probably should be covered by position control */
          };

/**
 * @brief An abstract class that implements a robot.
 *
 * This class encaptulates functionalities that abstracts any robot hardware.
 * The core functionalities are read from and send to the robot positions,
 * trajectories, velocities or efforts (joint torques or wrenches in task space)
 * either to the joint space or the task space. Also reading the base Jacobian
 * matrix of the robot in each cycle. Furthermore reading or sending joint/task
 * stiffness or damping in compliant robots. Other functionalities include
 * setting the mode of the robot, checking if the hardware is operable etc.
 * This API class aims to support multiple client libraries like KDL, Armadillo
 * or Eigen.
 */
class Robot
{
public:
  /**
   * @brief Empty constructor
   */
  Robot();

  /**
   * @brief Default constructor.
   *
   * @param m The model that this robot will use
   * @param name The name of the robot. Defaults to "Unnamed"
   */
  explicit Robot(std::shared_ptr<Model> m, const std::string& name = "Unnamed");

  /**
   * @brief Sends joint positions to the robot using KDL
   *
   * @param input The input for sending to robot
   * @param chain_index The index of the robots chain. Defaults to 0.
   */
  virtual void setJointPosition(const KDL::JntArray &input, const int chain_index = 0);
  /**
   * @brief Sends joint positions to the robot using Armadillo
   *
   * @param input The input for sending to robot
   * @param chain_index The index of the robots chain. Defaults to 0.
   */
  virtual void setJointPosition(const arma::vec &input, const int chain_index = 0);
  /**
   * @brief Sends joint positions to the robot using Eigen
   *
   * @param input The input for sending to robot
   * @param chain_index The index of the robots chain. Defaults to 0.
   */
  virtual void setJointPosition(const Eigen::VectorXd &input, const int chain_index = 0);

  /**
   * @brief Sends joint trajectories to the robot using KDL
   *
   * @param input The input for sending to robot
   * @param chain_index The index of the robots chain. Defaults to 0.
   */
  virtual void setJointTrajectory(const KDL::JntArray &input, double duration, const int chain_index = 0);
  /**
   * @brief Sends joint trajectories to the robot using Armadillo
   *
   * @param input The input for sending to robot
   * @param chain_index The index of the robots chain. Defaults to 0.
   */
  virtual void setJointTrajectory(const arma::vec &input, double duration, const int chain_index = 0);
  /**
   * @brief Sends joint trajectories to the robot using Eigen
   *
   * @param input The input for sending to robot
   * @param chain_index The index of the robots chain. Defaults to 0.
   */
  virtual void setJointTrajectory(const Eigen::VectorXd &input, double duration, const int chain_index = 0);

  /**
   * @brief Sends joint velocities to the robot using KDL
   *
   * @param input The input for sending to robot
   * @param chain_index The index of the robots chain. Defaults to 0.
   */
  virtual void setJointVelocity(const KDL::JntArray &input, const int chain_index = 0);
  /**
   * @brief Sends joint velocities to the robot using Armadillo
   *
   * @param input The input for sending to robot
   * @param chain_index The index of the robots chain. Defaults to 0.
   */
  virtual void setJointVelocity(const arma::vec &input, const int chain_index = 0);
  /**
   * @brief Sends joint velocities to the robot using Eigen
   *
   * @param input The input for sending to robot
   * @param chain_index The index of the robots chain. Defaults to 0.
   */
  virtual void setJointVelocity(const Eigen::VectorXd &input, const int chain_index = 0);

  /**
   * @brief Sends joint velocities to the robot using KDL
   *
   * @param input The input for sending to robot
   * @param chain_index The index of the robots chain. Defaults to 0.
   */
  virtual void setJointTorque(const KDL::JntArray &input, const int chain_index = 0);
  /**
   * @brief Sends joint torques to the robot using Armadillo
   *
   * @param input The input for sending to robot
   * @param chain_index The index of the robots chain. Defaults to 0.
   */
  virtual void setJointTorque(const arma::vec &input, const int chain_index = 0);
  /**
   * @brief Sends joint torques to the robot using Eigen
   *
   * @param input The input for sending to robot
   * @param chain_index The index of the robots chain. Defaults to 0.
   */
  virtual void setJointTorque(const Eigen::VectorXd &input, const int chain_index = 0);

  /**
   * @brief Sends joint torques to the robot using KDL
   *
   * @param input The input for sending to robot
   * @param chain_index The index of the robots chain. Defaults to 0.
   */
  virtual void setTaskPose(const KDL::Frame &input, const int chain_index = 0);
  /**
   * @brief Sends Cartesian pose to the robot using Armadillo
   *
   * @param input The input for sending to robot
   * @param chain_index The index of the robots chain. Defaults to 0.
   */
  virtual void setTaskPose(const arma::mat &input, const int chain_index = 0);
  /**
   * @brief Sends Cartesian pose to the robot using Eigen
   *
   * @param input The input for sending to robot
   * @param chain_index The index of the robots chain. Defaults to 0.
   */
  virtual void setTaskPose(const Eigen::MatrixXd &input, const int chain_index = 0);
  /**
   * @brief Sends Cartesian pose to the robot using KDL
   *
   * @param input The input for sending to robot
   * @param chain_index The index of the robots chain. Defaults to 0.
   */

  //shouldn't the doxygen comment be place before the function? [fotis]
  virtual void setCartStiffness(const KDL::Wrench &input, const int chain_index = 0);
  virtual void setCartStiffness(const arma::vec &input, const int chain_index = 0);
  virtual void setCartStiffness(const Eigen::VectorXd &input, const int chain_index = 0);

  virtual void setCartDamping(const KDL::Wrench &input, const int chain_index = 0);
  virtual void setCartDamping(const arma::vec &input, const int chain_index = 0);
  virtual void setCartDamping(const Eigen::VectorXd &input, const int chain_index = 0);

  virtual void setTwist(const KDL::Twist &input, const int chain_index = 0);
  /**
   * @brief Sends Cartesian velocity to the robot using Armadillo
   *
   * @param input The input for sending to robot
   * @param chain_index The index of the robots chain. Defaults to 0.
   */
  virtual void setTwist(const arma::vec &input, const int chain_index = 0);
  /**
   * @brief Sends Cartesian velocity to the robot using Eigen
   *
   * @param input The input for sending to robot
   * @param chain_index The index of the robots chain. Defaults to 0.
   */
  virtual void setTwist(const Eigen::VectorXd &input, const int chain_index = 0);

  /**
   * @brief Sends Cartesian velocity to the robot using KDL
   *
   * @param input The input for sending to robot
   * @param chain_index The index of the robots chain. Defaults to 0.
   */
  virtual void setWrench(const KDL::Wrench &input, const int chain_index = 0);
  /**
   * @brief Sends Cartesian force/torque to the robot using Armadillo
   *
   * @param input The input for sending to robot
   * @param chain_index The index of the robots chain. Defaults to 0.
   */
  virtual void setWrench(const arma::vec &input, const int chain_index = 0);
  /**
   * @brief Sends Cartesian force/torque to the robot using Eigen
   *
   * @param input The input for sending to robot
   * @param chain_index The index of the robots chain. Defaults to 0.
   */
  virtual void setWrench(const Eigen::VectorXd &input, const int chain_index = 0);

  /**
   * @brief Reads the current joint positions to the robot using KDL
   *
   * @param output The output given by the robot
   * @param chain_index The index of the robots chain. Defaults to 0.
   */
  virtual void getJointPosition(KDL::JntArray &output, const int chain_index = 0);
  /**
   * @brief Reads the current joint positions from the robot using Armadillo
   *
   * @param output The output given by the robot
   * @param chain_index The index of the robots chain. Defaults to 0.
   */
  virtual void getJointPosition(arma::vec &output, const int chain_index = 0);
  /**
   * @brief Reads the current joint positions from the robot using Eigen
   *
   * @param output The output given by the robot
   * @param chain_index The index of the robots chain. Defaults to 0.
   */
  virtual void getJointPosition(Eigen::VectorXd &output, const int chain_index = 0);

  /**
   * @brief Reads the current joint velocities from the robot using KDL
   *
   * @param output The output given by the robot
   * @param chain_index The index of the robots chain. Defaults to 0.
   */
  virtual void getJointVelocity(KDL::JntArray &output, const int chain_index = 0);
  /**
   * @brief Reads the current joint velocities from the robot using Armadillo
   *
   * @param output The output given by the robot
   * @param chain_index The index of the robots chain. Defaults to 0.
   */
  virtual void getJointVelocity(arma::vec &output, const int chain_index = 0);
  /**
   * @brief Reads the current joint velocities from the robot using Eigen
   *
   * @param output The output given by the robot
   * @param chain_index The index of the robots chain. Defaults to 0.
   */
  virtual void getJointVelocity(Eigen::VectorXd &output, const int chain_index = 0);

  /**
   * @brief Reads the current joint torques from the robot using KDL
   *
   * @param output The output given by the robot
   * @param chain_index The index of the robots chain. Defaults to 0.
   */
  virtual void getJointTorque(KDL::JntArray &output, const int chain_index = 0);
  /**
   * @brief Reads the current joint torques from the robot using Armadillo
   *
   * @param output The output given by the robot
   * @param chain_index The index of the robots chain. Defaults to 0.
   */
  virtual void getJointTorque(arma::vec &output, const int chain_index = 0);
  /**
   * @brief Reads the current joint torques from the robot using Eigen
   *
   * @param output The output given by the robot
   * @param chain_index The index of the robots chain. Defaults to 0.
   */
  virtual void getJointTorque(Eigen::VectorXd &output, const int chain_index = 0);

  /**
   * @brief Reads the current external joint torques from the robot using KDL
   *
   * @param output The output given by the robot
   * @param chain_index The index of the robots chain. Defaults to 0.
   */
  virtual void getJointExternalTorque(KDL::JntArray &output, const int chain_index = 0);
  /**
   * @brief Reads the current external joint torques from the robot using Armadillo
   *
   * @param output The output given by the robot
   * @param chain_index The index of the robots chain. Defaults to 0.
   */
  virtual void getJointExternalTorque(arma::vec &output, const int chain_index = 0);
  /**
   * @brief Reads the current external joint torques from the robot using Eigen
   *
   * @param output The output given by the robot
   * @param chain_index The index of the robots chain. Defaults to 0.
   */
  virtual void getJointExternalTorque(Eigen::VectorXd &output, const int chain_index = 0);

  /**
   * @brief Reads the current joint stiffness from the robot using KDL
   *
   * @param output The output given by the robot
   * @param chain_index The index of the robots chain. Defaults to 0.
   */
  virtual void getJointStiffness(KDL::JntArray &output, const int chain_index = 0);
  /**
   * @brief Reads the current joint stiffness from the robot using Armadillo
   *
   * @param output The output given by the robot
   * @param chain_index The index of the robots chain. Defaults to 0.
   */
  virtual void getJointStiffness(arma::vec &output, const int chain_index = 0);
  /**
   * @brief Reads the current joint stiffness from the robot using Eigen
   *
   * @param output The output given by the robot
   * @param chain_index The index of the robots chain. Defaults to 0.
   */
  virtual void getJointStiffness(Eigen::VectorXd &output, const int chain_index = 0);

  /**
   * @brief Reads the current joint damping from the robot using KDL
   *
   * @param output The output given by the robot
   * @param chain_index The index of the robots chain. Defaults to 0.
   */
  virtual void getJointDamping(KDL::JntArray &output, const int chain_index = 0);
  /**
   * @brief Reads the current joint damping from the robot using Armadillo
   *
   * @param output The output given by the robot
   * @param chain_index The index of the robots chain. Defaults to 0.
   */
  virtual void getJointDamping(arma::vec &output, const int chain_index = 0);
  /**
   * @brief Reads the current joint damping from the robot using Eigen
   *
   * @param output The output given by the robot
   * @param chain_index The index of the robots chain. Defaults to 0.
   */
  virtual void getJointDamping(Eigen::VectorXd &output, const int chain_index = 0);

  /**
   * @brief Reads the current Cartesian pose from the robot using KDL
   *
   * @param output The output given by the robot
   * @param chain_index The index of the robots chain. Defaults to 0.
   */
  virtual void getTaskPose(KDL::Frame &output, const int chain_index = 0);
  /**
   * @brief Reads the current Cartesian pose from the robot using Armadillo
   *
   * @param output The output given by the robot
   * @param chain_index The index of the robots chain. Defaults to 0.
   */
  virtual void getTaskPose(arma::mat &output, const int chain_index = 0);
  /**
   * @brief Reads the current Cartesian pose from the robot using Eigen
   *
   * @param output The output given by the robot
   * @param chain_index The index of the robots chain. Defaults to 0.
   */
  virtual void getTaskPose(Eigen::MatrixXd &output, const int chain_index = 0);

  /**
   * @brief Reads the current Cartesian Position from the robot using KDL
   *
   * @param output The output given by the robot
   * @param chain_index The index of the robots chain. Defaults to 0.
   */
  virtual void getTaskPosition(KDL::Vector &output, const int chain_index = 0);
  /**
   * @brief Reads the current Cartesian position from the robot using Armadillo
   *
   * @param output The output given by the robot
   * @param chain_index The index of the robots chain. Defaults to 0.
   */
  virtual void getTaskPosition(arma::vec &output, const int chain_index = 0);
  /**
   * @brief Reads the current joint positions from the robot using Eigen
   *
   * @param output The output given by the robot
   * @param chain_index The index of the robots chain. Defaults to 0.
   */
  virtual void getTaskPosition(Eigen::Vector3d &output, const int chain_index = 0);

  /**
   * @brief Reads the current Cartesian orientation (Rotation matrix) from the robot using KDL
   *
   * @param output The output given by the robot
   * @param chain_index The index of the robots chain. Defaults to 0.
   */
  virtual void getTaskOrientation(KDL::Rotation &output, const int chain_index = 0);
  /**
   * @brief Reads the current Cartesian orientation (Rotation matrix) from the robot using Armadillo
   *
   * @param output The output given by the robot
   * @param chain_index The index of the robots chain. Defaults to 0.
   */
  virtual void getTaskOrientation(arma::mat &output, const int chain_index = 0);
  /**
   * @brief Reads the current Cartesian orientation (Rotation matrix) from the robot using Eigen
   *
   * @param output The output given by the robot
   * @param chain_index The index of the robots chain. Defaults to 0.
   */
  virtual void getTaskOrientation(Eigen::Matrix3d &output, const int chain_index = 0);

  /**
   * @brief Reads the current Cartesian velocity from the robot using KDL
   *
   * @param output The output given by the robot
   * @param chain_index The index of the robots chain. Defaults to 0.
   */
  virtual void getTwist(KDL::Twist &output, const int chain_index = 0);
  /**
   * @brief Reads the current Cartesian velocity from the robot using Armadillo
   *
   * @param output The output given by the robot
   * @param chain_index The index of the robots chain. Defaults to 0.
   */
  virtual void getTwist(arma::vec &output, const int chain_index = 0);
  /**
   * @brief Reads the current Cartesian velocity from the robot using Eigen
   *
   * @param output The output given by the robot
   * @param chain_index The index of the robots chain. Defaults to 0.
   */
  virtual void getTwist(Eigen::VectorXd &output, const int chain_index = 0);

  /**
   * @brief Reads the current Cartesian force/torque from the robot using KDL
   *
   * @param output The output given by the robot
   * @param chain_index The index of the robots chain. Defaults to 0.
   */
  virtual void getWrench(KDL::Wrench &output, const int chain_index = 0);
  /**
   * @brief Reads the current Cartesian force/torque from the robot using Armadillo
   *
   * @param output The output given by the robot
   * @param chain_index The index of the robots chain. Defaults to 0.
   */
  virtual void getWrench(arma::vec &output, const int chain_index = 0);
  /**
   * @brief Reads the current Cartesian force/torque from the robot using Eigen
   *
   * @param output The output given by the robot
   * @param chain_index The index of the robots chain. Defaults to 0.
   */
  virtual void getWrench(Eigen::VectorXd &output, const int chain_index = 0);

  /**
   * @brief Reads the current external Cartesian force/torque from the robot using KDL
   *
   * @param output The output given by the robot
   * @param chain_index The index of the robots chain. Defaults to 0.
   */
  virtual void getExternalWrench(KDL::Wrench &output, const int chain_index = 0);
  /**
   * @brief Reads the current external Cartesian force/torque from the robot using Armadillo
   *
   * @param output The output given by the robot
   * @param chain_index The index of the robots chain. Defaults to 0.
   */
  virtual void getExternalWrench(arma::vec &output, const int chain_index = 0);
  /**
   * @brief Reads the current external Cartesian force/torque from the robot using Eigen
   *
   * @param output The output given by the robot
   * @param chain_index The index of the robots chain. Defaults to 0.
   */
  virtual void getExternalWrench(Eigen::VectorXd &output, const int chain_index = 0);

  /**
   * @brief Reads the current Cartesian stiffness from the robot using KDL
   *
   * @param output The output given by the robot
   * @param chain_index The index of the robots chain. Defaults to 0.
   */
  virtual void getTaskStiffness(KDL::JntArray &output, const int chain_index = 0);
  /**
   * @brief Reads the current Cartesian stiffness from the robot using Armadillo
   *
   * @param output The output given by the robot
   * @param chain_index The index of the robots chain. Defaults to 0.
   */
  virtual void getTaskStiffness(arma::vec &output, const int chain_index = 0);
  /**
   * @brief Reads the current Cartesian stiffness from the robot using Eigen
   *
   * @param output The output given by the robot
   * @param chain_index The index of the robots chain. Defaults to 0.
   */
  virtual void getTaskStiffness(Eigen::VectorXd &output, const int chain_index = 0);

  /**
   * @brief Reads the current Cartesian damping from the robot using KDL
   *
   * @param output The output given by the robot
   * @param chain_index The index of the robots chain. Defaults to 0.
   */
  virtual void getTaskDamping(KDL::JntArray &output, const int chain_index = 0);
  /**
   * @brief Reads the current Cartesian damping rom the robot using Armadillo
   *
   * @param output The output given by the robot
   * @param chain_index The index of the robots chain. Defaults to 0.
   */
  virtual void getTaskDamping(arma::vec &output, const int chain_index = 0);
  /**
   * @brief Reads the current Cartesian damping from the robot using Eigen
   *
   * @param output The output given by the robot
   * @param chain_index The index of the robots chain. Defaults to 0.
   */
  virtual void getTaskDamping(Eigen::VectorXd &output, const int chain_index = 0);

  /**
   * @brief Reads the current base Jacobian matrix from the robot using KDL
   *
   * @param output The output given by the robot
   * @param chain_index The index of the robots chain. Defaults to 0.
   */
  virtual void getJacobian(KDL::Jacobian &output, const int chain_index = 0);
  /**
   * @brief Reads the current base Jacobian matrix from the robot using Armadillo
   *
   * @param output The output given by the robot
   * @param chain_index The index of the robots chain. Defaults to 0.
   */
  virtual void getJacobian(arma::mat &output, const int chain_index = 0);
  /**
   * @brief Reads the current base Jacobian matrix from the robot using Eigen
   *
   * @param output The output given by the robot
   * @param chain_index The index of the robots chain. Defaults to 0.
   */
  virtual void getJacobian(Eigen::MatrixXd &output, const int chain_index = 0);

  virtual void stop();

  /**
   * @brief Sets the mode of the robot.
   *
   * @param mode The desired Mode of the Robot.
   * @param chain_index The chain that will change mode. Defaults to zero.
   */
  virtual void setMode(Mode mode, int chain_index = 0);
  virtual void measure();

  /**
   * @brief Function that blocks the control loop before the next control cycle
   * starts.
   *
   * Can be implemented by sleeping or waiting for a signal from the robot. The
   * function is not const in case in some implementations you want to change
   * variable (e.g. a flag for reading the velocity by differentiating the
   * position).
   */
  virtual void waitNextCycle() = 0;

  /**
   * @brief Returns if the robot hardware is ok and operable. Should be implemented
   * according to the robot in use.
   *
   * @return True if the robot haven't returns any errors. False otherwise.
   */
  virtual bool isOk() = 0;

  /**
   * @brief Reads the joint velocity using the derivative of the position
   * (previous and current position)
   *
   * @param output The current joint velocities
   * @param chain_index The index of the chain
   */
  template <typename T> void getJointVelocityGeneric(T &output, const int chain_index = 0)
  {
    static unsigned int nr_joints;

    // Get the number of joints of this chain. If index == -1 then the nr of
    // joints is the sum of the joints of the main chains
    if (chain_index == -1)
    {
      nr_joints = model->getNrOfJoints();
    }
    else if (chain_index > -1 && chain_index < model->getNrOfChains())
    {
      nr_joints = model->getNrOfJoints(chain_index);
    }
    else
    {
      std::cerr << "[AUTh-ARL Robot] Get Joint Velocity: The chain index is invalid." << std::endl;
      return;
    }

    output.resize(nr_joints);
    static arma::vec joint_pos, joint_pos_prev_local;
    this->getJointPosition(joint_pos, chain_index);

    static bool first_time = true;
    if (first_time)
    {
      joint_pos_prev_local = joint_pos;
      first_time = false;
    }

    static unsigned int index;
    for (unsigned int i = 0; i < nr_joints; i++)
    {
      index = model->getGlobalIndex(chain_index, i);
      output(i) = (joint_pos(i) - joint_pos_prev_local(index)) / cycle;
    }

    joint_pos_prev_local = joint_pos;
  }

  /**
   * @brief Reads the joint velocity by integrating the position
   * (previous and current position)
   *
   * @param output The commanded joint velocities
   * @param chain_index The index of the chain
   */
  template <typename T> void setJointVelocityGeneric(T &input, const int chain_index = 0)
  {
    // Run only if the user has initilized the robot to Velocity Control
    if (this->mode == arl::robot::Mode::VELOCITY_CONTROL)
    {
      static unsigned int nr_joints;
      if (chain_index == -1)
      {
        nr_joints = model->getNrOfJoints();
      }
      else if (chain_index > -1 && chain_index < model->getNrOfChains())
      {
        nr_joints = model->getNrOfJoints(chain_index);
      }
      else
      {
        std::cerr << "[AUTh-ARL Robot] Get Joint Velocity: The chain index is invalid." << std::endl;
        return;
      }

      static unsigned int index;
      for (unsigned int i = 0; i < nr_joints; i++)
      {
        index = model->getGlobalIndex(chain_index, i);
        joint_pos_prev(index) += input(i) * cycle;
      }
      setJointPosition(joint_pos_prev(arma::span(index - nr_joints + 1, index)), chain_index);
    }
    else
    {
      std::cerr << "setJointVelocity only available in VELOCITY_CONTROL mode" << std::endl;
    }
  }

  /**
   * @brief Stores the previous joint position used by
   * arl::robot::setJointPositionGeneric
   */
  arma::vec joint_pos_prev;
  /**
   * @brief The control cycle of the robot.
   *
   * Can be reading it online by the robot hardware or setted by the contructor
   * of your robot.
   */
  double cycle;

  /**
   * @brief A pointer to the Model that this Robot is using.
   */
  std::shared_ptr<Model> model;


  /**
   * @brief The name of the robot. E.g. "KUKA LWR4".
   */
  std::string robot_name;

  /**
   * @brief The alias of the robot, which is the name in lower case without spaces. E.g. "kuka_lwr4".
   */
  std::string alias;

  /**
   * @brief The current Mode of the robot.
   */
  Mode mode;

protected:
  /**
   * @brief The current state of the robot.
   */
  std::vector<State> state;

  /**
   * Prints a warning that the function is not implemented by the derived
   * class.
   */
  void throwFunctionIsNotImplemented(std::string function_name);
};
}  // namespace robot
}  // namespace arl

#endif  // AUTHARL_CORE_ROBOT_ROBOT_H
