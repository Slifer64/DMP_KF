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

#ifndef AUTHARL_CORE_ROBOT_ROBOT_SIM_H
#define AUTHARL_CORE_ROBOT_ROBOT_SIM_H

#include <autharl_core/robot/robot.h>
#include <autharl_core/robot/trajectories.h>
#include <ros/ros.h>

namespace arl
{
namespace robot
{
/**
 * @brief A simulated robot implementing the API for kinematics.
 *
 * Currently this dummy class provide only kinematic simulations, which means
 * that does not solve for the dynamics. Dynamic simulations (reading and
 * sending forces/torques to the robot) are not possible. It can be used for
 * visualization purposes.
 */
class RobotSim : public Robot
{
public:
  /**
   * Empty contructor
   */
  RobotSim();

 /**
  * @brief Constructs a simulated object based on its model and the control
  * cycle given by the user
  *
  * @param m The model to be used by the simulated robot
  * @param cycle_rate_sec The The rate that the robot will run in sec. Defaults
  * to 0.001
  */
  RobotSim(std::shared_ptr<arl::robot::Model> m, double cycle_rate_sec);

  /**
   * @brief Waits the remaining amount of time for the next cycle using a
   * ros::Rate
   */
  void waitNextCycle();

  /**
   * @brief return current condition of the robot. By default a running
   * controller will terminate if false is returned.
   */
  bool isOk();

  /**
   * @brief Sets a dummy mode for the robot in order to be compliant with the
   * interfaces of a real robot
   */
  void setMode(arl::robot::Mode mode, int chain_index = 0);

  void setJointTrajectory(const KDL::JntArray &input, double duration, const int chain_index = 0);
  void setJointTrajectory(const arma::vec &input, double duration, const int chain_index = 0);
  void setJointTrajectory(const Eigen::VectorXd &input, double duration, const int chain_index = 0);
  /**
   * @brief Perfoms a 5th-order trajectory. Called by the
   * RobotSim::setJointTrajectory() functions.
   *
   * @attention It will throw an error in case the robot has not been set to
   * Mode::POSITION_CONTROL
   */
  template <typename T>
  void setJointTrajectoryTemplate(const T &input, double duration, const int chain_index = 0)
  {
    if (this->mode == arl::robot::Mode::POSITION_CONTROL)
    {
      // Get the number of joints of this chain
      unsigned int num_joints = model->getNrOfJoints(chain_index);

      // Parse the input to an arma vector in order to use it later for calling
      // the 5th order trajectory
      arma::vec arma_input = arma::zeros<arma::vec>(num_joints);
      for (int i = 0; i < num_joints; i++)
      {
        arma_input(i) = input(i);
      }

      // Inital joint position values
      arma::vec q0 = arma::zeros<arma::vec>(num_joints);
      getJointPosition(q0);
      arma::vec qref = q0;

      // initalize time
      double t = 0.0;
      // the main while
      while (t < duration)
      {
        // compute time now
        t += cycle;
        // update trajectory
        qref = (arl::robot::trajectory::get5thOrder(t, q0, arma_input, duration)).col(0);
        // set joint positions
        setJointPosition(qref);
        waitNextCycle();
      }
    }
    else
    {
      std::cerr << "setJointTrajectory only available in POSITION_CONTROL mode" << std::endl;
    }
  }

  void setJointPosition(const KDL::JntArray &input, const int chain_index = 0);
  void setJointPosition(const arma::vec &input, const int chain_index = 0);
  void setJointPosition(const Eigen::VectorXd &input, const int chain_index = 0);
  /**
   * @brief Called by the RobotSim::setJointPosition() functions. Just writes
   * the input to an internal state variable
   *
   * @attention It will throw an error in case the robot has not been set to
   * Mode::POSITION_CONTROL
   */
  template <typename T> void setJointPositionTemplate(const T &input, const int chain_index = 0)
  {
    if (this->mode == arl::robot::Mode::POSITION_CONTROL || this->mode == arl::robot::Mode::VELOCITY_CONTROL)
    {
      if (chain_index == -1)
      {
        unsigned int index = 0;
        for (unsigned int i = 0; i < model->getNrOfMainChains(); i++)
        {
          for (unsigned int j = 0; j < model->getNrOfJoints(i); j++)
          {
            state.at(i).msr.jnt.pos(j) = input(index);
            index++;
          }
        }
      }
      else
      {
        for (int i = 0; i < model->getNrOfJoints(chain_index); i++)
        {
          state.at(chain_index).msr.jnt.pos(i) = input(i);
        }
      }  // saveLastJointPosition(temp);
    }
    else
    {
      std::cerr << "setJointPosition only available in POSITION_CONTROL mode" << std::endl;
    }
  }

  void getJointPosition(KDL::JntArray &output, const int chain_index = 0);
  void getJointPosition(arma::vec &output, const int chain_index = 0);
  void getJointPosition(Eigen::VectorXd &output, const int chain_index = 0);
  /**
   * @brief Called by the RobotSim::getJointPosition() functions. Just writes
   * the the internal state variable to the output.
   */
  template <typename T> void getJointPositionTemplate(T &output, const int chain_index = 0)
  {
    if (chain_index == -1)
    {
      output.resize(model->getNrOfJoints());
      unsigned int index = 0;
      for (unsigned int i = 0; i < model->getNrOfMainChains(); i++)
      {
        for (unsigned int j = 0; j < model->getNrOfJoints(i); j++)
        {
          output(index) = state.at(i).msr.jnt.pos(j);
          index++;
        }
      }
    }
    else
    {
      output.resize(model->getNrOfJoints(chain_index));
      for (unsigned int i = 0; i < model->getNrOfJoints(chain_index); i++)
      {
        output(i) = state.at(chain_index).msr.jnt.pos(i);
      }
    }
  }

  void getJacobian(KDL::Jacobian &output, const int chain_index = 0);
  void getJacobian(arma::mat &output, const int chain_index = 0);
  /**
   * @brief Called by the RobotSim::getJacobian() functions. Reads the jacobian
   * from the Jacobian Solvers of the used Model.
   */
  void getJacobian(Eigen::MatrixXd &output, const int chain_index = 0);
  template <typename T> void getJacobianTemplate(T &output, const int chain_index = 0)
  {
    static KDL::Jacobian jac(model->getNrOfJoints(chain_index));
    model->jac_solver.at(chain_index).JntToJac(state.at(chain_index).msr.jnt.pos, jac);
    for (size_t i = 0; i < 6; i++)
    {
      for (size_t j = 0; j < model->getNrOfJoints(chain_index); j++)
      {
        output(i, j) = jac(i, j);
      }
    }
  }

  void getTaskPose(KDL::Frame &output, const int chain_index = 0);
  void getTaskPose(arma::mat &output, const int chain_index = 0);
  void getTaskPose(Eigen::MatrixXd &output, const int chain_index = 0);
  /**
   * @brief Called by the RobotSim::getTaskPose() functions. Returns the
   * Cartesian pose by using the current joint positions stored internally and
   * the forward kinematics solvers of the used Model.
   */
  template <typename T> void getTaskPoseTemplate(T &output, const int chain_index = 0)
  {
    KDL::Frame pose;
    model->fk_solver.at(chain_index).JntToCart(state.at(chain_index).msr.jnt.pos, pose);
    for (size_t i = 0; i < 3; i++)
    {
      for (size_t j = 0; j < 4; j++)
      {
        output(i, j) = pose(i, j);
      }
    }
  }

  // Reads the task position based on the measured joint positions by solving the
  // forward kinematics
  void getTaskPosition(KDL::Vector &output, const int chain_index = 0);
  void getTaskPosition(arma::vec &output, const int chain_index = 0);
  void getTaskPosition(Eigen::Vector3d &output, const int chain_index = 0);
  /**
   * @brief Called by the RobotSim::getTaskPosition() functions. The same as
   * RobotSim::getTaskPose(), but only for position
   */
  template <typename T> void getTaskPositionTemplate(T &output, const int chain_index = 0)
  {
    KDL::Frame pose;
    model->fk_solver.at(chain_index).JntToCart(state.at(chain_index).msr.jnt.pos, pose);
    for (size_t i = 0; i < 3; i++)
    {
      output(i) = pose.p(i);
    }
  }

  void getTaskOrientation(KDL::Rotation &output, const int chain_index = 0);
  void getTaskOrientation(arma::mat &output, const int chain_index = 0);
  void getTaskOrientation(Eigen::Matrix3d &output, const int chain_index = 0);
  /**
   * @brief Called by the RobotSim::getTaskOrientation() functions. The same as
   * RobotSim::getTaskPose(), but only for the orientation
   */
  template <typename T> void getTaskOrientationTemplate(T &output, const int chain_index = 0)
  {
    KDL::Frame pose;
    model->fk_solver.at(chain_index).JntToCart(state.at(chain_index).msr.jnt.pos, pose);
    for (size_t i = 0; i < 3; i++)
    {
      for (size_t j = 0; j < 3; j++)
      {
        output(i, j) = pose.M(i, j);
      }
    }
  }

  void getTwist(KDL::Twist &output, const int chain_index = 0);
  void getTwist(arma::vec &output, const int chain_index = 0);
  void getTwist(Eigen::VectorXd &output, const int chain_index = 0);
  /**
   * @brief Called by the RobotSim::getTwist() functions. Calls the
   * getJointVelocity() and getJacobian() and multiply them in order to produce
   * the Cartesian velocity
   */
  template <typename T> void getTwistTemplate(T &output, const int chain_index = 0)
  {
    KDL::Jacobian jac(model->getNrOfJoints(chain_index));
    KDL::JntArray vel(6);
    getJointVelocity(vel, chain_index);
    getJacobian(jac, chain_index);
    Eigen::VectorXd task_vel = jac.data * vel.data;
    output(0) = task_vel(0);
    output(1) = task_vel(1);
    output(2) = task_vel(2);
    output(3) = task_vel(3);
    output(4) = task_vel(4);
    output(5) = task_vel(5);
  }

private:
  /**
   * @brief A ros loop rate in order use it in wait for next cycle
   */
  ros::Rate loop_rate;

  /**
   * @brief Stores internally the current state of the robot
   */
  std::vector<arl::robot::State> state;

  /**
   * @brief Stores the last joint position used for calculation for velocities
   */
  std::vector<KDL::JntArray> last_jnt_pos;
};
}  // namespace robot
}  // namespace arl

#endif  // AUTHARL_CORE_ROBOT_ROBOT_SIM_H
