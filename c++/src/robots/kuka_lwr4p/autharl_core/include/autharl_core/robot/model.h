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

#ifndef AUTHARL_CORE_ROBOT_MODEL_H
#define AUTHARL_CORE_ROBOT_MODEL_H

#include <string>
#include <vector>
#include <map>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <memory>

namespace arl
{
namespace robot
{
/**
 * @brief Defines an abstract model for a robot storing information like DOFs,
 * kinematics etc.
 *
 * The Model of a robot contains the following things:
 *   - A number of kinematic chains. For example a bimanual robot can have two
 *   main chains which are the two arms and a number of other chains. The other
 *   chains could be chains from the base of the robot to each joint which can
 *   be utilized for reading the Cartesian pose of the each joint in real time.
 *   - The number of the main chains. As main chains we define the chains
 *   which do not share any common joints between
 *   them and also the sum of their joints are the total number of dinstict
 *   DOFs of the robot. In the bimanual arm example these are the two arms of
 *   the robot.
 *   - The names of the chains and their joints. The convention is that we
 *   store firstly the main chains and then the rest of the chains. If for
 *   example a model has 10 chains and 3 main chains, the indeces 0, 1, 2
 *   denote the three main chains.
 *   - The kinematics of each chain stored as KDL::Chain.
 *   - Forward and inversed kinematics solvers for calculating FKs and Jacobian
 *   matrices
 *
 * A class which derives from Model should always define the following:
 *   - The number of the main chains
 *   - The names of each chain. First the names of the main chains and then the
 *   rest of them.
 *   - The names of each joint of each chain.
 *   - Fill the global indecies (global_index) array
 *
 * A class which derives from Model could define, if desired, the following:
 *   - The kinematic chains as KDL::Chains
 *   - The Forward Kinematic solvers
 *   - Jacobian and Inverse Kinematic solvers
 *
 * Upon availability of a URDF model of the robot someone can parse the URDF to
 * Model class.
 */
class Model
{
public:
  /**
   * @brief An empty constructor.
   */
  Model();

  /**
   * @brief A basic constructor parsing the name of the model.
   */
  explicit Model(const std::string& n);

  /**
   * @brief Returns the total number of the chains of the model
   *
   * @return The number of the chains
   */
  unsigned int getNrOfChains();

  /**
   * @brief Returns the number of the main chains of the model.
   *
   * @return The number of the main chains
   */
  unsigned int getNrOfMainChains();

  /**
   * @brief Returns the name of a chain.
   *
   * @attention In case it is given a non-existent chain for this robot model,
   * this function will return an empty string and it will print a ROS_ERROR.
   *
   * @param chain The desired chain
   * @return The name of the chain
   *
   * @see getChainNames()
   */
  std::string getChainName(unsigned int chain);

  /**
   * @brief Returns the names of every chain of the model.
   *
   * @return The names of the chains
   * @see getChainName(int chain)
   */
  std::vector<std::string> getChainNames();

  /**
   * @brief Returns the total number of the joints (total DOFs) of the robot
   * model.
   *
   * As the total number of the main chains, i.e. all the distinct joints of
   * this model because some models may have multiple chains which share joints
   * between them.
   *
   * @return The total number of the joints of the robot
   *
   * @see getNrOfJoints(unsigned int)
   */
  unsigned int getNrOfJoints();

  /**
   * @brief Returns the number of the joints of a specific chain.
   *
   * @attention In case it is given a non-existent chain for this robot model
   * it will return zero and it will print a ROS_ERROR in the console.
   *
   * @param chain The index of the kinematic chain
   * @return The number of the joints
   *
   * @see getNrOfJoints()
   */
  unsigned int getNrOfJoints(int chain);

  /**
   * @brief Returns the names of total joints of the robot model.
   *
   * Returns the names of the joints of the whole robot as a standard vector of
   * strings. The size of this vector will match the output of
   * Model::getNrOfJoints(). The sequence of the names will follow the sequence
   * of the joints and the sequench of the mutually exclusive chains stored in
   * this Model.
   *
   * @return The names of the joint of the robot.
   *
   * @see getJointName(unsigned int)
   * @see getJointName(unsigned int)
   * @see getJointNames(unsigned int)
   * @see getNrOfJoints()
   */
  std::vector<std::string> getJointNames();

  /**
   * @brief Returns the names of joints of one chain.
   *
   * Returns the names of the joints of the one chain of the robot as a standard vector of
   * strings. The size of this vector will match the output of
   * Model::getNrOfJoints(unsigned int chain).
   *
   * @attention In case it is given a non-existent chain it will print a
   * ROS_ERROR and returns an empty string.
   *
   * @param chain The desired chain
   * @return The name of the joints
   *
   * @see getJointName(unsigned int)
   * @see getJointName(unsigned int)
   * @see getJointNames(unsigned int)
   * @see getNrOfJoints()
   */
  std::vector<std::string> getJointNames(unsigned int chain);

  /**
   * @brief Returns the name of a joint of the whole robot.
   *
   * This means that needs to be given an index from the total number of joints
   * of this model as given by Model::getNrOfJoints().
   *
   * @attention Returns an empty string with a ROS error in case of a non
   * existing joint.
   *
   * @param joint The desired joint
   * @return The name of the joint
   *
   * @see getJointName(unsigned int)
   * @see getJointName(unsigned int)
   * @see getJointNames(unsigned int)
   * @see getNrOfJoints()
   */
  std::string getJointName(unsigned int joint);

  /**
   * @brief Returns the name of a joint existing in a given chain.
   *
   * The following call will return the name of the 2nd joint of the 3rd chain:
   * @code{.cpp} name = model.getJointName(2, 3); @endcode
   *
   * @attention In case it is given a non-existent joint/chain combination for
   * this robot model, this function will return 0 and it will print a
   * ROS_ERROR.
   *
   * @param chain The chain of the desired joint
   * @param joint The desired joint
   * @return The name of the joint
   *
   * @see getJointName(unsigned int)
   * @see getJointNames(unsigned int)
   * @see getJointNames()
   */
  std::string getJointName(unsigned int chain, unsigned int joint);

  /**
   * @brief Returns the limit of a joint existing in a given chain.
   *
   * The following call will return a pair containing the lower and upper limit of the 2nd joint of the 3rd chain:
   * @code{.cpp} limit = model.getJointLimit(2, 3); @endcode
   *
   * @attention In case it is given a non-existent joint/chain combination for
   * this robot model, this function will return a pair with first value -M_PI and second value M_PI
   * and it will print a ROS_ERROR.
   *
   * @param chain The chain of the desired joint
   * @param joint The desired joint
   * @return The limit of the joint
   *
   * @see getJointLimit(unsigned int)
   * @see getJointLimits(unsigned int)
   * @see getJointLimits()
   */
  std::pair<double, double> getJointLimit(unsigned int chain, unsigned int joint);

  /**
   * @brief Returns the limit of a joint existing in the complete list joints.
   *
   * This means that needs to be given an index from the total number of joints
   * of this model as given by Model::getNrOfJoints().
   *
   * @attention Returns a pair of nan values with a ROS error in case of
   * a non existing joint/chain combination.
   * @attention Returns a pair of -pi, pi values with a ROS warning in case of
   * an undefined existing joint/chain combination.
   *
   * @param joint The desired joint
   * @return The limit of the joint
   *
   * @see getJointLimit(unsigned int, unsigned int)
   * @see getJointLimits(unsigned int)
   * @see getJointLimits()
   * @see getNrOfJoints()
   */
  std::pair<double, double> getJointLimit(unsigned int joint);

  /**
   * @brief Returns the limits of joints of one chain.
   *
   * Returns the limits of the joints of the one chain of the robot as a standard vector of
   * pairs (min, max). The size of this vector will match the output of
   * Model::getNrOfJoints(unsigned int chain).
   *
   * @attention In case it is given a non-existent chain it will print a
   * ROS_ERROR and returns a pair of nan values.
   *
   * @param chain The desired chain
   * @return The limits of the joints
   *
   * @see getJointLimit(unsigned int, unsigned int)
   * @see getJointLimit(unsigned int)
   * @see getJointLimits()
   * @see getNrOfJoints(unsigned int)
   */
  std::vector<std::pair<double, double>> getJointLimits(unsigned int chain);

  /**
   * @brief Returns the index of a joint existing in a given main chain. This
   * index is out of the total main joints of the robot
   *
   * @param chain The chain of the desired joint
   * @param joint The desired joint
   * @return The index of the joint
   */
  unsigned int getGlobalIndex(int chain, unsigned int joint);


  /**
   * @brief Returns the limits of total joints of the robot model.
   *
   * Returns the limits of the joints of the whole robot as a standard vector of
   * pairs (min, max). The size of this vector will match the output of
   * Model::getNrOfJoints(). The sequence of the limits will follow the sequence
   * of the joints and the sequence of the mutually exclusive chains stored in
   * this Model.
   *
   * @return The limits of the joints of the robot.
   *
   * @see getJointLimit(unsigned int, unsigned int)
   * @see getJointLimit(unsigned int)
   * @see getJointLimits(unsigned int)
   * @see getNrOfJoints()
   */
  std::vector<std::pair<double, double>> getJointLimits();


  // A map which provides the index of the chain given its name
  std::map<std::string, int> chain_index;

  /**
   * @brief A number of KDL forward kinematic solvers, one for each chain.
   */
  std::vector<KDL::ChainFkSolverPos_recursive> fk_solver;

  /**
   * @brief A number of KDL Jacobian solvers, one for each chain. Calculates
   * the base Jacobian matrix of the chain.
   */
  std::vector<KDL::ChainJntToJacSolver> jac_solver;

  /**
   * @brief A number of KDL Chains storing the kinematics of each chain.
   */
  std::vector<KDL::Chain> chain;

protected:
  /**
   * @brief An array storing the global indeces. Should be filled by every
   * model which derives from this class.
   */
  std::vector<std::vector<unsigned int>> global_index;

  /**
   * @brief The name of this Model
   *
   * The name as a standard string. It should be filled by any class which
   * inherits from Model.
   */
  std::string name;

  /**
   * @brief The number of the mutually exclusive chains
   *
   * Mutually exclusive chains will not share common joints between them. The
   * mutually exclussive chains are a subset of the total chains of the model.
   * For instance a class which inherits from Model and defines a model for the
   * 7DOF KUKA LWR4+ arm can have 1 chain which is exclusive and contains the 7
   * joints and a number of other chains depended on the usage of the class.
   * The rest of the chains can be 7 other chains from the base of the robot to
   * each joint of the robot and can be used for example in order to have the
   * task pose of each joint of the robot. A robot with two arms can have two
   * mutually exclusive chains (the two arms) or a robotic hand can have a
   * number of mutually exclusive chains equal to its fingers.
   *
   * @attention It is of paramount importance to be filled by any class which
   * inherits from Model, because other function depends on the consistency of
   * this variable.
   */
  unsigned int num_chains_exclusive;

  /**
   * @brief The names of the chains of this Model
   *
   * The joint names as a tandard vector of strings. For example the
   * chain_name.at(2) should contain the name of the 2nd chain of the model
   *
   * @attention It is of paramount importance to be filled by any class which
   * inherits from Model, because other function depends on the consistency of
   * this variable. It should contain the name of every chain that the model
   * defines.
   */
  std::vector<std::string> chain_name;

  /**
   * @brief The joint names of this Model
   *
   * The joint names as a 2D standard vector of strings.  For example the
   * joint_name.at(1).at(2) should contain the name of the 2nd joint of the
   * 1st defined chain.
   *
   * @attention It is of paramount importance to be filled by any class which
   * inherits from Model, because other function depends on the consistency of
   * this variable. It should contain the name of every joint of every chain
   * that the model defines.
   */
  std::vector<std::vector<std::string>> joint_name;

  /**
   * @brief The joint limits of this Model
   *
   * The joint limit as a standard vector of double pairs. For example the
   * joint_limit.at(1).at(2).first should contain the lower (min) limit of the 2nd joint of the
   * 1st defined chain whereas joint_limit.at(1).at(2).second should contain the upper (max) limit
   * of the same chain
   *
   * @attention It is of paramount importance to be filled by any class which
   * inherits from Model, because other function depends on the consistency of
   * this variable. It should contain the lower and upper joint limits of every joint of every chain
   * that the model defines.
   */
  std::vector<std::vector<std::pair<double, double>>> joint_limit;

  /**
   * @brief The link names of this Model
   *
   * The link names as a 2D standard vector of strings.  For example the
   * link_name.at(1).at(2) should contain the name of the 2nd link of the
   * 1st defined chain.
   *
   */
  std::vector<std::vector<std::string>> link_name;
};
}  // namespace robot
}  // namespace arl
#endif  // AUTHARL_CORE_ROBOT_MODEL_H
