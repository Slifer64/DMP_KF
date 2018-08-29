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

#ifndef AUTHARL_CORE_ROBOT_CONTROLLER_H
#define AUTHARL_CORE_ROBOT_CONTROLLER_H

#include <autharl_core/robot/robot.h>

#include <string>
#include <vector>

namespace arl
{
namespace robot
{
/**
 * @brief An abstract class which implements a robot controller.
 *
 * This class defines an interface for implementing a robot controller. You can
 * create your own robot controller by inherit this class and implement the
 * different functionalities like:
 *   - How the controller is initializing. For example, the controller before
 *   it starts it may need to send the robot to an initial configuration
 *   - What measurements your controller needs. For example, reading the joint
 *   positions, or the joint torques of the robot.
 *   - The most important part: the calculations that your controller performs
 *   in order to produce commands from the measurements.
 *   - What commands your controller send to the robot. For example, sending
 *   calculates and sends task velocities or joint torques.
 *   - What is the stopping policy of your controller. For example, it stops
 *   when your robot stops moving or a high dangerous external force is detected.
 *   - When your controller is successful.
 */
class Controller
{
public:
  /**
   * @brief An empty constructor.
   */
  Controller();

  /**
   * @brief Constructor which initializes the robot controller.
   *
   * @param r A pointer to the robot to be controlled.
   * @param name The name of your controller. Defaults to "Unnamed".
   */
  explicit Controller(std::shared_ptr<Robot> r,
                       const std::string& name = "Unnamed");

  /**
   * @brief The main function that runs the controller.
   *
   * This function is the main thread of the controller. What is basically do
   * is that first initializes the controller by calling the Controller::init()
   * and then runs the control loop which stops if the Controller::stop() returns
   * true. In each loop the function measures by calling Controller::measure(),
   * perform calculations by calling Controller::update(), commands the robot
   * by calling Controller::command() and finally waits for the next cycle of
   * the given robot by calling Robot::waitNextCycle(). After the loop ends it
   * will return the result of the Controller::success().
   *
   * @return True if the controller is successful. False otherwise.
   */
  virtual bool run();

  /**
   * @brief Sets an external stop to the controller. Another external thread
   * can use this function to stop the controller.
   *
   * @param arg Set True to stop the controller.
   */
  void setExternalStop(bool arg);

  /**
   * @brief Returns the name of the controller
   *
   * @return The name of the controller
   */
  std::string getName();

protected:
  /**
   * @brief Initializes the controller.
   *
   * Implemented according to your controller needs. Common use cases involve
   * sending a the robot in an initial configuration, setting the robot mode
   * (to position or torque control) or setting the paramters and gains of your
   * controller.
   */
  virtual void init();

  /**
   * @brief Implements the stopping condition of your controller.
   *
   * Implemented according to your controller needs. Common use cases involve
   * stopping conditions like your robot has zero velocity, the robot hardware
   * signals an error or dangerous movements indicates the immediate stop.
   *
   * @return True if the controller should stop. False otherwise.
   */
  virtual bool stop();

  /**
   * @brief Implements the success condition of your controller.
   *
   * Implemented according to your controller needs. Did your controller
   * deliver the task appropriantly?
   *
   * @return True if the controller succeded. False otherwise.
   */
  virtual bool success();

  /**
   * @brief Measures the desired data.
   *
   * Implemented according to your controller needs. Basically makes calls to
   * functions of the Robot class and store them to internal variables, like the
   * joint positions of the robot, the task pose of a robotic arm etc.
   */
  virtual void measure();

  /**
   * @brief Commands the robot.
   *
   * Implemented according to your controller needs. Basically makes calls to
   * functions of the Robot class, like sending new desired joint torques or
   * joint positions. These desired signals can be stored in internal variables
   * and should have been calculated based on the measurements of the
   * Controller::measure() and the calculations of the Controller::update()
   */
  virtual void command();

  /**
   * @brief Performs the necessary calculations of the controller.
   *
   * Implemented according to your controller needs. The calculations can use
   * the measurements from the Controller::measure() which is stored in
   * internal variables and store the results for the Controller::command() to
   * use.
   */
  virtual void update() = 0;

  /**
   * @brief The name of the controller
   */
  std::string name;

  /**
   * @brief The external stopped setted by Controller::setExternalStop()
   */
  bool external_stop;

protected:
  /**
   * @brief The pointer to the robot to be controlled.
   */
  std::shared_ptr<Robot> robot;
};
}  // namespace robot
}  // namespace arl

#endif  // AUTHARL_CORE_ROBOT_CONTROLLER_H
