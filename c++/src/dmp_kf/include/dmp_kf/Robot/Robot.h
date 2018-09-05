#ifndef ROBOT_OL_2D_RUP_H
#define ROBOT_OL_2D_RUP_H

#include <cstdlib>
#include <exception>
#include <vector>
#include <cstring>
#include <dmp_kf/utils.h>
#include <armadillo>
#include <Eigen/Dense>

/** ====  Generic robot class  ====
 *  A generic robot class that handles the communication with the actual robot.
 *  Can be used to set the robot's control mode, send commands and read robot's
 *  joint positions, task pose, velocities, forces etc.
 *
 *  Example usage for velocity control:
 *  Robot robot; // create Robot object
 *  robot.init(); // initialize the robot object
 *  robot.setMode(Robot::Mode::VELOCITY_CONTROL); // set the control mode to velocity.
 *  while (true)
 *  {
 *     robot.update(); // read the current robot's state from the actual hardware.
 *
 *    // Do stuff like reading position, forces etc.
 *    // Calculate a control input.
 *
 *    robot.setTaskVelocity(vel); // set the task velocity
 *
 *    robot.command(); // send the command to the actual hardware.
 *  }
 */
class Robot
{
public:
  /** The control modes that can be applied to the robot. */
  enum Mode
  {
    VELOCITY_CONTROL, // velocity control
    FREEDRIVE_MODE, // freedrive mode
    IDLE_MODE, // robot is idle and doesn't move
  };

  /** Limits applied for safety. */
  enum SafetyStatus
  {
    OK, // no limit exceeded
    VELOCITY_LIMIT, // velocity limit exceeded
    WRENCH_LIMIT, // force limit exceeded
  };

  /** Constructor.
   *  Can be empty or can be used for initializing the robot as 'init'(\see init).
   */
  Robot();

  /** Destructor.
   *  Can be used to shut down the robot and deallocate any memory (if it was allocated
   *   during initialization).
   */
  ~Robot();

  /**
   *  @return The current control mode of the robot.
   */
  Robot::Mode getMode() const;

  /**
   *  @return The name of the current control mode of the robot.
   */
  std::string getModeName() const;

  // ===========================================
  // ========   Virtual functions  =============
  // ===========================================

  /** Initializes the robot.
   */
  virtual void init() = 0;

  /**
   *  @return The control cycle of the robot.
   */
  virtual double getControlCycle() const = 0;

  /**
   *  @return True if the actual robot is ok. False if some error occured.
   */
  virtual bool isOk();

  /** In case an error occured on the robot (i.e. isOk()==false) returns the error message.
   *  @return A message describing the error that occured on the robot.
   */
  virtual std::string getErrMsg() const;

  /** Does all necessary actions after a robot error to renable the robot
   *  and resets all safety/error flags.
   */
  virtual void enable();

  /** Updates the robot state (position, forces, velocities etc.) by reading them
   *  from the actual hardware. Must be called once in each control cycle.
   */
  virtual void update() = 0;

  /** Sends a command to the robot according to the current control mode.
   *  Must be called once in each control cycle.
   */
  virtual void command() = 0;

  /** Sets the robot's mode.
   *  @param[in] mode The control mode to apply to the robot.
   */
  virtual void setMode(const Robot::Mode &mode) = 0;

  /** Sets the robot's task velocity.
   *  @param[in] vel 6x1 vector expressing the task velocity in the base frame.
   */
  virtual void setTaskVelocity(const arma::vec &vel) = 0;

  /** Moves the robot to the specified joint positions within the specified duration.
   *  @param[in] qT Desired joint position.
   *  @param[in] duration Desired movement duration.
   */
  virtual void setJointTrajectory(const arma::vec &qT, double duration) = 0;

  /**
   *  @return The current joint positions.
   */
  virtual arma::vec getJointPosition() const = 0;

  /**
   *  @return The current task pose as a 4x4 homogenous transform.
   */
  virtual arma::mat getTaskPose() const = 0;

  /**
   *  @return The task postion as a 3x1 vector.
   */
  virtual arma::vec getTaskPosition() const = 0;

  /**
   *  @return The task orientation as 4x1 unit quaternion.
   */
  virtual arma::vec getTaskOrientation() const = 0;

  /**
   *  @return The task wrench as a 6x1 vector (expressed in the base frame).
   */
  virtual arma::vec getTaskWrench() = 0;

protected:

  Eigen::Vector4d rotm2quat(Eigen::Matrix3d rotm) const;
  arma::vec rotm2quat(const arma::mat &rotm) const ;

  Mode mode; ///< robot's control mode
  std::vector<std::string> modeName; ///< robot's control mode name
  arma::vec vel_cmd; ///< commanded task velocity expressed in base frame.

  arma::vec Fext_dead_zone; ///< dead zone applied to external force measurement
  arma::vec vel_limit; ///< maximum velocity limits
  arma::vec wrench_limit; ///<  maximum wrench limits

  SafetyStatus safe_status; ///< safety status
  bool safety_check_on;

  std::string err_msg; ///< contains the description of an occured error.

  /** Checks the robot's safety status. */
  void safetyCheck();
  void setSafetyStatus(const Robot::SafetyStatus &status);
  Robot::SafetyStatus getSafetyStatus();


  void readParams(const char *params_file = NULL);

};

#endif // ROBOT_OL_2D_RUP_H
