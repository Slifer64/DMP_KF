/**
 * Copyright (C) 2016 AUTH-ARL
 */

#ifndef LWR_ROBOT_LWR_ROBOT_H
#define LWR_ROBOT_LWR_ROBOT_H

#include <FastResearchInterface.h>
#include <autharl_core/robot/robot.h>
#include <autharl_core/robot/trajectories.h>
#include <lwr_robot/lwr_model.h>
#include <ros/ros.h>

#include <memory>
#include <vector>
#include <string>


namespace lwr
{
namespace robot
{
class Robot : public arl::robot::Robot
{
public:
  Robot();
  explicit Robot(std::shared_ptr<arl::robot::Model> m, const std::string& name = "KUKA LWR4+");
  void stop();
  void setMode(arl::robot::Mode mode, int chain_index = 0);
  void waitNextCycle();


  void setJointTrajectory(const KDL::JntArray &input, double duration, const int chain_index = 0);
  void setJointTrajectory(const arma::vec &input, double duration, const int chain_index = 0);
  void setJointTrajectory(const Eigen::VectorXd &input, double duration, const int chain_index = 0);
  template <typename T>
  void setJointTrajectoryTemplate(const T &input, double duration, const int chain_index = 0)
  {
    // setJntPosTrajTemplate(input, duration, chain_index);
    // inital joint position values
    arma::vec q0 = arma::zeros<arma::vec>(7);
    arma::vec temp = arma::zeros<arma::vec>(7);
    for (int i = 0; i < 7; i++) {
      temp(i) = input(i);
    }
    getJointPosition(q0);
    // keep last known robot mode
    arl::robot::Mode prev_mode = mode;
    arma::vec qref = q0;
    // start conttroller
    setMode(arl::robot::Mode::POSITION_CONTROL);
    // initalize time
    double t = 0.0;
    // the main while
    while (t < duration)
    {
      // waits for the next tick also
      FRI->WaitForKRCTick();
      // compute time now
      t += cycle;
      // update trajectory
      qref = (arl::robot::trajectory::get5thOrder(t, q0, temp, duration)).col(0);
      // set joint positions
      setJointPosition(qref);
    }
    // reset last known robot mode
    setMode(prev_mode);
  }

  void setJointPosition(const KDL::JntArray &input, const int chain_index = 0);
  void setJointPosition(const arma::vec &input, const int chain_index = 0);
  void setJointPosition(const Eigen::VectorXd &input, const int chain_index = 0);
  template <typename T> void setJointPositionTemplate(const T &input, const int chain_index = 0) {
    if (this->mode == arl::robot::Mode::POSITION_CONTROL)
    {
      if (FRI->GetFRIMode() == FRIStateSet::FRI_STATE_MON)
      {
        printf("[JointPosController::setJointPosition] Joint positions are commanded with closed controller.\n");
        printf("Opening controller ...\n");
        // start te cotroller
        startJointPositionController();
        // wait one tick
        FRI->WaitForKRCTick();
      }
      // temp variables
      float temp[7];
      // put the values from arma to float[]
      for (int i = 0; i < 7; i++) {
        temp[i] = input(i);
      }
      // set commanded joint positions to qd
      FRI->SetCommandedJointPositions(temp);
      saveLastJointPosition(temp);
    }
    else
    {
      std::cerr << "setJointPosition only available in POSITION_CONTROL mode" << std::endl;
    }
  }

  void setJointVelocity(const KDL::JntArray &input, const int chain_index = 0);
  void setJointVelocity(const arma::vec &input, const int chain_index = 0);
  void setJointVelocity(const Eigen::VectorXd &input, const int chain_index = 0);
  template <typename T> void setJointVelocityTemplate(const T &input, const int chain_index = 0)
  {
    if (this->mode == arl::robot::Mode::VELOCITY_CONTROL)
    {
      static float temp[7];
      for (size_t i = 0; i < 7; i++)
      {
        last_jnt_pos[i] += input(i) * cycle;
      }
      FRI->SetCommandedJointPositions(last_jnt_pos);
    }
    else
    {
      std::cerr << "setJointVelocity only available in VELOCITY_CONTROL mode" << std::endl;
    }
  }

  void setJointTorque(const KDL::JntArray &input, const int chain_index = 0);
  void setJointTorque(const arma::vec &input, const int chain_index = 0);
  void setJointTorque(const Eigen::VectorXd &input, const int chain_index = 0);
  template <typename T> void setJointTorqueTemplate(const T &input, const int chain_index = 0)
  {
    if (this->mode == arl::robot::Mode::TORQUE_CONTROL)
    {
      static float torques[7];
      for (size_t i = 0; i < 7; i++)
      {
        torques[i] = input(i);
      }
      FRI->SetCommandedJointTorques(torques);
      // Mirror the joint positions and the cartesian pose in order to avoid
      // cartesian deviation errors
      static float temp_position[7];
      FRI->GetMeasuredJointPositions(temp_position);
      FRI->SetCommandedJointPositions(temp_position);
      static float temp_pose[12];
      FRI->GetMeasuredCartPose(temp_pose);
      FRI->SetCommandedCartPose(temp_pose);

      saveLastJointPosition(temp_position);
    }
    else
    {
      std::cerr << "setJointTorque only available in TORQUE_CONTROL mode" << std::endl;
    }
  }

  void getJointPosition(KDL::JntArray &output, const int chain_index = 0);
  void getJointPosition(arma::vec &output, const int chain_index = 0);
  void getJointPosition(Eigen::VectorXd &output, const int chain_index = 0);
  template <typename T> void getJointPositionTemplate(T &output, const int chain_index = 0)
  {
    output.resize(model->getNrOfJoints(chain_index));
    static float temp[7];
    FRI->GetMeasuredJointPositions(temp);
    for (size_t i = 0; i < 7; i++) {
      output(i) = temp[i];
    }
  }

  void getJointVelocity(KDL::JntArray &output, const int chain_index = 0);
  void getJointVelocity(arma::vec &output, const int chain_index = 0);
  void getJointVelocity(Eigen::VectorXd &output, const int chain_index = 0);
  template <typename T> void getJointVelocityTemplate(T &output, const int chain_index = 0)
  {
    output.resize(model->getNrOfJoints(chain_index));
    static float temp[7];
    FRI->GetMeasuredJointPositions(temp);
    for (size_t i = 0; i < 7; i++) {
      output(i) = (temp[i] - last_jnt_pos[i]) / cycle;
    }
  }

  void getJointTorque(KDL::JntArray &output, const int chain_index = 0);
  void getJointTorque(arma::vec &output, const int chain_index = 0);
  void getJointTorque(Eigen::VectorXd &output, const int chain_index = 0);
  template <typename T> void getJointTorqueTemplate(T &output, const int chain_index = 0)
  {
    output.resize(model->getNrOfJoints(chain_index));
    static float joint_torques[7];
    FRI->GetMeasuredJointTorques(joint_torques);
    output(0) = joint_torques[0];
    output(1) = joint_torques[1];
    output(2) = joint_torques[2];
    output(3) = joint_torques[3];
    output(4) = joint_torques[4];
    output(5) = joint_torques[5];
    output(6) = joint_torques[6];
  }

  void getJointExternalTorque(KDL::JntArray &output, const int chain_index = 0);
  void getJointExternalTorque(arma::vec &output, const int chain_index = 0);
  void getJointExternalTorque(Eigen::VectorXd &output, const int chain_index = 0);
  template <typename T> void getJointExternalTorqueTemplate(T &output, const int chain_index = 0)
  {
    output.resize(model->getNrOfJoints(chain_index));
    static float estimated_external_joint_torques[7];
    FRI->GetEstimatedExternalJointTorques(estimated_external_joint_torques);
    output(0) = estimated_external_joint_torques[0];
    output(1) = estimated_external_joint_torques[1];
    output(2) = estimated_external_joint_torques[2];
    output(3) = estimated_external_joint_torques[3];
    output(4) = estimated_external_joint_torques[4];
    output(5) = estimated_external_joint_torques[5];
    output(6) = estimated_external_joint_torques[6];
  }

  void getJacobian(KDL::Jacobian &output, const int chain_index = 0);
  void getJacobian(arma::mat &output, const int chain_index = 0);
  void getJacobian(Eigen::MatrixXd &output, const int chain_index = 0);
  template <typename T> void getJacobianTemplate(T &output, const int chain_index = 0)
  {
    float **temp;
    temp = reinterpret_cast<float**>(malloc(sizeof(float *) * 6));
    for (size_t i = 0; i < 6; i++) {
      temp[i] = reinterpret_cast<float*>(malloc(sizeof(float) * 7));
    }
    FRI->GetCurrentJacobianMatrix(temp);
    std::vector<int> jac_indexes{0, 1, 2, 5, 4, 3};
    for (size_t i = 0; i < jac_indexes.size(); i++)
    {
      for (size_t j = 0; j < 7; j++)
      {
        output(i, j) = temp[jac_indexes[i]][j];
      }
    }
  }

  void getTaskPose(KDL::Frame &output, const int chain_index = 0);
  void getTaskPose(arma::mat &output, const int chain_index = 0);
  void getTaskPose(Eigen::MatrixXd &output, const int chain_index = 0);
  template <typename T> void getTaskPoseTemplate(T &output, const int chain_index = 0)
  {
    output.resize(3, 4);
    static float temp[12];
    FRI->GetMeasuredCartPose(temp);
    for (size_t i = 0; i < 3; i++)
    {
      for (size_t j = 0; j < 4; j++)
      {
        output(i, j) = temp[i * 4 + j];
      }
    }
  }

  void getTaskPosition(KDL::Vector &output, const int chain_index = 0);
  void getTaskPosition(arma::vec &output, const int chain_index = 0);
  void getTaskPosition(Eigen::Vector3d &output, const int chain_index = 0);
  template <typename T> void getTaskPositionTemplate(T &output, const int chain_index = 0)
  {
    static float temp[12];
    FRI->GetMeasuredCartPose(temp);
    output(0) = temp[3];
    output(1) = temp[7];
    output(2) = temp[11];
  }

  void getTaskOrientation(KDL::Rotation &output, const int chain_index = 0);
  void getTaskOrientation(arma::mat &output, const int chain_index = 0);
  void getTaskOrientation(Eigen::Matrix3d &output, const int chain_index = 0);
  template <typename T> void getTaskOrientationTemplate(T &output, const int chain_index = 0)
  {
    static float temp[12];
    FRI->GetMeasuredCartPose(temp);
    output(0, 0) = temp[0];
    output(0, 1) = temp[1];
    output(0, 2) = temp[2];
    output(1, 0) = temp[4];
    output(1, 1) = temp[5];
    output(1, 2) = temp[6];
    output(2, 0) = temp[8];
    output(2, 1) = temp[9];
    output(2, 2) = temp[10];
  }

  void getTwist(KDL::Twist &output, const int chain_index = 0);
  void getTwist(arma::vec &output, const int chain_index = 0);
  void getTwist(Eigen::VectorXd &output, const int chain_index = 0);
  template <typename T> void getTwistTemplate(T &output, const int chain_index = 0)
  {
    KDL::Jacobian jac(7);
    KDL::JntArray vel(7);
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

  void getExternalWrench(KDL::Wrench &output, const int chain_index = 0);
  void getExternalWrench(arma::vec &output, const int chain_index = 0);
  void getExternalWrench(Eigen::VectorXd &output, const int chain_index = 0);
  template <typename T> void getExternalWrenchTemplate(T &output, const int chain_index = 0)
  {
    static float estimated_external_cart_forces_and_torques[6];
    FRI->GetEstimatedExternalCartForcesAndTorques(estimated_external_cart_forces_and_torques);
    output(0) = estimated_external_cart_forces_and_torques[0];
    output(1) = estimated_external_cart_forces_and_torques[1];
    output(2) = estimated_external_cart_forces_and_torques[2];
    output(3) = estimated_external_cart_forces_and_torques[3];
    output(4) = estimated_external_cart_forces_and_torques[4];
    output(5) = estimated_external_cart_forces_and_torques[5];
  }

  void setWrench(const KDL::Wrench &input, const int chain_index = 0);
  void setWrench(const arma::vec &input, const int chain_index = 0);
  void setWrench(const Eigen::VectorXd &input, const int chain_index = 0);
  template <typename T> void setWrenchTemplate(T &input, const int chain_index = 0)
  {
    if (this->mode == arl::robot::Mode::IMPEDANCE_CONTROL)
    {
      if (FRI->GetFRIMode() == FRIStateSet::FRI_STATE_MON)
      {
        printf("[CartImpedanceController::setWrench] Cartesian wrench is commanded with closed controller.\n");
        printf("Opening controller ...\n");
        // start te controller
        startCartImpController();
        // wait one tick
        FRI->WaitForKRCTick();
      }
      // temp variables
      float temp[6];
      // put the values from arma to float[]
      for (int i = 0; i < 6; i++) {
        temp[i] = input(i);
      }

      // set commanded Cartesian forces/torques
      FRI->SetCommandedCartForcesAndTorques(temp);
      
      static float temp_position[7];
      FRI->GetMeasuredJointPositions(temp_position);
      // FRI->SetCommandedJointPositions(temp_position);
      // static float temp_pose[12];
      // FRI->GetMeasuredCartPose(temp_pose);
      // FRI->SetCommandedCartPose(temp_pose);
      saveLastJointPosition(temp_position); //needed for numeric differentation to obtain q_dot [isn't it?]
      
    }
    else
    {
      std::cerr << "setWrench only available in IMPEDANCE_CONTROL mode" << std::endl;
    }
  }

  void setTaskPose(const KDL::Frame &input, const int chain_index = 0);
  void setTaskPose(const arma::mat &input, const int chain_index = 0);
  void setTaskPose(const Eigen::MatrixXd &input, const int chain_index = 0);
  template <typename T> void setTaskPoseTemplate(T &input, const int chain_index = 0)
  {
    if (this->mode == arl::robot::Mode::IMPEDANCE_CONTROL)
    {
      if (FRI->GetFRIMode() == FRIStateSet::FRI_STATE_MON)
      {
        printf("[CartImpedanceController::setTaskPose] Cartesian wrench is commanded with closed controller.\n");
        printf("Opening controller ...\n");
        // start te controller
        startCartImpController();
        // wait one tick
        FRI->WaitForKRCTick();
      }
      // temp variables
      float temp[12];
      for (size_t i = 0; i < 3; i++)
        {
          for (size_t j = 0; j < 4; j++)
          {
            temp[i * 4 + j] = input(i, j);
          }
      }
      
      // set commanded task pose
      FRI->SetCommandedCartPose(temp);
    }
    else
    {
      std::cerr << "setTaskPose only available in IMPEDANCE_CONTROL mode" << std::endl;
    }
  }

  void setCartStiffness(const KDL::Wrench &input, const int chain_index = 0);
  void setCartStiffness(const arma::vec &input, const int chain_index = 0);
  void setCartStiffness(const Eigen::VectorXd &input, const int chain_index = 0);
  template <typename T> void setCartStiffnessTemplate(T &input, const int chain_index = 0)
  {
    if (this->mode == arl::robot::Mode::IMPEDANCE_CONTROL)
    {
      if (FRI->GetFRIMode() == FRIStateSet::FRI_STATE_MON)
      {
        printf("[CartImpedanceController::setCartStiffness] Cartesian wrench is commanded with closed controller.\n");
        printf("Opening controller ...\n");
        // start te controller
        startCartImpController();
        // wait one tick
        FRI->WaitForKRCTick();
      }
      // temp variables
      float temp[6];
      // put the values from arma to float[]
      for (int i = 0; i < 6; i++) {
        temp[i] = input(i);
      }
      
      // set value
      FRI->SetCommandedCartStiffness(temp);
    }
    else
    {
      std::cerr << "setCartStiffness only available in IMPEDANCE_CONTROL mode" << std::endl;
    }
  }

  void setCartDamping(const KDL::Wrench &input, const int chain_index = 0);
  void setCartDamping(const arma::vec &input, const int chain_index = 0);
  void setCartDamping(const Eigen::VectorXd &input, const int chain_index = 0);
  template <typename T> void setCartDampingTemplate(T &input, const int chain_index = 0)
  {
    if (this->mode == arl::robot::Mode::IMPEDANCE_CONTROL)
    {
      if (FRI->GetFRIMode() == FRIStateSet::FRI_STATE_MON)
      {
        printf("[CartImpedanceController::setCartDamping] Cartesian wrench is commanded with closed controller.\n");
        printf("Opening controller ...\n");
        // start te controller
        startCartImpController();
        // wait one tick
        FRI->WaitForKRCTick();
      }
      // temp variables
      float temp[6];
      // put the values from arma to float[]
      for (int i = 0; i < 6; i++) {
        temp[i] = input(i);
      }
      
      // set value
      FRI->SetCommandedCartDamping(temp);
    }
    else
    {
      std::cerr << "setCartDamping only available in IMPEDANCE_CONTROL mode" << std::endl;
    }
  }



  bool isOk();

private:
  std::shared_ptr<FastResearchInterface> FRI;
  void startJointPositionController();
  void startJointTorqueController();
  void startCartImpController();
  void stopController();
  void startLogging();
  void stopLogging();
  void saveLastJointPosition(float input[7]);
  void saveLastJointPosition();
  float last_jnt_pos[7];
};
}  // namespace robot
}  // namespace lwr

#endif  // LWR_ROBOT_LWR_ROBOT_H
