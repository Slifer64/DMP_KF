/**
 * Copyright (C) 2017 as64_
 */

#ifndef OL_2D_RUP_H
#define OL_2D_RUP_H

#define _USE_MATH_DEFINES

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>

#include <memory>
#include <random>
#include <iomanip>
#include <chrono>
#include <cmath>
#include <algorithm>
#include <cctype>
#include <string>
#include <thread>
#include <csignal>
#include <mutex>

#include <armadillo>
#include <Eigen/Dense>

#include <dmp_kf/utils.h>

#include <dmp_kf/GUI/GUI.h>
#include <dmp_kf/LogData.h>
#include <dmp_kf/Robot/Robot.h>
#include <dmp_kf/Controller/Controller.h>

//using namespace as64_;

class dmp_kf
{
public:
  dmp_kf();
  ~dmp_kf();

  void init();
  void execute();
  void finalize();

private:

  ros::NodeHandle n;

  std::shared_ptr<GUI> gui;
  std::shared_ptr<LogData> log_data;
  std::shared_ptr<Robot> robot;
  std::shared_ptr<Controller> controller;

  arma::vec q_start; ///< starting pose

  bool save_logData_finished;
  std::shared_ptr<std::thread> save_logData_thread;
  void saveLogDataThreadFun();
  void saveLogData();
  void clearLoggedData();

  void saveCurrentPoseAsStartPose();
  void gotoStartPose();
};

#endif // OL_2D_RUP_H
