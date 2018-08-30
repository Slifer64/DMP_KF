#ifndef VIRTUAL_FT_SENSOR_H
#define VIRTUAL_FT_SENSOR_H

#include <iostream>
#include <cstdlib>
#include <vector>
#include <memory>
#include <cmath>
#include <cstring>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <random>

#include <ros/package.h>
#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>

#include <mainwindow.h>
#include <QApplication>

namespace as64_
{

class VFTSensor
{
private:

  std::shared_ptr<MainWindow> gui;
  std::shared_ptr<QApplication> q_app;

  /// Trackbars max_values and offsets
  std::vector<double> F_max; ///< maximum trackbar value
  std::vector<bool> f_reset;
  std::vector<double> F; ///< actual F/T value
  std::vector<double> F_filt; ///< filtered F/T value
  std::vector<double> noise_ampl;
  double scale;

  double freq; ///< publish frequency
  std::string pub_topic;
  ros::NodeHandle node;
  ros::Publisher wrench_pub; ///< wrench publisher
  std::shared_ptr<ros::Rate> pub_rate;
  geometry_msgs::WrenchStamped wrench_msg;

  double a_filt;
  void parseConfigParams();
  double getCalibForce(int i);
  int getRawForce(int i);

  int runGui();
  double getForce(int i) const;
  double setForce(double f_val, int i);

  std::thread run_thread;
  bool is_running;

public:

  /**
   * Constructor.
   */
  VFTSensor();

  /**
   * Destructor.
   */
  ~VFTSensor();

  /**
   * Initialization.
   */
  void init();

  /**
   * Start the FT sensor.
   */
  void run();

  bool isRunning() const;

};

} // namespace as64_

#endif // VIRTUAL_FT_SENSOR_H
