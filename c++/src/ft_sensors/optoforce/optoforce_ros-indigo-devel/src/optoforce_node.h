/**
 * @file   optoforce_node.h
 * @author Asier Fernandez <asier.fernandez@tecnalia.com>
 * @date   2016
 *
 * Copyright 2016 Tecnalia Research & Innovation.
 * Distributed under the GNU GPL v3.
 * For full terms see https://www.gnu.org/licenses/gpl.txt
 *
 * @brief Basic ROS node. Add an interface to OptoForce driver
 *          Initialize ROS node
 *          Read parameters from ROS parameter Server
 *          Initialize OptoForce devices
 */

#include <optoforce/optoforce_acquisition.hpp>

#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>

class optoforce_node {
  public:
    //! Constructor
    optoforce_node();

    //! Destructor
    ~optoforce_node();

    //! init
    int init();

    //! configure node
    //! Read Parameters from parameter server
    int configure();

    //! Flag that enables publishing
    bool publish_enable_;

    //! Flag that enables storing data
    bool storeData_enable_;
    bool storeData_cmd_;

    //! This function must be overwriten by the derived class
    //! Start transmision of data independently of the interface
    virtual int run(){};

    //! This function must be overwriten by the derived class
    //! Start transmision of data independently of the interface
    void add_ros_interface(){};

    //! This function must be overwriten by the derived class
    //! Start transmision of data independently of the interface
    virtual void transmitStart() {};

    //! This function must be overwriten by the derived class
    //! Stop transmision of data independently of the interface
    virtual void transmitStop() {};

    void storeStart();
    void storeStop();

  protected:

    OptoforceAcquisition * force_acquisition_;

    //! ROS node handler
    ros::NodeHandle nh_;

    //! Number of devices connected
    int connectedDAQs_;

    //! Publish frequency
    int loop_rate_;

    //! Struct containing each sensor related parameters
    struct SensorConfig {
      std::string name;
      int speed;
      int filter;
      std::vector<float> calib;
      std::vector<int> F_trans;
      std::vector<int> T_trans;
    };

    //! List of all connected devices parameters
    std::vector<SensorConfig> device_list_;

  private:

    //! finish node
    void finish();

    //! Frequency in which the program will read sensor data
    int acquisition_rate_;

    //! Sensor's transmission frequency
    int transmission_speed_;

    //! Sensor Filter
    int filter_;

    //! File where data will be stored
    std::string filename_;

    //! Samples to be stored
    int num_samples_;

    std::vector<std::string> ldevice_;
    std::vector<std::vector<float> > lcalib_;
    std::vector<int> lspeed_;
    std::vector<int> lfilter_;

};
