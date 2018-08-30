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

#include "optoforce_node.h"

optoforce_node::optoforce_node(): nh_("~"),
                                  publish_enable_(false),
                                  storeData_enable_(false),
                                  storeData_cmd_(false),
                                  force_acquisition_(NULL),
                                  connectedDAQs_(0),
                                  loop_rate_(100),
                                  acquisition_rate_(1000),
                                  transmission_speed_(100),
                                  filter_(15),
                                  num_samples_(600000)
{
}

optoforce_node::~optoforce_node()
{
  finish();
}
void optoforce_node::finish()
{
  std::cout << "FINISH program" << std::endl;
  if (force_acquisition_->isRecording())
  {
    force_acquisition_->stopRecording();
    force_acquisition_->storeData();
  }

  if (force_acquisition_ != NULL)
  {
    delete force_acquisition_;
    force_acquisition_ = NULL;
  }
}

int optoforce_node::init()
{

  std::cout << "Looking for " << connectedDAQs_ << " connected DAQ " << std::endl;
  force_acquisition_ = new OptoforceAcquisition();

  // Read configuration file
  configure();

  force_acquisition_->setDesiredNumberSamples(num_samples_);

  if (!force_acquisition_->initDevices(connectedDAQs_))
  {
    std::cerr << "Something went wrong during initialization. Bye!" << std::endl;
    return -1;
  }

  for (int i = 0; i < connectedDAQs_; ++i)
  {
    std::cout << "ldevice[" << i << "] " << ldevice_[i] << std::endl;

    if (!force_acquisition_->isDeviceConnected(ldevice_[i]))
    {
      std::cerr << "[optoforce_initi]Could not find device " << ldevice_[i] << std::endl;
      std::cerr << "Quitting the application." << std::endl;
      finish();
    }
  }
  // Configure Force Sensors Speed and Filter
  for (int i = 0; i < connectedDAQs_; ++i)
  {
    if (!force_acquisition_->setSensorSpeed(lspeed_[i]))
    {
      std::cerr << "Could not setSensorSpeed" << std::endl;
    }
    else
      std::cout << "Correctly setFrequency:  " << lspeed_[i] << std::endl;

    if (!force_acquisition_->setSensorFilter(lfilter_[i]))
    {
      std::cerr << "Could not setSensorFilter" << std::endl;
    }
    else
      std::cout << "Correctly setSensorFilter:  " << lfilter_[i] << std::endl;
  }


  // Configure Force Sensors Filter Frequency in all connected devices
  // if (!force_acquisition_->setSensorFilter(filter_))
  //   std::cerr << "Could not setSensorFilter" << std::endl;
  // else
  //   std::cout << "Correctly setSensorFilter:  " << filter_ << std::endl;

  // Configure FT Calibration
  for (int i = 0; i < connectedDAQs_; ++i)
  {
    std::cout << "[optoforce_node::init]" << std::endl;
    if (!force_acquisition_->setDeviceCalibration(ldevice_[i], lcalib_[i]))
    {
      std::cerr << "Could not setDeviceCalibration" << std::endl;
    }
    else
    {
      std::cout << "Correctly setDeviceCalibration: [ ";

      for (int j = 0; j < lcalib_[i].size(); j++)
      {
        std::cout << lcalib_[i][j] << " ";
      }
      std::cout << "]" << std::endl;

    }
  }

  // Set to Zero All OptoForce devices
  force_acquisition_->setZeroAll();

  // Set Acquisition Frequency
  // This frequency determines how often we get a new data.
  // Independently from Sensor Transmission Rate
  force_acquisition_->setAcquisitionFrequency(acquisition_rate_);

  // Set Filename to Store Data
  force_acquisition_->setFilename(filename_);

  // Initialize ROS publisher
  std::vector<std::string> serial_numbers;
  force_acquisition_->getSerialNumbers(serial_numbers);

  // By default Do Store publish
  storeData_enable_ = false;
  storeData_cmd_ = false;

  if (publish_enable_)
  {
    if (!force_acquisition_->isReading())
    {
      force_acquisition_->startReading();
    }
  }

  return 0;
}

// Read Parameters from parameter server
int optoforce_node::configure()
{
  ROS_INFO("READING CONFIGURATION FILE");

  nh_.param("loop_rate", loop_rate_, 100); // Loop Rate in Hz. Default 100

  nh_.param("acquisition_freq", acquisition_rate_, 1000); // Loop Rate in Hz. Default 100

  nh_.param("num_samples", num_samples_, 600000); // Maximun Number of Samples to be stored. Default 600000

  nh_.param<std::string>("filename", filename_, "/tmp/optoforce_node"); // Loop Rate in Hz

  nh_.param("publish", publish_enable_, true); // Automatically publish data

  XmlRpc::XmlRpcValue devices_list;
  if (!nh_.getParam("devices", devices_list))
  {
    ROS_ERROR_STREAM("Parameter devices undefined");
    return -1;
  }
  if (devices_list.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_WARN("Parameter 'devices_list' is not an array");
    return -1;
  }

  // Initialize variable to store all devices information
  device_list_.clear();
  device_list_.resize(devices_list.size());

  connectedDAQs_ = devices_list.size();

  // List of devices names
  ldevice_.clear();

  // List of calibration matrix
  lcalib_.clear();

  // Loop over each device to extract the name
  for (int i = 0; i < connectedDAQs_; ++i)
  {
      XmlRpc::XmlRpcValue& device_data = devices_list[i];

      if(device_data.getType() == XmlRpc::XmlRpcValue::TypeStruct)
      {
        // Get Name
        if (device_data.hasMember("name"))
        {
          device_list_[i].name = (std::string)device_data["name"];
        }
        else
        {
          device_list_[i].name = "unknown";
        }

        // Get Speed
        if (device_data.hasMember("speed"))
        {
          device_list_[i].speed = (int)device_data["speed"];
        }
        else
        {
          device_list_[i].speed = 1000;
        }

        // Get Filter
        if (device_data.hasMember("filter"))
        {
          device_list_[i].filter = (int)device_data["filter"];
        }
        else
        {
          device_list_[i].filter = 15;
        }

        // Get Calibration matrix
        XmlRpc::XmlRpcValue calib_list;
        if (device_data.hasMember("calibration"))
        {
            calib_list = device_data["calibration"];
            for (int j = 0; j < calib_list.size(); j++)
            {
                ROS_ASSERT(calib_list[j].getType() == XmlRpc::XmlRpcValue::TypeDouble);
                device_list_[i].calib.push_back(static_cast<double>(calib_list[j]));
            }
            lcalib_.push_back(device_list_[i].calib);
        }
        else
        {
          for (int j = 0; j < 6; j++)
          {
            device_list_[i].F_trans.push_back(1);
          }
        }

        // Get Force Transformation matrix
        XmlRpc::XmlRpcValue force_trans_list;
        if (device_data.hasMember("force_transformation"))
        {
            force_trans_list = device_data["force_transformation"];
            for (int j = 0; j < force_trans_list.size(); j++)
            {
                ROS_ASSERT(force_trans_list[j].getType() == XmlRpc::XmlRpcValue::TypeInt);
                device_list_[i].F_trans.push_back(static_cast<int>(force_trans_list[j]));
            }
        }
        else
        {
          for (int j = 0; j < 9; j++)
          {
            device_list_[i].F_trans.push_back(1);
          }
        }

        // Get Torque Transformation matrix
        XmlRpc::XmlRpcValue torque_trans_list;
        if (device_data.hasMember("torque_transformation"))
        {
            torque_trans_list = device_data["torque_transformation"];
            for (int j = 0; j < torque_trans_list.size(); j++)
            {
                ROS_ASSERT(torque_trans_list[j].getType() == XmlRpc::XmlRpcValue::TypeInt);
                device_list_[i].T_trans.push_back(static_cast<int>(torque_trans_list[j]));
            }
        }
        else
        {
          for (int j = 0; j < 9; j++)
          {
            device_list_[i].T_trans.push_back(1);
          }
        }

        ldevice_.push_back(device_list_[i].name);
        lspeed_.push_back(device_list_[i].speed);
        lfilter_.push_back(device_list_[i].filter);


      }
  }
  ROS_INFO_STREAM("num_samples: " << num_samples_);
  ROS_INFO_STREAM("loop_rate: " << loop_rate_);

  for (int j = 0; j < connectedDAQs_; j++)
  {
    std::cout << "device " << j << std::endl;
    std::cout << "  name: " << device_list_[j].name << std::endl;
    std::cout << "  speed: " << device_list_[j].speed << std::endl;
    std::cout << "  filter: " << device_list_[j].filter << std::endl;
    std::cout << "  calib: [ ";
    for (int k = 0; k < device_list_[j].calib.size(); k ++)
    {
      std::cout << device_list_[j].calib[k] << " ";
    }
    std::cout << "]" << std::endl;
    std::cout << "  F_trans: [ ";
    for (int k = 0; k < device_list_[j].F_trans.size(); k ++)
    {
      std::cout << device_list_[j].F_trans[k] << " ";
    }
    std::cout << "]" << std::endl;
    std::cout << "  T_trans: [ ";
    for (int k = 0; k < device_list_[j].T_trans.size(); k ++)
    {
      std::cout << device_list_[j].T_trans[k] << " ";
    }
    std::cout << "]" << std::endl;
  }

  filter_ = 15; // in Hz

  return 0;
}

// Whatever the recording state is, top it.
// This way all previously get data will be discarded.
void optoforce_node::storeStart()
{
  force_acquisition_->startRecording();

}
void optoforce_node::storeStop()
{
    force_acquisition_->stopRecording();

}
