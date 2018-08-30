/**
 * @file   optoforce_node.cpp
 * @author Asier Fernandez <asier.fernandez@tecnalia.com>
 * @date   2016
 *
 * Copyright 2016 Tecnalia Research & Innovation.
 * Distributed under the GNU GPL v3.
 * For full terms see https://www.gnu.org/licenses/gpl.txt
 *
 * @brief OptoForce ROS node with an interface through topics.
 *        The program inherits optoforce_node, which:
 *          reads parameters from ROS parameter server
 *          initialize  optoforce driver
 *          get periodically wrench data
 *
 *        ROS Subscribers. Type std_msgs::Bool
 *          start_publishing:
 *            true:   start publishing wrench data
 *            false:  stop publishing wrench data
 *          start_new_acquisition:
 *            true:   start recording data to be stored in a csv file
 *            false:  stop recording data to be stored in a csv file
 *
 */
#include "optoforce_topic.h"

optoforce_topic::optoforce_topic() : start_recording_(false)
{

}

optoforce_topic::~optoforce_topic()
{
  start_recording_ = false;
}

void optoforce_topic::add_ros_interface()
{
  ROS_INFO_STREAM("[optoforce_ros_interface]connectedDAQs_" << connectedDAQs_);

  // Initialize Publishers and Subscribers
  wrench_pub_.clear();
  subs_.clear();

  for (int i = 0; i < device_list_.size(); i++)
  {
      std::string publisher_name = "wrench_" + device_list_[i].name;
      wrench_pub_.push_back(nh_.advertise<geometry_msgs::WrenchStamped>(publisher_name, 1));
  }


  subs_.push_back( nh_.subscribe("start_publishing",
                           1,
                           &optoforce_topic::startPublishingCB,
                           this));

  subs_.push_back(nh_.subscribe("start_new_acquisition",
                                1,
                                &optoforce_topic::startRecordingCB,
                                this));

  subs_.push_back(nh_.subscribe("reset",
                           1,
                           &optoforce_topic::resetCB,
                           this));
}

// Calback enable/disable publishing topic
void optoforce_topic::startPublishingCB(const std_msgs::Bool::ConstPtr& msg)
{
  ROS_INFO_STREAM("[optoforce_topic::startPublishingCB] data: " << int(msg->data));

  publish_enable_ = msg->data;

  if (publish_enable_)
  {
    if (!force_acquisition_->isReading())
    {
      ROS_INFO("[optoforce_topic::startPublishingCB] start acq");
      force_acquisition_->startReading();
    }
    //transmitStart();
  }
  else
  {
    publish_enable_ = false;
    //transmitStop();
  }
}

// Callback enable/disable storing data to file when accquisition finish
void optoforce_topic::startRecordingCB(const std_msgs::Bool::ConstPtr& msg)
{
  ROS_INFO_STREAM("[optoforce_topic::startRecordingCB] data: " << int(msg->data));

  start_recording_ = msg->data;

  if (start_recording_)
  {
   if (!force_acquisition_->isRecording())
   {
     ROS_INFO("[optoforce_topic::startRecordingCB] is not Recording. START RECORDING");
     force_acquisition_->startRecording();
   }
   else
   {
     ROS_INFO("[optoforce_topic::startRecordingCB] is not Reading. proceed to start reading and recording data");
     ROS_INFO("[optoforce_topic::startRecordingCB] STOP RECORDING");
     force_acquisition_->stopRecording();
     ROS_INFO("[optoforce_topic::startRecordingCB] START RECORDING");
     force_acquisition_->startRecording();
   }

  }
  else if (msg->data == false)
  {
    if (force_acquisition_->isRecording())
    {
      ROS_INFO("[optoforce_topic::startRecordingCB] is Recording. proceed to stop recording and save data ");
      force_acquisition_->stopRecording();
    }
  }
}

// Calback to reset (to 0) the optoforce sensors readings
void optoforce_topic::resetCB(const std_msgs::Empty::ConstPtr& msg)
{
  ROS_INFO("[optoforce_topic::reset] ");

  force_acquisition_->setZeroAll();
}

// Inherited virtual method from optoforce_node
// Start transmision trough topics, only enable the flag
int optoforce_topic::run()
{
  ros::Rate loop_rate(loop_rate_);
  geometry_msgs::WrenchStamped wrench;
  std::vector< std::vector<float> > latest_samples;

  while(ros::ok())
  {
    // It is assumed that, callback function enables acquisition
    if(publish_enable_ && force_acquisition_->isReading() )
    {
      latest_samples.clear();

      force_acquisition_->getData(latest_samples);

      if ( latest_samples.size() == connectedDAQs_)
      {
        for (int i = 0; i < connectedDAQs_; i++)
        {
          if (latest_samples[i].size() == 6)
          {
            wrench.header.stamp = ros::Time::now();
            wrench.header.frame_id = device_list_[i].name;
            wrench.wrench.force.x  = latest_samples[i][0];
            wrench.wrench.force.y  = latest_samples[i][1];
            wrench.wrench.force.z  = latest_samples[i][2];
            wrench.wrench.torque.x = latest_samples[i][3];
            wrench.wrench.torque.y = latest_samples[i][4];
            wrench.wrench.torque.z = latest_samples[i][5];
            wrench_pub_[i].publish(wrench);
          }
        }
      }
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
}

// Inherited virtual method from optoforce_node
// Start transmision trough topics, only enable the flag
void optoforce_topic::transmitStart()
{
  publish_enable_ = true;
}

// Inherited virtual method from optoforce_node
// Stop transmision trough topics, only enable the flag
void optoforce_topic::transmitStop()
{
  publish_enable_ = false;
}


int main(int argc, char* argv[])
{
  ros::init(argc, argv, "optoforce_topic");
  ROS_INFO_STREAM("[optoforce_topic] Node name is:" << ros::this_node::getName());

  optoforce_topic of_topic;

  if (of_topic.init() < 0)
  {
    ROS_ERROR_STREAM("[optoforce_topic] optoforce_topic could not be initialized");
  }
  else
  {
    ROS_INFO_STREAM("[optoforce_topic] optoforce_topic Correctly initialized");

    // Add ROS Publisher and Subscribers
    of_topic.add_ros_interface();

    // Execute main loop of optoforce_node
    of_topic.run();
  }
  ROS_INFO_STREAM("[optoforce_topic] exit main");

  return 1;
}
