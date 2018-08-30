/**
 * @file   optoforce_action_server.cpp
 * @author Asier Fernandez <asier.fernandez@tecnalia.com>
 * @date   2016
 *
 * Copyright 2016 Tecnalia Research & Innovation.
 * Distributed under the GNU GPL v3.
 * For full terms see https://www.gnu.org/licenses/gpl.txt
 *
 * @brief Basic OptoForce ROS action server
 *
 */
#include "optoforce_action_server.h"

optoforce_action_server::optoforce_action_server(std::string name) : as_name_(name),
                                                                     as_(NULL)
{
}

void optoforce_action_server::add_ros_interface()
{
  as_ = new ActionServer(nh_, as_name_, boost::bind(&optoforce_action_server::run, this, _1),false);
  as_->start();
  ROS_INFO("[add_ros_interface] actinlib started");
}
optoforce_action_server::~optoforce_action_server()
{

}
void optoforce_action_server::run(const ActionServer::GoalConstPtr& goal)
{

  ROS_INFO("[optoforce_action_server::run] Enter runCB");
  ROS_INFO_STREAM("[optoforce_action_server::run] goal store: " << static_cast<int>(goal->store));
  ROS_INFO_STREAM("[optoforce_action_server::run] goal acq_duration: " << goal->acq_duration);
  ROS_INFO_STREAM("[optoforce_action_server::run] goal publish_freq: " << goal->publish_freq);

  // Start Force Sensor Acquisition
  force_acquisition_->startRecording();
  if (goal->store)
    force_acquisition_->setAutoStore(true);

  ros::Rate timer(goal->publish_freq); // 1Hz timer
  geometry_msgs::WrenchStamped wrench;
  std::vector< std::vector<float> > latest_samples;

  double time_end = ros::Time::now().toNSec() + goal->acq_duration*1000000;

  while (ros::ok() && (ros::Time::now().toNSec() < time_end) )
  {
    if (as_->isPreemptRequested()){
       ROS_WARN("goal cancelled!");
       result_.result = 0;
       as_->setAborted(result_); // tell the client we have given up on this goal; send the result message as well
       return; // done with callback
    }

    latest_samples.clear();
    force_acquisition_->getData(latest_samples);

    feedback_.wrench_lst.clear();
    if (latest_samples.size() == connectedDAQs_ )
    {
      // Check if all received data's dimension is 6
      bool isDataValid = true;
      for (int i = 0; i < latest_samples.size(); i++)
      {
        if (latest_samples[i].size() == 6)
          isDataValid = (isDataValid & true);
        else
          isDataValid = false;
      }

      if (connectedDAQs_ > 0 && isDataValid)
      {
        wrench.header.stamp = ros::Time::now();
        wrench.wrench.force.x  = latest_samples[0][0];
        wrench.wrench.force.y  = latest_samples[0][1];
        wrench.wrench.force.z  = latest_samples[0][2];
        wrench.wrench.torque.x = latest_samples[0][3];
        wrench.wrench.torque.y = latest_samples[0][4];
        wrench.wrench.torque.z = latest_samples[0][5];
        feedback_.wrench_lst.push_back(wrench);
      }
      if (connectedDAQs_ == 2 && isDataValid)
      {
        wrench.header.stamp = ros::Time::now();
        wrench.wrench.force.x  = latest_samples[1][0];
        wrench.wrench.force.y  = latest_samples[1][1];
        wrench.wrench.force.z  = latest_samples[1][2];
        wrench.wrench.torque.x = latest_samples[1][3];
        wrench.wrench.torque.y = latest_samples[1][4];
        wrench.wrench.torque.z = latest_samples[1][5];
        feedback_.wrench_lst.push_back(wrench);
      }
      as_->publishFeedback(feedback_); // send feedback to the action client that requested this goal

    }
    else
      ROS_INFO("Not valida data received from Force Sensor");

    timer.sleep();
  }
  force_acquisition_->stopRecording();

  std::cout << "time finish: " << ros::Time::now().toNSec() << std::endl;


  result_.result = 1;
  as_->setSucceeded(result_);
  ROS_INFO("[optoforce_action_server::run] Finish loop");
}

// Inherited virtual method from optoforce_node
// Start transmision trough topics, only enable the flag
void optoforce_action_server::transmitStart()
{
  publish_enable_ = true;
}

// Inherited virtual method from optoforce_node
// Stop transmision trough topics, only enable the flag
void optoforce_action_server::transmitStop()
{
  publish_enable_ = false;
}


int main(int argc, char* argv[])
{

  ros::init(argc, argv, "optoforce_action_server");
  ROS_INFO_STREAM("[optoforce_action_server] Node name is:" << ros::this_node::getName());

  std::string action_name = "action_server";
  optoforce_action_server optoforce_as(action_name);

  if (optoforce_as.init() < 0)
  {
    std::cout << "optoforce_action_server could not be initialized" << std::endl;
  }
  else
  {
    std::cout << "optoforce_action_server Correctly initialized" << std::endl;

    // Add Actionlib ROS Interface
    optoforce_as.add_ros_interface();

    ros::spin();
  }
  std::cout << "exit main" << std::endl;

  return 1;

}
