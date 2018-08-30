/**
 * @file   optoforce_action_client.cpp
 * @author Asier Fernandez <asier.fernandez@tecnalia.com>
 * @date   2016
 *
 * Copyright 2016 Tecnalia Research & Innovation.
 * Distributed under the GNU GPL v3. For full terms see https://www.gnu.org/licenses/gpl.txt
 *
 * @brief Basic ROS action client to communicate with OptoForce action server
 *
 */
#include <optoforce_ros/OptoForceAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<optoforce_ros::OptoForceAction> Client;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "optoforce_action_client");

  std::string as_name = "optoforce_node/action_server";
  Client client(as_name, true); // true -> don't need ros::spin()

  client.waitForServer(ros::Duration(5.0));

  optoforce_ros::OptoForceGoal goal;

  // Fill in goal here
  goal.store = true;
  goal.acq_duration = 1000.0;
  goal.publish_freq = 1000;
  client.sendGoal(goal);
  client.waitForResult(ros::Duration(5.0));

  if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    std::cout << "Action Client succeeded";

  std::cout << "Current State: " << client.getState().toString().c_str();


  return 0;
}
