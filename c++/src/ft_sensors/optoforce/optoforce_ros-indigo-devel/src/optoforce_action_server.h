/**
 * @file   optoforce_action_server.h
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
#include "optoforce_node.h"
#include <actionlib/server/simple_action_server.h>
#include <optoforce_ros/OptoForceAction.h>

typedef actionlib::SimpleActionServer<optoforce_ros::OptoForceAction> ActionServer;

class optoforce_action_server : public optoforce_node {

  public:

    //! Constructor
    //! Initialize ROS Subscribers
    optoforce_action_server(std::string name);

    //! Destructor
    ~optoforce_action_server();

    //! Inherited virtual method from optoforce_node
    //! Start transmision trough topics, only enable the flag
    //! Add ROS Publisher and Subscribers
    void add_ros_interface();

    //! Inherited virtual method from optoforce_node
    //! Start transmision trough topics, only enable the flag
    void transmitStart ();

    //! Inherited virtual method from optoforce_node
    //! Stop transmision trough topics, only enable the flag
    void transmitStop ();

    //! Inherited virtual method from optoforce_node
    //! Start transmision trough topics, only enable the flag
    void run (const ActionServer::GoalConstPtr& goal);

  private:

    ActionServer* as_;
    std::string as_name_;

    //! goal message, received from client
    optoforce_ros::OptoForceGoal goal_;
    //! put results here, to be sent back to the client when done w/ goal
    optoforce_ros::OptoForceResult result_;
    //! for feedback
    optoforce_ros::OptoForceFeedback feedback_;
};
