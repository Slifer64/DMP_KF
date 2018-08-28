/**
 * Copyright (C) 2016 AUTH-ARL
 */

#include <lwr_robot/lwr_model.h>
#include <string>
#include <vector>
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <ros/ros.h>

namespace lwr
{
namespace robot
{
Model::Model()
{
  load();
}

Model::Model(const std::string& path)
{
  load(path);
}

void Model::load()
{
  name = "KUKA LWR4+";
  num_chains_exclusive = 1;
  chain_name.push_back("kuka_lwr_arm");
  joint_name.push_back({"lwr_arm_0_joint", "lwr_arm_1_joint", "lwr_arm_2_joint",  // NOLINT(whitespace/braces)
                        "lwr_arm_3_joint", "lwr_arm_4_joint", "lwr_arm_5_joint",
                        "lwr_arm_6_joint"});  // NOLINT(whitespace/braces)

  link_name.push_back({"lwr_arm_base_link", "lwr_arm_1_link", "lwr_arm_2_link",  // NOLINT(whitespace/braces)
                       "lwr_arm_3_link", "lwr_arm_4_link", "lwr_arm_5_link",
                       "lwr_arm_6_link", "lwr_arm_7_link"});  // NOLINT(whitespace/braces)

  global_index.push_back({0, 1, 2, 3, 4, 5, 6});  // NOLINT(whitespace/braces)
}

void Model::load(const std::string& path)
{
  load();

  urdf::Model urdf_model;
  if (!urdf_model.initParam(path))
  {
    ROS_INFO_STREAM("Error getting robot description");
  }
  else
  {
    ROS_INFO_STREAM("Read urdf model");
    KDL::Tree tree;
    kdl_parser::treeFromUrdfModel(urdf_model, tree);
    for (size_t i = 0; i < getNrOfMainChains(); i++)
    {
      KDL::Chain new_chain;
      if (!tree.getChain(link_name.at(i).at(0), link_name.at(i).at(link_name.at(i).size() - 1), new_chain))
      {
        ROS_INFO_STREAM("Creating chain from " << link_name.at(i).at(0) << " to "
                                               << link_name.at(i).at(link_name.at(i).size() - 1) << " failed");
      }
      else
      {
        ROS_INFO_STREAM("Created chain from " << link_name.at(i).at(0) << " to "
                                              << link_name.at(i).at(link_name.at(i).size() - 1));
        fk_solver.push_back(KDL::ChainFkSolverPos_recursive(new_chain));
        jac_solver.push_back(KDL::ChainJntToJacSolver(new_chain));
        chain.push_back(new_chain);
      }
      std::vector<std::pair<double, double>> pairs;
      for (size_t j = 0; j < joint_name.at(i).size(); j++)
      {
        std::pair<double, double> limit;
        limit.first = urdf_model.getJoint(joint_name.at(i).at(j))->limits->lower;
        limit.second = urdf_model.getJoint(joint_name.at(i).at(j))->limits->upper;
        pairs.push_back(limit);
      }
      joint_limit.push_back(pairs);
    }
  }
}

}  // namespace robot
}  // namespace lwr
