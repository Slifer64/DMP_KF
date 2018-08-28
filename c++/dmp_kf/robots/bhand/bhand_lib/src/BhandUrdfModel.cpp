#include "bhand_lib/BhandUrdfModel.h"

#include <stack>

BhandUrdfModel::BhandUrdfModel()
{
  joint_names = {"bh_j11_joint", "bh_j12_joint", "bh_j22_joint", "bh_j32_joint"};
  joint_alias = {"spread", "finger1", "finger2", "finger3"};

  std::string urdf_file_path = ros::package::getPath("bhand_lib") + "/urdf/bh282_robot.urdf";

  if (!urdf_model.initFile(urdf_file_path.c_str()))
  {
    throw std::ios_base::failure("Failed to load urdf model from \"" + urdf_file_path + "\"...\n");
  }

  std::vector<boost::shared_ptr<urdf::Joint>> _joints;
  std::stack<boost::shared_ptr<const urdf::Link>> link_stack;
  link_stack.push(urdf_model.getRoot());

  while (!link_stack.empty())
  {
    auto link = link_stack.top();
    link_stack.pop();

    link_names.push_back(link->name);

    for (int i=0;i<link->child_links.size();i++) link_stack.push(link->child_links[i]);
    for (int i=0;i<link->child_joints.size();i++) _joints.push_back(link->child_joints[i]);
  }

  base_link = {"bh_base_link", "bh_base_link", "bh_base_link"};
  tool_link = {"bh_finger_1_tip_link", "bh_finger_2_tip_link", "bh_finger_3_tip_link"};

  for (int i=0;i<base_link.size();i++)
  {
    bool found_base_link = false;
    bool found_tool_link = false;

    for (int j=0;j<link_names.size();j++)
    {
      if (base_link[i].compare(link_names[j]) == 0) found_base_link = true;
      else if (tool_link[i].compare(link_names[j]) == 0) found_tool_link = true;
    }

    if (!found_base_link)
      throw std::runtime_error("Couldn't find specified base link \"" + base_link[i] + "\" in the robot urdf model...\n");

    if (!found_tool_link)
      throw std::runtime_error("Couldn't find specified tool link \"" + tool_link[i] + "\" in the robot urdf model...\n");
  }

  for (int i=0;i<_joints.size();i++)
  {
    auto joint = _joints[i];
    auto jtype = joint->type;

    if (jtype==urdf::Joint::FIXED || jtype==urdf::Joint::FLOATING) continue;

    if (joint->mimic) continue;

    // std::cout << "===> joint " << i+1 << ": name = " << joint->name << ", type: " << joint_type[jtype] << "\n";
    //
    // boost::shared_ptr<urdf::JointMimic> j_mimic = joint->mimic;
    // if (j_mimic)
    // {
    //   std::cout << "mimic joint: " << j_mimic->joint_name << "\n";
    //   std::cout << "offset = " << j_mimic->offset << "\n";
    //   std::cout << "multiplier = " << j_mimic->multiplier << "\n";
    // }

    joint_names.push_back(joint->name);

    if (jtype==urdf::Joint::CONTINUOUS)
    {
      joint_pos_lower_lim.push_back(-M_PI);
      joint_pos_upper_lim.push_back(M_PI);
    }
    else
    {
      joint_pos_lower_lim.push_back(joint->limits->lower);
      joint_pos_upper_lim.push_back(joint->limits->upper);
    }

    effort_lim.push_back(joint->limits->effort);
    joint_vel_lim.push_back(joint->limits->velocity);
  }

  KDL::Tree tree;
  kdl_parser::treeFromUrdfModel(urdf_model, tree);
  int N_chains = base_link.size();
  chain.resize(N_chains);
  fk_solver.resize(N_chains);
  jac_solver.resize(N_chains);

  for (int i=0;i<N_chains;i++)
  {
    if (!tree.getChain(base_link[i], tool_link[i], chain[i]))
    {
      throw std::runtime_error("Failed to create kdl chain from " + base_link[i] + " to " + tool_link[i] + " ...\n");
    }
    else
    {
      fk_solver[i].reset(new KDL::ChainFkSolverPos_recursive(chain[i]));
      jac_solver[i].reset(new KDL::ChainJntToJacSolver(chain[i]));
    }
  }

  // N_JOINTS = joint_names.size();
  // joint_pos.zeros(N_JOINTS);
  // joint_prev_pos.zeros(N_JOINTS);
  // joint_vel.zeros(N_JOINTS);
  // joint_torques.zeros(N_JOINTS);
  // pose.resize(4,4);
  // Jacob.resize(6,N_JOINTS);
  // Fext.zeros(6);
}

BhandUrdfModel::~BhandUrdfModel()
{

}
