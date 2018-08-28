#ifndef BHAND_URDF_MODEL_H
#define BHAND_URDF_MODEL_H

#include <vector>

#include <ros/ros.h>
#include <ros/package.h>

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>

class BhandUrdfModel
{
struct Chain
{
  std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver;
  std::shared_ptr<KDL::ChainJntToJacSolver> jac_solver;
  KDL::Chain chain;

  std::string base_link;
  std::string tool_link;

  std::string name;
};

struct Joint
{
  Joint()
  {
    name = "joint";

    pos = 0;

    pos_lower_lim = -M_PI;
    pos_upper_lim = M_PI;
    vel_limit = 2*M_PI;
    effort_lim = 1000000;

    offset = 0;
    mult = 1.0;
  }

  std::string name;

  double pos;

  double pos_lower_lim;
  double pos_upper_lim;
  double vel_limit;
  double effort_lim;

  double offset;
  double mult;

  void setPosition(double p) { pos = (p-offset)*mult; }
};

public:
  BhandUrdfModel();
  ~BhandUrdfModel();

private:
  urdf::Model urdf_model;
  std::vector<Chain> chain;

  std::vector<Joint> joints;
  std::vector<std::shared_ptr<const urdf::Joint>> mimic_joints;

  std::vector<std::string> joint_alias;
  std::vector<std::string> link_names;
  std::vector<std::string> joint_names;
  std::vector<double> joint_pos_lower_lim;
  std::vector<double> joint_pos_upper_lim;
  std::vector<double> joint_vel_lim;
  std::vector<double> effort_lim;



};

#endif // BHAND_URDF_MODEL_H
