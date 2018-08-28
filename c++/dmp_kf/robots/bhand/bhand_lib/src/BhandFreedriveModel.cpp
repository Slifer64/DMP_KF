#include <bhand_lib/BhandFreedriveModel.h>
#include <bhand_lib/parser.h>

#include <ros/ros.h>
#include <ros/package.h>

BhandFreedriveModel::BhandFreedriveModel()
{
  std::string cfg_file_path = ros::package::getPath("bhand_lib") + "/config/freedrive_config.yaml";

  bhand_::Parser parser(cfg_file_path);

  if (!parser.getParam("inertia", M)) M = arma::vec().ones(4)*1.0;
  if (!parser.getParam("damping", D)) D = arma::vec().ones(4)*20.0;
}

BhandFreedriveModel::~BhandFreedriveModel()
{

}

void BhandFreedriveModel::init()
{

}

void BhandFreedriveModel::run()
{

}
