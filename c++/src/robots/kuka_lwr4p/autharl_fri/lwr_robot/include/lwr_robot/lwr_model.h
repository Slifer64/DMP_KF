/**
 * Copyright (C) 2016 AUTH-ARL
 */

#ifndef LWR_ROBOT_LWR_MODEL_H
#define LWR_ROBOT_LWR_MODEL_H

#include <autharl_core/robot/model.h>
#include <memory>
#include <vector>
#include <string>

namespace lwr
{
namespace robot
{
class Model : public arl::robot::Model
{
public:
  Model();
  explicit Model(const std::string& path);

private:
  void load();
  void load(const std::string& path);
};
}  // namespace robot
}  // namespace lwr

#endif  // LWR_ROBOT_LWR_MODEL_H
