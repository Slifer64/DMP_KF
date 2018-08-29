#include <dmp_kf/Controller/Controller.h>

Controller::Controller(std::shared_ptr<Robot> &robot, std::shared_ptr<RefModel> &ref_model)
{
  this->robot = robot;
  this->ref_model = ref_model;
}

Controller::~Controller()
{

}
