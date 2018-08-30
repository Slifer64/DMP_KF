#include <dmp_kf/Controller/Controller.h>

Controller::Controller(std::shared_ptr<Robot> &robot)
{
  this->robot = robot;
}

Controller::~Controller()
{

}
