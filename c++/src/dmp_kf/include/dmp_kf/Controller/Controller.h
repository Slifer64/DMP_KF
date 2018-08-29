#ifndef CONTROLLER_OL_2D_RUP_H
#define CONTROLLER_OL_2D_RUP_H

#include <cstdlib>
#include <vector>
#include <memory>
#include <exception>

#include <ros/package.h>
#include <armadillo>

#include <dmp_kf/utils.h>
#include <dmp_kf/Robot/Robot.h>

class Controller
{
public:
  Controller(std::shared_ptr<Robot> &robot);
  ~Controller();

  virtual void init() = 0;

  virtual void update() = 0;

  virtual void run() = 0;

  // ========= Controller variables ============
  arma::vec S; ///< controller's state
  arma::vec dS;
  arma::vec ddS;

  bool is_okay;

protected:

  std::shared_ptr<Robot> robot;
};

#endif // CONTROLLER_OL_2D_RUP_H
