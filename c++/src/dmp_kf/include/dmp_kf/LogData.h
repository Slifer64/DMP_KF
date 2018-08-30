#ifndef OL_2D_RUP_LOG_DATA_H
#define OL_2D_RUP_LOG_DATA_H

#include <iostream>
#include <cstdlib>
#include <string>
#include <fstream>
#include <memory>

#include <armadillo>

#include <dmp_kf/Robot/Robot.h>
#include <dmp_kf/Controller/Controller.h>

class LogData
{
public:

  LogData(const std::shared_ptr<Robot> &r, const std::shared_ptr<Controller> &c);
  void init();
  void log();
  void save();
  void clear();

private:

  std::shared_ptr<Controller> controller;
  std::shared_ptr<Robot> robot;

  arma::rowvec Time_data;

  arma::mat Y_data;
  arma::mat dY_data;
  arma::mat ddY_data;

  arma::mat Fext_data;
  arma::mat Fext_filt_data;

  arma::mat g_hat_data;
  arma::mat Sigma_g_hat_data;
  arma::rowvec tau_hat_data;
  arma::rowvec sigma_tau_hat_data;
  arma::rowvec mf_data;


  bool binary;
  std::string data_input_path;
  std::string data_output_path;

  std::string in_data_filename;
  std::string out_data_filename;
};

#endif // OL_2D_RUP_LOG_DATA_H
