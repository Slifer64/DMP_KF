#ifndef OL_2D_RUP_LOG_DATA_H
#define OL_2D_RUP_LOG_DATA_H

#include <iostream>
#include <cstdlib>
#include <string>
#include <fstream>
#include <memory>

#include <armadillo>

#include <OL_2D_rup/Robot/Robot.h>
#include <OL_2D_rup/Controller/Controller.h>

class LogData
{
public:

  LogData();
  void init();
  void log(const std::shared_ptr<Robot> &robot, const std::shared_ptr<Controller> &controller);
  void save();
  void clear();

private:
  arma::rowvec Time_data;

  arma::mat S_ref_data;
  arma::mat dS_ref_data;
  arma::mat ddS_ref_data;

  arma::mat S_r_data;
  arma::mat dS_r_data;
  arma::mat ddS_r_data;

  arma::mat Fext_data;
  arma::mat F_c_data;
  arma::mat F_c_d_data;

  bool binary;
  std::string data_input_path;
  std::string data_output_path;

  std::string in_data_filename;
  std::string out_data_filename;
};

#endif // OL_2D_RUP_LOG_DATA_H
