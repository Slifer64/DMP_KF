#ifndef DMP_KF_LOG_DATA_H
#define DMP_KF_LOG_DATA_H

#include <iostream>
#include <cstdlib>
#include <string>
#include <fstream>
#include <memory>

#include <armadillo>

#include <dmp_kf/Robot/Robot.h>

// =======================================
// ===========  KinematicData  ===========
// =======================================

struct KinematicData
{
public:
  arma::rowvec Time;
  arma::mat Y_data;
  arma::mat dY_data;
  arma::mat ddY_data;

  KinematicData() {};

  // saves the data in "ros::package::getPath(PACKAGE_NAME)+ "/data/" + file_name"
  // On success returns true, otherwise false and the error message in 'err_msg'
  bool save(const std::string &file_name, std::string &err_msg);

  // adds t,Y,dY,dYY to the corresponding data matrices
  void log(double t, const arma::vec &Y, const arma::vec &dY, const arma::vec &ddY);

  // empties all data
  void clear();

  // returns true if empty
  bool isempty() const { return Time.size()==0; }

  // number/length of data points
  int length() const { return Y_data.n_cols; }

  // dimensionallity of data
  int dim() const { return Y_data.n_rows; }

  // returns the last point
  arma::vec getFinalPoint() const { return Y_data.col(length()-1); }

  double getTimeDuration() const { return Time(length()-1)-Time(0); }
};

// ===========================================================
// ===========================================================

struct ExecutionData
{
public:
  arma::rowvec Time;

  arma::mat Y_data;
  arma::mat dY_data;
  arma::mat ddY_data;

  arma::mat Fext_data;
  arma::mat Fext_filt_data;
  arma::rowvec mf_data;

  arma::mat theta_data;
  arma::mat Sigma_theta_data;

  arma::vec y_g; // goal position

  ExecutionData();

  // saves the data in "ros::package::getPath(PACKAGE_NAME)+ "/data/" + file_name"
  // On success returns true, otherwise false and the error message in 'err_msg'
  bool save(const std::string &file_name, std::string &err_msg);

  // adds t,Y,dY,dYY to the corresponding data matrices
  void log(double t, const arma::vec &Y, const arma::vec &dY, const arma::vec &ddY,
          const arma::vec &Fext, const arma::vec &Fext_filt, double mf,
					const arma::vec &theta, const arma::mat &P_theta);

  // empties all data
  void clear();

  // returns true if empty
  bool isempty() const { return Time.size()==0; }

};

// ===========================================================
// ===========================================================

// class LogData
// {
// public:
//
//   LogData(const std::shared_ptr<Robot> &r, const std::shared_ptr<Controller> &c);
//   void init();
//   void log();
//   void save();
//   void clear();
//
//   void saveTraining();
//   void loadTraining();
//
// private:
//
//   std::shared_ptr<Controller> controller;
//   std::shared_ptr<Robot> robot;
//
//   arma::rowvec Time_data;
//
//   arma::mat Y_data;
//   arma::mat dY_data;
//   arma::mat ddY_data;
//
//   arma::mat Fext_data;
//   arma::mat Fext_filt_data;
//
//   arma::mat g_hat_data;
//   arma::mat Sigma_g_hat_data;
//   arma::rowvec tau_hat_data;
//   arma::rowvec sigma_tau_hat_data;
//   arma::rowvec mf_data;
//
//
//   bool binary;
//   std::string data_input_path;
//   std::string data_output_path;
//
//   std::string in_data_filename;
//   std::string out_data_filename;
//
// };

#endif // DMP_KF_LOG_DATA_H
