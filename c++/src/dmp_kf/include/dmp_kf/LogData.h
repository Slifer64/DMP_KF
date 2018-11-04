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

  // Trims the data from the beginning and end base on the velocity
  // thresholds specified.
  void trim(double v_start_thres, double v_end_thres);

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

  arma::mat Y_ref_data;
  arma::mat dY_ref_data;
  arma::mat ddY_ref_data;

  arma::mat Fext_data;
  arma::mat Fext_filt_data;

  arma::mat theta_data;
  arma::mat Sigma_theta_data;

  arma::vec y_g; // goal position

  ExecutionData();

  // saves the data in "ros::package::getPath(PACKAGE_NAME)+ "/data/" + file_name"
  // On success returns true, otherwise false and the error message in 'err_msg'
  bool save(const std::string &file_name, std::string &err_msg);

  // adds t,Y,dY,dYY to the corresponding data matrices
  void log(double t, const arma::vec &Y, const arma::vec &dY, const arma::vec &ddY,
          const arma::vec &Y_ref, const arma::vec &dY_ref, const arma::vec &ddY_ref,
          const arma::vec &Fext, const arma::vec &Fext_filt,
					const arma::vec &theta, const arma::mat &P_theta);

  // empties all data
  void clear();

  // returns true if empty
  bool isempty() const { return Time.size()==0; }

};

// ===========================================================
// ===========================================================


#endif // DMP_KF_LOG_DATA_H
