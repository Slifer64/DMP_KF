/**
 * Copyright (C) 2017 GMM_GMR
 */

#ifndef GAUSSIAN_MIXTURE_MODELS_WITH_GAUSSIAN_MIXTURE_REGRESSION_H
#define GAUSSIAN_MIXTURE_MODELS_WITH_GAUSSIAN_MIXTURE_REGRESSION_H

#include <Eigen/Dense>
#include <memory>
#include <vector>
#include <cmath>
#include <string>
#include <exception>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <cstdlib>

namespace as64_
{
	
struct GMM_GMR_PARAMS
{
	Eigen::VectorXd Priors;
	std::vector<Eigen::VectorXd> Mu; 
	std::vector<Eigen::MatrixXd> Sigma;
};

class GMM_GMR
{
public:
  GMM_GMR();
  
  void get_output(const Eigen::MatrixXd &X_in, Eigen::MatrixXd *X_out) const;
  bool set_params(const GMM_GMR_PARAMS *params);
  bool get_params(GMM_GMR_PARAMS *params) const;
  bool load_params(const std::string &filename);
  bool save_params(const std::string &filename, int  precision=7) const;

  Eigen::VectorXd gaussPDF(const Eigen::MatrixXd &X, const Eigen::VectorXd &Mu, const Eigen::MatrixXd &Sigma) const;
  Eigen::VectorXd gaussPDF(const Eigen::MatrixXd &X, const Eigen::VectorXd &Mu, const Eigen::MatrixXd &inv_Sigma, double det_Sigma) const;

private:
	GMM_GMR_PARAMS GMR_params;
	std::vector<Eigen::MatrixXd> inv_Sigma_in;
	std::vector<double> det_Sigma_in;
	std::vector<Eigen::MatrixXd> A;
	
	void calc_meta_params();
	
	const double realmin;
};

}  // namespace as64_

#endif  // GAUSSIAN_MIXTURE_MODELS_WITH_GAUSSIAN_MIXTURE_REGRESSION_H
