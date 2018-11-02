#include <kalmanfilter_lib/EKF.h>

namespace as64_
{

namespace kf_
{

EKF::EKF(unsigned N_params, unsigned N_out)
{}

void EKF::setFadingMemoryCoeff(double a_p)
{}

void EKF::enableParamsContraints(bool enable_contraints)
{}

void EKF::setParamsConstraints(const arma::mat &D_up, const arma::vec &d_up, const arma::mat &D_low, const arma::vec &d_low)
{}

void EKF::setProcessNoiseCov(const arma::mat &Q)
{}

void EKF::setMeasureNoiseCov(const arma::mat &R)
{}

void EKF::setPartDerivStep(const arma::vec &dtheta)
{}

void EKF::predict(const arma::mat &F_k)
{}

void EKF::predictApprox(void (*state_trans_fun_ptr)(const arma::vec &, void *), void *cookie)
{}

void EKF::correct(const arma::vec &z, const arma::vec &z_hat, const arma::mat &H_k)
{}

void EKF::correctApprox(void (*measure_trans_fun_ptr)(const arma::vec &, void *), void *cookie)
{}

} // namespace kf_

} // namespace as64_
