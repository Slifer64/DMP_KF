#ifndef JOINT_LIMIT_AVOIDANCE_H
#define JOINT_LIMIT_AVOIDANCE_H


#include <armadillo>
#include <iostream>

namespace as64_
{

class JLAv
{
public:
    JLAv();
    JLAv( arma::vec qmin, arma::vec qmax);
    void init(arma::vec qmin, arma::vec qmax);

    void setLowLimit( int joint_index, double low_limit);
    void setLowLimits( arma::vec qmins);

    void setHighLimit( int joint_index, double high_limit);
    void setHighLimits( arma::vec qmaxs);

    void setGain( int joint_index, double gain);
    void setGains(double gain);
    void setGains( arma::vec gains);

    void printParams(void);
    arma::vec getControlSignal(arma::vec q_meas);

private:

    void initAllVarsToZero();

    // Initialization for YuMi
    int Njoints_ = 7;

    // parameters
    arma::vec qmin_;
    arma::vec qmax_;
    arma::vec qmean_;
    arma::vec rho_;
    arma::vec kq_;
    double gain_;

    // variables
    arma::vec e_, c_, epsilon_, dT_;
    arma::vec sig_;

};

} // namespace as64_

#endif // JOINT_LIMIT_AVOIDANCE_H
