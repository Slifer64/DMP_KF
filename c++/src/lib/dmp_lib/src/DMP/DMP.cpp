#include <dmp_lib/DMP/DMP.h>
#include <dmp_lib/trainMethods/LeastSquares.h>
#include <dmp_lib/trainMethods/LWR.h>

namespace as64_
{

  DMP::DMP(int N_kernels, double a_z, double b_z, std::shared_ptr<CanonicalClock> &can_clock_ptr,
    std::shared_ptr<GatingFunction> &shape_attr_gating_ptr)
  {
    this->init(N_kernels, a_z, b_z, can_clock_ptr, shape_attr_gating_ptr);
  }

  void DMP::init(int N_kernels, double a_z, double b_z, std::shared_ptr<CanonicalClock> &can_clock_ptr,
    std::shared_ptr<GatingFunction> &shape_attr_gating_ptr)
  {
    this->zero_tol = 1e-30; // realmin;

    this->shape_attr_gating_ptr = shape_attr_gating_ptr;

    this->N_kernels = N_kernels;
    this->a_z = a_z;
    this->b_z = b_z;
    this->can_clock_ptr = can_clock_ptr;

    this->w = arma::vec().zeros(this->N_kernels);
    this->setCenters();
    double kernel_std_scaling = 1.0;
    this->setStds(kernel_std_scaling);
  }


  double DMP::train(const std::string &train_method, const arma::rowvec &Time,
    const arma::rowvec &yd_data, const arma::rowvec &dyd_data, const arma::rowvec &ddyd_data, bool ret_train_err)
  {
    int n_data = Time.size();
    int i_end = n_data-1;

    double tau0 = this->getTau();
    double tau = Time(i_end);
    double y0 = yd_data(0);
    double g = yd_data(i_end);

    this->setTau(tau);

    arma::rowvec x(n_data);
    arma::rowvec s(n_data);
    arma::rowvec Fd(n_data);
    arma::mat Psi(this->numOfKernels(), n_data);
    for (int i=0; i<n_data; i++)
    {
      x(i) = this->phase(Time(i));
      s(i) = this->forcingTermScaling(y0, g) * this->shapeAttrGating(x(i));
      Fd(i) = this->calcFd(x(i), yd_data(i), dyd_data(i), ddyd_data(i), y0, g);
      Psi.col(i) = this->kernelFunction(x(i));
    }

    if (train_method.compare("LWR")==0)
    {
      this->w = LWR(Psi, s, Fd, this->zero_tol);
    }
    else if (train_method.compare("LS")==0)
    {
      this->w = leastSquares(Psi, s, Fd, this->zero_tol);
    }
    else
    {
      throw std::runtime_error(std::string("Unsopported training method \"") + train_method);
    }

    double train_error = -1;
    if (ret_train_err)
    {
      arma::rowvec F(Fd.size());
      for (int i=0; i<F.size(); i++)
      {
        F(i) = this->shapeAttractor(x(i), y0, g);
      }
      train_error = arma::norm(F-Fd)/F.size();
    }

    return train_error;
  }


  arma::vec DMP::statesDot(double x, double y, double z, double y0, double g, double y_c, double z_c) const
  {
    double tau = this->getTau();

    double shape_attr = this->shapeAttractor(x, y0, g);
    double goal_attr = this->goalAttractor(x, y, z, g);

    double dz = ( goal_attr + shape_attr + z_c) / tau;
    double dy = ( z + y_c) / tau;
    double dx = this->phaseDot(x);

    arma::vec statesDot(3);
    statesDot << dz << dy << dx;
    return statesDot;
  }


  void DMP::setTau(double tau)
  {
    this->can_clock_ptr->setTau(tau);
  }


  double DMP::getTau() const
  {
    return this->can_clock_ptr->getTau();
  }


  double DMP::phase(double t) const
  {
    return this->can_clock_ptr->getPhase(t);
  }


  double DMP::phaseDot(double x) const
  {
    return this->can_clock_ptr->getPhaseDot(x);
  }


  arma::vec DMP::kernelFunction(double x) const
  {
    arma::vec psi = arma::exp(-this->h % (arma::pow(x-this->c,2)));
    return psi;
  }


  double DMP::goalAttractor(double x, double y, double z, double g) const
  {
    double goal_attr = this->a_z*(this->b_z*(g-y)-z);
    goal_attr *= this->goalAttrGating(x);

    return goal_attr;
  }


  double DMP::shapeAttractor(double x, double y0, double g) const
  {
    double f = this->forcingTerm(x);
    double f_scale = this->forcingTermScaling(y0, g);
    double shape_attr = f * f_scale * this->shapeAttrGating(x);
    return shape_attr;
  }


  double DMP::shapeAttrGating(double x) const
  {
    double sAttrGat = this->shape_attr_gating_ptr->getOutput(x);
    if (sAttrGat<0) sAttrGat = 0.0;
    return sAttrGat;
  }


  double DMP::goalAttrGating(double x) const
  {
    double gAttrGat = 1.0;
    return gAttrGat;
  }


  double DMP::forcingTerm(double x) const
  {
    arma::vec Psi = this->kernelFunction(x);
    double f = arma::dot(Psi,this->w) / (arma::sum(Psi)+this->zero_tol); // add 'zero_tol' to avoid numerical issues
    return f;
  }


  double DMP::forcingTermScaling(double y0, double g) const
  {
    double f_scale = (g-y0);
    return f_scale;
  }


  void DMP::setCenters()
  {
    int N_kernels = this->numOfKernels();
    this->c.resize(N_kernels);
    arma::rowvec t = arma::linspace<arma::rowvec>(0,N_kernels-1, N_kernels)/(N_kernels-1);
    for (int i=0;i<t.size();i++)
    {
      this->c(i) = this->phase(t(i)*this->getTau());
    }

  }


  void DMP::setStds(double kernelStdScaling)
  {
    int N_kernels = this->numOfKernels();
    this->h.resize(N_kernels);
    for (int i=0; i<N_kernels-1; i++)
    {
      this->h(i) = 1 / std::pow(kernelStdScaling*(this->c(i+1)-this->c(i)),2);
    }
    this->h(N_kernels-1) = this->h(N_kernels-2);
  }


  int DMP::numOfKernels() const
  {
    return w.size();
  }


  double DMP::calcFd(double x, double y, double dy, double ddy, double y0, double g) const
  {
    double tau = this->getTau();
    double Fd = (ddy*std::pow(tau,2) - this->goalAttractor(x, y, tau*dy, g));
    return Fd;
  }

  arma::vec DMP::getAcellPartDev_g_tau(double t, double y, double dy, double y0,
                                  double x_hat, double g_hat, double tau_hat) const
  {
    arma::vec dC_dtheta = arma::vec().zeros(2);

    double K_dmp = this->a_z*this->b_z;
    double D_dmp = this->a_z;
    arma::vec psi = this->kernelFunction(x_hat);
    double sum_psi = arma::sum(psi) + this->zero_tol;
    double sum_w_psi = arma::dot(psi, this->w);
    double shape_attr_gat = this->shapeAttrGating(x_hat);

    double theta1 = g_hat;
    double theta2 = 1/tau_hat;

    double dshape_attr_gat_dtheta2 = this->shape_attr_gating_ptr->getPartDev_1oTau(t,x_hat);

    arma::vec dPsidtheta2 = -2*t*this->h%(theta2*t-this->c)%psi;
    double sum_w_dPsidtheta2 = arma::dot(this->w, dPsidtheta2);
    double dSumWPsi_dtheta2 = (sum_w_dPsidtheta2*sum_psi - sum_w_psi*arma::sum(dPsidtheta2) ) / std::pow(sum_psi,2);

    dC_dtheta(0) = (K_dmp + shape_attr_gat*sum_w_psi/sum_psi)*std::pow(theta2,2);

    dC_dtheta(1) = 2*theta2* (K_dmp*(theta1-y) + shape_attr_gat*(theta1-y0)*sum_w_psi/sum_psi) \
        -D_dmp*dy + std::pow(theta2,2)*(theta1-y0)*( dshape_attr_gat_dtheta2*sum_w_psi/sum_psi + shape_attr_gat*dSumWPsi_dtheta2 );
    dC_dtheta(1) = dC_dtheta(1)*(-1/std::pow(tau_hat,2));

    return dC_dtheta;
  }


  double DMP::getAccel(double y, double dy, double y0, double y_c, double z_c,
                  double x_hat, double g_hat, double tau_hat) const
  {
    double z = dy*tau_hat;
    arma::vec s_dot = this->statesDot(x_hat, y, z, y0, g_hat, y_c, z_c);
    double dz = s_dot(0);
    double ddy = dz/tau_hat;
    return ddy;
  }

} // namespace as64
