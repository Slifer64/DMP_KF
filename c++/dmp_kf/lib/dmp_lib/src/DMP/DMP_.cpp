#include <dmp_lib/DMP/DMP_.h>
#include <optimization_lib/optimization_lib.h>

#include <plot_lib/plot_lib.h>
#include <io_lib/io_lib.h>

namespace as64_
{

  DMP_::DMP_(int N_kernels, double a_z, double b_z, std::shared_ptr<CanonicalClock> canClockPtr,
    std::shared_ptr<GatingFunction> shapeAttrGatingPtr, std::shared_ptr<GatingFunction> goalAttrGatingPtr,
    const param_::ParamList *paramListPtr)
  {
    this->init(N_kernels, a_z, b_z, canClockPtr, shapeAttrGatingPtr, goalAttrGatingPtr, paramListPtr);
  }

  DMP_::DMP_(int N_kernels, double a_z, double b_z, std::shared_ptr<CanonicalClock> canClockPtr,
    std::shared_ptr<GatingFunction> shapeAttrGatingPtr, std::shared_ptr<GatingFunction> goalAttrGatingPtr,
    double kernel_std_scaling)
  {
    this->init(N_kernels, a_z, b_z, canClockPtr, shapeAttrGatingPtr, goalAttrGatingPtr, kernel_std_scaling);
  }

  DMP_::DMP_() {}


  void DMP_::init(int N_kernels, double a_z, double b_z, std::shared_ptr<CanonicalClock> canClockPtr,
    std::shared_ptr<GatingFunction> shapeAttrGatingPtr, std::shared_ptr<GatingFunction> goalAttrGatingPtr,
    const param_::ParamList *paramListPtr)
  {
    this->zero_tol = 1e-30; // realmin;

    this->shapeAttrGatingPtr = shapeAttrGatingPtr;
    this->goalAttrGatingPtr = goalAttrGatingPtr;

    this->N_kernels = N_kernels;
    this->a_z = a_z;
    this->b_z = b_z;
    this->canClockPtr = canClockPtr;

    this->parseExtraArgs(paramListPtr);

    this->a_s = 1.0/canClockPtr->getTau();

    this->w = arma::vec().zeros(this->N_kernels);

    this->setCenters();
    this->setStds(this->kernelStdScaling);

    param_::ParamList trainParamList;
    trainParamList.setParam("lambda", 0.99);
    trainParamList.setParam("P_cov", 1e6);
    this->setTrainingParams(&trainParamList);

  }

  void DMP_::init(int N_kernels, double a_z, double b_z, std::shared_ptr<CanonicalClock> canClockPtr,
    std::shared_ptr<GatingFunction> shapeAttrGatingPtr, std::shared_ptr<GatingFunction> goalAttrGatingPtr,
    double kernel_std_scaling)
  {
    param_::ParamList param_list;
    param_list.setParam("kernelStdScaling", kernel_std_scaling);

    this->init(N_kernels, a_z, b_z, canClockPtr, shapeAttrGatingPtr, goalAttrGatingPtr, &param_list);
  }


  double DMP_::shapeAttrGating(double x) const
  {
    double sAttrGat = this->shapeAttrGatingPtr->getOutput(x);
    if (sAttrGat<0) sAttrGat = 0.0;
    return sAttrGat;
  }


  double DMP_::goalAttrGating(double x) const
  {
    double gAttrGat = this->goalAttrGatingPtr->getOutput(x);
    if (gAttrGat>1.0) gAttrGat = 1.0;
    return gAttrGat;
  }


  double DMP_::phase(double t) const
  {
    double x = this->canClockPtr->getPhase(t);
    return x;
  }


  double DMP_::phaseDot(double x) const
  {
    double dx = this->canClockPtr->getPhaseDot(x);
    return dx;
  }


  double DMP_::get_v_scale() const
  {
    double v_scale = this->getTau() * this->a_s;
    return v_scale;
  }


  arma::vec DMP_::getStatesDot(double x, double y, double z, double y0, double g, double y_c, double z_c) const
  {
    double v_scale = this->get_v_scale();

    double shape_attr = this->shapeAttractor(x, y0, g);
    double goal_attr = this->goalAttractor(x, y, z, g);

    double dz = ( goal_attr + shape_attr + z_c) / v_scale;
    double dy = ( z + y_c) / v_scale;
    double dx = this->phaseDot(x);

    arma::vec statesDot(3);
    statesDot << dz << dy << dx;
    return statesDot;
  }


  double DMP_::train(const arma::rowvec &Time, const arma::rowvec &yd_data,
    const arma::rowvec &dyd_data, const arma::rowvec &ddyd_data, double y0, double g,
    const std::string &train_method, bool ret_train_err)
  {
    int n_data = Time.size();
    arma::rowvec x(n_data);
    arma::rowvec s(n_data);
    arma::rowvec Fd(n_data);
    arma::mat Psi(this->N_kernels, n_data);
    for (int i=0; i<n_data; i++)
    {
      x(i) = this->phase(Time(i));
      s(i) = this->forcingTermScaling(y0, g) * this->shapeAttrGating(x(i));
      Fd(i) = this->calcFd(x(i), yd_data(i), dyd_data(i), ddyd_data(i), y0, g);
      Psi.col(i) = this->kernelFunction(x(i));
    }

    if (train_method.compare("LWR")==0)
    {
      this->w = opt_::LWR(Psi, s, Fd, this->zero_tol);
    }
    else if (train_method.compare("RLWR")==0)
    {
      this->w = opt_::RLWR(Psi, s, Fd, this->lambda, this->P_cov, this->zero_tol);
    }
    else if (train_method.compare("LS")==0)
    {
      this->w = opt_::normKernelLS(Psi, s, Fd, this->zero_tol);
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
        F(i) = this->learnedForcingTerm(x(i), y0, g);
      }
      train_error = arma::norm(F-Fd)/F.size();
    }

    return train_error;
  }


  void DMP_::setTrainingParams(const param_::ParamList *paramListPtr)
  {
    this->lambda = 0.995;
    this->P_cov = 1e6;

    paramListPtr->getParam("lambda", this->lambda);
    paramListPtr->getParam("P_cov", this->P_cov);
  }


  double DMP_::calcFd(double x, double y, double dy, double ddy, double y0, double g) const
  {
    double v_scale = this->get_v_scale();
    double Fd = (ddy*std::pow(v_scale,2) - this->goalAttractor(x, y, v_scale*dy, g));
    return Fd;
  }


  double DMP_::learnedForcingTerm(double x, double y0, double g) const
  {
    double learnForcTerm = this->shapeAttrGating(x) * this->forcingTerm(x) * this->forcingTermScaling(y0, g);
    return learnForcTerm;
  }


  double DMP_::forcingTerm(double x) const
  {
    arma::vec Psi = this->kernelFunction(x);

    double f = arma::dot(Psi,this->w) / (arma::sum(Psi)+this->zero_tol); // add 'zero_tol' to avoid numerical issues
    return f;
  }


  double DMP_::forcingTermScaling(double y0, double g) const
  {
    double f_scale = (g-y0);
    return f_scale;
  }


  double DMP_::goalAttractor(double x, double y, double z, double g) const
  {
    double goal_attr = this->a_z*(this->b_z*(g-y)-z);
    goal_attr *= this->goalAttrGating(x);

    return goal_attr;
  }


  double DMP_::shapeAttractor(double x, double y0, double g) const
  {
    double f = this->forcingTerm(x);
    double f_scale = this->forcingTermScaling(y0, g);
    double shape_attr = f * f_scale * this->shapeAttrGating(x);
    return shape_attr;
  }


  arma::vec DMP_::kernelFunction(double x) const
  {
    arma::vec psi = arma::exp(-this->h % (arma::pow(x-this->c,2)));
    return psi;
  }


  void DMP_::setTau(double tau)
  {
    this->canClockPtr->setTau(tau);
  }

  double DMP_::getTau() const
  {
    double tau = this->canClockPtr->getTau();
    return tau;
  }


  void DMP_::setCenters()
  {
    this->c.resize(this->N_kernels);
    arma::rowvec t = arma::linspace<arma::rowvec>(0,this->N_kernels-1, this->N_kernels)/(this->N_kernels-1);
    for (int i=0;i<t.size();i++)
    {
      this->c(i) = this->phase(t(i)*this->getTau());
    }

  }


  void DMP_::setStds(double kernelStdScaling)
  {
    this->h.resize(this->N_kernels);
    int n = this->N_kernels;
    for (int i=0; i<n-1; i++)
    {
      this->h(i) = 1 / std::pow(kernelStdScaling*(this->c(i+1)-this->c(i)),2);
    }
    this->h(n-1) = this->h(n-2);
  }

  int DMP_::getNumKernels() const
  {
    return w.size();
  }

  void DMP_::parseExtraArgs(const param_::ParamList *paramListPtr)
  {
    this->kernelStdScaling = 1.0;

    if (paramListPtr)
    {
      paramListPtr->getParam("kernelStdScaling",this->kernelStdScaling);
    }
  }

  void DMP_::update_weights_with_RLWR(double x, double Ferr, double y0, double g, arma::mat &Sigma_w, double lambda)
  {
    double s = this->forcingTermScaling(y0, g);
    arma::vec psi = this->kernelFunction(x);

    arma::vec P = arma::diagvec(Sigma_w);

    // double error = Fd - this->w*s;
    double error = Ferr;

    arma::vec P_prev = P;

    P = (P - (arma::pow(P,2)*std::pow(s,2)) / (lambda/psi + P*std::pow(s,2))) / lambda;

    arma::uvec ind = arma::find(P>P_prev);
    P.elem(ind) = P_prev.elem(ind);

    this->w = this->w + psi%P*(s*error);

    Sigma_w = diagmat(P);
  }

  void DMP_::update_weights_with_KF(double x, double Ferr, double y0, double g, arma::mat &Sigma_w, double sigma_noise)
  {
    arma::vec psi = this->kernelFunction(x);
    arma::vec Psi = psi / (arma::sum(psi) + this->zero_tol);

    arma::vec K = Sigma_w*Psi*arma::inv(sigma_noise + Psi.t()*Sigma_w*Psi);

    this->w = this->w + K*Ferr;
    Sigma_w = Sigma_w - K*(Sigma_w*Psi).t();
  }

  void DMP_::update_weights_with_RLS(double x, double Ferr, double y0, double g, arma::mat &Sigma_w, double lambda)
  {
    arma::vec psi = this->kernelFunction(x);
    arma::vec Psi = psi / (sum(psi) + this->zero_tol);

    arma::vec K = Sigma_w*Psi*inv(lambda + Psi.t()*Sigma_w*Psi);

    this->w = this->w + K*Ferr;
    Sigma_w = (1/lambda) * (Sigma_w - K*(Sigma_w*Psi).t());
  }

} // namespace as64
