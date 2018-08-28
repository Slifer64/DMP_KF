#include <dmp_lib/DMP/DMP_plus.h>
#include <optimization_lib/optimization_lib.h>

namespace as64_
{

  DMP_plus::DMP_plus() {}

  DMP_plus::DMP_plus(int N_kernels, double a_z, double b_z, std::shared_ptr<CanonicalClock> canClockPtr,
    std::shared_ptr<GatingFunction> shapeAttrGatingPtr, std::shared_ptr<GatingFunction> goalAttrGatingPtr,
    const param_::ParamList *paramListPtr)
  {
    this->init(N_kernels, a_z, b_z, canClockPtr, shapeAttrGatingPtr, goalAttrGatingPtr, paramListPtr);
  }

  double DMP_plus::train(const arma::rowvec &Time, const arma::rowvec &yd_data,
    const arma::rowvec &dyd_data, const arma::rowvec &ddyd_data, double y0, double g,
    const std::string &train_method, bool ret_train_err)
  {
    int n_data = Time.size();
    arma::rowvec x(n_data);
    arma::rowvec u(n_data);
    arma::rowvec Fd(n_data);
    arma::mat Psi(this->N_kernels, n_data);
    for (int i=0; i<n_data; i++)
    {
      x(i) = this->phase(Time(i));
      u(i) = this->shapeAttrGating(x(i));
      Fd(i) = this->calcFd(x(i), yd_data(i), dyd_data(i), ddyd_data(i), y0, g);
      Psi.col(i) = this->kernelFunction(x(i));
    }

    this->w.resize(this->N_kernels);
    this->b.resize(this->N_kernels);

    arma::mat X = arma::join_vert(u, arma::rowvec(n_data).ones())*this->forcingTermScaling(y0,g);

    arma::mat W;
    if (train_method.compare("LWR")==0)
    {
      W = opt_::LWR(Psi, X, Fd, this->zero_tol);
    }
    else if (train_method.compare("RLWR")==0)
    {
      W = opt_::RLWR(Psi, X, Fd, this->lambda, this->P_cov, this->zero_tol);
    }
    else if (train_method.compare("LS")==0)
    {
      W = opt_::normKernelLS(Psi, X, Fd, this->zero_tol);
    }
    else
    {
      throw std::runtime_error(std::string("Unsopported training method \"") + train_method);
    }

    this->w = W.col(0);
    this->b = W.col(1);

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

  double DMP_plus::learnedForcingTerm(double x, double y0, double g) const
  {
    double learnForcTerm = this->forcingTerm(x) * this->forcingTermScaling(y0, g);
    return learnForcTerm;
  }

  double DMP_plus::forcingTerm(double x) const
  {
    arma::vec Psi = this->kernelFunction(x);
    double u = this->shapeAttrGating(x);

    double f = arma::dot(Psi,this->w*u+this->b) / (arma::sum(Psi)+this->zero_tol); // add 'zero_tol' to avoid numerical issues
    return f;
  }

  double DMP_plus::shapeAttractor(double x, double y0, double g) const
  {
    double f = this->forcingTerm(x);
    double f_scale = this->forcingTermScaling(y0, g);
    double shape_attr = f * f_scale;
    return shape_attr;
  }

  arma::vec DMP_plus::kernelFunction(double x) const
  {
    arma::vec t = this->h % (arma::pow(x-this->c,2));
    arma::vec psi = arma::exp(-t);
    double std_thres = 2*std::pow(this->k_trunc_kernel,2);
    arma::uvec ind = arma::find(t>std_thres);
    psi.elem(ind) = arma::rowvec(ind.size()).zeros();

    return psi;
  }

  void DMP_plus::parseExtraArgs(const param_::ParamList *paramListPtr)
  {
    this->kernelStdScaling = 1.0;
    this->k_trunc_kernel = 4;

    if (paramListPtr)
    {
      paramListPtr->getParam("kernelStdScaling",this->kernelStdScaling);
      paramListPtr->getParam("k_trunc_kernel",this->k_trunc_kernel);
    }

  }

} // namespace as64_
