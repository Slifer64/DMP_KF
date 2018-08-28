#include <dmp_lib/DMP/DMP_Shannon.h>
#include <sigproc_lib/sigproc_lib.h>
#include <optimization_lib/optimization_lib.h>
#include <io_lib/io_lib.h>
#include <sigproc_lib/sigproc_lib.h>

namespace as64_
{

  DMP_Shannon::DMP_Shannon() {}

  DMP_Shannon::DMP_Shannon(int N_kernels, double a_z, double b_z, std::shared_ptr<CanonicalClock> canClockPtr,
    std::shared_ptr<GatingFunction> shapeAttrGatingPtr, std::shared_ptr<GatingFunction> goalAttrGatingPtr,
    const param_::ParamList *paramListPtr)
  {
    this->init(N_kernels, a_z, b_z, canClockPtr, shapeAttrGatingPtr, goalAttrGatingPtr, paramListPtr);
  }

  double DMP_Shannon::train(const arma::rowvec &Time, const arma::rowvec &yd_data,
    const arma::rowvec &dyd_data, const arma::rowvec &ddyd_data, double y0, double g,
    const std::string &train_method, bool ret_train_err)
  {
    double tau = this->getTau();
    int n_data = Time.size();
    arma::rowvec x(n_data);
    arma::rowvec s(n_data);
    arma::rowvec Fd(n_data);
    for (int i=0; i<n_data; i++)
    {
        x(i) = this->phase(Time(i));
        s(i) = this->forcingTermScaling(y0, g) * this->shapeAttrGating(x(i));
        Fd(i) = this->calcFd(x(i), yd_data(i), dyd_data(i), ddyd_data(i), y0, g);
    }

    double Ts = Time(1)-Time(0);
    double Fs = 1.0/Ts;
    arma::rowvec f, P1;
    as64_::spl_::getSingleSidedFourier(Fd, Fs, f, P1);

    double Freq_max = std::min(this->Freq_max, f(f.size()-1));
    // double Freq_max = f(f.size()-1);

    // find the maximum required frequency to get at least 'Wmin' percent of the
    // total signal's energy
    double W = arma::sum(arma::pow(P1.elem(arma::find(f<=Freq_max)),2));
    //double W = arma::sum(arma::pow(P1.t(),2));
    double W_temp = 0.0;
    int k = -1;
    double W_temp_min = W*this->Wmin;
    while (W_temp < W_temp_min)
    {
      k++;
      W_temp += std::pow(P1(k),2);
    }

    double Freq1 = f(k);
    // printf("Frequency to get at least %.3f of the energy: Freq=%.3f Hz\n", this->Wmin, Freq1);

    while (k < f.size())
    {
        if (f(k) >= Freq_max) break;
        if (P1(k) < this->P1_min) break;
        k++;
    }

    if (k == f.size()) k--;

    double Fmax = f(k);
    Fmax = std::min(this->Freq_max,Fmax);
    Fmax = std::max(this->Freq_min, Fmax);
    // printf("Frequency after which the amplitude drops below %.3f: Freq=%.3f Hz\n", this->P1_min, Fmax);

    // ==> Filter the signal retaining at least 'Wmin' energy
    as64_::spl_::FIR_filt<double,double,double> fir_filt;
  	arma::vec filter_b = as64_::spl_::fir1(Fs/this->Freq_min, Fmax/(Fs/2));
    arma::rowvec Fd_filt = arma::conv(Fd, filter_b.t(), "same");

    double T1 = 1.0/(2*Fmax);
    arma::rowvec T_sync(std::round(tau/T1));
    double t = 0.0;
    for (int i=0;i<T_sync.size();i++)
    {
      T_sync(i) = t;
      t += T1;
    }

    arma::rowvec w_sync;
    arma::interp1(Time, Fd_filt, T_sync, w_sync);
    this->N_kernels = T_sync.size();
    this->c.resize(this->N_kernels);
    this->h.resize(this->N_kernels);
    this->w.resize(this->N_kernels);

    for (int i=0;i<this->N_kernels;i++)
    {
      this->c(i) = this->phase(T_sync(i));
      this->h(i) = T1/tau;
      this->w(i) = w_sync(i);
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

  double DMP_Shannon::calcFd(double x, double y, double dy, double ddy, double y0, double g) const
  {
    double v_scale = this->get_v_scale();
    double s = (this->forcingTermScaling(y0, g) + this->zero_tol) * this->shapeAttrGating(x);
    double Fd = (ddy*std::pow(v_scale,2) - this->goalAttractor(x, y, v_scale*dy, g)) / s;
    return Fd;
  }

  double DMP_Shannon::forcingTerm(double x) const
  {
    arma::vec Psi = this->kernelFunction(x);

    double f = arma::dot(Psi,this->w);
    return f;
  }

  arma::vec DMP_Shannon::kernelFunction(double x) const
  {
    arma::vec t = arma::datum::pi*(x-this->c)/this->h;
    arma::vec psi = arma::sin(t)/(t+1e-250);
    return psi;
  }

  double DMP_Shannon::learnedForcingTerm(double x, double y0, double g) const
  {
    double learnForcTerm = this->forcingTerm(x);
    return learnForcTerm;
  }

  void DMP_Shannon::parseExtraArgs(const param_::ParamList *paramListPtr)
  {
    this->kernelStdScaling = 1.0;
    this->Wmin = 0.95;
    this->Freq_min = 10;
    this->Freq_max = 200;
    this->P1_min = 0.5;

    if (paramListPtr)
    {
      paramListPtr->getParam("kernelStdScaling",this->kernelStdScaling);
      paramListPtr->getParam("Wmin",this->Wmin);
      paramListPtr->getParam("Freq_min",this->Freq_min);
      paramListPtr->getParam("Freq_max",this->Freq_max);
      paramListPtr->getParam("P1_min",this->P1_min);
    }
  }

} // namespace as64_
