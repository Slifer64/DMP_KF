#include <dmp_lib/DMP/DMP_orient.h>
#include <math_lib/math_lib.h>

namespace as64_
{
  /** brief DMP empty constructor.
   */
  DMP_orient::DMP_orient():D(3)
  {
    dmp.resize(D);
  }


  DMP_orient::DMP_orient(std::vector<std::shared_ptr<DMP_>> &vec3D_dmp):D(3)
  {
    this->init(vec3D_dmp);
  }


  void DMP_orient::init(std::vector<std::shared_ptr<DMP_>> &vec3D_dmp)
  {
    this->dmp.resize(this->D);
    for (int i=0; i<this->D; i++)
    {
      this->dmp[i] = vec3D_dmp[i];
    }
  }


  void DMP_orient::setCenters()
  {
    for (int i=0; i<this->D; i++)
    {
      this->dmp[i]->setCenters();
    }
  }


  void DMP_orient::setStds(double kernelStdScaling)
  {
    for (int i=0; i<this->D; i++)
    {
      this->dmp[i]->setStds(kernelStdScaling);
    }
  }


  arma::vec DMP_orient::train(const arma::rowvec &Time, const arma::mat &Q_data,
                  const arma::mat &v_rot_data, const arma::mat &dv_rot_data,
                  const arma::vec &Q0, const arma::vec &Qg,
                  const std::string &train_method, bool ret_train_err)
  {
    using namespace as64_::math_;

    int n_data = Q_data.n_cols;
    arma::mat yd_data(3, n_data);
    for (int i=0; i<n_data; i++)
    {
      yd_data.col(i) = -quatLog(quatProd(Qg,quatInv(Q_data.col(i))));
    }

    arma::vec y0 = -quatLog(quatProd(Qg,quatInv(Q0)));
    arma::vec g = arma::vec(3).zeros();

    arma::vec train_error(this->D);
    for (int i=0; i<this->D; i++)
    {
      train_error(i) = this->dmp[i]->train(Time, yd_data.row(i), v_rot_data.row(i), dv_rot_data.row(i),
                                          y0(i), g(i), train_method, ret_train_err);
    }
    return train_error;
  }


  void DMP_orient::setTrainingParams(const param_::ParamList *paramListPtr)
  {
    for (int i=0; i<this->D; i++){
      this->dmp[i]->setTrainingParams(paramListPtr);
    }
  }


  arma::vec DMP_orient::learnedForcingTerm(const arma::vec &X, const arma::vec &Q0, const arma::vec &Qg) const
  {
    using namespace as64_::math_;

    arma::vec learnForcTerm(this->D);

    arma::vec y0 = -quatLog(quatProd(Qg,quatInv(Q0)));
    arma::vec g = arma::vec(3).zeros();

    for (int i=0; i<this->D; i++){
      learnForcTerm(i) = this->dmp[i]->learnedForcingTerm(X(i), y0(i), g(i));
    }
    return learnForcTerm;
  }


  arma::vec DMP_orient::calcFd(const arma::vec &X, const arma::vec &Q, const arma::vec &v_rot,
                  const arma::vec &dv_rot, const arma::vec &Q0, const arma::vec &Qg) const
  {
    using namespace as64_::math_;

    arma::vec y = -quatLog(quatProd(Qg,quatInv(Q)));
    arma::vec y0 = -quatLog(quatProd(Qg,quatInv(Q0)));
    arma::vec g = arma::vec(3).zeros();

    arma::vec Fd(this->D);
    for (int i=0; i<this->D; i++){
      Fd(i) = this->dmp[i]->calcFd(X(i), y(i), v_rot(i), dv_rot(i), y0(i), g(i));
    }
    return Fd;
  }


  arma::vec DMP_orient::forcingTerm(const arma::vec &X) const
  {
    arma::vec f(this->D);
    for (int i=0; i<this->D; i++){
      f(i) = this->dmp[i]->forcingTerm(X(i));
    }
    return f;
  }


  arma::vec DMP_orient::forcingTermScaling(const arma::vec &Q0, const arma::vec &Qg) const
  {
    using namespace as64_::math_;

    arma::vec y0 = -quatLog(quatProd(Qg,quatInv(Q0)));
    arma::vec g = arma::vec(3).zeros();

    arma::vec f_scale(this->D);
    for (int i=0; i<this->D; i++){
      f_scale(i) = this->dmp[i]->forcingTermScaling(y0(i), g(i));
    }
    return f_scale;
  }


  arma::vec DMP_orient::goalAttractor(const arma::vec &X, const arma::vec &Q, const arma::vec &eta, const arma::vec &Qg) const
  {
    using namespace as64_::math_;

    arma::vec y = -quatLog(quatProd(Qg,quatInv(Q)));
    arma::vec g = arma::vec(3).zeros();

    arma::vec goal_attr(this->D);
    for (int i=0; i<this->D; i++){
      goal_attr(i) = this->dmp[i]->goalAttractor(X(i), y(i), eta(i), g(i));
    }
    return goal_attr;
  }


  arma::vec DMP_orient::shapeAttractor(const arma::vec &X, const arma::vec &Q0, const arma::vec &Qg) const
  {
    using namespace as64_::math_;

    arma::vec y0 = -quatLog(quatProd(Qg,quatInv(Q0)));
    arma::vec g = arma::vec(3).zeros();

    arma::vec shape_attr(this->D);
    for (int i=0; i<this->D; i++){
      shape_attr(i) = this->dmp[i]->shapeAttractor(X(i), y0(i), g(i));
    }
    return shape_attr;
  }


  std::vector<arma::vec> DMP_orient::getStatesDot(const arma::vec &X, const arma::vec &Q, const arma::vec &eta,
                              const arma::vec &Q0, const arma::vec &Qg,
                              const arma::vec &Q_c, const arma::vec &eta_c) const
  {
    std::vector<arma::vec> statesDot(this->D);

    arma::vec v_scale = this->get_v_scale();
    arma::vec shape_attr = this->shapeAttractor(X, Q0, Qg);
    arma::vec goal_attr = this->goalAttractor(X, Q, eta, Qg);

    arma::vec deta = ( goal_attr + shape_attr + eta_c) / v_scale;

    arma::vec temp(4);
    temp(0) = 0.0;
    temp.subvec(1,3) = (eta+Q_c)/v_scale;
    arma::vec dQ = 0.5*as64_::math_::quatProd(temp, Q);

    arma::vec dX = this->phaseDot(X);

    statesDot[0] = deta;
    statesDot[1] = dQ;
    statesDot[2] = dX;

    return statesDot;
  }


  std::vector<arma::vec> DMP_orient::kernelFunction(const arma::vec &X) const
  {
    std::vector<arma::vec> Psi(this->D);
    for (int i=0; i<this->D; i++){
      Psi[i] = this->dmp[i]->kernelFunction(X(i));
    }
    return Psi;
  }


  arma::vec DMP_orient::get_v_scale() const
  {
    arma::vec v_scale(this->D);
    for (int i=0; i<this->D; i++){
      v_scale(i) = this->dmp[i]->get_v_scale();
    }
    return v_scale;
  }


  arma::vec DMP_orient::getTau() const
  {
    arma::vec tau(this->D);
    for (int i=0; i<this->D; i++){
      tau(i) = this->dmp[i]->getTau();
    }
    return tau;
  }

  arma::vec DMP_orient::phase(double t) const
  {
    arma::vec X(this->D);
    for (int i=0; i<this->D; i++){
      X(i) = this->dmp[i]->phase(t);
    }
    return X;
  }

  arma::vec DMP_orient::phaseDot(const arma::vec &X) const
  {
    arma::vec dX(this->D);
    for (int i=0; i<this->D; i++){
      dX(i) = this->dmp[i]->phaseDot(X(i));
    }
    return dX;
  }

} // namespace as64_
