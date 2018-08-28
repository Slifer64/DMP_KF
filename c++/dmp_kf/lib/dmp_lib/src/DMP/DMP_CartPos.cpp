#include <dmp_lib/DMP/DMP_CartPos.h>

namespace as64_
{
  /** brief DMP empty constructor.
   */
  DMP_CartPos::DMP_CartPos():D(3)
  {
    dmp.resize(D);
  }


  DMP_CartPos::DMP_CartPos(std::vector<std::shared_ptr<DMP_>> &vec3D_dmp):D(3)
  {
    this->init(vec3D_dmp);
  }


  void DMP_CartPos::init(std::vector<std::shared_ptr<DMP_>> &vec3D_dmp)
  {
    this->dmp.resize(this->D);
    for (int i=0; i<this->D; i++)
    {
      this->dmp[i] = vec3D_dmp[i];
    }
  }


  void DMP_CartPos::setCenters()
  {
    for (int i=0; i<this->D; i++)
    {
      this->dmp[i]->setCenters();
    }
  }


  void DMP_CartPos::setStds(double kernelStdScaling)
  {
    for (int i=0; i<this->D; i++)
    {
      this->dmp[i]->setStds(kernelStdScaling);
    }
  }


  arma::vec DMP_CartPos::train(const arma::rowvec &Time, const arma::mat &Y_data,
                  const arma::mat &dY_data, const arma::mat &ddY_data,
                  const arma::vec &Y0, const arma::vec &Yg,
                  const std::string &train_method, bool ret_train_err)
  {
    arma::vec train_error(this->D);
    for (int i=0; i<this->D; i++)
    {
      train_error(i) = this->dmp[i]->train(Time, Y_data.row(i), dY_data.row(i), ddY_data.row(i),
                                            Y0(i), Yg(i), train_method, ret_train_err);
    }
    return train_error;
  }


  void DMP_CartPos::setTrainingParams(const param_::ParamList *paramListPtr)
  {
    for (int i=0; i<this->D; i++){
      this->dmp[i]->setTrainingParams(paramListPtr);
    }
  }


  arma::vec DMP_CartPos::learnedForcingTerm(const arma::vec &X, const arma::vec &y0, const arma::vec &g) const
  {
    arma::vec learnForcTerm(this->D);
    for (int i=0; i<this->D; i++){
      learnForcTerm(i) = this->dmp[i]->learnedForcingTerm(X(i), y0(i), g(i));
    }
    return learnForcTerm;
  }


  arma::vec DMP_CartPos::calcFd(const arma::vec &X, const arma::vec &Y, const arma::vec &dY,
                  const arma::vec &ddY, const arma::vec &Y0, const arma::vec &Yg) const
  {
    arma::vec Fd(this->D);
    for (int i=0; i<this->D; i++){
      Fd(i) = this->dmp[i]->calcFd(X(i), Y(i), dY(i), ddY(i), Y0(i), Yg(i));
    }
    return Fd;
  }


  arma::vec DMP_CartPos::forcingTerm(const arma::vec &X) const
  {
    arma::vec f(this->D);
    for (int i=0; i<this->D; i++){
      f(i) = this->dmp[i]->forcingTerm(X(i));
    }
    return f;
  }


  arma::vec DMP_CartPos::forcingTermScaling(const arma::vec &Y0, const arma::vec &Yg) const
  {
    arma::vec f_scale(this->D);
    for (int i=0; i<this->D; i++){
      f_scale(i) = this->dmp[i]->forcingTermScaling(Y0(i), Yg(i));
    }
    return f_scale;
  }


  arma::vec DMP_CartPos::goalAttractor(const arma::vec &X, const arma::vec &Y, const arma::vec &Z, const arma::vec &Yg) const
  {
    arma::vec goal_attr(this->D);
    for (int i=0; i<this->D; i++){
      goal_attr(i) = this->dmp[i]->goalAttractor(X(i), Y(i), Z(i), Yg(i));
    }
    return goal_attr;
  }


  arma::vec DMP_CartPos::shapeAttractor(const arma::vec &X, const arma::vec &Y0, const arma::vec &Yg) const
  {
      arma::vec shape_attr(this->D);
      for (int i=0; i<this->D; i++){
        shape_attr(i) = this->dmp[i]->shapeAttractor(X(i), Y0(i), Yg(i));
      }
      return shape_attr;
  }


  arma::mat DMP_CartPos::getStatesDot(const arma::vec &X, const arma::vec &Y, const arma::vec &Z,
                              const arma::vec &Y0, const arma::vec &Yg,
                              const arma::vec &Y_c, const arma::vec &Z_c) const
  {
    arma::mat statesDot(3, this->D);

    for (int i=0; i<this->D; i++){
      statesDot.col(i) = this->dmp[i]->getStatesDot(X(i), Y(i), Z(i), Y0(i), Yg(i), Y_c(i), Z_c(i));
    }
    return statesDot.t();
  }


  std::vector<arma::vec> DMP_CartPos::kernelFunction(const arma::vec &X) const
  {
    std::vector<arma::vec> Psi(this->D);
    for (int i=0; i<this->D; i++){
      Psi[i] = this->dmp[i]->kernelFunction(X(i));
    }
  }


  arma::vec DMP_CartPos::get_v_scale() const
  {
    arma::vec v_scale(this->D);
    for (int i=0; i<this->D; i++){
      v_scale(i) = this->dmp[i]->get_v_scale();
    }
    return v_scale;
  }


  arma::vec DMP_CartPos::getTau() const
  {
    arma::vec tau(this->D);
    for (int i=0; i<this->D; i++){
      tau(i) = this->dmp[i]->getTau();
    }
    return tau;
  }

  arma::vec DMP_CartPos::phase(double t) const
  {
    arma::vec X(this->D);
    for (int i=0; i<this->D; i++){
      X(i) = this->dmp[i]->phase(t);
    }
    return X;
  }

} // namespace as64_
