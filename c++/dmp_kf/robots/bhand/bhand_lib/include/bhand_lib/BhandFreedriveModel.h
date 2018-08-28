#ifndef BHAND_FREEDIRVE_MODEL_H
#define BHAND_FREEDIRVE_MODEL_H

#include <memory>
#include <armadillo>

class BarrettHand; // forward declaration

class BhandFreedriveModel
{

public:
  BhandFreedriveModel();
  ~BhandFreedriveModel();

  void init();
  void run();

private:

  arma::vec M;
  arma::vec D;

  arma::vec q_ref;
  arma::vec dq_ref;
  arma::vec ddq_ref;

};

#endif // BHAND_FREEDIRVE_MODEL_H
