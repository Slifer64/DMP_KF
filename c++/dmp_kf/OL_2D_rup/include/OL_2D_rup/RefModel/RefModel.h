#ifndef REF_MODEL_OL_2D_RUP_H
#define REF_MODEL_OL_2D_RUP_H

#include <cstdlib>
#include <vector>
#include <cstring>
#include <memory>
#include <armadillo>

#include <OL_2D_rup/utils.h>

class RefModel
{
public:
  /** Constructor. Calls 'init()'. */
  RefModel() {}

  /** Destructor. */
  ~RefModel() {}

  /** Initializes the model.
   *  @param[in] S0 Initial state.
   */
  virtual void init(const arma::vec &S0) = 0;

  /** Updates the model's params and variables.
   *  @param[in] dt The timestep used for numerical integration.
   *  @param[in] model_error The error based on which the model is updated.
   */
  virtual void update(double dt, const arma::vec &model_error) = 0;

  /** Calculates the model's output (based on the current value of the model's variables-state). */
  virtual void calcOutput(arma::vec &S_ref, arma::vec &dS_ref, arma::vec &ddS_ref) = 0;


  // /**  Reads arguments from yaml file and initializes the model parameters. */
  // virtual void initParams() = 0;
  //
  // /** Initializes the model variables (before execution of the model).
  //  *  @param[in] S0 Initial model state.
  //  */
  // virtual void initVars(const arma::vec &S0) = 0;
  //
  // /** Updates the model's parameters based on an error metric.
  //  *  @param[in] model_error The error based on which the model is updated.
  //  */
  // virtual void updateParams(const arma::vec &model_error) = 0;
  //
  // /** Updates the model variables by numerical integration using the output from 'calcModelOutput()'.
  //  *  @param[in] dt The timestep for numerical integration.
  //  */
  // virtual void updateVars(double dt) = 0;

  // ========== Model variables ============
  arma::vec S; ///< model's state
  arma::vec dS; ///< model's state velocity
  arma::vec ddS; ///< model's state acceleration

  double x; ///< model's timing variable
  double dx; ///< model's timing variable derivative

  arma::vec dS_c; ///< optional coupling term for model's velocity
  arma::vec ddS_c; ///< optional coupling term for model's acceleration

protected:
  /** Reads the model's arguments from a file.
   *  @param[in] params_file The file from which the model's arguments are read.
   */
  virtual void readParams(const char *params_file = NULL) = 0;
};

#endif // REF_MODEL_OL_2D_RUP_H
