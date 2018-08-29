/** Locally Weighted Learning techniques
 */

#ifndef DMP_LIB_LOCALLY_WEIGHTED_REGRESSION_H
#define DMP_LIB_LOCALLY_WEIGHTED_REGRESSION_H

#include <cstdlib>
#include <vector>
#include <string>
#include <exception>
#include <cmath>
#include <armadillo>

namespace as64_
{

/** \brief Locally Weighted Regression
 * Performs locally weighted regression learning on the input data and returns the learned weights.
 * N denotes the number of data points.
 * M denotes the dimensionality of the input data.
 * K denotes the number of kernels.
 * The kernels are allocated in the data point space and provide as output the activation value of the
 * kernel for a specific data point.
 * The data point can be anything, e.g. a 1-D data point representing time (or a substitute variable for time),
 * or a 3-D data point representing Cartesian position, or Cartesian force, or it can be any other M-D datapoint
 * representing anything you like.
 * @param[in] Psi: K x N matrix, where the k-th row contains the k-th kernel function values for all data points.
 * @param[in] X: M x N matrix, where the j-th column corresponds to the j-th data point.
 * @param[in] Fd: 1 x N row vector, where the j-th value corresponds to the desired output for the j-th data point.
 * @param[in] zero_tol: Tollerance value to avoid divisions by zero.
 * @return K x N matrix with the learned weights.
 */
arma::mat LWR(const arma::mat &Psi, const arma::mat &X, const arma::rowvec &Fd, double zero_tol=0.0);


} // namespace as64_

#endif // DMP_LIB_LOCALLY_WEIGHTED_REGRESSION_H
