/** Least Squares based learning techniques
 */

#include <dmp_lib/trainMethods/LeastSquares.h>

namespace as64_
{

arma::mat leastSquares(const arma::mat &Psi, const arma::mat &X, const arma::rowvec &Fd, double zero_tol)
{
  int N_kernels = Psi.n_rows;
  int w_dim = X.n_rows;
  int n_data = Psi.n_cols;

  // normalize Psi
  arma::mat Psi_norm = Psi;
  Psi_norm = Psi / (arma::repmat(arma::sum(Psi,0), Psi.n_rows,1) + zero_tol);

  arma::mat H(N_kernels*w_dim, n_data);
  int k = 0;
  for (int i=0; i<w_dim; i++)
  {
      H.rows(k,k+N_kernels-1) = Psi_norm % arma::repmat(X.row(i),N_kernels,1);
      k += N_kernels;
  }

  arma::mat w = arma::reshape((Fd*arma::pinv(H)).t(), N_kernels, w_dim);

  return w;
}


} // namespace as64_
