#ifndef SIGNAL_PROCESSING_LIBRARY_64_H
#define SIGNAL_PROCESSING_LIBRARY_64_H

#include <cstdlib>
#include <vector>
#include <string>
#include <cmath>
#include <sstream>

#include <armadillo>

namespace as64_
{

namespace spl_
{


  /** \brief function Make signals to have the same length
   *  Makes the signals the same length by recomputing their values at specific timestamps
   *  using linear interpolation.
   *  @param[in] Time1: 1 x N vector with the timestamps of the first signal.
   *  @param[in] y1: 1 x N vector with the values of the first signal.
   *  @param[in] Time2: 1 x N vector with the timestamps of the second signal.
   *  @param[in] y2: 1 x N vector with the values of the second signal.
   *  @param[in] Timeq: 1 x N vector with the timestamps of the returned signals.
   *  @param[out] z1: 1 x N vector with the values of the first signal computed at timestamps 'Timeq'.
   *  @param[out] z2: 1 x N vector with the values of the second signal computed at timestamps 'Timeq'.
   */
  void makeSignalsEqualLength(const arma::rowvec &Time1, const arma::rowvec &y1,
          const arma::rowvec &Time2, const arma::rowvec &y2, const arma::rowvec &Timeq,
          arma::rowvec &z1, arma::rowvec &z2);


  /** \brief Calculates the single sided Fourier transform of a signal
   *  @param[in] s: The input signal (vector 1x N).
   *  @param[in] Fs: The sampling frequency of the 's'.
   *  @param[out] f: The frequncies where the fourier is calculated.
   *  @param[out] P1: The amplitudes of the signal's single sided Fourier.
   */
  void getSingleSidedFourier(const arma::rowvec &s, double Fs, arma::rowvec &f, arma::rowvec &P1);



} // namespace spl_

} // namespace as64_

#endif // SIGNAL_PROCESSING_LIBRARY_64_H
