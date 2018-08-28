#include <sigproc_lib/sigproc.h>

namespace as64_
{

namespace spl_
{

  void makeSignalsEqualLength(const arma::rowvec &Time1, const arma::rowvec &y1,
          const arma::rowvec &Time2, const arma::rowvec &y2, const arma::rowvec &Timeq,
          arma::rowvec &z1, arma::rowvec &z2)
  {
    arma::interp1(Time1, y1, Timeq, z1, "linear", y1(y1.size()-1));
    arma::interp1(Time2, y2, Timeq, z2, "linear", y2(y2.size()-1));
  }

	void getSingleSidedFourier(const arma::rowvec &s, double Fs, arma::rowvec &f, arma::rowvec &P1)
	{
		int L = s.size();
		arma::cx_rowvec Y = arma::fft(s);
		arma::rowvec P2 = arma::abs(Y/L);

		int n = std::floor(L/2);
		P1 = 2*P2.cols(0,n);
		P1(0) = P1(0)/2;

		f = arma::linspace<arma::rowvec>(0,n+1,n)*(Fs/L);
		P1 = P1/arma::max(P1);
	}

} // namespace spl_

} // namespace as64_
