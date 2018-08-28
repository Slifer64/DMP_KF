#ifndef SIGNAL_PROCESSING_LIB_MOVING_AVERAGE_FILTER_64_H
#define SIGNAL_PROCESSING_LIB_MOVING_AVERAGE_FILTER_64_H

#include <queue>
#include <vector>
#include <cmath>
#include <iostream>
#include <exception>
#include <stdexcept>
#include <sstream>

namespace as64_
{

namespace spl_
{

/**
 * ==> Description:
 * Implements a moving average filter of the form:
 * x_{filt}(N) = ( sum_{i=1}^{N-1}(x_{filt}(i)*exp(-a*(N-i))) + x(N) ) / (sum_{i=1}^{N}*exp(-a*(N-i)))
 * Parameter 'a' is a weighting factor that diminishes the importance of older values.
 * It must be non-negative.
 * If 'a'=0 then the output is the classical moving average filter.
 *
 * ==> Usage:
 * Call 'init' to initialize the filter.
 * Call 'filter' to get the filtered output of the input value.
 */
class MovingAverageFilter
{
public:
    /** Empty Constructor.
     */
    MovingAverageFilter();

    /** Constructor.
     *  \see init
     */
    MovingAverageFilter(int n_samples, double init_value = 0.0, double a = 0.0);

    /** Initializes the moving average filter.
     *  @param[in] n_samples The number of samples used in the moving average window.
     *  @param[in] init_value The initial value used for padding the filter in initialization (optional, default = 0.0).
     *  @param[in] a The exponential weighting rate (optional, default = 0.0).
     */
    void init(int n_samples, double init_value = 0.0, double a = 0.0);

    /** Filters the input value and outputs the filtered value.
     *  @param[in] value The value to be filtered.
     *  @return The filtered value.
     */
    double filter(double value);

    /** Sets the number of samples used in the moving average window.
     *  @param[in] n_samples The number of samples.
     */
    void setNumOfSamples(int n_samples);

    /** Returns the number of samples used in the moving average window.
     *  @return The number of samples.
     */
    double getNumOfSamples() const;

    /** Sets the exponential weighting rate. The i-th sample will be weighted by exp(-a*(N-i))
     *  @param[in] a The exponential weighting rate.
     */
    void setExpWeight(double a);

    /** Returns the exponential weighting rate.
     *  @return The exponential weighting rate.
     */
    double getExpWeight() const;

private:
    int N; ///< number of samples
    double sum_val; ///< the current sum of the filter samples
    std::queue<double> q_values; ///< queue with samples (the oldest at the front and the newest sample at the end)
    double a; ///< exponential weighting rate
    double w_s; ///< sum of sample weights
};

} // namespace spl_

} // namespace as64_

#endif // SIGNAL_PROCESSING_LIB_MOVING_AVERAGE_FILTER_64_H
