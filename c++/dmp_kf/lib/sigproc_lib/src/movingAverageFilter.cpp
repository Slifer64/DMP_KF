#include <sigproc_lib/movingAverageFilter.h>

namespace as64_
{

namespace spl_
{

MovingAverageFilter::MovingAverageFilter()
{
  //init(10);
}

MovingAverageFilter::MovingAverageFilter(int n_samples, double init_value, double a)
{
  init(n_samples, init_value, a);
}

void MovingAverageFilter::init(int n_samples, double init_value, double a)
{
  setNumOfSamples(n_samples);
  setExpWeight(a);

  q_values = std::queue<double>();

  sum_val = 0.0;
  w_s = 0.0;
  for (int i=0;i<getNumOfSamples();i++)
  {
    q_values.push(init_value);
    double w = std::exp(-a*(N-1-i));
    sum_val += w*init_value;
    w_s += w;
  }
}

void MovingAverageFilter::setNumOfSamples(int n_samples)
{
  if (n_samples < 1)
  {
    std::ostringstream out_str;
    out_str << "MovingAverageFilter::setNumOfSamples: Invalid number of samples: " << n_samples << "\n";
    throw std::invalid_argument(out_str.str());
  }
  N = n_samples;
}

double MovingAverageFilter::getNumOfSamples() const
{
  return N;
}

void MovingAverageFilter::setExpWeight(double a)
{
  if (a<0.0)
  {
    std::ostringstream out_str;
    out_str << "MovingAverageFilter::setExpWeight: Invalid forgetting: " << a << "\n";
    throw std::invalid_argument(out_str.str());
  }

  this->a = a;
}

double MovingAverageFilter::getExpWeight() const
{
  return a;
}

double MovingAverageFilter::filter(double value)
{
  double front_value = q_values.front();

  // std::cout << "w_s = " << w_s << "\n";
  // std::cout << "sum_val = " << sum_val << "\n";
  // std::cout << "front_value*std::exp(-a*(N-1)) = " << front_value*std::exp(-a*(N-1)) << "\n";


  q_values.pop();
  sum_val = (sum_val - front_value*std::exp(-a*(N-1)))*std::exp(-a) + value;
  double filt_value = sum_val/w_s;
  q_values.push(filt_value);

  // std::cout << "w_s = " << w_s << "\n";
  // std::cout << "sum_val = " << sum_val << "\n";

  return filt_value;
}

} // namespace spl_

} // namespace as64_
