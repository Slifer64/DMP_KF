#include <sigproc_lib/movingWinMeanFilter.h>

namespace as64_
{

namespace spl_
{

void MovingWinMeanFilter::init(int filt_win, double F_init)
{
  set_filter_window(filt_win);
  sum_F = F_init;
  w_n = 1;
  F_filt = sum_F/w_n;
  F_queue.push(F_filt);
}

void MovingWinMeanFilter::set_filter_window(int filt_win)
{
  w_n_max = filt_win;
}

double MovingWinMeanFilter::get_filter_window() const
{
  return w_n_max;
}

void MovingWinMeanFilter::update(double F)
{
  if (w_n >= w_n_max){
    double F_front = F_queue.front();
    sum_F -= F_front;
    w_n--;
  }
  sum_F += F;
  w_n++;
  F_filt = sum_F/w_n;
  F_queue.pop();
  F_queue.push(F_filt);
}

double MovingWinMeanFilter::get_filtered_output() const
{
  return F_filt;
}

} // namespace spl_

} // namespace as64_
