#ifndef SIGNAL_PROCESSING_LIB_MOVING_WINDOW_MEAN_FILTER_64_H
#define SIGNAL_PROCESSING_LIB_MOVING_WINDOW_MEAN_FILTER_64_H

#include <queue>

namespace as64_
{

namespace spl_
{

class MovingWinMeanFilter
{
public:
    void init(int filt_win, double F_init);
    void set_filter_window(int filt_win);
    double get_filter_window() const;
    void update(double F);
    double get_filtered_output() const;
private:
    int w_n_max;
    int w_n;
    double sum_F;
    double F_filt;
    std::queue<double> F_queue;
};

} // namespace spl_

} // namespace as64_

#endif // SIGNAL_PROCESSING_LIB_MOVING_WINDOW_MEAN_FILTER_64_H
