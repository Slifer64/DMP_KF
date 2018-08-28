
#ifndef PLOT_LIBRARY_64_H
#define PLOT_LIBRARY_64_H

#include <cstdarg>
#include <plot_lib/gnuplot_i.hpp>

namespace as64_
{

namespace plot_
{

class GraphObj: public Gnuplot
{
public:

  /** Constructor
   */
  GraphObj()
  {
    hold_on_enable = false;
    setLineStyle("lines");
  }


  /** brief 2D plot.
   *  Plots a vector of data. The y-axis contains the data.
   *  The x-axis contains the indices of the data.
   *  The vector of datapoints can be any type of object that supports
   *  the "size()" function for returning the size of the vector and the
   *  "[]" operator for accessing the vector's elements.
   *  @param[in] x: Vector of data points.
   */
  template<typename X>
  void plot(const X& x, const std::string &legend_ = "")
  {
    if (!hold_on_enable) nplots = 0;
    plot_x(x, legend_);
  }

  /** brief 2D plot.
   *  The vector of datapoints can be any type of object that supports
   *  the "size()" function for returning the size of the vector and the
   *  "[]" operator for accessing the vector's elements.
   *  @param[in] x: Vector of data points in the x-axis.
   *  @param[in] y: Vector of data points in the y-axis.
   */
  template<typename X, typename Y>
  void plot(const X& x, const Y& y, const std::string &legend_ = "")
  {
    if (!hold_on_enable) nplots = 0;
    plot_xy(x, y, legend_);
  }

  /** brief 3D plot.
   *  The vector of datapoints can be any type of object that supports
   *  the "size()" function for returning the size of the vector and the
   *  "[]" operator for accessing the vector's elements.
   *  @param[in] x: Vector of data points in the x-axis.
   *  @param[in] y: Vector of data points in the y-axis.
   *  @param[in] z: Vector of data points in the z-axis.
   */
  template<typename X, typename Y, typename Z>
  void plot3(const X& x, const Y& y, const Z& z, const std::string &legend_ = "")
  {
    if (!hold_on_enable) nplots = 0;
    plot_xyz(x, y, z, legend_);
  }

  /** brief Enables "hold on" so that the next figures that will
   *  be plotted won't erase the previous ones.
   */
  void hold_on()
  {
    hold_on_enable = true;
  }

  /** brief Disables "hold on" so that the next figure that will
   *  be plotted will erase the previous ones.
   */
  void hold_off()
  {
    hold_on_enable = false;
  }


  /// resets a gnuplot session (next plot will erase previous ones)
  //Gnuplot& reset_plot();

  /// resets a gnuplot session and sets all variables to default
  //Gnuplot& reset_all();

  /** brief Sets the figure's title.
   *  @param[in] title_: The title of the figure.
   */
  void title(const std::string &title_)
  {
    set_title(title_);
    replot();
  }

  /** brief Sets the figure's x-axis label.
   *  @param[in] label: The label.
   */
  void xlabel(const std::string &label)
  {
    set_xlabel(label);
    replot();
  }

  /** brief Sets the figure's y-axis label.
   *  @param[in] label: The label.
   */
  void ylabel(const std::string &label)
  {
    set_ylabel(label);
    replot();
  }

  /** brief Sets the figure's z-axis label.
   *  @param[in] label: The label.
   */
  void zlabel(const std::string &label)
  {
    set_zlabel(label);
    replot();
  }

  void legend(int n, ...)
  {/*
    std::vector<std::string> legend_str(n);
    std::va_list vl;
    va_start(vl,n);

    for (int i=0; i<n; i++)
    {
      legend_str[i] = va_arg(vl,char*);

      std::cout << "legend_str[i] = " << legend_str[i] << "\n";
    }

    if (n > tmpfile_list.size())
    {
      std::cerr << "Warning: Ignoring extra legend arguments...\n";
      n = tmpfile_list.size();
    }

    for (int i=0; i<n; i++)
    {
      std::string filename = tmpfile_list[i];
      //
      // check if file exists
      //
      file_available(filename);

      std::ostringstream cmdstr;
      //
      // command to be sent to gnuplot
      //
      if (nplots > 0  &&  two_dim == true)
          cmdstr << "replot ";
      else
          cmdstr << "plot ";

      cmdstr << "\"" << filename << "\" using " << column;

      if (title == "")
          cmdstr << " notitle ";
      else
          cmdstr << " title \"" << title << "\" ";

      if(smooth == "")
          cmdstr << "with " << pstyle;
      else
          cmdstr << "smooth " << smooth;

      //
      // Do the actual plot
      //
      cmd(cmdstr.str()); //nplots++; two_dim = true;  already in cmd();
    }

    va_end(vl);
*/
  }

  /** brief Sets the x-axis limits.
   *  @param[in] iFrom: Start value of the axis.
   *  @param[in] iTo: End value of the axis.
   */
  void xlim(const double iFrom, const double iTo)
  {
    set_xrange(iFrom, iTo);
    replot();
  }

  /** brief Sets the y-axis limits.
   *  @param[in] iFrom: Start value of the axis.
   *  @param[in] iTo: End value of the axis.
   */
  void ylim(const double iFrom, const double iTo)
  {
    set_yrange(iFrom, iTo);
    replot();
  }

  /** brief Sets the z-axis limits.
   *  @param[in] iFrom: Start value of the axis.
   *  @param[in] iTo: End value of the axis.
   */
  void zlim(const double iFrom, const double iTo)
  {
    set_zrange(iFrom, iTo);
    replot();
  }

  /** brief Set line style. Available line styles:
   *  lines, points, linespoints, impulses, dots, steps, fsteps, histeps,
   *  boxes, histograms, filledcurves.
   *  @param[in] lineStyle: The line style.
   */
  void setLineStyle(const std::string &lineStyle)
  {
    set_style(lineStyle);
  }

  /** brief Scales the size of the points used in plots.
   *  @param[in] pointsize: The marker width.
   */
  void setMarkerWidth(const double pointsize)
  {
    set_pointsize(pointsize);
  }

  /** brief Turns grid on.
   */
  void grid_on()
  {
    set_grid();
    replot();
  }

  /** brief Turns grid off.
   */
  void grid_off()
  {
    unset_grid();
    replot();
  }

  /** brief Autoscales the figure's axes.
   */
  void autoscale()
  {
    set_xautoscale();
    set_yautoscale();
    set_zautoscale();
    replot();
  }

  /** brief Sets the position of the legend. Available positions are:
   * position: inside/outside, left/center/right, top/center/bottom, nobox/box.
   *  @param[in] position: The position of the legend.
   */
  void setLegendPos(const std::string &position)
  {
    set_legend(position);
    replot();
  }

  /** brief Removes all plots, titles, legends, labels etc from this figure.
   *  Set all parameters to default values.
   */
  void clear()
  {
    nplots = 0;
    cmd("reset");
    cmd("clear");
    pstyle = "lines";
    smooth = "";
    showonscreen();
  }

private:
  bool hold_on_enable;
};


} // namespace plot_

} // namespace as64_

#endif // PLOT_LIBRARY_64_H
