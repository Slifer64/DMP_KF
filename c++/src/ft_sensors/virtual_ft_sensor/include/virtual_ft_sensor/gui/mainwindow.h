#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QCheckBox>
#include <slider.h>
#include <cstring>
#include <vector>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  explicit MainWindow(const std::vector<double> &F_max, std::vector<double> &F, double scale, QWidget *parent=0);
  ~MainWindow();

  // returns the i-th slider's position
  double getSliderPosition(int i) const;

  // sets the i-th slider's position
  void setSliderPosition(double val, int i);

private slots:
    void on_auto_reset_checkbox_clicked(bool val);

private:
  Ui::MainWindow *ui;

  int N_sliders;
  int bar_width;
  int bar_height;
  int bar_offset;
  std::vector<std::shared_ptr<Slider>> sliders;
  std::shared_ptr<QCheckBox> auto_reset_checkbox;

  std::string wind_name;
  std::vector<std::string> trackbar_name;
};

#endif // MAINWINDOW_H
