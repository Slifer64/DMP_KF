#include "mainwindow.h"
#include "ui_mainwindow.h"


MainWindow::MainWindow(const std::vector<double> &F_max, std::vector<double> &F, double scale, QWidget *parent) :
    QMainWindow(parent), ui(new Ui::MainWindow)
{

  ui->setupUi(this);

  N_sliders = 6;

  wind_name = "F/T sensor";
  trackbar_name.resize(N_sliders);
  trackbar_name[0] = "fx";
  trackbar_name[1] = "fy";
  trackbar_name[2] = "fz";
  trackbar_name[3] = "tx";
  trackbar_name[4] = "ty";
  trackbar_name[5] = "tz";

  // std::vector<double> F_max = {30, 30, 30, 60, 60, 60};
  // scale = 0.01;
  bar_width = 350;
  bar_height = 25;
  bar_offset = bar_height + 20;
  int x_offset = 20;
  int y_offset = 0;

  int x_checkbox = 20;
  int y_checkbox = 10;
  auto_reset_checkbox.reset(new QCheckBox(this));
  auto_reset_checkbox->setGeometry(QRect(x_checkbox, y_checkbox, 150, 50));
  auto_reset_checkbox->setText("Autoreset");

  y_offset += y_checkbox + 50;

  sliders.resize(N_sliders);

  for (int i=0;i<N_sliders;i++)
  {
    sliders[i].reset(new Slider(ui->centralWidget, x_offset, y_offset, bar_width, bar_height, &F[i],
                                -F_max[i], F_max[i], scale, Qt::Horizontal, trackbar_name[i]));

    sliders[i]->setSliderPosition(0.0);
    sliders[i]->auto_reset_checkbox = auto_reset_checkbox;
    y_offset += bar_offset;
  }

  y_offset += bar_offset;

  int window_width = 120 + bar_width;
  int window_height = y_offset;
  this->resize(window_width, window_height);
  this->setWindowTitle(QApplication::translate(wind_name.c_str(), wind_name.c_str(), 0));

  QObject::connect(auto_reset_checkbox.get(), SIGNAL(clicked(bool)), this, SLOT(on_auto_reset_checkbox_clicked(bool)));
}

MainWindow::~MainWindow()
{
  delete ui;
}

double MainWindow::getSliderPosition(int i) const
{
  return sliders[i]->getSliderPosition();
}

void MainWindow::setSliderPosition(double val, int i)
{
  sliders[i]->setSliderPosition(val);
}

void MainWindow::on_auto_reset_checkbox_clicked(bool val)
{
  for (int i=0;i<sliders.size();i++) sliders[i]->resetSliderPosition();
}
