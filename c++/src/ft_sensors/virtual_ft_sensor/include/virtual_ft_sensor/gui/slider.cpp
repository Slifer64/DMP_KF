#include "slider.h"

Slider::Slider(QWidget *parent, int x, int y, int width, int height, double *pos, double min_pos, double max_pos, double scale,
               const Qt::Orientation &orient, const std::string name)
{
  reset_slider = false;

  this->parent = parent;
  this->x = x;
  this->y = y;
  this->width = width;
  this->height = height;
  this->min_pos = min_pos;
  this->max_pos = max_pos;
  this->scale = scale;
  this->orient = orient;
  this->name = name;

  trackbar_pos = pos;

  init();
}

Slider::~Slider()
{
  is_running = false;
  if (reset_slider_thread.joinable()) reset_slider_thread.join();
  this->disconnect();
}

void Slider::init()
{
  int name_label_x_offset = 10;
  int name_label_width = 30;
  int value_label_width = 50;

  int btw_width = 30;

  int x_pos = name_label_x_offset;

  std::string label_name = name;
  trackbar_label.reset(new QLabel(parent));
  trackbar_label->setObjectName(label_name.c_str());
  trackbar_label->setGeometry(QRect(x_pos, y, name_label_width, height));
  trackbar_label->setText(label_name.c_str());

  x_pos += btw_width;

  trackbar.reset(new QScrollBar(parent));
  trackbar->setObjectName(name.c_str());
  trackbar->setGeometry(QRect(x_pos, y, width, height));
  trackbar->setOrientation(orient);
  trackbar->setMinimum(min_pos/scale);
  trackbar->setMaximum(max_pos/scale);
  *trackbar_pos = 0.0;

  x_pos += width + btw_width;

  std::string label_pos_name = name + "_pos_label";
  trackbar_pos_label.reset(new QLabel(parent));
  trackbar_pos_label->setObjectName(label_pos_name.c_str());
  trackbar_pos_label->setGeometry(QRect(x_pos, y, value_label_width, height));
  trackbar_pos_label->setNum(*trackbar_pos);

  setSliderPosition(*trackbar_pos);

  QObject::connect(trackbar.get(), SIGNAL(sliderMoved(int)), this, SLOT(on_trackbar_sliderMoved(int)));
  QObject::connect(trackbar.get(), SIGNAL(sliderReleased()), this, SLOT(on_trackbar_sliderReleased()));
  // QObject::connect(trackbar.get(), SIGNAL(sliderPressed()), this, SLOT(on_trackbar_sliderPressed()));

  is_running = true;
  // reset_slider_thread = std::thread(&Slider::resetSlider, this);
}

double Slider::int2double(int i_pos) const
{
    return (i_pos*scale);
}

int Slider::double2int(int d_pos) const
{
    return (d_pos/scale);
}

void Slider::on_trackbar_sliderMoved(int position)
{
  std::unique_lock<std::mutex> lck(value_change_mtx);

  *trackbar_pos = int2double(position);
  trackbar_pos_label->setNum(*trackbar_pos);
}

void Slider::on_trackbar_sliderReleased()
{
  // std::cerr << "Slider released!\n";

  // std::unique_lock<std::mutex> lck(reset_slider_mtx);
  // reset_slider = true;
  // reset_slider_cond.notify_one();
  if (auto_reset_checkbox->isChecked())
    resetSliderPosition();
}

void Slider::on_trackbar_sliderPressed()
{
  // std::cerr << "Slider pressed!\n";

  reset_slider = false;
}

double Slider::getSliderPosition() const
{
  // return *trackbar_pos;
  return int2double(trackbar->sliderPosition());
}

void Slider::setSliderPosition(double pos)
{
  // emit trackbar->sliderMoved(double2int(pos));
  std::unique_lock<std::mutex> lck(value_change_mtx);
  *trackbar_pos = pos;
  trackbar->setSliderPosition(double2int(pos));
  trackbar_pos_label->setNum(*trackbar_pos);
}

void Slider::resetSlider()
{
  double a_filt = 0.001;

  while (is_running)
  {
    // std::unique_lock<std::mutex> lck(reset_slider_mtx);
    while (!reset_slider) std::this_thread::sleep_for(std::chrono::milliseconds(1)); // reset_slider_cond.wait(lck);

    double pos = getSliderPosition();
    while (reset_slider)
    {
      pos = (1-a_filt)*pos;
      if (std::fabs(pos) < 1e-6){
        pos = 0.0;
        reset_slider = false;
      }
      setSliderPosition(pos);

      std::cerr << "pos = " << pos << "\n";
    }

  }
}

void Slider::resetSliderPosition()
{
  setSliderPosition(0.0);
}
