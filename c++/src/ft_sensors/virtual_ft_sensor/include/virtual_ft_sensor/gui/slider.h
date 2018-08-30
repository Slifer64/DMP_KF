#ifndef SLIDER_H
#define SLIDER_H

#include <QObject>
#include <QLabel>
#include <QtWidgets/QScrollBar>
#include <QCheckBox>
#include <cmath>
#include <sstream>
#include <iostream>
#include <vector>
#include <memory>
#include <mutex>
#include <thread>
#include <condition_variable>

class Slider : public QObject
{
    Q_OBJECT

public:
    explicit Slider(QWidget *parent, int x, int y, int width, int height, double *pos, double min_pos, double max_pos, double scale,
                    const Qt::Orientation &orient=Qt::Horizontal, const std::string name="");
    ~Slider();

    double getSliderPosition() const;
    void setSliderPosition(double pos);

    void resetSliderPosition();

    std::shared_ptr<const QCheckBox> auto_reset_checkbox;

private slots:
    void on_trackbar_sliderMoved(int position);
    void on_trackbar_sliderReleased();
    void on_trackbar_sliderPressed();

private:
    void init();

    QWidget *parent;

    int x;
    int y;
    int width;
    int height;
    std::string name;

    double *trackbar_pos;
    double min_pos;
    double max_pos;
    double scale;

    Qt::Orientation orient;

    std::shared_ptr<QScrollBar> trackbar;
    std::shared_ptr<QLabel> trackbar_label;
    std::shared_ptr<QLabel> trackbar_pos_label;

    double int2double(int i_pos) const;
    int double2int(int d_pos) const;

    std::mutex value_change_mtx;

    bool is_running;

    std::mutex reset_slider_mtx;
    bool reset_slider;
    std::condition_variable reset_slider_cond;
    void resetSlider();
    std::thread reset_slider_thread;
};

#endif // SLIDER_H
