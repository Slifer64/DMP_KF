#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QAbstractButton>
#include <QLabel>
#include <QString>
#include <QColor>
#include <QPalette>
#include <QFont>
#include <QString>
#include <QSlider>

#include <iostream>
#include <cstdlib>
#include <vector>
#include <thread>
#include <mutex>
#include <cstring>
#include <memory>
#include <sstream>
#include <condition_variable>
#include <iomanip>

#include <QLineEdit>
#include <QSlider>

namespace Ui {

enum ProgramState
{
  RUN_CONTROLLER,
  FREEDRIVE_MODE,
  PAUSE_PROGRAM,
  STOP_PROGRAM,
  DEMO_RECORDING
};

enum MSG_TYPE
{
  INFO,
  WARNING,
  ERROR,
  SUCCESS
};

class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    void init();

    Ui::ProgramState state;

    bool print_pos;

    bool save_training_data;
    bool load_training_data;

    bool run_trained_model;
    bool save_trained_model;
    bool load_trained_model;

    bool start_demo_record;
    bool stop_demo_record;
    bool demo_recorded; ///< true if a demo has been recorded
    bool train_model;
    bool model_trained;

    bool run_sim;
    bool log_sim;
    bool save_sim_data;

    bool log_controller;
    bool save_controller_data;

    bool log_modelRun;
    bool save_modelRun_data;

    bool goto_start_pose;
    bool current_pose_as_start;

    std::vector<std::string> stateName;

    void setMsg(const std::string &msg, Ui::MSG_TYPE msg_type);
    void setModeMsg(const std::string &msg);

    Ui::ProgramState getState();
    void setState(const Ui::ProgramState &new_state);

    void setJointSliderPos(double pos, double min, double max, int i);
    void setEndEffectorPosition(double x, double y, double z);

    void finalize();

private slots:
    void on_freedrive_mode_button_clicked();
    void on_pause_program_button_clicked();
    void on_stop_program_button_clicked();

    void on_run_controller_button_clicked();

    void on_move_to_start_btn_clicked();
    void on_register_startPose_btn_clicked();

    void on_record_demo_clicked();

    void on_train_model_btn_clicked();

    void on_save_trained_model_btn_clicked();

    void on_load_trained_model_btn_clicked();

    void on_run_trained_model_btn_clicked();

    void on_save_training_data_btn_clicked();

    void on_load_training_data_btn_clicked();

    void on_start_demo_record_btn_clicked();

    void on_stop_demo_record_btn_clicked();

    void on_controller_log_checkbox_toggled(bool checked);

    void on_save_controller_data_btn_clicked();

    void on_modelRun_log_checkbox_toggled(bool checked);

    void on_save_modelRun_data_btn_clicked();

    void on_sim_log_checkbox_toggled(bool checked);

    void on_save_sim_data_btn_clicked();

    void on_sim_controller_button_clicked();

    void on_print_pos_btn_clicked();

private:
    std::shared_ptr<QLabel> j_pos_label;
    std::vector<std::shared_ptr<QLineEdit>> j_pos_box;
    std::vector<std::shared_ptr<QLineEdit>> j_name_box;
    std::vector<std::shared_ptr<QLineEdit>> j_pos_units_box;
    std::vector<std::shared_ptr<QSlider>> j_slider;
    void createJointPositionsDisplay();

    std::shared_ptr<QLabel> ee_pos_label;
    std::vector<std::shared_ptr<QLineEdit>> ee_pos_box;
    std::vector<std::shared_ptr<QLineEdit>> ee_pos_name_box;
    std::vector<std::shared_ptr<QLineEdit>> ee_pos_units_box;
    void createEndEffectorPositionDisplay();

    void createJointSlider(int i);
    void getJointSliderLimits(int &min, int &max, int i);

    Ui::MainWindow *ui;

    QWidget *parent;

    std::mutex msg_mtx;
    std::mutex mode_msg_mtx;
    std::mutex btn_click_mtx;
    std::mutex state_mtx;

    void checkForReceivedMsgs();
    std::thread receiveMsgs_thread;
    std::string receiv_msg;
    Ui::MSG_TYPE receiv_msg_type;
    std::condition_variable msg_receiv_cond;

    void setStyleSheet(QAbstractButton *btn, const std::string &style_sheet);
    void setStyleSheet(QLabel *label, const QString &style_sheet);

    QColor info_color, warn_color, err_color, success_color;
    QColor label_bg_clolor;
    QFont msg_label_font;
    QFont mode_label_font;

    void PRINT_INFO_MSG(const std::string &msg);
    void PRINT_WARN_MSG(const std::string &msg);
    void PRINT_ERR_MSG(const std::string &msg);
    void PRINT_SUCCESS_MSG(const std::string &msg);

};

#endif // MAINWINDOW_H
