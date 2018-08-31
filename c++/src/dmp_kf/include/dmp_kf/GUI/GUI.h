#ifndef OL_2D_RUP_GUI_H
#define OL_2D_RUP_GUI_H

#include <cstdlib>
#include <iostream>
#include <iomanip>
#include <vector>
#include <cstring>
#include <thread>
#include <mutex>
#include <condition_variable>

#include <mainwindow.h>

class GUI
{
public:

  GUI();
  ~GUI();

  void init();
  int guiThread();

  bool logOnEnable() { return gui_obj->log_on_enable; }

  bool recordDemo() { return gui_obj->record_demo; }

  bool train() { return gui_obj->train; }
  bool resetTrain() { gui_obj->train = false; }

  bool saveLoggedData() const { return gui_obj->save_logged_data; }
  void resetSaveLoggedData() { gui_obj->save_logged_data = false; }

  bool clearLoggedData() const { return gui_obj->clear_logged_data; }
  void resetClearLoggedData() { gui_obj->clear_logged_data = false; }

  bool gotoStartPose() const { return gui_obj->goto_start_pose; }
  void resetGotoStartPose() { gui_obj->goto_start_pose = false; }

  bool currentPoseAsStart() const { return gui_obj->current_pose_as_start; }
  void resetCurrentPoseAsStart() { gui_obj->current_pose_as_start = false; }

  bool saveTrainedModel() const { return gui_obj->save_trained_model; }
  void resetSaveTrainedModel() { gui_obj->save_trained_model = false; }

  bool loadTrainedModel() const { return gui_obj->load_trained_model; }
  void resetLoadTrainedModel() { gui_obj->load_trained_model = false; }

  bool runTrainedModel() const { return gui_obj->run_trained_model; }
  void resetRunTrainedModel() { gui_obj->run_trained_model = false; }

  Ui::ProgramState getState() const;
  void setState(const Ui::ProgramState &state);

  void printMsg(const std::string &msg, Ui::MSG_TYPE msg_type=Ui::MSG_TYPE::INFO) { gui_obj->setMsg(msg, msg_type); }
  void printModeMsg(const std::string &msg) { gui_obj->setModeMsg(msg); }

private:
  std::shared_ptr<std::thread> gui_thread;

  std::shared_ptr<MainWindow> gui_obj;
  std::shared_ptr<QApplication> q_app;
  std::condition_variable start_cond;
};

#endif // OL_2D_RUP_GUI_H
