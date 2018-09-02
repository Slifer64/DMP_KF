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

  bool recordDemo() const { return gui_obj->record_demo; }
  void resetRecordDemo() { gui_obj->record_demo = false; }

  bool trainModel() const { return gui_obj->train_model; }
  bool resetTrainModel() { gui_obj->train_model = false; }

  bool runSimulation() const { return gui_obj->run_sim; }
  void resetRunSimulation() { gui_obj->run_sim = false; }
  bool saveSimulationData() const { return gui_obj->save_sim_data; }
  void resetSaveSimulationData() { gui_obj->save_sim_data = false; }
  bool logSimulationData() const { return gui_obj->log_sim; }

  bool saveControllerData() const { return gui_obj->save_controller_data; }
  void resetSaveControllerData() { gui_obj->save_controller_data = false; }
  bool logControllerData() const { return gui_obj->log_controller; }

  bool saveModelRunData() const { return gui_obj->save_modelRun_data; }
  void resetSaveModelRunData() { gui_obj->save_modelRun_data = false; }
  bool logModelRunData() const { return gui_obj->log_modelRun; }

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

  bool saveTrainingData() const { return gui_obj->save_training_data; }
  void resetSaveTrainingData() { gui_obj->save_training_data = false; }

  bool loadTrainingData() const { return gui_obj->load_training_data; }
  void resetLoadTrainingData() { gui_obj->load_training_data = false; }

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
