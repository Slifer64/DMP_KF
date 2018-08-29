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

  bool logOnEnable();

  bool saveLoggedData() const;
  void resetSaveLoggedData();

  bool clearLoggedData() const;
  void resetClearLoggedData();

  bool gotoStartPose() const;
  void resetGotoStartPose();

  bool currentPoseAsStart() const;
  void resetCurrentPoseAsStart();

  Ui::ProgramState getState() const;
  void setState(const Ui::ProgramState &state);

  void printMsg(const std::string &msg, Ui::MSG_TYPE msg_type=Ui::MSG_TYPE::INFO);
  void printModeMsg(const std::string &msg);

private:
  std::shared_ptr<std::thread> gui_thread;

  std::shared_ptr<MainWindow> gui_obj;
  std::shared_ptr<QApplication> q_app;
  std::condition_variable start_cond;
};

#endif // OL_2D_RUP_GUI_H
