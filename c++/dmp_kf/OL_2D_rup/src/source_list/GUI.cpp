#include <OL_2D_rup/GUI/GUI.h>
#include <QApplication>

GUI::GUI()
{
  init();
}

GUI::~GUI()
{
  q_app->exit(0);
  if (gui_thread->joinable()) gui_thread->join();
}

void GUI::init()
{
  gui_thread.reset(new std::thread(&GUI::guiThread,this));

  std::mutex mtx;
  std::unique_lock<std::mutex> lck(mtx);
  start_cond.wait(lck); // wait for gui to be initialized

  //gui_thread->detach();
}

int GUI::guiThread()
{
  int argc = 0;
  char **argv = NULL;
  q_app.reset(new QApplication (argc, argv));
  gui_obj.reset(new MainWindow);
  gui_obj->show();

  gui_obj->setMsg("MODE initialized to IDLE", Ui::MSG_TYPE::INFO);
  gui_obj->setModeMsg("== MODE set to IDLE ==");

  start_cond.notify_one();

  int ret_val = q_app->exec();

  gui_obj->finalize();

  return ret_val;
}

bool GUI::logOnEnable()
{
  return gui_obj->log_on_enable;
}

bool GUI::saveLoggedData() const
{
  return gui_obj->save_logged_data;
}

void GUI::resetSaveLoggedData()
{
  gui_obj->save_logged_data = false;
}

bool GUI::clearLoggedData() const
{
  return gui_obj->clear_logged_data;
}

void GUI::resetClearLoggedData()
{
  gui_obj->clear_logged_data = false;
}

bool GUI::currentPoseAsStart() const
{
  return gui_obj->current_pose_as_start;
}

void GUI::resetCurrentPoseAsStart()
{
  gui_obj->current_pose_as_start = false;
}

bool GUI::gotoStartPose() const
{
  return gui_obj->goto_start_pose;
}

void GUI::resetGotoStartPose()
{
  gui_obj->goto_start_pose = false;
}

Ui::ProgramState GUI::getState() const
{
  return gui_obj->state;
}

void GUI::setState(const Ui::ProgramState &new_state)
{
  gui_obj->setState(new_state);
  printModeMsg("== MODE: " + gui_obj->stateName[gui_obj->state] + " ==");
}

void GUI::printMsg(const std::string &msg, Ui::MSG_TYPE msg_type)
{
  gui_obj->setMsg(msg, msg_type);
}

void GUI::printModeMsg(const std::string &msg)
{
  gui_obj->setModeMsg(msg);
}
