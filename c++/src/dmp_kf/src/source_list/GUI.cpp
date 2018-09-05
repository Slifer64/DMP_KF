#include <dmp_kf/GUI/GUI.h>
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

void GUI::printRobotState(const arma::vec &q, const arma::vec &q_min, const arma::vec &q_max, const arma::vec &pos)
{
  for (int i=0;i<q.size();i++)
  {
    gui_obj->setJointSliderPos(q(i), q_min(i), q_max(i), i);
  }
  gui_obj->setEndEffectorPosition(pos(0), pos(1), pos(2));
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


Ui::ProgramState GUI::getState() const
{
  return gui_obj->state;
}

void GUI::setState(const Ui::ProgramState &new_state)
{
  gui_obj->setState(new_state);
  printModeMsg("== MODE: " + gui_obj->stateName[gui_obj->state] + " ==");
}
