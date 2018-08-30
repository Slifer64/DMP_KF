#include <dmp_kf/dmp_kf.h>

#include <dmp_kf/Robot/LWR4p_Robot.h>
#include <dmp_kf/Controller/DMP_EKF_Controller.h>

using namespace as64_;


dmp_kf::dmp_kf()
{
  ros::NodeHandle nh("~");

  std::string robot_type;
  if (!nh.getParam("robot_type", robot_type)) robot_type="lwr4p";
  if (!robot_type.compare("lwr4p")) robot.reset(new LWR4p_Robot());
  // else if (!robot_type.compare("ur10")) robot.reset(new UR10_Robot());
  // else if (!robot_type.compare("robot_sim")) robot.reset(new Sim_Robot());

  controller.reset(new DMP_EKF_Controller(robot));
  gui.reset(new GUI());
  log_data.reset(new LogData());
}

dmp_kf::~dmp_kf()
{
  this->finalize();
}

void dmp_kf::init()
{
  robot->init();
  robot->setMode(Robot::Mode::IDLE_MODE);
  //gui->init();
  log_data->init();
}

void dmp_kf::execute()
{
  init();

  bool exit_program = false;

  robot->update();

  while (!exit_program && ros::ok())
  {
    if (!robot->isOk())
    {
      gui->setState(Ui::ProgramState::PAUSE_PROGRAM);
      std::string err_msg = robot->getErrMsg();
      gui->printMsg(err_msg, Ui::MSG_TYPE::ERROR);
      PRINT_ERROR_MSG(err_msg);
      robot->enable();
    }
    else robot->command();

    switch (gui->getState())
    {
      case Ui::ProgramState::RUN_CONTROLLER:
        if (robot->getMode() != Robot::Mode::VELOCITY_CONTROL)
        {
          robot->setMode(Robot::Mode::VELOCITY_CONTROL);
          gui->printModeMsg("== MODE: RUN_CONTROLLER ==");
          gui->printMsg("Mode changed to velocity control.",Ui::MSG_TYPE::INFO);
          gui->printMsg("Initializing controller...",Ui::MSG_TYPE::INFO);
          controller->initExecution();
          gui->printMsg("Initialized! Controller is running...",Ui::MSG_TYPE::INFO);
        }
        controller->run();
        break;

      case Ui::ProgramState::FREEDRIVE_MODE:
        if (robot->getMode() != Robot::Mode::FREEDRIVE_MODE)
        {
          gui->printMsg("Entering freedrive mode...",Ui::MSG_TYPE::INFO);
          robot->setMode(Robot::Mode::FREEDRIVE_MODE);
          gui->printModeMsg("== MODE: FREEDRIVE ==");
          gui->printMsg("Mode changed to freedrive!",Ui::MSG_TYPE::INFO);
        }
        break;

      case Ui::ProgramState::PAUSE_PROGRAM:
        if (robot->getMode() != Robot::Mode::IDLE_MODE)
        {
          gui->printMsg("Entering idle mode...",Ui::MSG_TYPE::INFO);
          robot->setMode(Robot::Mode::IDLE_MODE);
          gui->printModeMsg("== MODE: IDLE ==");
          gui->printMsg("Mode changed to idle!",Ui::MSG_TYPE::INFO);
        }
        if (gui->saveLoggedData()) this->saveLogData();
        if (gui->gotoStartPose()) this->gotoStartPose();
        break;

      case Ui::ProgramState::STOP_PROGRAM:
        gui->printMsg("Terminating program...",Ui::MSG_TYPE::INFO);
        robot->setMode(Robot::Mode::IDLE_MODE);
        gui->printModeMsg("== MODE: STOP ==");
        gui->printMsg("The program terminated!",Ui::MSG_TYPE::INFO);
        exit_program = true;
        raise(SIGINT);
        break;
    }

    if (gui->currentPoseAsStart()) this->saveCurrentPoseAsStartPose();
    if (gui->clearLoggedData()) this->clearLoggedData();
    if (gui->logOnEnable()) log_data->log(robot, controller);

    robot->update();
  }

}

void dmp_kf::finalize()
{
  if (save_logData_thread->joinable()) save_logData_thread->join();
}

void dmp_kf::saveLogDataThreadFun()
{
  log_data->save();
  save_logData_finished = true;
}

void dmp_kf::saveLogData()
{
  PRINT_INFO_MSG("Saving logged data...\n");
  gui->printMsg("Saving logged data...\n", Ui::MSG_TYPE::INFO);

  save_logData_finished = false;
  save_logData_thread.reset(new std::thread(&dmp_kf::saveLogDataThreadFun, this));
  while (!save_logData_finished) // wait for logging thread to set 'log_data_finished'
  {
    robot->update();
    robot->command();
  }
  if (save_logData_thread->joinable()) save_logData_thread->join();
  gui->resetSaveLoggedData(); // reset gui flag

  PRINT_INFO_MSG("Saved logged data!\n");
  gui->printMsg("Saved logged data!\n", Ui::MSG_TYPE::INFO);
}

void dmp_kf::clearLoggedData()
{
  PRINT_INFO_MSG("Clearing logged data...\n");
  gui->printMsg("Clearing logged data...\n", Ui::MSG_TYPE::INFO);

  log_data->clear();
  gui->resetClearLoggedData(); // reset gui flag

  PRINT_INFO_MSG("Cleared logged data!\n");
  gui->printMsg("Cleared logged data!\n", Ui::MSG_TYPE::INFO);
}

void dmp_kf::saveCurrentPoseAsStartPose()
{
  q_start = robot->getJointPosition();
  gui->printMsg("Registered current pose as start.", Ui::MSG_TYPE::INFO);
  gui->resetCurrentPoseAsStart(); // reset gui flag
}

void dmp_kf::gotoStartPose()
{
  PRINT_INFO_MSG("Moving to start pose...\n");
  gui->printMsg("Moving to start pose...", Ui::MSG_TYPE::INFO);

  robot->update();
  arma::vec q_current = robot->getJointPosition();
  double duration = std::max(arma::max(arma::abs(q_start-q_current))*7.0/arma::datum::pi,2.0);
  robot->setJointTrajectory(q_start, duration);
  robot->update();

  gui->resetGotoStartPose(); // reset gui flag

  q_current = robot->getJointPosition();
  if (arma::norm(q_current-q_start) < 1e-3)
  {
    PRINT_CONFIRM_MSG("Reached start pose!\n");
    gui->printMsg("Reached start pose!", Ui::MSG_TYPE::INFO);
  }
  else
  {
    PRINT_WARNING_MSG("Failed to reach start pose...\n");
    gui->printMsg("Failed to reach start pose...\n", Ui::MSG_TYPE::WARNING);
  }

}
