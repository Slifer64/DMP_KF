#include <dmp_kf/dmp_kf.h>

#include <dmp_kf/Robot/LWR4p_Robot.h>
#include <dmp_kf/Robot/UR10_Robot.h>
#include <dmp_kf/Robot/Sim_Robot.h>
#include <dmp_kf/Controller/DMP_EKF_Controller.h>

using namespace as64_;


dmp_kf::dmp_kf()
{
  ros::NodeHandle nh("~");

  std::string robot_type;
  if (!nh.getParam("robot_type", robot_type)) robot_type="lwr4p";
  if (!robot_type.compare("lwr4p")) robot.reset(new LWR4p_Robot());
  else if (!robot_type.compare("ur10")) robot.reset(new UR10_Robot());
  else if (!robot_type.compare("robot_sim")) robot.reset(new Sim_Robot());

  gui.reset(new GUI());
  controller.reset(new DMP_EKF_Controller(robot, gui));

}

dmp_kf::~dmp_kf()
{
  this->finalize();
}

void dmp_kf::execute()
{
  robot->init();
  robot->setMode(Robot::Mode::IDLE_MODE);

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
          gui->printMsg("Entering RUN_CONTROLLER mode...",Ui::MSG_TYPE::INFO);
          robot->setMode(Robot::Mode::VELOCITY_CONTROL);
          gui->printModeMsg("== MODE: RUN_CONTROLLER ==");
          gui->printMsg("Mode changed to RUN_CONTROLLER!",Ui::MSG_TYPE::SUCCESS);
          gui->printMsg("Initializing controller...",Ui::MSG_TYPE::INFO);
          controller->initExecution();
          gui->printMsg("Initialized! Controller is running...",Ui::MSG_TYPE::INFO);
        }
        controller->execute();
        //if (gui->logOnEnable()) log_data->log();
        break;

      case Ui::ProgramState::FREEDRIVE_MODE:
        if (robot->getMode() != Robot::Mode::FREEDRIVE_MODE)
        {
          gui->printMsg("Entering FREEDRIVE mode...",Ui::MSG_TYPE::INFO);
          robot->setMode(Robot::Mode::FREEDRIVE_MODE);
          gui->printModeMsg("== MODE: FREEDRIVE ==");
          gui->printMsg("Mode changed to FREEDRIVE!",Ui::MSG_TYPE::SUCCESS);
        }
        break;

      case Ui::ProgramState::DEMO_RECORDING:
        if (robot->getMode() != Robot::Mode::FREEDRIVE_MODE)
        {
          gui->printMsg("Entering DEMO_RECORDING mode...",Ui::MSG_TYPE::INFO);
          robot->setMode(Robot::Mode::FREEDRIVE_MODE);
          controller->initDemo();
          gui->printModeMsg("== MODE: DEMO_RECORDING ==");
          gui->printMsg("Mode changed to DEMO_RECORDING!",Ui::MSG_TYPE::SUCCESS);
          gui->printMsg("Registered current pose as start.", Ui::MSG_TYPE::INFO);
        }
        if (gui->recordDemo()) controller->logDemoData();
        break;

      case Ui::ProgramState::PAUSE_PROGRAM:

        if (robot->getMode() != Robot::Mode::IDLE_MODE)
        {
          gui->printMsg("Entering IDLE mode...",Ui::MSG_TYPE::INFO);
          robot->setMode(Robot::Mode::IDLE_MODE);
          gui->printModeMsg("== MODE: IDLE ==");
          gui->printMsg("Mode changed to IDLE!",Ui::MSG_TYPE::SUCCESS);
        }

        if (gui->gotoStartPose()) this->gotoStartPose();

        if (gui->saveModelRunData())
        {
          gui->printMsg("Saving model run data...",Ui::MSG_TYPE::INFO);
          if (controller->saveModelRunData(err_msg)) gui->printMsg("Saved model-run data successfully!",Ui::MSG_TYPE::SUCCESS);
          else gui->printMsg(err_msg.c_str(),Ui::MSG_TYPE::WARNING);
          gui->resetSaveModelRunData();
        }

        if (gui->saveControllerData())
        {
          gui->printMsg("Saving execution data...",Ui::MSG_TYPE::INFO);
          if (controller->saveExecutionData(err_msg)) gui->printMsg("Saved execution data successfully!",Ui::MSG_TYPE::SUCCESS);
          else gui->printMsg(err_msg.c_str(),Ui::MSG_TYPE::WARNING);
          gui->resetSaveControllerData();
        }

        if (gui->saveTrainedModel())
        {
          gui->printMsg("Saving trained model...",Ui::MSG_TYPE::INFO);
          if (controller->saveTrainedModel(err_msg)) gui->printMsg("Trained model saved successfully!",Ui::MSG_TYPE::SUCCESS);
          else gui->printMsg(err_msg.c_str() ,Ui::MSG_TYPE::WARNING);
          gui->resetSaveTrainedModel();
        }

        if (gui->loadTrainedModel())
        {
          gui->printMsg("Loading trained model...",Ui::MSG_TYPE::INFO);
          if (controller->loadTrainedModel(err_msg)) gui->printMsg("Trained model loaded successfully!",Ui::MSG_TYPE::SUCCESS);
          else gui->printMsg(err_msg.c_str() ,Ui::MSG_TYPE::WARNING);
          gui->resetLoadTrainedModel();
        }

        if (gui->runTrainedModel())
        {
          gui->printMsg("Running trained model...",Ui::MSG_TYPE::INFO);
          robot->setMode(Robot::Mode::VELOCITY_CONTROL);
          controller->runModel();
          robot->setMode(Robot::Mode::IDLE_MODE);
          gui->printMsg("Finished running trained model!",Ui::MSG_TYPE::SUCCESS);
          gui->resetRunTrainedModel();
        }

        if (gui->saveTrainingData())
        {
          gui->printMsg("Saving training data...",Ui::MSG_TYPE::INFO);
          if (controller->saveTrainingData(err_msg)) gui->printMsg("Training data saved successfully!",Ui::MSG_TYPE::SUCCESS);
          else gui->printMsg(err_msg.c_str(),Ui::MSG_TYPE::WARNING);
          gui->resetSaveTrainingData();
        }

        if (gui->loadTrainingData())
        {
          gui->printMsg("Loading training data...",Ui::MSG_TYPE::INFO);
          if (controller->loadTrainingData(err_msg)) gui->printMsg("Loaded training data successfully!",Ui::MSG_TYPE::SUCCESS);
          else gui->printMsg(err_msg.c_str(), Ui::MSG_TYPE::WARNING);
          gui->resetLoadTrainingData();
        }

        if (gui->trainModel())
        {
          gui->printMsg("Started training...",Ui::MSG_TYPE::INFO);
          if (controller->train(err_msg)) gui->printMsg("Finished training!",Ui::MSG_TYPE::SUCCESS);
          else gui->printMsg(err_msg,Ui::MSG_TYPE::WARNING);
          gui->resetTrainModel();
        }

        break;

      case Ui::ProgramState::STOP_PROGRAM:
        gui->printMsg("Terminating program...",Ui::MSG_TYPE::INFO);
        robot->setMode(Robot::Mode::IDLE_MODE);
        gui->printModeMsg("== MODE: STOP ==");
        gui->printMsg("The program terminated!",Ui::MSG_TYPE::SUCCESS);
        exit_program = true;
        raise(SIGINT);
        break;
    }

    if (gui->currentPoseAsStart())
    {
      controller->setStartPose();
      gui->printMsg("Registered current pose as start!", Ui::MSG_TYPE::SUCCESS);
      gui->resetCurrentPoseAsStart(); // reset gui flag
    }

    robot->update();
  }

}

void dmp_kf::finalize()
{
  if (save_logData_thread->joinable()) save_logData_thread->join();
}

void dmp_kf::saveLogDataThreadFun()
{
  //log_data->save();
  save_logData_finished = true;
}

void dmp_kf::saveLogData()
{
  // PRINT_INFO_MSG("Saving logged data...\n");
  // gui->printMsg("Saving logged data...\n", Ui::MSG_TYPE::INFO);
  //
  // save_logData_finished = false;
  // save_logData_thread.reset(new std::thread(&dmp_kf::saveLogDataThreadFun, this));
  // while (!save_logData_finished) // wait for logging thread to set 'log_data_finished'
  // {
  //   robot->update();
  //   robot->command();
  // }
  // if (save_logData_thread->joinable()) save_logData_thread->join();
  // gui->resetSaveLoggedData(); // reset gui flag
  //
  // PRINT_INFO_MSG("Saved logged data!\n");
  // gui->printMsg("Saved logged data!\n", Ui::MSG_TYPE::INFO);
}

void dmp_kf::gotoStartPose()
{
  PRINT_INFO_MSG("Moving to start pose...\n");
  gui->printMsg("Moving to start pose...", Ui::MSG_TYPE::INFO);

  robot->update();
  arma::vec q_current = robot->getJointPosition();
  double duration = std::max(arma::max(arma::abs(controller->q_start-q_current))*7.0/arma::datum::pi,2.0);
  robot->setJointTrajectory(controller->q_start, duration);
  robot->update();

  gui->resetGotoStartPose(); // reset gui flag

  q_current = robot->getJointPosition();
  if (arma::norm(q_current-controller->q_start) < 5e-3)
  {
    PRINT_CONFIRM_MSG("Reached start pose!\n");
    gui->printMsg("Reached start pose!", Ui::MSG_TYPE::SUCCESS);
  }
  else
  {
    PRINT_WARNING_MSG("Failed to reach start pose...\n");
    gui->printMsg("Failed to reach start pose...\n", Ui::MSG_TYPE::WARNING);
  }

}
