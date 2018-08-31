#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
  ui->setupUi(this);

  stateName.resize(5);
  stateName[0] = "RUN_CONTROLLER";
  stateName[1] = "FREEDRIVE_MODE";
  stateName[2] = "PAUSE_PROGRAM";
  stateName[3] = "STOP_PROGRAM";
  stateName[4] = "DEMO_RECORDING";

  init();
}

MainWindow::~MainWindow()
{
  delete ui;
  if (receiveMsgs_thread.joinable()) receiveMsgs_thread.join();
}

void MainWindow::init()
{
  this->setWindowTitle(QApplication::translate("Robot Control", "Robot Control", 0));

  std::string cmd_btn_style_sheet = "QCommandLinkButton {background-color: rgb(233, 185, 110);}";
  ui->freedrive_mode_button->setStyleSheet(cmd_btn_style_sheet.c_str());
  ui->pause_program_button->setStyleSheet(cmd_btn_style_sheet.c_str());
  ui->stop_program_button->setStyleSheet(cmd_btn_style_sheet.c_str());
  ui->run_controller_button->setStyleSheet(cmd_btn_style_sheet.c_str());

  ui->save_logged_data_btn->setStyleSheet(cmd_btn_style_sheet.c_str());
  ui->clear_logged_data_btn->setStyleSheet(cmd_btn_style_sheet.c_str());
  ui->register_startPose_btn->setStyleSheet(cmd_btn_style_sheet.c_str());
  ui->move_to_start_btn->setStyleSheet(cmd_btn_style_sheet.c_str());

  state = Ui::ProgramState::PAUSE_PROGRAM;
  save_logged_data = false;
  clear_logged_data = false;
  log_on_enable = false;
  startPose_registered = false;
  current_pose_as_start = false;
  goto_start_pose = false;

  record_demo = false;
  demo_recorded = false;
  train = false;
  model_trained = false;

  save_trained_model = false;
  load_trained_model = false;

  run_trained_model = false;

  ui->msg_label->setWordWrap(true);
  ui->msg_label2->setWordWrap(true);
  ui->msg_label3->setWordWrap(true);
  ui->mode_msg->setWordWrap(true);

  setMsg("", Ui::MSG_TYPE::INFO);
  setMsg("", Ui::MSG_TYPE::INFO);
  setMsg("", Ui::MSG_TYPE::INFO);
  setModeMsg("");

  receiveMsgs_thread = std::thread(&MainWindow::checkForReceivedMsgs, this);
}

void MainWindow::setState(const Ui::ProgramState &new_state)
{
  std::unique_lock<std::mutex> lck(state_mtx);
  state = new_state;
}

Ui::ProgramState MainWindow::getState()
{
  std::unique_lock<std::mutex> lck(state_mtx);
  return state;
}

void MainWindow::on_freedrive_mode_button_clicked()
{
  std::unique_lock<std::mutex> lck(btn_click_mtx);

  if (getState() == Ui::ProgramState::FREEDRIVE_MODE)
  {
    setMsg("Mode already in freedrive.", Ui::MSG_TYPE::WARNING);
    return;
  }

  if (getState() != Ui::ProgramState::PAUSE_PROGRAM)
  {
    setMsg("The program must be paused\nto run enter freedrive mode...", Ui::MSG_TYPE::WARNING);
  }
  else
  {
      setState(Ui::ProgramState::FREEDRIVE_MODE);
      // setModeMsg("== MODE: FREEDRIVE ==");
      // setMsg("The robot is free to be moved.", Ui::MSG_TYPE::INFO);
  }
}

void MainWindow::on_pause_program_button_clicked()
{
  std::unique_lock<std::mutex> lck(btn_click_mtx);

  if (getState() == Ui::ProgramState::PAUSE_PROGRAM)
  {
    setMsg("Mode already idle.", Ui::MSG_TYPE::WARNING);
    return;
  }

  setState(Ui::ProgramState::PAUSE_PROGRAM);
  // setModeMsg("== MODE: " + stateName[state] + " ==");
  // setMsg("Pausing program. Entering idle state.", Ui::MSG_TYPE::INFO);
}

void MainWindow::on_stop_program_button_clicked()
{
  std::unique_lock<std::mutex> lck(btn_click_mtx);

  if (getState() == Ui::ProgramState::STOP_PROGRAM)
  {
    setMsg("Mode already stopped.", Ui::MSG_TYPE::WARNING);
    return;
  }

  setState(Ui::ProgramState::STOP_PROGRAM);
  // setModeMsg("== MODE: STOP ==");
  // setMsg("Stopping program execution. Exiting...", Ui::MSG_TYPE::INFO);
}

void MainWindow::on_run_controller_button_clicked()
{
  std::unique_lock<std::mutex> lck(btn_click_mtx);

  if (getState() == Ui::ProgramState::RUN_CONTROLLER)
  {
    setMsg("Mode already in run controller.", Ui::MSG_TYPE::WARNING);
    return;
  }

  if (getState() != Ui::ProgramState::PAUSE_PROGRAM)
  {
    setMsg("The program must be paused\nto run the controller...", Ui::MSG_TYPE::WARNING);
  }
  else
  {
    setState(Ui::ProgramState::RUN_CONTROLLER);
    // setModeMsg("== MODE: RUN_CONTROLLER ==");
    // setMsg("Running Controller", Ui::MSG_TYPE::INFO);
  }
}

void MainWindow::on_move_to_start_btn_clicked()
{
  std::unique_lock<std::mutex> lck(btn_click_mtx);

  if (getState() != Ui::ProgramState::PAUSE_PROGRAM)
  {
    setMsg("The program must be paused\nto go to start pose...", Ui::MSG_TYPE::WARNING);
  }
  else if (!startPose_registered)
  {
    setMsg("No start pose has been registered...", Ui::MSG_TYPE::WARNING);
  }
  else
  {
    goto_start_pose = true;
    // setMsg("Moving to start pose...", Ui::MSG_TYPE::INFO);
  }
}

void MainWindow::on_register_startPose_btn_clicked()
{
  std::unique_lock<std::mutex> lck(btn_click_mtx);

  current_pose_as_start = true;
  startPose_registered = true;

  // setMsg("Registered current pose as start.", Ui::MSG_TYPE::INFO);
}

void MainWindow::on_save_logged_data_btn_clicked()
{
  std::unique_lock<std::mutex> lck(btn_click_mtx);
  if (getState() != Ui::ProgramState::PAUSE_PROGRAM)
  {
    setMsg("Cannot save the logged data.\nThe program must be paused...", Ui::MSG_TYPE::WARNING);
  }
  else
  {
    save_logged_data = true;
    // PRINT_INFO_MSG("Saving logged data...");
  }

}

void MainWindow::on_clear_logged_data_btn_clicked()
{
  std::unique_lock<std::mutex> lck(btn_click_mtx);

  clear_logged_data = true;
  //PRINT_INFO_MSG("Clearing logged data.");
}


void MainWindow::on_data_logging_checkbox_toggled(bool checked)
{
  std::unique_lock<std::mutex> lck(btn_click_mtx);

  log_on_enable = checked;
  std::string msg = (checked?"Enabled":"Disabled");
  msg += " data logging.";
  setMsg(msg.c_str(), Ui::MSG_TYPE::INFO);
}


void MainWindow::PRINT_INFO_MSG(const std::string &msg)
{
  // std::unique_lock<std::mutex> lck(msg_mtx);
  ui->msg_label->setStyleSheet("QLabel { background-color : rgb(238, 238, 236); color : blue; font: 75 12pt \"DejaVu Serif\"}");
  // ui->msg_label->setText(("[INFO]: " + msg).c_str());
  ui->msg_label->setText(msg.c_str());
}

void MainWindow::PRINT_WARN_MSG(const std::string &msg)
{
  // std::unique_lock<std::mutex> lck(msg_mtx);
  ui->msg_label->setStyleSheet("QLabel { background-color : rgb(238, 238, 236); color : rgb(245, 121, 0); font: 75 12pt \"DejaVu Serif\"}");
  // ui->msg_label->setText(("[WARNING]: " + msg).c_str());
  ui->msg_label->setText(msg.c_str());
}

void MainWindow::PRINT_ERR_MSG(const std::string &msg)
{
  // std::unique_lock<std::mutex> lck(msg_mtx);
  ui->msg_label->setStyleSheet("QLabel { background-color : rgb(238, 238, 236); color : red; font: 75 12pt \"DejaVu Serif\"}");
  // ui->msg_label->setText(("[ERROR]: " + msg).c_str());
  ui->msg_label->setText(msg.c_str());
}

void MainWindow::setMsg(const std::string &msg, Ui::MSG_TYPE msg_type)
{
  std::unique_lock<std::mutex> lck(msg_mtx);

  ui->msg_label3->setStyleSheet(ui->msg_label2->styleSheet());
  ui->msg_label3->setText(ui->msg_label2->text());

  ui->msg_label2->setStyleSheet(ui->msg_label->styleSheet());
  ui->msg_label2->setText(ui->msg_label->text());

  switch (msg_type)
  {
    case Ui::MSG_TYPE::INFO:
      PRINT_INFO_MSG(msg.c_str());
      break;
    case Ui::MSG_TYPE::WARNING:
      PRINT_WARN_MSG(msg.c_str());
      break;
    case Ui::MSG_TYPE::ERROR:
      PRINT_ERR_MSG(msg.c_str());
      break;
  }
}

void MainWindow::setModeMsg(const std::string &msg)
{
  std::unique_lock<std::mutex> lck(mode_msg_mtx);
  ui->mode_msg->setStyleSheet("QLabel { background-color : rgb(238, 238, 236); color : blue; font: 75 13pt \"DejaVu Serif\"}");
  ui->mode_msg->setText(msg.c_str());
}

void MainWindow::checkForReceivedMsgs()
{
  while (true)
  {
    std::mutex mtx;
    std::unique_lock<std::mutex> lck(mtx);

    msg_receiv_cond.wait(lck);

    std::unique_lock<std::mutex> lck2(msg_mtx);
    switch (receiv_msg_type) {
      case Ui::MSG_TYPE::INFO:
        PRINT_INFO_MSG(receiv_msg.c_str());
        break;
      case Ui::MSG_TYPE::WARNING:
        PRINT_WARN_MSG(receiv_msg.c_str());
        break;
      case Ui::MSG_TYPE::ERROR:
        PRINT_ERR_MSG(receiv_msg.c_str());
        break;
    }
  }
}

void MainWindow::finalize()
{

}

void MainWindow::on_record_demo_clicked()
{
    std::unique_lock<std::mutex> lck(btn_click_mtx);

    if (getState() == Ui::ProgramState::DEMO_RECORDING)
    {
      setMsg("Mode already in DEMO_RECORDING.", Ui::MSG_TYPE::WARNING);
      return;
    }

    if (getState() != Ui::ProgramState::PAUSE_PROGRAM)
    {
      setMsg("The program must be paused\nto enter DEMO recording state...", Ui::MSG_TYPE::WARNING);
    }
    else
    {
      setState(Ui::ProgramState::DEMO_RECORDING);
      record_demo = true;
      demo_recorded = true;
    }
}

void MainWindow::on_stop_demo_record_btn_clicked()
{
    std::unique_lock<std::mutex> lck(btn_click_mtx);

    if (getState() != Ui::ProgramState::DEMO_RECORDING)
    {
      setMsg("The program is not in DEMO_RECORDING...", Ui::MSG_TYPE::WARNING);
    }
    else
    {
      record_demo = false;
      setMsg("Stopped demo recording...", Ui::MSG_TYPE::INFO);
    }

}

void MainWindow::on_train_btn_clicked()
{
    std::unique_lock<std::mutex> lck(btn_click_mtx);

    if (getState() != Ui::ProgramState::DEMO_RECORDING)
    {
      setMsg("The program is not in DEMO_RECORDING...", Ui::MSG_TYPE::WARNING);
    }
    else if (record_demo == true)
    {
       setMsg("Demo recording is still on...", Ui::MSG_TYPE::WARNING);
    }
    else
    {
      train = true;
      model_trained = true;
    }
}

void MainWindow::on_save_trained_model_btn_clicked()
{
    if (getState() != Ui::ProgramState::PAUSE_PROGRAM)
    {
      setMsg("The program must be paused\nto save the trained model...", Ui::MSG_TYPE::WARNING);
    }
    else if (model_trained == false)
    {
      setMsg("There is no trained model...", Ui::MSG_TYPE::WARNING);
    }
    else
    {
      save_trained_model = true;
    }
}

void MainWindow::on_load_trained_model_btn_clicked()
{
    if (getState() != Ui::ProgramState::PAUSE_PROGRAM)
    {
      setMsg("The program must be paused\nto load the trained model...", Ui::MSG_TYPE::WARNING);
    }
    else
    {
      load_trained_model = true;
      model_trained = true;
      startPose_registered = true;
    }
}

void MainWindow::on_run_trained_model_btn_clicked()
{
    if (getState() != Ui::ProgramState::PAUSE_PROGRAM)
    {
      setMsg("The program must be paused\nto run the trained model...", Ui::MSG_TYPE::WARNING);
    }
    else if (model_trained == false)
    {
      setMsg("There is no trained model to run...", Ui::MSG_TYPE::WARNING);
    }
    else
    {
      run_trained_model = true;
    }
}
