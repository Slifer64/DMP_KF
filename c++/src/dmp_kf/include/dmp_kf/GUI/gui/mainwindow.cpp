#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
  ui->setupUi(this);

  this->parent = parent;

  stateName.resize(5);
  stateName[0] = "RUN_CONTROLLER";
  stateName[1] = "FREEDRIVE_MODE";
  stateName[2] = "PAUSE_PROGRAM";
  stateName[3] = "STOP_PROGRAM";
  stateName[4] = "DEMO_RECORDING";

  info_color = QColor(0, 0, 255); // blue
  warn_color = QColor(200, 100, 0); // yellow-orange
  err_color = QColor(255, 0, 0); // red
  success_color = QColor(0, 125, 0); // green
  label_bg_clolor = QColor(238, 238, 236);

  msg_label_font = QFont("DejaVu Serif", 14);

  mode_label_font = QFont("DejaVu Serif", 16);
  mode_label_font.setBold(true);

  j_slider.resize(7);
  // for (int i=0;i<7;i++) createSlider(i);

  init();
}

void MainWindow::createSlider(int i)
{
  std::ostringstream slider_name;
  slider_name << "j" << i << "_slider";

  j_slider[i].reset(new QSlider(parent));
  j_slider[i]->setObjectName(QString(slider_name.str().c_str()));
  j_slider[i]->setGeometry(QRect(660, 90 + i*30, 161, 21));
  j_slider[i]->setOrientation(Qt::Horizontal);
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

  setStyleSheet(ui->freedrive_mode_button, cmd_btn_style_sheet.c_str());
  setStyleSheet(ui->pause_program_button, cmd_btn_style_sheet.c_str());
  setStyleSheet(ui->pause_program_button, cmd_btn_style_sheet.c_str());
  setStyleSheet(ui->pause_program_button, cmd_btn_style_sheet.c_str());
  setStyleSheet(ui->pause_program_button, cmd_btn_style_sheet.c_str());
  setStyleSheet(ui->pause_program_button, cmd_btn_style_sheet.c_str());
  setStyleSheet(ui->pause_program_button, cmd_btn_style_sheet.c_str());
  setStyleSheet(ui->pause_program_button, cmd_btn_style_sheet.c_str());

  state = Ui::ProgramState::PAUSE_PROGRAM;

  run_sim = false;
  log_sim = false;
  save_sim_data = false;

  save_training_data = false;
  load_training_data = false;

  log_controller = false;
  save_controller_data = false;

  log_modelRun = false;
  save_modelRun_data = false;

  current_pose_as_start = false;
  goto_start_pose = false;

  record_demo = false;
  demo_recorded = false;

  train_model = false;
  model_trained = false;

  save_trained_model = false;
  load_trained_model = false;

  run_trained_model = false;

  ui->msg_label->setWordWrap(true);
  ui->msg_label->setAutoFillBackground(true);
  ui->msg_label->setFont(msg_label_font);

  ui->msg_label2->setWordWrap(true);
  ui->msg_label2->setAutoFillBackground(true);
  ui->msg_label2->setFont(msg_label_font);

  ui->msg_label3->setWordWrap(true);
  ui->msg_label3->setAutoFillBackground(true);
  ui->msg_label3->setFont(msg_label_font);

  ui->mode_msg->setWordWrap(true);
  ui->mode_msg->setAutoFillBackground(true);
  ui->mode_msg->setFont(mode_label_font);

  setStyleSheet(ui->mode_msg, "QLabel { background-color : rgb(238, 238, 236)}");
  setStyleSheet(ui->msg_label, "QLabel { background-color : rgb(238, 238, 236)}"); //; color : blue; font: 75 12pt \"DejaVu Serif\"}");
  setStyleSheet(ui->msg_label2, "QLabel { background-color : rgb(238, 238, 236)}"); //; color : blue; font: 75 12pt \"DejaVu Serif\"}");
  setStyleSheet(ui->msg_label3, "QLabel { background-color : rgb(238, 238, 236)}"); //; color : blue; font: 75 12pt \"DejaVu Serif\"}");

  setMsg("", Ui::MSG_TYPE::INFO);
  setMsg("", Ui::MSG_TYPE::INFO);
  setMsg("", Ui::MSG_TYPE::INFO);
  setModeMsg("");

  receiveMsgs_thread = std::thread(&MainWindow::checkForReceivedMsgs, this);
}

void MainWindow::setStyleSheet(QAbstractButton *btn, const std::string &style_sheet)
{
  btn->setStyleSheet(style_sheet.c_str());
  btn->style()->unpolish(btn);
  btn->style()->polish(btn);
  btn->update();
}

void MainWindow::setStyleSheet(QLabel *label, const QString &style_sheet)
{
  label->setStyleSheet(style_sheet);
  label->style()->unpolish(label);
  label->style()->polish(label);
  label->update();
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
  setModeMsg("== MODE: PAUSED ==");
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
  setModeMsg("== MODE: STOPPED ==");
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

  // setMsg("Registered current pose as start.", Ui::MSG_TYPE::INFO);
}

void MainWindow::PRINT_INFO_MSG(const std::string &msg)
{
  // setStyleSheet(ui->msg_label, "QLabel { background-color : rgb(238, 238, 236); color : blue; font: 75 12pt \"DejaVu Serif\"}");
  QPalette palette = ui->msg_label->palette();
  // palette.setColor(ui->msg_label->backgroundRole(), label_bg_clolor);
  palette.setColor(ui->msg_label->foregroundRole(), info_color);
  ui->msg_label->setPalette(palette);

  ui->msg_label->setText(msg.c_str());
}

void MainWindow::PRINT_WARN_MSG(const std::string &msg)
{
  // setStyleSheet(ui->msg_label, "QLabel { background-color : rgb(238, 238, 236); color : rgb(245, 121, 0); font: 75 12pt \"DejaVu Serif\"}");
  QPalette palette = ui->msg_label->palette();
  // palette.setColor(ui->msg_label->backgroundRole(), label_bg_clolor);
  palette.setColor(ui->msg_label->foregroundRole(), warn_color);
  ui->msg_label->setPalette(palette);

  ui->msg_label->setText(msg.c_str());
}

void MainWindow::PRINT_ERR_MSG(const std::string &msg)
{
  // setStyleSheet(ui->msg_label, "QLabel { background-color : rgb(238, 238, 236); color : red; font: 75 12pt \"DejaVu Serif\"}");

  QPalette palette = ui->msg_label->palette();
  // palette.setColor(ui->msg_label->backgroundRole(), label_bg_clolor);
  palette.setColor(ui->msg_label->foregroundRole(), err_color);
  ui->msg_label->setPalette(palette);

  ui->msg_label->setText(msg.c_str());
}

void MainWindow::PRINT_SUCCESS_MSG(const std::string &msg)
{
  // setStyleSheet(ui->msg_label, "QLabel { background-color : rgb(238, 238, 236); color : red; font: 75 12pt \"DejaVu Serif\"}");

  QPalette palette = ui->msg_label->palette();
  // palette.setColor(ui->msg_label->backgroundRole(), label_bg_clolor);
  palette.setColor(ui->msg_label->foregroundRole(), success_color);
  ui->msg_label->setPalette(palette);

  ui->msg_label->setText(msg.c_str());
}

void MainWindow::setMsg(const std::string &msg, Ui::MSG_TYPE msg_type)
{
  std::unique_lock<std::mutex> lck(msg_mtx);

  // setStyleSheet(ui->msg_label3, ui->msg_label2->styleSheet());
  ui->msg_label3->setPalette(ui->msg_label2->palette());
  ui->msg_label3->setText(ui->msg_label2->text());

  // setStyleSheet(ui->msg_label2, ui->msg_label->styleSheet());
  ui->msg_label2->setPalette(ui->msg_label->palette());
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
    case Ui::MSG_TYPE::SUCCESS:
      PRINT_SUCCESS_MSG(msg.c_str());
      break;
  }
}

void MainWindow::setModeMsg(const std::string &msg)
{
  std::unique_lock<std::mutex> lck(mode_msg_mtx);
  // setStyleSheet(ui->mode_msg, "QLabel { background-color : rgb(238, 238, 236); color : blue; font: 75 13pt \"DejaVu Serif\"}");
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
      case Ui::MSG_TYPE::SUCCESS:
        PRINT_SUCCESS_MSG(receiv_msg.c_str());
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
      setMsg("The program must be paused\nto enter DEMO_RECORDING state...", Ui::MSG_TYPE::WARNING);
    }
    else
    {
      setState(Ui::ProgramState::DEMO_RECORDING);
    }
}

void MainWindow::on_start_demo_record_btn_clicked()
{
    std::unique_lock<std::mutex> lck(btn_click_mtx);

    if (getState() != Ui::ProgramState::DEMO_RECORDING)
    {
      setMsg("The program must be in DEMO_RECORDING to start recording...", Ui::MSG_TYPE::WARNING);
    }
    else
    {
      record_demo = true;
      setMsg("Started recording!", Ui::MSG_TYPE::INFO);
    }
}

void MainWindow::on_stop_demo_record_btn_clicked()
{
    std::unique_lock<std::mutex> lck(btn_click_mtx);

    if (getState() != Ui::ProgramState::DEMO_RECORDING)
    {
      setMsg("The program must be in DEMO_RECORDING to start recording...", Ui::MSG_TYPE::WARNING);
    }
    else
    {
      record_demo = false;
      setMsg("Stopped recording!", Ui::MSG_TYPE::INFO);
    }
}

void MainWindow::on_train_model_btn_clicked()
{
    std::unique_lock<std::mutex> lck(btn_click_mtx);

    if (getState() != Ui::ProgramState::PAUSE_PROGRAM)
    {
      setMsg("The program is not in IDLE state...", Ui::MSG_TYPE::WARNING);
    }
    else
    {
      train_model = true;
      model_trained = true;
    }
}

void MainWindow::on_save_trained_model_btn_clicked()
{
    if (getState() != Ui::ProgramState::PAUSE_PROGRAM)
    {
      setMsg("The program must be paused\nto save the trained model...", Ui::MSG_TYPE::WARNING);
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

void MainWindow::on_save_training_data_btn_clicked()
{
    if (getState() != Ui::ProgramState::PAUSE_PROGRAM)
    {
      setMsg("The program must be paused\nto save the training data...", Ui::MSG_TYPE::WARNING);
    }
    else
    {
      save_training_data = true;
    }
}

void MainWindow::on_load_training_data_btn_clicked()
{
    if (getState() != Ui::ProgramState::PAUSE_PROGRAM)
    {
      setMsg("The program must be paused\nto load the training data...", Ui::MSG_TYPE::WARNING);
    }
    else
    {
      load_training_data = true;
    }
}

void MainWindow::on_controller_log_checkbox_toggled(bool checked)
{
    std::unique_lock<std::mutex> lck(btn_click_mtx);

    log_controller = checked;
    std::string msg = (checked?"Enabled":"Disabled");
    msg += " controller data logging.";
    setMsg(msg.c_str(), Ui::MSG_TYPE::INFO);
}

void MainWindow::on_save_controller_data_btn_clicked()
{
    std::unique_lock<std::mutex> lck(btn_click_mtx);
    if (getState() != Ui::ProgramState::PAUSE_PROGRAM)
    {
      setMsg("Cannot save execution data.\nThe program must be paused...", Ui::MSG_TYPE::WARNING);
    }
    else
    {
      save_controller_data = true;
      // PRINT_INFO_MSG("Saving logged data...");
    }
}

void MainWindow::on_modelRun_log_checkbox_toggled(bool checked)
{
    std::unique_lock<std::mutex> lck(btn_click_mtx);

    log_modelRun = checked;
    std::string msg = (checked?"Enabled":"Disabled");
    msg += " model run data logging.";
    setMsg(msg.c_str(), Ui::MSG_TYPE::INFO);
}

void MainWindow::on_save_modelRun_data_btn_clicked()
{
    std::unique_lock<std::mutex> lck(btn_click_mtx);
    if (getState() != Ui::ProgramState::PAUSE_PROGRAM)
    {
      setMsg("Cannot save model-run data.\nThe program must be paused...", Ui::MSG_TYPE::WARNING);
    }
    else
    {
      save_modelRun_data = true;
      // PRINT_INFO_MSG("Saving logged data...");
    }
}

void MainWindow::on_sim_log_checkbox_toggled(bool checked)
{
    std::unique_lock<std::mutex> lck(btn_click_mtx);

    log_sim = checked;
    std::string msg = (checked?"Enabled":"Disabled");
    msg += " simulation data logging.";
    setMsg(msg.c_str(), Ui::MSG_TYPE::INFO);
}

void MainWindow::on_save_sim_data_btn_clicked()
{
    std::unique_lock<std::mutex> lck(btn_click_mtx);
    if (getState() != Ui::ProgramState::PAUSE_PROGRAM)
    {
      setMsg("Cannot save simulation data.\nThe program must be paused...", Ui::MSG_TYPE::WARNING);
    }
    else
    {
      save_sim_data = true;
    }
}

void MainWindow::on_sim_controller_button_clicked()
{
    std::unique_lock<std::mutex> lck(btn_click_mtx);

    if (getState() != Ui::ProgramState::PAUSE_PROGRAM)
    {
      setMsg("The program must be paused\nto start simulating the controller...", Ui::MSG_TYPE::WARNING);
    }
    else if (model_trained == false)
    {
      setMsg("There is no trained model to run...", Ui::MSG_TYPE::WARNING);
    }
    else
    {
      run_sim = true;
    }
}
