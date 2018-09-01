#include <dmp_kf/Controller/Controller.h>
#include <ros/package.h>

Controller::Controller(std::shared_ptr<Robot> &robot, const std::shared_ptr<GUI> gui)
{
  this->robot = robot;
  this->gui = gui;
  is_trained = false;

  setStartPose();
}

Controller::~Controller()
{

}

bool Controller::saveTrainingData(std::string &err_msg)
{
  std::string data_file = ros::package::getPath(PACKAGE_NAME)+ "/data/training_data.bin";
  bool binary = true;

  std::ofstream out(data_file.c_str(), std::ios::binary);
  if (!out)
  {
    err_msg = std::string("Error saving training data:\nCouldn't create file: \"" + data_file + "\"");
    return false;
  }

  if (train_data.isempty())
  {
    err_msg = std::string("Error saving training data:\nNo training data were recorded...");
    remove(data_file.c_str());
    return false;
  }

  write_mat(q_start, out, binary);
  write_mat(train_data.Time, out, binary);
  write_mat(train_data.Y_data, out, binary);
  write_mat(train_data.dY_data, out, binary);
  write_mat(train_data.ddY_data, out, binary);

  out.close();

  return true;
}

bool Controller::loadTrainingData(std::string &err_msg)
{
  std::string data_file = ros::package::getPath(PACKAGE_NAME)+ "/data/training_data.bin";
  bool binary = true;

  std::ifstream in(data_file.c_str(), std::ios::binary);
  if (!in)
  {
    err_msg = std::string("Error loading training data:\nCouldn't open file: \"" + data_file + "\"");
    return false;
  }

  read_mat(q_start, in, binary);
  read_mat(train_data.Time, in, binary);
  read_mat(train_data.Y_data, in, binary);
  read_mat(train_data.dY_data, in, binary);
  read_mat(train_data.ddY_data, in, binary);

  g_d = train_data.getFinalPoint();
  tau_d = train_data.getTimeDuration();

  in.close();

  return true;
}

void Controller::setStartPose()
{
  robot->update();
  q_start = robot->getJointPosition();
}

bool Controller::saveExecutionData(std::string &err_msg)
{
  if (exec_data.isempty())
  {
    err_msg = "Error saving execution data: The data are empty...";
    return false;
  }

  std::string file_name = "execution_data.bin";

  return exec_data.save(file_name, err_msg);
}

bool Controller::saveModelRunData(std::string &err_msg)
{
  if (modelRun_data.isempty())
  {
    err_msg = "Error saving model-run data: The data are empty...";
    return false;
  }

  std::string file_name = "model_run_data.bin";

  return modelRun_data.save(file_name, err_msg);
}
