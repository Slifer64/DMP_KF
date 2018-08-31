#include <dmp_kf/Controller/Controller.h>
#include <ros/package.h>

Controller::Controller(std::shared_ptr<Robot> &robot)
{
  this->robot = robot;
  is_trained = false;
  is_q_start_set = false;
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

  if (Timed.size() == 0)
  {
    err_msg = std::string("Error saving training data:\nNo training data were recorded...");
    return false;
  }

  if (!is_q_start_set)
  {
    err_msg = std::string("Error saving training data:\nNo starting pose has been registered...");
    return false;
  }

  write_mat(q_start, out, binary);
  write_mat(Timed, out, binary);
  write_mat(Yd_data, out, binary);
  write_mat(dYd_data, out, binary);
  write_mat(ddYd_data, out, binary);

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
  read_mat(Timed, in, binary);
  read_mat(Yd_data, in, binary);
  read_mat(dYd_data, in, binary);
  read_mat(ddYd_data, in, binary);

  int n_data = Yd_data.n_cols;
  g_d = Yd_data.col(n_data-1);
  tau_d = Timed(n_data-1);

  in.close();

  return true;
}

void Controller::setStartPose()
{
  robot->update();
  q_start = robot->getJointPosition();
  is_q_start_set = true;
}
