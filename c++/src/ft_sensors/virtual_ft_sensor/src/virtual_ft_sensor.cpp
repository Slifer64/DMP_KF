#include <virtual_ft_sensor/virtual_ft_sensor.h>

namespace as64_
{

VFTSensor::VFTSensor()
{
  is_running = false;

  parseConfigParams();
}

VFTSensor::~VFTSensor()
{
  q_app->exit(0);
  if (run_thread.joinable()) run_thread.join();
}

void VFTSensor::parseConfigParams()
{
  ros::NodeHandle nh("~");

  if (!nh.getParam("freq",freq)) freq = 1000;
  if (!nh.getParam("pub_topic",pub_topic)) pub_topic = "/wrench";
  if (!nh.getParam("F_max",F_max)) F_max = {20, 20, 20, 40, 40, 40};
  if (!nh.getParam("noise_ampl",noise_ampl)) noise_ampl = {0, 0, 0, 0, 0, 0};
  F = std::vector<double>(6,0.0);
  F_filt = F;
  if (!nh.getParam("scale",scale)) scale = 0.1;
  if (!nh.getParam("a_filt",a_filt)) a_filt = 0.1;
}

int VFTSensor::runGui()
{
  int argc = 0;
  char **argv = NULL;
  q_app.reset(new QApplication(argc, argv));

  gui.reset(new MainWindow(F_max, F, scale));

  gui->show();
  is_running = true;

  int ret_val = q_app->exec();

  is_running = false;

  return ret_val;
}

void VFTSensor::init()
{
  run_thread = std::thread(&VFTSensor::runGui, this);

  while (!isRunning()) std::this_thread::sleep_for(std::chrono::milliseconds(1)); // wait for gui to initialize

  f_reset = std::vector<bool>(6,false);
  F.resize(6);
  for (int i=0;i<F.size();i++) F[i] = 0.0;
  for (int i=0;i<F.size();i++) setForce(F[i], i);

  wrench_pub = node.advertise<geometry_msgs::WrenchStamped>(pub_topic, 1);
  pub_rate.reset(new ros::Rate(freq));
}

void VFTSensor::run()
{
  std::default_random_engine generator;
  std::vector<std::normal_distribution<double>> f_n(6);
  for (int i=0;i<6;i++) f_n[i] = std::normal_distribution<double>(0.0,1.0);

  while (node.ok() && isRunning())
  {
    for (int i=0;i<F.size(); i++)
    {
      double noise = f_n[i](generator);
      F_filt[i] = (1-a_filt)*F_filt[i] + a_filt*F[i] + noise_ampl[i]*noise;
    }

    wrench_msg.header.stamp = ros::Time::now();
    wrench_msg.wrench.force.x = F_filt[0];
    wrench_msg.wrench.force.y = F_filt[1];
    wrench_msg.wrench.force.z = F_filt[2];
    wrench_msg.wrench.torque.x = F_filt[3];
    wrench_msg.wrench.torque.y = F_filt[4];
    wrench_msg.wrench.torque.z = F_filt[5];

    wrench_pub.publish(wrench_msg);

    pub_rate->sleep();
  }
}

double VFTSensor::getForce(int i) const
{
  return gui->getSliderPosition(i);
}

double VFTSensor::setForce(double f_val, int i)
{
  gui->setSliderPosition(f_val,i);
}

bool VFTSensor::isRunning() const
{
  return is_running;
}

} // namespace as64_
