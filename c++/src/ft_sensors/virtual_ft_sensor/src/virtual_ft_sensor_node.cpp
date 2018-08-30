#include <virtual_ft_sensor/virtual_ft_sensor.h>

int main( int argc, char** argv )
{
  ros::init(argc, argv, "virtual_ft_sensor");

  as64_::VFTSensor sensor;
  sensor.init();
  sensor.run();

  return 0;
}
