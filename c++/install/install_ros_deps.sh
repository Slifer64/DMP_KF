
# ==================================
# define some colors for output
# ==================================
COLOR_RED="\033[1;31m"
COLOR_GREEN="\033[1;32m"
COLOR_YELLOW="\033[1;33m"
COLOR_BLUE="\033[1;34m"
COLOR_CYAN="\033[1;36m"
COLOR_WHITE="\033[1;37m"
COLOR_RESET="\033[0m"

echo -e $COLOR_CYAN"*********************************************"$COLOR_RESET
echo -e $COLOR_CYAN"********  Additional ROS packages  **********"$COLOR_RESET
echo -e $COLOR_CYAN"*********************************************"$COLOR_RESET

SLEEP_DURATION=1
ROS_DEPS_FAIL=0


echo -e $COLOR_BLUE"Installing catkin..."$COLOR_RESET
printf 'y\n' | sudo apt-get install ros-$ROS_DISTRO-catkin >> log.txt
if [ $? -eq 0 ]; then
  CATKIN_FAIL=0
  echo -e $COLOR_GREEN"Installed catkin successfully!"$COLOR_RESET
else
  echo -e $COLOR_RED"Failed to install catkin..."$COLOR_RESET
  CATKIN_FAIL=1
  ROS_DEPS_FAIL=1
fi
echo -e $COLOR_CYAN"**************************************"$COLOR_RESET
sleep $SLEEP_DURATION


echo -e $COLOR_BLUE"Installing rviz-visual-tools..."$COLOR_RESET
printf 'y\n' | sudo apt-get install ros-$ROS_DISTRO-rviz-visual-tools >> log.txt
if [ $? -eq 0 ]; then
  RVIZ_VIZUAL_TOOLS_FAIL=0
  echo -e $COLOR_GREEN"Installed rviz-visual-tools successfully!"$COLOR_RESET
else
  echo -e $COLOR_RED"Failed to install rviz-visual-tools..."$COLOR_RESET
  RVIZ_VIZUAL_TOOLS_FAIL=1
  ROS_DEPS_FAIL=1
fi
echo -e $COLOR_CYAN"**************************************"$COLOR_RESET
sleep $SLEEP_DURATION


echo -e $COLOR_BLUE"Installing gazebo-ros..."$COLOR_RESET
printf 'y\n' | sudo apt-get install ros-$ROS_DISTRO-gazebo-ros >> log.txt
if [ $? -eq 0 ]; then
  GAZEBO_ROS_FAIL=0
  echo -e $COLOR_GREEN"Installed gazebo-ros successfully!"$COLOR_RESET
else
  echo -e $COLOR_RED"Failed to install gazebo-ros..."$COLOR_RESET
  GAZEBO_ROS_FAIL=1
  ROS_DEPS_FAIL=1
fi
echo -e $COLOR_CYAN"**************************************"$COLOR_RESET
sleep $SLEEP_DURATION


echo -e $COLOR_BLUE"Installing gazebo-msgs..."$COLOR_RESET
printf 'y\n' | sudo apt-get install ros-$ROS_DISTRO-gazebo-msgs >> log.txt
if [ $? -eq 0 ]; then
  GAZEBO_MSGS_FAIL=0
  echo -e $COLOR_GREEN"Installed gazebo-msgs successfully!"$COLOR_RESET
else
  echo -e $COLOR_RED"Failed to install gazebo-msgs..."$COLOR_RESET
  GAZEBO_MSGS_FAIL=1
  ROS_DEPS_FAIL=1
fi
echo -e $COLOR_CYAN"**************************************"$COLOR_RESET
sleep $SLEEP_DURATION


echo -e $COLOR_BLUE"Installing control-msgs..."$COLOR_RESET
printf 'y\n' | sudo apt-get install ros-$ROS_DISTRO-control-msgs >> log.txt
if [ $? -eq 0 ]; then
  CONTROL_MSGS_FAIL=0
  echo -e $COLOR_GREEN"Installed control-msgs successfully!"$COLOR_RESET
else
  echo -e $COLOR_RED"Failed to install control-msgs..."$COLOR_RESET
  CONTROL_MSGS_FAIL=1
  ROS_DEPS_FAIL=1
fi
echo -e $COLOR_CYAN"**************************************"$COLOR_RESET
sleep $SLEEP_DURATION


echo -e $COLOR_BLUE"Installing control-toolbox..."$COLOR_RESET
printf 'y\n' | sudo apt-get install ros-$ROS_DISTRO-control-toolbox >> log.txt
if [ $? -eq 0 ]; then
  CONTROL_TOOLBOX_FAIL=0
  echo -e $COLOR_GREEN"Installed control-toolbox successfully!"$COLOR_RESET
else
  echo -e $COLOR_RED"Failed to install control-toolbox..."$COLOR_RESET
  CONTROL_TOOLBOX_FAIL=1
  ROS_DEPS_FAIL=1
fi
echo -e $COLOR_CYAN"**************************************"$COLOR_RESET
sleep $SLEEP_DURATION


echo -e $COLOR_BLUE"Installing hardware-interface..."$COLOR_RESET
printf 'y\n' | sudo apt-get install ros-$ROS_DISTRO-hardware-interface >> log.txt
if [ $? -eq 0 ]; then
  HARDWARE_INTERFACE_FAIL=0
  echo -e $COLOR_GREEN"Installed hardware-interface successfully!"$COLOR_RESET
else
  echo -e $COLOR_RED"Failed to install hardware-interface..."$COLOR_RESET
  HARDWARE_INTERFACE_FAIL=1
  ROS_DEPS_FAIL=1
fi
echo -e $COLOR_CYAN"**************************************"$COLOR_RESET
sleep $SLEEP_DURATION


echo -e $COLOR_BLUE"Installing moveit..."$COLOR_RESET
printf 'y\n' | sudo apt-get install ros-$ROS_DISTRO-moveit >> log.txt
if [ $? -eq 0 ]; then
  MOVEIT_FAIL=0
  echo -e $COLOR_GREEN"Installed moveit successfully!"$COLOR_RESET
else
  echo -e $COLOR_RED"Failed to install moveit..."$COLOR_RESET
  MOVEIT_FAIL=1
  ROS_DEPS_FAIL=1
fi
echo -e $COLOR_CYAN"**************************************"$COLOR_RESET
sleep $SLEEP_DURATION


echo -e $COLOR_BLUE"Installing controller-manager..."$COLOR_RESET
printf 'y\n' | sudo apt-get install ros-$ROS_DISTRO-controller-manager >> log.txt
if [ $? -eq 0 ]; then
  CONTROL_MANAGER_FAIL=0
  echo -e $COLOR_GREEN"Installed controller-manager successfully!"$COLOR_RESET
else
  echo -e $COLOR_RED"Failed to install controller-manager..."$COLOR_RESET
  CONTROL_MANAGER_FAIL=1
  ROS_DEPS_FAIL=1
fi
echo -e $COLOR_CYAN"**************************************"$COLOR_RESET
sleep $SLEEP_DURATION


echo -e $COLOR_BLUE"Installing openni-launch..."$COLOR_RESET
printf 'y\n' | sudo apt-get install ros-$ROS_DISTRO-openni-launch >> log.txt
if [ $? -eq 0 ]; then
  OPENNI_LAUNCH_FAIL=0
  echo -e $COLOR_GREEN"Installed openni-launch successfully!"$COLOR_RESET
else
  echo -e $COLOR_RED"Failed to install openni-launch..."$COLOR_RESET
  OPENNI_LAUNCH_FAIL=1
  ROS_DEPS_FAIL=1
fi
echo -e $COLOR_CYAN"**************************************"$COLOR_RESET
sleep $SLEEP_DURATION


echo -e $COLOR_BLUE"Installing freenect-launch..."$COLOR_RESET
printf 'y\n' | sudo apt-get install ros-$ROS_DISTRO-freenect-launch >> log.txt
if [ $? -eq 0 ]; then
  FREENECT_LAUNCH_FAIL=0
  echo -e $COLOR_GREEN"Installed freenect-launch successfully!"$COLOR_RESET
else
  echo -e $COLOR_RED"Failed to install freenect-launch..."$COLOR_RESET
  FREENECT_LAUNCH_FAIL=1
  ROS_DEPS_FAIL=1
fi
echo -e $COLOR_CYAN"**************************************"$COLOR_RESET
sleep $SLEEP_DURATION


echo -e $COLOR_BLUE"Installing openni2-launch..."$COLOR_RESET
printf 'y\n' | sudo apt-get install ros-$ROS_DISTRO-openni2-launch >> log.txt
if [ $? -eq 0 ]; then
  OPENNI2_LAUNCH_FAIL=0
  echo -e $COLOR_GREEN"Installed openni2-launch successfully!"$COLOR_RESET
else
  echo -e $COLOR_RED"Failed to install openni2-launch..."$COLOR_RESET
  OPENNI2_LAUNCH_FAIL=1
  ROS_DEPS_FAIL=1
fi
echo -e $COLOR_CYAN"**************************************"$COLOR_RESET
sleep $SLEEP_DURATION


echo -e $COLOR_CYAN"**************************************"$COLOR_RESET
echo -e $COLOR_CYAN"**************************************"$COLOR_RESET

if [ $ROS_DEPS_FAIL -eq 1 ]; then
  echo -e $COLOR_RED"Failed to install some packages..."$COLOR_RESET
else
  echo -e $COLOR_GREEN"All ros packages were installed successfully!"$COLOR_RESET
fi

echo -e $COLOR_BLUE"==== Ros packages installation summary ===="$COLOR_RESET
if [ $CATKIN_FAIL -eq 1 ]; then echo -e $COLOR_RED"* catkin (FAIL)"$COLOR_RESET; else echo -e $COLOR_GREEN"* catkin (SUCCESS)"$COLOR_RESET; fi
if [ $RVIZ_VIZUAL_TOOLS_FAIL -eq 1 ]; then echo -e $COLOR_RED"* rviz-visual-tools (FAIL)"$COLOR_RESET; else echo -e $COLOR_GREEN"* rviz-visual-tools (SUCCESS)"$COLOR_RESET; fi
if [ $GAZEBO_ROS_FAIL -eq 1 ]; then echo -e $COLOR_RED"* gazebo-ros (FAIL)"$COLOR_RESET; else echo -e $COLOR_GREEN"* gazebo-ros (SUCCESS)"$COLOR_RESET; fi
if [ $GAZEBO_MSGS_FAIL -eq 1 ]; then echo -e $COLOR_RED"* gazebo-msgs (FAIL)"$COLOR_RESET; else echo -e $COLOR_GREEN"* gazebo-msgs (SUCCESS)"$COLOR_RESET; fi

if [ $CONTROL_MSGS_FAIL -eq 1 ]; then echo -e $COLOR_RED"* control-msgs (FAIL)"$COLOR_RESET; else echo -e $COLOR_GREEN"* control-msgs (SUCCESS)"$COLOR_RESET; fi
if [ $CONTROL_TOOLBOX_FAIL -eq 1 ]; then echo -e $COLOR_RED"* control-toolbox (FAIL)"$COLOR_RESET; else echo -e $COLOR_GREEN"* control-toolbox (SUCCESS)"$COLOR_RESET; fi
if [ $HARDWARE_INTERFACE_FAIL -eq 1 ]; then echo -e $COLOR_RED"* hardware-interface (FAIL)"$COLOR_RESET; else echo -e $COLOR_GREEN"* hardware-interface (SUCCESS)"$COLOR_RESET; fi
if [ $MOVEIT_FAIL -eq 1 ]; then echo -e $COLOR_RED"* moveit (FAIL)"$COLOR_RESET; else echo -e $COLOR_GREEN"* moveit (SUCCESS)"$COLOR_RESET; fi
if [ $CONTROL_MANAGER_FAIL -eq 1 ]; then echo -e $COLOR_RED"* controller-manager (FAIL)"$COLOR_RESET; else echo -e $COLOR_GREEN"* controller-manager (SUCCESS)"$COLOR_RESET; fi
if [ $OPENNI_LAUNCH_FAIL -eq 1 ]; then echo -e $COLOR_RED"* openni-launch (FAIL)"$COLOR_RESET; else echo -e $COLOR_GREEN"* openni-launch (SUCCESS)"$COLOR_RESET; fi
if [ $FREENECT_LAUNCH_FAIL -eq 1 ]; then echo -e $COLOR_RED"* freenect-launch (FAIL)"$COLOR_RESET; else echo -e $COLOR_GREEN"* freenect-launch (SUCCESS)"$COLOR_RESET; fi
if [ $OPENNI2_LAUNCH_FAIL -eq 1 ]; then echo -e $COLOR_RED"* openni2-launch (FAIL)"$COLOR_RESET; else echo -e $COLOR_GREEN"* openni2-launch (SUCCESS)"$COLOR_RESET; fi

sleep 4
