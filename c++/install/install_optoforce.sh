#!/bin/bash

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


echo -e $COLOR_CYAN"*******************************"$COLOR_RESET
echo -e $COLOR_CYAN"*** Optoforce sensor driver ***"$COLOR_RESET
echo -e $COLOR_CYAN"*******************************"$COLOR_RESET


# Add privileges for the user to "dialout" group
sudo usermod -a -G dialout $USER && \
#Change permission to usb port.
sudo touch /etc/udev/rules.d/72-OptoForce.rules && \
sudo echo "ATTR{idVendor}==\"04d8\", ATTR{idProduct}==\"000a\", MODE=\"0666\", GROUP=\"dialout\"" | sudo tee --append  /etc/udev/rules.d/72-OptoForce.rules && \

cd $INSTALL_SCRIPTS_DIR/deps
echo -e $COLOR_BLUE"Installing optoforce dependencies: yaml-cpp..."$COLOR_RESET

echo -e $COLOR_BLUE"Downloading yaml-cpp (0.5) ..."$COLOR_RESET
wget https://github.com/jbeder/yaml-cpp/archive/release-0.5.0.tar.gz && \
tar -zxvf release-0.5.0.tar.gz > /dev/null && \
rm -rf release-0.5.0.tar.gz && \
cd yaml-cpp-release-0.5.0/ && \
mkdir build && \
cd build/ && \
cmake .. > /dev/null && \
echo -e $COLOR_BLUE"Building yaml-cpp (0.5) ..."$COLOR_RESET
make && \
sudo make install > /dev/null


# cd $UR_WS_DIR/src/optoforce/ && \
optoforce_folder_path=$(find $MAIN_WS_DIR/src/ -name 'optoforce' -print | head -n 1) && \
cd $optoforce_folder_path && \

if [ -d "optoforce_driver" ]; then
  rm -rf optoforce_driver
fi

echo -e $COLOR_BLUE"Downloading optoforce drivers..."$COLOR_RESET
git clone https://github.com/tecnalia-medical-robotics/optoforce.git  && \
mv optoforce/ optoforce_driver/ && \
echo -e $COLOR_BLUE"Installing optoforce drivers..."$COLOR_RESET
cd optoforce_driver/  && \
mkdir build  && \
cd build  && \
cmake ../  && \
make
cd .. && \
rm -rf build/


if [ $? -eq 0 ]; then
  echo -e $COLOR_GREEN"Optoforce sensor driver Successfully installed."$COLOR_RESET
  UR_ERROR=0
else
  echo -e $COLOR_RED"Failed to install Optoforce sensor driver."$COLOR_RESET
  UR_ERROR=1
fi

