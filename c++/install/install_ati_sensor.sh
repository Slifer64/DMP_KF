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


echo -e $COLOR_CYAN"******************"$COLOR_RESET
echo -e $COLOR_CYAN"*** ATI sensor ***"$COLOR_RESET
echo -e $COLOR_CYAN"******************"$COLOR_RESET


echo -e $COLOR_BLUE"Installing ATI Sensor ROS Package..."$COLOR_RESET

cd $MAIN_WS_DIR/src/ft_sensors/ && \

if [ -d ati ]; then
  rm -rf ati
  #echo -e $COLOR_GREEN"ATI Sensor package already exists!"$COLOR_RESET
  #return
fi

# rm -rf ati/ > /dev/null && \
mkdir ati && \
cd ati && \
git clone https://github.com/UTNuclearRoboticsPublic/netft_utils.git && \
# mv netft_utils ati_sensor && \
git clone https://github.com/kuka-isir/ati_sensor.git && \

if [ $? -eq 0 ]; then
  echo -e $COLOR_GREEN"ATI Sensor was successfully installed!"$COLOR_RESET
  AS64_ERROR=0
else
  echo -e $COLOR_RED"Failed to install ATI Sensor..."$COLOR_RESET
  AS64_ERROR=1
fi
