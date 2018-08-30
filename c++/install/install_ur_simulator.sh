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


echo -e $COLOR_CYAN"**********************************"$COLOR_RESET
echo -e $COLOR_CYAN"*** Universal Robots simulator ***"$COLOR_RESET
echo -e $COLOR_CYAN"**********************************"$COLOR_RESET


cd ~/

if [ ! -d "universal_robots" ]; then
  mkdir universal_robots
fi

cd universal_robots && \

if [ -d "ursim-3.5.4.10845" ]; then
  echo -e $COLOR_GREEN"Universal Robots simulator already exists!"$COLOR_RESET
  UR_ERROR=0
  return
fi

echo -e $COLOR_BLUE"Downloading simulator..."$COLOR_RESET
wget https://s3-eu-west-1.amazonaws.com/ur-support-site/38720/Ursim_Linux-3.5.4.10845.tar.gz && \
tar -zxvf Ursim_Linux-3.5.4.10845.tar.gz > /dev/null && \
rm -rf Ursim_Linux-3.5.4.10845.tar.gz && \
rm Software_licence_agreement.doc
cd ursim-3.5.4.10845/ && \
echo -e $COLOR_BLUE"Installing simulator..."$COLOR_RESET && \
sed -i -e 's/versionString\=/versionString\=5 \#/g' ./install.sh
./install.sh


echo -e $COLOR_GREEN"Universal Robots simulator successfully installed!"$COLOR_RESET
UR_ERROR=0
#if [ $? -eq 0 ]; then
#  echo -e $COLOR_GREEN"Universal Robots simulator successfully installed!"$COLOR_RESET
#  UR_ERROR=0
#else
#  echo -e $COLOR_RED"Universal Robots simulator installation failed!"$COLOR_RESET
#  UR_ERROR=1
#fi

