#!/bin/bash

cd $INSTALL_SCRIPTS_DIR/deps

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

cd $INSTALL_SCRIPTS_DIR/deps

echo -e $COLOR_CYAN"******************************"$COLOR_RESET
echo -e $COLOR_CYAN"********    Eigen   **********"$COLOR_RESET
echo -e $COLOR_CYAN"******************************"$COLOR_RESET

echo -e $COLOR_BLUE"Searching for Eigen"$COLOR_RESET
FOUND_EIGEN=`find /usr/local/include/ -name "Eigen" -print`

if [ -n "$FOUND_EIGEN" ]; then
  echo -e $COLOR_GREEN"Found Eigen!"$COLOR_RESET
  UR_ERROR=0
  return
else
  echo -e $COLOR_YELLOW"Didn't find Eigen!"$COLOR_RESET
fi

if [ -d eigen3 ]; then
  rm -rf eigen3
fi

echo -e $COLOR_BLUE"Installing dependencies: fortran..."$COLOR_RESET
printf 'y\n' | sudo apt-get install gfortran &&\

echo -e $COLOR_BLUE"Downloading Eigen (v3.3.4)..."$COLOR_RESET
wget http://bitbucket.org/eigen/eigen/get/3.3.4.tar.gz > /dev/null &&\
tar xvf 3.3.4.tar.gz > /dev/null && \
rm -rf 3.3.4.tar.gz && \
mv eigen-eigen-5a0156e40feb eigen3 && \
cd eigen3 && \
mkdir build && \
cd build && \
cmake .. > /dev/null && \
echo -e $COLOR_BLUE"Installing Eigen Library..."$COLOR_RESET && \
sudo make install > /dev/null

if [ $? -eq 0 ]; then
  echo -e $COLOR_GREEN"Eigen Successfully installed!"$COLOR_RESET
  UR_ERROR=0
else
  echo -e $COLOR_RED"Failed to install Eigen..."$COLOR_RESET
  UR_ERROR=1
fi
