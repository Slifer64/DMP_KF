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

cd $INSTALL_SCRIPTS_DIR/deps

echo -e $COLOR_CYAN"*******************************"$COLOR_RESET
echo -e $COLOR_CYAN"********   Armadillo   ********"$COLOR_RESET
echo -e $COLOR_CYAN"*******************************"$COLOR_RESET

echo -e $COLOR_BLUE"Searching for Armadillo"$COLOR_RESET
FOUND_ARMADILLO=`find /usr/lib/x86_64-linux-gnu/ -name "libarmadillo.so" -print`

if [ -n "$FOUND_ARMADILLO" ]; then
  echo -e $COLOR_GREEN"Found Armadillo!"$COLOR_RESET
  UR_ERROR=0
  return
else
  echo -e $COLOR_YELLOW"Didn't find Armadillo!"$COLOR_RESET
fi

echo -e $COLOR_BLUE"Installing Armadillo Dependencies: cmake, OpenBLAS and LAPACK, wget, xz-utils..."$COLOR_RESET
sudo apt-get update > /dev/null && \
sudo apt-get install -y cmake libopenblas-dev liblapack-dev wget xz-utils > /dev/null && \

echo -e $COLOR_BLUE"Downloading and building the Armadillo (v8.300)"$COLOR_RESET

if [ -d armadillo-8.300.1 ]; then
  rm -rf armadillo-8.300.
fi

wget --no-check-certificate http://sourceforge.net/projects/arma/files/armadillo-8.300.1.tar.xz > /dev/null && \
tar xvf armadillo-8.300.1.tar.xz > /dev/null && \
rm -rf armadillo-8.300.1.tar.xz && \
cd armadillo-8.300.1 && \
echo -e $COLOR_BLUE"Building Armadillo"$COLOR_RESET && \
cmake . && \
make && \
echo -e $COLOR_BLUE"Installing Armadillo"$COLOR_RESET && \
sudo make install > /dev/null

if [ $? -eq 0 ]; then
  echo -e $COLOR_GREEN"Armadillo Successfully installed!"$COLOR_RESET
  UR_ERROR=0
else
  echo -e $COLOR_RED"Armadillo installation failed!"$COLOR_RESET
  UR_ERROR=1
fi
