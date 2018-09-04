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
echo -e $COLOR_CYAN"******   FRI library    *******"$COLOR_RESET
echo -e $COLOR_CYAN"*******************************"$COLOR_RESET

echo -e $COLOR_BLUE"Installing FRILibarry Dependencies: gcc-multilib, g++-multilib..."$COLOR_RESET
# Install dependencies
sudo apt-get update > /dev/null && \
sudo apt-get install -y gcc-multilib g++-multilib > /dev/null && \

# cd $MAIN_WS_DIR/src/autharl_fri && \
fri_folder_path=$(find $MAIN_WS_DIR/src/ -name 'lwr4p' -print | head -n 1) && \
cd $fri_folder_path && \

echo -e $COLOR_BLUE"Downloading FRILibrary..."$COLOR_RESET

if [ -d "FRILibrary" ]; then
  rm -rf FRILibrary/
fi

git clone https://github.com/auth-arl/FRILibrary.git && \
cd FRILibrary && \
echo -e $COLOR_BLUE"Building FRILibrary..."$COLOR_RESET && \
cd Linux && \
mkdir -p x64/debug/bin && \
mkdir -p x64/release/bin && \
mkdir -p x64/debug/lib && \
mkdir -p x64/release/lib && \
mkdir -p x64/debug/obj && \
mkdir -p x64/release/obj && \
mkdir -p x86/debug/bin && \
mkdir -p x86/release/bin && \
mkdir -p x86/debug/lib && \
mkdir -p x86/release/lib && \
mkdir -p x86/debug/obj && \
mkdir -p x86/release/obj && \

# Build the library
make clean all > /dev/null

if [ $? -eq 0 ]; then
  echo -e $COLOR_GREEN"FRIL library successfully installed!"$COLOR_RESET
  AS64_ERROR=0
else
  echo -e $COLOR_RED"FRIL library failed to be built installed..."$COLOR_RESET
  AS64_ERROR=1
fi

