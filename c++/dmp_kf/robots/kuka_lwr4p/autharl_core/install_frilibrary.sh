#!/bin/bash

MESSAGE="Installing FRILibrary..."; blue_echo && \

MESSAGE="Installing FRILibarry Dependencies: gcc-multilib, g++-multilib..."; blue_echo && \
# Install dependencies
sudo apt-get update > /dev/null
sudo apt-get install -y gcc-multilib g++-multilib > /dev/null

cd $AUTHARL_CORE_DIR/lwr
MESSAGE="Downloading and building a local copy of the FRILibrary..."; blue_echo && \

if [ -d "FRILibrary" ]; then
  rm -rf FRILibrary/
fi

git clone https://github.com/auth-arl/FRILibrary.git

cd FRILibrary

MESSAGE="Building FRILibrary"; blue_echo && \
cd Linux
mkdir -p x64/debug/bin
mkdir -p x64/release/bin
mkdir -p x64/debug/lib
mkdir -p x64/release/lib
mkdir -p x64/debug/obj
mkdir -p x64/release/obj
mkdir -p x86/debug/bin
mkdir -p x86/release/bin
mkdir -p x86/debug/lib
mkdir -p x86/release/lib
mkdir -p x86/debug/obj
mkdir -p x86/release/obj

# Build the library
make clean all > /dev/null

if [ $? -eq 0 ]; then
  MESSAGE="FRIL library successfully installed."; green_echo
  AUTHARL_ERROR=0
else
  echo "FRIL library failed to be built installed."
  AUTHARL_ERROR=1
fi
