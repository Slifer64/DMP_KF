#!/bin/bash

set -e

MESSAGE="Installing MuJoCo"; blue_echo

MESSAGE="Downloading and building a local copy of the MuJoCo (v1.50)"; blue_echo

cd $MAIN_WS_DIR/deps

if [ -d mjpro150 ]; then
  rm -rf mjpro150
fi

wget https://www.roboti.us/download/mjpro150_linux.zip
unzip mjpro150_linux.zip > /dev/null
rm -rf mjpro150_linux.zip

cd $MAIN_WS_DIR/deps/mjpro150/bin
ln -s libglfw.so.3 libglfw.so

cd $MAIN_WS_DIR

if [ $? -eq 0 ]; then
  MESSAGE="MuJoCo Successfully installed."; green_echo
  AS64_ERROR=0
else
  MESSAGE="Failed to install MuJoCo"; red_echo
  AS64_ERROR=1
fi
