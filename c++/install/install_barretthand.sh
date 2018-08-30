#!/usr/bin/env bash

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


echo -e $COLOR_CYAN"*******************"$COLOR_RESET
echo -e $COLOR_CYAN"*** BarrettHand ***"$COLOR_RESET
echo -e $COLOR_CYAN"*******************"$COLOR_RESET

echo -e $COLOR_BLUE"Installing BarrettHand Dependencies: codeblocks build-essential wxgtk2.8-dev freeglut3-dev libpopt-dev libpoco-dev wget..."$COLOR_RESET
sudo apt-get update > /dev/null
sudo apt-get install -y codeblocks build-essential wxgtk2.8-dev freeglut3-dev libpopt-dev libpoco-dev wget > /dev/null

#
echo -e $COLOR_CYAN"Download and install the PCAN Driver for BarrettHand..."$COLOR_RESET
wget http://www.peak-system.com/fileadmin/media/linux/files/peak-linux-driver-7.15.2.tar.gz
tar xvf peak-linux-driver-7.15.2.tar.gz > /dev/null
cd peak-linux-driver-7.15.2
make NET=NO_NETDEV_SUPPORT
sudo make install
sudo echo <<EOF KERNEL=="pcanusb*", NAME="pcanusb/%n", SYMLINK+="%k", GROUP="plugdev" | sudo tee -a /etc/udev/rules.d/45-pcan.rules
EOF
sudo modprobe pcan
cd ..
rm -rf peak-linux-driver-7.15.2.tar.gz
rm -rf peak-linux-driver-7.15.2
cd ..

if [ $? -eq 0 ]; then
  echo -e $COLOR_GREEN"BarrettHand drivers successfully installed."$COLOR_RESET
  cd ..
else
  echo -e $COLOR_RED"BarrettHand drivers failed to install."$COLOR_RESET
  cd ..
  exit 1
fi
