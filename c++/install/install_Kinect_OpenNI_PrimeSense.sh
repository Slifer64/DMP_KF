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

FAILURE=0

echo -e $COLOR_CYAN"*************************************************"$COLOR_RESET
echo -e $COLOR_CYAN"*** Kinect - OpenNI - PrimeSense installation ***"$COLOR_RESET
echo -e $COLOR_CYAN"*************************************************"$COLOR_RESET

# Here’s an overview of the process to get the OpenNI and PrimeSense drivers working with the Kinect and Ubuntu. 

echo -e $COLOR_BLUE"Installing dependencies..."$COLOR_RESET

# ========================================
# Begin by installing some dependencies:
# ========================================
sudo apt-get install git-core cmake freeglut3-dev pkg-config build-essential libxmu-dev libxi-dev libusb-1.0-0-dev default-jdk

if [ $? -ne 0 ]; then
  echo -e $COLOR_RED"Failed to install some package dependencies..."$COLOR_RESET
  FAILURE=1
fi 

# ===================
# These are optional
# ===================
sudo apt-get install doxygen graphviz mono-complete

if [ $? -ne 0 ]; then
  echo -e $COLOR_YELLOW"Failed to install some optional dependencies..."$COLOR_RESET
fi 

echo -e $COLOR_BLUE"Downloading and installing OpenNI..."$COLOR_RESET

# ========================================
# Make a directory to store the build, then clone the OpenNI source from Github.
# ========================================
mkdir ~/kinect && \
cd ~/kinect && \
git clone https://github.com/OpenNI/OpenNI.git && \

# ========================================================================================
# Run the RedistMaker script in the Platform/Linux folder and install the output binaries
# ========================================================================================
cd OpenNI/Platform/Linux/CreateRedist/ && \
chmod +x RedistMaker && \
./RedistMaker && \
cd ../Redist/OpenNI-Bin-Dev-Linux-x64-v1.5.2.23/ && \
sudo ./install.sh && \

if [ $? -ne 0 ]; then
  echo -e $COLOR_RED"Failed to install OpenNI..."$COLOR_RESET
  FAILURE=1
fi

echo -e $COLOR_BLUE"Downloading and installing SensorKinect..."$COLOR_RESET

# =====================================================
#Next, clone the Avin2 SensorKinect source from Github.
# =====================================================
cd ~/kinect/ && \
git clone git://github.com/avin2/SensorKinect.git && \

# Run the RedistMaker script in the Platform/Linux folder and install the output binaries.
cd SensorKinect/Platform/Linux/CreateRedist/ && \
chmod +x RedistMaker && \
./RedistMaker && \
cd ../Redist/Sensor-Bin-Linux-x64-v5.1.2.1/ && \
chmod +x install.sh && \
sudo ./install.sh && \

if [ $? -ne 0 ]; then
  echo -e $COLOR_RED"Failed to install SensorKinect..."$COLOR_RESET
  FAILURE=1
fi

echo -e $COLOR_BLUE"Downloading and installing NITE..."$COLOR_RESET

# ===================================================================================================
# Then download the OpenNI Compliant Middleware Binaries to ~/kinect Select these options from the dropdown menus: 
# Unstable PrimeSense NITE Unstable Build for Ubuntu 10.10 x64 (or x86 if your Ubuntu is 32bits) v 1.5.2.21 
# Extract the contents of the archive and switch to the Data directory contained within.
# ===================================================================================================
cd ~/kinect && \
wget http://www.openni.ru/wp-content/uploads/2013/10/NITE-Bin-Linux-x64-v1.5.2.23.tar.zip
unzip NITE-Bin-Linux-x64-v1.5.2.23.tar.zip -d ./ > /dev/null && \
rm -rf NITE-Bin-Linux-x64-v1.5.2.23.tar.zip
tar -xvjpf NITE-Bin-Linux-x64-v1.5.2.23.tar.bz2 > /dev/null && \
rm -rf NITE-Bin-Linux-x64-v1.5.2.23.tar.bz2
# Now modify the license in the files: Sample-Scene.xml, Sample-Tracking.xml, and Sample-User.xml. 
# Change <License vendor=”PrimeSense” key=”"/> to:
# <License vendor=”PrimeSense” key=”0KOIk2JeIBYClPWVnMoRKn5cdY4=”/>
grep -rl "insert key here" ./NITE-Bin-Dev-Linux-x64-v1.5.2.23/Data/ | xargs sed -i "s/insert key here/0KOIk2JeIBYClPWVnMoRKn5cdY4=/g"
# Go to the NITE directory and run the install script.
cd NITE-Bin-Dev-Linux-x64-v1.5.2.23 && \
chmod +x install.sh && \
sudo ./install.sh && \

if [ $? -ne 0 ]; then
  echo -e $COLOR_RED"Failed to install NITE..."$COLOR_RESET
  FAILURE=1
fi

if [ $FAILURE -eq 1 ]; then
  echo -e $COLOR_RED"Failed to install some packages..."$COLOR_RESET
  FAILURE=1
else
  echo -e $COLOR_GREEN"Successful installation of Kinect - OpenNI - PrimeSense!"$COLOR_RESET
fi


#That’s it! If you followed steps through to here you should be able to run the sample applications.
# The OpenNI samples are here: ~/kinect/OpenNI/Platform/Linux/Bin/x64-Release 
# and the PrimeSense samples are here: ~/kinect/NITE-Bin-Dev-Linux-x64-v1.5.2.21/Samples/Bin/x64-Release

# Note: if you get the error: InitFromXml failed: Failed to set USB interface! the solution is to remove the gspca_kinect kernel module:
# sudo rmmod gspca_kinect

# ======================================
# Install some ros related packages
# ======================================
echo -e $COLOR_BLUE"Installing some ros related packages..."$COLOR_RESET
# for Kinect
sudo apt-get install ros-indigo-openni-launch
sudo apt-get install ros-indigo-freenect-launch

# for Asus Xtion
sudo apt-get install ros-indigo-openni2-launch
