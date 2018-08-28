
Fast Research Interface Wrapper [![CircleCI](https://circleci.com/gh/auth-arl/autharl_fri/tree/master.svg?style=svg&circle-token=a58a50b95c6e7ccf492662cc5c4d209472cbe527)](https://circleci.com/gh/auth-arl/fril_wrapper/tree/master) ![version](https://img.shields.io/badge/version-v0.4-blue.svg) ![ros_version](https://img.shields.io/badge/ROS_release-Indigo-orange.svg)
================================

Wrapper for communication with Kuka LWR4+
using a custom version of Standfords' *[Fast Research Interface Library](http://cs.stanford.edu/people/tkr/fri/html/)*

Dependecies
-------------------------
* Ubuntu 14.04
* ROS Indigo
* *[FRILibrary](https://github.com/auth-arl/FRILibrary.git)*
* *[AUTH ARL Core Library](https://github.com/auth-arl/autharl_core)*

Installation
-------------------------
1.  FRILibrary

    Clone & build the FRILibrary package. [[Instructions](https://github.com/auth-arl/FRILibrary/blob/master/README.md)]
    ```bash
    git clone https://github.com/auth-arl/FRILibrary.git
    cd FRILibrary
    ./build.sh
    ```
2. AUTH ARL Core Library

    Clone autharl_core package in ROS Workspace src directory
    ```bash
    git clone https://github.com/auth-arl/autharl_core
    ```
3. Fril Wrapper

    Clone fril_wrapper package in ROS Workspace src directory
    ```bash
    git clone https://github.com/auth-arl/autharl_fri
    ```
3. Build

    Switch to ROS Workspace root directory & build 
    ```bash
    catkin_make
    ```    
   
Visualization
-------------------------

In order to visualize the robot in rviz you can launch the following file:

```bash
roslaunch lwr_description rviz.launch
```

The launch file also provides the option to visualize the tool in use. Currently the package supports the "handle" tool:

```bash
roslaunch lwr_description rviz.launch tool:=handle
```

