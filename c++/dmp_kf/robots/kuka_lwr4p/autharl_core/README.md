# AUTH-ARL Core [![CircleCI](https://circleci.com/gh/auth-arl/autharl_core/tree/master.svg?style=svg&circle-token=9cca80b6cef927320b717a247c93b8ee0e0409ab)](https://circleci.com/gh/auth-arl/autharl_core/tree/master) ![version](https://img.shields.io/badge/version-v0.4.1-blue.svg) ![ros_version](https://img.shields.io/badge/ROS_release-Indigo-orange.svg)

This repository stores the core C++ library of AUTH-ARL. Its purpose is to have
a central codebase that can be re-used in multiple (robotics) C++ projects.

We use the [Semantic Versioning Specifications](http://semver.org/) for release numbering.

Here you can find the [change log](CHANGELOG.md).

Dependecies
-------------------------
* ROS indigo
* *[Orocos KDL](http://www.orocos.org/kdl)*
* *[Armadillo](http://arma.sourceforge.net/)*

Installation
-------------------------
Clone repository to catkin workspace src directory
```bash
git clone https://github.com/auth-arl/autharl_core
```

Install the dependencies:

```bash
cd $YOUR_CATKIN_WORKSPACE
rosdep install --from-paths src/ --ignore-src --rosdistro indigo -y
```

Build the library from catkin workspace root directory
```bash
catkin_make
```

Documentation
-------------------------
There is no online available documentation but you can create it locally by
yourself. Install `rosdoc_lite` package and use it for producing the documentation:

```bash
sudo apt-get install ros-indigo-rosdoc-lite
roscd autharl_core
rosdoc_lite .
```

Access the documentation by opening this file in your browser:

```bash
gnome-open autharl_core/doc/html/index.html
```

Usage
-------------------------
This library is designed for use with a new robot wrapper library.
For detailed information see wiki.
