# Change Log

## v0.4.1 (2017-05-16)
* New features
  - Read and send joint velocities to the robot in the base class, in order to
    avoid replicating the same code in the wrappers.
  - Add support in RobotSim for sending joint positions to all the main chains of the robot.
  - Add support for visualizing vectors in RViz.
  - Add support for getting the global index of a joint in the Robot model among every other main joint.
  - Add trajectory generation for the orientation.
  - Add license.
  - Improve CI.

* Fixes
  - Remove constant functions from API in order to simplify the code.

## v0.4 (2017-04-28)
* Breaking changes
  - Refactor produced libraries (You should change the linking into you CMakeLists):
    - Change the name of the `misc` library to `utils`.
    - Move mappings to `utils` library.
  - Refactor the constructors of the Controller class
  - Remove `setParams()` function from controller.
  - Remove variables for joint names etc from Model class due to adding getters
    functions. Now you have to use these getters functions.

* New features
  - Add new RobotSim class for kinematic simulations
  - Add documentation
  - Add functionalities in `Model` class for reading the names of the joint/chains and
    the number of joints/chains. Also add functionalities for reading the joint
    limits of the robot.
  - Add visualization in RViz for cylinders, spheres, frames and text. Frames
    can be also visualized in real time in the visualization thread of the
    robot.
  - Add functions for getting screw transformations.

## v0.3 (2017-04-06)
* Breaking changes
  - Change the robot model to have solvers as the number of chains
  - Change the name of the functions for the API of arl::robot

* New features
  - Add new library for visualization of paths, spheres and the robot in RViz
  - Add general header files (you can include `robot.h` instead of `robot/controller.h`, `robot/model.h` etc)
  - Add new API classes for robot controllers

## v0.2 (2017-03-27)
* Fix mappings KDL to Armadillo improving the perfomance
* Remove pure virtual functions from robot API and make them virtual and print
  error when a function is not implemented by the interface
* Add trajectories library
* Add orientation and quaternion tools

## v0.1 (2017-03-21)
* Add basic API for robots
* Add mappings from KDL to Armadillo
* Add math tools for testing rotation metrices, finding skew symmetrics etc
