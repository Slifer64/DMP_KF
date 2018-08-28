/**
 * Copyright (C) 2017 as64_
 */

#include <ros/ros.h>
#include <ros/package.h>

#include <memory>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>

#include <OL_2D_rup/OL_2D_rup.h>


int main(int argc, char** argv)
{
  #ifdef CATCH_EXCEPTIONS
  try{
  #endif

  // Initialize the ROS node
  ros::init(argc, argv, "OL_2D_rup");

  // =========  Create controller instance for the ur10 robot  =========
  std::unique_ptr<OL_2D_rup> controller(new OL_2D_rup());

  // =========  Main loop running the controller  =========
  controller->execute();

  std::cout << "[MAIN]: Exited loop...\n";

  controller->finalize();

  ROS_INFO_STREAM("OL_2D_rup node is going down.");

  // Shutdown ROS node
  ros::shutdown();

  #ifdef CATCH_EXCEPTIONS
  }
  catch(std::exception &e)
  {
  std::cerr << as64_::io_::red << as64_::io_::bold << e.what() << as64_::io_::reset << "\n";
  }
  #endif

  return 0;
}
