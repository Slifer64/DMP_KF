FROM ros:indigo

MAINTAINER Iason Sarantopoulos <iasons@iti.gr>

RUN apt-get update && apt-get install -y build-essential
RUN rm /bin/sh && ln -s /bin/bash /bin/sh

# Create catkin workspace
ENV CATKIN_WS=/root/catkin_ws
RUN source /opt/ros/indigo/setup.bash && \
  mkdir -p $CATKIN_WS/src && cd $CATKIN_WS/src && catkin_init_workspace && \
  cd $CATKIN_WS && catkin_make && source $CATKIN_WS/devel/setup.bash

# Build the code, run tests and run linter
ENTRYPOINT source /opt/ros/indigo/setup.bash && \
  cd $CATKIN_WS && \
  rosdep install --from-paths src/ --ignore-src --rosdistro indigo -y && \
  catkin_make && \
  catkin_make run_tests && catkin_test_results && \
  catkin_make roslint_autharl_core
