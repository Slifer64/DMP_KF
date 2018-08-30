# Description
This folder contains different test programs

### plot_contact.py
This util has been develop to easily verify that OptoForce sensor readings are ok.
The python script reads a bag file, calculates contact position when a vertical force is applied on top of OptoForce sensor, and plots the result

#### Basic usage
The ROS topic to be recorded must be **geometry_msgs/WrenchStamped** format.

##### Run optoforce node
Instructions to launch optoforce node can be found in the main [README](../README.md)

##### Create a rosbag file
```bash
rosbag record /topic_name -o bagfile
```

##### Execute script to plot contact position
```bash
python plot_contact.py -b <bagfile> -t <topic-name>
```