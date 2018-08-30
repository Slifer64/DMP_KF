#!/usr/bin/env python
"""
@package optoforce_ros
@file plot_contact.py
@author Anthony Remazeilles and Asier Fernandez
@brief see README.md
Copyright Tecnalia Research and Innovation 2016
Distributed under the GNU GPL v3. For full terms see https://www.gnu.org/licenses/gpl.txt
"""
import roslib

import numpy as np
import rospy

import os
import sys,getopt
import matplotlib.pyplot as plt

import rosbag_utilities

bag_name   = None
topic_name = None 

try:
    opts, args = getopt.getopt(sys.argv[1:],'hb:t:',['help', 'bag=', 'topic-name='])
except getopt.GetoptError:
    print 'plot_contact.py -h -b <bagfile> -t <topic-name>'
    sys.exit(2)

for opt, arg in opts:
    if opt == '-h':
        print 'plot_contact.py -b <bagfile> -t <topic-name>'
        sys.exit()
    elif opt in ("--bag", "-b"):
        bag_name = arg
    elif opt in ("--topic-name", "-t"):
        topic_name = arg

if bag_name is None:
    print 'No bag file defined'
    print 'plot_contact.py -b <bagfile>'
    sys.exit(2)

if topic_name is None:
    print 'No topic-name defined'
    print 'plot_contact.py -t <topic-name>'
    sys.exit(2)

print "bag is  : %s"%bag_name
print "topic is: %s"%topic_name

# next two lines only for debuging
#bag_name   = '../bags/contact_estimation_example_2016-11-02-11-08-59.bag'
#topic_name = '/optoforce_IRE004'

data = rosbag_utilities.extract_data_from_bag(bag_name,[topic_name])

print "The {} topic contains {} lines ".format(topic_name, len(data[topic_name][0]))


# Intialize variables
Fx = np.zeros(len(data[topic_name][0]))
Fy = np.zeros(len(Fx))
Fz = np.zeros(len(Fx))
Tx = np.zeros(len(Fx))
Ty = np.zeros(len(Fx))
Tz = np.zeros(len(Fx))


# extract Force and Torque data
time = np.asarray(data[topic_name][0])
Fx_lst = [item.wrench.force.x for item in data[topic_name][1]]
Fy_lst = [item.wrench.force.y for item in data[topic_name][1]]
Fz_lst = [item.wrench.force.z for item in data[topic_name][1]]
Tx_lst = [item.wrench.torque.x for item in data[topic_name][1]]
Ty_lst = [item.wrench.torque.y for item in data[topic_name][1]]
Tz_lst = [item.wrench.torque.z for item in data[topic_name][1]]

# convert from list to numpy array
Fx = np.asarray(Fx_lst)
Fy = np.asarray(Fy_lst)
Fz = np.asarray(Fz_lst)

Tx = np.asarray(Tx_lst)
Ty = np.asarray(Ty_lst)
Tz = np.asarray(Tz_lst)

# delete unnecesary variables
del Fx_lst, Fy_lst, Fz_lst, Tx_lst, Ty_lst, Tz_lst
del data

# Define limite for displacement of contact positions
d_lim = 0.03

# calculate conctact point
# Tx = dy.Fz - dz.Fy
# Ty = dz.Fx - dx.Fz
# Tz = dx.Fy - dy.Fx
dz = 0.0
dx = (dz*Fx - Ty) / Fz;
dy = (dz*Fy + Tx) / Fz;

# filter points above d_lim. The division of T/F can increase a lot.
mask = (dx>d_lim)|(dx<-d_lim)
dx[mask] = 0
dy[mask] = 0


# plot contact point estimation
fig = plt.figure()

plt_dx = fig.add_subplot(2,1,1)
plt_dy = fig.add_subplot(2,1,2)

plt_dx.set_xlabel("Time [sec]")
plt_dy.set_xlabel("Time [sec]")

plt_dx.set_ylabel("Displacement [m]")
plt_dy.set_ylabel("Displacement [m]")

plt_dx.set_title("Displacement in X")
plt_dy.set_title("Displacement in Y")

plt_dx.set_ylim([-d_lim, d_lim])
plt_dy.set_ylim([-d_lim, d_lim])

plt_dy.plot(time,dy, 'g.', label='dy (m)', lw=1)
plt_dx.plot(time,dx, 'r.', label='dx (m)', lw=1)

plt_dx.legend()
plt_dy.legend()

plt.show()