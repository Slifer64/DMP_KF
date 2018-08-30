#!/usr/bin/env python
import roslib
#roslib.load_manifest('kdl')
import PyKDL as kdl

import rosbag
import os
import sys,getopt
import matplotlib.pyplot as plt
import matplotlib

def extract_data_from_bag(bag_name, list_topics):
    bag = rosbag.Bag( bag_name)

    data = dict()

    #todo make sure these default data are not in the package already
    data['tinit'] = None
    data['tfinal'] = None

    for topic in list_topics:
        data[topic] = [[],[]]

    for topic, msg, t in bag.read_messages( topics=list_topics):
        if not data['tinit']:
            data['tinit'] = t

        t_cur =(t - data['tinit']).to_sec()
        data['tfinal'] = t_cur

        data[topic][0].append(t_cur)
        data[topic][1].append(msg)

    bag.close()
    return data
