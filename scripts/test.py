#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import copy
from task_allocation.msg import poruka
from std_msgs.msg import String
import yaml

class test(object):
    def __init__(self):
        stream = open("data.yaml", 'r')
        x = yaml.load(stream, Loader=yaml.FullLoader)
        agents = x['agents']
        for agent in agents:
            print(agents[agent]['speed'])
        
        

if __name__ == "__main__":
    rospy.init_node("test", disable_signals=True)
    try:
        node = test()
    except rospy.ROSInterruptException:
        pass