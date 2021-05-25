#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import copy
from task_allocation.msg import poruka
from std_msgs.msg import String
import yaml

class test(object):
    def __init__(self):
        sub = rospy.Subscriber('/varijabla_funkciji', poruka, self.callback)
        rospy.spin()

    def callback(self, data):
        print(data.data)

        
        

if __name__ == "__main__":
    rospy.init_node("test", disable_signals=True)
    try:
        node = test()
    except rospy.ROSInterruptException:
        pass