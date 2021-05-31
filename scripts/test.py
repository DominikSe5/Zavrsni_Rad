#!/usr/bin/env python
# -*- coding: utf-8 -*-
from networkx.readwrite.graph6 import data_to_n
import rospy
import copy
from task_allocation.msg import MSmessage, KeyValueString
from std_msgs.msg import String
import yaml

class test(object):
    def __init__(self):
        pub = rospy.Publisher('/varijabla_funkciji', MSmessage, queue_size=5)
        sub = rospy.Subscriber('/varijabla_funkciji', MSmessage, self.callback)
        while not rospy.is_shutdown():
            rospy.sleep(1)
            d = {'prvi': 1, 'drugi': 2, 'treci': 3}
            nova_poruka = MSmessage()
            nova_poruka.sender = 'netko'
            nova_poruka.receiver = 'nesto'
            for key, value in d.items():
                ros_dict = KeyValueString(key, value)
                nova_poruka.dict.append(ros_dict)
            pub.publish(nova_poruka)

    def callback(self, data):
        dict = data.dict
        for i in dict:
            print(i.key)
        

if __name__ == "__main__":
    rospy.init_node("test", disable_signals=True)
    try:
        node = test()
    except rospy.ROSInterruptException:
        pass