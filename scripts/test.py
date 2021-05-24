#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import copy
from task_allocation.msg import poruka
from std_msgs.msg import String

class test(object):
    def __init__(self):
        self.pub = rospy.Publisher("/poruka", String, queue_size = 3)
        self.brojanje = 0
        while not rospy.is_shutdown():
            rospy.sleep(1)
            poruka = String()
            poruka.data = 'Radim'
            self.pub.publish(poruka)
            self.brojanje += 1
            if self.brojanje == 10:
                poruka = String()
                poruka.data = 'Ne radim'
                self.pub.publish(poruka)
                rospy.signal_shutdown('Kraj')

if __name__ == "__main__":
    rospy.init_node("test", disable_signals=True)
    try:
        node = test()
    except rospy.ROSInterruptException:
        pass