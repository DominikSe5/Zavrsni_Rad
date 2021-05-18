#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from task_allocation.msg import poruka

class initiator(object):
    def __init__(self):
        self.M = {
            'agent1' : ['agent1', 'agent2'],
            'agent2' : ['agent1', 'agent2', 'agent3'],
            'agent3' : ['agent2', 'agent3']
            }
        msg = poruka()
        self.pub = rospy.Publisher("/varijabla_funkciji", poruka, queue_size = 1)
        for i in self.M:
            msg.posiljatelj = i
            for j in self.M[i]:
                msg.data = [0, 0]
                msg.primatelj = j
                self.pub.publish(msg)
if __name__ == "__main__":
    rospy.init_node("initiaotor")
    try:
        node = initiator()
    except rospy.ROSInterruptException:
        pass