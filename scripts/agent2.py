#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from task_allocation.msg import poruka

class agent2(object):
    def __init__(self):
        self.M = ['funkcija1', 'funkcija2', 'funkcija3']
        self.gamma = [-0.1, 0.1]
        self.pub = rospy.Publisher("/varijabla_funkciji", poruka, queue_size = 5, latch=True)
        self.sub = rospy.Subscriber("/varijabla2/funkcija_varijabli", poruka, self.callback)
        self.Rs = {'funkcija1': [0, 0], 'funkcija2': [0, 0], 'funkcija3': [0, 0]}
        rospy.spin()

    def callback(self, data):
        print("Agent2 je primio poruku")
        self.Rs[data.posiljatelj] = data.data
        for funkcija_kojim_saljemo in self.M:
            poruka_funkciji = poruka()
            poruka_funkciji.posiljatelj = 'varijabla2'
            poruka_funkciji.primatelj = funkcija_kojim_saljemo
            poruka_funkciji.data = self.Poruka_v_f(funkcija_kojim_saljemo)
            self.pub.publish(poruka_funkciji)

    def Poruka_v_f(self, f):
        if f in self.M:
            out = [0, 0]
            for primatelj, vrijednost in self.Rs.items():
                if primatelj != f:
                    out[0] += vrijednost[0]
                    out[1] += vrijednost[1]
        return out

if __name__ == "__main__":
    rospy.init_node("agent2")
    try:
        node = agent2()
    except rospy.ROSInterruptException:
        pass