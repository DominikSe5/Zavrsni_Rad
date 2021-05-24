#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import copy
from task_allocation.msg import poruka

class agent3(object):
    def __init__(self):
        self.cycle_limit = 25
        self.convergence_check = []
        self.M = ['funkcija2', 'funkcija3']
        self.ready_check = []
        self.gamma = [-0.1, 0.1]
        self.pub = rospy.Publisher("/varijabla_funkciji", poruka, queue_size = 3, latch=True)
        self.sub = rospy.Subscriber("/varijabla3/funkcija_varijabli", poruka, self.callback)
        self.Rs = {'funkcija2': [0, 0], 'funkcija3': [0, 0]}
        rospy.spin()

    def callback(self, data):
        self.Rs[data.posiljatelj] = data.data
        if data.posiljatelj not in self.ready_check:
            self.ready_check.append(data.posiljatelj)
        if set(self.M) <= set(self.ready_check):
            R = copy.deepcopy(self.Rs)
            for funkcija_kojim_saljemo in self.M:
                poruka_funkciji = poruka()
                poruka_funkciji.posiljatelj = 'varijabla3'
                poruka_funkciji.primatelj = funkcija_kojim_saljemo
                poruka_funkciji.data = self.Poruka_v_f(funkcija_kojim_saljemo)
                self.pub.publish(poruka_funkciji)
            Z = self.calc_Z(R)
            self.convergence_func(Z)
            self.ready_check.clear()

    def convergence_func(self, Z):
        self.cycle_limit += -1
        if Z[0] >= Z[1]:
            self.convergence_check.append(0)
        else:
            self.convergence_check.append(1)
        if len(self.convergence_check) > 5:
            temp = self.convergence_check[1:]
            self.convergence_check = temp
        print('Z3 =',Z)
        if self.cycle_limit == 0 or (self.cycle_limit < 15 and len(self.convergence_check) == 5 and all(x == self.convergence_check[0] for x in self.convergence_check)):
            print('Agent 3 se odlucio za stanje {}'.format(self.convergence_check[-1]))
            rospy.signal_shutdown('Kraj')

    def Poruka_v_f(self, f):
        if f in self.M:
            out = [0, 0]
            for primatelj, vrijednost in self.Rs.items():
                if primatelj != f:
                    out[0] += vrijednost[0]
                    out[1] += vrijednost[1]
        return out

    def calc_Z(self, R):
        sum = [0, 0]
        for func in R:
            sum[0] += R[func][0]
            sum[1] += R[func][1]
        return sum

if __name__ == "__main__":
    rospy.init_node("agent3", disable_signals=True)
    try:
        node = agent3()
    except rospy.ROSInterruptException:
        pass