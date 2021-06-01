#!/usr/bin/env python
# -*- coding: utf-8 -*-
from networkx.algorithms.reciprocity import reciprocity
from networkx.classes.graph import Graph
import rospy
import copy
from task_allocation.msg import MSQmessage, MSRmessage, KeyValueString, CheckForNewLayer
import networkx as nx
import yaml
import matplotlib.pyplot as plt
import math
import pprint

class main_hub(object):
    
    def __init__(self):

        self.pr_graph = nx.DiGraph()
        self.create_pr_graph() ## Precedence graph
        self.M = {}
        self.agents_info = self.create_agents_info() ## M(n) is "set of indices of all the function(task) nodes connected to variable node xn in the factor graph"

        self.received = {}
        

        rospy.sleep(4)
        
        self.Qs = {}

        pub = rospy.Publisher("/funkcija_agentu", MSRmessage, queue_size=8, latch=True)
        sub = rospy.Subscriber('/agent_funkciji', MSQmessage, self.callback)
        sub1 = rospy.Subscriber('/newLayer', CheckForNewLayer, self.callbackLayer)
    
        Qs_za_racunanje = {}
        cycle = 1


        while not rospy.is_shutdown():
            rospy.sleep(1)

            tasks_in_current_layer = []
            check = []
            alpha = {}
            alpha_tmp = {}
            n = 0

            pr_graph = copy.deepcopy(self.pr_graph)
            for task in pr_graph.nodes():
                if list(pr_graph.predecessors(task)) == []:
                    if task not in tasks_in_current_layer:
                        tasks_in_current_layer.append(task)

            for agent in self.received:
                if set(tasks_in_current_layer) <= set(self.received[agent]): ## moguci problemi
                    check.append(True)
            
            print("R checks", check)

            if all(check) and len(check) == len(list(self.received.keys())) and len(check) != 0:
                
                Qs_za_racunanje = copy.deepcopy(self.Qs)                                             
                agenti_kojima_saljemo = {}                                                                      
                for agent in self.M:
                    for task in tasks_in_current_layer:
                        if task in self.M[agent] and task not in list(agenti_kojima_saljemo.keys()):
                            agenti_kojima_saljemo[task] = [agent]
                        elif task in self.M[agent]:
                            agenti_kojima_saljemo[task].append(agent)

                for task in tasks_in_current_layer:

                    f = self.f_of_task(task, agenti_kojima_saljemo[task])
                    suma = [0, 0]
                    for agent in agenti_kojima_saljemo[task]:
                        R = self.calc_R(f, agent, alpha, Qs_za_racunanje, agenti_kojima_saljemo[task], task)
                        msg = MSRmessage()
                        msg.data = R
                        msg.receiver = agent
                        msg.sender = task
                        self.received[agent].clear()
                        pub.publish(msg)

    def callbackLayer(self, data):
        tasks = data.distributedTasks
        for task in tasks:
            self.pr_graph.remove_node(task)
            for agent in self.M:
                self.M[agent].remove(task)
        self.received.clear()


    def callback(self, data):
        self.Qs['{}, {}'.format(data.sender, data.receiver)] = data.dict
        if data.sender in self.received:
            if data.receiver not in self.received[data.sender]:
                self.received[data.sender].append(data.receiver)
        else:
            self.received[data.sender] = [data.receiver]

    def calc_R(self, f, agent_primatelj, alpha, Qs, agenti_kojima_saljemo, task):
        sum_Qs = {}
        
        for agent in agenti_kojima_saljemo:
            if agent != agent_primatelj:
                sum_Qs[agent] = Qs['{}, {}'.format(agent, task)]

        for i1 in range(0, pow(2, len(agenti_kojima_saljemo))):            
            temp = [1 if i1 & (1 << (7-n)) else 0 for n in range(8)] ##pogledaj funkciju calc_U za objasnjenje ove sekcije koda --
            while len(temp) != len(agenti_kojima_saljemo): ## --
                if temp[0] == 0: ## --
                    del temp[0] ## --
            for bit in temp:
                if bit == 1:
                    agent_koji_radi = agenti_kojima_saljemo[temp.index(bit)]
                    if agent_koji_radi != agent_primatelj:
                        for i in sum_Qs[agent_koji_radi]:
                            if i.key == task:
                                f[i1] += i.value
        return min(i for i in f if i != 0)



    def calc_m(self, Cj, duration, bits):
        m = []
        max_stl = 0
        finish_time = {}
        for i2 in range(0, pow(2, len(Cj))):
            temp = bits[i2]
            for agent in Cj:
                stl = self.agents_info[agent][2]
                if len(list(stl.nodes())) == 0:
                    finish_time[agent] = 0
                else:
                    for edge in list(stl.edges()):
                        if agent not in list(finish_time.keys()):
                            finish_time[agent] = stl.edges[edge]['duration'] 
                        else:
                            finish_time[agent] += stl.edges[edge]['duration'] 
                if temp[Cj.index(agent)] == 1:
                    if max_stl <= finish_time[agent]:
                        max_stl = finish_time[agent]
            num_of_agents = temp.count(1)
            if num_of_agents == 0:
                m.append(0)
            else:
                m.append(0.7 * ((duration / num_of_agents) + max_stl))
        return m

    def f_of_task(self, task, Cj):
        Cj.sort()
        bits = {}
        tt = []
        tt_sum = 0
        f = []
        duration = self.pr_graph.nodes[task]['duration']
        for i1 in range(0, pow(2, len(Cj))):
            temp = [1 if i1 & (1 << (7-n)) else 0 for n in range(8)]
            while len(temp) != len(Cj):
                if temp[0] == 0:
                    del temp[0]
            bits[i1] = temp
        for i2 in range(0, pow(2, len(Cj))):
            temp = bits[i2]
            for agent in Cj:
                if temp[Cj.index(agent)] == 1:
                    speed_of_agent = self.agents_info[agent][0]
                    p1 = self.pr_graph.nodes[task]['location']
                    p2 = self.agents_info[agent][1]
                    distance = math.sqrt( ((p1[0]-p2[0])**2)+((p1[1]-p2[1])**2) )
                    tt_sum += 0.3 * distance / speed_of_agent
            tt.append(tt_sum)
            tt_sum = 0
        m = self.calc_m(Cj, duration, bits)
        output = [x + y for x, y in zip(m, tt)]
        return output     

    def create_pr_graph(self):
        stream = open("/home/dominik/catkin_ws/src/task_allocation/scripts/data1.yaml", 'r')
        dict = yaml.load(stream, Loader=yaml.FullLoader)
        tasks = dict['tasks']
        while list(self.pr_graph.nodes()) != list(tasks.keys()):
            for task in tasks:
                self.pr_graph.add_node(task)
                for predecessor in tasks[task]['ordering_constraint']:
                    self.pr_graph.add_edge(predecessor, task)
        for task in self.pr_graph.nodes():
            self.pr_graph.nodes[task]['duration'] = tasks[task]['duration']
            self.pr_graph.nodes[task]['location'] = tasks[task]['location']

    def create_agents_info(self):
        stream = open("/home/dominik/catkin_ws/src/task_allocation/scripts/data1.yaml", 'r')
        dict = yaml.load(stream, Loader=yaml.FullLoader)
        agents = dict['agents']
        agents_info = {}
        for agent in agents:
            stl = nx.DiGraph()
            self.M[agent] = agents[agent]['resources']
            agents_info[agent] = [agents[agent]['speed'], agents[agent]['location'], stl]
        return agents_info

if __name__ == "__main__":
    rospy.init_node("Main_Hub")
    try:
        node = main_hub()
    except rospy.ROSInterruptException:
        pass