#!/usr/bin/env python
# -*- coding: utf-8 -*-
from networkx.classes.graph import Graph
import rospy
import copy
from task_allocation.msg import MSmessage, KeyValueString
import networkx as nx
import yaml
import matplotlib.pyplot as plt
import math


class agents(object):
    
    def __init__(self):
        self.pr_graph = nx.DiGraph()
        self.create_pr_graph() ## Precedence graph
        self.M = {}
        agents_info = self.create_M() ## M(n) is "set of indices of all the function(task) nodes connected to variable node xn in the factor graph"

        pub = rospy.Publisher("/agent_funkciji", MSmessage, queue_size = 8, latch=True)
        sub = rospy.Subscriber('/funkcija_agentu', MSmessage, self.callback)
        self.working = False
        
        self.Rs = {}
        self.received = {}
        self.tasks_in_current_layer = []


        while not rospy.is_shutdown():
            check = []

            for task in self.pr_graph.nodes():
                if list(self.pr_graph.predecessors(task)) == []:
                    if task not in self.tasks_in_current_layer:
                        self.tasks_in_current_layer.append(task) 

            if not self.working:
                for agent in self.M:
                    for task in self.M[agent]:
                        if list(self.pr_graph.predecessors(task)) == []:
                            msg = MSmessage()
                            ros_dict = KeyValueString()
                            temp_dict = {}
                            location = agents_info[agent][1]
                            speed = agents_info[agent][0]
                            p1 = self.pr_graph.nodes[task]['location']
                            p2 = location 
                            distance = math.sqrt( ((p1[0]-p2[0])**2)+((p1[1]-p2[1])**2) )
                            travel_time = distance / speed
                            for i in self.M[agent]:
                                if task == i:
                                    temp_dict[i] = travel_time
                                else:
                                    temp_dict[i] = 0
                            for key, value in temp_dict.items():
                                ros_dict.key = key
                                ros_dict.values = value
                                msg.dict.append(ros_dict)
                            msg.sender = agent
                            msg.receiver = task
                            pub.publish(msg)
                            print(msg)
            if self.working:
                received = copy.deepcopy(self.received)
                for agent in received:
                    if set(self.tasks_in_current_layer) <= set(received[agent]):
                        check.append(True)
                #print(received)
                #print(check)
                if all(check) and len(check) == len(list(received.keys())) and len(check) != 0:
                    print("Hello")
                    sum_Rs = {}
                    for agent in received:
                        for task in self.tasks_in_current_layer:
                            for func_agent in self.Rs:
                                if task not in func_agent:
                                    if agent in func_agent and agent not in sum_Rs:
                                        sum_Rs[agent] = list(self.Rs[func_agent])
                                    elif agent in func_agent:
                                        sum_Rs[agent][0] += self.Rs[func_agent][0]
                                        sum_Rs[agent][1] += self.Rs[func_agent][1]
                            msg = MSmessage()
                            msg.data = sum_Rs[agent]
                            msg.sender = agent
                            msg.receiver = task
                            self.received[agent].clear()
                            pub.publish(msg)
                        Z = self.cal_Z(agent)
                        if Z[0] < Z[1]:
                            print("{} se odlucio za rad".format(agent))
                        else:
                            print("{} nece raditi".format(agent))

                          



    def cal_Z(self, agent):
        sum = [0, 0]
        for func_agent in self.Rs:
            if agent in func_agent:
                sum[0] += self.Rs[func_agent][0]
                sum[1] += self.Rs[func_agent][1]
        return sum

        
    def callback(self, data):
        self.Rs["{}, {}".format(data.sender, data.receiver)] = data.data
        if data.receiver in self.received:
            if data.sender not in self.received[data.receiver]:
                self.received[data.receiver].append(data.sender)
        else:
            self.received[data.receiver] = [data.sender]
        self.working = True
        #print(self.received)

    def create_pr_graph(self):
        stream = open("data.yaml", 'r')
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

    def create_M(self):
        stream = open("data.yaml", 'r')
        dict = yaml.load(stream, Loader=yaml.FullLoader)
        agents = dict['agents']
        agents_info = {}
        for agent in agents:
            self.M[agent] = agents[agent]['resources']
            agents_info[agent] = [agents[agent]['speed'], agents[agent]['location']]
        return agents_info

if __name__ == "__main__":
    rospy.init_node("agents")
    try:
        node = agents()
    except rospy.ROSInterruptException:
        pass