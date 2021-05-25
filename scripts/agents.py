#!/usr/bin/env python
# -*- coding: utf-8 -*-
from networkx.classes.graph import Graph
import rospy
import copy
from task_allocation.msg import poruka
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

        pub = rospy.Publisher("/agent_funkciji", poruka, queue_size = 8, latch=True)

        working = False
        
        while not rospy.is_shutdown():
            if not working:
                for agent in self.M:
                    for task in self.M[agent]:
                        if list(self.pr_graph.predecessors(task)) == []:
                            msg = poruka()
                            p1 = self.pr_graph.nodes[task]['location']
                            p2 = agents_info[agent][1]
                            distance = math.sqrt( ((p1[0]-p2[0])**2)+((p1[1]-p2[1])**2) )
                            travel_time = distance / agents_info[agent][0]
                            msg.data = travel_time
                            msg.sender = agent
                            msg.reciver = task
                            pub.publish(msg)
                #working = True
            
                




            

        





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