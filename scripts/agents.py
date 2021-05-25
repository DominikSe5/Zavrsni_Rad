#!/usr/bin/env python
# -*- coding: utf-8 -*-
from networkx.classes.graph import Graph
import rospy
import copy
from task_allocation.msg import poruka
import networkx as nx
import yaml
import matplotlib.pyplot as plt


class agents(object):
    
    def __init__(self):
        self.pr_graph = nx.DiGraph()
        self.create_pr_graph() ## Precedence graph
        self.M = {}
        self.create_M() ## M(n) is "set of indices of all the function(task) nodes connected to variable node xn in the factor graph"

        working = False
        
        # while not rospy.is_shutdown():
        #     if not working:
        #         for agent in self.M:
        #             for task in self.M[agent]:



            

        


        

    def create_pr_graph(self):
        stream = open("data.yaml", 'r')
        dict = yaml.load(stream, Loader=yaml.FullLoader)
        tasks = dict['tasks']
        while list(self.pr_graph.nodes()) != list(tasks.keys()):
            for task in tasks:
                if tasks[task] == []:
                    self.pr_graph.add_node(task, {'duration': tasks[task]['duration'], 'location': tasks[task]['location']})
                else:
                    self.pr_graph.add_node(task, {'duration': tasks[task]['duration'], 'location': tasks[task]['location']})
                    for predecessor in tasks[task]['ordering_constraint']:
                        self.pr_graph.add_edge(predecessor, task)
        print(self.pr_graph.nodes['task1'])

    def create_M(self):
        stream = open("data.yaml", 'r')
        dict = yaml.load(stream, Loader=yaml.FullLoader)
        agents = dict['agents']
        for agent in agents:
            self.M[agent] = agents[agent]['resources']

if __name__ == "__main__":
    rospy.init_node("agents")
    try:
        node = agents()
    except rospy.ROSInterruptException:
        pass