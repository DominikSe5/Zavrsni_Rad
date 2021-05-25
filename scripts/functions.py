#!/usr/bin/env python
# -*- coding: utf-8 -*-
from networkx.algorithms.reciprocity import reciprocity
from networkx.classes.graph import Graph
import rospy
import copy
from task_allocation.msg import poruka
import networkx as nx
import yaml


class main_hub(object):
    
    def __init__(self):

        self.pr_graph = nx.DiGraph()
        self.create_pr_graph() ## Precedence graph
        self.M = {}
        agents_info = self.create_M() ## M(n) is "set of indices of all the function(task) nodes connected to variable node xn in the factor graph"

        self.received = {}
        self.tasks_in_current_layer = []

        rospy.sleep(4)
        
        self.Qs = {}

        Qs_za_racunanje = {}
        cycle = 1


        while not rospy.is_shutdown():
            sub = rospy.Subscriber('/agent_funkciji', poruka, self.callback)
            rospy.sleep(1)
            check = []
            alpha = {}
            alpha_tmp = {}
            for task in self.pr_graph.nodes():
                if list(self.pr_graph.predecessors(task)) == []:
                    self.tasks_in_current_layer.append(task) ## dodavanje svih task-ova koji nemaju order restriction u listu s kojom radimo

            for agent in self.received:
                if set(self.tasks_in_current_layer) <= set(self.received[agent]):
                    check.append(True)
            if all(check) and len(check) == len(list(self.received.keys())):
                Qs_za_racunanje = copy.deepcopy(self.Qs) 
                for task in self.tasks_in_current_layer:
                    for agent_func in self.Qs:
                        keys = alpha_tmp.keys()
                        if task in agent_func:
                            if task not in keys:
                                alpha_tmp[task] = copy.deepcopy(Qs_za_racunanje[agent_func])
                            else:
                                alpha_tmp[task] += copy.deepcopy(Qs_za_racunanje[agent_func])
            print("Alpha", alpha_tmp)
            print(Qs_za_racunanje,"\n")
            alpha_tmp.clear()



            
            #if True:#all(check) and len(check) == len(list(self.M.keys())):
                # self.pubs = {}
                # for agent in self.received:
                #     self.pubs[agent] = rospy.Publisher('/{}/task_varijabli'.format(agent), poruka, queue_size=8, latch=True)
                # for task in self.tasks:
                #     agenti_kojima_saljemo = list(self.received.keys())
                #     for agent in agenti_kojima_saljemo:
                #         msg = poruka()
                #         msg.reciver = agent                       
                #         msg.sender = task
                #         msg.data = self.Poruka_f_v(task, agent)
                #         # Rs['{}, {}'.format(funkcija, varijabla)] = [round(num, 2) for num in msg.data]
                #         # self.received[varijabla].clear()
                #         # self.pubs[varijabla].publish(msg)



    def callback(self, data):
        self.Qs['{}, {}'.format(data.sender, data.reciver)] = data.data
        if data.sender in self.received:
            if data.reciver not in self.received[data.sender]:
                self.received[data.sender].append(data.reciver)
        else:
            self.received[data.sender] = [data.reciver]

    def Poruka_f_v(self, task, agent):
        finish_time = 0
        stl = self.factor_graph.nodes[agent]['stl']
        if len(list(stl.nodes())) == 0:
            finish_time = 0
        else:
            for edge in list(stl.edges()):
                finish_time += stl.edges[edge]['dur'] 
        finish_time += self.pr_graph.nodes[task]['duration']
        return finish_time

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
    rospy.init_node("Main_Hub")
    try:
        node = main_hub()
    except rospy.ROSInterruptException:
        pass