#!/usr/bin/env python
# -*- coding: utf-8 -*-
from networkx.algorithms.reciprocity import reciprocity
from networkx.classes.graph import Graph
import rospy
import copy
from task_allocation.msg import MSQmessage, KeyValueString, CheckForNewLayer
import networkx as nx
import yaml
import matplotlib.pyplot as plt
import math
import pprint
import itertools
import numpy as np

class main_hub(object):
    
    def __init__(self):

        self.pr_graph = nx.DiGraph()
        self.create_pr_graph() ## Precedence graph
        self.M = {}
        self.agents_info = self.create_agents_info() ## M(n) is "set of indices of all the function(task) nodes connected to variable node xn in the factor graph"

        self.received = {}
        

        rospy.sleep(2)
        
        self.Qs = {}

        pub = rospy.Publisher("/funkcija_agentu", MSQmessage, queue_size=8, latch=True)
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
                print("Ciklus",cycle)
                cycle += 1
                
                Qs_za_racunanje = copy.deepcopy(self.Qs)   
                print("Qs =")
                for key in Qs_za_racunanje:
                    print(key)
                    for x in Qs_za_racunanje[key]:
                        print(x)                                   
                agenti_kojima_saljemo = {}                                                                      
                for agent in self.M:
                    for task in tasks_in_current_layer:
                        if task in self.M[agent] and task not in list(agenti_kojima_saljemo.keys()):
                            agenti_kojima_saljemo[task] = [agent]
                        elif task in self.M[agent]:
                            agenti_kojima_saljemo[task].append(agent)
                        

                for task in tasks_in_current_layer:

                    f = self.f_of_task(task, agenti_kojima_saljemo[task], tasks_in_current_layer)
                    #print("\nFunkcija f za {} = {}".format(task, f))
                    for agent in agenti_kojima_saljemo[task]:
                        R = self.calc_R(f, agent, Qs_za_racunanje, agenti_kojima_saljemo[task], task, tasks_in_current_layer)
                        print("\nPoruka {}_{} = {}".format(task, agent, R))
                        msg = MSQmessage()
                        for key, value in R.items():
                                ros_dict = KeyValueString(key, value)
                                msg.dict.append(ros_dict)
                        msg.receiver = agent
                        msg.sender = task
                        self.received[agent].clear()
                        pub.publish(msg)

    def callbackLayer(self, data):
        tasks = data.distributedTasks
        for task in tasks:
            self.pr_graph.remove_node(task)
            for agent in self.M:
                if task in self.M[agent]:
                    self.M[agent].remove(task)
        self.received.clear()
        self.Qs.clear()


    def callback(self, data):
        self.Qs['{}, {}'.format(data.sender, data.receiver)] = data.dict
        if data.sender in self.received:
            if data.receiver not in self.received[data.sender]:
                self.received[data.sender].append(data.receiver)
        else:
            self.received[data.sender] = [data.receiver]

    def calc_R(self, f, agent_primatelj, Qs, Cj, task, tasks_in_current_layer):
        M = copy.deepcopy(self.M)
        tasks_in_agents_domain = []
        sum_Qs = {}
        temp = {}
        R = {}
        temp_list = []
        for agent in Cj:
            for tsk in tasks_in_current_layer:
                if tsk in M[agent]:
                    temp_list.append(tsk)
            tasks_in_agents_domain.append(copy.deepcopy(temp_list))
            temp_list.clear()
            if agent != agent_primatelj:
                sum_Qs[agent] = Qs['{}, {}'.format(agent, task)]
        for element in itertools.product(*tasks_in_agents_domain):
            index_of_ag_primatelj = Cj.index(agent_primatelj)
            if task in element:
                index_counter = 0
                for task_to_be_done_by_agents in element:
                    if index_counter != index_of_ag_primatelj:
                        for i in sum_Qs[Cj[index_counter]]:
                            if i.key == task_to_be_done_by_agents:
                                temp[element] = f[element] + i.value
                                
                    index_counter += 1
        for element in temp:
            
            if element[index_of_ag_primatelj] not in R:
                R[element[index_of_ag_primatelj]] = temp[element]
            else:
                if temp[element] < R[element[index_of_ag_primatelj]]:
                    R[element[index_of_ag_primatelj]] = temp[element]
        return R
                
    def calc_m(self, Cj, duration, task, tasks_in_agents_domain):
        m = {}
        finish_time = {}
        M = copy.deepcopy(self.M)

        for element in itertools.product(*tasks_in_agents_domain):
            max_stl = 0
            if task in element:
                indexes = self.list_duplicates_of(element, task)
                if len(indexes) == 1:
                    agent = Cj[indexes[0]]
                    stl = self.agents_info[agent][2]
                    if len(list(stl.nodes())) == 0:
                        finish_time[agent] = 0
                    else:
                        for edge in list(stl.edges()):
                            if agent not in list(finish_time.keys()):
                                finish_time[agent] = stl.edges[edge]['duration'] 
                            else:
                                finish_time[agent] += stl.edges[edge]['duration'] 
                    max_stl = finish_time[agent]
                else:
                    for index in indexes:
                        agent = Cj[index]
                        stl = self.agents_info[agent][2]
                        if len(list(stl.nodes())) == 0:
                            finish_time[agent] = 0
                        else:
                            for edge in list(stl.edges()):
                                if agent not in list(finish_time.keys()):
                                    finish_time[agent] = stl.edges[edge]['duration'] 
                                else:
                                    finish_time[agent] += stl.edges[edge]['duration'] 
                        if max_stl <= finish_time[agent]:
                            max_stl = finish_time[agent]
                m[element] = 0.7 * ((duration / len(indexes)) + max_stl)
            else:
                m[element] = np.inf
        return m

    def f_of_task(self, task, Cj, tasks_in_current_layer):
        f = {}
        tt = {}
        M = copy.deepcopy(self.M)
        tasks_in_agents_domain = []
        duration = self.pr_graph.nodes[task]['duration']
        temp_list = []

        for agent in Cj:
            for tsk in tasks_in_current_layer:
                if tsk in M[agent]:
                    temp_list.append(tsk)
            tasks_in_agents_domain.append(copy.deepcopy(temp_list))
            temp_list.clear()

        for element in itertools.product(*tasks_in_agents_domain):
            tt_sum = 0
            if task in element:
                indexes = self.list_duplicates_of(element, task)
                if len(indexes) == 1:
                    agent = Cj[indexes[0]]
                    speed_of_agent = self.agents_info[agent][0]
                    p1 = self.pr_graph.nodes[task]['location']
                    p2 = self.agents_info[agent][1]
                    distance = math.sqrt( ((p1[0]-p2[0])**2)+((p1[1]-p2[1])**2) )
                    tt_sum += 0.3 * distance / speed_of_agent
                else:
                    for index in indexes:
                        agent = Cj[index]
                        speed_of_agent = self.agents_info[agent][0]
                        p1 = self.pr_graph.nodes[task]['location']
                        p2 = self.agents_info[agent][1]
                        distance = math.sqrt( ((p1[0]-p2[0])**2)+((p1[1]-p2[1])**2) )
                        tt_sum += 0.3 * distance / speed_of_agent
                tt[element] = tt_sum
            else:
                tt[element] = np.inf
        m = self.calc_m(Cj, duration, task, tasks_in_agents_domain)
        for element in m:
            f[element] = m[element] + tt[element]
        return f

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

    def list_duplicates_of(self, seq, item):
        start_at = -1
        locs = []
        while True:
            try:
                loc = seq.index(item,start_at+1)
            except ValueError:
                break
            else:
                locs.append(loc)
                start_at = loc
        return locs

if __name__ == "__main__":
    rospy.init_node("Main_Hub")
    try:
        node = main_hub()
    except rospy.ROSInterruptException:
        pass