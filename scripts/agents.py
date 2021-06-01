#!/usr/bin/env python
# -*- coding: utf-8 -*-
from os import getcwd
from networkx.algorithms.cycles import minimum_cycle_basis
from networkx.classes.graph import Graph
import rospy
import copy
from task_allocation.msg import MSQmessage, MSRmessage, KeyValueString, CheckForNewLayer
from std_msgs.msg import String
import networkx as nx
import yaml
import matplotlib.pyplot as plt
import math


class agents(object):
    
    def __init__(self):
        self.pr_graph = nx.DiGraph()
        self.create_pr_graph() ## Precedence graph
        self.M = {}
        self.agents_info = self.create_M() ## M(n) is "set of indices of all the function(task) nodes connected to variable node xn in the factor graph"

        pub = rospy.Publisher("/agent_funkciji", MSQmessage, queue_size = 8, latch=True)
        pub1 = rospy.Publisher("/newLayer", CheckForNewLayer, queue_size = 8, latch=True)
        sub = rospy.Subscriber('/funkcija_agentu', MSRmessage, self.callback)
        self.working = False
        
        self.Rs = {}
        self.received = {}
        
        self.cycle_limiter = {}

        while not rospy.is_shutdown():

            pr_graph = copy.deepcopy(self.pr_graph)
            check = []
            tasks_in_current_layer = []
            task_distribution = {}
            for task in pr_graph.nodes():
                if len(list(pr_graph.predecessors(task))) == 0:
                    if task not in tasks_in_current_layer:
                        tasks_in_current_layer.append(task) 

            if not self.working:
                for agent in self.M:
                    for task in self.M[agent]:
                        if list(pr_graph.predecessors(task)) == []:
                            msg = MSQmessage()
                            temp_dict = {}
                            location = self.agents_info[agent][1]
                            speed = self.agents_info[agent][0]
                            p1 = pr_graph.nodes[task]['location']
                            p2 = location 
                            distance = math.sqrt( ((p1[0]-p2[0])**2)+((p1[1]-p2[1])**2) )
                            travel_time = distance / speed
                            for i in self.M[agent]:
                                if task == i:
                                    temp_dict[i] = travel_time + pr_graph.nodes[task]['duration']
                                else:
                                    temp_dict[i] = 0
                            for key, value in temp_dict.items():
                                ros_dict = KeyValueString()
                                ros_dict.key = key
                                ros_dict.value = value
                                msg.dict.append(ros_dict)
                            msg.sender = agent
                            msg.receiver = task
                            pub.publish(msg)
            if self.working:
                
                received = copy.deepcopy(self.received)
                for agent in received:
                    if set(tasks_in_current_layer) <= set(received[agent]):
                        check.append(True)
                
                if check:
                    print("Q checks", check)

                if all(check) and len(check) == len(list(received.keys())) and len(check) != 0:
                    rs = copy.deepcopy(self.Rs)
                    temp_dict = {}
                    for agent in received:
                        for task in tasks_in_current_layer:
                            if len(tasks_in_current_layer) == 1:
                                temp_dict[task] = 0
                            for func_agent in rs:
                                if task not in func_agent:
                                    if agent in func_agent:
                                        temp_dict[task] = rs[func_agent]
                            msg = MSQmessage()
                            msg.sender = agent
                            msg.receiver = task

                            alpha_tmp = 0
                            for i in temp_dict.values():
                                alpha_tmp += i
                            alpha = -alpha_tmp / len(temp_dict.values())

                            for key, value in temp_dict.items():
                                poruka = value + alpha
                                ros_dict = KeyValueString(key, poruka)
                                msg.dict.append(ros_dict)
                            self.received[agent].clear()
                            pub.publish(msg)
                        
                        if agent not in self.cycle_limiter:
                            self.cycle_limiter[agent] = [self.cal_Z(agent, tasks_in_current_layer)]
                        else:
                            self.cycle_limiter[agent].append(self.cal_Z(agent, tasks_in_current_layer))
                        if len(self.cycle_limiter[agent]) > 3:
                            temp = self.cycle_limiter[agent][1:]
                            self.cycle_limiter[agent] = temp
                        print("Odluke: ",self.cycle_limiter)

                        if all(x == self.cycle_limiter[agent][0] for x in self.cycle_limiter[agent]) and len(self.cycle_limiter[agent]) == 3:
                            print("{} se odlucio za zadatak {}".format(agent, self.cycle_limiter[agent][0]))
                            task_distribution[agent] = self.cycle_limiter[agent][0]
              
                        if set(received) == set(task_distribution):
                            self.received.clear()
                            self.Rs.clear()
                            print("rjesen layer")
                            for agent in task_distribution:
                                stl = self.agents_info[agent][2]
                                if len(stl.edges()) == 0:
                                    stl.add_edge('start', '{}_start'.format(task_distribution[agent]))
                                    stl.add_edge('{}_start'.format(task_distribution[agent]), '{}_finish'.format(task_distribution[agent]))

                                    location = self.agents_info[agent][1]
                                    speed = self.agents_info[agent][0]
                                    p1 = pr_graph.nodes[task_distribution[agent]]['location']
                                    p2 = location 
                                    distance = math.sqrt( ((p1[0]-p2[0])**2)+((p1[1]-p2[1])**2) )
                                    travel_time = distance / speed

                                    stl['start']['{}_start'.format(task_distribution[agent])]['duration'] = travel_time
                                    stl['{}_start'.format(task_distribution[agent])]['{}_finish'.format(task_distribution[agent])]['duration'] = self.pr_graph.nodes[task_distribution[agent]]['duration']
                                else:
                                    stl.add_edge(list(list(stl.edges())[-1])[1], '{}_start'.format(task_distribution[agent]))
                                    stl.add_edge('{}_start'.format(task_distribution[agent]), '{}_finish'.format(task_distribution[agent]))

                                    location = self.agents_info[agent][1]
                                    speed = self.agents_info[agent][0]
                                    p1 = pr_graph.nodes[task_distribution[agent]]['location']
                                    p2 = location 
                                    distance = math.sqrt( ((p1[0]-p2[0])**2)+((p1[1]-p2[1])**2) )
                                    travel_time = distance / speed

                                    stl[list(list(stl.edges())[-2])[0]][list(list(stl.edges())[-2])[1]]['duration'] = travel_time
                                    stl['{}_start'.format(task_distribution[agent])]['{}_finish'.format(task_distribution[agent])]['duration'] = self.pr_graph.nodes[task_distribution[agent]]['duration']
                                
                                
                                self.agents_info[agent][1] = pr_graph.nodes[task_distribution[agent]]['location']
                                
                                for agent1 in task_distribution:
                                    if len(self.M[agent]) != 0:
                                        self.M[agent].remove(task_distribution[agent1])

                                
                            
                            self.working = False
                            
                            

                            newLayerMsg = CheckForNewLayer()
                            
                            for agent1 in task_distribution:
                                if len(list(self.pr_graph.nodes())) != 0:
                                    self.pr_graph.remove_node(task_distribution[agent1])
                                    newLayerMsg.distributedTasks.append(task_distribution[agent1])
                            pub1.publish(newLayerMsg)

                            if len(list(self.pr_graph.nodes())) == 0:
                                for agent in self.M:
                                    g = self.agents_info[agent][2]            
                                    pos=nx.spring_layout(g) # positions for all nodes
                                    nx.draw_networkx_labels(g,pos,font_size=20,font_family='sans-serif')
                                    nx.draw(g,pos)
                                    
                                    plt.title("Raspored od {}".format(agent))
                                    plt.show()

                                rospy.signal_shutdown('Kraj')
                            



    def cal_Z(self, agent, tasks_in_current_layer):
        rs_for_agent = {}
        for func_agent in self.Rs:
            if agent in func_agent:
                rs_for_agent[func_agent] = self.Rs[func_agent]
        minimum = min(rs_for_agent.values())
        for key, value in rs_for_agent.items():
            if value == minimum:
                decided_task = key
        for task in tasks_in_current_layer:
            if task in decided_task:
                return task
    
    def callback(self, data):
        print("R poruka", data.sender, data.receiver, data.data)
        self.Rs["{}, {}".format(data.sender, data.receiver)] = data.data
        if data.receiver in self.received:
            if data.sender not in self.received[data.receiver]:
                self.received[data.receiver].append(data.sender)
        else:
            self.received[data.receiver] = [data.sender]
        self.working = True

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

    def create_M(self):
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
    rospy.init_node("agents", disable_signals=True)
    try:
        node = agents()
    except rospy.ROSInterruptException:
        pass