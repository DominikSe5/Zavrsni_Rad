#!/usr/bin/env python
# -*- coding: utf-8 -*-
from networkx.algorithms.reciprocity import reciprocity
from networkx.classes.graph import Graph
import rospy
import copy
from task_allocation.msg import poruka_v1
import networkx as nx


class main_hub(object):
    
    def __init__(self):
        self.pr_graph = nx.DiGraph()
        self.create_pr_graph() ## Precedence graph
        self.factor_graph = nx.Graph()
        self.create_fc_graph() ## Factor graph

        self.M = {}

        self.received = {}
        self.tasks = []

        #sub = rospy.Subscriber('/agent_funkciji', poruka, self.callback)

        rospy.sleep(4)
        
        Qs_za_racunanje = {}
        cycle = 1

        while not rospy.is_shutdown():
            check = []
            for task in self.pr_graph.nodes: ## stvaranje liste zadataka u sadasnjem sloju 
                if len(list(self.pr_graph.predecessors(task))) == 0:
                    self.tasks.append(task)

            for agent in self.factor_graph.nodes:
                for task in self.tasks:
                    if task in list(self.factor_graph.adj[agent]):
                        if agent not in self.received: ## inicijalizacija dict, prvo upisivanje key, value
                            self.received[agent] = [task] 
                        elif agent in self.received and task not in self.received[agent]:
                            self.received[agent].append(task)## 1 - postavljanje recived zastaica za prvi prolazak kroz algoritam

            self.M = copy.deepcopy(self.received) ## M(n) is "set of indices of all the function(task) nodes 
                                                  ## connected to variable node xi in the factor graph"
            for agent in self.received:
                if set(self.M[agent]) <= set(self.received[agent]):
                    check.append(True)
            print(check)
            
            if all(check) and len(check) == len(list(self.M.keys())):
                self.pubs = {}
                for agent in self.received:
                    self.pubs[agent] = rospy.Publisher('/{}/task_varijabli'.format(agent), poruka_v1, queue_size=8, latch=True)
                for task in self.tasks:
                    agenti_kojima_saljemo = list(self.received.keys())
                    for agent in agenti_kojima_saljemo:
                        msg = poruka_v1()
                        msg.primatelj = agent                       
                        msg.posiljatelj = task
                        msg.data = self.Poruka_f_v(task, agent)
                        # Rs['{}, {}'.format(funkcija, varijabla)] = [round(num, 2) for num in msg.data]
                        # self.received[varijabla].clear()
                        # self.pubs[varijabla].publish(msg)



            
            
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
        self.pr_graph.add_nodes_from([
            ('task1', {'layer': 1, 'travel_time': 2, 'duration': 5}),
            ('task2', {'layer': 1, 'travel_time': 4, 'duration': 3}),
            ('task3', {'layer': 1, 'travel_time': 2, 'duration': 5}),
            ('task4', {'layer': 2, 'travel_time': 6, 'duration': 1}),
            ('task5', {'layer': 2, 'travel_time': 3, 'duration': 2}),
            ('task6', {'layer': 2, 'travel_time': 1, 'duration': 5}),
            ('task7', {'layer': 3, 'travel_time': 5, 'duration': 2}),
            ('task8', {'layer': 3, 'travel_time': 6, 'duration': 3})
            ])
        self.pr_graph.add_edges_from([
            ('task1', 'task4'), ('task1', 'task5'),
            ('task2', 'task5'),
            ('task3', 'task6'),
            ('task4', 'task7'),
            ('task5', 'task8'),
            ('task6', 'task8')
            ])

    def create_fc_graph(self):
        self.factor_graph.add_nodes_from([
            ('agent1', {'stl': nx.Graph()}),
            ('agent2', {'stl': nx.Graph()}),
            ('agent3', {'stl': nx.Graph()}),
            ('agent4', {'stl': nx.Graph()}),
            'task1', 'task2', 'task3', 'task4', 'task5', 'task6', 'task7', 'task8'
            ])
        self.factor_graph.add_edges_from([
            ('agent1', 'task1'),
            ('agent1', 'task3'),
            ('agent1', 'task6'),
            ('agent1', 'task7'),
            ('agent2', 'task1'),
            ('agent2', 'task2'),
            ('agent2', 'task4'),
            ('agent2', 'task5'),
            ('agent2', 'task6'),
            ('agent2', 'task7'),
            ('agent2', 'task8'),
            ('agent3', 'task2'),
            ('agent3', 'task5'),
            ('agent3', 'task8'),
            ('agent4', 'task2'),
            ('agent4', 'task6'),
            ('agent4', 'task7')
            ])

if __name__ == "__main__":
    rospy.init_node("Main_Hub")
    try:
        node = main_hub()
    except rospy.ROSInterruptException:
        pass