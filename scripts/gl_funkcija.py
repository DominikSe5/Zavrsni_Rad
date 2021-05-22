#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import copy
import pprint
from task_allocation.msg import poruka

class main_hub(object):
    def __init__(self):
        self.N = {
            'funkcija1' : ['varijabla1', 'varijabla2'],
            'funkcija2' : ['varijabla1', 'varijabla2', 'varijabla3'],
            'funkcija3' : ['varijabla2', 'varijabla3']
            }
        self.M = {
            'varijabla1' : ['funkcija1', 'funkcija2'],
            'varijabla2' : ['funkcija1', 'funkcija2', 'funkcija3'],
            'varijabla3' : ['funkcija2', 'funkcija3']
            }
        self.gamma = {
            'funkcija1' : [0.1, -0.1],
            'funkcija2' : [-0.1, 0.1],
            'funkcija3' : [-0.1, 0.1]
            }
        ready = 1
        self.Qs = {
            'varijabla1, funkcija1': [0, 0], 'varijabla1, funkcija2': [0, 0], ## posiljatelj, primatelj: [poruka]
            'varijabla2, funkcija1': [0, 0], 'varijabla2, funkcija2': [0, 0], 'varijabla2, funkcija3': [0, 0],
            'varijabla3, funkcija2': [0, 0], 'varijabla3, funkcija3': [0, 0]
            }
        self.received = {
            'varijabla1' : ['funkcija1', 'funkcija2'],
            'varijabla2' : ['funkcija1', 'funkcija2', 'funkcija3'],
            'varijabla3' : ['funkcija2', 'funkcija3']
            }
        
        sub = rospy.Subscriber('/varijabla_funkciji', poruka, self.callback)

        self.pubs = {}
        for varijabla in self.received:
            self.pubs[varijabla] = rospy.Publisher('/{}/funkcija_varijabli'.format(varijabla), poruka, queue_size=8, latch=True)

        rospy.sleep(4)
        
        Qs_za_racunanje = {}
        cycle = 0
        Rs = {'funkcija1, varijabla1': [], 'funkcija1, varijabla2': [],
            'funkcija2, varijabla1': [], 'funkcija2, varijabla2': [], 'funkcija2, varijabla3': [],
            'funkcija3, varijabla2': [], 'funkcija3, varijabla3': [],}

        while not rospy.is_shutdown():
            print('cycle=',cycle)
            cycle += 1
            rospy.sleep(1)
            n = 0
            alpha_tmp = {}
            alpha = {}
            check = []
            for varijabla in self.received:
                if set(self.M[varijabla]) <= set(self.received[varijabla]):
                    check.append(True)
            print(check)
            if all(check) and len(check) == 3:
                Qs_za_racunanje = copy.deepcopy(self.Qs)  
                for i in Qs_za_racunanje: ## pretvaranje iz tuple u listu
                    temp = list(Qs_za_racunanje[i])
                    temp_1 = [round(num, 2) for num in temp] ## round-anje
                    Qs_za_racunanje[i] = temp_1
                print('Qs=')
                pprint.pprint(Qs_za_racunanje)
                for var_func in Qs_za_racunanje:
                    for funkcije in self.N:
                        if funkcije in var_func:
                            keys = alpha_tmp.keys()
                            if not funkcije in keys:
                                alpha_tmp[funkcije] = Qs_za_racunanje[var_func]
                            else:
                                alpha_tmp[funkcije][0] += Qs_za_racunanje[var_func][0]
                                alpha_tmp[funkcije][1] += Qs_za_racunanje[var_func][1]
                print('Qs=')
                pprint.pprint(Qs_za_racunanje)
                for funkcije in alpha_tmp:
                    temp_0 = round(- alpha_tmp[funkcije][0] / len(self.N[funkcije]), 2)
                    temp_1 = round(- alpha_tmp[funkcije][1] / len(self.N[funkcije]), 2)
                    temp_list = [temp_0, temp_1]
                    alpha[funkcije] = temp_list
                print('\nalpha=')
                pprint.pprint(alpha)
                for funkcija in self.gamma:
                    varijable_kojima_saljemo = self.N[funkcija]
                    for varijabla in varijable_kojima_saljemo:
                        msg = poruka()
                        msg.primatelj = varijabla                       
                        msg.posiljatelj = funkcija
                        msg.data = self.Poruka_f_v(varijabla, funkcija, Qs_za_racunanje, alpha)
                        Rs['{}, {}'.format(funkcija, varijabla)] = [round(num, 2) for num in msg.data]
                        self.received[varijabla].clear()
                        self.pubs[varijabla].publish(msg)
                print('Rs=')
                pprint.pprint(Rs)
    def calc_U(self, f):
        xor_sum = []
        U = []
        N = self.N[f]
        gamma = self.gamma[f]
        bits = {}
        i1 = 0
        i2 = 0
        i3 = 0
        temp = list(self.N.keys())
        m_num = temp.index(f) + 1
        m = 'varijabla{}'.format(m_num)
        while i1 != pow(2, len(N)):            
            xor_sum.append(0) ## stvaranje liste [0..0] velicine 2^len(N)
            temp = [1 if i1 & (1 << (7-n)) else 0 for n in range(8)] ##varijabla u koju spremam binarni oblik countera i
            while len(temp) != len(N):
                if temp[0] == 0:
                    del temp[0] ## u gornjem koraku za npr. i=5, temp ima oblik [0,0,0,0,0,1,0,1], nakon ovog koraka micu se sve nule na lijevoj strani sve dok temp nije velicine N
            bits[i1] = temp ## dictinoary sa svim temp-ovima, npr: len(N) == 2 => bits{0 : [0, 0], 1 : [0, 1], 2 : [1, 0], 3 : [1, 1]}
            i1 = i1 + 1
        while i2 != pow(2, len(N)):
            temp = bits[i2]
            for j in N:
                if j != m:
                    xor_sum[i2] += self.calc_XOR(temp[N.index(j)], temp[N.index(m)]) ##racunanje svih xorova za kombinacije varijabli 
            i2 += 1
        while i3 != pow(2, len(N)):
            temp = bits[i3]
            if temp[N.index(m)] == 0:
                U.append(gamma[0] - xor_sum[i3])
            else:
                U.append(gamma[1] - xor_sum[i3])
            i3 += 1
        return U
    
    def calc_XOR(self, x, y):
        if x == y: return 1
        else: return 0
    
    def Poruka_f_v(self, v, f, qs, alpha):
        sum_Qs = {}
        for i in qs:
            temp = [round(num, 2) for num in qs[i]]
            qs[i] = temp
        for varijabla in self.N[f]:
            if varijabla != v:
                sum_Qs[varijabla] = qs['{}, {}'.format(varijabla, f)]
        U_0 = []
        U_1 = []
        i1 = 0
        output = [0, 0]
        U = self.calc_U(f)
        N = self.N[f]
        while i1 != pow(2, len(N)):            
            temp = [1 if i1 & (1 << (7-n)) else 0 for n in range(8)] ##pogledaj funkciju calc_U za objasnjenje ove sekcije koda --
            while len(temp) != len(N): ## --
                if temp[0] == 0: ## --
                    del temp[0] ## --
            for varijabla in sum_Qs:
                if temp[N.index(varijabla)] == 0:
                    U[i1] += round(sum_Qs[varijabla][0] + alpha[f][0], 2)
                else:
                    U[i1] += round(sum_Qs[varijabla][1] + alpha[f][1], 2)
            #print(U)
            if temp[N.index(v)] == 0: ## razvrstavanje vrijednosti funkcije U na dvije liste, u jednu idu vrijednosti za koje varijabla kojoj saljem poruku ima vrijednost 0, a u drugu za vrijednosti 1
                U_0.append(U[i1])
            elif temp[N.index(v)] == 1:
                U_1.append(U[i1])   
            i1 = i1 + 1
        # if f == 'funkcija2' or f == 'funkcija3':
        #     print('R{}_{} = '.format(f, v), U)
        #     print('sum_qs=',sum_Qs)
        output[0] += max(U_0)   
        output[1] += max(U_1)  
        return output

    def callback(self, data):
        self.Qs['{}, {}'.format(data.posiljatelj, data.primatelj)] = data.data
        if data.primatelj not in self.received[data.posiljatelj]:
            self.received[data.posiljatelj].append(data.primatelj)

if __name__ == "__main__":
    rospy.init_node("Main_Hub")
    try:
        node = main_hub()
    except rospy.ROSInterruptException:
        pass