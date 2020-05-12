#!/usr/bin/env python
# coding: utf-8

import numpy as np
import math 
import random 
from numpy import inf
import matplotlib.pyplot as plt

class pso:
    def __init__(self,N, n, alpha, beta1, beta2, domain):
        self.N = N  #Total number of particles
        self.n = n # Total design variable
        self.alpha = alpha #Inertia Component [0,1]
        self.beta1 = beta1 
        self.beta2 = beta2
        self.domain = domain
        self.pbest = [inf]*self.N
        self.gbest = inf
        self.gbest_iter = []
        self.gbest_iter_pos = []
        self.mean_obj = []
        self.vel = [[0]*N for i in range(self.n)]
        self.pos = self.particles()
        self.pbest_pos = [[0]*N for i in range(self.n)]
        self.gbest_pos = []
        
    def particles(self): 
        pop = [list(random.uniform(self.domain[0], self.domain[1]) for i in range(self.N)) for t in range(self.n)]
        return pop
    
    def dejong_fun(self):
        tmp = self.pos    
        f1 = [sum(row[j]**2 for row in tmp) for j in range(len(tmp[0]))]
        print("f1: ",f1)
        self.mean_obj.append(np.mean(f1))
        return f1
    
    def fitness(self,f1):
        tmp = f1
        tmp_1 = self.pos
        for i in range(len(tmp)):
            if tmp[i] < self.pbest[i]:
                self.pbest[i] = tmp[i]
                for j in range(len(tmp_1)):
                    self.pbest_pos[j][i] = tmp_1[j][i]
            else:
                continue
        print("P_best array {}".format(self.pbest))
        self.gbest = min(self.pbest)
        t = np.where(np.array(self.pbest) == self.gbest)[0][0]
        self.gbest_pos = [row[t] for row in tmp_1]
        self.gbest_iter.append(self.gbest)
        self.gbest_iter_pos.append(tuple(self.gbest_pos))
        print("position of g_best_ {}" .format(self.gbest_pos))
        
    def calc(self):
        tmp1 = self.pos
        v = 0
        for i in range(self.n):
            for j in range(len(self.pbest)):
                v_new = (self.alpha*self.vel[i][j]) + (self.beta1*random.random()) * (self.pbest_pos[i][j] - self.pos[i][j]) + (random.random()*self.beta2) * (self.gbest_pos[i] - self.pos[i][j])
                self.vel[i][j] = v_new
        print("Velocity", self.vel)
        for m in range(len(tmp1)):
            for n in range(len(tmp1[0])):
                pos_new = self.pos[m][n] + self.vel[m][n]
                self.pos[m][n] = pos_new
        
    def run_pso(self):
        i = 0
        while i < 10: 
            print("Particles..:", self.pos)
            f1 = self.dejong_fun()
            self.fitness(f1)
            self.calc()
            i += 1

        plt.plot(self.gbest_iter)
        plt.show()
#         plt.plot(self.mean_obj)


pso_run = pso(100, 2 ,0.5,1.49,1.49, (-5.12,5.12))
pso_run.run_pso()
