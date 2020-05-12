#!/usr/bin/env python
# coding: utf-8
import rospy
import math
import numpy as np
from math import atan2
import time
from geometry_msgs.msg import Twist, Point
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan
import numpy as np
import math 
import random 
from numpy import inf
import matplotlib.pyplot as plt
from pso_movement import pso_movement
from go_to_point import go_to_point1
from oa import obstacle_avoidance
from multiprocessing import Process, Queue

class pso_ros(): 
    def __init__(self, robot_id, decision_horizon=1., is_sync=True): 
        self.N = 1  #Total number of particles
        self.n = 2 # Total design variable
        self.alpha = 0.5 #Inertia Component [0,1]
        self.beta1 = 1.49 
        self.beta2 = 1.49
        self.domain = (-8,8)
        self.pbest = [inf]*self.N
        self.gbest = inf
        self.gbest_iter = []
        self.gbest_iter_pos = []
        self.mean_obj = []
        self.pbest_pos = [[0]*self.n for i in range(self.N)]
        self.goal = [6,9]
        self.l = 8.5
        self.f1 = [0]*self.N
        self.reached = 0
        self.is_sync = is_sync
        self.decision_horizon = decision_horizon
        self.robot_id = robot_id

    def particles(self): 
        pop = [list(random.uniform(self.domain[0], self.domain[1]) for i in range(self.n)) for t in range(self.N)]
        return pop
       
    def obj_fun(self): 
        tmp_f1 = self.pos    
        self.f1 = [-np.exp(-(np.linalg.norm(np.array(self.goal)-np.array(i))**2)/self.l) for i in tmp_f1]
    
    def fitness_ros(self):
        tmp = self.f1
        tmp_1 = self.pos
        for i in range(len(tmp)):
            if tmp[i] < self.pbest[i]:
                self.pbest[i] = tmp[i]
                for j in range(self.n):
                    self.pbest_pos[i][j] = tmp_1[i][j]
            else:
                continue
        print("P_best array {}".format(self.pbest))
        # self.gbest = min(self.pbest)
        # t = np.where(np.array(self.pbest) == self.gbest)[0][0]
        # self.gbest_pos = self.pbest_pos[t]
        # self.gbest_iter.append(self.gbest)
        self.gbest_iter_pos.append(tuple(self.gbest_pos))
        print("position of g_best_ {}" .format(self.gbest_pos))
        
    def calc_ros(self): 
        tmp1 = self.pos
        pos_new = self.pos
        for i in range(len(tmp1)):
            # print(i)
            v_new = (self.alpha*np.array(self.vel[i])) + ((self.beta1*random.random()) * (np.array(self.pbest_pos[i]) - np.array(tmp1[i]))) + ((random.random()*self.beta2) * (np.array(self.gbest_pos) - np.array(tmp1[i])))
            # print("vnew",v_new)
            self.vel[i] = v_new
        # print("Velocity", vel)

        for i in range(len(tmp1)):
            if np.linalg.norm(self.vel[i]) != 0:
                if self.is_sync:
                    if self.robot_id == 0:
                        pos_new[i] = tmp1[i] + self.decision_horizon * np.asarray(self.vel[i]) / np.linalg.norm(self.vel[i])
                    else:
                        pos_new[i] = tmp1[i] + 0.9*self.decision_horizon * np.asarray(self.vel[i]) / np.linalg.norm(self.vel[i])
                    
                else:
                    pos_new[i] = tmp1[i] + self.vel[i]
            else: 
                if self.is_sync:
                    if self.robot_id == 0:
                        pos_new[i] = tmp1[i] + self.decision_horizon * np.asarray(self.vel[i]) / 1
                    else:
                        pos_new[i] = tmp1[i] + 0.9*self.decision_horizon * np.asarray(self.vel[i]) / 1
                    
                else:
                    pos_new[i] = tmp1[i] + self.vel[i]
        self.pos = pos_new
        # print(pos_new)
    
    def run_pso(self, tmp_pos,tmp_vel,gbestl_1):
        i = 0
        self.pos = tmp_pos
        self.vel = tmp_vel
        self.gbest_pos = gbestl_1
        while i < 1:  
            print("Particles..:", self.pos)
            self.obj_fun()
            print("f1",self.f1)
            self.fitness_ros()
            self.calc_ros()
            i += 1
        return self.pos, self.vel
        # plt.plot(self.gbest_iter)
        # plt.show()
if __name__ == "__main__":
    pso_run = pso_ros([[-5,-2]],[[0,0]],[6,7])
    x,y = pso_run.run_pso()
    print(x,y)