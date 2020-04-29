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
    def __init__(self): 
        self.N = 2  #Total number of particles
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
        self.vel = [[0]*self.n for i in range(self.N)]
        self.pos = self.particles()
        self.pbest_pos = [[0]*self.n for i in range(self.N)]
        self.gbest_pos = []
        self.goal = [7,5]
        self.l = 8.5
        self.f1 = [0]*self.N
        self.reached = 0
        
    def particles(self): 
        pop = [list(random.uniform(self.domain[0], self.domain[1]) for i in range(self.n)) for t in range(self.N)]
        return pop

    def obj_fun(self): 
        tmp = self.pos    
        self.f1 = [-np.exp(-(np.linalg.norm(np.array(self.goal)-np.array(i))**2)/self.l) for i in tmp]
    
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
        self.gbest = min(self.pbest)
        t = np.where(np.array(self.pbest) == self.gbest)[0][0]
        self.gbest_pos = self.pbest_pos[t]
        self.gbest_iter.append(self.gbest)
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
            pos_new[i] = tmp1[i] + self.vel[i]
        self.pos = pos_new
        # print(pos_new)
    
    def run_pso(self):
        i = 0
        check = 0
        while not rospy.is_shutdown(): 
            if self.reached <= 1:
                print("Particles..:", self.pos)
                print("CHECK",check)
                self.obj_fun()
                self.fitness_ros()
                ini_pos = [(-8,-2), (-5,-2)]
                laser_topic = '/robot_0/base_scan'
                pose_topic = '/robot_0/odom'
                pub_topic = '/robot_0/cmd_vel'
                print("Got a local goal", self.pbest_pos[0])
                laser_topic1 = '/robot_1/base_scan'
                pose_topic1 = '/robot_1/odom'
                pub_topic1 = '/robot_1/cmd_vel'
                print("Got a local goal_Robot_1", self.pbest_pos[1])
                obj = pso_movement(laser_topic, pose_topic, pub_topic,ini_pos[0],check,self.pbest_pos[0])
                obj1 = pso_movement(laser_topic1, pose_topic1, pub_topic1,ini_pos[1],check,self.pbest_pos[1])
                # q1 = Queue()
                # q2 = Queue()
                pr1 = Process(target=obj.main())
                ini_pos[0] = tuple(self.pbest_pos[0])
                pr2 = Process(target=obj1.main())
                ini_pos[1] = tuple(self.pbest_pos[1])
                pr1.start()
                pr2.start()
                pr1.join()
                pr2.join()
                check = 1
                print("CHECK DOWN", check)
                self.reached += 1
                if obj.flag == 1 and obj1.flag == 1:
                    self.calc_ros()
                    i += 1
                    print("XXX I have updated the ini_pos XXX", obj.flag, obj1.flag)
                    if i == 20:
                        break
                    else:
                        self.reached = 0
                else:
                    pass
                    
            else:
                # print("im printing else")
                self.reached = 2

            

        plt.plot(self.gbest_iter)
        plt.show()
if __name__ == "__main__":
    pso_run = pso_ros()
    pso_run.run_pso()