#!/usr/bin/env python
import numpy as np
from geneticalgorithm1 import geneticalgorithm as ga
import multiprocessing
import time 
import math
from pso_movement import pso_movement
import rospy


def f(X):
    exec(open("/home/rugvedhattekar/catkin_ws/src/ros_pa1_rug/scripts/Swarm-Behavior-Tree/reset_env.py").read())
    robots = [
    ['/robot_0/base_scan','/robot_0/odom', '/robot_0/cmd_vel'],
    ['/robot_1/base_scan','/robot_1/odom', '/robot_1/cmd_vel'],
    ['/robot_2/base_scan','/robot_2/odom', '/robot_2/cmd_vel'],
    ['/robot_3/base_scan','/robot_3/odom', '/robot_3/cmd_vel'],
    ['/robot_4/base_scan','/robot_4/odom', '/robot_4/cmd_vel'],
    ['/robot_5/base_scan','/robot_5/odom', '/robot_5/cmd_vel'],
    ['/robot_6/base_scan','/robot_6/odom', '/robot_6/cmd_vel'],
    ['/robot_7/base_scan','/robot_7/odom', '/robot_7/cmd_vel'], 
    ]
    ini_pos = [(-8.0,-8.0),(-4.0,-8.0),(1.5,-8.0),(5.8,-8.0),(-8.0,2.0),(-4.0,2.0),(2.0,2.0),(7.0,2.0)]
    goal_pos = [(-8,-1.2),(-2,-3),(2,-1),(7,-2),(-8,8),(-3,8),(2,8),(7,8)]
    toa = 50
    data = []
    obj_fun = []
    for i in range(len(robots) - 5):
        obj = pso_movement(robots[i][0],robots[i][1], robots[i][2],ini_pos[i],goal_pos[i],X)
        print("X",X)
        obj.main()
        if obj.time_c == 1 and obj.flag == 0: 
            print("Exceeded Time")
            ti = obj.total_time
        else: 
            print("Success")
            ti = obj.total_time
        delta = math.hypot(obj.x1 - obj.x, obj.y1 - obj.y)
        print("Delta and time", delta ,ti)
        if delta > 0.40 and ti >= toa: 
            koa = 3
        else: 
            koa = 0
        data.append((ti, delta))
        if ti < toa and delta < 0.40:
            succ = 1
        else: 
            succ = 0.05
        
        tm = koa * (1- succ) + succ * (ti/toa)
        print("Objective function", tm)
        obj_fun.append(tm)
    print(obj_fun)
    return (1/8) * np.sum(obj_fun) 

rospy.init_node('Navigation')
varbound=np.array([[0.5,1.7],[1,100]])
vartype=np.array([['real'],['int']])
model=ga(function=f,dimension=2,variable_type_mixed=vartype,variable_boundaries=varbound)
model.run()