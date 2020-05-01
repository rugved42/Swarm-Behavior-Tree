#!/usr/bin/env python
# coding: utf-8
import rospy
import math
from pso_movement import pso_movement
from go_to_point import go_to_point1
from oa import obstacle_avoidance
from pso_ros import pso_ros
from ros_pa1.msg import robot
import random
import numpy as np

goal = [6,9]
g_gbest = []
n_robots = 2
pbest_bot1, pbest_bot2 = [], []
pbest = {}
def callback(msg): 
    global goal, g_gbest,pbest_bot1,pbest_bot2,pbest
    l = 8.5
    # if msg.robot_1_id == 'Robot_1' and msg.pbest1_status == 'bot_1_pbest': 
    pbest_bot1.append(msg.pbest_1)
        # print("hi")
    # if msg.robot_2_id == 'Robot_2' and msg.pbest2_status == 'bot_2_pbest':
    pbest_bot2.append(msg.pbest_2)
    pbest = {pbest_bot1[-1], pbest_bot2[-1]}
    print("pbest_bot", pbest)
    if not pbest_bot1[-1]: 
        f1 = -np.exp(-(np.linalg.norm(np.array(goal)-np.array(pbest_bot1[-1]))**2)/l)
    if not pbest_bot2[-1]:
        f2 = -np.exp(-(np.linalg.norm(np.array(goal)-np.array(pbest_bot2[-1]))**2)/l)
    t = min(f1, f2)
    if t == f1:
        g_gbest = pbest_bot1[-1]
    if t == f2:
        g_gbest = pbest_bot2[-1]
    print("gbest is here", g_gbest)
    mypub = rospy.Publisher('Rugved', robot, queue_size= 1)
    update_gbest = robot()
    update_gbest.gbest_status = 'gbest_done'
    update_gbest.gbest = g_gbest
    mypub.publish(update_gbest)
    
    rospy.sleep(5)
        

while not rospy.is_shutdown():
    rospy.init_node('receiver')
    rospy.Rate(20)
    mysub = rospy.Subscriber('Rugved', robot, callback)