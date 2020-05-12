#!/usr/bin/env python
# coding: utf-8
import rospy
import math
from pso_movement import pso_movement
from go_to_point import go_to_point1
from oa import obstacle_avoidance
from pso_ros import pso_ros
from ros_pa1.msg import swarm
import random
import numpy as np
import message_filters as mf

goal = [6,9]
bot1_pbest, bot2_pbest,bot3_pbest,bot4_pbest,bot5_pbest  = [], [], [], [], []

def callback(bot1, bot2, bot3,bot4,bot5): 
    rospy.sleep(5)
    global bot1_pbest, bot2_pbest,bot3_pbest,bot4_pbest,bot5_pbest
    assert bot1.header.stamp == bot2.header.stamp == bot3.header.stamp == bot4.header.stamp == bot4.header.stamp
    bot1_pbest = bot1.pbest
    bot2_pbest = bot2.pbest
    bot3_pbest = bot3.pbest
    bot4_pbest = bot4.pbest
    bot5_pbest = bot5.pbest
    print(bot1_pbest, bot2_pbest,bot3_pbest,bot4_pbest,bot5_pbest )

def gbest_calculator(): 
    global bot1_pbest, bot2_pbest,bot3_pbest,bot4_pbest,bot5_pbest, goal
    if bot1_pbest and bot2_pbest and bot3_pbest and bot4_pbest and bot5_pbest:
        l = 8.5
        f1 = -np.exp(-(np.linalg.norm(np.array(goal)-np.array(bot1_pbest))**2)/l)
        f2 = -np.exp(-(np.linalg.norm(np.array(goal)-np.array(bot2_pbest))**2)/l)
        f3 = -np.exp(-(np.linalg.norm(np.array(goal)-np.array(bot3_pbest))**2)/l)
        f4 = -np.exp(-(np.linalg.norm(np.array(goal)-np.array(bot4_pbest))**2)/l)
        f5 = -np.exp(-(np.linalg.norm(np.array(goal)-np.array(bot5_pbest))**2)/l)
        t = min(f1, f2, f3 ,f4, f5)
        if t == f1:
            g_gbest = bot1_pbest
        if t == f2:
            g_gbest = bot2_pbest
        if t == f3:
            g_gbest = bot3_pbest
        if t == f4:
            g_gbest = bot4_pbest
        if t == f5:
            g_gbest = bot5_pbest
        
        print("gbest is here", g_gbest)
        gbest_pub = rospy.Publisher('/gbest_calc', swarm, queue_size= 1)
        update_gbest = swarm()
        update_gbest.gbest = g_gbest
        gbest_pub.publish(update_gbest)
        rospy.sleep(5)

def main(): 
    rospy.init_node('gbestcalc')
    rospy.Rate(10)
    bot1_sub = mf.Subscriber('/Bot_1/pbest', swarm)
    bot2_sub = mf.Subscriber('/Bot_2/pbest', swarm)
    bot3_sub = mf.Subscriber('/Bot_3/pbest', swarm)
    bot4_sub = mf.Subscriber('/Bot_4/pbest', swarm)
    bot5_sub = mf.Subscriber('/Bot_5/pbest', swarm)
    ats = mf.TimeSynchronizer([bot1_sub, bot2_sub,bot3_sub,bot4_sub,bot5_sub], 10)
    ats.registerCallback(callback)
    gbest_calculator()


while not rospy.is_shutdown():
    main()
