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

bot0_pbest, bot1_pbest,bot2_pbest,bot3_pbest,bot4_pbest  = [], [], [], [], []

def callback(bot0, bot1, bot2): 
    rospy.sleep(5)
    global bot0_pbest, bot1_pbest,bot2_pbest,bot3_pbest,bot4_pbest
    assert bot0.header.stamp == bot1.header.stamp == bot2.header.stamp
    print("I'm coming here")
    bot0_pbest = bot0.pbest
    bot1_pbest = bot1.pbest
    bot2_pbest = bot2.pbest
    # bot4_pbest = bot4.pbest
    # bot5_pbest = bot5.pbest
    print(bot0_pbest, bot1_pbest,bot2_pbest)

def gbest_calculator(): 
    global bot0_pbest, bot1_pbest,bot2_pbest,bot3_pbest,bot4_pbest, goal
    if bot0_pbest and bot1_pbest and bot2_pbest:
        opti_val = 1/8 * (bot0_pbest[0] + bot1_pbest[0] + bot2_pbest[0])
        print("optimum_value", opti_val)
        gbest_pub = rospy.Publisher('/Get_Optimum_Value', swarm, queue_size= 1)
        update_gbest = swarm()
        update_gbest.gbest = [opti_val]
        gbest_pub.publish(update_gbest)
        rospy.sleep(5)

def main(): 
    rospy.init_node('optivalcalc')
    rospy.Rate(10)
    bot0_sub = mf.Subscriber('/Bot_0/opti', swarm)
    bot1_sub = mf.Subscriber('/Bot_1/opti', swarm)
    bot2_sub = mf.Subscriber('/Bot_2/opti', swarm)
    # bot4_sub = mf.Subscriber('/Bot_4/pbest', swarm)
    # bot5_sub = mf.Subscriber('/Bot_5/pbest', swarm)
    ats = mf.TimeSynchronizer([bot0_sub, bot1_sub,bot2_sub], 10)
    ats.registerCallback(callback)
    gbest_calculator()


while not rospy.is_shutdown():
    main()
