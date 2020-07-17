#!/usr/bin/env python
import numpy as np
from geneticalgorithm1 import geneticalgorithm as ga
import multiprocessing
import time 
import math
from pso_movement import pso_movement
import rospy
from ros_pa1.msg import swarm
opti_val = None

def sub_call(msg): 
    global opti_val
    opti_a = msg.gbest
    opti_val = opti_a[0]

def f(X):
    global opti_val
    exec(open("/home/rugvedhattekar/catkin_ws/src/ros_pa1_rug/scripts/Swarm-Behavior-Tree/reset_env.py").read())
    pub = rospy.Publisher('/_To_Initiator', swarm, queue_size=1)
    checker = False
    while checker == False: 
        bot0 = swarm()
        bot0.header.stamp = rospy.Time.now()
        bot0.header.frame_id = '/Initiator'
        bot0.pbest = X
        pub.publish(bot0)
        sub = rospy.Subscriber('/Get_Optimum_Value', swarm,sub_call)
        if opti_val != None:
            return opti_val
        else: 
            pass 
            

rospy.init_node('Navigation')
varbound=np.array([[0.5,1.7],[1,100]])
vartype=np.array([['real'],['int']])
model=ga(function=f,dimension=2,variable_type_mixed=vartype,variable_boundaries=varbound)
model.run()