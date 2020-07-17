#!/usr/bin/env python
import numpy as np
from geneticalgorithm1 import geneticalgorithm as ga
import multiprocessing
import time 
import math
from pso_movement import pso_movement
import rospy
from ros_pa1.msg import swarm

def fleet_callback(msg): 
    value_ = msg.pbest
    fl_pub = rospy.Publisher('/New_value_',swarm,queue_size= 1)
    fl_man = swarm()
    fl_man.header.stamp = rospy.Time.now()
    fl_man.header.frame_id = '/Fleet_Manager'
    fl_man.pbest = value_
    fl_pub.publish(fl_man)
rospy.init_node('Fleet_Manager')
while not rospy.is_shutdown():
    sub = rospy.Subscriber('/_To_Initiator', swarm,fleet_callback)

