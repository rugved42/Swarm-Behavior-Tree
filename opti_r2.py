#!/usr/bin/env python
import numpy as np
from geneticalgorithm1 import geneticalgorithm as ga
import multiprocessing
import time 
import math
from pso_movement import pso_movement
import rospy
from ros_pa1.msg import swarm

nv = None

def r2_callback(msg): 
    global nv
    nv = msg.pbest

def main(): 
    global nv
    data = []
    ini_pos = (1.5,-8.0)
    goal_pos = (2,-1)
    toa = 50
    robot = ['/robot_2/base_scan','/robot_2/odom', '/robot_2/cmd_vel']
    while not rospy.is_shutdown():
        sub = rospy.Subscriber('/New_value_', swarm, r2_callback)
        if nv: 
            obj = pso_movement(robot[0],robot[1], robot[2],ini_pos,goal_pos,nv)
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
                koa = 2
            else: 
                koa = 0
            data.append((ti, delta))
            if ti < toa and delta < 0.40:
                succ = 1
            else: 
                succ = 0.2
            
            tm = koa * (1- succ) + succ * (ti/toa)
            print("Objective function", tm)
            obj_pub = rospy.Publisher('/Bot_2/opti',swarm,queue_size= 1)
            bot2 = swarm()
            bot2.header.stamp = rospy.Time.now()
            bot2.header.frame_id = '/opti_bot2'
            bot2.pbest = [tm]
            obj_pub.publish(bot2)

if __name__ == "__main__":
    rospy.init_node('Bot2opti')
    rate = rospy.Rate(20)
    main()