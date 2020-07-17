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

def r1_callback(msg): 
    global nv
    nv = msg.pbest

def main(): 
    global nv
    data = []
    ini_pos = (-4.0,-8.0)
    goal_pos = (-2,-3)
    toa = 50
    robot = ['/robot_1/base_scan','/robot_1/odom', '/robot_1/cmd_vel']
    while not rospy.is_shutdown():
        sub = rospy.Subscriber('/New_value_', swarm, r1_callback)
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
            obj_pub = rospy.Publisher('/Bot_1/opti',swarm,queue_size= 1)
            bot1 = swarm()
            bot1.header.stamp = rospy.Time.now()
            bot1.header.frame_id = '/opti_bot1'
            bot1.pbest = [tm]
            obj_pub.publish(bot1)
        else: 
            pass 


if __name__ == "__main__":
    rospy.init_node('Bot1opti')
    rate = rospy.Rate(20)
    main()