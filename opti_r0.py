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

def r0_callback(msg): 
    global nv
    nv = msg.pbest

def main(): 
    global nv
    data = []
    ini_pos = (-8.0,-8.0)
    goal_pos = (-8,-1.2)
    toa = 50
    robot = ['/robot_0/base_scan','/robot_0/odom', '/robot_0/cmd_vel']
    x = True
    while not rospy.is_shutdown(): 
        if nv != old_val: 
            while x:
                sub = rospy.Subscriber('/New_value_', swarm, r0_callback)
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
                    old_val = nv
                else:
                    pass
                obj_pub = rospy.Publisher('/Bot_0/opti',swarm,queue_size= 1)
                bot0 = swarm()
                bot0.header.stamp = rospy.Time.now()
                bot0.header.frame_id = '/opti_bot0'
                bot0.pbest = [tm]
                obj_pub.publish(bot0)
                break
        else:
            pass 
        



if __name__ == "__main__":
    rospy.init_node('Bot0opti')
    rate = rospy.Rate(20)
    main()