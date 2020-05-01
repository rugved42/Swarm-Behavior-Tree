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
domain = (-8,8)
N = 1
n = 2
ini_pos = [list(random.uniform(domain[0], domain[1]) for i in range(n)) for t in range(N)]
vel = [[0,0]]
gbestl_1 = [0,0]

def callback_s(msg):
    global gbestl_1
    if msg.gbest_status == 'gbest_done': 
        gbestl_1 = msg.gbest

def main():
    global ini_pos, vel, gbestl_1, domain
    ini_pos = [list(random.uniform(domain[0], domain[1]) for i in range(2)) for t in range(1)]
    pflag = 0
    i = 0
    while not rospy.is_shutdown():
        mypub = rospy.Publisher('Rugved', robot, queue_size= 1)
        tmp_pos = ini_pos
        tmp_vel = vel
        bot1 = robot()
        bot1.robot_1_id = 'Robot_1'
        bot1.pbest_1 = tmp_pos[0]
        # if pflag == 0:
        bot1.pbest1_status = 'bot_1_pbest'
            # pflag = 1
        # else:
        #     bot1.pbest1_status = 'none'
        mypub.publish(bot1)
        mysub = rospy.Subscriber('Rugved', robot, callback_s)
        if not gbestl_1:
            print("am i violating robot 1", gbestl_1)
            pso_run = pso_ros(tmp_pos,tmp_vel,gbestl_1)
            new_pos,n_vel = pso_run.run_pso()
            start_pos = [-8,-2]
            laser_topic = '/robot_0/base_scan'
            pose_topic = '/robot_0/odom'
            pub_topic = '/robot_0/cmd_vel'
            oa_obj = pso_movement(laser_topic, pose_topic, pub_topic,start_pos,new_pos)
            oa_obj.main()
            if oa_obj.flag == 1: 
                ini_pos = new_pos
                vel = n_vel
                start_pos = new_pos[0]
                # pflag = 0
        else: 
            pass
        # i += 1
        # if i%100 == 0: 
        #     pflag = 0


if __name__ == "__main__":
    rospy.init_node('Bot1',anonymous= True)
    rate = rospy.Rate(20)
    main()

        

