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
gbestl_2 = [0,0]

def callback_s(msg):
    global gbestl_2
    if msg.gbest_status == 'gbest_done': 
        gbestl_2 = msg.gbest

def main():
    global ini_pos, vel, gbestl_2, domain
    ini_pos = [list(random.uniform(domain[0], domain[1]) for i in range(2)) for t in range(1)]
    pflag = 0
    i = 0
    while not rospy.is_shutdown():
        mypub = rospy.Publisher('Rugved', robot, queue_size= 1)
        tmp_pos = ini_pos
        tmp_vel = vel
        bot2 = robot()
        bot2.robot_2_id = 'Robot_2'
        bot2.pbest_2 = tmp_pos[0]
        # if pflag == 0:
        bot2.pbest2_status = 'bot_2_pbest'
            # pflag = 1
        # else:
        #     bot2.pbest2_status = 'none'
        mypub.publish(bot2)
        mysub = rospy.Subscriber('Rugved', robot, callback_s)
        if not gbestl_2:
            print("am i violating robot 2", gbestl_2)
            pso_run = pso_ros(tmp_pos,tmp_vel,gbestl_2)
            new_pos,n_vel = pso_run.run_pso()
            start_pos = [4,-4]
            laser_topic = '/robot_1/base_scan'
            pose_topic = '/robot_1/odom'
            pub_topic = '/robot_1/cmd_vel'
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
    rospy.init_node('Bot2',anonymous= True)
    rate = rospy.Rate(20)
    main()