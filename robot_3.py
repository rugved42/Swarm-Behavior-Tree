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
domain = (-8,8)
N = 1
n = 2
ini_pos = [list(random.uniform(domain[0], domain[1]) for i in range(n)) for t in range(N)]
vel = [[0,0]]
gbestl_3 = []


def callback_s(msg):
    global gbestl_3
    gbestl_3 = msg.gbest
    

def main():
    global ini_pos, vel, gbestl_3, domain
    ini_pos = [list(random.uniform(domain[0], domain[1]) for i in range(2)) for t in range(1)]
    vel = [[0,0]]
    start_pos = [4,-7]
    robot_id = 2
    pso_run = pso_ros(robot_id)
    print("ini_pos",ini_pos)
    while not rospy.is_shutdown():
        mypub = rospy.Publisher('/Bot_3/pbest', swarm, queue_size= 1)
        tmp_pos = ini_pos
        tmp_vel = vel
        bot3 = swarm()
        bot3.header.stamp = rospy.Time.now()
        bot3.header.frame_id = '/Bot-3'
        bot3.pbest = tmp_pos[0]
        mypub.publish(bot3)
        get_gbest = rospy.Subscriber('/gbest_calc', swarm, callback_s)
        print("gbestl2", gbestl_3)
        if gbestl_3:
            print("am i violating swarm 2", gbestl_3)
            new_pos,n_vel = pso_run.run_pso(tmp_pos,tmp_vel,gbestl_3)
            laser_topic = '/robot_2/base_scan'
            pose_topic = '/robot_2/odom'
            pub_topic = '/robot_2/cmd_vel'
            oa_obj = pso_movement(laser_topic, pose_topic, pub_topic,start_pos,new_pos[0])
            oa_obj.main()
            if oa_obj.flag == 1: 
                ini_pos = new_pos
                vel = n_vel
                start_pos = new_pos[0]
        else: 
            pass
if __name__ == "__main__":
    rospy.init_node('Bot3')
    rate = rospy.Rate(20)
    main()