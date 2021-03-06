#!/usr/bin/env python
import rospy
import math
import numpy as np
from math import atan2
import time
from geometry_msgs.msg import Twist, Point
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan
from go_to_point import go_to_point1
from oa import obstacle_avoidance 
from geneticalgorithm1 import geneticalgorithm as ga

class pso_movement(): 
    def __init__(self, laser_topic, pose_topic, pub_topic, ini_pos_,desired_position_,X):
        self.path = desired_position_
        self.ini_pos_ =  ini_pos_
        self.x1 = self.path[0]
        self.y1 = self.path[1]  #path list
        # print("hi from init")
        self.regions_ = {'right': 0,'fright': 0,'front': 0,'fleft': 0,'left': 0}
        self.sub = rospy.Subscriber(laser_topic, LaserScan, self.clbk_laser)
        # rospy.sleep(2)
        self.sub1 = rospy.Subscriber(pose_topic, Odometry, self.poseCallback)
        self.vel_pub = rospy.Publisher(pub_topic,Twist,queue_size = 1)
        self.x = 0
        self.y = 0
        self.ini_x = self.ini_pos_[0]
        self.ini_y = self.ini_pos_[1]
        self.theta = 0
        self.velocity = Twist()
        self.opti = X
        self.go_to_point = go_to_point1(pub_topic,pose_topic,self.path)
        self.wall_follow = obstacle_avoidance(laser_topic,pub_topic,self.opti)
        self.state_ = 0
        self.state_desc_ = ['Go to point', 'wall following']
        self.distance_position_to_line = 0
        self.flag = 0
        self.time_c = 0
        self.total_time = 0
        
        # self.check = check
    
    def clbk_laser(self,msg):
        self.regions_ = {
            'right':  min(msg.ranges[0:72]),
            'fright': min(msg.ranges[73:144]),
            'front':  min(msg.ranges[145:215]),
            'fleft':  min(msg.ranges[216:287]),
            'left':   min(msg.ranges[288:360]),}
        # print("Regions_cal", self.regions_)
    
    def poseCallback(self, pose_msg):
        self.x = pose_msg.pose.pose.position.x
        self.y = pose_msg.pose.pose.position.y
        rot_q = pose_msg.pose.pose.orientation
        (roll, pitch, self.theta) = euler_from_quaternion([rot_q.x,rot_q.y,rot_q.z,rot_q.w])
    
    def distance_to_line(self):
        up_eq = math.fabs((self.y1 - self.ini_y) * self.x - (self.x1 - self.ini_x) * self.y + (self.x1 * self.ini_y) - (self.y1 * self.ini_x))
        lo_eq = math.sqrt(pow(self.y1 - self.ini_y, 2) + pow(self.x1 - self.ini_x, 2))
        if lo_eq != 0:
            distance = up_eq / lo_eq
        else: 
            distance = up_eq
        return distance 
    
    def change_state(self,state):
        self.state_ = state
        log = "state changed: %s" % self.state_desc_[state]
        rospy.loginfo(log)
        if self.state_ == 0:
            self.go_to_point.main()
            # print("HI from state")
        if self.state_ == 1:
            self.wall_follow.main()

    def done(self):
        self.velocity = Twist()
        self.velocity.linear.x = 0
        self.velocity.angular.z = 0
        self.vel_pub.publish(self.velocity)
        self.done_status = True
    
    def main(self): 
        print("Robot started")
        start_time = time.time()
        # rospy.init_node('pso')
        # self.state_ = 0
        self.change_state(0)
        rate = rospy.Rate(20)
        # print("Regions_",self.regions_)
        # print("x,y",self.x,self.y)
        i = 0
        while not rospy.is_shutdown():
            if self.regions_ == None:
                continue
            self.distance_position_to_line = self.distance_to_line()
            # print("Distance", distance_position_to_line)
            if self.state_ == 0:
                # print("True")
                if self.regions_['front'] < 1.0:
                    self.change_state(1)
                    # print("state_changed_wall_follow_now")
                else:
                    self.change_state(0)
            
            elif self.state_ == 1:
                # print("Distance", self.distance_position_to_line)
                if self.distance_position_to_line < 0.2:
                    self.change_state(0)
                    # print("state_changed_go_to_now")
                elif self.regions_['front'] > 1.0 and self.regions_['fleft'] > 1.0 and self.regions_['fright'] > 1.0:
                    self.change_state(0)
                    # print("Nothing ahead so changing to go_to")
                else:
                    self.change_state(1)
            p = self.x1 - self.x
            q = self.y1 - self.y
            if abs(p)< 0.25 and abs(q) < 0.25:
                print("DONE DONE DONE")
                self.flag = 1
                self.total_time = time.time() - start_time
                self.done()
                rospy.sleep(2)
                break
            # print("Printing_state",self.state_)
            self.total_time = time.time() - start_time
            if self.total_time > 50: 
                self.time_c = 1
                self.done
                rospy.sleep(2)
                break 
                
            rate.sleep()
def f(X):
    # rospy.init_node('Rugved')
    exec(open("/home/rugvedhattekar/catkin_ws/src/ros_pa1_rug/scripts/Swarm-Behavior-Tree/reset_env.py").read())
    robots = [
    ['/robot_0/base_scan','/robot_0/odom', '/robot_0/cmd_vel'],
    ['/robot_1/base_scan','/robot_1/odom', '/robot_1/cmd_vel'],
    ['/robot_2/base_scan','/robot_2/odom', '/robot_2/cmd_vel'],
    ['/robot_3/base_scan','/robot_3/odom', '/robot_3/cmd_vel'],
    ['/robot_4/base_scan','/robot_4/odom', '/robot_4/cmd_vel'],
    ['/robot_5/base_scan','/robot_5/odom', '/robot_5/cmd_vel'],
    ['/robot_6/base_scan','/robot_6/odom', '/robot_6/cmd_vel'],
    ['/robot_7/base_scan','/robot_7/odom', '/robot_7/cmd_vel'], 
    ]
    ini_pos = [(-8.0,-8.0),(-4.0,-8.0),(1.5,-8.0),(5.8,-8.0),(-8.0,2.0),(-4.0,2.0),(2.0,2.0),(7.0,2.0)]
    goal_pos = [(-8,-1.2),(-2,-3),(2,-1),(7,-2),(-8,8),(-3,8),(2,8),(7,8)]
    toa = 50
    data = []
    obj_fun = []
    for i in range(len(robots)):
        obj = pso_movement(robots[i][0],robots[i][1], robots[i][2],ini_pos[i],goal_pos[i],X)
        print("X",X)
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
        obj_fun.append(tm)
    print(obj_fun)
    return (1/8) * np.sum(obj_fun) 

if __name__ == '__main__':
    varbound=np.array([[0.5,1.7],[1,100]])
    vartype=np.array([['real'],['int']])
    model=ga(function=f,dimension=2,variable_type_mixed=vartype,variable_boundaries=varbound)
    model.run()


# if __name__ == '__main__':
#     paths = [(-5,-4),(3,2),(6,-3)]
#     ini_pos = [(-8,-2)]
#     laser_topic = '/robot_1/base_scan'
#     pose_topic = '/robot_1/odom'
#     pub_topic = '/robot_1/cmd_vel'
#     for i,n in enumerate(paths):
#         obj = pso_movement(laser_topic, pose_topic, pub_topic,ini_pos[i],n)
#         obj.main()
#         ini_pos.append(n) 
#         print("Flag", obj.flag)
#     # main()
