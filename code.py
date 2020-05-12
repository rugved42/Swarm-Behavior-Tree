#!/usr/bin/env python

import rospy 
from sensor_msgs.msg import LaserScan 
from geometry_msgs.msg import Twist 
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from tf import transformations
import math
import numpy as np
#Test = 0
def final_position(msg):
    global final_pos_x, final_pos_y
    final_pos_x = msg.pose.position.x
    final_pos_y = msg.pose.position.y
    print(final_pos_x)
    print(final_pos_y)
    sub1.unregister()
    
def obstacle_detector(msg):
    print ("go")
    move.linear.x = 1.2
    R = min(msg.ranges[0:72])
    FR = min(msg.ranges[73:144])
    F = min(msg.ranges[145:216])
    FL = min(msg.ranges[217:288])
    L = min(msg.ranges[289:360])
    
    if FL > 1 and F > 1 and FR > 1:    # s_0
        print("Everyhing is clear, Goalseeking!")
        move.linear.x = 1.2
        move.angular.z = 0
    elif FL < 1 and F > 1 and FR > 1: #s_0
        move.linear.x = 1.2
        move.angular.z = -0.3
        print("Let me find a wall to follow_s_0")
    elif FL > 1 and F < 1 and FR > 1: #s_1
        move.linear.x = 0
        move.angular.z = 0.3
        print("Found a wall.I'm turning left s_1")
    elif FL > 1 and F > 1 and FR < 1: #s_2 Follow the wall 
        move.linear.x = 1.2
        move.angular.z = 0
        print("Chala jata hoon_wall ke sath mein s_2")
        #if FR > 1: 
        #move.linear.x = 0
        #move.anguar.z = 0.3
        print ("Arey wall toh thi na yaha")  
    elif FL < 1 and F < 1 and FR > 1: # s_1
        move.linear.x = 1.2
        move.angular.z = 0.3
        print("Found a wall.I'm turning left s_1")
    elif FL > 1 and F < 1 and FR < 1: #s_1
        move.linear.x = 0
        move.angular.z = 0.3
        print("Found a wall.I'm turning left s_1")
    elif FL < 1 and F >1 and FR < 1: # s_0
        move.linear.x = 1.2
        move.angular.z = -0.3
        print("Let me find a wall to follow_s_0")
    elif FL < 1 and F < 1 and FR < 1: #s_1
        move.linear.x = 0
        move.angular.z = 0.3
        print("Found a wall.I'm turning left s_1")
    pub.publish(move)

def angle_conver(msg):
   # Test = 0
    global px_, py_
    px_ = msg.pose.pose.position.x
    py_ = msg.pose.pose.position.y
    #if Test == 0:
    
    initial_position = rospy.wait_for_message("/base_pose_ground_truth", Odometry)
    initial_position_x = initial_position.pose.pose.position.x
    initial_position_y = initial_position.pose.pose.position.y
    print("here")
    print(initial_position_x)
    print(initial_position_y)
        #initial_position.x, initial_position_y
        #nitial_positionx = msg.pose.pose.position.x
        #initial_position_y = msg.pose.pose.position.y
        #initial_orientation_z = msg.pose.pose.orientation.z
    py_ = initial_position.y + ((final_pos_y - initial_position.y)*(px_- initial_position.x))/(final_pos_x - initial_position.x)
        #print(initial_position_y)
    py_ = LHS
    RHS = initial_position.y + ((final_pos_y - initial_position.y)*(px_- initial_position.x))/(final_pos_x - initial_position.x)
    if LHS == RHS:
        move.linear.x = 1

       # initial_position = rospy.wait_for_message("/base_pose_ground_truth", Odometry)
        #print(initial_position)
       # Test = 1
    
    #if Test == 1:
        #global px_, py_
        #px_ = msg.pose.pose.position.x
        #py_ = msg.pose.pose.position.y

        yaw = 0
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w)
        euler = transformations.euler_from_quaternion(quaternion)
        yaw = euler[2]
        desired_yaw = math.atan2(final_pos_y - py_, final_pos_x - px_)
        err_yaw = desired_yaw - yaw

    if math.fabs(err_yaw) > math.pi/90:
        move.angular.z = 0.3 if err_yaw > 0 else -0.3 
   
    if err_yaw == 0:
        move.linear.x = 0.7
    
        move = Twist()
        pub.publish(move)   

    




    

rospy.init_node('obstacle_avoidance', anonymous=True)
pub= rospy.Publisher('/cmd_vel',Twist, queue_size= 10)
#sub = rospy.Subscriber("/base_scan", LaserScan, obstacle_detector)
sub1 = rospy.Subscriber("/homing_signal",PoseStamped,final_position)
sub2 = rospy.Subscriber("/base_pose_ground_truth",Odometry,angle_conver)
initial_position = rospy.wait_for_message("/base_pose_ground_truth", Odometry)
initial_position_x = initial_position.pose.pose.position.x
initial_position_y = initial_position.pose.pose.position.y
print("here")
print(initial_position_x)
print(initial_position_y)
move = Twist()
rate = rospy.Rate(20)

rospy.spin()