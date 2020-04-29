#!/usr/bin/env python
import rospy
import math
from math import atan2
import time
from geometry_msgs.msg import Twist, Point
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan

class obstacle_avoidance():
    def __init__(self,sub_topic, pub_topic): 
        self.sub_topic = sub_topic
        self.pub_topic = pub_topic
        self.pub_ = rospy.Publisher(self.pub_topic, Twist, queue_size=1)
        self.regions_ = {'right': 0,'fright': 0,'front': 0,'fleft': 0,'left': 0}
        self.state_ = 0
        self.state_dict_ = {0: 'find the wall',1: 'turn left',2: 'follow the wall'}
        self.sub = rospy.Subscriber(self.sub_topic, LaserScan, self.clbk_laser)
        self.velocity = Twist()

    
    def clbk_laser(self,msg):
        self.regions_ = {
            'right':  min(msg.ranges[0:72]),
            'fright': min(msg.ranges[72:144]),
            'front':  min(msg.ranges[144:215]),
            'fleft':  min(msg.ranges[216:287]),
            'left':   min(msg.ranges[288:360]),}
        self.take_action()
    
    def change_state(self,state):
        if state is not self.state_:
            # print 'Wall follower - [%s] - %s' % (state, self.state_dict_[state])
            self.state_ = state
    
    def take_action(self):
        regions = self.regions_
        self.velocity = Twist()
        linear_x = 0
        angular_z = 0
        state_description = ''
        
        d = 1.20
        
        if regions['front'] > d and regions['fleft'] > d and regions['fright'] > d:
            state_description = 'case 1 - nothing'
            self.change_state(0)
        elif regions['front'] < d and regions['fleft'] > d and regions['fright'] > d:
            state_description = 'case 2 - front'
            self.change_state(1)
        elif regions['front'] > d and regions['fleft'] > d and regions['fright'] < d:
            state_description = 'case 3 - fright'
            self.change_state(2)
        elif regions['front'] > d and regions['fleft'] < d and regions['fright'] > d:
            state_description = 'case 4 - fleft'
            self.change_state(0)
        elif regions['front'] < d and regions['fleft'] > d and regions['fright'] < d:
            state_description = 'case 5 - front and fright'
            self.change_state(1)
        elif regions['front'] < d and regions['fleft'] < d and regions['fright'] > d:
            state_description = 'case 6 - front and fleft'
            self.change_state(1)
        elif regions['front'] < d and regions['fleft'] < d and regions['fright'] < d:
            state_description = 'case 7 - front and fleft and fright'
            self.change_state(1)
        elif regions['front'] > d and regions['fleft'] < d and regions['fright'] < d:
            state_description = 'case 8 - fleft and fright'
            self.change_state(0)
        else:
            state_description = 'unknown case'
            rospy.loginfo(regions)
    
    def find_wall(self):
        msg = Twist()
        msg.linear.x = 0.5
        # msg.angular.z = -0.4
        return msg
    
    def turn_left(self):
        msg = Twist()
        msg.angular.z = -2
        return msg
    
    def follow_the_wall(self):
        msg = Twist()
        msg.linear.x = 0.5
        return msg
    
    def main(self):
        # rospy.init_node('reading_laser')
        rate = rospy.Rate(10)
        i = 0
        while not rospy.is_shutdown():
            msg = Twist()
            if self.state_ == 0:
                msg = self.find_wall()
            elif self.state_ == 1:
                msg = self.turn_left()
            elif self.state_ == 2:
                msg = self.follow_the_wall()
                pass
            else:
                rospy.logerr('Unknown state!')
            self.pub_.publish(msg)
            
            if i % 50 == 0:
                # print("Breaking_Wall_Following")
                break
            i += 1
            rate.sleep()
 
if __name__ == '__main__':
    sub_topic = '/robot_0/base_scan'
    pub_topic = '/robot_0/cmd_vel'
    print(sub_topic)
    obj = obstacle_avoidance(sub_topic, pub_topic)
    obj.main()