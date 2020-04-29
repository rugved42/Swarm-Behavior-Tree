#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations
 
import math
 
# robot state variables

# goal
desired_position_ = [-5,-4]
# desired_position_[0]
# desired_position_.y = -4
# desired_position_.z = 0
# parameters
class go_to_point1():
    def __init__(self,pub_topic, pose_topic, desired_position_):
        self.yaw_precision_ = math.pi / 90 # +/- 2 degree allowed
        self.dist_precision_ = 0.3
        self.pub = rospy.Publisher(pub_topic, Twist, queue_size=1)
        self.sub_odom = rospy.Subscriber(pose_topic, Odometry, self.clbk_odom)
        self.desired_position_ = desired_position_
        self.position_ = Point()
        self.yaw_ = 0
        self.state_ = 0
        self.err_yaw = 0
        self.err_pos = 0
        self.velocity = Twist()
        self.done_status = False

    
    # callbacks
    def clbk_odom(self, msg):
        self.position_ = msg.pose.pose.position

        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w)
        euler = transformations.euler_from_quaternion(quaternion)
        self.yaw_ = euler[2]
    
    def change_state(self, state):
        self.state_ = state
        #print 'State changed to [%s]' % self.state_
    
    def fix_yaw(self):
        des_pos = self.desired_position_
        desired_yaw = math.atan2(des_pos[1] - self.position_.y, des_pos[0] - self.position_.x)
        self.err_yaw = desired_yaw - self.yaw_
        
        if math.fabs(self.err_yaw) > self.yaw_precision_:
            self.velocity.angular.z = 0.7 if self.err_yaw > 0 else -0.7
        self.pub.publish(self.velocity)
        
        # state change conditions
        if math.fabs(self.err_yaw) <= self.yaw_precision_:
            #print 'Yaw error: [%s]' % self.err_yaw
            self.change_state(1)
    
    def go_straight_ahead(self):
        des_pos = self.desired_position_
        desired_yaw = math.atan2(des_pos[1] - self.position_.y, des_pos[0] - self.position_.x)
        self.err_yaw = desired_yaw - self.yaw_
        self.err_pos = math.sqrt(pow(des_pos[1] - self.position_.y, 2) + pow(des_pos[0] - self.position_.x, 2))
        
        if self.err_pos > self.dist_precision_:
            self.velocity.linear.x = 0.6
            self.pub.publish(self.velocity)
        else:
            #print 'Position error: [%s]' % self.err_pos
            self.change_state(2)
        
        # state change conditions
        if math.fabs(self.err_yaw) > self.yaw_precision_:
            #print 'Yaw error: [%s]' % self.err_yaw
            self.change_state(0)
    
    def done(self):
        self.velocity = Twist()
        self.velocity.linear.x = 0
        self.velocity.angular.z = 0
        self.pub.publish(self.velocity)
        self.done_status = True
    
    def main(self):
        i = 0
        # rospy.init_node('go_to_point')
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.state_ == 0:
                self.fix_yaw()
            elif self.state_ == 1:
                self.go_straight_ahead()
            elif self.state_ == 2:
                self.done()
                if self.done_status == True: 
                    break
                pass
            else:
                rospy.logerr('Unknown state!')
                pass
            rate.sleep()
            
            if i%15 == 0:
                # print("Breaking")
                break
            i += 1
        # return self.done_status
        

if __name__ == '__main__':
    obj = go_to_point1(desired_position_)
    obj.main()
    # print(donestatus)
    # main()