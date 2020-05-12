from pso_movement import pso_movement
from go_to_point import go_to_point1
from oa import obstacle_avoidance
from pso_ros import pso_ros

ini_pos = [(-8,-2), (-5,-2)]
                laser_topic = '/robot_0/base_scan'
                pose_topic = '/robot_0/odom'
                pub_topic = '/robot_0/cmd_vel'
                print("Got a local goal", self.pbest_pos[0])
                laser_topic1 = '/robot_1/base_scan'
                pose_topic1 = '/robot_1/odom'
                pub_topic1 = '/robot_1/cmd_vel'
                print("Got a local goal_Robot_1", self.pbest_pos[1])
                obj = pso_movement(laser_topic, pose_topic, pub_topic,ini_pos[0],check,self.pbest_pos[0])
                obj1 = pso_movement(laser_topic1, pose_topic1, pub_topic1,ini_pos[1],check,self.pbest_pos[1])
                # q1 = Queue()
                # q2 = Queue()
                pr1 = Process(target=obj.main())
                ini_pos[0] = tuple(self.pbest_pos[0])
                pr2 = Process(target=obj1.main())
                ini_pos[1] = tuple(self.pbest_pos[1])
                pr1.start()
                pr2.start()
                pr1.join()
                pr2.join()
                check = 1
                print("CHECK DOWN", check)
                self.reached += 1
                if obj.flag == 1 and obj1.flag == 1:
                    self.calc_ros()
                    i += 1
                    print("XXX I have updated the ini_pos XXX", obj.flag, obj1.flag)
                    if i == 20:
                        break
                    else:
                        self.reached = 0
                else:
                    pass
                    
            else:
                # print("im printing else")
                self.reached = 2

def robot_0_main():
    laser_topic = '/robot_0/base_scan'
    pose_topic = '/robot_0/odom'
    pub_topic = '/robot_0/cmd_vel'
    next_waypoint = pso_ros():

def robot_1_main():
    laser_topic1 = '/robot_1/base_scan'
    pose_topic1 = '/robot_1/odom'
    pub_topic1 = '/robot_1/cmd_vel'

