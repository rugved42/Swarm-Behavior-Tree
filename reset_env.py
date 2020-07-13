#! /usr/bin/env python

import rospy
from std_srvs.srv import Empty
import os
# rospy.init_node('reset_world')
# nodes = os.popen("rosnode list").readlines()
# for i in range(len(nodes)):
#     nodes[i] = nodes[i].replace("\n","")
# print("nodes",nodes)
# if '/stage_sim' in nodes: 
#     os.system("rosnode kill "+ '/stage_sim')
#     print("Killed")
#     rospy.sleep(6)
# else: 
#     pass
rospy.wait_for_service('/reset_positions')
reset_world = rospy.ServiceProxy('/reset_positions', Empty)
reset_world()