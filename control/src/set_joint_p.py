#! /usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float64

def set_joint_p(q):
    #rospy.init_node("joint_state_publisher")
    q = np.array(q)

    joint0_pose_pub = rospy.Publisher('/arm/joint0_position_controller/command', Float64, queue_size=1)
    joint1_pose_pub = rospy.Publisher('/arm/joint1_position_controller/command', Float64, queue_size=1)
    r = rospy.Rate(100)
    while not rospy.is_shutdown():
        for ii in range(250):
            joint0_pose_pub.publish(q[0])
            joint1_pose_pub.publish(q[1])
            r.sleep()
        break
