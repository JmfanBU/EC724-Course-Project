#! /usr/bin/env python
import rospy
from set_joint_p import set_joint_p

if __name__ == "__main__":
    rospy.init_node("manipulation")

    q = [0, 1]
    set_joint_p(q)
    print 5
    q_0 = [0,0]
    set_joint_p(q_0)

