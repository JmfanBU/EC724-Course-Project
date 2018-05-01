#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

Queue = 1
pi = np.pi

class Joint:
    def __init__(self, joint_name):
        self.name = joint_name
        self.pub = rospy.Publisher('/dvrk_psm/PSM/'+joint_name+'/SetPositionTarget', Float64, queue_size = Queue)

    def move(self, value):
        self.pub.publish(value)

def talker():
    #initiate node for controlling joints
    rospy.init_node('controller')

    joint_groups = Joint_initialization()

    data = np.zeros(len(joint_groups))
    rate = rospy.Rate(100) #100 Hz


    while not rospy.is_shutdown():
        for i in range(len(joint_groups)):
            joint_groups[i].move(data[i])
        rate.sleep()

def Joint_initialization():
    name_space = ['outer_yaw_joint', 'outer_pitch_joint_1',
                'outer_insertion_joint', 'outer_roll_joint',
                'outer_wrist_yaw_joint', 'outer_wrist_pitch_joint',
                'outer_wrist_open_angle_joint_1', 'outer_wrist_open_angle_joint_2']

    joint_groups = [Joint(j_name) for j_name in name_space]

    return joint_groups

if __name__ == '__main__':
    try:
	talker()
    except rospy.ROSInterruptException:
	pass
