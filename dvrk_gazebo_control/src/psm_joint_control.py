#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64

Queue = 1

class Joint:
    def __init__(self, joint_name):
        self.name = joint_name
        self.pub = rospy.Publisher('/dvrk_psm/PSM/'+joint_name+'/SetPositionTarget', Float64, queue_size = Queue)

    def move(self, value):
        self.pub.publish(value)

def Joint_position(data):
    #initiate node for controlling joints
    rospy.init_node('controller')

    joint_groups = Joint_initialization()


    rate = rospy.Rate(100) #100 Hz

    while not rospy.is_shutdown():
        for i in range(0, len(joint_groups)-2):
            joint_groups[i].move(data[i])
        rate.sleep()

def Gripper_angle(data):
    #initiate node for controlling joints
    rospy.init_node('gripper_control')

    joint_groups = Joint_initialization()

    rate = rospy.Rate(100) #100 Hz

    while not rospy.is_shutdown():
        for i in [1,2]:
            joint_groups[-i].move(data[i-1])
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
	Joint_position()
    except rospy.ROSInterruptException:
	pass
