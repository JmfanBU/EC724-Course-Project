#!/usr/bin/env python

import rospy
import threading
import math
import sys
import logging
import time
import inspect
import numpy as np
import tfx
from PyKDL import *
from tf import transformations
from tf_conversions import posemath
from std_msgs.msg import String, Bool, Float64
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import JointState
from math import pi
from psm_joint_control import Joint_position, Gripper_angle

#initialize the robot
class robot:
    def __init__(self, robot_name, ros_namespace = '/dvrk_psm/'):
        #data members, event based
        self.__robot_name = robot_name
        self.__ros_namespace = ros_namespace
        self.__robot_state = 'uninitialized'
        self.__robot_state_event = threading.Event()
        self.__goal_reached = False
        self.__goal_reached_event = threading.Event()

        #continuous publish from ros
        self.__position_joint_desired = []
        self.__effort_joint_desired = []
        self.__position_cartesian_desired = Frame()
        self.__position_joint_current = []
        self.__velocity_joint_current = []
        self.__effort_joint_current = []
        self.__position_cartesian_current = Frame()
        rospy.init_node('robot_control', anonymous = True, log_level = rospy.WARN)
        #publishers
        frame = Frame()
        full_ros_namespace = self.__ros_namespace + self.__robot_name
        self.set_position_joint = Joint_position
        self.set_robot_state = rospy.Publisher(full_ros_namespace + '/robot_state',
                                               String, latch=True, queue_size=1)
        self.set_position_goal_joint = rospy.Publisher(full_ros_namespace + '/state_joint_desired',
                                                       JointState, latch=True, queue_size=1)
        self.set_position_goal_cartesian = rospy.Publisher(full_ros_namespace + '/position_cartesian_desired',
                                                           Pose, latch=True, queue_size=1)
        self.set_goal_reached_state = rospy.Publisher(full_ros_namespace + '/goal_reached',
                                                      Bool, latch=True, queue_size=1)
        self.set_jaw_position = Gripper_angle

        #self.set_goal_reached_state.publish(False)

        #subscribers
        rospy.Subscriber(full_ros_namespace + '/robot_state',
                         String, self.__robot_state_callback)
        rospy.Subscriber(full_ros_namespace + '/goal_reached',
                         Bool, self.__goal_reached_callback)
        rospy.Subscriber(full_ros_namespace + '/state_joint_desired',
                         JointState, self.__state_joint_desired_callback)
        rospy.Subscriber(full_ros_namespace + '/position_cartesian_desired',
                         Pose, self.__position_cartesian_desired_callback)

        rospy.Subscriber(self.__ros_namespace + '/joint/states',
                         JointState, self.__state_joint_current_callback)
        rospy.Subscriber('/gazebo/link_states',
                         LinkStates, self.__position_cartesian_current_callback)

        #create node
        rospy.loginfo(rospy.get_caller_id() + ' -> started robot: ' + self.__robot_name)

    def __robot_state_callback(self, data):
        """Callback for robot state.
        :param data: the current robot state"""
        rospy.loginfo(rospy.get_caller_id() + " -> current state is %s", data.data)
        self.__robot_state = data.data
        self.__robot_state_event.set()

    def __goal_reached_callback(self, data):
        """Callback for the goal reached.
        :param data: the goal reached"""
        rospy.loginfo(rospy.get_caller_id() + " -> goal reached is %s", data.data)
        self.__goal_reached = data.data
        self.__goal_reached_event.set()

    def __state_joint_desired_callback(self, data):
        """Callback for the joint desired position.
        :param data: the `JointState <http://docs.ros.org/api/sensor_msgs/html/msg/JointState.html>`_desired"""
        self.__position_joint_desired[:] = data.position
        self.__effort_joint_desired[:] = data.effort

    def __position_cartesian_desired_callback(self, data):
        """Callback for the cartesian desired position.
        :param data: the cartesian position desired"""
        self.__position_cartesian_desired = posemath.fromMsg(data)

    def __state_joint_current_callback(self, data):
        self.__position_joint_current[:] = data.position
        self.__velocity_joint_current[:] = data.velocity
        self.__effort_joint_current[:] = data.effort

    def __position_cartesian_current_callback(sefl, data):
        self.__position_cartesian_current = posemath.fromMsg(data)

    def __dvrk_set_state(self, state, timeout = 10):
        """Simple set state with block.
        :param state: the robot state
        :param timeout: the lenghth you want to wait for robot to change state
        :return: whether or not the robot state has been successfuly set
        :rtype: Bool"""
        if (self.__robot_state == state):
            return True
        self.__robot_state_event.clear()
        self.set_robot_state.publish(state)
        self.__robot_state_event.wait(timeout)
        # if the state is not changed return False
        if (self.__robot_state != state):
            rospy.logfatal(rospy.get_caller_id() + ' -> failed to reach state ' + state)
            return False
        return True

    def home(self):
        """This method will provide power to the robot as will as home
        the robot. This method requries the robot name."""
        rospy.loginfo(rospy.get_caller_id() + ' -> start homing')
        self.__robot_state_event.clear()
        self.set_robot_state.publish('DVRK_READY')
        self.set_goal_reached_state.publish(False)
        data = np.zeros(len(self.__position_joint_current))
        self.set_position_joint(data)

    def get_robot_state(self):
        return self.__robot_state

    def get_current_cartesian_position(self):
        current_Frame = self.__position_cartesian_current
        return self.__frame_to_tfxPose(current_Frame)

    def get_current_joint_position(self):
        return self.__position_joint_current

    def get_current_joint_velocity(self):
        return self.__velocity_joint_current

    def get_current_joint_effort(self):
        return self.__effort_joint_current

    def get_desired_cartesian_position(self):
        """Get the :ref:`desired cartesian position <currentvdesired>` of the robot in terms of caretsian space.
        :returns: the desired position of the robot in cartesian space
        :rtype: tfx.canonical.CanonicalTransform object"""
        desired_Frame = self.__position_cartesian_desired
        return self.__frame_to_tfxPose(desired_Frame)

    def get_desired_joint_position(self):
        """Gets the :ref:`desired joint position <currentvdesired>` of the robot in terms of joint space.
        :returns: the desired position of the robot in joint space
        :rtype: `JointState <http://docs.ros.org/api/sensor_msgs/html/msg/JointState.html>`_"""
        return self.__position_joint_desired

    def get_desired_joint_effort(self):
        """Gets the :ref:`desired joint effort <currentvdesired>` of the robot in terms of joint space.
        :returns: the desired effort of the robot in joint space
        :rtype: `JointState <http://docs.ros.org/api/sensor_msgs/html/msg/JointState.html>`_"""
        return self.__effort_joint_desired

    def get_joint_number(self):
        """Gets the number of joints on the arm specified.
        :returns: the number of joints on the specified arm
        :rtype: int"""
        joint_num = len(self.__position_joint_current)
        return joint_num

    def close_gripper(self):
        "Close the arm gripper"
        if (not self.__dvrk_set_state('DVRK_POSITION_GOAL_CARTESIAN')):
            return False
        self.__goal_reached_event.clear()
        self.__goal_reached = False
        angle = np.array([[-10.0 * pi / 180.0],[-10.0 * pi / 180.0]])
        self.set_jaw_position(angle)
        self.__goal_reached_event.wait(20) # 1 minute at most
        if not self.__goal_reached: print 'Jaw not fully closed'
        else: self.set_goal_reached_state.publish(True)
        # Need more

    def open_gripper(self, degree):
        "Open the arm gripper"
        if (not self.__dvrk_set_state('DVRK_POSITION_GOAL_CARTESIAN')):
            return False
        angle = np.array([[degree * pi / 180.0],[degree * pi / 180.0]])
        self.set_jaw_position(angle)

   # def delta_move_cartesian(self, delta_input, interpolate = True):
    def move_cartesian_frame(self, abs_frame, interpolate=True):
        rospy.loginfo(rospy.get_caller_id() + ' -> starting absolute move cartesian frame')

        #checks for abs_frame as a tfx pose
        if not isinstance(abs_frame, tfx.canonical.CanonicalTransform):
            raise Exception("abs_frame must be a tfx.canonical.CanonicalTransform object")
        #move based on value of interpolate
        if (interpolate):
            self.__move_cartesian_goal(abs_frame)
        else:
            self.__move_cartesian_direct(abs_frame)
        rospy.loginfo(rospy.get_caller_id() + ' -> completing absolute move cartesian frame')

    def move_cartesian_frame_linear_interpolation(self, abs_frame, speed):
        if not isinstance(abs_frame, tfx.canonical.CanonicalTransform):
            raise Exception("Type error: abs_frame should be a tfx.canonical.CanonicalTransform object")
        elif not speed > 0:
            raise Exception("Speed should be positive")

        interval = 0.0001 # 1 step per X m of distance
        start_vect = self.get_current_cartesian_position()
        end_vect = abs_frame
        displacement = np.array(end_vect.position - start_vect.position) #Displacement vector
        tinterval = max(int(np.linalg.norm(displacement)/ interval), 50) #Total interval present between start and end poses

        #Done: use i not ii
        for i in range(tinterval):
            mid_pose = start_vect.interpolate(end_vect, (i + 1.0)/tinterval) #SLERP interpolation from tfx function
            #print mid_pose
            self.move_cartesian_frame(mid_pose, interpolate=False)
            Tval = self.__get_time_interval(interval, speed)
            time.sleep(Tval)

        rospy.loginfo(rospy.get_caller_id() + ' -> completing absolute move cartesian SLERP')

    def __move_cartesian_direct(self, end_frame):
        """Move the robot to the end position by passing the trajectory generator.
        :param end_frame: the ending `PyKDL.Frame <http://docs.ros.org/diamondback/api/kdl/html/python/geometric_primitives.html>`_
        :returns: true if you had successfully move
        :rtype: Bool"""
        rospy.loginfo(rospy.get_caller_id() + ' -> starting move cartesian direct')
        # set in position cartesian mode
        end_position = end_frame.msg.Pose()
        if (not self.__dvrk_set_state('DVRK_POSITION_CARTESIAN')):
            return False
        # go to that position directly
        self.set_position_cartesian.publish(end_position)
        rospy.loginfo(rospy.get_caller_id() + ' <- completing move cartesian direct')
        return True

    def __move_cartesian_goal(self, end_frame):
        """Move the robot to the end position by providing a goal for trajectory generator.
        :param end_frame: the ending `PyKDL.Frame <http://docs.ros.org/diamondback/api/kdl/html/python/geometric_primitives.html>`_
        :returns: true if you had succesfully move
        :rtype: Bool"""
        rospy.loginfo(rospy.get_caller_id() + ' -> starting move cartesian goal')
        # set in position cartesian mode
        end_position = end_frame.msg.Pose()
        if (not self.__dvrk_set_state('DVRK_POSITION_GOAL_CARTESIAN')):
            return False
        # go to that position by goal
        return self.__set_position_goal_cartesian_publish_and_wait(end_position)

    def __set_position_goal_cartesian_publish_and_wait(self, end_position):
        """Wrapper around publisher/subscriber to manage events for cartesian coordinates.
        :param end_position: the ending `PyKDL.Frame <http://docs.ros.org/diamondback/api/kdl/html/python/geometric_primitives.html>`_
        :returns: returns true if the goal is reached
        :rtype: Bool"""
        self.__goal_reached_event.clear()
        # the goal is originally not reached
        self.__goal_reached = False
        # recursively call this function until end is reached
        self.set_position_goal_cartesian.publish(end_position)
        self.__goal_reached_event.wait(20) # 1 minute at most
        if not self.__goal_reached:
            return False
        rospy.loginfo(rospy.get_caller_id() + ' -> compeleting set position goal cartesian publish and wait')
        return True

    def __frame_to_tfxPose(self, frame):
        """ Function converts a PyKDL Frame object to a tfx object """
        """ We convert a PyKDL.Frame object to a ROS posemath object and then reconvert it to a tfx.canonical.CanonicalTransform object """

        """:returns: tfx pose """
        rosMsg = posemath.toMsg(frame)
        return tfx.pose(rosMsg)


