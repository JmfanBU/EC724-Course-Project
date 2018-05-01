__copyright__ = "Copyright 2013, RLPy http://www.acl.mit.edu/RLPy"
__credits__ = ["Alborz Geramifard", "Robert H. Klein", "Christoph Dann",
               "William Dabney", "Jonathan P. How"]
__license__ = "BSD 3-Clause"
__author__ = "Ray N. Forcement"

from rlpy.Tools import plt, mpatches, fromAtoB
from rlpy.Domains.Domain import Domain
from set_joint_p import set_joint_p
import numpy as np
import matplotlib.pyplot as plt
import rospy
from sensor_msgs.msg import JointState

class two_link_domain(Domain):

    def __init__(self, arm, start_xy, target_xy):
        self.statespace_limits = np.array([[0.04, 0.6],[-0.4, 0.4]])
        self.episodeCap = 100
        self.continuous_dims = [0,1]
        self.DimNames = ['X', 'Y']
        self.actions_num = 4
        self.discount_factor = 0.9
        self.stepsize = 0.02
        self.tolerance = 0.04
        self.scalefactor = 100
        self.visited_states = []

        self.robot = arm
        self.start_xy = start_xy
        self.target_xy = target_xy
        self.time = 0

        super(two_link_domain, self).__init__()

    def joint_state_callback(self, data):
        self.robot.q = data.position

    def s0(self):
        self.home_robot()
        rospy.Subscriber('/arm/joint_states', JointState, self.joint_state_callback)
        s = self.robot.position()
        self.state = np.copy(s[:,2])
        self.time = 0
        return self.state, self.isTerminal(), self.possibleActions()

    def step(self,a):
        print  "Action Applied", a, "at state=", self.state, "time=", self.time
        rospy.Subscriber('/arm/joint_states', JointState, self.joint_state_callback)

        self.time = self.time + 1

        if a == 0:
            self.moveToPlanarPos(self.state[0]+self.stepsize, self.state[1])
        elif a == 1:
            self.moveToPlanarPos(self.state[0]-self.stepsize, self.state[1])
        elif a == 2:
            self.moveToPlanarPos(self.state[0], self.state[1]-self.stepsize)
        elif a == 3:
            self.moveToPlanarPos(self.state[0], self.state[1]+self.stepsize)

        s = self.robot.position()

        print self.robot.q
        s = np.copy(s[:,2])
        self.state = np.copy(s)

        terminal = self.isTerminal()

        reward = -self.scalefactor*np.linalg.norm(s-self.target_xy)**2

        print reward
        self.visited_states.append(np.copy(s))

        return reward, s, terminal, self.possibleActions()

    def moveToPlanarPos(self, x, y, fake=False):
        print "Moving to", [x, y]
        if not fake:
            target_q = self.robot.inv_kinematics([x, y])
            set_joint_p(target_q)

    def isTerminal(self):
        return(np.linalg.norm(self.state - self.target_xy)< self.tolerance)

    def possibleActions(self):
        rtn = []
        # print self.state[0]+self.stepsize < self.statespace_limits[0,1]
        if self.state[0]+self.stepsize < self.statespace_limits[0,1]:
            rtn.append(0)

        if self.state[0]-self.stepsize > self.statespace_limits[0,0]:
            rtn.append(1)

        if self.state[1]-self.stepsize > self.statespace_limits[1,0]:
            rtn.append(2)

        if self.state[1]+self.stepsize < self.statespace_limits[1,1]:
            rtn.append(3)

        print self.statespace_limits[0,1]

        return rtn

    def home_robot(self):
        self.moveToPlanarPos(self.start_xy[0], self.start_xy[1])

    def showExploration(self):
        plt.figure()
        plt.scatter([i[0] for i in self.visited_states],[i[1] for i in self.visited_states], color='k')
        plt.scatter(self.start_xy, color='r')
        plt.scatter(self.target_xy, color='b')
        plt.show()

