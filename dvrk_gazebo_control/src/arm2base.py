'''
2-link arm base class.
Based off of studywolf_control library:
https://github.com/studywolf/control
'''

from armbase import ArmBase
from sensor_msgs.msg import JointState
from set_joint_p import set_joint_p
#import threading
import numpy as np
# import rospy

class Arm2Base(ArmBase):
    """
    Base information for simulation of a two-link arm.
    """

    def __init__(self,init_dq=[0.0,0.0],
                    l1=.4, l2=.4, foot= 0.04, dt=None):


        self.DOF = 2
        self.q= []
        #self.__goal_reached = False
        #self.__goal_reached_event = threading.Event()

        # rospy.Subscriber('/arm/joint_states', JointState, self.joint_state_callback)
        # rospy.init_node('manipulation',anonymous = True, log_level = rospy.WARN)
        ArmBase.__init__(self, init_q=self.q, init_dq=init_dq, singularity_thresh=.00025)

        # length of arm links
        self.l1 = l1
        self.l2 = l2
        self.foot = foot
        self.L = np.array([self.l1, self.l2])
        # mass of links
        self.m1 = 1
        self.m2 = 1

        self.rest_angles = np.array([np.pi/4.0, np.pi/4.0])

        if dt is not None:
            self.dt = dt

        print "dt: " + str(self.dt)

        # compute non changing constants
        self.K1 = (self.m1 + self.m2) * self.l1**2.0 + self.m2 * self.l2**2.0
        self.K2 = 2.0 * self.m2 * self.l1 * self.l2
        self.K3 = self.m2 * self.l2**2.0
        self.K4 = self.m2 * self.l1 * self.l2

        """
        self.K1 = ((1/3. * self.m1 + self.m2) * self.l1**2. + 1/3. * self.m2 * self.l2**2.)
        self.K2 = self.m2 * self.l1 * self.l2
        self.K3 = 1/3. * self.m2 * self.l2**2.
        self.K4 = 1/2. * self.m2 * self.l1 * self.l2
        """

        # force at end-effecor
        self.fEE = np.array([0.0, 0.0])

        # calling reset sets up:
        #   self.q = self.init_q
        #   self.dq = self.init_dq
        #   self.t = 0.0
        self.reset()

    def joint_state_callback(self, data):
        self.q = data.position


    def gen_jacCOM1(self, q=None):
        """
        Generates the Jacobian from the COM of the first
        link to the origin frame
        """
        q = self.q if q is None else q

        JCOM1 = np.zeros((6,2))
        #JCOM1[0,0] = self.l1 / 2. * -np.sin(q[0])
        #JCOM1[1,0] = self.l1 / 2. * np.cos(q[0])
        JCOM1[0,0] = self.l1 * -np.sin(q[0])
        JCOM1[1,0] = self.l1 * np.cos(q[0])
        JCOM1[5,0] = 1.0

        return JCOM1

    def gen_jacCOM2(self, q=None):
        """
        Generates the Jacobian from the COM of the second
        link to the origin frame
        """
        q = self.q if q is None else q

        JCOM2 = np.zeros((6,2))
        # define column entries right to left
        #JCOM2[0,1] = self.l2 / 2. * -np.sin(q[0]+q[1])
        #JCOM2[1,1] = self.l2 / 2. * np.cos(q[0]+q[1])
        JCOM2[0,1] = self.l2 * -np.sin(q[0]+q[1])
        JCOM2[1,1] = self.l2 * np.cos(q[0]+q[1])
        JCOM2[5,1] = 1.0

        JCOM2[0,0] = self.l1 * -np.sin(q[0]) + JCOM2[0,1]
        JCOM2[1,0] = self.l1 * np.cos(q[0]) + JCOM2[1,1]
        JCOM2[5,0] = 1.0

        return JCOM2

    def gen_jacEE(self, q=None):
        """
        Generates the Jacobian from end-effector to the origin frame
        """
        q = self.q if q is None else q

        JEE = np.zeros((2,2))
        # define column entries right to left
        JEE[0,1] = -self.l2 * np.sin(q[0]+q[1])
        JEE[1,1] = self.l2 * np.cos(q[0]+q[1])

        JEE[0,0] = -self.l1 * np.sin(q[0]) + JEE[0,1]
        JEE[1,0] = self.l1 * np.cos(q[0]) + JEE[1,1]

        return JEE

    def inv_kinematics(self, xy):
        """
        Calculate the joint angles for a given (x,y) hand position
        """
        xy[0] = xy[0] - self.foot
        import scipy.optimize
        # function to optimize
        def distance_to_target(q, xy, L):
            x = L[0] * np.cos(q[0]) + L[1] * np.cos(q[0] + q[1])
            y = L[0] * np.sin(q[0]) + L[1] * np.sin(q[0] + q[1])
            return np.sqrt((x - xy[0])**2 + (y - xy[1])**2)
        return scipy.optimize.minimize(fun=distance_to_target, x0=self.q,
                args=([xy[0], xy[1]], self.L))['x']

    def position(self, q=None):
        """
        Compute (x,y) position of the hand
        q np.array: a set of angles to return positions for
        Pos_Elbow = [ l1*cos(q0); <--(x1)
                      l1*sin(q0); <--(y1)
                      0 ]
        Pos_EE = [ l1*cos(q0) + l2*cos(q0+q1); <--(x2)
                   l1*sin(q0) + l2*sin(q0+q1); <--(y2)
                   0 ]
        """
        q = np.copy(self.q) if q is None else q

        x = np.cumsum([0,
                       self.l1 * np.cos(q[0]),
                       self.l2 * np.cos(q[0]+q[1])])
        y = np.cumsum([0,
                       self.l1 * np.sin(q[0]),
                       self.l2 * np.sin(q[0]+q[1])])
        return np.array([x+self.foot, y])

    '''
    def move_plane(self, xy, interpolate=True):
        if(interpolate):
            self.__move_plane_goal(xy)
        else:
            self.__move_plane_direct(xy)

    def __move_plane_direct(self, xy):
        target_q = self.inv_kinematics(xy)
        set_joint_p(target_q)
        return True

    def __move_cartesian_goal(self, xy):
        return self.__set_position_goal_plane_publish_and_wait(xy)

    def __set_position_goal_plane_publish_and_wait(self, xy):
        self.__goal_reached_event.clear()
        self.__goal_reached = False
        target_q = self.inv_kinematics(xy)
        set_joint_p(target_q)
        self.__goal_reached_event.wait(20)
        if not self.__goal_reached:
            return False
        return True


    def isGoal_reached(self, target_q):
        self.__goal_reached = (self.position() == self.position(target_q))
        return self.__goal_reached
    '''

    def plot(self, plt):
        """
        Plots the links of the arm and the joints.
        plt matplitlib fig: plot to display arm on
        """
        pos = self.position()

        plt.plot([pos[0][0], pos[0][1]], [pos[1][0], pos[1][1]], 'b', linewidth=5)
        plt.plot([pos[0][1], pos[0][2]], [pos[1][1], pos[1][2]], 'b', linewidth=5)

        # plot joints
        plt.plot(0,0,'ko')
        plt.plot(pos[0][1], pos[1][1], 'ko')
        plt.plot(pos[0][2], pos[1][2], 'ko')
