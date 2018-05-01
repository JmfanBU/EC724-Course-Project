import numpy as np
from numpy import linalg
from numpy import linspace
import matplotlib.pyplot as plt
import time
import math
import json

from sympy import symbols
from sympy import lambdify

import trajoptpy
import openravepy
from openravepy import *

import logging
import copy

import random

HUMAN_TASK = 0
TABLE_TASK = 1

OBS_CENTER = [-1.3858/2.0 - 0.1, -0.1, 0.0]
HUMAN_CENTER = [0.0, 0.2, 0.0]


class Planner(object):
    """
    This class plans a trajectory from start to goal
    with TrajOpt.
    """

    def __init__(self, task, l1=0.4, l2=0.3):
        # ---- important internal variables ---- #
        self.task = task
        self.l1 = l1
        self.l2 = l2

        self.start_time = None
        self.final_time = None
        self.curr_waypt_idx = None

        # these variables are for trajopt
        self.waypts_plan = None
        self.num_waypts_plan = None
        self.step_time_plan = None

        # these variables are for the unsampled trajectory

        self.waypts = None
        self.num_waypts = None
        self.step_time = None
        self.waypts_time = None

        self.weights = 0
        self.waypts_prev = None

        # ---- OpenRAVE Initialization ---- #

        # initialize robot and empty environment
        env = openravepy.Environment()
        env.StopSimulation()
        env.Load(
            "/home/jiameng/catkin_AIM/src/dvrk_env/dvrk_model/Two_Link/arm_model.zae")
        self.robot = env.GetRobots()[0]
        self.env = env

        # insert any objects you want into environment
        self.bodies = []

        # plot the table and table mount
        # plotTable(self.env)
        # plotTableMount(self.env, self.bodies)
        # plotLaptop(self.env, self.bodies)

        # ---- DEFORMATION Initialization --- #
        self.opt_traj = None
        self.n = None
        self.eps = None

    def get_waypts_plan(self):
        """
        Returns reference to waypts_plan (used by trajopt)
        Used mostly for recording experimental data by pid_trajopt.py
        """
        return self.waypts_plan

    # ---- custom feature and cost functions ---- #

    def featurize(self, waypts):
        """
        Computs the user-defined features for a given trajectory.
        ---
        input trajectory, output list of feature values
        """

        features = [None, None]
        features[0] = self.velocity_features(waypts)
        features[1] = [0.0]*(len(waypts)-1)
        for index in range(0, len(waypts)-1):
            if self.task == TABLE_TASK:
                features[1][index] = self.table_features(waypts[index+1])
            elif self.task == HUMAN_TASK:
                features[1][index] = self.human_features(
                    waypts[index+1], waypts[index])
        return features

    # -- Velocity -- #
    def velocity_features(self, waypts):
        """
        Computes total velocity cost over waypoints, confirmed to match trajopt.
        ---
        input trajectory, output scalar feature
        """
        vel = 0.0
        for i in range(1, len(waypts)):
            curr = waypts[i]
            prev = waypts[i-1]
            vel += np.linalg.norm(curr - prev)**2
        return vel

    def velocity_cost(self, waypts):
        """
        Computes the total velocity cost.
        ---
        input trajectory, output scalar cost
        """
        return self.velocity_features(waypts)

    # -- Distance to Table -- #

    def table_features(self, waypt):
        """
        Computes the cost over one waypoint based on z-axis distance to table
        ---
        input one point in trajectory, output scalar feature
        """
        if len(waypt) < 2:
            waypt = waypt.reshape(2)
        self.robot.SetDOFValues(waypt)
        coords = self.robotToCartesian(waypt)
        EEcoord_z = coords[0]
        return 2 - EEcoord_z

    def table_cost(self, waypt):
        """
        Computes the total distance to table cost
        ---
        input trajectory, output scalar cost
        """
        feature = self.table_features(waypt)
        return feature * self.weights

    # -- Distance to Human -- #

    def human_features(self, waypt, prev_waypt):
        """
        Computes laptop cost over waypoints, interpolating and
        sampling between each pair to check for intermediate collisions
        ---
        input trajectory, output scalar feature
        """
        feature = 0.0
        NUM_STEPS = 4
        for step in range(NUM_STEPS):
            inter_waypt = prev_waypt + (1.0 + step)/(
                NUM_STEPS)*(waypt - prev_waypt)
            feature += self.human_dist(inter_waypt)
        return feature

    def human_dist(self, waypt):
        """
        Computes distance from end-effector to laptop in xy coords
        input trajectory, output scalar distance where
                0: EE is at more than 0.4 meters away from laptop
                +: EE is closer than 0.4 meters to laptop
        """
        if len(waypt) < 2:
            waypt = waypt.reshape(2)
        self.robot.SetDOFValues(waypt)
        coords = self.robotToCartesian(waypt)
        EE_coord_xy = [0, coords[1]]
        human_xy = np.array(HUMAN_CENTER[0:2])
        dist = np.linalg.norm(EE_coord_xy - human_xy) - 0.4
        if dist > 0:
            return 0
        return -dist

    def human_cost(self, waypt):
        """
        Computes the total distance to laptop cost
        ---
        input trajectory, output scalar cost
        """
        prev_waypt = waypt[0:7]
        curr_waypt = waypt[7:14]
        feature = self.human_features(curr_waypt, prev_waypt)
        return feature*self.weights*np.linalg.norm(curr_waypt - prev_waypt)

    # --- custom constraints --- #

    def table_constraint(self, waypt):
        """
        Constraints z-axis of robot's end-effector to always be
        above the table
        """
        if len(waypt) < 2:
            waypt = waypt.reshape(2)
        self.robot.SetDOFValues(waypt)
        EE_coord_z = self.robotToCartesian(waypt)[0]
        if EE_coord_z > 0:
            EE_coord_z = 0
        return -EE_coord_z

    def trajOpt(self, start, goal):
        """
        Computes a plan from start to goal using trajectory optimizer.
        Reference: http://joschu.net/docs/trajopt-paper.pdf
        ---
        input is start and goal pos, updates the waypts_plan
        """

        self.robot.SetDOFValues(start)
        self.num_waypts_plan = 10
        if self.waypts_plan is None or True:
            init_waypts = np.zeros((self.num_waypts_plan, 2))
            for count in range(self.num_waypts_plan):
                init_waypts[count,
                            :] = start + count/(self.num_waypts_plan - 1.0)*(goal - start)
        else:
            init_waypts = self.waypts_plan

        request = {
                "basic_info": {
                        "n_steps": self.num_waypts_plan,
                        "manip": "active",
                        "max_iter": 40
                },
                "costs": [
                    {
                        "type": "joint_vel",
                        "params": {"coeffs": [1]}
                        }
                ],
                "constraints": [
                    {
                        "type": "joint",
                        "params": {"vals": goal.tolist()}
                        }
                ],
                "init_info": {
                    "type": "given_traj",
                    "data": init_waypts.tolist()
                }
        }

        s = json.dumps(request)
        prob = trajoptpy.ConstructProblem(s, self.env)

        for t in range(1, self.num_waypts_plan):
            if self.task == TABLE_TASK:
                prob.AddCost(
                    self.table_cost, [
                        (t, j) for j in range(2)], "table%i" %
                    t)
            elif self.task == HUMAN_TASK:
                prob.AddCost(
                    self.human_cost, [(t - 1, j) for j in range(2)] +
                    [(t, j) for j in range(2)],
                    "human%i" % t)
            #prob.AddErrorCost(self.laptop_cost, [(t-1,j) for j in range(7)]+[(t,j) for j in range(7)], "HINGE", "laptop%i"%t)

        for t in range(1, self.num_waypts_plan - 1):
            prob.AddConstraint(
                self.table_constraint, [
                    (t, j) for j in range(2)], "INEQ", "up%i" %
                t)
            #prob.AddConstraint(self.laptop_cost, [(t-1,j) for j in range(7)]+[(t,j) for j in range(7)], "EQ", "up%i"%t)
            #prob.AddConstraint(self.coffee_constraint, self.coffee_constraint_derivative, [(t,j) for j in range(7)], "EQ", "up%i"%t)

        result = trajoptpy.OptimizeProblem(prob)
        self.waypts_plan = result.GetTraj()
        # self.step_time_plan = (self.final_time - self.start_time)/(self.num_waypts_plan - 1)

    def robotToCartesian(self, q):
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
        q = self.q if q is None else q

        x = np.cumsum([0,
                       self.l1 * np.cos(q[0]),
                       self.l2 * np.cos(q[0]+q[1])])
        y = np.cumsum([0,
                       self.l1 * np.sin(q[0]),
                       self.l2 * np.sin(q[0]+q[1])])
        return np.array([x+0.04, -y])[:, 2]

    # --- here's the learning algorithm to modify the trajectory --- #

    def optimal_traj(self, start, goal):
        self.waypts_plan = None
        self.weights = 1
        self.trajOpt(start, goal)
        self.opt_traj = self.waypts_plan
        self.weights = 0
        self.waypts_plan = None

    def deform(self):
        """
        Deforms the next n waypoints of the trajectory
        ---
        input is human desired traj, return a deformed and old waypts
        """
        self.n = 3
        deform_waypt_idx = np.zeros((self.n))
        for x in range(self.n):
            deform_waypt_idx[x] = random.randint(0, 9)
        waypts_prev = copy.deepcopy(self.waypts_plan)
        waypts_deform = copy.deepcopy(self.waypts_plan)
        print deform_waypt_idx
        for i in deform_waypt_idx:
            i = int(i)
            waypts_deform[i, :] = self.opt_traj[i, :]
        return (waypts_prev, waypts_deform)

    def learnWeights(self):
        """
        Deforms the trajctory given human correction, and update features
        by computing difference between features of new trajctory and old
        trajectory
        ---
        input is None returns updated weights
        """
        (waypts_prev, waypts_deform) = self.deform()
        if waypts_deform.tolist() is not None:
            new_features = self.featurize(waypts_deform)
            old_features = self.featurize(waypts_prev)
            Phi_p = np.array([new_features[0], sum(new_features[1])])
            Phi = np.array([old_features[0], sum(old_features[1])])

            update_gain = 1.0
            max_weight = 1.0

            if self.task == TABLE_TASK:
                update_gain = 2.0
                max_weight = 2.0
            elif self.task == HUMAN_TASK:
                update_gain = 100.0
                max_weight = 10.0

            update = Phi_p - Phi
            curr_weight = self.weights - update_gain*update[1]
            if curr_weight > max_weight:
                curr_weight = max_weight
            elif curr_weight < 0.0:
                curr_weight = 0.0

            self.weights = curr_weight
            return self.weights

    def replan(self, start, goal):
        self.trajOpt(start, goal)

    def Correction(self, start, goal):
        self.optimal_traj(start, goal)
        traj_opt = self.opt_traj
        self.trajOpt(start, goal)
        self.eps = 1e-5
        times = 0
        gap = 100
        weight_list = []
        while gap > self.eps:
            prev_traj = self.waypts_plan
            self.replan(start, goal)
            curr_traj = self.waypts_plan
            pre_weights = self.weights
            weights = self.learnWeights()
            times += 1
            print weights, times
            prev_features = self.featurize(prev_traj)
            curr_features = self.featurize(curr_traj)
            opt_features = self.featurize(traj_opt)
            prev_Phi = np.array([prev_features[0], sum(prev_features[1])])
            curr_Phi = np.array([curr_features[0], sum(curr_features[1])])
            opt_Phi = np.array([opt_features[0], sum(opt_features[1])])
            gap = np.linalg.norm(curr_Phi - opt_Phi)
            print gap
            weight_list.append(weights)
        return times, weight_list


if __name__ == '__main__':
    time.sleep(5)
    experiment = Planner(0)
    # experiment.weights = 4
    start = np.array([-0.57043903, -2.96817033])
    goal = np.array([0.57043903, 2.96817033])
    """
    experiment.trajOpt(start, goal)
    print experiment.waypts_plan
    cur_traj = experiment.waypts_plan
    for waypt in experiment.waypts_plan:
        print experiment.robotToCartesian(waypt)

    """

    """
    Test for Correction

    experiment.optimal_traj(start, goal)
    traj_opt = experiment.opt_traj
    eps = 0.01
    times = 0
    while np.linalg.norm(cur_traj - traj_opt) > eps:
        experiment.replan(start, goal)
        pre_weights = experiment.weights
        weights = experiment.learnWeights()
        times += 1
        print weights, times
        if abs(pre_weights - weights) == 0:
            break
    """
    times, weight_list = experiment.Correction(start, goal)
    cur_traj = experiment.waypts_plan
    for waypt in experiment.waypts_plan:
        print experiment.robotToCartesian(waypt)
    for waypt in experiment.opt_traj:
        print experiment.robotToCartesian(waypt)
    print weight_list
