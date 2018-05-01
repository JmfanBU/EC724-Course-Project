#!/usr/bin/env python
"""
Getting Started Tutorial for RLPy
=================================
This file contains a very basic example of a RL experiment:
A simple Grid-World.
"""
__author__ = "Robert H. Klein"
from rlpy.Domains import GridWorld
from rlpy.Agents import Q_Learning
from rlpy.Representations import *
from rlpy.Policies import eGreedy
from rlpy.Experiments import Experiment

from two_link_domain import *
from two_link_angular_domain import *
from arm2base import *

import numpy as np

def make_experiment(arm, exp_id = 1, path = "./Results/Tutorial/two_link-planar"):
    """
    Each file specifying an experimental setup should contain a
    make_experiment function which returns an instance of the Experiment
    class with everything set up.

    @param id: number used to seed the random number generators
    @param path: output directory where logs and results are stored
    """
    opt = {}
    opt["exp_id"] = exp_id
    opt["path"] = path

    start_xy = np.array([0.4, -0.2])
    target_xy = np.array([0.1, -0.3])
    #Domain
    domain = two_link_angular_domain(arm, start_xy, target_xy)
    opt["domain"] = domain

    ## Representation
    # discretization only needed for continuous state spaces, discarded otherwise
    # representation  = Tabular(domain)
    representation = RBF(domain, num_rbfs=1000,resolution_max=10, resolution_min=10,
                         const_feature=False, normalize=True, seed=2)

    #Policy
    policy = eGreedy(representation, epsilon=0.2)

    # Agent
    opt["agent"] = Q_Learning(representation=representation, policy=policy,
                       discount_factor=domain.discount_factor,
                       initial_learn_rate=0.875,
                       learn_rate_decay_mode="boyan", boyan_N0=1000,
                       lambda_=0.0)

    opt["checks_per_policy"] = 1
    opt["max_steps"] = 100
    opt["num_policy_checks"] = 10
    experiment = Experiment(**opt)
    return experiment, domain, policy, representation

if __name__ == '__main__':
    init_dq = [0.0, 0.0]
    l1 = 0.4
    l2 = 0.3
    foot = 0.04
    dt = 1e-2

    #setup 2 link arm
    arm = Arm2Base(init_dq, l1, l2, foot, dt = dt)

    experiment, domain, policy, representation = make_experiment(arm, 1)

    experiment.run(visualize_steps=True,  # should each learning step be shown?
                   visualize_learning=True,  # show policy / value function?
                   visualize_performance=1)  # show performance runs?

    experiment.plot()
    domain.showExploration()

    traj = []
    cur_state = domain.s0()[0]
    for j in range(0, domain.episodeCap):
        traj.append(cur_state)
        terminal = domain.isTerminal()

        if terminal:
            break

        a = policy.pi(cur_state, terminal, domain.possibleActions())
        cur_state = domain.step(a)[1]

    print traj
