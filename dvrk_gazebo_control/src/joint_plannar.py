import rospy
import numpy as np
from arm_planner import Planner
from set_joint_p import set_joint_p
import time

if __name__ == "__main__":
    rospy.init_node("manipulation")

    time.sleep(5)
    experiment = Planner(1)

    start = np.array([0.57043903, 2.9681703])
    goal = np.array([-0.57043903, -2.96817033])

    times, weight_list, traj_list = experiment.Correction(start, goal)
    for traj in traj_list:
        for q in traj:
            print experiment.robotToCartesian(q)
            set_joint_p(q)
