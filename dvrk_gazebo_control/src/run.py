import numpy as np
import planner
import arm2base
import math
from set_joint_p import set_joint_p

if __name__ == '__main__':
    init_dq = [0.0, 0.0]
    l1 = 0.4
    l2 = 0.3
    foot = 0.04
    dt = 1e-2

    #setup 2 link arm
    arm = arm2base.Arm2Base(init_dq, l1, l2, foot, dt = dt)
    arm.q = init_dq
    #setup target position
    #traj =[[0,  0],[-0.02372982, -0.74681801],[ 0.04902266, -1.39542123],[0.25,-1.9]]
    #traj = [[ 0.,0.],
    #       [-1.01235026, -1.47345205],
    #       [-0.90414561, -1.94490009],
    #       [ 0.25, -1.9]]
    #traj = [[ 0.,          0.        ],
    #        [ 0.04599779, -0.68174785],
    #        [ 0.12287343, -1.32429418],
    #        [ 0.25,       -1.9       ]]
    #traj = [[ 0.,          0.        ],
    #        [ 0.08333333, -0.63333333],
    #        [ 0.16666667, -1.26666664],
    #        [ 0.25,       -1.9       ]]
    # start_xy = np.array([0.74, 0])
    target_xy = np.array([0.1,0.1])
    # init_waypts = np.zeros((10, 2))
    #for i in range(10):
    #    iner_pt = start_xy + i/9.0 *(target_xy - start_xy)
    #    print iner_pt
    #    init_waypts[i,:] = arm.inv_kinematics(iner_pt.tolist())

    #print init_waypts
    target_q = arm.inv_kinematics(target_xy)
    #start_q = arm.q
    print target_q, arm.position(target_q)
    #for q in traj:
    #    pos = arm.position(q)[:,2]
    #    print pos
