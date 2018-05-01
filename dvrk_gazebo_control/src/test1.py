import rospy
from set_joint_p import set_joint_p

if __name__ == "__main__":
    rospy.init_node("manipulation")

    q = [0.5, 0.2]
    set_joint_p(q)
