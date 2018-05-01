from forwardKin import forwardPositionKinematics
import json
from rospkg import RosPack
import numpy as np
import rospy
from sensor_msgs.msg import JointState

pi = np.pi
rospack = RosPack()
currentJointState = JointState()
#import DH model
def import_DH_model(model):
    path = rospack.get_path('dvrk_gazebo_control') + '/DH_model/' + model
    with open(path, 'r') as json_file:
        json_data = json.load(json_file)
    joint_list = json_data['DH']['links']
    joint_num = len(joint_list)
    dh_table = np.zeros((joint_num,4))
    for i in range(0,joint_num):
        offset = 0
        offset1 = 0
        if joint_list[i]['alpha'] == 1.5708:
            alpha = pi/2
        elif joint_list[i]['alpha'] == -1.5708:
            alpha = -pi/2
        else:
            alpha = joint_list[i]['alpha']
        if joint_list[i]['offset'] == 1.5708:
            offset = pi/2
        elif joint_list[i]['offset'] == -1.5708:
            offset = -pi/2
        elif joint_list[i]['offset'] == 3.1415:
            offset = pi
        else:
            offset1 = joint_list[i]['offset']

        dh_table[i][0] = joint_list[i]['theta'] + offset
        dh_table[i][1] = joint_list[i]['D'] + offset1
        dh_table[i][2] = joint_list[i]['A']
        dh_table[i][3] = alpha
    cm = np.array([[0],[0],[0]])

    return dh_table,cm

def Joint_states_callback(msg):
    global currentJointState
    currentJointState = msg

def current_Joint_position():
    rospy.init_node('Kinematics', anonymous=True)
    while not currentJointState.position:
        rospy.Subscriber("/dvrk_psm/joint/states", JointState, Joint_states_callback)
    dh_table, cm = import_DH_model('psm-large-needle-driver-tip.json')
    print currentJointState
    for i in range(0, dh_table.shape[0]):
        if i == 2:
            dh_table[i][1] += currentJointState.position[i]
        else:
            dh_table[i][0] += currentJointState.position[i]
    print dh_table
    position = forwardPositionKinematics(6, dh_table,cm) + np.array([[-0.499699, -0.00001858, 0.58629]])
    return position

if __name__ == '__main__':
    print current_Joint_position()
