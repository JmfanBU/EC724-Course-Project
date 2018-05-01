import numpy as np
import json
from rospkg import RosPack
from dh_transform import dhTransform
from dh_transform import translationMatrix

pi = np.pi

#import DH table
rospack = RosPack()
path = rospack.get_path('dvrk_gazebo_control') + '/DH_model/psm-large-needle-driver-tip.json'
with open(path,'r') as json_file:
    json_data = json.load(json_file)

# Construc DH table
joint_list = json_data['DH']['links']
joint_num = len(joint_list)
dh_table = np.zeros((joint_num,4))
for i in range(0,joint_num):
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
    else:
        offset = joint_list[i]['offset']
    dh_table[i][0] = joint_list[i]['theta'] + offset
    dh_table[i][1] = joint_list[i]['D']
    dh_table[i][2] = joint_list[i]['A']
    dh_table[i][3] = alpha

cm1 = np.array([[1],[1],[1]])
cm2 = np.array([[1],[1],[1]])
cm3 = np.array([[1],[1],[1]])
cm4 = np.array([[1],[1],[1]])
cm5 = np.array([[1],[1],[1]])
cm6 = np.array([[1],[1],[1]])
cm = np.concatenate((cm1,cm2,cm3,cm4,cm5,cm6),axis=1)
def forwardKinematics(i,dh_table):
#Perform forward kinematics and return a transformation matrix of given DH parameters
#at ith link

	T = np.identity(4)
	#print i
	for j in range (0,i):
		#print "this is loop: ",j+1
		ith_dh_table = dh_table[j]
		theta = ith_dh_table[0]
		d = ith_dh_table[1]
		a = ith_dh_table[2]
		alpha = ith_dh_table[3]
		T_trans = dhTransform(theta,d,a,alpha)
		T = np.dot(T,T_trans)

	return T

def forwardPositionKinematics(i,dh_table,cm):
#compute the transformation and position of the center of mass of ith link

	T = np.identity(4)
	#print i
	for j in range (0,i):
		#print "this is loop: ",j+1
		ith_dh_table = dh_table[j]
		theta = ith_dh_table[0]
		d = ith_dh_table[1]
		a = ith_dh_table[2]
		alpha = ith_dh_table[3]
		T_trans = dhTransform(theta,d,a,alpha)
		T = np.dot(T,T_trans)
	cm_transl = translationMatrix(cm,'all')
	ith_T_cm = np.dot(T,cm_transl)
	P_Ci = ith_T_cm[0:3,3]
	return P_Ci

# print forwardPositionKinematics(5, dh_table, cm)
