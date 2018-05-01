#!/usr/bin/env python
import numpy as np
import math
#import sympy


def dhTransform(theta,d,a,alpha):

	T = np.dot(np.dot(rotationMatrix(theta,'z'),translationMatrix(d,'z')),np.dot(translationMatrix(a,'x'),rotationMatrix(alpha,'x')))
	for r in range (0,4):
		for c in range (0,4):
			if abs(T[r,c]) < 1e-4:
				T[r,c] = 0

	return T


def translationMatrix(d,t_ax):

	if t_ax == 'x':
		T = np.array([[1,0,0,d],[0,1,0,0],[0,0,1,0],[0,0,0,1]])

	elif t_ax == 'y':
		T = np.array([[1,0,0,0],[0,1,0,d],[0,0,1,0],[0,0,0,1]])

	elif t_ax == 'z':
		T = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,d],[0,0,0,1]])

	elif t_ax == 'all':
		T = np.array([[1,0,0,d[0]],[0,1,0,d[1]],[0,0,1,d[2]],[0,0,0,1]])

	else:
		print('invalid axis')

	return	T


def rotationMatrix(theta,r_ax):

	if r_ax == 'x':
		R = np.array([[1,0,0,0],[0,math.cos(theta),-math.sin(theta),0],[0,math.sin(theta),math.cos(theta),0],[0,0,0,1]])

	elif r_ax == 'y':
		R = np.array([[math.cos(theta),0,math.sin(theta),0],[0,1,0,0],[-math.sin(theta),0,math.cos(theta),0],[0,0,0,1]])

	elif r_ax == 'z':
		R = np.array([[math.cos(theta),-math.sin(theta),0,0],[math.sin(theta),math.cos(theta),0,0],[0,0,1,0],[0,0,0,1]])
	else:
		print('invalid axis')

	return R
