import math
import numpy as np

def vector_times_scalar(v, s):
	return [(v[0] * s), (v[1] * s) ]

def vector_add(v1, v2):
	return [(v1[0] + v2[0]), (v1[1] + v2[1])]

def vector_subtract(v1, v2):
	return [(v2[0] - v1[0]), (v2[1] - v1[1])]

def dot(v1, v2):
	return ((v1[0]*v2[0]) + (v1[1]*v2[1]))

def cross(v1,v2):
	return [(v1[1]*v2[2] - v1[2]*v2[1]), (v1[2]*v2[0] - v1[0]*v2[2]), (v1[0]*v2[1] - v1[1]*v2[0])]

def magnitude(v):
	if len(v)==2:
		return math.hypot(v[0], v[1])
	elif len(v)==3:
		return math.hypot(v[0],v[1],v[2])

def get_perp(v):
	return [-v[1], v[0]]

def get_vector(p1, p2):
	return [(p2[0]-p1[0]), (p2[1]-p1[1])]

def get_unit_vector(v):
	if magnitude(v)>0:
		return vector_times_scalar(v, (1/magnitude(v)))
	else:
		return [0,0]

def get_angle(v1,v2):
	if len(v1)<3:
		v1.extend([0])
	if len(v2)<3:
		v2.extend([0])
	s = magnitude(cross(v1,v2))
	c = dot(v1,v2)

	angle = -math.atan2(s,c)
	return angle

def get_signed_angle(v1,v2):
	#Only works for 2D vectors
	v1.extend([0])
	v2.extend([0])
	s = cross(v1,v2)[2]
	c = dot(v1,v2)

	angle = -math.atan2(s,c)
	return angle

def rotate(v,angle):
	vector = np.transpose(v)
	Rmatrix = np.array([[math.cos(angle),-1*math.sin(angle)],[math.sin(angle),math.cos(angle)]])
	return np.transpose(np.matmul(Rmatrix,vector))