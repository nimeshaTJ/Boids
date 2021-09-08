import math

def vector_times_scalar(v, s):
	return [(v[0] * s), (v[1] * s) ]

def vector_add(v1, v2):
	return [(v1[0] + v2[0]), (v1[1] + v2[1])]

def vector_subtract(v1, v2):
	return [(v2[0] - v1[0]), (v2[1] - v1[1])]

def dot(v1, v2):
	return ((v1[0]*v2[0]) + (v1[1]*v2[1]))

def magnitude(v):
	return math.hypot(v[0], v[1])

def get_perp(v):
	return [-v[1], v[0]]

def get_vector(p1, p2):
	return [(p2[0]-p1[0]), (p2[1]-p1[1])]

def get_unit_vector(v):
	if magnitude(v)>0:
		return vector_times_scalar(v, (1/magnitude(v)))
	else:
		return [0,0]