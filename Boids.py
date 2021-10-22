import pygame 
import random as rnd
import math
import numpy as np
from Vector_Operations import * 

# Simulation parameters
display_width = 1280
display_height = 720
top_speed = 5
num_of_boids = 20
num_rays = 30
follow_mouse = False	

cohesion = 0.8
alignment = 0.5
separation = 5
vision = 200

# Colours
white = (255,255,255)
black = (0,0,0)
red = (255,0,0)
blue = (58, 169, 234)

pygame.init()
gameDisplay = pygame.display.set_mode((display_width,display_height))
pygame.display.set_caption("Boids")
clock = pygame.time.Clock()
crashed = False
drawing = False

cycle_frame = 0		
animation = [pygame.image.load('Sprites/Fish1.png').convert_alpha(),pygame.image.load('Sprites/Fish2.png').convert_alpha(),pygame.image.load('Sprites/Fish1.png').convert_alpha(),pygame.image.load('Sprites/Fish3.png').convert_alpha()]*2
boids = []
obstacles = [ [[0,0],[display_width,0]] , [[0,0],[0,display_height]] , [[0,display_height],[display_width,display_height]] , [[display_width,0],[display_width,display_height]] ]
leader_exists = False
leader = None
leader_cycle = 0
acceleration = [0,0]

class Boid():
	def __init__(self, x, y):
		self.pos = [x,y]
		self.velocity = [rnd.uniform(-top_speed,top_speed),rnd.uniform(-top_speed,top_speed)]
		self.next_pos = [x,y]
		self.next_velocity = self.velocity

	def set_next_vel(self,v):
		if magnitude(v)<=top_speed:
			self.next_velocity = v
		else:
			vector = vector_times_scalar(get_unit_vector(v),top_speed)
			self.next_velocity = vector 	


for i in range(num_of_boids):
	boid = Boid(rnd.uniform(0,display_width),rnd.uniform(0,display_height))
	boids.append(boid)

def assign_leader(acceleration,leader_cycle,leader_exists,leader):
	if leader_cycle >= 360-1:
		leader_cycle = 0
		leader_exists = False
	if leader_cycle == 330-1:
		acceleration = vector_times_scalar(get_unit_vector([rnd.uniform(-1,1),rnd.uniform(-1,1)]),top_speed/5)
		index = rnd.randint(0,len(boids)-1)
		leader_exists = True
		leader = boids[index]

	leader_cycle+=1

def rotateimage(image,angle,coords):
	rotated = pygame.transform.rotate(image, angle)
	rect = rotated.get_rect(center=coords)
	return rotated, rect

def draw_frame(boids,cycle_frame):
	gameDisplay.fill(blue)
	if cycle_frame>=64-1:
		cycle_frame = 0
	for boid in boids:
		angle = get_signed_angle([0,-10],boid.velocity)*180/math.pi
		fish,rect = rotateimage(animation[cycle_frame//8],angle,boid.pos)
		gameDisplay.blit(fish,rect)	
	cycle_frame+=1
	for obstacle in obstacles:
		pygame.draw.line(gameDisplay,black,obstacle[0],obstacle[1],width = 2)
	return cycle_frame

def about_to_collide(line_of_sight,obstacle):
	x1 = line_of_sight[0][0]
	y1 = line_of_sight[0][1]
	x2 = line_of_sight[1][0]
	y2 = line_of_sight[1][1]
	x3 = obstacle[0][0]
	y3 = obstacle[0][1]
	x4 = obstacle[1][0]
	y4 = obstacle[1][1]
	a = np.array([[x1-x3, x3-x4],[y1-y3, y3-y4]])
	b = np.array([[x1-x2, x3-x4],[y1-y2, y3-y4]])
	c = np.array([[x2-x1, x1-x3],[y2-y1, y1-y3]])
	d = np.array([[x1-x2, x3-x4],[y1-y2, y3-y4]])
	t = np.linalg.det(a)/np.linalg.det(b)
	u = np.linalg.det(c)/np.linalg.det(d)
	return (t>=0 and t<=1 and u>=0 and u<=1)

def avoid_obstacles(boid):
	magVel = magnitude(boid.next_velocity)	
	sight_vector = vector_times_scalar(boid.next_velocity,vision/top_speed)
	line_of_sight = [boid.pos,vector_add(boid.pos,sight_vector)]
	angle = math.pi/num_rays
	for obstacle in obstacles:
		if about_to_collide(line_of_sight,obstacle):
			multiples = []	
			for n in range(1,(num_rays//2+1)):
				multiples.extend([n,-1*n])
			index = 0
			next_ray = rotate(sight_vector,multiples[index]*angle)
			blocked = True
			while blocked:
				blocked = False
				index+=1
				next_ray = rotate(sight_vector,multiples[index]*angle)
				line_of_sight = [boid.pos,vector_add(boid.pos,next_ray)]
				# pygame.draw.line(gameDisplay,red,boid.pos,line_of_sight[1],width =1)
				for obstacle2 in obstacles:
					if about_to_collide(line_of_sight,obstacle2):
						if index<len(multiples)-1:	
							blocked = True
							break
						else:
							next_ray = vector_times_scalar(boid.velocity,-1)

			boid.next_velocity = vector_times_scalar(get_unit_vector(next_ray),magVel)	
			break	

# def avoid_walls(boid):
# 	magVel = magnitude(boid.next_velocity)
# 	angle = math.pi/num_rays
# 	line_of_sight = vector_times_scalar(boid.next_velocity,vision/top_speed)
# 	multiples = [0]	
# 	for n in range(1,(num_rays//2+1)):
# 		multiples.extend([n,-1*n])
# 	index = 0
# 	next_ray = rotate(line_of_sight,multiples[index]*angle)
# 	destination = vector_add(boid.pos,next_ray)
# 	while (destination[0]>display_width or destination[0]<0 or destination[1]>display_height or destination[1]<0) and index<len(multiples)-1:
# 		index+=1
# 		next_ray = rotate(line_of_sight,multiples[index]*angle)
# 		destination = vector_add(boid.pos,next_ray)
# 	boid.next_velocity = vector_times_scalar(get_unit_vector(next_ray),magVel)

def follow_point(boid,point):
	vector = get_vector(boid.pos,point)
	if magnitude(vector)<=vision:
		change = vector_times_scalar(get_unit_vector(vector),0.5)
		boid.set_next_vel(vector_add(boid.next_velocity,change))

def follow_leader(boid,leader):
	vector = get_vector(boid.pos,leader.pos)
	if magnitude(vector)<=vision:
		difference = vector_subtract(boid.next_velocity,leader.velocity)
		vel_change = vector_times_scalar(difference,rnd.uniform(0,1))
		boid.set_next_vel(vector_add(boid.next_velocity,vel_change))

def update(boids,leader_exists,leader):
	
	for boid in boids:

		neighbours = [neighbour for neighbour in boids if ( (neighbour != boid) and (magnitude(get_vector(boid.pos,neighbour.pos))<=vision) )]
		
		if len(neighbours)>0:
			pos_total = [0,0]
			vel_total = [0,0]
			for neighbour in neighbours:
				#
				#
				# Check if fish are not separated by a wall
				#
				#
				pos_total = vector_add(pos_total,neighbour.pos)
				vel_total = vector_add(vel_total,neighbour.velocity)

				# Separation
				vector = get_vector(boid.pos,neighbour.pos)
				distance = magnitude(vector)
				unit = get_unit_vector(vector)
				force = vector_times_scalar(unit,-1*separation/distance)
				if distance<100:
					boid.set_next_vel(vector_add(boid.next_velocity,force))

			# Cohesion				
			CoM = vector_times_scalar(pos_total,1/len(neighbours))
			CoMvector = vector_times_scalar(get_unit_vector(get_vector(boid.pos,CoM)),cohesion)
			boid.set_next_vel(vector_add(boid.next_velocity,CoMvector))

			# Alignment
			avgVel = vector_times_scalar(vel_total,1/len(neighbours))
			difference = vector_subtract(boid.next_velocity,avgVel)
			vel_change = vector_times_scalar(difference,alignment)
			boid.set_next_vel(vector_add(boid.next_velocity,vel_change))

		if leader_exists:
			if boid == leader:
				boid.set_next_vel(vector_add(acceleration,boid.next_velocity))
			else:
				follow_leader(boid,leader)
			

		# Follow mouse
		if follow_mouse:
			mouse_pos = pygame.mouse.get_pos()
			follow_point(boid,mouse_pos)

		# Walls
		# if boid.pos[0]<0.1*display_width:
		# 	boid.velocity[0]+=0.5*top_speed
		# elif boid.pos[0]>display_width - 0.1*display_width:
		# 	boid.velocity[0]-=0.5*top_speed			
		# if boid.pos[1]<0.1*display_height:
		# 	boid.velocity[1]+=0.5*top_speed
		# elif boid.pos[1]>display_height - 0.1*display_height:
		# 	boid.velocity[1]-=0.5*top_speed

		avoid_obstacles(boid)
		# avoid_walls(boid)

	for boid in boids:
		boid.set_next_vel(vector_times_scalar(get_unit_vector(boid.next_velocity),magnitude(boid.next_velocity)+0.1))	
		boid.velocity = boid.next_velocity

		# check = vector_add(boid.pos,boid.velocity)
		# if check[0]<0 or check[0]>display_width:
		# 	boid.velocity[0] = 0
		# if check[1]<0 or check[1]>display_height:
		# 	boid.velocity[1] = 0
		boid.pos = vector_add(boid.pos,boid.velocity)

if __name__ == "__main__":

	while not crashed:
		
		for event in pygame.event.get():

			if event.type == pygame.QUIT:
				crashed = True
			
			if event.type == pygame.MOUSEBUTTONDOWN:
				if event.button == 1:
					follow_mouse = not follow_mouse
				elif event.button == 3:
					drawing = True
					draw_start = pygame.mouse.get_pos()
			elif event.type == pygame.MOUSEBUTTONUP:
				if event.button == 3:
					if drawing:
						drawing = False
						draw_end = pygame.mouse.get_pos()
						obstacles.append([draw_start,draw_end])

		cycle_frame = draw_frame(boids,cycle_frame)
		if drawing:
			pygame.draw.line(gameDisplay,black,draw_start,pygame.mouse.get_pos())
		update(boids,leader_exists,leader)
		assign_leader(acceleration,leader_cycle,leader_exists,leader)
	
		pygame.display.update()
		clock.tick(60)

	pygame.quit()
	quit()