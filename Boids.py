import pygame 
import random as rnd
import math
from Vector_Operations import * 

# Simulation parameters
display_width = 1280
display_height = 720
boid_radius = 5
top_speed = 5
num_of_boids = 25
num_sweeps = 10
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

class Boid():
	def __init__(self, x, y, colour):
		self.pos = [x,y]
		self.velocity = [rnd.uniform(-top_speed,top_speed),rnd.uniform(-top_speed,top_speed)]
		self.radius = boid_radius
		self.colour = colour
		self.drift = rnd.uniform(-1,1)
		self.next_pos = [x,y]
		self.next_velocity = self.velocity

	def set_next_vel(self,v):
		if magnitude(v)<=top_speed:
			self.next_velocity = v
		else:
			vector = vector_times_scalar(get_unit_vector(v),top_speed)
			self.next_velocity = vector 	

cycle_frame = 0		
animation = [pygame.image.load('Sprites/Fish1.png'),pygame.image.load('Sprites/Fish2.png'),pygame.image.load('Sprites/Fish1.png'),pygame.image.load('Sprites/Fish3.png')]*2
boids = []

for i in range(num_of_boids):
	boid = Boid(rnd.uniform(0,display_width),rnd.uniform(0,display_height),white)
	boids.append(boid)

def rotateimage(image,angle,coords):
	rotated = pygame.transform.rotate(image, angle)
	rect = rotated.get_rect(center=coords)
	return rotated, rect

def draw_frame(boids):
	global cycle_frame
	gameDisplay.fill(blue)
	if cycle_frame>=64-1:
		cycle_frame = 0
	for boid in boids:
		angle = get_signed_angle([0,-10],boid.velocity)*180/math.pi
		fish,rect = rotateimage(animation[cycle_frame//8],angle,boid.pos)
		gameDisplay.blit(fish,rect)	
	cycle_frame+=1

def avoid_walls(boid):
	magVel = magnitude(boid.next_velocity)
	angle = (2/3)*math.pi/num_sweeps
	sweeper = vector_times_scalar(boid.next_velocity,vision/top_speed)
	sweeps = [0]	
	for n in range(1,(num_sweeps//2+1)):
		sweeps.extend([n,-1*n])
	index = 0
	next_sweep = rotate(sweeper,sweeps[index]*angle)
	destination = vector_add(boid.pos,next_sweep)
	while (destination[0]>display_width or destination[0]<0 or destination[1]>display_height or destination[1]<0) and index<len(sweeps)-1:
		index+=1
		next_sweep = rotate(sweeper,sweeps[index]*angle)
		destination = vector_add(boid.pos,next_sweep)
	boid.next_velocity = vector_times_scalar(get_unit_vector(next_sweep),magVel)

def update(boids):
	
	for boid in boids:

		neighbours = [neighbour for neighbour in boids if ( (neighbour != boid) and (magnitude(get_vector(boid.pos,neighbour.pos))<=vision) )]
		
		if len(neighbours)>0:
			pos_total = [0,0]
			vel_total = [0,0]
			for neighbour in neighbours:
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

		#Follow mouse
		if follow_mouse:
			mouse_pos = pygame.mouse.get_pos()
			mouse_vector = get_vector(boid.pos,mouse_pos)
			if magnitude(mouse_vector)<=vision:
				change = vector_times_scalar(get_unit_vector(mouse_vector),0.5)
				boid.set_next_vel(vector_add(boid.next_velocity,change))

		# Walls
		# if boid.pos[0]<0.1*display_width:
		# 	boid.velocity[0]+=0.5*top_speed
		# elif boid.pos[0]>display_width - 0.1*display_width:
		# 	boid.velocity[0]-=0.5*top_speed			
		# if boid.pos[1]<0.1*display_height:
		# 	boid.velocity[1]+=0.5*top_speed
		# elif boid.pos[1]>display_height - 0.1*display_height:
		# 	boid.velocity[1]-=0.5*top_speed

		avoid_walls(boid)

	for boid in boids:
		boid.set_next_vel(vector_times_scalar(boid.next_velocity,1.01))	
		boid.velocity = boid.next_velocity

		check = vector_add(boid.pos,boid.velocity)
		if check[0]<0 or check[0]>display_width:
			boid.velocity[0] = 0
		if check[1]<0 or check[1]>display_height:
			boid.velocity[1] = 0
		boid.pos = vector_add(boid.pos,boid.velocity)

pygame.init()
gameDisplay = pygame.display.set_mode((display_width,display_height))
pygame.display.set_caption("Boids")
clock = pygame.time.Clock()
crashed = False
if __name__ == "__main__":

	while not crashed:
		
		for event in pygame.event.get():

			if event.type == pygame.QUIT:
				crashed = True

			if event.type ==pygame.MOUSEBUTTONDOWN:
				if event.button == 1:
					follow_mouse = not follow_mouse
		
		draw_frame(boids)
		update(boids)			
	
		pygame.display.update()
		clock.tick(64)

	pygame.quit()
	quit()