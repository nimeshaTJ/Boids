import pygame 
import random as rnd
import math
from Vector_Operations import * 

# Simulation parameters
display_width = 1540
display_height = 780
boid_radius = 5
top_speed = 10
num_of_boids = 50

cohesion = 0.8
alignment = 0.5
separation = 10
vision = 200

# Colours
white = (255,255,255)
black = (0,0,0)
red = (255,0,0)

class Boid():
	def __init__(self, x, y, colour):
		self.pos = [x,y]
#		self.velocity = [rnd.uniform(-top_speed,top_speed),rnd.uniform(-top_speed,top_speed)]
		self.velocity = [0,0]
		self.radius = boid_radius
		self.colour = colour
		self.next_pos = [x,y]
		self.next_velocity = self.velocity

	def set_next_vel(self,v):
		if magnitude(v)<=top_speed:
			self.next_velocity = v
		else:
			vector = top_speed*get_unit_vector(v)
			self.next_velocity = vector 

def draw_boids(boids):
	gameDisplay.fill(black)
	for boid in boids:
		pygame.draw.circle(gameDisplay,boid.colour,boid.pos,boid.radius)

def update(boids):
	
	for boid in boids:

		# if boid != leader:
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
		# else:
		# 	mouse_pos = pygame.mouse.get_pos()
		# 	vector =get_vector(boid.pos,mouse_pos)
		# 	boid.set_next_vel(vector)

		# Walls
		if boid.pos[0]<0.1*display_width:
			boid.velocity[0]+=1
		if boid.pos[0]>display_width - 0.1*display_width:
			boid.velocity[0]-=1			
		if boid.pos[1]<0.1*display_height:
			boid.velocity[1]+=1
		if boid.pos[1]>display_height - 0.1*display_height:
			boid.velocity[1]-=1

	for boid in boids:
		boid.set_next_vel(vector_times_scalar(boid.next_velocity,1.01))	
		boid.velocity = boid.next_velocity
		boid.pos = [boid.pos[0]+boid.velocity[0],boid.pos[1]+boid.velocity[1]]			

boids = []

for i in range(num_of_boids):
	boid = Boid(rnd.uniform(0,display_width),rnd.uniform(0,display_height),white)
	boids.append(boid)
# leader = Boid(0,0,red)
# boids.append(leader)


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

		update(boids)			
		draw_boids(boids)
		pygame.display.update()
		clock.tick(60)

	pygame.quit()
	quit()