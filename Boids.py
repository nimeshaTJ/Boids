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
num_rays = 50
follow_mouse = False	
sampling_rate = 5

cohesion = 0.8
alignment = 0.5
separation = 5
vision = 200

chunk_rows = int(display_height/vision)+3
chunk_cols = int(display_width/vision)+3

# Colours
white = (255,255,255)
black = (0,0,0)
red = (255,0,0)
blue = (58, 169, 234)
green = (0,255,0)

pygame.init()
gameDisplay = pygame.display.set_mode((display_width,display_height))
pygame.display.set_caption("Boids")
clock = pygame.time.Clock()
crashed = False
drawing = False
show_chunks = False

cycle_frame = 0		
animation = [pygame.image.load('Sprites/Fish1.png').convert_alpha(),pygame.image.load('Sprites/Fish2.png').convert_alpha(),pygame.image.load('Sprites/Fish1.png').convert_alpha(),pygame.image.load('Sprites/Fish3.png').convert_alpha()]*2
boids = []
chunks_raw = np.empty((chunk_rows,chunk_cols),dtype=list) # array of chunks
chunks_processed = chunks_raw.copy()
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

for r in range(chunk_rows):
	for c in range(chunk_cols):
		chunks_raw[r,c] = []

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

def draw_frame(boids,cycle_frame,drawing,follow_mouse):
	gameDisplay.fill(blue)
	if cycle_frame>=64-1:
		cycle_frame = 0
	for boid in boids:
		angle = get_signed_angle([0,-10],boid.velocity)*180/math.pi
		fish,rect = rotateimage(animation[cycle_frame//8],angle,boid.pos)
		gameDisplay.blit(fish,rect)
		# pygame.draw.circle(gameDisplay,red,(boid.pos[0],boid.pos[1]),vision,width=2)	
	cycle_frame+=1
	for obstacle in obstacles:
		pygame.draw.line(gameDisplay,black,obstacle[0],obstacle[1],width=2)
	if drawing:
		pygame.draw.line(gameDisplay,black,draw_start,pygame.mouse.get_pos())
	if follow_mouse:
		pygame.draw.circle(gameDisplay,green,pygame.mouse.get_pos(),5)
	
	return cycle_frame

def draw_chunks(chunks,selected):
	for r in range(1,chunk_rows-1):
		for c in range(1,chunk_cols-1):
			# if len(chunks[r,c])>0:
			# 	pygame.draw.rect(gameDisplay,red,((c-1)*vision,(r-1)*vision,int(vision),int(vision)),2)
			# else:
			pygame.draw.rect(gameDisplay,white,((c-1)*vision,(r-1)*vision,int(vision),int(vision)),2)
	highlighted_chunks = []
	if selected:
		for line in chunks[selected[0],selected[1]]:
			highlighted = get_line_chunks(line)
			for c in highlighted:
				if not (c in highlighted_chunks):
					highlighted_chunks.append(c)
		for chunk in highlighted_chunks:
			pygame.draw.rect(gameDisplay,red,((chunk[1]-1)*vision,(chunk[0]-1)*vision,int(vision),int(vision)),2)

def intersect(line1,line2):
	x1,y1 = line1[0][0],line1[0][1]
	x2,y2 = line1[1][0],line1[1][1] 
	x3,y3 = line2[0][0],line2[0][1]
	x4,y4 = line2[1][0],line2[1][1]
	t = ((x1-x3)*(y3-y4) - (y1-y3)*(x3-x4))/((x1-x2)*(y3-y4) - (y1-y2)*(x3-x4))
	u = ((x1-x3)*(y1-y2) - (y1-y3)*(x1-x2))/((x1-x2)*(y3-y4) - (y1-y2)*(x3-x4))
	return (t>=0 and t<=1 and u>=0 and u<=1)	

def avoid_obstacles(boid,chunks_processed,r,c):
	magVel = magnitude(boid.next_velocity)	
	sight_vector = vector_times_scalar(boid.next_velocity,vision/top_speed)
	line_of_sight = [boid.pos,vector_add(boid.pos,sight_vector)]
	angle = math.pi/num_rays
	for obstacle in chunks_processed[r,c]:

		if intersect(line_of_sight,obstacle):
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
				for obstacle2 in chunks_processed[r,c]:
					if intersect(line_of_sight,obstacle2):
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

def distance(point1,point2):
	return math.sqrt( (point2[0]-point1[0])**2 + (point2[1]-point1[1])**2 )

def get_chunk(point):
	c = int(point[0]/vision)+1
	r = int(point[1]/vision)+1
	return (r,c)

def get_line_chunks(line):
	unit_vector = get_unit_vector(get_vector(line[0],line[1]))
	sample_vector = vector_times_scalar(unit_vector,sampling_rate)
	chunks_of_line = [get_chunk(line[0])] 
	current_point = line[0]
	for n in range(int(distance(line[0],line[1])/sampling_rate)):
		current_point = vector_add(current_point,sample_vector)
		chunk = get_chunk(current_point)
		if not (chunk in chunks_of_line):
			chunks_of_line.append(chunk)
	return chunks_of_line

def process_chunks(chunks_raw):
	processed = chunks_raw.copy()
	for r in range(1,len(chunks_raw)-1):
		for c in range(1,len(chunks_raw[0])-1):
			surrounding = [[r-1,c-1],[r-1,c],[r-1,c+1],[r,c-1],[r,c],[r,c+1],[r+1,c-1],[r+1,c],[r+1,c+1]]
			lines = []
			for index in surrounding:
				line_list = chunks_raw[index[0],index[1]]
				for line in line_list:
					if not (line in lines):
						lines.append(line)
			processed[r,c] = lines
	return processed

def update(boids,leader_exists,leader,chunks_processed):
	
	for boid in boids:

		r,c = int(boid.pos[1]/vision)+1,int(boid.pos[0]/vision)+1 		# r,c of the chunk that the boid is in
		neighbours = [neighbour for neighbour in boids if ( (neighbour != boid) and (magnitude(get_vector(boid.pos,neighbour.pos))<=vision) )]
		for i in range(len(neighbours)-1,-1,-1):
			distance_line = [boid.pos,neighbours[i].pos]
			blocked = False
			x=0
			for obstacle in chunks_processed[r,c]:
				if intersect(distance_line, obstacle):
					neighbours.pop(i)
					break

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

		if leader_exists:
			if boid == leader:
				boid.set_next_vel(vector_add(acceleration,boid.next_velocity))
			else:
				follow_leader(boid,leader)
			
		# Follow mouse
		if follow_mouse:
			mouse_pos = pygame.mouse.get_pos()
			line = [boid.pos,mouse_pos]
			blocked = False
			for obstacle in chunks_processed[r,c]:
				if intersect(line,obstacle):
					blocked = True 
					break
			if not blocked:
				follow_point(boid,mouse_pos)

		avoid_obstacles(boid,chunks_processed,r,c)
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

	for obstacle in obstacles:
		for chunk in get_line_chunks(obstacle):
			chunks_raw[chunk[0],chunk[1]].append(obstacle)
	chunks_processed = process_chunks(chunks_raw)
	selected = None

	while not crashed:

		for event in pygame.event.get():

			if event.type == pygame.QUIT:
				crashed = True
			
			if event.type == pygame.KEYDOWN:
				if event.key == pygame.K_ESCAPE:
					crashed = True

				if event.key == pygame.K_c:
					show_chunks = not show_chunks

			if event.type == pygame.MOUSEBUTTONDOWN:
				if event.button == 1:
					follow_mouse = not follow_mouse
				elif event.button == 2:
					if selected != get_chunk(pygame.mouse.get_pos()):
						selected = get_chunk(pygame.mouse.get_pos())
					else:
						selected = None
				elif event.button == 3:
					drawing = True
					draw_start = pygame.mouse.get_pos()
			elif event.type == pygame.MOUSEBUTTONUP:
				if event.button == 3:
					if drawing:
						drawing = False
						draw_end = pygame.mouse.get_pos()
						line = [draw_start,draw_end]
						obstacles.append(line)
						for chunk in get_line_chunks(line):
							chunks_raw[chunk].append(line)
						chunks_processed = process_chunks(chunks_raw)

		cycle_frame = draw_frame(boids,cycle_frame,drawing,follow_mouse)
		if show_chunks:
			draw_chunks(chunks_processed,selected)
		update(boids,leader_exists,leader,chunks_processed)
		assign_leader(acceleration,leader_cycle,leader_exists,leader)
		pygame.display.update()
		clock.tick(60)

	pygame.quit()
	quit()