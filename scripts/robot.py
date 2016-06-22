#! /usr/bin/env python
'''
Author: Lucas Tindall
'''
from __future__ import division
import rospy
import itertools
import random as r
import math as m
import numpy as np
from copy import deepcopy
from std_msgs.msg import String, Float32, Bool
#from cse_190_assi_1.msg import temperatureMessage, RobotProbabilities
#from cse_190_assi_1.srv import requestMapData, moveService, requestTexture
from read_config import read_config
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import OccupancyGrid, MapMetaData
from map_utils import Map
from helper_functions import get_pose,move_function
from sklearn.neighbors import KDTree
from sensor_msgs.msg import LaserScan


class Particle(): 
    def __init__(self,x,y,angle,weight,pose, index): 
	self.y = y  
	self.x = x
	self.angle = angle
	self.weight = weight
	self.pose = pose
	self.index = index

    def turn_particle(self,angle, sigma_angle): 
	self.angle += (angle*m.pi/180) + r.gauss(0,sigma_angle)
	self.pose = get_pose(self.x, self.y, self.angle) 

    def move_particle(self,distance, sigma_x, sigma_y):
	
	self.x += (distance*m.cos(self.angle) + 
	    r.gauss(0,sigma_x))
	self.y += (distance*m.sin(self.angle) + 
	    r.gauss(0,sigma_y))

	self.pose = get_pose(self.x, self.y, self.angle)


class robot(): 
    def __init__(self): 
	self.config = read_config()
	rospy.init_node("robot")
	self.map_subscriber = rospy.Subscriber(
		"/map",
		OccupancyGrid, 
		self.handle_map_subscriber
	)
	self.laser_subscriber = rospy.Subscriber(
		"/base_scan",
		LaserScan, 
		self.handle_laser_subscriber
	)
	self.likelihood_publisher = rospy.Publisher(
		"/likelihood_field",
		OccupancyGrid,
		queue_size = 10, 
		latch = True
	)
	self.particlecloud_publisher = rospy.Publisher(
		"/particlecloud",
		PoseArray,
		queue_size = 10, 
		latch = True
	)

	self.result_publisher = rospy.Publisher(
		"/result_update",
		Bool,
		queue_size = 10 
	)
	self.complete_publisher = rospy.Publisher(
		"/sim_complete",
		Bool,
		queue_size = 10 
	)
	self.laserScan = LaserScan()	
	self.move_index = 0
	self.moves = self.config['move_list']
	self.baseMapSet = False
	self.startedSim = False
	rospy.spin()


    def turn_all_particles(self, particles, angle, sigma_angle ): 
	for particle in particles: 
	    #print "turning particles" 
	    particle.turn_particle(angle, sigma_angle)
    
    def move_all_particles(self, particles, distance, sigma_x, sigma_y): 
	for particle in particles:
	    #print "moving particles"  
	    particle.move_particle(distance, sigma_x, sigma_y) 
    
#    def update_particles(event,self): 
#	self.particlecloud_publisher.publish(self.pose_array)
	



    def handle_map_subscriber(self, message): 
	"""
	Callback function for getting map data
	"""
	#rospy.sleep(25)
	rospy.sleep(20)
	self.baseMap = Map(message)
	self.baseMapMessage = message 
	self.baseMapSet = True
	if(self.startedSim == False):
	    self.startedSim == True
	    print "map is ready" 
	    self.start_sim()

    def handle_laser_subscriber(self,message): 
	self.laserScan = message

    def start_sim(self):
	particles = []
        self.pose_array = PoseArray()
        self.pose_array.header.stamp = rospy.Time.now()
        self.pose_array.header.frame_id = 'map' 
        self.pose_array.poses = []
	for i in range(self.config['num_particles']):
	    #randomX = r.randint(0,self.baseMap.width)
	    #randomY = r.randint(0,self.baseMap.height)
	    randomX = self.baseMapMessage.info.width*np.random.rand()
	    randomY = self.baseMapMessage.info.height * np.random.rand()
	    while(self.baseMap.get_cell(randomX,randomY) != 0): 		
                randomX = self.baseMapMessage.info.width*np.random.rand()
	        randomY = self.baseMapMessage.info.height * np.random.rand()
	    randomAngle = 2*m.pi * np.random.rand()
	    #randomAngle = 0
	    weight = 1/self.config['num_particles']
	    pose =  get_pose(randomX, randomY, randomAngle)
	    tempParticle = Particle(randomX, randomY, randomAngle,weight, pose, i)
	    particles.append(tempParticle)
	    self.pose_array.poses.append(particles[i].pose)


	self.particlecloud_publisher.publish(self.pose_array)


	occupied = []
	points = []
	for i in range(self.baseMap.height):
	    for j in  range(self.baseMap.width): 
		points.append(self.baseMap.cell_position(i,j))
		x,y = self.baseMap.cell_position(i,j)
		if self.baseMap.get_cell(x,y) == 1 : 
		    occupied.append(self.baseMap.cell_position(i,j))
		
	np.asarray(occupied)
	np.asarray(points)
	kdt = KDTree(occupied)
	(dist,indices) = kdt.query(points, k=1)

	self.likelihood = Map(self.baseMapMessage)
	index = 0
	for i in range(self.likelihood.height): 
	    for j in range(self.likelihood.width): 
		x,y = self.likelihood.cell_position(i,j)
	    	#if self.likelihood.get_cell(x,y) == 0 :
		#val = (1/(m.sqrt(2*m.pi)*self.config['laser_sigma_hit'])) * m.exp(-m.pow(dist[index] ,2)/(2*m.pow(self.config['laser_sigma_hit'],2)) )
		val =  m.exp(-m.pow(dist[index] ,2)/(2*m.pow(self.config['laser_sigma_hit'],2)) )
		if self.likelihood.get_cell(x,y) == 1: 
		    val = 0
		self.likelihood.set_cell(x,y,val) 
		index += 1 
	
	self.likelihood_publisher.publish(self.likelihood.to_message())
	

	for move in self.moves: 
	    self.result_publisher.publish(True)
	    angle = move[0]
	    distance = move[1]
	    steps = move[2]
	    move_function(angle,0)
	    
	    
    	    if(self.move_index == 0): 
	        self.turn_all_particles(particles,angle, self.config['first_move_sigma_angle'])	 
	        #self.turn_all_particles(particles,angle, 0.22)	 
		resample_sigma_angle = self.config['resample_sigma_angle']
	    	#resample_sigma_angle = 0.22
	    else: 
		self.turn_all_particles(particles,angle, 0)
		resample_sigma_angle = self.config['resample_sigma_angle']
	   
	    for i in range(steps): 
		move_function(0,distance)
		
		
		if(self.move_index == 0): 
		    #self.move_all_particles(particles,distance, 4.4,4.4) 
		    resample_sigma_x = self.config['resample_sigma_x'] 
		    resample_sigma_y = self.config['resample_sigma_y']
		    #resample_sigma_x = 4.4
		    #resample_sigma_y = 4.4
		    self.move_all_particles(particles,distance, 
		        self.config['first_move_sigma_x'],self.config['first_move_sigma_y']) 
		else: 
		    self.move_all_particles(particles,distance,0,0) 
		    resample_sigma_x = self.config['resample_sigma_x'] 
		    resample_sigma_y = self.config['resample_sigma_y']
		    
		
		self.pose_array.poses = []
		#print "pose_array" 
		#print self.pose_array.poses

		for i in range(self.config['num_particles']):
		    self.pose_array.poses.append(particles[i].pose)
		
		#rospy.Timer(rospy.Duration(1),self.update_particles)
		self.particlecloud_publisher.publish(self.pose_array)
		# re-weigh particlesi
		
		#print self.laserScan.ranges
		normalizationConstant = 0
		badParticles = []
		oldWeights = []
		for particle in particles: 
		    #calculate endpoint
		    # angle calculation 
		    # laser_scan_local(with respect to robot) = angle_min+index*angle_increment
		    # laser_scan_global(with respect to map) = laser_scan_local+theta of robot
		
		    # pz[] probabilities where laser hit obstacle 
		    # pz = laser_z_hit* Lp * laser_z_rand
		    pz = []
		    rangeIndex = 0
		    badSenseCount = 0 
		    gridValue = self.baseMap.get_cell(particle.x, particle.y)
		    for scanDist in self.laserScan.ranges: 
			# find location/angle of particle then add add on dist/angle increment 
			# above should be location of endpoint 
			# find likelihood field value at endpoint and add it onto pz
			endPointX = particle.x + scanDist*m.cos(particle.angle + 
					(self.laserScan.angle_min+rangeIndex*
					self.laserScan.angle_increment))
			endPointY = particle.y + scanDist*m.sin(particle.angle + 
					(self.laserScan.angle_min+rangeIndex* 
					self.laserScan.angle_increment))
			
			likelihood = self.likelihood.get_cell(endPointX, endPointY)
			badParticle = False
			# check if particle location is in an obstacle or out of the map 
			if(m.isnan(gridValue) or gridValue == 1):
			#if(m.isnan(gridValue)):
			    #print "gridValue = ",gridValue, "likelihood = ",likelihood
			    likelihood = 0
			    badParticle = True
			    badParticles.append(particle.index)
			    break  
			badSense = False
			
			if(m.isnan(likelihood)): 
			    likelihood = 0 
			    badSense = True
			    badSenseCount += 1
			    #print "bad sense"
			p = self.config['laser_z_hit']*likelihood+self.config['laser_z_rand']
			#if(badSense or likelihood == 0): 
			#    p = 0
			pz.append(m.pow(p,1))
			rangeIndex += 1
		    #print "weight = ",particle.weight
		    #print "pz = ", pz

		    ptot = sum(pz)
		    #ptot = np.product(pz)
		  
		    """ 
		    if(badSenseCount > 20 ) : 
			print "badSenseCount = ",badSenseCount 
		    """
		    oldWeights.append(particle.weight)
		    #if(badParticle or badSenseCount >15): 
		    if(badParticle):
			particle.weight = 0
		    else: 
		        #print "ptot = ",ptot
			particle.weight = particle.weight*(1/(1+m.exp(-ptot)))
		    normalizationConstant += particle.weight
		weights = []
		i = 0
		for particle in particles: 
		    particle.weight = particle.weight/normalizationConstant
		    #print "old weight = ",oldWeights[i]
		    i += 1 
	            #print "new weight = ", particle.weight
		    weights.append(particle.weight) 
		newParticles = []
		"""
		print "sum of weights = "
	        print sum(weights)
		print "reweighing" 
		"""
		for i in range(self.config['num_particles']): 
		    tempParticle = np.random.choice(particles, 1, replace=True, p=weights)
		    tempX = tempParticle[0].x + r.gauss(0,resample_sigma_x)
		    #print "Temp X = ",tempX
		    if tempParticle[0].index in badParticles: 
			print "It picked a bad Particle!!!"
		    tempY = tempParticle[0].y + r.gauss(0,resample_sigma_y)
		    #print "Temp Y = ",tempY
		    tempAngle = tempParticle[0].angle + r.gauss(0,resample_sigma_angle)
		    tempPose = get_pose(tempX, tempY, tempAngle)
		    #print "Temp Particle weight = ",tempParticle[0].weight
		    newParticles.append(Particle(tempX,tempY,tempAngle,tempParticle[0].weight,tempPose, i))
		particles = newParticles
	
	    self.move_index += 1 		
		        	
        			

	#rospy.sleep(20)
	
  	self.complete_publisher.publish(True)
	rospy.sleep(5)
	rospy.signal_shutdown("completed all moves, shutting down")
	"""
	7.6
	https://piazza.com/class/im16013n43027q?cid=249
	"""	
	rospy.spin()
	rospy.spin()

if __name__ == '__main__':
    robo = robot()		
 
		

