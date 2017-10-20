#!/usr/bin/env python  
'''
Snake Monster stabilization with CPG controls
 Created 24 May 2017
 requires the HEBI API and the Snake Monster HEBI API

 Setting up the Snake Monster

 NOTE: If the modules have been changed, first correct the names and run
calibrateSM.m
'''
from IPython import embed
from copy import copy
import time
import numpy as np
import hebiapi
from std_msgs.msg import Float64
#import setupfunctions
import tools
#import roslib; roslib.load_manifest('snake_monster_ros')
import rospy
from geometry_msgs.msg import Twist
# import SMCF
# from SMCF.SMComplementaryFilter import feedbackStructure,decomposeSO3
# import seatools.hexapod as hp
from Functions.Controller import Controller
from Functions.CPGgs import CPGgs

def publish_commands( hz ):
	pub={}
	ns_str = '/snake_monster'
	cont_str = 'eff_pos_controller'
	for i in xrange(6) :
		for j in xrange(3) :
			leg_str='L' + str(i+1) + '_' + str(j+1)
			pub[leg_str] = rospy.Publisher( ns_str + '/' + leg_str + '_'
											+ cont_str + '/command',
											Float64, queue_size=10 )
	rospy.init_node('walking_controller', anonymous=False)
	rate = rospy.Rate(hz)
	jntcmds = JointCmds()
	while not rospy.is_shutdown():
		jnt_cmd_dict = jntcmds.update(1./hz)
		for jnt in jnt_cmd_dict.keys() :
			pub[jnt].publish( jnt_cmd_dict[jnt] )
		rate.sleep()

rospy.init_node('walking_controller', anonymous=False)
pub={}
ns_str = '/snake_monster'
cont_str = 'eff_pos_controller'
for i in xrange(6) :
	for j in xrange(3) :
		leg_str='L' + str(i+1) + '_' + str(j+1)
		pub[leg_str] = rospy.Publisher( ns_str + '/' + leg_str + '_'
											+ cont_str + '/command',
											Float64, queue_size=10 )


print('Setting up Snake Monster...')

# names = SMCF.NAMES
names = ['SA012', 'SA059', 'SA030',
		'SA058', 'SA057', 'SA001',
		'SA078', 'SA040', 'SA048',
		'SA081', 'SA050', 'SA018',
		'SA046', 'SA032', 'SA026',
		'SA041', 'SA072', 'SA077']
#HebiLookup = tools.HebiLookup
# embed()
# shoulders = names[::3]
# imu = HebiLookup.getGroupFromNames(shoulders)

#setupfunctions.setupSnakeMonster()

#snakeMonster = HebiLookup.getGroupFromNames(names)

# while imu.getNumModules() != 6:
#     print('Found {} modules in shoulder group, {} in robot.'.format(imu.getNumModules(), snakeMonster.getNumModules()), end='  \r')
#     imu = HebiLookup.getGroupFromNames(shoulders)
# print('Found {} modules in shoulder group, {} in robot.'.format(imu.getNumModules(), snakeMonster.getNumModules()))
# snakeData = setup.setupSnakeMonsterShoulderData()
# smk = hp.HexapodKinematics()

# fbk = feedbackStructure(imu)
# gyroOffset, accelOffset = SMCF.calibrateOffsets(fbk)
# #setup.setupSnakeMonster()

# CF = SMCF.SMComplementaryFilter(accelOffset=accelOffset, gyroOffset=gyroOffset)
# fbk.getNextFeedback()
# CF.update(fbk)
# time.sleep(0.02)
# pose = []
# while pose is None or not list(pose):
#     fbk.getNextFeedback()
#     CF.update(fbk)
#     pose = copy(CF.R)


# print('Setup complete!')






### BILL TESTING (doesn't apss for now)
#shoulders1          = list(range(0,18,3)) # joint IDs of the shoulders
#shoulders2          = list(range(1,18,3)) # joint IDs of the second shoulder joints
#elbows              = list(range(2,18,3)) # joint IDs of the elbow joints
#cpg['legs'][0,shoulders1] = [-3.14/4, 3.14/4, 0, 0, 3.14/4, -3.14/4]; #% offset so that legs are more spread out
#cpg['legs'][0,shoulders2] = [3.14/2, 3.14/2, 3.14/2, 3.14/2, 3.14/2, 3.14/2];
#cpg['legs'][0,elbows] = [-3.14/2, -3.14/2, -3.14/2, -3.14/2, -3.14/2, -3.14/2];
#cmd.position = cpg['legs']; #%FJLFJLKJSDLKFJLSKJFLKSJFL
#snakeMonster.setAngles(cmd.position[0]);    
#print(smk.getLegPositions(cpg['legs']))
#J = cpg['smk'].getLegJacobians(cpg['legs'])
#print('J')
#print(J)
#while True:
	#asdf = 0

class joystickController(object):
	"""docstring for ClassName"""
	def __init__(self, cpg):
		self.cpg = copy(cpg)
		

	def callback(self, msg):
		
	
		# print(msg.linear.x)
		# print(msg.linear.z)

		


		# if(msg.linear.x > 0.1 and msg.angular.z > 0.1):
		# 	self.cntr = self.cntr+1
		# 	if(self.cntr%4 == 0):
		# 		self.cpg['direction']= self.cpg['forward']
		# 		print("forward")
		# 	else: 
		# 		self.cpg['direction']= self.cpg['leftturn']
		# 		print("left")	

		# if(msg.linear.x > 0.1 and msg.angular.z < 0.1):
		# 	self.cntr = self.cntr+1
		# 	if(self.cntr%4 == 0):
		# 		self.cpg['direction']= self.cpg['forward']
		# 		print("forward")
		# 	else: 
		# 		self.cpg['direction']= self.cpg['rightturn']

		# 		print("right")	

		# if(msg.linear.x < 0.1 and msg.angular.z > 0.1):
		# 	self.cntr = self.cntr+1
		# 	if(self.cntr%4 == 0):
		# 		self.cpg['direction']= self.cpg['backward']
		# 		print("backward")
		# 	else: 
		# 		self.cpg['direction']= self.cpg['leftturn']
		# 		print("left")

		# if(msg.linear.x < 0.1 and msg.angular.z < 0.1):
		# 	self.cntr = self.cntr+1
		# 	if(self.cntr%4 == 0):
		# 		self.cpg['direction']= self.cpg['backward']
		# 		print("backward")
		# 	else: 
		# 		self.cpg['direction']= self.cpg['rightturn']
		# 		print("right")		

		# if(msg.linear.x > 0.1  ):
		# 	self.cpg['direction']= self.cpg['forward']
		# 	print("forward")	

		if(msg.linear.x == -2):

				self.cpg['direction']= self.cpg['rightturn']
		 		print("backward")

		if(msg.angular.z < 0.2 and msg.angular.z > -0.2):

			if(msg.linear.x > 0.09):

				self.cpg['direction']= self.cpg['forward']
		 		print("forward")

		 	# if(msg.linear.x < 0):

				# self.cpg['direction']= self.cpg['backward']
		 	# 	print("backward")	

		if(msg.angular.z > 0.25 or msg.angular.z < -0.25):

			if(msg.angular.z > 0.25):

				self.cpg['direction']= self.cpg['leftturn']

				print("left")

			if(msg.angular.z < -0.25):

				self.cpg['direction']= self.cpg['rightturn']
				print("right")

		 											

		

		# elif(msg.linear.x < 0.1):
		# 	self.cpg['direction']= self.cpg['backward']
		# 	print("backward")

		# if(msg.linear.x > 0.1  ):
		# 	self.cpg['direction']= self.cpg['forward']
		# 	print("forward")
		# elif(msg.angular.z > 0.1):
		# 	self.cpg['direction']= self.cpg['leftturn']
		# 	print("left")
		# elif(msg.angular.z < 0.1):
		# 	self.cpg['direction']= self.cpg['rightturn']
		# 	print("right")	
		
		# elif(msg.linear.x < 0.1):
		# 	self.cpg['direction']= self.cpg['backward']
		# 	print("backward")
		
				
		

		#self.cpg['direction']= self.cpg['forward']

		
		self.cpg = CPGgs(self.cpg, self.t, self.dt)
		self.t += 1
		self.cpg['feetLog'].append(self.cpg['feetTemp'])
		#cpg['planeLog'].append(cpg['planeTemp'])

		# Command
		self.cmd.position = self.cpg['legs']

		pub['L'+'1'+'_'+'1'].publish(self.cmd.position[0][0])
		pub['L'+'1'+'_'+'2'].publish(self.cmd.position[0][1])
		pub['L'+'1'+'_'+'3'].publish(self.cmd.position[0][2])
		pub['L'+'6'+'_'+'1'].publish(self.cmd.position[0][3])
		pub['L'+'6'+'_'+'2'].publish(self.cmd.position[0][4])
		pub['L'+'6'+'_'+'3'].publish(self.cmd.position[0][5])
		pub['L'+'2'+'_'+'1'].publish(self.cmd.position[0][6])
		pub['L'+'2'+'_'+'2'].publish(self.cmd.position[0][7])
		pub['L'+'2'+'_'+'3'].publish(self.cmd.position[0][8])
		pub['L'+'5'+'_'+'1'].publish(self.cmd.position[0][9])
		pub['L'+'5'+'_'+'2'].publish(self.cmd.position[0][10])
		pub['L'+'5'+'_'+'3'].publish(self.cmd.position[0][11])
		pub['L'+'3'+'_'+'1'].publish(self.cmd.position[0][12])
		pub['L'+'3'+'_'+'2'].publish(self.cmd.position[0][13])
		pub['L'+'3'+'_'+'3'].publish(self.cmd.position[0][14])
		pub['L'+'4'+'_'+'1'].publish(self.cmd.position[0][15])
		pub['L'+'4'+'_'+'2'].publish(self.cmd.position[0][16])
		pub['L'+'4'+'_'+'3'].publish(self.cmd.position[0][17])


		#cpg['direction']= cpg['backward']

		###cmd.torque = gravComp(smk, cpg['legs'], cpg['gravVec'])

		#snakeMonster.set(cmd);
		#snakeMonster.setAngles(self.cmd.position[0])
		#snakeMonster.setTorques(cmd.torque)


	# def listener():
	# 	rospy.init_node('cmd_vel_listener')
	# 	rospy.Subscriber("/cmd_vel", Twist, callback)
	# 	rospy.spin()


# for t in range(nIter):
# 	tStart = time.perf_counter()

# 	if t == cpg['initLength']:
# 		print('Snake Monster is ready!')
# 		print('Begin walking')

# 	## Joystick stuffs - Exit Button
# 	if joy.pressed[2]:
# 		resetSnakeMonster()
# 		print('Reset.\n')
# 		print('Exiting at t = {}\n'.format(t))
# 		break

# 	if joy.pressed[1]:
# 		print('Exiting at t = {}\n'.format(t))
# 		break


	




	# listener()
	
	##Get pose/gravity vector
	# fbk.getNextFeedback()
	# CF.update(fbk)
	#cpg['pose']= copy(CF.R)
	
	# TEMPORARY print
	'''
	currentOrientation = decomposeSO3(cpg['pose'])
	if cnt == 0 and t > 0:
		print(currentOrientation, cpg['theta2'])
	cnt = (cnt + 1) % 50
'''

	#cpg['gravVec']= np.linalg.lstsq(cpg['pose'][:3, :3], [[0],[0],[-1]])[0]
	#cpg['poseLog'].append(cpg['pose'][:3, :3])

	# Get leg positions

	#realLegs = snakeMonster.getAngles()[:18]
	#cpg['realLegPositions']= smk.getLegPositions(realLegs)


	# Apply CPG

	'''
	if t >= cpg['initLength']and cpgJoy:
		if any([c != 0 for c in joy.channel[:2]]):
			cpg['move']= True
			if joy.channel(2) > 0:
				cpg['direction']= cpg['forward']
			elif joy.channel(2) < 0:
				cpg['direction']= cpg['backward']
			elif joy.channel(1) > 0:
				cpg['direction']= cpg['rightturn']
			elif joy.channel(1) < 0:
				cpg['direction']= cpg['leftturn']
		else:
			cpg['move']= false
	'''
	#cpg['requestedLegPositions']= smk.getLegPositions(cpg['legs'])
	

	
	
	
	#joy.running = False
	# pause('Program completed')

if __name__ == '__main__':
	

	## Initialize Variables

	T = 600
	dt = 0.02
	nIter = round(T/dt)
	cntr = 0

	cpg = {
    'initLength': 250,
    'w_y': 30.0,
    'bodyHeight':0.13,
    'bodyHeightReached':False,
    'zDist':0,
    'zHistory':np.ones((1,10)),
    'zHistoryCnt':0,
    'direction': np.ones((1,6)),
    'x':3 * np.array([[.11, -.1, .1, -.01, .12, -.12]]+[[0, 0, 0, 0, 0, 0] for i in range(30000)]),
    'y':np.zeros((30000+1,6)),
    'forward': np.ones((1,6)),
    'backward': -1 * np.ones((1,6)),
    'leftturn': [1, -1, 1, -1, 1, -1],
    'rightturn': [-1, 1, -1, 1, -1, 1],
    'legs': np.zeros((1,18)),
    'requestedLegPositions': np.zeros((3,6)),
    'correctedLegPositions': np.zeros((3,6)),
    'realLegPositions': np.zeros((3,6)),
    #'smk': smk,
    'isStance':np.zeros((1,6)),
    'pose': np.identity(3),
    'move':True,
    'groundNorm':np.zeros((1,3)),
    'groundD': 0,
    'gravVec':np.zeros((3,1)),
    'planePoint': [[0], [0], [-1.0]],
    'theta2':0,
    'theta3':0,
    'theta2Trap': 0,
    'groundTheta':np.zeros((1,30000)),
    'yOffset':np.zeros((1,6)),
    'eOffset':np.zeros((1,6)),
    'theta3Trap': 0,
    'planeTemp':np.zeros((3,3)),
    'feetTemp':np.zeros((3,6)),
    'yReq':0,
    'o':0,
    'poseLog':[],
    'feetLog':[],
    'planeLog':[]
    }

	

	cpg['zHistory'] = cpg['zHistory'] * cpg['bodyHeight']
	## Walk the Snake Monster

	print('Finding initial stance...')

	#joy = Controller()
	cpgJoy = True

	shoulders2 = list(range(2,18,3))
	elbows = list(range(3,18,3))

	sampleIter = 600
	cnt = 0
	

	controller = joystickController(cpg)
	controller.cmd = tools.CommandStruct()
	controller.dt = dt
	controller.nIter = nIter
	controller.cntr = cntr

	controller.t = 0
	#rospy.init_node('cmd_vel_listener', anonymous=False)
	rospy.Subscriber("/cmd_vel", Twist, controller.callback,queue_size=10)
	
	#rospy.Rate(1/dt)


	rospy.spin()
