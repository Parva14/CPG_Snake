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
import geometry_msgs.msg
from nav_msgs.msg import Path
import tf

from Functions.Controller import Controller
from Functions.CPGgs import CPGgs
from tf.transformations import euler_from_quaternion, quaternion_from_euler

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


class joystickController(object):
	"""docstring for ClassName"""
	def __init__(self, cpg):
		self.cpg = copy(cpg)
		self.listener = tf.TransformListener()      
		
		
	#   self.plan = None
		# rospy.Subscriber("/cmd_vel", Twist, self.callback,queue_size=4)
		
	
	def planner(self, path):
		self.path = path
		# self.execute()
		
		# print("Hello")
		#print(trans[0])

		# print(path.pose.position.x)
		# embed()
		# diff_x = path.position.x - trans[0];
		# print(diff_x)




		


	# def callback(self, msg):
	#   if(msg.angular.z < 0.040 and msg.angular.z > -0.124):

	#       if(msg.linear.x > 0.09):

	#           self.cpg['direction']= self.cpg['forward']
	#           print("forward")

	#       if(msg.linear.x == -2):

	#           self.cpg['direction']= self.cpg['backward']
	#           print("backward")   

	#   if(msg.angular.z > 0.040 or msg.angular.z < -0.124):

	#       if(msg.angular.z > 0.040):

	#           self.cpg['direction']= self.cpg['leftturn']
	#           print("left")

	#       if(msg.angular.z < -0.124):

	#           self.cpg['direction']= self.cpg['rightturn']
	#           print("right")




		# self.cpg = CPGgs(self.cpg, self.t, self.dt)
		# self.t += 1
		# self.cpg['feetLog'].append(self.cpg['feetTemp'])
		# #cpg['planeLog'].append(cpg['planeTemp'])

		# # Command
		# self.cmd.position = self.cpg['legs']

		# pub['L'+'1'+'_'+'1'].publish(self.cmd.position[0][0])
		# pub['L'+'1'+'_'+'2'].publish(self.cmd.position[0][1])
		# pub['L'+'1'+'_'+'3'].publish(self.cmd.position[0][2])
		# pub['L'+'6'+'_'+'1'].publish(self.cmd.position[0][3])
		# pub['L'+'6'+'_'+'2'].publish(self.cmd.position[0][4])
		# pub['L'+'6'+'_'+'3'].publish(self.cmd.position[0][5])
		# pub['L'+'2'+'_'+'1'].publish(self.cmd.position[0][6])
		# pub['L'+'2'+'_'+'2'].publish(self.cmd.position[0][7])
		# pub['L'+'2'+'_'+'3'].publish(self.cmd.position[0][8])
		# pub['L'+'5'+'_'+'1'].publish(self.cmd.position[0][9])
		# pub['L'+'5'+'_'+'2'].publish(self.cmd.position[0][10])
		# pub['L'+'5'+'_'+'3'].publish(self.cmd.position[0][11])
		# pub['L'+'3'+'_'+'1'].publish(self.cmd.position[0][12])
		# pub['L'+'3'+'_'+'2'].publish(self.cmd.position[0][13])
		# pub['L'+'3'+'_'+'3'].publish(self.cmd.position[0][14])
		# pub['L'+'4'+'_'+'1'].publish(self.cmd.position[0][15])
		# pub['L'+'4'+'_'+'2'].publish(self.cmd.position[0][16])
		# pub['L'+'4'+'_'+'3'].publish(self.cmd.position[0][17])

	def execute(self):
		(trans,rot) = self.listener.lookupTransform('map', 'base_link', rospy.Time(0))
		(roll, pitch, yaw_c) = euler_from_quaternion (rot)
		path = self.path
		reached = False

		i=0
		
		while(reached == False):

			if((np.abs((path.poses[(len(path.poses)-1)].pose.position.x) - trans[0]) < 0.7) and (np.abs((path.poses[(len(path.poses)-1)].pose.position.y) - trans[1]) < 0.7)):

				reached = True

			else:

				reached = False 

			if(reached == True):

				self.cpg['move']= False 
				self.path = None 
				path = None

			else:
			
				self.cpg['move']= True  

			(trans,rot) = self.listener.lookupTransform('map', 'base_link', rospy.Time(0))
			print(i)
			print(len(path.poses))
			print(np.abs((path.poses[(len(path.poses)-1)].pose.position.x) - trans[0])) 
			print(reached)  

			(trans,rot) = self.listener.lookupTransform('map', 'base_link', rospy.Time(0))

			if(i > (len(path.poses)-5)):
					path.poses[i].pose.position.x = path.poses[(len(path.poses)-1)].pose.position.x
					path.poses[i].pose.position.y = path.poses[(len(path.poses)-1)].pose.position.y

			while((np.abs((path.poses[i+1].pose.position.x) - trans[0]) > 0.015) and (np.abs((path.poses[i+1].pose.position.y) - trans[1]) > 0.015)):

				if(i > (len(path.poses)-5)):
					path.poses[i].pose.position.x = path.poses[(len(path.poses)-1)].pose.position.x
					path.poses[i].pose.position.y = path.poses[(len(path.poses)-1)].pose.position.y


				(trans,rot) = self.listener.lookupTransform('map', 'base_link', rospy.Time(0))
				(roll, pitch, yaw_l) = euler_from_quaternion (rot)
				yaw_c = -1*yaw_l
				point = path.poses[i]
				yaw_d = np.arctan2((path.poses[i+4].pose.position.x - trans[0]),(path.poses[i+4].pose.position.y - trans[1])) 
				yaw_dd = (yaw_d* (180/np.pi))
				diff = yaw_d - yaw_c


				while((np.abs(diff)) > 0.1):

					if(diff > 0.1):

						self.cpg['direction']= self.cpg['rightturn']

					else:

						self.cpg['direction']= self.cpg['leftturn'] 

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

					(trans,rot) = self.listener.lookupTransform('map', 'base_link', rospy.Time(0))
					(roll, pitch, yaw_l) = euler_from_quaternion (rot)
					yaw_c = -1*yaw_l
					diff = (yaw_d - yaw_c)

				(trans,rot) = self.listener.lookupTransform('map', 'base_link', rospy.Time(0))
				(roll, pitch, yaw_l) = euler_from_quaternion (rot)
				yaw_c = -1*yaw_l
				diff = (yaw_d - yaw_c)  
			
				self.cpg['direction']= self.cpg['forward']  

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

				(trans,rot) = self.listener.lookupTransform('map', 'base_link', rospy.Time(0))
				(roll, pitch, yaw_l) = euler_from_quaternion (rot)

			i = i+1     

			if((np.abs((path.poses[(len(path.poses)-1)].pose.position.x) - trans[0]) < 0.7) and (np.abs((path.poses[(len(path.poses)-1)].pose.position.y) - trans[1]) < 0.7)):

				reached = True

			else:

				reached = False     

			if(reached == True):

				self.cpg['move']= False 

			else:
			
				self.cpg['move']= True      

			# self.r.sleep()    



			


					

if __name__ == '__main__':
	

	## Initialize Variables

	T = 600
	dt = 0.02
	nIter = round(T/dt)
	cntr = 0
	# r.sleep()
	cpg = {
	'initLength': 250,
	'w_y': 2.0,
	'bodyHeight':0.13,
	'bodyHeightReached':False,
	'zDist':0,
	'zHistory':np.ones((1,10)),
	'zHistoryCnt':0,
	'direction': np.ones((1,6)),
	'x':3 * np.array([[.11, -.1, .1, -.01, .12, -.12]]+[[0, 0, 0, 0, 0, 0] for i in range(300000)]),
	'y':np.zeros((300000+1,6)),
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
	'groundTheta':np.zeros((1,300000)),
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

	# rospy.init_node('cmd_vel_listener', anonymous=False)
	

	

	# #rospy.Rate(1/dt)

	# while True:
	#   rate = rospy.rate(100)
	while not rospy.is_shutdown():
		r = rospy.Rate(20)
		r.sleep()
		# controller.listener = tf.TransformListener()      
		rospy.Subscriber("/move_base/NavfnROS/plan", Path, controller.planner,queue_size=1)
		raw_input("Press Enter to continue...")
		controller.execute()
		quit()
