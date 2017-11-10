#!/usr/bin/env python  

from IPython import embed
from copy import copy
import time
import numpy as np
import hebiapi
from std_msgs.msg import Float64
import tools
import rospy
from geometry_msgs.msg import Twist
import geometry_msgs.msg
from nav_msgs.msg import Path
import tf

from Functions.Controller import Controller
from Functions.CPGgs import CPGgs
from tf.transformations import euler_from_quaternion, quaternion_from_euler

ind = 0
path_x = list()
path_y = list()


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
		      
	def motion(self):
		
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

if __name__ == '__main__':

	T = 600
	dt = 0.02
	nIter = round(T/dt)
	cntr = 0
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
	print('Finding initial stance...')
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

	def planner(path):
		# print("Callback called")
		ind = 0
		del path_x[:]
		del path_y[:]
		for i in range((len(path.poses))):
			path_x.append(path.poses[i].pose.position.x)  
			path_y.append(path.poses[i].pose.position.y)  
		for j in range(5):
			path_x.append(path.poses[-1].pose.position.x)
			path_y.append(path.poses[-1].pose.position.y)


	while not rospy.is_shutdown():
		rospy.Subscriber("/move_base/NavfnROS/plan", Path, planner,queue_size=20)
		r = rospy.Rate(15)
		r.sleep()
		if( path_x != [] and path_y != []):

			(trans,rot) = controller.listener.lookupTransform('map', 'base_link', rospy.Time(0))
			(roll, pitch, yaw_c) = euler_from_quaternion (rot)

			try:
				destn_pose = (np.abs((path_x[len(path_x)-1]) - trans[0]) < 0.1) and (np.abs((path_y[len(path_y)-1]) - trans[1]) < 0.1)
			except:	
				destn_pose = False

			if(destn_pose):

				reached = True


			else:

				reached = False

			if(reached == True):
				del path_x[:]
				del path_y[:]
				ind = 0
				print("reached")
				print(path_x)
				continue
			(trans,rot) = controller.listener.lookupTransform('map', 'base_link', rospy.Time(0))
			(roll, pitch, yaw_c) = euler_from_quaternion (rot)
			reached = False
			
			if(reached == False):

				(trans,rot) = controller.listener.lookupTransform('map', 'base_link', rospy.Time(0))
				if( path_x != [] and path_y != []):
					print(ind)	

					next_pos = True

					while(next_pos and not rospy.is_shutdown()):

						(trans,rot) = controller.listener.lookupTransform('map', 'base_link', rospy.Time(0))
						(roll, pitch, yaw_l) = euler_from_quaternion (rot)
						yaw_c = -1*yaw_l

						if((np.abs((path_x[len(path_x)-1]) - trans[0]) < 0.1) and (np.abs((path_y[len(path_y)-1]) - trans[1]) < 0.1)):

							reached = True

						else:

							reached = False

						if(reached == True):
							del path_x[:]
							del path_y[:]
							ind = 0
							print("reached11")
							print(path_x)
							break	
						if( path_x != [] and path_y != []):	
							try:
								yaw_d = np.arctan2((path_x[ind+3] - trans[0]),(path_y[ind+3] - trans[1]))
							except:
								yaw_d = yaw_c	 
							yaw_dd = (yaw_d* (180/np.pi))
							diff = yaw_d - yaw_c
						
							if( path_x != [] and path_y != []):	
								while((np.abs(diff)) > 0.1 and not rospy.is_shutdown()):

									if(diff > 0):

										controller.cpg['direction']= controller.cpg['rightturn']
										controller.motion()

									else:

										controller.cpg['direction']= controller.cpg['leftturn'] 
										controller.motion()

									(trans,rot) = controller.listener.lookupTransform('map', 'base_link', rospy.Time(0))
									(roll, pitch, yaw_l) = euler_from_quaternion (rot)
									yaw_c = -1*yaw_l
									diff = (yaw_d - yaw_c) 

									if((np.abs((path_x[len(path_x)-1]) - trans[0]) < 0.1) and (np.abs((path_y[len(path_y)-1]) - trans[1]) < 0.1)):

										reached = True

									else:

										reached = False

									if(reached == True):
										del path_x[:]
										del path_y[:]
										ind = 0
										print("reached22")
										print(path_x)
										break	

							(trans,rot) = controller.listener.lookupTransform('map', 'base_link', rospy.Time(0))
							(roll, pitch, yaw_l) = euler_from_quaternion (rot)
							yaw_c = -1*yaw_l
							diff = (yaw_d - yaw_c)  
						
							controller.cpg['direction']= controller.cpg['forward']
							controller.motion()  
							(trans,rot) = controller.listener.lookupTransform('map', 'base_link', rospy.Time(0))
							(roll, pitch, yaw_l) = euler_from_quaternion (rot)

						try:	
							next_pos = (np.abs(path_x[ind] - trans[0]) > 0.015) and (np.abs(path_y[ind] - trans[1]) > 0.015)	
						except:
							next_pos = True	

				ind = ind +1         