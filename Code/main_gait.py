#Author : varun verlencar
"""
Main implementation of Bipedal Gait trajectory
"""
import time
import openravepy

if not __openravepy_build_doc__:
	from openravepy import *
	from numpy import *

def get_footsteps():
	footsteps = [[(0.9,2.45,0,0,0,0)],[(0.69,1.53,0.076)]]
	return footsteps

def waitrobot(robot):
	"""busy wait for robot completion"""
	while not robot.GetController().IsDone():
		time.sleep(0.01)
def get_init_foot():
	return 0

if __name__ == "__main__":

	env = Environment()
	env.SetViewer('qtcoin')
	env.Reset() 

	# load a scene
	env.Load('plannerplugin/scenes/basicMap_V2.env.xml')
	time.sleep(0.1)
	robot2 = env.ReadRobotXMLFile('plannerplugin/robots/LegChair_RightBased_V3.robot.xml')
	# robot2 = env.ReadRobotXMLFile('robots/puma.robot.xml')


	# get the robot
	robot1 = env.GetRobots()[0]
	robot = robot1
	# robot = env.ReadRobotXMLFile('robots/neuronics-katana.zae')
	# env.Add(robot)

	robot.SetTransform(robot1.GetTransform())

	RaveInitialize()
	RaveLoadPlugin('plannerplugin/build/plannerplugin')

	# jointnames =[0,0,0,0,0,0]
	jointnames = ['leftAnkle','leftKnee','leftHip','rightHip','rightKnee','rightAnkle']
	robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])  #set all dofs active      

	footsteps = get_footsteps()
	init_pose = [0,0,0,0,0,0]
	left_footsteps = footsteps[0]
	right_footsteps = footsteps[1]
	planned_gait = []
	starting_foot = get_init_foot()#???? int 0=left, 5 = right
	#get foot

	# lfoot = robot.GetLinks()[0]
	# rfoot = robot.GetLinks()[5]
	Tgoal = None
	active_foot = None
	next_foot = 5

	# robot.SetActiveManipulator()
	T = robot.GetManipulator('foot').GetTransform() 

	for i in range(len(left_footsteps)):
	########### Find IK solution #############
		print 'Finding inversekinematics'
		h = env.plot3([0.4,1.53,0.076],20) # plot one point
		ikmodel = databases.inversekinematics.InverseKinematicsModel(robot,iktype=IkParameterization.Type.Translation3D)
		if not ikmodel.load():
			ikmodel.autogenerate()

		with robot: # lock environment and save robot state
			robot.SetDOFValues(init_pose,[0,1,2,3,4,5]) # set the initial dof values
			if starting_foot == 0:
				active_foot = robot.GetManipulator('foot')
				Tgoal = left_footsteps[i] # get leftfoot pose
			elif starting_foot == 5:
				active_foot = robot.GetManipulator('foot')
				Tgoal = right_footsteps[i] # get rightfoot pose
			else:
				raveLogInfo('starting foot not stated\n')
				break

			Tgoal= [0.4,1.53,0.076] ######testing parameter, comment when done

			ikparam = IkParameterization(Tgoal,ikmodel.iktype)
			sols = active_foot.FindIKSolutions(ikparam, IkFilterOptions.CheckEnvCollisions) # get all solutions
			assert(sols is not None)

		
		with robot: # save robot state
			raveLogInfo('%d solutions'%len(sols))
			for sol in sols: # go through every solution
				print 'inversekinematics'
				robot.SetDOFValues(sol,[0,1,2,3,4,5]) # set the current solution
				env.UpdatePublishedBodies() # allow viewer to update new robot
				time.sleep(10.0/len(sols))

	########### Plan Gait #############
	# left_foot_orient = 0 # for dynamic walking
	# right_foot_orient = 0
	# foot_orient = 0 #for static walking
	# goal_footstep = None

	# 	# goal_footsteps = [left_footsteps[i][0],left_footsteps[i][1],left_footsteps[i][2],left_foot_orient,right_footsteps[i][0],right_footsteps[i][1],right_footsteps[i][2],right_foot_orient]
	# 	if starting_foot == 1:
	# 		goal_footstep = [left_footsteps[i],foot_orient]
	# 		next_foot = 2
	# 	elif starting_foot == 2:
	# 		goal_footstep = [right_footsteps[i],foot_orient]
	# 		next_foot = 1
	# 	starting_foot = next_foot

	# 	#call gaitplanner with above goals
	# 	stride = gaitplanner(init_pose,goal_footsteps)
	# 	planned_gait.append(stride)
	waitrobot(robot)
	raw_input("Press enter to exit...")
""" Trjectory execution"""
