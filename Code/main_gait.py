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

	Tr =robot.GetTransform()[1:4,0]
	# Tr[0,1] = 0;Tr[0,2] = 0;Tr[0,3] = 0
	ar = Tr
	anew = ar

	RaveInitialize()
	RaveLoadPlugin('plannerplugin/build/plannerplugin')

	# jointnames =[0,0,0,0,0,0]
	init_pose = [0,0,0,0,0,0]
	jointnames = ['leftAnkle','leftKnee','leftHip','rightHip','rightKnee','rightAnkle']
	robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])  #set all dofs active      
	robot.SetActiveDOFValues(init_pose);
	robot.GetController().SetDesired(robot.GetDOFValues());
	waitrobot(robot)

	footsteps = get_footsteps()
	
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
	flag = True
	zp = 1
	# robot.SetActiveManipulator()
	T = robot.GetManipulator('foot').GetTransform() 

	# for i in range(len(left_footsteps)):
	# ########### Find IK solution #############
	# 	print 'Finding inversekinematics'
	# 	h = env.plot3([0.4,1.53,0.076],20) # plot one point
	# 	ikmodel = databases.inversekinematics.InverseKinematicsModel(robot,iktype=IkParameterization.Type.Translation3D)
	# 	if not ikmodel.load():
	# 		ikmodel.autogenerate()

	# 	with robot: # lock environment and save robot state
	# 		robot.SetDOFValues(init_pose,[0,1,2,3,4,5]) # set the initial dof values
	# 		if starting_foot == 0:
	# 			active_foot = robot.GetManipulator('foot')
	# 			Tgoal = left_footsteps[i] # get leftfoot pose
	# 		elif starting_foot == 5:
	# 			active_foot = robot.GetManipulator('foot')
	# 			Tgoal = right_footsteps[i] # get rightfoot pose
	# 		else:
	# 			raveLogInfo('starting foot not stated\n')
	# 			break

	# 		Tgoal= [0.4,1.53,0.076] ######testing parameter, comment when done

	# 		ikparam = IkParameterization(Tgoal,ikmodel.iktype)
	# 		sols = active_foot.FindIKSolutions(ikparam, IkFilterOptions.CheckEnvCollisions) # get all solutions
	# 		assert(sols is not None)

		
	# 	with robot: # save robot state
	# 		raveLogInfo('%d solutions'%len(sols))
	# 		for sol in sols: # go through every solution
	# 			print 'inversekinematics'
	# 			robot.SetDOFValues(sol,[0,1,2,3,4,5]) # set the current solution
	# 			env.UpdatePublishedBodies() # allow viewer to update new robot
	# 			time.sleep(10.0/len(sols))

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
	goalconfig = [[.5,-0.45,-0.1,-.1,-.45,0.65],[.5,-0.45,-0.1,-.1,-.45,0.65],[.5,-0.45,-0.1,-.1,-.45,0.65],[.5,-0.45,-0.1,-.1,-.45,0.65]]
	startconfig = [[0,0,0,0,0,0],[.5,-0.45,-0.1,-.1,-.45,0.65],[.5,-0.45,-0.1,-.1,-.45,0.65],[.5,-0.45,-0.1,-.1,-.45,0.65]]
	plannermodule = RaveCreateModule(env,'plannermodule')
	# i =0
	for m in range(0,len(goalconfig)):
		with env:
			
			if flag:
				robot = robot1
				if next_foot ==0:
					env.Remove(robot2)
					env.Add(robot)
				flag = False				
				next_foot = 5
				if zp==1:
					ar = Tr
					y = .92
					z = 0.076
				else:
					ar= Tr[0:3,3]
					robot.SetTransform(numpy.dot(matrixFromPose([1,0,0,0,ar[0]+anew[1],ar[1]+y,ar[2]-z]),robot.GetTransform()))
			else:
				robot = robot2
				flag = True
				if next_foot ==5:
					env.Remove(robot1)
					env.Add(robot)
				next_foot = 0
				ar= Tr[0:3,3]
				robot.SetTransform(numpy.dot(matrixFromPose([1,0,0,0,ar[0]+anew[1],ar[1]-.92,ar[2]+0.076]),robot.GetTransform()))

			
			

			robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])  #set all dofs active      
			robot.SetActiveDOFValues(startconfig[m]);
			env.UpdatePublishedBodies() 

			# [.25,-0.15,-0.1,-.1,-.15,0.25]
			initConfig = startconfig[m] + goalconfig[m]
			a = time.time()
			path = plannermodule.SendCommand('gaitplanner %f %f %f %f %f %f %f %f %f %f %f %f flag'%tuple(initConfig))
			a = time.time()-a
			print '\n time:', a
			

			_unsmoothPath = []; nodes =[]; lowerlimit=[];upperlimit=[]
			# n = cmdout.size();
			# n =n/7
			# print " size",n
			# print cmdout
			if path is None:
				raveLogWarn('command failed!')
			else:
				nodes = path.split(';')
				print 'lenghth',len(nodes)

				# for i in lines[:-1]:
				for i in range(0,len(nodes)-1):
					d = nodes[i].split()
					# print 'node:',i,' ', d,"\n";
					_unsmoothPath.append([float(x) for x in d])
					# for x in d[:-1]:
					#     print x,","
					# print "\n"

			# print _unsmoothPath
			indices = robot.GetActiveDOFIndices()       
			lowerlimit,upperlimit = robot.GetDOFLimits(indices)
			# lowerlimit[4]= -3.14
			# upperlimit[4]= 3.14
			# lowerlimit[6]= -3.14
			# upperlimit[6]= 3.14

			handles1=[]
			for i in (_unsmoothPath):

					for k in range(0,len(i)-1):
						if (i[k] != goalconfig[m][k]):
							if (i[k] < lowerlimit[k]):
								i[k]  = lowerlimit[k]
							elif(i[k] > upperlimit[k]):
								i[k] = upperlimit[k]
					arr=array([i[0],i[1],i[2],i[3],i[4],i[5]])
					robot.SetActiveDOFValues(arr)
					pt=robot.GetLinks()[next_foot].GetTransform()[0:3,3]
					handles1.append(env.plot3(pt,pointsize=0.03,colors=array(((0,0,1))),drawstyle=1))
			env.UpdatePublishedBodies() 


			anew= robot.GetTransform()[0:3,3]
			traj = RaveCreateTrajectory(env,'')
			traj.Init(robot.GetActiveConfigurationSpecification())

			for j in range(len(_unsmoothPath)):
				traj.Insert(j,_unsmoothPath[j])

			planningutils.RetimeActiveDOFTrajectory(traj,robot,hastimestamps=False,maxvelmult=1)
			print 'duration',traj.GetDuration()
		Tr = robot.GetTransform()
		robot.GetController().SetPath(traj)
		robot.WaitForController(0)

		### END OF YOUR CODE ###
		waitrobot(robot)
	raw_input("Press enter to exit...")
""" Trjectory execution"""
