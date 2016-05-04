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
	robot = robot2
	# robot = env.ReadRobotXMLFile('robots/neuronics-katana.zae')
	# env.Add(robot)

	RaveInitialize()
	RaveLoadPlugin('plannerplugin/build/plannerplugin')

	init_pose = [0,0,0,0,0,0]
	jointnames = ['leftAnkle','leftKnee','leftHip','rightHip','rightKnee','rightAnkle']
	indices = [robot.GetJoint(name).GetDOFIndex() for name in jointnames]
	# robot.SetActiveDOFs(indices)  #set all dofs active      
	# robot.SetActiveDOFValues(init_pose);
	# robot.GetController().SetDesired(robot.GetDOFValues());
	# waitrobot(robot)

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
	flag = True
	zp = 2
	T = robot1.GetTransform() 
	TR = T[0:4,3]
	# TR = robot.GetManipulator('foot').GetTransform()[0:4,3]

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

	# 		Tgoal= [0.69,1.53,0.076] ######testing parameter, comment when done

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
	left_foot_orient = 0 # for dynamic walking
	right_foot_orient = 0
	foot_orient = 0 #for static walking
	goal_footstep = None
	next_foot = 0
	stepsize = [0.25]
	goalbias = [0.26]

	##call gaitplanner with above goals
	## stride = gaitplanner(init_pose,goal_footsteps)
	## planned_gait.append(stride)
	
	goalconfig = [[.3,-0.35,-0.17,-.17,.35,0.05],[-0.05,-0.35,0.17,.17,.35,-0.30]]
	startconfig = [[0,0,0,0,0,0],[.3,-0.35,-0.17,-.17,.35,0.05]]
	q = 1
	namer = 'left'

	# goalconfig = [[-.05,-0.35,0.17,.17,.35,-0.3]]
	# startconfig = [[.3,-0.35,-0.17,-.17,.35,0.05]]

	# goalconfig = [[.3,-0.35,-0.17,-.17,.35,0.05],[-0.05,-0.35,0.17,.17,.35,-0.3],[.3,-0.35,-0.17,-.17,.35,0.05],[-0.05,-0.35,0.17,.17,.35,-0.3]]
	# startconfig = [[0,0,0,0,0,0],[.3,-0.35,-0.17,-.17,.35,0.05],[-0.05,-0.35,0.17,.17,.35,-0.3],[.3,-0.35,-0.17,-.17,.35,0.05]]
	plannermodule = RaveCreateModule(env,'plannermodule')
	
	for m in range(0,len(goalconfig)-1): #remove -1 for continuos steps
		
		with env:
			  #set all dofs active      
			# startconfig[m] = 	
			if next_foot == 0:
				robot = robot1	
				namer = 'left'
				flag = False				
				next_foot = 5			
				if zp !=1:
					env.Add(robot)
					env.Remove(robot2)	
					zp =1
				
				T = robot.GetTransform()[0:4,0:4]
				T[0,3] = TR[0]
				T[1,3] = TR[1]
				T[2,3] = TR[2]
				T[3,3] = TR[3]								
				robot.SetTransform(T)									
				
				
			else:
				robot = robot2	
				namer = 'right'	
				env.Remove(robot1)
				env.Add(robot)
				flag = True
				next_foot = 0
				T = robot.GetTransform()[0:4,0:4]
				T[0,3] = TR[0]
				T[1,3] = TR[1]
				T[2,3] = TR[2]
				T[3,3] = TR[3]
				robot.SetTransform(T)
			
			robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])
			robot.SetActiveDOFValues(startconfig[m]);
			
			if (q):
				q =0
				initConfig =  startconfig[m] + goalconfig[m] + goalbias + stepsize
				a = time.time()
				print "Planning Started"
				path = plannermodule.SendCommand('gaitplanner %f %f %f %f %f %f %f %f %f %f %f %f %f %f end'%tuple(initConfig))
				a = time.time()-a
				print '\n time:', a
				

				_unsmoothPath = []; nodes =[]; lowerlimit=[];upperlimit=[]

				if path is None:
					raveLogWarn('command failed!')
				else:
					nodes = path.split(';')
					print 'lenghth',len(nodes)

					# for i in lines[:-1]:
					for i in range(0,len(nodes)-1):
						d = nodes[i].split()
						_unsmoothPath.append([float(x) for x in d])
						# for x in d:
						#     print x,","
						# print "\n"

				print 'Smooth length',len(_unsmoothPath)
				
				lowerlimit,upperlimit = robot.GetDOFLimits(indices)

				handles1=[]
				for i in (_unsmoothPath):
					# for k in range(0,len(i)-1):
					# 	if (i[k] != goalconfig[m][k]):
					# 		if (i[k] < lowerlimit[k]):
					# 			i[k]  = lowerlimit[k]
					# 		elif(i[k] > upperlimit[k]):
					# 			i[k] = upperlimit[k]
					arr=array([i[0],i[1],i[2],i[3],i[4],i[5]])
					robot.SetDOFValues(arr,indices)
					pt=robot.GetManipulator('foot').GetTransform()[0:3,3]
					handles1.append(env.plot3(pt,pointsize=0.03,colors=array(((0,0,1))),drawstyle=1))
				# env.UpdatePublishedBodies() 

				traj = RaveCreateTrajectory(env,'')
				traj.Init(robot.GetActiveConfigurationSpecification())

				with open('Leftfoot.txt', 'a') as f1:
						f1.write("\n")
						f1.write(str(namer))
						f1.close()

				for j in range(0,len(_unsmoothPath)):
					with open('Leftfoot.txt', 'a') as f1:
						f1.write(str(_unsmoothPath[j]))
						f1.write("\n")
						f1.close()
					traj.Insert(j,_unsmoothPath[j])

				planningutils.RetimeActiveDOFTrajectory(traj,robot,hastimestamps=False,maxvelmult=1)
				print 'duration',traj.GetDuration()
				TR = robot.GetManipulator('foot').GetTransform()[0:4,3]
				# print TR
		robot.GetController().SetPath(traj)
		robot.WaitForController(0)
		

		### END OF YOUR CODE ###
	waitrobot(robot)
	raw_input("Press enter to exit...")
""" Trjectory execution"""
