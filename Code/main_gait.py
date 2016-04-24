#Author : varun verlencar
"""
Main implementation of Bipedal Gait trajectory
"""

if not __openravepy_build_doc__:
	from openravepy import *
	from numpy import *

def get_footsteps(steps):
	footsteps = steps

def waitrobot(robot):
	"""busy wait for robot completion"""
	while not robot.GetController().IsDone():
		time.sleep(0.01)

if __name__ == "__main__":

	env = Environment()
	env.SetViewer('qtcoin')
	env.Reset() 

	# load a scene
	env.Load('.env.xml')
	time.sleep(0.1)

	# get the robot
	robot = env.GetRobots()[0]

	init_pose = [0,0,0,0,0,0]
	left_footsteps = footsteps[0]
	right_footsteps = footsteps[1]
	planned_gait = []
	starting_foot = get_init_foot()????
	# get foot

	lfoot = robot.GetDOF()
	rfoot = robot.GetDOF()

	ikmodel = databases.inversekinematics.InverseKinematicsModel(robot,iktype=IkParameterization.Type.Translation3D)
	if not ikmodel.load():
	    ikmodel.autogenerate()

	with robot: # lock environment and save robot state
	    robot.SetDOFValues([2.58, 0.547, 1.5, -0.7],[0,1,2,3]) # set the first 4 dof values
	    Tee = manip.GetEndEffectorTransform() # get end effector
	    ikparam = IkParameterization(Tee[0:3,3],ikmodel.iktype) # build up the translation3d ik query
	    sols = manip.FindIKSolutions(ikparam, IkFilterOptions.CheckEnvCollisions) # get all solutions

	h = env.plot3(Tee[0:3,3],10) # plot one point
	with robot: # save robot state
	    raveLogInfo('%d solutions'%len(sols))
	    for sol in sols: # go through every solution
	        robot.SetDOFValues(sol,manip.GetArmIndices()) # set the current solution
	        env.UpdatePublishedBodies() # allow viewer to update new robot
	        time.sleep(1000.0/len(sols))

	for i in range(len(left_footsteps)-1)
		goal_footsteps = [left_footsteps[0][0],left_footsteps[0][1],left_footsteps[0][2],right_footsteps[0][0],right_footsteps[0][1],right_footsteps[0][2]]
		

		#call gaitplanner with above goals
		stride = gaitplanner(init_pose,goal_footsteps)
		planned_gait.append(stride)

""" Trjectory execution"""
