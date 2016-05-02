#!/usr/bin/env python
# -*- coding: utf-8 -*-
#HW1 for RBE 595/CS 525 Motion Planning
#code based on the simplemanipulation.py example
#Author : Varun Visnudas Verlencar
import time
import openravepy

if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

def waitrobot(robot):
    """busy wait for robot completion"""
    while not robot.GetController().IsDone():
        time.sleep(0.01)

# def tuckarms(env,robot):
#     with env:
#         jointnames = ['leftAnkle','leftKnee','leftHip','rightHip','rightKnee','rightAnkle']
#         robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])
#         robot.SetActiveDOFValues([0,0,0,0,0,0]);
#         robot.GetController().SetDesired(robot.GetDOFValues());
#     waitrobot(robot)

if __name__ == "__main__":

    env = Environment()
    env.SetViewer('qtcoin')
    env.Reset()
    # load a scene from ProjectRoom environment XML file
    env.Load('plannerplugin/scenes/basicMap_V2.env.xml')
    time.sleep(0.1)

    # 1) get the 1st robot that is inside the loaded scene
    # 2) assign it to the variable named 'robot'
    PR2 = env.GetRobots()[0]
    # env.Remove(PR2)
    robot2 = env.ReadRobotXMLFile('plannerplugin/scenes/LegChair_RightBased_V3.robot.xml')
    env.Add(robot2)
    # robot3 = 
# 
    PR22 =robot2
    # tuck in the PR2's arms for driving
    # tuckarms(env,PR2);


    #### YOUR CODE HERE ####
    
    # pumarobot = env.ReadRobotXMLFile('robots/puma.robot.xml')
    # env.Add(pumarobot)
    # time.sleep(0.1)

    # PUMA = env.GetRobots()[1]
    # #Get next to PR2
    # # PUMA.SetTransform(numpy.dot(matrixFromPose([1,0,0,0,0,1,0]),PR2.GetTransform())) # set new pose next to PR2

    with env:
        jointnames = ['leftAnkle','leftKnee','leftHip','rightHip','rightKnee','rightAnkle']
        # [.5,-0.45,-0.1,-.1,-.45,0.65]
        # [.25,-0.15,-0.1,-.1,-.15,0.25]
        # manip = PR2.GetManipulator('foot')
        # print manip
        # print manip.GetTransform()
        indices = [PR2.GetJoint(name).GetDOFIndex() for name in jointnames]
        # print indices
        PR2.SetActiveDOFs([PR2.GetJoint(name).GetDOFIndex() for name in jointnames])
        PR2.SetDOFValues([.3,-0.35,-0.17,-.17,.35,0.05],indices) # set the first 6 dof values
        # PR2.GetLinks()[0].SetTransform()
        # print PR2.GetLinks()[0].GetTransform()
        TR = PR2.GetManipulator('foot').GetTransform()[0:4,3]
        print TR
        T = PR22.GetTransform()[0:4,0:4]
        T[0,3] = TR[0]
        T[1,3] = TR[1]
        T[2,3] = TR[2]
        T[3,3] = TR[3]
        # env.Remove(PR2)
        # arr =numpy.dot(matrixFromPose([1,0,0,0,T[0,4],T[1,4],T[1]])
        jointnames = ['leftAnkle','leftKnee','leftHip','rightHip','rightKnee','rightAnkle']
        indices = [PR22.GetJoint(name).GetDOFIndex() for name in jointnames]
        # print indices
        PR22.SetActiveDOFs([PR22.GetJoint(name).GetDOFIndex() for name in jointnames])
        # PR22.SetTranslation3D(T)
        PR22.SetTransform(T)
        PR22.SetDOFValues([-0.05,-0.35,0.17,.17,.35,-0.3],indices)

        TR = PR22.GetManipulator('foot').GetTransform()[0:4,3]
        # print TR
        T = PR2.GetTransform()[0:4,0:4]
        T[0,3] = TR[0]
        T[1,3] = TR[1]
        T[2,3] = TR[2]
        T[3,3] = TR[3]

        PR2.SetActiveDOFs([PR2.GetJoint(name).GetDOFIndex() for name in jointnames])
        # PR22.SetTranslation3D(T)
        PR2.SetTransform(T)
        PR2.SetDOFValues([.3,-0.35,-0.17,-.17,.35,0.05],indices)

        # print PR2.GetChain(manip.GetBase().GetIndex(),manip.GetEndEffector().GetIndex(),returnjoints=False)[1:]
        # PR2.SetActiveDOFValues([0,0,-1,-1,-.2,0])
        # manip = PR2.GetManipulator('foot')
        # print manip
        # print '\nRight foot at:',PR2.GetTransform()[1:4,0]
        incollision = PR2.CheckSelfCollision()  
        if incollision:
            print '\n1collision!!\n'
        incollision = PR22.CheckSelfCollision()  
        if incollision:
            print '\n2collision!!\n'
    # with env:
    #     jointnames = ['l_shoulder_pan_joint','l_shoulder_lift_joint','l_elbow_flex_joint','l_wrist_flex_joint']
    #     PR2.SetActiveDOFs([PR2.GetJoint(name).GetDOFIndex() for name in jointnames])
    #     # robot.SetActiveDOFValues([1.4,-2.32099996,-0.69800004]);

    #     with PR2:
    #         PR2.SetActiveDOFValues([1.2,1.5,-1.32099996,0])
    #         PR2.GetController().SetDesired(PR2.GetDOFValues());
    #         incollision = env.CheckCollision(PR2)  
    #         if incollision:
    #             print '\ncollision!!\n'
    
    env.UpdatePublishedBodies() 
    # basemanip.MoveActiveJoints(goal=[1.4,-2.32099996,-0.69800004],maxiter=5000,steplength=0.15,maxtries=2)
    waitrobot(PR2)
    # env.UpdatePublishedBodies() # allow viewer to update new robot


    #### END OF YOUR CODE ###


    raw_input("Press enter to exit...")

