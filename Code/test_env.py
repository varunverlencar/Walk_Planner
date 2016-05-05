#!/usr/bin/env python
# -*- coding: utf-8 -*-
#HW1 for RBE 595/CS 525 Motion Planning
#code based on the simplemanipulation.py example
#Author : Varun Visnudas Verlencar
import time
import openravepy

from copy import deepcopy
from math import sin,cos,acos,asin,pow,sqrt,atan2
from math import degrees as d
import random

if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

def waitrobot(robot):
    """busy wait for robot completion"""
    while not robot.GetController().IsDone():
        time.sleep(0.01)

def randomGoalIk(x,z,baseFoot):
    
    while (True):
        
        #generate random values for support leg
         t0 = (random.random()*(pi/2.0))-(pi/4.0)
         t1 = (random.random()*(pi/2.0))-(pi/2.0)
         t2 = (random.random()*(1.0*pi))-(pi/2.0)
         
         t01 = t0+t1
         t012 = t01+t2
         
         if ((t012 > 0.349066) or (t012 < -0.349066)):
             print('Chair angle fail')
             print('')
             continue
         l = 62.0
         
         #foward kinematics from support ankle to hip node
         xhip = l*sin(t0) + l*sin(t01)
         zhip = l*cos(t0) + l*cos(t01)
         
         print('hip', xhip,zhip)
         direction = 1.0
         if (baseFoot == 'left'):
             direction = 1.0
         elif (baseFoot == 'right'):
             direction = -1.0
             #~define fail case
         #define new reference for goal point to place ankle of swing leg
         xNew = (direction*100*x) - xhip
         zNew = zhip- (100*z)
         
         #rotate referrence frame to use standard equations
         copyX = deepcopy(xNew)
         xNew = deepcopy(zNew)
         zNew = -1.0*deepcopy(copyX)
         
         print('new s',xNew,zNew)
         phi = 0.0 #total angle constrant to define third joint of swing leg, based in rotated reference frame
         
         
         #calculate needed values
         x2 = pow(xNew,2)
         z2 = pow(zNew,2)
         ls = pow(l,2)
         dist = sqrt(x2 + z2)
         
         #check that desired point is withing workspace of swing leg
         if (dist > 124.0):
             print('dist fail')
             continue
        
         #calculate angle of knee joint
         t4 = acos( ((x2+z2)-(ls+ls))/(2*l*l))
         
         #calculate joint one, t4 or hip joint
         gamma = acos( (x2 + z2 + ls - ls)/(2*l*sqrt(x2 + z2)))
         beta = atan2(zNew,xNew) 
         t3 = beta - gamma
         
         #calculate ankle joint to maintaing flat ground contact
         t5 = phi - (t3+t4)
         
         #convert calculated joints to reference basis for Legchair model
         t3Real = t3-t012 #account for rotation of hip frame due to support leg angles
         t4Real = t4
         t5Real = t5
         
         
         #print values
         print('in joints', d(t0),d(t1),d(t2))
         print('calculated joints',d(t3),d(t4),d(t5))
         print('updated joints', d(t0),d(t1),d(t2),d(t3Real),d(t4Real),d(t5Real))
         print('fk', l*cos(t3)+l*cos(t3+t4),l*sin(t3)+l*sin(t3+t4))
         print('gamma inner',(x2 + z2 + ls - ls)/(2*l*sqrt(x2 + z2)))
         print('gamma', gamma)
         print('beta', beta)
         
         #check that all joint constraints are met
         if ( (t3Real < -1.0*pi/2) or (t3Real > 1.0*pi/2)):
             print('t3fail')
             print('')
             continue
         
         if ( (t4Real < 0) or (t3Real > 1.0*pi/2)): #opposite of ik direction
             print('t4fail')
             print('')
             continue
             
         if ( (t5Real < -1.0*pi/4) or (t3Real > 1.0*pi/4)): #opposite of ik direction
             print('t5fail')
             print('')
             continue
         
         if ((t0+t1+t2+t3Real+t4Real+t5Real) != 0):
             print('tAllfail')
             print('')
             continue
         
         
         #return random-calculated succesful joints for desired pose
         break
    print('')
    return [t0, t1, t2, t3Real, t4Real, t5Real]
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
    # f = open('Left.txt')
    # lines = f.read()

    # bisect = make_tuple(lines.split('\n'))
    #     # cuta = bisect.split(' ')
    # print bisect[5]


    #### YOUR CODE HERE ####
    
    # pumarobot = env.ReadRobotXMLFile('robots/puma.robot.xml')
    # env.Add(pumarobot)
    # time.sleep(0.1)

    # PUMA = env.GetRobots()[1]
    # #Get next to PR2
    # # PUMA.SetTransform(numpy.dot(matrixFromPose([1,0,0,0,0,1,0]),PR2.GetTransform())) # set new pose next to PR2
    ik = randomGoalIk(0.4,0,'left')

    with env:
        jointnames = ['leftAnkle','leftKnee','leftHip','rightHip','rightKnee','rightAnkle']
        # [.5,-0.45,-0.1,-.1,-.45,0.65]
        # [.25,-0.15,-0.1,-.1,-.15,0.25]
        # manip = PR2.GetManipulator('foot')
        # print manip
        # print manip.GetTransform()
        indices = [PR2.GetJoint(name).GetDOFIndex() for name in jointnames]
        print PR2.GetDOFValues(indices)
        # print indices
        PR2.SetActiveDOFs([PR2.GetJoint(name).GetDOFIndex() for name in jointnames])
        PR2.SetDOFValues(ik,indices) # set the first 6 dof values
        lowerlimit, upperlimit = PR2.GetDOFLimits(indices);
        # for i in range(0,len(lowerlimit)):
        #     print lowerlimit[i], upperlimit[i]
        # PR2.GetLinks()[0].SetTransform()
        # print PR2.GetLinks()[0].GetTransform()
        ################
        # TR = PR2.GetManipulator('foot').GetTransform()[0:4,3]
        # print TR
        # T = PR22.GetTransform()[0:4,0:4]
        # T[0,3] = TR[0]
        # T[1,3] = TR[1]
        # T[2,3] = TR[2]
        # T[3,3] = TR[3]
        # # env.Remove(PR2)
        # # arr =numpy.dot(matrixFromPose([1,0,0,0,T[0,4],T[1,4],T[1]])
        # jointnames = ['leftAnkle','leftKnee','leftHip','rightHip','rightKnee','rightAnkle']
        # indices = [PR22.GetJoint(name).GetDOFIndex() for name in jointnames]
        # # print indices
        # PR22.SetActiveDOFs([PR22.GetJoint(name).GetDOFIndex() for name in jointnames])
        # # PR22.SetTranslation3D(T)
        # PR22.SetTransform(T)
        # PR22.SetDOFValues([.25,-0.15,-0.1,-.1,-.15,0.25],indices)

        # TR = PR22.GetManipulator('foot').GetTransform()[0:4,3]
        # # print TR
        # T = PR2.GetTransform()[0:4,0:4]
        # T[0,3] = TR[0]
        # T[1,3] = TR[1]
        # T[2,3] = TR[2]
        # T[3,3] = TR[3]

        # PR2.SetActiveDOFs([PR2.GetJoint(name).GetDOFIndex() for name in jointnames])
        # # PR22.SetTranslation3D(T)
        # PR2.SetTransform(T)
        # PR2.SetDOFValues([-.05,-0.35,0.17,.17,.35,-0.3],indices)
        #############
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

