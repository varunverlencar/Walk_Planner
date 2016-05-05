
#Randomly finds one Legchair Configuration, given the Constraint of placinf the swing foot at the given XZ point
#	takes 
#		a x location in meters
#		a z location in meters
#		a 'string' to indicate which foot is the base of the model, 'left' or 'right'
#example command randomFoalIk(0.4,0.2,'left')
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
