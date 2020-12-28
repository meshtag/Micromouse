#! /usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist,Point
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *
# from pid_tune.msg import PidTune
from std_msgs.msg import Float32,String
import math
import sys
import random
import copy

position_ = Point()
active_ = False
pub_ = None
count = 0
prev_error = 0
integ = 0
diff = 0
error = 0
check = 0


# nodelist = [['x y coordinates, distance betw nodes, node type, direction']]
nodelist = []
# node cases = {0 - left, 1 - right,2 - left and right, 3 - left and straight, 4 - right and straight, 5 - all , 6 - Uturn}
# priority = {0 - Left, 1 - Straight, 2 - Right, 3 - Uturn}
# 'Wall follower - [%s] - %s' % (state, state_dict_[state])

x_dist = 0
y_dist = 0
yaw_ = 0
kP = 0.5
pos = 0
switch = False

final_path = []
final_rev_path = []

def getAngle():
	msg = Odometry()
	quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
	euler = transformations.euler_from_quaternion(quaternion)
	a = euler[2]
	return a

def rotate(angle_deg):
	global yaw_, switch
	global kP
	msg = Twist()
	switch = True
	r = rospy.Rate(10)
	angle = angle_deg * math.pi / 180
	print(angle, yaw_%1.57, yaw_)
	if angle > 1.57:
		if yaw_%1.57 < 0.7535:
			if yaw_ < 0:
				angle -= ((yaw_)%1.57)
			else:
				angle -= ((yaw_)%1.57)
		else:
			if yaw_ < 0:
				angle -= (abs((yaw_)%1.57) - 1.57)
			else:
				angle -= (abs((yaw_)%1.57) - 1.57)
	elif angle < 1.57:
		if yaw_%1.57 < 0.7535:
			if yaw_ < 0:
				angle -= ((yaw_)%1.57)
			else:
				angle -= ((yaw_)%1.57)
		else:
			if yaw_ < 0:
				angle -= (abs((yaw_)%1.57) - 1.57)
			else:
				angle -= (abs((yaw_)%1.57) - 1.57)

	# print(yaw_+ angle)
	print(angle)
	a0 = yaw_
	current = 0.0
	while not rospy.is_shutdown() and abs(current) < (abs(angle)-0.01):
		# print(current, angle)
		# print(rospy.is_shutdown())
		a1 = yaw_
		current = (a1 - a0)
		msg.angular.z = kP*(angle - current)
		pub_.publish(msg)
		r.sleep()
	stop_bot()
	print('Yooo')
	switch = False

def TurnLeft():
	print('left')
	rotate(90)

def TurnRight():
	print('right')
	rotate(-90)

def Turn180():
	print('Uturn')
	rotate(180)

# def take_action(regions):
#     msg = Twist()
#     linear_x = 0
#     angular_z = 0

def clbk_laser(msg):
    global regions_
	# regions_ = {
	# 	'right':max(msg.ranges[0:30]),
	# 	'left':max(msg.ranges[330:360]),
	# 	'front':max(msg.ranges[30,330]),
	# 	}
    #360/5 = 72
    regions_ = {
        # 'right':  min(min(msg.ranges[0:49]),10.0),
        # 'fright': min(min(msg.ranges[72:143]),10.0),
        # 'front':  min(min(msg.ranges[159:215]),10.0),
        # 'fleft':  min(min(msg.ranges[216:287]),10.0),
        # 'left':   min(min(msg.ranges[310:359]),10.0),
		'right':min(msg.ranges[0:30]),
		'left':min(msg.ranges[330:360]),
		'front':min(msg.ranges[165:195]),
		'fleft':min(msg.ranges[195:330]),
		'fright':min(msg.ranges[30:165]),
        'frontmax':max(msg.ranges[165:195]),
    	}


def clbk_odom(msg):
	global position_,pos
	global yaw_

    # position
	position_ = msg.pose.pose.position
	if pos == 0:
		new_node = [position_.x,position_.y,0.0,0,1]
		nodelist.append(new_node)
		pos = 1
	quaternion =(
		msg.pose.pose.orientation.x,
		msg.pose.pose.orientation.y,
		msg.pose.pose.orientation.z,
		msg.pose.pose.orientation.w)
	euler = transformations.euler_from_quaternion(quaternion)
	yaw_ = euler[2]
	# print(yaw_)

#Func for editing node list
def edit_node_list(case, dir):
	global position_,regions_
	print(case, dir)
	print(regions_)
	prev_x = nodelist[-1][0]
	prev_y = nodelist[-1][1]
	dist = abs(prev_x - position_.x) + abs(prev_y - position_.y)
    # This mechanism of detecting same node needs to be modified.
	if -0.1 <= round((prev_x-position_.x),2) <= 0.1 and -0.1 <= round((prev_y-position_.y),2) <= 0.1 and dist < 0.2:#This will give error in certain cases.
		print('Same node')
		print(nodelist)
	else:
		dist = abs(prev_x - position_.x) + abs(prev_y - position_.y)
        # dist = abs(prev_x - position_.x) + abs(prev_y - position_.y)
		# posx = prev_node
		# distance = if(nodelist[-1])
		# new = '%s %s %s %s' % (position_.x, position_.y, distance, case, dir)
		new_node = [position_.x,position_.y,dist,case,dir]
		nodelist.append(new_node)
		dfs(nodelist)
		print(nodelist)
		print("New Node Added")


def findPath(nodelist):

	# Assumption : The maze has no loops and nodelist includes all nodes till center of maze
	# The code returns 1 good path without U-Turns

	# Node Cases : 0 = Left, 1 = Right, 2 = Left and Right, 3 = Left and Straight, 4 = Right and Straight, 5 = All , and 6 = U-Turn
	# dir = {0 - Left, 1 - Straight, 2 - Right, 3 - Uturn}
	# First Node = [position_.x, position_.y, 0.0, 0, 1]
	# Node = [position_.x, position_.y, dist, case, dir]

	nl = copy.deepcopy(nodelist)
	dir_stack = []

	# Final path is in format [dir, dir, dir, ...]
	# Here, dir is coded as : Left = 0, Straight = 1, and Right = 2

	path_f = []
	for node in nl:
		print(dir_stack)
		#if ((node[0]*100) in range(-16, 16)) and ((node[1]*100) in range(-16, 16)):
		if (node[0] > -0.16 and node[0] < 0.16) and (node[1] > -0.16 and node[1] < 0.16):
			i = 2
			for j in range(0, len(dir_stack)):
				if i >= len(dir_stack):
					break
				if i < 2:
					i = 2
				nset = str(dir_stack[i-2]) + str(dir_stack[i-1]) + str(dir_stack[i])
				if nset == '130':
					del dir_stack[(i-2):i]
					dir_stack[i-2] = 2
					i -= 2
				elif nset == '031':
					del dir_stack[(i-2):i]
					dir_stack[i-2] = 2
					i -= 2
				elif nset == '030':
					del dir_stack[(i-2):i]
					dir_stack[i-2] = 1
					i -= 2
				elif nset == '230':
					del dir_stack[(i-2):i]
					dir_stack[i-2] = 3
					i -= 2
				else:
					i += 1
			#path_f = copy.deepcopprint("Final path : ", o)y(dir_stack)
			break
		elif node[3] in range(2, 7):
			dir_stack.append(node[4])
	#Just for testing
	i = 2
	for j in range(0, len(dir_stack)):
		if i >= len(dir_stack):
			break
		if i < 2:
			i = 2
		nset = str(dir_stack[i-2]) + str(dir_stack[i-1]) + str(dir_stack[i])
		if nset == '130':
			del dir_stack[(i-2):i]
			dir_stack[i-2] = 2
			i -= 2
		elif nset == '031':
			del dir_stack[(i-2):i]
			dir_stack[i-2] = 2
			i -= 2
		elif nset == '030':
			del dir_stack[(i-2):i]
			dir_stack[i-2] = 1
			i -= 2
		elif nset == '230':
			del dir_stack[(i-2):i]
			dir_stack[i-2] = 3
			i -= 2
		else:
			i += 1
	path_f = copy.deepcopy(dir_stack)
	print(path_f)
	x = raw_input("wait here")
	return path_f

def ReversePath(path_orig):
	path_r = copy.deepcopy(path_orig)
	for i in range(0, len(path_r)):
		if path_r[i] == 0:
			path_r[i] = 2
		elif path_r[i] == 2:
			path_r[i] = 0
	path_rev = path_r[::-1]
	print(path_rev)
	return path_rev

def FinalPaths(nodelist):

	# Call this function to get both paths
	# path_rev = Path to go back to origin from centre
	# path_orig = Final path to go to centre from origin

	path_orig = findPath(nodelist)
	path_rev = ReversePath(path_orig)
	return path_orig, path_rev

c2 = 0

def checkNode():

	global regions_, count
	global c2
	regions = regions_
	msg = Twist()
	cb = False
	d = 0.1
	e = 0.12
	x = ""
	if regions['front'] > d and regions['left'] < e and regions['right'] < e and c2 == 0:
	    	return False
	elif c2 == 0:
		stop_bot()
		cb = error_correction(0.08)
		c2 = 1
		if cb == True :
			c2 = 1
	elif c2 == 1:
		if regions['front'] < d and regions['left'] > e and regions['right'] > e:
			c2 = 2
			return True

		elif (position_.x > -0.2 and position_.x < 0.2) and (position_.y > -0.2 and position_.y < 0.2):
			print("FINAL")
			stop_bot()
			x = raw_input("hold on...")
			c2 = 2
			return False
					
		elif (regions['front'] == 0.24 and regions['right'] == 0.24 and regions['fright'] ==0.34) or (regions['front'] == 0.24 and regions['fleft'] == 0.34 and regions['left'] == 0.24):
			print("FINAL - CENTRE")
			stop_bot()
			x = raw_input("hold on...")
			c2 = 2
			return False
			
		elif regions['front'] > d and regions['left'] > e and regions['right'] < e:
			c2 = 2
			return True

		elif regions['front'] > d and regions['left'] > e and regions['right'] > e:
			c2 = 2
			return True

		elif regions['front'] > d and regions['left'] < e and regions['right'] > e:
			c2 = 2
			return True

		elif regions['front'] < d and regions['left'] > e and regions['right'] < e:
			c2 = 2
			TurnLeft()
			return False

		elif regions['front'] < d and regions['left'] < e and regions['right'] > e:
			c2 = 2
			TurnRight()
			return False

		elif regions['front'] > d and regions['left'] < e and regions['right'] < e:
			c2 = 0
			return False

		else:
			return False

	elif c2 == 2:
		c2 = 0
		error_correction(0.1)
		if cb == True :
			c2 = 2
		stop_bot()
		print('count 2')

	else:
		follow_the_wall()
		print(regions_)
	return False

def takeTurn(pdir):
	msg = Twist()
	if pdir == 0:
		TurnLeft()
	elif pdir == 2:
		TurnRight()
	follow_the_wall()

def followPath(path):
	# path = [dir, dir, ...]
	# dir = 0 - Left, 1 - Straight, 2 - Right
	
	msg = Twist()
	global regions_
	pdir = -1
	while len(path) > 0:
		msg.linear.x = 0.25
		pub_.publish(msg)
		if checkNode() == True:
			stop_bot()
			pdir = path[0]
			del path[0]
			takeTurn(pdir)
			print(pdir, path)
			stop_bot()
	msg.linear.x = 0.25
	pub_.publish(msg)

def reverse():
	followPath(final_rev_path)
	print("REACHED ORIGIN")
	y = raw_input("hold on a sec, wait...")
	Turn180()
	followPath(final_path)
	print("REACHED CENTRE AGAIN")
	z = raw_input("woah! 1st run done!")
	
# func for following the wall
def follow_the_wall():
	global regions_,long_error,zero_error,prev_error
	global error, integ, diff
	global kp,ki,kd
    # global long_error, zero_error
	msg = Twist()
	integ = 0
	diff = 0
	dist = 0.1
	error = dist - regions_['left']
	diff = (error - prev_error)/0.05
	integ += error
	output = kp*error + ki*integ + kd*diff
	if error >= 0.1:
		error = 0.0
		print(error)
		output = 0
	prev_error = error
	if(regions_['frontmax'] > 0.3):
        	msg.linear.x = 0.4
	else:
		msg.linear.x = 0.16
	if(0.18>regions_['left'] > 0.1 or regions_['left'] <0.06):
		msg.angular.z = 0.0 - (abs(output)/output)*0.1
	long_error.publish(output)
	zero_error.publish(0.0)
	pub_.publish(msg)

def error_correction(arg):
	global yaw_, regions_, switch
	error_x,error_y = 0,0
	kP = 0.5
	switch = True
	r = rospy.Rate(10)
	print('Error correction')
	# print(yaw_)
	if(1.3 <= round(yaw_,1) <= 1.7):
	    error_x, error_y = arg,0
	elif(-1.7 <= round(yaw_,1) <= -1.3):
	    error_x, error_y = -arg,0
	elif (-0.2 <= round(yaw_,1) <= 0.2):
	    error_x, error_y = 0,-arg
	elif (2.8 <= round(yaw_,1) <= 3.2) or (-3.2 <= round(yaw_,1) <= -2.8) :
	    error_x, error_y = 0,arg
	else:
		print('No Yaw')
	targetx = position_.x + error_x
	targety = position_.y + error_y
	msg = Twist()
	while not rospy.is_shutdown():
		current = position_.x if (error_x != 0) else position_.y
		msg.linear.x = abs(targetx - position_.x) if (error_x != 0) else abs(targety - position_.y)
		print(msg.linear.x)
		if(round(msg.linear.x,2) == 0.0) or regions_['front'] < 0.08:
			stop_bot()
			r.sleep()
			switch = False
			return False
		else:
		    msg.linear.x = 0.16
		pub_.publish(msg)
		r.sleep()


def move_ahead():
	msg = Twist()
	msg.linear.x = 0.09
	pub_.publish(msg)
	rate = rospy.Rate(20)
	rate.sleep()

#func for stopping the bot
def stop_bot():
    print('bot stopped')
    msg = Twist()
    msg.linear.x = 0.0
    msg.angular.z = 0.0
    pub_.publish(msg)

# def Uturn():
#     print(turn180)
#     posx = nodelist[-1]#Will give error
#     posy = nodelist[-1]

centre_detect = 0
#func for classifying the
def take_action():
	global switch
	global check
	global regions_,count,centre_detect
	global final_path
	global final_rev_path
	check = 0
	regions = regions_
	msg = Twist()
	linear_x = 0
	angular_z = 0
	bool = False
	a = ""

	if (centre_detect == 1) and (not ((position_.x > -0.2 and position_.x < 0.2) and (position_.y > -0.2 and position_.y < 0.2))):
		print("HERE I AM, OUT OF CENTRE")
		stop_bot()
		print(final_path)
		print(final_rev_path)
		a = raw_input("wait a sec...")
		reverse()

	state_description = ''
	# print("State Description        ...........................")
	# print(state_description)
	# print(count)
	d = 0.1
	e = 0.12
	if regions['front'] > d and regions['left'] < e and regions['right'] < e and count == 0:
	    state_description = 'case 8 - left and right'
	    follow_the_wall()
	elif count == 0:
		stop_bot()
		# if pos == 0
		bool = error_correction(0.08)
		count = 1
		print("I was here .............")
		print(count)
		# stop_bot()
		if bool == True :
			count = 1
	elif count == 1:
		print(nodelist)
		if regions['front'] < d and regions['left'] > e and regions['right'] > e:
			state_description = 'case 2 - front obs'
			edit_node_list(2,0)
			TurnLeft()
			count = 2
		elif (position_.x > -0.2 and position_.x < 0.2) and (position_.y > -0.2 and position_.y < 0.2) and centre_detect == 0:
			centre_detect = 1
			state_description = 'case 10- centre'
			#stop_bot()
			o, r = FinalPaths(nodelist)
			print("Final path : ", o)
			print("Reverse path : ", r)
			final_path = copy.deepcopy(o)
			final_rev_path = copy.deepcopy(r)
			#Turn180()
			print("Uturn done")
			#while (position_.x > -0.2 and position_.x < 0.2) and (position_.y > -0.2 and position_.y < 0.2):
				#msg.linear.x = 0.07
				#pub_.publish(msg)
			count = 2		
		elif (regions['front'] == 0.24 and regions['right'] == 0.24 and regions['fright'] ==0.34) or (regions['front'] == 0.24 and regions['fleft'] == 0.34 and regions['left'] == 0.24) and centre_detect == 0:
			state_description = 'case 10- centre'
            
			centre_detect = 1
			o, r = FinalPaths(nodelist)
			print("Final path : ", o)
			print("Reverse path : ", r)
			final_path = copy.deepcopy(o)
			final_rev_path = copy.deepcopy(r)
			#Turn180()
			print("Uturn done")
			#while (regions['front'] == 0.24 and regions['right'] == 0.24 and regions['fright'] ==0.34) or (regions['front'] == 0.24 and regions['fleft'] == 0.34 and regions['left'] == 0.24):
				#msg.linear.x = 0.07
				#pub_.publish(msg)
			count = 2	
		elif regions['front'] > d and regions['left'] > e and regions['right'] < e:
			state_description = 'case 3 - right obs'
			edit_node_list(3,0)
			TurnLeft()
			count = 2
		elif regions['front'] > d and regions['left'] > e and regions['right'] > e:
			state_description = 'case 4 - no obs'
			edit_node_list(5,0)
			# print("I was here ...............................hahahahahaha")
			count = 2
			TurnLeft()
		elif regions['front'] > d and regions['left'] < e and regions['right'] > e:
			state_description = 'case 5 - left obs'
			edit_node_list(4,1)
			count = 2
		elif regions['front'] < d and regions['left'] > e and regions['right'] < e:
			state_description = 'case 6 - front and right obs'
			edit_node_list(0,0)
			TurnLeft()
			count = 2
		elif regions['front'] < d and regions['left'] < e and regions['right'] > e:
			state_description = 'case 7 - front and left obs'
			edit_node_list(1,2)
			TurnRight()
			count = 2

		elif regions['front'] < d and regions['left'] < e and regions['right'] < e:
			state_description = 'case 8 - front and left and right'
			print( regions_)
			edit_node_list(6,3)
			Turn180()
			count = 2
		elif regions['front'] > d and regions['left'] < e and regions['right'] < e:
			state_description = 'case 9 - left and right'
			count = 0
			follow_the_wall()
		#elif (regions['front'] > 0.1 and regions['right'] > 0.1 and regions['fright'] > 0.1) or (regions['front'] > 0.1 and regions['fleft'] > 0.1 and regions['left'] > 0.1):
			#state_description = 'case 9- centre'
			#stop_bot()
			#o, r = FinalPaths(nodelist)
			#print("Final path : ", o)
			#print("Reverse path : ", r)
				
		else:
			state_description = 'unknown case'
			print('Wrongcase')
			rospy.loginfo(regions)
		print("State_Description                 ..........................")
		print(state_description)
	# elif count == 2 and regions['fleft'] > e and regions['fright'] > e and regions['front'] > d and regions['left'] > e and regions['right'] > e:
	# 	count = 0
	# 	# move_ahead()
	# 	follow_the_wall()
	elif count == 2:
		count = 0
		error_correction(0.1)
		if bool == True :
			count = 2
		stop_bot()
		print('count 2')

	else:
		follow_the_wall()
		print(regions_)



def wall_follower_switch(req):
    global active_
    active_ = req.data
    res = SetBoolResponse()
    res.success = True
    res.message = 'Done!'
    return res

def wall_follow(alt):
    global kp,ki,kd
    kp = alt.Kp * 0.0006
    ki = alt.Ki * 0.000008
    kd = alt.Kd * 0.0003

def main():
	global pub_, active_,long_error,zero_error
	global kp,ki,kd
	global check
	# global long_error, zero_error
	kp = 0.5
	ki = 0
	kd = 1500 *0.03
	rospy.init_node('reading_laser')

	pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

	sub = rospy.Subscriber('/my_mm_robot/laser/scan', LaserScan, clbk_laser)
	sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
	# rospy.Subscriber('/pid_tuning_altitude', PidTune, wall_follow)

	long_error = rospy.Publisher('/long_error',Float32, queue_size=1)
	zero_error = rospy.Publisher('/zero_error',Float32, queue_size=1)

	srv = rospy.Service('wall_follower_switch', SetBool, wall_follower_switch)
	print(srv)
	print(position_.x,position_.y)
	rate = rospy.Rate(20)
	while not rospy.is_shutdown():


		try:
			if switch == False :
				take_action()
				#print("we are in")
				#pass
				
		except Exception as e:
			print(e)
		rate.sleep()
		# msg = Twist()
		# if state_ == 0:
		# 	msg = find_wall()
		# elif state_ == 1:
		# 	msg = turn_left()
		# elif state_ == 2:
		# 	msg = follow_the_wall()
		# 	pass
		# else:
		# 	rospy.logerr('Unknown state!')

    	# pub_.publish(msg)



if __name__ == '__main__':
    main()
	