#!/usr/bin/env python

#Author: Emanuele Sansebastiano
#Date: December 2016
#Mail: emanuele.sansebastiano@outlook.com

from geometry_msgs.msg import TwistStamped
import termios, fcntl, sys, os
import rospy
from std_msgs.msg import Float64MultiArray

#import service library
from std_srvs.srv import Empty

#topic to command
twist_topic="/g500/velocityCommand"
#base velocity for the teleoperation (0.5 m/s) / (0.5rad/s)
baseVelocity=0.5

msg_mode ="""
Options to control the robot Girona 500:

a) Velocity input (Twist input)
b) Force input (Thrust input)

Entering 'a' or 'b' to choose.
"""

msg_vel = """
Control Girona 500 by VELOCITY (Twist) input
---------------------------
"""

msg_thrust = """
Control Girona 500 by FORCE (Thrust) input
---------------------------
"""

msg_command = """Enter one of the following keys to move the robot Girona 500...

w-s: go up-down
i-k: go frontward-backward
j-l: go left-right

e-d: tilt frontward-backward
u-o: turn left-right

enter-space bar: benchmark service start-stop

Recall that Girona 500 
cannot rotate around the x-axis

CTRL-C to quit
"""

msg_empty = """



"""

#Console input variables to teleop it from the console
fd = sys.stdin.fileno()
oldterm = termios.tcgetattr(fd)
newattr = termios.tcgetattr(fd)
newattr[3] = newattr[3] & ~termios.ICANON & ~termios.ECHO
termios.tcsetattr(fd, termios.TCSANOW, newattr)
oldflags = fcntl.fcntl(fd, fcntl.F_GETFL)
fcntl.fcntl(fd, fcntl.F_SETFL, oldflags | os.O_NONBLOCK)

##wait for benchmark init service
rospy.wait_for_service('/startBench')
start=rospy.ServiceProxy('/startBench', Empty)

##wait for benchmark stop service
rospy.wait_for_service('/stopBench')
stop=rospy.ServiceProxy('/stopBench', Empty)

#The try is necessary for the console input!
try:
    
    counter_wrong = 0
    print msg_mode
    while not rospy.is_shutdown():
	try:
	    c = sys.stdin.read(1)		
	    if c=='a':
		print msg_empty
		print msg_empty
		print msg_empty
	        print msg_empty
	        
		##create the publishers for velocity mode
		pub = rospy.Publisher(twist_topic, TwistStamped, queue_size=1)
		rospy.init_node('keyboardCommand')


	        ##Start velocity command loop
		counter_wrong = 0
		print msg_vel
		print msg_command

		while not rospy.is_shutdown():
		    msg = TwistStamped()
		    try:
			c = sys.stdin.read(1)
			##Depending on the character set the proper speeds
			if c=='\n':
			    start()
			    print "Benchmarking Started!"
			elif c==' ':
			    stop()
			    print "Benchmark finished!"
			   
			##commands to guide the robot 	  
			else:
			    #go up-down
			    if c=='w' or c=='s':
				if c=='w':
				    msg.twist.linear.z = -baseVelocity
				else:
				    msg.twist.linear.z = baseVelocity

			    #tilt front-back
			    elif c=='e' or c=='d':
				if c=='e':
				    msg.twist.angular.y = -baseVelocity
				else:
				    msg.twist.angular.y = baseVelocity

			    #go frontward-backward
			    elif c=='i' or c=='k':
				if c=='i':
				    msg.twist.linear.x = baseVelocity
				else:
				    msg.twist.linear.x = -baseVelocity

			    #go left-right
			    elif c=='j' or c=='l':
				if c=='j':
				     msg.twist.linear.y = -baseVelocity
				else:
				    msg.twist.linear.y = baseVelocity

			    #turn left-right
			    elif c=='u' or c=='o':
			        if c=='u':
				    msg.twist.angular.z = -baseVelocity
				else:
				    msg.twist.angular.z = baseVelocity

			    else:
				##start wrong key feedback for velocity mode
				print 'wrong key pressed'
				counter_wrong = counter_wrong +1
				if counter_wrong > 10:
				    counter_wrong = 0
				    print msg_empty
				    print msg_empty
				    print msg_vel
				    print msg_command
				#end wrong key feedback for velocity mode

			while c!='':
			    c = sys.stdin.read(1)
		    except IOError: pass

		    ##publish the message
		    pub.publish(msg)
		    rospy.sleep(0.1)
		#end velocity command loop 

	    if c=='b':
		print msg_empty
		print msg_empty
		print msg_empty

		##create the publishers for force mode
		pub = rospy.Publisher('/g500/thrusters_input', Float64MultiArray, queue_size=1)
		rospy.init_node('keyboardCommand')

	        ##Start force command loop
		counter_wrong = 0
		print msg_thrust
		print msg_command

		while not rospy.is_shutdown():
		    msg = Float64MultiArray()
		    #to lock the vehicle position
		    msg.data = [0,0,0,0,0]

		    try:
			c = sys.stdin.read(1)
			##Depending on the character set the proper speeds
			if c=='\n':
			    start()
			    print "Benchmarking Started!"
			elif c==' ':
			    stop()
			    print "Benchmark finished!"
			   
			##commands to guide the robot 	  
			else:
			    #go up-down
			    if c=='w' or c=='s':
				if c=='w':
				    msg.data = [0,0,1,1,0]
				else:
				    msg.data = [0,0,-1,-1,0]

			    #tilt front-back
			    elif c=='e' or c=='d':
				if c=='e':
				    msg.data = [0,0,1,-1,0]
				else:
				    msg.data = [0,0,-1,1,0]

			    #go frontward-backward
			    elif c=='i' or c=='k':
				if c=='i':
				    msg.data = [-1,-1,0,0,0]
				else:
				    msg.data = [1,1,0,0,0]

			    #go left-right
			    elif c=='j' or c=='l':
				if c=='j':
				     msg.data = [0,0,0,0,-1]
				else:
				    msg.data = [0,0,0,0,1]

			    #turn left-right
			    elif c=='u' or c=='o':
			        if c=='u':
				    msg.data = [1,-1,0,0,0]
				else:
				    msg.data = [-1,1,0,0,0]

			    else:
				##start wrong key feedback for force mode
				print 'wrong key pressed'
				counter_wrong = counter_wrong +1
				if counter_wrong > 10:
				    counter_wrong = 0
				    print msg_empty
				    print msg_empty
				    print msg_thrust
				    print msg_command
				#end wrong key feedback for force mode

			while c!='':
			    c = sys.stdin.read(1)
		    except IOError: pass

		    ##publish the message
		    pub.publish(msg)
		    rospy.sleep(0.1)
		#end force command loop 
		    
	    else:
		#start wrong key feedback for choosing mode
		print 'wrong key pressed'
		counter_wrong = counter_wrong +1
		if counter_wrong > 10:
		    counter_wrong = 0
		    print msg_empty
		    print msg_empty
		    print msg_mode
		#end wrong key feedback for choosing mode 

	    while c!='':
		c = sys.stdin.read(1)
	except IOError: pass

##Other input stuff
finally:
    termios.tcsetattr(fd, termios.TCSAFLUSH, oldterm)
    fcntl.fcntl(fd, fcntl.F_SETFL, oldflags)
