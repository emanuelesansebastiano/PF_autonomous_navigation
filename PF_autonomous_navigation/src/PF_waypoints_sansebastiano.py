#!/usr/bin/env python

#Author: Emanuele Sansebastiano
#Date: January 2017
#Mail: emanuele.sansebastiano@outlook.com

from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float64MultiArray
import rospy
import numpy as nq
import tf

from getkey import getkey, keys
# if the system turn a error about getkey, use the following command in a shell to install the extension
# 'sudo pip install getkey'

#import service library
from std_srvs.srv import Empty

msg_mode ="""
Options to control the robot Girona 500:

a) Velocity input (Twist input)
b) Force input (Thrust input)

Entering 'a' or 'b' to choose.
"""

msg_scenario ="""
Choose the scenario you are using:

a) Basic
b) Turns
c) Heights

Entering 'a', 'b' or 'c' to choose.
"""

msg_empty = """



"""

### WAYPOINTS
BasicP=[[1.4,4.5,7.5],[-1.4,-4.5,7.5]]
TurnsP=[[-1.4,-4.5,7.5],[1.49,4.8,7.5],[7.4,2.975,7.5],[9.29,9.0,7.5],[7.15,9.68,7.5]]
HeightsP=[[-4.96,-15.9,6.92],[-2.16,-6.9,6.92],[-1.96,-6.3,7.5],[1.4,4.5,7.5],[1.6,5.15,6.97],[3.84,12.35,6.97],[4.07,13,6.43],[4.63,14.8,6.43],[4.85,15.43,5.9],[6.53,20.83,5.9]]

#point velocity 
v_point = [0,0,0]

##initialize the node
rospy.init_node('waypointFollow')

##wait for benchmark init service
rospy.wait_for_service('/startBench')
start=rospy.ServiceProxy('/startBench', Empty)

##wait for benchmark stop service
rospy.wait_for_service('/stopBench')
stop=rospy.ServiceProxy('/stopBench', Empty)

##position listener generator
listener = tf.TransformListener()

#funtion to extract the coordinations from the translation matrix
def matrix2vector_translation(matrix):
    coor = nq.zeros((3,1))
    coor[0] = matrix[0,3]
    coor[1] = matrix[1,3]
    coor[2] = matrix[2,3]
    return coor

def tang_XYplane(quat):
    q = quat
    tan = 2*(q[0]*q[1] + q[2]*q[3])/(1 - 2*(q[1]*q[1] + q[2]*q[2]))
    return tan 

#functions to move the vehicle (dynamic mode)
def up_down_trans(direction,intensity):
    q= 0; w= 0; e= 0; r= 0; t= 0
    if direction == 'up':
        e = -intensity; r = -intensity
    elif direction == 'down':
        e = +intensity; r = +intensity
    else:
        print 'error occurred in the function: up_down_trans'
        exit() 
    thrust = [q,w,e,r,t]
    return thrust

def front_back_trans(direction,intensity):
    q= 0; w= 0; e= 0; r= 0; t= 0
    if direction == 'front':
        q = -intensity; w = -intensity
    elif direction == 'back':
        q = +intensity; w = +intensity
    else:
        print 'error occurred in the function: front_back_trans'
        exit() 
    thrust = [q,w,e,r,t]
    return thrust

def left_right_trans(direction,intensity):
    q= 0; w= 0; e= 0; r= 0; t= 0
    if direction == 'left':
        t = +intensity
    elif direction == 'right':
        t = -intensity
    else:
        print 'error occurred in the function: left_right_trans'
        exit() 
    thrust = [q,w,e,r,t]
    return thrust

def left_right_turn(direction,intensity):
    q= 0; w= 0; e= 0; r= 0; t= 0
    if direction == 'left':
        q = -intensity; w = +intensity
    elif direction == 'right':
        q = +intensity; w = -intensity
    else:
        print 'error occurred in the function: left_right_trans'
        exit() 
    thrust = [q,w,e,r,t]
    return thrust


## main ----------------------------------------------------------------------------------

time_step = 0.05

# initial selector ----------------------------------

one_time_pid = 1
counter_wrong = 0
exit = 0

# kind of input selection
while not rospy.is_shutdown() and exit == 0:

    while one_time_pid == 1:
	print msg_mode
        one_time_pid = 0

    key = getkey()

    if key == 'a':
        # velocity input
        input_type = 0
        #create the publisher
        pub = rospy.Publisher('/g500/velocityCommand', TwistStamped,queue_size=1)
        exit = 1
    elif key == 'b':
        # thrust input
        input_type = 1
        #create the publishers for force mode
	pub = rospy.Publisher('/g500/thrusters_input', Float64MultiArray, queue_size=1)
        exit = 1
    else:
        print 'wrong key pressed'
        counter_wrong = counter_wrong +1
        if counter_wrong > 10:
            one_time_pid = 1
            counter_wrong = 0
            print msg_empty

    rospy.sleep(time_step)


one_time_pid = 1
counter_wrong = 0
exit = 0

# kind of scenario selection
while not rospy.is_shutdown() and exit == 0:

    while one_time_pid == 1:
	print msg_scenario
        one_time_pid = 0

    key = getkey()

    if key == 'a':
        # Basic scenario
        point = BasicP
        exit = 1
        print msg_empty
    elif key == 'b':
        # Turns scenario
        point = TurnsP
        exit = 1
        print msg_empty
    elif key == 'c':
        # Heights scenario
        point = HeightsP
        exit = 1
        print msg_empty

    else:
        print 'wrong key pressed'
        counter_wrong = counter_wrong +1
        if counter_wrong > 10:
            one_time_pid = 1
            counter_wrong = 0
            print msg_empty

    rospy.sleep(time_step)

# Control ------------------------------------------

#initilization
starting_waypoint = 0
j = starting_waypoint
size = len(point)
err = nq.zeros((4,2))
#print err

# start benchmark -stop service-
start()

exit = 0;
while not rospy.is_shutdown() and exit == 0:

    if j == 0:
        point_trans1 = point[j]
    else:
        point_trans1 = point[j-1]
    point_trans2 = point[j]
    
    point_rot = [0,0,0,0]

    try:
        (trans,rot) = listener.lookupTransform('/world','/girona500',rospy.Time(0))
    except (tf.LookupException,tf.ConnectivityException,tf.ExtrapolationException):
        continue

    wRv = tf.transformations.quaternion_matrix(rot)
    wTv = tf.transformations.translation_matrix(trans)
    wMv = nq.dot(wTv, wRv)
    vMw = tf.transformations.inverse_matrix(wMv)
    
    #points
    wRp = tf.transformations.quaternion_matrix(point_rot)
    #point1
    wTp1 = tf.transformations.translation_matrix(point_trans1) 
    wMp1 = nq.dot(wTp1, wRp)
    vMp1 = nq.dot(vMw, wMp1)
    #point2
    wTp2 = tf.transformations.translation_matrix(point_trans2)
    wMp2 = nq.dot(wTp2, wRp)
    vMp2 = nq.dot(vMw, wMp2)

    #distance error definition from the vehicle to the second point (point2)
    vTp = tf.transformations.translation_from_matrix(vMp2)

    #orientation of the line connecting point1 and point2 on the X-Y plane
    if j == 0:
        tanP = 0
    else:
        wTp2a = matrix2vector_translation(wTp2) 
        wTp1a = matrix2vector_translation(wTp1)
        tanP = (wTp2a[1]-wTp1a[1])/(wTp2a[0]-wTp1a[0])  
    #orientation vehicle
    tanV = tang_XYplane(rot)

    #tang error
    tan_err = tanP - tanV
    print vTp, tan_err
    #print tanP, -tanV,tan_err

    # Velocity input defintion
    if input_type == 0:
        #meta control
        K_init = 1
        vel_err = K_init*vTp; 
        vel_des = v_point + vel_err

        #control core
        max_vel = 0.5
        min_dist = 0.01 #[m]

        msg = TwistStamped()
        msg.twist.linear.x = 0; msg.twist.linear.y = 0; msg.twist.linear.z = 0
        msg.twist.angular.x = 0; msg.twist.angular.y = 0; msg.twist.angular.z = 0

        if nq.abs(vTp[0]) < min_dist:
            vel_des[0] = 0
        if nq.abs(vTp[1]) < min_dist:
            vel_des[1] = 0
        if nq.abs(vTp[2]) < min_dist:
            vel_des[2] = 0
        
        #P CONTROL (velocity error)
        #I want the pole in (-0.5) / K_p = 0.5
        # input to vehicle system: velocity
        # output from vehicle system: position
        K_p = 0.5
        vel_c = K_p*vel_des
        rot_c = nq.zeros((3,1))

        #Saturation control
        if nq.abs(vel_c[0]) > max_vel:
            vel_c[0] = max_vel*vel_c[0]/nq.abs(vel_c[0])
        if nq.abs(vel_c[1]) > max_vel:
            vel_c[1] = max_vel*vel_c[1]/nq.abs(vel_c[1])
        if nq.abs(vel_c[2]) > max_vel:
            vel_c[2] = max_vel*vel_c[2]/nq.abs(vel_c[2])

        #Proprtional translation motion along every direction (X-Y)
        if nq.abs(vTp[0]) > nq.abs(vTp[1]):
            vel_c[1] = vel_c[0]*vTp[1]/vTp[0]
            if nq.abs(vTp[1]) < min_dist:
                vel_c[1] = 0
        else:
            vel_c[0] = vel_c[1]*vTp[0]/vTp[1]
            if nq.abs(vTp[1]) < min_dist:
                vel_c[0] = 0

        #Rotational motion (forward) {P control}
        min_tan = min_dist
        if nq.abs(tan_err) > min_tan or vTp[0] < 0:
            rot_c[2] = K_p * tan_err
        #saturation
        if nq.abs(rot_c[2]) > max_vel:
            rot_c[2] = max_vel*rot_c[2]/nq.abs(rot_c[2])
        
        #Velocity publisher
        msg.twist.linear.x=vel_c[0]
        msg.twist.linear.y=vel_c[1]
        msg.twist.linear.z=vel_c[2]

        msg.twist.angular.x=rot_c[0]
        msg.twist.angular.y=rot_c[1]
        msg.twist.angular.z=rot_c[2]
        pub.publish(msg)
  
    else:
        max_vel = 0.5
        min_dist = 0.05 #[m] 
        down_scale = 4

        msg = Float64MultiArray()
	#to lock the vehicle position
        q= 0; w= 0; e= 0; r= 0; t= 0
	msg.data = [q,w,e,r,t]
        
        #PD CONTROL (position error)
        # I want the pole in (-pole) / K_p; K_d = ??? (by test: K_p=0.4 - K_d=1.6)
        # input to vehicle system: thrust [(rho*A*(v)^2)/a]
        # output from vehicle system: position
        K_p = 0.4; K_d = 1.6
        pos_err = vTp
        err[:,0] = err[:,1]
        err[0:3,1] = nq.transpose(pos_err)
        err[3,1] = tan_err
        #error derivate
        pos_err_d = (err[:,1] - err[:,0])/time_step

        vel_c = K_p*pos_err + K_d*pos_err_d[0:3]
        rot_c = nq.zeros((3,1))

        #Saturation control
        if nq.abs(vel_c[0]) > max_vel:
            vel_c[0] = max_vel*vel_c[0]/nq.abs(vel_c[0])
        if nq.abs(vel_c[1]) > max_vel:
            vel_c[1] = max_vel*vel_c[1]/nq.abs(vel_c[1])
        if nq.abs(vel_c[2]) > max_vel:
            vel_c[2] = max_vel*vel_c[2]/nq.abs(vel_c[2])

        #Proprtional translation motion along every direction (X-Y)
        if nq.abs(pos_err[0]) > nq.abs(pos_err[1]):
            vel_c[1] = vel_c[0]*pos_err[1]/pos_err[0]
            if nq.abs(pos_err[1]) < min_dist:
                vel_c[1] = 0
        else:
            vel_c[0] = vel_c[1]*pos_err[0]/pos_err[1]
            if nq.abs(pos_err[1]) < min_dist:
                vel_c[0] = 0

        if j == 0:
            rotation = 0
        else:
            rotation = 1
		# substitute 1 in the following line with a number bigger than 1 to esclude the rotation module
        if rotation == 1:
            #Rotational motion (forward) {PD control}
            min_tan = min_dist
            if nq.abs(tan_err) > min_tan or pos_err[0] < 0:
                rot_c[2] = (5*K_p*tan_err+ K_d*pos_err_d[3])
                #almost pure rotation
                if nq.abs(tan_err) > min_tan or pos_err[1] > min_dist:
                    vel_c[0] = 0; vel_c[1] = vel_c[1]/down_scale
            #saturation
            if nq.abs(rot_c[2]) > max_vel:
                rot_c[2] = max_vel*rot_c[2]/nq.abs(rot_c[2])

        #control on x velocity 
        # if it is not on the line and not close the the point --> vel_x scaled down
        if nq.abs(pos_err[1]) > min_dist and nq.abs(pos_err[0]) > 10*min_dist:
            vel_c[0] = vel_c[0]/down_scale
           
        #Velocity publisher (no tilt)
        if vel_c[0] > 0:
            temp_vel1 = front_back_trans('front', nq.abs(vel_c[0]))
        else:
            temp_vel1 = front_back_trans('back', nq.abs(vel_c[0]))  
        if vel_c[1] > 0:
            temp_vel2 = left_right_trans('left', nq.abs(vel_c[1]))
        else:
            temp_vel2 = left_right_trans('right', nq.abs(vel_c[1]))
        if vel_c[2] > 0:
            temp_vel3 = up_down_trans('up', nq.abs(vel_c[2]))
        else:
            temp_vel3 = up_down_trans('down', nq.abs(vel_c[2]))
        if rot_c[2] > 0:
            temp_vel4 = left_right_turn('left', nq.abs(rot_c[2]))
        else:
            temp_vel4 = left_right_turn('right', nq.abs(rot_c[2]))

        temp_tot = [0,0,0,0,0]; max_temp_tot_vel = max_vel; overpassed = 0
        for h in range(0,5):
            temp_tot[h] = temp_vel1[h] + temp_vel2[h] + temp_vel3[h] + temp_vel4[h]
            #saturation
            if temp_tot[h] > max_temp_tot_vel:
                max_temp_tot_vel = temp_tot[h]
                scale = max_vel/max_temp_tot_vel
                overpassed = 1
        #scale down on the saturation
        if overpassed == 1:
            for h in range(0,5):
                 temp_tot[h] = temp_tot[h]*scale         
        msg.data = [temp_tot[0],temp_tot[1],temp_tot[2],temp_tot[3],temp_tot[4]]
        #print msg.data
        pub.publish(msg)


    #If the point is reached, go to the following or conclude the protocol
    if nq.abs(vTp[0]) < min_dist and nq.abs(vTp[1]) < min_dist and nq.abs(vTp[2]) < min_dist:
        if j == size-1:
            exit = 1
        else:
            j = j+1

    rospy.sleep(time_step)
# stop benchmark -stop service-
stop()

print 'Do not forget to run "genResults.sh" to generate the PDF graphs'
print ' '

