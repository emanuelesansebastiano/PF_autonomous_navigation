#!/usr/bin/env python

#Author: Emanuele Sansebastiano
#Date: January 2017
#Mail: emanuele.sansebastiano@outlook.com

from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image
import rospy
import numpy as nq
import cv2
from cv_bridge import CvBridge, CvBridgeError

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

msg_empty = """



"""

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

class imageGrabber:

    ##Init function, create subscriber and required vars.  
    def __init__(self):
      image_sub = rospy.Subscriber("/g500/camera1",Image,self.image_callback)
      self.bridge = CvBridge()
      self.height=-1
      self.width=-1
      self.channels=-1

    ##Image received -> process the image
    def image_callback(self,data):
        global x_y_dis,det
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        except CvBridgeError as e:
            print(e)

        self.height, self.width, self.channels = hsv.shape
        h = self.height; w = self.width; c = self.channels  
        #print h, w, cv_image_disp

        hsv_disp = hsv
        # keeping the green layer of the HVS
        hsv_disp[:,:,0] = 0; hsv_disp[:,:,2] = 0 

        canny = cv2.Canny(hsv_disp, 50,150)

        #circles
        f_circ = 60
        gap = 40
        det = nq.zeros((2,1))
        x_y_dis = nq.zeros((2,3))
        for i in range(0,2):
            r = f_circ+i*gap
            z = 0
            pxy = nq.zeros((20,2))
            for j in range(1,(r+1)):
                x = j
                y = nq.sqrt((r*r)-(x*x))
                y = nq.round(h/2-y, 0)#; y = h/2
                p1 = (nq.round(w/2-x,0),y)#; print p1[0],p1[1]
                p2 = (nq.round(w/2+x,0),y)#; print p2[0],p2[1]
                
                #blue
                if i == 0: 
                    hsv_disp[p1[1]-1,p1[0]-1,1] = 0
                    hsv_disp[p2[1]-1,p2[0]-1,1] = 0
                    hsv_disp[p1[1]-1,p1[0]-1,2] = 255
                    hsv_disp[p2[1]-1,p2[0]-1,2] = 255
                #red
                else:
                    hsv_disp[p1[1]-1,p1[0]-1,1] = 0
                    hsv_disp[p2[1]-1,p2[0]-1,1] = 0
                    hsv_disp[p1[1]-1,p1[0]-1,0] = 255
                    hsv_disp[p2[1]-1,p2[0]-1,0] = 255
               
                #intersection with pipe
                if canny[p1[1]-1,p1[0]-1] != 0:
                    pxy[z,0] = p1[0]-1; pxy[z,1] = p1[1]-1
                    z = z +1
                if canny[p2[1]-1,p2[0]-1] != 0:
                    pxy[z,0] = p2[0]-1; pxy[z,1] = p2[1]-1
                    z = z +1
        
            #interpolation of the points found by the two arcs
            #(h --> x; w--> y)
            temp1 = 0; temp2 = 0
            #to avoid 'num/0'
            if z == 0: z = 1
            # compute the distance between the points of the arcs 
            if z == 2:
                dist = nq.sqrt((pxy[0,0]-pxy[1,0])*(pxy[0,0]-pxy[1,0])+(pxy[0,1]-pxy[1,1])*(pxy[0,1]-pxy[1,1]))
            else:
                dist = 0
            #middle point of the intersections found
            for j in range(0,z):
                temp1 = temp1 + pxy[j,1]
                temp2 = temp2 + pxy[j,0]
            x_y_dis[i,0] = nq.round(temp1/z,0)
            x_y_dis[i,1] = nq.round(temp2/z,0)
            x_y_dis[i,2] = dist
            det[i,0] = z 

            # reference point color
            l = 5; lm = 3
            for i in range(0,l):
                for j in range(0,l):
                    #middle points
                    hsv_disp[x_y_dis[0,0]+(lm-i), x_y_dis[0,1]+(lm-j),0] = 255
                    hsv_disp[x_y_dis[1,0]+(lm-i), x_y_dis[1,1]+(lm-j),0] = 255
                    hsv_disp[x_y_dis[0,0]+(lm-i), x_y_dis[0,1]+(lm-j),1] = 255
                    hsv_disp[x_y_dis[1,0]+(lm-i), x_y_dis[1,1]+(lm-j),1] = 255
                    hsv_disp[x_y_dis[0,0]+(lm-i), x_y_dis[0,1]+(lm-j),2] = 255
                    hsv_disp[x_y_dis[1,0]+(lm-i), x_y_dis[1,1]+(lm-j),2] = 255
                    # vehicle points
                    hsv_disp[h/2-gap/2+(lm-i), w/2+(lm-j),1] = 0
                    hsv_disp[h/2-gap/2+(lm-i), w/2+(lm-j),1] = 0
                    hsv_disp[h/2+(lm-i), w/2+(lm-j),1] = 0
                    hsv_disp[h/2+(lm-i), w/2+(lm-j),1] = 0
        #print x_y_dis

        canny = cv2.Canny(hsv_disp, 50,150)
        #cv2.imshow("Image window", canny)
        cv2.imshow("Image window", hsv_disp)
        cv2.waitKey(3)    

    ##Return size of the image
    def getSize(self):
        return self.width,self.height


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

# Control ------------------------------------------

##initialize the node
rospy.init_node('pipevision')

##wait for benchmark init service
rospy.wait_for_service('/startBench')
start=rospy.ServiceProxy('/startBench', Empty)

##wait for benchmark stop service
rospy.wait_for_service('/stopBench')
stop=rospy.ServiceProxy('/stopBench', Empty)

#Create the imageGrabber
IG=imageGrabber()
rospy.sleep(1)

global x_y_dis,det
err = nq.zeros((4,2))
start()

p = x_y_dis
exit_c = 0; treshold_exit = 2/time_step
while not rospy.is_shutdown():
    #filter correct input
    for i in range(0,2):
        if det[i,0] == 2:
            exit_c = 0
            p[i,0] = x_y_dis[i,0]; p[i,1] = x_y_dis[i,1]; p[i,2] = x_y_dis[i,2]
        elif det[i,0] < 2:
            exit_c = exit_c +1

    if exit_c >= treshold_exit:
        #end the big while loop
        break     

    #get width x height of the last received image
    w,h = IG.getSize()
    
    #position / orientation:
    #vehicle
    x_v = h/2
    y_v = w/2
    tan_v = 0
    #pipe
    # average
    x_p = (p[1,0]+p[0,0])/2
    y_p = (p[0,1]+p[1,1])/2
    z_p = (p[0,2]+p[1,2])/2
    # error between the two points
    x_err_p = -(p[1,0]-p[0,0])
    y_err_p = p[0,1]-p[1,1]
    z_err_p = p[0,2]-p[1,2]
    if x_err_p != 0:
        tan_p = (y_err_p)/(x_err_p)
    #depth
    z_ref = 33

    #errors
    x_err = -(x_p - x_v)
    y_err = (y_p - y_v)
    tan_err = -(tan_p - tan_v)  
    z_err = -(z_p - z_ref)
    if tan_err > 0.1:
       z_err = 0
    
    #scaling to adapt to an acceptable input
    K_s = 0.01
    pos_err = [K_s*x_err_p,K_s*y_err,K_s*z_err]
    vTp = pos_err
    print vTp, tan_err

    # Velocity input defintion
    if input_type == 0:
        #meta control
        K_init = 1
        vel_err = [K_init*vTp[0],K_init*vTp[1],K_init*vTp[2]]
        vel_des = vel_err

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
        vel_c = [K_p*vel_des[0],K_p*vel_des[1],K_p*vel_des[2]]
        rot_c = nq.zeros((3,1))
        print vel_err 

        #Saturation control
        if nq.abs(vel_c[0]) > max_vel:
            vel_c[0] = max_vel*nq.sign(vel_c[0])
        if nq.abs(vel_c[1]) > max_vel:
            vel_c[1] = max_vel*nq.sign(vel_c[1])
        if nq.abs(vel_c[2]) > max_vel:
            vel_c[2] = max_vel*nq.sign(vel_c[2])

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
            rot_c[2] = max_vel*nq.sign(rot_c[2])
        

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
        min_dist = 0.03 #[m] 
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
        #front velocity X - scaled down
        #Uncomment the following line if you use the turns scenario
        #vTp[0] = vTp[0]/down_scale
        pos_err = vTp
        err[:,0] = err[:,1]
        err[0:3,1] = nq.transpose(pos_err)
        err[3,1] = tan_err
        #error derivate
        pos_err_d = (err[:,1] - err[:,0])/time_step

        vel_c = [K_p*pos_err[0] + K_d*pos_err_d[0],K_p*pos_err[1] + K_d*pos_err_d[1],K_p*pos_err[2] + K_d*pos_err_d[2]]
        rot_c = nq.zeros((3,1))

        #Saturation control
        if nq.abs(vel_c[0]) > max_vel:
            vel_c[0] = max_vel*nq.sign(vel_c[0])
        if nq.abs(vel_c[1]) > max_vel:
            vel_c[1] = max_vel*nq.sign(vel_c[1])
        if nq.abs(vel_c[2]) > max_vel:
            vel_c[2] = max_vel*nq.sign(vel_c[2])

        #Proprtional translation motion along every direction (X-Y)
        if nq.abs(pos_err[0]) > nq.abs(pos_err[1]):
            vel_c[1] = vel_c[0]*pos_err[1]/pos_err[0]
            if nq.abs(pos_err[1]) < min_dist:
                vel_c[1] = 0
        else:
            vel_c[0] = vel_c[1]*pos_err[0]/pos_err[1]
            if nq.abs(pos_err[1]) < min_dist:
                vel_c[0] = 0

        rotation = 1
        if rotation == 1:
            #Rotational motion (forward) {PD control}
            min_tan = min_dist
            if nq.abs(tan_err) > min_tan or pos_err[0] < 0:
                rot_c[2] = (5*K_p*tan_err+ K_d*pos_err_d[3])
                #no pure rotation
                if nq.abs(tan_err) > min_tan or pos_err[1] > min_dist:
                    vel_c[0] = vel_c[0]/down_scale; vel_c[1] = vel_c[1]
            #saturation
            if nq.abs(rot_c[2]) > max_vel:
                rot_c[2] = max_vel*nq.sign(rot_c[2])

        #control on x velocity
        if nq.abs(pos_err[1]) > min_dist:
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
    
    rospy.sleep(time_step)
  
stop()

print 'Do not forget to run "genResults.sh" to generate the PDF graphs'
print ' '
