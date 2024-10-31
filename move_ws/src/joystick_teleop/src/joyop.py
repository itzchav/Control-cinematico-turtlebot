#!/usr/bin/python3

'''
ackermann_drive_joyop.py:
    A ros joystick teleoperation script for ackermann steering based robots
'''

__author__ = 'Filiberto Martinez'
__license__ = 'GPLv1'
__maintainer__ = 'Filiberto Martinez'
__email__ = 'filiberto.martinez133@gmail.com'
__instagram__='filibertomartinez'

#import queue
from tokenize import Double
import rospy
#from ackermann_msgs.msg import AckermannDrive
from sensor_msgs.msg import Joy
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from geometry_msgs.msg  import Twist
import sys

class AckermannDriveJoyop:

    def __init__(self, args):
        if len(args)==1 or len(args)==2:
            self.max_speed = 0.2*float(args[0])
            self.max_steering_angle = float(args[len(args)-1])
            vel_topic= 'set_velocity'
            angle_topic= 'set_angle'
        elif len(args) == 3:
            self.max_speed = 0.2*float(args[0])
            self.max_steering_angle = float(args[1])
            #cmd_topic = '/' + args[2]
            vel_topic= 'set_velocity'
            angle_topic= 'set_angle'
        else:
            self.max_speed = 0.5
            #self.max_steering_angle = 0.3967852340780831
            #self.max_steering_angle =180
            #self.min_steering_angle =0.41853095368099036
            #vel_topic= 'set_velocity'
            #angle_topic= 'set_angle'
            #angulo_pub='Angulo'
        self.vel_msg = Twist()
        self.speed = 0
        self.steering_angle = 0.01659725047647953
        self.joy_sub = rospy.Subscriber('/joy', Joy, self.joy_callback)
        self.velocity_publisher = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=10)
        #self.vel_pub=rospy.Publisher('/AutoNOMOS_mini/manual_control/speed',Int16,queue_size=10)
        #self.vel_pub=rospy.Publisher('/AutoModelMini/manual_control/speed',Int16,queue_size=10)
        #self.angle_pub=rospy.Publisher('/AutoNOMOS_mini/manual_control/steering',Int16,queue_size=10)
        #self.angle_pub=rospy.Publisher('/AutoModelMini/manual_control/steering',Int16,queue_size=10)
        #self.angulo_pub=rospy.Publisher('/angulo',Int16,queue_size=10)
        #self.loop_rate = rospy.Rate(20)
        rospy.Timer(rospy.Duration(1.0/30.0), self.pub_callback, oneshot=False)
        rospy.loginfo('ackermann_drive_joyop_node initialized')

    def joy_callback(self, joy_msg):
        #gamma_negativo=89
        self.speed = float(0.4*(joy_msg.axes[1] * (self.max_speed)))
        self.vel_msg.linear.x=self.speed
        self.steering_angle = int(joy_msg.axes[2])  

	# [2] control feo == [3] control xbox 
        #if(joy_msg.axes[3]>0):
        #    self.steering_angle = int(90+(joy_msg.axes[3] * gamma_positivo))
        #if(joy_msg.axes[3]<0):
        #    self.steering_angle = int(89+(joy_msg.axes[3] * gamma_negativo))
        if(joy_msg.axes[2]==0):
            self.steering_angle = 0
        self.vel_msg.angular.z=self.steering_angle      
        
    def pub_callback(self, event):
        self.velocity_publisher.publish(self.vel_msg)
        #self.angulo_pub.publish(self.steering_angle)
        #self.print_state()

    def print_state(self):
        sys.stderr.write('\x1b[2J\x1b[H')
        rospy.loginfo('\x1b[1M\r''\033[31;1m''Salida')
        rospy.loginfo('\x1b[1M\r'
                      '\033[34;1mSpeed: \033[32;1m%0.2f m/s, '
                      '\033[34;1mSteering Angle: \033[32;1m%0.2f rad\033[0m',
                      self.speed, self.steering_angle)

    def finalize(self):
        rospy.loginfo('Halting motors, aligning wheels and exiting...')
        #self.speed=0
        #self.steering_angle=0

        sys.exit()

if __name__ == '__main__':
    rospy.init_node('ackermann_drive_joyop_node')
    joyop = AckermannDriveJoyop(sys.argv[1:len(sys.argv)])
    rospy.spin()
