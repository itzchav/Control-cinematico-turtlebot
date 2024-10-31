#!/usr/bin/python3

import rospy
import cv2
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import os
from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import sin, cos, pi, sqrt


import sys, select, os
if os.name == 'nt':
  import msvcrt
else:
  import tty, termios
x = []
y = []
tiempos = []
V=0
V_max = 0.22; W_max = 2.84; #Timer, initial time,W=0
T = 100.0


def control_velocidad(self):

    k=0.2; a=1; b =0.5
    h=0.05
    w = 2*pi/T; X0=0;Y0=0
    t=self.time
    
    Xhd=self.xd; Yhd=self.yd
    tetha=self.yaw
    print(t)
    xh = self.x+h*cos(tetha)
    yh = self.y+h*sin(tetha)
   
    #Xhdp=5
    #Yhdp=5
    ex = xh-Xhd;  ey = yh-Yhd
    #Ux = Xhdp-k*ex;  Uy =Yhdp-k*ey
    Ux = -k*ex;  Uy =-k*ey
    self.V= Ux*cos(tetha)+Uy*sin(tetha)
    self.W=-Ux*sin(tetha)/h+Uy*cos(tetha)/h
    
    
    #Evitar la saturacion en las velocidades 
    if (abs(self.V)>V_max):
        self.V = V_max*abs(self.V)/self.V
        print("Saturacion en V\t")
    if (abs(self.W)>W_max):
        self.W = W_max*abs(self.W)/self.W
        print("Saturacion en W\t")

class Nodo(object):
    def __init__(self):
        self.loop_rate = rospy.Rate(10)
        #inicializar
        self.x = None
        self.V=0
        self.W=0
        
        # Subscribirse
        odom_sub =rospy.Subscriber('/odom',Odometry, self.Callback)
        #Publicar
        self.pub = rospy.Publisher('/cmd_vel_mux/input/navi',Twist,queue_size=10)
        self.pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)
        #Inicializar Tiempo
        self.t0 = rospy.Time.now().to_sec()


    def Callback(self,msg):
        #Posicion
        self.x=msg.pose.pose.position.x
        self.y=msg.pose.pose.position.y
        print("X={0} Y={1}".format(self.x,self.y))
        #Inicializa velocidad
        self.velocidad=Twist()
        #contador para obtener el tiempo inicial
        if (self.i==0):
            print("if")
            self.t0=rospy.Time.now().to_sec()
            self.i=1
        #Orientacion
        quater = msg.pose.pose.orientation
        quater_list = [quater.x, quater.y, quater.z, quater.w]
        (roll, pitch, self.yaw) = euler_from_quaternion(quater_list)
        #Tiempo
        self.time = rospy.Time.now().to_sec()-self.t0
        #Funcion de control
        control_velocidad(self)
        #Fatrac(self)
        

    def start(self,arg1,arg2):
        rospy.loginfo("Comenzando codigo")
        self.i=0
        self.xd=float(arg1)
        self.yd=float(arg2)
        print(self.xd,self.yd)
        
        while not rospy.is_shutdown():   
            if self.x is not None:    
                #Asigna la velocidad y el angulo del robot
                self.velocidad.linear.x = self.V 
                self.velocidad.angular.z = self.W
                #Se publica en el nodo
                self.pub.publish(self.velocidad)
            self.loop_rate.sleep()

if __name__ == '__main__':
    rospy.init_node("MOVIMIENTO", anonymous=True)  
    
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    try:
        my_node = Nodo()
        if len(sys.argv) < 3:
            print("usage: my_node.py arg1 arg2")
        else:
            #my_node(sys.argv[1], sys.argv[2])
            my_node.start(sys.argv[1], sys.argv[2])
    except rospy.ROSInterruptException:
        pass
    
    
