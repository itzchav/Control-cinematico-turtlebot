#!/usr/bin/python3

import rospy
import cv2
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import os
import pandas as pd


from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import sin, cos, pi


import sys, select, os
if os.name == 'nt':
  import msvcrt
else:
  import tty, termios

V=0
V_max = 0.22; W_max = 2.84; #Timer, initial time,W=0
T = 100.0
data={}

#Funcion para el control de la velocidad lineal y angular
def control_velocidad(self):
##Parametros para el control
    k=1.2; a=1; b =0
    t=self.time
    t=0.2*t
    h=0.05
    
    self.delta_t=self.ta-t

    Xhd = a*cos(t)/(2+pow(sin(t),2))
    Yhd=  (a*sin(t)*cos(t))/(2+pow(sin(t),2))+b 

    #Xhd=1; Yhd=1
    tetha=self.yaw

    xh = self.x+h*cos(tetha)
    yh = self.y+h*sin(tetha)
    #Regulacion
    Xhdp=0
    Yhdp=0
    #Seguimiento
    #Xhdp=(self.Xhda-Xhd)/t
    #Yhdp=(self.Yhda-Yhd)/t
    
    #Xhdp=a*(-sin(t)*(2+pow(sin(t),2))-(sin(2*t)*cos(t)))/(pow((2+pow(sin(t),2)),2))
    #Yhdp=a*(cos(2*t)*(2+pow(sin(t),2))-sin(2*t)*sin(t)*cos(t))/(pow((2+pow(sin(t),2)),2))
    
    #self.pos_x.append(Xhd) 
    #self.pos_y.append(Yhd)
    self.data_tiempos.append(t)
    #print(Xhd,Yhd)

    self.pos_x=xh
    self.pos_y=yh
    
    ex = xh-Xhd;  ey = yh-Yhd
    
    Ux = Xhdp-k*ex;  Uy =Yhdp-k*ey
    #Ux =-k*ex;  Uy =-k*ey
    self.V= Ux*cos(tetha)+Uy*sin(tetha)
    self.W=-Ux*sin(tetha)/h+Uy*cos(tetha)/h
    
    ta=t
    Xhda=Xhd
    Yhda=Yhd
    #Evitar la saturacion en las velocidades 
    if (abs(self.V)>V_max):
        self.V = V_max*abs(self.V)/self.V
        #print("Saturacion en V\t")
    if (abs(self.W)>W_max):
        self.W = W_max*abs(self.W)/self.W
        #print("Saturacion en W\t")

class Nodo(object):
    def __init__(self):
        self.loop_rate = rospy.Rate(10)
        #inicializar
        self.x = None
        self.V=0
        self.W=0
        self.i=0
        self.Xhda=0
        self.Yhda=0
        self.ta=0
        self.delta_t=0
        self.pos_x = [0]
        self.pos_y = [0]
        self.data_tiempos = [0]
        self.pos_x = 0
        self.pos_y = 0
        #self.data_tiempos = 0
        
        #Inicializar Tiempo
        self.t1 = rospy.Time.now().to_sec()  
        # Subscribirse    
        odom_sub =rospy.Subscriber('/odom',Odometry, self.Callback)
        #Publicar
        self.pub_pos = rospy.Publisher('posicion_deseada', Twist, queue_size=10)
        #self.pub = rospy.Publisher('/cmd_vel_mux/input/navi',Twist,queue_size=10)
        self.pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)
        


    def Callback(self,msg):
        #Posicion
        self.x=msg.pose.pose.position.x
        self.y=msg.pose.pose.position.y
        print("X={0} Y={1}".format(self.x,self.y))

        self.velocidad=Twist()
        self.pos_deseada=Twist()
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

    def start(self):
        rospy.loginfo("Comenzando codigo")
        self.i=0

        now = rospy.get_rostime()

        while not rospy.is_shutdown():
            #data = {'x': self.pos_x, 'y': self.pos_y, 'tiempo': self.data_tiempos}
            #df = pd.DataFrame(data)
            #df.to_csv('pos_des.csv', index=False)
               
            if self.x is not None:    
                #Asigna la velocidad y el angulo del robot
                self.velocidad.linear.x = self.V 
                self.velocidad.angular.z = self.W
                self.pos_deseada.linear.x = self.pos_x 
                self.pos_deseada.linear.y = self.pos_y
                
                #Se publica en el nodo
                self.pub.publish(self.velocidad)
                #Se publica en el nodo
                self.pub_pos.publish(self.pos_deseada)
                
            self.loop_rate.sleep()
        
                


if __name__ == '__main__':
    rospy.init_node("MOVIMIENTO", anonymous=True)
    time = rospy.get_rostime()
    print("Holiii")
    print(time)
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    try:
        my_node = Nodo()
        my_node.start()
        
        
        
            
    except rospy.ROSInterruptException:
        pass
    
    
