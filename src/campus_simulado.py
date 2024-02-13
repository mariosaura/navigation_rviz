# https://github.com/ros-drivers/gps_umd/blob/master/gps_common/src/utm_odometry_node.cpp

import rospy
import utm
import message_filters

from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg  import Odometry
from sensor_msgs.msg import Imu

import numpy as np
from math import pow
import math
import time

#Transformada
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf
import roslib
#roslib.load_manifest('learning_tf')

def husky_pose(x,y,o_act):
    br = tf.TransformBroadcaster()
    br.sendTransform((0, 0, 0),tf.transformations.quaternion_from_euler(0, 0, o_act),rospy.Time.now(),"odom","map")
    #br.sendTransform((x, y, 0),tf.transformations.quaternion_from_euler(0, 0, o_act),rospy.Time.now(),"odom","map")


waypoints = [[1 , 0.4],[2.766 , 10.53 ],[-0.75 , 16.84],[-2.92 , 20.38],[-5.36 , 24.16],[-3.57 , 30],
             [-4.53 , 36.45],[-6.40 , 39.9],[-8.76 , 42.4],[-17.67 , 48.04],[-26.18 , 47],[-27.6 , 45.08], 
             [-28.11 , 42.76],  [-29.67,39.26] , [-30.59,38.54] , [-32.57 , 36.99],[-34.53,35.48] , [-37.68,34.3]  ]

#Ganancia
ke_a = 0.2
kd_a = 0.1
ke_v = 0.01
kd_v = 0.01


a = 0
alpha_prev = 0.0
d_prev = 0.0

pi = math.pi
x_ini,y_ini,Num1,Letra1=utm.from_latlon(38.274756, -0.686131)   #Mismos parametros q haya en husky.urdf.xacro y rviz_satellite/launch/demo.gps
x_ant = 0.01 
y_ant = 0.01
v_ant = 0

def talker(x, z):

    move_cmd = Twist()
    move_cmd.linear.x = x
    move_cmd.angular.z = z

    rate = rospy.Rate(10)
    pub.publish(move_cmd)
    


def callback(gps):
    
    global a,x_ant,y_ant,alpha_prev,d_prev,v_ant
    print("latitud: ",gps.latitude,"  longi: ",gps.longitude)
    x_i,y_i,Num,Letra = utm.from_latlon(gps.latitude, gps.longitude)
    x,y  = x_i - x_ini , y_i - y_ini            #Me hago mi sistema local 

    x2,y2= waypoints[a][0] ,waypoints[a][1]


    d = math.sqrt(pow(x2-x,2)+pow(y2-y,2))
    o_des = math.atan2((y2-y),(x2-x))  #*(180/math.pi)


    if(abs(v_ant)>0.0005 or v_ant==0): o_act = math.atan2((y-y_ant),(x-x_ant))   #SI no me he desplazado, mi orientacion actual no la voy a calcular bien --> me quedo con la que tenia     
    
    x_ant,y_ant = x,y
    print('x =',x,'            y = ',y)
    print('x2 =',x2,'            y2 = ',y2)


    # PD ANGULAR

    #CALCULO DE ALPHA
    o_act , o_des = (o_act +2*pi)%(2*pi) , (o_des +2*pi)%(2*pi)

    husky_pose(x,y,o_act)
    
    alpha = (((o_des - o_act)+pi) % (2*pi) ) - pi

    deriv_a = alpha-alpha_prev

    w = ke_a*alpha + kd_a*deriv_a
    # print('alpha:',alpha,' o_des',o_des,' o_act',o_act,' w',w)
    
  

    #PD VELOCIDAD
    deriv_v = d - d_prev
    v = ke_a*(d) + kd_v*deriv_v
    if (abs(v) > 0.3): v = 0.3
    if (abs(w) > 0.3 and v == 0.3): v = 0.25
    
    # if(o_des-o_act>2): v = 0.1
    # print('v:',v)

    talker(v,w)
    
    
    #ME GUARDO ERRORES ANTERIORES
    alpha_prev, d_prev, v_ant = alpha , d , v



    # HE LLEGADO ? ?
    if (abs(x2-x)) < 0.4 and (abs(y2-y)) < 0.4 : 
        a = a+1
        print('\nPUNTO ',a,' ALCANZADO\n')





if __name__ == '__main__':
    try:
        rospy.init_node('gps_utm',anonymous=True)
        rate = rospy.Rate(1)        
        pub = rospy.Publisher('/husky_velocity_controller/cmd_vel', Twist , queue_size=5)
        sub = rospy.Subscriber('/navsat/fix' , NavSatFix , callback)      

        while not rospy.is_shutdown():
            rate.sleep()
        
    except rospy.ROSInterruptException:
        pass



