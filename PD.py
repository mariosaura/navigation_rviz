# https://github.com/ros-drivers/gps_umd/blob/master/gps_common/src/utm_odometry_node.cpp

import rospy
import utm
import message_filters

from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg  import Odometry
from sensor_msgs.msg import Imu

from math import pow
import math
import numpy as np

import time
from tf.transformations import euler_from_quaternion, quaternion_from_euler


from itertools import count
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.path import Path
import matplotlib.patches as patches 

plt.style.use('fivethirtyeight')


class get_pose_current():
    def __init__(self):
        self.latitud=0.0
        self.longitud=0.0

    def listener(self):
        self.sub=rospy.Subscriber("/gnss/fix" , NavSatFix, self.callback_pos)

    def callback_pos(self, data):
        self.latitud = data.latitude
        self.longitud = data.longitude
    

def talker(x, z):

    move_cmd = Twist()
    move_cmd.linear.x = x
    move_cmd.angular.z = z

    rate = rospy.Rate(100)
    pub.publish(move_cmd)


def dibujar():
        plt.pause(0.1)


if __name__ == '__main__':

    rospy.init_node('gps_utm',anonymous=True)


    #Ganancia
    ke_a = 0.33
    kd_a = 0.05
    ke_v = 0.25
    kd_v = 0.05

    #Iniciales
    x_ini,y_ini,Num1,Letra1=utm.from_latlon(38.2759133, -0.6853013)    #Punto de referencia    38.2751274,-0.6856421 
    i,alpha_prev,d_prev = 0 , 0.0 , 0.0
    x_ant,y_ant,v_ant = -0.01 , -0.01 ,0
    pi = math.pi

    
    #Config
    pose = get_pose_current()
    pose.listener()
    pub = rospy.Publisher('/husky_velocity_controller/cmd_vel', Twist , queue_size=1)
    speed = Twist()
    r = rospy.Rate(10)
  

    #FICHERO    
    fichero = "path_planning/csv_routes/path.csv"

    x2 = []
    y2 = []
    
    coordenadas = pd.read_csv(fichero,delimiter=",")        #leo el csv
    coordenadas = np.array(coordenadas)                     #lo paso a array para quedarme con solo los numer
    print(coordenadas)


    while i < len(coordenadas):
        pointx,pointy,Num2,Letra2 = utm.from_latlon(coordenadas[i,1],coordenadas[i,0])
        pointx,pointy = pointx - x_ini , pointy - y_ini            #Me hago mi sistema local
        x2.append(float(pointx))
        y2.append(float(pointy))
        i = i+1

    #DIBUJO

    # Crear una figura y un eje
    fig, ax = plt.subplots()
    ax.set_title('Mapa')
    ax.set_xlabel('Este [m]')
    ax.set_ylabel('Norte [m]')
    # Dibujo las coordenadas fijas
    ax.plot(x2, y2, 'x--', color='black', label='Coordenadas fijas', lw=1) 
    # ax.plot(x2, y2, 'bo', label='Coordenadas fijas') 
        # Inicializar tus datos
    x_data = []
    y_data = []
        # Configurar el gráfico
    line, = ax.plot(x_data, y_data)

  
    i=0


    while not rospy.is_shutdown():

        x_i,y_i,Num,Letra = utm.from_latlon(pose.latitud, pose.longitud)
        x,y  = x_i - x_ini , y_i - y_ini            #Me hago mi sistema local 


        #DISTANCIA DESPLAZADA EN CADA ITERACION
        d_desplazado = math.sqrt(pow(x-x_ant,2)+pow(y-y_ant,2))
        print("dist desplazado : ",d_desplazado)
       
        o_des = math.atan2((y2[i]-y_ant),(x2[i]-x_ant))  #*(180/math.pi)


        #SI no me he desplazado, mi orientacion actual no la voy a calcular bien --> me quedo con la que tenia     
        if((abs(v_ant)>0.0005 and d_desplazado > 0.1) or (v_ant==0) ):
            o_act = math.atan2((y-y_ant),(x-x_ant))   
           
        
        print('x =',x,'            y = ',y)
        print('x2 =',x2[i],'            y2 = ',y2[i])


        #CALCULO DE ALPHA
        o_act , o_des = (o_act + 2*pi)%(2*pi) , (o_des + 2*pi)%(2*pi)
        alpha = (((o_des - o_act)+pi) % (2*pi) ) - pi
        deriv_a = alpha-alpha_prev

        w = ke_a*alpha + kd_a*deriv_a
        print("o_des:",o_des,"o_act: ",o_act,"alpha: ",alpha)

        #PD VELOCIDAD
        d = math.sqrt(pow(x2[i]-x,2)+pow(y2[i]-y,2))
        deriv_v = d - d_prev
        v = ke_a*(d) + kd_v*(deriv_v)
        if (abs(v) > 0.6): v = 0.6                  #v max
        if (abs(v) < 0.15): v=0.15                  #v min
        if (abs(w) > 0.3 and v == 0.6): v = 0.3     #v max para giros grandes
        

        speed.linear.x = v
        speed.angular.z = w
        pub.publish(speed)
        
        #DIBUJO
        dibujar()


        #ME GUARDO ERRORES ANTERIORES
        alpha_prev, d_prev, v_ant = alpha , d , v
        if(d_desplazado > 0.1 ):  x_ant,y_ant = x,y
       



        # HE LLEGADO ? ?
        if (abs(x2[i]-x)) < 0.6 and (abs(y2[i]-y)) < 0.6 : 
            i = i+1 
            print('\nPUNTO ',i,' ALCANZADO\n')

        
        r.sleep()

    ax.legend()
    plt.show()
