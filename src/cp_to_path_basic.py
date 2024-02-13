import rospy
#importo la funcion del codigo de sergio
from path_planning.main import main_funtion
from std_msgs.msg import String
#obtengo el tipo de mensaje q usa el topico 
from geometry_msgs.msg import PointStamped 
#para ejecutar scribs
import os

import utm

import time
import tf
import random
import numpy as np


class get_pose_current():
    def __init__(self):
        self.x=0.0
        self.y=0.0

    def listener(self):
        self.sub=rospy.Subscriber("/clicked_point" , PointStamped, self.callback_pos)

    def callback_pos(self,data):
        #el mensaje tiene un header y un point. dentro del point tengo las variables http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PointStamped.html
        self.x = data.point.x
        self.y = data.point.y



if __name__ == '__main__':

    rospy.init_node('clicked_point_subscriber',anonymous=True)

    pose = get_pose_current()
    pose.listener()

    r = rospy.Rate(10)


    #Iniciales
    x_ini,y_ini,Num,Letra=utm.from_latlon(38.274756, -0.686131)  
    


    while not rospy.is_shutdown():
        x = pose.x
        y = pose.y

        

        if(x!=0):
            utm_x = x_ini + x  
            utm_y = y_ini + y 

            print("x_ini,y_ini: ",x_ini, y_ini)
            print("x,y: ",x, y)
            print(utm_x , utm_y)
            
            lat,lon = utm.to_latlon(utm_x, utm_y  , Num,Letra)
            print("lat,lon:",lat, lon)
            main_funtion(38.274756, -0.686131,lat,lon)
            time.sleep(60)

        
        r.sleep()   
    # 1 paso las coordenadas de ref de rviz_satellite a utm
    # 2 les sumo el desplazamiento en x e y
    # 3 las vuelvo a pasar a lat, lon
    # 4 se las mando al codigo de sergio para q haga el camino