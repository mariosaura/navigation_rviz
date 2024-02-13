import rospy

from std_msgs.msg import String
#obtengo el tipo de mensaje q usa el topico 
from geometry_msgs.msg import PointStamped 
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
    while not rospy.is_shutdown():
        x = pose.x
        y = pose.y

        print(x)
        print(y)

        r.sleep()