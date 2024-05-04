#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid
from random import randint
from visualization_msgs.msg import Marker
from dynamic_reconfigure.client import Client


def map_callback(map_msg):
    print('Mapa recibido')
    global map_data
    
    map_data = map_msg

    free_space = 0
    obstacle_space = 0
    for value in map_data.data:
        if (value == 0):
            free_space += 1
        if (value == 100):
            obstacle_space += 1

    print("Espacio libre:", free_space, "Espacio ocupado:", obstacle_space, "Espacio total conocido:", free_space + obstacle_space)

if __name__ == '__main__':
    try:
        rospy.init_node('exploration', anonymous=True)
        map_data = None

        map_subscriber = rospy.Subscriber('/map', OccupancyGrid, map_callback, queue_size=1)
        
        # rate = rospy.Rate(1)

        while not rospy.is_shutdown():
            #rate.sleep()
            rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("Exploration finished.")
        
