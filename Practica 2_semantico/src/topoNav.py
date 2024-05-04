#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseWithCovarianceStamped
from cv_bridge import CvBridge
from random import randint
from ultralytics import YOLO

def camera_callback(camera_msg):
    global camera_data
    camera_data = camera_msg

def pose_callback(pose_msg):
    global pose_data
    pose_data = pose_msg

if __name__ == '__main__':
    global camera_data, pose_data
    try:
        rospy.init_node('topoNav', anonymous=True)

        camera_subscriber = rospy.Subscriber('/camera/rgb/image_raw', Image, camera_callback)
        pose_subscriber = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, pose_callback)

        goal_client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        goal_client.wait_for_server()

        rate = rospy.Rate(1)
        bridge = CvBridge()
        model = YOLO("yolov8n.pt")

        #Cargamos archivos con nodos y subnodos
        x_coords = []
        y_coords = []
        regions = []
        with open('subNodes.txt', 'r') as file:
            for line in file:
                values = line.strip().split(',')
                x_coords.append(float(values[0]))
                y_coords.append(float(values[1]))
                regions.append(values[2].strip())

        while not rospy.is_shutdown():

            #Elegimos un subnodo aleatorio de la lista
            randNode = randint(0, len(regions) - 1)

            print('Destino elegido!')

            #Enviamos la coordenada a move_base y esperamos a llegar
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = x_coords[randNode]
            goal.target_pose.pose.position.y = y_coords[randNode]
            goal.target_pose.pose.orientation.w = 1.0 #Modificar por la orientacion del punto correspondiente

            goal_client.send_goal(goal)
            wait = goal_client.wait_for_result()

            print('Analizando imagen...')

            #Analizamos la imagen con YOLO
            cv_image = bridge.imgmsg_to_cv2(camera_data, desired_encoding="bgr8")
            results = model.predict(source=cv_image, show=True, classes=[56, 57, 58, 59, 60, 61, 62, 68, 69, 70, 71, 72], conf=0.15, iou=0.9)

            print(results)

            rate.sleep()

    except rospy.ROSInterruptException:
        rospy.loginfo("Exploration finished.")
