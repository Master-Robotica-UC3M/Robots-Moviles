#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid
from random import randint
from visualization_msgs.msg import Marker
from dynamic_reconfigure.client import Client

import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import math
import random

from sklearn.cluster import KMeans

visited = []
map_data = OccupancyGrid()

localmin_counter = 0

tolerance = 0.5
enlargement = 4

next = None
prev = None

x1_cell = None
y1_cell = None

start_time = rospy.Time(0)
end_time = rospy.Time(0)

represent_graphs = True # Falso para ocultar ventanas emergentes

def clustering(gap):
    global x1, y1
    X = []

    for g in gap:
        X.append(g)

    X = np.array(X).copy()

    # Número máximo de clusters que queremos probar
    max_k = 10

    if max_k > len(gap):
        max_k = len(gap)

    # Inicializar variables para almacenar los resultados
    sum_variances = []

    # Iterar sobre diferentes valores de k
    for k in range(1, max_k + 1):
        # Crear un objeto de KMeans
        kmeans = KMeans(n_clusters=k, random_state=42)

        # Realizar el clustering
        kmeans.fit(X)

        # Calcular la suma de las varianzas de los clusters
        sum_of_cluster_variances = sum(np.min(np.sum((X - kmeans.cluster_centers_[kmeans.labels_])**2, axis=1)) for k in range(kmeans.n_clusters))
        
        # Guardar la suma de varianzas para este valor de k
        sum_variances.append(sum_of_cluster_variances)

    # Encontrar el valor de k que minimiza la suma de varianzas
    best_k = np.argmin(sum_variances) + 1  # Sumamos 1 porque los índices comienzan en 0

    # Crear un objeto de KMeans con el mejor valor de k
    best_kmeans = KMeans(n_clusters=best_k, random_state=42)

    # Realizar el clustering con el mejor valor de k
    best_kmeans.fit(X)

    # Obtener las etiquetas de los clusters y los centroides
    labels = best_kmeans.labels_
    centroids = best_kmeans.cluster_centers_

    # elements_per_cluster = [np.sum(labels==i) for i in range(best_k)]
    # biggest_cluster = np.argmax(elements_per_cluster)

    
    # else:
    elements_per_cluster = [np.sum(labels==i) for i in range(best_k)]
    closest_cluster = np.argmax(elements_per_cluster)
    
    next_ok = False

    while not next_ok:
        print(range(len(labels)))
        index = random.choice(range(len(labels)))

        if labels[index] == closest_cluster:
            print("Escogido: ", labels[index])
            print(index)
            next = X[index]
            print(next)
            next_ok = True

    
    print("indice",index)
    print(X[index])
    # next = X[index]


    # Visualizar los clusters
    
    if represent_graphs:
        plt.figure(figsize=(6,8))
        plt.scatter(X[:, 1], X[:, 0], c=labels, cmap='viridis')
        if x1_cell is not None:
            plt.scatter(y1_cell,x1_cell,marker='x',color='blue') 
        plt.scatter(centroids[:, 1], centroids[:, 0], marker='x', color='red', s=200, label='Centroids')
        plt.title('Clustering con K-Means (Mejor k)')
        plt.xlabel('Característica 1')
        plt.ylabel('Característica 2')
        plt.legend()
        plt.show()
    

    return (next[0], next[1])

def feedback_callback(feedback):
    global next
    global goal_client
    global map_data
    global x1, y1
    global x1_cell, y1_cell

    x1 = feedback.base_position.pose.position.x
    y1 = feedback.base_position.pose.position.y

    y1_cell = (x1- map_data.info.origin.position.x)/map_data.info.resolution # OJO cambiado
    x1_cell = (y1- map_data.info.origin.position.y)/map_data.info.resolution # OJO cambiado

    x2 = next[1]*map_data.info.resolution + map_data.info.origin.position.x
    y2 = next[0]*map_data.info.resolution + map_data.info.origin.position.y

    dist = math.sqrt((x1-x2)**2 + (y1-y2)**2)

    print('x1: ',x1)
    print('y1: ',y1)
    print(dist)
    
    if dist < tolerance:
        print('Estás muy cerca')

        goal_client.cancel_goal()



def enlarge_obstacles(map, d=1):
    # Función para engrosar los bordes

    rows = map.shape[0]
    cols = map.shape[1]

    enlarged_map = map.copy()

    for i in range(rows):
            for j in range(cols):

                if map[i][j] == 100:
                    # Para cada celda ocupada
                    
                    for di in range(-d, d+1):
                        for dj in range(-d, d+1):
                            ni = i + di
                            nj = j + dj
                    
                            if (0 <= ni < rows) and (0 <= nj < cols):
                                
                                enlarged_map[ni][nj] = 100
    
    if represent_graphs:
        plt.imshow(enlarged_map, cmap = 'gray')
        plt.show()

    return enlarged_map

n_recv = 0
def map_callback(map_msg):
    print('Mapa recibido')
    global map_data
    global n_recv
    map_data = map_msg
    

def select_and_publish_goal(event):
    print('MoveBase callback')
    global map_data
    global visited
    global prev
    global next
    global localmin_counter
    global goal_client
    global enlargement
    global n_recv
    
    n_recv += 1

    if (n_recv <= 4):
        print("Ignorando mapa recibido")
        return
    else:
        n_recv = 0

    if map_data is not None:
        print('Eligiendo destino...')
        width = map_data.info.width
        height = map_data.info.height

        print(width)
        print(height)


        A = np.reshape(map_data.data, (width, height))
        
        print(A)
        print(A.shape)
        print(width, height)
        
        rows = A.shape[0]
        cols = A.shape[1]

        
        A = enlarge_obstacles(A,enlargement).copy()

        gap = []

        # Buscar las celdas libres vecinas a una celda con valor desconocido
        for i in range(rows):
            for j in range(cols):

                if A[i][j].astype(int) == 0 and (i,j) not in visited:
                    # Para cada celda no ocupada
                    
                    neighbours = [(i-1,j), (i+1,j), (i,j-1), (i,j+1)]
                    for r, c in neighbours:
                        if (0 <= r and r < rows) and (0 <= c and c < cols):
                            if (A[r][c].astype(int) == -1): # En caso de adyacente a celda inexplorada
                                gap.append((i,j))

        # Si no hay mas celdillas, exploracion finalizada.
        if len(gap) == 0:
            print('¡EXPLORACIÓN COMPLETA!')
            exit(0)
        
        else:
            rospy.sleep(3)

            next = clustering(gap)

            near = False

            if prev is not None:

                dist = np.array(next) - prev
                length = np.linalg.norm(dist)

                if length < 10:
                    near = True
                    localmin_counter += 1
            
            if localmin_counter > 3:

                for i in range(rows):
                    for j in range(cols):
                        if A[i][j].astype(int) == 0:
                            next = (i,j)

                localmin_counter = 0

            prev = next

            if not near:
                
                visited.append(next)
                   
                print(next)
                
                print('Destino elegido. Navegando hasta el punto...')

                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = "map"
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.pose.position.x = (next[1] * map_data.info.resolution) + map_data.info.origin.position.x
                goal.target_pose.pose.position.y = (next[0] * map_data.info.resolution) + map_data.info.origin.position.y
                goal.target_pose.pose.orientation.w = 1.0

                print(goal.target_pose.pose.position.x, goal.target_pose.pose.position.y)
                

                goal_client.send_goal(goal, feedback_cb=feedback_callback)
                wait = goal_client.wait_for_result() # Esperar por la navegacion

                print('Punto alcanzado!')

            else:
                print('Skipping:', next)

    return map

if __name__ == '__main__':
    try:
        rospy.init_node('exploration', anonymous=True)
        map_data = None

        map_subscriber = rospy.Subscriber('/map', OccupancyGrid, map_callback, queue_size=1)
        marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size = 2)

        goal_client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        goal_client.wait_for_server()

        timer = rospy.Timer(rospy.Duration(1), select_and_publish_goal)
        
        start_time = rospy.Time().now() # Tomar tiempo inicial de ejecucion

        rate = rospy.Rate(1)
        # Mostrar el tiempo que llevae ejecutando el algoritmo.
        while not rospy.is_shutdown():
            rate.sleep()
            end_time = rospy.Time.now()
            elapsed_time = end_time - start_time
            
            print("Tiempo empleado en la exploración", elapsed_time.secs)

    except rospy.ROSInterruptException:
        rospy.loginfo("Exploration finished.")
        
