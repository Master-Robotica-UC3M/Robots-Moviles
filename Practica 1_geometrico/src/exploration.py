# MASTER EN ROBOTICA Y AUTOMATIZACIÓN
# ROBOTS MOVILES
# NOELIA FERNANDEZ TALAVERA (100463135)
# GONZALO FELIPE ESPINOZA CHONLON (100383960)
# ANGELO VINCENZO BARRA (100520043)
#------------------------------------------------------------------------------
#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker

import matplotlib.pyplot as plt
import numpy as np
import math
import random

from sklearn.cluster import KMeans

visited = []
map_data = OccupancyGrid()

localmin_counter = 0
stucked_counter = 0

tolerance = 0.4 # Tolerance radius around goal in meters
enlargement = 6 # Enlargement of obstacles in cells

next = None # Next goal
prev = None # Previoius goal

x1_cell = None # Robot x position in cells
y1_cell = None # Robot y position in cells

start_time = rospy.Time(0)
end_time = rospy.Time(0)
finished = False

def clustering(gaps):
    # Function to group the obtained gaps in cluster with K-Means

    gaps = np.array(gaps).copy()

    # Maximum number of clusters
    max_k = 10

    if max_k > len(gaps): # Crop to number of gaps
        max_k = len(gaps)


    sum_variances = []

    # Iteration to get the optimal value of K
    for k in range(1, max_k + 1):
        
        kmeans = KMeans(n_clusters=k, random_state=42)
        kmeans.fit(gaps)

        # Get the total variance for each K
        sum_of_cluster_variances = sum(np.min(np.sum((gaps - kmeans.cluster_centers_[kmeans.labels_])**2, axis=1)) for k in range(kmeans.n_clusters))
        sum_variances.append(sum_of_cluster_variances)

    # Optimal K value has lowest total variance:
    best_k = np.argmin(sum_variances) + 1
    best_kmeans = KMeans(n_clusters=best_k, random_state=42)
    best_kmeans.fit(gaps) # Create clusters with optimal K value

    # Obtain cluster label and centroids for each gap
    labels = best_kmeans.labels_
    centroids = best_kmeans.cluster_centers_


    '''
    CODE TO GET CLOSEST CLUSTER:

    if x1_cell is not None and y1_cell is not None:
        closest_cluster = 0
        print('No se que es esto: ', centroids[closest_cluster][0])

        x2 = centroids[closest_cluster][0]
        y2 = centroids[closest_cluster][1]

        closest_distance = math.sqrt((x1_cell-x2)**2 + (y1_cell-y2)**2)

        print("Mejor valor de k:", best_k)
        print("Labels:", labels)
        print("Centroids:", centroids)
        print("Mi posición es: ", x1_cell,",", y1_cell)
        

        for i in range(len(centroids)):
            x2 = centroids[i][0]
            y2 = centroids[i][1]

            print("Centroide: ", x2, ',', y2)

            dist_to_centroid = math.sqrt((x1_cell-x2)**2 + (y1_cell-y2)**2)
            print("Distance: ", dist_to_centroid)

            if dist_to_centroid < closest_distance:
                closest_distance = dist_to_centroid
                # print(np.where(centroids)[0][0])
                closest_cluster = i

        print('Mas cercano: ', closest_cluster)
    '''
    
    # Next goal position comes from biggest cluster (least explored area)
    elements_per_cluster = [np.sum(labels==i) for i in range(best_k)]
    next_cluster = np.argmax(elements_per_cluster)
    
    next_ok = False

    while not next_ok:
        # Get random element from biggest cluster

        print(range(len(labels)))
        index = random.choice(range(len(labels)))

        if labels[index] == next_cluster:
            print("Escogido: ", labels[index])
            print(index)
            next = gaps[index]
            print(next)
            next_ok = True

    
    print("indice",index)
    print(gaps[index])

  
    # UNCOMMENT TO VISUALIZE CLUSTERS
    plt.figure(figsize=(6,8))
    plt.scatter(gaps[:, 1], gaps[:, 0], c=labels, cmap='viridis')
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

    # Robot Position in meters
    x1 = feedback.base_position.pose.position.x
    y1 = feedback.base_position.pose.position.y

    # Robot Position in cells
    y1_cell = (x1- map_data.info.origin.position.x)/map_data.info.resolution
    x1_cell = (y1- map_data.info.origin.position.y)/map_data.info.resolution

    # Goal position in meters
    x2 = next[1]*map_data.info.resolution + map_data.info.origin.position.x
    y2 = next[0]*map_data.info.resolution + map_data.info.origin.position.y

    dist = math.sqrt((x1-x2)**2 + (y1-y2)**2)

    #print('x1: ',x1)
    #print('y1: ',y1)
    #print(dist)
    
    # Cancel move_base if close enough to goal
    if dist < tolerance:
        print('Estás muy cerca')

        goal_client.cancel_goal()



def enlarge_obstacles(map, d=1):
    # Function to enlarge obstacles in 'd' number of cells

    rows = map.shape[0]
    cols = map.shape[1]

    enlarged_map = map.copy()

    for i in range(rows):
            for j in range(cols):

                if map[i][j] == 100:
                    # For each occupied room
                    
                    for di in range(-d, d+1):
                        for dj in range(-d, d+1):
                            ni = i + di
                            nj = j + dj
                    
                            if (0 <= ni < rows) and (0 <= nj < cols):
                                
                                #Enlarge neighbor cells
                                enlarged_map[ni][nj] = 100
    
    '''
    plt.imshow(enlarged_map, cmap = 'gray')
    plt.show()
    '''

    return enlarged_map

def map_callback(map_msg):
    print('Mapa recibido')
    global map_data
    
    map_data = map_msg

    free_cells = 0
    occupied_cells = 0

    if finished:
        for i in map_data.data:
            
            if i == 0:
                free_cells += 1
            elif i == 100:
                occupied_cells += 1

        print('Free cells: ', free_cells)
        print('Occupied cells: ', occupied_cells)
    

def select_and_publish_goal(event):
    print('MoveBase callback')
    global map_data
    global visited
    global prev
    global next
    global localmin_counter
    global stucked_counter
    global goal_client
    global enlargement
    global finished

    

    if map_data is not None:
        print('Eligiendo destino...')

        width = map_data.info.width
        height = map_data.info.height

        map = np.reshape(map_data.data, (width, height))
        
        rows = map.shape[0]
        cols = map.shape[1]

        
        map = enlarge_obstacles(map, enlargement).copy()

        # Create array to store gaps
        gaps = []

        for i in range(rows):
            for j in range(cols):
                if map[i][j].astype(int) == 0 and [i,j] not in visited:
                    # For each free cell
                    
                    neighbours = [[i-1,j], [i+1,j], [i,j-1], [i,j+1]]

                    for r, c in neighbours:
                        if (0 <= r < rows) and (0 <= c < cols):
                            if (map[r][c].astype(int) == -1):

                                # Add gap if neighbor cell is not explored
                                gaps.append([i,j])

        # print(gaps)
                    
        # Exploration ending condition
        if len(gaps) == 0 or stucked_counter > 10:
            if stucked_counter <=10: print('EXPLORATION FINISHED!!')
            else: print('EXPLORATION ABORTED')

            end_time = rospy.Time.now()
            elapsed_time = end_time - start_time
            print("Tiempo empleado en la exploración", elapsed_time.secs)

            finished = True

            exit(0)
        
        else:
            
            rospy.sleep(3) # Wait to refresh map
            next = clustering(gaps) # Next goal obtained from biggest cluster

            near = False

            # Check if the goal has changed
            if prev is not None:

                dist = np.array(next) - prev
                length = np.linalg.norm(dist)

                if length < 10:
                    near = True
                    localmin_counter += 1
            
            if localmin_counter > 3: # Robot stucked in local minimum

                for i in range(rows):
                    for j in range(cols):
                        if map[i][j].astype(int) == 0:
                            next = [i,j] 

                localmin_counter = 0
                stucked_counter +=1

            prev = next

            if not near: # Goal not near to previous destination
                
                print('Goal selected. Navigating to: ', next)
                visited.append(next)

                
                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = "map"
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.pose.position.x = (next[1] * map_data.info.resolution) + map_data.info.origin.position.x
                goal.target_pose.pose.position.y = (next[0] * map_data.info.resolution) + map_data.info.origin.position.y
                goal.target_pose.pose.orientation.w = 1.0

                print(goal.target_pose.pose.position.x, goal.target_pose.pose.position.y)
                

                goal_client.send_goal(goal, feedback_cb=feedback_callback) # Send goal
                wait = goal_client.wait_for_result()

                print('Goal reached!')

            else: print('Skipping: ', next) # Skip goal if close to previous destination

    return map

if __name__ == '__main__':
    try:
        rospy.init_node('exploration', anonymous=True)
        map_data = None

        map_subscriber = rospy.Subscriber('/map', OccupancyGrid, map_callback, queue_size=1)
        map_pub = rospy.Publisher('/map_2', OccupancyGrid, queue_size=1)

        goal_client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        goal_client.wait_for_server()

        timer = rospy.Timer(rospy.Duration(1), select_and_publish_goal)
        start_time = rospy.Time().now()

        while not rospy.is_shutdown():
            rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("Exploration finished.")
        
