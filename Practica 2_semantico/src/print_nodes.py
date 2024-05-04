import rospy
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseWithCovarianceStamped
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import random
import math
from collections import deque
import copy
from tf import transformations

def manhattan(a, b):
  return abs(a) + abs(b)

def euclidean(a, b):
  return math.hypot(a, b)

# Funciones auxiliares para mostrar nodos y arcos del grafo
def build_marker_node(id : int, x : float, y : float, r : float, g : float, b : float):
  marker = Marker()
  marker.header.stamp = rospy.Time.now()
  marker.header.frame_id = "map"
  marker.type = marker.SPHERE
  marker.action = marker.ADD
  marker.scale.x = 0.1
  marker.scale.y = 0.1
  marker.scale.z = 0.1
  marker.color.a = 0.8
  marker.color.r = r
  marker.color.g = g
  marker.color.b = b
  marker.pose.orientation.w = 1.0
  marker.pose.position.x = x
  marker.pose.position.y = y
  marker.pose.position.z = 0
  marker.id = id

  return marker

def build_text_marker(id : int, x : float, y : float, text : str):
  marker = Marker()
  marker.header.stamp = rospy.Time.now()
  marker.header.frame_id = "map"
  marker.type = marker.TEXT_VIEW_FACING
  marker.text = text
  marker.action = marker.ADD
  marker.scale.x = 0.3
  marker.scale.y = 0.3
  marker.scale.z = 0.3
  marker.color.a = 1.0
  marker.color.r = 1.0
  marker.color.g = 1.0
  marker.color.b = 1.0
  marker.pose.orientation.w = 1.0
  marker.pose.position.x = x + 0.05
  marker.pose.position.y = y
  marker.pose.position.z = 0
  marker.id = id

  return marker

def build_arc_marker(id : int, x_0: float, y_0 : float, x_1: float, y_1 : float, r : float, g : float, b : float, size = 0.05, transparency = 0.5):
  marker = Marker()
  marker.header.stamp = rospy.Time.now()
  marker.header.frame_id = "map"
  marker.type = marker.LINE_STRIP
  marker.action = marker.ADD
  marker.scale.x = size
  marker.scale.y = size
  marker.scale.z = size
  marker.color.a = transparency
  marker.color.r = r
  marker.color.g = g
  marker.color.b = b
  marker.pose.orientation.w = 1.0

  p0 = Point()
  p0.x = x_0
  p0.y = y_0

  marker.points.append(p0)

  p1 = Point()
  p1.x = x_1
  p1.y = y_1

  marker.points.append(p1)
  
  marker.id = id

  return marker

def clear_marker():
  marker_array = MarkerArray()
  marker = Marker()
  marker.id = 0
  marker.action = Marker.DELETEALL
  marker_array.markers.append(marker)
  return marker_array

# Clse con la que se maneja el grafo topológico
class GraphHandler:
  def __init__(self):
    self.subNodes = self.get_subnodes()
    self.connection_area, self.connections_pos = self.get_connections_between_areas(max(self.subNodes.keys()) + 1)

    # Inicializar marker array.
    self.array = MarkerArray()

    # Mostrar información cargada de los ficheros
    print("SUBNODOS", self.subNodes)
    print("CONEXIONES MAPA", self.connection_area)
    print("CONEXIONES COORDENADAS", self.connections_pos)
  
  # Funcion para leer el fichero subNodos.txt para cargarlo.
  # Retorna un diccionario donde las claves son identificadores (int)
  # y los valores, son tuplas que contienen (x, y, zona)
  def get_subnodes(self):
    subNodes = dict()
    with open('subNodos.txt', 'r') as file:
      id = 0
      # Iterar sobre cada linea
      for line in file:
          values = line.strip().split(',')

          current_x = float(values[0])
          current_y = float(values[1])

          # Antes de almacenar el nodo, revisar si ya se tiene uno guardado
          # cerca de el. Downsampling
          near_other = False

          for comp_x, comp_y, _ in subNodes.values():
            dist = euclidean(comp_x - current_x, comp_y - current_y)

            if dist < 0.1:
              near_other = True

          # Se resta 2 al valor de la zona para iniciar siempre en 0
          if not near_other:
            subNodes[id] = (current_x, current_y, int(values[2]) - 2)
        
          id += 1
    
    return subNodes

  # Cargar conexiones entre áreas
  # Retorna dos diccionarios donde las claves son identificadores (int)
  # Primer dicccionario: los valores son las conexiones entre zonas.
  # Segundo diccionario: las coordenadas de dicha conexión.
  # Los identificadores de los nodos conexión, continuan con los valores de los
  # nodos de las habitaciones
  def get_connections_between_areas(self, starting_id : int):
    connections_pos = dict()
    connection_area = dict()
    with open('mapaTopo.txt', 'r') as file:
      id = starting_id
      for line in file:
        values = line.strip().split(',')
        connections_pos[id] = (float(values[0]), float(values[1]), id)
        connection_area[id] = (int(values[2]) - 2, int(values[3]) - 2)

        id += 1

    return connection_area, connections_pos
 
  # Mostrar subnodos junto a su identificador
  def displayNodes(self, subNodes:dict, seed : int):
    markers = list()
    
    # Generar una lista de colores (R,G,B) proporcional al número de zonas
    max_value = max(subNodes.values(), key=lambda x: x[2])
    max_number_of_zones = max_value[2]
    colors = list()
    random.seed(seed)
    for _ in range(max_number_of_zones + 1):
      colors.append((random.uniform(0.2, 1), random.uniform(0.2, 1), random.uniform(0.2, 1)))

    # Almacenar el marker para cada nodo, con el color correspondiente a su zona
    for id in subNodes.keys():
      x, y, zone = subNodes[id]
      color_r, color_g, color_b = colors[zone]
      markers.append(build_marker_node(int(id + seed), x, y, color_r, color_g, color_b))
      markers.append(build_text_marker(int(id + seed + 1000), x, y, str(id)))

    return markers

  def find_closest_node(self, interest_node : tuple, id : int, nodes : dict):

    x_interest, y_interest, zone_interest = interest_node
    closest = -1
    closest_dist = 1000000
    for k in nodes.keys():
      x, y, zone = nodes[k]

      # Continue. Different zone
      if k == id or (zone != zone_interest and zone_interest != -1):
        continue
      
      # Calculate distance with this node
      dist = euclidean(x - x_interest, y - y_interest)
      
      # Update if closer
      if dist < closest_dist:
        closest = k
        closest_dist = dist
    return closest

  # Algoritmo BFS que retorna la ruta entre dos nodos. Retorna una lista vacia
  # si no hay plan
  def bfs(self, graph : dict, start_node : int, end_node : int):
      # Initialize a queue for BFS
      queue = [(start_node, [])]
      
      # Initialize a set to keep track of visited nodes
      visited = set()
      
      # Mark the start node as visited
      visited.add(start_node)
      
      # Perform BFS
      while len(queue) > 0:
          # Get the next node from the queue
          current_node, path_to_node = queue.pop(0)

          visited.add(current_node)

          if current_node == end_node:
            return path_to_node
                  
          # Visit all neighbors of the current node
          for neighbor in graph.get(current_node):
              already_added = False
              for n, _ in queue:
                if n == neighbor:
                  already_added = True
                  break

              if neighbor not in visited and not already_added:
                  # Mark the neighbor as visited and enqueue it
                  path_to_this_neighbor = list(path_to_node) + [neighbor]
                  queue.append((neighbor, path_to_this_neighbor))

      return list()

  # Algoritmo BFS, cuyo objetivo es retornar todos los nodos vecinos
  # al nodo introducido
  def bfs_connectivity(self, graph : dict, start_node : int):
      # Initialize a queue for BFS
      queue = [start_node]
      
      # Initialize a set to keep track of visited nodes
      visited = set()
      
      # Mark the start node as visited
      visited.add(start_node)
      
      # Perform BFS
      while len(queue) > 0:
          # Get the next node from the queue
          current_node = queue.pop(0)

          if current_node == -1:
            return visited
          visited.add(current_node)
                  
          # Visit all neighbors of the current node
          for neighbor in graph.get(current_node):
              already_added = False
              for n in queue:
                if n == neighbor:
                  already_added = True
                  break

              if neighbor not in visited and not already_added:
                  # Mark the neighbor as visited and enqueue it
                  queue.append(neighbor)
      
      return visited

  # Separar el grafo en subgrafos en función de las zonas de cada uno 
  # de los nodos. Es un diccionario indexado por la zona, cuyo valor 
  # es otro diccionario con los nodos y sus conexiones
  def get_rooms_subgraphs(self, graph : dict, subNodes : dict):
    subgraphs = dict()

    for node in graph.keys():
      _, _, zone = subNodes.get(node)

      if len(subgraphs) == 0 or zone not in subgraphs.keys():
        subgraphs[zone] = dict()

      aux = {node: graph[node]}
      subgraphs[zone].update(aux)

    # Mostar el subgrafo de cada zona
    for k in sorted(subgraphs.keys()):
      print("Zona", k, subgraphs.get(k)) 
    
    return subgraphs

  # En esta función se conectan los grafos aislados dentro de cada grafo
  def join_sub_regions(self, graph: dict, subNodes : dict):

    print("Uniendo subgrafo", graph)

    # Si el grafo únicamente tiene un nodo no se hace nada.
    if (len(graph.keys()) == 1):
      return graph
    
    i = 0
    clusters = list()
  
    # Obtener nodos pertenecientes a cada subgrafo de cada zona.
    # Se guardan en la lista clusters y se obtienen empleando BFS.

    for n_1 in graph.keys():
      
      # Buscar nodos a los que el nodo n_1 tiene conexión
      cluster = sorted(self.bfs_connectivity(graph, n_1))

      # Si no se tienen almacenados los vecinos de este nodo, se almacenan
      if cluster not in clusters:
        clusters.append(cluster)

    clusters_info = dict()
    i = 0

    # Si solo hay un cluster, el grafo de la zona esta totalmente conectado.
    if len(clusters) == 1:
      return graph
    
    # Se guardan en clusters_info, un diccionario donde las claves son un identificador
    # y los valores son un diccionario que contienen:
    # cluster: lista con los nodos de ese cluster y centroid_x, centroid_y: coordenadas del centroide
    for cluster in clusters:
      c = dict()
      c["cluster"] = cluster
      centroid_x = 0
      centroid_y = 0

      x_sum = 0
      y_sum = 0

      for n in cluster:
        x, y, zone = subNodes[n]
        x_sum += x
        y_sum += y 

      centroid_x = x_sum / len(cluster)
      centroid_y = y_sum / len(cluster)

      c["centroid_x"] = centroid_x
      c["centroid_y"] = centroid_y

      clusters_info[i] = c
      i += 1

    
    proximity = dict()

    # Obtener cuales son los clusters mas proximos entre ellos, respecto a sus centroides.
    # Se almacenan las parejas de cada cluster en
    for i in range(len(clusters_info.keys())):
      closest = -1
      dist = 10000000

      for j in range(len(clusters_info.keys())):
        if (i == j):
          continue
        
        dist_pair = euclidean(clusters_info[i]["centroid_x"] - clusters_info[j]["centroid_x"], clusters_info[i]["centroid_y"] - clusters_info[j]["centroid_y"])
  
        if  dist_pair < dist:
          closest = j
          dist = dist_pair
    
      proximity[i] = closest

    # Conectar los clusters en el grafo
    new_connected_graph = copy.deepcopy(graph)
    for i in range(len(proximity.keys())):
      cluster_1 = clusters_info[i]["cluster"]
      cluster_2 = clusters_info[proximity[i]]["cluster"]

      # Encontrar la pareja de nodos más cercana entre los dos clusters
      n_1_min = -1
      n_2_min = -1
      dist = 100000000
      for n_1 in cluster_1:
        for n_2 in cluster_2:
          dist_pair = euclidean(subNodes[n_1][0] - subNodes[n_2][0], subNodes[n_1][1] - subNodes[n_2][1])
          if (dist_pair < dist):
            dist = dist_pair
            n_1_min = n_1
            n_2_min = n_2

      # Conectar los dos nodos más cercanos entre los dos clusters
      new_connected_graph[n_1_min].add(n_2_min)
      new_connected_graph[n_2_min].add(n_1_min)
    
    # Mostrar grafo previo a la conexión y posterior
    print("O", graph)
    print("N", new_connected_graph)

    return new_connected_graph

  # Contruye los grafos de cada habitación
  def joint_zone_graph(self, graph : dict, subNodes):

    new_graph = copy.deepcopy(graph)

    # Se itera por todo el grafo de la habitación hasta que el grafo este completamente conectado
    # La desconexión se debe a que cuando se conectan los nodos, entre ellos se busca solo el más cercano
    # por lo que hay que acabar de conectar los que se han logrado conectar con los que no
    while True:
      new_graph = self.join_sub_regions(copy.deepcopy(new_graph), subNodes)
      clusters = list()
    
      # Obtener nodos pertenecientes a cada subgrafo de cada zona
      for n_1 in new_graph.keys():
      
        cluster = sorted(self.bfs_connectivity(new_graph, n_1))

        if cluster not in clusters:
          clusters.append(cluster)

      # Si solo hay 1 cluster, esta totalmente conectado.
      if len(clusters) == 1:
        return new_graph

  def get_map_graph(self):
    
    # Por cada nodo, se busca su vecino más cercano
    neighbors = dict()

    for s in self.subNodes.keys():
      closest = self.find_closest_node(self.subNodes[s], s, self.subNodes)
      # Por cada nodo (id) se guarda un set con el vecino
      neighbors[s] = set()
      neighbors[s].add(closest)

    # Recorrer los vecinos y añadir conexiones en doble sentido
    for node in neighbors.keys():
      neighbors_set : set = neighbors[node]

      for neighbor in neighbors_set:
        # Ignorar los casos -1, ya que se corresponden a nodos que no tienen vecinos
        # son nodos que están solos en su zona
        if neighbor == -1:
          continue
        # Si en nodo actual no esta en el set del vecino, se añade.
        if node not in neighbors[neighbor]:
          neighbors[neighbor].add(node)
    
    # Separar el grafo completo, en subgrafos por cada habitación (zona)
    subgraphs_per_zones = self.get_rooms_subgraphs(neighbors, self.subNodes)

    connected_subgraps = dict()
    
    i = 0
    full_graph = dict()
    
    # Unir los grafos de las habitaciones en un único grafo
    for k in subgraphs_per_zones.keys():
      print("Uniendo subgrafo")
      connected_subgraps[k] = self.joint_zone_graph(subgraphs_per_zones[k], self.subNodes)

      # Unir los subgrafos por cada zona en un grafo completo
      for node in connected_subgraps[k].keys():
        if node in full_graph.keys():
          full_graph[node].add(connected_subgraps[k][node])
        else:
          full_graph[node] = copy.deepcopy(connected_subgraps[k][node])
      
    print(full_graph)

    # Construir markers para los arcos entre los distintos nodos
    for n in full_graph.keys():
      for v in full_graph[n]:
        if v == -1:
          continue
        self.array.markers.append(build_arc_marker(i, self.subNodes[n][0], self.subNodes[n][1], self.subNodes[v][0], self.subNodes[v][1], 0.0, 1.0, 0))
    
        i+= 1

    #graph_zones = self.get_rooms_subgraphs(full_graph, self.subNodes)
    #print("Grafo unidos", graph_zones)
    #print(self.connection_area)

    # Iterar sobre las conexiones entre las distintas habitaciones, e incorporarlas al grafo
    # para conectar el grafo completamente.
    for connection in self.connection_area.keys():
      zone_1, zone_2 = self.connection_area[connection]

      # Obtener el nodo más cercano a la conexión de cada zona
      zone_1_closest = self.find_closest_node((self.connections_pos[connection][0], self.connections_pos[connection][1], zone_1), -1, self.subNodes)
      zone_2_closest = self.find_closest_node((self.connections_pos[connection][0], self.connections_pos[connection][1], zone_2), -1, self.subNodes)

      # Añadir conexión a los nodos en el grafo
      full_graph[zone_1_closest].add(connection)
      full_graph[zone_2_closest].add(connection)

      # Añadir la conexión al grafo con las conexiones a las dos zonas
      if connection not in full_graph.keys():
        full_graph[connection] = set([zone_1_closest, zone_2_closest])
      else:
        full_graph[connection].add(zone_1_closest, zone_2_closest)

      # Construir markers
      self.array.markers.append(build_arc_marker(i, self.subNodes[zone_1_closest][0], self.subNodes[zone_1_closest][1], self.connections_pos[connection][0], self.connections_pos[connection][1], 0.0, 0.0, 1.0))
      i+= 1
      self.array.markers.append(build_arc_marker(i, self.subNodes[zone_2_closest][0], self.subNodes[zone_2_closest][1], self.connections_pos[connection][0], self.connections_pos[connection][1], 0.0, 0.0, 1.0))
      i+= 1
    
    self.array.markers = self.array.markers + self.displayNodes(self.subNodes, 1000) + self.displayNodes(self.connections_pos, 10000)

    # Añadir conexión a los datos de los subnodos, ya no se considerarán como nodos especiales,
    # sino como un nodo más del grafo. La zona que se les adjunta es una de las que conecta.
    for connection in self.connections_pos:
      self.subNodes[connection] = (self.connections_pos[connection][0], self.connections_pos[connection][1], self.connection_area[connection][0])
    
    # Se retorna el grafo construido
    return copy.deepcopy(full_graph)

  # Añadir un nodo al grafo conectandolo con el más cercano.
  # La zona asignada al nodo nuevo se corresponderá con el más cercano.
  def add_node(self, graph : dict, node : tuple, id = None):
    x, y = node

    # Encontrar nodo más cercano
    closest_node = self.find_closest_node((x, y, -1), -1, self.subNodes)
    
    if id is None:
      node_id = max(list(graph.keys())) + 1
    else:
      node_id = id

    # Añadir conexiones bidireccionales en el grafo
    graph[node_id] = set()
    graph[node_id].add(closest_node)
    graph[closest_node].add(node_id)

    closest_def = self.subNodes[closest_node]
    print("Adding node: ", x, y, "close to", closest_def)

    # Añadir información del nodo nuevo
    self.subNodes[node_id] = (x, y, closest_def[2])

    self.array.markers = self.array.markers + [build_marker_node(50000 + node_id, x, y, 1, 0, 0), build_text_marker(int(node_id + 51000), x, y, str(node_id))]
    
    # Retorna el id asignado al nodo nuevo.
    return node_id

# Clase que se encarga de la navegación del robot
class TopologicNavigation:
  # Constructor: crea los publicadores, subscriptores y action clients necesarios
  def __init__(self) -> None:

    self.marker_pub = rospy.Publisher("markers", MarkerArray, queue_size = 2, latch=True)
    self.pose_sub = rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, self.robot_pose_callback, queue_size=3)

    self.navigation_client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    self.navigation_client.wait_for_server()

    # Cargar el grafo y mostrarlo
    self.graph_handler = GraphHandler()
    self.graph = self.graph_handler.get_map_graph()
    self.marker_pub.publish(clear_marker())
    self.marker_pub.publish(self.graph_handler.array)

    self.navigation_thread = rospy.Timer(rospy.Duration(2), self.navigation)

    self.robot_pose = None
    self.goal_node = None
    pass

  def robot_pose_callback(self, msg : PoseWithCovarianceStamped):
    self.robot_pose = msg
    pass

  def navigation(self, event):
    if self.robot_pose is None:
      return
    
    # Guando se conozca la posición del robot y no se tenga objetivo definido
    # solicitarlo.
    if self.goal_node is None:
      try:
        self.goal_node = int(input("Introducir el identificador del nodo al que se desea enviar el robot: "))
  
        # Reset graph
        self.graph_handler = GraphHandler()
        self.graph = self.graph_handler.get_map_graph()
        self.marker_pub.publish(clear_marker())
        self.marker_pub.publish(self.graph_handler.array)

        if self.goal_node not in self.graph.keys():
          print("El nodo introducido como objetivo no existe en el grafo")
          self.goal_node = None
          return
        
      except TypeError as e:
        print("El valor indicado no es correcto")
        return

      graph = copy.deepcopy(self.graph)

      # Añadir un nodo en la posición del robot al grafo
      robot_node_id = self.graph_handler.add_node(graph, (self.robot_pose.pose.pose.position.x, self.robot_pose.pose.pose.position.y))


      print(robot_node_id)

      # Planificar entre el nodo correspondiente al robot y el recibido
      topological_plan = self.graph_handler.bfs(graph, robot_node_id, self.goal_node)
      print(topological_plan)
      
      if len(topological_plan) == 0:
        print("No se ha encontrado plan hasta el nodo introducido")
      
      # Mostrar ruta en rviz (naranja)
      self.graph_handler.array.markers = self.graph_handler.array.markers + [build_arc_marker(10000, self.robot_pose.pose.pose.position.x, self.robot_pose.pose.pose.position.y, self.graph_handler.subNodes[topological_plan[0]][0], self.graph_handler.subNodes[topological_plan[0]][1], 1.0, 0.624, 0.0, 0.1, 1.0)]
      for i in range(len(topological_plan) - 1):
        node = topological_plan[i]
        next = topological_plan[i + 1]
        self.graph_handler.array.markers = self.graph_handler.array.markers + \
          [build_arc_marker(10001 + i, self.graph_handler.subNodes[node][0], self.graph_handler.subNodes[node][1], self.graph_handler.subNodes[next][0], self.graph_handler.subNodes[next][1], 1.0, 0.624, 0.0, 0.1, 1.0)] 

      self.marker_pub.publish(clear_marker())
      self.marker_pub.publish(self.graph_handler.array)

      # Iterar para enviar los goals al navigation
      for i in range(len(topological_plan)):
        if i == 0:
          start_pos = (self.robot_pose.pose.pose.position.x, self.robot_pose.pose.pose.position.y)
          end_pos = (self.graph_handler.subNodes[topological_plan[i]][0], self.graph_handler.subNodes[topological_plan[i]][1])
        else:
          start_pos = (self.graph_handler.subNodes[topological_plan[i - 1]][0], self.graph_handler.subNodes[topological_plan[i - 1]][1])
          end_pos = (self.graph_handler.subNodes[topological_plan[i]][0], self.graph_handler.subNodes[topological_plan[i]][1])

        # Calcular angulo de llegada a la posicion final
        angle = math.atan2(end_pos[1] - start_pos[1], end_pos[0] - start_pos[0])
        print("Yendo a :", end_pos, "con orientacion:", math.degrees(angle), "en el frame de map")

        quaternion = transformations.quaternion_from_euler(0, 0, angle)

        # Llamar a move base
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = end_pos[0]
        goal.target_pose.pose.position.y = end_pos[1]
        goal.target_pose.pose.orientation.x = quaternion[0]
        goal.target_pose.pose.orientation.y = quaternion[1]
        goal.target_pose.pose.orientation.z = quaternion[2]
        goal.target_pose.pose.orientation.w = quaternion[3]
        
        self.navigation_client.send_goal(goal)
        
        wait = self.navigation_client.wait_for_result() # Esperar por la navegacion
  
      self.goal_node = None
    pass

if __name__ == "__main__":

  rospy.init_node('exploration', anonymous=True)

  topo_nav = TopologicNavigation()
  
  rospy.spin()
