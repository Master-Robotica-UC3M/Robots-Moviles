# Robots Moviles - UC3M
## _Master de Robótica y Automatización, Universidad Carlos 3 de Madrid_
### PRÁCTICA 2 
</p>

***
#### [ENLACE AL REPOSITORIO GITHUB ](https://github.com/Master-Robotica-UC3M/Robots-Moviles)

</p>


***
#### ORGANIZACIÓN DE CARPETAS:
* **Practica 1_geométrico:** archivos para lanzar el funcionamiento de la practica 1
* **Practica 2_semantico:**  archivos para lanzar el fincionamiento de la practica 2

***
#### FINALIDAD DE LA PRÁCTICA
En la práctica anterior se obtuvieron mapas geométricos de cuatro entornos distintos. Serán usados en la práctica actual para generar mapas topológicos que permitan la navegación del robot por todo el espacio. Esta navegación se centra en definir una estrategia basada en nodos de modo que el robot irá de un nodo a otro, con una orientación realista en sentido al punto final.
Por lo tanto los objetivos definidos son:

* Generar un mapa topológico del entorno dado a partir de un mapa geométrico obtenido con anterioridad
* Definir un algoritmo de exploración autónoma usando el mapa topológico.
* Estudio teórico sobre la inferencia de conocimientos mediante la observación del usuario analizando beneficios y aplicaciones, y realizar una presentación en clase

***

#### ALGORITMOS DESARROLLADOS:

##### topoMap.py 
Lo primero que se ha hecho es modificar el archivo topoMap.py para lograr un buen procesado del mapa topológico y crear los nodos y necesarios para la posterior navegación. Se emplea el algoritmo proporcionado por la profesora de la asignatura.
La novedad reside en un método iterativo que ajusta los parámetros de segmentación para obtener un mapa topológico óptimo. Dentro de un bucle iterativo, se realizan múltiples iteraciones variando los parámetros de la segmentación. Después de cada iteración, se muestra la imagen resultante y se espera a que el usuario presione una tecla (q) para continuar con la siguiente iteración. Este enfoque iterativo proporciona una mayor flexibilidad, comodidad y control sobre el proceso de segmentación, lo que puede conducir a una mejor calidad del mapa topológico final. El usuario decide hasta qué punto debe seguir iterando el algoritmo cuando considere que el mapa esta correctamente segmentado, cuando presiona control + C en el terminal y q en la imagen.
Este algoritmo solo se ejecuta una vez para conseguir los .txt necesarios para usarlos en la navegación de manera posterior.

##### topoNav.py
Se ha creado un código que implementa un sistema de navegación para que el robot pueda moverse entre nodos predefinidos en el mapa de forma que se planifiquen rutas optimas haciendo uso de un grafo topológico. 
La clase GraphHandler se usa para manejar el grafo topológico del entorno del robot de forma que se puedan cargar nodos y conexiones desde los archivos .txt generados anteriormente. Partiendo de estos dos ficheros, el algoritmo para unir todos los nodos consta a grandes rasgos de los siguientes pasos.
* Carga de los nodos y las conexiones. En el caso de los nodos se realiza un downsampling para no tener muchos nodos de la misma zona juntos. A cada uno se le asocia un identificador único.
* Una vez se tienen todos los nodos, en un diccionario se almacenan las relaciones entre nodos, siendo las claves los identificadores de cada nodo, y los valores, los identificadores de los vecinos.
* Posteriormente, se busca de cada nodo, el nodo vecino más cercano y su relación se añade al diccionario. Una vez se ha iterado sobre todos, se vuelve a iterar para añadir conexiones bidireccionales. En caso de que un nodo no tenga vecino, para que el set tenga al menos un valor, se asigna el valor -1. Esta fase da una conexión inicial entre los nodos.
* Posteriormente, se separa el diccionario con las relaciones, en una lista de diccionarios donde se almacenan las relaciones de los nodos de las distintas zonas. Se itera por el diccionario de cada zona uniendo los nodos, este proceso se realiza hasta que todos los nodos de la zona estén conectados, comprobación que se realiza con BFS en un modo de inundación. Lo interesante es que el paso 3 dejó por cada zona distintos grafos no conectados (clusters), que se conectan buscando centroides y nodos más próximos entre dos. Para más información ver el código.
* Una vez se han conectado todos los nodos de cada zona, las zonas se vuelven a unir en un único diccionario, al que se comprueban las conexiones bidireccionales y por último se añaden las conexiones entre las zonas, que cuya información se incorpora al grafo.


Este algoritmo es fundamental para el resto del programa de navegación, que contiene además funciones auxiliares para representar el grafo con markers en rviz y dar de alta nuevos nodos en el grafo. Los markers en verde representan conexiones dentro de zonas, los azules las conexiones entre zonas y los naranjas el plan topológico que seguirá el robot.


El resto del control e interacción con el move_base se realiza en la clase TopologicNavigation. Aquí se carga el grafo con la clase anterior y se representa. Cuando el programa recibe la posición del robot, se le solicita al usuario el número del nodo al que se quiere llegar. Lo novedoso de este código es que cuando se recibe el identificador del nodo al que el usuario quiere enviar al robot, se da de alta la posición del robot como un nuevo nodo y se inicia un proceso de exploración basado en el algoritmo BFS, en modo path planning, para encontrar la ruta óptima desde el nodo actual del robot al nodo objetivo. Desde la posición inicial, se exploran todas las direcciones de manera uniforme, estudiando los nodos vecinos antes de avanzar al siguiente nodo. Esto hace que el robot se dirija hacia el nodo más cercano del grafo que este unido.


A medida que se realiza la identificación de nodos que se deberían seguir para llegar al objetivo final, se va marcando de manera visual la trayectoria óptima. Tras encontrar la mejor ruta, el planificador move_base es el que determina la trayectoria que debe seguir el robot para pasar de un nodo a otro. 


***
### RESULTADOS
En la siguiente sección se ve la imagen de cada escenario, el mapa en RVIZ teleoperado y el que se registra con la exploración autónoma.

**[1. ESCENARIO 1](https://github.com/Master-Robotica-UC3M/Robots-Moviles/tree/main/Practica%202_semantico/fotos):**

Es esta imágenes see puede ver como se va generando el mapa topológico de nodos

<p algin="center">
    <img src="https://github.com/Master-Robotica-UC3M/Robots-Moviles/blob/main/Practica%202_semantico/fotos/1_1.png">
    <img src="https://github.com/Master-Robotica-UC3M/Robots-Moviles/blob/main/Practica%202_semantico/fotos/1_2.png">
    <img src="https://github.com/Master-Robotica-UC3M/Robots-Moviles/blob/main/Practica%202_semantico/fotos/1_3.png">
    
</p>
A continuación se unen y se define el path que hay que seguir entre nodos para llegar a destino.
<p algin="center">
    <img src="https://github.com/Master-Robotica-UC3M/Robots-Moviles/blob/main/Practica%202_semantico/fotos/1_4.png">
    <img src="https://github.com/Master-Robotica-UC3M/Robots-Moviles/blob/main/Practica%202_semantico/fotos/1_5.png">
    
</p>

**[2. ESCENARIO 2](https://github.com/Master-Robotica-UC3M/Robots-Moviles/tree/main/Practica%202_semantico/fotos):**
<p algin="center">
    <img src="https://github.com/Master-Robotica-UC3M/Robots-Moviles/blob/main/Practica%202_semantico/fotos/2_1.png">
    <img src="https://github.com/Master-Robotica-UC3M/Robots-Moviles/blob/main/Practica%202_semantico/fotos/2_2.png">
    <img src="https://github.com/Master-Robotica-UC3M/Robots-Moviles/blob/main/Practica%202_semantico/fotos/2_3.png">
    
</p>
A continuación se unen y se define el path que hay que seguir entre nodos para llegar a destino.
<p algin="center">
    <img src="https://github.com/Master-Robotica-UC3M/Robots-Moviles/blob/main/Practica%202_semantico/fotos/2_4.png">
    <img src="https://github.com/Master-Robotica-UC3M/Robots-Moviles/blob/main/Practica%202_semantico/fotos/2_5.png">
    
</p>
**[3. ESCENARIO 3](https://github.com/Master-Robotica-UC3M/Robots-Moviles/tree/main/Practica%202_semantico/fotos):**

<p algin="center">
    <img src="https://github.com/Master-Robotica-UC3M/Robots-Moviles/blob/main/Practica%202_semantico/fotos/3_1.png">
    <img src="https://github.com/Master-Robotica-UC3M/Robots-Moviles/blob/main/Practica%202_semantico/fotos/3_2.png">
    <img src="https://github.com/Master-Robotica-UC3M/Robots-Moviles/blob/main/Practica%202_semantico/fotos/3_3.png">
    
</p>
A continuación se unen y se define el path que hay que seguir entre nodos para llegar a destino.
<p algin="center">
    <img src="https://github.com/Master-Robotica-UC3M/Robots-Moviles/blob/main/Practica%202_semantico/fotos/3_4.png">
    <img src="https://github.com/Master-Robotica-UC3M/Robots-Moviles/blob/main/Practica%202_semantico/fotos/3_5.png">
    
</p>

**[4. ESTUDIO](https://github.com/Master-Robotica-UC3M/Robots-Moviles/tree/main/Practica%202_semantico/fotos):**
<p algin="center">
    <img src="https://github.com/Master-Robotica-UC3M/Robots-Moviles/blob/main/Practica%202_semantico/fotos/4_1.png">
    <img src="https://github.com/Master-Robotica-UC3M/Robots-Moviles/blob/main/Practica%202_semantico/fotos/4_2.png">
    <img src="https://github.com/Master-Robotica-UC3M/Robots-Moviles/blob/main/Practica%202_semantico/fotos/4_3.png">
    
</p>
A continuación se unen y se define el path que hay que seguir entre nodos para llegar a destino.
<p algin="center">
    <img src="https://github.com/Master-Robotica-UC3M/Robots-Moviles/blob/main/Practica%202_semantico/fotos/4_4.png">
    <img src="https://github.com/Master-Robotica-UC3M/Robots-Moviles/blob/main/Practica%202_semantico/fotos/4_5.png">
    
</p>

***
### [VIDEO DE DEMOSTRACIÓN](https://www.youtube.com/watch?v=36naFUwaNik)
