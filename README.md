# Robots Moviles - UC3M
## _Master de Robótica y Automatización, Universidad Carlos 3 de Madrid_
### PRÁCTICA 1 
</p>

***
#### [ENLACE AL REPOSITORIO GITHUB ](https://github.com/Master-Robotica-UC3M/Robots-Moviles)

</p>


***
#### ORGANIZACIÓN DE CARPETAS:
* **launch:** archivos para lanzar los entornos
* **photos:**  imágenes
* **src:**  carpetas de ejecución
* **worlds:**  escenarios.

***
#### FINALIDAD DE LA PRÁCTICA
En esta práctica se prentende generar un algortimo de exploración autónoma para un robot, el turtlebot3, en 4 escenarios indoor diferentes. Para ello se planteará una solución de exploración que permita cumplir el objetivo requerido de la forma más eficiente posible.

***

#### ALGORITMOS DESARROLLADOS:

* **EXPLORATION.PY:** Este algotirmo se basa en buscar fronteras en un escenario usando clusters basados en k-means. Se exploran los vecinos (arriba, abajo, derecha e izquierda) de una celda para saber si la celda esta explorada o no. En caso de que no esté explorada se guarda en una variable. Tras esto,el metodo de los clusters basado en k-means agrupa de la forma más optima posible las celdas sin explorar. Posteriormente el algoritmo decide que el cluster mayor, es decir, el que mayor nñumero de celdas incexplordas contiene, es la mejor opción para que el robot explore. Es en este momento cuando se envía la posición del centroide del cluster al paquede de navegación para que el nodo move_base planifique una ruta desde la posición actual del tobot hasta el objetivo. De esta forma el robot conseguirá explorar de manera autónoma todo el espacio hasta que no haya celdas inexploradas.
* **COUNT_EXPLORES_SPACE.PY:**  Este algoritmo cuenta las celdas libres y ocupadas de los entornos para posteriormente calcular el porcentaje de mapa explorado.


***
### RESULTADOS
En la siguiente sección se ve la imagen de cada escenario, el mapa en RVIZ teleoperado y el que se registra con la exploración autónoma.

**[1. ESCENARIO 1](https://github.com/Master-Robotica-UC3M/Robots-Moviles/blob/main/photos/escenario1.jpg):**

<p algin="center">
    <img src="https://github.com/Master-Robotica-UC3M/Robots-Moviles/blob/main/photos/escenario1.jpg">
    <img src="https://github.com/Master-Robotica-UC3M/Robots-Moviles/blob/main/photos/ESCENARIO1_.jpg">
    <img src="https://github.com/Master-Robotica-UC3M/Robots-Moviles/blob/main/photos/escenario1_t.jpg">
    
</p>

**[2. ESCENARIO 2](https://github.com/Master-Robotica-UC3M/Robots-Moviles/blob/main/photos/escenario2.png):**

<p algin="center">
    <img src="https://github.com/Master-Robotica-UC3M/Robots-Moviles/blob/main/photos/escenario2.png">
    <img src="https://github.com/Master-Robotica-UC3M/Robots-Moviles/blob/main/photos/ESCENARIO2_.jpg">
    <img src="https://github.com/Master-Robotica-UC3M/Robots-Moviles/blob/main/photos/escenario2_t.jpg">
</p>

**[3. ESCENARIO 3](https://github.com/Master-Robotica-UC3M/Robots-Moviles/blob/main/photos/escenario3.png):**

<p algin="center">
    <img src="https://github.com/Master-Robotica-UC3M/Robots-Moviles/blob/main/photos/escenario3.png">
    <img src="https://github.com/Master-Robotica-UC3M/Robots-Moviles/blob/main/photos/ESCENARIO3_.jpg">
    <img src="https://github.com/Master-Robotica-UC3M/Robots-Moviles/blob/main/photos/escenario3_t.jpg">

</p>

**[4. ESTUDIO](https://github.com/Master-Robotica-UC3M/Robots-Moviles/blob/main/photos/estudio.png):**

<p algin="center">
    <img src="https://github.com/Master-Robotica-UC3M/Robots-Moviles/blob/main/photos/estudio.png">
    <img src="https://github.com/Master-Robotica-UC3M/Robots-Moviles/blob/main/photos/ESTUDIO_.jpg">  
    <img src="https://github.com/Master-Robotica-UC3M/Robots-Moviles/blob/main/photos/escenario4_t.jpg">
</p>

**TIEMPOS DE EXPLORACIÓN PARA 10 INTENTOS**
|X| Escenario 1| Escenario 2 | Escenario 3 | Estudio|
|   :---:    |     :---:      |      :---:    |    :---:  | :---:  |
| Media tiempo medio (segundos)  | 90,8 | 326,8 |-|324,3 |
| Desviación estándar     | 30,3 | 62,3 |-|88,1|


**PORCENTAJE DE MAPA EXPLORADO PARA 10 INTENTOS**
|-| Escenario 1| Escenario 2 | Escenario 3 | Estudio|
|   :---:    |     :---:      |      :---:    |    :---:  |:---:|
|Celdas Libres teleoperadas|8029,0|15667,0|15620,0|19309,0 |
| Media celdas libres auto|7998,5|15623,0|12890,3|19253,3|
|Desviación estándar celdas libres auto|28,5|48,0|3166,0|37,6|
|Celdas Ocupadas teleoperadas|1029,0|1907,0|2293,0|1704,0|
| Media celdas ocupadas auto|946,9|1862,2|1676,1|1816,2|
| Desviación estándar celdas ocupadas|37,9|75,1|464,2|78,2|
| % Mapa explorado|99,6|99,7|82,5|99,7|

***
### [VIDEO DE DEMOSTRACIÓN](https://www.youtube.com/watch?v=36naFUwaNik)
