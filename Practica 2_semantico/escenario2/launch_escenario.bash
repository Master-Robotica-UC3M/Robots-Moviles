#!/bin/bash

. ~/.bashrc

roscore &
sleep 1
roslaunch robots_moviles turtlebot3_escenario2.launch &
sleep 1
map=$PWD/map2.yaml
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$map open_rviz:=false &
sleep 1
rviz -d ./../topo_nav.rviz &
sleep 1
python3 ./../src/navegacion_topologica.py 
