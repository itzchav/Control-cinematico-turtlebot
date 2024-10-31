# Control cinematico Turtlebot 3
Control cinematico en un robot movil diferencial Turtlebot3

## Ejecución en el Turtlebot 3

### Entorno 
Consultar el workspace en:
- [Entorno](https://github.com/itzchav/Entorno-Division-Estudios-Posgrado)


cd ~/catkin_ws

source ./devel/setup.bash

export TURTLEBOT3_MODEL=burger

roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch 


### Control cinemático
cd ~/move_ws

source ./devel/setup.bash

rosrun mov_turtle control_punto_arg.py 1 2 # x=1 y=2 Estos parametros se pueden modificar

### Control cinemático de trayectoria en un tiempo determinado
cd ~/move_ws

source ./devel/setup.bash

rosrun mov_turtle control_trayectoria_recta.py 1 2 10 # x=1 y=2 t=10 Estos parametros se pueden modificar


### Control cinemático de trayectoria Lemniscata
cd ~/move_ws

source ./devel/setup.bash

rosrun mov_turtle control_trayectoria_cata_ph.py
