# eduardo_correa_PB
Projeto de Bloco de Sistemas Robóticos
```bash
$ cd eduardo_correa_PB
$ catkin_make
$ source devel/setup.bash
$ export TURTLEBOT3_MODEL=burger
$ roslaunch turtlebot3_gazebo turtlebot3_world.launch
$ roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=src/eduardo_correa_pb/src/turtlebot3_world.yaml
$ rosrun eduardo_correa_pb postoffice.py
```
