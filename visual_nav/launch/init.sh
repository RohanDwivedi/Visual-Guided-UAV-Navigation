#!/bin/bash


xterm  -e " sudo systemctl start firmwared.service && sudo firmwared " &
sleep 10
xterm  -hold -e "sphinx --datalog /home/rohan/Desktop/catkin_ws/src/visual_nav/world/map.world /home/rohan/Desktop/catkin_ws/src/visual_nav/drones/bebop2.drone::name=drone /home/rohan/Desktop/catkin_ws/src/visual_nav/actors/off_road_vehicle.actor::name=vehicle::path=/home/rohan/Desktop/catkin_ws/src/visual_nav/paths/square.path" &
sleep 120
xterm  -hold -e "roslaunch bebop_tools bebop_nodelet_iv.launch " &
sleep 100
xterm   -e " rostopic pub --once bebop/takeoff std_msgs/Empty "  &
sleep 10
xterm   -e " rostopic pub --once bebop/cmd_vel geometry_msgs/Twist '[0,0,1]' '[0,0,0]' && rostopic pub --once bebop/cmd_vel geometry_msgs/Twist '[0,0,0.3]' '[0,0,0]'" &
sleep 30
xterm  -hold -e " source /home/rohan/Desktop/catkin_ws/src/TrainYourOwnYOLO/env/bin/activate  && source /home/rohan/Desktop/cv_bridge_workspace/install/setup.bash --extend && cd /home/rohan/Desktop/catkin_ws/src/visual_nav/scripts/ && python object_detection.py " &
sleep 1
xterm -e " firefox http://localhost:9002 "

 
