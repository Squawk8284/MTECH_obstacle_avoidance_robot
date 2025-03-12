
List of application implemented on 0xdelta robot

1. interactive marker movement

This is useful in case of navigating the robot through rviz GUI.

launch files:
roslaunch delta_gazebo delta_world.launch
roslaunch delta_description view_robot.launch

2. obstacle avoidance

launch files:
roslaunch delta_gazebo delta_world.launch
roslaunch delta_description view_robot.launch
roslaunch delta_obstacle_avoidance delta_sim_obstacle.launch

3.autonomous map navigation

3.1 To create a map
launch files:
roslaunch delta_gazebo delta_world.launch
roslaunch delta_navigation gmapping_demo.launch 
roslaunch delta_description view_robot.launch config:=gmapping

After all scanning 
rosrun map_server map_saver -f /home/tushar/nex/ROS/catkin_ws/src/delta/delta_navigation/maps/map_2.yaml

3.2 To start naviation autonomously on map_2
launch files:
roslaunch delta_gazebo delta_world.launch
roslaunch delta_navigation amcl_demo.launch map_file:=/home/nex/catkin_ws/src/delta_navigation/maps/map_2.yaml
roslaunch delta_description view_robot.launch config:=localization

4. Robot navigation without map

launch files:
roslaunch delta_gazebo delta_world.launch
roslaunch delta_navigation odom_navigation_demo.launch 
roslaunch delta_description view_robot.launch config:=navigation

