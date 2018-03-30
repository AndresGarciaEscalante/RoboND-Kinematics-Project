[![Udacity - Robotics NanoDegree Program]
# Robotic arm - Pick & Place project
## Using the KUKA KR 210
---------------------------------Falta image-------------------------------------------------

## Setting up the enviroment:
For this project used the following programs:
- Ubuntu 16.04 LTS OS
- Ros Luna 1.13.6
- Gazebo 7.9
- Rviz 1.12.15
## Installation steps:
- Clone this repository to your home directory:
``` Python
$ git clone https://github.com/mkhuthir/RoboND-Kinematics-Project.git ~/catkin_ws 
```
- As this project uses custom Gazebo 3D models, we need to add the path through environment variable:
```$ echo "export GAZEBO_MODEL_PATH=~/catkin_ws/src/kuka_arm/models" >> ~/.bashrc
```
- Install missing ROS dependencies using the rosdep install command:
```
$ cd ~/catkin_ws/
$ rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y
```
- Run catkin_make from within your workspace to build the project:
```
$ cd ~/catkin_ws/
$ catkin_make
```
- Run the following shell commands to source the setup files:
```
$ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```
- For demo mode make sure the demo flag is set to true in inverse_kinematics.launch file under ~/catkin_ws/src/kuka_arm/launch/

- You can also control the spawn location of the target object in the shelf by modifying the spawn_location argument in target_description.launch file under ~/catkin_ws/src/kuka_arm/launch/. 0-9 are valid values for spawn_location with 0 being random mode.

- To run forward kinematics test us:
```
$ roslaunch kuka_arm forward_kinematics.launch
```
- To run simulator use:
```
$ rosrun kuka_arm safe_spawner.sh
```
- To run IK Server use:
```
$ rosrun kuka_arm IK_server.py 
```
