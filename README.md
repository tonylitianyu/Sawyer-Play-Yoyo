# Sawyer_Yoyo
Author: Tianyu Li


## Introduction
This project utilizes data-driven approach to teach a Sawyer robot to play yoyo. There are two high speed camera with perpendicular view to each other to provide state esimation. The package also contains a visual feedback controller for demonstrations. Currently it is still under further development for research.

## Dependencies
### Packages
```
intera_sdk from Rethink Robotics
sawyer_kdl https://github.com/wuphilipp/sawyer_kdl  
orocos_kd  https://github.com/orocos/orocos_kinematics_dynamics
```
### Libraries
```
AprilTag
UEye      (for Edmund Optics Camera)
Spinnaker (for Blackfly Camera)
Pygame    (mainly for user interface)
```

## Instructions

### Connection Setup
1. Turn on Sawyer
2. Connect with Sawyer through Ethernet, and ```nmcli connection up Rethink```
3. Ping 10.42.0.2 or ```sawyer.local``` to confirm connection
4. Set environment variables
  ```
  export ROS_MASTER_URI=http://10.42.0.2:11311
  export ROS_IP=10.42.0.1
  unset ROS_HOSTNAME
  ```
5. source rethink workspace ```source rethink_ws/devel/setup.bash```
6. Enable the robot ```rosrun intera_interface enable_robot.py -e```
7. Run the joint_server ```rosrun intera_interface joint_trajectory_action_server.py```
8. Open a new terminal window
9. cd into yoyo workspace and do ```catkin_make``` and ```source devel/setup.bash```
10. cd into rethink workspace, and ```rm -rf``` the ```build``` and ```devel``` folders
11. ```catkin_make``` in the rethink workspace and do ```source devel/setup.bash```

### Pipeline Launch
1. ```roslaunch sawyer_move sawyer.launch```
2. Wind the yoyo up
3. At the pygame window, press ```q``` to start

