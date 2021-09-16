# Sawyer_Yoyo
Author: Tianyu Li


## Introduction
This project utilizes data-driven control to make a Sawyer robot to play yoyo.

## Packages
https://github.com/wuphilipp/sawyer_kdl
https://github.com/orocos/orocos_kinematics_dynamics

## Instructions

### Environment Setup
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
9. cd into yoyo workspace and do ```catkin_make_isolated```
10. cd into rethink workspace, and ```rm -rf``` the ```build``` and ```devel``` folders
11. ```catkin_make``` in the rethink workspace and do ```source devel/setup.bash```
12. cd back to the yoyo workspace, and do ```source devel/setup.bash```

### Camera Setup
1. Connect the camera to the computer
2. Launch Spinviewer
3. Set the following
  - exposure auto: off
  - exposure time: ~1000
  - gain auto: off
  - width: 548
  - ADC Bit Depth: 8 bit
  - Back to the top, set Acquisition Frame Rate to 500 Hz

### Pipeline Launch
1. ```roslaunch sawyer_move sawyer.launch```

