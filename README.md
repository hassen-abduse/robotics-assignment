# robotics-assignment
1. Abduselam Assen – ATR/0138/09
2. Amir Mustefa – ATR/6830/09
3. Beshir Dekebo – ATR/3178/09
4. Mehammed Teshome – ATR/3319/09
#Execution Steps
Execute the following commands to run the program correctly:
$ roscore
$ rosrun arm_gazebo ik_server.py
$ rosrun arm_gazebo fk_server.py
$ roslaunch arm_gazebo arm.launch

To move the End Effector to a position make option 1 to catch and option 0 to release:
$ rostopic pub /arm/pose_cmd arm_gazebo/pose "x: 3 y: 0. z: 0.0 option: 1" 
