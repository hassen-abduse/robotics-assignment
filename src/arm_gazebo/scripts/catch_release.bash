#!/bin/bash
rostopic pub /arm/pose_cmd arm_gazebo/pose "x: 3 y: 0. z: 0.0 option: 1" 
rostopic pub /arm/pose_cmd arm_gazebo/pose "x: -3 y: 0. z: 0.0 option: 0" 
