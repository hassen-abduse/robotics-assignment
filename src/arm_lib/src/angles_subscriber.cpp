#include "ros/ros.h"
#include "arm_lib/angles.h"

void on_angle_publish(const arm_lib::angles &angles)
{
    ROS_INFO("Current Angle Values: \nChasis-Arm1: [%f] \nArm1-Arm2: [%f] \nArm2-Arm3: [%f] \nArm3-Arm4: [%f]",
             angles.joint1, angles.joint2, angles.joint3, angles.joint4);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "angles_subscriber");
    ros::NodeHandle node;
    ros::Subscriber sub = node.subscribe("arm/current_joint_angles", 1000, on_angle_publish);

    ros::spin();
    return 0;
}