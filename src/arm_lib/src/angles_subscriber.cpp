#include "ros/ros.h"
#include "arm_lib/angles.h"

void on_angle_publish(const arm_lib::angles &angles)
{
    ROS_INFO("Current Angle Values: \nChasis-Arm1-Z: [%f] \nChasis-Arm1-X: [%f]\nArm1-Arm2-X: [%f] \nArm2-Arm3-X: [%f] \nArm3-Arm4-X: [%f]",
             angles.joint1_z, angles.joint1_x, angles.joint2_x, angles.joint3_x, angles.joint4_x);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "angles_subscriber");
    ros::NodeHandle node;
    ros::Subscriber sub = node.subscribe("current_joint_angles", 1000, on_angle_publish);

    ros::spin();
    return 0;
}