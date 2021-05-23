#include "ros/ros.h"
#include "arm_lib/Transform.h"

std::vector<double> rot_trans(std::vector<double> vectors, std::vector<double> angles, double d);
bool transform(arm_lib::Transform::Request &req, arm_lib::Transform::Response &res)
{
    ROS_INFO("Transformation: ");
    res.output_vectors.resize(3);

    std::vector<double> vectors = req.input_vectors;
    std::vector<double> angles = req.angles;
    double d = req.d;

    std::vector<double> output = rot_trans(vectors, angles, d);
    res.output_vectors = output;
    for (int i = 0; i < 3; i++)
    {
        ROS_INFO("Input Vectors: [v%u: %f]", i, req.input_vectors[i]);
    }
    for (int i = 0; i < 3; i++)
    {
        ROS_INFO("Input Angles: [v%u: %f]", i, req.angles[i]);
    }
    ROS_INFO("Input d: %f", req.d);
    for (int i = 0; i < 3; i++)
    {
        ROS_INFO("Output Vectors: [v%u: %f]", i, res.output_vectors[i]);
    }

    return true;
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "transformation_server");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("transform_vector", transform);
    ROS_INFO("Ready to transform vectors.");
    ros::spin();

    return 0;
}

std::vector<double> rot_trans(std::vector<double> vectors, std::vector<double> angles, double d)
{
    double x = vectors[0];
    double y = vectors[1];
    double z = vectors[2];
    double roll = M_PI * angles[0] / 180;
    double pitch = M_PI * angles[1] / 180;
    double yaw = M_PI * angles[2] / 180;

    const double Rx[3][3] = {
        {1, 0, 0},
        {0, cos(roll), -sin(roll)},
        {0, sin(roll), cos(roll)}};
    const double Ry[3][3] = {
        {cos(pitch), 0, sin(pitch)},
        {0, 1, 0},
        {-sin(pitch), 0, cos(pitch)}};

    const double Rz[3][3] = {
        {cos(yaw), -sin(yaw), 0},
        {sin(yaw), cos(yaw), 0},
        {0, 0, 1}};

    double Rxy[3][3];
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            for (int k = 0; k < 3; ++k)
            {
                Rxy[i][j] += Rx[i][k] * Ry[k][j];
            }
    double R[3][3];
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            for (int k = 0; k < 3; ++k)
            {
                R[i][j] += Rxy[i][k] * Rz[k][j];
            }
    double H[4][4] = {
        {1, 0, 0, d * cos(roll)},
        {0, 1, 0, d * cos(pitch)},
        {0, 0, 1, d * cos(yaw)},
        {0, 0, 0, 1}};

    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
        {
            H[i][j] = R[i][j];
        }

    double output[4][3];
    double hom_vec[1][4] = {{x, y, z, 1}};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            for (int k = 0; k < 3; ++k)
            {
                output[i][j] += H[i][k] * hom_vec[k][j];
            }
    std::vector<double> result = {output[0][3], output[1][3], output[2][3]};
    return result;
}
