#include "ros/ros.h"
#include "arm_lib/Transform.h"
#include <cstdlib>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "transfomation_client");
    if(argc != 8)
    {
        ROS_INFO("usage: transformation_client X Y Z a b g d");
        return 1;
    }

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<arm_lib::Transform>("transform_vector");

    arm_lib::Transform srv;

    std::vector<double> vectors = { atof(argv[1]), atof(argv[2]), atof(argv[3]) };
    std::vector<double> angles = { atof(argv[4]), atof(argv[5]), atof(argv[6]) };
    double d = atof(argv[7]);
    
    srv.request.input_vectors = vectors;
    srv.request.angles = angles;
    srv.request.d = d;
    for (int i=0; i<3; i++) {
        ROS_INFO("Input Vectors: [v%u: %f]", i, srv.request.input_vectors[i]);
    }
    for (int i=0; i<3; i++) {
        ROS_INFO("Input Angles: [v%u: %f]", i, srv.request.angles[i]);
    }
    ROS_INFO("Input d: %f", srv.request.d);
    if(client.call(srv))
    {
        for (int i=0; i<3; i++) {
            ROS_INFO("Output Vectors: [v%u: %f]", i, srv.response.output_vectors[i]);
    }
        
    }

    else
    {
        ROS_ERROR("Failed to call service transform_vector");
        return 1;
    }

    return 0;


}
