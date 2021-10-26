#include <ros/ros.h>
#include "example_interfaces/AddTwoInts.h"

bool add(
        example_interfaces::AddTwoInts::Request& req,
        example_interfaces::AddTwoInts::Response& res)
{
    res.sum = req.a + req.b;
    ROS_INFO("Request: a=%ld, b=%ld", (long int)req.a, (long int)req.b);
    ROS_INFO("Sending back response: [%ld]", (long int)res.sum);
    return true;
}

int main(
        int argc,
        char** argv)
{
    ros::init(argc, argv, "add_two_ints_server");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("add_two_ints", add);
    ROS_INFO("Ready to add two ints.");
    ros::spin();

    return 0;
}