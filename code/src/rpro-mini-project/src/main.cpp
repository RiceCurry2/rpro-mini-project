#include <ros/ros.h>
#include <thread>
#include <chrono>

#include "rmp_turtlemap/rmpTurtleMap.hpp"
#include <turtlesim/TeleportAbsolute.h>

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "rpro_mini_project");
    ros::NodeHandle nodeHandle("~");
    
    std::thread t1([&]{
        rmp_turtlemap::rmpTurtleMap turtleMap;
    });

    std::this_thread::sleep_for(std::chrono::seconds(3));

    ros::service::waitForService("/turtle1/teleport_absolute" -1);

    //ros::ServiceClient teleport_client = nodeHandle.serviceClient<turtlesim::TeleportAbsolute("/turtle1/teleport_absolute");

    ROS_INFO("I am here");

    t1.join();
    
    return 0;
}