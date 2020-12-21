#include "rmp_turtlemap/rmpTurtleMap.hpp"

namespace rmp_turtlemap{

rmpTurtleMap::rmpTurtleMap(void)
    {
        std::thread map([&]{
        system("terminator --e rosrun turtlesim turtlesim_node");        
    });

    ROS_INFO("Inside class");

    turtlesim::TeleportAbsolute srv;

    map.join();

    }
}