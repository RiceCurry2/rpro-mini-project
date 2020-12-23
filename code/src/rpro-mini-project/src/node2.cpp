#include "ros/ros.h"
#include "geometry_msgs/Point.h" // Used in dummy function

namespace mineoperation {

    class MiningOutput
    {
    private:
	ros::NodeHandle n_;             // NodeHandle definition
    ros::Subscriber mine_sub;       // Klargøring til subscriber (I må selv lige ændre navnet)
    int dummyValue;                 // Definition of dummy value


    // Methods go below this //////////////////////////////////////////////////////////////////////


    // Dummy function for publisher, change stuff as needed.
    void Dummyfunction(const geometry_msgs::Point p) {
        dummyValue = 1;
    }


    // Methods end here ////////////////////////////////////////////////////////////////////////////
    

    ////////////////////////////////////////////////////////////////////////////////////////////////

    // Corona distance added to constructor

    ////////////////////////////////////////////////////////////////////////////////////////////////


    // Constructor for the MiningOutput class            
    public:
    MiningOutput() :

    // Nodehandle initializing
    n_(),


    // Initializing of global values
    dummyValue(0)

    {

    // Subscriber to a topic -> pointing to a dummyfunction, change as you please.
    mine_sub = n_.subscribe("topic", 100, &MiningOutput::Dummyfunction, this);

    std::cout << "MiningOutput class initialized" << std::endl;

    }
    
    // Deconstrcutor for the MiningOutput class
    ~MiningOutput() {}
    };
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "MiningOutput");

    mineoperation::MiningOutput node2;

    ros::spin();
    
    return 0;
}