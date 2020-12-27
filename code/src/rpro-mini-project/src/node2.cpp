#include "ros/ros.h"
#include "rpro_mini_project/logOutput.h" // Used in publisher function

namespace mineoperation {

    class MiningOutput
    {
    private:
	ros::NodeHandle nh;             // NodeHandle definition
    ros::Subscriber mine_sub;       // mine_sub definition

    // Methods ////////////////////////////////////////////////////////////////////////////////////


    // Subscriber function
    void printLog(const rpro_mini_project::logOutput& logOut) {
        // If .comm contains the string "cls" the system("clear")-command is called and the terminal is cleared
        if (logOut.comm == "cls")
            {
                system("clear");
            }
        // If .comm contains anything other than the string "cls", the terminal is cleared and the contend of .comm is printed to the terminal
        else
        {
 //           system("clear");
            std::cout << logOut.comm << std::endl;
        }
    }


    // Methods end here ////////////////////////////////////////////////////////////////////////////
    

    ////////////////////////////////////////////////////////////////////////////////////////////////

    // Corona distance added to constructor

    ////////////////////////////////////////////////////////////////////////////////////////////////


    // Constructor for the MiningOutput class            
    public:
    MiningOutput() :

    // Nodehandle initialisation
    nh()


    {

    // Subscriber to "miningLog" topic -> pointing to printLog function
    mine_sub = nh.subscribe("miningLog", 1000, &MiningOutput::printLog, this);

    system("clear");
    std::cout << "MiningOutput class initialized" << std::endl;

    }
    
    // Deconstrcutor for the MiningOutput class
    ~MiningOutput() {}
    };
} /*namespace*/

int main(int argc, char** argv)
{
    // Initialiser for ROS
    ros::init(argc, argv, "MiningOutput");

    // Instantiation of MiningOutput class
    mineoperation::MiningOutput node2;

    // Rosspin function
    ros::spin();
    
    // Returnvalue of main-function
    return 0;
}