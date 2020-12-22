#include "ros/ros.h"
#include "geometry_msgs/Point.h" // Used in dummy function

namespace mine_operation {

    class MiningCart
    {
    private:
	ros::NodeHandle n_;
    ros::Publisher mine_pub; // Klargøring til subscriber (I må selv lige ændre navnet)

    //
    //
    //
    // Methods go below this


    // Dummy publisher (change stuff as needed)
    void dummyPublisher(){
            int operation = 0;
            geometry_msgs::Point p;

            if (operation == 0){
                ros::spinOnce;
                }else
                {
                ros::Rate publish_rate(20);

                p.x = 1;
                p.y = 2;

                mine_pub.publish(p);

                publish_rate.sleep();

                ros::spinOnce;

            return;
            }     
        }


    // Methods end here
    //
    //
    // 

 


    // Corona distance added to constructor



    // Constructor for the MiningCart class            
    public:
    MiningCart() :

    // Nodehandle initializing
    n_()

    {

    // Publisher (change name, type and info as needed)
    mine_pub = n_.advertise<geometry_msgs::Point>("/mine_message",10);

    std::cout << "MiningCart class initialized" << std::endl;

    }
    
    // Deconstrcutor for the MiningCart class
    ~MiningCart() {}
    };
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "MiningCart");

    mine_operation::MiningCart node1;

    ros::spin();
    
    return 0;
}