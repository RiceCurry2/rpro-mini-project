#include <iostream>
#include <sstream>

#include "ros/ros.h"
#include "rpro_mini_project/logOutput.h" // Used in dummy function

namespace mine_operation {

    class MiningCart
    {
    private:
	ros::NodeHandle nh;      // Nodehandle definition
    ros::Publisher mine_pub; // mine_pub definition

    //----------------------//----------------------//----------------------//----------------------//----------------------//
    //---Global Variables---//---Global Variables---//---Global Variables---//---Global Variables---//---Global Variables---//
    //----------------------//----------------------//----------------------//----------------------//----------------------//

    int RM_Heading;
    int RM_XCoord;
    int RM_YCoord;
    int RM_Done;

    int CartTimer;
    int done;
    int DoneMining;
    int Cart;
    int CartFull;
    int TimeSpent;
    int OreCollected;
    int cartFull;

    std::ostringstream stringBuilder;
    std::string logVar;


    //-----------------
    //---Movement---
    //-----------------

    //The robot turns clockwise
    void RM_TurnRight()
    {
        switch(RM_Heading)
        {
        case 1:
            RM_Heading=2;
            break;
        case 2:
            RM_Heading=3;
            break;
        case 3:
            RM_Heading=4;
            break;
        case 4:
            RM_Heading=1;
            break;
        }
    }

    //The robot turns counter clockwise
    void RM_TurnLeft()
    {
        switch(RM_Heading)
        {
        case 1:
            RM_Heading=4;
            break;
        case 2:
            RM_Heading=1;
            break;
        case 3:
            RM_Heading=2;
            break;
        case 4:
            RM_Heading=3;
            break;
        }
    }

    //The robot turns 180 degrees
    void RM_FullTurn()
    {
        switch(RM_Heading)
        {
        case 1:
            RM_Heading=3;
            break;
        case 2:
            RM_Heading=4;
            break;
        case 3:
            RM_Heading=1;
            break;
        case 4:
            RM_Heading=2;
            break;
        }
    }

    //The robot moves forward one unit
    void RM_MoveForward()
    {
        logVar.clear();

        if(RM_Heading==2)
        {
            RM_XCoord++;
        }
        else if(RM_Heading==4)
        {
            RM_XCoord--;
        }
        else if(RM_Heading==1)
        {
            RM_YCoord++;
        }
        else if(RM_Heading==3)
        {
            RM_YCoord--;
        }
        else
        {
            logVar = "cls\r";
            printLog();
            logVar = "Invalid input\r";
            printLog();
        }
    }

    //The robot moves forward two units
    void RM_MoveForwardDouble()
    {
        RM_MoveForward();
        RM_MoveForward();
    }

    //The robot moves backward one unit
    void RM_MoveBackward()
    {
        logVar.clear();

        if(RM_Heading==2)
        {
            RM_XCoord--;
        }
        else if(RM_Heading==4)
        {
            RM_XCoord++;
        }
        else if(RM_Heading==1)
        {
            RM_YCoord--;
        }
        else if(RM_Heading==3)
        {
            RM_YCoord++;
        }
        else
        {
            logVar = "cls\r";
            printLog();
            logVar = "Invalid input\r";
            printLog();
        }
    }

    //The robot moves forward three unit
    void RM_Sprint()
    {
        RM_MoveForward();
        RM_MoveForward();
        RM_MoveForward();
    }


    //-----------------------------
    //---Location and Heading---
    //-----------------------------

    //Prints position and heading
    void RM_EchoPositionAndHeading()
    {
        char direction;
        stringBuilder.clear();
        logVar.clear();

        switch(RM_Heading)
        {
        case 1:
            direction='N';
            break;
        case 2:
            direction='E';
            break;
        case 3:
            direction='S';
            break;
        case 4:
            direction='W';
            break;
        }
        logVar = "cls";
        printLog();
        stringBuilder << "Heading: " << direction << ". Position (x,y): " << RM_XCoord << ", " << RM_YCoord << ".                                        \r";
        logVar = stringBuilder.str();
        printLog();
    }

    //----------------------
    //---Mining Functions---
    //----------------------

    //The robot moves from entrance to mineral deposit
    void MoveFromEntranceToMinerals()
    {
        logVar.clear();

        logVar = "cls";
        printLog();
        logVar = "Moving to mineraldeposit...\r";
        printLog();
        RM_TurnRight();
        RM_Sprint();
        RM_TurnLeft();
        RM_MoveForwardDouble();
        RM_TurnLeft();
        RM_MoveForward();
        logVar = "Cart at mineraldeposit\n Traveltime: 335 sec\r";
        TimeSpent+=335;
        printLog();
    }

    //The robot moves from mineral deposit to unload point
    void MoveToUnload()
    {
        logVar.clear();

        logVar = "Moving to unload point...\r";
        printLog();
        CartTimer=0;
        RM_MoveBackward();
        RM_TurnLeft();
        RM_MoveForwardDouble();
        RM_TurnRight();
        RM_MoveForwardDouble();
        RM_TurnRight();
        RM_MoveForward();
        logVar = "Cart at unload point\n Traveltime: 350 sec\r";
        printLog();
        TimeSpent+=350;
        logVar = "Unloading cart...\r";
        printLog();
        Cart=0;
        CartFull=0;
        logVar = "Cart empty\n Time spent unloading: 70 sec\n Amount of ore unloaded: 4 units\r";
        printLog();
        OreCollected+=4;
        TimeSpent+=70;
    }

    //The robot moves from unload point to mineral deposit
    void MoveFromUnloadToMinerals()
    {
        logVar.clear();

        logVar = "Moving to mineraldeposit...\r";
        printLog();
        RM_MoveBackward();
        RM_TurnRight();
        RM_MoveForwardDouble();
        RM_TurnLeft();
        RM_MoveForwardDouble();
        RM_TurnRight();
        RM_MoveForward();
        logVar = "Cart at mineral deposit\n Traveltime: 350 sec..\r";
        printLog();
        TimeSpent+=350;
    }

    //The robot mines minerals, moves to unload point and returns to mineral deposit
    void MiningProgram()
    {
        stringBuilder.clear();
        logVar.clear();

        while(!DoneMining)
        {
            MineMinerals();
            MoveToUnload();
            MoveFromUnloadToMinerals();
            stringBuilder << "Total time spent: " << TimeSpent << ". Total amount of ore collected: " << cartFull << " units.\r";
            logVar = stringBuilder.str();
            printLog();
            DoneMining=1;
        }
    }

    //The robot mines minerals
    void MineMinerals()
    {
        logVar.clear();

        logVar = "Mining...\r";
        printLog();
        while(!IsCartFull())
        {
                Cart++;
                CartTimer+=60;
        }
        TimeSpent+=CartTimer;
    }

    //The robot checks if the cart is full
    int IsCartFull()
    {
        cartFull = 0;
        logVar.clear();

        if (Cart<4){
            CartTimer+=10;
            return 0;
        }
        else if(Cart==4){
            CartTimer+=10;
            cartFull = 4;

            return 1;
        }
        else
        {
            logVar = "***Cart error!***\r";
            printLog();
        }
    }

    //The robot mines minerals, moves to unload point and returns to mineral deposit. This is repeated for 24 hours
    void MiningProgram24H()
    {
        int done24h = 0;
        stringBuilder.clear();
        logVar.clear();

        while(!done24h)
        {
            if (TimeSpent<85176)
            {
                MineMinerals();
                MoveToUnload();
                MoveFromUnloadToMinerals();
            }
            else
            {
            logVar = "cls\r";
            printLog();
            stringBuilder << "Total time spent: " << TimeSpent << ". Total amount of ore collected: " << OreCollected << " units.\r";
            logVar = stringBuilder.str();
            printLog();
            done24h=1;
            }
        }

    }

    //-------------------------
    //---Misc SubRoutines---
    //-------------------------

    //User interface
    void SetProgramToExecute()
    {
        while(!done)
        {
            DoneMining = 0;
            int selection = 0;
            TimeSpent = 0;
            OreCollected = 0;
            logVar.clear();

        
            system("clear");
            std::cout << "***ABD inc mining mannager***" << std::endl;
            std::cout << "Choose function and press enter:" << std::endl;
            std::cout << "1: Initiate mining starting from entrance" << std::endl;
            std::cout << "2: Initiate mining starting from mineral deposit" << std::endl;
            std::cout << "3: Initiate 24 hour mining starting from entrance" << std::endl;
            std::cout << "4: Initiate 24 hour mining starting from mineral deposit" << std::endl;
            std::cout << "5: Echo position and heading" << std::endl;
            std::cout << "0: Exit menu" << std::endl;
            std::cin >> selection;

            switch(selection)
            {
            case 1:
                // Makes the robot move from entrance to mineral deposit, mine minerals, move to unload point,
                // unload and return to mineral deposit
                system("clear");
                std::cout << "Mining ..." << std::endl;
                logVar = "cls";
                printLog();
                MoveFromEntranceToMinerals();
                MiningProgram();
                std::cout << "Done" << std::endl;
                std::cout << "Press any key to return to menu" << std::endl;
                pause();
                break;
            case 2:
                // Makes the robot mine minerals, move to unload point, unload and return to mineral deposit
                system("clear");
                std::cout << "Mining ..." << std::endl;
                logVar = "cls";
                printLog();
                MiningProgram();
                std::cout << "Done" << std::endl;
                std::cout << "Press any key to return to menu" << std::endl;
                pause();
                break;
            case 3:
                // Makes the robot move from entrance to mineral deposit, mine minerals, move to unload point,
                // unload and return to mineral deposit. This cycle repeats for 24 hrs
                system("clear");
                std::cout << "Mining ..." << std::endl;
                logVar = "cls";
                printLog();
                MoveFromEntranceToMinerals();
                MiningProgram24H();
                std::cout << "Done" << std::endl;
                std::cout << "Press any key to return to menu" << std::endl;
                pause();
                break;
            case 4:
                // Makes the robot mine minerals, move to unload point, unload and return to mineral deposit. This cycle repeats for 24 hrs
                system("clear");
                std::cout << "Mining ..." << std::endl;
                logVar = "cls";
                printLog();
                MiningProgram24H();
                std::cout << "Done" << std::endl;
                std::cout << "Press any key to return to menu" << std::endl;
                pause();
                break;
            case 5:
                // Publishes position and heading on logOutput
                system("clear");
                logVar = "cls";
                printLog();
                RM_EchoPositionAndHeading();
                std::cout << "Press any key to return to menu" << std::endl;
                pause();
                break;
            case 0:
                // Ends while loop
                system("clear");
                done=1;
                break;
            default:
                // Catches any unknown input
                std::cout <<    "Unknown input\n" <<
                                "Press any key to return to menu\n";
                pause();
            }
        }
    }

    // Pauses program execution untill uses input is recieved
    void pause() 
    {
        std::cin.clear();
        std::cin.ignore().get();
    }
    
    // Publisher function (Places content of logVar in logOutput.comm and publishes once via mine_pub)
    void printLog()
    {
        rpro_mini_project::logOutput log;

        log.comm = logVar;

        mine_pub.publish(log);

        ros::spinOnce;
    }     


    ////////////////////////////////////////////////////////////////////////////////////////////////

    // Corona distance added to constructor

    ////////////////////////////////////////////////////////////////////////////////////////////////


    // Constructor for the MiningCart class            
    public:
    MiningCart() :

    // Nodehandle initializing
    nh(),

    // Initalization of global values
    CartTimer(0),
    done(0),
    DoneMining(0),
    Cart(0),
    CartFull(0),
    TimeSpent(0),
    OreCollected(0),
       
    RM_XCoord(0),
    RM_YCoord(0),
    RM_Heading(1)

    {
        // Publisher (Advertises publication on nodehandle)
        mine_pub = nh.advertise<rpro_mini_project::logOutput>("miningLog",10);

        std::cout << "MiningCart class initialized" << std::endl;
        SetProgramToExecute();

    }
    
    // Deconstrcutor for the MiningCart class
    ~MiningCart() {}
    };
} /*namespace*/

int main(int argc, char** argv)
{
    // Initialiser for ROS
    ros::init(argc, argv, "MiningCart");

    // Instantiation of MiningCart class
    mine_operation::MiningCart node1;
    
    // Returnvalue of main-function
    return 0;
}