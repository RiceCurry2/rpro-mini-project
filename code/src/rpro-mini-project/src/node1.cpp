#include "ros/ros.h"
#include "geometry_msgs/Point.h" // Used in dummy function

namespace mine_operation {

    class MiningCart
    {
    private:
	ros::NodeHandle n_;
    ros::Publisher mine_pub; // Klargøring til subscriber (I må selv lige ændre navnet)

    //----------------------//----------------------//----------------------//----------------------//----------------------//
    //---Global Variables---//---Global Variables---//---Global Variables---//---Global Variables---//---Global Variables---//
    //----------------------//----------------------//----------------------//----------------------//----------------------//

    int RM_Heading;
    int RM_XCoord;
    int RM_YCoord;
    int RM_Done;

    int CartTimer;
    int done;
    int Choice;
    int DoneMining;
    int Cart;
    int CartFull;
    int TimeSpent;
    int OreCollected;

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // dobRM below this line

    //-----------------
    //---Movement---
    //-----------------

    //The robot turns clockwise
    void RM_TurnRight(){
        switch(RM_Heading){
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
    void RM_TurnLeft(){
        switch(RM_Heading){
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
    void RM_FullTurn(){
        switch(RM_Heading){
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
    void RM_MoveForward(){
        if(RM_Heading==2){
            RM_XCoord++;
        }
        else if(RM_Heading==4){
            RM_XCoord--;
        }
        else if(RM_Heading==1){
            RM_YCoord++;
        }
        else if(RM_Heading==3){
            RM_YCoord--;
        }
        else
            puts("Ugyldigt input");
    }

    //The robot moves forward two units
    void RM_MoveForwardDouble(){
        RM_MoveForward();
        RM_MoveForward();
    }

    //The robot moves backward one unit
    void RM_MoveBackward(){
        if(RM_Heading==2){
            RM_XCoord--;
        }
        else if(RM_Heading==4){
            RM_XCoord++;
        }
        else if(RM_Heading==1){
            RM_YCoord--;
        }
        else if(RM_Heading==3){
            RM_YCoord++;
        }
        else
            puts("Ugyldigt input");
    }

    //The robot moves forward three unit
    void RM_Sprint(){
        RM_MoveForward();
        RM_MoveForward();
        RM_MoveForward();
    }


    //-----------------------------
    //---Location and Heading---
    //-----------------------------

    //Prints position and heading
    void RM_EchoPositionAndHeading(){
        char direction;

        switch(RM_Heading){
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
        printf("\rHeading %c. Placering (x,y):%d,%d",direction,RM_XCoord,RM_YCoord);
    }

    //-------------
    //---Help---
    //-------------

    //Runs help menu page 1
    void Help_RM(){
        int selection = 0;

        while(!RM_Done){
            system("cls");
            printf("dobRM.h help text, page 1 of 2\n");
            printf("Functions:\n");
            printf("1: Previous Page\n");
            printf("2: RM_Initialise()\n");
            printf("3: RM_TurnRight()\n");
            printf("4: RM_TurnLeft()\n");
            printf("5: RM_FullTurn()\n");
            printf("6: RM_MoveForward()\n");
            printf("7: RM_MoveForwardDouble()\n");
            printf("8: RM_MoveBackward()\n");
            printf("9: Next Page\n");
            printf("0: Exit Menu\n");
            printf("Enter selection to read more...\n");
            scanf("%d",&selection);

            switch(selection){
                case 1:
                    system("cls");
                    printf("No previous page\n");
                    printf("press any key to return to menu...\n");
                    getch();
                    break;
                case 2:
                    system("cls");
                    printf("Name:   RM_Initialize\n");
                    printf("Class:  Function\n");
                    printf("Input:  None\n");
                    printf("Output: None\n");
                    printf("Effect: Sets up local and global variables\n");
                    printf("Att.:   Must run before any movements can be carried out\n");
                    printf("press any key to return to menu...");
                    getch();
                    break;
                case 3:
                    system("cls");
                    printf("Name:   RM_TurnRight\n");
                    printf("Class:  Function\n");
                    printf("Input:  None\n");
                    printf("Output: None\n");
                    printf("Effect: Changes the robots heading a quarter circle to the right\n");
                    printf("Att.:   Only works, if the robots is facing along an x-axis or y-axis\n");
                    printf("press any key to return to menu...");
                    getch();
                    break;
                case 4:
                    system("cls");
                    printf("Name:   RM_TurnLeft\n");
                    printf("Class:  Function\n");
                    printf("Input:  None\n");
                    printf("Output: None\n");
                    printf("Effect: Changes the robots heading a quarter circle to the left\n");
                    printf("Att.:   Only works, if the robots is facing along an x-axis or y-axis\n");
                    printf("press any key to return to menu...");
                    getch();
                    break;
                case 5:
                    system("cls");
                    printf("Name:   RM_FullTurn\n");
                    printf("Class:  Function\n");
                    printf("Input:  None\n");
                    printf("Output: None\n");
                    printf("Effect: Changes the robots heading by 180 degrees\n");
                    printf("Att.:   Only works, if the robots is facing along an x-axis or y-axis\n");
                    printf("press any key to return to menu...");
                    getch();
                    break;
                case 6:
                    system("cls");
                    printf("Name:   RM_MoveForward\n");
                    printf("Class:  Function\n");
                    printf("Input:  None\n");
                    printf("Output: None\n");
                    printf("Effect: Moves the robot forward one square\n");
                    printf("Att.:   Only works, if the robots is facing along an x-axis or y-axis\n");
                    printf("press any key to return to menu...");
                    getch();
                    break;
                case 7:
                    system("cls");
                    printf("Name:   RM_MoveForwardDouble\n");
                    printf("Class:  Function\n");
                    printf("Input:  None\n");
                    printf("Output: None\n");
                    printf("Effect: Moves the robot forward two squares\n");
                    printf("Att.:   Only works, if the robots is facing along an x-axis or y-axis\n");
                    printf("press any key to return to menu...");
                    getch();
                    break;
                case 8:
                    system("cls");
                    printf("Name:   RM_BackForward\n");
                    printf("Class:  Function\n");
                    printf("Input:  None\n");
                    printf("Output: None\n");
                    printf("Effect: Moves the robot backward one square\n");
                    printf("Att.:   Only works, if the robots is facing along an x-axis or y-axis\n");
                    printf("press any key to return to menu...");
                    getch();
                    break;

                case 9:
                    HelpPageTwo();
                    break;
                case 0:
                    system("cls");
                    RM_Done = 1;
                    break;
                default:
                    system("cls");
                    printf("Invalid selection\n");
                    printf("press any key to return to menu...");
                    getch();
                    break;
            }
        }
    }

    //Runs help menu page 2
    void HelpPageTwo(){
        int done=0;
        int selectiontwo=0;

        while(!done){
            system("cls");
            printf("dobRM.h help text, page 2 of 2\n");
            printf("Functions:\n");
            printf("1: Previous Page\n");
            printf("2: EchoPositionAndHeading()\n");
            printf("3: Sprint()\n");
            printf("4: ()\n");
            printf("5: ()\n");
            printf("6: ()\n");
            printf("7: ()\n");
            printf("8: ()\n");
            printf("9: Next Page\n");
            printf("0: Exit Menu\n");
            printf("Enter selection to read more...\n");
            scanf("%d",&selectiontwo);

            switch(selectiontwo){
                case 1:
                    system("cls");
                    done=1;
                    break;
                case 2:
                    system("cls");
                    printf("Name:   RM_EchoPosotionAndHeading\n");
                    printf("Class:  Function\n");
                    printf("Input:  None\n");
                    printf("Output: None\n");
                    printf("Effect: Prints the robots current position (x,y) and heading(N/E/S/W) to terminal\n");
                    printf("Att.:   None\n");
                    printf("press any key to return to menu...");
                    getch();
                    break;
                case 3:
                    system("cls");
                    printf("Name:   RM_Sprint\n");
                    printf("Class:  Function\n");
                    printf("Input:  None\n");
                    printf("Output: None\n");
                    printf("Effect: Moves the robot forward three squares\n");
                    printf("Att.:   Only works, if the robots is facing along an x-axis or y-axis\n");
                    printf("press any key to return to menu...");
                    getch();
                    break;
                case 9:
                    system("cls");
                    printf("No further pages\n");
                    printf("press any key to return to menu...\n");
                    getch();
                    break;
                case 0:
                    system("cls");
                    RM_Done=1;
                    done=1;
                    break;
            }
        }
    }


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


    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    // dobMining below this line
    ////////////////////////////////////////////////////////////////////////////////////////////////////////


    //----------------------
    //---Mining Functions---
    //----------------------

    //Robotten kører fra indgang til mineraldepot
    void MoveFromEntranceToMinerals(){
        printf("Moving to mineraldeposit...\n");
        printf("\n");
        RM_TurnRight();
        RM_Sprint();
        RM_TurnLeft();
        RM_MoveForwardDouble();
        RM_TurnLeft();
        RM_MoveForward();
        printf("Cart at mineraldeposit\n");
        printf("Traveltime: 335 sec\n");
        printf("\n");
        TimeSpent+=335;
    }

    //Robotten kører fra mineraldepot til lossepunkt
    void MoveToUnload(){
        printf("Moving to unload point...\n");
        printf("\n");
        CartTimer=0;
        RM_MoveBackward();
        RM_TurnLeft();
        RM_MoveForwardDouble();
        RM_TurnRight();
        RM_MoveForwardDouble();
        RM_TurnRight();
        RM_MoveForward();
        printf("Cart at unload point\n");
        printf("Traveltime: 350 sec\n");
        printf("\n");
        TimeSpent+=350;
        printf("Unloading cart...\n");
        printf("\n");
        Cart=0;
        CartFull=0;
        printf("Cart empty\n");
        printf("Time spent unloading: 70 sec\n");
        printf("Amount of ore unloaded: 4 units");
        OreCollected+=4;
        printf("\n");
        TimeSpent+=70;

    }

    //Robotten kører fra lossepunkt til mineraldepot
    void MoveFromUnloadToMinerals(){
        printf("Moving to mineraldeposit...\n");
        printf("\n");
        RM_MoveBackward();
        RM_TurnRight();
        RM_MoveForwardDouble();
        RM_TurnLeft();
        RM_MoveForwardDouble();
        RM_TurnRight();
        RM_MoveForward();
        printf("Cart at mineral deposit\n");
        printf("Traveltime: 350 sec\n");
        printf("\n");
        TimeSpent+=350;
    }

    //Robotten kører til og fra, samt udfører minearbejde
    void MiningProgram(){
        while(!DoneMining){
            MineMinerals();
            MoveToUnload();
            MoveFromUnloadToMinerals();
            printf("Total time spent: %d sec\n", TimeSpent);
            printf("Total amount of ore collected: 4 units\n");
            printf("Press any key to return to menu\n");
            getch();
            DoneMining=1;
        }
    }

    //Robotten kører til og fra, samt udfører minearbejde
    void MiningProgram24H(){
        int done24h=0;

        while(!done24h){
            if (TimeSpent<85176){
            MineMinerals();
            MoveToUnload();
            MoveFromUnloadToMinerals();
            }
            else{
            printf("Total time spent: %d sec\n", TimeSpent);
            printf("Total amount of ore collected: %d units\n",OreCollected);
            printf("Press any key to return to menu\n");
            getch();
            done24h=1;
            }
        }

    }

    //Robotten udfører minearbejde
    void MineMinerals(){
        while(!IsCartFull()){
                Cart++;
                CartTimer+=60;
                printf("   ***Mining***\n");
                printf("\n");
        }
        printf("Time spent mining: %d sec\n", CartTimer);
        printf("\n");
        TimeSpent+=CartTimer;
    }

    //-------------------------
    //---Misc SubRoutines---
    //-------------------------

    //Sætter programmet igang fra konsol
    void SetProgramToExecute(){
        while(!done){
            DoneMining=0;
            TimeSpent=0;
            OreCollected=0;

            system("cls");
            printf("***African blood diamonds inc mining mannager***\n");
            printf("Choose function:\n");
            printf("1: Initiate mining starting from entrance\n");
            printf("2: Initiate mining starting from mineral deposit\n");
            printf("3: Initiate 24 hour mining starting from entrance\n");
            printf("4: Initiate 24 hour mining starting from mineral deposit\n");
            printf("0: Exit menu\n");
            scanf("%d",&Choice);

            switch(Choice){
            case 1:
                system("cls");
                MoveFromEntranceToMinerals();
                MiningProgram();
                break;
            case 2:
                system("cls");
                MiningProgram();
                break;
            case 3:
                system("cls");
                MoveFromEntranceToMinerals();
                MiningProgram24H();
                break;
            case 4:
                system("cls");
                MiningProgram24H();
                break;
            case 0:
                system("cls");
                done=1;
                break;
            default:
                printf("Unknown input\n");
                printf("Press any key to return to menu\n");
                getch();
            }
        }
    }

    //Tjekker om minevognen er fuld
    int IsCartFull(){
        printf("***Checking cart***\n");
        printf("\n");
        if (Cart<4){
            CartTimer+=10;
            return 0;
        }
        else if(Cart==4){
            CartTimer+=10;
            printf("  ***Cart full***\n");
            printf("\n");
            return 1;
        }
        else
            printf("***Cart error!***\n");
    }
    

    ////////////////////////////////////////////////////////////////////////////////////////////////

    // Corona distance added to constructor

    ////////////////////////////////////////////////////////////////////////////////////////////////


    // Constructor for the MiningCart class            
    public:
    MiningCart() :

    // Nodehandle initializing
    n_(),

    // Initalization of global values
    CartTimer(0),
    done(0),
    Choice(0),
    DoneMining(0),
    Cart(0),
    CartFull(0),
    TimeSpent(0),
    OreCollected(0),
       
    RM_XCoord(0),
    RM_YCoord(0),
    RM_Heading(0)

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