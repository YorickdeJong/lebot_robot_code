#include "generalInterface.h"


GeneralInterface::GeneralInterface()
{ 
}

void GeneralInterface::OptionsMenu()
{
    bool quitHomeScreen = false;
    while (!quitHomeScreen)
    {    
        int choice;
        std::cout << std::endl;
        std::cout << std::endl;
        std::cout << "[Home Screen Options]" << std::endl;
        std::cout << "1: Calibrate" << std::endl;
        std::cout << "2: Camera Data And Raw Lidar Data" << std::endl;    
        std::cout << "3: Camera Data With Object Detection And Raw Lidar Data" << std::endl; 
        std::cout << "4: Camera Data And Adjusted Lidar Data" << std::endl;
        std::cout << "5: Camera Data With Object Detection And Adjusted Lidar Data" << std::endl;
        std::cout << "6: Camera Data with QR Code Detection And Raw Lidar Data" << std::endl;    
        std::cout << "7: Test Program" << std::endl;
        std::cout << "8: Initialize Race" << std::endl;
        std::cout << "9: Quit" << std::endl;
        std::cout << "10: Test client" << std::endl;

        std::cout << std::endl;
        std::cout << "Your Choice: ";
        std::cin >> choice; 

        switch(choice)
        {
            case 1:
            {
                system(CALIBRATE);
                system("rosnode kill --all"); //kill all nodes if user wants to choose this option again without exiting the program
                break;
            }
            case 2:
            {
                system(CAMLIDAR);
                system("rosnode kill --all");
                break;
            }
            case 3: 
            {
                system(CAMDETECTIONLIDAR);
                system("rosnode kill --all");
                break;
            }
            case 4: 
            {
                system(CAMADJUSTEDLIDAR);
                system("rosnode kill --all");
                break;
            }
            case 5: 
            {
                system(CAMDETECTIONADJUSTEDLIDAR);
                system("rosnode kill --all");
                break;
            }      
            case 6:
            {
                system(CAMQRCODELIDAR);
                system("rosnode kill --all");
                break;
            }    
            case 7:
            {
                system(DRIVER);
                system("rosnode kill --all");
                break;
            }
            case 8:
            {
                system(RACETEST); 
                system("rosnode kill --all");
                break;
            }
            case 9:
            {
                quitHomeScreen = true; 
                system("rosnode kill --all");
                std::cout << "Quiting program" << std::endl;
                break;
            }
            case 10:
            {
                system(TEST); 
                system("rosnode kill --all");
                std::cout << "Quiting program" << std::endl;
                break;
            }
        }
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "generalInterface");
    GeneralInterface generalInterface;
    generalInterface.OptionsMenu();
}