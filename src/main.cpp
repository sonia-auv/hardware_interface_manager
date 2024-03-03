#include "hardware_interface_manager/RS485Interface.h"
#include <stdlib.h>
#include <iostream>

int main(int argc, char const *argv[])
{
    if (argc < 2) 
    {
        return EXIT_FAILURE;
    }   
    sonia_hw_interface::RS485Interface interface;
    if (!interface.OpenPort())
    {
        std::cout << "Could not open port..." << std::endl;
        return EXIT_FAILURE;
    }

    switch (*argv[1])
    {
    case 'd':
        //Dropper
        std::cout << "Dropper" << std::endl;
        break;
    default:
        std::cout << "Command not registered" << std::endl;
        break;
    }


    return EXIT_FAILURE;
}
