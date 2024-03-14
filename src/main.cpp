#include "hardware_interface_manager/RS485Interface.h"
#include <stdlib.h>
#include <iostream>
#include <chrono>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto interface = std::make_shared<sonia_hw_interface::RS485Interface>();
    if (!interface->OpenPort())
    {   
        std::cout << "Could not open port..." << std::endl;
        return EXIT_FAILURE;
    }

    
    rclcpp::spin(interface);

    rclcpp::shutdown();

    interface->Kill();

    return EXIT_SUCCESS;
}
