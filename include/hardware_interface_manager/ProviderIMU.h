#pragma

#include <stdio.h>
#include <functional>
#include <tuple>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "sonia_common_cpp/SerialConn.h"

namespace provider_IMU
{
    class ProviderIMU
    {
        public:
            ProviderIMU();
            ~ProviderIMU();

            void spin();
        private:
            sonia_common_cpp::SerialConn _rs485Connection;
    }
}