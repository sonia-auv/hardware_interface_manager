#pragma once

#include <thread>
#include <stdio.h>
#include <functional>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "sonia_common_cpp/SerialConn.h"
#include "sonia_common_ros2/msg/SerialMessage.hpp"
#include "SharedQueue.h"

namespace sonia_hw_interface {
    struct queueObject{
        uint8_t slave;
        uint8_t cmd;
        std::vector<uint8_t> data;
    };
    

    class RS485Interface : public rclcpp::Node
    {
        public:
            RS485Interface();
            ~RS485Interface();

            bool OpenPort();
            void Kill();


        private:
            uint16_t checkSum(uint8_t slave, uint8_t cmd, uint8_t nbByte, std::vector<uint8_t> data);

            //void receivedData(const sonia_common::SendRS485Msg::ConstPtr &recievedData);
            void readData();
            void writeData();
            void parseData();
            // TODO add type
            // void processTx(data);

            // double sleepTime;
            sonia_cpp::SerialConn _serial_connection;

            rclcpp::Subscription<sonia_common_ros2::msg::SerialMessage> _subscriber;

            std::thread _reader;
            std::thread _parser;
            std::thread _writer;

            SharedQueue<queueObject> writerQueue;
            SharedQueue<uint8_t> parseQueue;

            bool _thread_control;

            static const int DATA_READ_CHUNCK = 8192;
            const u_int8_t START_BIT = 0x3A;
    };

}