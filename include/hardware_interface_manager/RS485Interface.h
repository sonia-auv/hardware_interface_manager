#pragma once

#include <thread>
#include <stdio.h>
#include <functional>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "sonia_common_cpp/SerialConn.h"
#include "sonia_common_ros2/msg/serial_message.hpp"
#include "sonia_common_ros2/srv/dropper_service.hpp"
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
            enum _SlaveId: uint8_t {
                SLAVE_PSU0 = 0,
                SLAVE_PSU1 = 1,
                SLAVE_PSU2 = 2,
                SLAVE_PSU3 = 3,
                SLAVE_KILLMISSION = 4,
                SLAVE_ESC = 5,
                SLAVE_IO = 6,
                SLAVE_STATE_SCREEN = 7,
                SLAVE_PWR_MANAGEMENT = 8,
            };
            enum _Cmd: uint8_t {
                CMD_MISSION = 0,
                CMD_KILL = 1,
                CMD_VOLTAGE = 0,
                CMD_CURRENT = 1,
                CMD_TEMPERATURE = 2,
                CMD_READ_MOTOR = 15,
                CMD_ACT_MOTOR = 16,
                CMD_PWM = 17,
                CMD_IO_TEMP= 0,
                CMD_IO_DROPPER_ACTION = 1,
                CMD_IO_TORPEDO_ACTION = 2,
                CMD_IO_ARM_ACTION = 3,
                CMD_IO_LEAK_SENSOR = 4,
                CMD_KEEP_ALIVE = 30,
            };
            uint16_t checkSum(uint8_t slave, uint8_t cmd, uint8_t nbByte, std::vector<uint8_t> data);

            //void receivedData(const sonia_common::SendRS485Msg::ConstPtr &recievedData);
            void readData();
            void writeData();
            void parseData();
            // TODO add type
            
            /**
            
            */
            void processTx(const sonia_common_ros2::msg::SerialMessage &data) const;
            void processDropperRequest(const std::shared_ptr<sonia_common_ros2::srv::DropperService::Request> request, std::shared_ptr<sonia_common_ros2::srv::DropperService::Response> response);
            
            // double sleepTime;
            sonia_common_cpp::SerialConn _serial_connection;

            rclcpp::Subscription<sonia_common_ros2::msg::SerialMessage>::SharedPtr _subscriber;
            rclcpp::Service<sonia_common_ros2::srv::DropperService>::SharedPtr _dropper_server;

            std::thread _reader;
            std::thread _parser;
            std::thread _writer;

            SharedQueue<queueObject> _writerQueue;
            SharedQueue<uint8_t> _parseQueue;

            bool _thread_control;

            static const int DATA_READ_CHUNCK = 8192;
            const u_int8_t _START_BYTE = 0x3A;
            const u_int8_t _END_BYTE = 0x0D;
    };

}