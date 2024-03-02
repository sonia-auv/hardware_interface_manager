#pragma once

#include <thread>
#include <stdio.h>
#include "sonia_common_cpp/SerialConn.h"


namespace sonia_hw_interface {

    class RS485Interface {
    public:
        RS485Interface();
        ~RS485Interface();

        bool OpenPort();

    private:
        uint16_t checksum(uint8_t slave, uint8_t cmd, uint8_t nbByte, uint8_t* data);

            // void receivedData(const sonia_common::SendRS485Msg::ConstPtr &recievedData);
        void readData();
        void writeData();
        void parseData();

        // double sleepTime;

        sonia_cpp::SerialConn _serialConnection;

        std::thread _reader;
        std::thread _parser;
        std::thread _writer;
        
        bool _thread_control;

        const int DATA_READ_CHUNCK;
    };

}