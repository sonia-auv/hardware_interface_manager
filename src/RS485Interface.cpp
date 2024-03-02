#include "RS485Interface.h"

namespace sonia_hw_interface {

    const int DATA_READ_CHUNCK = 8192;

    RS485Interface::RS485Interface()
    :_serial_connection("/dev/RS485", B115200, false), _thread_control(true)
    {
        _reader = std::thread(std::bind(&RS485Interface::readData, this));
        _writer = std::thread(std::bind(&RS485Interface::writeData, this));
        _parser = std::thread(std::bind(&RS485Interface::parseData, this));
    }
    
    //node destructor
    RS485Interface::~RS485Interface()
    {

    }

    bool RS485Interface::OpenPort()
    {
        return _serial_connection.OpenPort();
    }

    uint16_t RS485Interface::checksum(uint8_t slave, uint8_t cmd, uint8_t nbByte, uint8_t* data)
    {
        uint16_t check= (uint16_t)(0x3A+slave+cmd+nbByte+00D);
        for(uint8_t i=0; i<nByte; i++)
        {
            check+= (uint8_t)data[i];
        }
        return check;
    }

    void RS485Interface::readData()
    {
        char data[DATA_READ_CHUNCK];
        while(!ros::isShuttingDown())
        {
            ros::Duration(sleepTime).sleep();
            ssize_t str_len = serialConnection.receive(data, dataReadChunk);

            if(str_len != -1)
            {
                for(ssize_t i = 0; i < str_len; i++)
                {
                    parseQueue.push_back((uint8_t) data[i]);
                }
            }
        }
    }

    void RS485Interface::writeData()
    {
        ROS_INFO("begin the write data threads");
        while(_thread_control)
        {
            ros::Duration(sleepTime).sleep();
            while(!writerQueue.empty())
            {
                sonia_common::SendRS485Msg::ConstPtr msg_ptr = writerQueue.get_n_pop_front();

                size_t data_size = msg_ptr->data.size() + 7;
                uint8_t data[data_size];
                data[0] = 0x3A;
                data[1] = msg_ptr->slave;
                data[2] = msg_ptr->cmd;
                data[3] = (uint8_t)msg_ptr->data.size();

                for(int i = 0; i < data[3]; i++)
                {
                    data[i+4] = msg_ptr->data[i];
                }

                uint16_t checksum = calculateCheckSum(data[1], data[2], data[3], (uint8_t*) &data[4]);

                data[data_size-3] = (uint8_t)(checksum >> 8);
                data[data_size-2] = (uint8_t)(checksum & 0xFF);
                data[data_size-1] = 0x0D;

                ROS_DEBUG("%0x\n%0x\n%0x\n%0x\n%0x\n%0x\n%0x\n%0x\n", data[0],data[1],data[2],data[3],data[4],data[5],data[6],data[7]);

                serialConnection.transmit((const uint8_t*)data, data_size);
            }
        }
    }

    // void RS485Interface::parseData()
    // {
    //     ROS_INFO("begin the parse data threads");
    //     while(_thread_control)
    //     {
    //         ros::Duration(sleepTime).sleep();
    //         //read until the start there or the queue is empty
    //         while(!parseQueue.empty())
    //         {
    //             if(parseQueue.front() != 0x3A)
    //             {
    //                 parseQueue.pop_front();
    //             }
    //             else
    //             {
    //                 sonia_common::SendRS485Msg msg = sonia_common::SendRS485Msg();

    //                 //pop the unused start data
    //                 parseQueue.pop_front();

    //                 msg.slave = parseQueue.get_n_pop_front();
    //                 msg.cmd = parseQueue.get_n_pop_front();
    //                 unsigned char nbByte = parseQueue.get_n_pop_front();

    //                 for(int i = 0; i < nbByte; i++)
    //                 {
    //                     msg.data.push_back(parseQueue.get_n_pop_front());
    //                 }

    //                 uint16_t checksum = (uint16_t)(parseQueue.get_n_pop_front()<<8);
    //                 checksum += parseQueue.get_n_pop_front();

    //                 //pop the unused end data
    //                 parseQueue.pop_front();

    //                 uint16_t calc_checksum = calculateCheckSum(msg.slave, msg.cmd, nbByte, msg.data);

    //                 // if the checksum is bad, drop the packet
    //                 if(checksum == calc_checksum)
    //                 {
    //                     publisher.publish(msg);
    //                 }
    //             }
    //         }
    //     }
    // }
}
