#include "hardware_interface_manager/RS485Interface.h"


namespace sonia_hw_interface {

    RS485Interface::RS485Interface()
    : Node("rs485_interface"), _serial_connection("/dev/RS485", B115200, false), _thread_control(true)
    {
        _reader = std::thread(std::bind(&RS485Interface::readData, this));
        _writer = std::thread(std::bind(&RS485Interface::writeData, this));
        _parser = std::thread(std::bind(&RS485Interface::parseData, this));

        _subscriber = this->create_subscription<>("/rs485_interface/tx", 10, std::bind(&RS485Interface::processTX, this))
    }
    
    //node destructor
    RS485Interface::~RS485Interface()
    {

    }

    bool RS485Interface::OpenPort()
    {
        return _serial_connection.OpenPort();
    }

    uint16_t RS485Interface::checkSum(uint8_t slave, uint8_t cmd, uint8_t nbByte, std::vector<uint8_t> data)
    {
        uint16_t check = (uint16_t)(0x3A+slave+cmd+nbByte+0x0D);
        for(uint8_t i=0; i<nbByte; i++)
        {
            check+= (uint8_t)data[i];
        }
        return check;
    }

    void RS485Interface::Kill()
    {
        _thread_control = false;
    }

// no need.
    void RS485Interface::readData()
    {
        uint8_t data[DATA_READ_CHUNCK];
        while(_thread_control)
        {
           ssize_t str_len = _serial_connection.ReadPackets(DATA_READ_CHUNCK, data);

            if(str_len != -1)
            {
                for(ssize_t i = 0; i < str_len; i++)
                {
                    parseQueue.push_back((uint8_t)data[i]);
                }
            }
        }
    }

    void RS485Interface::writeData()
    {
    
        while(_thread_control)
        {
            while(!writerQueue.empty())
            {
                queueObject msg = writerQueue.get_n_pop_front(); //TODO Replace sonia_common

                const size_t data_size = msg.data.size() + 7;
                uint8_t *data = new uint8_t[data_size] ;
                data[0] = START_BIT;
                data[1] = msg.slave;
                data[2] = msg.cmd;
                data[3] = (uint8_t)msg.data.size();

                for(int i = 0; i < data[3]; i++)
                {
                    data[i+4] = msg.data[i];
                }

                uint16_t checksum = checkSum(data[1], data[2], data[3], (std::vector<uint8_t>) data[4]);

                data[data_size-3] = (uint8_t)(checksum >> 8);
                data[data_size-2] = (uint8_t)(checksum & 0xFF);
                data[data_size-1] = (uint8_t)(0x0D);

                _serial_connection.Transmit((const uint8_t*)data, data_size);
                delete data;
            }
        }
    }

    void RS485Interface::parseData()
    {
        
        while(_thread_control)
        {
            
            //read until the start there or the queue is empty
            while(!parseQueue.empty())
            {
                // check if the bit is the start bit:
                if(parseQueue.front() != START_BIT) 
                {
                    parseQueue.pop_front();
                }
                else
                {
                    queueObject msg;

                    //pop the unused start data
                    parseQueue.pop_front();

                    msg.slave = parseQueue.get_n_pop_front();
                    msg.cmd = parseQueue.get_n_pop_front();
                    uint8_t nbByte = parseQueue.get_n_pop_front();

                    for(int i = 0; i < nbByte; i++)
                    {
                        msg.data.push_back(parseQueue.get_n_pop_front());
                    }

                    uint16_t checkResult= (uint16_t)(parseQueue.get_n_pop_front()<<8);
                    checkResult += parseQueue.get_n_pop_front();

                    //pop the unused end data
                    parseQueue.pop_front();

                    uint16_t calc_checksum = checkSum(msg.slave, msg.cmd, nbByte, msg.data);

                    // if the checksum is bad, drop the packet
                    if(checkResult != calc_checksum)
                    {
                        //publisher.publish(msg);
                    }
                }
            }
        }
    }
}
