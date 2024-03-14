#include "hardware_interface_manager/RS485Interface.h"
#include <chrono>
using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

namespace sonia_hw_interface
{

    RS485Interface::RS485Interface()
        : Node("rs485_interface"), _rs485_connection("/dev/RS485", B115200, false), _thread_control(true)
    {
        _reader = std::thread(std::bind(&RS485Interface::readData, this));
        _writer = std::thread(std::bind(&RS485Interface::writeData, this));
        _parser = std::thread(std::bind(&RS485Interface::parseData, this));
        _publisher_kill = this->create_publisher<sonia_common_ros2::msg::KillStatus>("/kill/tx", 10);
        _publisher_mission = this->create_publisher<sonia_common_ros2::msg::MissionStatus>("/mission/tx", 10);
        _dropper_server = this->create_service<sonia_common_ros2::srv::DropperService>("actuate_dropper", std::bind(&RS485Interface::processDropperRequest, this, _1, _2));
        _kill_mission_timer = this->create_wall_timer(500ms, std::bind(&RS485Interface::pollKillMission, this));
    }

    // node destructor
    RS485Interface::~RS485Interface()
    {
    }

    bool RS485Interface::OpenPort()
    {
        bool res = _rs485_connection.OpenPort();
        if (res){
            _rs485_connection.Flush();
        }
        return res;
    }

    void RS485Interface::pollKillMission()
    {
        _rs485_connection.Transmit(_GET_KILL_STATUS_MSG, 8);
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        _rs485_connection.Transmit(_GET_MISSION_STATUS_MSG, 8);
    }

    uint16_t RS485Interface::checkSum(uint8_t slave, uint8_t cmd, uint8_t nbByte, std::vector<uint8_t> data)
    {
        uint16_t check = (uint16_t)(0x3A + slave + cmd + nbByte + 0x0D);
        for (uint8_t i = 0; i < nbByte; i++)
        {
            check += (uint8_t)data[i];
        }
        return check;
    }

    void RS485Interface::Kill()
    {
        _thread_control = false;
    }

    void RS485Interface::processTx(const sonia_common_ros2::msg::SerialMessage &data) const
    {
        std::cout << (int)data.id << std::endl;
    }

    void RS485Interface::processDropperRequest(const std::shared_ptr<sonia_common_ros2::srv::DropperService::Request> request, std::shared_ptr<sonia_common_ros2::srv::DropperService::Response> response)
    {
        ssize_t transmit_status;
        uint16_t checksum = checkSum(_SlaveId::SLAVE_IO, _Cmd::CMD_IO_DROPPER_ACTION, 1, std::vector<uint8_t>{request->side});
        uint8_t checksum_1 = checksum >> 8;
        uint8_t checksum_2 = checksum & 0xFF;
        const uint8_t dropper[8] = {_START_BYTE, _SlaveId::SLAVE_IO, _Cmd::CMD_IO_DROPPER_ACTION, 1, request->side, checksum_1, checksum_2, _END_BYTE};
        transmit_status = _rs485_connection.Transmit(dropper, 8);
        response->result = transmit_status;
    }

    void RS485Interface::processKill(bool status)
    {
        sonia_common_ros2::msg::KillStatus state;
        state.status = status;
        _publisher_kill->publish(state);
    }

    void RS485Interface::processMission(bool status)
    {
        sonia_common_ros2::msg::MissionStatus state;
        state.status = status;
        _publisher_mission->publish(state);
    }

    // no need.
    void RS485Interface::readData()
    {
        uint8_t data[1024];
        while (_thread_control)
        {
            // This sleep is needed... I DON"T KNOW WHY...
            std::this_thread::sleep_for(std::chrono::milliseconds(300));
            ssize_t str_len = _rs485_connection.ReadPackets(1024, data);
            
            if (str_len != -1)
            {
                for (ssize_t i = 0; i < str_len; i++)
                {
                    _parseQueue.push_back((uint8_t)data[i]);
                }
            }
        }
    }

    void RS485Interface::writeData()
    {
        // close the thread.
        while (_thread_control)
        {
            // pause the thread.
            while (!_writerQueue.empty())
            {
                queueObject msg = _writerQueue.get_n_pop_front(); // TODO Replace sonia_common

                const size_t data_size = msg.data.size() + 7;
                uint8_t *data = new uint8_t[data_size];
                data[0] = _START_BYTE;
                data[1] = msg.slave;
                data[2] = msg.cmd;
                data[3] = (uint8_t)msg.data.size();

                for (int i = 0; i < data[3]; i++)
                {
                    data[i + 4] = msg.data[i];
                }

                uint16_t checksum = checkSum(data[1], data[2], data[3], (std::vector<uint8_t>)data[4]);

                data[data_size - 3] = (uint8_t)(checksum >> 8);
                data[data_size - 2] = (uint8_t)(checksum & 0xFF);
                data[data_size - 1] = (uint8_t)(0x0D);

                _rs485_connection.Transmit((const uint8_t *)data, data_size);
                delete data;
            }
        }
    }

    void RS485Interface::parseData()
    {

        while (_thread_control)
        {

            // read until the start there or the queue is empty
            while (!_parseQueue.empty())
            {
                // check if the bit is the start bit:
                if (_parseQueue.front() != _START_BYTE)
                {
                    _parseQueue.pop_front();
                }
                else
                {
                    queueObject msg;

                    // pop the unused start data
                    _parseQueue.pop_front();

                    msg.slave = _parseQueue.get_n_pop_front();
                    msg.cmd = _parseQueue.get_n_pop_front();
                    uint8_t nbByte = _parseQueue.get_n_pop_front();

                    for (int i = 0; i < nbByte; i++)
                    {
                        msg.data.push_back(_parseQueue.get_n_pop_front());
                    }

                    uint16_t checkResult = (uint16_t)(_parseQueue.get_n_pop_front() << 8);
                    checkResult += _parseQueue.get_n_pop_front();

                    // pop the unused end data
                    _parseQueue.pop_front();

                    uint16_t calc_checksum = checkSum(msg.slave, msg.cmd, nbByte, msg.data);
                    // std::cout << (int)msg.slave << std::endl;
                    // std::cout << (int)msg.cmd << std::endl;
                    // std::cout << (int)msg.data[0] << std::endl;

                    // if the checksum is bad, drop the packet
                    if (checkResult == calc_checksum)
                    {
                        // publisher.publish(msg);
                        switch (msg.slave)
                        {
                        case _SlaveId::SLAVE_KILLMISSION:
                            switch (msg.cmd)
                            {
                            case _Cmd::CMD_KILL:
                                // get data value
                                // publish on kill publisher
                                processKill(msg.data[0] == 1);
                                break;
                            case _Cmd::CMD_MISSION:
                                // get data value
                                // publish on mission publisher
                                processMission(msg.data[0] == 1);
                                break;
                            default:
                                break;
                            }
                            break;

                        default:
                            break;
                        }
                    }
                    // packet dropped
                }
            }
        }
    }
}
