#include <chrono>

#include "hardware_interface_manager/RS485Interface.h"

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

namespace sonia_hw_interface
{

    RS485Interface::RS485Interface()
        : Node("rs485_interface"), _rs485Connection("/dev/RS485", B115200, false), _thread_control(true)
    {
        _reader = std::thread(std::bind(&RS485Interface::readData, this));
        _writer = std::thread(std::bind(&RS485Interface::writeData, this));
        _parser = std::thread(std::bind(&RS485Interface::parseData, this));
        _publisherKill = this->create_publisher<sonia_common_ros2::msg::KillStatus>("/provider_rs485/kill_status", 10);
        _publisherMission = this->create_publisher<sonia_common_ros2::msg::MissionStatus>("/provider_rs485/mission_status", 10);
        _publisherMotorVoltages = this->create_publisher<sonia_common_ros2::msg::MotorVoltages>("/provider_power/motor_voltages", 10);
        _publisherBatteryVoltages = this->create_publisher<sonia_common_ros2::msg::BatteryVoltage>("/provider_power/battery_voltages", 10);
        _dropperServer = this->create_service<sonia_common_ros2::srv::DropperService>("actuate_dropper", std::bind(&RS485Interface::processDropperRequest, this, _1, _2));
        _timerKillMission = this->create_wall_timer(500ms, std::bind(&RS485Interface::pollKillMission, this));
    }

    // node destructor
    RS485Interface::~RS485Interface()
    {
    }

    bool RS485Interface::OpenPort()
    {
        bool res = _rs485Connection.OpenPort();
        if (res)
        {
            _rs485Connection.Flush();
        }
        return res;
    }

    void RS485Interface::pollKillMission()
    {
        // Transmit request to get kill status
        _rs485Connection.Transmit(_GET_KILL_STATUS_MSG, 8);
        // Wait for a short duration to allow for processing
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        // Transmit request to get mission status
        _rs485Connection.Transmit(_GET_MISSION_STATUS_MSG, 8);
    }

    std::tuple<uint8_t, uint8_t> RS485Interface::checkSum(uint8_t slave, uint8_t cmd, uint8_t nbByte, std::vector<uint8_t> data)
    {
        uint16_t check = (uint16_t)(_START_BYTE + slave + cmd + nbByte + _END_BYTE);
        for (uint8_t i = 0; i < nbByte; i++)
        {
            check += (uint8_t)data[i];
        }
        return {check >> 8, check & 0XFF};
    }

    void RS485Interface::Kill()
    {
        _thread_control = false;
    }

    void RS485Interface::processDropperRequest(const std::shared_ptr<sonia_common_ros2::srv::DropperService::Request> request, std::shared_ptr<sonia_common_ros2::srv::DropperService::Response> response)
    {
        // Variables for transmission status and data vector
        ssize_t transmit_status;
        std::vector<uint8_t> data_vec;

        // Add dropper side to data vector
        data_vec.push_back(request->side);

        // Calculate checksum for the command
        std::tuple<uint8_t, uint8_t> checksum = checkSum(_SlaveId::SLAVE_IO, _Cmd::CMD_IO_DROPPER_ACTION, data_vec.size(), data_vec);

        // Construct the command packet
        const uint8_t dropper[8] = {_START_BYTE, _SlaveId::SLAVE_IO, _Cmd::CMD_IO_DROPPER_ACTION, 1, request->side, std::get<0>(checksum), std::get<1>(checksum), _END_BYTE};

        transmit_status = _rs485Connection.Transmit(dropper, 8);
        response->result = transmit_status;
    }

    void RS485Interface::publishKill(bool status)
    {
        sonia_common_ros2::msg::KillStatus state;
        state.status = status;
        _publisherKill->publish(state);
    }

    void RS485Interface::publishMission(bool status)
    {
        sonia_common_ros2::msg::MissionStatus state;
        state.status = status;
        _publisherMission->publish(state);
    }

    void RS485Interface::publishMotorVoltages(std::vector<float> data)
    {
        if (data.size() != 8)
        {
            return;
        }
        sonia_common_ros2::msg::MotorVoltages msg;
        msg.motor1 = data[0];
        msg.motor2 = data[1];
        msg.motor3 = data[2];
        msg.motor4 = data[3];
        msg.motor5 = data[4];
        msg.motor6 = data[5];
        msg.motor7 = data[6];
        msg.motor8 = data[7];

        _publisherMotorVoltages->publish(msg);
    }

    void RS485Interface::publishBatteryVoltages(float *data)
    {
        sonia_common_ros2::msg::BatteryVoltage msg;
        msg.battery1 = data[0];
        msg.battery2 = data[1];

        _publisherBatteryVoltages->publish(msg);
    }

    void RS485Interface::processPowerManagement(uint8_t cmd, std::vector<uint8_t> data)
    {
        std::vector<float> voltageData;

        switch (cmd)
        {
        case _Cmd::CMD_VOLTAGE:

            if (convertBytesToFloat(data, voltageData) < 0 || voltageData.size() != _EXPECTED_PWR_VOLT_SIZE)
            {
                std::cerr << "ERROR in the message. Dropping VOLTAGE packet" << std::endl;
                return;
            }

            float batteryData[2];
            batteryData[0] = voltageData[voltageData.size() - 2];
            batteryData[1] = voltageData[voltageData.size() - 1];
            voltageData.pop_back();
            voltageData.pop_back();

            publishMotorVoltages(voltageData);
            publishBatteryVoltages(batteryData);

            break;

        default:
            break;
        }
    }

    void RS485Interface::readData()
    {
        uint8_t data[_DATA_READ_CHUNCK];
        while (_thread_control)
        {
            // This sleep is needed... I DON'T KNOW WHY...
            std::this_thread::sleep_for(std::chrono::milliseconds(300));
            ssize_t str_len = _rs485Connection.ReadPackets(_DATA_READ_CHUNCK, data);

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
                std::this_thread::sleep_for(std::chrono::milliseconds(300));

                queueObject msg = _writerQueue.get_n_pop_front();
                const size_t data_size = msg.data.size() + 7;
                uint8_t *data = new uint8_t[data_size];
                data[0] = _START_BYTE;
                data[1] = msg.slave;
                data[2] = msg.cmd;
                data[3] = (uint8_t)msg.data.size();

                std::vector<uint8_t> data_vec;
                for (int i = 0; i < data[3]; i++)
                {
                    data[i + 4] = msg.data[i];
                    data_vec.push_back(msg.data[i]);
                }

                std::tuple<uint8_t, uint8_t> checksum = checkSum(data[1], data[2], data[3], data_vec);

                data[data_size - 3] = std::get<0>(checksum);
                data[data_size - 2] = std::get<1>(checksum);
                data[data_size - 1] = _END_BYTE;
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

                    std::tuple<uint8_t, uint8_t> checkResult = {(_parseQueue.get_n_pop_front() << 8), _parseQueue.get_n_pop_front()};

                    // pop the unused end data
                    _parseQueue.pop_front();

                    std::tuple<uint8_t, uint8_t> calc_checksum = checkSum(msg.slave, msg.cmd, nbByte, msg.data);

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
                                publishKill(msg.data[0] == 1);
                                break;
                            case _Cmd::CMD_MISSION:
                                // get data value
                                // publish on mission publisher
                                publishMission(msg.data[0] == 1);
                                break;
                            default:
                                break;
                            }
                            break;
                        case _SlaveId::SLAVE_PWR_MANAGEMENT:
                            processPowerManagement(msg.cmd, msg.data);
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

    int RS485Interface::convertBytesToFloat(const std::vector<uint8_t> &req, std::vector<float> &res)
    {
        uint8_t size_req = req.size();
        if (size_req % 4 != 0)
            return -1;

        _bytesToFloat converter;

        for (uint8_t i = 0; i < req.size(); ++i) // shifting of 4 for each data
        {
            converter.bytes[0] = req[4 * i];
            converter.bytes[1] = req[4 * i + 1];
            converter.bytes[2] = req[4 * i + 2];
            converter.bytes[3] = req[4 * i + 3];
            res.push_back(converter.value);
        }
        return 0;
    }
}
