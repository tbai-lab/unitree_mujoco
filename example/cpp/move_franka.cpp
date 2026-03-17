#include <iostream>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/common/time/time_tool.hpp>
#include <unitree/common/thread/thread.hpp>

using namespace unitree::common;
using namespace unitree::robot;

#define TOPIC_LOWCMD "rt/lowcmd"
#define TOPIC_LOWSTATE "rt/lowstate"

constexpr double PosStopF = (2.146E+9f);
constexpr double VelStopF = (16000.0f);
constexpr int NUM_JOINTS = 8; // 7 arm + 1 gripper

class FrankaControl
{
public:
    FrankaControl(){};
    ~FrankaControl(){};
    void Init();

private:
    void InitLowCmd();
    void LowStateMessageHandler(const void *messages);
    void LowCmdWrite();

private:
    // Franka home position
    double home_pos[NUM_JOINTS] = {0.0, 0.0, 0.0, -1.57079, 0.0, 1.57079, -0.7853, 0.04};
    // Pick-ready position
    double pick_ready_pos[NUM_JOINTS] = {0.0, -0.3, 0.0, -2.0, 0.0, 2.0, 0.7854, 0.04};

    double dt = 0.002;
    double running_time = 0.0;
    double phase = 0.0;

    unitree_go::msg::dds_::LowCmd_ low_cmd{};
    unitree_go::msg::dds_::LowState_ low_state{};

    ChannelPublisherPtr<unitree_go::msg::dds_::LowCmd_> lowcmd_publisher;
    ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> lowstate_subscriber;
    ThreadPtr lowCmdWriteThreadPtr;
};

uint32_t crc32_core(uint32_t *ptr, uint32_t len)
{
    unsigned int xbit = 0;
    unsigned int data = 0;
    unsigned int CRC32 = 0xFFFFFFFF;
    const unsigned int dwPolynomial = 0x04c11db7;

    for (unsigned int i = 0; i < len; i++)
    {
        xbit = 1 << 31;
        data = ptr[i];
        for (unsigned int bits = 0; bits < 32; bits++)
        {
            if (CRC32 & 0x80000000)
            {
                CRC32 <<= 1;
                CRC32 ^= dwPolynomial;
            }
            else
            {
                CRC32 <<= 1;
            }

            if (data & xbit)
                CRC32 ^= dwPolynomial;
            xbit >>= 1;
        }
    }

    return CRC32;
}

void FrankaControl::Init()
{
    InitLowCmd();
    lowcmd_publisher.reset(new ChannelPublisher<unitree_go::msg::dds_::LowCmd_>(TOPIC_LOWCMD));
    lowcmd_publisher->InitChannel();

    lowstate_subscriber.reset(new ChannelSubscriber<unitree_go::msg::dds_::LowState_>(TOPIC_LOWSTATE));
    lowstate_subscriber->InitChannel(std::bind(&FrankaControl::LowStateMessageHandler, this, std::placeholders::_1), 1);

    lowCmdWriteThreadPtr = CreateRecurrentThreadEx("writebasiccmd", UT_CPU_ID_NONE, int(dt * 1000000), &FrankaControl::LowCmdWrite, this);
}

void FrankaControl::InitLowCmd()
{
    low_cmd.head()[0] = 0xFE;
    low_cmd.head()[1] = 0xEF;
    low_cmd.level_flag() = 0xFF;
    low_cmd.gpio() = 0;

    for (int i = 0; i < 20; i++)
    {
        low_cmd.motor_cmd()[i].mode() = (0x01);
        low_cmd.motor_cmd()[i].q() = (PosStopF);
        low_cmd.motor_cmd()[i].kp() = (0);
        low_cmd.motor_cmd()[i].dq() = (VelStopF);
        low_cmd.motor_cmd()[i].kd() = (0);
        low_cmd.motor_cmd()[i].tau() = (0);
    }
}

void FrankaControl::LowStateMessageHandler(const void *message)
{
    low_state = *(unitree_go::msg::dds_::LowState_ *)message;
}

void FrankaControl::LowCmdWrite()
{
    running_time += dt;

    if (running_time < 3.0)
    {
        // Move to home position
        phase = tanh(running_time / 1.2);
        for (int i = 0; i < NUM_JOINTS; i++)
        {
            low_cmd.motor_cmd()[i].q() = phase * home_pos[i];
            low_cmd.motor_cmd()[i].dq() = 0;
            low_cmd.motor_cmd()[i].kp() = phase * 200.0 + (1 - phase) * 50.0;
            low_cmd.motor_cmd()[i].kd() = 20.0;
            low_cmd.motor_cmd()[i].tau() = 0;
        }
    }
    else if (running_time < 6.0)
    {
        // Move to pick-ready position
        phase = tanh((running_time - 3.0) / 1.2);
        for (int i = 0; i < NUM_JOINTS; i++)
        {
            low_cmd.motor_cmd()[i].q() = phase * pick_ready_pos[i] + (1 - phase) * home_pos[i];
            low_cmd.motor_cmd()[i].dq() = 0;
            low_cmd.motor_cmd()[i].kp() = 200.0;
            low_cmd.motor_cmd()[i].kd() = 20.0;
            low_cmd.motor_cmd()[i].tau() = 0;
        }
    }
    else
    {
        // Move back to home
        phase = tanh((running_time - 6.0) / 1.2);
        for (int i = 0; i < NUM_JOINTS; i++)
        {
            low_cmd.motor_cmd()[i].q() = phase * home_pos[i] + (1 - phase) * pick_ready_pos[i];
            low_cmd.motor_cmd()[i].dq() = 0;
            low_cmd.motor_cmd()[i].kp() = 200.0;
            low_cmd.motor_cmd()[i].kd() = 20.0;
            low_cmd.motor_cmd()[i].tau() = 0;
        }
    }

    low_cmd.crc() = crc32_core((uint32_t *)&low_cmd, (sizeof(unitree_go::msg::dds_::LowCmd_) >> 2) - 1);
    lowcmd_publisher->Write(low_cmd);
}

int main(int argc, const char **argv)
{
    if (argc < 2)
    {
        ChannelFactory::Instance()->Init(1, "lo");
    }
    else
    {
        ChannelFactory::Instance()->Init(0, argv[1]);
    }
    std::cout << "Press enter to start";
    std::cin.get();
    FrankaControl control;
    control.Init();

    while (1)
    {
        sleep(10);
    }

    return 0;
}
