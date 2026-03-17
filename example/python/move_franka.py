import time
import sys
import numpy as np

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_
from unitree_sdk2py.utils.crc import CRC

NUM_JOINTS = 8  # 7 arm joints + 1 gripper

# Franka home position (from keyframe)
home_joint_pos = np.array([
    0.0, 0.0, 0.0, -1.57079, 0.0, 1.57079, -0.7853, 0.04
], dtype=float)

# A pick-ready position with arm extended forward
pick_ready_pos = np.array([
    0.0, -0.3, 0.0, -2.0, 0.0, 2.0, 0.7854, 0.04
], dtype=float)

dt = 0.002
running_time = 0.0
crc = CRC()

input("Press enter to start")

if __name__ == '__main__':

    if len(sys.argv) < 2:
        ChannelFactoryInitialize(1, "lo")
    else:
        ChannelFactoryInitialize(0, sys.argv[1])

    pub = ChannelPublisher("rt/lowcmd", LowCmd_)
    pub.Init()

    cmd = unitree_go_msg_dds__LowCmd_()
    cmd.head[0] = 0xFE
    cmd.head[1] = 0xEF
    cmd.level_flag = 0xFF
    cmd.gpio = 0
    for i in range(20):
        cmd.motor_cmd[i].mode = 0x01
        cmd.motor_cmd[i].q = 0.0
        cmd.motor_cmd[i].kp = 0.0
        cmd.motor_cmd[i].dq = 0.0
        cmd.motor_cmd[i].kd = 0.0
        cmd.motor_cmd[i].tau = 0.0

    while True:
        step_start = time.perf_counter()
        running_time += dt

        if running_time < 3.0:
            # Move to home position in first 3 seconds
            phase = np.tanh(running_time / 1.2)
            for i in range(NUM_JOINTS):
                cmd.motor_cmd[i].q = phase * home_joint_pos[i]
                cmd.motor_cmd[i].kp = phase * 200.0 + (1 - phase) * 50.0
                cmd.motor_cmd[i].dq = 0.0
                cmd.motor_cmd[i].kd = 20.0
                cmd.motor_cmd[i].tau = 0.0
        elif running_time < 6.0:
            # Move to pick-ready position
            phase = np.tanh((running_time - 3.0) / 1.2)
            for i in range(NUM_JOINTS):
                cmd.motor_cmd[i].q = phase * pick_ready_pos[i] + (1 - phase) * home_joint_pos[i]
                cmd.motor_cmd[i].kp = 200.0
                cmd.motor_cmd[i].dq = 0.0
                cmd.motor_cmd[i].kd = 20.0
                cmd.motor_cmd[i].tau = 0.0
        else:
            # Move back to home
            phase = np.tanh((running_time - 6.0) / 1.2)
            for i in range(NUM_JOINTS):
                cmd.motor_cmd[i].q = phase * home_joint_pos[i] + (1 - phase) * pick_ready_pos[i]
                cmd.motor_cmd[i].kp = 200.0
                cmd.motor_cmd[i].dq = 0.0
                cmd.motor_cmd[i].kd = 20.0
                cmd.motor_cmd[i].tau = 0.0

        cmd.crc = crc.Crc(cmd)
        pub.Write(cmd)

        time_until_next_step = dt - (time.perf_counter() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)
