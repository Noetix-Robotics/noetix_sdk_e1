import os
import sys
from time import sleep
import numpy as np

# DDS 配置
os.environ["CYCLONEDDS_URI"] = "file://config/dds.xml"

sys.path.append(os.path.abspath("./build"))
from highcontrol_py import *

# 初始化
ctrl = HighController.instance()
ctrl.init()

Key1 = 1
Key2 = 2
Key5 = 5
Key6 = 6
Key7 = 7
Key8 = 8
Key9 = 9
Key10 = 10
Key12 = 12

# 共享状态，由回调更新
remote_data = None
curmode = -1

key_updown = [0] * 14
key_inuse = [0] * 14

fileindex = 0

lasmode = curmode = 0


def on_robot_hardware_status(status: RobotHardwareStatus):
    """硬件状态回调：更新遥控器数据和模式"""
    global remote_data, curmode
    remote_data = status.remote_data
    if status.workmode != curmode:
        curmode = status.workmode
        print(f"[PYTHON DEBUG]: curmode is {curmode}")


# 注册回调
ctrl.subscribe_robot_hardware_status(on_robot_hardware_status)

while True:
    if remote_data is None:
        sleep(0.01)
        continue

    for i in range(14):
        if remote_data.button[i] == 0:
            key_updown[i] = 0
            key_inuse[i] = 0
        elif remote_data.button[i] == 1:
            key_updown[i] = 1

    action = ControlCmd.DEFAULT

    x = remote_data.axes[1]
    yaw = remote_data.axes[0]

    if key_updown[Key2] == 0 and key_updown[Key5] == 1 and key_inuse[Key5] == 0:
        action = ControlCmd.STARTTEACH
        key_inuse[Key5] = 1
        print("STARTTEACH")

    elif (
        key_updown[Key6] == 1
        and key_updown[Key2] == 0
        and key_inuse[Key6] == 0
        and key_updown[Key1] == 0
    ):
        action = ControlCmd.SWING
        key_inuse[Key6] = 1
        print("SWING")

    elif (
        key_updown[Key7] == 1
        and key_updown[Key2] == 0
        and key_inuse[Key7] == 0
        and key_updown[Key1] == 0
    ):
        action = ControlCmd.SHAKE
        key_inuse[Key7] = 1
        print("SHAKE")

    elif (
        key_updown[Key8] == 1
        and key_updown[Key2] == 0
        and key_inuse[Key8] == 0
        and key_updown[Key1] == 0
    ):
        action = ControlCmd.CHEER
        key_inuse[Key8] = 1
        print("CHEER")

    elif key_updown[Key9] == 1 and key_inuse[Key9] == 0:
        key_inuse[Key9] = 1
        action = ControlCmd.START
        print("START")

    elif key_updown[Key10] == 1 and key_inuse[Key10] == 0:
        action = ControlCmd.SWITCH
        key_inuse[Key10] = 1
        print("SWITCH")

    elif key_updown[Key2] == 1 and key_updown[Key5] == 1 and key_inuse[Key5] == 0:
        action = ControlCmd.WALK
        key_inuse[Key5] = 1
        print("WALK")

    elif key_updown[Key1] == 1 and key_updown[Key6] == 1 and key_inuse[Key6] == 0:
        action = ControlCmd.SAVETEACH
        key_inuse[Key6] = 1
        fileindex += 1
        print("SAVETEACH")

    elif key_updown[Key1] == 1 and key_updown[Key7] == 1 and key_inuse[Key7] == 0:
        action = ControlCmd.ENDTEACH
        key_inuse[Key7] = 1
        print("ENDTEACH")

    elif key_updown[Key1] == 1 and key_updown[Key8] == 1 and key_inuse[Key8] == 0:
        action = ControlCmd.PLAYTEACH
        key_inuse[Key8] = 1
        fileindex = 1
        print("PLAYTEACH")

    elif key_updown[Key2] == 1 and key_updown[Key6] == 1:
        action = ControlCmd.RUN
        key_inuse[Key6] = 1
        print("RUN")

    ctrl.publish_cmd(x, yaw, action, fileindex)

    # ️必须限频
    sleep(0.002)
