import time

from Arm_Lib import Arm_Device

Arm = Arm_Device(com="COM9")
time.sleep(1)

Arm.Arm_serial_set_torque(1)
time.sleep(1)

angle_list = []

while True:
    angle_list.clear()
    for i in range(1, 7):
        angle = Arm.Arm_serial_servo_read(i)
        angle_list.append(angle)
        time.sleep(0.1)
    print(angle_list)
