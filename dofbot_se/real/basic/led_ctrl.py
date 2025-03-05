import time

from Arm_Lib import Arm_Device

if __name__ == '__main__':
    Arm = Arm_Device(com="COM9")
    time.sleep(2)

    Arm.Arm_RGB_set(50, 0, 0)
    time.sleep(2)
    Arm.Arm_RGB_set(0, 50, 0)
    time.sleep(2)
    Arm.Arm_RGB_set(0, 0, 50)
    time.sleep(2)
    Arm.Arm_RGB_set(0, 0, 0)

    del Arm
