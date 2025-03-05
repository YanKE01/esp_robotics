import time

from Arm_Lib import Arm_Device

if __name__ == '__main__':
    Arm = Arm_Device(com="COM9")
    time.sleep(1)

    # Return to Center Position
    Arm.Arm_serial_servo_write6(90, 90, 90, 90, 90, 0, 1000)
    time.sleep(1)

    del Arm
