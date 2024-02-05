import pypot.dynamixel
import time
import math
import kinematics
import constants

'''normalize_to_angle_range : input: 0 - 360 degrees | output : - angle to angle ranges'''
def normalize_to_angle_range(angle_deg, range):
    return (angle_deg + range) % 360.0 - range


def test():
    available_ports = pypot.dynamixel.get_available_ports()
    print('Opening connection...')
    dxl = pypot.dynamixel.DxlIO(available_ports[0], baudrate=1000000)
    # list = dxl.scan()
    # print(list)
    # dxl.disable_torque([21,22,23]) 
    # while True:
    #     positions = dxl.get_present_position([21,22,23])
    #     print(positions)
    #     pos_dk = kinematics.computeDK(positions[0],positions[1],positions[2])
    #     print(pos_dk)
    #     print()

    pre_conv = kinematics.computeIK(0.4, 0, 0)
    alphas = kinematics.computeDK(pre_conv[0], pre_conv[1], pre_conv[2])

    dxl.set_goal_position({21:alphas[0],22:alphas[1],23:alphas[2]})
    # dxl.set_goal_position({21:0,22:0,23:0})

    # time.sleep(1)
    # print(dxl.get_present_position([21,22,23]))

if __name__ == "__main__":
    test()