import pypot.dynamixel
import time
import math
import kinematics




if __name__ == '__main__':
    # available_ports = pypot.dynamixel.get_available_ports()
    # print('Opening connection...')
    # dxl = pypot.dynamixel.DxlIO(available_ports[0], baudrate=1000000)
    # dxl.disable_torque([21,22,23]) 
    # while True:
    #     positions = dxl.get_present_position([21,22,23])
    #     print(positions)
    #     pos_dk = kinematics.computeDK(positions[0],positions[1],positions[2])
    #     print(pos_dk)
    #     print()

    pos_dk = kinematics.computeIK(1,0,0)
    print(pos_dk)
    
    print(kinematics.normalize_angle(pos_dk[2]))

    
    
    # dxl.set_goal_position({21:alphas[0],22:alphas[1],23:alphas[2]})
    # dxl.set_goal_position({21:0,22:0,23:0})
    
    # time.sleep(1)
    # print(dxl.get_present_position([21,22,23]))

