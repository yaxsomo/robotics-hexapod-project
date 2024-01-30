import pypot.dynamixel
import time
import math
import kinematics



    

if __name__ == '__main__':
    available_ports = pypot.dynamixel.get_available_ports()
    # print(pypot.__version__)
    theta2Correction = 16.0 * math.pi / 180.0
    theta3Correction = 43.76 * math.pi / 180.0  # -theta2Correction ??
    THETA3_MOTOR_SIGN = 1
    THETA2_MOTOR_SIGN = -1
    print('Opening connection...')
    print(available_ports)
    dxl = pypot.dynamixel.DxlIO(available_ports[0], baudrate=1000000)
    dxl.disable_torque([21,22,23]) 
    while True:
        positions = dxl.get_present_position([21,22,23])
        print(positions)
        pos_dk = kinematics.computeDK(positions[0],positions[1],positions[2])
        print(pos_dk)
        print()

    
    # dxl.set_goal_position({21:alphas[0],22:alphas[1],23:alphas[2]})
    # dxl.set_goal_position({21:0,22:0,23:0})
    
    time.sleep(1)
    print(dxl.get_present_position([21,22,23]))



    # with pypot.dynamixel.DxlIO(available_ports[0], baudrate=1000000) as dxl:
    #     print('Ok!')
    #     print('Scanning the bus...',)
    #     # ids = dxl.scan()
    #     dxl.get_present_position([11])
        
            
    #     # print('Done!')
    #     # print('Ids found: {}'.format(ids))

    # print('Closing port')