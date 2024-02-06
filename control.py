import pypot.dynamixel
import time
import math
import kinematics
import constants
import robot

available_ports = None
dxl = None
motor_id_list = None
legs = None

def control_init():
    global available_ports, dxl, motor_id_list
    available_ports = pypot.dynamixel.get_available_ports()
    print(available_ports)
    print("Opening connection to the robot..")
    dxl = pypot.dynamixel.DxlIO(available_ports[0], baudrate=1000000)
    # print("Scanning..")
    # motor_id_list = dxl.scan()
    # print(motor_id_list)
    # motor_id_list.pop()


def create_legs():
    global legs
    legs_groups = []
    
    for i in range(0, len(motor_id_list), 3):
        legs_groups.append(motor_id_list[i:i + 3])
        
    return legs_groups


def create_robot(legs):
    r = robot.robot_physique(legs)
    return r


def test():
    control_init()
    create_legs()
    r = create_robot(legs)
    print(r.read())
    # 
    # print('Opening connection...')
    # 
    # print(list)
    # dxl.disable_torque([21,22,23]) 
    # while True:
    #     positions = dxl.get_present_position([21,22,23])
    #     print(positions)
    #     pos_dk = kinematics.computeDK(positions[0],positions[1],positions[2])
    #     print(pos_dk)
    #     print()

    # pre_conv = kinematics.computeIK(0.4, 0, 0)
    


    

    # dxl.set_goal_position({21:0,22:0,23:0})

    # time.sleep(1)
    # print(dxl.get_present_position([21,22,23]))

