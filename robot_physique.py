import control
import kinematics
import time
import numpy as np
import constants
import traceback
import math

legs = {1:[11,12,13],
        2:[21,22,23],
        3:[31,32,33],
        4:[41,42,43],
        5:[51,52,53],
        6:[61,62,63],
}
initial_state = [150,10,90]

def read():
    result = []
    for _,motors in legs.items():
        motors_group = []
        for single_motor in motors:
            temp = control.dxl.get_present_position([single_motor])
            if single_motor in constants.LIST_OF_INVERTED_IDS:
                motors_group.append(-temp[0])
            else:
                motors_group.append(temp[0])
        result.append(motors_group)     
    return result


def format_dict(positions):
    result = {}
    for leg_index, motors in legs.items():
        for motor, position in zip(motors, positions[leg_index - 1]):
            result[motor] = position
    return result


def write(new_positions):
    result_dict = {}
    for key, value in new_positions.items():
        if key in constants.LIST_OF_INVERTED_IDS:
            result_dict[key] = -value
        else:
            result_dict[key] = value
    control.dxl.set_goal_position(result_dict)

        
def move_leg(x, y , z, leg):
    pass


def set_robot_initial_position(position):
    interpolation(position, 1)


def interpolation(final_pos,duration):
    t0 = time.time()
    leg_pos = read()
    A = np.array(leg_pos)
    B = np.array(kinematics.computeIK(final_pos[0], final_pos[1] , final_pos[2]))
    t = time.time() - t0
    alpha = 0
    
    while(t < duration):
        alpha = t / duration
        M = ((B-A) * alpha) + A 
        new_pos = format_dict(M)
        write(new_pos)
        time.sleep(0.01)
        t = time.time() - t0



def move(method="walk",direction=0):
    try:
        while True:   
            final_pos = []
            index = 0
            for leg_id, _ in legs.items():
                index += 1
                match method:
                    case "walk":
                        thetas = kinematics.triangle(-30,5,-60,50,(time.time()*0.5 +(0.5*(index % 2))) ,oriented=True,leg_id=leg_id, angle_direction=direction)
                    case "rotate":
                        thetas = kinematics.triangle(150,100,50,90,(time.time()*0.5 +(0.5*(index % 2))))
                    case "tilt-x":
                        thetas = kinematics.computeIKOriented(0,50 * math.sin(time.time()),0,0,leg_id) # balancement sur X
                    case "tilt-y":
                        thetas = kinematics.computeIKOriented(0,50 * math.cos(time.time()),0,leg_id) # balancement sur Y
                    case "tilt-z":
                        thetas = kinematics.computeIKOriented(0,0,50 * math.cos(time.time()),leg_id) # balancement sur Y
                    case "tilt-xy":
                        thetas = kinematics.computeIKOriented(50 * math.cos(time.time()),50 * math.sin(time.time()),0,leg_id) # robot fait un cercle sur lui meme
                final_pos.append(thetas)
            to_feed = format_dict(final_pos)
            write(to_feed)
    except Exception as e:
        print(e)
        print(traceback.format_exc())
    


if __name__ == "__main__":    
    control.control_init()
    # move(method="tilt-x",direction=0)
    set_robot_initial_position(initial_state)
