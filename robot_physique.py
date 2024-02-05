import control
import kinematics
control.control_init()
ids = control.create_legs()
# print(legs[0])


def read():
    result = []
    for leg in ids:
        result.append(control.dxl.get_present_position(leg))
    return result

positions = read()
# positions = [(75.22, -16.86, -55.28), (1.91, -27.71, 97.21), (-8.94, -8.94, 118.91), (-32.4, 80.79, 32.99), (25.07, -87.24, -150.0), (35.34, -8.94, -48.53)]
# print(positions)
def format_dict(ids, positions):
    new_dict = {id_val: position_val for id_list, position in zip(ids, positions) for id_val, position_val in zip(id_list, position)}
    return new_dict


new_positions = format_dict(ids, positions)

def write(new_positions):
    control.dxl.set_goal_position(new_positions)
        

# write(new_positions)

def move_leg(x, y , z, leg_id):
    leg = ids[leg_id]
    alphas = kinematics.computeIK(x, y, z)
    control.dxl.set_goal_position({leg[0]:alphas[0],leg[1]:alphas[1],leg[2]:alphas[2]})




# move_leg(10,0,0,1)
# print(read())

dk = kinematics.computeDK(-1.91, 5.72, 17.45)
def set_robot_initial_position(ids):
    for leg_id in range(len(ids)):
        move_leg(dk[0],dk[1],dk[2], leg_id)

set_robot_initial_position(ids)


