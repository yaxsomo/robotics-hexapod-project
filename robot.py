# import kinematics
# import constants
# import control
# import simulation

# class Motor:
#     motor_id = 0
#     shaft_present_position = 0.0
#     present_speed = 0.0
    
#     # Constructor 
#     def __init__(self, id = 0, position = 0.0, speed = 0.0): 
#         self.motor_id = id 
#         self.shaft_present_position = position 
#         self.present_speed = speed 
    
#     def get_motor_id(self):
#         return self.motor_id
    
#     def get_shaft_present_position(self):
#         return self.shaft_present_position
    
#     def set_shaft_present_position(self, current_pos):
#         self.shaft_present_position = current_pos

    
#     def get_present_speed(self):
#         return self.present_speed


# class Leg:
#     motors = [Motor() for _ in range(3)]
#     leg_angle = 0
    
#     # Constructor 
#     def __init__(self, angle, motor_group = None):
#         self.motors = motor_group
#         self.leg_angle = angle
        
#     def get_point(self):
#         positions = [motor.get_shaft_present_position() for motor in self.motors]
#         return kinematics.computeDK(positions[0],positions[1],positions[2])

#     def move_leg_physical(self, x, y , z):
#         alphas = kinematics.computeIK(x, y, z)
#         return control.dxl.set_goal_position({self.motors[0]:alphas[0],self.motors[1]:alphas[1],self.motors[2]:alphas[2]})
    
#     def move_leg_virtual(self, x, y , z):
#         alphas = kinematics.computeIK(x, y, z)
#         return simulation.sim.setJoints({self.motors[0]:alphas[0],self.motors[1]:alphas[1],self.motors[2]:alphas[2]})

#     def set_joints(self, joints):
#         for i in range(len(self.motors)):
#             self.motors[i].set_shaft_present_position(joints[i])

# class RobotClass:
#     legs = [Leg() for _ in range(6)]

#      # Constructor 
#     def __init__(self, legs = None):
#         self.legs = legs
    
    
        
#     def move_robot(self, x, y ,z):
#         for leg in self.legs:
#             rotated_positions = kinematics.rotation_2d(x, y, z,leg.leg_angle)
#             alphas = kinematics.computeIK(rotated_positions[0],rotated_positions[1],rotated_positions[2])
#             leg.move_leg(alphas[0], alphas[1], alphas[2])


# class robot_physique(RobotClass):
#     legs = {1:[SimpleMotor(11), Simp]}
    
#     # Constructor 
#     def __init__(self, legs = None):
#         self.legs = legs
    
#     def read(self):
#         legs = []
#         for leg in self.legs:
#             single_leg = []
#             for joints in leg.motors:
#                 single_leg.append(joints.get_shaft_present_position())
#             legs.append(single_leg)
#         return legs
    
#     def write(self, all_positions):
#         for i in range(0, len(all_positions), 3):
#             leg_temp = all_positions[i:i + 3]
#             for leg in self.legs:
#                     control.dxl.get_pres_position()
#                     leg.set_joints(leg_temp)
                    



# class Robot_Simulation(Robot):
    
#     def read(self):
#         pass
    
#     def write(self):
#         pass