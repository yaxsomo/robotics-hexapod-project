import control
import kinematics
import time
import numpy as np
import constants
import traceback
import math

"""Class Robot : This class serve to define the common attributes and methods for the Simulation and Physical Robot"""
class Robot:
    INITIAL_POSITION = [150, 10, 90]  # Define the hexapod initial position 

    
    def __init__(self):
        """__init__() :This method must be implemented in subclassess."""
        raise NotImplementedError("__init__() method must be implemented in subclasses.")
        
    def read(self, leg_id):
        """read() :This method must be implemented in subclassess."""
        raise NotImplementedError("read() method must be implemented in subclasses.")
    
    def write(self, new_positions):
        """write() :This method must be implemented in subclassess."""
        raise NotImplementedError("write() method must be implemented in subclasses.")
    
    def move_leg(self,x, y , z, duration, leg_id):
        """move_leg() : This method can move a hexapod's single leg to an arbitrary x, y, z position"""
        leg = self.legs[leg_id] # Gets the motors ids of the desired leg
        self.interpolate([x, y , z],duration,leg) # Makes a linear interpolation of the desired motor, from its current position to the desired x, y, z position

    
    def interpolate(self, final_pos,duration,leg_id = -1):
        """interpolate() : This method can execute an interpolation on a single leg or the entire robot from its present position to an arbitrary x, y, z position"""
        alpha = 0 # Initialization of the 'alpha' variable for interpolation
        try:
            t0 = time.time() # Define the initial time
            leg_pos = self.read(leg_id) # Reads the current position of one or all the legs
            A = np.array(leg_pos) # Creates a np.array of the current leg/legs positions
            B = np.array(kinematics.compute_ik(final_pos[0], final_pos[1] , final_pos[2])) # Creates a np.array of the final x, y, z positions
            t = time.time() - t0 # Created the differential time
            
            
            while(t < duration): # Wile loop -> Continues till the differential time is lower than the 'duration' parameter
                alpha = t / duration #Updating the 'alpha' variable at each While loop for smooth interpolation
                M = ((B-A) * alpha) + A #Linear Interpolation formula
                new_pos = self.format_dict(M,leg_id=leg_id) #Creates a dictionnary of the calculated interpolation, in order to be fed to the 'write()' method
                self.write(new_pos) #Writing the positions to one or all the motors
                time.sleep(0.01) # Sleep function -> '0.01' = 100 Hertz
                t = time.time() - t0 # Decreasing the differential time
        except Exception: # Exception handling
            print(traceback.format_exc()) # Traceback print
            
    def set_robot_initial_position(self):
        """set_robot_initial_position() : This method moves the hexapod to its initial position."""
        try:
            self.interpolation(self.INITIAL_POSITION, 1)  # Makes a linear interpolation from the motors current position to the initial position
        except Exception: # Exception handling
            print(traceback.format_exc()) # Traceback print
    
    def format_dict(self,positions, leg_id = -1):
        """write() : This method creates a dictionnary that can be fed to the write() method."""
        result = {} # Dictionnary result
        try:    
            if(len(positions) > 1 and leg_id == -1): # Checks if the 'positions' array have a length greater than 1 and 'leg_id' is equal to -1 (Means that we have a list of lists)
                for leg_index, motors in self.legs.items(): #Key:value For-loop in the 'self.legs' array
                    for motor, position in zip(motors, positions[leg_index - 1]): #Key:value For-loop in the zipped result of motors and positions[leg_index -1]
                        result[motor] = position # Creates a key:value element in the dictionnary 'result'
                return result # Returns the result
            elif(len(positions) == 1 and leg_id != -1): # Checks if the 'positions' array have a length of 1 and 'leg_id' is different from -1 (Means that we have only one list on the 'positions' variable instead of a list of lists)
                temp = self.legs[leg_id] # Take the first (and only) element in the 'positions' list
                return dict(zip(temp, positions[0])) # Returns a new dictionary by zipping the keys from 'self.legs' with the values from 'positions'
        except Exception: # Exception handling
            print(traceback.format_exc()) # Traceback print
    
    def move(self, method="walk", direction=0):
        """move() : This method moves the hexapod, based on the method passed as parameter."""
        try:
            while True:   
                final_pos = [] # Final positions array initialization
                index = 0 # Index variable initialization for motor de-phasing
                for leg_id, _ in self.legs.items(): # For-loop on the legs array
                    index += 1 # Increasing the index at each loop for legs de-phasing
                    match method: # Match-Case on the 'method' parameter
                        case "walk":
                            thetas = kinematics.triangle(-30, 
                                                         5, 
                                                         -60, 
                                                         50, 
                                                         (time.time() * 0.5 + (0.5 * (index % 2))), 
                                                         oriented=True, 
                                                         leg_id=leg_id, 
                                                         angle_direction=direction) # Move the robot towards the angle direction (0 = forward)
                        case "rotate":
                            thetas = kinematics.triangle(150, 
                                                         100, 
                                                         50, 
                                                         90, 
                                                         (time.time() * 0.5 + (0.5 * (index % 2)))
                                                         ) # Rotates the robot
                        case "tilt-x":
                            thetas = kinematics.compute_ik_oriented(0, 
                                                                    50 * math.sin(time.time()),
                                                                    0, 
                                                                    0, 
                                                                    leg_id) # Tilt on X axis
                        case "tilt-y":
                            thetas = kinematics.compute_ik_oriented(0, 
                                                                    50 * math.cos(time.time()), 
                                                                    0, 
                                                                    leg_id) # Tilt on Y axis
                        case "tilt-z":
                            thetas = kinematics.compute_ik_oriented(0, 
                                                                    0, 
                                                                    50 * math.cos(time.time()), 
                                                                    leg_id) # Tilt on Z axis
                        case "tilt-xy":
                            thetas = kinematics.compute_ik_oriented(50 * math.cos(time.time()), 
                                                                    50 * math.sin(time.time()), 
                                                                    0, 
                                                                    leg_id) # Tilt on X-Y axis (circle)
                    final_pos.append(thetas) #Appends the result of kinematics computation at each loop
                to_feed = self.format_dict(final_pos) # Creates a dictionnary that can be fed to the 'write()' function
                self.write(to_feed) #Writes the new positions to the motors
        except Exception: # Exception handling
            print(traceback.format_exc()) # Traceback print

"""Class robot_physical : This class serve to define the Physical Robot particularities"""
class robot_physical(Robot):
    
    def __init__(self):
        """__init__() :This method instantiates the robot legs."""
        self.legs = {1:[11,12,13],
                        2:[21,22,23],
                        3:[31,32,33],
                        4:[41,42,43],
                        5:[51,52,53],
                        6:[61,62,63]}
    
    def read(self, leg_id = -1):
        """read() : This method reads all the motors current positions and returns a dictionnary."""
        result = [] # Final result array initialization
        try:
            match leg_id: # Match-Case on the 'leg_id' parameter
                case -1: # in this case, we assume that we don't have specified a 'leg_id', so we will read positions to all motors
                    for _,motors in self.legs.items(): # Key:value For-loop in the 'self.legs' array
                        motors_group = [] # 3 element array initialization (leg motors)
                        for single_motor in motors: #For-loop in the leg's motors ids
                            temp = control.dxl.get_present_position([single_motor]) # Calling dxl to gather the present position of a single motor
                            if single_motor in constants.LIST_OF_INVERTED_IDS: # Checks if the motor id is present in the 'LIST_OF_INVERTED_IDS' constant
                                motors_group.append(-temp[0]) # True case : It inverts the motor present position value and adds it to the motor group array
                            else:
                                motors_group.append(temp[0]) # False case : It simply appends the motor present position value in the motor group array, unchanged.
                        result.append(motors_group) # Once it has looped through the 3 motors of each leg, it appends the motor group to the final result array    
                    return result # Returns the final result array, containing all the motors position
                case _: # In this case, we assume that we have specified a 'leg_id', so we will read positions to only one leg
                    return control.dxl.get_present_position([leg_id]) # Returns the 3 motors positions of one leg 
        except Exception: # Exception handling
            print(traceback.format_exc()) # Traceback print

    def write(self, new_positions):
        """write() : This method writes positions to one or all the motors."""
        result_dict = {} # Final result dictionnary initialization
        try:
            for key, value in new_positions.items(): #Key-Value For-loop through the 'new_positions' dictionnary
                if key in constants.LIST_OF_INVERTED_IDS: # Checks if the key of the current element in the dictionnary is present in the 'LIST_OF_INVERTED_IDS' constant
                    result_dict[key] = -value # True case : It inverts the motor goal position value and adds it to the final result dictionnary
                else:
                    result_dict[key] = value # False case : It simply appends the motor goal position value in the final result dictionnary, unchanged.
            control.dxl.set_goal_position(result_dict) # Writes the positions of one or all the motors after the For-loop
        except Exception: # Exception Handling
            print(traceback.format_exc()) # Traceback print
    
"""Class robot_simulation : This class serve to define the Simulation Robot particularities"""
class robot_simulation(Robot):
    
    def __init__(self):
        """__init__() :This method instantiates the robot legs."""
        self.legs = {1:['j_c1_rf','j_thigh_rf','j_tibia_rf'],
                     2:['j_c1_rm','j_thigh_rm','j_tibia_rm'],
                     3:['j_c1_rr','j_thigh_rr','j_tibia_rr'],
                     4:['j_c1_lf','j_thigh_lf','j_tibia_lf'],
                     5:['j_c1_lm','j_thigh_lm','j_tibia_lm'],
                     6:['j_c1_lr','j_thigh_lr','j_tibia_lr']}
    
    def read(self, leg_id):
        """read() : This method reads all the motors current positions and returns a dictionnary."""
        raise NotImplementedError("robot_simulation:read() -> Method not implemented yet.")
    
    def write(self, new_positions):
        """write() : This method writes positions to one or all the motors."""
        raise NotImplementedError("robot_simulation:write() -> Method not implemented yet.")
