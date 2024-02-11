#!/usr/bin/env python
import argparse
import pybullet as p
from onshape_to_robot.simulation import Simulation
import robot
import constants

# Global variable for simulation mode
rob = None
sim = None

def simulation_execute():
    while True:  
        rob.execute_task(sim.t)
        sim.tick()

def __init__():
    global rob,sim
    
    rob = robot.robot_simulation()
    
    rob.ask_for_parameters()
    
    robot_path = "phantomx_description/urdf/phantomx.urdf"
    sim = Simulation(robot_path, gui=True, panels=True, useUrdfInertia=False)
    
    sim.setRobotPose([0, 0, 0.5], [0, 0, 0, 1])
    
    simulation_execute()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", "-m", type=str, default="direct", help="test")
    args = parser.parse_args()
    constants.set_behaviour_mode(constants.BEHAVIOUR_MODE(int(args.mode)))
    
    __init__()
