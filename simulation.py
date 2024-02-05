#!/usr/bin/env python
import math
import argparse
import pybullet as p
from onshape_to_robot.simulation import Simulation
import kinematics
from scipy.spatial.transform import Rotation
from main import start_thread
import readchar
import threading

# Global variable for simulation mode
simulation_mode = None
cross = None
crosses = None
controls = None
sim = None
state = None
leg_center_pos = [0.1248, -0.06164, 0.001116 + 0.5]
bx = 0.07
bz = 0.25
# Global flag to control the loop
exit_flag = False
exit_lock = threading.Lock()  # Lock for ensuring thread-safe access to exit_flag


def to_pybullet_quaternion(roll, pitch, yaw, degrees=False):
    # q = Quaternion.from_euler(roll, pitch, yaw, degrees=degrees)
    # return [q[1], q[2], q[3], q[0]]

    # Create a rotation object from Euler angles specifying axes of rotation
    rot = Rotation.from_euler("xyz", [roll, pitch, yaw], degrees=degrees)

    # Convert to quaternions and print
    rot_quat = rot.as_quat()
    # print(rot_quat)
    return rot_quat

def setup_simulation(sim_mode):
    global simulation_mode
    global sim
    simulation_mode = sim_mode
    
    robot_path = "phantomx_description/urdf/phantomx.urdf"
    sim = Simulation(robot_path, gui=True, panels=True, useUrdfInertia=False)
    sim.setRobotPose([0, 0, 0.5], [0, 0, 0, 1])

def setup_controls():
    controls_setup = {}
    global controls
    global crosses
    global cross
    
    if simulation_mode == "frozen-direct":
        crosses = [p.loadURDF("target2/robot.urdf") for _ in range(4)]
        for name in sim.getJoints():
            if "c1" in name or "thigh" in name or "tibia" in name:
                controls_setup[name] = p.addUserDebugParameter(name, -math.pi, math.pi, 0)
    elif simulation_mode == "direct":
        for name in sim.getJoints():
            if "c1" in name or "thigh" in name or "tibia" in name:
                controls_setup[name] = p.addUserDebugParameter(name, -math.pi, math.pi, 0)
    elif simulation_mode == "inverse":
        cross = [p.loadURDF("target2/robot.urdf")]
        pre_conv = kinematics.computeIK(0.4, 0, 0)
        alphas = kinematics.computeDK(pre_conv[0], pre_conv[1], pre_conv[2])
        controls_setup["target_x"] = p.addUserDebugParameter("target_x", -0.4, 0.4, alphas[0])
        controls_setup["target_y"] = p.addUserDebugParameter("target_y", -0.4, 0.4, alphas[1])
        controls_setup["target_z"] = p.addUserDebugParameter("target_z", -0.4, 0.4, alphas[2])
    elif simulation_mode == "triangle":
        controls_setup["triangle_x"] = p.addUserDebugParameter("triangle_x", 0.01, 0.8, 0.4)
        controls_setup["triangle_z"] = p.addUserDebugParameter("triangle_z", -0.2, 0.3, 0)
        controls_setup["triangle_h"] = p.addUserDebugParameter("triangle_h", 0.01, 0.3, 0.1)
        controls_setup["triangle_w"] = p.addUserDebugParameter("triangle_w", 0.01, 0.3, 0.2)
        controls_setup["triangle_duration"] = p.addUserDebugParameter("triangle_duration", 0.01, 10, 3)
    controls = controls_setup

def detect_stop():
    global exit_flag
    key_pressed = readchar.readkey()
    if key_pressed == "q":
        exit_flag = True





def execute():
    global state
    while True:
        # Check if the exit flag is set
        with exit_lock:
            if exit_flag:
                break  # Exit the loop if the flag is set
        targets = {}
        for name in sim.getJoints():
            if "c1" in name or "thigh" in name or "tibia" in name:
                targets[name] = 0
        if simulation_mode == "frozen-direct":
            for name in controls.keys():
                targets[name] = p.readUserDebugParameter(controls[name])
            points = kinematics.computeDKDetailed(
                targets["j_c1_rf"],
                targets["j_thigh_rf"],
                targets["j_tibia_rf"],
                use_rads=True,
            )
            for i, pt in enumerate(points):
                leg_angle = -math.pi / 4
                T = kinematics.rotation_2d(pt[0], pt[1], pt[2], leg_angle)
                T = [T[0] + leg_center_pos[0], T[1] + leg_center_pos[1], T[2] + leg_center_pos[2]]
                p.resetBasePositionAndOrientation(
                    crosses[i], T, to_pybullet_quaternion(0, 0, leg_angle)
                )
            sim.setRobotPose([0, 0, 0.5], to_pybullet_quaternion(0, 0, 0))
            state = sim.setJoints(targets)
        elif simulation_mode == "direct":
            try:
                for name in controls.keys():
                    targets[name] = p.readUserDebugParameter(controls[name])
            except Exception as e:
                continue
            state = sim.setJoints(targets)
        elif simulation_mode == "inverse":
            try:
                x = p.readUserDebugParameter(controls["target_x"])
                y = p.readUserDebugParameter(controls["target_y"])
                z = p.readUserDebugParameter(controls["target_z"])
            except Exception as e:
                continue
            alphas = kinematics.computeIK(x, y, z)
            targets["j_c1_rf"] = alphas[0]
            targets["j_thigh_rf"] = alphas[1]
            targets["j_tibia_rf"] = alphas[2]
            state = sim.setJoints(targets)
            sim.setRobotPose([0, 0, 0.5], [0, 0, 0, 1])
            leg_angle = -math.pi / 4
            T = kinematics.rotation_2d(x, y, z, leg_angle)
            T = [T[0] + leg_center_pos[0], T[1] + leg_center_pos[1], T[2] + leg_center_pos[2]]
            p.resetBasePositionAndOrientation(
                cross[0], T, to_pybullet_quaternion(0, 0, leg_angle)
            )
        elif simulation_mode == "triangle":
            try:
                x = p.readUserDebugParameter(controls["triangle_x"])
                z = p.readUserDebugParameter(controls["triangle_z"])
                h = p.readUserDebugParameter(controls["triangle_h"])
                w = p.readUserDebugParameter(controls["triangle_w"])
                duration = p.readUserDebugParameter(controls["triangle_duration"])
            except Exception as e:
                continue
            alphas = kinematics.triangle(x, z, h, w, sim.t, duration)
            targets = {
                "j_c1_rf": alphas[0],
                "j_thigh_rf": alphas[1],
                "j_tibia_rf": alphas[2],
            }
            
            pos = kinematics.computeDK(alphas[0], alphas[1], alphas[2])
            pos[0] += bx
            pos[2] += bz
            sim.addDebugPosition(pos, duration=3)
            sim.setJoints(targets)
        
        sim.tick()



def simulation_init(mode):
    global listener_thread
    print("Hey")
    setup_simulation(mode)
    if(mode == "move_leg" or mode == "move_robot_center" or mode == "direct" or mode == "inverse"):
        setup_controls()
    # start_thread(1)
    global exit_lock, exit_flag
    execution_thread = threading.Thread(target=execute())
    try:
        
        # Start the execution thread
        execution_thread.start()

        # Continue with other operations if needed

    except KeyboardInterrupt:
        # Handle keyboard interrupt (Ctrl+C) to stop the execution
        with exit_lock:
            exit_flag = True
    finally:
        execution_thread.join()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", "-m", type=str, default="direct", help="test")
    args = parser.parse_args()
    
    setup_simulation(args.mode)
    setup_controls()
    execute()
