import argparse
import constants
import control


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument(
    "--mode",
    "-m",
    type=str,
    default="robot",
    choices=["robot", "simulation", "arm", "complete"],
    help="Available modes : robot, simulation, arm, complete",
    )
    args = parser.parse_args()
    
    match args.mode:
        case "robot":
            print("Robot Mode")
            constants.set_constants(constants.SOFTMODE.PHANTOMX)
        case "simulation":
            print("Simulation Mode")
            constants.set_constants(constants.SOFTMODE.PHANTOMX_SIMULATION)
        case "arm":
            print("Arm Mode")
            constants.set_constants(constants.SOFTMODE.ARM_SIMULATION)
        case "complete":
            print("Complete Mode (Not handled yet, Exiting...)")
        case _:
            print("Error: This mode does not exists")