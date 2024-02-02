import argparse
import constants
import simulation


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--mode",
        "-m",
        type=str,
        default="robot",
        choices=["robot", "simulation", "arm", "complete"],
        help="Available modes: robot, simulation, arm, complete",
    )
    parser.add_argument(
        "--kinematics",
        "-k",
        type=str,
        default="direct",
        choices=["direct", "inverse", "frozen-direct"],
        help="Kinematics for simulation: direct, inverse, frozen-direct",
    )
    args = parser.parse_args()

    match args.mode:
        case "robot":
            print("Robot Mode")
            constants.set_constants(constants.SOFTMODE.PHANTOMX)
        case "simulation":
            print("Simulation Mode")
            constants.set_constants(constants.SOFTMODE.PHANTOMX_SIMULATION)
            simulation.setup_simulation(args.kinematics)
            simulation.setup_controls()
            simulation.execute()
        case "arm":
            print("Arm Mode")
            constants.set_constants(constants.SOFTMODE.ARM_SIMULATION)
        case "complete":
            print("Complete Mode (Not handled yet, Exiting...)")
        case _:
            print("Error: This mode does not exist")
