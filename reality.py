import robot
import time

rob = None

def __init__():
    global rob
    robot.robot_serial_init()
    rob = robot.robot_physical()
    rob.ask_for_parameters()
    reality_execute()


def reality_execute():
    while True:
        T = time.time()
        rob.execute_task(T)
    