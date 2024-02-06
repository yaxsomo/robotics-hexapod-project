import pypot.dynamixel
import traceback


available_ports = None
dxl = None

def control_init():
    global available_ports, dxl
    try:
        available_ports = pypot.dynamixel.get_available_ports()
        print(available_ports)
        print("Opening connection to the robot..")
        dxl = pypot.dynamixel.DxlIO(available_ports[0], baudrate=1000000)
    except Exception:
        print(traceback.format_exc())


def get_motors_list_physical():
    try:
        print("Scanning..")
        result = dxl.scan()
        return(result)
    except Exception:
        print(traceback.format_exc())
        
 