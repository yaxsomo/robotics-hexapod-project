import simulation
import reality
import threading
import os
import robot

def start_thread(thread_id):
    simulation_thread = threading.Thread(target=simulation.__init__())
    reality_thread = threading.Thread(target=reality.__init__)
    test_thread = threading.Thread(target=robot.robot_action)

    match thread_id:
        case 1:
            reality_thread.start()        
        case 2:
            simulation_thread.start()
        case 3:
            test_thread.start()
            
def clear_screen():
    if(os.name == 'nt'):
        os.system('cls')
    else:
        os.system('clear')
        
def print_behaviour_choice():
    print("Select Behaviour :")
    print("1. Move leg to position")
    print("2. Move robot center to position")
    print("3. Walk in a straight line")
    print("4. Rotate robot")
    print("5. Exit")
    behaviour_choice = input("Choice : ")
    
    return behaviour_choice


if __name__ == '__main__':
    clear_screen()
    while(True):
        
        print("- Select Software Mode : -")
        print("1. Robot")
        print("2. Simulation")
        print("3. Exit")
        robot_mode_choice = input("Choice : ")
        clear_screen()

        if robot_mode_choice == "1" or robot_mode_choice == "2":
            print("- Behavior Mode Selection -")
            # constants.set_constants(constants.SOFTMODE.PHANTOMX)
            while(True):
                behaviour_choice = print_behaviour_choice()
                if behaviour_choice == "5":
                    break
                else:
                    robot.robot_action(robot_mode_choice,behaviour_choice)
            clear_screen()
        else:    
            print("Bye!")
            break























