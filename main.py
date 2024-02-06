import argparse
import constants
import simulation
import threading
import menu

def start_thread(thread_id):
    simulation_thread = threading.Thread(target=simulation.execute())
    get_tick_thread = threading.Thread(target=None)

    match thread_id:
        case 1:
            simulation_thread.start()         
        case 2:
            get_tick_thread.start()

if __name__ == '__main__':
    menu.clear_screen()
    main_menu_thread = threading.Thread(target=menu.main_menu())
    main_menu_thread.start()