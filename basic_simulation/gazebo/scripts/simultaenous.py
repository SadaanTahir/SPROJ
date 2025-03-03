#!/usr/bin/env python
import rospy
from concurrent.futures import ThreadPoolExecutor, as_completed
from utils import make_connections, setmode, arm, takeoff, move, return_to_start, land, disarm


def simultaneous_takeoff(conn):
    """
    Function to set the mode, arm, and takeoff a single drone.
    """
    ready = setmode(conn, 'GUIDED')
    if ready:
        arm(conn)
        takeoff(conn)
        conn.close()
        return conn.target_system, True
    else:
        return conn.target_system, False

def handle_simultaneous_takeoff(connections, func):
        # Create a ThreadPoolExecutor to handle drones concurrently
    with ThreadPoolExecutor(max_workers=len(connections)) as executor:
        # Submit all tasks to the executor
        futures = [executor.submit(func, conn) for conn in connections]

        # Process the results as they complete
        for future in as_completed(futures):
            target_system, success = future.result()
            if success:
                print(f"Drone {target_system} successfully took off.")
            else:
                print(f"Drone {target_system} will not take off --- due to some error")

def testing():
    # Making connections to all drones
    connections = make_connections()
    handle_simultaneous_takeoff()
    

if __name__ == '__main__':
    testing()




