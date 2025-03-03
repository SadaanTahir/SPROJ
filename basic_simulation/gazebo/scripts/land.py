#!/usr/bin/env python
import rospy
from concurrent.futures import ThreadPoolExecutor, as_completed
from utils import make_connections, setmode,land


def simland(conn):
    """
    Function to land a single drone.
    """
    #setmode(conn, 'GUIDED')
    land(conn)
    conn.close()
    return conn.target_system, True

def handle_simultaneous_land(connections):
        # Create a ThreadPoolExecutor to handle drones concurrently
    with ThreadPoolExecutor(max_workers=len(connections)) as executor:
        # Submit all tasks to the executor
        futures = [executor.submit(simland, conn) for conn in connections]

        # Process the results as they complete
        for future in as_completed(futures):
            target_system, success = future.result()
            print("Drone landed")

def testing():
    # Making connections to all drones
    connections = make_connections()
    handle_simultaneous_land(connections)
    
if __name__ == '__main__':
    testing()




