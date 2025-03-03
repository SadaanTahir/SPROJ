#!/usr/bin/env python
from utils_final import *
from concurrent.futures import ThreadPoolExecutor, as_completed
from geopy.distance import distance


def move_gps(lat, lon, meters,dir):
    # Define the starting point
    start_point = (lat, lon)

    # Calculate the new point after moving forward by 'meters'
    new_point = distance(meters=meters).destination(start_point, dir)
    
    return new_point.latitude, new_point.longitude

def simultaneous_land(conn,coord):
    """
    Function to set the mode, arm, and takeoff, and land a single drone to its initial location.
    """
    ready = setmode(conn, 'GUIDED')
    if ready:
        arm(conn)
        takeoff(conn)
        moveglobal(conn,coord[0],coord[1])
        # land(conn)
        time.sleep(3)
        lat,lon = move_gps(coord[0],coord[1],3,0)
        moveglobal(conn,lat,lon)
        conn.close()
        return conn.target_system, True
    else:
        return conn.target_system, False

def handle_simultaneous_landing(connections,coordinates):
        # Create a ThreadPoolExecutor to handle drones concurrently
    with ThreadPoolExecutor(max_workers=len(connections)) as executor:
        # Submit all tasks to the executor
        futures = [executor.submit(simultaneous_land, conn, coordinates[i]) for i,conn in enumerate(connections)]

        # Process the results as they complete
        for future in as_completed(futures):
            target_system, success = future.result()
            if success:
                print(f"Drone {target_system} successfully landed to its initial location.")
            else:
                print(f"Drone {target_system} will not take off --- due to some error")


def testing():
    #Making connections to all drones
    connections = make_connections()
    coords = [[ -35.3632709,149.1653365],[-35.3631819,149.1653374],[-35.3631816,149.1652272],[-35.3632709,149.1652273]]  #lat,long
    handle_simultaneous_landing(connections,coords)

if __name__ == '__main__':
    testing()

# drone1:
# latitude: -35.3632709
# longitude: 149.1653365

# drone2:
# latitude: -35.3631819
# longitude: 149.1653374

# drone3:
# latitude: -35.3631816
# longitude: 149.1652272

# drone4:
# latitude: -35.3632709
# longitude: 149.1652273
