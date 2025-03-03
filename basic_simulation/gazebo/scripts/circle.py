import time
import math
from pymavlink import mavutil

def move_in_circle_loop(conn, center_lat, center_long, radius=0.001, altitude=10, speed=1, period=60):
    """
    Moves the drone in a continuous circular path around a given center point.

    :param conn: The pymavlink connection object
    :param center_lat: Latitude of the circle's center
    :param center_long: Longitude of the circle's center
    :param radius: Radius of the circle in degrees (approx. 1 degree ~ 111km, so small radius values are in degrees)
    :param altitude: Altitude to maintain in meters
    :param speed: Speed of movement in degrees per second (0.001 degrees per second is a small movement)
    :param period: Time period in seconds for one full circle
    """
    start_time = time.time()
    while True:
        elapsed_time = time.time() - start_time
        angle = (2 * math.pi * elapsed_time / period) % (2 * math.pi)  # Current angle in radians
        
        target_lat = center_lat + radius * math.sin(angle)
        target_long = center_long + radius * math.cos(angle)

        # Send the target position command to the drone
        mask = GLOBAL_SET_POSITION_DICTIONARY['pos']

        conn.mav.send(mavutil.mavlink.MAVLink_set_position_target_global_int_message(
            10, conn.target_system, conn.target_component, 
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mask, 
            int(target_lat * 10 ** 7), int(target_long * 10 ** 7), 
            altitude + 1, 0, 0, 0, 0, 0, 0, 0, 0
        ))

        # Wait before sending the next command
        time.sleep(1)  # Adjust based on your needs and speed

def testing():
    #Making connections to all drones
    connections = make_connections()
    coords = [[ -35.3632709,149.1653365],[-35.3631819,149.1653374],[-35.3631816,149.1652272],[-35.3632709,149.1652273]]  #lat,long
    move_in_circle_loop(connections[0],coords[0][0],coords[0][1], )

if __name__ == '__main__':
    testing()