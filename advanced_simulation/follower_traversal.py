# cd Desktop/sim2_fresh
# ./begin_sim2_v3.sh
# export YOLO_VERBOSE='false'
# python3 ....
#./kill.sh

import time
import os
import cv2
import sys
import math
import rospy
import numpy as np
from utils import * 
from heatmap_utils import *
from ultralytics import YOLO
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, NavSatFix
import matplotlib.pyplot as plt
import asyncio
import contextlib
import io

coords = {0:[-35.3631757, 149.1653398], 1:[-35.3631803, 149.1651279], 2:[-35.3633461, 149.1651348]}  # initial lat long (middle of the grid) of all drones
# coords = {0:[-35.363342, 149.1653417]}
# 0:[-35.363342, 149.1653417], 
grid_size = (20, 20)
flag_array = np.zeros(grid_size)
flag_value = 1 # initial heat value for flags

maxlat = -35.3637033
minlat = -35.3628182
maxlong = 149.1646992
minlong = 149.165782

def Grid_to_LatLong(row, col):
    latx = (maxlat - minlat) / grid_size[0]  # Latitude step per row
    lat = latx * col + minlat

    longy = (maxlong - minlong) / grid_size[1]  # Longitude step per column (reversed direction) as long decreases as we go to 20
    longi = longy * row + minlong

    return lat, longi

def get_locations_in_grid():
    values = []
        
    latx = (maxlat - minlat) / grid_size[0]
    longy = (maxlong - minlong) / grid_size[1]
    
    for _, v in coords.items():
        lat, longi = v[0], v[1]
        col = (lat - minlat) / latx
        row = (longi - minlong) / longy

        row = round(row)
        col = round(col)
        values.append([row,col])
        print(values)
    return values

def suppress_yolo_output():  # redirect stdout to suppress yolo output
    f = io.StringIO()
    with contextlib.redirect_stdout(f):
        return f

def connect_and_go_to_starting_points(coords, height_to_loiter, no_of_drones):
    print("I am in connect and go to starting points, will takeoff MANUALLY")
    connections =  make_connections()[:3] # changed for 3 drones for follower traversal
    conns = []
    for i in range(no_of_drones):
        conns.append(connections[i])

    handle_simultaneous_landing(connections,coords,height_to_loiter)
    time.sleep(10)
    return conns

async def traverse(no_of_drones, has_reached_initial_position, land_these_drones):
    height_to_maintain = 15
    connections = connect_and_go_to_starting_points(coords, height_to_maintain, no_of_drones)
    has_reached_initial_position[0] = True
    init_coords = [coords[i] for i in sorted(coords.keys())]
    landed_drones = []

    drones = get_locations_in_grid()
    
    last_sector = drones.copy()
    spare_drones = drones.copy()
    moves = [[] for _ in range(no_of_drones)]
    spare_drones2 =[]
    paths = [[drone_pos] for drone_pos in drones]
    current_heatmap = load_heatmap()
    poped = []
    num_iterations = 10000
    p = 0.4 # prob of exploration
    file_path = "out.txt"

    for iteration in range(num_iterations):
        for dro in land_these_drones:
            if dro not in landed_drones:  #if NOT already landed
                landed_drones.append(dro)           
                conn = connections[dro]
                land(conn)
                conn.close() 
        try:
            paths,spare_drones2,spare_drones,moves, current_heatmap, last_sector = move_actual(height_to_maintain,init_coords, last_sector, already_detected, connections,drones, current_heatmap,poped,paths,spare_drones2,spare_drones,moves,p, no_of_drones, iteration, file_path, grid_size)
            if paths is None and spare_drones is None and spare_drones2 is None and last_sector is None:
                print("returns-none")
        except:
            if len(poped) == no_of_drones:
                print("Done after ",iteration," iterations.")
                break

    if iteration % 100 == 1:
        print("Drone positions after", iteration, "iterations:")
        print(drones)

def process_disp(disp):
    abs_x_disp = np.abs(disp)  # Take the absolute value
    rounded_x_disp = np.round(abs_x_disp).astype(int) # Round to nearest integer
    # Convert numpy.int64 to Python int
    rounded_x_disp = rounded_x_disp.item() if isinstance(rounded_x_disp, np.int64) else rounded_x_disp
    
    # Handle corner cases
    if rounded_x_disp == -1:
        rounded_x_disp = 0  # if -1, set to 0
    elif rounded_x_disp == grid_size[0]:
        rounded_x_disp = grid_size[0] - 1  # if 100, set to 99
    
    return rounded_x_disp


def global_position_callback(msg, drone_id, drone_positions):
    # rospy.loginfo(f"Updating position for drone {drone_id}")
    drone_positions[drone_id]['latitude'] = msg.latitude
    drone_positions[drone_id]['longitude'] = msg.longitude
    drone_positions[drone_id]['altitude'] = msg.altitude

def land_center(x, y, pixel_width, pixel_height, field_width, field_height, old_lat, old_lon):
    cx, cy = pixel_width / 2, pixel_height / 2

    scale_x = field_width / pixel_width  ## this is m per pixel
    scale_y = field_height / pixel_height

    dx = x - cx   ## diff in pixels
    dy = y - cy

    real_x = dx * scale_x   ## meters hopefully this works
    real_y = dy * scale_y

    if abs(dy)>0:
        marker_latitude, marker_longitude = move_gps(old_lat, old_lon, real_y, 0)
    else:
        marker_latitude, marker_longitude = old_lat, old_lon
    
    if abs(dx)>0:
        marker_latitude, marker_longitude = move_gps(old_lat,old_lon, real_x, 90)

    return marker_latitude, marker_longitude
# marker_latitude, marker_longitude = land_center(
#       center_x,center_y,img_width,img_height,22.5,15,marker_latitude,marker_longitude
# )


def closest_neighbours(detected_flags,step=1):
    all_neighbours = set()
    if step == 1:        # Generate all neighbors within a 1-step radius
        for (row,col) in detected_flags:
            neighbors = [
                (row - 1, col - 1), (row - 1, col), (row - 1, col + 1),
                (row, col - 1),                   (row, col + 1),
                (row + 1, col - 1), (row + 1, col), (row + 1, col + 1)
            ]
            for nghbr in neighbors:
                all_neighbours.add(nghbr)
    elif step == 2:       # Generate all neighbors within a 2-step radius
        for (row,col) in detected_flags:
            neighbors = [
                (row + dr, col + dc)
                for dr in range(-2, 3)  # -2 to +2 for row
                for dc in range(-2, 3)  # -2 to +2 for column
                if not (dr == 0 and dc == 0)  # Exclude the center point (new_area itself)
            ]
            for nghbr in neighbors:
                all_neighbours.add(nghbr)

    return list(all_neighbours)



def image_callback(msg, drone_id, last_image_callback_time, drone_positions, has_reached_initial_position, already_detected, sector_length, land_these_drones):
    callback_interval = 1
    current_time = time.time()
    if current_time - last_image_callback_time >= callback_interval and has_reached_initial_position[0] == True:
        last_image_callback_time = current_time
        try:
            #f = suppress_yolo_output()
            # with contextlib.redirect_stdout(f): 
            cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
            results = model(cv_image)
            img_height, img_width, _ = cv_image.shape
            horizontal_fov = math.radians(0.5)
            focal_length = (img_width / 2) / math.tan(horizontal_fov / 2)
            for result in results:
                for box in result.boxes:
                    conf = box.conf.item()
                    if conf > 0.97:
                        print(f"Detected Marker with Confidence > 97 percent, Already Detected: {already_detected}")
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        center_x = (x1 + x2) // 2
                        center_y = (y1 + y2) // 2
                        
                        marker_x = (center_x - img_width / 2) * drone_positions[drone_id]['altitude'] / focal_length
                        marker_y = (center_y - img_height / 2) * drone_positions[drone_id]['altitude'] / focal_length
                        marker_latitude, marker_longitude = move_gps(
                            drone_positions[drone_id]['latitude'], drone_positions[drone_id]['longitude'], marker_y, 0
                        )
                        marker_latitude, marker_longitude = move_gps(
                            marker_latitude, marker_longitude, marker_x, 90
                        )
                        x_disp, y_disp = displacement_components(coords[drone_id - 1][0], coords[drone_id - 1][1], marker_latitude, marker_longitude)
                        r, c = int(process_disp(x_disp)), int(process_disp(y_disp))
                        row, col = r // sector_length, c // sector_length
                        flag_array[row, col] = flag_value
                        if (row,col) not in closest_neighbours(already_detected):
                            already_detected.add((row, col))
                            print(f"Drone {drone_id}: Marker detected at Latitude {marker_latitude}, Longitude {marker_longitude} with Confidence: {conf}")
                        else:
                            print(f"Drone {drone_id}: LAND ME NOW , I HAVE ALREADY SEEN THIS FLAG @ {(row,col)}")
                            land_these_drones.append(drone_id-1)
        except Exception as error:
            rospy.logerr(f"Error processing image from drone {drone_id}: {error}")

async def main():
    no_of_drones = 3     # represents number of drones in our scenario currently(should match NUM_FLAGS) #keep it low for better convergence          # no_of_drones = 1 means drone 0, connections[0]
    drone_positions = {}
    global model
    global bridge
    global has_reached_initial_position
    global already_detected
    global sector_length # in metres
    global last_image_callback_time
    global landed_drones 
    land_these_drones = []
    sector_length = 5
    has_reached_initial_position = [False]
    last_image_callback_time = 0
    weights_path = '/home/sadaan/Desktop/YOLOv8_NEW/runs/detect/train/weights/best.pt'
    model = YOLO(model = weights_path, verbose = False)
    bridge = CvBridge()
    already_detected = set()
    print("Yolo Initializing...")
    await asyncio.sleep(5)
    rospy.init_node('yolo_v8_multi_drone_detector', anonymous=True)
    for i in range(no_of_drones):
        drone_positions[i + 1] = {'latitude': 0.0, 'longitude': 0.0, 'altitude': 0.0}
        rospy.Subscriber(f"/chase_cam_{i+1}/camera_{i+1}/image_raw", Image, lambda msg: image_callback(msg, i+1, last_image_callback_time, drone_positions, has_reached_initial_position, already_detected,sector_length, land_these_drones))
        rospy.Subscriber(f"/drone{i+1}/mavros/global_position/global", NavSatFix, lambda msg: global_position_callback(msg, i + 1, drone_positions))

    await asyncio.sleep(5)
    print("Yolo Initialized!")
    await asyncio.gather(traverse(no_of_drones, has_reached_initial_position,land_these_drones))

if __name__ == '__main__':
    asyncio.run(main())
