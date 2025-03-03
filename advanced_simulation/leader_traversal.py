# traverse the 10 by 10 grid ASAP in same way as 1st sim
# create heatmap at the end using sectorwise flag_array
import time
import os
import cv2
import sys
import math
import rospy
import numpy as np
from utils import * 
from ultralytics import YOLO
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, NavSatFix
import matplotlib.pyplot as plt
import asyncio
import contextlib
import io

def suppress_yolo_output():
    # redirect stdout to suppress yolo output
    f = io.StringIO()
    with contextlib.redirect_stdout(f):
        return f


# guided -35.362902123877845 149.16568816978538 15  (moves to exact center of the initial 4 sectors of bottom right grid) --- NEED  TO ADJUST CAMERA TO ZOOM OUT AND DECREASE HEIGHT FROM 45 TO 15-20 ISH (for better flag detection)
# coords = [-35.36283224412858, 149.16575308534777]  #initial lat,long (BOTTOM RIGHT of grid), need to change this to center of each sector
# coords = [-35.362902123877845, 149.16568816978538 ]  # center of 4 bottom right sectors of grid

coords = [-35.3628348, 149.1657587] # center of 5m by 5m sector (starting point for leader - bottom right of 20 sectors by 20 sectors grid)
grid_size = (20, 20)
flag_array = np.zeros((grid_size))
flag_value = 1  # initial heat value for flags

def create_flag_heatmap(flag_array, size = grid_size, peak_value = 1.0, spread = 3.5,flag_value = 1):
    heatmap = np.zeros(size)
    for i in range(size[0]):
        for j in range(size[1]):
            if flag_array[i, j] == flag_value:  # Check if there is a flag at this location
                
                # Create Gaussian-like heat spread around the flag
                for x in range(size[0]):
                    for y in range(size[1]):
                        distance = np.sqrt((x - i) ** 2 + (y - j) ** 2)
                        if heatmap[x, y] < 5:
                            print("heatmap value:", heatmap[x, y])
                            heatmap[x, y] += peak_value * np.exp(-distance / spread)  # Accumulate heat values
                        else:
                            print("in else")
                            continue
    return heatmap

def create_and_save_heatmap(flag_array):
    # Create a n*n grid (n^2 sectors)
    heatmap = create_flag_heatmap(flag_array)
        
    # Normalize the heatmap to range [0, 1]
    heatmap = heatmap / np.max(heatmap)
    print("min-heat-val",np.min(heatmap),"max-heat-val",np.max(heatmap))

    #Save the heatmap
    np.save('heatmap.npy', heatmap)
    print("Heatmap saved to file 'heatmap.npy'")


def connect_and_go_to_starting_point(coord, height_to_loiter, sys_id):
    #conn = connect_n_drones(1)[0]
    conn_test = make_connections()
    conn = conn_test[sys_id]
    ready = setmode(conn, 'GUIDED')
    arm(conn)
    time.sleep(2)
    takeoff(conn)
    moveglobal(conn,coord[0],coord[1], height_to_loiter)
    time.sleep(6)
    return conn


def validmove(pt, xmin, ymin, ymax):
    if pt[1] < xmin or pt[1] > (ymax + 100) or pt[0] < ymin or pt[0] > (ymax + 100): # did +10 on ymax
        print("invalid move", pt)
        return False
    return True


def getnextmove(bdry, start, lastmove,step_size,shorter_step=10):

  # Get min and max values from the boundary
  min_x, min_y = bdry[0]
  max_x, max_y = bdry[0]
  for x, y in bdry[1:]:
      min_x = min(min_x, x)
      max_x = max(max_x, x)
      min_y = min(min_y, y)
      max_y = max(max_y, y)
      
  if lastmove == "":
    print("SHOULD NEVER COME TO THIS POINT OF CODE")
    return "", start


  if lastmove == "left":
    start[1] += step_size
    if validmove(start,min_x,min_y,max_y):
      return "backward", tuple(start)
    else:
      start[1] -=step_size
      return "", tuple(start)

  elif lastmove == "right":
    start[1] -=step_size
    if validmove(start,min_x,min_y,max_y):
      return "forward", tuple(start)
    else:
      start[1] +=step_size
      return "", tuple(start)

  elif lastmove == "forward":
    if start[0] + step_size < max_y:
      start[0] +=step_size
      if validmove(start,min_x,min_y,max_y):
        return "forward", tuple(start)
      else:
        start[0] -=step_size
        return "", tuple(start)
    else:
      start[1]+=shorter_step  
      if validmove(start,min_x,min_y,max_y):
        return "left", tuple(start) 
      else:
        start[1] -=shorter_step
        return "", tuple(start)


  elif lastmove == "backward":
    if start[0] - step_size > min_y:
      start[0] -=step_size
      if validmove(start,min_x,min_y,max_y):
        return "backward", tuple(start)
      else:
        start[0] +=step_size
        return "", tuple(start)
    else:
      start[1]+=shorter_step
      if validmove(start,min_x,min_y,max_y):
        return "right", tuple(start) # original
      else:
        start[1] -=shorter_step
        return "", tuple(start)

  else: # first move is a valid move
    start[0] +=step_size
    return "forward", tuple(start)


async def traverse(sys_id, has_reached_initial_position,sector_length):
    height_to_maintain = 15
    conn = connect_and_go_to_starting_point(coords, height_to_maintain, sys_id)
    has_reached_initial_position[0] = True
    maxboundary = 0.95 * sector_length
    geofence = [[(0, 0), (0, grid_size[1]*maxboundary), (grid_size[0]*maxboundary, grid_size[1]*maxboundary), (grid_size[0]*maxboundary, 0)]]
    start = [[-1, 1]]
    step_size = 85  # originally 10
    shorter_step = 10 # for taking LEFTS and RIGHTs  
    thismove = [None] * 1
    notmoving = [0] * 1
    prev = [[0, 0] for _ in range(1)]
    tmp = [[0, 0] for _ in range(1)]
    newcoords = [coords]
    await asyncio.sleep(5)
    while True:
        i = 0
        thismove[i], start[i] = getnextmove(geofence[i], list(start[i]), thismove[i], step_size)
        print(thismove[i], start[i], prev[i],flush=True)

        # Check if the drone is outside the grid boundary
        if start[i][0] < 0 or start[i][0] > 200 or start[i][1] < 0 or start[i][1] > 200 or start[i]==(84, 186): # changed from 100 to 110, wasnt working with 100
            print(f"Drone {i} exceeded grid boundaries. Stopping.",flush=True)
            notmoving[i] = 1
            land(conn)
            conn.close()
            break

        px, py = prev[i]
        oldlat, oldlon = newcoords[i]
        if thismove[i] == "forward":
            newcoords[i] = move_gps(oldlat, oldlon, step_size, 270)
            px -= step_size
        elif thismove[i] == "backward":
            newcoords[i] = move_gps(oldlat, oldlon, step_size, 90)
            px += step_size
        elif thismove[i] == "left":    
            newcoords[i] = move_gps(oldlat, oldlon, shorter_step, 180)
            py += shorter_step
        elif thismove[i] == "right":
            newcoords[i] = move_gps(oldlat, oldlon, shorter_step, 180)
            py -= shorter_step
        else:
            notmoving[i] = 1
            print("stopping drone ", i,flush=True)
            land(conn)
            conn.close()
            break
        tmp[i] = [px, py]
        if notmoving[i] == 0:
            moveglobalfast(conn, newcoords[i][0], newcoords[i][1], height_to_maintain)
            if thismove[i] in ['right','left']:
                time.sleep(4) #2
            else:
                time.sleep(15) #11
        prev[0] = tmp[0]
    print("Simulation ending... Saving heatmap... ",flush=True)
    create_and_save_heatmap(flag_array)

def process_disp(disp):
    # Take the absolute value
    abs_x_disp = np.abs(disp)
    
    # Round to nearest integer
    rounded_x_disp = np.round(abs_x_disp).astype(int)
    
    # Convert numpy.int64 to Python int
    rounded_x_disp = rounded_x_disp.item() if isinstance(rounded_x_disp, np.int64) else rounded_x_disp
    
    # Handle specific cases
    if rounded_x_disp == -1:
        rounded_x_disp = 0  # if -1, set to 0
    elif rounded_x_disp == grid_size[0]:
        rounded_x_disp = grid_size[0] - 1  # if 100, set to 99
    
    return rounded_x_disp


def not_already_detected(new_area, already_detected, step):
    row, col = new_area
    
    if step == 1:        # Generate all neighbors within a 1-step radius
        neighbors = [
            (row - 1, col - 1), (row - 1, col), (row - 1, col + 1),
            (row, col - 1),                   (row, col + 1),
            (row + 1, col - 1), (row + 1, col), (row + 1, col + 1)
        ]
    elif step == 2:       # Generate all neighbors within a 2-step radius
        neighbors = [
            (row + dr, col + dc)
            for dr in range(-2, 3)  # -2 to +2 for row
            for dc in range(-2, 3)  # -2 to +2 for column
            if not (dr == 0 and dc == 0)  # Exclude the center point (new_area itself)
        ]
    else:
       print("implement-this-step")
    
    
    # Combine neighbors with the new_area itself for checking
    all_positions_to_check = neighbors + [new_area]
    
    # Check if any position already exists in the detected set
    for position in all_positions_to_check:
        if position in already_detected:
            return False  # Return False if any position is found
    
    return True

def global_position_callback(msg, drone_id, drone_positions):
    # rospy.loginfo(f"Updating position for drone {drone_id}")
    drone_positions[drone_id]['latitude'] = msg.latitude
    drone_positions[drone_id]['longitude'] = msg.longitude
    drone_positions[drone_id]['altitude'] = msg.altitude

def image_callback(msg, drone_id, last_image_callback_time, drone_positions,has_reached_initial_position, already_detected,sector_length):
    callback_interval = 1 # Changed from 1 to maybe fix lag associated from yolo
    current_time = time.time()
    if current_time - last_image_callback_time >= callback_interval and has_reached_initial_position[0] == True:
        last_image_callback_time = current_time
        try:
            f = suppress_yolo_output()
            with contextlib.redirect_stdout(f): 
                cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
                results = model(cv_image)
            img_height, img_width, _ = cv_image.shape
            horizontal_fov = math.radians(0.5)
            focal_length = (img_width / 2) / math.tan(horizontal_fov / 2)
            for result in results:
                for box in result.boxes:
                    conf = box.conf.item()
                    if conf > 0.85:
                        print("Detected Marker with Confidence: ", conf)
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        center_x = (x1 + x2) // 2
                        center_y = (y1 + y2) // 2
                        
                        # cls = int(box.cls)
                        # label = f"{model.names[cls]} {conf:.2f}"
                        # cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2) 
                        # cv2.putText(cv_image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                        
                        marker_x = (center_x - img_width / 2) * drone_positions[drone_id]['altitude'] / focal_length
                        marker_y = (center_y - img_height / 2) * drone_positions[drone_id]['altitude'] / focal_length
                        
                        marker_latitude, marker_longitude = move_gps(
                            drone_positions[drone_id]['latitude'], drone_positions[drone_id]['longitude'], marker_y, 0
                        )
                        marker_latitude, marker_longitude = move_gps(
                            marker_latitude, marker_longitude, marker_x, 90
                        )
                        x_disp, y_disp = displacement_components(coords[0], coords[1], marker_latitude, marker_longitude)
                        r, c = int(process_disp(x_disp)), int(process_disp(y_disp))      
                        row, col = r // sector_length, c // sector_length

                        if not_already_detected((row,col), already_detected, 2):
                            # filename = f"marker_image_at_{row}_{col}.jpg"
                            # cv2.imwrite(filename, cv_image)
                            print("row before division, col before division:", r, c)
                            print(f'displacement in x {x_disp}, displacement in y {y_disp} maps to [row,col] = [{row},{col}]')
                            print(f"Drone {drone_id}: Marker detected at Latitude {marker_latitude}, Longitude {marker_longitude}")
                            already_detected.append((row,col))
                            flag_array[row, col] = flag_value
        except Exception as e:
            rospy.logerr(f"Error processing image from drone {drone_id}: {e}")

async def main():
    sys_id = 1
    drone_positions = {sys_id: {'latitude': 0.0, 'longitude': 0.0, 'altitude': 0.0}}
    global model
    global bridge
    global last_image_callback_time
    global has_reached_initial_position
    global already_detected
    global sector_length    #in metres
    sector_length = 5       #originally was 10, when we were using grid_size (10,10)
    already_detected = []
    has_reached_initial_position = [False]
    last_image_callback_time = 0
    # print('YOLO VERBOSE IS SET TO ...',os.environ['YOLO_VERBOSE'])
    weights_path = '/home/sadaan/Desktop/YOLOv8_weights/best.pt'
    model = YOLO(model=weights_path, verbose=False)
    bridge = CvBridge()
    print("Yolo Initializing...")
    await asyncio.sleep(5)
    rospy.init_node('yolo_v8_multi_drone_detector', anonymous=True)
    rospy.Subscriber(f"/chase_cam_{sys_id}/camera_{sys_id}/image_raw", Image, lambda msg: image_callback(msg, sys_id, last_image_callback_time, drone_positions, has_reached_initial_position, already_detected,sector_length))
    rospy.Subscriber(f"/drone{sys_id}/mavros/global_position/global", NavSatFix, lambda msg: global_position_callback(msg, sys_id, drone_positions))
    await asyncio.sleep(5)
    print("Yolo Initialized!")
    await asyncio.gather(traverse(sys_id - 1, has_reached_initial_position,sector_length))
    print("flag values:", already_detected,flush=True)

if __name__ == '__main__':
    asyncio.run(main())
