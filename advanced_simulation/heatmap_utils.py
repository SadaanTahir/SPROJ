import cv2
import os
import numpy as np
from utils import * 
import matplotlib.pyplot as plt

maxlat = -35.3637033
minlat = -35.3628182
maxlong = 149.1646992
minlong = 149.165782

def Grid_to_LatLong(row, col):
    latx = (maxlat - minlat) / 20  # Latitude step per row
    lat = latx * col + minlat

    longy = (maxlong - minlong) / 20  # Longitude step per column (reversed direction) as long decreases as we go to 20
    longi = longy * row + minlong

    return lat, longi

def load_heatmap():  #load from file
    return np.load('heatmap.npy')

def save_heatmap(heatmap):  #save changes back to file
    np.save('heatmap.npy', heatmap)

def create_flag_heatmap(flag_array, size, flag_value, peak_value=1.0, spread=5.0):
    heatmap = np.zeros(size)
    for i in range(size[0]):
        for j in range(size[1]):
            if flag_array[i, j] == flag_value:  # Check if there is a flag at this location
                # Create Gaussian-like heat spread around the flag
                for x in range(size[0]):
                    for y in range(size[1]):
                        distance = np.sqrt((x - i) ** 2 + (y - j) ** 2)
                        heatmap[x, y] += peak_value * np.exp(-distance / spread)  # Accumulate heat values

    # Normalize the heatmap to range [0, 1]
    heatmap = heatmap / np.max(heatmap)
    return heatmap

def remove_flag_from_heatmap(heatmap, flag_pos, peak_value=1.0, spread=2.0):
    i, j = flag_pos
    for x in range(heatmap.shape[0]):
        for y in range(heatmap.shape[1]):
            distance = np.sqrt((x - i) ** 2 + (y - j) ** 2)
            heatmap[x, y] -= peak_value * np.exp(-distance / spread)
    heatmap[heatmap < 0] = 0 
    return heatmap

def is_covered_by_others(nx, ny, drones, drone_index):
    for j, other_drone_pos in enumerate(drones):
        if j != drone_index:  
            ox, oy = int(other_drone_pos[0]), int(other_drone_pos[1])
            if abs(ox - nx) <= 1 and abs(oy - ny) <= 1:
                return True  
    return False  

def calculate_drone_utility(drone_pos, drones, heatmap, drone_index):
    covered_cells = []
    grid_size = heatmap.shape
    dx, dy = drone_pos
    for nx in range(max(0, dx - 1), min(grid_size[0], dx + 2)): ## check row wise
        for ny in range(max(0, dy - 1), min(grid_size[1], dy + 2)): ## check col wise
            if not is_covered_by_others(nx, ny, drones, drone_index):  
                covered_cells.append((nx, ny))  ## unique ones
    utility = sum(heatmap[nx, ny] for nx, ny in covered_cells)
    return utility

def evaluate_movement_random(drone_pos, drones, heatmap, drone_index):
    best_utility = calculate_drone_utility(drone_pos, drones, heatmap, drone_index)
    best_move = drone_pos  
    dx = np.random.choice([-1, 0, 1])
    dy = np.random.choice([-1, 0, 1])
    new_x = drone_pos[0] + dx
    new_y = drone_pos[1] + dy
    if 0 <= new_x < heatmap.shape[0] and 0 <= new_y < heatmap.shape[1]:
        new_pos = (new_x, new_y)
        new_utility = calculate_drone_utility(new_pos, drones, heatmap, drone_index)
        if new_utility > best_utility:
            best_utility = new_utility
            best_move = new_pos
    return best_move, best_utility

def vis_drones(heatmap, drones, utilities, paths, grid_size=(20, 20)):
    fig, ax = plt.subplots(figsize=(10, 10))
    
    # Plot the heatmap
    heatmap_plot = ax.imshow(heatmap, cmap='hot', interpolation='bicubic')
    ax.set_xticks(np.arange(-0.5, grid_size[0], 1), minor=True)
    ax.set_yticks(np.arange(-0.5, grid_size[1], 1), minor=True)
    ax.grid(which='minor', color='black', linestyle='-', linewidth=0.5)
    ax.invert_yaxis()  # Invert y-axis to match the grid coordinates
    
    # Plot each drone's path and current position with its utility
    for idx, drone_path in enumerate(paths):
        # Convert path to numpy array for indexing
        drone_path = np.array(drone_path)
        
        # Plot the path
        ax.plot(drone_path[:, 1], drone_path[:, 0], 'g-', linewidth=1, label=f'Drone {idx + 1} Path')

        # Plot the current position
        current_pos = drone_path[-1]  # Last position in the path
        ax.plot(current_pos[1], current_pos[0], 'bo', markersize=5, label=f'Drone {idx + 1} (Utility: {utilities[idx]:.2f})')

    plt.colorbar(heatmap_plot, ax=ax, label='Heatmap Value')
    plt.title('Drone Coverage and Utility Visualization with Paths')
    # plt.legend(loc='upper right')
    plt.tight_layout()
    plt.show()

def create_images_with_paths(paths, heatmap, grid_size=(20, 20), output_folder='FINAL'):
    os.makedirs(output_folder, exist_ok=True)

    # Pre-compute some data for efficiency
    heatmap_cmap = plt.get_cmap('hot')
    max_length = max(len(path) for path in paths)  # Get the maximum length of paths

    for i in range(500):
        fig, ax = plt.subplots(figsize=(8, 8))

        # Plot the heatmap
        heatmap_plot = ax.imshow(heatmap, cmap=heatmap_cmap, interpolation='bicubic')
        ax.set_xticks(np.arange(-0.5, grid_size[0], 1), minor=True)
        ax.set_yticks(np.arange(-0.5, grid_size[1], 1), minor=True)
        ax.grid(which='minor', color='black', linestyle='-', linewidth=0.5)
        ax.invert_yaxis()

        # Plot paths for all drones up to the current point
        for player_index, path in enumerate(paths):
            if i < len(path):  # Check if the current index is within the length of the path
                # Ensure path is a NumPy array if it isn't already
                path = np.array(path) if not isinstance(path, np.ndarray) else path

                # Draw the path line and markers
                ax.plot(path[:i + 1, 1], path[:i + 1, 0], linestyle='-', linewidth=1, marker='o', markersize=2, label=f'Drone {player_index + 1} Path')

                # Highlight the current position of each drone
                ax.scatter(path[i, 1], path[i, 0], s=50, marker='X', color='black', label=f'Drone {player_index + 1} Current Position')

        ax.set_title('Drone Path Visualization')
        ax.legend()

        # Save the image using a tight layout
        plt.colorbar(heatmap_plot, ax=ax, label='Heatmap Values')
        plt.tight_layout()
        plt.savefig(f'{output_folder}/image_{i:04d}.png', dpi=100)  # Save image
        plt.close(fig)  # Close the figure to avoid display

def create_video_from_images(output_folder='FINAL', video_file='FINAL.mp4', fps=4):
    # Get the list of image files sorted by name
    images = sorted([img for img in os.listdir(output_folder) if img.endswith(".png")])

    # Check if there are images to process
    if not images:
        print("No images found in the specified folder.")
        return

    # Get dimensions from the first image
    first_image = cv2.imread(os.path.join(output_folder, images[0]))
    height, width, layers = first_image.shape

    # Create a VideoWriter object with MP4 codec
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # Use 'mp4v' for MP4 format
    video = cv2.VideoWriter(video_file, fourcc, fps, (width, height))

    # Write images to video
    for image in images:
        img_path = os.path.join(output_folder, image)
        video.write(cv2.imread(img_path))

    video.release()  # Finalize the video file
    print(f'Video saved as {video_file}')

def move_sim(drones, current_heatmap,poped,flags,paths,spare_drones2,spare_drones, p=0.1,num_drones=4):   #function for testing on notebook simulation

    random_number = np.random.rand()
    if len(poped)==num_drones:
        return None
    
    idx = np.random.choice(range(len(drones)))
    drone_pos = drones[idx]     

    while idx in poped:
        idx = np.random.choice(range(len(drones)))
        drone_pos = drones[idx]
    

    if random_number > p:
        best_move, _ = evaluate_movement_random(drone_pos, drones, current_heatmap, idx)
        
        drones[idx] = best_move  
        paths[idx].append(drones[idx])  

        if drones[idx] in flags:        
            print(f"------------------------------Drone at {drones[idx]} reached a flag.")
            # current_heatmap = remove_flag_from_heatmap(current_heatmap, drones[idx])
            flags.remove(drones[idx])
            paths[idx].append(drones[idx])  
            spare_drones2.append(drones[idx])  
            spare_drones[idx]=drones[idx]   
            poped.append(idx)

            return paths,flags,spare_drones2,spare_drones,current_heatmap
    else:
        moves = [-1,0,1]
        new_x = drones[idx][0] + np.random.choice(moves)
        new_y = drones[idx][1] + np.random.choice(moves)

        # while new_x < 0 or new_x >= 20 or new_y < 0 or new_y >= 20:
        #     new_x = drones[idx][0] + np.random.choice(moves)
        #     new_y = drones[idx][1] + np.random.choice(moves) 

        drone_pos = (new_x, new_y)
        valid = validmove(drone_pos, drones, idx)
        count = 0
        while (new_x < 0 or new_x >= 20 or new_y < 0 or new_y >= 20) and valid:
            new_x = drones[idx][0] + np.random.choice(moves)
            new_y = drones[idx][1] + np.random.choice(moves)
            drone_pos = (new_x, new_y)
            valid = validmove(drone_pos, drones,idx)
            count += 1
            # waits for max 15 iters (it may be the case that all possible movements are resulting in collision --- in that case don't MOVE)
            if count == 15:
                if valid:
                    break
                return paths, flags, spare_drones2, spare_drones, current_heatmap

        drones[idx] = (new_x, new_y)
        paths[idx].append(drones[idx])
        
        if drones[idx] in flags:
            print(f"---------------------Drone at {drones[idx]} reached a flag.")
            # current_heatmap = remove_flag_from_heatmap(current_heatmap, drones[idx])
            flags.remove(drones[idx])
            
            paths[idx].append(drones[idx])  
            spare_drones2.append(drones[idx])  
            spare_drones[idx]=drones[idx] 
            poped.append(idx)  
            return paths,flags,spare_drones2,spare_drones,current_heatmap




############################################### NEW FUNCTIONS ###################################################
def randomly_select_a_drone(drones,poped):
    idx = np.random.choice(range(len(drones)))
    drone_pos = drones[idx]     

    while idx in poped:
        idx = np.random.choice(range(len(drones)))
        drone_pos = drones[idx]
    
    return idx, drone_pos

def move_drone_to_this_sector(height_to_maintain, init_coords, prev_move, conn, move):
    # best move is a tuple with min:0, max:grid_size-1 , basically its the indexwise sector where drone is currently
    # given old sector, new sector, find if its moving left,forward,backward,or right and move accordingly

    if move[1] > prev_move[1]:  
        new_coords = move_gps(init_coords[0], init_coords[1], move[1], 270)  #move forward
    elif move[1] < prev_move[1]:  
        new_coords = move_gps(init_coords[0], init_coords[1], move[1], 90)  #move backward

    if move[0] > prev_move[0]:  
        new_coords =  move_gps(new_coords[0], new_coords[1], move[0], 0)  #move right
    elif move[0] < prev_move[0]:  
        new_coords =  move_gps(new_coords[0], new_coords[1], move[0], 180)  #move left
    
    
    moveglobalfast(conn, new_coords[0], new_coords[1], height_to_maintain)
    return

def stop_drone(conn):
    land(conn)
    conn.close()

def validmove(drone_pos, drones,drone_index):

    low_dist = 99999
    for j, other_drone_pos in enumerate(drones):
        if j != drone_index:
            ox, oy = int(other_drone_pos[0]), int(other_drone_pos[1])
            dist = np.sqrt((ox - drone_pos[0])**2 + (oy - drone_pos[1])**2)
            if dist < low_dist:
                low_dist = dist
    if low_dist > 1:
        return True
    else:
        return False
    
def move_actual(height_to_maintain,init_coords, last_sector, detected_flags, connections, drones, current_heatmap,poped,paths,spare_drones2,spare_drones, moves, p,num_drones, iteration, file_path, grid_size=(20,20)): #function for testing on actual simulation 
    """
        -----------------------------------INPUT PARAMETERS-----------------
        height_to_maintain: height to loiter at while moving
        init_coords: initial latitude and longitude of all drones
        last_sector: previous indexwise sector all the drones were in i.e. (3,4) 
        detected_flags: the list of detected flags, so other drones avoid visiting these flags
        connections: Mavlink connections that will be used to move the drones
        drones: 
        current_heatmap: current heatmap, used in the case when we are updating heatmap, removing visited flags
        poped: the list of drones that have found their respective flags
        paths: traversal paths of all of the drones
        spare_drones:
        spare_drones2:
        p: probability of exploring randomly in the grid

        -----------------------------------OUTPUT PARAMETERS-----------------
        paths:
        spare_drones:
        spare_drones2:
        current_heatmap: updated heatmap if ur updating it (by removing flags for FASTER CONVERGENCE)
        last_sector: previously visited sectors for all drones

    """
        

    random_number = np.random.rand()
    if len(poped)==num_drones:
        return None
    
    time.sleep(0.1)
    idx, drone_pos = randomly_select_a_drone(drones, poped)
    
    if random_number > p:   #play best response
        best_move, _ = evaluate_movement_random(drone_pos, drones, current_heatmap, idx)

        if len(moves[idx]) >0:  #check if its a new move
            lastmovee = moves[idx][-1]
            if lastmovee == best_move:
                return paths,spare_drones2,spare_drones, moves, current_heatmap, last_sector

        drones[idx] = best_move  
        paths[idx].append(drones[idx])  

        lat, longi = Grid_to_LatLong(best_move[0], best_move[1])
        moveglobalfast(connections[idx], lat, longi, height_to_maintain)
        moves[idx].append(best_move)
        # move_drone_to_this_sector(height_to_maintain + idx, init_coords[idx], last_sector[idx], connections[idx], best_move)
        print(f"Iter # {iteration} move drone number {idx} to:", best_move, "as best response")
        print("DETECTED FLAGS:", detected_flags)
        # time.sleep(0.5)
        # content = f"iter#{iteration} move drone number {idx} to: {best_move} as best response\n"      
        # with open(file_path, "a") as file:
        #     file.write(content)
        
        # if drones[idx] in flags:   # this will happen when detected[2] = True  (i.e. drone 2 saw flag) using image_callback function   
        if drones[idx] in detected_flags:  
            print(f"------------------------------Drone at {drones[idx]} reached a flag.")
            # current_heatmap = remove_flag_from_heatmap(current_heatmap, drones[idx])
            paths[idx].append(drones[idx])  
            spare_drones2.append(drones[idx])  
            spare_drones[idx]=drones[idx]   
            poped.append(idx)

            # stop_drone(connections[idx])

            last_sector[idx] = best_move  # update last sector with this move
            return paths,spare_drones2,spare_drones, moves, current_heatmap, last_sector

    else: # play random action (exploration)
        movez = [-1,0,1]
        new_x = drones[idx][0] + np.random.choice(movez)
        new_y = drones[idx][1] + np.random.choice(movez)
        drone_pos = (new_x, new_y)
        valid = validmove(drone_pos, drones, idx)
        count = 0

        while (new_x < 0 or new_x >= grid_size[0] or new_y < 0 or new_y >= grid_size[1]) and valid:
            new_x = drones[idx][0] + np.random.choice(movez)
            new_y = drones[idx][1] + np.random.choice(movez)
            drone_pos = (new_x, new_y)
            valid = validmove(drone_pos, drones,idx)
            count += 1
            # waits for max 15 iters (it may be the case that all possible movements are resulting in collision --- in that case don't MOVE)
            if count == 15:
                if valid:
                    break
                return paths,spare_drones2,spare_drones, moves, current_heatmap, last_sector
            
        
        if len(moves[idx]) >0:  #check if its a new move
            lastmovee = moves[idx][-1]
            # print(len(moves[idx]), lastmovee, (new_x,new_y))
            if lastmovee == (new_x,new_y):
                return paths,spare_drones2,spare_drones, moves, current_heatmap, last_sector
         
        drones[idx] = (new_x, new_y)
        paths[idx].append(drones[idx])
        
        lat, longi = Grid_to_LatLong(new_x, new_y)
        moveglobalfast(connections[idx], lat, longi, height_to_maintain)
        print(f"Iter # {iteration} move drone number {idx} to: (", new_x, "," ,new_y, ") as random move")
        print("DETECTED FLAGS:", detected_flags)
        moves[idx].append((new_x,new_y))
        if drones[idx] in detected_flags:  
            print(f"------------------------------Drone at {drones[idx]} reached a flag.")
            # current_heatmap = remove_flag_from_heatmap(current_heatmap, drones[idx])
            # flags.remove(drones[idx])
            
            paths[idx].append(drones[idx])  
            spare_drones2.append(drones[idx])  
            spare_drones[idx]=drones[idx] 
            poped.append(idx)  

            # stop_drone(connections[idx])
            
            last_sector[idx] = (new_x, new_y)  #update last sector with this move
            return paths,spare_drones2,spare_drones, moves, current_heatmap, last_sector
        
    return paths,spare_drones2,spare_drones, moves, current_heatmap, last_sector