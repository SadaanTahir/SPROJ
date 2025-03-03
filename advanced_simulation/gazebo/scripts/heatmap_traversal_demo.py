import cv2
import os
import numpy as np
from utils import * 
import matplotlib.pyplot as plt


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

def create_images_with_paths(paths, heatmap, grid_size=(100, 100), output_folder='FINAL'):
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

def move_sim(drones, current_heatmap,poped,flags,paths,spare_drones2,spare_drones, p=0.1,num_drones=4):

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

        while new_x < 0 or new_x >= 20 or new_y < 0 or new_y >= 20:
            new_x = drones[idx][0] + np.random.choice(moves)
            new_y = drones[idx][1] + np.random.choice(moves) 

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

def move_actual(drones, current_heatmap,poped,flags,paths,spare_drones2,spare_drones, p=0.1,num_drones=4):

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

        while new_x < 0 or new_x >= 20 or new_y < 0 or new_y >= 20:
            new_x = drones[idx][0] + np.random.choice(moves)
            new_y = drones[idx][1] + np.random.choice(moves) 

        drones[idx] = (new_x, new_y)
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






########################################## MAIN CODE ###############################################

# Create a n*n grid (n^2 sectors)
grid_size = (20, 20)

# number of flags,drones and initial heat value for flags
num_flags = 4       
flag_value = 1
NUM_DRONES = 4   #  NUM_DRONES = num_flags currently

flag_array = np.zeros(grid_size)
flag_positions = np.array([317, 95, 322, 84])
for pos in flag_positions:   # Assign flags to the selected positions
    row = pos // grid_size[0]  # Get the row index
    col = pos % grid_size[1]   # Get the column index
    flag_array[row, col] = flag_value

heatmap = create_flag_heatmap(flag_array, grid_size, flag_value)
print("min-heat-val",np.min(heatmap),"max-heat-val",np.max(heatmap))


drones = [(1, 1), (1, 1), (1,1), (1,1)]  
spare_drones = drones.copy()
spare_drones2 =[]
flags = [(pos // 20, pos % 20) for pos in flag_positions] 

print(flags)
paths = [[drone_pos] for drone_pos in drones]
current_heatmap = np.copy(heatmap)
poped = []
num_iterations = 10000
p=0.5 ## probability of random movement

## HERE THE ITERATION IS THE TOTAL ITERATION IN WHICH YOLO IS TO BE INCLUDED
## TO SHOW THAT IT MOVES ONCE AND RETURNS TO LET OTHER THINGS BE PRINTED positions of drones
for iteration in range(num_iterations):
    try:
        paths,flags,spare_drones2,spare_drones,current_heatmap = move_actual(drones, current_heatmap,poped,flags,paths,spare_drones2,spare_drones,p)
    except:
        if len(poped)==NUM_DRONES:
            print("--------------------------------Done after ",iteration," iterations.")
            break

    if iteration % 500 == 1:
        print("Drone positions after", iteration, "iterations:",drones)


# utilities = [calculate_drone_utility(drone_pos, spare_drones2, heatmap, idx) for idx, drone_pos in enumerate(spare_drones2)]
# vis_drones(heatmap, spare_drones2, utilities, paths)
# print(utilities)