import numpy as np
import matplotlib.pyplot as plt

def vis(heatmap, grid_size):
  # Create subplots
  fig, axes = plt.subplots(1, 1, figsize=(16, 8))

#   # Plot the flag array on the first subplot
#   axes[0].imshow(flag_array, interpolation='nearest')
#   axes[0].set_xticks(np.arange(-0.5, grid_size[0], 1), minor=True)
#   axes[0].set_yticks(np.arange(-0.5, grid_size[1], 1), minor=True)
#   axes[0].grid(which='minor', color='black', linestyle='-', linewidth=0.5)
#   axes[0].invert_yaxis()
#   axes[0].set_title(f'Flag Placement for {num_flags} Flags ({grid_size} Grid)')

  # Plot the heatmap on the second subplot
  heatmap_plot = axes[0].imshow(heatmap, cmap='hot', interpolation='bicubic')
  axes[0].set_xticks(np.arange(-0.5, grid_size[0], 1), minor=True)
  axes[0].set_yticks(np.arange(-0.5, grid_size[1], 1), minor=True)
  axes[0].grid(which='minor', color='black', linestyle='-', linewidth=0.5)
  axes[0].invert_yaxis()
  axes[0].set_title('Gaussian Spread Heatmap')

  # Add color bar to the heatmap
  plt.colorbar(heatmap_plot, ax=axes[0], label='Heatmap Values (Gaussian Distribution)')

  # Display the plots
  plt.tight_layout()
  plt.show()

grid_size = (10, 10)
loaded_heatmap = np.load('heatmap.npy')
vis(loaded_heatmap, grid_size)