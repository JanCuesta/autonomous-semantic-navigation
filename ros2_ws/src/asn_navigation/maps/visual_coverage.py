import os
import math
import yaml
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image

robot_locations = [(1,-1), (3,-2), (4,-4)]

def plot_robots_visual(ax, locations,resolution):
    """
    Overlays the robots and their circular camera ranges onto the provided axes.
    """
    camera_range = 8*resolution
    
    # Loop through each robot coordinate pair
    for array in locations:

        # 1. Generate the Circle patch object
        circle = plt.Circle(array, camera_range, facecolor='lightblue', edgecolor='blue', alpha=0.6, linewidth=2)
        
        # 2. Explicitly add the patch to the active map axes
        ax.add_patch(circle)
        
        # Optional: Plot a small red dot at the absolute center of each robot for precision
        ax.plot(array[0], array[1], 'ro', markersize=5)


def plot_ros_map(yaml_path):
    # 1. Load and parse the YAML file
    with open(yaml_path, 'r') as file:
        map_metadata = yaml.safe_load(file)
    
    # Extract important metadata
    image_filename = map_metadata['image']
    resolution = map_metadata['resolution']  # meters/pixel
    origin = map_metadata['origin']          # [x, y, yaw]
    
    # Handle relative paths for the PGM file
    yaml_dir = os.path.dirname(yaml_path)
    pgm_path = os.path.join(yaml_dir, image_filename)
    
    # 2. Open the PGM image using PIL
    try:
        img = Image.open(pgm_path)
        img_array = np.array(img)
    except FileNotFoundError:
        print(f"Error: Could not find the PGM file at {pgm_path}")
        return


    # 3. Calculate spatial boundaries (Extent) for plotting
    height, width = img_array.shape[:2]
    
    x_min = origin[0]
    x_max = x_min + (width * resolution)
    y_min = origin[1]
    y_max = y_min + (height * resolution)
    
    # 4. Create the figure and save a reference to the 'ax' object
    fig, ax = plt.subplots(figsize=(10, 8))
    
    # 'extent' aligns the image coordinates with real-world meters
    # 'origin="lower"' matches standard ROS coordinate frames
    ax.imshow(img_array, cmap='gray', extent=[x_min, x_max, y_min, y_max], origin='lower')
    
    # Add labels and styling
    ax.set_title(f"Map: {image_filename} ({resolution} m/px)", fontsize=14)
    ax.set_xlabel("X (meters)", fontsize=12)
    ax.set_ylabel("Y (meters)", fontsize=12)
    ax.grid(True, color='red', alpha=0.3, linestyle='--')
    
    # Ensure standard aspect ratio so your camera circles don't look warped/stretched
    ax.set_aspect('equal', adjustable='box')

    # 5. Call the modified visualization function passing 'ax' and ALL robot locations
    plot_robots_visual(ax, robot_locations,resolution)

    plt.show()

# Run the plotting execution
plot_ros_map(os.path.expanduser(
    '~/projects/autonomous-semantic-navigation/ros2_ws/src/asn_navigation/maps/v4_sim_map.yaml'
))

