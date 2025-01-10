import yaml
import numpy as np
import matplotlib.pyplot as plt

# Path to the YAML file
results_dir = "/home/marcus/orchard_template_ws/results_data/"
results_a = "experiment_a_results.yaml"
results_b = "experiment_b_results.yaml"
results_c = "experiment_c_results.yaml"
results_d = "experiment_d_results.yaml"
results_e = "experiment_e_results.yaml"
results_f = "experiment_f_results.yaml"
results_g = "experiment_g_results.yaml"
results_h = "experiment_h_results.yaml"
results_i = "experiment_i_results.yaml"
results_j = "experiment_j_results.yaml"
results_files = [results_a, results_b, results_c, results_d, results_e, results_f, results_g, results_h, results_i, results_j]  

# Function to extract the required data
def extract_reachable_apple_data(file_path, dir="/home/marcus/orchard_template_ws/results_data/"):
    with open(dir + file_path, "r") as file:
        data = yaml.safe_load(file)
    
    apples_found = data.get("apples_found", None)
    apples_reached_templating = data.get("apples_reached_templating", None)
    apples_reached_voxelization = data.get("apples_reached_voxelization", None)
    unreached_idx_voxelization = data.get("unreached_idx_voxelization", None)
    unreached_idx_templating = data.get("unreached_idx_templating", None)
    apple_coordinates = data.get("apple_coordinates", None)
    side_branch_locations = data.get("side_branch_locations", None)
    
    return {
        "apples_found": apples_found,
        "apples_reached_templating": apples_reached_templating,
        "apples_reached_voxelization": apples_reached_voxelization,
        "unreached_idx_voxelization": unreached_idx_voxelization,
        "unreached_idx_templating": unreached_idx_templating,
        "apple_coords": apple_coordinates,
        "side_branch_locations": side_branch_locations
    }

def get_avg_reachable_rate(data_filenames):
    reachable_rates_template = []
    reachable_rates_voxel = []
    for i, filenames in enumerate(data_filenames):
        apple_data = extract_reachable_apple_data(filenames)

        reachable_rates_template.append(apple_data['apples_reached_templating'] / apple_data['apples_found'])
        reachable_rates_voxel.append(apple_data['apples_reached_voxelization'] / apple_data['apples_found'])

    avg_reachble_rate_template = np.average(reachable_rates_template)
    avg_reachble_rate_voxel = np.average(reachable_rates_voxel)

    print(f'Average reachable rate of templating method: {np.round(avg_reachble_rate_template * 100, 1)}%')
    print(f'Average reachable rate of voxelization method: {np.round(avg_reachble_rate_voxel * 100, 1)}%')

def norm_apple_coord_to_branch(branch_location, apple_coords):
    """
    Normalize apple coordinates relative to the branch location.

    Parameters:
    branch_location (list or np.array): A 3D point representing the branch location.
    apple_coords (list of lists or np.array): A list of 3D points representing apple coordinates.

    Returns:
    np.array: A list of normalized 3D points.
    """
    branch_location = np.array(branch_location)
    apple_coords = np.array(apple_coords)
    
    normalized_coords = apple_coords - branch_location

    # Calculate Euclidean distance on the yz plane
    distances_yz = np.sqrt((normalized_coords[:, 1])**2 + (normalized_coords[:, 2])**2)

    return normalized_coords, distances_yz

def plot_apple_reachability(data_filenames, side_branch_idx=1, radii_threshold=0.3, figsize=(10, 6)):
    total_unreached_coords = []
    total_reached_coords = []
    for i, filename in enumerate(data_filenames):
        apple_data = extract_reachable_apple_data(filename)

        apple_coordinates = apple_data['apple_coords']
        unreached_idx_templating = apple_data['unreached_idx_templating']
        side_branch_locations = apple_data['side_branch_locations']

        # Normalize apple coordinates
        normalized_coords, distances_yz = norm_apple_coord_to_branch(side_branch_locations[side_branch_idx], apple_coordinates)

        unreached_coords = np.array(normalized_coords)[unreached_idx_templating]
        reached_coords = np.array(normalized_coords)[[i for i in range(len(normalized_coords)) if i not in unreached_idx_templating]]

        unreached_radii = distances_yz[unreached_idx_templating]
        reached_radii = distances_yz[[i for i in range(len(normalized_coords)) if i not in unreached_idx_templating]]

        # Filter out coordinates with radii greater than the radiance threshold
        unreached_coords = unreached_coords[unreached_radii <= radii_threshold]
        reached_coords = reached_coords[reached_radii <= radii_threshold]

        # Append the coordinates to the total list
        total_unreached_coords.append(unreached_coords)
        total_reached_coords.append(reached_coords)

    total_unreached_coords = np.vstack(total_unreached_coords)
    total_reached_coords = np.vstack(total_reached_coords)

    # Create the plot
    fig, ax = plt.subplots(figsize=figsize)

    # Plot reachable coordinates
    ax.scatter(total_reached_coords[:, 1], total_reached_coords[:, 2], color='green', label='Reachable Apples')
    
    # Plot unreachable coordinates
    ax.scatter(total_unreached_coords[:, 1], total_unreached_coords[:, 2], color='red', label='Unreachable Apples')

    # Plot a circle at the origin point with the target radius (side branch)
    circle = plt.Circle((0, 0), 0.04, color='brown', fill=False, linestyle='-', linewidth=2, label='Side Branch')
    ax.add_patch(circle)

    # Ensure the aspect ratio is equal to make the circle appear correctly
    ax.set_aspect('equal', adjustable='box')

    # Label the axes
    ax.set_xlabel('y')
    ax.set_ylabel('z')

    # Set the axis limits
    ax.set_xlim(-radii_threshold, radii_threshold)
    ax.set_ylim(-radii_threshold, radii_threshold)

    # Add a legend
    ax.legend()

    # Show the plot
    plt.grid(True)
    plt.show()


if __name__ == '__main__':
    # Plot the reachability of apples
    plot_apple_reachability(['v2/experiment_a_results.yaml', 'v2/experiment_b_results.yaml', 'v2/experiment_c_results.yaml', 'v2/experiment_d_results.yaml'])

    # get_avg_reachable_rate(results_files)