import yaml
import numpy as np
import matplotlib.pyplot as plt

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

def get_branch_position(branch_radii, angle_deg, length, span='x'):
    angle_rad = np.deg2rad(angle_deg)

    # Direction vector components
    dx = np.cos(angle_rad)
    dy = np.sin(angle_rad)

    if span == 'x':
        x0 = 0
        y0_0 = -branch_radii
        y0_1 = branch_radii

        # Define line endpoints extending in both directions
        x1 = x0 - dx * length
        x2 = x0 + dx * length

        y1_0 = y0_0 - dy * length
        y2_0 = y0_0 + dy * length
        y1_1 = y0_1 - dy * length
        y2_1 = y0_1 + dy * length

        return x1, x2, y1_0, y2_0, y1_1, y2_1
    elif span == 'y':
        x0_0 = -branch_radii
        x0_1 = branch_radii
        y0 = 0.0

        # Define line endpoints extending in both directions
        x1_0 = x0_0 - dx * length
        x2_0 = x0_0 + dx * length
        x1_1 = x0_1 - dx * length
        x2_1 = x0_1 + dx * length

        y1 = y0 - dy * length
        y2 = y0 + dy * length
        return x1_0, x2_0, x1_1, x2_1, y1, y2  

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

        # Create the mask for the radii threshold
        radii_threshold_mask = distances_yz <= radii_threshold

        # Apply the mask to unreached_idx_templating to filter out invalid indices
        valid_unreached_mask = radii_threshold_mask[unreached_idx_templating]
        filtered_unreached_idx_templating = np.array(unreached_idx_templating)[valid_unreached_mask]

        # Extract unreached and reached coordinates
        unreached_coords = normalized_coords[np.isin(range(len(normalized_coords)), filtered_unreached_idx_templating)] # "is unreached"

        reached_coords = normalized_coords[~np.isin(range(len(normalized_coords)), filtered_unreached_idx_templating)] # "is reached"
        
        # Apply the radii threshold mask to the reached coordinates only
        reached_coords = reached_coords[radii_threshold_mask[~np.isin(range(len(normalized_coords)), filtered_unreached_idx_templating)]]

        # Append the coordinates to the total list
        total_unreached_coords.append(unreached_coords)
        total_reached_coords.append(reached_coords)

    total_unreached_coords = np.vstack(total_unreached_coords)
    total_reached_coords = np.vstack(total_reached_coords)

    # Create the plot
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=figsize, sharey=True)
    # fig, (ax0, ax1, ax2) = plt.subplots(1, 3, figsize=figsize)

    ###############################
    # unreached_coords = np.array(apple_coordinates)[unreached_idx_templating]
    # reached_coords = np.array(apple_coordinates)[[i for i in range(len(normalized_coords)) if i not in unreached_idx_templating]]
    # # Plot reachable coordinates
    # ax0.scatter(reached_coords[:, 0], reached_coords[:, 2], color='green', label='Reachable Apples')

    # # Plot unreachable coordinates
    # ax0.scatter(unreached_coords[:, 0], unreached_coords[:, 2], color='red', label='Unreachable Apples')

    # # Set the axis limits
    # ax0.set_xlim(-1.0, 1.0)
    # ax0.set_ylim(0.3, 1.3)
    # ax0.set_aspect('equal', adjustable='box')
    ###############################

    ### DATA FOR LEFTMOST PLOT ###
    x1, x2, y1_0, y2_0, y1_1, y2_1 = get_branch_position(0.04, 0, 2.0, span='x')
    ax1.plot([x1, x2], [y1_0, y2_0], linestyle='-', label=f'Side Branch', color='brown')
    ax1.plot([x1, x2], [y1_1, y2_1], linestyle='-', color='brown')

    x1_0, x2_0, x1_1, x2_1, y1, y2 = get_branch_position(0.08, 90, 1.0, span='y')
    ax1.plot([x1_0, x2_0], [y1, y2], 'b-', label=f'Leader Branch')
    ax1.plot([x1_1, x2_1], [y1, y2], 'b-')

    # Plot reachable coordinates
    ax1.scatter(total_reached_coords[:, 0], total_reached_coords[:, 2], color='green', label='Reachable Apples')
    
    # Plot unreachable coordinates
    ax1.scatter(total_unreached_coords[:, 0], total_unreached_coords[:, 2], color='red', label='Unreachable Apples')

    # Ensure the aspect ratio is equal to make the circle appear correctly
    ax1.set_aspect('equal', adjustable='box')

    # Label the axes
    ax1.set_xlabel('x')
    ax1.set_ylabel('z')

    # Set the axis limits
    ax1.set_xlim(-1.0, 1.0)
    ax1.set_ylim(-radii_threshold, radii_threshold)
    ax1.grid(True)
    ax1.legend(loc='upper left')
    ax1.set_title('Front View')

    ### DATA FOR RIGHTMOST PLOT ###
    # Convert angle to radians
    angle_deg = 90 + 18.435
    x1_0, x2_0, x1_1, x2_1, y1, y2 = get_branch_position(0.08, angle_deg, 1.0, span='y')

    # Plotting the line
    ax2.plot([x1_0, x2_0], [y1, y2], 'b-', label=f'Leader Branch')
    ax2.plot([x1_1, x2_1], [y1, y2], 'b-')

    # Plot reachable coordinates
    ax2.scatter(total_reached_coords[:, 1], total_reached_coords[:, 2], color='green', label='Reachable Apples')
    
    # Plot unreachable coordinates
    ax2.scatter(total_unreached_coords[:, 1], total_unreached_coords[:, 2], color='red', label='Unreachable Apples')

    # Plot a circle at the origin point with the target radius (side branch)
    circle = plt.Circle((0, 0), 0.04, color='brown', fill=False, linestyle='-', linewidth=2, label='Side Branch')
    ax2.add_patch(circle)

    # Ensure the aspect ratio is equal to make the circle appear correctly
    ax2.set_aspect('equal', adjustable='box')

    # Label the axes
    ax2.set_xlabel('y')
    ax2.set_ylabel('z')

    # Set the axis limits
    ax2.set_xlim(-radii_threshold, radii_threshold)
    ax2.set_ylim(-radii_threshold, radii_threshold)
    ax2.set_title('Side View')

    # Add a legend
    ax2.grid(True)

    # Show the plot
    plt.tight_layout()
    plt.show()

def test_plot(data_filenames, side_branch_idx=1, radii_threshold=0.3, figsize=(10, 6)):
    total_unreached_coords = []
    total_reached_coords = []
    for i, filename in enumerate(data_filenames):
        apple_data = extract_reachable_apple_data(filename)

        apple_coordinates = apple_data['apple_coords']
        unreached_idx_templating = apple_data['unreached_idx_templating']

        apple_coordinates = np.array(apple_coordinates)

        # Extract unreached and reached coordinates
        unreached_coords = apple_coordinates[np.isin(np.arange(len(apple_coordinates)), unreached_idx_templating)]
        reached_coords = apple_coordinates[~np.isin(np.arange(len(apple_coordinates)), unreached_idx_templating)]

        # Append the coordinates to the total list
        total_unreached_coords.append(unreached_coords)
        total_reached_coords.append(reached_coords)

    total_unreached_coords = np.vstack(total_unreached_coords)
    total_reached_coords = np.vstack(total_reached_coords)

    # Create the plot
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=figsize, sharey=True)

    # Plot reachable coordinates
    ax1.scatter(total_reached_coords[:, 0], total_reached_coords[:, 2], color='green', label='Reachable Apples')
    
    # Plot unreachable coordinates
    ax1.scatter(total_unreached_coords[:, 0], total_unreached_coords[:, 2], color='red', label='Unreachable Apples')

    # Ensure the aspect ratio is equal to make the circle appear correctly
    ax1.set_aspect('equal', adjustable='box')

    # Label the axes
    ax1.set_xlabel('x')
    ax1.set_ylabel('z')

    # Set the axis limits
    ax1.set_xlim(-1.0, 1.0)
    ax1.set_ylim(0.5, 1.0)
    ax1.grid(True)
    ax1.legend(loc='upper left')
    ax1.set_title('Front View')

    ### DATA FOR RIGHTMOST PLOT ###
    # Plot reachable coordinates
    ax2.scatter(total_reached_coords[:, 1], total_reached_coords[:, 2], color='green', label='Reachable Apples')
    
    # Plot unreachable coordinates
    ax2.scatter(total_unreached_coords[:, 1], total_unreached_coords[:, 2], color='red', label='Unreachable Apples')

    # Ensure the aspect ratio is equal to make the circle appear correctly
    ax2.set_aspect('equal', adjustable='box')

    # Label the axes
    ax2.set_xlabel('y')
    ax2.set_ylabel('z')

    # Set the axis limits
    ax2.set_xlim(0.5, 1.0)
    ax2.set_ylim(0.5, 1.0)
    ax2.set_title('Side View')

    # Add a legend
    ax2.grid(True)

    # Show the plot
    plt.tight_layout()
    plt.show()


if __name__ == '__main__':
    # Path to the YAML file
    version = "v2"

    results_a = f"{version}/experiment_a_results.yaml"
    results_b = f"{version}/experiment_b_results.yaml"
    results_c = f"{version}/experiment_c_results.yaml"
    results_d = f"{version}/experiment_d_results.yaml"
    results_e = f"{version}/experiment_e_results.yaml"
    results_f = f"{version}/experiment_f_results.yaml"
    results_g = f"{version}/experiment_g_results.yaml"
    results_h = f"{version}/experiment_h_results.yaml"
    results_i = f"{version}/experiment_i_results.yaml"
    results_j = f"{version}/experiment_j_results.yaml"
    results_files = [results_a, results_b, results_c, results_d, results_e, results_f, results_g, results_h, results_i, results_j]  

    # Get the average reachable rate
    get_avg_reachable_rate(results_files)
    
    # Plot the reachability of apples (results idx 7 looks the best!)
    # plot_apple_reachability([results_files[7]], figsize=(18, 10)) # Plot reachability of a single experiment
    # plot_apple_reachability(results_files, figsize=(18, 10), radii_threshold=0.25) # Plot reachability of a multiple experiments

    plot_apple_reachability(['tests/experiment_a_closer_camera_greater_tol.yaml'], radii_threshold=0.25)
    # plot_apple_reachability(['v2/experiment_a_results.yaml'])
    # test_plot(['tests/experiment_a_less_ori_tol.yaml'])
