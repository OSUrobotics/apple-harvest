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
    apples_found_all_exp = []
    apples_reached_template = []
    apples_reached_voxel = []
    for i, filenames in enumerate(data_filenames):
        apple_data = extract_reachable_apple_data(filenames)

        reachable_rates_template.append(apple_data['apples_reached_templating'] / apple_data['apples_found'])
        reachable_rates_voxel.append(apple_data['apples_reached_voxelization'] / apple_data['apples_found'])
        apples_found_all_exp.append(apple_data['apples_found'])
        apples_reached_template.append(apple_data['apples_reached_templating'])
        apples_reached_voxel.append(apple_data['apples_reached_voxelization'])

    avg_reachble_rate_template = np.average(reachable_rates_template)
    avg_reachble_rate_voxel = np.average(reachable_rates_voxel)

    print(f'Average reachable rate of templating method: {np.round(avg_reachble_rate_template * 100, 1)}%')
    print(f'Average reachable rate of voxelization method: {np.round(avg_reachble_rate_voxel * 100, 1)}%')

    return reachable_rates_template, reachable_rates_voxel, apples_found_all_exp, apples_reached_template, apples_reached_voxel

def filter_apple_coords_ur5_reachability(apple_coords, ur5_w_gripper_reachable_radius=1.25):
    """
    Filter apple coordinates based on the UR5 reachability.

    Parameters:
    apple_coords (list of lists or np.array): A list of 3D points representing apple coordinates.
    ur5_w_gripper_reachable_radius (float): The maximum reachable radius of the UR5 with the gripper.

    Returns:
    np.array: A list of 3D points that are reachable by the UR5 with the gripper.
    """
    apple_coords = np.array(apple_coords)
    apple_coords[:, 1] -= 0.04  # Offset the apple coordinates by the approach distance

    distances = np.sqrt((apple_coords[:, 0])**2 + (apple_coords[:, 1])**2 + (apple_coords[:, 2])**2)

    # Filter the apple coordinates based on the reachable radius
    unreachable_mask = distances >= ur5_w_gripper_reachable_radius

    return np.where(unreachable_mask)[0]

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

def plot_norm_apple_reachability(data_filenames, side_branch_idx=1, radii_threshold=0.3, figsize=(10, 6), branch_alpha=0.2, turtle_green=(150/255, 207/255, 103/255), light_purple=(150/255, 81/255, 206/255)):
    total_unreached_coords = []
    total_reached_coords = []
    total_unreached_via_ur5 = []
    for i, filename in enumerate(data_filenames):
        apple_data = extract_reachable_apple_data(filename)

        apple_coordinates = apple_data['apple_coords']
        unreached_idx_templating = apple_data['unreached_idx_templating']
        side_branch_locations = apple_data['side_branch_locations']

        # Filter apple coordinates based on UR5e reachable radius
        unreached_idx_via_ur5 = filter_apple_coords_ur5_reachability(apple_coordinates)

        # Normalize apple coordinates
        normalized_coords, distances_yz = norm_apple_coord_to_branch(side_branch_locations[side_branch_idx], apple_coordinates)

        # Create the mask for the radii threshold
        radii_threshold_mask = distances_yz <= radii_threshold

        # Apply the mask to unreached_idx_templating to filter out invalid indices
        valid_unreached_mask = radii_threshold_mask[unreached_idx_templating]
        filtered_unreached_idx_templating = np.array(unreached_idx_templating)[valid_unreached_mask]

        # Extract unreached and reached coordinates
        unreached_coords = normalized_coords[np.isin(range(len(normalized_coords)), filtered_unreached_idx_templating)]
        reached_coords = normalized_coords[~np.isin(range(len(normalized_coords)), filtered_unreached_idx_templating)]
        unreached_coords_via_ur5 = normalized_coords[np.isin(range(len(normalized_coords)), unreached_idx_via_ur5)]
        
        # Apply the radii threshold mask to the reached coordinates only
        reached_coords = reached_coords[radii_threshold_mask[~np.isin(range(len(normalized_coords)), filtered_unreached_idx_templating)]]

        # Append the coordinates to the total list
        total_unreached_coords.append(unreached_coords)
        total_reached_coords.append(reached_coords)
        total_unreached_via_ur5.append(unreached_coords_via_ur5)

    total_unreached_coords = np.vstack(total_unreached_coords)
    total_reached_coords = np.vstack(total_reached_coords)
    total_unreached_via_ur5 = np.vstack(total_unreached_via_ur5)

    # Combine all arrays to find the global min/max y-values
    all_coords = np.vstack([total_unreached_coords, total_reached_coords, total_unreached_via_ur5])
    y_vals = all_coords[:, 1] 
    y_min, y_max = np.min(y_vals), np.max(y_vals)

    # Compute the scatter plot dot sizes based on the depth values
    def compute_sizes(coords, depth_min, depth_max):
        depth_vals = coords[:, 1] 
        sizes = ((depth_max - depth_vals) / (depth_max - depth_min)) * 150 + 20  # Scale and shift sizes
        return sizes

    # Scatter plot sizes
    reached_sizes = compute_sizes(total_reached_coords, y_min, y_max)
    unreached_sizes = compute_sizes(total_unreached_coords, y_min, y_max)
    unreached_ur5_sizes = compute_sizes(total_unreached_via_ur5, y_min, y_max)

    ### FIRST FIGURE: FRONT VIEW ###
    # fig1, ax1 = plt.subplots(figsize=figsize)
    fig1, ax1 = plt.subplots()

    x1, x2, y1_0, y2_0, y1_1, y2_1 = get_branch_position(0.04, 0, 2.0, span='x')
    ax1.plot([x1, x2], [y1_0, y2_0], linestyle='-', color='brown', alpha=branch_alpha-0.1)
    ax1.plot([x1, x2], [y1_1, y2_1], linestyle='-', color='brown', alpha=branch_alpha-0.1)
    ax1.fill_between([x1, x2], [y1_0, y2_0], [y1_1, y2_1], color='brown', alpha=branch_alpha)

    x1_0, x2_0, x1_1, x2_1, y1, y2 = get_branch_position(0.08, 90, 1.0, span='y')
    ax1.plot([x1_0, x2_0], [y1, y2], '-', color='brown', alpha=branch_alpha-0.1)
    ax1.plot([x1_1, x2_1], [y1, y2], '-', color='brown', alpha=branch_alpha-0.1)
    ax1.fill_between([x1_0, x1_1], 0.04, y2, color='brown', alpha=branch_alpha)
    ax1.fill_between([x1_0, x1_1], -0.04, y1, color='brown', alpha=branch_alpha, label=f'Branches')

    ax1.scatter(total_reached_coords[:, 0], total_reached_coords[:, 2], s=reached_sizes, color=turtle_green, label='Reachable Apples')
    ax1.scatter(total_unreached_coords[:, 0], total_unreached_coords[:, 2], s=unreached_sizes, color=light_purple, label='Unreachable Apples')

    ax1.set_xlabel('x', fontsize=16)
    ax1.set_ylabel('z', fontsize=16)
    ax1.set_xlim(-0.75, 0.75)
    ax1.set_ylim(-radii_threshold, radii_threshold)
    ax1.set_aspect('equal')
    ax1.grid(True)
    ax1.legend(loc='upper left', fontsize=14)
    ax1.tick_params(axis='both', which='major', labelsize=14)

    plt.tight_layout()
    plt.show()

    ### SECOND FIGURE: SIDE VIEW ###
    # fig2, ax2 = plt.subplots(figsize=figsize)
    fig2, ax2 = plt.subplots()

    angle_deg = 90 + 18.435
    x1_0, x2_0, x1_1, x2_1, y1, y2 = get_branch_position(0.08, angle_deg, 1.0, span='y')

    ax2.plot([x1_0, x2_0], [y1, y2], '-', color='brown', alpha=branch_alpha)
    ax2.plot([x1_1, x2_1], [y1, y2], '-', color='brown', alpha=branch_alpha)
    ax2.fill_betweenx([y1, y2], [x1_0, x2_0], [x1_1, x2_1], color='brown', alpha=branch_alpha)

    circle = plt.Circle((0, 0), 0.04, color='brown', fill=False, linestyle='-', linewidth=2, alpha=branch_alpha+0.1)
    ax2.add_patch(circle)

    ax2.scatter(total_reached_coords[:, 1], total_reached_coords[:, 2], color=turtle_green, label='Reachable Apples')
    ax2.scatter(total_unreached_coords[:, 1], total_unreached_coords[:, 2], color=light_purple, label='Unreachable Apples')

    ax2.set_xlabel('y', fontsize=16)
    ax2.set_ylabel('z', fontsize=16)
    ax2.set_xlim(-radii_threshold, radii_threshold)
    ax2.set_ylim(-radii_threshold, radii_threshold)
    ax2.set_aspect('equal')
    ax2.grid(True)
    # ax2.legend(loc='upper left', fontsize=12)
    ax2.tick_params(axis='both', which='major', labelsize=14)

    plt.tight_layout()
    plt.show()

def plot_reachability(data_filenames):
    """
    Plot reachability comparison between templating and voxelization.

    Parameters:
        x (array): x-axis values (number of detected fruits).
        templating (array): Percent reachability for templating.
        voxelization (array): Percent reachability for voxelization.
    """
    template_reachability, voxel_reachability, apples_found = get_avg_reachable_rate(data_filenames)

    template_reachability = np.array(template_reachability) * 100
    voxel_reachability = np.array(voxel_reachability) * 100

    plt.figure(figsize=(8, 5))
    plt.scatter(apples_found, template_reachability, label="Templating", marker='o', color='blue')
    plt.scatter(apples_found, voxel_reachability, label="Voxelization", marker='s', color='green')

    plt.xlabel("Number of Detected Fruit", fontsize=12)
    plt.ylabel("Percent Reachability (%)", fontsize=12)
    plt.title("Reachability Comparison: Templating vs. Voxelization", fontsize=14)
    plt.legend(fontsize=10)
    plt.grid(True, linestyle='--', alpha=0.7)
    plt.tight_layout()
    plt.xticks(np.arange(min(apples_found), max(apples_found) + 1, 1))
    plt.show()

def plot_reachability_histogram(data_filenames, turtle_green=(150/255, 207/255, 103/255), light_blue=(123/255, 175/255, 222/255), light_orange=(246/255, 193/255, 64/255)):
    """
    Plot reachability comparison as a histogram. Colors based on Paul Tol's Figure 19

    Parameters:
        data_filenames (list): List of filenames containing the data.
    """
    _, _, apples_found, apples_reached_template, apples_reached_voxel = get_avg_reachable_rate(data_filenames)

    # Sort based on apples_found
    sorted_indices = np.argsort(apples_found)
    apples_found = np.array(apples_found)[sorted_indices]
    apples_reached_template = np.array(apples_reached_template)[sorted_indices]
    apples_reached_voxel = np.array(apples_reached_voxel)[sorted_indices]

    bar_width = 0.25
    x_indices = np.arange(len(apples_found))

    plt.figure(figsize=(10, 6))
    plt.bar(x_indices - bar_width, apples_found, width=bar_width, label="Total Apples Found", color=light_orange)
    plt.bar(x_indices, apples_reached_template, width=bar_width, label="Apples Reached (Templating)", color=turtle_green)
    plt.bar(x_indices + bar_width, apples_reached_voxel, width=bar_width, label="Apples Reached (Voxelization)", color=light_blue)

    plt.xlabel("Tree Index", fontsize=16)
    plt.ylabel("Number of Apples", fontsize=16)
    plt.title("Number of Apples Found: Total vs. Templating vs. Voxelization", fontsize=16)
    plt.xticks(ticks=x_indices, labels=[f'{i+1}' for i in x_indices], fontsize=16)
    plt.legend(fontsize=14)
    plt.grid(axis='y', linestyle='--', alpha=0.7)
    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    # Path to the YAML file
    version = "v5"

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

    new_files = [
                'v4/experiment_a_60sec_rrtconnect.yaml',
                'v4/experiment_b_60sec_rrtconnect.yaml',
                'v4/experiment_c_60sec_rrtconnect.yaml',
                'v4/experiment_d_60sec_rrtconnect.yaml',
                'v4/experiment_e_60sec_rrtconnect.yaml',
                'v4/experiment_f_60sec_rrtconnect.yaml',
                'v4/experiment_g_60sec_rrtconnect.yaml',
                'v4/experiment_h_60sec_rrtconnect.yaml',
                'v4/experiment_i_60sec_rrtconnect.yaml',
                'v4/experiment_j_60sec_rrtconnect.yaml',
                ]

    # Get the average reachable rate
    get_avg_reachable_rate(results_files)
    # get_avg_reachable_rate(new_files)

    # plot_norm_apple_reachability(new_files, figsize=(18, 10), radii_threshold=0.25)
    # plot_norm_apple_reachability(results_files, radii_threshold=0.25)

    # plot_reachability(results_files)
    plot_reachability_histogram(results_files)


