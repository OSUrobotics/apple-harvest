import yaml
import numpy as np

# Path to the YAML file
results_dir = "/home/marcus/orchard_template_ws/results_data/"
results_a = "experiment_a_results.yaml"
results_b = "experiment_b_results.yaml"
results_c = "experiment_c_results.yaml"
results_d = "experiment_d_results.yaml"
results_e = "experiment_e_results.yaml"
results_files = [results_a, results_b, results_c, results_d, results_e]

# Function to extract the required data
def extract_apple_data(file_path, dir="/home/marcus/orchard_template_ws/results_data/"):
    with open(dir + file_path, "r") as file:
        data = yaml.safe_load(file)
    
    apples_found = data.get("apples_found", None)
    apples_reached_templating = data.get("apples_reached_templating", None)
    apples_reached_voxelization = data.get("apples_reached_voxelization", None)
    
    return {
        "apples_found": apples_found,
        "apples_reached_templating": apples_reached_templating,
        "apples_reached_voxelization": apples_reached_voxelization
    }

def compile_experiments(data_filenames):
    reachable_rates_template = []
    reachable_rates_voxel = []
    for i, filenames in enumerate(data_filenames):
        apple_data = extract_apple_data(filenames)

        reachable_rates_template.append(apple_data['apples_reached_templating'] / apple_data['apples_found'])
        reachable_rates_voxel.append(apple_data['apples_reached_voxelization'] / apple_data['apples_found'])

    avg_reachble_rate_template = np.average(reachable_rates_template)
    avg_reachble_rate_voxel = np.average(reachable_rates_voxel)

    print(f'Average reachable rate of templating method: {np.round(avg_reachble_rate_template * 100, 1)}%')
    print(f'Average reachable rate of voxelization method: {np.round(avg_reachble_rate_voxel * 100, 1)}%')


compile_experiments(results_files)