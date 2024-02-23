import os
import numpy as np
import matplotlib.pyplot as plt
import sys

# Function to parse log files and extract Mean ATE values
def parse_log_file(log_file_path):
    mean_ate_values = []
    with open(log_file_path, 'r') as file:
        lines = file.readlines()
        for line in lines:
            if line.startswith('Frame Number'):
                continue
            tokens = line.split()
            mean_ate = float(tokens[6])
            mean_ate_values.append(mean_ate)
    return mean_ate_values

# Function to calculate average Mean ATE for each value of filter
def calculate_average_mean_ate(log_folder):
    mean_ate_values_all_runs = []
    for exp_folder in os.listdir(log_folder):
        exp_folder_path = os.path.join(log_folder, exp_folder)
        mean_ate_values = []
        for run in range(5):  # Assuming 5 runs
            log_file_path = os.path.join(exp_folder_path, f'log_file_{run}.txt')
            if os.path.exists(log_file_path):
                mean_ate_values_run = parse_log_file(log_file_path)
                mean_ate_values.extend(mean_ate_values_run)
        if mean_ate_values:
            mean_ate_values_all_runs.append(np.mean(mean_ate_values))
    return mean_ate_values_all_runs

# Function to plot Mean ATE evolution for each filter type
def plot_mean_ate_evolution(filter_type, mean_ate_values_dict):
    plt.figure()
    for num_frames, mean_ate_values in mean_ate_values_dict.items():
        plt.plot(mean_ate_values, label=f'{num_frames} frames')
    plt.xlabel('Value of applied filter')
    plt.ylabel('Mean ATE')
    plt.title(f'Evolution of Mean ATE for {filter_type}')
    plt.legend()
    plt.show()

# Main function
def main():

    datasets_folder = sys.argv[1]
    filters = ['blur', 'contrast', 'brightness']
    for filter_type in filters:
        mean_ate_values_dict = {}
        filter_folder = os.path.join(datasets_folder, filter_type)
        for frames_folder in os.listdir(filter_folder):
            frames_folder_path = os.path.join(filter_folder, frames_folder)
            num_frames = frames_folder.split('_')[-1]
            mean_ate_values = calculate_average_mean_ate(frames_folder_path)
            mean_ate_values_dict[num_frames] = mean_ate_values
        plot_mean_ate_evolution(filter_type, mean_ate_values_dict)

if __name__ == "__main__":
    main()
